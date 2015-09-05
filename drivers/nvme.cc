/*
 * Copyright (C) 2015 Cyril Plisko.
 *
 * This work is open source software, licensed under the terms of the
 * BSD license as described in the LICENSE file in the top-level directory.
 */

#include <algorithm>
#include <typeinfo>
#include <string.h>
#include <osv/clock.hh>
#include <osv/debug.h>
#include <osv/mmu.hh>
#include <osv/device.h>
#include <osv/bio.h>
#include <osv/types.h>

#include "drivers/nvme.hh"

namespace nvme {

struct nvme_state {
    devop_strategy_t    strategy;
    nvme*               controller;
    nvme_namespace*     ns;
};

int nvme::ctrl_idx = 0;
std::string nvme::_driver_name = "nvme";

static void nvme_strategy(struct bio *bio)
{
    auto prv = nvme::get_state(bio);
    prv->ns->make_request(bio);
}

static int nvme_read(struct device *dev, struct uio *uio, int ioflags)
{
    return bdev_read(dev, uio, ioflags);
}

static int nvme_write(struct device *dev, struct uio *uio, int ioflags)
{
    return bdev_write(dev, uio, ioflags);
}

static struct devops nvme_devops {
    no_open,
    no_close,
    nvme_read,
    nvme_write,
    no_ioctl,
    no_devctl,
    multiplex_strategy,
};

static struct driver nvme_driver = {
    "nvme",
    &nvme_devops,
    sizeof(struct nvme_state),
};

qpair::qpair(size_t nentries, size_t align,
    std::function<void (uint32_t)> tdb,
    std::function<void (uint32_t)> hdb)
    : _submission_queue(nentries, align)
    , _completion_queue(nentries, align)
    , _tail_doorbell_update(tdb)
    , _head_doorbell_update(hdb)
{
    // nvme_d("QPair SQ/CQ 0x%llX/0x%llX\n",
    //     submission_queue_addr(),
    //     completion_queue_addr());
}

void qpair::submit(submission_queue_entry& sqe)
{
    sqe.cdw0.cid = _cid++;
    _submission_queue.push(sqe);
    _tail_doorbell_update(_submission_queue.tail());
}

bool qpair::exec_sync_cmd(submission_queue_entry& sqe,
    completion_queue_entry& cqe, bool poll)
{
    submit(sqe);

    if (poll) {
        auto phase = _completion_queue.phase();
        auto& _cqe = _completion_queue.peek();
        //nvme_d("Checking completion queue slot %d, phase %d", _completion_queue.head(), phase);
        uint64_t spin = 0;
        while (_cqe.p != phase) {
            sched::thread::sleep(std::chrono::nanoseconds(10));
            barrier();
            spin++;
        }
        barrier();
        //nvme_d("Spun %lld times on CQE", spin);
    } else {
        // TODO Wait for completion interrupt to wake us
    }

    // TODO Need to update CQ tail pointer somehow
    // When I'll support interrupt it could be possible to update _tail from isr

    cqe = _completion_queue.pop(); // FIXME Need better handling
    //decode_cqe(cqe);
    //nvme_d("ringing completion queue head doorbell %d", _completion_queue.head());
    _head_doorbell_update(_completion_queue.head());
    _submission_queue.head(cqe.sqhd);

    auto good = (cqe.sc == SUCCESSFUL_COMPLETION);
    if (!good) {
        nvme_d("Command failed, SQE and CQE dump follows");
        dump_sqe(sqe);
        dump_cqe(cqe);
    }
    return good;
}

const char* qpair::sc_to_str(uint8_t sc) const
{
    const char* status_codes[] = {
        "0x00 SUCCESSFUL_COMPLETION",
        "0x01 INVALID_COMMAND_OPCODE",
        "0x02 INVALID_FIELD_IN_COMMAND",
        "0x03 COMMAND_ID_CONFLICT",
        "0x04 DATA_TRANSFER_ERROR",
        "0x05 COMMAND_ABORTED_POWER_LOSS",
        "0x06 INTERNAL_ERROR",
        "0x07 COMMAND_ABORT_REQUESTED",
        "0x08 COMMAND_ABORTED_SQ_DELETION",
        "0x09 COMMAND ABORTED DUE TO FAILED FUSED COMMAND",
        "0x0A COMMAND ABORTED DUE TO MISSING FUSED COMMAND",
        "0x0B INVALID_NAMESPACE_OR_FORMAT",
        "0x0C COMMAND SEQUENSE ERROR",
    };

    return status_codes[sc];
}

void qpair::dump_sqe(submission_queue_entry& sqe) const
{
    nvme_d("Submission Queue Entry\n"
        "\t\tCDW0.OPC\t0x%X\n"
        "\t\tCDW0.FUSE\t0x%X\n"
        "\t\tCDW0.PSDT\t0x%X\n"
        "\t\tCDW0.CID\t0x%X\n"
        "\t\tCDW1 NSID\t0x%X\n"
        "\t\tRESERVED\t0x%llX\n"
        "\t\tMPTR\t\t0x%llX\n"
        "\t\t%s\t0x%llX\n"
        "\t\t%s\t0x%llX\n"
        "\t\tSLBA\t\t0x%llX\n"
        "\t\tCDW10\t\t0x%X\n"
        "\t\tCDW11\t\t0x%X\n"
        "\t\tCDW12\t\t0x%X\n"
        "\t\tCDW13\t\t0x%X\n"
        "\t\tCDW14\t\t0x%X\n"
        "\t\tCDW15\t\t0x%X\n",
        sqe.cdw0.opc, sqe.cdw0.fuse, sqe.cdw0.psdt, sqe.cdw0.cid,
        sqe.nsid, sqe.reserved0, sqe.mptr,
        (sqe.cdw0.psdt == 0) ? "PRP1\t" : "SGL ADDRESS", sqe.prp1,
        (sqe.cdw0.psdt == 0) ? "PRP2\t" : "SGL LENGTH", sqe.prp2, sqe.slba,
        sqe.cdw10, sqe.cdw11, sqe.cdw12, sqe.cdw13, sqe.cdw14, sqe.cdw15);
}

void qpair::dump_cqe(completion_queue_entry& cqe) const
{
    nvme_d("Completion Queue Entry\n"
        "\t\tDo Not Retry (DNR)\t\t%d\n"
        "\t\tMore (M)\t\t\t%d\n"
        "\t\tStatus Code Type (SCT)\t\t%d\n"
        "\t\tStatus Code (SC)\t\t%s\n"
        "\t\tPhase (P)\t\t\t%d\n"
        "\t\tCommand ID (CID)\t\t%d\n"
        "\t\tSQ Identifier (SQID)\t\t%d\n"
        "\t\tSQ Head Pointer (SQHD)\t\t%d\n",
        cqe.dnr, cqe.m, cqe.sct, sc_to_str(cqe.sc),
        cqe.p, cqe.cid, cqe.sqid, cqe.sqhd);
}

void nvme_namespace::report_nvme_namespace() const
{
    nvme_d("Namespace Identify Data\n"
        "\tLBA Size\t\t\t%lld\n"
        "\tSize\t\t\t\t%lld (0x%llX) x %lld\n"
        "\tCapacity\t\t\t%lld (0x%llX) x %lld\n",
         _lbasize, _size, _size, _lbasize, _capacity, _capacity, _lbasize);
}

nvme_namespace::nvme_namespace(uint32_t nsid, identify_namespace_data* ind)
    : _nsid(nsid)
    , _size(ind->nsze)
    , _capacity(ind->ncap)
    , _utilization(ind->nuse)
{
    // Find out the LBA size for this namespace
    _lbasize = 1 << ind->lbaf[ind->flbas].lbads;

    report_nvme_namespace();

    nvme_d("\tNSFEAT 0x%hhx", ind->nsfeat);
    nvme_d("\tNLBAF %hhd", ind->nlbaf);
    nvme_d("\tFLBAS %hhd", ind->flbas);
    nvme_d("\tMC %hhd", ind->mc);
    nvme_d("\tNVMCAP 0x%llX%016llX", ind->nvmcap[1], ind->nvmcap[0]);
    for (int i = 0; i <= ind->nlbaf; i++) {
        nvme_d("\t\tLBAF[%d] RP %d, LBADS 0x%hhx, MS 0x%hd",
            i, ind->lbaf[i].rp, ind->lbaf[i].lbads, ind->lbaf[i].ms);
    }
}

void nvme_namespace::nvm_rw(struct bio* bio)
{
    assert(bio->bio_offset % _lbasize == 0);
    assert(bio->bio_bcount % _lbasize == 0);

    uint64_t slba = bio->bio_offset / _lbasize;
    uint16_t nlb = bio->bio_bcount / _lbasize;
    auto buf = mmu::virt_to_phys(bio->bio_data);
    auto cmd = (bio->bio_cmd == BIO_READ) ? READ : WRITE;
    auto prv = nvme::get_state(bio);
    auto nvme = prv->controller;

    auto done = nvme->readwrite(_nsid, cmd, buf, bio->bio_bcount, slba, nlb);
    if (!done) {
        nvme_d("FAILED COMMAND %s(%d, %lld, %hd)->%p x %d",
            (cmd == READ) ? "READ" : "WRITE",
            _nsid, slba, nlb, buf, bio->bio_bcount);
    }
    if (nvme->poll_mode()) {
        biodone(bio, done);
    }
}

void nvme_namespace::nvm_flush(struct bio *bio)
{
    auto prv = nvme::get_state(bio);
    auto nvme = prv->controller;
    auto done = nvme->flush(_nsid);

    if (nvme->poll_mode()) {
        biodone(bio, done);
    }
}

int nvme_namespace::make_request(struct bio* bio)
{
    WITH_LOCK(_lock) {
        if (!bio)
            return EIO;

        switch (bio->bio_cmd) {
        case BIO_FLUSH:
            nvm_flush(bio);
            break;
        case BIO_READ:
        case BIO_WRITE:
            nvm_rw(bio);
            break;
        default:
            return ENOTBLK;
        }

        return 0;
    }
}

nvme::nvme(pci::device& pci_dev)
    : hw_driver()
    , _pci_dev(pci_dev)
    , _msix(&pci_dev)
{
    static_assert(sizeof(submission_queue_entry) == 64,
        "Submission Queue Entry Size is not 64 bytes");
    static_assert(sizeof(completion_queue_entry) == 16,
        "Completion Queue Entry Size is not 16 bytes");

    if (parse_pci_config() == false) {
        nvme_d("parse_pci_config() failed");
        // XXX Now what ? Abort ?
    }

    dump_config();

    reset();
    setup();
    report_controller();
    scan();


    std::string dev_name = "nvmectl" + std::to_string(ctrl_idx++);

    struct device* dev = device_create(&nvme_driver, dev_name.c_str(), D_CHR);

    if (dev->private_data == nullptr) {
        dev->private_data = this;
    }
}

nvme::~nvme()
{
    for (auto it : _namespaces) {
        auto ns = it.second;
        delete ns;
    }

    drain_io_queue();

    delete_io_submission_queue(1);
    delete_io_completion_queue(1);

    delete _io_queue;
    delete _admin_queue;

    shutdown();
}

void nvme::reset()
{
    controller_capabilities cap = { .raw = nvme_get64(NVME_CR_CAP) };

    if (cap.nssrs == 1) {
        nvme_d("NVM Subsystem Reset support detected");
        nvme_put32(NVME_CR_NSSR, NVME_RESET);
    } else {
        nvme_d("No NVM Subsystem Reset support detected, "
            "performing controller level reset");
        controller_configuration cc = { .raw = nvme_get32(NVME_CR_CC) };
        cc.en = 0;
        nvme_put32(NVME_CR_CC, cc.raw);
    }

    controller_status csts = { .raw = nvme_get32(NVME_CR_CSTS) };

    while (csts.rdy != 0) {
        nvme_d("Waiting for NVM subsystem to reset");
        csts.raw = nvme_get32(NVME_CR_CSTS);
    }

    nvme_d("NVM Reset Completed (controller status 0x%X)", csts.raw);
}

void nvme::shutdown()
{
    controller_configuration cc = { .raw = nvme_get32(NVME_CR_CC) };

    cc.shn = NORMAL_SHUTDOWN;
    nvme_put32(NVME_CR_CC, cc.raw);

    controller_status csts = { .raw = nvme_get32(NVME_CR_CSTS) };

    while (csts.shst != SHUTDOWN_COMPLETE) {
        csts.raw = nvme_get32(NVME_CR_CSTS);
    }
}

qpair* nvme::setup_io_qpair(size_t nentries, int qn)
{
    auto tdb = [=] (uint32_t t) { nvme_put32(sq_tail_doorbell(qn), t); };
    auto hdb = [=] (uint32_t h) { nvme_put32(cq_head_doorbell(qn), h); };
    auto ioq = new qpair(nentries, _mps, tdb, hdb);

    nvme_d("Create IO Completion Queue %d", qn);
    create_io_completion_queue(qn, ioq->completion_queue_size() - 1,
        ioq->completion_queue_addr());

    nvme_d("Create IO Submission Queue %d", qn);
    create_io_submission_queue(qn, ioq->submission_queue_size() - 1,
        ioq->submission_queue_addr());

    return ioq;
}

void nvme::setup()
{
    // Memory Page Size is 4KB based in NVME parlance (4KB == 1 << 12)
    controller_capabilities cap = { .raw = nvme_get64(NVME_CR_CAP) };
    controller_configuration cc = { .raw = nvme_get32(NVME_CR_CC) };
    admin_queue_attributes aqa = { .raw = nvme_get32(NVME_CR_AQA) };

    _version = nvme_get32(NVME_CR_VS);

    // Choose Memory Page Size
    // Start with 4KB and scale up to MPSMIN if needed and limit by MPSMAX
    int _min = cap.mpsmin;
    int _max = cap.mpsmax;

    cc.mps = std::min(std::max(0, _min), _max);
    cc.mps = _max;

    _mpsmax = (1 << (12 + cap.mpsmax));
    _mpsmin = (1 << (12 + cap.mpsmin));
    _mps    = (1 << (12 + cc.mps));

    _doorbell_stride = cap.dstrd;

    cc.ams = 0;
    cc.css = 0;
    cc.iosqes = _sqes;
    cc.iocqes = _cqes;

    nvme_put32(NVME_CR_CC, cc.raw);

    // Allocate Admin Queue
    {
        auto _tdb = [=] (uint32_t t) { nvme_put32(sq_tail_doorbell(0), t); };
        auto _hdb = [=] (uint32_t h) { nvme_put32(cq_head_doorbell(0), h); };
        _admin_queue = new qpair(32, _mps, _tdb, _hdb);
    }

    // Setup Admin Queue
    aqa.asqs = _admin_queue->submission_queue_size();
    aqa.acqs = _admin_queue->completion_queue_size();

    nvme_put32(NVME_CR_AQA, aqa.raw);
    nvme_put64(NVME_CR_ASQ, _admin_queue->submission_queue_addr());
    nvme_put64(NVME_CR_ACQ, _admin_queue->completion_queue_addr());

    // Enable Controller
    cc.en = 1;
    nvme_put32(NVME_CR_CC, cc.raw);

    auto timeout = osv::clock::uptime::now();
    using namespace osv::clock::literals;
    timeout += cap.to * 500_ms;

    controller_status csts = { .raw = nvme_get32(NVME_CR_CSTS) };

    while ((csts.rdy != 1) && (csts.cfs != 1) &&
        (timeout > osv::clock::uptime::now())) {
        //nvme_d("Waiting for NVM subsystem to become READY\n");
        csts.raw = nvme_get32(NVME_CR_CSTS);
    }

    if (csts.cfs == 1) {
        nvme_d("Controller status 0x%X", csts.raw);
        nvme_d("Controller Fatal Status detected");
        // XXX Now what ? Abort ?
    }

    identify_controller();

    get_number_of_queues();
    set_number_of_queues(0, 0);
    get_number_of_queues();

    _io_queue = setup_io_qpair(cap.mqes + 1, 1);

    // Setup interrupt
    if (_pci_dev.is_msix()) {
        nvme_d("MSI-X interrupt supported and %sabled",
            _pci_dev.is_msix_enabled() ? "en" : "dis");
        if (!_pci_dev.is_msix_enabled()) {
            nvme_d("Enabling MSI-X interrupt");
            _pci_dev.msix_enable();
        }
        nvme_d("MSI-X interrupt supported and %sabled",
            _pci_dev.is_msix_enabled() ? "en" : "dis");
        _msix.easy_register({
            { 0, [=] { isr(); }, nullptr},
            { 1, [=] { isr(); }, nullptr} });
    } else if (_pci_dev.is_msi()) {
        nvme_d("MSI interrupt supported and %sabled",
            _pci_dev.is_msi_enabled() ? "en" : "dis");
    } else {
        nvme_d("PIN interrupt supported");
    }
}

void nvme::scan()
{
    uint32_t nsid = 1;

    for (uint32_t i = 0; i < _nn; i++) {
        nvme_namespace* _ns = nullptr;

        // Increment the namespace id until we hit existing namespace
        while (_ns == nullptr) {
            _ns = identify_namespace(nsid);
            if (_ns != nullptr) {
                _ns->name("nvme" + std::to_string(nsid));
                add_namespace(nsid, _ns);
                auto dev = device_create(&nvme_driver, _ns->name(), D_BLK);
                auto prv = static_cast<struct nvme_state*>(dev->private_data);
                prv->controller = this;
                prv->ns = _ns;
                prv->strategy = nvme_strategy;
                dev->size = _ns->size() * _ns->lbasize(); // Size in bytes
                // IF SGL is not supported limit transfers to
                // one MPS worth of data (I want to avoid handling PRP list)
                dev->max_io_size = _sgl_ok ? _max_xfer_size : _mps;
                read_partition_table(dev);
            }
            nsid++;
        }

#if 0
        {
            nvme_d("Trying identify_namespace(%d)\n", nsid);
            _ns = identify_namespace(nsid);
            delete _ns;
        }
#endif
    }
}

void nvme::add_namespace(uint32_t nsid, nvme_namespace* ns)
{
    _namespaces.insert(std::make_pair(nsid, ns));

    debugf("NVMe: Add namespace %d as %s, lbasize=%zd, devsize=%zd bytes\n",
        nsid, ns->name(), ns->lbasize(), ns->lbasize() *  ns->size());
}

// Low Level Admin Command Set implementation

bool nvme::delete_io_submission_queue(uint16_t qid)
{
    submission_queue_entry sqe = {};

    sqe.cdw0.opc = DELETE_SQ;
    sqe.cdw10.qid = qid;

    return _admin_queue->exec_sync_cmd(sqe, _poll_mode);
}

bool nvme::create_io_submission_queue(uint16_t qid, uint16_t qsize, mmu::phys pa)
{
    submission_queue_entry sqe = {};

    sqe.cdw0.opc = CREATE_SQ;
    sqe.prp1 = pa;
    sqe.cdw10.qsize = qsize;
    sqe.cdw10.qid = qid;
    sqe.cdw11.csq.pc = 1;
    sqe.cdw11.csq.qprio = URGENT;
    sqe.cdw11.csq.cqid = qid; // Assume 1:1 submission queue:completion queue

    return _admin_queue->exec_sync_cmd(sqe, _poll_mode);
}

bool nvme::get_log_page()
{
    ::abort("%s not implemented yet", __PRETTY_FUNCTION__);
    return false;
}

bool nvme::delete_io_completion_queue(uint16_t qid)
{
    submission_queue_entry sqe = {};

    sqe.cdw0.opc = DELETE_CQ;
    sqe.cdw10.qid = qid;

    return _admin_queue->exec_sync_cmd(sqe, _poll_mode);
}

bool nvme::create_io_completion_queue(uint16_t qid, uint16_t qsize, mmu::phys pa)
{
    submission_queue_entry sqe = {};

    sqe.cdw0.opc = CREATE_CQ;
    sqe.prp1 = pa;
    sqe.cdw10.qsize = qsize;
    sqe.cdw10.qid = qid;
    sqe.cdw11.ccq.pc = 1;
    sqe.cdw11.ccq.ien = 1;
    sqe.cdw11.ccq.iv = 1;

    return _admin_queue->exec_sync_cmd(sqe, _poll_mode);
}

bool nvme::identify(cdw10_t cdw10, uint32_t nsid, mmu::phys pa)
{
    submission_queue_entry sqe = {};

    sqe.cdw0.opc = IDENTIFY;
    sqe.nsid = nsid;
    sqe.cdw0.fuse = 0;   // Normal operation
    sqe.cdw0.psdt = 0;   // PRP is used for this transfer
    sqe.prp1 = pa;
    sqe.cdw10 = cdw10;

    return _admin_queue->exec_sync_cmd(sqe, _poll_mode);
}

bool nvme::abort(uint16_t cid, uint16_t sqid)
{
    ::abort("%s not implemented yet", __PRETTY_FUNCTION__);
    return false;
}

bool nvme::get_features(uint8_t feature, completion_queue_entry& cqe)
{
    submission_queue_entry sqe = {};

    sqe.cdw0.opc = GET_FEATURES;
    sqe.cdw10.sel = 0;      // Only support _current_ values for now
    sqe.cdw10.fid = feature;

    return _admin_queue->exec_sync_cmd(sqe, cqe, _poll_mode);
}

bool nvme::set_features(uint8_t feature, cdw11_t cdw11,
    completion_queue_entry& cqe)
{
    submission_queue_entry sqe = {};

    sqe.cdw0.opc = SET_FEATURES;
    sqe.cdw10.fid = feature;
    sqe.cdw11 = cdw11;

    return _admin_queue->exec_sync_cmd(sqe, cqe, _poll_mode);
}

bool nvme::async_event_request()
{
    ::abort("%s not implemented yet", __PRETTY_FUNCTION__);
    return false;
}

// High Level Wrappers to Low Level Admin Command Set functions

nvme_namespace* nvme::identify_namespace(uint32_t nsid)
{
    static_assert(sizeof(identify_namespace_data) == 4096,
        "identify_namespace_data is not 4KB");

    memory::phys_contiguous_memory buf(sizeof(identify_namespace_data), _mps);
    constexpr cdw10_t identify_namespace = { .cdw10 = 0x00000000 };

    if (identify(identify_namespace, nsid, buf.get_pa())) {
        auto ind = reinterpret_cast<identify_namespace_data*>(buf.get_va());
        return new nvme_namespace(nsid, ind);
    } else {
        return nullptr;
    }
}

void nvme::identify_controller()
{
    static_assert(sizeof(identify_data) == 4096,
        "identify_data is not 4KB");

    memory::phys_contiguous_memory buf(sizeof(identify_data), _mps);
    constexpr cdw10_t identify_controller = { .cdw10 = 0x00000001 };

    if (identify(identify_controller, 0, buf.get_pa())) {
        auto icd = reinterpret_cast<identify_data*>(buf.get_va());

        _serial = std::string(icd->sn, sizeof(icd->sn));
        _model = std::string(icd->mn, sizeof(icd->mn));
        _firmware = std::string(icd->fr, sizeof(icd->fr));
        _nn = icd->nn;
        //_version = icd->ver;
        _max_xfer_size = (icd->mdts) ? (_mpsmin << icd->mdts) : MAX_XFER_SIZE;
        _sgl_ok = (icd->sgls.sglsupported == 1);

        report_identify_data(*icd);
    } else {
        debug("[NVMe] IDENTIFY command failed\n");
    }
}

void nvme::get_number_of_queues()
{
    completion_queue_entry cqe;

    get_features(NUMBER_OF_QUEUES, cqe);
    nvme_d("GET_FEATURES[NUMBER_OF_QUEUES] %X", cqe.command_specific);
}

void nvme::set_number_of_queues(uint16_t ncqr, uint16_t nsqr)
{
    completion_queue_entry cqe;
    cdw11_t cdw11;

    cdw11.ncqr = ncqr;
    cdw11.nsqr = nsqr;

    set_features(NUMBER_OF_QUEUES, cdw11, cqe);
    nvme_d("SET_FEATURES[NUMBER_OF_QUEUES] %X", cqe.command_specific);
}

// NVM Command Set Low Level operations

bool nvme::flush(uint16_t nsid)
{
    submission_queue_entry sqe = {};

    sqe.cdw0.opc = FLUSH;
    sqe.nsid = nsid;

    return _io_queue->exec_sync_cmd(sqe, _poll_mode);
}

bool nvme::readwrite(uint16_t nsid, enum nvm_opcode opc, mmu::phys buf,
    uint32_t len, uint64_t slba, uint16_t nlb)
{
    submission_queue_entry sqe = {};

    assert(len <= _mps);

    sqe.cdw0.opc = opc;
    sqe.cdw0.psdt = _sgl_ok ? 1 : 0;
    sqe.nsid = nsid;
    if (_sgl_ok) {
        // Setup SGL
        sqe.address = buf;
        sqe.length = len;
        sqe.sgl_descriptor_type = DATA_BLOCK;
    } else {
        // Setup PRP
        sqe.prp1 = buf;
        if ((buf % _mps) &&
            ((buf / _mps) < (buf + len) / _mps)) {
                sqe.prp2 = (buf + len) & ~(_mps - 1);
        }
    }
    sqe.slba = slba;
    sqe.cdw12.nlb = nlb - 1; // The value here is 0 based; HW designers must die

    return _io_queue->exec_sync_cmd(sqe, _poll_mode);
}

void nvme::dump_config()
{
    u8 B, D, F;

    _pci_dev.get_bdf(B, D, F);
}

void nvme::report_controller_capabilities(controller_capabilities cap) const
{
    nvme_d("NVME Controller Capabilities\n"
        "\tMaximum Queue Entries Supported (MQES)\t%d\n"
        "\tContiguous Queues Required (CQR)\t%d\n"
        "\tArbitration Mechanisms Supported (AMS)\t%d\n"
        "\tTimeout (TO)\t\t\t\t%d\n"
        "\tDoorbell Stride (DSTRD)\t\t\t%d\n"
        "\tNVM Subsystem Reset Supported (NSSRS)\t%d\n"
        "\tCommand Sets Supported (CSS)\t\t%d\n"
        "\tMemory Page Size Minimum (MPSMIN)\t%d\n"
        "\tMemory Page Size Maximum (MPSMAX)\t%d\n",
        cap.mqes, cap.cqr, cap.ams, cap.to, cap.dstrd,
        cap.nssrs, cap.css, cap.mpsmin, cap.mpsmax);
}

void nvme::report_identify_data(const identify_data& id) const
{
    nvme_d("NVME Controller Identify Data\n"
        "\tVendor ID (VID)\t\t\t\t%hX\n"
        "\tSubsystem Vendor ID (SSVID)\t\t%hX\n"
        "\tRecommended Arbitration Burst (RAB)\t%hhd\n"
        "\tIEEE OUI Identifier (IEEE)\t\t%02hhx%02hhx%02hhx\n"
        "\tMaximum Data Transfer Size (MDTS)\t%d\n"
        "\tController ID (CNTLID)\t\t\t%hd\n"
        "\tVersion (VER)\t\t\t\t%X\n"
        "\tOptional Async Events Supported (OAES)\t0x%X\n"
        "\tOptional Admin Command Supported (OACS)\t0x%hX\n"
        "\tAbort Command Limit (ACL)\t\t%hhd\n"
        "\tAsync Event Request Limit (AERL)\t%hhd\n"
        "\tSubmission Queue Entry Size (SQES)\t0x%hhX\n"
        "\tCompletion Queue Entry Size (CQES)\t0x%hhX\n"
        "\tNumber of Namespaces (NN)\t\t%d\n"
        "\tSGL Support (SGLS)\t\t\t%X\n",
        id.vid, id.ssvid, id.rab, id.ieee0, id.ieee1, id.ieee2,
        id.mdts, id.cntlid, id.ver, id.oaes, id.oacs, id.acl, id.aerl,
        id.sqes, id.cqes, id.nn, id.sgls);
}

void nvme::report_controller()
{
    controller_capabilities cap = { .raw = nvme_get64(NVME_CR_CAP) };
    auto version = nvme_get32(NVME_CR_VS);
    auto cmbloc = nvme_get32(NVME_CR_CMBLOC);
    auto cmbsz = nvme_get32(NVME_CR_CMBSZ);

    report_controller_capabilities(cap);

    nvme_i("NVME Controller version 0x%08X", version);
    nvme_i("Serial Number: %s", _serial.c_str());
    nvme_i("Model Number: %s", _model.c_str());
    nvme_i("Firmware Revision: %s", _firmware.c_str());

    nvme_i("MPS/MIN/MAX - 0x%zX/0x%zX/0x%zX", _mps, _mpsmin, _mpsmax);

    nvme_i("Controller Memory Buffer Loc/size 0x%08X/0x%08X", cmbloc, cmbsz);
    nvme_i("SGL support %sabled ", _sgl_ok ? "en" : "dis");

    if (version == 0x10001) {
        nvme_w("Looks like you're using outdated QEMU,"
            " upgrade to 2.3 or newer");
    }
}

bool nvme::isr()
{
    bool done = false;

    //nvme_d("Interrupt !");
    done = true;

    return done;
}

void nvme::drain_io_queue()
{
    nvme_w("%s not implemented yet", __PRETTY_FUNCTION__);
}

bool nvme::parse_pci_config()
{
    // _pci_dev BAR index is 1-based
    _bar0 = _pci_dev.get_bar(1);

    if (_bar0 == nullptr) {
        return false;
    }
    // nvme_d("BAR0 is %s bit and %d bytes long\n",
    //     _bar0->is_64() ? "64" : "32",
    //     _bar0->get_size());

    _bar0->map();

    // auto _bar5 = _pci_dev.get_bar(5);
    // if (_bar5) {
    //     _bar5->map();
    // }

    if (!_pci_dev.get_bus_master()) {
        _pci_dev.set_bus_master(true);
    }

    if (_pci_dev.is_intx_enabled()) {
        nvme_d("INTx enabled");
    }

    if (_pci_dev.is_msi()) {
        nvme_d("MSI supported (%d entries)", _pci_dev.msi_get_num_entries());
    } else {
        nvme_d("MSI not supported");
    }

    if (_pci_dev.is_msix()) {
        nvme_d("MSI-X supported (%d entries)", _pci_dev.msix_get_num_entries());
    } else {
        nvme_d("MSI-X not supported");
    }

    return true;
}

hw_driver* nvme::probe(hw_device* hw_dev)
{
    if (auto pci_dev = dynamic_cast<pci::device*>(hw_dev)) {
        auto base_class = pci_dev->get_base_class_code();
        auto sub_class = pci_dev->get_sub_class_code();
        auto prog_if = pci_dev->get_programming_interface();
        auto vid = pci_dev->get_vendor_id();
        auto did = pci_dev->get_device_id();
        if (base_class == pci::function::PCI_CLASS_STORAGE &&
            sub_class == pci::function::PCI_SUB_CLASS_STORAGE_NVMC &&
            prog_if == pci::function::PCI_PROG_IF_ENTERPRISE_NVMHCI) {
                nvme_i("NVMe: Found NVMHCI - %hX:%hX", vid, did);
                return new nvme(*pci_dev);
            }
        }

    return nullptr;
}

} // namespace nvme
