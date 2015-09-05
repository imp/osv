/*
 * Copyright (C) 2015 Cyril Plisko.
 *
 * This work is open source software, licensed under the terms of the
 * BSD license as described in the LICENSE file in the top-level directory.
 */

#ifndef NVME_HH
#define NVME_HH

#include "driver.hh"
#include <osv/pci.hh>
#include "drivers/driver.hh"
#include "drivers/pci-function.hh"
#include "drivers/pci-device.hh"
#include <osv/interrupt.hh>
#include <osv/msi.hh>
#include <osv/mmu.hh>
#include <osv/mempool.hh>
#include <osv/bio.h>
#include <osv/types.h>

#include "nvme_hw.hh"

namespace nvme {

#define nvme_tag "nvme"
#define nvme_d(...)   tprintf_d(nvme_tag, __VA_ARGS__)
#define nvme_i(...)   tprintf_i(nvme_tag, __VA_ARGS__)
#define nvme_w(...)   tprintf_w(nvme_tag, __VA_ARGS__)
#define nvme_e(...)   tprintf_e(nvme_tag, __VA_ARGS__)

struct nvme_state;

template <typename T>
class queue {
public:
    queue(size_t nentries, size_t align)
        : _nentries(nentries)
        , _base(memory::make_phys_array<T>(nentries, align))
    {
        for (size_t i = 0; i < _nentries; i++) {
            _base[i] = {};
        }
    }

    mmu::phys pa() const { return memory::virt_to_phys(_base); }
    size_t size() const { return _nentries; }

    bool empty() const { return _tail == _head; }
    bool full() const { return _tail - _head == 1; } // FIXME handle wraparound

    // Head/Tail manipulators
    uint32_t head() const { return _head; }
    void head(uint32_t h) { _head = h; }
    uint32_t tail() const { return _tail; }
    void tail(uint32_t t) { _tail = t; }
    int phase() const { return _phase; }

    void push(const T& cmd) {
        //stats(__PRETTY_FUNCTION__);
        assert(!full());

        _base[_tail++] = cmd;
        _tail %= _nentries;
    }

    T& pop() {
        //stats(__PRETTY_FUNCTION__);
        //assert(!empty());

        T& qentry = _base[_head++];
        _head %= _nentries;
        // CRAZY HW ENGINEER ALERT ! On wrap around phase is flipping
        if (_head == 0) _phase = !_phase;

        return qentry;
    }

    T& peek() const { return _base[_head]; }

    void stats(const char* f) const {
        nvme_d("%s\n\t_base va/pa %p/%p\n\t_nentries/_tail_/head %d/%d/%d",
            f, &_base[0], pa(), _nentries, _tail, _head);
    }

protected:
    size_t _nentries;
    size_t _entry_size = sizeof(T);
    size_t _head = 0;
    size_t _tail = 0;
    int _phase = 1;
    memory::phys_ptr<T[]> _base;
};

class qpair {
public:
    qpair(size_t nentries, size_t align,
        std::function<void (uint32_t)> tdb,
        std::function<void (uint32_t)> hdb);

    mmu::phys submission_queue_addr() const { return _submission_queue.pa(); }
    mmu::phys completion_queue_addr() const { return _completion_queue.pa(); }
    size_t submission_queue_size() const { return _submission_queue.size(); }
    size_t completion_queue_size() const { return _completion_queue.size(); }

    void submit(submission_queue_entry& sqe);
    //uint32_t readslot(uint32_t index) const {}
    uint32_t cq_head() const { return _completion_queue.head(); }

    bool exec_sync_cmd(submission_queue_entry& sqe,
        completion_queue_entry& cqe, bool poll=false);

    bool exec_sync_cmd(submission_queue_entry& sqe, bool poll=false) {
        completion_queue_entry cqe;
        return exec_sync_cmd(sqe, cqe, poll);
    }

    const char* sc_to_str(uint8_t sc) const;
    void dump_sqe(submission_queue_entry& sqe) const;
    void dump_cqe(completion_queue_entry& cqe) const;

protected:
    uint16_t _cid = 0;
    queue<submission_queue_entry> _submission_queue;
    queue<completion_queue_entry> _completion_queue;
    std::function<void (uint32_t)> _tail_doorbell_update;
    std::function<void (uint32_t)> _head_doorbell_update;
};

class nvme_namespace {
public:
    nvme_namespace(uint32_t nsid, identify_namespace_data* ind);

    uint32_t nsid() const { return _nsid; }
    size_t size() const { return _size; }
    size_t capacity() const { return _capacity; }
    size_t lbasize() const { return _lbasize; }

    void name(const std::string& n) { _name = n; }
    const char* name() const { return _name.c_str(); }

    void nvm_flush(struct bio *bio);
    void nvm_rw(struct bio* bio);
    int make_request(struct bio* bio);

    void report_nvme_namespace() const;

protected:
    uint32_t _nsid;
    size_t _size;
    size_t _capacity;
    size_t _utilization;
    size_t _lbasize;
    uint8_t _guid[16];
    mutex _lock;
    std::string _name;
};

class nvme : public hw_driver {
public:
    nvme(pci::device& pci_dev);
    ~nvme();

    virtual void dump_config();

    static struct nvme_state* get_state(struct bio* bio) {
        return reinterpret_cast<struct nvme_state*>(bio->bio_dev->private_data);
    }
    bool poll_mode() { return _poll_mode; }
    pci::device& pci_device() { return _pci_dev; }
    static hw_driver* probe(hw_device* hw_dev);
    virtual std::string get_name() const { return _driver_name; }
    bool parse_pci_config();

    uint32_t sq_tail_doorbell(int i) {
        return NVME_CR_SQ0TDBL + 2 * i * (4 << _doorbell_stride);
    }
    uint32_t cq_head_doorbell(int i) {
        return NVME_CR_SQ0TDBL + (2 * i + 1) * (4 << _doorbell_stride);
    }

    void report_controller_capabilities(controller_capabilities cap) const;
    void report_identify_data(const identify_data& id) const;
    void report_controller();
    bool ack_irq();
    void enable_irq();
    void reset();
    void shutdown();
    void setup();
    void scan();
    void add_namespace(uint32_t nsid, nvme_namespace* ns);
    qpair* setup_io_qpair(size_t nentries, int qn);

    void identify_controller();
    nvme_namespace* identify_namespace(uint32_t nsid);
    void get_number_of_queues();
    void set_number_of_queues(uint16_t ncqr, uint16_t nsqr);

    // interrupt handler
    bool isr();
    void drain_io_queue();

    // Admin commands
    bool delete_io_submission_queue(uint16_t qid);
    bool create_io_submission_queue(uint16_t qid, uint16_t qsize, mmu::phys pa);
    bool get_log_page();
    bool delete_io_completion_queue(uint16_t qid);
    bool create_io_completion_queue(uint16_t qid, uint16_t qsize, mmu::phys pa);
    bool identify(cdw10_t cdw10, uint32_t nsid, mmu::phys pa);
    bool abort(uint16_t cid, uint16_t sqid);
    bool get_features(uint8_t feature, completion_queue_entry& cqe);
    bool set_features(uint8_t feature, cdw11_t cdw11,
        completion_queue_entry& cqe);
    bool async_event_request();

    // IO Commands
    bool flush(uint16_t nsid);
    bool readwrite(uint16_t nsid, enum nvm_opcode opc, mmu::phys data,
        uint32_t len, uint64_t slba, uint16_t nlb);

    // Access NVMHCI config space
    uint8_t nvme_get8(uint32_t off)   { return _bar0->readb(off); }
    uint16_t nvme_get16(uint32_t off) { return _bar0->readw(off); }
    uint32_t nvme_get32(uint32_t off) { return _bar0->readl(off); }
    uint64_t nvme_get64(uint32_t off) { return _bar0->readq(off); }

    void nvme_put8(uint32_t off, uint8_t val)   { _bar0->writeb(off, val); }
    void nvme_put16(uint32_t off, uint16_t val) { _bar0->writew(off, val); }
    void nvme_put32(uint32_t off, uint32_t val) { _bar0->writel(off, val); }
    void nvme_put64(uint32_t off, uint64_t val) { _bar0->writeq(off, val); }

private:
    // Somewhat arbitrary chosen default max data transfer size
    static constexpr size_t MAX_XFER_SIZE = 1ULL << 20;

    static std::string _driver_name;
    static int ctrl_idx;
    pci::device& _pci_dev;
    bool _poll_mode = true;
    interrupt_manager _msix;
    pci::bar *_bar0 = nullptr;

    // Hardware properties
    uint32_t _version;
    size_t _mpsmin, _mpsmax, _mps;
    size_t _max_xfer_size;
    uint64_t _doorbell_stride;
    uint32_t _nn;
    // These look like magic numbers, and indeed, there is some magic involved
    // in setting them. They are supposed to be discoverable from the "identify
    // controller" data. Alas, in order to perform initial setup one has to know
    // these numbers. So we are starting from knowing good values, and later
    // we will have a chance to adjust them if needed.
    int _sqes = 6;
    int _cqes = 4;

    bool _sgl_ok = false;

    std::string _serial;
    std::string _model;
    std::string _firmware;

    qpair* _admin_queue = nullptr;
    qpair* _io_queue = nullptr;
    std::map<uint32_t, nvme_namespace*> _namespaces;
};

} // namespace nvme
#endif // NVME_HH
