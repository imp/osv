/*
 * Copyright (C) 2015 Cyril Plisko.
 *
 * This work is open source software, licensed under the terms of the
 * BSD license as described in the LICENSE file in the top-level directory.
 */

#ifndef NVME_HW_HH
#define NVME_HW_HH

namespace nvme {

enum nvme_controller_registers {
    NVME_CR_CAP        = 0x0000, // Controller Capabilities
    NVME_CR_VS         = 0x0008, // Version
    NVME_CR_INTMS      = 0x000C, // Interrupt Mask Set
    NVME_CR_INTMC      = 0x0010, // Interrupt Mask Clear
    NVME_CR_CC         = 0x0014, // Controller Configuration
    NVME_CR_CSTS       = 0x001C, // Controller Status
    NVME_CR_NSSR       = 0x0020, // NVM Subsystem Reset (Optional)
    NVME_CR_AQA        = 0x0024, // Admin Queue Attributes
    NVME_CR_ASQ        = 0x0028, // Admin Submission Queue Base Address
    NVME_CR_ACQ        = 0x0030, // Admin Completion Queue Base Address
    NVME_CR_CMBLOC     = 0x0038, // Controller Memory Buffer Location (Optional)
    NVME_CR_CMBSZ      = 0x003C, // Controller Memory Buffer Size     (Optional)
    NVME_CR_SQ0TDBL    = 0x1000, // Submission Queue 0 Tail Doorbell (Admin)
};

union controller_capabilities {
    uint64_t    raw;
    struct {
        uint64_t mqes       : 16;   // Maximum Queue Entries Supported
        uint64_t cqr        : 1;    // Contiguous Queues Required
        uint64_t ams        : 2;    // Arbitration Mechanism Supported
        uint64_t reserved1  : 5;
        uint64_t to         : 8;    // Timeout in 500 ms units
        uint64_t dstrd      : 4;    // Doorbell Stride
        uint64_t nssrs      : 1;    // NVM Subsystem Reset Supported
        uint64_t css        : 8;    // Command Sets Supported
        uint64_t reserved2  : 3;
        uint64_t mpsmin     : 4;    // Memory Page Size Minimum
        uint64_t mpsmax     : 4;    // Memory Page Size Maximum
        uint64_t reserved3  : 8;
    };
};

union controller_configuration {
    uint32_t    raw;
    struct {
        uint32_t en         : 1;    // Enable
        uint32_t reserved1  : 3;
        uint32_t css        : 3;    // I/O Command Set Selected
        uint32_t mps        : 4;    // Memory Page Size
        uint32_t ams        : 3;    // Arbitration Mechanism Selected
        uint32_t shn        : 2;    // Shutdown Notification
        uint32_t iosqes     : 4;    // I/O Submission Queue Entry Size
        uint32_t iocqes     : 4;    // I/O Completion Queue Entry Size
        uint32_t reserved2  : 8;
    };
};

enum shutdown_notification {
    NORMAL_SHUTDOWN = 1,
    ABRUPT_SHUTDOWN = 2,
};

enum io_command_set {
    NVM_COMMAND_SET = 0,
};

union controller_status {
    uint32_t    raw;
    struct {
        uint32_t rdy        : 1;    // Ready
        uint32_t cfs        : 1;    // Controller Fatal Status
        uint32_t shst       : 2;    // Shutdown Status
        uint32_t nssro      : 1;    // NVM Subsystem Reset Occurred
        uint32_t pp         : 1;    // Processing Paused
        uint32_t reserved1  : 26;
    };
};

enum shutdown_status {
    NORMAL_OPERATION = 0,
    SHUTDOWN_IN_PROGRESS = 1,
    SHUTDOWN_COMPLETE = 2,
};

enum nvme_reset {
    NVME_RESET = 0x4E564D65     // 'NVMe'
};

union admin_queue_attributes {
    uint32_t    raw;
    struct {
        uint32_t asqs       : 12;   // Admin Submission Queue Size
        uint32_t reserved1  : 4;
        uint32_t acqs       : 12;   // Admin Completion Queue Size
        uint32_t reserved2  : 4;
    };
};

union controller_memory_buffer_location {
    uint32_t    raw;
    struct {
        uint32_t bir        : 3;    // Base Indicator Register
        uint32_t reserved1  : 9;
        uint32_t ofst       : 20;   // Offset
    };
};

union controller_memory_buffer_size {
    uint32_t    raw;
    struct {
        uint32_t sqs        : 1;    // Submission Queue Support
        uint32_t cqs        : 1;    // Completion Queue Support
        uint32_t lists      : 1;    // PRP SGL List Support
        uint32_t rds        : 1;    // Read Data Support
        uint32_t wds        : 1;    // Write Data Support
        uint32_t reserved1  : 3;
        uint32_t szu        : 4;    // Size Units
        uint32_t sz         : 20;   // Size
    };
};

enum controller_memory_buffer_size_unit {
    CMBSZ_SZU_4KB       = 0x0,
    CMBSZ_SZU_64KB      = 0x1,
    CMBSZ_SZU_1MB       = 0x2,
    CMBSZ_SZU_16MB      = 0x3,
    CMBSZ_SZU_256MB     = 0x4,
    CMBSZ_SZU_4GB       = 0x5,
    CMBSZ_SZU_64GB      = 0x6,
};

enum nvme_version {
    NVME_1_0    = 0x00010000,
    NVME_1_1    = 0x00010100,
    NVME_1_2    = 0x00010200,
};

typedef struct {
    uint32_t opc            : 8;    // Opcode
    uint32_t fuse           : 2;    // Fused Operation
    uint32_t reserved       : 4;
    uint32_t psdt           : 2;    // PRP or SGL for Data Transfer
    uint32_t cid            : 16;   // Command Identifier
} cdw0_t;

enum admin_opcode {
    DELETE_SQ       = 0x00,
    CREATE_SQ       = 0x01,
    GET_LOG_PAGE    = 0x02,
    DELETE_CQ       = 0x04,
    CREATE_CQ       = 0x05,
    IDENTIFY        = 0x06,
    ABORT           = 0x08,
    SET_FEATURES    = 0x09,
    GET_FEATURES    = 0x0A,
    ASYNC_EVENT_REQ = 0x0C,
    NS_MANAGEMENT   = 0x0D,
    FW_COMMIT       = 0x10,
    FW_IMAGE_DL     = 0x11,
    NS_ATTACHMENT   = 0x15,
};

enum features {
    ARBITRATION                 = 0x01,
    POWER_MANAGEMENT,
    LBA_RANGE_TYPE,
    TEMPERATURE_THRESHOLD,
    ERROR_RECOVERY,
    VOLATILE_WRITE_CACHE,
    NUMBER_OF_QUEUES,
    INTERRUPT_COALESCING,
    INTERRUPT_VECTOR_CONFIGURATION,
    WRITE_ATOMICITY_NORMAL,
    ASYNC_EVENT_CONFIGURATION,
    AUTONOMOUS_POWER_STATE_TRANSITION,
    HOST_MEMORY_BUFFER,
    SOFTWARE_PROGRESS_MARKER    = 0x80,
    HOST_IDENTIFIER,
    RESERVATION_NOTIFICATION_MASK,
    RESERVATION_PERSISTANCE
};

typedef union {
    uint32_t cdw10;
    // Create Completion Queue
    struct {
        uint32_t qid        : 16;   // Queue Identifier
        uint32_t qsize      : 16;   // Queue Size
    };
    // Get/Set Features
    struct {
        uint32_t fid        : 8;    // Feature Identifier
        uint32_t sel        : 3;    // Select
        uint32_t reserved   : 20;
        uint32_t sv         : 1;    // Save
    };
} cdw10_t;

typedef union {
    uint32_t cdw11;
    // Create Completion Queue
    struct {
        uint32_t pc         : 1;    // Physically Contiguous
        uint32_t ien        : 1;    // Interrupts Enabled
        uint32_t reserved   : 14;
        uint32_t iv         : 16;   // Interrupt Vector
    } ccq;
    // Create Submission Queue
    struct {
        uint32_t pc         : 1;
        uint32_t qprio      : 2;
        uint32_t reserved   : 13;
        uint32_t cqid       : 16;
    } csq;
    // Set Features
    struct {
        uint32_t ncqr       : 16;
        uint32_t nsqr       : 16;
    };
} cdw11_t;

typedef union {
    uint32_t cdw12;
    struct {
        uint32_t nlb        : 16;   // Number of Logical Blocks
        uint32_t reserved   : 10;
        uint32_t prinfo     : 4;    // Protection Information Field
        uint32_t fua        : 1;    // Force Unit Access
        uint32_t lr         : 1;    // Limited Retry
    };
} cdw12_t;

enum queue_priority {
    URGENT  = 0x00,
    HIGH    = 0x01,
    MEDIUM  = 0x02,
    LOW     = 0x03,
};

enum sgl_descriptor_type {
    DATA_BLOCK      = 0x0,
    BIT_BUCKET      = 0x1,
    SEGMENT         = 0x2,
    LAST_SEGMENT    = 0x3,
    VENDOR_SPECIFIC = 0xF,
};

// Both Admin Command Set and NVM Command Set have same structure
struct submission_queue_entry {
    cdw0_t   cdw0;
    uint32_t nsid;
    uint64_t reserved0;
    uint64_t mptr;
    union {
        struct {
            uint64_t address;
            uint32_t length;
            uint32_t reserved1              : 24;
            uint32_t type_specific          : 4;
            uint32_t sgl_descriptor_type    : 4;
        };
        struct {
            uint64_t prp1;
            uint64_t prp2;
        };
    };
    union {
        uint64_t slba;
        struct {
            cdw10_t  cdw10;
            cdw11_t  cdw11;
        };
    };
    cdw12_t cdw12;
    uint32_t cdw13;
    uint32_t cdw14;
    uint32_t cdw15;
};

struct completion_queue_entry {
    uint32_t command_specific;
    uint32_t reserved0;
    uint32_t sqhd               : 16;   // SQ Head Pointer
    uint32_t sqid               : 16;   // SQ Identifier
    uint32_t cid                : 16;   // Command Identifier
    uint32_t p                  : 1;    // Phase
    uint32_t sc                 : 8;    // Status Code
    uint32_t sct                : 3;    // Status Code Type
    uint32_t reserved1          : 2;
    uint32_t m                  : 1;    // More
    uint32_t dnr                : 1;    // Do Not Retry
};

enum status_code {
    // Generic Command Status Codes
    SUCCESSFUL_COMPLETION       = 0x00,
    INVALID_COMMAND_OPCODE      = 0x01,
    INVALID_FIELD_IN_COMMAND    = 0x02,
    COMMAND_ID_CONFLICT         = 0x03,
    DATA_TRANSFER_ERROR         = 0x04,
    COMMAND_ABORTED_POWER_LOSS  = 0x05, // Command Aborted due to
                                        // Power Loss Notification
    INTERNAL_ERROR              = 0x06,
    COMMAND_ABORT_REQUESTED     = 0x07,
    COMMAND_ABORTED_SQ_DELETION = 0x08,
    COMMAND_ABORTED_FAILED_FUSED_COMMAND = 0x09,
    COMMAND_ABORTED_MISSING_FUSED_COMMAND = 0x0A,
    INVALID_NAMESPACE_OR_FORMAT = 0x0B,
    COMMAMD_SEQUENCE_ERROR      = 0x0C,
    INVALID_SGL_SEGMENT_DESC    = 0x0D,
    INVALID_NUMBER_OF_SGL_DESC  = 0x0E,
    DATA_SGL_LENGTH_INVALID     = 0x0F,
    METADATA_SGL_LENGTH_INVALID = 0x10,
    SGL_DESCRIPTOR_TYPE_INVALID = 0x11,
    INVALID_USE_OF_CMB          = 0x12, // Controller Memory Block invalid use
    PRP_OFFSET_INVALID          = 0x13,
    ATOMIC_WRITE_UNIT_EXCEEDED  = 0x14,
    // NVM Command Set Status Codes
    LBA_OUT_OF_RANGE            = 0x80,
    CAPACITY_EXCEEDED           = 0x81,
    NAMESPACE_NOT_READY         = 0x82,
    RESERVATION_CONFLICT        = 0x83,
    FORMAT_IN_PROGRESS          = 0x84
};

typedef struct power_state_descriptor {
    uint8_t     psd[32];
} psd_t;

typedef struct {
    uint32_t sglsupported   : 1;
    uint32_t reserved0      : 15;
    uint32_t bitbucket      : 1;
    uint32_t bytealignedmd  : 1;
    uint32_t sgloversize    : 1;
    uint32_t reserved1      : 13;
} sgls_t;

struct identify_data {
    uint16_t    vid;
    uint16_t    ssvid;
    char        sn[20];
    char        mn[40];
    char        fr[8];
    uint8_t     rab;
    uint8_t     ieee0;
    uint8_t     ieee1;
    uint8_t     ieee2;
    uint8_t     cmic;
    uint8_t     mdts;
    uint16_t    cntlid;
    uint32_t    ver;
    uint32_t    rtd3r;
    uint32_t    rtd3e;
    uint32_t    oaes;   // Optional Asynchronous Events Supported
    uint8_t     reserved0[144];
    uint8_t     mgmt[16];
    // 256 bytes
    uint16_t    oacs;   // Optional Admin Command Support
    uint8_t     acl;    // Abort Command limit
    uint8_t     aerl;   // Asynchronous Event Request Limit
    uint8_t     frmw;   // Firmware Updates
    uint8_t     lpa;    // Log Page Attributes
    uint8_t     elpe;   // Error Log Page Entries
    uint8_t     npss;   // Number of Power States Support
    uint8_t     avscc;  // Admin Vendor Specific Command Configuration
    uint8_t     apsta;  // Autonomous Power State Transition Attributes
    uint16_t    wctemp; // Warning Composite Temperature Threshold
    uint16_t    cctemp; // Critical Composite Temperature Threshold
    uint16_t    mtfa;   // Maximum Time for Firmware Activation
    uint32_t    hmpre;  // Host Memory Buffer Preferred Size
    uint32_t    hmmin;  // Host Memory Buffer Minimum Size
    uint64_t    tnvmcap[2]; // Total NVM Capacity
    uint64_t    unvmcap[2]; // Unallocated NVM Capacity
    uint32_t    rpmbs;  // Replay Protected Memory Block Support
    uint8_t     reserved1[196];
    // 512 bytes
    // NVM Command Set Attributes
    uint8_t     sqes;   // Submission Queue Entry Size
    uint8_t     cqes;   // Completion Queue Entry Size
    uint16_t    reserved2;
    uint32_t    nn;     // Number of Namespaces
    uint16_t    oncs;   // Optional NVM Command Support
    uint16_t    fuses;  // Fused Operation Support
    uint8_t     fna;    // Format NVM Attributes
    uint8_t     vwc;    // Volatile Write Cache
    uint16_t    awun;   // Atomic Write Unit Normal
    uint16_t    awupf;  // Atomic Write Unit Power Failed
    uint8_t     nvscc;  // NVM Vendor Specific Command Configuration
    uint8_t     reserved3;
    uint16_t    acwu;   // Atomic Compare & Write Unit
    uint16_t    reserved4;
    sgls_t      sgls;   // SGL Support
    uint8_t     reserved5[164];
    // 704 bytes
    // IO Command Set Attributes
    uint8_t     reserved6[1344];
    // 2048 bytes
    // Power States Descriptors
    psd_t       psd[32];// Power State 0-31 Descriptor;
    // 3072 bytes
    // Vendor Specific
    uint8_t     vs[1024];   // Vendor Specific
};

typedef struct lba_format_support {
    uint32_t ms         : 16;   // Metadata Size
    uint32_t lbads      : 8;    // LBA Data Size (power of two)
    uint32_t rp         : 2;    // Relative Performace
    uint32_t reserved   : 6;
} lbaf_t;

enum lba_format_relative_performance {
    BEST_PERFORMANCE        = 0x0,
    BETTER_PERFORMANCE      = 0x1,
    GOOD_PERFORMANCE        = 0x2,
    DEGRADED_PERFORMANCE    = 0x3,
};

struct identify_namespace_data {
    uint64_t    nsze;   // Namespace Size
    uint64_t    ncap;   // Namespace Capacity
    uint64_t    nuse;   // Namespace Utilization
    uint8_t     nsfeat; // Namespace features
    uint8_t     nlbaf;  // Number of LBA formats
    uint8_t     flbas;  // Formatted LBA Size
    uint8_t     mc;     // Metadata Capabilities
    uint8_t     dpc;    // End-to-end Data Protection Capabilities
    uint8_t     dps;    // End-to-end Data Protection Type Settings
    uint8_t     nmic;   // Namespace Multi-path IO and Namespace Sharing Capabilities
    uint8_t     rescap; // Reservation Capabilities
    uint8_t     fpi;    // Format Progress Indicator
    uint8_t     reserved0;
    uint16_t    nawun;  // Namespace Atomic Write Unit Normal
    uint16_t    nawupf; // Namespace Atomic Write Unit Power Fail
    uint16_t    nacwu;  // Namespace Atomic Compare & Write Unit
    uint16_t    nabsn;  // Namespace Atomic Boundary Size Normal
    uint16_t    nabo;   // Namespace Atomic Boundary Offset
    uint16_t    nabspf; // Namespace Atomic Boundary Size Power Fail
    uint16_t    reserved1;
    uint64_t    nvmcap[2];  // Namespace Capacity
    // 64 bytes
    uint8_t     reserved2[40];
    uint8_t     nguid[16];  // Namespace GUID
    uint8_t     eui64[8];   // IEEE Extended Unique Identifier
    // 128 bytes
    lbaf_t      lbaf[16]; // LBA Format 0-15 Support
    // 192 bytes
    uint8_t     reserved3[192];
    // 384 bytes
    uint8_t     vs[3712];   // Vendor Specific
};

// NVM Command Set
enum nvm_opcode {
    FLUSH                   = 0x00,
    WRITE                   = 0x01,
    READ                    = 0x02,
    WRITE_UNCORRECTABLE     = 0x04,
    COMPARE                 = 0x05,
    WRITE_ZEROES            = 0x08,
    DATASET_MANAGEMENT      = 0x09,
    RESERVATION_REGISTER    = 0x0D,
    RESERVATION_REPORT      = 0x0E,
    RESERVATION_ACQUIRE     = 0x11,
    RESERVATION_RELEASE     = 0x15,
};

} // namespace nvme

#endif // NVME_HW_HH
