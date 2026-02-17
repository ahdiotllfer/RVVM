#ifndef RVVM_VIRTIO_FS_H
#define RVVM_VIRTIO_FS_H

#include "rvvmlib.h"

#define VIRTIO_FS_MMIO_ADDR_DEFAULT 0x10001000U

PUBLIC rvvm_mmio_dev_t* virtio_fs_init(rvvm_machine_t* machine, rvvm_addr_t addr, rvvm_intc_t* intc, rvvm_irq_t irq,
                                       const char* tag, const char* shared_dir);

PUBLIC rvvm_mmio_dev_t* virtio_fs_init_auto(rvvm_machine_t* machine, const char* tag, const char* shared_dir);

#endif
