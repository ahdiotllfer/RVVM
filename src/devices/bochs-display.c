/*
bochs-display.c - Bochs Display
Copyright (C) 2025  LekKit <github.com/LekKit>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include "bochs-display.h"

#include "mem_ops.h"
#include "utils.h"
#include "vma_ops.h"

PUSH_OPTIMIZATION_SIZE

typedef struct {
    rvvm_fbdev_t* fbdev;

    uint32_t version;
    uint32_t enable;
    uint32_t xres;
    uint32_t yres;
    uint32_t xvirt;
    uint32_t yvirt;
    uint32_t xoff;
    uint32_t yoff;
} bochs_display_t;

/*
 * Bochs Display registers
 */
#define BOCHS_REG_ID          0x0500 // Version ID (0xBOC0 - 0xBOC5)
#define BOCHS_REG_XRES        0x0502 // X Resolution
#define BOCHS_REG_YRES        0x0504 // Y Resolution
#define BOCHS_REG_BPP         0x0506 // Bits per pixel (32bpp is XRGB8888)
#define BOCHS_REG_ENABLE      0x0508 // Enable register
#define BOCHS_REG_BANK        0x050A // Current 64KiB VRAM bank exposed at 0xA0000 (x86 VGA only)
#define BOCHS_REG_VIRT_WIDTH  0x050C // Framebuffer stride
#define BOCHS_REG_VIRT_HEIGHT 0x050E // Not meaningful, should be set >= YRES by the guest
#define BOCHS_REG_X_OFFSET    0x0510 // Framebuffer X offset in VRAM
#define BOCHS_REG_Y_OFFSET    0x0512 // Framebuffer Y offset in VRAM
#define BOCHS_REG_VRAM        0x0514 // VRAM size (In 64KiB units, i.e. shifted right by 16)

/*
 * Extended registers (Added in device PCI Revision 2)
 */
#define BOCHS_REG_QEXT_SIZE   0x0600 // Extended registers region size, should be 8
#define BOCHS_REG_ENDIAN_LO   0x0604 // Framebuffer endianness register, should be 0x1E1E
#define BOCHS_REG_ENDIAN_HI   0x0606 // Framebuffer endianness register, should be 0x1E1E

/*
 * Bochs Display verions in ID register
 */
#define BOCHS_VER_ID0         0xB0C0 // Bochs VBE version 0
#define BOCHS_VER_ID5         0xB0C5 // Bochs VBE version 5

/*
 * Enable register bits
 */
#define BOCHS_ENABLE          0x01 // Enable the display engine, applies XRES/YRES/BPP and disallows writes to them
#define BOCHS_ENABLE_CAPS     0x02 // Enable capabilities (XRES/YRES/BPP report max values instead)
#define BOCHS_ENABLE_8BIT     0x20 // Enable 8-bit DAC (x86 VGA only)
#define BOCHS_ENABLE_LFB      0x40 // Enable Linear Framebuffer in BAR 0 (x86 VGA only)
#define BOCHS_ENABLE_NOCLR    0x80 // Do not zero VRAM on enable
#define BOCHS_ENABLE_MASK     0xE3 // Mask of valid bits

static void bochs_display_update_mode(bochs_display_t* disp, bool upd_res)
{
    rvvm_fb_t fb   = ZERO_INIT;
    uint8_t*  vram = rvvm_fbdev_get_vram(disp->fbdev, NULL);
    uint32_t  off  = atomic_load_uint32_relax(&disp->xoff);
    rvvm_fbdev_get_scanout(disp->fbdev, &fb);
    if (upd_res) {
        fb.width  = atomic_load_uint32_relax(&disp->xres);
        fb.height = atomic_load_uint32_relax(&disp->yres);
    }
    fb.stride = EVAL_MAX(atomic_load_uint32_relax(&disp->xvirt), fb.width * 4);
    fb.format = RVVM_RGB_XRGB8888;
    fb.buffer = vram + off + (atomic_load_uint32_relax(&disp->yoff) * fb.stride);
    rvvm_fbdev_set_scanout(disp->fbdev, &fb);
}

static bool bochs_display_read(rvvm_mmio_dev_t* dev, void* data, size_t offset, uint8_t size)
{
    bochs_display_t* disp = dev->data;
    uint16_t         val  = 0;
    UNUSED(size);

    switch (offset) {
        case BOCHS_REG_ID:
            val = atomic_load_uint32_relax(&disp->version);
            if (val < BOCHS_VER_ID0 || val > BOCHS_VER_ID5) {
                val = BOCHS_VER_ID5;
            }
            break;
        case BOCHS_REG_XRES:
            if (atomic_load_uint32_relax(&disp->enable) & BOCHS_ENABLE_CAPS) {
                val = 2560;
            } else {
                val = atomic_load_uint32_relax(&disp->xres);
            }
            break;
        case BOCHS_REG_YRES:
            if (atomic_load_uint32_relax(&disp->enable) & BOCHS_ENABLE_CAPS) {
                val = 1440;
            } else {
                val = atomic_load_uint32_relax(&disp->yres);
            }
            break;
        case BOCHS_REG_BPP:
            // NOTE: Only claim bpp32 (XRGB8888) support for now
            val = 32;
            break;
        case BOCHS_REG_ENABLE:
            val = atomic_load_uint32_relax(&disp->enable);
            break;
        case BOCHS_REG_VIRT_WIDTH:
            val = atomic_load_uint32_relax(&disp->xvirt);
            break;
        case BOCHS_REG_VIRT_HEIGHT:
            val = atomic_load_uint32_relax(&disp->yvirt);
            break;
        case BOCHS_REG_X_OFFSET:
            val = atomic_load_uint32_relax(&disp->xoff);
            break;
        case BOCHS_REG_Y_OFFSET:
            val = atomic_load_uint32_relax(&disp->yoff);
            break;
        case BOCHS_REG_VRAM: {
            size_t vram_size = RVVM_BOCHS_DISPLAY_VRAM;
            rvvm_fbdev_get_vram(disp->fbdev, &vram_size);
            val = vram_size >> 16;
            break;
        }
        case BOCHS_REG_QEXT_SIZE:
            val = 8;
            break;
        case BOCHS_REG_ENDIAN_LO:
        case BOCHS_REG_ENDIAN_HI:
            val = 0x1E1EU;
            break;
    }

    write_uint16_le(data, val);
    return true;
}

static bool bochs_display_write(rvvm_mmio_dev_t* dev, void* data, size_t offset, uint8_t size)
{
    bochs_display_t* disp = dev->data;
    uint16_t         val  = read_uint16_le(data);
    UNUSED(size);

    switch (offset) {
        case BOCHS_REG_ID:
            atomic_store_uint32_relax(&disp->version, val);
            break;
        case BOCHS_REG_XRES:
            if (!(atomic_load_uint32_relax(&disp->enable) & BOCHS_ENABLE)) {
                atomic_store_uint32_relax(&disp->xres, val);
            }
            break;
        case BOCHS_REG_YRES:
            if (!(atomic_load_uint32_relax(&disp->enable) & BOCHS_ENABLE)) {
                atomic_store_uint32_relax(&disp->yres, val);
            }
            break;
        case BOCHS_REG_VIRT_WIDTH:
            atomic_store_uint32_relax(&disp->xvirt, val);
            bochs_display_update_mode(disp, false);
            break;
        case BOCHS_REG_VIRT_HEIGHT:
            atomic_store_uint32_relax(&disp->yvirt, val);
            break;
        case BOCHS_REG_X_OFFSET:
            atomic_store_uint32_relax(&disp->xoff, val);
            bochs_display_update_mode(disp, false);
            break;
        case BOCHS_REG_Y_OFFSET:
            atomic_store_uint32_relax(&disp->yoff, val);
            bochs_display_update_mode(disp, false);
            break;
        case BOCHS_REG_ENABLE:
            atomic_store_uint32_relax(&disp->enable, val & BOCHS_ENABLE_MASK);
            if (val & BOCHS_ENABLE) {
                if (!(val & BOCHS_ENABLE_NOCLR)) {
                    // Clear VRAM
                    size_t vsiz = 0;
                    void*  vram = rvvm_fbdev_get_vram(disp->fbdev, &vsiz);
                    vma_clean(vram, vsiz, false);
                }
                // Fully update video mode
                bochs_display_update_mode(disp, true);
            }
            break;
    }

    return true;
}

static bool bochs_vram_write(rvvm_mmio_dev_t* dev, void* data, size_t offset, uint8_t size)
{
    bochs_display_t* disp = dev->data;
    rvvm_fbdev_dirty(disp->fbdev);
    UNUSED(data);
    UNUSED(offset);
    UNUSED(size);
    return true;
}

static void bochs_display_remove(rvvm_mmio_dev_t* dev)
{
    bochs_display_t* disp = dev->data;
    if (rvvm_fbdev_dec_ref(disp->fbdev)) {
        safe_free(disp);
    }
}

static void bochs_display_update(rvvm_mmio_dev_t* dev)
{
    bochs_display_t* disp = dev->data;
    rvvm_fbdev_update(disp->fbdev);
}

static void bochs_vram_suspend(rvvm_mmio_dev_t* dev, rvvm_state_t* state)
{
    if (!dev || !state) {
        return;
    }
    if (!dev->size) {
        rvvm_state_fail(state);
        return;
    }
    if (dev->size < 4) {
        if (!dev->mapping) {
            rvvm_state_fail(state);
            return;
        }
        rvvm_state_write(state, dev->mapping, dev->size);
        return;
    }

    bochs_display_t* disp = dev->data;
    const bool headless = !disp || rvvm_fbdev_is_headless(disp->fbdev);
    const char hdr[4] = {'R', 'V', 'F', 'B'};
    rvvm_state_write(state, hdr, sizeof(hdr));
    rvvm_state_write_u32(state, headless ? 0U : 1U);
    if (headless) {
        return;
    }
    if (!dev->mapping) {
        rvvm_state_fail(state);
        return;
    }
    rvvm_state_write(state, dev->mapping, dev->size);
}

static void bochs_vram_resume(rvvm_mmio_dev_t* dev, rvvm_state_t* state)
{
    if (!dev || !state) {
        return;
    }
    if (!dev->size) {
        rvvm_state_fail(state);
        return;
    }
    if (dev->size < 4) {
        if (!dev->mapping) {
            rvvm_state_fail(state);
            return;
        }
        rvvm_state_read(state, dev->mapping, dev->size);
        bochs_display_t* disp = dev->data;
        if (disp) {
            rvvm_fbdev_dirty(disp->fbdev);
        }
        return;
    }

    char hdr[4] = {0};
    rvvm_state_read(state, hdr, sizeof(hdr));
    if (memcmp(hdr, "RVFB", 4) == 0) {
        uint32_t has_data = 0;
        rvvm_state_read_u32(state, &has_data);
        if (has_data) {
            if (!dev->mapping) {
                rvvm_state_fail(state);
                return;
            }
            rvvm_state_read(state, dev->mapping, dev->size);
        }
        bochs_display_t* disp = dev->data;
        if (disp) {
            rvvm_fbdev_dirty(disp->fbdev);
        }
        return;
    }

    if (!dev->mapping) {
        rvvm_state_fail(state);
        return;
    }
    memcpy(dev->mapping, hdr, sizeof(hdr));
    rvvm_state_read(state, ((uint8_t*)dev->mapping) + sizeof(hdr), dev->size - sizeof(hdr));
    bochs_display_t* disp = dev->data;
    if (disp) {
        rvvm_fbdev_dirty(disp->fbdev);
    }
}

static void bochs_display_suspend(rvvm_mmio_dev_t* dev, rvvm_state_t* state)
{
    if (!dev || !state) {
        return;
    }
    bochs_display_t* disp = dev->data;
    if (!disp) {
        rvvm_state_fail(state);
        return;
    }
    rvvm_state_write_u32(state, atomic_load_uint32_relax(&disp->version));
    rvvm_state_write_u32(state, atomic_load_uint32_relax(&disp->enable));
    rvvm_state_write_u32(state, atomic_load_uint32_relax(&disp->xres));
    rvvm_state_write_u32(state, atomic_load_uint32_relax(&disp->yres));
    rvvm_state_write_u32(state, atomic_load_uint32_relax(&disp->xvirt));
    rvvm_state_write_u32(state, atomic_load_uint32_relax(&disp->yvirt));
    rvvm_state_write_u32(state, atomic_load_uint32_relax(&disp->xoff));
    rvvm_state_write_u32(state, atomic_load_uint32_relax(&disp->yoff));
}

static void bochs_display_resume(rvvm_mmio_dev_t* dev, rvvm_state_t* state)
{
    if (!dev || !state) {
        return;
    }
    bochs_display_t* disp = dev->data;
    if (!disp) {
        rvvm_state_fail(state);
        return;
    }

    uint32_t version = 0;
    uint32_t enable = 0;
    uint32_t xres = 0;
    uint32_t yres = 0;
    uint32_t xvirt = 0;
    uint32_t yvirt = 0;
    uint32_t xoff = 0;
    uint32_t yoff = 0;

    rvvm_state_read_u32(state, &version);
    rvvm_state_read_u32(state, &enable);
    rvvm_state_read_u32(state, &xres);
    rvvm_state_read_u32(state, &yres);
    rvvm_state_read_u32(state, &xvirt);
    rvvm_state_read_u32(state, &yvirt);
    rvvm_state_read_u32(state, &xoff);
    rvvm_state_read_u32(state, &yoff);

    atomic_store_uint32_relax(&disp->version, version);
    atomic_store_uint32_relax(&disp->enable, enable);
    atomic_store_uint32_relax(&disp->xres, xres);
    atomic_store_uint32_relax(&disp->yres, yres);
    atomic_store_uint32_relax(&disp->xvirt, xvirt);
    atomic_store_uint32_relax(&disp->yvirt, yvirt);
    atomic_store_uint32_relax(&disp->xoff, xoff);
    atomic_store_uint32_relax(&disp->yoff, yoff);

    if (enable & BOCHS_ENABLE) {
        bochs_display_update_mode(disp, true);
    } else {
        bochs_display_update_mode(disp, false);
    }
    rvvm_fbdev_dirty(disp->fbdev);
}

static const rvvm_mmio_type_t bochs_vram_type = {
    .name   = "bochs_vram",
    .remove = bochs_display_remove,
    .suspend = bochs_vram_suspend,
    .resume  = bochs_vram_resume,
};

static const rvvm_mmio_type_t bochs_display_type = {
    .name   = "bochs_display",
    .remove = bochs_display_remove,
    .update = bochs_display_update,
    .suspend = bochs_display_suspend,
    .resume  = bochs_display_resume,
};

pci_dev_t* rvvm_bochs_display_init(pci_bus_t* pci_bus, rvvm_fbdev_t* fbdev)
{
    bochs_display_t* disp = safe_new_obj(bochs_display_t);

    size_t vram_size = RVVM_BOCHS_DISPLAY_VRAM;
    void*  vram      = rvvm_fbdev_get_vram(fbdev, &vram_size);

    // Handle is released twice
    rvvm_fbdev_inc_ref(fbdev);
    disp->fbdev = fbdev;

    pci_func_desc_t bochs_desc = {
        .vendor_id  = 0x1234, // Not in PCI ID database yet, should be Bochs
        .device_id  = 0x1111, // Not in PCI ID database yet, should be Bochs-Display
        .class_code = 0x0380, // Display controller (Legacy-free, no VGA)
        .rev        = 2,      // Revision 2
        .bar[0] = {
            .size    = vram_size,
            .data    = disp,
            .mapping = vram,
            .type    = &bochs_vram_type,
            .write   = bochs_vram_write,
        },
        .bar[2] = {
            .size        = 0x1000,
            .data        = disp,
            .type        = &bochs_display_type,
            .read        = bochs_display_read,
            .write       = bochs_display_write,
            .min_op_size = 2,
            .max_op_size = 2,
        },
    };

    pci_dev_t* pci_dev = pci_attach_func(pci_bus, &bochs_desc);
    return pci_dev;
}

pci_dev_t* rvvm_bochs_display_init_auto(rvvm_machine_t* machine, rvvm_fbdev_t* fbdev)
{
    return rvvm_bochs_display_init(rvvm_get_pci_bus(machine), fbdev);
}

POP_OPTIMIZATION_SIZE
