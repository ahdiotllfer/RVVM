#include "hid_api.h"

#include "fdtlib.h"
#include "mem_ops.h"
#include "spinlock.h"
#include "utils.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

enum {
    VIRTIO_MMIO_MAGIC               = 0x000,
    VIRTIO_MMIO_VERSION             = 0x004,
    VIRTIO_MMIO_DEVICE_ID           = 0x008,
    VIRTIO_MMIO_VENDOR_ID           = 0x00c,
    VIRTIO_MMIO_DEVICE_FEATURES     = 0x010,
    VIRTIO_MMIO_DEVICE_FEATURES_SEL = 0x014,
    VIRTIO_MMIO_DRIVER_FEATURES     = 0x020,
    VIRTIO_MMIO_DRIVER_FEATURES_SEL = 0x024,
    VIRTIO_MMIO_QUEUE_SEL           = 0x030,
    VIRTIO_MMIO_QUEUE_NUM_MAX       = 0x034,
    VIRTIO_MMIO_QUEUE_NUM           = 0x038,
    VIRTIO_MMIO_QUEUE_READY         = 0x044,
    VIRTIO_MMIO_QUEUE_NOTIFY        = 0x050,
    VIRTIO_MMIO_INTERRUPT_STATUS    = 0x060,
    VIRTIO_MMIO_INTERRUPT_ACK       = 0x064,
    VIRTIO_MMIO_STATUS              = 0x070,
    VIRTIO_MMIO_QUEUE_DESC_LOW      = 0x080,
    VIRTIO_MMIO_QUEUE_DESC_HIGH     = 0x084,
    VIRTIO_MMIO_QUEUE_DRIVER_LOW    = 0x090,
    VIRTIO_MMIO_QUEUE_DRIVER_HIGH   = 0x094,
    VIRTIO_MMIO_QUEUE_DEVICE_LOW    = 0x0a0,
    VIRTIO_MMIO_QUEUE_DEVICE_HIGH   = 0x0a4,
    VIRTIO_MMIO_CONFIG_GENERATION   = 0x0fc,
    VIRTIO_MMIO_CONFIG              = 0x100,
};

enum {
    VIRTIO_MMIO_MAGIC_VALUE   = 0x74726976,
    VIRTIO_MMIO_VERSION_VALUE = 2,
    VIRTIO_MMIO_VENDOR_VALUE  = 0x5256564d,
    VIRTIO_ID_INPUT           = 18,
};

enum {
    VIRTIO_F_VERSION_1       = 32,
    VIRTIO_F_ACCESS_PLATFORM = 33,
};

enum {
    VIRTIO_RING_F_INDIRECT_DESC = 28,
};

enum {
    VIRTQ_DESC_F_NEXT     = 1,
    VIRTQ_DESC_F_WRITE    = 2,
    VIRTQ_DESC_F_INDIRECT = 4,
};

enum {
    VIRTIO_INPUT_CFG_UNSET    = 0x00,
    VIRTIO_INPUT_CFG_ID_NAME  = 0x01,
    VIRTIO_INPUT_CFG_ID_SERIAL = 0x02,
    VIRTIO_INPUT_CFG_ID_DEVIDS = 0x03,
    VIRTIO_INPUT_CFG_PROP_BITS = 0x10,
    VIRTIO_INPUT_CFG_EV_BITS   = 0x11,
    VIRTIO_INPUT_CFG_ABS_INFO  = 0x12,
};

enum {
    EV_SYN = 0x00,
    EV_KEY = 0x01,
    EV_REL = 0x02,
    EV_ABS = 0x03,
};

enum {
    SYN_REPORT = 0,
};

enum {
    REL_X     = 0,
    REL_Y     = 1,
    REL_WHEEL = 8,
};

enum {
    ABS_X = 0,
    ABS_Y = 1,
};

enum {
    BTN_LEFT   = 0x110,
    BTN_RIGHT  = 0x111,
    BTN_MIDDLE = 0x112,
};

typedef struct __attribute__((packed)) {
    uint64_t addr;
    uint32_t len;
    uint16_t flags;
    uint16_t next;
} virtq_desc_t;

typedef struct __attribute__((packed)) {
    uint16_t flags;
    uint16_t idx;
    uint16_t ring[];
} virtq_avail_t;

typedef struct __attribute__((packed)) {
    uint32_t id;
    uint32_t len;
} virtq_used_elem_t;

typedef struct __attribute__((packed)) {
    uint16_t flags;
    uint16_t idx;
    virtq_used_elem_t ring[];
} virtq_used_t;

typedef struct {
    uint16_t num;
    uint8_t  ready;
    uint16_t last_avail_idx;
    uint16_t used_idx;
    uint64_t desc_addr;
    uint64_t avail_addr;
    uint64_t used_addr;
} virtio_queue_t;

typedef struct __attribute__((packed)) {
    uint16_t type;
    uint16_t code;
    int32_t  value;
} virtio_input_event_t;

typedef struct __attribute__((packed)) {
    int32_t  min;
    int32_t  max;
    int32_t  fuzz;
    int32_t  flat;
    int32_t  res;
} virtio_input_absinfo_t;

typedef struct __attribute__((packed)) {
    uint16_t bus;
    uint16_t vendor;
    uint16_t product;
    uint16_t version;
} virtio_input_devids_t;

typedef struct __attribute__((packed)) {
    uint8_t select;
    uint8_t subsel;
    uint8_t size;
    uint8_t reserved[5];
    union {
        uint8_t bitmap[128];
        virtio_input_absinfo_t abs;
        virtio_input_devids_t ids;
        uint8_t string[128];
    } u;
} virtio_input_config_t;

typedef struct {
    rvvm_machine_t* machine;
    rvvm_intc_t*    intc;
    rvvm_irq_t      irq;

    spinlock_t lock;

    uint32_t host_features[2];
    uint32_t driver_features[2];
    uint32_t host_features_sel;
    uint32_t driver_features_sel;

    uint32_t status;
    uint32_t interrupt_status;
    uint32_t queue_sel;
    uint32_t config_generation;

    virtio_queue_t q[2];
    virtio_input_config_t cfg;

    uint16_t width;
    uint16_t height;

    uint8_t is_keyboard;

    virtio_input_event_t pending[256];
    uint16_t pending_head;
    uint16_t pending_tail;
} virtio_input_dev_t;

struct hid_keyboard {
    virtio_input_dev_t* dev;
};

struct hid_mouse {
    virtio_input_dev_t* dev;
    uint32_t abs_x;
    uint32_t abs_y;
};

static inline uint32_t mmio_load_u32(const void* data, uint8_t size)
{
    if (size == 1) {
        return read_uint8(data);
    }
    if (size == 2) {
        return read_uint16_le(data);
    }
    return read_uint32_le(data);
}

static inline void mmio_store_u32(void* data, uint32_t val, uint8_t size)
{
    if (size == 1) {
        write_uint8(data, (uint8_t)val);
        return;
    }
    if (size == 2) {
        write_uint16_le(data, (uint16_t)val);
        return;
    }
    write_uint32_le(data, val);
}

static void* dma_ptr(virtio_input_dev_t* v, uint64_t addr, size_t len)
{
    return rvvm_get_dma_ptr(v->machine, (rvvm_addr_t)addr, len);
}

static bool virtq_load_desc_at(virtio_input_dev_t* v, uint64_t table_addr, uint32_t table_entries, uint16_t idx, virtq_desc_t* out)
{
    if (!v || !out || table_addr == 0 || table_entries == 0 || idx >= table_entries) {
        return false;
    }
    uint64_t off = (uint64_t)idx * sizeof(virtq_desc_t);
    uint8_t* p = dma_ptr(v, table_addr + off, sizeof(virtq_desc_t));
    if (!p) {
        return false;
    }
    out->addr  = read_uint64_le(p + 0);
    out->len   = read_uint32_le(p + 8);
    out->flags = read_uint16_le(p + 12);
    out->next  = read_uint16_le(p + 14);
    return true;
}

static bool virtq_find_first_write_desc(virtio_input_dev_t* v, virtio_queue_t* q, uint64_t table_addr, uint32_t table_entries,
                                       uint16_t head, uint64_t* out_addr, uint32_t* out_len)
{
    if (!v || !q || !out_addr || !out_len || table_addr == 0 || table_entries == 0 || head >= table_entries) {
        return false;
    }

    uint16_t cur = head;
    for (uint32_t steps = 0; steps < table_entries; ++steps) {
        virtq_desc_t d;
        if (!virtq_load_desc_at(v, table_addr, table_entries, cur, &d)) {
            return false;
        }
        if ((d.flags & VIRTQ_DESC_F_WRITE) && d.len >= sizeof(virtio_input_event_t)) {
            *out_addr = d.addr;
            *out_len = d.len;
            return true;
        }
        if (!(d.flags & VIRTQ_DESC_F_NEXT)) {
            return false;
        }
        cur = d.next;
        if (cur >= table_entries) {
            return false;
        }
    }
    return false;
}

static bool virtq_get_event_write_buf(virtio_input_dev_t* v, virtio_queue_t* q, uint16_t head, uint64_t* out_addr, uint32_t* out_len)
{
    if (!v || !q || !out_addr || !out_len || q->desc_addr == 0 || q->num == 0) {
        return false;
    }

    virtq_desc_t d0;
    if (!virtq_load_desc_at(v, q->desc_addr, q->num, head, &d0)) {
        return false;
    }

    if (d0.flags & VIRTQ_DESC_F_INDIRECT) {
        uint32_t entries = d0.len / (uint32_t)sizeof(virtq_desc_t);
        if (entries == 0) {
            return false;
        }
        return virtq_find_first_write_desc(v, q, d0.addr, entries, 0, out_addr, out_len);
    }

    return virtq_find_first_write_desc(v, q, q->desc_addr, q->num, head, out_addr, out_len);
}

static uint16_t virtq_avail_idx(virtio_input_dev_t* v, virtio_queue_t* q)
{
    uint8_t* avail_p = dma_ptr(v, q->avail_addr, 4);
    if (!avail_p) {
        return q->last_avail_idx;
    }
    return read_uint16_le(avail_p + 2);
}

static bool virtq_pop_avail_head(virtio_input_dev_t* v, virtio_queue_t* q, uint16_t* out_head)
{
    if (!v || !q || !out_head || q->avail_addr == 0 || q->num == 0) {
        return false;
    }
    uint8_t* avail_p = dma_ptr(v, q->avail_addr, 4 + (size_t)q->num * 2);
    if (!avail_p) {
        return false;
    }
    uint16_t idx = read_uint16_le(avail_p + 2);
    if (q->last_avail_idx == idx) {
        return false;
    }
    uint32_t slot = (uint32_t)(q->last_avail_idx % q->num);
    size_t off = 4 + (size_t)slot * 2;
    *out_head = read_uint16_le(avail_p + off);
    q->last_avail_idx++;
    return true;
}

static void virtq_push_used(virtio_input_dev_t* v, virtio_queue_t* q, uint16_t head, uint32_t len)
{
    if (!v || !q || q->used_addr == 0 || q->num == 0) {
        return;
    }
    uint8_t* used_p = dma_ptr(v, q->used_addr, 4 + (size_t)q->num * sizeof(virtq_used_elem_t));
    if (!used_p) {
        return;
    }
    uint16_t used_idx = q->used_idx;
    uint32_t slot = (uint32_t)(used_idx % q->num);
    size_t elem_off = 4 + (size_t)slot * sizeof(virtq_used_elem_t);
    write_uint32_le(used_p + elem_off + 0, head);
    write_uint32_le(used_p + elem_off + 4, len);
    used_idx++;
    q->used_idx = used_idx;
    write_uint16_le(used_p + 2, used_idx);

    v->interrupt_status |= 1;
    rvvm_raise_irq(v->intc, v->irq);
}

static void virtio_input_reset(virtio_input_dev_t* v)
{
    if (!v) {
        return;
    }
    v->status = 0;
    v->interrupt_status = 0;
    v->queue_sel = 0;
    v->host_features_sel = 0;
    v->driver_features_sel = 0;
    v->driver_features[0] = 0;
    v->driver_features[1] = 0;
    for (size_t i = 0; i < STATIC_ARRAY_SIZE(v->q); ++i) {
        v->q[i].num = 0;
        v->q[i].ready = 0;
        v->q[i].last_avail_idx = 0;
        v->q[i].used_idx = 0;
        v->q[i].desc_addr = 0;
        v->q[i].avail_addr = 0;
        v->q[i].used_addr = 0;
    }
    v->pending_head = 0;
    v->pending_tail = 0;
    v->config_generation++;
    rvvm_lower_irq(v->intc, v->irq);
}

static void bitmap_set(uint8_t* b, uint16_t bit)
{
    b[bit >> 3] |= (uint8_t)(1u << (bit & 7));
}

static uint8_t bitmap_min_size(const uint8_t* b, size_t cap)
{
    for (size_t i = cap; i > 0; --i) {
        if (b[i - 1] != 0) {
            return (uint8_t)i;
        }
    }
    return 0;
}

static uint16_t hid_to_linux_key(hid_key_t key)
{
    switch (key) {
        case HID_KEY_A: return 30;
        case HID_KEY_B: return 48;
        case HID_KEY_C: return 46;
        case HID_KEY_D: return 32;
        case HID_KEY_E: return 18;
        case HID_KEY_F: return 33;
        case HID_KEY_G: return 34;
        case HID_KEY_H: return 35;
        case HID_KEY_I: return 23;
        case HID_KEY_J: return 36;
        case HID_KEY_K: return 37;
        case HID_KEY_L: return 38;
        case HID_KEY_M: return 50;
        case HID_KEY_N: return 49;
        case HID_KEY_O: return 24;
        case HID_KEY_P: return 25;
        case HID_KEY_Q: return 16;
        case HID_KEY_R: return 19;
        case HID_KEY_S: return 31;
        case HID_KEY_T: return 20;
        case HID_KEY_U: return 22;
        case HID_KEY_V: return 47;
        case HID_KEY_W: return 17;
        case HID_KEY_X: return 45;
        case HID_KEY_Y: return 21;
        case HID_KEY_Z: return 44;

        case HID_KEY_1: return 2;
        case HID_KEY_2: return 3;
        case HID_KEY_3: return 4;
        case HID_KEY_4: return 5;
        case HID_KEY_5: return 6;
        case HID_KEY_6: return 7;
        case HID_KEY_7: return 8;
        case HID_KEY_8: return 9;
        case HID_KEY_9: return 10;
        case HID_KEY_0: return 11;

        case HID_KEY_ENTER: return 28;
        case HID_KEY_ESC: return 1;
        case HID_KEY_BACKSPACE: return 14;
        case HID_KEY_TAB: return 15;
        case HID_KEY_SPACE: return 57;

        case HID_KEY_MINUS: return 12;
        case HID_KEY_EQUAL: return 13;
        case HID_KEY_LEFTBRACE: return 26;
        case HID_KEY_RIGHTBRACE: return 27;
        case HID_KEY_BACKSLASH: return 43;
        case HID_KEY_SEMICOLON: return 39;
        case HID_KEY_APOSTROPHE: return 40;
        case HID_KEY_GRAVE: return 41;
        case HID_KEY_COMMA: return 51;
        case HID_KEY_DOT: return 52;
        case HID_KEY_SLASH: return 53;

        case HID_KEY_CAPSLOCK: return 58;

        case HID_KEY_F1: return 59;
        case HID_KEY_F2: return 60;
        case HID_KEY_F3: return 61;
        case HID_KEY_F4: return 62;
        case HID_KEY_F5: return 63;
        case HID_KEY_F6: return 64;
        case HID_KEY_F7: return 65;
        case HID_KEY_F8: return 66;
        case HID_KEY_F9: return 67;
        case HID_KEY_F10: return 68;
        case HID_KEY_F11: return 87;
        case HID_KEY_F12: return 88;

        case HID_KEY_SYSRQ: return 99;
        case HID_KEY_SCROLLLOCK: return 70;
        case HID_KEY_PAUSE: return 119;
        case HID_KEY_INSERT: return 110;
        case HID_KEY_HOME: return 102;
        case HID_KEY_PAGEUP: return 104;
        case HID_KEY_DELETE: return 111;
        case HID_KEY_END: return 107;
        case HID_KEY_PAGEDOWN: return 109;
        case HID_KEY_RIGHT: return 106;
        case HID_KEY_LEFT: return 105;
        case HID_KEY_DOWN: return 108;
        case HID_KEY_UP: return 103;

        case HID_KEY_NUMLOCK: return 69;
        case HID_KEY_KPSLASH: return 98;
        case HID_KEY_KPASTERISK: return 55;
        case HID_KEY_KPMINUS: return 74;
        case HID_KEY_KPPLUS: return 78;
        case HID_KEY_KPENTER: return 96;
        case HID_KEY_KP1: return 79;
        case HID_KEY_KP2: return 80;
        case HID_KEY_KP3: return 81;
        case HID_KEY_KP4: return 75;
        case HID_KEY_KP5: return 76;
        case HID_KEY_KP6: return 77;
        case HID_KEY_KP7: return 71;
        case HID_KEY_KP8: return 72;
        case HID_KEY_KP9: return 73;
        case HID_KEY_KP0: return 82;
        case HID_KEY_KPDOT: return 83;

        case HID_KEY_102ND: return 86;
        case HID_KEY_MENU: return 139;
        case HID_KEY_POWER: return 116;

        case HID_KEY_LEFTCTRL: return 29;
        case HID_KEY_LEFTSHIFT: return 42;
        case HID_KEY_LEFTALT: return 56;
        case HID_KEY_RIGHTCTRL: return 97;
        case HID_KEY_RIGHTSHIFT: return 54;
        case HID_KEY_RIGHTALT: return 100;

        default: return 0;
    }
}

static void virtio_input_update_cfg(virtio_input_dev_t* v)
{
    if (!v) {
        return;
    }
    uint8_t sel = v->cfg.select;
    uint8_t sub = v->cfg.subsel;
    v->cfg.size = 0;
    memset(v->cfg.u.bitmap, 0, sizeof(v->cfg.u.bitmap));

    if (sel == VIRTIO_INPUT_CFG_ID_NAME) {
        const char* s = v->is_keyboard ? "virtio-input-keyboard" : "virtio-input-mouse";
        size_t n = strlen(s);
        if (n >= sizeof(v->cfg.u.string)) {
            n = sizeof(v->cfg.u.string) - 1;
        }
        memcpy(v->cfg.u.string, s, n);
        v->cfg.u.string[n] = 0;
        v->cfg.size = (uint8_t)(n + 1);
        return;
    }

    if (sel == VIRTIO_INPUT_CFG_ID_SERIAL) {
        const char* s = "rvvm";
        size_t n = strlen(s);
        memcpy(v->cfg.u.string, s, n);
        v->cfg.u.string[n] = 0;
        v->cfg.size = (uint8_t)(n + 1);
        return;
    }

    if (sel == VIRTIO_INPUT_CFG_ID_DEVIDS) {
        virtio_input_devids_t ids;
        write_uint16_le_m(&ids.bus, 0x06);
        write_uint16_le_m(&ids.vendor, 1);
        write_uint16_le_m(&ids.product, v->is_keyboard ? 1 : 2);
        write_uint16_le_m(&ids.version, 1);
        v->cfg.u.ids = ids;
        v->cfg.size = (uint8_t)sizeof(ids);
        return;
    }

    if (sel == VIRTIO_INPUT_CFG_PROP_BITS) {
        v->cfg.size = 0;
        return;
    }

    if (sel == VIRTIO_INPUT_CFG_EV_BITS) {
        if (sub == 0) {
            if (v->is_keyboard) {
                bitmap_set(v->cfg.u.bitmap, EV_KEY);
            } else {
                bitmap_set(v->cfg.u.bitmap, EV_KEY);
                bitmap_set(v->cfg.u.bitmap, EV_REL);
            }
            v->cfg.size = bitmap_min_size(v->cfg.u.bitmap, sizeof(v->cfg.u.bitmap));
            return;
        }

        if (v->is_keyboard) {
            if (sub != EV_KEY) {
                v->cfg.size = 0;
                return;
            }
            for (uint16_t k = 0; k < 256; ++k) {
                uint16_t code = hid_to_linux_key((hid_key_t)k);
                if (code != 0 && code < 1024) {
                    bitmap_set(v->cfg.u.bitmap, code);
                }
            }
            v->cfg.size = bitmap_min_size(v->cfg.u.bitmap, sizeof(v->cfg.u.bitmap));
            return;
        }

        if (!v->is_keyboard) {
            if (sub == EV_KEY) {
                bitmap_set(v->cfg.u.bitmap, BTN_LEFT);
                bitmap_set(v->cfg.u.bitmap, BTN_RIGHT);
                bitmap_set(v->cfg.u.bitmap, BTN_MIDDLE);
                v->cfg.size = bitmap_min_size(v->cfg.u.bitmap, sizeof(v->cfg.u.bitmap));
                return;
            }
            if (sub == EV_REL) {
                bitmap_set(v->cfg.u.bitmap, REL_X);
                bitmap_set(v->cfg.u.bitmap, REL_Y);
                bitmap_set(v->cfg.u.bitmap, REL_WHEEL);
                v->cfg.size = bitmap_min_size(v->cfg.u.bitmap, sizeof(v->cfg.u.bitmap));
                return;
            }
        }
        v->cfg.size = 0;
        return;
    }

    if (sel == VIRTIO_INPUT_CFG_ABS_INFO) {
        if (v->is_keyboard) {
            v->cfg.size = 0;
            return;
        }
        v->cfg.size = 0;
        return;
    }

    v->cfg.size = 0;
}

static bool pending_push(virtio_input_dev_t* v, virtio_input_event_t e)
{
    uint16_t next = (uint16_t)(v->pending_tail + 1);
    if ((uint16_t)(next - v->pending_head) == (uint16_t)STATIC_ARRAY_SIZE(v->pending)) {
        return false;
    }
    v->pending[(uint16_t)(v->pending_tail % STATIC_ARRAY_SIZE(v->pending))] = e;
    v->pending_tail = next;
    return true;
}

static bool pending_pop(virtio_input_dev_t* v, virtio_input_event_t* out)
{
    if (v->pending_head == v->pending_tail) {
        return false;
    }
    *out = v->pending[(uint16_t)(v->pending_head % STATIC_ARRAY_SIZE(v->pending))];
    v->pending_head++;
    return true;
}

static void virtio_input_try_flush_events(virtio_input_dev_t* v)
{
    virtio_queue_t* q = &v->q[0];
    if (!q->ready || q->num == 0 || q->desc_addr == 0 || q->avail_addr == 0 || q->used_addr == 0) {
        return;
    }

    while (v->pending_head != v->pending_tail) {
        uint16_t head = 0;
        if (!virtq_pop_avail_head(v, q, &head)) {
            return;
        }
        uint64_t buf_addr = 0;
        uint32_t buf_len = 0;
        if (!virtq_get_event_write_buf(v, q, head, &buf_addr, &buf_len)) {
            virtq_push_used(v, q, head, 0);
            continue;
        }

        void* p = dma_ptr(v, buf_addr, sizeof(virtio_input_event_t));
        if (!p) {
            virtq_push_used(v, q, head, 0);
            continue;
        }
#
        virtio_input_event_t e;
        if (!pending_pop(v, &e)) {
            virtq_push_used(v, q, head, 0);
            continue;
        }
        uint8_t* b = (uint8_t*)p;
        write_uint16_le(b + 0, e.type);
        write_uint16_le(b + 2, e.code);
        write_uint32_le(b + 4, (uint32_t)e.value);
        virtq_push_used(v, q, head, (uint32_t)sizeof(virtio_input_event_t));
    }
}

static void virtio_input_process_status_queue(virtio_input_dev_t* v)
{
    virtio_queue_t* q = &v->q[1];
    if (!q->ready || q->num == 0 || q->desc_addr == 0 || q->avail_addr == 0 || q->used_addr == 0) {
        return;
    }
    uint16_t avail = virtq_avail_idx(v, q);
    while (q->last_avail_idx != avail) {
        uint16_t head = 0;
        if (!virtq_pop_avail_head(v, q, &head)) {
            break;
        }
        virtq_push_used(v, q, head, 0);
        avail = virtq_avail_idx(v, q);
    }
}

static bool virtio_input_mmio_read(rvvm_mmio_dev_t* dev, void* data, size_t offset, uint8_t size)
{
    virtio_input_dev_t* v = dev->data;
    uint32_t val = 0;

    if (!v) {
        mmio_store_u32(data, 0, size);
        return true;
    }

    if (offset >= VIRTIO_MMIO_CONFIG) {
        uint32_t off = (uint32_t)(offset - VIRTIO_MMIO_CONFIG);
        uint8_t* cfg = (uint8_t*)&v->cfg;
        uint32_t cap = (uint32_t)sizeof(v->cfg);
        if (off + size <= cap) {
            if (size == 1) {
                val = cfg[off];
            } else if (size == 2) {
                val = read_uint16_le(cfg + off);
            } else {
                val = read_uint32_le(cfg + off);
            }
        }
        mmio_store_u32(data, val, size);
        return true;
    }

    if (size != 4) {
        mmio_store_u32(data, 0, size);
        return true;
    }

    switch (offset) {
        case VIRTIO_MMIO_MAGIC:
            val = VIRTIO_MMIO_MAGIC_VALUE;
            break;
        case VIRTIO_MMIO_VERSION:
            val = VIRTIO_MMIO_VERSION_VALUE;
            break;
        case VIRTIO_MMIO_DEVICE_ID:
            val = VIRTIO_ID_INPUT;
            break;
        case VIRTIO_MMIO_VENDOR_ID:
            val = VIRTIO_MMIO_VENDOR_VALUE;
            break;
        case VIRTIO_MMIO_DEVICE_FEATURES:
            val = v->host_features[v->host_features_sel & 1];
            break;
        case VIRTIO_MMIO_DEVICE_FEATURES_SEL:
            val = v->host_features_sel;
            break;
        case VIRTIO_MMIO_DRIVER_FEATURES:
            val = v->driver_features[v->driver_features_sel & 1];
            break;
        case VIRTIO_MMIO_DRIVER_FEATURES_SEL:
            val = v->driver_features_sel;
            break;
        case VIRTIO_MMIO_QUEUE_SEL:
            val = v->queue_sel;
            break;
        case VIRTIO_MMIO_QUEUE_NUM_MAX:
            val = (v->queue_sel < STATIC_ARRAY_SIZE(v->q)) ? 128 : 0;
            break;
        case VIRTIO_MMIO_QUEUE_NUM:
            val = (v->queue_sel < STATIC_ARRAY_SIZE(v->q)) ? v->q[v->queue_sel].num : 0;
            break;
        case VIRTIO_MMIO_QUEUE_READY:
            val = (v->queue_sel < STATIC_ARRAY_SIZE(v->q)) ? v->q[v->queue_sel].ready : 0;
            break;
        case VIRTIO_MMIO_INTERRUPT_STATUS:
            val = v->interrupt_status;
            break;
        case VIRTIO_MMIO_STATUS:
            val = v->status;
            break;
        case VIRTIO_MMIO_QUEUE_DESC_LOW:
            if (v->queue_sel < STATIC_ARRAY_SIZE(v->q)) {
                val = (uint32_t)(v->q[v->queue_sel].desc_addr);
            }
            break;
        case VIRTIO_MMIO_QUEUE_DESC_HIGH:
            if (v->queue_sel < STATIC_ARRAY_SIZE(v->q)) {
                val = (uint32_t)(v->q[v->queue_sel].desc_addr >> 32);
            }
            break;
        case VIRTIO_MMIO_QUEUE_DRIVER_LOW:
            if (v->queue_sel < STATIC_ARRAY_SIZE(v->q)) {
                val = (uint32_t)(v->q[v->queue_sel].avail_addr);
            }
            break;
        case VIRTIO_MMIO_QUEUE_DRIVER_HIGH:
            if (v->queue_sel < STATIC_ARRAY_SIZE(v->q)) {
                val = (uint32_t)(v->q[v->queue_sel].avail_addr >> 32);
            }
            break;
        case VIRTIO_MMIO_QUEUE_DEVICE_LOW:
            if (v->queue_sel < STATIC_ARRAY_SIZE(v->q)) {
                val = (uint32_t)(v->q[v->queue_sel].used_addr);
            }
            break;
        case VIRTIO_MMIO_QUEUE_DEVICE_HIGH:
            if (v->queue_sel < STATIC_ARRAY_SIZE(v->q)) {
                val = (uint32_t)(v->q[v->queue_sel].used_addr >> 32);
            }
            break;
        case VIRTIO_MMIO_CONFIG_GENERATION:
            val = v->config_generation;
            break;
    }

    write_uint32_le(data, val);
    return true;
}

static bool virtio_input_mmio_write(rvvm_mmio_dev_t* dev, void* data, size_t offset, uint8_t size)
{
    virtio_input_dev_t* v = dev->data;
    uint32_t val = mmio_load_u32(data, size);

    if (!v) {
        return true;
    }

    if (offset >= VIRTIO_MMIO_CONFIG) {
        uint32_t off = (uint32_t)(offset - VIRTIO_MMIO_CONFIG);
        if (off == 0 && size == 1) {
            spin_lock(&v->lock);
            v->cfg.select = (uint8_t)val;
            virtio_input_update_cfg(v);
            v->config_generation++;
            spin_unlock(&v->lock);
        } else if (off == 1 && size == 1) {
            spin_lock(&v->lock);
            v->cfg.subsel = (uint8_t)val;
            virtio_input_update_cfg(v);
            v->config_generation++;
            spin_unlock(&v->lock);
        }
        return true;
    }

    if (size != 4) {
        return true;
    }

    spin_lock(&v->lock);
    switch (offset) {
        case VIRTIO_MMIO_DEVICE_FEATURES_SEL:
            v->host_features_sel = val;
            break;
        case VIRTIO_MMIO_DRIVER_FEATURES:
            v->driver_features[v->driver_features_sel & 1] = val;
            break;
        case VIRTIO_MMIO_DRIVER_FEATURES_SEL:
            v->driver_features_sel = val;
            break;
        case VIRTIO_MMIO_QUEUE_SEL:
            v->queue_sel = val;
            break;
        case VIRTIO_MMIO_QUEUE_NUM:
            if (v->queue_sel < STATIC_ARRAY_SIZE(v->q)) {
                if (val > 128) {
                    val = 128;
                }
                v->q[v->queue_sel].num = (uint16_t)val;
            }
            break;
        case VIRTIO_MMIO_QUEUE_READY:
            if (v->queue_sel < STATIC_ARRAY_SIZE(v->q)) {
                v->q[v->queue_sel].ready = (val & 1) ? 1 : 0;
                v->q[v->queue_sel].last_avail_idx = 0;
                v->q[v->queue_sel].used_idx = 0;
            }
            break;
        case VIRTIO_MMIO_QUEUE_NOTIFY:
            if (val == 0) {
                virtio_input_try_flush_events(v);
            } else if (val == 1) {
                virtio_input_process_status_queue(v);
            }
            break;
        case VIRTIO_MMIO_INTERRUPT_ACK:
            v->interrupt_status &= ~val;
            if (!v->interrupt_status) {
                rvvm_lower_irq(v->intc, v->irq);
            }
            break;
        case VIRTIO_MMIO_STATUS: {
            uint32_t s = val & 0xFF;
            if (s == 0) {
                virtio_input_reset(v);
                break;
            }
            v->status = s;
            if (v->status & 0x08) {
                uint32_t bad0 = v->driver_features[0] & ~v->host_features[0];
                uint32_t bad1 = v->driver_features[1] & ~v->host_features[1];
                if (bad0 || bad1) {
                    v->status &= ~0x08;
                }
            }
            break;
        }
        case VIRTIO_MMIO_QUEUE_DESC_LOW:
            if (v->queue_sel < STATIC_ARRAY_SIZE(v->q)) {
                uint64_t cur = v->q[v->queue_sel].desc_addr;
                cur = (cur & 0xFFFFFFFF00000000ULL) | (uint64_t)val;
                v->q[v->queue_sel].desc_addr = cur;
            }
            break;
        case VIRTIO_MMIO_QUEUE_DESC_HIGH:
            if (v->queue_sel < STATIC_ARRAY_SIZE(v->q)) {
                uint64_t cur = v->q[v->queue_sel].desc_addr;
                cur = (cur & 0x00000000FFFFFFFFULL) | ((uint64_t)val << 32);
                v->q[v->queue_sel].desc_addr = cur;
            }
            break;
        case VIRTIO_MMIO_QUEUE_DRIVER_LOW:
            if (v->queue_sel < STATIC_ARRAY_SIZE(v->q)) {
                uint64_t cur = v->q[v->queue_sel].avail_addr;
                cur = (cur & 0xFFFFFFFF00000000ULL) | (uint64_t)val;
                v->q[v->queue_sel].avail_addr = cur;
            }
            break;
        case VIRTIO_MMIO_QUEUE_DRIVER_HIGH:
            if (v->queue_sel < STATIC_ARRAY_SIZE(v->q)) {
                uint64_t cur = v->q[v->queue_sel].avail_addr;
                cur = (cur & 0x00000000FFFFFFFFULL) | ((uint64_t)val << 32);
                v->q[v->queue_sel].avail_addr = cur;
            }
            break;
        case VIRTIO_MMIO_QUEUE_DEVICE_LOW:
            if (v->queue_sel < STATIC_ARRAY_SIZE(v->q)) {
                uint64_t cur = v->q[v->queue_sel].used_addr;
                cur = (cur & 0xFFFFFFFF00000000ULL) | (uint64_t)val;
                v->q[v->queue_sel].used_addr = cur;
            }
            break;
        case VIRTIO_MMIO_QUEUE_DEVICE_HIGH:
            if (v->queue_sel < STATIC_ARRAY_SIZE(v->q)) {
                uint64_t cur = v->q[v->queue_sel].used_addr;
                cur = (cur & 0x00000000FFFFFFFFULL) | ((uint64_t)val << 32);
                v->q[v->queue_sel].used_addr = cur;
            }
            break;
    }
    spin_unlock(&v->lock);
    return true;
}

static void virtio_input_remove(rvvm_mmio_dev_t* dev)
{
    virtio_input_dev_t* v = dev->data;
    if (!v) {
        return;
    }
    free(v);
}

static void virtio_input_dev_reset(rvvm_mmio_dev_t* dev)
{
    virtio_input_dev_t* v = dev ? dev->data : NULL;
    if (!v) {
        return;
    }
    spin_lock(&v->lock);
    virtio_input_reset(v);
    spin_unlock(&v->lock);
}

static void virtio_input_suspend(rvvm_mmio_dev_t* dev, rvvm_state_t* state)
{
    virtio_input_dev_t* v = dev ? dev->data : NULL;
    if (!v || !state) {
        return;
    }

    rvvm_state_write_u32(state, 1);

    spin_lock(&v->lock);
    rvvm_state_write_u32(state, v->host_features_sel);
    rvvm_state_write_u32(state, v->driver_features_sel);
    rvvm_state_write_u32(state, v->driver_features[0]);
    rvvm_state_write_u32(state, v->driver_features[1]);
    rvvm_state_write_u32(state, v->status);
    rvvm_state_write_u32(state, v->interrupt_status);
    rvvm_state_write_u32(state, v->queue_sel);
    rvvm_state_write_u32(state, v->config_generation);
    rvvm_state_write_u32(state, v->cfg.select);
    rvvm_state_write_u32(state, v->cfg.subsel);
    rvvm_state_write_u32(state, v->width);
    rvvm_state_write_u32(state, v->height);
    rvvm_state_write_u32(state, v->is_keyboard);

    for (size_t i = 0; i < STATIC_ARRAY_SIZE(v->q); ++i) {
        virtio_queue_t* q = &v->q[i];
        rvvm_state_write_u32(state, q->num);
        rvvm_state_write_u32(state, q->ready);
        rvvm_state_write_u32(state, q->last_avail_idx);
        rvvm_state_write_u32(state, q->used_idx);
        rvvm_state_write_u64(state, q->desc_addr);
        rvvm_state_write_u64(state, q->avail_addr);
        rvvm_state_write_u64(state, q->used_addr);
    }

    uint16_t pending_cnt = (uint16_t)(v->pending_tail - v->pending_head);
    rvvm_state_write_u32(state, pending_cnt);
    for (uint16_t i = 0; i < pending_cnt; ++i) {
        virtio_input_event_t e = v->pending[(uint16_t)((v->pending_head + i) % STATIC_ARRAY_SIZE(v->pending))];
        rvvm_state_write_u32(state, e.type);
        rvvm_state_write_u32(state, e.code);
        rvvm_state_write_u32(state, (uint32_t)e.value);
    }
    spin_unlock(&v->lock);
}

static void virtio_input_resume(rvvm_mmio_dev_t* dev, rvvm_state_t* state)
{
    virtio_input_dev_t* v = dev ? dev->data : NULL;
    if (!v || !state) {
        return;
    }

    uint32_t version = 0;
    if (!rvvm_state_read_u32(state, &version) || version != 1) {
        rvvm_state_fail(state);
        return;
    }

    spin_lock(&v->lock);

    uint32_t w = 0;
    if (!rvvm_state_read_u32(state, &v->host_features_sel) || !rvvm_state_read_u32(state, &v->driver_features_sel) ||
        !rvvm_state_read_u32(state, &v->driver_features[0]) || !rvvm_state_read_u32(state, &v->driver_features[1]) ||
        !rvvm_state_read_u32(state, &v->status) || !rvvm_state_read_u32(state, &v->interrupt_status) ||
        !rvvm_state_read_u32(state, &v->queue_sel) || !rvvm_state_read_u32(state, &v->config_generation) ||
        !rvvm_state_read_u32(state, &w)) {
        spin_unlock(&v->lock);
        rvvm_state_fail(state);
        return;
    }
    v->cfg.select = (uint8_t)w;
    if (!rvvm_state_read_u32(state, &w)) {
        spin_unlock(&v->lock);
        rvvm_state_fail(state);
        return;
    }
    v->cfg.subsel = (uint8_t)w;
    if (!rvvm_state_read_u32(state, &w)) {
        spin_unlock(&v->lock);
        rvvm_state_fail(state);
        return;
    }
    v->width = (uint16_t)w;
    if (!rvvm_state_read_u32(state, &w)) {
        spin_unlock(&v->lock);
        rvvm_state_fail(state);
        return;
    }
    v->height = (uint16_t)w;
    if (!rvvm_state_read_u32(state, &w)) {
        spin_unlock(&v->lock);
        rvvm_state_fail(state);
        return;
    }
    v->is_keyboard = (uint8_t)w;

    for (size_t i = 0; i < STATIC_ARRAY_SIZE(v->q); ++i) {
        virtio_queue_t* q = &v->q[i];
        uint64_t a = 0, b = 0, c = 0;
        if (!rvvm_state_read_u32(state, &w)) {
            spin_unlock(&v->lock);
            rvvm_state_fail(state);
            return;
        }
        q->num = (uint16_t)w;
        if (!rvvm_state_read_u32(state, &w)) {
            spin_unlock(&v->lock);
            rvvm_state_fail(state);
            return;
        }
        q->ready = (uint8_t)w;
        if (!rvvm_state_read_u32(state, &w)) {
            spin_unlock(&v->lock);
            rvvm_state_fail(state);
            return;
        }
        q->last_avail_idx = (uint16_t)w;
        if (!rvvm_state_read_u32(state, &w)) {
            spin_unlock(&v->lock);
            rvvm_state_fail(state);
            return;
        }
        q->used_idx = (uint16_t)w;
        if (!rvvm_state_read_u64(state, &a) || !rvvm_state_read_u64(state, &b) || !rvvm_state_read_u64(state, &c)) {
            spin_unlock(&v->lock);
            rvvm_state_fail(state);
            return;
        }
        q->desc_addr = a;
        q->avail_addr = b;
        q->used_addr = c;
    }

    uint32_t pending_cnt_u32 = 0;
    if (!rvvm_state_read_u32(state, &pending_cnt_u32)) {
        spin_unlock(&v->lock);
        rvvm_state_fail(state);
        return;
    }
    uint16_t max_pending = (uint16_t)(STATIC_ARRAY_SIZE(v->pending) - 1);
    uint16_t pending_cnt = (pending_cnt_u32 > max_pending) ? max_pending : (uint16_t)pending_cnt_u32;
    v->pending_head = 0;
    v->pending_tail = 0;
    for (uint16_t i = 0; i < pending_cnt; ++i) {
        virtio_input_event_t e = { 0 };
        uint32_t t = 0, c = 0, vv = 0;
        if (!rvvm_state_read_u32(state, &t) || !rvvm_state_read_u32(state, &c) || !rvvm_state_read_u32(state, &vv)) {
            spin_unlock(&v->lock);
            rvvm_state_fail(state);
            return;
        }
        e.type = (uint16_t)t;
        e.code = (uint16_t)c;
        e.value = (int32_t)vv;
        pending_push(v, e);
    }

    virtio_input_update_cfg(v);

    if (v->interrupt_status) {
        rvvm_raise_irq(v->intc, v->irq);
    } else {
        rvvm_lower_irq(v->intc, v->irq);
    }

    spin_unlock(&v->lock);
}

static rvvm_mmio_type_t virtio_input_dev_type = {
    .name = "virtio_input_mmio",
    .remove = virtio_input_remove,
    .reset = virtio_input_dev_reset,
    .suspend = virtio_input_suspend,
    .resume = virtio_input_resume,
};

static rvvm_mmio_dev_t* virtio_input_init(rvvm_machine_t* machine, rvvm_addr_t addr, rvvm_intc_t* intc, rvvm_irq_t irq, bool keyboard)
{
    if (!machine || !intc) {
        return NULL;
    }

    virtio_input_dev_t* v = safe_new_obj(virtio_input_dev_t);
    v->machine = machine;
    v->intc = intc;
    v->irq = irq;
    v->is_keyboard = keyboard ? 1 : 0;
    v->width = 1024;
    v->height = 768;
    v->host_features[0] = 0;
    v->host_features[1] = 0;
    v->host_features[VIRTIO_RING_F_INDIRECT_DESC / 32] |= 1U << (VIRTIO_RING_F_INDIRECT_DESC % 32);
    v->host_features[VIRTIO_F_VERSION_1 / 32] |= 1U << (VIRTIO_F_VERSION_1 % 32);
    v->host_features[VIRTIO_F_ACCESS_PLATFORM / 32] |= 1U << (VIRTIO_F_ACCESS_PLATFORM % 32);
    v->cfg.select = VIRTIO_INPUT_CFG_ID_NAME;
    v->cfg.subsel = 0;
    virtio_input_update_cfg(v);
    virtio_input_reset(v);

    rvvm_mmio_dev_t mmio_desc = {
        .addr = addr,
        .size = 0x1000,
        .data = v,
        .type = &virtio_input_dev_type,
        .read = virtio_input_mmio_read,
        .write = virtio_input_mmio_write,
        .min_op_size = 1,
        .max_op_size = 4,
    };
#
    rvvm_mmio_dev_t* mmio = rvvm_attach_mmio(machine, &mmio_desc);
    if (!mmio) {
        virtio_input_remove(&mmio_desc);
        return NULL;
    }

#ifdef USE_FDT
    struct fdt_node* node = fdt_node_create_reg("virtio_mmio", mmio_desc.addr);
    fdt_node_add_prop_reg(node, "reg", mmio_desc.addr, mmio_desc.size);
    fdt_node_add_prop_str(node, "compatible", "virtio,mmio");
    rvvm_fdt_describe_irq(node, intc, irq);
    fdt_node_add_child(rvvm_get_fdt_soc(machine), node);
#endif

    return mmio;
}

static void virtio_input_send_event(virtio_input_dev_t* v, uint16_t type, uint16_t code, int32_t value)
{
    if (!v) {
        return;
    }
    spin_lock(&v->lock);
    virtio_input_event_t e;
    e.type = type;
    e.code = code;
    e.value = value;
    if (pending_push(v, e)) {
        virtio_input_try_flush_events(v);
    }
    spin_unlock(&v->lock);
}

static virtio_input_dev_t* virtio_input_from_mmio(rvvm_mmio_dev_t* mmio)
{
    if (!mmio) {
        return NULL;
    }
    return (virtio_input_dev_t*)mmio->data;
}

PUBLIC hid_keyboard_t* hid_keyboard_init_auto_virtio(rvvm_machine_t* machine)
{
    rvvm_intc_t* intc = rvvm_get_intc(machine);
    rvvm_addr_t addr = rvvm_mmio_zone_auto(machine, 0x10002000, 0x1000);
    rvvm_mmio_dev_t* mmio = virtio_input_init(machine, addr, intc, rvvm_alloc_irq(intc), true);
    if (!mmio) {
        return NULL;
    }
    hid_keyboard_t* kb = safe_new_obj(hid_keyboard_t);
    kb->dev = virtio_input_from_mmio(mmio);
    return kb;
}

PUBLIC hid_mouse_t* hid_mouse_init_auto_virtio(rvvm_machine_t* machine)
{
    rvvm_intc_t* intc = rvvm_get_intc(machine);
    rvvm_addr_t addr = rvvm_mmio_zone_auto(machine, 0x10003000, 0x1000);
    rvvm_mmio_dev_t* mmio = virtio_input_init(machine, addr, intc, rvvm_alloc_irq(intc), false);
    if (!mmio) {
        return NULL;
    }
    hid_mouse_t* m = safe_new_obj(hid_mouse_t);
    m->dev = virtio_input_from_mmio(mmio);
    m->abs_x = 0;
    m->abs_y = 0;
    return m;
}

PUBLIC void hid_keyboard_press_virtio(hid_keyboard_t* kb, hid_key_t key)
{
    if (!kb || !kb->dev) {
        return;
    }
    uint16_t code = hid_to_linux_key(key);
    if (code == 0) {
        return;
    }
    virtio_input_send_event(kb->dev, EV_KEY, code, 1);
    virtio_input_send_event(kb->dev, EV_SYN, SYN_REPORT, 0);
}

PUBLIC void hid_keyboard_release_virtio(hid_keyboard_t* kb, hid_key_t key)
{
    if (!kb || !kb->dev) {
        return;
    }
    uint16_t code = hid_to_linux_key(key);
    if (code == 0) {
        return;
    }
    virtio_input_send_event(kb->dev, EV_KEY, code, 0);
    virtio_input_send_event(kb->dev, EV_SYN, SYN_REPORT, 0);
}

PUBLIC void hid_mouse_press_virtio(hid_mouse_t* mouse, hid_btns_t btns)
{
    if (!mouse || !mouse->dev) {
        return;
    }
    if (btns & HID_BTN_LEFT) virtio_input_send_event(mouse->dev, EV_KEY, BTN_LEFT, 1);
    if (btns & HID_BTN_RIGHT) virtio_input_send_event(mouse->dev, EV_KEY, BTN_RIGHT, 1);
    if (btns & HID_BTN_MIDDLE) virtio_input_send_event(mouse->dev, EV_KEY, BTN_MIDDLE, 1);
    virtio_input_send_event(mouse->dev, EV_SYN, SYN_REPORT, 0);
}

PUBLIC void hid_mouse_release_virtio(hid_mouse_t* mouse, hid_btns_t btns)
{
    if (!mouse || !mouse->dev) {
        return;
    }
    if (btns & HID_BTN_LEFT) virtio_input_send_event(mouse->dev, EV_KEY, BTN_LEFT, 0);
    if (btns & HID_BTN_RIGHT) virtio_input_send_event(mouse->dev, EV_KEY, BTN_RIGHT, 0);
    if (btns & HID_BTN_MIDDLE) virtio_input_send_event(mouse->dev, EV_KEY, BTN_MIDDLE, 0);
    virtio_input_send_event(mouse->dev, EV_SYN, SYN_REPORT, 0);
}

PUBLIC void hid_mouse_scroll_virtio(hid_mouse_t* mouse, int32_t offset)
{
    if (!mouse || !mouse->dev || offset == 0) {
        return;
    }
    virtio_input_send_event(mouse->dev, EV_REL, REL_WHEEL, offset);
    virtio_input_send_event(mouse->dev, EV_SYN, SYN_REPORT, 0);
}

PUBLIC void hid_mouse_resolution_virtio(hid_mouse_t* mouse, uint32_t x, uint32_t y)
{
    if (!mouse || !mouse->dev) {
        return;
    }
    spin_lock(&mouse->dev->lock);
    mouse->dev->width = (uint16_t)((x > 0xFFFF) ? 0xFFFF : x);
    mouse->dev->height = (uint16_t)((y > 0xFFFF) ? 0xFFFF : y);
    virtio_input_update_cfg(mouse->dev);
    mouse->dev->config_generation++;
    spin_unlock(&mouse->dev->lock);
}

PUBLIC void hid_mouse_move_virtio(hid_mouse_t* mouse, int32_t x, int32_t y)
{
    if (!mouse || !mouse->dev) {
        return;
    }
    if (x) virtio_input_send_event(mouse->dev, EV_REL, REL_X, x);
    if (y) virtio_input_send_event(mouse->dev, EV_REL, REL_Y, y);
    virtio_input_send_event(mouse->dev, EV_SYN, SYN_REPORT, 0);
}

PUBLIC void hid_mouse_place_virtio(hid_mouse_t* mouse, int32_t x, int32_t y)
{
    if (!mouse || !mouse->dev) {
        return;
    }
    uint32_t nx = (x < 0) ? 0 : (uint32_t)x;
    uint32_t ny = (y < 0) ? 0 : (uint32_t)y;
    spin_lock(&mouse->dev->lock);
    uint32_t maxx = mouse->dev->width ? (uint32_t)(mouse->dev->width - 1) : 0;
    uint32_t maxy = mouse->dev->height ? (uint32_t)(mouse->dev->height - 1) : 0;
    if (nx > maxx) nx = maxx;
    if (ny > maxy) ny = maxy;
    spin_unlock(&mouse->dev->lock);
    virtio_input_send_event(mouse->dev, EV_ABS, ABS_X, (int32_t)nx);
    virtio_input_send_event(mouse->dev, EV_ABS, ABS_Y, (int32_t)ny);
    virtio_input_send_event(mouse->dev, EV_SYN, SYN_REPORT, 0);
}
