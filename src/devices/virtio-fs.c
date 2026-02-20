#include "virtio-fs.h"

#include "fdtlib.h"
#include "hashmap.h"
#include "mem_ops.h"
#include "utils.h"

#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/statvfs.h>
#include <unistd.h>

#if !defined(seekdir) && !defined(__APPLE__)
extern void seekdir(DIR* dirp, long loc);
extern long telldir(DIR* dirp);
#endif

#if !defined(__APPLE__)
extern int lstat(const char* path, struct stat* buf);
extern int ftruncate(int fd, off_t length);
extern int truncate(const char* path, off_t length);
extern ssize_t readlink(const char* path, char* buf, size_t bufsiz);
extern int symlink(const char* target, const char* linkpath);
extern int mknod(const char* pathname, mode_t mode, dev_t dev);
extern int dirfd(DIR* dirp);
extern ssize_t pread(int fd, void* buf, size_t count, off_t offset);
extern ssize_t pwrite(int fd, const void* buf, size_t count, off_t offset);
#endif

#ifndef S_IFMT
#define S_IFMT 0170000
#endif
#ifndef S_IFSOCK
#define S_IFSOCK 0140000
#endif
#ifndef S_ISSOCK
#define S_ISSOCK(m) (((m)&S_IFMT) == S_IFSOCK)
#endif

enum {
    VIRTIO_MMIO_MAGIC              = 0x000,
    VIRTIO_MMIO_VERSION            = 0x004,
    VIRTIO_MMIO_DEVICE_ID          = 0x008,
    VIRTIO_MMIO_VENDOR_ID          = 0x00c,
    VIRTIO_MMIO_DEVICE_FEATURES    = 0x010,
    VIRTIO_MMIO_DEVICE_FEATURES_SEL = 0x014,
    VIRTIO_MMIO_DRIVER_FEATURES    = 0x020,
    VIRTIO_MMIO_DRIVER_FEATURES_SEL = 0x024,
    VIRTIO_MMIO_QUEUE_SEL          = 0x030,
    VIRTIO_MMIO_QUEUE_NUM_MAX      = 0x034,
    VIRTIO_MMIO_QUEUE_NUM          = 0x038,
    VIRTIO_MMIO_QUEUE_READY        = 0x044,
    VIRTIO_MMIO_QUEUE_NOTIFY       = 0x050,
    VIRTIO_MMIO_INTERRUPT_STATUS   = 0x060,
    VIRTIO_MMIO_INTERRUPT_ACK      = 0x064,
    VIRTIO_MMIO_STATUS             = 0x070,
    VIRTIO_MMIO_QUEUE_DESC_LOW     = 0x080,
    VIRTIO_MMIO_QUEUE_DESC_HIGH    = 0x084,
    VIRTIO_MMIO_QUEUE_DRIVER_LOW   = 0x090,
    VIRTIO_MMIO_QUEUE_DRIVER_HIGH  = 0x094,
    VIRTIO_MMIO_QUEUE_DEVICE_LOW   = 0x0a0,
    VIRTIO_MMIO_QUEUE_DEVICE_HIGH  = 0x0a4,
    VIRTIO_MMIO_CONFIG_GENERATION  = 0x0fc,
    VIRTIO_MMIO_CONFIG             = 0x100,
};

__attribute__((weak)) void rvvm_ios_virtiofs_debug_print(const char* s)
{
    if (!s) {
        return;
    }
    size_t n = strlen(s);
    if (n && s[n - 1] == '\n') {
        fputs(s, stderr);
    } else {
        fputs(s, stderr);
        fputc('\n', stderr);
    }
    fflush(stderr);
}

static void virtio_fs_uart_printf(const char* fmt, ...)
{
    static uint32_t budget = 200000;
    if (budget == 0) {
        return;
    }
    budget--;
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    rvvm_ios_virtiofs_debug_print(buf);
}

static bool virtio_fs_hostfs_active = false;

bool virtio_fs_hostfs_enabled(void)
{
    return virtio_fs_hostfs_active;
}

enum {
    VIRTIO_MMIO_MAGIC_VALUE = 0x74726976,
    VIRTIO_MMIO_VERSION_VALUE = 2,
    VIRTIO_MMIO_VENDOR_VALUE = 0x5256564d,
    VIRTIO_ID_FS = 26,
};

enum {
    VIRTIO_F_VERSION_1 = 32,
    VIRTIO_F_ACCESS_PLATFORM = 33,
};

enum {
    VIRTIO_RING_F_INDIRECT_DESC = 28,
    VIRTIO_RING_F_EVENT_IDX = 29,
};

enum {
    VIRTQ_DESC_F_NEXT     = 1,
    VIRTQ_DESC_F_WRITE    = 2,
    VIRTQ_DESC_F_INDIRECT = 4,
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

typedef struct {
    uint64_t unique;
    uint64_t nodeid;
    uint32_t uid;
    uint32_t gid;
    uint32_t pid;
    uint32_t padding;
    uint32_t opcode;
    uint32_t len;
} fuse_in_hdr_fields_t;

enum {
    FUSE_ROOT_ID = 1,
};

enum {
    FUSE_SETATTR_MODE      = 1u << 0,
    FUSE_SETATTR_UID       = 1u << 1,
    FUSE_SETATTR_GID       = 1u << 2,
    FUSE_SETATTR_SIZE      = 1u << 3,
    FUSE_SETATTR_ATIME     = 1u << 4,
    FUSE_SETATTR_MTIME     = 1u << 5,
    FUSE_SETATTR_FH        = 1u << 6,
    FUSE_SETATTR_ATIME_NOW = 1u << 7,
    FUSE_SETATTR_MTIME_NOW = 1u << 8,
};

enum {
    FUSE_LOOKUP     = 1,
    FUSE_FORGET     = 2,
    FUSE_GETATTR    = 3,
    FUSE_SETATTR    = 4,
    FUSE_READLINK   = 5,
    FUSE_SYMLINK    = 6,
    FUSE_MKNOD      = 8,
    FUSE_MKDIR      = 9,
    FUSE_UNLINK     = 10,
    FUSE_RMDIR      = 11,
    FUSE_RENAME     = 12,
    FUSE_LINK       = 13,
    FUSE_OPEN       = 14,
    FUSE_READ       = 15,
    FUSE_WRITE      = 16,
    FUSE_STATFS     = 17,
    FUSE_RELEASE    = 18,
    FUSE_FSYNC      = 20,
    FUSE_SETXATTR   = 21,
    FUSE_GETXATTR   = 22,
    FUSE_LISTXATTR  = 23,
    FUSE_REMOVEXATTR = 24,
    FUSE_FLUSH      = 25,
    FUSE_INIT       = 26,
    FUSE_OPENDIR    = 27,
    FUSE_READDIR    = 28,
    FUSE_RELEASEDIR = 29,
    FUSE_FSYNC_DIR  = 30,
    FUSE_GETLK      = 31,
    FUSE_SETLK      = 32,
    FUSE_SETLKW     = 33,
    FUSE_ACCESS     = 34,
    FUSE_CREATE     = 35,
    FUSE_INTERRUPT  = 36,
    FUSE_BMAP       = 37,
    FUSE_DESTROY    = 38,
    FUSE_IOCTL      = 39,
    FUSE_POLL       = 40,
    FUSE_NOTIFY_REPLY = 41,
    FUSE_BATCH_FORGET = 42,
    FUSE_FALLOCATE  = 43,
    FUSE_READDIRPLUS = 44,
};

typedef struct {
    rvvm_machine_t* machine;
    rvvm_intc_t*    intc;
    rvvm_irq_t      irq;

    char            tag[36];
    uint32_t        num_request_queues;
    uint32_t        notify_buf_size;
    uint32_t        config_generation;

    uint32_t        host_features_sel;
    uint32_t        driver_features_sel;
    uint32_t        host_features[2];
    uint32_t        driver_features[2];

    uint32_t        status;
    uint32_t        interrupt_status;

    uint32_t        queue_sel;
    virtio_queue_t  q[2];

    char*           root_path;
    hashmap_t       inode_to_path;
    hashmap_t       fh_to_handle;

    uint8_t*        req_buf;
    uint8_t*        resp_buf;
    size_t          buf_cap;
} virtio_fs_dev_t;

typedef struct {
    uint8_t is_dir;
    uint32_t open_flags;
    char* path;
    union {
        int  fd;
        DIR* dir;
    } u;
} virtio_fs_handle_t;

static uint64_t fnv1a64(const char* s)
{
    uint64_t h = 1469598103934665603ULL;
    if (!s) {
        return h;
    }
    for (const uint8_t* p = (const uint8_t*)s; *p; ++p) {
        h ^= *p;
        h *= 1099511628211ULL;
    }
    return h;
}

static char* str_dup(const char* s)
{
    if (!s) {
        return NULL;
    }
    size_t n = strlen(s);
    char* out = safe_malloc(n + 1);
    memcpy(out, s, n + 1);
    return out;
}

static bool is_safe_name(const char* name)
{
    if (!name || !*name) {
        return false;
    }
    if (strcmp(name, ".") == 0 || strcmp(name, "..") == 0) {
        return false;
    }
    for (const char* p = name; *p; ++p) {
        if (*p == '/') {
            return false;
        }
    }
    return true;
}

static char* path_join2(const char* base, const char* name)
{
    if (!base || !name) {
        return NULL;
    }
    size_t bl = strlen(base);
    size_t nl = strlen(name);
    bool need_slash = (bl > 0 && base[bl - 1] != '/');
    size_t out_len = bl + (need_slash ? 1 : 0) + nl;
    char* out = safe_malloc(out_len + 1);
    memcpy(out, base, bl);
    size_t o = bl;
    if (need_slash) {
        out[o++] = '/';
    }
    memcpy(out + o, name, nl);
    out[out_len] = 0;
    return out;
}

static int vfs_lstat(const char* path, struct stat* st)
{
    if (!path || !st) {
        errno = EINVAL;
        return -1;
    }
    if (lstat(path, st) == 0) {
        return 0;
    }
    int err_lstat = errno;

#if defined(AT_FDCWD) && defined(AT_SYMLINK_NOFOLLOW)
    if (fstatat(AT_FDCWD, path, st, AT_SYMLINK_NOFOLLOW) == 0) {
        return 0;
    }
    int err_fstatat = errno;
#else
    int err_fstatat = ENOSYS;
#endif

    if (err_lstat == ENOSYS || err_fstatat == ENOSYS) {
        if (stat(path, st) == 0) {
            return 0;
        }
    }

    int err_stat = errno;

    int err_open = ENOSYS;
    int err_fstat = ENOSYS;
    int fd = open(path, O_RDONLY);
    if (fd >= 0) {
        if (fstat(fd, st) == 0) {
            close(fd);
            return 0;
        }
        err_fstat = errno;
        close(fd);
    } else {
        err_open = errno;
    }

    virtio_fs_uart_printf("virtio-fs: vfs_lstat failed path=%s lstat=%d fstatat=%d stat=%d open=%d fstat=%d errno=%d\n",
                          path,
                          err_lstat,
                          err_fstatat,
                          err_stat,
                          err_open,
                          err_fstat,
                          errno);
    return -1;
}

static const char* inode_to_path(virtio_fs_dev_t* vfs, uint64_t nodeid)
{
    if (!vfs) {
        return NULL;
    }
    if (nodeid == FUSE_ROOT_ID) {
        return vfs->root_path;
    }
    return (const char*)hashmap_get_ptr(&vfs->inode_to_path, (size_t)nodeid);
}

static uint64_t ensure_inode_for_path(virtio_fs_dev_t* vfs, const char* path)
{
    if (!vfs || !path) {
        return 0;
    }
    uint64_t id = fnv1a64(path);
    if (id < 2) {
        id += 2;
    }
    for (uint32_t tries = 0; tries < 1024; ++tries) {
        const char* existing = (const char*)hashmap_get_ptr(&vfs->inode_to_path, (size_t)id);
        if (!existing) {
            hashmap_put_ptr(&vfs->inode_to_path, (size_t)id, str_dup(path));
            return id;
        }
        if (strcmp(existing, path) == 0) {
            return id;
        }
        id++;
        if (id < 2) {
            id += 2;
        }
    }
    return 0;
}

static void* dma_ptr(virtio_fs_dev_t* vfs, uint64_t addr, size_t len)
{
    return rvvm_get_dma_ptr(vfs->machine, (rvvm_addr_t)addr, len);
}

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

typedef struct {
    uint16_t head;
    uint32_t total_len;
    uint32_t in_count;
    uint32_t out_count;
    struct {
        uint64_t addr;
        uint32_t len;
        uint8_t  write;
    } seg[128];
} virtq_chain_t;

static bool virtq_load_desc_at(virtio_fs_dev_t* vfs, uint64_t table_addr, uint32_t table_entries, uint16_t idx, virtq_desc_t* out)
{
    if (!vfs || !out || table_addr == 0 || table_entries == 0 || idx >= table_entries) {
        return false;
    }
    uint64_t off = (uint64_t)idx * sizeof(virtq_desc_t);
    uint8_t* p = dma_ptr(vfs, table_addr + off, sizeof(virtq_desc_t));
    if (!p) {
        return false;
    }
    out->addr  = read_uint64_le(p + 0);
    out->len   = read_uint32_le(p + 8);
    out->flags = read_uint16_le(p + 12);
    out->next  = read_uint16_le(p + 14);
    return true;
}

static bool virtq_collect_chain(virtio_fs_dev_t* vfs, virtio_queue_t* q, uint16_t head, virtq_chain_t* chain)
{
    if (!vfs || !q || !chain || q->num == 0) {
        return false;
    }
    memset(chain, 0, sizeof(*chain));
    chain->head = head;

    virtq_desc_t head_desc;
    if (!virtq_load_desc_at(vfs, q->desc_addr, q->num, head, &head_desc)) {
        return false;
    }

    if (head_desc.flags & VIRTQ_DESC_F_INDIRECT) {
        uint32_t entries = head_desc.len / (uint32_t)sizeof(virtq_desc_t);
        if (entries == 0 || entries > 4096) {
            return false;
        }
        uint16_t idx = 0;
        for (uint32_t hops = 0; hops < entries && chain->in_count + chain->out_count < (uint32_t)STATIC_ARRAY_SIZE(chain->seg); ++hops) {
            virtq_desc_t d;
            if (!virtq_load_desc_at(vfs, head_desc.addr, entries, idx, &d)) {
                return false;
            }
            if (d.flags & VIRTQ_DESC_F_INDIRECT) {
                return false;
            }
            uint8_t write = (d.flags & VIRTQ_DESC_F_WRITE) ? 1 : 0;
            uint32_t si = chain->in_count + chain->out_count;
            chain->seg[si].addr = d.addr;
            chain->seg[si].len = d.len;
            chain->seg[si].write = write;
            chain->total_len += d.len;
            if (write) {
                chain->out_count++;
            } else {
                chain->in_count++;
            }
            if (d.flags & VIRTQ_DESC_F_NEXT) {
                idx = d.next;
                continue;
            }
            return true;
        }
        return false;
    } else {
        uint16_t idx = head;
        for (uint32_t hops = 0; hops < q->num && chain->in_count + chain->out_count < (uint32_t)STATIC_ARRAY_SIZE(chain->seg); ++hops) {
            virtq_desc_t d;
            if (!virtq_load_desc_at(vfs, q->desc_addr, q->num, idx, &d)) {
                return false;
            }
            uint8_t write = (d.flags & VIRTQ_DESC_F_WRITE) ? 1 : 0;
            uint32_t si = chain->in_count + chain->out_count;
            chain->seg[si].addr = d.addr;
            chain->seg[si].len = d.len;
            chain->seg[si].write = write;
            chain->total_len += d.len;
            if (write) {
                chain->out_count++;
            } else {
                chain->in_count++;
            }
            if (d.flags & VIRTQ_DESC_F_NEXT) {
                idx = d.next;
                continue;
            }
            return true;
        }
        return false;
    }
}

static size_t chain_read_all(virtio_fs_dev_t* vfs, const virtq_chain_t* chain, uint8_t* dst, size_t cap)
{
    size_t off = 0;
    uint32_t segs = chain->in_count + chain->out_count;
    for (uint32_t i = 0; i < segs; ++i) {
        if (chain->seg[i].write) {
            continue;
        }
        size_t n = chain->seg[i].len;
        if (off + n > cap) {
            n = (cap > off) ? (cap - off) : 0;
        }
        if (n == 0) {
            continue;
        }
        void* p = dma_ptr(vfs, chain->seg[i].addr, n);
        if (!p) {
            return 0;
        }
        memcpy(dst + off, p, n);
        off += n;
        if (off == cap) {
            break;
        }
    }
    return off;
}

static size_t chain_write_all(virtio_fs_dev_t* vfs, const virtq_chain_t* chain, const uint8_t* src, size_t len)
{
    size_t off = 0;
    uint32_t segs = chain->in_count + chain->out_count;
    for (uint32_t i = 0; i < segs; ++i) {
        if (!chain->seg[i].write) {
            continue;
        }
        size_t n = chain->seg[i].len;
        if (off + n > len) {
            n = (len > off) ? (len - off) : 0;
        }
        if (n == 0) {
            continue;
        }
        void* p = dma_ptr(vfs, chain->seg[i].addr, n);
        if (!p) {
            return 0;
        }
        memcpy(p, src + off, n);
        off += n;
        if (off == len) {
            break;
        }
    }
    return off;
}

static void virtq_push_used(virtio_fs_dev_t* vfs, virtio_queue_t* q, uint16_t head, uint32_t len)
{
    if (!vfs || !q || q->used_addr == 0 || q->num == 0) {
        return;
    }
    uint8_t* used_p = dma_ptr(vfs, q->used_addr, 4 + (size_t)q->num * sizeof(virtq_used_elem_t));
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

    vfs->interrupt_status |= 1;
    rvvm_raise_irq(vfs->intc, vfs->irq);
}

static bool parse_in_header(const uint8_t* buf, size_t len, fuse_in_hdr_fields_t* out)
{
    if (!buf || !out || len < 40) {
        return false;
    }
    out->len = read_uint32_le(buf + 0);
    out->opcode = read_uint32_le(buf + 4);
    out->unique = read_uint64_le(buf + 8);
    out->nodeid = read_uint64_le(buf + 16);
    out->uid = read_uint32_le(buf + 24);
    out->gid = read_uint32_le(buf + 28);
    out->pid = read_uint32_le(buf + 32);
    out->padding = read_uint32_le(buf + 36);
    return true;
}

static void write_out_header(uint8_t* out, uint32_t len, int32_t err, uint64_t unique)
{
    write_uint32_le(out + 0, len);
    write_uint32_le(out + 4, (uint32_t)err);
    write_uint64_le(out + 8, unique);
}

static uint32_t fill_attr(uint8_t* out, const struct stat* st)
{
    memset(out, 0, 88);
    write_uint64_le(out + 0, (uint64_t)st->st_ino);
    write_uint64_le(out + 8, (uint64_t)st->st_size);
    write_uint64_le(out + 16, (uint64_t)st->st_blocks);
    write_uint64_le(out + 24, (uint64_t)st->st_atime);
    write_uint64_le(out + 32, (uint64_t)st->st_mtime);
    write_uint64_le(out + 40, (uint64_t)st->st_ctime);
#if defined(__APPLE__)
    write_uint32_le(out + 48, (uint32_t)st->st_atimespec.tv_nsec);
    write_uint32_le(out + 52, (uint32_t)st->st_mtimespec.tv_nsec);
    write_uint32_le(out + 56, (uint32_t)st->st_ctimespec.tv_nsec);
#else
#if defined(__GLIBC__) && !defined(__USE_XOPEN2K8) && !defined(__USE_MISC)
    write_uint32_le(out + 48, (uint32_t)st->st_atimensec);
    write_uint32_le(out + 52, (uint32_t)st->st_mtimensec);
    write_uint32_le(out + 56, (uint32_t)st->st_ctimensec);
#else
    write_uint32_le(out + 48, (uint32_t)st->st_atim.tv_nsec);
    write_uint32_le(out + 52, (uint32_t)st->st_mtim.tv_nsec);
    write_uint32_le(out + 56, (uint32_t)st->st_ctim.tv_nsec);
#endif
#endif
    write_uint32_le(out + 60, (uint32_t)st->st_mode);
    write_uint32_le(out + 64, (uint32_t)st->st_nlink);
    write_uint32_le(out + 68, (uint32_t)st->st_uid);
    write_uint32_le(out + 72, (uint32_t)st->st_gid);
    write_uint32_le(out + 76, (uint32_t)st->st_rdev);
    write_uint32_le(out + 80, 0);
    write_uint32_le(out + 84, 0);
    return 88;
}

static uint32_t fill_entry_out(uint8_t* out, uint64_t nodeid, const struct stat* st)
{
    memset(out, 0, 40 + 88);
    write_uint64_le(out + 0, nodeid);
    write_uint64_le(out + 8, 1);
    write_uint64_le(out + 16, 1);
    write_uint32_le(out + 24, 1);
    write_uint32_le(out + 28, 0);
    write_uint32_le(out + 32, 1);
    write_uint32_le(out + 36, 0);
    fill_attr(out + 40, st);
    return 40 + 88;
}

static uint32_t fill_attr_out(uint8_t* out, const struct stat* st)
{
    memset(out, 0, 16 + 88);
    write_uint64_le(out + 0, 1);
    write_uint32_le(out + 8, 0);
    write_uint32_le(out + 12, 1);
    fill_attr(out + 16, st);
    return 16 + 88;
}

static uint32_t fill_open_out(uint8_t* out, uint64_t fh)
{
    memset(out, 0, 24);
    write_uint64_le(out + 0, fh);
    write_uint32_le(out + 8, 0);
    write_uint32_le(out + 12, 0);
    write_uint64_le(out + 16, 0);
    return 24;
}

static uint32_t fill_write_out(uint8_t* out, uint32_t size)
{
    memset(out, 0, 8);
    write_uint32_le(out + 0, size);
    write_uint32_le(out + 4, 0);
    return 8;
}

static uint32_t fill_init_out(uint8_t* out, uint32_t minor, uint32_t max_readahead)
{
    memset(out, 0, 64);
    write_uint32_le(out + 0, 7);
    write_uint32_le(out + 4, minor);
    write_uint32_le(out + 8, max_readahead);
    write_uint32_le(out + 12, 0);
    write_uint16_le(out + 16, 0xffff);
    write_uint16_le(out + 18, (uint16_t)((0xffffu / 4u) * 3u));
    write_uint32_le(out + 20, 256u * 1024u);
    write_uint32_le(out + 24, 1);
    write_uint16_le(out + 28, 64);
    write_uint16_le(out + 30, 0);
    write_uint32_le(out + 32, 0);
    return 64;
}

static uint32_t align8(uint32_t v)
{
    return (v + 7u) & ~7u;
}

static uint32_t mode_to_dt(mode_t m)
{
    if (S_ISFIFO(m)) {
        return 1;
    }
    if (S_ISCHR(m)) {
        return 2;
    }
    if (S_ISDIR(m)) {
        return 4;
    }
    if (S_ISBLK(m)) {
        return 6;
    }
    if (S_ISREG(m)) {
        return 8;
    }
    if (S_ISLNK(m)) {
        return 10;
    }
    if (S_ISSOCK(m)) {
        return 12;
    }
    return 0;
}

static uint32_t fill_dirents(virtio_fs_dev_t* vfs, uint8_t* out, uint32_t cap, DIR* dir, uint64_t dir_nodeid, const char* dir_path, uint64_t off)
{
    uint32_t written = 0;
    if (!dir || !dir_path) {
        return 0;
    }

    if (off == 0) {
        rewinddir(dir);
    } else {
        seekdir(dir, (long)off);
    }

    struct dirent* ent;
    while ((ent = readdir(dir)) != NULL) {
        const char* name = ent->d_name;
        if (!is_safe_name(name) && strcmp(name, ".") != 0 && strcmp(name, "..") != 0) {
            continue;
        }
        uint64_t ino = 0;
        uint32_t dtype = (uint32_t)ent->d_type;
        if (strcmp(name, ".") == 0 || strcmp(name, "..") == 0) {
            ino = dir_nodeid;
            if (dtype == 0) {
                dtype = 4;
            }
        } else {
            char* full = path_join2(dir_path, name);
            if (full) {
                struct stat st;
                if (vfs_lstat(full, &st) == 0) {
                    if (dtype == 0) {
                        dtype = mode_to_dt(st.st_mode);
                    }
                }
                ino = ensure_inode_for_path(vfs, full);
                free(full);
            }
            if (ino == 0) {
                ino = (uint64_t)ent->d_ino;
            }
        }
        uint32_t name_len = (uint32_t)strlen(name);
        uint32_t ent_len = align8(24 + name_len);
        if (written + ent_len > cap) {
            break;
        }
        uint8_t* p = out + written;
        long next_off_l = telldir(dir);
        uint64_t next_off = (next_off_l > 0) ? (uint64_t)next_off_l : (off + 1);
        write_uint64_le(p + 0, ino);
        write_uint64_le(p + 8, next_off);
        write_uint32_le(p + 16, name_len);
        write_uint32_le(p + 20, dtype);
        memcpy(p + 24, name, name_len);
        memset(p + 24 + name_len, 0, ent_len - (24 + name_len));
        written += ent_len;
        off = next_off;
    }
    return written;
}

static uint32_t fill_direntsplus(virtio_fs_dev_t* vfs, uint8_t* out, uint32_t cap, DIR* dir, uint64_t dir_nodeid, const char* dir_path, uint64_t off)
{
    uint32_t written = 0;
    if (!dir || !dir_path) {
        return 0;
    }

    if (off == 0) {
        rewinddir(dir);
    } else {
        seekdir(dir, (long)off);
    }

    struct dirent* ent;
    while ((ent = readdir(dir)) != NULL) {
        const char* name = ent->d_name;
        if (!is_safe_name(name) && strcmp(name, ".") != 0 && strcmp(name, "..") != 0) {
            continue;
        }

        const char* full_path = NULL;
        char* full_alloc = NULL;
        uint64_t ino = 0;
        if (strcmp(name, ".") == 0 || strcmp(name, "..") == 0) {
            ino = dir_nodeid;
            full_path = dir_path;
        } else {
            full_alloc = path_join2(dir_path, name);
            if (!full_alloc) {
                continue;
            }
            full_path = full_alloc;
            ino = ensure_inode_for_path(vfs, full_alloc);
            if (ino == 0) {
                ino = (uint64_t)ent->d_ino;
            }
        }

        struct stat st;
        if (!full_path || vfs_lstat(full_path, &st) != 0) {
            free(full_alloc);
            continue;
        }

        uint32_t name_len = (uint32_t)strlen(name);
        uint32_t entry_len = 40 + 88;
        uint32_t dirent_len = align8(24 + name_len);
        uint32_t rec_len = align8(entry_len + dirent_len);
        if (written + rec_len > cap) {
            free(full_alloc);
            break;
        }

        uint8_t* p = out + written;
        uint32_t payload = fill_entry_out(p, ino, &st);
        if (payload != entry_len) {
            free(full_alloc);
            break;
        }

        long next_off_l = telldir(dir);
        uint64_t next_off = (next_off_l > 0) ? (uint64_t)next_off_l : (off + 1);

        uint8_t* d = p + entry_len;
        uint32_t dtype = (uint32_t)ent->d_type;
        if (dtype == 0) {
            dtype = mode_to_dt(st.st_mode);
        }
        write_uint64_le(d + 0, ino);
        write_uint64_le(d + 8, next_off);
        write_uint32_le(d + 16, name_len);
        write_uint32_le(d + 20, dtype);
        memcpy(d + 24, name, name_len);
        memset(d + 24 + name_len, 0, dirent_len - (24 + name_len));
        if (rec_len > entry_len + dirent_len) {
            memset(d + dirent_len, 0, rec_len - (entry_len + dirent_len));
        }

        written += rec_len;
        off = next_off;
        free(full_alloc);
    }
    return written;
}

static const char* fuse_opcode_name(uint32_t opcode)
{
    switch (opcode) {
        case FUSE_LOOKUP: return "LOOKUP";
        case FUSE_FORGET: return "FORGET";
        case FUSE_GETATTR: return "GETATTR";
        case FUSE_SETATTR: return "SETATTR";
        case FUSE_READLINK: return "READLINK";
        case FUSE_SYMLINK: return "SYMLINK";
        case FUSE_MKNOD: return "MKNOD";
        case FUSE_MKDIR: return "MKDIR";
        case FUSE_UNLINK: return "UNLINK";
        case FUSE_RMDIR: return "RMDIR";
        case FUSE_RENAME: return "RENAME";
        case FUSE_LINK: return "LINK";
        case FUSE_OPEN: return "OPEN";
        case FUSE_READ: return "READ";
        case FUSE_WRITE: return "WRITE";
        case FUSE_STATFS: return "STATFS";
        case FUSE_RELEASE: return "RELEASE";
        case FUSE_FSYNC: return "FSYNC";
        case FUSE_SETXATTR: return "SETXATTR";
        case FUSE_GETXATTR: return "GETXATTR";
        case FUSE_LISTXATTR: return "LISTXATTR";
        case FUSE_REMOVEXATTR: return "REMOVEXATTR";
        case FUSE_FLUSH: return "FLUSH";
        case FUSE_INIT: return "INIT";
        case FUSE_OPENDIR: return "OPENDIR";
        case FUSE_READDIR: return "READDIR";
        case FUSE_RELEASEDIR: return "RELEASEDIR";
        case FUSE_FSYNC_DIR: return "FSYNC_DIR";
        case FUSE_GETLK: return "GETLK";
        case FUSE_SETLK: return "SETLK";
        case FUSE_SETLKW: return "SETLKW";
        case FUSE_ACCESS: return "ACCESS";
        case FUSE_CREATE: return "CREATE";
        case FUSE_INTERRUPT: return "INTERRUPT";
        case FUSE_BMAP: return "BMAP";
        case FUSE_DESTROY: return "DESTROY";
        case FUSE_IOCTL: return "IOCTL";
        case FUSE_POLL: return "POLL";
        case FUSE_NOTIFY_REPLY: return "NOTIFY_REPLY";
        case FUSE_BATCH_FORGET: return "BATCH_FORGET";
        case FUSE_FALLOCATE: return "FALLOCATE";
        case FUSE_READDIRPLUS: return "READDIRPLUS";
        default: return "UNKNOWN";
    }
}

static uint32_t fuse_handle_request(virtio_fs_dev_t* vfs, const uint8_t* req, size_t req_len, uint8_t* resp, size_t resp_cap)
{
    fuse_in_hdr_fields_t in;
    if (!parse_in_header(req, req_len, &in)) {
        return 0;
    }

    uint32_t out_len = 16;
    int32_t out_err = 0;

    virtio_fs_uart_printf("virtio-fs: req op=%u(%s) node=%llu uniq=%llu len=%u uid=%u gid=%u pid=%u\n",
                          (unsigned)in.opcode,
                          fuse_opcode_name(in.opcode),
                          (unsigned long long)in.nodeid,
                          (unsigned long long)in.unique,
                          (unsigned)in.len,
                          (unsigned)in.uid,
                          (unsigned)in.gid,
                          (unsigned)in.pid);

    switch (in.opcode) {
        case FUSE_INIT: {
            if (req_len < 40 + 16) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            uint32_t major = read_uint32_le(req + 40);
            uint32_t minor = read_uint32_le(req + 44);
            uint32_t max_readahead = read_uint32_le(req + 48);
            if (major < 7) {
                out_err = -EPROTO;
                out_len = 16;
                break;
            }
            if (resp_cap < 16 + 64) {
                return 0;
            }
            uint32_t out_minor = minor;
            if (out_minor > 38) {
                out_minor = 38;
            }
            uint32_t payload = fill_init_out(resp + 16, out_minor, max_readahead);
            out_len = 16 + payload;
            break;
        }
        case FUSE_LOOKUP: {
            const char* parent_path = inode_to_path(vfs, in.nodeid);
            if (!parent_path) {
                out_err = -ENOENT;
                out_len = 16;
                break;
            }
            const char* name = (const char*)(req + 40);
            size_t max_name = req_len > 40 ? (req_len - 40) : 0;
            size_t name_len = rvvm_strnlen(name, max_name);
            if (name_len == 0 || name_len == max_name) {
                out_err = -ENOENT;
                out_len = 16;
                break;
            }
            if (!is_safe_name(name)) {
                out_err = -ENOENT;
                out_len = 16;
                break;
            }
            char* full = path_join2(parent_path, name);
            if (!full) {
                out_err = -ENOMEM;
                out_len = 16;
                break;
            }
            struct stat st;
            if (vfs_lstat(full, &st) != 0) {
                free(full);
                out_err = -errno;
                out_len = 16;
                break;
            }
            uint64_t nodeid = ensure_inode_for_path(vfs, full);
            if (nodeid == 0) {
                free(full);
                out_err = -ENOMEM;
                out_len = 16;
                break;
            }
            free(full);
            if (resp_cap < 16 + 40 + 88) {
                return 0;
            }
            uint32_t payload = fill_entry_out(resp + 16, nodeid, &st);
            out_len = 16 + payload;
            break;
        }
        case FUSE_GETATTR: {
            const char* path = inode_to_path(vfs, in.nodeid);
            if (!path) {
                out_err = -ENOENT;
                out_len = 16;
                break;
            }
            struct stat st;
            if (vfs_lstat(path, &st) != 0) {
                out_err = -errno;
                out_len = 16;
                break;
            }
            if (resp_cap < 16 + 16 + 88) {
                return 0;
            }
            uint32_t payload = fill_attr_out(resp + 16, &st);
            out_len = 16 + payload;
            break;
        }
        case FUSE_SETATTR: {
            if (req_len < 40 + 88) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            const char* path = inode_to_path(vfs, in.nodeid);
            if (!path) {
                out_err = -ENOENT;
                out_len = 16;
                break;
            }
            uint32_t valid = read_uint32_le(req + 40);
            uint64_t fh = read_uint64_le(req + 48);
            uint64_t size = read_uint64_le(req + 56);
            uint32_t mode = read_uint32_le(req + 108);
            uint32_t uid = read_uint32_le(req + 116);
            uint32_t gid = read_uint32_le(req + 120);

            if (valid & FUSE_SETATTR_MODE) {
                if (chmod(path, (mode_t)(mode & 07777u)) != 0) {
                    out_err = -errno;
                    out_len = 16;
                    break;
                }
            }
            if (valid & (FUSE_SETATTR_UID | FUSE_SETATTR_GID)) {
                uid_t u = (valid & FUSE_SETATTR_UID) ? (uid_t)uid : (uid_t)-1;
                gid_t g = (valid & FUSE_SETATTR_GID) ? (gid_t)gid : (gid_t)-1;
                if (chown(path, u, g) != 0) {
                    out_err = -errno;
                    out_len = 16;
                    break;
                }
            }
            if (valid & FUSE_SETATTR_SIZE) {
                int rc = -1;
                if (valid & FUSE_SETATTR_FH) {
                    virtio_fs_handle_t* h = (virtio_fs_handle_t*)hashmap_get_ptr(&vfs->fh_to_handle, (size_t)fh);
                    if (h && !h->is_dir) {
                        rc = ftruncate(h->u.fd, (off_t)size);
                    } else {
                        rc = truncate(path, (off_t)size);
                    }
                } else {
                    rc = truncate(path, (off_t)size);
                }
                if (rc != 0) {
                    out_err = -errno;
                    out_len = 16;
                    break;
                }
            }
            if (valid & (FUSE_SETATTR_ATIME | FUSE_SETATTR_MTIME | FUSE_SETATTR_ATIME_NOW | FUSE_SETATTR_MTIME_NOW)) {
                struct stat st0;
                if (vfs_lstat(path, &st0) != 0) {
                    out_err = -errno;
                    out_len = 16;
                    break;
                }
                struct timeval tv[2];
                tv[0].tv_sec = st0.st_atime;
                tv[0].tv_usec = 0;
                tv[1].tv_sec = st0.st_mtime;
                tv[1].tv_usec = 0;

                if (valid & FUSE_SETATTR_ATIME_NOW) {
                    struct timeval now;
                    gettimeofday(&now, NULL);
                    tv[0] = now;
                } else if (valid & FUSE_SETATTR_ATIME) {
                    uint64_t atime_sec = read_uint64_le(req + 72);
                    uint32_t atime_nsec = read_uint32_le(req + 96);
                    tv[0].tv_sec = (time_t)atime_sec;
                    tv[0].tv_usec = (suseconds_t)(atime_nsec / 1000u);
                }

                if (valid & FUSE_SETATTR_MTIME_NOW) {
                    struct timeval now;
                    gettimeofday(&now, NULL);
                    tv[1] = now;
                } else if (valid & FUSE_SETATTR_MTIME) {
                    uint64_t mtime_sec = read_uint64_le(req + 80);
                    uint32_t mtime_nsec = read_uint32_le(req + 100);
                    tv[1].tv_sec = (time_t)mtime_sec;
                    tv[1].tv_usec = (suseconds_t)(mtime_nsec / 1000u);
                }

                if (utimes(path, tv) != 0) {
                    out_err = -errno;
                    out_len = 16;
                    break;
                }
            }

            struct stat st;
            if (vfs_lstat(path, &st) != 0) {
                out_err = -errno;
                out_len = 16;
                break;
            }
            if (resp_cap < 16 + 16 + 88) {
                return 0;
            }
            uint32_t payload = fill_attr_out(resp + 16, &st);
            out_len = 16 + payload;
            break;
        }
        case FUSE_READLINK: {
            const char* path = inode_to_path(vfs, in.nodeid);
            if (!path) {
                out_err = -ENOENT;
                out_len = 16;
                break;
            }
            size_t cap = (resp_cap > 16) ? (resp_cap - 16) : 0;
            if (cap == 0) {
                return 0;
            }
            ssize_t n = readlink(path, (char*)(resp + 16), cap);
            if (n < 0) {
                out_err = -errno;
                out_len = 16;
                break;
            }
            out_len = 16 + (uint32_t)n;
            break;
        }
        case FUSE_SYMLINK: {
            const char* parent_path = inode_to_path(vfs, in.nodeid);
            if (!parent_path) {
                out_err = -ENOENT;
                out_len = 16;
                break;
            }
            const uint8_t* buf = req + 40;
            size_t buf_len = req_len - 40;
            const char* name = (const char*)buf;
            size_t name_len = rvvm_strnlen(name, buf_len);
            if (name_len == 0 || name_len == buf_len) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            const char* linkname = name + name_len + 1;
            size_t rem = buf_len - (name_len + 1);
            size_t link_len = rvvm_strnlen(linkname, rem);
            if (link_len == 0 || link_len == rem) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            if (!is_safe_name(name)) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            char* full = path_join2(parent_path, name);
            if (!full) {
                out_err = -ENOMEM;
                out_len = 16;
                break;
            }
            if (symlink(linkname, full) != 0) {
                out_err = -errno;
                free(full);
                out_len = 16;
                break;
            }
            struct stat st;
            if (vfs_lstat(full, &st) != 0) {
                out_err = -errno;
                free(full);
                out_len = 16;
                break;
            }
            uint64_t nodeid = ensure_inode_for_path(vfs, full);
            free(full);
            if (nodeid == 0) {
                out_err = -ENOMEM;
                out_len = 16;
                break;
            }
            if (resp_cap < 16 + 40 + 88) {
                return 0;
            }
            uint32_t payload = fill_entry_out(resp + 16, nodeid, &st);
            out_len = 16 + payload;
            break;
        }
        case FUSE_LINK: {
            if (req_len < 40 + 8) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            const char* parent_path = inode_to_path(vfs, in.nodeid);
            uint64_t oldnodeid = read_uint64_le(req + 40);
            const char* oldpath = inode_to_path(vfs, oldnodeid);
            if (!parent_path || !oldpath) {
                out_err = -ENOENT;
                out_len = 16;
                break;
            }
            const char* name = (const char*)(req + 48);
            size_t max_name = req_len > 48 ? (req_len - 48) : 0;
            size_t name_len = rvvm_strnlen(name, max_name);
            if (name_len == 0 || name_len == max_name) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            if (!is_safe_name(name)) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            char* full = path_join2(parent_path, name);
            if (!full) {
                out_err = -ENOMEM;
                out_len = 16;
                break;
            }
            if (link(oldpath, full) != 0) {
                out_err = -errno;
                free(full);
                out_len = 16;
                break;
            }
            struct stat st;
            if (vfs_lstat(full, &st) != 0) {
                out_err = -errno;
                free(full);
                out_len = 16;
                break;
            }
            uint64_t nodeid = ensure_inode_for_path(vfs, full);
            free(full);
            if (nodeid == 0) {
                out_err = -ENOMEM;
                out_len = 16;
                break;
            }
            if (resp_cap < 16 + 40 + 88) {
                return 0;
            }
            uint32_t payload = fill_entry_out(resp + 16, nodeid, &st);
            out_len = 16 + payload;
            break;
        }
        case FUSE_MKNOD: {
            if (req_len < 40 + 16) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            const char* parent_path = inode_to_path(vfs, in.nodeid);
            if (!parent_path) {
                out_err = -ENOENT;
                out_len = 16;
                break;
            }
            uint32_t mode = read_uint32_le(req + 40);
            uint32_t rdev = read_uint32_le(req + 44);
            uint32_t umask = read_uint32_le(req + 48);
            const char* name = (const char*)(req + 56);
            size_t max_name = req_len > 56 ? (req_len - 56) : 0;
            size_t name_len = rvvm_strnlen(name, max_name);
            if (name_len == 0 || name_len == max_name) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            if (!is_safe_name(name)) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            char* full = path_join2(parent_path, name);
            if (!full) {
                out_err = -ENOMEM;
                out_len = 16;
                break;
            }
            mode &= 07777u;
            mode &= ~umask;
            if (mknod(full, (mode_t)mode, (dev_t)rdev) != 0) {
                out_err = -errno;
                free(full);
                out_len = 16;
                break;
            }
            struct stat st;
            if (vfs_lstat(full, &st) != 0) {
                out_err = -errno;
                free(full);
                out_len = 16;
                break;
            }
            uint64_t nodeid = ensure_inode_for_path(vfs, full);
            free(full);
            if (nodeid == 0) {
                out_err = -ENOMEM;
                out_len = 16;
                break;
            }
            if (resp_cap < 16 + 40 + 88) {
                return 0;
            }
            uint32_t payload = fill_entry_out(resp + 16, nodeid, &st);
            out_len = 16 + payload;
            break;
        }
        case FUSE_ACCESS: {
            if (req_len < 40 + 8) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            const char* path = inode_to_path(vfs, in.nodeid);
            if (!path) {
                out_err = -ENOENT;
                out_len = 16;
                break;
            }
            uint32_t mask = read_uint32_le(req + 40);
            if (access(path, (int)mask) != 0) {
                out_err = -errno;
            }
            out_len = 16;
            break;
        }
        case FUSE_FLUSH: {
            out_len = 16;
            break;
        }
        case FUSE_FSYNC:
        case FUSE_FSYNC_DIR: {
            if (req_len < 40 + 16) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            uint64_t fh = read_uint64_le(req + 40);
            virtio_fs_handle_t* h = (virtio_fs_handle_t*)hashmap_get_ptr(&vfs->fh_to_handle, (size_t)fh);
            if (!h) {
                out_err = -EBADF;
                out_len = 16;
                break;
            }
            int rc = 0;
            if (h->is_dir) {
                int fd = h->u.dir ? dirfd(h->u.dir) : -1;
                if (fd >= 0) {
                    rc = fsync(fd);
                }
            } else {
                if (h->u.fd >= 0) {
                    rc = fsync(h->u.fd);
                }
            }
            if (rc != 0) {
                out_err = -errno;
            }
            out_len = 16;
            break;
        }
        case FUSE_SETXATTR:
        case FUSE_REMOVEXATTR:
        case FUSE_FALLOCATE:
        case FUSE_GETLK:
        case FUSE_SETLK:
        case FUSE_SETLKW:
        case FUSE_BMAP:
        case FUSE_IOCTL:
        case FUSE_POLL:
        case FUSE_NOTIFY_REPLY: {
            out_err = -EOPNOTSUPP;
            out_len = 16;
            break;
        }
        case FUSE_GETXATTR:
        case FUSE_LISTXATTR: {
            if (req_len < 40 + 8) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            uint32_t size = read_uint32_le(req + 40);
            if (size == 0) {
                if (resp_cap < 16 + 8) {
                    return 0;
                }
                memset(resp + 16, 0, 8);
                out_len = 16 + 8;
                break;
            }
            out_err = -ENODATA;
            out_len = 16;
            break;
        }
        case FUSE_BATCH_FORGET: {
            out_len = 16;
            break;
        }
        case FUSE_OPENDIR: {
            const char* path = inode_to_path(vfs, in.nodeid);
            if (!path) {
                out_err = -ENOENT;
                out_len = 16;
                break;
            }
            DIR* dir = opendir(path);
            if (!dir) {
                out_err = -errno;
                out_len = 16;
                break;
            }
            virtio_fs_handle_t* h = safe_new_obj(virtio_fs_handle_t);
            h->is_dir = 1;
            h->open_flags = 0;
            h->path = str_dup(path);
            h->u.dir = dir;
            uint64_t fh = fnv1a64(path) ^ ((uint64_t)(uintptr_t)h);
            if (fh == 0) {
                fh = 1;
            }
            hashmap_put_ptr(&vfs->fh_to_handle, (size_t)fh, h);
            virtio_fs_uart_printf("virtio-fs: opendir node=%llu fh=%llu path=%s\n",
                                  (unsigned long long)in.nodeid,
                                  (unsigned long long)fh,
                                  path);
            if (resp_cap < 16 + 24) {
                return 0;
            }
            uint32_t payload = fill_open_out(resp + 16, fh);
            out_len = 16 + payload;
            break;
        }
        case FUSE_READDIR: {
            if (req_len < 40 + 40) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            uint64_t fh = read_uint64_le(req + 40);
            uint64_t offset = read_uint64_le(req + 48);
            uint32_t size = read_uint32_le(req + 56);
            virtio_fs_handle_t* h = (virtio_fs_handle_t*)hashmap_get_ptr(&vfs->fh_to_handle, (size_t)fh);
            if (!h || !h->is_dir) {
                out_err = -EBADF;
                out_len = 16;
                break;
            }
            if (size > resp_cap - 16) {
                size = (uint32_t)(resp_cap - 16);
            }
            const char* dir_path = inode_to_path(vfs, in.nodeid);
            if (!dir_path) {
                out_err = -ENOENT;
                out_len = 16;
                break;
            }
            uint32_t written = fill_dirents(vfs, resp + 16, size, h->u.dir, in.nodeid, dir_path, offset);
            virtio_fs_uart_printf("virtio-fs: readdir node=%llu fh=%llu off=%llu size=%u -> %u\n",
                                  (unsigned long long)in.nodeid,
                                  (unsigned long long)fh,
                                  (unsigned long long)offset,
                                  (unsigned)size,
                                  (unsigned)written);
            out_len = 16 + written;
            break;
        }
        case FUSE_READDIRPLUS: {
            if (req_len < 40 + 40) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            uint64_t fh = read_uint64_le(req + 40);
            uint64_t offset = read_uint64_le(req + 48);
            uint32_t size = read_uint32_le(req + 56);
            virtio_fs_handle_t* h = (virtio_fs_handle_t*)hashmap_get_ptr(&vfs->fh_to_handle, (size_t)fh);
            if (!h || !h->is_dir) {
                out_err = -EBADF;
                out_len = 16;
                break;
            }
            if (size > resp_cap - 16) {
                size = (uint32_t)(resp_cap - 16);
            }
            const char* dir_path = inode_to_path(vfs, in.nodeid);
            if (!dir_path) {
                out_err = -ENOENT;
                out_len = 16;
                break;
            }
            uint32_t written = fill_direntsplus(vfs, resp + 16, size, h->u.dir, in.nodeid, dir_path, offset);
            virtio_fs_uart_printf("virtio-fs: readdirplus node=%llu fh=%llu off=%llu size=%u -> %u\n",
                                  (unsigned long long)in.nodeid,
                                  (unsigned long long)fh,
                                  (unsigned long long)offset,
                                  (unsigned)size,
                                  (unsigned)written);
            out_len = 16 + written;
            break;
        }
        case FUSE_OPEN: {
            const char* path = inode_to_path(vfs, in.nodeid);
            if (!path) {
                out_err = -ENOENT;
                out_len = 16;
                break;
            }
            if (req_len < 40 + 8) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            uint32_t flags = read_uint32_le(req + 40);
            int fd = open(path, (int)flags);
            if (fd < 0) {
                out_err = -errno;
                out_len = 16;
                break;
            }
            virtio_fs_handle_t* h = safe_new_obj(virtio_fs_handle_t);
            h->is_dir = 0;
            h->open_flags = flags;
            h->path = str_dup(path);
            h->u.fd = fd;
            uint64_t fh = fnv1a64(path) ^ ((uint64_t)(uintptr_t)h);
            if (fh == 0) {
                fh = 1;
            }
            hashmap_put_ptr(&vfs->fh_to_handle, (size_t)fh, h);
            if (resp_cap < 16 + 24) {
                return 0;
            }
            uint32_t payload = fill_open_out(resp + 16, fh);
            out_len = 16 + payload;
            break;
        }
        case FUSE_READ: {
            if (req_len < 40 + 40) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            uint64_t fh = read_uint64_le(req + 40);
            uint64_t offset = read_uint64_le(req + 48);
            uint32_t size = read_uint32_le(req + 56);
            virtio_fs_handle_t* h = (virtio_fs_handle_t*)hashmap_get_ptr(&vfs->fh_to_handle, (size_t)fh);
            if (!h || h->is_dir) {
                out_err = -EBADF;
                out_len = 16;
                break;
            }
            if (size > resp_cap - 16) {
                size = (uint32_t)(resp_cap - 16);
            }
            ssize_t n = pread(h->u.fd, resp + 16, size, (off_t)offset);
            if (n < 0) {
                out_err = -errno;
                out_len = 16;
                break;
            }
            out_len = 16 + (uint32_t)n;
            break;
        }
        case FUSE_WRITE: {
            if (req_len < 40 + 40) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            uint64_t fh = read_uint64_le(req + 40);
            uint64_t offset = read_uint64_le(req + 48);
            uint32_t size = read_uint32_le(req + 56);
            if (req_len < 40 + 40 + (size_t)size) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            virtio_fs_handle_t* h = (virtio_fs_handle_t*)hashmap_get_ptr(&vfs->fh_to_handle, (size_t)fh);
            if (!h || h->is_dir) {
                out_err = -EBADF;
                out_len = 16;
                break;
            }
            const uint8_t* payload = req + 80;
            ssize_t n = pwrite(h->u.fd, payload, size, (off_t)offset);
            if (n < 0) {
                out_err = -errno;
                out_len = 16;
                break;
            }
            if (resp_cap < 16 + 8) {
                return 0;
            }
            uint32_t w = fill_write_out(resp + 16, (uint32_t)n);
            out_len = 16 + w;
            break;
        }
        case FUSE_CREATE: {
            if (req_len < 40 + 16) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            const char* parent_path = inode_to_path(vfs, in.nodeid);
            if (!parent_path) {
                out_err = -ENOENT;
                out_len = 16;
                break;
            }
            uint32_t flags = read_uint32_le(req + 40);
            uint32_t mode = read_uint32_le(req + 44);
            uint32_t umask = read_uint32_le(req + 48);
            const char* name = (const char*)(req + 56);
            size_t max_name = req_len > 56 ? (req_len - 56) : 0;
            size_t name_len = rvvm_strnlen(name, max_name);
            if (name_len == 0 || name_len == max_name) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            if (!is_safe_name(name)) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            char* full = path_join2(parent_path, name);
            if (!full) {
                out_err = -ENOMEM;
                out_len = 16;
                break;
            }
            mode &= 07777u;
            mode &= ~umask;
            int fd = open(full, (int)flags | O_CREAT, (mode_t)mode);
            if (fd < 0) {
                out_err = -errno;
                free(full);
                out_len = 16;
                break;
            }
            struct stat st;
            if (vfs_lstat(full, &st) != 0) {
                out_err = -errno;
                close(fd);
                free(full);
                out_len = 16;
                break;
            }
            uint64_t nodeid = ensure_inode_for_path(vfs, full);
            free(full);
            if (nodeid == 0) {
                out_err = -ENOMEM;
                close(fd);
                out_len = 16;
                break;
            }
            virtio_fs_handle_t* h = safe_new_obj(virtio_fs_handle_t);
            h->is_dir = 0;
            h->u.fd = fd;
            uint64_t fh = fnv1a64(parent_path) ^ ((uint64_t)(uintptr_t)h) ^ nodeid;
            if (fh == 0) {
                fh = 1;
            }
            hashmap_put_ptr(&vfs->fh_to_handle, (size_t)fh, h);
            if (resp_cap < 16 + (40 + 88) + 24) {
                return 0;
            }
            uint32_t payload0 = fill_entry_out(resp + 16, nodeid, &st);
            uint32_t payload1 = fill_open_out(resp + 16 + payload0, fh);
            out_len = 16 + payload0 + payload1;
            break;
        }
        case FUSE_MKDIR: {
            if (req_len < 40 + 8) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            const char* parent_path = inode_to_path(vfs, in.nodeid);
            if (!parent_path) {
                out_err = -ENOENT;
                out_len = 16;
                break;
            }
            uint32_t mode = read_uint32_le(req + 40);
            uint32_t umask = read_uint32_le(req + 44);
            const char* name = (const char*)(req + 48);
            size_t max_name = req_len > 48 ? (req_len - 48) : 0;
            size_t name_len = rvvm_strnlen(name, max_name);
            if (name_len == 0 || name_len == max_name) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            if (!is_safe_name(name)) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            char* full = path_join2(parent_path, name);
            if (!full) {
                out_err = -ENOMEM;
                out_len = 16;
                break;
            }
            mode &= 07777u;
            mode &= ~umask;
            if (mkdir(full, (mode_t)mode) != 0) {
                out_err = -errno;
                free(full);
                out_len = 16;
                break;
            }
            struct stat st;
            if (vfs_lstat(full, &st) != 0) {
                out_err = -errno;
                free(full);
                out_len = 16;
                break;
            }
            uint64_t nodeid = ensure_inode_for_path(vfs, full);
            free(full);
            if (nodeid == 0) {
                out_err = -ENOMEM;
                out_len = 16;
                break;
            }
            if (resp_cap < 16 + 40 + 88) {
                return 0;
            }
            uint32_t payload = fill_entry_out(resp + 16, nodeid, &st);
            out_len = 16 + payload;
            break;
        }
        case FUSE_UNLINK:
        case FUSE_RMDIR: {
            const char* parent_path = inode_to_path(vfs, in.nodeid);
            if (!parent_path) {
                out_err = -ENOENT;
                out_len = 16;
                break;
            }
            const char* name = (const char*)(req + 40);
            size_t max_name = req_len > 40 ? (req_len - 40) : 0;
            size_t name_len = rvvm_strnlen(name, max_name);
            if (name_len == 0 || name_len == max_name) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            if (!is_safe_name(name)) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            char* full = path_join2(parent_path, name);
            if (!full) {
                out_err = -ENOMEM;
                out_len = 16;
                break;
            }
            int rc = (in.opcode == FUSE_RMDIR) ? rmdir(full) : unlink(full);
            if (rc != 0) {
                out_err = -errno;
            }
            free(full);
            out_len = 16;
            break;
        }
        case FUSE_RENAME: {
            if (req_len < 40 + 8) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            const char* old_parent = inode_to_path(vfs, in.nodeid);
            uint64_t newdir = read_uint64_le(req + 40);
            const char* new_parent = inode_to_path(vfs, newdir);
            if (!old_parent || !new_parent) {
                out_err = -ENOENT;
                out_len = 16;
                break;
            }
            const uint8_t* names = req + 48;
            size_t names_len = req_len - 48;
            const char* oldname = (const char*)names;
            size_t old_len = rvvm_strnlen(oldname, names_len);
            if (old_len == 0 || old_len == names_len) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            const char* newname = oldname + old_len + 1;
            size_t rem = names_len - (old_len + 1);
            size_t new_len = rvvm_strnlen(newname, rem);
            if (new_len == 0 || new_len == rem) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            if (!is_safe_name(oldname) || !is_safe_name(newname)) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            char* oldfull = path_join2(old_parent, oldname);
            char* newfull = path_join2(new_parent, newname);
            if (!oldfull || !newfull) {
                free(oldfull);
                free(newfull);
                out_err = -ENOMEM;
                out_len = 16;
                break;
            }
            if (rename(oldfull, newfull) != 0) {
                out_err = -errno;
            }
            free(oldfull);
            free(newfull);
            out_len = 16;
            break;
        }
        case FUSE_RELEASE:
        case FUSE_RELEASEDIR: {
            if (req_len < 40 + 24) {
                out_err = -EINVAL;
                out_len = 16;
                break;
            }
            uint64_t fh = read_uint64_le(req + 40);
            virtio_fs_handle_t* h = (virtio_fs_handle_t*)hashmap_get_ptr(&vfs->fh_to_handle, (size_t)fh);
            if (h) {
                if (h->is_dir) {
                    if (h->u.dir) {
                        closedir(h->u.dir);
                    }
                } else {
                    if (h->u.fd >= 0) {
                        close(h->u.fd);
                    }
                }
                hashmap_erase(&vfs->fh_to_handle, (size_t)fh);
                free(h->path);
                free(h);
            }
            out_len = 16;
            break;
        }
        case FUSE_STATFS: {
            const char* path = inode_to_path(vfs, in.nodeid);
            if (!path) {
                out_err = -ENOENT;
                out_len = 16;
                break;
            }
            struct statvfs sv;
            if (statvfs(path, &sv) != 0) {
                out_err = -errno;
                out_len = 16;
                break;
            }
            if (resp_cap < 16 + 80) {
                return 0;
            }
            memset(resp + 16, 0, 80);
            write_uint64_le(resp + 16 + 0, (uint64_t)sv.f_blocks);
            write_uint64_le(resp + 16 + 8, (uint64_t)sv.f_bfree);
            write_uint64_le(resp + 16 + 16, (uint64_t)sv.f_bavail);
            write_uint64_le(resp + 16 + 24, (uint64_t)sv.f_files);
            write_uint64_le(resp + 16 + 32, (uint64_t)sv.f_ffree);
            uint32_t bsize = sv.f_bsize ? (uint32_t)sv.f_bsize : 512u;
            uint32_t frsize = sv.f_frsize ? (uint32_t)sv.f_frsize : bsize;
            write_uint32_le(resp + 16 + 40, bsize);
            write_uint32_le(resp + 16 + 44, (uint32_t)sv.f_namemax);
            write_uint32_le(resp + 16 + 48, frsize);
            write_uint32_le(resp + 16 + 52, 0);
            out_len = 16 + 80;
            break;
        }
        case FUSE_FORGET:
        case FUSE_DESTROY:
        case FUSE_INTERRUPT: {
            out_len = 16;
            break;
        }
        default: {
            out_err = -ENOSYS;
            out_len = 16;
            virtio_fs_uart_printf("virtio-fs: unhandled opcode=%u(%s)\n",
                                  (unsigned)in.opcode,
                                  fuse_opcode_name(in.opcode));
            break;
        }
    }

    if (out_len > resp_cap) {
        out_len = (uint32_t)resp_cap;
    }
    virtio_fs_uart_printf("virtio-fs: resp uniq=%llu op=%u(%s) err=%d len=%u\n",
                          (unsigned long long)in.unique,
                          (unsigned)in.opcode,
                          fuse_opcode_name(in.opcode),
                          (int)out_err,
                          (unsigned)out_len);
    write_out_header(resp, out_len, out_err, in.unique);
    return out_len;
}

static void virtio_fs_clear_inode_map(virtio_fs_dev_t* vfs)
{
    if (!vfs) {
        return;
    }
    hashmap_foreach(&vfs->inode_to_path, k, v) {
        char* p = (char*)v;
        free(p);
    }
    hashmap_free(&vfs->inode_to_path);
    hashmap_init(&vfs->inode_to_path);
}

static void virtio_fs_clear_handles(virtio_fs_dev_t* vfs)
{
    if (!vfs) {
        return;
    }
    hashmap_foreach(&vfs->fh_to_handle, k, v) {
        virtio_fs_handle_t* h = (virtio_fs_handle_t*)v;
        if (h) {
            if (h->is_dir) {
                if (h->u.dir) {
                    closedir(h->u.dir);
                }
            } else {
                if (h->u.fd >= 0) {
                    close(h->u.fd);
                }
            }
            free(h->path);
            free(h);
        }
    }
    hashmap_free(&vfs->fh_to_handle);
    hashmap_init(&vfs->fh_to_handle);
}

static bool rvvm_state_write_str(rvvm_state_t* state, const char* s)
{
    uint32_t len = s ? (uint32_t)strlen(s) : 0;
    if (!rvvm_state_write_u32(state, len)) {
        return false;
    }
    if (len == 0) {
        return true;
    }
    return rvvm_state_write(state, s, len);
}

static bool rvvm_state_read_str(rvvm_state_t* state, char** out)
{
    if (!out) {
        return false;
    }
    uint32_t len = 0;
    if (!rvvm_state_read_u32(state, &len) || len > (uint32_t)(1024u * 1024u)) {
        return false;
    }
    if (len == 0) {
        *out = NULL;
        return true;
    }
    char* s = safe_malloc((size_t)len + 1);
    if (!rvvm_state_read(state, s, len)) {
        free(s);
        return false;
    }
    s[len] = 0;
    *out = s;
    return true;
}

static void virtio_fs_suspend(rvvm_mmio_dev_t* dev, rvvm_state_t* state)
{
    virtio_fs_dev_t* vfs = dev ? dev->data : NULL;
    if (!vfs || !state) {
        return;
    }

    rvvm_state_write_u32(state, 1);

    rvvm_state_write_u32(state, vfs->num_request_queues);
    rvvm_state_write_u32(state, vfs->notify_buf_size);
    rvvm_state_write_u32(state, vfs->config_generation);

    rvvm_state_write_u32(state, vfs->host_features_sel);
    rvvm_state_write_u32(state, vfs->driver_features_sel);
    rvvm_state_write_u32(state, vfs->host_features[0]);
    rvvm_state_write_u32(state, vfs->host_features[1]);
    rvvm_state_write_u32(state, vfs->driver_features[0]);
    rvvm_state_write_u32(state, vfs->driver_features[1]);

    rvvm_state_write_u32(state, vfs->status);
    rvvm_state_write_u32(state, vfs->interrupt_status);
    rvvm_state_write_u32(state, vfs->queue_sel);

    for (size_t i = 0; i < STATIC_ARRAY_SIZE(vfs->q); i++) {
        virtio_queue_t* q = &vfs->q[i];
        rvvm_state_write_u32(state, (uint32_t)q->num);
        rvvm_state_write_u32(state, (uint32_t)q->ready);
        rvvm_state_write_u32(state, (uint32_t)q->last_avail_idx);
        rvvm_state_write_u32(state, (uint32_t)q->used_idx);
        rvvm_state_write_u64(state, q->desc_addr);
        rvvm_state_write_u64(state, q->avail_addr);
        rvvm_state_write_u64(state, q->used_addr);
    }

    rvvm_state_write_str(state, vfs->root_path);
    rvvm_state_write(state, vfs->tag, sizeof(vfs->tag));

    uint32_t inode_count = (uint32_t)hashmap_size(&vfs->inode_to_path);
    rvvm_state_write_u32(state, inode_count);
    hashmap_foreach(&vfs->inode_to_path, k, v) {
        rvvm_state_write_u64(state, (uint64_t)k);
        rvvm_state_write_str(state, (const char*)v);
    }

    uint32_t handle_count = (uint32_t)hashmap_size(&vfs->fh_to_handle);
    rvvm_state_write_u32(state, handle_count);
    hashmap_foreach(&vfs->fh_to_handle, k, v) {
        virtio_fs_handle_t* h = (virtio_fs_handle_t*)v;
        rvvm_state_write_u64(state, (uint64_t)k);
        rvvm_state_write_u32(state, h ? (uint32_t)h->is_dir : 0);
        rvvm_state_write_u32(state, h ? h->open_flags : 0);
        rvvm_state_write_str(state, h ? h->path : NULL);
    }
}

static void virtio_fs_resume(rvvm_mmio_dev_t* dev, rvvm_state_t* state)
{
    virtio_fs_dev_t* vfs = dev ? dev->data : NULL;
    if (!vfs || !state) {
        return;
    }

    uint32_t version = 0;
    if (!rvvm_state_read_u32(state, &version) || version != 1) {
        rvvm_state_fail(state);
        return;
    }

    virtio_fs_clear_handles(vfs);
    virtio_fs_clear_inode_map(vfs);

    if (!rvvm_state_read_u32(state, &vfs->num_request_queues) || !rvvm_state_read_u32(state, &vfs->notify_buf_size) ||
        !rvvm_state_read_u32(state, &vfs->config_generation) || !rvvm_state_read_u32(state, &vfs->host_features_sel) ||
        !rvvm_state_read_u32(state, &vfs->driver_features_sel) || !rvvm_state_read_u32(state, &vfs->host_features[0]) ||
        !rvvm_state_read_u32(state, &vfs->host_features[1]) || !rvvm_state_read_u32(state, &vfs->driver_features[0]) ||
        !rvvm_state_read_u32(state, &vfs->driver_features[1]) || !rvvm_state_read_u32(state, &vfs->status) ||
        !rvvm_state_read_u32(state, &vfs->interrupt_status) || !rvvm_state_read_u32(state, &vfs->queue_sel)) {
        rvvm_state_fail(state);
        return;
    }

    for (size_t i = 0; i < STATIC_ARRAY_SIZE(vfs->q); i++) {
        virtio_queue_t* q = &vfs->q[i];
        uint32_t num = 0;
        uint32_t ready = 0;
        uint32_t last_avail_idx = 0;
        uint32_t used_idx = 0;
        uint64_t desc_addr = 0;
        uint64_t avail_addr = 0;
        uint64_t used_addr = 0;
        if (!rvvm_state_read_u32(state, &num) || !rvvm_state_read_u32(state, &ready) || !rvvm_state_read_u32(state, &last_avail_idx) ||
            !rvvm_state_read_u32(state, &used_idx) || !rvvm_state_read_u64(state, &desc_addr) || !rvvm_state_read_u64(state, &avail_addr) ||
            !rvvm_state_read_u64(state, &used_addr)) {
            rvvm_state_fail(state);
            return;
        }
        q->num = (uint16_t)num;
        q->ready = (uint16_t)ready;
        q->last_avail_idx = (uint16_t)last_avail_idx;
        q->used_idx = (uint16_t)used_idx;
        q->desc_addr = desc_addr;
        q->avail_addr = avail_addr;
        q->used_addr = used_addr;
    }

    char* saved_root = NULL;
    if (!rvvm_state_read_str(state, &saved_root) || !rvvm_state_read(state, vfs->tag, sizeof(vfs->tag))) {
        free(saved_root);
        rvvm_state_fail(state);
        return;
    }
    if (saved_root && vfs->root_path && strcmp(saved_root, vfs->root_path) != 0) {
        free(saved_root);
        rvvm_state_fail(state);
        return;
    }
    free(saved_root);

    uint32_t inode_count = 0;
    if (!rvvm_state_read_u32(state, &inode_count) || inode_count > (uint32_t)(1024u * 1024u)) {
        rvvm_state_fail(state);
        return;
    }
    for (uint32_t i = 0; i < inode_count; i++) {
        uint64_t nodeid = 0;
        char* path = NULL;
        if (!rvvm_state_read_u64(state, &nodeid) || !rvvm_state_read_str(state, &path)) {
            free(path);
            rvvm_state_fail(state);
            return;
        }
        if (path) {
            hashmap_put_ptr(&vfs->inode_to_path, (size_t)nodeid, path);
        }
    }

    uint32_t handle_count = 0;
    if (!rvvm_state_read_u32(state, &handle_count) || handle_count > (uint32_t)(1024u * 1024u)) {
        rvvm_state_fail(state);
        return;
    }
    for (uint32_t i = 0; i < handle_count; i++) {
        uint64_t fh = 0;
        uint32_t is_dir = 0;
        uint32_t open_flags = 0;
        char* path = NULL;
        if (!rvvm_state_read_u64(state, &fh) || !rvvm_state_read_u32(state, &is_dir) || !rvvm_state_read_u32(state, &open_flags) ||
            !rvvm_state_read_str(state, &path) || fh == 0 || !path) {
            free(path);
            rvvm_state_fail(state);
            return;
        }

        virtio_fs_handle_t* h = safe_new_obj(virtio_fs_handle_t);
        h->is_dir = (uint8_t)(is_dir ? 1 : 0);
        h->open_flags = open_flags;
        h->path = path;
        if (h->is_dir) {
            h->u.dir = opendir(path);
            if (!h->u.dir) {
                free(h->path);
                free(h);
                rvvm_state_fail(state);
                return;
            }
        } else {
            h->u.fd = open(path, (int)open_flags);
            if (h->u.fd < 0) {
                free(h->path);
                free(h);
                rvvm_state_fail(state);
                return;
            }
        }
        hashmap_put_ptr(&vfs->fh_to_handle, (size_t)fh, h);
    }

    if (vfs->interrupt_status) {
        rvvm_raise_irq(vfs->intc, vfs->irq);
    } else {
        rvvm_lower_irq(vfs->intc, vfs->irq);
    }
}

static void virtio_fs_process_queue(virtio_fs_dev_t* vfs, virtio_queue_t* q)
{
    if (!vfs || !q || !q->ready || q->num == 0) {
        return;
    }
    if (!vfs->req_buf || !vfs->resp_buf || vfs->buf_cap == 0) {
        return;
    }
    uint8_t* avail_p = dma_ptr(vfs, q->avail_addr, 4 + (size_t)q->num * 2);
    if (!avail_p) {
        return;
    }
    uint16_t avail_idx = read_uint16_le(avail_p + 2);
    while (q->last_avail_idx != avail_idx) {
        uint16_t ring_slot = (uint16_t)(q->last_avail_idx % q->num);
        uint16_t head = read_uint16_le(avail_p + 4 + (size_t)ring_slot * 2);
        q->last_avail_idx++;

        virtq_chain_t chain;
        if (!virtq_collect_chain(vfs, q, head, &chain)) {
            virtq_push_used(vfs, q, head, 0);
            continue;
        }

        size_t req_len = chain_read_all(vfs, &chain, vfs->req_buf, vfs->buf_cap);
        if (req_len == 0) {
            virtq_push_used(vfs, q, head, 0);
            continue;
        }
        uint32_t resp_len = fuse_handle_request(vfs, vfs->req_buf, req_len, vfs->resp_buf, vfs->buf_cap);
        if (resp_len == 0) {
            virtq_push_used(vfs, q, head, 0);
            continue;
        }
        size_t w = chain_write_all(vfs, &chain, vfs->resp_buf, resp_len);
        if (w == 0) {
            virtq_push_used(vfs, q, head, 0);
            continue;
        }
        virtq_push_used(vfs, q, head, (uint32_t)w);
    }
}

static void virtio_fs_reset(virtio_fs_dev_t* vfs)
{
    if (!vfs) {
        return;
    }
    vfs->status = 0;
    vfs->interrupt_status = 0;
    vfs->queue_sel = 0;
    vfs->host_features_sel = 0;
    vfs->driver_features_sel = 0;
    vfs->driver_features[0] = 0;
    vfs->driver_features[1] = 0;
    for (size_t i = 0; i < STATIC_ARRAY_SIZE(vfs->q); ++i) {
        vfs->q[i].num = 0;
        vfs->q[i].ready = 0;
        vfs->q[i].last_avail_idx = 0;
        vfs->q[i].used_idx = 0;
        vfs->q[i].desc_addr = 0;
        vfs->q[i].avail_addr = 0;
        vfs->q[i].used_addr = 0;
    }
    vfs->config_generation++;
    rvvm_lower_irq(vfs->intc, vfs->irq);
}

static bool virtio_fs_mmio_read(rvvm_mmio_dev_t* dev, void* data, size_t offset, uint8_t size)
{
    virtio_fs_dev_t* vfs = dev->data;
    uint32_t val = 0;

    if (!vfs) {
        mmio_store_u32(data, 0, size);
        return true;
    }

    if (offset >= VIRTIO_MMIO_CONFIG) {
        uint32_t off = (uint32_t)(offset - VIRTIO_MMIO_CONFIG);
        uint8_t cfg[44];
        memset(cfg, 0, sizeof(cfg));
        memcpy(cfg + 0, vfs->tag, 36);
        write_uint32_le(cfg + 36, vfs->num_request_queues);
        write_uint32_le(cfg + 40, vfs->notify_buf_size);
        if (off + size <= sizeof(cfg)) {
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
            val = VIRTIO_ID_FS;
            break;
        case VIRTIO_MMIO_VENDOR_ID:
            val = VIRTIO_MMIO_VENDOR_VALUE;
            break;
        case VIRTIO_MMIO_DEVICE_FEATURES:
            val = vfs->host_features[vfs->host_features_sel & 1];
            break;
        case VIRTIO_MMIO_DEVICE_FEATURES_SEL:
            val = vfs->host_features_sel;
            break;
        case VIRTIO_MMIO_DRIVER_FEATURES:
            val = vfs->driver_features[vfs->driver_features_sel & 1];
            break;
        case VIRTIO_MMIO_DRIVER_FEATURES_SEL:
            val = vfs->driver_features_sel;
            break;
        case VIRTIO_MMIO_QUEUE_SEL:
            val = vfs->queue_sel;
            break;
        case VIRTIO_MMIO_QUEUE_NUM_MAX:
            val = (vfs->queue_sel < STATIC_ARRAY_SIZE(vfs->q)) ? 1024 : 0;
            break;
        case VIRTIO_MMIO_QUEUE_NUM:
            val = (vfs->queue_sel < STATIC_ARRAY_SIZE(vfs->q)) ? vfs->q[vfs->queue_sel].num : 0;
            break;
        case VIRTIO_MMIO_QUEUE_READY:
            val = (vfs->queue_sel < STATIC_ARRAY_SIZE(vfs->q)) ? vfs->q[vfs->queue_sel].ready : 0;
            break;
        case VIRTIO_MMIO_INTERRUPT_STATUS:
            val = vfs->interrupt_status;
            break;
        case VIRTIO_MMIO_STATUS:
            val = vfs->status;
            break;
        case VIRTIO_MMIO_QUEUE_DESC_LOW:
            if (vfs->queue_sel < STATIC_ARRAY_SIZE(vfs->q)) {
                val = (uint32_t)(vfs->q[vfs->queue_sel].desc_addr);
            }
            break;
        case VIRTIO_MMIO_QUEUE_DESC_HIGH:
            if (vfs->queue_sel < STATIC_ARRAY_SIZE(vfs->q)) {
                val = (uint32_t)(vfs->q[vfs->queue_sel].desc_addr >> 32);
            }
            break;
        case VIRTIO_MMIO_QUEUE_DRIVER_LOW:
            if (vfs->queue_sel < STATIC_ARRAY_SIZE(vfs->q)) {
                val = (uint32_t)(vfs->q[vfs->queue_sel].avail_addr);
            }
            break;
        case VIRTIO_MMIO_QUEUE_DRIVER_HIGH:
            if (vfs->queue_sel < STATIC_ARRAY_SIZE(vfs->q)) {
                val = (uint32_t)(vfs->q[vfs->queue_sel].avail_addr >> 32);
            }
            break;
        case VIRTIO_MMIO_QUEUE_DEVICE_LOW:
            if (vfs->queue_sel < STATIC_ARRAY_SIZE(vfs->q)) {
                val = (uint32_t)(vfs->q[vfs->queue_sel].used_addr);
            }
            break;
        case VIRTIO_MMIO_QUEUE_DEVICE_HIGH:
            if (vfs->queue_sel < STATIC_ARRAY_SIZE(vfs->q)) {
                val = (uint32_t)(vfs->q[vfs->queue_sel].used_addr >> 32);
            }
            break;
        case VIRTIO_MMIO_CONFIG_GENERATION:
            val = vfs->config_generation;
            break;
    }

    write_uint32_le(data, val);
    return true;
}

static bool virtio_fs_mmio_write(rvvm_mmio_dev_t* dev, void* data, size_t offset, uint8_t size)
{
    virtio_fs_dev_t* vfs = dev->data;
    uint32_t val = mmio_load_u32(data, size);

    if (!vfs) {
        return true;
    }

    if (offset >= VIRTIO_MMIO_CONFIG) {
        return true;
    }

    if (size != 4) {
        return true;
    }

    switch (offset) {
        case VIRTIO_MMIO_DEVICE_FEATURES_SEL:
            vfs->host_features_sel = val;
            break;
        case VIRTIO_MMIO_DRIVER_FEATURES:
            vfs->driver_features[vfs->driver_features_sel & 1] = val;
            break;
        case VIRTIO_MMIO_DRIVER_FEATURES_SEL:
            vfs->driver_features_sel = val;
            break;
        case VIRTIO_MMIO_QUEUE_SEL:
            vfs->queue_sel = val;
            break;
        case VIRTIO_MMIO_QUEUE_NUM:
            if (vfs->queue_sel < STATIC_ARRAY_SIZE(vfs->q)) {
                if (val > 1024) {
                    val = 1024;
                }
                vfs->q[vfs->queue_sel].num = (uint16_t)val;
            }
            break;
        case VIRTIO_MMIO_QUEUE_READY:
            if (vfs->queue_sel < STATIC_ARRAY_SIZE(vfs->q)) {
                vfs->q[vfs->queue_sel].ready = (val & 1) ? 1 : 0;
                vfs->q[vfs->queue_sel].last_avail_idx = 0;
                vfs->q[vfs->queue_sel].used_idx = 0;
            }
            break;
        case VIRTIO_MMIO_QUEUE_NOTIFY:
            if (val < STATIC_ARRAY_SIZE(vfs->q)) {
                virtio_fs_process_queue(vfs, &vfs->q[val]);
            }
            break;
        case VIRTIO_MMIO_INTERRUPT_ACK:
            vfs->interrupt_status &= ~val;
            if (!vfs->interrupt_status) {
                rvvm_lower_irq(vfs->intc, vfs->irq);
            }
            break;
        case VIRTIO_MMIO_STATUS: {
            if ((val & 0xFF) == 0) {
                virtio_fs_reset(vfs);
                break;
            }
            vfs->status = val & 0xFF;
            if (vfs->status & 0x08) {
                uint32_t bad0 = vfs->driver_features[0] & ~vfs->host_features[0];
                uint32_t bad1 = vfs->driver_features[1] & ~vfs->host_features[1];
                if (bad0 || bad1) {
                    vfs->status &= ~0x08;
                }
            }
            break;
        }
        case VIRTIO_MMIO_QUEUE_DESC_LOW:
            if (vfs->queue_sel < STATIC_ARRAY_SIZE(vfs->q)) {
                uint64_t cur = vfs->q[vfs->queue_sel].desc_addr;
                cur = (cur & 0xFFFFFFFF00000000ULL) | (uint64_t)val;
                vfs->q[vfs->queue_sel].desc_addr = cur;
            }
            break;
        case VIRTIO_MMIO_QUEUE_DESC_HIGH:
            if (vfs->queue_sel < STATIC_ARRAY_SIZE(vfs->q)) {
                uint64_t cur = vfs->q[vfs->queue_sel].desc_addr;
                cur = (cur & 0x00000000FFFFFFFFULL) | ((uint64_t)val << 32);
                vfs->q[vfs->queue_sel].desc_addr = cur;
            }
            break;
        case VIRTIO_MMIO_QUEUE_DRIVER_LOW:
            if (vfs->queue_sel < STATIC_ARRAY_SIZE(vfs->q)) {
                uint64_t cur = vfs->q[vfs->queue_sel].avail_addr;
                cur = (cur & 0xFFFFFFFF00000000ULL) | (uint64_t)val;
                vfs->q[vfs->queue_sel].avail_addr = cur;
            }
            break;
        case VIRTIO_MMIO_QUEUE_DRIVER_HIGH:
            if (vfs->queue_sel < STATIC_ARRAY_SIZE(vfs->q)) {
                uint64_t cur = vfs->q[vfs->queue_sel].avail_addr;
                cur = (cur & 0x00000000FFFFFFFFULL) | ((uint64_t)val << 32);
                vfs->q[vfs->queue_sel].avail_addr = cur;
            }
            break;
        case VIRTIO_MMIO_QUEUE_DEVICE_LOW:
            if (vfs->queue_sel < STATIC_ARRAY_SIZE(vfs->q)) {
                uint64_t cur = vfs->q[vfs->queue_sel].used_addr;
                cur = (cur & 0xFFFFFFFF00000000ULL) | (uint64_t)val;
                vfs->q[vfs->queue_sel].used_addr = cur;
            }
            break;
        case VIRTIO_MMIO_QUEUE_DEVICE_HIGH:
            if (vfs->queue_sel < STATIC_ARRAY_SIZE(vfs->q)) {
                uint64_t cur = vfs->q[vfs->queue_sel].used_addr;
                cur = (cur & 0x00000000FFFFFFFFULL) | ((uint64_t)val << 32);
                vfs->q[vfs->queue_sel].used_addr = cur;
            }
            break;
    }
    return true;
}

static void virtio_fs_remove(rvvm_mmio_dev_t* dev)
{
    virtio_fs_dev_t* vfs = dev->data;
    if (!vfs) {
        return;
    }
    hashmap_foreach(&vfs->fh_to_handle, k, v) {
        virtio_fs_handle_t* h = (virtio_fs_handle_t*)v;
        if (h) {
            if (h->is_dir) {
                if (h->u.dir) {
                    closedir(h->u.dir);
                }
            } else {
                if (h->u.fd >= 0) {
                    close(h->u.fd);
                }
            }
            free(h->path);
            free(h);
        }
    }
    hashmap_free(&vfs->fh_to_handle);
    hashmap_foreach(&vfs->inode_to_path, k, v) {
        char* p = (char*)v;
        free(p);
    }
    hashmap_free(&vfs->inode_to_path);
    free(vfs->root_path);
    free(vfs->req_buf);
    free(vfs->resp_buf);
    free(vfs);
}

static rvvm_mmio_type_t virtio_fs_dev_type = {
    .name = "virtio_fs_mmio",
    .remove = virtio_fs_remove,
    .suspend = virtio_fs_suspend,
    .resume = virtio_fs_resume,
};

PUBLIC rvvm_mmio_dev_t* virtio_fs_init(rvvm_machine_t* machine, rvvm_addr_t addr, rvvm_intc_t* intc, rvvm_irq_t irq,
                                       const char* tag, const char* shared_dir)
{
    if (!machine || !intc || !shared_dir) {
        return NULL;
    }
    struct stat st;
    if (stat(shared_dir, &st) != 0 || !S_ISDIR(st.st_mode)) {
        return NULL;
    }
    virtio_fs_hostfs_active = true;

    virtio_fs_dev_t* vfs = safe_new_obj(virtio_fs_dev_t);
    vfs->machine = machine;
    vfs->intc = intc;
    vfs->irq = irq;
    vfs->num_request_queues = 1;
    vfs->notify_buf_size = 0;
    vfs->root_path = str_dup(shared_dir);
    hashmap_init(&vfs->inode_to_path);
    hashmap_init(&vfs->fh_to_handle);
    vfs->buf_cap = 256u * 1024u;
    vfs->req_buf = safe_malloc(vfs->buf_cap);
    vfs->resp_buf = safe_malloc(vfs->buf_cap);

    memset(vfs->tag, 0, sizeof(vfs->tag));
    const char* use_tag = (tag && *tag) ? tag : "share";
    strncpy(vfs->tag, use_tag, sizeof(vfs->tag) - 1);

    vfs->host_features[0] = 0;
    vfs->host_features[1] = 0;
    vfs->host_features[VIRTIO_RING_F_INDIRECT_DESC / 32] |= 1U << (VIRTIO_RING_F_INDIRECT_DESC % 32);
    vfs->host_features[VIRTIO_F_VERSION_1 / 32] |= 1U << (VIRTIO_F_VERSION_1 % 32);
    vfs->host_features[VIRTIO_F_ACCESS_PLATFORM / 32] |= 1U << (VIRTIO_F_ACCESS_PLATFORM % 32);

    virtio_fs_reset(vfs);

    rvvm_mmio_dev_t mmio_desc = {
        .addr = addr,
        .size = 0x1000,
        .data = vfs,
        .type = &virtio_fs_dev_type,
        .read = virtio_fs_mmio_read,
        .write = virtio_fs_mmio_write,
        .min_op_size = 1,
        .max_op_size = 4,
    };

    rvvm_mmio_dev_t* mmio = rvvm_attach_mmio(machine, &mmio_desc);
    if (!mmio) {
        virtio_fs_remove(&mmio_desc);
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

PUBLIC rvvm_mmio_dev_t* virtio_fs_init_auto(rvvm_machine_t* machine, const char* tag, const char* shared_dir)
{
    rvvm_intc_t* intc = rvvm_get_intc(machine);
    rvvm_addr_t addr = rvvm_mmio_zone_auto(machine, VIRTIO_FS_MMIO_ADDR_DEFAULT, 0x1000);
    return virtio_fs_init(machine, addr, intc, rvvm_alloc_irq(intc), tag, shared_dir);
}
