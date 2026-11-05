/* epd-wl.c — see epd-wl.h. The backend sends each buffer's fd once (BUF),
 * then per frame the buffer id and its damage (FRAME); we map every buffer
 * read-only and answer DONE (epd_wl_done) when the frame's pixels are no
 * longer needed. One backend at a time: a new connection replaces the old
 * (sway restarted), and it resends its buffers and a whole frame.
 *
 * The socket is root:<panel group> 0660 (epd-group.h: video unless
 * $GLASSINE_EPD_GROUP) and each connection's peer credentials are
 * checked at accept (root, the daemon's own user, or a member of that group):
 * a refused peer never replaces the connected backend. A buffer's fd is
 * mapped only if it can't shrink under the mapping (a memfd sealed with
 * F_SEAL_SHRINK, which we add when the sender left it open, or a dma-buf) and
 * is as large as the buffer: a short or shrunk file would raise SIGBUS in the
 * root daemon on the first read. */
#define _GNU_SOURCE
#include "epd-wl.h"
#include "epd-group.h"
#include "glassine-epd-output-proto.h"
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <grp.h>
#include <pwd.h>
#include <linux/dma-buf.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <sys/vfs.h>
#include <time.h>
#include <unistd.h>

#define MAXBUF 16
#define XRGB8888 0x34325258u                /* DRM fourccs: the alpha is ignored */
#define ARGB8888 0x34325241u
#define R8 0x20203852u                      /* grey, panel order */
#ifndef DMA_BUF_MAGIC
#define DMA_BUF_MAGIC 0x444d4142            /* statfs f_type of a dma-buf's file (linux/magic.h) */
#endif

struct epd_wl {
    int ls, fd;                             /* listening, connected (-1: none) */
    uid_t uid;                              /* the user allowed besides root and the group (-1: none) */
    gid_t gid;                              /* the group allowed (-1: none) */
    int w, h;
    epd_wl_rect_fn fn;
    void *ctx;
    epd_wl_grey_fn gfn;
    struct {
        uint32_t id, w, h, stride, format, offset;
        uint8_t *map; size_t len;           /* shm: mapped at BUF; dma-buf: when the CPU needs it */
        int live;                           /* a slot in use */
        int grey;                           /* R8, h x w: panel order */
        epd_wl_dmabuf dma;                  /* dma.fd >= 0: a dma-buf (kept open) */
    } buf[MAXBUF];
    const epd_wl_dmabuf_ops *ops;
    void *ops_ctx;
    unsigned long dma_gpu, dma_cpu;
    int pend;                               /* a FRAME not yet answered */
    uint32_t pend_id, pend_seq;
    unsigned long frames;
    unsigned long long px;
    double done_t;                          /* when the last DONE went out (0: none since) */
    unsigned long ch_n;
    double ch_sum, ch_max;
    double ring[EPD_WL_CHAIN_N];            /* the last DONE -> FRAME times (ms) */
    int ring_n, ring_i;
    uint32_t caps;                          /* the backend's HELLO (GLASSINE_EPD_CAP_*) */
    struct {                                /* the gamma tables (GLASSINE_EPD_CAP_GAMMA) */
        uint8_t luma[256], lum[256];
        int set;
        uint32_t sent;                      /* the tag sent on this connection (0: none) */
        int pre;                            /* converting a frame that went through it */
        unsigned long stale, pre_n;         /* (dropped; read through the backend's table) */
    } lut;
    struct {                                /* the dead band (GLASSINE_EPD_CAP_BAND) */
        int t, quiet;
        int sent_t, sent_quiet;             /* what the backend knows (a connection starts at T 0) */
        epd_wl_damage_fn fn;
        void *ctx;
    } band;
    epd_wl_damage_fn dmg;
    void *dmg_ctx;
    struct {                                /* the newest frame, while its convert waits (dmg set) */
        int on, idx, nexted;                /* nexted: NEXT went out, DONE still owed */
        uint32_t id, seq;
        epd_wl_rect r[GLASSINE_EPD_MAXRECT];
        int n;
        uint32_t tag;                       /* its FRAME's gamma tag */
    } held;
    unsigned long skipped;
    struct {                                /* the frame the daemon reads in place (ops->frame 1): its DONE waits */
        int on, idx;                        /* (for the next frame converted, or epd_wl_unkeep) */
        uint32_t id, seq;
    } keep;
    struct {                                /* the pointer (CURSOR_IMG / CURSOR_POS) */
        uint8_t *map;                       /* the backend's memfd (NULL: none yet) */
        int w, h, hx, hy, panel;            /* the newest IMG: buffer size, hotspot, layout; w 0: none */
        uint32_t offset, stride;
        int x, y;                           /* the hotspot (buffer coordinates) */
        int img_new, pos_new;               /* since the last callback */
        uint32_t px[GLASSINE_EPD_CUR_MAX * GLASSINE_EPD_CUR_MAX];   /* the sprite, portrait */
        epd_wl_cursor_fn fn;
        void *ctx;
    } cur;
};

static double mono(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec / 1e9;
}

static void drop_one(epd_wl *s, int i)
{
    if (s->keep.on && s->keep.idx == i) s->keep.on = 0;     /* (its buffer is gone: no DONE owed) */
    if (s->buf[i].map) munmap(s->buf[i].map, s->buf[i].len);
    s->buf[i].map = NULL;
    if (s->buf[i].dma.fd >= 0) {
        if (s->buf[i].dma.handle >= 0 && s->ops) s->ops->drop(s->ops_ctx, &s->buf[i].dma);
        close(s->buf[i].dma.fd);
    }
    s->buf[i].dma.fd = -1;
    s->buf[i].dma.handle = -1;
    s->buf[i].live = 0;
}

static void drop_all(epd_wl *s)
{
    for (int i = 0; i < MAXBUF; i++) if (s->buf[i].live) drop_one(s, i);
    s->pend = 0;
    s->held.on = 0;
    s->caps = 0;
    s->lut.sent = 0;
    s->band.sent_t = s->band.sent_quiet = 0;
    if (s->cur.map) munmap(s->cur.map, 2 * GLASSINE_EPD_CUR_SLOT);
    s->cur.map = NULL;
    if (s->cur.w && s->cur.fn) s->cur.fn(s->cur.ctx, &(epd_wl_cursor){ 0 });  /* (the backend's gone: so is its pointer) */
    s->cur.w = 0;
    s->cur.img_new = s->cur.pos_new = 0;
}

static void disconnect(epd_wl *s)
{
    if (s->fd >= 0) { close(s->fd); s->fd = -1; fprintf(stderr, "glassine-epdd-desktop: wl: backend gone\n"); }
    drop_all(s);
}

epd_wl *epd_wl_open(const char *path, int w, int h, epd_wl_rect_fn fn, void *ctx)
{
    epd_wl *s = calloc(1, sizeof *s);
    if (!s) return NULL;
    s->fd = -1; s->w = w; s->h = h; s->fn = fn; s->ctx = ctx;
    s->uid = geteuid();
    s->gid = epd_group((gid_t)-1);          /* the panel group (video): none: root and our own user only */
    for (int i = 0; i < MAXBUF; i++) { s->buf[i].dma.fd = -1; s->buf[i].dma.handle = -1; }
    s->ls = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
    struct sockaddr_un sa = { .sun_family = AF_UNIX };
    if (!path) path = getenv("GLASSINE_EPD_SOCK");    /* as the backend: host tests */
    snprintf(sa.sun_path, sizeof sa.sun_path, "%s", path ? path : GLASSINE_EPD_SOCK);
    unlink(sa.sun_path);
    mode_t um = umask(0177);                /* never open wider than 0600 until the group is set */
    int bad = s->ls < 0 || bind(s->ls, (struct sockaddr *)&sa, sizeof sa) || listen(s->ls, 1);
    umask(um);
    if (bad) {
        perror("glassine-epdd-desktop: wl: listen");
        if (s->ls >= 0) close(s->ls);
        free(s);
        return NULL;
    }
    /* root:<panel group> 0660: sway runs as the desktop user, a member of it (as for the control FIFO);
     * any other user can't connect. (A failed chown leaves the daemon's own group, still not world) */
    if (s->gid != (gid_t)-1 && chown(sa.sun_path, (uid_t)-1, s->gid)) perror(sa.sun_path);
    if (chmod(sa.sun_path, 0660)) perror(sa.sun_path);
    fprintf(stderr, "glassine-epdd-desktop: wl: listening on %s for %dx%d frames\n", sa.sun_path, w, h);
    return s;
}

void epd_wl_set_peers(epd_wl *s, uid_t uid, gid_t gid) { s->uid = uid; s->gid = gid; }

/* may a process with these credentials be the backend: root, the one user, or a member of the group */
int epd_wl_peer_ok(const epd_wl *s, uid_t uid, gid_t gid)
{
    if (uid == 0 || uid == s->uid) return 1;
    if (s->gid == (gid_t)-1) return 0;
    if (gid == s->gid) return 1;
    struct passwd *pw = getpwuid(uid);
    if (!pw) return 0;
    int n = 32, ok = 0;
    gid_t *g = malloc(n * sizeof *g);
    if (g && getgrouplist(pw->pw_name, pw->pw_gid, g, &n) < 0 && n > 0) {      /* (n: how many it needs) */
        gid_t *g2 = realloc(g, n * sizeof *g);
        if (!g2 || getgrouplist(pw->pw_name, pw->pw_gid, g2, &n) < 0) n = 0;
        g = g2 ? g2 : g;
    }
    for (int i = 0; g && i < n; i++) ok |= g[i] == s->gid;
    free(g);
    return ok;
}

void epd_wl_close(epd_wl *s)
{
    if (!s) return;
    disconnect(s);
    close(s->ls);
    free(s);
}

int epd_wl_fds(const epd_wl *s, int fd[2])
{
    fd[0] = s->ls;
    fd[1] = s->fd;
    return s->fd >= 0 ? 2 : 1;
}
int epd_wl_pending(const epd_wl *s) { return s->pend; }

static int find(const epd_wl *s, uint32_t id)
{
    for (int i = 0; i < MAXBUF; i++) if (s->buf[i].live && s->buf[i].id == id) return i;
    return -1;
}

/* NULL: bytes [0, need) of fd can be mapped and read without a SIGBUS; else why not.
 * A dma-buf is bounded by the kernel (its mapping can't outgrow the buffer), its size is checked.
 * Any other file must be a memfd that can't shrink: sealed against it here when the sender
 * left that to us (the seal first, then the size: after it the size can't drop). A regular
 * file, or a memfd that allows no sealing, can be truncated by its owner while we read it */
static const char *fd_refuse(int fd, size_t need)
{
    struct statfs sf;
    struct stat st;
    if (!fstatfs(fd, &sf) && (unsigned long)sf.f_type == (unsigned long)DMA_BUF_MAGIC) {
        if (fstat(fd, &st)) return "fstat failed";
        off_t n = st.st_size ? st.st_size : lseek(fd, 0, SEEK_END);
        return n >= 0 && (unsigned long long)n >= need ? NULL : "dma-buf smaller than the buffer";
    }
    int seals = fcntl(fd, F_GET_SEALS);
    if (seals < 0) return "not a memfd (could be truncated while mapped)";
    if (!(seals & F_SEAL_SHRINK) && fcntl(fd, F_ADD_SEALS, F_SEAL_SHRINK) < 0)
        return "memfd that can't be sealed against shrinking (F_SEAL_SHRINK)";
    if (fstat(fd, &st)) return "fstat failed";
    return st.st_size >= 0 && (unsigned long long)st.st_size >= need ? NULL : "file smaller than the buffer";
}

static void on_buf(epd_wl *s, const struct glassine_epd_msg *m, int rfd, int dma)
{
    int i = find(s, m->id);
    if (i < 0) for (i = 0; i < MAXBUF && s->buf[i].live; i++) ;
    /* XRGB8888 portrait (w x h), or R8 in panel order (h x w: sway's grey output). A row is padded by a
     * few bytes, never by 16 times its width: a stride that large would turn negative in cpu_rects' int */
    int grey = m->format == R8;
    if (i == MAXBUF || rfd < 0 || m->width != (uint32_t)(grey ? s->h : s->w) || m->height != (uint32_t)(grey ? s->w : s->h) ||
        m->stride < m->width * (grey ? 1 : 4) || m->stride > 16 * m->width || (!grey && m->format != XRGB8888 && m->format != ARGB8888)) {
        fprintf(stderr, "glassine-epdd-desktop: wl: %s %u refused (%ux%u stride %u format %.4s; want %dx%d XRGB8888 or %dx%d R8)\n",
                dma ? "dma-buf" : "buffer", m->id, m->width, m->height, m->stride, (const char *)&m->format, s->w, s->h,
                s->h, s->w);
        if (rfd >= 0) close(rfd);
        return;
    }
    size_t len = (size_t)m->offset + (size_t)m->stride * m->height;
    const char *why = fd_refuse(rfd, len);
    if (s->buf[i].live) drop_one(s, i);     /* (a refused re-send leaves no buffer for the id: not a stale one) */
    if (why) {
        fprintf(stderr, "glassine-epdd-desktop: wl: %s %u refused: %s\n", dma ? "dma-buf" : "buffer", m->id, why);
        close(rfd);
        return;
    }
    s->buf[i].id = m->id; s->buf[i].w = m->width; s->buf[i].h = m->height; s->buf[i].stride = m->stride;
    s->buf[i].format = m->format; s->buf[i].offset = m->offset; s->buf[i].len = len;
    s->buf[i].live = 1;
    s->buf[i].grey = grey;
    if (dma) {                              /* kept open; imported by ops, mapped only for the CPU way */
        s->buf[i].dma = (epd_wl_dmabuf){ m->id, rfd, (int)m->width, (int)m->height, (int)m->stride, (int)m->offset,
                                         m->format, (uint64_t)m->nrect << 32 | m->seq, -1 };
        if (s->ops && s->ops->import(s->ops_ctx, &s->buf[i].dma) < 0) s->buf[i].dma.handle = -1;
        fprintf(stderr, "glassine-epdd-desktop: wl: dma-buf %u (%s, modifier 0x%llx): %s\n", m->id, grey ? "R8" : "XRGB",
                (unsigned long long)s->buf[i].dma.modifier, s->buf[i].dma.handle >= 0 ? "on the GPU" : "the CPU way");
        return;
    }
    uint8_t *p = mmap(NULL, len, PROT_READ, MAP_SHARED, rfd, 0);
    close(rfd);
    if (p == MAP_FAILED) { perror("glassine-epdd-desktop: wl: mmap"); s->buf[i].live = 0; return; }
    s->buf[i].map = p;
}

/* a dma-buf's CPU mapping (lazily); NULL: none */
static const uint8_t *dma_map(epd_wl *s, int i)
{
    if (!s->buf[i].map) {
        uint8_t *p = mmap(NULL, s->buf[i].len, PROT_READ, MAP_SHARED, s->buf[i].dma.fd, 0);
        if (p == MAP_FAILED) { perror("glassine-epdd-desktop: wl: dma-buf mmap"); return NULL; }
        s->buf[i].map = p;
    }
    return s->buf[i].map;
}

/* a buffer rect as a portrait rect: a grey (panel order) buffer's rect is
 * turned back (panel col c, row r = portrait (r, h - 1 - c)), so the damage,
 * the rect callbacks, the importer and the pointer all see portrait rects
 * whatever the buffer's layout */
static epd_wl_rect portrait(const epd_wl *s, int grey, int x, int y, int w, int h)
{
    return grey ? (epd_wl_rect){ y, s->h - (x + w), h, w } : (epd_wl_rect){ x, y, w, h };
}

/* a FRAME rect, clipped to the buffer, as a portrait rect */
static int clip(const epd_wl *s, int grey, struct glassine_epd_rect r, epd_wl_rect *o)
{
    int bw = grey ? s->h : s->w, bh = grey ? s->w : s->h;
    int x0 = r.x < 0 ? 0 : r.x, y0 = r.y < 0 ? 0 : r.y;
    int x1 = r.x + r.w > bw ? bw : r.x + r.w, y1 = r.y + r.h > bh ? bh : r.y + r.h;
    if (x1 <= x0 || y1 <= y0) return 0;
    *o = portrait(s, grey, x0, y0, x1 - x0, y1 - y0);
    return 1;
}

/* buffer i's mapped pixels of portrait rects into the callbacks */
static void cpu_rects(epd_wl *s, int i, const uint8_t *m, const epd_wl_rect *r, int n, epd_wl_rect_fn fn,
                      epd_wl_grey_fn gfn, void *ctx)
{
    const uint8_t *base = m + s->buf[i].offset;
    int st = s->buf[i].stride;
    for (int k = 0; k < n; k++) {
        if (s->buf[i].grey) {               /* the panel rect: cols [h - y - h', h - y), rows [x, x + w) */
            int px = s->h - r[k].y - r[k].h, py = r[k].x;
            if (gfn) gfn(ctx, base + (size_t)py * st + px, st, px, py, r[k].h, r[k].w);
        } else
            fn(ctx, (const uint32_t *)(base + (size_t)r[k].y * st) + r[k].x, st / 4, r[k].x, r[k].y, r[k].w, r[k].h);
    }
}

int epd_wl_cpu_rects(epd_wl *s, const epd_wl_dmabuf *b, const epd_wl_rect *r, int n, epd_wl_rect_fn fn,
                     epd_wl_grey_fn gfn, void *ctx)
{
    int i = find(s, b->id);
    const uint8_t *m = i < 0 ? NULL : s->buf[i].dma.fd >= 0 ? dma_map(s, i) : s->buf[i].map;
    if (!m) return -1;
    struct dma_buf_sync sy = { DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ };
    int dfd = s->buf[i].dma.fd;
    if (dfd >= 0) ioctl(dfd, DMA_BUF_IOCTL_SYNC, &sy);     /* waits for the renderer's fence */
    cpu_rects(s, i, m, r, n, fn, gfn, ctx);
    sy.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
    if (dfd >= 0) ioctl(dfd, DMA_BUF_IOCTL_SYNC, &sy);
    return 0;
}

void epd_wl_set_dmabuf(epd_wl *s, const epd_wl_dmabuf_ops *ops, void *ctx) { s->ops = ops; s->ops_ctx = ctx; }
void epd_wl_set_grey(epd_wl *s, epd_wl_grey_fn fn) { s->gfn = fn; }

void epd_wl_dmabuf_stats(epd_wl *s, unsigned long *gpu, unsigned long *cpu)
{
    *gpu = s->dma_gpu; *cpu = s->dma_cpu;
    s->dma_gpu = s->dma_cpu = 0;
}

/* the table the backend applies (its HELLO's formula) and its tag */
static const uint8_t *lut_cur(const epd_wl *s) { return s->caps & GLASSINE_EPD_CAP_LUM ? s->lut.lum : s->lut.luma; }
static uint32_t lut_tag(const epd_wl *s) { return s->lut.set ? glassine_epd_lut_tag(lut_cur(s)) : 0; }

static void lut_push(epd_wl *s)
{
    uint32_t t = lut_tag(s);
    if (s->fd < 0 || !(s->caps & GLASSINE_EPD_CAP_GAMMA) || !t || t == s->lut.sent) return;
    struct glassine_epd_msg m = { .type = GLASSINE_EPD_LUT, .id = t };
    memcpy(m.rect, lut_cur(s), 256);
    size_t len = offsetof(struct glassine_epd_msg, rect) + 256;
    if (send(s->fd, &m, len, MSG_NOSIGNAL | MSG_DONTWAIT) == (ssize_t)len) s->lut.sent = t;
}

void epd_wl_set_luts(epd_wl *s, const uint8_t luma[256], const uint8_t lum[256])
{
    memcpy(s->lut.luma, luma, 256);
    memcpy(s->lut.lum, lum, 256);
    s->lut.set = 1;
    lut_push(s);
}

static void band_push(epd_wl *s)
{
    if (s->fd < 0 || !(s->caps & GLASSINE_EPD_CAP_BAND)) return;
    if (s->band.t == s->band.sent_t && (!s->band.t || s->band.quiet == s->band.sent_quiet)) return;
    struct glassine_epd_msg m = { .type = GLASSINE_EPD_BAND, .seq = (uint32_t)s->band.t, .width = (uint32_t)s->band.quiet };
    if (send(s->fd, &m, offsetof(struct glassine_epd_msg, rect), MSG_NOSIGNAL | MSG_DONTWAIT) ==
        (ssize_t)offsetof(struct glassine_epd_msg, rect)) {
        s->band.sent_t = s->band.t;
        s->band.sent_quiet = s->band.quiet;
    }
}

void epd_wl_set_band(epd_wl *s, int t, int quiet_ms, epd_wl_damage_fn fn, void *ctx)
{
    s->band.t = t;
    s->band.quiet = quiet_ms;
    s->band.fn = fn;
    s->band.ctx = ctx;
    band_push(s);
}

int epd_wl_prelut(const epd_wl *s) { return s && s->lut.pre; }

unsigned long epd_wl_stale(epd_wl *s, unsigned long *pre)
{
    unsigned long n = s->lut.stale;
    *pre = s->lut.pre_n;
    s->lut.stale = s->lut.pre_n = 0;
    return n;
}

/* buffer i's rects into the callbacks (the GPU, or the CPU way). tag: the
 * FRAME's; one that is not our table's (it changed since) is not read: the
 * backend sends the whole picture through the new one */
static int convert_raw(epd_wl *s, int i, const epd_wl_rect *rc, int n);
static void send_ans(epd_wl *s, uint32_t type, uint32_t id, uint32_t seq);

/* the kept frame's DONE (the daemon no longer reads it) */
void epd_wl_unkeep(epd_wl *s)
{
    if (!s->keep.on) return;
    s->keep.on = 0;
    send_ans(s, GLASSINE_EPD_DONE, s->keep.id, s->keep.seq);
}

/* 1: the daemon now reads buffer i in place (kept: its DONE waits); the frame
 * before it, kept or not, is answered */
static int convert(epd_wl *s, int i, const epd_wl_rect *rc, int n, uint32_t tag, uint32_t seq)
{
    if (tag && tag != lut_tag(s)) { s->lut.stale++; return 0; }
    s->lut.pre = tag != 0;
    s->lut.pre_n += tag != 0;
    int kept = convert_raw(s, i, rc, n);
    s->lut.pre = 0;
    epd_wl_unkeep(s);                       /* (kept again or copied: the old view is done with) */
    if (kept) s->keep = (typeof(s->keep)){ 1, i, s->buf[i].id, seq };
    return kept;
}

static int convert_raw(epd_wl *s, int i, const epd_wl_rect *rc, int n)
{
    if (s->buf[i].dma.fd >= 0) {
        epd_wl_dmabuf *b = &s->buf[i].dma;
        int r = s->ops && b->handle >= 0 ? s->ops->frame(s->ops_ctx, b, rc, n) : -1;
        if (r >= 0) { s->dma_gpu++; return r > 0; }
        s->dma_cpu++;
        epd_wl_cpu_rects(s, b, rc, n, s->fn, s->gfn, s->ctx);
        return 0;
    }
    cpu_rects(s, i, s->buf[i].map, rc, n, s->fn, s->gfn, s->ctx);
    return 0;
}

static void send_ans(epd_wl *s, uint32_t type, uint32_t id, uint32_t seq)
{
    struct glassine_epd_msg d = { .type = type, .id = id, .seq = seq };
    if (s->fd >= 0 && send(s->fd, &d, offsetof(struct glassine_epd_msg, rect), MSG_NOSIGNAL) < 0 && errno != EAGAIN)
        disconnect(s);
}

/* a frame's answer: DONE, or for the kept one NEXT (sway goes on; DONE later) */
static void answer(epd_wl *s, uint32_t id, uint32_t seq, int nexted)
{
    if (s->keep.on && s->keep.seq == seq && s->keep.id == id) {
        if (!nexted) send_ans(s, GLASSINE_EPD_NEXT, id, seq);
        return;
    }
    send_ans(s, GLASSINE_EPD_DONE, id, seq);
}

void epd_wl_flush(epd_wl *s)
{
    if (!s->held.on) return;
    s->held.on = 0;
    if (s->buf[s->held.idx].live && s->buf[s->held.idx].id == s->held.id)
        convert(s, s->held.idx, s->held.r, s->held.n, s->held.tag, s->held.seq);
    if (s->held.nexted) answer(s, s->held.id, s->held.seq, 1);
}

static void add_rect(epd_wl_rect *r, int *n, epd_wl_rect a)
{
    if (*n < GLASSINE_EPD_MAXRECT) { r[(*n)++] = a; return; }
    epd_wl_rect *b = &r[0];                 /* full: all into the first one's bounding box */
    for (int k = 1; k < *n; k++) {
        int x1 = b->x + b->w > r[k].x + r[k].w ? b->x + b->w : r[k].x + r[k].w;
        int y1 = b->y + b->h > r[k].y + r[k].h ? b->y + b->h : r[k].y + r[k].h;
        if (r[k].x < b->x) b->x = r[k].x;
        if (r[k].y < b->y) b->y = r[k].y;
        b->w = x1 - b->x; b->h = y1 - b->y;
    }
    *n = 1;
    add_rect(r, n, a);
}

static int on_frame(epd_wl *s, const struct glassine_epd_msg *m)
{
    if (s->pend) epd_wl_done(s);            /* the backend moved on (its timeout): answer the old one */
    s->pend = 1; s->pend_id = m->id; s->pend_seq = m->seq;
    s->frames++;
    if (s->done_t > 0) {
        double ms = (mono() - s->done_t) * 1e3;
        s->ring[s->ring_i] = ms;
        s->ring_i = (s->ring_i + 1) % EPD_WL_CHAIN_N;
        if (s->ring_n < EPD_WL_CHAIN_N) s->ring_n++;
        s->ch_n++; s->ch_sum += ms;
        if (ms > s->ch_max) s->ch_max = ms;
        s->done_t = 0;
    }
    int i = find(s, m->id);
    if (i < 0) return 0;                    /* refused buffer: DONE all the same */
    epd_wl_rect rc[GLASSINE_EPD_MAXRECT];
    int n = 0, grey = s->buf[i].grey;
    struct glassine_epd_rect whole = { 0, 0, grey ? s->h : s->w, grey ? s->w : s->h };
    uint32_t nr = m->nrect > GLASSINE_EPD_MAXRECT ? GLASSINE_EPD_MAXRECT : m->nrect;
    for (uint32_t k = 0; k < (nr ? nr : 1); k++)
        if (clip(s, grey, nr ? m->rect[k] : whole, &rc[n])) {
            s->px += (unsigned long long)rc[n].w * rc[n].h;
            n++;
        }
    if (!n) return 0;
    /* the dead band's true frame: no new content, not damage (returns 0: no
     * pol_damage either); else a frame as any: read when due, answered as usual */
    int tr = m->offset & GLASSINE_EPD_FRAME_TRUE && s->band.fn;
    if (tr) s->band.fn(s->band.ctx, rc, n);
    if (!s->dmg) { convert(s, i, rc, n, m->format, m->seq); return tr ? 0 : n; }
    if (!tr) s->dmg(s->dmg_ctx, rc, n);
    /* an older frame NEXT'ed and never read: answered unread, its damage rides
     * on this one (its buffer has the whole picture) */
    int nn = n;
    if (s->held.on && s->held.nexted) {
        for (int k = 0; k < s->held.n; k++) add_rect(rc, &nn, s->held.r[k]);
        s->held.on = 0;
        s->skipped++;
        send_ans(s, GLASSINE_EPD_DONE, s->held.id, s->held.seq);
    } else epd_wl_flush(s);                 /* (one not yet answered was by epd_wl_done above) */
    s->held.on = 1; s->held.nexted = 0; s->held.idx = i; s->held.id = m->id; s->held.seq = m->seq;
    s->held.tag = m->format;              /* (FRAME's format: the gamma tag, 0 before) */
    memcpy(s->held.r, rc, nn * sizeof *rc);
    s->held.n = nn;
    return tr ? 0 : n;
}

void epd_wl_set_damage(epd_wl *s, epd_wl_damage_fn fn, void *ctx) { s->dmg = fn; s->dmg_ctx = ctx; }

void epd_wl_set_cursor(epd_wl *s, epd_wl_cursor_fn fn, void *ctx) { s->cur.fn = fn; s->cur.ctx = ctx; }

/* CURSOR_IMG: only noted (its slot holds the sprite until the IMG after next:
 * read at the dispatch's end, the newest one only); the memfd comes with the first */
static void on_cursor_img(epd_wl *s, const struct glassine_epd_msg *m, int rfd)
{
    if (rfd >= 0) {
        if (s->cur.map) munmap(s->cur.map, 2 * GLASSINE_EPD_CUR_SLOT);
        s->cur.map = NULL;
        const char *why = fd_refuse(rfd, 2 * GLASSINE_EPD_CUR_SLOT);
        void *p = why ? MAP_FAILED : mmap(NULL, 2 * GLASSINE_EPD_CUR_SLOT, PROT_READ, MAP_SHARED, rfd, 0);
        close(rfd);
        s->cur.map = p == MAP_FAILED ? NULL : p;
        if (why) fprintf(stderr, "glassine-epdd-desktop: wl: cursor memfd refused: %s\n", why);
        else if (!s->cur.map) perror("glassine-epdd-desktop: wl: cursor mmap");
    }
    int ok = m->width <= GLASSINE_EPD_CUR_MAX && m->height <= GLASSINE_EPD_CUR_MAX && m->stride >= m->width * 4 &&
             m->offset <= GLASSINE_EPD_CUR_SLOT && m->stride % 4 == 0 && (size_t)m->stride * m->height <= GLASSINE_EPD_CUR_SLOT;
    s->cur.w = ok && m->height ? (int)m->width : 0;
    s->cur.h = (int)m->height;
    s->cur.hx = m->rect[0].x; s->cur.hy = m->rect[0].y;
    s->cur.panel = m->format == GLASSINE_EPD_CUR_PANEL;
    s->cur.offset = m->offset;
    s->cur.stride = m->stride;
    s->cur.img_new = 1;
}

/* the pointer's news, once per dispatch: the sprite turned portrait (as the
 * frames' rects: a panel-order sprite pixel (u, v) = portrait (v, w - 1 - u)
 * of its box), the box from the hotspot's place */
static void cursor_out(epd_wl *s)
{
    if (!s->cur.img_new && !s->cur.pos_new) return;
    int img = s->cur.img_new, moved = s->cur.pos_new, on = s->cur.w > 0 && s->cur.map;
    s->cur.img_new = s->cur.pos_new = 0;
    if (!s->cur.fn) return;
    int w = s->cur.w, h = s->cur.h;
    if (on && img) {
        const uint32_t *src = (const uint32_t *)(s->cur.map + s->cur.offset);
        int st = (int)s->cur.stride / 4;
        for (int v = 0; v < h; v++)
            for (int u = 0; u < w; u++)
                s->cur.px[s->cur.panel ? (size_t)(w - 1 - u) * h + v : (size_t)v * w + u] = src[(size_t)v * st + u];
    }
    epd_wl_cursor c = { on, img, portrait(s, s->cur.panel, s->cur.x - s->cur.hx, s->cur.y - s->cur.hy, w, h), s->cur.px,
                        moved, s->cur.panel };
    s->cur.fn(s->cur.ctx, &c);
}

uint32_t epd_wl_caps(const epd_wl *s) { return s ? s->caps : 0; }

unsigned long epd_wl_skipped(epd_wl *s)
{
    unsigned long n = s->skipped;
    s->skipped = 0;
    return n;
}

int epd_wl_dispatch(epd_wl *s)
{
    int n = 0;
    for (;;) {                              /* a new backend replaces the old one, if it is one we allow */
        int c = accept4(s->ls, NULL, NULL, SOCK_CLOEXEC | SOCK_NONBLOCK);
        if (c < 0) break;
        struct ucred uc;
        socklen_t ul = sizeof uc;
        if (getsockopt(c, SOL_SOCKET, SO_PEERCRED, &uc, &ul) || ul != sizeof uc || !epd_wl_peer_ok(s, uc.uid, uc.gid)) {
            fprintf(stderr, "glassine-epdd-desktop: wl: connection from uid %u refused\n", ul == sizeof uc ? (unsigned)uc.uid : ~0u);
            close(c);                       /* (the connected backend is not disturbed) */
            continue;
        }
        disconnect(s);
        s->fd = c;
        fprintf(stderr, "glassine-epdd-desktop: wl: backend connected\n");
    }
    while (s->fd >= 0) {
        struct glassine_epd_msg m;
        union { char b[CMSG_SPACE(sizeof(int))]; struct cmsghdr a; } ctl;
        struct iovec iov = { &m, sizeof m };
        struct msghdr mh = { .msg_iov = &iov, .msg_iovlen = 1, .msg_control = ctl.b, .msg_controllen = sizeof ctl.b };
        memset(&m, 0, sizeof m);
        ssize_t k = recvmsg(s->fd, &mh, MSG_CMSG_CLOEXEC);
        if (k < 0 && (errno == EAGAIN || errno == EINTR)) break;
        if (k <= 0) { disconnect(s); break; }
        int rfd = -1;
        struct cmsghdr *cm = CMSG_FIRSTHDR(&mh);
        if (cm && cm->cmsg_level == SOL_SOCKET && cm->cmsg_type == SCM_RIGHTS) memcpy(&rfd, CMSG_DATA(cm), sizeof rfd);
        if (m.type == GLASSINE_EPD_BUF || m.type == GLASSINE_EPD_DMABUF) on_buf(s, &m, rfd, m.type == GLASSINE_EPD_DMABUF);
        else if (m.type == GLASSINE_EPD_CURSOR_IMG) on_cursor_img(s, &m, rfd);
        else {
            if (rfd >= 0) close(rfd);
            if (m.type == GLASSINE_EPD_DROP) {
                int i = find(s, m.id);
                if (i >= 0) drop_one(s, i);
            } else if (m.type == GLASSINE_EPD_FRAME) n += on_frame(s, &m);
            else if (m.type == GLASSINE_EPD_CURSOR_POS) {
                s->cur.x = m.rect[0].x; s->cur.y = m.rect[0].y;
                s->cur.pos_new = 1;
            } else if (m.type == GLASSINE_EPD_HELLO) {
                s->caps = m.seq;
                fprintf(stderr, "glassine-epdd-desktop: wl: backend can%s hold a frame (NEXT), grey output by %s, gamma on %s\n",
                        s->caps & GLASSINE_EPD_CAP_NEXT ? "" : "not", s->caps & GLASSINE_EPD_CAP_LUM ? "luminance" : "luma",
                        s->caps & GLASSINE_EPD_CAP_GAMMA ? "its GPU" : "our CPU");
                lut_push(s);
                band_push(s);
            }
        }
    }
    if (s->fd >= 0) cursor_out(s);
    return n;
}

void epd_wl_done(epd_wl *s)
{
    if (!s->pend) return;
    epd_wl_flush(s);                        /* its buffer goes back to sway: read it now */
    s->pend = 0;
    s->done_t = mono();
    answer(s, s->pend_id, s->pend_seq, 0);
}

void epd_wl_next(epd_wl *s)
{
    if (!s->pend) return;
    if (!(s->caps & GLASSINE_EPD_CAP_NEXT) || !s->held.on || s->held.seq != s->pend_seq) { epd_wl_done(s); return; }
    s->pend = 0;
    s->done_t = mono();
    s->held.nexted = 1;
    send_ans(s, GLASSINE_EPD_NEXT, s->pend_id, s->pend_seq);
}

void epd_wl_stats(epd_wl *s, unsigned long *frames, unsigned long long *px)
{
    *frames = s->frames; *px = s->px;
    s->frames = 0; s->px = 0;
}

void epd_wl_chain(epd_wl *s, unsigned long *n, double *avg_ms, double *max_ms)
{
    *n = s->ch_n; *avg_ms = s->ch_n ? s->ch_sum / s->ch_n : 0; *max_ms = s->ch_max;
    s->ch_n = 0; s->ch_sum = s->ch_max = 0;
}

double epd_wl_chain_pct(const epd_wl *s, int pct, double none)
{
    if (!s->ring_n) return none;
    double v[EPD_WL_CHAIN_N];
    memcpy(v, s->ring, sizeof v);
    for (int i = 1; i < s->ring_n; i++)     /* insertion sort: 16 values */
        for (int j = i; j > 0 && v[j - 1] > v[j]; j--) { double t = v[j]; v[j] = v[j - 1]; v[j - 1] = t; }
    int k = (s->ring_n - 1) * pct / 100;
    return v[k];
}

void epd_wl_ask_swayd(void)
{
    glob_t g;
    if (glob("/run/user/*/glassine-swayd", 0, NULL, &g)) return;
    int fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
    for (size_t i = 0; fd >= 0 && i < g.gl_pathc; i++) {
        struct sockaddr_un sa = { .sun_family = AF_UNIX };
        snprintf(sa.sun_path, sizeof sa.sun_path, "%s", g.gl_pathv[i]);
        sendto(fd, "apps?", 5, 0, (struct sockaddr *)&sa, sizeof sa);
    }
    if (fd >= 0) close(fd);
    globfree(&g);
}
