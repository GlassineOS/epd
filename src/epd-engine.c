/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-engine — see epd-engine.h. The frame generator writes the panel's
 * drive frames. */

#include <errno.h>
#include <malloc.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "epd-engine.h"
#ifdef __linux__                                /* (W as a dma-buf) */
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/memfd.h>
#include <linux/udmabuf.h>
#ifndef F_ADD_SEALS
#define F_ADD_SEALS 1033
#define F_SEAL_SHRINK 0x0002
#endif
int close(int fd);                              /* (not unistd.h: epd-claim-bench includes this file) */
int ftruncate(int fd, off_t n);
int memfd_create(const char *name, unsigned flags);
#endif
#ifdef EPD_CLAIM_PROF
#include <time.h>
#endif
#ifdef __aarch64__
#include <arm_neon.h>
#endif

/* healing's runs (epd_set_heal) have the last two slots: their pixels' busy
 * values are >= HEAL_B, one compare. A heal only pushes a pixel toward the
 * level it shows, so a step that changes the pixel takes it at once
 * (claim_row: preempts) and nothing waits for a heal (device run:
 * content lagged behind the heal runs) */
#define RUN_SLOTS EPD_RUN_SLOTS
#define HEAL_B EPD_HEAL_B

/* the panel's line and frame timings (the values the device tree sets) */
enum {
    LSL = 18, LBL = 17, LEL = 20 - 4, LDL = EPD_W * 2 / 16,
    FSL = 1, FBL = 4, FEL = 14, GDCK_STA = 17, LGONL = 192,
    FDL_POS = EPD_LINE * (FBL + FSL) + (LBL - (LBL >> 1)),     /* 1434 */
    LDL_POS = LSL + LBL,                                        /* 35 */
    DATA0 = FDL_POS + LDL_POS,                                  /* word of panel col 0, wire row 0 */
    DATA_LINE0 = DATA0 / EPD_LINE,                              /* 5 */
};
#define GDCLK 0x020000u
#define GDSP  0x040000u
#define SDSP  0x080000u
#define SDOE  0x100000u
#define GDOE  0x200000u
#define SDLE  0x400000u

_Static_assert(LSL + LBL + LEL + LDL == EPD_LINE, "line length");
_Static_assert(DATA0 == EPD_DATA0, "EPD_DATA0");
_Static_assert(EPD_H + FSL + FEL + FBL == EPD_LINES, "frame lines");

/* ---- waveform file (standard E Ink WBF) ---------------------------------- */

static uint32_t u24(const epd_wbf *w, uint32_t o)
{
    return o + 2 < w->n ? w->d[o] | w->d[o + 1] << 8 | (uint32_t)w->d[o + 2] << 16 : 0;
}

int epd_wbf_load(epd_wbf *w, const uint8_t *d, size_t n)
{
    memset(w, 0, sizeof *w);
    if (n < 0x100) return -1;
    w->d = d; w->n = n;
    w->modes = d[0x25] + 1;
    w->temps = d[0x26] + 1;
    w->frame_rate = d[0x17];
    if (w->modes > 16 || w->temps > 62) return -1;
    memcpy(w->ranges, d + 0x30, w->temps + 1);
    w->mode_table = u24(w, 0x20);
    return 0;
}

int epd_wbf_temp_index(const epd_wbf *w, int c)
{
    for (int i = 0; i < w->temps; i++)
        if (c < w->ranges[i + 1]) return i;
    return w->temps - 1;
}

static int decode(const epd_wbf *w, uint32_t off, uint8_t **out)
{
    size_t cap = 64 * 1024, n = 0;
    uint8_t *v = malloc(cap);
    int rle = 1;
    uint8_t eb = w->d[0x28], sb = w->d[0x29];
    if (!v) return -1;
    for (uint32_t i = off; i < w->n;) {
        uint8_t b = w->d[i++];
        if (b == eb) break;
        if (b == sb) { rle = !rle; continue; }
        int cnt = 1;
        if (rle) { if (i >= w->n) break; cnt = w->d[i++] + 1; }
        while (cnt--) {
            if (n + 4 > cap) {
                uint8_t *nv = realloc(v, cap *= 2);
                if (!nv) { free(v); return -1; }
                v = nv;
            }
            for (int s = 0; s < 8; s += 2) v[n++] = (b >> s) & 3;
        }
    }
    *out = v;
    return (int)(n / 1024);
}

int epd_wbf_frames(const epd_wbf *w, int mode, int temp, uint8_t **out)
{
    if (mode < 0 || mode >= w->modes || temp < 0 || temp >= w->temps) return -1;
    return decode(w, u24(w, u24(w, w->mode_table + 4 * mode) + 4 * temp), out);
}

/* ---- the control template (a 16-bit timing buffer) ------------------- */

static void orr(uint32_t *b, long p, long cnt, uint32_t bit)
{
    if (p < 0) { cnt += p; p = 0; }
    if (p + cnt > EPD_WORDS) cnt = EPD_WORDS - p;
    for (long i = 0; i < cnt; i++) b[p + i] |= bit;
}

void epd_template(uint32_t *b)
{
    const long L = EPD_LINE;
    long low = GDCK_STA + LBL + LBL + LSL - (LBL >> 1), high = LGONL, p, cnt;
    memset(b, 0, EPD_WORDS * sizeof *b);
    for (long r = 0; r < FSL + FBL + EPD_H; r++) orr(b, r * L + low, high, GDCLK);
    long k = (high >> 1) + low;
    orr(b, 0, k, GDSP);
    p = k;
    orr(b, p, L * FSL, GDSP);
    p += L * FSL * 2;
    cnt = ((L - low) + ((FBL + EPD_H - 1 + FEL) - FSL) * L) - (high >> 1) - 1;
    orr(b, p, cnt, GDSP);
    orr(b, L * FSL, L * (EPD_H + FBL), GDOE);
    orr(b, L * FSL, L * FBL, SDOE);
    orr(b, L * FSL + L * FBL, L * EPD_H, SDOE);
    long h = LBL >> 1;
    p = (LBL - h) + L * (FBL + FSL);
    for (long r = 0; r < EPD_H; r++) orr(b, p + r * L, LSL, SDLE);
    long first = LBL - h;
    orr(b, 0, first, SDSP);
    p = first;
    orr(b, p, L * (FBL + FSL), SDSP);
    p += L * (FBL + FSL);
    long a = LBL + LSL;
    for (long r = 0; r < EPD_H; r++) {
        orr(b, p + r * L, a, SDSP);
        orr(b, p + r * L + a + LDL, LEL, SDSP);
    }
    p += EPD_H * L;
    uint32_t c = (uint32_t)((h - LBL) + FEL * L - 1);
    if (c < 0x80000000u) orr(b, p, c, SDSP);
}

/* ---- engine --------------------------------------------------------------- */

/* ---- the knobs (epd-knobs.h): defined here, the env read once at epd_init */
EPD_KNOBS_ENGINE(EPD_KNOB_DEFINE)
#define KNOB_IDX(var, ...) KN_##var,
enum { EPD_KNOBS_ENGINE(KNOB_IDX) KN_N };
static const epd_knob eng_knobs[] = { EPD_KNOBS_ENGINE(EPD_KNOB_ROW) };
#define KNOB_DEF(var) (eng_knobs[KN_##var].def)  /* (a knob set back to -1 after epd_init: its default) */
static int pk_w_malloc;                         /* GLASSINE_CLAIM_STATE_W=malloc (pk_wfd) */

/* ---- : the state's allocation, D/W/T --------------- */
unsigned long epd_claim_state_diffs, epd_claim_state_checked;

static int claim_state_of(void)
{
    int k = __atomic_load_n(&epd_claim_state, __ATOMIC_RELAXED);
    if (k < 0) k = KNOB_DEF(epd_claim_state);   /* (GLASSINE_CLAIM_STATE: epd_init) */
    return k > 2 ? 0 : k;
}

const char *epd_claim_state_name(const epd_engine *e)
{
    return e->cst == 1 ? "packed" : e->cst == 2 ? "check" : "arrays";
}

/* stagger_state (epd_stagger_state; -1 = GLASSINE_STAGGER_STATE=lanes|runs
 * at epd_init, not live): lanes = D and T dma-bufs too (packed only), the
 * stagger's GPU pass imports them (epd-gpu.c stag_lanes) */
static int stagger_state_of(void)
{
    int k = __atomic_load_n(&epd_stagger_state, __ATOMIC_RELAXED);
    if (k < 0) k = KNOB_DEF(epd_stagger_state);
    return k;
}

/* DMA_BUF_IOCTL_SYNC over [off, off + len) (epd-engine.h): the ACK's range
 * ioctl, probed at the first call (ENOTTY: none, the whole buffer from then
 * on, as epd-ingest-cpu.h's icpu_sync_ioctl latches it; GLASSINE_DMABUF_PARTIAL=0:
 * never tried) */
struct epd_sync_partial { uint64_t flags; uint32_t offset, len; };
#define EPD_SYNC_PARTIAL _IOW(DMA_BUF_BASE, 2, struct epd_sync_partial)
double epd_dmabuf_sync_s;
unsigned long epd_dmabuf_sync_n;
static double sync_now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec / 1e9; }
#define SYNC_SITES 32
static struct sync_site { const char *file; int line; unsigned long n, nwhole; double s; unsigned long long bytes; } sync_site[SYNC_SITES];
static pthread_mutex_t sync_mu = PTHREAD_MUTEX_INITIALIZER;   /* (beside an ioctl: free) */
static void site_add(const char *file, int line, size_t len, double dt)
{
    pthread_mutex_lock(&sync_mu);
    for (int i = 0; i < SYNC_SITES; i++) {
        struct sync_site *x = &sync_site[i];
        if (!x->file) { x->file = file; x->line = line; }
        if (x->file != file || x->line != line) continue;
        x->n++;
        if (!len) x->nwhole++;
        x->bytes += len;
        x->s += dt;
        break;
    }
    pthread_mutex_unlock(&sync_mu);
}
void epd_dmabuf_sync_sites(void *fp)
{
    FILE *f = fp;
    int ord[SYNC_SITES], n = 0;
    pthread_mutex_lock(&sync_mu);
    for (int i = 0; i < SYNC_SITES; i++) if (sync_site[i].file && sync_site[i].n) ord[n++] = i;
    for (int i = 1; i < n; i++)
        for (int j = i; j > 0 && sync_site[ord[j]].s > sync_site[ord[j - 1]].s; j--) { int t = ord[j]; ord[j] = ord[j - 1]; ord[j - 1] = t; }
    for (int k = 0; k < n; k++) {
        struct sync_site *x = &sync_site[ord[k]];
        const char *b = strrchr(x->file, '/');
        fprintf(f, "epd: dma-buf sync %s:%d: %lu calls (%lu whole buffer), %.1f ms, avg %.3f ms, %.1f MB asked in ranges\n",
                b ? b + 1 : x->file, x->line, x->n, x->nwhole, x->s * 1e3, x->s * 1e3 / x->n, x->bytes / 1e6);
        x->n = x->nwhole = 0; x->s = 0; x->bytes = 0;
    }
    pthread_mutex_unlock(&sync_mu);
}
int epd_dmabuf_sync_at(const char *file, int line, int fd, unsigned flags, size_t off, size_t len)
{
#ifdef __linux__
    if (fd < 0) return 0;
    double t0 = sync_now();
    int r = -1, p = __atomic_load_n(&epd_dmabuf_partial, __ATOMIC_RELAXED);
    if (p < 0) p = 2;                               /* (to be probed; GLASSINE_DMABUF_PARTIAL=0: 0 from epd_init) */
    if (p && len) {
        struct epd_sync_partial sp = { flags, (uint32_t)off, (uint32_t)len };
        r = ioctl(fd, EPD_SYNC_PARTIAL, &sp);
        if (p == 2) p = r == 0 || errno != ENOTTY;      /* (ENOTTY: no such ioctl; another error: this call only, the whole buffer) */
    }
    if (p != 2) __atomic_store_n(&epd_dmabuf_partial, p, __ATOMIC_RELAXED);
    if (r) {
        struct dma_buf_sync sy = { flags };
        r = ioctl(fd, DMA_BUF_IOCTL_SYNC, &sy);
    }
    double dt = sync_now() - t0;
    __atomic_fetch_add(&epd_dmabuf_sync_n, 1, __ATOMIC_RELAXED);
    epd_dmabuf_sync_s += dt;                        /* (a statistic: races only blur it) */
    site_add(file, line, len, dt);
    return r;
#else
    (void)file; (void)line; (void)fd; (void)flags; (void)off; (void)len;
    return 0;
#endif
}

/* a dma-buf the GPU can import (epd-gpu.c heap_fd's order: the system heap,
 * else a udmabuf over a memfd), -1 = none */
static int pk_dmabuf(size_t n, const char **src)
{
#ifdef __linux__
    int h = open("/dev/dma_heap/system", O_RDONLY | O_CLOEXEC);
    if (h >= 0) {
        struct dma_heap_allocation_data al = { .len = n, .fd_flags = O_RDWR | O_CLOEXEC };
        int r = ioctl(h, DMA_HEAP_IOCTL_ALLOC, &al);
        close(h);
        if (r == 0) { *src = "dma-heap"; return (int)al.fd; }
    }
    int m = memfd_create("glassine-epd-state", MFD_ALLOW_SEALING | MFD_CLOEXEC), u = -1, fd = -1;
    if (m < 0) return -1;
    if (!ftruncate(m, (off_t)n) && !fcntl(m, F_ADD_SEALS, F_SEAL_SHRINK) && (u = open("/dev/udmabuf", O_RDWR | O_CLOEXEC)) >= 0) {
        struct udmabuf_create c = { .memfd = (unsigned)m, .flags = UDMABUF_FLAGS_CLOEXEC, .offset = 0, .size = n };
        fd = ioctl(u, UDMABUF_CREATE, &c);
        close(u);
    }
    close(m);
    *src = "udmabuf";
    return fd;
#else
    (void)n; (void)src;
    return -1;
#endif
}

/* W's buffer: a dma-buf the GPU can import (epd-gpu.c heap_fd's order: the
 * system heap, else a udmabuf over a memfd), -1 = none */
static int pk_wfd(size_t n, const char **src)
{
#ifdef __linux__
    if (pk_w_malloc) return -1;                 /* (GLASSINE_CLAIM_STATE_W=malloc: no dma-buf, epd-gpu packs st from W: the tex fallback, a test) */
    return pk_dmabuf(n, src);
#else
    (void)n; (void)src;
    return -1;
#endif
}

static void pk_free(epd_pk *k)
{
    if (!k) return;
#ifdef __linux__
    if (k->wfd >= 0) { munmap(k->W, k->wsize); close(k->wfd); } else
#endif
    free(k->W);
#ifdef __linux__
    if (k->dfd >= 0) { munmap(k->D, k->dsize); munmap(k->T, k->tsize); close(k->dfd); close(k->tfd); } else
#endif
    { free(k->D); free(k->T); }
    free(k->Wsh);
    free(k);
}

int epd_state_alloc(epd_engine *e)
{
    if (e->cst != 1 && !e->level) {             /* arrays, check */
        e->level = calloc(EPD_NPX, 1); e->target = calloc(EPD_NPX, 1); e->busy = calloc(EPD_NPX, 1);
        e->op = calloc(EPD_NPX, 1); e->stale = calloc(EPD_NPX, 1); e->dc = calloc(EPD_NPX, sizeof *e->dc);
        if (!e->level || !e->target || !e->busy || !e->op || !e->stale || !e->dc) return -1;
    }
    if (e->cst && !e->pk) {                     /* packed, check: D/W/T */
        epd_pk *k = e->pk = calloc(1, sizeof *k);
        if (!k) return -1;
        k->wfd = -1;
        k->dfd = k->tfd = -1;
        size_t nw = (size_t)EPD_PK_S * EPD_H;
#ifdef __linux__
        /* (stagger_state=lanes, packed only: D and T dma-bufs for the
         * stagger's pass, D with EPD_PK_DPAD scratch rows below (LEAD's)) */
        if (e->cst == 1 && stagger_state_of()) {
            const char *sd = "", *st = "";
            k->dsize = ((size_t)EPD_PK_S * 4 * (EPD_H + EPD_PK_DPAD) + 4095) & ~(size_t)4095;
            k->tsize = (nw + 4095) & ~(size_t)4095;
            if ((k->dfd = pk_dmabuf(k->dsize, &sd)) >= 0 && (k->tfd = pk_dmabuf(k->tsize, &st)) >= 0) {
                void *dm = mmap(NULL, k->dsize, PROT_READ | PROT_WRITE, MAP_SHARED, k->dfd, 0);
                void *tm = dm == MAP_FAILED ? MAP_FAILED : mmap(NULL, k->tsize, PROT_READ | PROT_WRITE, MAP_SHARED, k->tfd, 0);
                if (tm != MAP_FAILED) { k->D = dm; k->T = tm; k->dtsrc = sd; }
                else if (dm != MAP_FAILED) munmap(dm, k->dsize);
            }
            if (!k->D) {
                if (k->dfd >= 0) close(k->dfd);
                if (k->tfd >= 0) close(k->tfd);
                k->dfd = k->tfd = -1;
                fprintf(stderr, "epd-engine: stagger_state=lanes: no dma-buf for D/T (malloc: the stagger keeps its runs)\n");
            }
        }
#endif
        if (!k->D) {
            k->dtsrc = "malloc";
            if (posix_memalign((void **)&k->D, 64, nw * 4) || posix_memalign((void **)&k->T, 64, nw)) return -1;
        }
        memset(k->D, 0, nw * 4); memset(k->T, 0, nw);
        k->wsize = (nw * 2 + 4095) & ~(size_t)4095;
#ifdef __linux__
        /* (packed only: check's W is a mirror, never imported) */
        if (e->cst == 1 && (k->wfd = pk_wfd(k->wsize, &k->wsrc)) >= 0) {
            void *m = mmap(NULL, k->wsize, PROT_READ | PROT_WRITE, MAP_SHARED, k->wfd, 0);
            if (m == MAP_FAILED) { close(k->wfd); k->wfd = -1; } else k->W = m;
        }
#endif
        if (k->wfd < 0) {
            k->wsrc = "malloc";
            if (posix_memalign((void **)&k->W, 64, k->wsize)) return -1;
        }
        for (size_t i = 0; i < k->wsize / 2; i++) k->W[i] = EPD_PK_IDLE;
    }
    return 0;
}

static void state_free(epd_engine *e)
{
    free(e->level); free(e->target); free(e->busy); free(e->op); free(e->stale); free(e->dc);
    e->level = e->target = e->busy = e->op = e->stale = NULL; e->dc = NULL;
    pk_free(e->pk);
    e->pk = NULL;
}

/* W's rows [y0, y1) about to be written by the CPU: START|WRITE over them
 * if none is open (the formal start: a no-op class on the system heap),
 * else the open range grown. Only those rows get the END at epd_pk_w_sync
 * ('s whole-buffer END|WRITE cost +0.3 ms a submit;
 * ranged with DMA_BUF_IOCTL_SYNC_PARTIAL, the whole buffer without it) */
void epd_pk_w_rows(const epd_engine *e, int y0, int y1)
{
    epd_pk *k = e->pk;
    if (!k) return;
    if (y0 < 0) y0 = 0;
    if (y1 > EPD_H) y1 = EPD_H;
    if (y0 >= y1) return;
    const size_t row = (size_t)EPD_PK_S * sizeof *k->W;
    if (!k->wdirty) {
        if (k->wfd >= 0) epd_dmabuf_sync(k->wfd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE, (size_t)y0 * row, (size_t)(y1 - y0) * row);
        k->wy0 = y0; k->wy1 = y1;
        k->wdirty = 1;
        return;
    }
    /* (more rows: no START for them — the GPU never writes W, so nothing of
     * the CPU's lines is stale; a START per extension would cost an ioctl a
     * row in per-pixel loops, a whole-buffer one each without the range ioctl) */
    if (y0 < k->wy0) k->wy0 = y0;
    if (y1 > k->wy1) k->wy1 = y1;
}

void epd_pk_w_begin(const epd_engine *e)
{
    epd_pk_w_rows(e, 0, EPD_H);
}

void epd_pk_w_sync(const epd_engine *e)
{
    epd_pk *k = e->pk;
    if (!k || !k->wdirty) return;
    const size_t row = (size_t)EPD_PK_S * sizeof *k->W;
    if (k->wfd >= 0) epd_dmabuf_sync(k->wfd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE, (size_t)k->wy0 * row, (size_t)(k->wy1 - k->wy0) * row);
    k->wdirty = 0;
}

/* finish_mask: row y may hold busy value b's pixels */
static inline void pk_mark(epd_pk *k, int b, int y)
{
    if (b > 0) __atomic_fetch_or(&k->rows[b - 1][y >> 6], 1ull << (y & 63), __ATOMIC_RELAXED);
}

/* the packed side of one pixel's fields (W from busy, T, D's level) */
static void pk_put(epd_pk *k, size_t i, unsigned what, int level, int target, int busy, int op, int dc, int stale)
{
    size_t q = epd_pk_at(i);
    uint32_t d = k->D[q];
    if (k->W[q] == EPD_PK_FIN) d = (d & ~31u) | (k->T[q] & 31u);       /* (resolved: D's level is T's) */
    int b = EPD_PK_BUSY(k->W[q]);
    if (what & EPD_F_LEVEL) d = (d & ~31u) | (uint32_t)(level & 31);
    if (what & EPD_F_OP) d = (d & ~(63u << 5)) | (uint32_t)(op & 63) << 5;
    if (what & EPD_F_STALE) d = (d & ~(1u << 11)) | (uint32_t)(stale & 1) << 11;
    if (what & EPD_F_DC) d = (d & 0xFFFFu) | (uint32_t)(uint16_t)dc << 16;
    if (what & EPD_F_TARGET) k->T[q] = (uint8_t)((k->T[q] & 0xE0u) | (target & 31));     /* (bits 5..7 kept) */
    if (what & EPD_F_BUSY) b = busy;
    k->D[q] = d;
    k->W[q] = epd_pk_w(b, k->T[q], (int)(d & 31));
    pk_mark(k, b, (int)(i / EPD_W));
}

void epd_px_put(epd_engine *e, size_t i, unsigned what, int level, int target, int busy, int op, int dc, int stale)
{
    if (e->level) {
        if (what & EPD_F_LEVEL) e->level[i] = (uint8_t)level;
        if (what & EPD_F_TARGET) e->target[i] = (uint8_t)target;
        if (what & EPD_F_BUSY) e->busy[i] = (uint8_t)busy;
        if (what & EPD_F_OP) e->op[i] = (uint8_t)op;
        if (what & EPD_F_DC) e->dc[i] = (int16_t)dc;
        if (what & EPD_F_STALE) e->stale[i] = (uint8_t)stale;
    }
    if (e->pk) {
        if (e->cst == 1) epd_pk_w_rows(e, (int)(i / EPD_W), (int)(i / EPD_W) + 1);
        pk_put(e->pk, i, what, level, target, busy, op, dc, stale);
    }
}

void epd_state_get(const epd_engine *e, uint8_t *level, uint8_t *target, uint8_t *busy, uint8_t *op, int16_t *dc, uint8_t *stale)
{
    if (e->level) {
        if (level) memcpy(level, e->level, EPD_NPX);
        if (target) memcpy(target, e->target, EPD_NPX);
        if (busy) memcpy(busy, e->busy, EPD_NPX);
        if (op) memcpy(op, e->op, EPD_NPX);
        if (dc) memcpy(dc, e->dc, EPD_NPX * sizeof *dc);
        if (stale) memcpy(stale, e->stale, EPD_NPX);
        return;
    }
    const epd_pk *k = e->pk;
    for (int y = 0; y < EPD_H; y++) {
        const uint32_t *D = k->D + (size_t)y * EPD_PK_S;
        const uint16_t *W = k->W + (size_t)y * EPD_PK_S;
        const uint8_t *T = k->T + (size_t)y * EPD_PK_S;
        size_t o = (size_t)y * EPD_W;
        for (int x = 0; x < EPD_W; x++) {
            uint32_t d = D[x];
            if (level) level[o + x] = (uint8_t)epd_pk_lev(d, W[x], T[x]);
            if (target) target[o + x] = T[x] & 31;
            if (busy) busy[o + x] = (uint8_t)EPD_PK_BUSY(W[x]);
            if (op) op[o + x] = (uint8_t)((d >> 5) & 63);
            if (dc) dc[o + x] = (int16_t)(d >> 16);
            if (stale) stale[o + x] = (uint8_t)((d >> 11) & 1);
        }
    }
}

void epd_state_set(epd_engine *e, const uint8_t *level, const uint8_t *target, const uint8_t *busy, const uint8_t *op,
                   const int16_t *dc, const uint8_t *stale)
{
    if (e->level) {
        if (level) memcpy(e->level, level, EPD_NPX);
        if (target) memcpy(e->target, target, EPD_NPX);
        if (busy) memcpy(e->busy, busy, EPD_NPX);
        if (op) memcpy(e->op, op, EPD_NPX);
        if (dc) memcpy(e->dc, dc, EPD_NPX * sizeof *dc);
        if (stale) memcpy(e->stale, stale, EPD_NPX);
    }
    if (!e->pk) return;
    unsigned what = (level ? EPD_F_LEVEL : 0) | (target ? EPD_F_TARGET : 0) | (busy ? EPD_F_BUSY : 0) |
                    (op ? EPD_F_OP : 0) | (dc ? EPD_F_DC : 0) | (stale ? EPD_F_STALE : 0);
    if (e->cst == 1) epd_pk_w_begin(e);
    for (size_t i = 0; i < EPD_NPX; i++)
        pk_put(e->pk, i, what, level ? level[i] : 0, target ? target[i] : 0, busy ? busy[i] : 0,
               op ? op[i] : 0, dc ? dc[i] : 0, stale ? stale[i] : 0);
}

void epd_state_fill(epd_engine *e, unsigned what, int v)
{
    if (e->level) {
        if (what & EPD_F_LEVEL) memset(e->level, v, EPD_NPX);
        if (what & EPD_F_TARGET) memset(e->target, v, EPD_NPX);
        if (what & EPD_F_BUSY) memset(e->busy, v, EPD_NPX);
        if (what & EPD_F_OP) memset(e->op, v, EPD_NPX);
        if (what & EPD_F_STALE) memset(e->stale, v, EPD_NPX);
        if (what & EPD_F_DC) for (size_t i = 0; i < EPD_NPX; i++) e->dc[i] = (int16_t)v;
    }
    if (!e->pk) return;
    if (e->cst == 1) epd_pk_w_begin(e);
    for (size_t i = 0; i < EPD_NPX; i++) pk_put(e->pk, i, what, v, v, v, v, v, v);
}

void epd_state_reset(epd_engine *e)
{
    epd_state_fill(e, EPD_F_BUSY | EPD_F_DC, 0);
    epd_state_fill(e, EPD_F_STALE, 1);
}

/* the knobs' environment, at each epd_init (the first one resolves them:
 * a knob already set, by its owner or the daemon, keeps its value); the only
 * getenv of the engine */
static void knobs_env(void)
{
    epd_knobs_env(eng_knobs, KN_N);
    const char *v = getenv("GLASSINE_DMABUF_PARTIAL");
    if (v && atoi(v) == 0 && __atomic_load_n(&epd_dmabuf_partial, __ATOMIC_RELAXED) < 0)
        __atomic_store_n(&epd_dmabuf_partial, 0, __ATOMIC_RELAXED);
    v = getenv("GLASSINE_CLAIM_STATE_W");
    pk_w_malloc = v && !strcmp(v, "malloc");
}

int epd_init(epd_engine *e, const uint8_t *wbf, size_t n, int celsius)
{
    knobs_env();
    memset(e, 0, sizeof *e);
    if (epd_wbf_load(&e->wbf, wbf, n)) return -1;
    e->celsius = celsius;
    e->next_id = 1;
    e->matched_from = 2;
    e->cst = claim_state_of();
    if (epd_state_alloc(e)) { fprintf(stderr, "epd-engine: no memory for the state\n"); state_free(e); return -1; }
    epd_template(e->tpl);
    epd_assume(e, 30);
    epd_state_fill(e, EPD_F_STALE, 1);          /* unknown: the first flash drives everything */
    return 0;
}

/* ---- the target buffers' pool (the epdd CPU work) -------------
 * A full-panel target is 2.6 MB. musl's malloc (Alpine) maps every block
 * above ~128 KB with its own mmap and unmaps it at free: each update paid
 * ~640 first-touch page faults (the kernel zeroing every page) on the main
 * thread and an munmap (a TLB shootdown) wherever it was dropped -- for a
 * superseded one inside submit, under the stream lock. Freed blocks of at
 * least TPOOL_MIN bytes are kept (up to epd_target_pool of them, the largest
 * win) and handed out again, best fit. Plain malloc blocks: a free of one
 * is still right, epd_target_free of any malloc block too. Every user writes
 * all n bytes before reading them (levels_of over every row, pend_tg's fetch
 * or memset), so the output is the same byte for byte */
#define TPOOL_MAX 8
#define TPOOL_MIN ((size_t)128 * 1024)
static struct { void *p; size_t cap; } tpool[TPOOL_MAX];
static int ntpool;
static pthread_mutex_t tpool_mu = PTHREAD_MUTEX_INITIALIZER;
static unsigned long tpool_hit, tpool_miss;

uint8_t *epd_target_alloc(size_t n)
{
    if (!n) n = 1;
    if (n >= TPOOL_MIN) {
        pthread_mutex_lock(&tpool_mu);
        int b = -1;
        for (int i = 0; i < ntpool; i++)
            if (tpool[i].cap >= n && (b < 0 || tpool[i].cap < tpool[b].cap)) b = i;
        if (b >= 0) {
            void *p = tpool[b].p;
            tpool[b] = tpool[--ntpool];
            tpool_hit++;
            pthread_mutex_unlock(&tpool_mu);
            return p;
        }
        tpool_miss++;
        pthread_mutex_unlock(&tpool_mu);
    }
    return malloc(n);
}

void epd_target_free(void *p)
{
    if (!p) return;
    size_t cap = malloc_usable_size(p);
    if (cap >= TPOOL_MIN) {
        void *out = p;
        pthread_mutex_lock(&tpool_mu);
        int lim = epd_target_pool < 0 ? 0 : epd_target_pool > TPOOL_MAX ? TPOOL_MAX : epd_target_pool;
        while (ntpool > lim) free(tpool[--ntpool].p);  /* (the limit lowered) */
        if (ntpool < lim) { tpool[ntpool].p = p; tpool[ntpool].cap = cap; ntpool++; out = NULL; }
        else if (lim) {                         /* full: it replaces the smallest, if larger */
            int k = 0;
            for (int i = 1; i < ntpool; i++) if (tpool[i].cap < tpool[k].cap) k = i;
            if (tpool[k].cap < cap) { out = tpool[k].p; tpool[k].p = p; tpool[k].cap = cap; }
        }
        pthread_mutex_unlock(&tpool_mu);
        free(out);
        return;
    }
    free(p);
}

void epd_target_pool_stats(unsigned long *hit, unsigned long *miss, int *kept)
{
    pthread_mutex_lock(&tpool_mu);
    *hit = tpool_hit; *miss = tpool_miss; *kept = ntpool;
    tpool_hit = tpool_miss = 0;
    pthread_mutex_unlock(&tpool_mu);
}

static void matched_free(epd_engine *e);
static void job_free(epd_engine *e);

/* stage 5: a pending's target levels on the CPU. A GPU target
 * (gt) is fetched once and kept beside its handle (the GPU passes go on
 * reading the handle); a fetch that fails (the GPU hung, its contents lost)
 * gives white, as gpe_lost assumes for the levels (a full flash follows) */
static const uint8_t *pend_tg(epd_engine *e, epd_pend *p)
{
    if (p->target) return p->target;
    size_t n = (size_t)p->r.w * p->r.h;
    if (!(p->target = epd_target_alloc(n))) { fprintf(stderr, "epd-engine: no memory for a target\n"); abort(); }
    if (!e->tops || !e->tops->fetch || e->tops->fetch(e->tops->ctx, p->gt, p->r, p->target)) memset(p->target, 30, n);
    return p->target;
}

/* the pending's target is gone (both halves) */
static void pend_drop(epd_engine *e, epd_pend *p)
{
    epd_target_free(p->target);
    p->target = NULL;
    if (p->gt && e->tops && e->tops->release) e->tops->release(e->tops->ctx, p->gt);
    p->gt = 0;
}

void epd_set_target_ops(epd_engine *e, const epd_target_ops *ops) { e->tops = ops; }

void epd_pend_targets_cpu(epd_engine *e)
{
    for (int i = 0; i < e->npend; i++) {
        epd_pend *p = &e->pend[i];
        if (!p->gt) continue;
        pend_tg(e, p);
        if (e->tops && e->tops->release) e->tops->release(e->tops->ctx, p->gt);
        p->gt = 0;
    }
}

void epd_free(epd_engine *e)
{
    for (int f = 0; f < 2; f++)
        for (int m = 0; m < 16; m++)
            for (int t = 0; t < 64; t++)
                for (int c = 0; c < EPD_LUT_CUTS; c++) free(e->lutcache[f][m][t][c]);
    for (int k = 0; k < 9; k++) free(e->heallut[k]);
    matched_free(e);
    job_free(e);
    for (int i = 0; i < e->npend; i++) pend_drop(e, &e->pend[i]);
    state_free(e);
}

void epd_set_cut(epd_engine *e, int cut)
{
    e->cut = cut < 0 ? 0 : cut > EPD_MAX_CUT ? EPD_MAX_CUT : cut;
}

void epd_set_matched(epd_engine *e, int on)
{
    e->matched = on != 0;
}

void epd_set_nonblock(epd_engine *e, int on)
{
    e->nonblock = on < 0 ? 0 : on > 2 ? 2 : on;
}

void epd_set_matched_rule(epd_engine *e, int from, int cap)
{
    e->matched_from = from < 2 ? 2 : from;
    e->matched_cap = cap < 0 ? 0 : cap;
}

void epd_set_model(epd_engine *e, int optical, int track)
{
    e->model = optical < 0 ? 0 : optical > 2 ? 2 : optical;
    e->op_track = track != 0;
}

void epd_set_heal(epd_engine *e, int min, int from)
{
    e->heal_min = min < 0 ? 0 : min;
    e->heal_from = from < 2 ? 2 : from;
}

void epd_set_heal_len(epd_engine *e, int n)
{
    e->heal_len = n < 0 ? 0 : n > 8 ? 8 : n;
}

void epd_set_snap(epd_engine *e, int on)
{
    e->snap = on != 0;
}

void epd_assume(epd_engine *e, int level5)
{
    if (e->level) memset(e->level, level5 & 31, EPD_NPX);
    if (e->pk) {                                /* (D's level; W's too where in flight: what the drive reads) */
        epd_pk *k = e->pk;
        if (e->cst == 1) epd_pk_w_begin(e);
        const uint32_t l = (uint32_t)(level5 & 31);
        for (size_t q = 0; q < (size_t)EPD_PK_S * EPD_H; q++) {
            k->D[q] = (k->D[q] & ~31u) | l;
            if (k->W[q] == EPD_PK_FIN) k->W[q] = EPD_PK_IDLE;          /* (D's level valid again) */
            else if (!EPD_PK_ISIDLE(k->W[q])) k->W[q] = (uint16_t)((k->W[q] & ~31u) | l);
        }
    }
    e->op_ready = 0;
}

static int clip(epd_rect *r)
{
    if (r->x < 0) { r->w += r->x; r->x = 0; }
    if (r->y < 0) { r->h += r->y; r->y = 0; }
    if (r->x + r->w > EPD_W) r->w = EPD_W - r->x;
    if (r->y + r->h > EPD_H) r->h = EPD_H - r->y;
    return r->w > 0 && r->h > 0;
}

static int overlap(epd_rect a, epd_rect b);
/* a step of a scroll for non-blocking payback: a partial A2 / A2_OR_DU, cut
 * or not (the uncut A2 is the ladder's slowest step: it queued behind the
 * fast steps' waiting rests, 46 order waits in the replay) */
static int a2_step(int mode, int full)
{
    return !full && (EPD_MODE_BASE(mode) == EPD_A2 || EPD_MODE_BASE(mode) == EPD_A2_OR_DU);
}

/* grey -> 5-bit level ((g >> 4) * 2): a plain loop the compiler vectorizes */
__attribute__((optimize("O3")))
static void levels_of(uint8_t *restrict d, const uint8_t *restrict s, int n)
{
    for (int x = 0; x < n; x++) d[x] = (uint8_t)((s[x] & 0xF0) >> 3);
}

uint8_t *epd_target_make(epd_rect *r, const uint8_t *grey, int stride)
{
    if (!clip(r)) return NULL;
    uint8_t *t = epd_target_alloc((size_t)r->w * r->h);
    if (!t) return NULL;
    for (int y = 0; y < r->h; y++) levels_of(t + (size_t)y * r->w, grey + (size_t)(r->y + y) * stride + r->x, r->w);
    return t;
}

/* epd_target_make with its rows in parts on a runner (the same bytes) */
struct tm_par { uint8_t *t; epd_rect r; const uint8_t *grey; int stride, n; };
static void tm_part(void *arg, int k)
{
    struct tm_par *p = arg;
    int y0 = (int)((long)p->r.h * k / p->n), y1 = (int)((long)p->r.h * (k + 1) / p->n);
    for (int y = y0; y < y1; y++) levels_of(p->t + (size_t)y * p->r.w, p->grey + (size_t)(p->r.y + y) * p->stride + p->r.x, p->r.w);
}
uint8_t *epd_target_make_par(epd_rect *r, const uint8_t *grey, int stride,
                             void (*run)(void *, void (*)(void *, int), void *, int), void *ctx, int parts)
{
    if (!clip(r)) return NULL;
    uint8_t *t = epd_target_alloc((size_t)r->w * r->h);
    if (!t) return NULL;
    struct tm_par p = { t, *r, grey, stride, parts < 1 ? 1 : parts > r->h ? r->h : parts };
    run(ctx, tm_part, &p, p.n);
    return t;
}

uint32_t epd_submit(epd_engine *e, epd_rect r, int mode, int full, const uint8_t *grey, int stride)
{
    uint8_t *t = epd_target_make(&r, grey, stride);
    return t ? epd_submit_target(e, r, mode, full, t, grey, stride) : 0;
}

static uint32_t submit(epd_engine *e, epd_rect r, int mode, int full, uint8_t *t, int gt, const uint8_t *grey, int stride);

uint32_t epd_submit_target(epd_engine *e, epd_rect r, int mode, int full, uint8_t *t, const uint8_t *grey, int stride)
{
    return submit(e, r, mode, full, t, 0, grey, stride);
}

uint32_t epd_submit_gt(epd_engine *e, epd_rect r, int mode, int full, int gt, const uint8_t *grey, int stride)
{
    return submit(e, r, mode, full, NULL, gt, grey, stride);
}

/* the new update's own target: t (CPU) or gt (GPU), dropped */
static void drop_new(epd_engine *e, uint8_t *t, int gt)
{
    epd_target_free(t);
    if (gt && e->tops && e->tops->release) e->tops->release(e->tops->ctx, gt);
}

static uint32_t submit(epd_engine *e, epd_rect r, int mode, int full, uint8_t *t, int gt, const uint8_t *grey, int stride)
{
    if (!clip(&r) || e->npend >= EPD_MAX_PENDING) { drop_new(e, t, gt); return 0; }
    int flash = full == 2;
    full = full != 0;
    /* non-blocking payback: a cut-A2 step takes in every update that waits for
     * payback pixels (kept) and overlaps it — its rect grows to the union, the
     * content from grey (the newest image) — instead of queueing behind it
     * (device run: steps of several rects rarely contained the
     * kept one, so every later step waited: 9621 order waits, a still image) */
    if (e->nonblock && a2_step(mode, full))
        for (int grew = 1; grew;) {
            grew = 0;
            for (int i = 0; i < e->npend; i++) {
                epd_pend *q = &e->pend[i];
                /* also a not yet started cut-A2 one: the grown rect would else
                 * queue behind it (an earlier band of the same step) */
                if (!(q->kept || a2_step(q->mode, q->full)) || q->job ||
                    EPD_MODE_BASE(q->mode) != EPD_MODE_BASE(mode) || !overlap(q->r, r)) continue;
                int x0 = q->r.x < r.x ? q->r.x : r.x, y0 = q->r.y < r.y ? q->r.y : r.y;
                int x1 = q->r.x + q->r.w > r.x + r.w ? q->r.x + q->r.w : r.x + r.w;
                int y1 = q->r.y + q->r.h > r.y + r.h ? q->r.y + q->r.h : r.y + r.h;
                epd_rect u = { x0, y0, x1 - x0, y1 - y0 };
                uint8_t *ut = epd_target_alloc((size_t)u.w * u.h);
                if (!ut) break;
                for (int y = 0; y < u.h; y++) levels_of(ut + (size_t)y * u.w, grey + (size_t)(u.y + y) * stride + u.x, u.w);
                drop_new(e, t, gt); pend_drop(e, q);
                t = ut; gt = 0; r = u;
                memmove(q, q + 1, (size_t)(e->npend - i - 1) * sizeof *q);
                e->npend--;
                e->absorbed++;
                grew = 1;
                break;
            }
        }
    for (int i = e->npend - 1; i >= 0; i--) {
        epd_pend *q = &e->pend[i];
        int same = q->mode == mode || (e->nonblock && a2_step(q->mode, q->full) && a2_step(mode, full) &&
                                       EPD_MODE_BASE(q->mode) == EPD_MODE_BASE(mode));   /* cut A2: any length */
        if (!same || q->job || q->full != full || q->flash != flash || q->r.x < r.x || q->r.y < r.y ||
            q->r.x + q->r.w > r.x + r.w || q->r.y + q->r.h > r.y + r.h) continue;
        int between = 0;
        for (int j = i + 1; j < e->npend && !between; j++) between = overlap(e->pend[j].r, q->r);
        if (between) continue;
        pend_drop(e, q);
        memmove(q, q + 1, (size_t)(e->npend - i - 1) * sizeof *q);
        e->npend--;
        e->superseded++;
    }
    /* merge: a pen stroke arrives as a stream of small touching pieces; queued one by
     * one, each waits for the previous (they overlap) and the line oozes out in blobs.
     * So grow the newest still-pending update of the same kind that the piece touches
     * into the union (its content taken from grey, i.e. the newest image), unless a
     * later pending update overlaps the union (ordering) or the union gets large. Only
     * pieces of the same buffer: its union content then is simply the newest image. */
    for (int i = e->npend - 1; i >= 0; i--) {
        epd_pend *q = &e->pend[i];
        if (q->mode != mode || q->job || q->full != full || q->flash != flash || q->src != grey || q->stride != stride) continue;
        epd_rect g = { q->r.x - EPD_MERGE_GAP, q->r.y - EPD_MERGE_GAP,
                       q->r.w + 2 * EPD_MERGE_GAP, q->r.h + 2 * EPD_MERGE_GAP };
        if (!overlap(g, r)) continue;
        int x0 = q->r.x < r.x ? q->r.x : r.x, y0 = q->r.y < r.y ? q->r.y : r.y;
        int x1 = q->r.x + q->r.w > r.x + r.w ? q->r.x + q->r.w : r.x + r.w;
        int y1 = q->r.y + q->r.h > r.y + r.h ? q->r.y + q->r.h : r.y + r.h;
        epd_rect u = { x0, y0, x1 - x0, y1 - y0 };
        if ((long)u.w * u.h > EPD_MERGE_MAX) break;
        int later = 0;
        for (int j = i + 1; j < e->npend && !later; j++) later = overlap(e->pend[j].r, u);
        if (later) break;
        uint8_t *ut = epd_target_alloc((size_t)u.w * u.h);
        if (!ut) break;
        for (int y = 0; y < u.h; y++) levels_of(ut + (size_t)y * u.w, grey + (size_t)(u.y + y) * stride + u.x, u.w);
        pend_drop(e, q); drop_new(e, t, gt);
        q->r = u; q->target = ut; q->bvalid = 0;
        e->merged++;
        return q->id;
    }
    epd_pend *p = &e->pend[e->npend++];
    p->r = r; p->mode = mode; p->full = full; p->target = t; p->gt = gt; p->id = e->next_id++;
    p->src = grey; p->stride = stride; p->kept = 0; p->flash = flash; p->bvalid = 0; p->job = 0;
    if (!e->next_id) e->next_id = 1;
    return p->id;
}

static int overlap(epd_rect a, epd_rect b)
{
    return a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
}

/* The DU frame cut:
 * DU keeps its first nframes - cut frames,
 * and on the Note Air 1 the last kept frame is
 * all zero. Cut 0 = the WBF waveform unchanged. Keyed by cut, so an update
 * that is running keeps the table it started with. */
/* a2len: an A2 shortened to a2len frames (EPD_MODE_A2LEN), 0 = uncut */
static const uint8_t *lut(epd_engine *e, int full, int mode, int a2len, int *nframes)
{
    int t = epd_wbf_temp_index(&e->wbf, e->celsius);
    int c = mode == EPD_DU ? e->cut : 0;
    if (mode < 0 || mode >= 16 || t < 0 || t >= 64) return NULL;
    if (mode == EPD_A2 && a2len > 0) {
        int n0;
        if (!lut(e, full, mode, 0, &n0)) return NULL;
        c = n0 - a2len;
        if (c < 0) c = 0;
        if (c > EPD_LUT_CUTS - 1) c = EPD_LUT_CUTS - 1;
    }
    if (!e->lutcache[full][mode][t][c]) {
        uint8_t *f;
        int n = epd_wbf_frames(&e->wbf, mode, t, &f);
        if (n <= 0) return NULL;
        if (!full)      /* a partial update: no grey change -> no drive */
            for (int k = 0; k < n; k++)
                for (int g = 0; g < 32; g++) f[k * 1024 + g * 33] = 0;
        if (c && n - c >= 1) {
            n -= c;
            memset(f + (size_t)(n - 1) * 1024, 0, 1024);
        }
        e->lutcache[full][mode][t][c] = f;
        e->lutframes[full][mode][t][c] = n;
    }
    *nframes = e->lutframes[full][mode][t][c];
    return e->lutcache[full][mode][t][c];
}

static int bw_pix(uint8_t tg, uint8_t lv)
{
    return (tg == 0 || tg == 30) && (lv == 0 || lv == 30);
}

/* net drive frames per transition of a LUT (the DC record) */
static void net_of(const uint8_t *l, int nf, int8_t net[1024])
{
    for (int i = 0; i < 1024; i++) {
        int n = 0;
        for (int k = 0; k < nf; k++) { int ph = l[k * 1024 + i] & 3; n += ph == 1 ? -1 : ph == 2 ? 1 : 0; }
        net[i] = (int8_t)(n < -127 ? -127 : n > 127 ? 127 : n);
    }
}

/* the panel model (epd_set_model): op_r for this temperature; the record from
 * the levels when it starts, rescaled when op_r changes */
static void op_prep(epd_engine *e)
{
    int t = epd_wbf_temp_index(&e->wbf, e->celsius), n0;
    if (e->op_ready && t == e->op_tidx) return;
    const uint8_t *l = lut(e, 0, EPD_A2, 0, &n0);
    int R = 9;
    if (l) { int8_t net[1024]; net_of(l, n0, net); R = net[30 << 5 | 0]; }
    if (R < 1 || R > 60) R = 9;
    if (e->level) {
        if (e->op_ready && e->op_r)
            for (size_t i = 0; i < EPD_NPX; i++) e->op[i] = (uint8_t)(e->op[i] * R / e->op_r);
        else
            for (size_t i = 0; i < EPD_NPX; i++) e->op[i] = (uint8_t)(e->level[i] >= 30 ? R : R * e->level[i] / 30);
    }
    if (e->pk) {                                /* (D's op; the padding columns too: never read) */
        uint32_t *D = e->pk->D;
        const uint8_t *T = e->pk->T;
        const int sc = e->op_ready && e->op_r;
        for (size_t q = 0; q < (size_t)EPD_PK_S * EPD_H; q++) {
            /* (stagger_state=lanes: a lane the stagger holds (arm / evict in
             * T) is its D, not ours: left as it is, as the runs' exit writes
             * back the stagger's own op over a rescale) */
            if (T[q] & (EPD_PK_ARM | EPD_PK_EVICT)) continue;
            uint32_t d = D[q], o = (d >> 5) & 63, l = d & 31;
            uint8_t v = sc ? (uint8_t)(o * R / e->op_r) : (uint8_t)(l >= 30 ? R : R * (int)l / 30);
            D[q] = (d & ~(63u << 5)) | (uint32_t)(v & 63) << 5;
        }
    }
    e->op_r = R; e->op_tidx = t; e->op_ready = 1;
}

/* healing (epd_set_heal), one run per step: a healed pixel's level holds
 * 2k - 1 while it is in flight (odd: never a real level, those are even) and
 * its target the real level, so the run's table drives k frames toward black
 * (index 0 << 5 | 2k - 1) or white (30 << 5 | 2k - 1) with the uncut A2's
 * most used phase toward each; hmax + 1 frames (the last one zero, as a cut A2
 * ends); finishing restores the level from the target */
static const uint8_t *heal_lut(epd_engine *e, int hmax, int *nf)
{
    if (hmax < 1 || hmax > 8) return NULL;
    if (!e->heallut[hmax]) {
        int n0;
        const uint8_t *a = lut(e, 0, EPD_A2, 0, &n0);
        if (!a) return NULL;
        int cw[4] = { 0 }, cb[4] = { 0 }, pw = 0, pb = 0;
        for (int f = 0; f < n0; f++) { cw[a[f * 1024 + (30 << 5 | 0)] & 3]++; cb[a[f * 1024 + (0 << 5 | 30)] & 3]++; }
        for (int v = 1; v < 4; v++) { if (cw[v] > cw[pw]) pw = v; if (cb[v] > cb[pb]) pb = v; }
        if (pw != 2 || pb != 1) return NULL;    /* heal_row's sign: white +1, black -1 per frame (net_of) */
        uint8_t *h = calloc((size_t)hmax + 1, 1024);
        if (!h) return NULL;
        for (int k = 1; k <= hmax; k++)
            for (int f = 0; f < k; f++) { h[f * 1024 + (30 << 5 | (2 * k - 1))] = (uint8_t)pw; h[f * 1024 + (2 * k - 1)] = (uint8_t)pb; }
        e->heallut[hmax] = h;
    }
    *nf = hmax + 1;
    return e->heallut[hmax];
}

/* a row of healing: the idle black/white pixels that stay (target = level)
 * and sit >= hmin frames short (the model) join run own for k = min(short,
 * hmax) frames; the model and the DC record move by k. Branch-free
 * (vectorizes). Marks the columns in col; returns the pixels taken. */
__attribute__((optimize("O3")))
static int heal_row(uint8_t *restrict b, uint8_t *restrict et, uint8_t *restrict lv, uint8_t *restrict op,
                    int16_t *restrict dc, const uint8_t *restrict tg, uint8_t *restrict col, int n, uint8_t own,
                    int R, int hmin, int hmax)
{
    const uint8_t R8 = (uint8_t)R, hmin8 = (uint8_t)hmin, hmax8 = (uint8_t)hmax;
    uint8_t any = 0;
    for (int x = 0; x < n; x++) {
        uint8_t l = lv[x], o = op[x], bb = b[x];
        uint8_t w = l == 30, bl = l == 0;
        uint8_t def = w ? (uint8_t)(R8 - o) : o;
        uint8_t ok = (uint8_t)((bb == 0) & (tg[x] == l) & (w | bl) & (def >= hmin8));
        uint8_t k = def > hmax8 ? hmax8 : def;
        uint8_t m = (uint8_t)-ok, ku = (uint8_t)(k & (uint8_t)-(ok & w)), kd = (uint8_t)(k & (uint8_t)-(ok & bl));
        op[x] = (uint8_t)(o + ku - kd);                     /* k <= short: within 0..R */
        dc[x] = (int16_t)(dc[x] + (int16_t)ku - (int16_t)kd);   /* (|dc| <= 30000 + 8: no wrap) */
        b[x] = (uint8_t)((own & m) | (bb & ~m));
        et[x] = (uint8_t)((l & m) | (et[x] & ~m));
        lv[x] = (uint8_t)(((2 * k - 1) & m) | (l & ~m));
        col[x] |= ok;
        any |= ok;
    }
    return any;
}

/* epd_set_snap, a row before a full waveform's claim: an idle pixel recorded
 * at black (white) whose target is white (black) and whose model is nearer
 * white (black) is there already: its level is snapped, the claim leaves it
 * alone (partial) and the heal tops it up. Toward the far rail the full swing
 * stays (snapping that too swung half-way pixels -21 where nothing drove them:
 * test/epd-matched-test, mean |dc| 2.3 -> 13.4). Branch-free; returns the
 * pixels snapped */
__attribute__((optimize("O3")))
static int snap_row(const uint8_t *restrict b, uint8_t *restrict lv, const uint8_t *restrict tg, const uint8_t *restrict op, int n, int R)
{
    int k = 0;
    for (int x = 0; x < n; x++) {
        uint8_t l = lv[x], t = tg[x], idle = b[x] == 0;
        int o2 = 2 * op[x];
        uint8_t up = (uint8_t)(idle & (l == 0) & (t == 30) & (o2 > R)), dn = (uint8_t)(idle & (l == 30) & (t == 0) & (o2 < R));
        lv[x] = up ? 30 : dn ? 0 : l;
        k += up | dn;
    }
    return k;
}

/* the model for a row of a split claim (no matched cuts): oa's pixels (all
 * black/white -> black/white) move by their net (an A2: one of four values,
 * as dc_row_bw) or reach their target; branch-free. od's (DU) reach theirs:
 * op_row_rail */
__attribute__((optimize("O3")))
static void op_row_bw(uint8_t *restrict op, const uint8_t *restrict b, const uint8_t *restrict tg,
                      const uint8_t *restrict lv, int n, uint8_t oa, const int8_t *net, int aclamp, int R)
{
    const int n00 = net[0 << 5 | 0], n0w = net[0 << 5 | 30], nw0 = net[30 << 5 | 0], nww = net[30 << 5 | 30];
    for (int x = 0; x < n; x++) {
        int v = tg[x] == 0 ? (lv[x] == 0 ? n00 : n0w) : (lv[x] == 0 ? nw0 : nww);
        int a = op[x] + v;
        a = a < 0 ? 0 : a > R ? R : a;
        int rl = tg[x] == 30 ? R : 0;
        op[x] = b[x] == oa ? (uint8_t)(aclamp ? a : rl) : op[x];
    }
}
static void op_row_rail(uint8_t *restrict op, const uint8_t *restrict b, const uint8_t *restrict tg, int n, uint8_t od,
                        const uint8_t *rail)
{
    for (int x = 0; x < n; x++) if (b[x] == od) op[x] = rail[tg[x]];
}
/* after a matched claim's gather: the pixels it marked (cls 1) move by their
 * net (nrow), clamped; branch-free */
__attribute__((optimize("O3")))
static void op_row_net(uint8_t *restrict op, const uint8_t *restrict cls, const int8_t *restrict nrow, int n, int R)
{
    for (int x = 0; x < n; x++) {
        int a = op[x] + nrow[x];
        a = a < 0 ? 0 : a > R ? R : a;
        op[x] = cls[x] ? (uint8_t)a : op[x];
    }
}

/* a free run slot that this claim has not handed out yet */
static int slot_free(const epd_engine *e, int sa, int sd, const int *ms, int nms, const int *hs, int nhs)
{
    for (int s = 0; s < RUN_SLOTS; s++) {
        if (e->run[s].nframes || s == sa || s == sd) continue;
        int used = 0;
        for (int k = 0; k < nms && !used; k++) used = ms[k] == s;
        for (int k = 0; k < nhs && !used; k++) used = hs[k] == s;
        if (!used) return s;
    }
    return -1;
}

/* matched cuts (epd_set_matched): per (full, temperature) the A2's net drive
 * per transition at each cut (c frames off, as lut: length n0 - c), and for
 * each transition and DC record d (clamped to +-EPD_MT_D) the cut whose net
 * cancels d best (|d + net| smallest; a tie goes to the shorter) */
#define MT_PAD 16                       /* padded tables up to this many frames */
typedef struct {
    int n0, cmax;                       /* the uncut A2's frames; cuts 0..cmax (length >= 2) */
    int8_t net[EPD_LUT_CUTS][1024];
    uint8_t best[1024][2 * EPD_MT_D + 1];
    uint8_t *pad[EPD_LUT_CUTS][MT_PAD + 1];     /* cut c's table, zero frames up to n */
} mtab;

/* cut c of the A2 (as lut) padded with zero frames to m: an update's
 * matched runs all end on the same frame, so the GPU path packs their
 * finished pixels' state once, not once per length (device run:
 * 27 Mpx of state per scroll step) */
static const uint8_t *matched_lut(epd_engine *e, mtab *m, int full, int c, int len, int *nf)
{
    const uint8_t *l = lut(e, full, EPD_A2, c ? m->n0 - c : 0, nf);
    if (!l || len <= *nf || len > MT_PAD) return l;
    if (!m->pad[c][len]) {
        uint8_t *p = calloc((size_t)len, 1024);
        if (!p) return l;
        memcpy(p, l, (size_t)*nf * 1024);
        m->pad[c][len] = p;
    }
    *nf = len;
    return m->pad[c][len];
}

static void matched_free(epd_engine *e)
{
    for (int f = 0; f < 2; f++)
        for (int t = 0; t < 64; t++) {
            mtab *m = e->mtab[f][t];
            for (int c = 0; m && c < EPD_LUT_CUTS; c++)
                for (int k = 0; k <= MT_PAD; k++) free(m->pad[c][k]);
            free(m);
        }
}

static mtab *matched_tab(epd_engine *e, int full)
{
    int t = epd_wbf_temp_index(&e->wbf, e->celsius), n0;
    if (t < 0 || t >= 64 || !lut(e, full, EPD_A2, 0, &n0)) return NULL;
    if (e->mtab[full][t]) return e->mtab[full][t];
    mtab *m = calloc(1, sizeof *m);
    if (!m) return NULL;
    m->n0 = n0;
    m->cmax = n0 - 2 < EPD_LUT_CUTS - 1 ? n0 - 2 : EPD_LUT_CUTS - 1;
    if (m->cmax < 0) { free(m); return NULL; }
    for (int c = 0; c <= m->cmax; c++) {
        int nf;
        const uint8_t *l = lut(e, full, EPD_A2, c ? n0 - c : 0, &nf);
        if (!l) { free(m); return NULL; }
        net_of(l, nf, m->net[c]);
    }
    for (int i = 0; i < 1024; i++)
        for (int d = -EPD_MT_D; d <= EPD_MT_D; d++) {
            int bc = 0, bv = 1 << 30;
            for (int c = 0; c <= m->cmax; c++) {
                int v = d + m->net[c][i];
                if (v < 0) v = -v;
                if (v <= bv) { bv = v; bc = c; }        /* <=: the shorter wins a tie */
            }
            m->best[i][d + EPD_MT_D] = (uint8_t)bc;
        }
    e->mtab[full][t] = m;
    return m;
}

/* one row of a claim, branch-free so it vectorizes (the claims were
 * ~2/3 of the drive thread's engine time in a scroll): the pixels not in
 * flight that change (all, full) go to run oa, or (split) the black/white
 * -> black/white ones to oa and the rest to od (0: left unclaimed). Marks the claimed columns in
 * ca/cd; returns bit 0 = oa claimed in this row, bit 1 = od. */
__attribute__((optimize("O3")))
static int claim_row(uint8_t *restrict bz, uint8_t *restrict et, const uint8_t *restrict tg, const uint8_t *restrict lv,
                     uint8_t *restrict ca, uint8_t *restrict cd, int n, const uint8_t *restrict fm, int split, uint8_t oa, uint8_t od)
{
    uint8_t any_a = 0, any_d = 0, sp = split != 0;
    for (int x = 0; x < n; x++) {
        uint8_t t = tg[x], l = lv[x], b = bz[x];
        uint8_t chg = (uint8_t)((b == 0) & (fm[x] | (t != l)));
        uint8_t bw = (uint8_t)(((t == 0) | (t == 30)) & ((l == 0) | (l == 30)));
        uint8_t ta = (uint8_t)(chg & (bw | !sp)), td = (uint8_t)(chg & sp & !bw & (od != 0));
        uint8_t own = (uint8_t)((ta ? oa : 0) | (td ? od : 0));
        et[x] = own ? t : et[x];
        bz[x] = (uint8_t)(b | own);
        ca[x] |= ta; cd[x] |= td;
        any_a |= ta; any_d |= td;
    }
    return (any_a ? 1 : 0) | (any_d ? 2 : 0);
}

/* claim_row while a heal run is in flight: its pixels that change are taken
 * (preempted), their level restored from the target, the heal frames they
 * will not get credited back to the model and the DC record */
__attribute__((optimize("O3")))
static int claim_row_heal(uint8_t *restrict bz, uint8_t *restrict et, const uint8_t *restrict tg, uint8_t *restrict lv,
                     uint8_t *restrict op, int16_t *restrict dc, uint8_t *restrict ca, uint8_t *restrict cd, int n,
                     const uint8_t *restrict fm, int split, uint8_t oa, uint8_t od, uint8_t f0, uint8_t f1)
{
    uint8_t any_a = 0, any_d = 0, sp = split != 0;
    for (int x = 0; x < n; x++) {
        uint8_t t = tg[x], l = lv[x], b = bz[x];
        uint8_t h = b >= HEAL_B;                        /* healing: preempted if it changes */
        uint8_t lr = h ? et[x] : l;                     /* (its level holds 2k - 1, its target the level) */
        uint8_t chg = (uint8_t)(((b == 0) | h) & (fm[x] | (t != lr)));
        uint8_t bw = (uint8_t)(((t == 0) | (t == 30)) & ((lr == 0) | (lr == 30)));
        uint8_t ta = (uint8_t)(chg & (bw | !sp)), td = (uint8_t)(chg & sp & !bw & (od != 0));
        uint8_t own = (uint8_t)((ta ? oa : 0) | (td ? od : 0));
        uint8_t pre = (uint8_t)(h & (own != 0));
        uint8_t k = (uint8_t)((l + 1) >> 1), done = b == HEAL_B ? f0 : f1;
        uint8_t rem = k > done ? (uint8_t)(k - done) : 0;  /* heal frames it will not get: credited back */
        uint8_t ru = (uint8_t)(rem & (uint8_t)-(pre & (lr == 30))), rd = (uint8_t)(rem & (uint8_t)-(pre & (lr == 0)));
        uint8_t mp = (uint8_t)-pre, mo = (uint8_t)-(own != 0);
        op[x] = (uint8_t)(op[x] - ru + rd);
        dc[x] = (int16_t)(dc[x] - (int16_t)ru + (int16_t)rd);
        lv[x] = (uint8_t)((lr & mp) | (l & ~mp));
        et[x] = (uint8_t)((t & mo) | (et[x] & ~mo));
        bz[x] = (uint8_t)((own & mo) | (b & ~mo));
        ca[x] |= ta; cd[x] |= td;
        any_a |= ta; any_d |= td;
    }
    return (any_a ? 1 : 0) | (any_d ? 2 : 0);
}

/* the DC record for a row of a split claim (A2_OR_DU): run oa owns only
 * black/white -> black/white pixels (claim_row), so its net drive is one of
 * four values — a select, no table gather: vectorizes (the gather was ~2/3
 * of the claim, ~12 % of glassine-epdd-desktop's CPU in a scroll). Same
 * values as the gather. */
__attribute__((optimize("O3")))
static void dc_row_bw(int16_t *restrict dc, const uint8_t *restrict b, const uint8_t *restrict tg,
                      const uint8_t *restrict lv, int n, uint8_t oa, const int8_t *net)
{
    const int16_t n00 = net[0 << 5 | 0], n0w = net[0 << 5 | 30], nw0 = net[30 << 5 | 0], nww = net[30 << 5 | 30];
    for (int x = 0; x < n; x++) {
        int16_t v = tg[x] == 0 ? (lv[x] == 0 ? n00 : n0w) : (lv[x] == 0 ? nw0 : nww);
        int d = dc[x] + (b[x] == oa ? v : 0);
        dc[x] = (int16_t)(d < -30000 ? -30000 : d > 30000 ? 30000 : d);
    }
}

/* the 8 bytes at b hold a or (b != 0) c: SWAR zero-byte tests, exact */
static inline int has_byte2(const uint8_t *b, uint8_t a, uint8_t c)
{
    const uint64_t ones = 0x0101010101010101ull, highs = 0x8080808080808080ull;
    uint64_t w, v;
    memcpy(&w, b, sizeof w);
    v = w ^ ones * a;
    if ((v - ones) & ~v & highs) return 1;
    if (!c) return 0;
    v = w ^ ones * c;
    return ((v - ones) & ~v & highs) != 0;
}

/* nonblock 2: a row's black/white pixels in a preemptible payback run (pre[b]
 * != 0: its row of rn, the net drive of that run's frames still to come per
 * black/white transition) whose new target tg differs from where they head:
 * credited back, their level set to where they were heading, freed for the
 * claim that follows. Words with no busy pixel are skipped (SWAR). */
static long preempt_row(uint8_t *restrict bz, uint8_t *restrict et, const uint8_t *restrict tg, uint8_t *restrict lv,
                        uint8_t *restrict op, int16_t *restrict dc, int n, const uint8_t *pre, int8_t (*rn)[4], int opm, int R)
{
    long np = 0;
    for (int x = 0; x < n; x++) {
        if (!(x & 7) && x + 8 <= n) {
            uint64_t w;
            memcpy(&w, bz + x, sizeof w);
            if (!w) { x += 7; continue; }
        }
        uint8_t b = bz[x];
        if (!b || !pre[b]) continue;
        uint8_t t = tg[x], to = et[x], l = lv[x];
        if (t == to || (t != 0 && t != 30) || (to != 0 && to != 30) || (l != 0 && l != 30)) continue;
        int r = rn[pre[b]][(to ? 2 : 0) | (l ? 1 : 0)];
        if (opm) { int a = op[x] - r; op[x] = (uint8_t)(a < 0 ? 0 : a > R ? R : a); }
        int d = dc[x] - r;
        dc[x] = (int16_t)(d < -30000 ? -30000 : d > 30000 ? 30000 : d);
        lv[x] = to;
        bz[x] = 0;
        np++;
    }
    return np;
}

/* preempt_row for up to 4 owners in one pass: the pixels of payback runs
 * ob[k] (busy values; 255 = unused, no busy value is) in a row, the net drive
 * still to come of their frames r4[k] per (to, from) black/white
 * (rn[pre[ob]]). Branch-free (masks: the ternaries did not if-convert, gcc 16
 * x86), vectorizes: the scalar preempt_row (a lookup pre[b] per busy pixel, an
 * 8-byte zero skip that rarely skips in a scroll) was ~35 % of the claim
 * pass, the pass 37.6 % of glassine-epdd-desktop's CPU in a GNOME Software scroll
 * (device profile, 999 Hz). A pixel has one busy value: the same
 * state as the scalar pass (npre: 1..3 in test/epd-sched-bench). Returns the
 * pixels taken */
__attribute__((optimize("O3")))
static int preempt_owners_row(uint8_t *restrict bz, const uint8_t *restrict et, const uint8_t *restrict tg,
                              uint8_t *restrict lv, uint8_t *restrict op, int16_t *restrict dc, int n,
                              const uint8_t ob[4], const int8_t r4[4][4], int opm, int R)
{
    const uint8_t o0 = ob[0], o1 = ob[1], o2 = ob[2], o3 = ob[3];
    const int8_t a0 = r4[0][0], a1 = r4[0][1], a2 = r4[0][2], a3 = r4[0][3];
    const int8_t b0 = r4[1][0], b1 = r4[1][1], b2 = r4[1][2], b3 = r4[1][3];
    const int8_t c0 = r4[2][0], c1 = r4[2][1], c2 = r4[2][2], c3 = r4[2][3];
    const int8_t d0 = r4[3][0], d1 = r4[3][1], d2 = r4[3][2], d3 = r4[3][3];
    const uint8_t rr = (uint8_t)R, mo = (uint8_t)-(opm != 0);
    int np = 0;
    for (int x = 0; x < n; x++) {
        uint8_t b = bz[x], t = tg[x], to = et[x], l = lv[x];
        uint8_t tw = (uint8_t)-(to == 30), lw = (uint8_t)-(l == 30);
        uint8_t bw = (uint8_t)(((t == 0) | (t == 30)) & ((to == 0) | (to == 30)) & ((l == 0) | (l == 30)));
        uint8_t k0 = (uint8_t)-(b == o0), k1 = (uint8_t)-(b == o1), k2 = (uint8_t)-(b == o2), k3 = (uint8_t)-(b == o3);
        uint8_t m = (uint8_t)((k0 | k1 | k2 | k3) & -(uint8_t)((t != to) & bw));
        /* (8-bit lanes: r by masks, the model by saturating steps — exact for any op) */
        uint8_t ra = (uint8_t)((tw & ((lw & (uint8_t)a3) | (~lw & (uint8_t)a2))) | (~tw & ((lw & (uint8_t)a1) | (~lw & (uint8_t)a0))));
        uint8_t rb = (uint8_t)((tw & ((lw & (uint8_t)b3) | (~lw & (uint8_t)b2))) | (~tw & ((lw & (uint8_t)b1) | (~lw & (uint8_t)b0))));
        uint8_t rc = (uint8_t)((tw & ((lw & (uint8_t)c3) | (~lw & (uint8_t)c2))) | (~tw & ((lw & (uint8_t)c1) | (~lw & (uint8_t)c0))));
        uint8_t rd = (uint8_t)((tw & ((lw & (uint8_t)d3) | (~lw & (uint8_t)d2))) | (~tw & ((lw & (uint8_t)d1) | (~lw & (uint8_t)d0))));
        int8_t r = (int8_t)((ra & k0) | (rb & k1) | (rc & k2) | (rd & k3));
        uint8_t sg = (uint8_t)(r >> 7), up = (uint8_t)(r & ~sg), dn = (uint8_t)(-r & sg), o = op[x];
        uint8_t u1 = up < o ? up : o, a = (uint8_t)(o - u1);         /* op - r, clamped to 0..R */
        uint8_t hd = (uint8_t)(255 - a), d1 = dn < hd ? dn : hd;
        uint8_t s2 = (uint8_t)(a + d1);
        s2 = s2 > rr ? rr : s2;
        int16_t d = (int16_t)(dc[x] - r);
        d = d < -30000 ? -30000 : d;
        d = d > 30000 ? 30000 : d;
        uint8_t mop = m & mo;
        op[x] = (uint8_t)((s2 & mop) | (o & ~mop));
        int16_t m16 = (int16_t)(int8_t)m;
        dc[x] = (int16_t)((d & m16) | (dc[x] & ~m16));
        lv[x] = (uint8_t)((to & m) | (l & ~m));
        bz[x] = (uint8_t)(b & ~m);
        np += m & 1;
    }
    return np;
}

/* preempt_owners_row's rows that have nothing to take: no pixel of the
 * owners whose new target differs (one load of three rows, an OR: rows
 * between text lines skip the full pass) */
__attribute__((optimize("O3")))
static int preempt_any(const uint8_t *restrict bz, const uint8_t *restrict et, const uint8_t *restrict tg, int n,
                       const uint8_t ob[4])
{
    const uint8_t o0 = ob[0], o1 = ob[1], o2 = ob[2], o3 = ob[3];
    uint8_t any = 0;
    for (int x = 0; x < n; x++) {
        uint8_t b = bz[x];
        any |= (uint8_t)(((b == o0) | (b == o1) | (b == o2) | (b == o3)) & (tg[x] != et[x]));
    }
    return any;
}

/* the claim pass's table gather (epd_claim_pixels; all but a split claim
 * without matched cuts, dc_row_bw), pass A of three per row: the black/white -> black/white pixels of oa (in a
 * motion run's A2 steps nearly all claimed pixels: its frames are dithered)
 * have one of four nets (as dc_row_bw), and whether they want a matched cut,
 * and which record d they ask with, is arithmetic on the model, the record
 * and that net: branch-free, vectorizes. fl: 1 such a pixel (nrow its net),
 * 3 one that wants a cut (cq its (to, from) << 6 | d + EPD_MT_D, for the
 * plan's cut table), 2 the rest that oa or od own (the scalar gather), 0
 * none. The scalar loop was ~56 % of the pass, the pass 37.6 % of
 * glassine-epdd-desktop's CPU in a GNOME Software scroll (device profile;
 * want fires for ~90 % of those pixels with the payback defaults, so the
 * cut choice has to be on the fast side too) */
__attribute__((optimize("O3")))
static void gather_bw_row(const uint8_t *restrict b, const uint8_t *restrict tg, const uint8_t *restrict lv,
                          const uint8_t *restrict op, const int16_t *restrict dc, uint8_t *restrict fl,
                          uint8_t *restrict cq, int8_t *restrict nrow, int n, uint8_t oa, uint8_t od,
                          const int8_t *net, int mt, int model, int R)
{
    const int16_t n00 = net[0 << 5 | 0], n0w = net[0 << 5 | 30], nw0 = net[30 << 5 | 0], nww = net[30 << 5 | 30];
    const uint8_t umt = mt != 0, m0 = model == 0, m2 = model == 2, hod = od != 0;
    const int16_t r = (int16_t)R;
    for (int x = 0; x < n; x++) {
        uint8_t w = b[x], t = tg[x], l = lv[x];
        uint8_t tb = t == 30, lb = l == 30;
        uint8_t bw = (uint8_t)(((t == 0) | tb) & ((l == 0) | lb));
        uint8_t a = w == oa, e = (uint8_t)(a & bw);
        uint8_t s = (uint8_t)((a & !bw) | (hod & (w == od)));
        int16_t nn = tb ? (lb ? nww : nw0) : (lb ? n0w : n00);
        int16_t o = op[x], v = dc[x];
        int16_t need = tb ? (int16_t)(r - o) : o, an = nn < 0 ? (int16_t)-nn : nn, av = v < 0 ? (int16_t)-v : v;
        uint8_t w1 = need > an;
        uint8_t opp = (uint8_t)((v != 0) & ((v < 0) != (nn < 0)));
        uint8_t w2 = (uint8_t)(m2 & opp & (!w1 | (av > need)));
        uint8_t want = (uint8_t)(umt & e & (nn != 0) & (m0 ? opp : (uint8_t)(w1 | w2)));
        int16_t d = (m0 | w2) ? v : nn > 0 ? (int16_t)-need : need;
        d = d < -EPD_MT_D ? -EPD_MT_D : d > EPD_MT_D ? EPD_MT_D : d;
        fl[x] = (uint8_t)(e | (want << 1) | (s << 1));
        cq[x] = (uint8_t)((tb << 7) | (lb << 6) | (d + EPD_MT_D));
        nrow[x] = (int8_t)nn;
    }
}

/* pass C: fl 1/3 pixels' record and model move by their net (after the cut
 * table, pass B, gave a matched one its cut's), clamped; branch-free */
__attribute__((optimize("O3")))
static void gather_bw_apply(int16_t *restrict dc, uint8_t *restrict op, const uint8_t *restrict fl,
                            const uint8_t *restrict tg, const int8_t *restrict nrow, int n, int opm, int aclamp, int R)
{
    const uint8_t mo = (uint8_t)-(opm != 0), ma = (uint8_t)-(aclamp != 0), rr = (uint8_t)R;
    for (int x = 0; x < n; x++) {
        uint8_t e = (uint8_t)-(fl[x] & 1);
        int16_t nn = nrow[x], d = (int16_t)(dc[x] + nn), a = (int16_t)(op[x] + nn);
        d = d < -30000 ? -30000 : d;
        d = d > 30000 ? 30000 : d;
        int16_t m16 = (int16_t)(int8_t)e;
        dc[x] = (int16_t)((d & m16) | (dc[x] & ~m16));
        a = a < 0 ? 0 : a;
        a = a > rr ? rr : a;
        uint8_t rl = (uint8_t)(rr & -(uint8_t)(tg[x] == 30));        /* (!aclamp: rail[] of a black/white target) */
        uint8_t v = (uint8_t)(((uint8_t)a & ma) | (rl & ~ma)), mop = e & mo;
        op[x] = (uint8_t)((v & mop) | (op[x] & ~mop));
    }
}

/* modes whose run leaves a pixel exactly at its level (no ghost, no cut) */
static int exact_mode(int m) { m = EPD_MODE_BASE(m); return m == EPD_GC16 || m == EPD_GL16; }

/* e->stale for a claimed row: the pixels runs oa / od / heal hb took now are
 * stale unless their mode is exact (va / vd: 1 = stale). Branch-free */
__attribute__((optimize("O3")))
static void stale_row(uint8_t *restrict st, const uint8_t *restrict bz, int n, uint8_t oa, uint8_t va,
                      uint8_t od, uint8_t vd, uint8_t hb)
{
    for (int x = 0; x < n; x++) {
        uint8_t b = bz[x];
        uint8_t a = (uint8_t)(b == oa), d = (uint8_t)((b == od) & (od != 0)), h = (uint8_t)((b == hb) & (hb != 0));
        st[x] = a ? va : d ? vd : h ? 1 : st[x];
    }
}

/* a flash's pixels left alone: not taken (busy elsewhere or at their level and not stale) */
static long flash_skips(const uint8_t *st, const uint8_t *tg, const uint8_t *lv, const uint8_t *bz, int n)
{
    long k = 0;
    for (int x = 0; x < n; x++) k += !st[x] && tg[x] == lv[x] && !bz[x];
    return k;
}

/* start pending p's pixels that are not in flight as run sa in mode ma, or
 * (sd >= 0) split by pixel: the black/white -> black/white ones as run sa in
 * ma, the rest as run sd in md (A2_OR_DU) — one pass for both (it was one
 * claim per run). A partial update owns only the pixels it changes (the
 * others are not driven anyway): an unchanged white pixel next to a pen piece
 * used to count as "in flight to white" and made the next piece, which inks
 * it, wait the whole waveform (367 such waits in four thick strokes).
 * Each run's rect shrinks to what it owns; a run that owns nothing is not
 * started.
 * In three parts (epd-engine.h, epd_claim_plan): claim_plan, the pixel pass
 * (epd_claim_pixels), claim_start. The matched lengths' run slots are taken
 * before the pass, in cut order, for every cut a pixel may get (with the
 * model and cap 3: at most 3); one that took no pixel is not started. */
typedef struct {
    int sa, ma, sd, md;
    const uint8_t *la, *ld;
    int nfa, nfd;
    mtab *mt;
    int mslot[EPD_LUT_CUTS];                /* -1: no slot (none free, or not a candidate) */
    int hs, hnf;
    const uint8_t *hlut;
} claim_runs;

/* 0: nothing to claim (no table). ahead (epd_set_overlap): the claim
 * takes effect that many frames on: the heal runs' frames and the preempted
 * runs' debit as then, a run over by then not preemptible (its pixels are
 * finished first: the job) */
static int claim_plan(epd_engine *e, epd_pend *p, int sa, int ma, int sd, int md, epd_claim_plan *pl, claim_runs *cr, int ahead)
{
    int nfa = 0, nfd = 0;
    const uint8_t *la = lut(e, p->full, ma, p->mode >> 8, &nfa);
    const uint8_t *ld = sd >= 0 ? lut(e, p->full, md, p->mode >> 8, &nfd) : NULL;
    int split = sd >= 0;
    if (sd >= 0 && !ld) sd = -1;                /* no DU: A2 still takes only the black/white ones */
    if (!la) {                                  /* no A2: DU takes them all */
        if (sd < 0) return 0;
        sa = sd; ma = md; la = ld; nfa = nfd; sd = -1; ld = NULL; split = 0;
    }
    memset(pl, 0, offsetof(epd_claim_plan, pre));
    pl->r = p->r; pl->tg = p->target; pl->gt = p->gt; pl->pend = p;
    pl->fm = p->flash ? 2 : p->full ? 1 : 0;
    pl->split = split;
    net_of(la, nfa, pl->neta);
    if (ld) net_of(ld, nfd, pl->netd);
    pl->oa = (uint8_t)(sa + 1); pl->od = sd >= 0 ? (uint8_t)(sd + 1) : 0;
    /* what the claimed pixels are afterwards: exact (GC16/GL16 to their level) or stale */
    pl->va = (uint8_t)!exact_mode(ma); pl->vd = (uint8_t)!exact_mode(md);
    /* matched cuts: the step's cut cs, and a run slot per cut a pixel may get */
    mtab *mt = e->matched && ma == EPD_A2 && (p->mode >> 8) >= e->matched_from ? matched_tab(e, p->full) : NULL;
    int hs = -1, hnf = 0;
    for (int c = 0; c < EPD_LUT_CUTS; c++) cr->mslot[c] = -1;
    if (mt) {
        int cs = mt->n0 - (p->mode >> 8);
        cs = cs < 0 ? 0 : cs > EPD_LUT_CUTS - 1 ? EPD_LUT_CUTS - 1 : cs;   /* as lut */
        pl->mt = 1; pl->cs = cs; pl->cap = e->matched_cap; pl->model = e->model;
        pl->mnet = (const int8_t (*)[1024])mt->net;
        pl->best = (const uint8_t (*)[2 * EPD_MT_D + 1])mt->best;
        /* the cuts the pixel pass may pick: best[] is 0..cmax, the model never
         * picks shorter than the step (> cs), cap never more than cap longer */
        int lo = e->matched_cap && cs - e->matched_cap > 0 ? cs - e->matched_cap : 0;
        int hi = e->model && cs < mt->cmax ? cs : mt->cmax, want = 0, nfree = 0;
        for (int c = lo; c <= hi; c++) want += c != cs;
        for (int s = 0; s < RUN_SLOTS; s++) nfree += !e->run[s].nframes && s != sa && s != sd;
        if (nfree >= want) {
            for (int c = lo; c <= hi; c++) {
                if (c == cs) continue;
                int s = slot_free(e, sa, sd, cr->mslot, EPD_LUT_CUTS, &hs, 1);
                cr->mslot[c] = s;
                pl->mown[c] = (uint8_t)(s + 1);
            }
        } else {                                /* too few: on first use, as they come (epd_claim_pixels) */
            pl->lazy = 1;
            pl->sa = sa; pl->sd = sd;
        }
    }
    pl->opm = e->model || e->heal_min || e->op_track || e->snap;
    int was_ready = e->op_ready && e->op_r, was_r = e->op_r;
    if (pl->opm) op_prep(e); else e->op_ready = 0;
    /* (the whole panel's model: from the levels, or rescaled to a new op_r) */
    pl->op_init = !pl->opm ? 0 : !was_ready ? 1 : was_r != e->op_r ? 2 : 0;
    pl->op_old = was_r;
    const int R = e->op_r;
    pl->R = R; pl->aclamp = ma == EPD_A2;
    for (int g = 0; g < 32; g++) pl->rail[g] = (uint8_t)(g >= 30 ? R : R * g / 30);
    /* healing (epd_set_heal): one payback run, its slot taken now (none free:
     * no healing this time) */
    int base = EPD_MODE_BASE(p->mode), hl = p->mode >> 8;
    int hmax = hl ? hl - 1 : 8;                 /* drive frames: a cut step of len has len - 1 */
    if (e->heal_len) hmax = e->heal_len;        /* (or a fixed length) */
    if (hmax > 8) hmax = 8;
    /* epd_set_snap: a full waveform starts from the model's rail; the heal run
     * tops up what the snapped (and other idle) pixels are short of */
    pl->snp = e->snap && pl->opm && !split && ma != EPD_A2 && ma != EPD_INIT;
    int hmin = e->heal_min;
    if (pl->snp) { hmax = (R + 1) / 2 > 8 ? 8 : (R + 1) / 2; hmin = 1; }    /* (a snapped pixel is <= R / 2 short) */
    const uint8_t *hlut = NULL;
    if (hmin && e->nonblock && pl->opm && !p->full &&
        (pl->snp || ((base == EPD_A2 || base == EPD_A2_OR_DU || base == EPD_DU) && (hl == 0 || hl >= e->heal_from) && hmax >= hmin)) &&
        (hlut = heal_lut(e, hmax, &hnf)))
        for (int s = RUN_SLOTS; s < EPD_MAX_UPD && hs < 0; s++) if (!e->run[s].nframes) hs = s;
    if (hs >= 0) { pl->hown = (uint8_t)(hs + 1); pl->hmin = hmin; pl->hmax = hmax; }
    /* the heal runs' frames so far (claim_row_heal: what a preempted pixel got) */
    const epd_run *h0 = &e->run[RUN_SLOTS], *h1 = &e->run[RUN_SLOTS + 1];
    int l0 = h0->nframes && h0->frame + ahead < h0->nframes, l1 = h1->nframes && h1->frame + ahead < h1->nframes;
    pl->hf0 = (uint8_t)(h0->frame + (l0 ? ahead : 0)); pl->hf1 = (uint8_t)(h1->frame + (l1 ? ahead : 0));
    pl->hact = l0 || l1;
    /* nonblock 2: the preemptible payback runs over p (A2, black/white) and the
     * net drive of their frames still to come (rn[1..]: 0 = not preemptible) */
    memset(pl->pre, 0, sizeof pl->pre);
    if (e->nonblock == 2 && ma == EPD_A2 && !p->full)
        for (int s = 0; s < RUN_SLOTS; s++) {
            const epd_run *u = &e->run[s];
            if (!u->nframes || u->frame + ahead >= u->nframes || !u->payback || u->mode != EPD_A2 || u->full || !overlap(u->r, p->r)) continue;
            static const int idx[4] = { 0 << 5 | 0, 0 << 5 | 30, 30 << 5 | 0, 30 << 5 | 30 };   /* (to, from) */
            pl->npre++;
            for (int k = 0; k < 4; k++) {
                int nn = 0;
                for (int f = u->frame + ahead; f < u->nframes; f++) { int ph = u->lut[(size_t)f * 1024 + idx[k]] & 3; nn += ph == 1 ? -1 : ph == 2 ? 1 : 0; }
                pl->rn[pl->npre][k] = (int8_t)nn;
            }
            pl->pre[s + 1] = (uint8_t)pl->npre;
        }
    cr->sa = sa; cr->ma = ma; cr->sd = sd; cr->md = md; cr->la = la; cr->ld = ld; cr->nfa = nfa; cr->nfd = nfd;
    cr->mt = mt; cr->hs = hs; cr->hnf = hnf; cr->hlut = hlut;      /* (mslot: above) */
    return 1;
}

static void ext_cols(epd_claim_out *o, int k, const uint8_t *col, int n)
{
    if (o->y0[k] < 0) { o->x0[k] = 0; o->x1[k] = -1; return; }
    int x0 = 0, x1 = n - 1;
    while (!col[x0]) x0++;
    while (!col[x1]) x1--;
    o->x0[k] = x0; o->x1[k] = x1;
}

epd_tile_stats epd_tile_stat;
epd_sched_times epd_sched_t;
static double sched_prof_now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec / 1e9; }

static void tile_stat(const epd_engine *e, const epd_claim_plan *pl)
{
    static const int ts[EPD_TS_N] = { 16, 32, 64 };
    static uint8_t why[(EPD_W / 16 + 2) * (EPD_H / 16 + 2)];   /* per 16-px tile: 0 none, 1 changed, 2 busy, 3 flash, 4 heal */
    const epd_rect r = pl->r;
    const int tw = EPD_W / 16 + 2;
    int tx0 = r.x / 16, tx1 = (r.x + r.w - 1) / 16, ty0 = r.y / 16, ty1 = (r.y + r.h - 1) / 16;
    for (int ty = ty0; ty <= ty1; ty++) memset(why + ty * tw + tx0, 0, (size_t)(tx1 - tx0 + 1));
    epd_tile_stats *st = &epd_tile_stat;
    st->claims++;
    st->px += (uint64_t)r.w * r.h;
    static uint8_t rb[EPD_W], rl[EPD_W], ro[EPD_W], rs[EPD_W];      /* (packed: the row decoded) */
    for (int y = 0; y < r.h; y++) {
        size_t o = (size_t)(r.y + y) * EPD_W + r.x;
        const uint8_t *b = e->busy + o, *lv = e->level + o, *op = e->op + o, *sl = e->stale + o, *tg = pl->tg + (size_t)y * r.w;
        if (!e->busy) {
            size_t q = (size_t)(r.y + y) * EPD_PK_S + r.x;
            for (int x = 0; x < r.w; x++) {
                uint32_t d = e->pk->D[q + x];
                rb[x] = (uint8_t)EPD_PK_BUSY(e->pk->W[q + x]); rl[x] = (uint8_t)epd_pk_lev(d, e->pk->W[q + x], e->pk->T[q + x]);
                ro[x] = (uint8_t)((d >> 5) & 63); rs[x] = (uint8_t)((d >> 11) & 1);
            }
            b = rb; lv = rl; op = ro; sl = rs;
        }
        uint8_t *wr = why + ((r.y + y) / 16) * tw;
        for (int x = 0; x < r.w; x++) {
            uint8_t l = lv[x], w = 0;
            if (tg[x] != l) { w = 1; st->px_chg++; }
            else if (b[x]) w = 2;
            else if (pl->fm == 1 || (pl->fm == 2 && sl[x])) w = 3;
            else if (pl->hown && (l == 0 || l == 30) && (l == 30 ? (uint8_t)(pl->R - op[x]) : op[x]) >= pl->hmin) w = 4;
            if (b[x] && w > 2) w = 2;
            uint8_t *q = wr + (r.x + x) / 16;
            if (w && (!*q || w < *q)) *q = w;
        }
    }
    for (int k = 0; k < EPD_TS_N; k++) {
        int f = ts[k] / 16;
        for (int ty = ty0 / f; ty <= ty1 / f; ty++)
            for (int tx = tx0 / f; tx <= tx1 / f; tx++) {
                uint8_t w = 0;
                for (int j = 0; j < f; j++)
                    for (int i = 0; i < f; i++) {
                        int yy = ty * f + j, xx = tx * f + i;
                        if (yy < ty0 || yy > ty1 || xx < tx0 || xx > tx1) continue;
                        uint8_t v = why[yy * tw + xx];
                        if (v && (!w || v < w)) w = v;
                    }
                st->tiles[k]++;
                if (!w) continue;
                st->work[k]++;
                if (w == 1) st->chg[k]++; else if (w == 2) st->busy[k]++; else if (w == 3) st->flash[k]++; else st->heal[k]++;
            }
    }
}

#ifdef EPD_CLAIM_PROF
/* (test/epd-sched-bench, built with -DEPD_CLAIM_PROF: the pixel pass's time) */
double epd_claim_prof_s;
unsigned long epd_claim_prof_n, epd_claim_prof_px, epd_claim_prof_fast;
static double claim_prof_now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec / 1e9; }
#endif
/* the arrays' pass (claim_kernel=soa; epd_claim_pixels below picks) */
static void claim_pixels_soa(epd_engine *e, epd_claim_plan *pl, epd_claim_out *out)
{
    uint8_t tried[EPD_LUT_CUTS] = { 0 };
    const int W = pl->r.w, split = pl->split, opm = pl->opm, R = pl->R, aclamp = pl->aclamp;
    const uint8_t oa = pl->oa, od = pl->od, hown = pl->hown;
    static uint8_t ca[EPD_W], cd[EPD_W], ch[EPD_W];    /* the claimed columns (the drive thread only) */
    static uint8_t zeros[EPD_W], ones[EPD_W];   /* claim_row's full mask: partial / full */
    if (!ones[0]) memset(ones, 1, sizeof ones);
    static uint8_t cls[EPD_W];                  /* opm, matched gather: the pixels whose model moves by nrow */
    static int8_t nrow[EPD_W];
    static uint8_t fl[EPD_W], cq[EPD_W];        /* the gather's passes (gather_bw_row) */
    static uint16_t mcol[EPD_W];                /* their matched columns: bit c = cut c */
    static uint16_t xl[EPD_W + 1];              /* (pass B's list) */
    if (!pl->tg) pl->tg = pend_tg(e, pl->pend);     /* (a GPU target: its levels fetched) */
    if (epd_tile_stat_on) tile_stat(e, pl);
    memset(out, 0, sizeof *out);
    for (int c = 0; c < EPD_LUT_CUTS; c++) out->mslot[c] = pl->mown[c] ? pl->mown[c] - 1 : -1;
    for (int k = 0; k < EPD_CL_OWNERS; k++) { out->x0[k] = INT32_MAX; out->y0[k] = out->x1[k] = out->y1[k] = -1; }
    memset(ca, 0, (size_t)W);
    memset(cd, 0, (size_t)W);
    if (hown) memset(ch, 0, (size_t)W);
    int mb[EPD_LUT_CUTS][4];                    /* matched: x0 y0 x1 y1 */
    for (int c = 0; c < EPD_LUT_CUTS; c++) { mb[c][0] = mb[c][1] = INT32_MAX; mb[c][2] = mb[c][3] = -1; }
    /* the gather in three passes per row (gather_bw_row: which
     * pixels, their net and cut request, vectorized; the cut table and the
     * other pixels, scalar; gather_bw_apply, vectorized) unless the plan is
     * lazy (its slots are taken as the cuts come, in pixel order: the scalar
     * loop as it was). The cut table: per (to, from) black/white and record
     * d, the cut the scalar want-branch picks (best[], then the model's and
     * the cap's limits) and what the pixel gets from it: owner, net, column
     * bit (0, the step's own net: no slot for that cut) */
    const int fast = !pl->lazy;
    struct { uint8_t own; int8_t net; uint16_t bit; } ctab[256];
    if (fast) memset(mcol, 0, (size_t)W * sizeof mcol[0]);
    if (fast && pl->mt)
        for (int k = 0; k < 4; k++) {
            int i = ((k & 2) ? 30 : 0) << 5 | ((k & 1) ? 30 : 0);
            for (int q = 0; q < 64; q++) {
                int c = q < 2 * EPD_MT_D + 1 ? pl->best[i][q] : pl->cs;      /* (63: not asked) */
                if (pl->model && c > pl->cs) c = pl->cs;
                if (pl->cap && c < pl->cs - pl->cap) c = pl->cs - pl->cap;
                int hit = c != pl->cs && pl->mown[c];
                ctab[k << 6 | q].own = hit ? pl->mown[c] : 0;
                ctab[k << 6 | q].net = hit ? pl->mnet[c][i] : pl->neta[i];
                ctab[k << 6 | q].bit = (uint16_t)(hit ? 1u << c : 0);
            }
        }
    /* preempt_owners_row: the preemptible runs' busy values (255: none) and their rows of rn */
    uint8_t pob[4] = { 255, 255, 255, 255 };
    int8_t prn[4][4] = { { 0 } };
    int npo = 0;
    if (pl->npre && pl->npre <= 4)
        for (int v = 1; v < 256 && npo < pl->npre; v++)
            if (pl->pre[v]) { pob[npo] = (uint8_t)v; memcpy(prn[npo], pl->rn[pl->pre[v]], 4); npo++; }
    for (int y = 0; y < pl->r.h; y++) {
        size_t o = (size_t)(pl->r.y + y) * EPD_W + pl->r.x;
        uint8_t *b = e->busy + o;
        const uint8_t *tg = pl->tg + (size_t)y * W, *lv = e->level + o;
        if (npo) {
            if (preempt_any(b, e->target + o, tg, W, pob))
                out->preempted += (uint64_t)preempt_owners_row(b, e->target + o, tg, e->level + o, e->op + o, e->dc + o, W,
                                                               pob, prn, opm, R);
        }
        else if (pl->npre) out->preempted += (uint64_t)preempt_row(b, e->target + o, tg, e->level + o, e->op + o, e->dc + o, W,
                                                                    pl->pre, (int8_t (*)[4])pl->rn, opm, R);
        if (pl->snp) out->snapped += (uint64_t)snap_row(b, e->level + o, tg, e->op + o, W, R);
        const uint8_t *fm = pl->fm == 2 ? e->stale + o : pl->fm ? ones : zeros;
        int any = pl->hact ? claim_row_heal(b, e->target + o, tg, e->level + o, e->op + o, e->dc + o, ca, cd, W, fm, split, oa, od, pl->hf0, pl->hf1)
                           : claim_row(b, e->target + o, tg, lv, ca, cd, W, fm, split, oa, od);
        int healed = hown && heal_row(b, e->target + o, e->level + o, e->op + o, e->dc + o, tg, ch, W, hown, R, pl->hmin, pl->hmax);
        if (healed) {
            if (out->y0[EPD_CL_H] < 0) out->y0[EPD_CL_H] = y;
            out->y1[EPD_CL_H] = y;
        }
        if (any || healed) stale_row(e->stale + o, b, W, oa, pl->va, od, pl->vd, healed ? hown : 0);
        if (pl->fm == 2) out->flash_skipped += (uint64_t)flash_skips(e->stale + o, tg, lv, b, W);
        if (!any) continue;
        if (any & 1) { if (out->y0[EPD_CL_A] < 0) out->y0[EPD_CL_A] = y; out->y1[EPD_CL_A] = y; }
        if (any & 2) { if (out->y0[EPD_CL_D] < 0) out->y0[EPD_CL_D] = y; out->y1[EPD_CL_D] = y; }
        int16_t *dc = e->dc + o;                /* the DC record */
        uint8_t *opr = e->op + o;               /* the panel model (opm) */
        if (split && !pl->mt) {                 /* oa: black/white only (dc_row_bw); od: the gather */
            if (any & 1) dc_row_bw(dc, b, tg, lv, W, oa, pl->neta);
            if (any & 2)
                for (int x = 0; x < W; x++)
                    if (b[x] == od) {
                        int d = dc[x] + pl->netd[tg[x] << 5 | lv[x]];
                        dc[x] = (int16_t)(d < -30000 ? -30000 : d > 30000 ? 30000 : d);
                    }
            if (opm) {
                if (any & 1) op_row_bw(opr, b, tg, lv, W, oa, pl->neta, aclamp, R);
                if (any & 2) op_row_rail(opr, b, tg, W, od, pl->rail);
            }
            continue;
        }
        if (fast) {
            gather_bw_row(b, tg, lv, opr, dc, fl, cq, nrow, W, oa, od, pl->neta, pl->mt, pl->model, R);
#ifdef EPD_CLAIM_PROF
            { unsigned long np = 0, nf = 0; for (int x = 0; x < W; x++) { np += fl[x] != 0; nf += fl[x] & 1; } epd_claim_prof_px += np; epd_claim_prof_fast += nf; }
#endif
            uint16_t rb = 0;                    /* the cuts matched in this row */
            /* the pixels that want a cut (fl 3) and the rest (fl 2), listed
             * branch-free: walking the row with a branch per pixel (or per 8)
             * mispredicted on a dithered row's pattern, ~40 cycles a listed
             * pixel (x86) */
            int nl = 0;
            for (int x = 0; x < W; x++) { xl[nl] = (uint16_t)x; nl += fl[x] >> 1; }
            uint64_t nm = 0;
            for (int j = 0; j < nl; j++) {
                int x = xl[j];
                if (fl[x] & 1) {                /* the cut table */
                    int k = cq[x];
                    uint8_t ow = ctab[k].own;
                    nrow[x] = ctab[k].net;
                    b[x] = ow ? ow : b[x];
                    mcol[x] |= ctab[k].bit;
                    rb |= ctab[k].bit;
                    nm += ow != 0;
                    continue;
                }
                uint8_t w = b[x];               /* the rest: the scalar gather (as below; never lazy here) */
                int i = tg[x] << 5 | lv[x], n = (w == oa ? pl->neta : pl->netd)[i];
                if (pl->mt && w == oa && n && !pl->model && dc[x] && (dc[x] < 0) != (n < 0)) {
                    int d = dc[x];              /* (the model's want needs black/white: not here) */
                    d = d < -EPD_MT_D ? -EPD_MT_D : d > EPD_MT_D ? EPD_MT_D : d;
                    int c = pl->best[i][d + EPD_MT_D];
                    if (pl->cap && c < pl->cs - pl->cap) c = pl->cs - pl->cap;
                    if (c != pl->cs && pl->mown[c]) {
                        b[x] = pl->mown[c];
                        n = pl->mnet[c][i];
                        mcol[x] |= (uint16_t)(1u << c);
                        rb |= (uint16_t)(1u << c);
                        out->matched++;
                    }
                }
                int dd = dc[x] + n;
                dc[x] = (int16_t)(dd < -30000 ? -30000 : dd > 30000 ? 30000 : dd);
                if (opm) opr[x] = pl->rail[tg[x]];      /* (oa's black/white ones: gather_bw_apply) */
            }
            out->matched += nm;
            gather_bw_apply(dc, opr, fl, tg, nrow, W, opm, aclamp, R);
            for (int c = 0; rb && c < EPD_LUT_CUTS; c++)
                if (rb & (1u << c)) { if (mb[c][1] == INT32_MAX) mb[c][1] = y; mb[c][3] = y; }
            continue;
        }
        if (opm) memset(cls, 0, (size_t)W);
        for (int x = 0; x < W; x++) {                   /* a table gather, scalar */
            if (!(x & 7) && x + 8 <= W && !has_byte2(b + x, oa, od)) { x += 7; continue; }   /* (most of a step's row) */
            uint8_t w = b[x];
            if (w != oa && (!od || w != od)) continue;      /* (no other run owns oa/od yet) */
            int i = tg[x] << 5 | lv[x], n = (w == oa ? pl->neta : pl->netd)[i];
            int want = 0, d = 0;
            if (pl->mt && w == oa && n) {
                if (pl->model) {                    /* short of what it needs to reach its target */
                    if (bw_pix(tg[x], lv[x])) {
                        int need = tg[x] == 30 ? R - opr[x] : opr[x], an = n < 0 ? -n : n;
                        if (need > an) { want = 1; d = n > 0 ? -need : need; }
                        /* model 2: a record opposing the drive asks for more where
                         * it is longer: past what the pixel needs it only drives
                         * a saturated pixel (no optical change) and pays the DC
                         * back (model 1 alone drifted, device: 20985 px
                         * beyond 60, nearly all toward white) */
                        if (pl->model == 2 && dc[x] && (dc[x] < 0) != (n < 0) && (!want || (dc[x] < 0 ? -dc[x] : dc[x]) > need)) {
                            want = 1; d = dc[x];
                        }
                    }
                } else if (dc[x] && (dc[x] < 0) != (n < 0)) { want = 1; d = dc[x]; }   /* opposes the record */
            }
            if (want) {
                d = d < -EPD_MT_D ? -EPD_MT_D : d > EPD_MT_D ? EPD_MT_D : d;
                int c = pl->best[i][d + EPD_MT_D];
                if (pl->model && c > pl->cs) c = pl->cs;        /* never shorter than its step */
                if (pl->cap && c < pl->cs - pl->cap) c = pl->cs - pl->cap;   /* at most cap frames longer */
                if (pl->lazy && c != pl->cs && !pl->mown[c] && !tried[c]) {
                    tried[c] = 1;               /* (one try: once none is free, none will be) */
                    int s = slot_free(e, pl->sa, pl->sd, out->mslot, EPD_LUT_CUTS, NULL, 0);
                    if (s >= 0) { pl->mown[c] = (uint8_t)(s + 1); out->mslot[c] = s; }
                }
                if (c != pl->cs && pl->mown[c]) {
                    b[x] = pl->mown[c];
                    n = pl->mnet[c][i];
                    int *q = mb[c];
                    if (x < q[0]) q[0] = x;
                    if (y < q[1]) q[1] = y;
                    if (x > q[2]) q[2] = x;
                    q[3] = y;
                    out->matched++;
                }
            }
            int dd = dc[x] + n;
            dc[x] = (int16_t)(dd < -30000 ? -30000 : dd > 30000 ? 30000 : dd);
            if (opm) {
                if (w == oa && aclamp && bw_pix(tg[x], lv[x])) { nrow[x] = (int8_t)n; cls[x] = 1; }
                else opr[x] = pl->rail[tg[x]];
            }
        }
        if (opm) op_row_net(opr, cls, nrow, W, R);
    }
    ext_cols(out, EPD_CL_A, ca, W);
    ext_cols(out, EPD_CL_D, cd, W);
    if (hown) ext_cols(out, EPD_CL_H, ch, W);
    else { out->x0[EPD_CL_H] = 0; out->x1[EPD_CL_H] = -1; }
    for (int c = 0; fast && c < EPD_LUT_CUTS; c++) {     /* (the passes: the matched columns) */
        if (mb[c][3] < 0) continue;
        int x0 = 0, x1 = W - 1;
        while (!(mcol[x0] & (1u << c))) x0++;
        while (!(mcol[x1] & (1u << c))) x1--;
        mb[c][0] = x0; mb[c][2] = x1;
    }
    for (int c = 0; c < EPD_LUT_CUTS; c++) {
        int k = EPD_CL_M0 + c;
        if (mb[c][2] < 0) { out->x0[k] = 0; out->x1[k] = -1; continue; }
        out->x0[k] = mb[c][0]; out->y0[k] = mb[c][1]; out->x1[k] = mb[c][2]; out->y1[k] = mb[c][3];
    }
    if (hown) {                                 /* (stats) the heal run's pixels */
        for (int y = out->y0[EPD_CL_H]; out->x1[EPD_CL_H] >= 0 && y <= out->y1[EPD_CL_H]; y++)
            for (int x = out->x0[EPD_CL_H]; x <= out->x1[EPD_CL_H]; x++)
                out->healed += e->busy[(size_t)(pl->r.y + y) * EPD_W + pl->r.x + x] == hown;
    }
}

/* ---- : the claim pass on D/W/T --------------
 * The passes of a row (preempt_owners_row, snap_row, claim_row[_heal],
 * heal_row, stale_row, flash_skips, then dc/op_row_bw or gather_bw_row) as
 * ONE sweep over the row's packed words: each pixel's chain in registers,
 * D/W/T loaded and stored once (today: up to 8 passes over 6 arrays). The
 * scalar parts stay as they are (the od gather, the cut table, pass B) and
 * pass C (gather_bw_apply) is a second sweep over D. Each step is the array
 * kernel's expression, letter for letter: test/epd-claim-bench KERNEL=packed
 * compares every claim against claim_pixels_soa bit for bit.
 * Row-level gates the array pass has (preempt_any, `any` before stale_row and
 * the gather) are per pixel here: they only skip pixels whose busy value is
 * one of this claim's slots, and a slot handed out is free (no pixel holds it)
 * — the one place they differ: dc_row_bw clamps the whole row's record when
 * oa took a pixel there (heal_row may leave it at 30008), done here after the
 * sweep for those rows. */
unsigned long epd_claim_kernel_soa, epd_claim_kernel_diffs;

/* the boundary: a row of the arrays into D/W/T and back (until the arrays go) */
__attribute__((optimize("O3")))
static void pk_pack_row(const epd_engine *e, size_t o, int n, uint32_t *restrict d, uint16_t *restrict w, uint8_t *restrict t)
{
    const uint8_t *restrict bz = e->busy + o, *restrict et = e->target + o, *restrict lv = e->level + o;
    const uint8_t *restrict op = e->op + o, *restrict st = e->stale + o;
    const int16_t *restrict dc = e->dc + o;
    for (int x = 0; x < n; x++) {
        uint8_t b = bz[x];
        d[x] = (uint32_t)lv[x] | (uint32_t)op[x] << 5 | (uint32_t)(st[x] & 1) << 11 | (uint32_t)(uint16_t)dc[x] << 16;
        w[x] = b ? (uint16_t)((b - 1) << 10 | et[x] << 5 | lv[x]) : (uint16_t)EPD_PK_IDLE;
        t[x] = et[x];
    }
}
__attribute__((optimize("O3")))
static void pk_unpack_row(epd_engine *e, size_t o, int n, const uint32_t *restrict d, const uint16_t *restrict w, const uint8_t *restrict t)
{
    uint8_t *restrict bz = e->busy + o, *restrict et = e->target + o, *restrict lv = e->level + o;
    uint8_t *restrict op = e->op + o, *restrict st = e->stale + o;
    int16_t *restrict dc = e->dc + o;
    for (int x = 0; x < n; x++) {
        uint32_t v = d[x];
        uint16_t u = w[x];
        bz[x] = EPD_PK_ISIDLE(u) ? 0 : (uint8_t)((u >> 10) + 1);
        et[x] = t[x] & 31;                      /* (T's bits 5..7 are 's requests) */
        lv[x] = u == EPD_PK_FIN ? et[x] : (uint8_t)(v & 31);     /* (FIN: a finish's level, T's) */
        op[x] = (uint8_t)((v >> 5) & 63);
        st[x] = (uint8_t)((v >> 11) & 1);
        dc[x] = (int16_t)(v >> 16);
    }
}

/* (v == 0) | (v == 30) with 30 read at run time (epd_claim_job l30): with the
 * literal gcc makes the pair a bit test (0x40000001 >> v), which x86 cannot
 * vectorize on bytes, and the sweep stayed scalar */
static inline uint8_t pk_rail(uint8_t v, uint8_t l30) { return (uint8_t)((v == 0) | (v == l30)); }
/* -(v == 30) for a 5-bit value by a shift: as -(bool), gcc made `mask & const`
 * a byte multiply, which SSE2 has not */
static inline uint8_t pk_eq30(uint8_t v, uint8_t l30) { return (uint8_t)((int8_t)(uint8_t)((v ^ l30) - 1) >> 7); }

/* the sweep: SPLIT = a split claim without matched cuts (dc/op_row_bw; the od
 * pixels after it, scalar), else the fast gather's pass A (fl/cq/nrow); HACT
 * = claim_row_heal. Returns bit 0 oa took, 1 od took, 2 healed, 3 a record
 * beyond +-30000 is in the row, 4 a word changed; c[]: preempted, snapped,
 * flash skips, healed */
__attribute__((always_inline))
static inline int pk_row(uint32_t *restrict D, uint16_t *restrict Wd, uint8_t *restrict T, const uint8_t *restrict tg,
                         uint8_t *restrict ca, uint8_t *restrict cd, uint8_t *restrict ch, uint8_t *restrict fl,
                         uint8_t *restrict cq, int8_t *restrict nrow, int n, const epd_claim_plan *pl, const epd_claim_job *j,
                         const int SPLIT, const int HACT, unsigned *c)
{
    const uint8_t o0 = j->pob[0], o1 = j->pob[1], o2 = j->pob[2], o3 = j->pob[3];
    const int8_t a0 = j->prn[0][0], a1 = j->prn[0][1], a2 = j->prn[0][2], a3 = j->prn[0][3];
    const int8_t b0 = j->prn[1][0], b1 = j->prn[1][1], b2 = j->prn[1][2], b3 = j->prn[1][3];
    const int8_t c0 = j->prn[2][0], c1 = j->prn[2][1], c2 = j->prn[2][2], c3 = j->prn[2][3];
    const int8_t d0 = j->prn[3][0], d1 = j->prn[3][1], d2 = j->prn[3][2], d3 = j->prn[3][3];
    const int R = pl->R;
    const uint8_t rr = (uint8_t)R, mo = (uint8_t)-(pl->opm != 0), ma = (uint8_t)-(pl->aclamp != 0);
    const uint8_t snp = pl->snp != 0, fm1 = pl->fm == 1, fm2 = (uint8_t)-(pl->fm == 2), sp = pl->split != 0;
    const uint8_t oa = pl->oa, od = pl->od, hod = od != 0, hown = pl->hown, hon = hown != 0;
    const uint8_t hmin8 = (uint8_t)pl->hmin, hmax8 = (uint8_t)pl->hmax, f0 = pl->hf0, f1 = pl->hf1, va = pl->va, vd = pl->vd;
    const int8_t *net = pl->neta;
    const int16_t n00 = net[0 << 5 | 0], n0w = net[0 << 5 | 30], nw0 = net[30 << 5 | 0], nww = net[30 << 5 | 30];
    const uint8_t umt = pl->mt != 0, m0 = pl->model == 0, m2 = pl->model == 2;
    const int16_t r16 = (int16_t)R;
    const uint8_t L30 = j->l30;
    uint8_t any_a = 0, any_d = 0, any_h = 0, ov = 0, chd = 0;
    uint16_t np = 0, ns = 0, nf = 0, nh = 0;      /* (n <= EPD_W) */
    for (int x = 0; x < n; x++) {
        const uint32_t dv = D[x];
        const uint16_t wv = Wd[x];
        const uint8_t et0 = T[x] & 31, t = tg[x];  /* (T's bits 5..7 are 's requests) */
        uint8_t l = wv == EPD_PK_FIN ? et0 : (uint8_t)(dv & 31), o = (uint8_t)((dv >> 5) & 63), s = (uint8_t)((dv >> 11) & 1);
        int16_t dc = (int16_t)(dv >> 16);
        uint8_t b = EPD_PK_ISIDLE(wv) ? 0 : (uint8_t)((wv >> 10) + 1);
        uint8_t et = et0;
        { /* preempt_owners_row (pob 255: none) */
            uint8_t tw = pk_eq30(et, L30), lw = pk_eq30(l, L30);
            uint8_t bw = (uint8_t)(pk_rail(t, L30) & pk_rail(et, L30) & pk_rail(l, L30));
            uint8_t k0 = (uint8_t)-(b == o0), k1 = (uint8_t)-(b == o1), k2 = (uint8_t)-(b == o2), k3 = (uint8_t)-(b == o3);
            uint8_t m = (uint8_t)((k0 | k1 | k2 | k3) & -(uint8_t)((t != et) & bw));
            uint8_t ra = (uint8_t)((tw & ((lw & (uint8_t)a3) | (~lw & (uint8_t)a2))) | (~tw & ((lw & (uint8_t)a1) | (~lw & (uint8_t)a0))));
            uint8_t rb = (uint8_t)((tw & ((lw & (uint8_t)b3) | (~lw & (uint8_t)b2))) | (~tw & ((lw & (uint8_t)b1) | (~lw & (uint8_t)b0))));
            uint8_t rc = (uint8_t)((tw & ((lw & (uint8_t)c3) | (~lw & (uint8_t)c2))) | (~tw & ((lw & (uint8_t)c1) | (~lw & (uint8_t)c0))));
            uint8_t rd = (uint8_t)((tw & ((lw & (uint8_t)d3) | (~lw & (uint8_t)d2))) | (~tw & ((lw & (uint8_t)d1) | (~lw & (uint8_t)d0))));
            int8_t r = (int8_t)((ra & k0) | (rb & k1) | (rc & k2) | (rd & k3));
            uint8_t sg = (uint8_t)(r >> 7), up = (uint8_t)(r & ~sg), dn = (uint8_t)(-r & sg);
            uint8_t u1 = up < o ? up : o, a = (uint8_t)(o - u1);
            uint8_t hd = (uint8_t)(255 - a), e1 = dn < hd ? dn : hd;
            uint8_t s2 = (uint8_t)(a + e1);
            s2 = s2 > rr ? rr : s2;
            int16_t d = (int16_t)(dc - r);
            d = d < -30000 ? -30000 : d;
            d = d > 30000 ? 30000 : d;
            uint8_t mop = m & mo;
            o = (uint8_t)((s2 & mop) | (o & ~mop));
            int16_t m16 = (int16_t)(int8_t)m;
            dc = (int16_t)((d & m16) | (dc & ~m16));
            l = (uint8_t)((et & m) | (l & ~m));
            b = (uint8_t)(b & ~m);
            np += (uint16_t)(m & 1);
        }
        { /* snap_row */
            uint8_t idle = b == 0;
            int oo = 2 * o;
            uint8_t up = (uint8_t)(snp & idle & (l == 0) & (t == L30) & (oo > R)), dn = (uint8_t)(snp & idle & (l == L30) & (t == 0) & (oo < R));
            uint8_t mu = (uint8_t)-up, md = (uint8_t)-dn;
            l = (uint8_t)((L30 & mu) | (l & ~(mu | md)));
            ns += (uint16_t)(up | dn);
        }
        const uint8_t fx = (uint8_t)(fm1 | (s & fm2));
        uint8_t ta, td;
        if (!HACT) { /* claim_row */
            uint8_t chg = (uint8_t)((b == 0) & (fx | (t != l)));
            uint8_t bw = (uint8_t)(pk_rail(t, L30) & pk_rail(l, L30));
            ta = (uint8_t)(chg & (bw | !sp)); td = (uint8_t)(chg & sp & !bw & hod);
            uint8_t own = (uint8_t)((ta ? oa : 0) | (td ? od : 0));
            et = own ? t : et;
            b = (uint8_t)(b | own);
        } else { /* claim_row_heal */
            uint8_t h = b >= HEAL_B;
            uint8_t lr = h ? et : l;
            uint8_t chg = (uint8_t)(((b == 0) | h) & (fx | (t != lr)));
            uint8_t bw = (uint8_t)(pk_rail(t, L30) & pk_rail(lr, L30));
            ta = (uint8_t)(chg & (bw | !sp)); td = (uint8_t)(chg & sp & !bw & hod);
            uint8_t own = (uint8_t)((ta ? oa : 0) | (td ? od : 0));
            uint8_t pre = (uint8_t)(h & (own != 0));
            uint8_t k = (uint8_t)((l + 1) >> 1), done = b == HEAL_B ? f0 : f1;
            uint8_t rem = k > done ? (uint8_t)(k - done) : 0;
            uint8_t ru = (uint8_t)(rem & (uint8_t)-(pre & (lr == L30))), rd = (uint8_t)(rem & (uint8_t)-(pre & (lr == 0)));
            uint8_t mp = (uint8_t)-pre, mw = (uint8_t)-(own != 0);
            o = (uint8_t)(o - ru + rd);
            dc = (int16_t)(dc - (int16_t)ru + (int16_t)rd);
            l = (uint8_t)((lr & mp) | (l & ~mp));
            et = (uint8_t)((t & mw) | (et & ~mw));
            b = (uint8_t)((own & mw) | (b & ~mw));
        }
        ca[x] |= ta; cd[x] |= td;
        any_a |= ta; any_d |= td;
        { /* heal_row */
            uint8_t w = l == L30, bl = l == 0;
            uint8_t def = w ? (uint8_t)(rr - o) : o;
            uint8_t ok = (uint8_t)(hon & (b == 0) & (t == l) & (w | bl) & (def >= hmin8));
            uint8_t k = def > hmax8 ? hmax8 : def;
            uint8_t m = (uint8_t)-ok, ku = (uint8_t)(k & (uint8_t)-(ok & w)), kd = (uint8_t)(k & (uint8_t)-(ok & bl));
            o = (uint8_t)(o + ku - kd);
            dc = (int16_t)(dc + (int16_t)ku - (int16_t)kd);
            b = (uint8_t)((hown & m) | (b & ~m));
            et = (uint8_t)((l & m) | (et & ~m));
            l = (uint8_t)(((2 * k - 1) & m) | (l & ~m));
            ch[x] |= ok;
            any_h |= ok;
            nh += (uint16_t)ok;
        }
        { /* stale_row (hb: hown, the slot no other pixel holds) */
            uint8_t ia = (uint8_t)(b == oa), id = (uint8_t)((b == od) & hod), ih = (uint8_t)((b == hown) & hon);
            s = ia ? va : id ? vd : ih ? 1 : s;
        }
        nf += (uint16_t)(fm2 & 1 & !s & (t == l) & (b == 0));
        if (SPLIT) { /* dc_row_bw, op_row_bw */
            int16_t v = t == 0 ? (l == 0 ? n00 : n0w) : (l == 0 ? nw0 : nww);
            uint8_t ia = b == oa;
            int dd = dc + v;
            dd = dd < -30000 ? -30000 : dd > 30000 ? 30000 : dd;
            int a = o + v;
            a = a < 0 ? 0 : a > R ? R : a;
            uint8_t rl = t == L30 ? rr : 0;
            o = (ia & mo & 1) ? (uint8_t)(ma ? a : rl) : o;
            dc = ia ? (int16_t)dd : dc;
            ov |= (uint8_t)((dc > 30000) | (dc < -30000));
        } else { /* gather_bw_row */
            uint8_t tb = t == L30, lb = l == L30;
            uint8_t bw = (uint8_t)(pk_rail(t, L30) & pk_rail(l, L30));
            uint8_t ia = b == oa, e = (uint8_t)(ia & bw);
            uint8_t sr = (uint8_t)((ia & !bw) | (hod & (b == od)));
            int16_t nn = tb ? (lb ? nww : nw0) : (lb ? n0w : n00);
            int16_t oo = o, v = dc;
            int16_t need = tb ? (int16_t)(r16 - oo) : oo, an = nn < 0 ? (int16_t)-nn : nn, av = v < 0 ? (int16_t)-v : v;
            uint8_t w1 = need > an;
            uint8_t opp = (uint8_t)((v != 0) & ((v < 0) != (nn < 0)));
            uint8_t w2 = (uint8_t)(m2 & opp & (!w1 | (av > need)));
            uint8_t want = (uint8_t)(umt & e & (nn != 0) & (m0 ? opp : (uint8_t)(w1 | w2)));
            int16_t dq = (m0 | w2) ? v : nn > 0 ? (int16_t)-need : need;
            dq = dq < -EPD_MT_D ? -EPD_MT_D : dq > EPD_MT_D ? EPD_MT_D : dq;
            fl[x] = (uint8_t)(e | (want << 1) | (sr << 1));
            cq[x] = (uint8_t)((tb << 7) | (lb << 6) | (dq + EPD_MT_D));
            nrow[x] = (int8_t)nn;
        }
        const uint32_t dn = (uint32_t)l | (uint32_t)o << 5 | (uint32_t)s << 11 | (dv & 0xF000u) | (uint32_t)(uint16_t)dc << 16;
        const uint16_t wn = b ? (uint16_t)((b - 1) << 10 | et << 5 | l) : (uint16_t)EPD_PK_IDLE;
        chd |= (uint8_t)((dn != dv) | (wn != wv) | (et != et0));
        D[x] = dn; Wd[x] = wn; T[x] = (uint8_t)(et | (T[x] & 0xE0u));      /* (the request bits kept) */
    }
    c[0] += np; c[1] += ns; c[2] += nf; c[3] += nh;
    return (any_a ? 1 : 0) | (any_d ? 2 : 0) | (any_h ? 4 : 0) | (ov ? 8 : 0) | (chd ? 16 : 0);
}
#define PK_ROW_ARGS uint32_t *restrict D, uint16_t *restrict Wd, uint8_t *restrict T, const uint8_t *restrict tg, \
    uint8_t *restrict ca, uint8_t *restrict cd, uint8_t *restrict ch, uint8_t *restrict fl, uint8_t *restrict cq, \
    int8_t *restrict nrow, int n, const epd_claim_plan *pl, const epd_claim_job *j, unsigned *c
#define PK_ROW_PASS D, Wd, T, tg, ca, cd, ch, fl, cq, nrow, n, pl, j
/* the sweep, PK_N pixels a step. The first version's
 * 16-px step in GCC vector types ran ~600 instructions on arm64 (~1300 on
 * x86): its vector arrays lived on the stack (spills, alias-check branches,
 * two-register tbl shuffles). Now named vectors only, no inner loop, no
 * branch in the step; every load, store, width change and lookup is a
 * primitive per platform below, the bytes' logic is shared GCC vector
 * code. arm64: PK_N 16 (.16b; a 16-bit value, the DC record first, as two
 * q) or, built with -DPK_N=8, 8 (.8b: every 16-bit value one q, ~1.7x the
 * instructions a pixel; the A73 decides: test/neon-width-bench); x86
 * (SSE2): 16. What keeps the 32 vector registers from spilling on arm64:
 * the uniforms as lanes of three 16-byte tables (a dup in the step), the
 * counts and the any-flags in general registers, clamps as one saturating
 * op, model 2 (production) compile-time, preempt with 2 or 4 owners, and
 * the matched cut's want in a second pass over the stored words. What the
 * scalar's order allows: a slot handed out is free in every pixel, so after
 * the claim b == oa is ta (od: td, hown: the heal's ok), b == 0 is the
 * preempt's idle minus those, and a word changes only where a decision took
 * it (preempt, snap, claim, heal: bit 4 of the return from their masks and
 * counts). Masks are 0xff / 0xffff lanes; every u8 step wraps as pk_row's
 * (uint8_t) casts do. pk_row: the reference and the row's tail (and the
 * whole sweep on other targets); pk_scalar (tests): 1 = pk_row for the whole
 * row */
static int pk_scalar;
#ifdef EPD_PK_PROF
/* (a profiling build, test/epd-claim-bench S8PROF: the sweep's matched-cut
 * pass and scalar tail timed; the timers sit outside the step) */
double epd_pk_prof_mt, epd_pk_prof_tail;
static double pk_prof_now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec / 1e9; }
#define PK_PROF_T0(v) const double v = pk_prof_now()
#define PK_PROF_ADD(acc, v) (acc += pk_prof_now() - (v))
#else
#define PK_PROF_T0(v)
#define PK_PROF_ADD(acc, v)
#endif
__attribute__((unused)) static int claim_skip_of(void)
{
    int k = __atomic_load_n(&epd_claim_skip, __ATOMIC_RELAXED);
    if (k < 0) k = KNOB_DEF(epd_claim_skip);    /* (GLASSINE_CLAIM_SKIP=1: epd_init) */
    return k;
}
#if defined(__aarch64__)
#ifndef PK_N
#define PK_N 16
#endif
#define PK_TBL 1
typedef uint8_t pk_b __attribute__((vector_size(PK_N)));
typedef int8_t pk_sb __attribute__((vector_size(PK_N)));
#if PK_N == 8
typedef int16x8_t pk_sh;
#define PK_U8(v) ((uint8x8_t)(v))
#define PK_SHR(v, n) ((pk_b)vshr_n_u8(PK_U8(v), n))
#define PK_SHL(v, n) ((pk_b)vshl_n_u8(PK_U8(v), n))
#define PK_NWSH(v, n) ((pk_b)vmovn_u16(vshrq_n_u16((uint16x8_t)(v), n)))   /* (u16 >> n) to bytes */
static inline pk_b pk_ld(const void *p) { return (pk_b)vld1_u8((const uint8_t *)p); }
static inline void pk_st(void *p, pk_b v) { vst1_u8((uint8_t *)p, PK_U8(v)); }
static inline pk_b pk_umin(pk_b a, pk_b b) { return (pk_b)vmin_u8(PK_U8(a), PK_U8(b)); }
static inline pk_b pk_smin(pk_b a, pk_b b) { return (pk_b)vmin_s8((int8x8_t)a, (int8x8_t)b); }
static inline pk_b pk_smax(pk_b a, pk_b b) { return (pk_b)vmax_s8((int8x8_t)a, (int8x8_t)b); }
static inline pk_b pk_qsub(pk_b a, pk_b b) { return (pk_b)vqsub_u8(PK_U8(a), PK_U8(b)); }
static inline pk_b pk_uadd(pk_b a, pk_b v) { return (pk_b)vsqadd_u8(PK_U8(a), (int8x8_t)v); }
/* a - v saturated to u8; v = -128 adds 127 (only ever clamped to R <= 60 after) */
static inline pk_b pk_usub(pk_b a, pk_b v) { return (pk_b)vsqadd_u8(PK_U8(a), vqneg_s8((int8x8_t)v)); }
static inline pk_b pk_rsh1(pk_b a) { return (pk_b)vrshr_n_u8(PK_U8(a), 1); }        /* (a + 1) >> 1 */
static inline pk_b pk_qabs(pk_b a) { return (pk_b)vqabs_s8((int8x8_t)a); }
static inline pk_b pk_tbl(uint8x16_t t, pk_b i) { return (pk_b)vqtbl1_u8(t, PK_U8(i)); }
static inline int pk_any(pk_b v) { return vmaxv_u8(PK_U8(v)) != 0; }
static inline unsigned pk_sum(pk_b v) { return vaddlv_u8(PK_U8(v)); }
static inline unsigned pk_cnt(pk_b m) { return (uint8_t)(0u - vaddv_u8(PK_U8(m))); }   /* (0xff lanes) */
static inline uint64_t pk_flag2(pk_b a, pk_b b) { return vget_lane_u64(vreinterpret_u64_u8(PK_U8((a & 1) | (b & 2))), 0); }
static inline pk_sh pk_h16(int c) { return vdupq_n_s16((int16_t)c); }
#define PK_LB(v, i) ((pk_b)vdup_laneq_u8(v, i))
static inline pk_sh pk_neg16(pk_sh a) { return vnegq_s16(a); }
#define PK_LH(v, i) (vdupq_laneq_s16((int16x8_t)(v), i))
static inline pk_sh pk_min16(pk_sh a, pk_sh b) { return vminq_s16(a, b); }
static inline pk_sh pk_max16(pk_sh a, pk_sh b) { return vmaxq_s16(a, b); }
static inline pk_sh pk_abs16(pk_sh a) { return vabsq_s16(a); }
static inline pk_sh pk_sel16(pk_b m, pk_sh a, pk_sh b) { return vbslq_s16((uint16x8_t)vmovl_s8((int8x8_t)m), a, b); }
static inline pk_sh pk_addz(pk_sh a, pk_b v) { return (pk_sh)vaddw_u8((uint16x8_t)a, PK_U8(v)); }   /* a + zx(v) */
static inline pk_sh pk_subz(pk_sh a, pk_b v) { return (pk_sh)vsubw_u8((uint16x8_t)a, PK_U8(v)); }
static inline pk_sh pk_adds(pk_sh a, pk_b v) { return vaddw_s8(a, (int8x8_t)v); }                      /* a + sx(v) */
static inline pk_sh pk_subs(pk_sh a, pk_b v) { return vsubw_s8(a, (int8x8_t)v); }
static inline pk_b pk_nw(pk_sh v) { return (pk_b)vmovn_s16(v); }
static inline pk_b pk_qnw(pk_sh v) { return (pk_b)vqmovn_s16(v); }
static inline pk_b pk_eq16(pk_sh v, int c) { return (pk_b)vmovn_u16(vceqq_s16(v, vdupq_n_s16((int16_t)c))); }
static inline uint16x8_t pk_mk16(pk_b lo, pk_b hi) { return vaddw_u8(vshll_n_u8(PK_U8(hi), 8), PK_U8(lo)); }
static inline pk_b pk_ovb(pk_sh v, pk_sh k) { return (pk_b)vqmovn_u16(vqsubq_u16((uint16x8_t)vabsq_s16(v), (uint16x8_t)k)); }   /* |v| > 30000 */
static inline void pk_ldd(const uint32_t *p, pk_sh *lo, pk_sh *dc)
{
    uint16x8x2_t v = vld2q_u16((const uint16_t *)p);
    *lo = (pk_sh)v.val[0]; *dc = (pk_sh)v.val[1];
}
/* (zip, not st2: gcc builds an st2's register pair on the stack) */
static inline void pk_std(uint32_t *p, pk_b lo, pk_b hi, pk_sh dc)
{
    uint16x8_t w = pk_mk16(lo, hi);
    vst1q_u16((uint16_t *)p, vzip1q_u16(w, (uint16x8_t)dc)); vst1q_u16((uint16_t *)p + 8, vzip2q_u16(w, (uint16x8_t)dc));
}
static inline pk_sh pk_ldw(const uint16_t *p) { return (pk_sh)vld1q_u16(p); }
static inline void pk_stw(uint16_t *p, pk_b lo, pk_b hi) { vst1q_u16(p, pk_mk16(lo, hi)); }
#else
typedef struct { int16x8_t a, b; } pk_sh;  /* pixels 0..7, 8..15 */
#define PK_U8(v) ((uint8x16_t)(v))
#define PK_SHR(v, n) ((pk_b)vshrq_n_u8(PK_U8(v), n))
#define PK_SHL(v, n) ((pk_b)vshlq_n_u8(PK_U8(v), n))
#define PK_NWSH(v, n) ((pk_b)vuzp1q_u8((uint8x16_t)vshrq_n_u16((uint16x8_t)(v).a, n), (uint8x16_t)vshrq_n_u16((uint16x8_t)(v).b, n)))
#define PK_2(f, x, y) ((pk_sh){ f((x).a, (y).a), f((x).b, (y).b) })
static inline pk_b pk_ld(const void *p) { return (pk_b)vld1q_u8((const uint8_t *)p); }
static inline void pk_st(void *p, pk_b v) { vst1q_u8((uint8_t *)p, PK_U8(v)); }
static inline pk_b pk_umin(pk_b a, pk_b b) { return (pk_b)vminq_u8(PK_U8(a), PK_U8(b)); }
static inline pk_b pk_smin(pk_b a, pk_b b) { return (pk_b)vminq_s8((int8x16_t)a, (int8x16_t)b); }
static inline pk_b pk_smax(pk_b a, pk_b b) { return (pk_b)vmaxq_s8((int8x16_t)a, (int8x16_t)b); }
static inline pk_b pk_qsub(pk_b a, pk_b b) { return (pk_b)vqsubq_u8(PK_U8(a), PK_U8(b)); }
static inline pk_b pk_uadd(pk_b a, pk_b v) { return (pk_b)vsqaddq_u8(PK_U8(a), (int8x16_t)v); }
/* a - v saturated to u8; v = -128 adds 127 (only ever clamped to R <= 60 after) */
static inline pk_b pk_usub(pk_b a, pk_b v) { return (pk_b)vsqaddq_u8(PK_U8(a), vqnegq_s8((int8x16_t)v)); }
static inline pk_b pk_rsh1(pk_b a) { return (pk_b)vrshrq_n_u8(PK_U8(a), 1); }        /* (a + 1) >> 1 */
static inline pk_b pk_qabs(pk_b a) { return (pk_b)vqabsq_s8((int8x16_t)a); }
static inline pk_b pk_tbl(uint8x16_t t, pk_b i) { return (pk_b)vqtbl1q_u8(t, PK_U8(i)); }
static inline int pk_any(pk_b v) { return vmaxvq_u8(PK_U8(v)) != 0; }
static inline unsigned pk_sum(pk_b v) { return vaddlvq_u8(PK_U8(v)); }
static inline unsigned pk_cnt(pk_b m) { return (uint8_t)(0u - vaddvq_u8(PK_U8(m))); }   /* (0xff lanes) */
/* a's lanes in the 0x11 bits, b's in the 0x22 bits of 64 (a pair of lanes a byte) */
static inline uint64_t pk_flag2(pk_b a, pk_b b)
{
    return vget_lane_u64(vreinterpret_u64_u8(vshrn_n_u16((uint16x8_t)((a & 0x11) | (b & 0x22)), 4)), 0);
}
static inline pk_sh pk_h16(int c) { int16x8_t v = vdupq_n_s16((int16_t)c); return (pk_sh){ v, v }; }
#define PK_LB(v, i) ((pk_b)vdupq_laneq_u8(v, i))
static inline pk_sh pk_neg16(pk_sh a) { int16x8_t h = vnegq_s16(a.a); return (pk_sh){ h, h }; }   /* (a splat) */
#define PK_LH(v, i) ({ int16x8_t h_ = vdupq_laneq_s16((int16x8_t)(v), i); (pk_sh){ h_, h_ }; })
static inline pk_sh pk_min16(pk_sh a, pk_sh b) { return PK_2(vminq_s16, a, b); }
static inline pk_sh pk_max16(pk_sh a, pk_sh b) { return PK_2(vmaxq_s16, a, b); }
static inline pk_sh pk_abs16(pk_sh a) { return (pk_sh){ vabsq_s16(a.a), vabsq_s16(a.b) }; }
static inline pk_sh pk_sel16(pk_b m, pk_sh a, pk_sh b)
{
    return (pk_sh){ vbslq_s16((uint16x8_t)vmovl_s8(vget_low_s8((int8x16_t)m)), a.a, b.a), vbslq_s16((uint16x8_t)vmovl_high_s8((int8x16_t)m), a.b, b.b) };
}
static inline pk_sh pk_addz(pk_sh a, pk_b v)
{
    return (pk_sh){ (int16x8_t)vaddw_u8((uint16x8_t)a.a, vget_low_u8(PK_U8(v))), (int16x8_t)vaddw_high_u8((uint16x8_t)a.b, PK_U8(v)) };
}
static inline pk_sh pk_subz(pk_sh a, pk_b v)
{
    return (pk_sh){ (int16x8_t)vsubw_u8((uint16x8_t)a.a, vget_low_u8(PK_U8(v))), (int16x8_t)vsubw_high_u8((uint16x8_t)a.b, PK_U8(v)) };
}
static inline pk_sh pk_adds(pk_sh a, pk_b v) { return (pk_sh){ vaddw_s8(a.a, vget_low_s8((int8x16_t)v)), vaddw_high_s8(a.b, (int8x16_t)v) }; }
static inline pk_sh pk_subs(pk_sh a, pk_b v) { return (pk_sh){ vsubw_s8(a.a, vget_low_s8((int8x16_t)v)), vsubw_high_s8(a.b, (int8x16_t)v) }; }
static inline pk_b pk_nw(pk_sh v) { return (pk_b)vuzp1q_u8((uint8x16_t)v.a, (uint8x16_t)v.b); }
static inline pk_b pk_qnw(pk_sh v) { return (pk_b)vqmovn_high_s16(vqmovn_s16(v.a), v.b); }
static inline pk_b pk_eq16(pk_sh v, int c)
{
    int16x8_t k = vdupq_n_s16((int16_t)c);
    return (pk_b)vuzp1q_u8((uint8x16_t)vceqq_s16(v.a, k), (uint8x16_t)vceqq_s16(v.b, k));
}
static inline pk_b pk_ovb(pk_sh v, pk_sh k)
{
    return (pk_b)vqmovn_high_u16(vqmovn_u16(vqsubq_u16((uint16x8_t)vabsq_s16(v.a), (uint16x8_t)k.a)), vqsubq_u16((uint16x8_t)vabsq_s16(v.b), (uint16x8_t)k.b));
}
static inline void pk_ldd(const uint32_t *p, pk_sh *lo, pk_sh *dc)
{
    uint16x8x2_t v = vld2q_u16((const uint16_t *)p), u = vld2q_u16((const uint16_t *)p + 16);
    *lo = (pk_sh){ (int16x8_t)v.val[0], (int16x8_t)u.val[0] }; *dc = (pk_sh){ (int16x8_t)v.val[1], (int16x8_t)u.val[1] };
}
/* (zip, not st2: gcc builds an st2's register pair on the stack) */
static inline void pk_std(uint32_t *p, pk_b lo, pk_b hi, pk_sh dc)
{
    uint16x8_t w0 = (uint16x8_t)vzip1q_u8(PK_U8(lo), PK_U8(hi)), w1 = (uint16x8_t)vzip2q_u8(PK_U8(lo), PK_U8(hi));
    uint16_t *q = (uint16_t *)p;
    vst1q_u16(q, vzip1q_u16(w0, (uint16x8_t)dc.a)); vst1q_u16(q + 8, vzip2q_u16(w0, (uint16x8_t)dc.a));
    vst1q_u16(q + 16, vzip1q_u16(w1, (uint16x8_t)dc.b)); vst1q_u16(q + 24, vzip2q_u16(w1, (uint16x8_t)dc.b));
}
static inline pk_sh pk_ldw(const uint16_t *p) { return (pk_sh){ (int16x8_t)vld1q_u16(p), (int16x8_t)vld1q_u16(p + 8) }; }
static inline void pk_stw(uint16_t *p, pk_b lo, pk_b hi)
{
    vst1q_u8((uint8_t *)p, vzip1q_u8(PK_U8(lo), PK_U8(hi))); vst1q_u8((uint8_t *)p + 16, vzip2q_u8(PK_U8(lo), PK_U8(hi)));
}
#endif
#elif defined(__SSE2__)
#include <emmintrin.h>
#define PK_N 16
typedef uint8_t pk_b __attribute__((vector_size(16)));
typedef int8_t pk_sb __attribute__((vector_size(16)));
typedef struct { __m128i a, b; } pk_sh;    /* pixels 0..7, 8..15 */
#define PK_M(v) ((__m128i)(v))
#define PK_SHR(v, n) ((pk_b)_mm_srli_epi16(PK_M(v), n) & (uint8_t)(0xff >> (n)))
#define PK_SHL(v, n) ((pk_b)_mm_slli_epi16(PK_M(v), n) & (uint8_t)(0xff << (n)))
#define PK_NWSH(v, n) pk_nw((pk_sh){ _mm_srli_epi16((v).a, n), _mm_srli_epi16((v).b, n) })
#define PK_2(f, x, y) ((pk_sh){ f((x).a, (y).a), f((x).b, (y).b) })
static inline pk_b pk_ld(const void *p) { return (pk_b)_mm_loadu_si128((const __m128i *)p); }
static inline void pk_st(void *p, pk_b v) { _mm_storeu_si128((__m128i *)p, PK_M(v)); }
static inline pk_b pk_umin(pk_b a, pk_b b) { return (pk_b)_mm_min_epu8(PK_M(a), PK_M(b)); }
static inline pk_b pk_smin(pk_b a, pk_b b) { pk_b m = (pk_b)((pk_sb)a > (pk_sb)b); return (b & m) | (a & ~m); }
static inline pk_b pk_smax(pk_b a, pk_b b) { pk_b m = (pk_b)((pk_sb)a > (pk_sb)b); return (a & m) | (b & ~m); }
static inline pk_b pk_qsub(pk_b a, pk_b b) { return (pk_b)_mm_subs_epu8(PK_M(a), PK_M(b)); }
static inline pk_b pk_uadd(pk_b a, pk_b v)
{
    pk_b m = (pk_b)((pk_sb)v < 0);
    return (pk_b)_mm_subs_epu8(_mm_adds_epu8(PK_M(a), PK_M(v & ~m)), PK_M(-v & m));
}
static inline pk_b pk_usub(pk_b a, pk_b v)
{
    pk_b m = (pk_b)((pk_sb)v < 0);
    return (pk_b)_mm_adds_epu8(_mm_subs_epu8(PK_M(a), PK_M(v & ~m)), PK_M(-v & m));
}
static inline pk_b pk_rsh1(pk_b a) { return (pk_b)_mm_avg_epu8(PK_M(a), _mm_setzero_si128()); }
static inline pk_b pk_qabs(pk_b a) { pk_b m = (pk_b)((pk_sb)a < 0); return ((pk_b)_mm_subs_epi8(_mm_setzero_si128(), PK_M(a)) & m) | (a & ~m); }
static inline int pk_any(pk_b v) { return _mm_movemask_epi8(_mm_cmpeq_epi8(PK_M(v), _mm_setzero_si128())) != 0xffff; }
static inline unsigned pk_sum(pk_b v)
{
    __m128i s = _mm_sad_epu8(PK_M(v), _mm_setzero_si128());
    return (unsigned)(_mm_cvtsi128_si32(s) + _mm_extract_epi16(s, 4));
}
static inline pk_sh pk_h16(int c) { __m128i v = _mm_set1_epi16((short)c); return (pk_sh){ v, v }; }
static inline pk_sh pk_add16(pk_sh a, pk_sh b) { return PK_2(_mm_add_epi16, a, b); }
static inline pk_sh pk_min16(pk_sh a, pk_sh b) { return PK_2(_mm_min_epi16, a, b); }
static inline pk_sh pk_max16(pk_sh a, pk_sh b) { return PK_2(_mm_max_epi16, a, b); }
static inline pk_sh pk_abs16(pk_sh a) { return pk_max16(a, PK_2(_mm_sub_epi16, pk_h16(0), a)); }
static inline pk_sh pk_sx(pk_b v) { return (pk_sh){ _mm_srai_epi16(_mm_unpacklo_epi8(PK_M(v), PK_M(v)), 8), _mm_srai_epi16(_mm_unpackhi_epi8(PK_M(v), PK_M(v)), 8) }; }
static inline pk_sh pk_sel16(pk_b m, pk_sh a, pk_sh b)
{
    pk_sh k = pk_sx(m);
    return (pk_sh){ _mm_or_si128(_mm_and_si128(k.a, a.a), _mm_andnot_si128(k.a, b.a)), _mm_or_si128(_mm_and_si128(k.b, a.b), _mm_andnot_si128(k.b, b.b)) };
}
static inline pk_sh pk_zx(pk_b v) { __m128i z = _mm_setzero_si128(); return (pk_sh){ _mm_unpacklo_epi8(PK_M(v), z), _mm_unpackhi_epi8(PK_M(v), z) }; }
static inline pk_sh pk_addz(pk_sh a, pk_b v) { return pk_add16(a, pk_zx(v)); }
static inline pk_sh pk_subz(pk_sh a, pk_b v) { return PK_2(_mm_sub_epi16, a, pk_zx(v)); }
static inline pk_sh pk_adds(pk_sh a, pk_b v) { return pk_add16(a, pk_sx(v)); }
static inline pk_sh pk_subs(pk_sh a, pk_b v) { return PK_2(_mm_sub_epi16, a, pk_sx(v)); }
static inline pk_b pk_nw(pk_sh v)
{
    __m128i m = _mm_set1_epi16(0xff);
    return (pk_b)_mm_packus_epi16(_mm_and_si128(v.a, m), _mm_and_si128(v.b, m));
}
static inline pk_b pk_qnw(pk_sh v) { return (pk_b)_mm_packs_epi16(v.a, v.b); }
static inline pk_b pk_eq16(pk_sh v, int c) { __m128i k = _mm_set1_epi16((short)c); return (pk_b)_mm_packs_epi16(_mm_cmpeq_epi16(v.a, k), _mm_cmpeq_epi16(v.b, k)); }
static inline pk_b pk_ovb(pk_sh v, pk_sh k)
{
    pk_sh a = pk_abs16(v);
    return (pk_b)_mm_packus_epi16(_mm_subs_epu16(a.a, k.a), _mm_subs_epu16(a.b, k.b));
}
static inline void pk_ldd(const uint32_t *p, pk_sh *lo, pk_sh *dc)
{
    __m128i q0 = _mm_loadu_si128((const __m128i *)p), q1 = _mm_loadu_si128((const __m128i *)p + 1);
    __m128i q2 = _mm_loadu_si128((const __m128i *)p + 2), q3 = _mm_loadu_si128((const __m128i *)p + 3);
#define PK_LO16(q) _mm_srai_epi32(_mm_slli_epi32(q, 16), 16)
    *lo = (pk_sh){ _mm_packs_epi32(PK_LO16(q0), PK_LO16(q1)), _mm_packs_epi32(PK_LO16(q2), PK_LO16(q3)) };
#undef PK_LO16
    *dc = (pk_sh){ _mm_packs_epi32(_mm_srai_epi32(q0, 16), _mm_srai_epi32(q1, 16)), _mm_packs_epi32(_mm_srai_epi32(q2, 16), _mm_srai_epi32(q3, 16)) };
}
static inline void pk_std(uint32_t *p, pk_b lo, pk_b hi, pk_sh dc)
{
    __m128i w0 = _mm_unpacklo_epi8(PK_M(lo), PK_M(hi)), w1 = _mm_unpackhi_epi8(PK_M(lo), PK_M(hi));
    _mm_storeu_si128((__m128i *)p, _mm_unpacklo_epi16(w0, dc.a)); _mm_storeu_si128((__m128i *)p + 1, _mm_unpackhi_epi16(w0, dc.a));
    _mm_storeu_si128((__m128i *)p + 2, _mm_unpacklo_epi16(w1, dc.b)); _mm_storeu_si128((__m128i *)p + 3, _mm_unpackhi_epi16(w1, dc.b));
}
static inline pk_sh pk_ldw(const uint16_t *p) { return (pk_sh){ _mm_loadu_si128((const __m128i *)p), _mm_loadu_si128((const __m128i *)p + 1) }; }
static inline void pk_stw(uint16_t *p, pk_b lo, pk_b hi)
{
    _mm_storeu_si128((__m128i *)p, _mm_unpacklo_epi8(PK_M(lo), PK_M(hi))); _mm_storeu_si128((__m128i *)p + 1, _mm_unpackhi_epi8(PK_M(lo), PK_M(hi)));
}
#endif
#define PK_B(c) ((pk_b){ 0 } + (uint8_t)(c))
#define PK_EQ(a, b) ((pk_b)((a) == (b)))
#define PK_UGT(a, b) ((pk_b)((a) > (b)))
#define PK_SGT(a, b) ((pk_b)((pk_sb)(a) > (pk_sb)(b)))
static inline pk_b pk_sel(pk_b m, pk_b a, pk_b b) { return (a & m) | (b & ~m); }

/* MT: 0 no matched cuts, 1 matched with model 2, 2 with model 0 / 1; PRE: 0
 * no preemptible run, 2 up to two, 4 up to four */
__attribute__((always_inline))
static inline int pk_rowv(PK_ROW_ARGS, const int SPLIT, const int HACT, const int PRE, const int SNAP, const int HEAL, const int FM, const int MT)
{
#ifndef PK_N
    return pk_row(PK_ROW_PASS, SPLIT, HACT, c);
#else
    if (pk_scalar) return pk_row(PK_ROW_PASS, SPLIT, HACT, c);
    const int R = pl->R;
    const int8_t *net = pl->neta;
    const pk_b Z = PK_B(0);
#ifdef PK_TBL
    /* one register per table, a splat is a lane dup in the step (each
     * table passes an empty asm a step: hoisted, the splats spill).
     * P (the claim): the preempt's net to come of owners 0 / 1 by k << 2 |
     * (to white) << 1 | (from white) at 0..7 (prn is [k][4]); what a pixel
     * ta / td / ok took, its stale value at 8 / 9 / 10 and its busy value at
     * 12 / 13 / 14; owner 3 at 15. PRN: all four owners' (4 owners only).
     * U: the other uniforms, owners 0 / 1 / 2 at 11 / 14 / 15, 30000 in
     * half 6. X: the gather's net by (to white) << 1 | (from white), its
     * model masks at 7 / 11 */
    uint8_t pt[16];
    memcpy(pt, j->prn, 8);
    pt[8] = pl->va; pt[9] = pl->vd; pt[10] = 1; pt[11] = 0; pt[12] = pl->oa; pt[13] = pl->od; pt[14] = pl->hown; pt[15] = j->pob[3];
    const uint8_t xt[16] = { (uint8_t)net[0 << 5 | 0], (uint8_t)net[0 << 5 | 30], (uint8_t)net[30 << 5 | 0], (uint8_t)net[30 << 5 | 30],
                             0, 0, 0, (uint8_t)-(pl->model == 0), 0, 0, 0, (uint8_t)-(pl->model == 2) };
    const uint16_t ut[8] = { (uint16_t)(R | (pl->aclamp != 0 ? 0xFF00u : 0)), (uint16_t)((uint8_t)-(pl->opm != 0) | (pl->fm == 1 ? 0xFF00u : 0)),
                             (uint16_t)((uint8_t)-(pl->fm == 2) | (pl->split == 0 ? 0xFF00u : 0)), (uint16_t)((uint8_t)-(pl->split != 0 && pl->od != 0) | pl->hmin << 8),
                             (uint16_t)(pl->hmax | pl->hf0 << 8), (uint16_t)(pl->hf1 | j->pob[0] << 8), 30000, (uint16_t)(j->pob[1] | j->pob[2] << 8) };
    uint8x16_t P = vld1q_u8(pt), X = vld1q_u8(xt), U = vreinterpretq_u8_u16(vld1q_u16(ut));
    const uint8x16_t PRN = vld1q_u8((const uint8_t *)j->prn);
#define PK_UNI __asm__("" : "+w"(P), "+w"(U)); \
        if (SPLIT || !MT) __asm__("" : "+w"(X)); \
        const pk_b ONE = PK_B(1), RR = PK_LB(U, 0), MA = PK_LB(U, 1), MO = MT == 1 ? ~Z : PK_LB(U, 2), FM1 = PK_LB(U, 3), FM2 = PK_LB(U, 4); \
        const pk_b NSP = PK_LB(U, 5), SPD = PK_LB(U, 6), HMIN = PK_LB(U, 7), HMAX = PK_LB(U, 8), F0 = PK_LB(U, 9), F1 = PK_LB(U, 10); \
        const pk_b O0 = PK_LB(U, 11), O1 = PK_LB(U, 14), O2 = PK_LB(U, 15), O3 = PK_LB(P, 15); \
        const pk_b N00 = PK_LB(X, 0), N0W = PK_LB(X, 1), NW0 = PK_LB(X, 2), NWW = PK_LB(X, 3); \
        const pk_sh C30K = PK_LH(U, 6), CM30K = pk_neg16(C30K);
#else
    const pk_b ONE = PK_B(1), RR = PK_B(R), MA = PK_B(-(pl->aclamp != 0));
    const pk_b MO = MT == 1 ? ~Z : PK_B(-(pl->opm != 0));     /* (model 2: opm) */
    const pk_b FM1 = PK_B(-(pl->fm == 1)), FM2 = PK_B(-(pl->fm == 2));
    const pk_b NSP = PK_B(-(pl->split == 0)), SPD = PK_B(-(pl->split != 0 && pl->od != 0));
    const pk_b HMIN = PK_B(pl->hmin), HMAX = PK_B(pl->hmax), F0 = PK_B(pl->hf0), F1 = PK_B(pl->hf1);
    const pk_b M0 = MT == 1 ? Z : PK_B(-(pl->model == 0)), M2 = MT == 1 ? ~Z : PK_B(-(pl->model == 2));
    const pk_b O0 = PK_B(j->pob[0]), O1 = PK_B(j->pob[1]), O2 = PK_B(j->pob[2]), O3 = PK_B(j->pob[3]);
    const pk_b N00 = PK_B(net[0 << 5 | 0]), N0W = PK_B(net[0 << 5 | 30]), NW0 = PK_B(net[30 << 5 | 0]), NWW = PK_B(net[30 << 5 | 30]);
    const pk_sh C30K = pk_h16(30000), CM30K = pk_h16(-30000);
    const pk_b OA = PK_B(pl->oa), OD = PK_B(pl->od), HOWN = PK_B(pl->hown), VA = PK_B(pl->va), VD = PK_B(pl->vd);
    const pk_b A0 = PK_B(j->prn[0][0]), A1 = PK_B(j->prn[0][1]), A2 = PK_B(j->prn[0][2]), A3 = PK_B(j->prn[0][3]);
    const pk_b B0 = PK_B(j->prn[1][0]), B1 = PK_B(j->prn[1][1]), B2 = PK_B(j->prn[1][2]), B3 = PK_B(j->prn[1][3]);
    const pk_b C0 = PK_B(j->prn[2][0]), C1 = PK_B(j->prn[2][1]), C2 = PK_B(j->prn[2][2]), C3 = PK_B(j->prn[2][3]);
    const pk_b D0 = PK_B(j->prn[3][0]), D1 = PK_B(j->prn[3][1]), D2 = PK_B(j->prn[3][2]), D3 = PK_B(j->prn[3][3]);
#define PK_UNI
#endif
#ifdef PK_TBL
    uint64_t acc = 0;    /* (ta in 0x11.., td in 0x22..) */
    /* (the counts: one addv a step into a general register; a vector each,
     * they spill) */
    unsigned np = 0, ns = 0, nf = 0, nh = 0;
#define PK_COUNT(n, m) ((n) += pk_cnt(m))
#define PK_TOTAL(n) (n)
#define PK_ACC(a, b) (acc |= pk_flag2(a, b))
#define PK_ANY_A (acc & 0x1111111111111111u)
#define PK_ANY_D (acc & 0x2222222222222222u)
#define PK_ANY acc
#else
    pk_b acc = Z;  /* ta in 0xf0, td in 0x0f */
    pk_b np = Z, ns = Z, nf = Z, nh = Z;
#define PK_COUNT(n, m) ((n) -= (m))
#define PK_TOTAL(n) pk_sum(n)
#define PK_ACC(a, b) (acc |= pk_sel(PK_B(0xF0), a, b))
#define PK_ANY_A pk_any(acc & 0xF0)
#define PK_ANY_D pk_any(acc & 0x0F)
#define PK_ANY pk_any(acc)
#endif
    pk_b ov = Z;
#ifdef PK_NO_SKIP
    const int QS = 0;                           /* (built without claim_skip: the step as FIN alone, for a device A/B of the codegen) */
#else
    const int QS = !FM && claim_skip_of();     /* (claim_skip) */
#endif
    int x = 0;
    for (; x + PK_N <= n; x += PK_N) {
        PK_UNI
        pk_sh lo, dc, wv = pk_ldw(Wd + x);
        pk_ldd(D + x, &lo, &dc);
        const pk_b tr = pk_ld(T + x);                      /* (T's bits 5..7 are the stagger's requests, kept) */
        pk_b et = tr & 31, t = pk_ld(tg + x);
        const pk_b fin = pk_eq16(wv, EPD_PK_FIN);          /* (a finish's pixel: its level is T's) */
        pk_b l = pk_sel(fin, et, pk_nw(lo) & 31), o = PK_NWSH(lo, 5) & 63, s = PK_NWSH(lo, 11) & 1;
        const pk_b kh = PK_NWSH(lo, 8) & 0xF0;
        pk_b b = (PK_NWSH(wv, 10) + 1) & ~(pk_eq16(wv, EPD_PK_IDLE) | fin);
        pk_b chg = fin;                         /* (lanes whose words change: else no store) */
        if (QS) { /* claim_skip: a step no decision can touch (every lane idle, not FIN, at its
                   * target, no heal; no flash: the kind) writes only the row buffers' zeros */
            pk_b q = pk_eq16(wv, EPD_PK_IDLE) & PK_EQ(t, l);
            if (HEAL) {
                const pk_b w = PK_EQ(l, 30), bl = PK_EQ(l, 0);
                q &= ~((w | bl) & ~PK_UGT(HMIN, pk_sel(w, RR - o, o)));
            }
            if (!pk_any(~q)) {
                if (SPLIT) ov |= pk_ovb(dc, C30K);
                else {
                    pk_st(fl + x, Z);
                    if (!MT) {
                        const pk_b lb = PK_EQ(l, 30), tw = PK_EQ(t, 30);
#ifdef PK_TBL
                        pk_st(nrow + x, pk_tbl(X, (tw & 2) | (lb & 1)));
#else
                        pk_st(nrow + x, pk_sel(tw, pk_sel(lb, NWW, NW0), pk_sel(lb, N0W, N00)));
#endif
                    }
                }
                continue;
            }
        }
        const pk_b t30 = PK_EQ(t, 30), t0 = PK_EQ(t, 0), rt = t0 | t30;
        if (PRE) { /* preempt_owners_row */
            pk_b tw = PK_EQ(et, 30), lw = PK_EQ(l, 30);
            pk_b bw = rt & (PK_EQ(et, 0) | tw) & (PK_EQ(l, 0) | lw);
            pk_b k0 = PK_EQ(b, O0), k1 = PK_EQ(b, O1), k2 = PRE > 2 ? PK_EQ(b, O2) : Z, k3 = PRE > 2 ? PK_EQ(b, O3) : Z;
            pk_b m = (k0 | k1 | k2 | k3) & bw & ~PK_EQ(t, et);
#ifdef PK_TBL
            pk_b r = pk_tbl(PRE > 2 ? PRN : P, (k1 & 4) | (k2 & 8) | (k3 & 12) | (tw & 2) | (lw & 1));
#else
            pk_b r = (pk_sel(tw, pk_sel(lw, A3, A2), pk_sel(lw, A1, A0)) & k0) | (pk_sel(tw, pk_sel(lw, B3, B2), pk_sel(lw, B1, B0)) & k1);
            if (PRE > 2)
                r |= (pk_sel(tw, pk_sel(lw, C3, C2), pk_sel(lw, C1, C0)) & k2) | (pk_sel(tw, pk_sel(lw, D3, D2), pk_sel(lw, D1, D0)) & k3);
#endif
            pk_b s2 = pk_umin(pk_usub(o, r), RR);
            o = pk_sel(m & MO, s2, o);
            dc = pk_sel16(m, pk_max16(pk_min16(pk_subs(dc, r), C30K), CM30K), dc);
            l = pk_sel(m, et, l);
            b &= ~m;
            chg |= m;
            PK_COUNT(np, m);
        }
        const pk_b z0 = PK_EQ(b, 0);            /* (idle after the preempt) */
        if (SNAP) { /* snap_row (snp: the kind) */
            pk_b oo = o + o;
            pk_b up = z0 & PK_EQ(l, 0) & t30 & PK_UGT(oo, RR), dn = z0 & PK_EQ(l, 30) & t0 & PK_UGT(RR, oo);
            l = pk_sel(up, PK_B(30), l & ~dn);
            chg |= up | dn;
            PK_COUNT(ns, up | dn);
        }
        const pk_b fx = FM ? FM1 | (FM2 & -s) : Z;
        pk_b ta, td, ok = Z;
        if (!HACT) { /* claim_row */
            pk_b chg = z0 & (fx | ~PK_EQ(t, l));
            pk_b bw = rt & (PK_EQ(l, 0) | PK_EQ(l, 30));
            ta = chg & (bw | NSP); td = chg & SPD & ~bw;
            et = pk_sel(ta | td, t, et);
        } else { /* claim_row_heal */
            pk_b h = PK_UGT(b, PK_B(HEAL_B - 1));
            pk_b lr = pk_sel(h, et, l);
            pk_b chg = (z0 | h) & (fx | ~PK_EQ(t, lr));
            pk_b bw = rt & (PK_EQ(lr, 0) | PK_EQ(lr, 30));
            ta = chg & (bw | NSP); td = chg & SPD & ~bw;
            pk_b mw = ta | td;
            pk_b rem = pk_qsub(pk_rsh1(l), pk_sel(PK_EQ(b, PK_B(HEAL_B)), F0, F1)) & h & mw;
            pk_b ru = rem & PK_EQ(lr, 30), rd = rem & PK_EQ(lr, 0);
            o = o - ru + rd;
            dc = pk_addz(pk_subz(dc, ru), rd);
            l = pk_sel(h & mw, lr, l);
            et = pk_sel(mw, t, et);
        }
        pk_st(ca + x, pk_sel(ta, ONE, pk_ld(ca + x)));
        pk_st(cd + x, pk_sel(td, ONE, pk_ld(cd + x)));
        PK_ACC(ta, td);
        if (HEAL) { /* heal_row (hown: the kind); b == 0 after the claim */
            pk_b w = PK_EQ(l, 30), bl = PK_EQ(l, 0);
            pk_b def = pk_sel(w, RR - o, o);
            ok = z0 & ~(ta | td) & PK_EQ(t, l) & (w | bl) & ~PK_UGT(HMIN, def);
            pk_b k = pk_umin(def, HMAX) & ok;
            pk_b ku = k & w, kd = k & bl;
            o = o + ku - kd;
            dc = pk_subz(pk_addz(dc, ku), kd);
            et = pk_sel(ok, l, et);
            l = pk_sel(ok, k + k - 1, l);
            pk_st(ch + x, pk_sel(ok, ONE, pk_ld(ch + x)));
            PK_COUNT(nh, ok);
        }
        /* what they took: busy (oa / od / hown), stale_row (b == oa is ta, od td, hown ok) */
        const pk_b tk = ta | td | ok;
#ifdef PK_TBL
        pk_b xi = pk_sel(ta, PK_B(8), pk_sel(td, PK_B(9), ok & 10));
        s = pk_sel(tk, pk_tbl(P, xi), s);
        b = pk_sel(tk, pk_tbl(P, xi + 4), b);
#else
        s = pk_sel(ta, VA, pk_sel(td, VD, HEAL ? pk_sel(ok, ONE, s) : s));
        b = pk_sel(ta, OA, pk_sel(td, OD, HEAL ? pk_sel(ok, HOWN, b) : b));
#endif
        const pk_b idle = z0 & ~tk;
        if (FM) PK_COUNT(nf, FM2 & PK_EQ(s, 0) & PK_EQ(t, l) & idle);
        /* back into W, T, and D unless the split's step moves it (bytes, then
         * interleaved; as early as they are known: the live set). only a
         * step with a changed lane stores (the words are the state, not an L1
         * copy: an unchanged line stays clean) */
        const int st = pk_any(chg | tk);
        if (st) {
            pk_stw(Wd + x, l | PK_SHL(et, 5) | idle, pk_sel(idle, PK_B(EPD_PK_IDLE >> 8), PK_SHR(et, 3) | PK_SHL(b - 1, 2)));
            pk_st(T + x, et | (tr & 0xE0));
        }
        const pk_b dh = kh | PK_SHL(s, 3);
        if (!SPLIT && st) pk_std(D + x, l | PK_SHL(o, 5), dh | PK_SHR(o, 3), dc);
        if (SPLIT) { /* dc_row_bw, op_row_bw */
            pk_b l0 = PK_EQ(l, 0);
            pk_b v = pk_sel(t0, pk_sel(l0, N00, N0W), pk_sel(l0, NW0, NWW));
            pk_b a = pk_umin(pk_uadd(o, v), RR);
            o = pk_sel(ta & MO, pk_sel(MA, a, t30 & RR), o);
            dc = pk_sel16(ta, pk_max16(pk_min16(pk_adds(dc, v), C30K), CM30K), dc);
            ov |= pk_ovb(dc, C30K);
            if (st) pk_std(D + x, l | PK_SHL(o, 5), dh | PK_SHR(o, 3), dc);
        } else { /* gather_bw_row (b == oa: ta); with matched cuts its want in a second pass */
            pk_b lb = PK_EQ(l, 30), bw = rt & (PK_EQ(l, 0) | lb);
            pk_st(fl + x, (ta & bw & ONE) | (((ta & ~bw) | td) & 2));
            if (!MT) {
#ifdef PK_TBL
                pk_st(nrow + x, pk_tbl(X, (t30 & 2) | (lb & 1)));
#else
                pk_st(nrow + x, pk_sel(t30, pk_sel(lb, NWW, NW0), pk_sel(lb, N0W, N00)));
#endif
            }
        }
    }
    /* the matched cut (gather_bw_row's want, the cut) as a second pass over
     * what the first stored (fl: its e in bit 0): in the first, the live set
     * spilled. In bytes: need is R - o or o, |v| and v saturated keep every
     * compare and the clamp */
    PK_PROF_T0(tmt);
    if (!SPLIT && MT && (PK_ANY_A || PK_ANY_D))     /* (a row nothing took has no pass B, so no want) */
        for (int y = 0; y < x; y += PK_N) {
#ifdef PK_TBL
            __asm__("" : "+w"(X), "+w"(U));
            const pk_b RR = PK_LB(U, 0), M0 = MT == 1 ? Z : PK_LB(X, 7), M2 = MT == 1 ? ~Z : PK_LB(X, 11);
#endif
            pk_sh lo, dc;
            pk_ldd(D + y, &lo, &dc);
            const pk_b t30 = PK_EQ(pk_ld(tg + y), 30), f = pk_ld(fl + y);
            const pk_b l = pk_nw(lo) & 31, o = PK_NWSH(lo, 5) & 63, lb = PK_EQ(l, 30);
#ifdef PK_TBL
            pk_b nn = pk_tbl(X, (t30 & 2) | (lb & 1));
#else
            pk_b nn = pk_sel(t30, pk_sel(lb, NWW, NW0), pk_sel(lb, N0W, N00));
#endif
            pk_b need = pk_sel(t30, RR - o, o), vs = pk_qnw(dc), av = pk_qnw(pk_abs16(dc));
            pk_b opp = ~PK_EQ(vs, 0) & PK_SGT(Z, vs ^ nn);
            pk_b w1 = PK_SGT(need, pk_qabs(nn));
            pk_b w2 = M2 & opp & (~w1 | PK_SGT(av, need));
            pk_b want = (f << 1) & ~PK_EQ(nn, 0) & pk_sel(M0, opp, w1 | w2);   /* (e in bit 1) */
            pk_b dq = pk_sel(M0 | w2, vs, pk_sel(PK_SGT(nn, Z), -need, need));
            dq = pk_smax(pk_smin(dq, PK_B(EPD_MT_D)), PK_B(-EPD_MT_D)) + EPD_MT_D;
            pk_st(cq + y, (t30 & 0x80) | (lb & 0x40) | dq);
            pk_st(fl + y, f | (want & 2)); pk_st(nrow + y, nn);
        }
    PK_PROF_ADD(epd_pk_prof_mt, tmt);
    const unsigned cp = PK_TOTAL(np), cs = PK_TOTAL(ns), cf = PK_TOTAL(nf), cn = PK_TOTAL(nh);
    int any = (PK_ANY_A ? 1 : 0) | (PK_ANY_D ? 2 : 0) | (cn ? 4 : 0) |
              (SPLIT && pk_any(ov) ? 8 : 0) | (PK_ANY || cp || cs || cn ? 16 : 0);
    c[0] += cp; c[1] += cs; c[2] += cf; c[3] += cn;
#undef PK_COUNT
#undef PK_TOTAL
#undef PK_ACC
#undef PK_ANY_A
#undef PK_ANY_D
#undef PK_ANY
    if (x < n) {
        PK_PROF_T0(ttl);
        any |= pk_row(D + x, Wd + x, T + x, tg + x, ca + x, cd + x, ch + x, fl + x, cq + x, nrow + x, n - x, pl, j, SPLIT, HACT, c);
        PK_PROF_ADD(epd_pk_prof_tail, ttl);
    }
    return any;
#undef PK_UNI
#endif
}
/* one per uniform shape of the plan, so a sweep does only the steps its claim
 * has: (matched cuts, split, preempt) groups x (heal variant, heal, flash);
 * snap only without preempt or matched cuts (claim_plan: snap is never A2,
 * those are only A2). pk_rows[] by pk_row_kind; a shape outside them takes
 * pk_row whole (the last four) */
#define PK_V(mt, sp, pr, ha, sn, he, fm) \
    static int pk_row_##mt##sp##pr##ha##sn##he##fm(PK_ROW_ARGS) { return pk_rowv(PK_ROW_PASS, c, sp, ha, pr, sn, he, fm, mt); }
#define PK_V8(mt, sp, pr, sn) PK_V(mt, sp, pr, 0, sn, 0, 0) PK_V(mt, sp, pr, 0, sn, 0, 1) PK_V(mt, sp, pr, 0, sn, 1, 0) PK_V(mt, sp, pr, 0, sn, 1, 1) \
    PK_V(mt, sp, pr, 1, sn, 0, 0) PK_V(mt, sp, pr, 1, sn, 0, 1) PK_V(mt, sp, pr, 1, sn, 1, 0) PK_V(mt, sp, pr, 1, sn, 1, 1)
#define PK_T8(mt, sp, pr, sn) pk_row_##mt##sp##pr##0##sn##00, pk_row_##mt##sp##pr##0##sn##01, pk_row_##mt##sp##pr##0##sn##10, pk_row_##mt##sp##pr##0##sn##11, \
    pk_row_##mt##sp##pr##1##sn##00, pk_row_##mt##sp##pr##1##sn##01, pk_row_##mt##sp##pr##1##sn##10, pk_row_##mt##sp##pr##1##sn##11
#define PK_G(F) F(0, 0, 0, 0), F(0, 0, 0, 1), F(0, 0, 2, 0), F(0, 0, 4, 0), F(1, 0, 0, 0), F(1, 0, 2, 0), F(1, 0, 4, 0), \
    F(2, 0, 0, 0), F(2, 0, 2, 0), F(2, 0, 4, 0), F(0, 1, 0, 0), F(0, 1, 2, 0), F(0, 1, 4, 0)
PK_V8(0, 0, 0, 0) PK_V8(0, 0, 0, 1) PK_V8(0, 0, 2, 0) PK_V8(0, 0, 4, 0) PK_V8(1, 0, 0, 0) PK_V8(1, 0, 2, 0) PK_V8(1, 0, 4, 0)
PK_V8(2, 0, 0, 0) PK_V8(2, 0, 2, 0) PK_V8(2, 0, 4, 0) PK_V8(0, 1, 0, 0) PK_V8(0, 1, 2, 0) PK_V8(0, 1, 4, 0)
static int pk_row_s00(PK_ROW_ARGS) { return pk_row(PK_ROW_PASS, 0, 0, c); }
static int pk_row_s01(PK_ROW_ARGS) { return pk_row(PK_ROW_PASS, 0, 1, c); }
static int pk_row_s10(PK_ROW_ARGS) { return pk_row(PK_ROW_PASS, 1, 0, c); }
static int pk_row_s11(PK_ROW_ARGS) { return pk_row(PK_ROW_PASS, 1, 1, c); }
static int (*const pk_rows[13 * 8 + 4])(PK_ROW_ARGS) = { PK_G(PK_T8), pk_row_s00, pk_row_s01, pk_row_s10, pk_row_s11 };
/* (snap with split: the split's own row, snp is 0 there: claim_plan; the
 * split's own row is the one without matched cuts) */
static int pk_row_kind(const epd_claim_plan *pl, const epd_claim_job *j)
{
    int split = pl->split && !pl->mt, snap = pl->snp != 0 && !split, hact = pl->hact != 0;
    int mt = pl->mt ? (pl->model == 2 ? 1 : 2) : 0, npo = 0;
    for (int k = 0; k < 4; k++) npo += j->pob[k] != 255;
    int pre = npo == 0 ? 0 : npo <= 2 ? 1 : 2;
    if (snap && (mt || pre)) return 13 * 8 + (split << 1 | hact);
    int g = split ? 10 + pre : mt ? 1 + 3 * mt + pre : snap ? 1 : pre ? 1 + pre : 0;
    return g * 8 | hact << 2 | (pl->hown != 0) << 1 | (pl->fm != 0);
}

/* pass C (gather_bw_apply) on D */
__attribute__((optimize("O3")))
static void pk_apply_row(uint32_t *restrict D, const uint8_t *restrict fl, const uint8_t *restrict tg,
                         const int8_t *restrict nrow, int n, int opm, int aclamp, int R)
{
    const uint8_t mo = (uint8_t)-(opm != 0), ma = (uint8_t)-(aclamp != 0), rr = (uint8_t)R;
    for (int x = 0; x < n; x++) {
        const uint32_t dv = D[x];
        uint8_t o = (uint8_t)((dv >> 5) & 63);
        int16_t dc = (int16_t)(dv >> 16);
        uint8_t e = (uint8_t)-(fl[x] & 1);
        int16_t nn = nrow[x], d = (int16_t)(dc + nn), a = (int16_t)(o + nn);
        d = d < -30000 ? -30000 : d;
        d = d > 30000 ? 30000 : d;
        int16_t m16 = (int16_t)(int8_t)e;
        dc = (int16_t)((d & m16) | (dc & ~m16));
        a = a < 0 ? 0 : a;
        a = a > rr ? rr : a;
        uint8_t rl = (uint8_t)(rr & -(uint8_t)(tg[x] == 30));
        uint8_t v = (uint8_t)(((uint8_t)a & ma) | (rl & ~ma)), mop = e & mo;
        o = (uint8_t)((v & mop) | (o & ~mop));
        D[x] = (dv & 0xF81Fu) | (uint32_t)o << 5 | (uint32_t)(uint16_t)dc << 16;
    }
}

int epd_claim_bandable(const epd_claim_plan *pl)
{
    return !pl->lazy && pl->npre <= 4;
}

void epd_claim_prep(epd_engine *e, epd_claim_plan *pl, epd_claim_job *j)
{
    if (!pl->tg) pl->tg = pend_tg(e, pl->pend);     /* (a GPU target: its levels fetched) */
    if (pl->mt)
        for (int k = 0; k < 4; k++) {               /* (as claim_pixels_soa's ctab) */
            int i = ((k & 2) ? 30 : 0) << 5 | ((k & 1) ? 30 : 0);
            for (int q = 0; q < 64; q++) {
                int c = q < 2 * EPD_MT_D + 1 ? pl->best[i][q] : pl->cs;
                if (pl->model && c > pl->cs) c = pl->cs;
                if (pl->cap && c < pl->cs - pl->cap) c = pl->cs - pl->cap;
                int hit = c != pl->cs && pl->mown[c];
                j->ctab[k << 6 | q].own = hit ? pl->mown[c] : 0;
                j->ctab[k << 6 | q].net = hit ? pl->mnet[c][i] : pl->neta[i];
                j->ctab[k << 6 | q].bit = (uint16_t)(hit ? 1u << c : 0);
            }
        }
    j->l30 = 30;
    memset(j->pob, 255, sizeof j->pob);
    memset(j->prn, 0, sizeof j->prn);
    int npo = 0;
    if (pl->npre && pl->npre <= 4)
        for (int v = 1; v < 256 && npo < pl->npre; v++)
            if (pl->pre[v]) { j->pob[npo] = (uint8_t)v; memcpy(j->prn[npo], pl->rn[pl->pre[v]], 4); npo++; }
}

/* a band's report cleared; its row kernel */
static int (*pk_band_init(const epd_claim_plan *pl, const epd_claim_job *j, epd_claim_band *B))(PK_ROW_ARGS)
{
    const int W = pl->r.w;
    memset(B->ca, 0, (size_t)W);
    memset(B->cd, 0, (size_t)W);
    if (pl->hown) memset(B->ch, 0, (size_t)W);
    memset(B->mcol, 0, (size_t)W * sizeof B->mcol[0]);
    for (int c = 0; c < EPD_LUT_CUTS; c++) { B->mb1[c] = INT32_MAX; B->mb3[c] = -1; }
    for (int k = 0; k < 3; k++) B->y0[k] = B->y1[k] = -1;
    B->preempted = B->snapped = B->matched = B->healed = B->flash_skipped = 0;
    return pk_rows[pk_row_kind(pl, j)];
}

/* tail16 (; epd_claim_tail16, -1 = GLASSINE_CLAIM_TAIL16 at
 * epd_init, default 1, 0 = off): a row's last W % 16 pixels as one vector
 * step over a 16-lane copy whose other lanes can't change (idle, at their
 * target 15, off the rails, stale 0, DC 0) instead of the scalar pk_row; not
 * for a flash plan (fm: its idle pixels count or claim whatever they hold) */
static int pk_tail_on(const epd_claim_plan *pl, int W)
{
    if (!(W & 15) || pl->fm || pk_scalar) return 0;
    int k = __atomic_load_n(&epd_claim_tail16, __ATOMIC_RELAXED);
    if (k < 0) k = KNOB_DEF(epd_claim_tail16);
    return k;
}
static int pk_tail16(epd_claim_band *B, int (*row)(PK_ROW_ARGS), uint32_t *restrict d, uint16_t *restrict w, uint8_t *restrict t,
                     const uint8_t *restrict tg, int x, int m, const epd_claim_plan *pl, const epd_claim_job *j, unsigned *c)
{
    typeof(B->tl) *L = &B->tl;
    for (int i = 0; i < 16; i++) {
        L->d[i] = 15; L->w[i] = EPD_PK_IDLE; L->t[i] = 15; L->tg[i] = 15;
        L->ca[i] = L->cd[i] = L->ch[i] = 0;
    }
    memcpy(L->d, d + x, (size_t)m * 4); memcpy(L->w, w + x, (size_t)m * 2); memcpy(L->t, t + x, (size_t)m);
    memcpy(L->tg, tg + x, (size_t)m);
    memcpy(L->ca, B->ca + x, (size_t)m); memcpy(L->cd, B->cd + x, (size_t)m); memcpy(L->ch, B->ch + x, (size_t)m);
    int any = row(L->d, L->w, L->t, L->tg, L->ca, L->cd, L->ch, L->fl, L->cq, L->nrow, 16, pl, j, c);
    memcpy(d + x, L->d, (size_t)m * 4); memcpy(w + x, L->w, (size_t)m * 2); memcpy(t + x, L->t, (size_t)m);
    memcpy(B->ca + x, L->ca, (size_t)m); memcpy(B->cd + x, L->cd, (size_t)m); memcpy(B->ch + x, L->ch, (size_t)m);
    memcpy(B->fl + x, L->fl, (size_t)m); memcpy(B->cq + x, L->cq, (size_t)m); memcpy(B->nrow + x, L->nrow, (size_t)m);
    return any;
}

/* rect row y of a claim on its words d / w / t (the row's D, W, T from the
 * rect's first column: the band's L1 copy, or the state in place).
 * Returns the sweep's bits: (any & 3) || (any & 16) = a word changed */
__attribute__((always_inline))
static inline int pk_claim_row(const epd_claim_plan *pl, const epd_claim_job *j, epd_claim_band *B, int (*row)(PK_ROW_ARGS),
                               int y, uint32_t *restrict d, uint16_t *restrict w, uint8_t *restrict t, epd_pk *mk)
{
    const int W = pl->r.w, split = pl->split && !pl->mt, opm = pl->opm, R = pl->R, aclamp = pl->aclamp;
    const uint8_t oa = pl->oa, od = pl->od;
    {
        const uint8_t *tg = pl->tg + (size_t)y * W;
        unsigned c[4] = { 0 };
        int any;
        if (!pk_tail_on(pl, W)) any = row(d, w, t, tg, B->ca, B->cd, B->ch, B->fl, B->cq, B->nrow, W, pl, j, c);
        else {
            const int n16 = W & ~15;
            any = n16 ? row(d, w, t, tg, B->ca, B->cd, B->ch, B->fl, B->cq, B->nrow, n16, pl, j, c) : 0;
            any |= pk_tail16(B, row, d, w, t, tg, n16, W - n16, pl, j, c);
        }
        B->preempted += c[0]; B->snapped += c[1]; B->flash_skipped += c[2]; B->healed += c[3];
        if (mk) {                               /* (finish_mask: the owners this row's pixels took) */
            if (any & 1) pk_mark(mk, oa, pl->r.y + y);
            if (any & 2) pk_mark(mk, od, pl->r.y + y);
            if (any & 4) pk_mark(mk, pl->hown, pl->r.y + y);
        }
        if (any & 4) { if (B->y0[EPD_CL_H] < 0) B->y0[EPD_CL_H] = y; B->y1[EPD_CL_H] = y; }
        if (any & 3) {
            if (any & 1) { if (B->y0[EPD_CL_A] < 0) B->y0[EPD_CL_A] = y; B->y1[EPD_CL_A] = y; }
            if (any & 2) { if (B->y0[EPD_CL_D] < 0) B->y0[EPD_CL_D] = y; B->y1[EPD_CL_D] = y; }
            if (split) {
                if ((any & 1) && (any & 8))     /* (dc_row_bw clamps the whole row) */
                    for (int x = 0; x < W; x++) {
                        int16_t dc = (int16_t)(d[x] >> 16);
                        if (dc > 30000 || dc < -30000)
                            d[x] = (d[x] & 0xFFFFu) | (uint32_t)(uint16_t)(dc < 0 ? -30000 : 30000) << 16;
                    }
                if (any & 2) {                  /* od: the gather (DU), its model at the rail */
                    int nl = 0;
                    for (int x = 0; x < W; x++) { B->xl[nl] = (uint16_t)x; nl += !EPD_PK_ISIDLE(w[x]) && (w[x] >> 10) + 1 == od; }
                    for (int k = 0; k < nl; k++) {
                        int x = B->xl[k];
                        uint32_t v = d[x];
                        int l = v & 31, dd = (int16_t)(v >> 16) + pl->netd[tg[x] << 5 | l];
                        dd = dd < -30000 ? -30000 : dd > 30000 ? 30000 : dd;
                        v = (v & 0xFFFFu) | (uint32_t)(uint16_t)dd << 16;
                        if (opm) v = (v & ~(63u << 5)) | (uint32_t)pl->rail[tg[x]] << 5;
                        d[x] = v;
                    }
                }
            } else {                            /* pass B: the cut table and the rest (claim_pixels_soa) */
                uint8_t *fl = B->fl, *cq = B->cq;
                int8_t *nrow = B->nrow;
                uint16_t rb = 0;
                int nl = 0;
                for (int x = 0; x < W; x++) { B->xl[nl] = (uint16_t)x; nl += fl[x] >> 1; }
                uint64_t nm = 0;
                for (int k = 0; k < nl; k++) {
                    int x = B->xl[k];
                    uint32_t v = d[x];
                    uint16_t u = w[x];
                    uint8_t b = EPD_PK_ISIDLE(u) ? 0 : (uint8_t)((u >> 10) + 1), l = (uint8_t)(v & 31);   /* (the sweep stored the row: no FIN left) */
                    if (fl[x] & 1) {
                        int q = cq[x];
                        uint8_t ow = j->ctab[q].own;
                        nrow[x] = j->ctab[q].net;
                        if (ow) w[x] = (uint16_t)((ow - 1) << 10 | (u & 1023));
                        B->mcol[x] |= j->ctab[q].bit;
                        rb |= j->ctab[q].bit;
                        nm += ow != 0;
                        continue;
                    }
                    int dc = (int16_t)(v >> 16);
                    int i = tg[x] << 5 | l, nn = (b == oa ? pl->neta : pl->netd)[i];
                    if (pl->mt && b == oa && nn && !pl->model && dc && (dc < 0) != (nn < 0)) {
                        int dd = dc;
                        dd = dd < -EPD_MT_D ? -EPD_MT_D : dd > EPD_MT_D ? EPD_MT_D : dd;
                        int cc = pl->best[i][dd + EPD_MT_D];
                        if (pl->cap && cc < pl->cs - pl->cap) cc = pl->cs - pl->cap;
                        if (cc != pl->cs && pl->mown[cc]) {
                            w[x] = (uint16_t)((pl->mown[cc] - 1) << 10 | (u & 1023));
                            nn = pl->mnet[cc][i];
                            B->mcol[x] |= (uint16_t)(1u << cc);
                            rb |= (uint16_t)(1u << cc);
                            B->matched++;
                        }
                    }
                    int dd = dc + nn;
                    dd = dd < -30000 ? -30000 : dd > 30000 ? 30000 : dd;
                    v = (v & 0xFFFFu) | (uint32_t)(uint16_t)dd << 16;
                    if (opm) v = (v & ~(63u << 5)) | (uint32_t)pl->rail[tg[x]] << 5;
                    d[x] = v;
                }
                B->matched += nm;
                pk_apply_row(d, fl, tg, nrow, W, opm, aclamp, R);
                for (int c = 0; rb && c < EPD_LUT_CUTS; c++)
                    if (rb & (1u << c)) {
                        if (B->mb1[c] == INT32_MAX) B->mb1[c] = y;
                        B->mb3[c] = y;
                        if (mk) pk_mark(mk, pl->mown[c], pl->r.y + y);
                    }
            }
        }
        return any;
    }
}

void epd_claim_rows(epd_engine *e, const epd_claim_plan *pl, const epd_claim_job *j, epd_claim_band *B, int ya, int yb)
{
    const int W = pl->r.w;
    int (*row)(PK_ROW_ARGS) = pk_band_init(pl, j, B);
    uint32_t *d = B->d;
    uint16_t *w = B->w;
    uint8_t *t = B->t;
    for (int y = ya; y < yb; y++) {
        size_t o = (size_t)(pl->r.y + y) * EPD_W + pl->r.x;
        pk_pack_row(e, o, W, d, w, t);
        int any = pk_claim_row(pl, j, B, row, y, d, w, t, NULL);
        if ((any & 3) || (any & 16)) pk_unpack_row(e, o, W, d, w, t);   /* (else nothing changed: nothing to write back) */
    }
}

/* (claim_state packed, and check's D/W/T): rows [ya, yb) of the claim on
 * the state in place: D and T at the rect, W from w0 (the rect's first
 * pixel), rows wst apart (the state's W, or the job's Wsh) */
static int claim_prefetch_of(void)
{
    int k = __atomic_load_n(&epd_claim_prefetch, __ATOMIC_RELAXED);
    if (k < 0) k = KNOB_DEF(epd_claim_prefetch);    /* (GLASSINE_CLAIM_PREFETCH=1: epd_init) */
    return k;
}

/* claim_prefetch: row y's lines of D, W, T and the target asked for (rows are
 * EPD_PK_S pixels apart: a short rect's row is a new stream each time) */
static inline void pk_prefetch_row(const epd_engine *e, const epd_claim_plan *pl, const uint16_t *w0, size_t wst, int y)
{
    const epd_pk *k = e->pk;
    const int W = pl->r.w;
    const size_t q = (size_t)(pl->r.y + y) * EPD_PK_S + pl->r.x;
    const char *d = (const char *)(k->D + q), *w = (const char *)(w0 + (size_t)y * wst);
    const char *t = (const char *)(k->T + q), *g = (const char *)(pl->tg + (size_t)y * W);
    for (int b = 0; b < 4 * W; b += 64) __builtin_prefetch(d + b, 1, 3);
    for (int b = 0; b < 2 * W; b += 64) __builtin_prefetch(w + b, 1, 3);
    for (int b = 0; b < W; b += 64) { __builtin_prefetch(t + b, 1, 3); __builtin_prefetch(g + b, 0, 3); }
}

static void pk_claim_rows(epd_engine *e, const epd_claim_plan *pl, const epd_claim_job *j, epd_claim_band *B, int ya, int yb,
                          uint16_t *w0, size_t wst)
{
    epd_pk *k = e->pk;
    int (*row)(PK_ROW_ARGS) = pk_band_init(pl, j, B);
    const int pf = claim_prefetch_of();
    if (pf && ya < yb) pk_prefetch_row(e, pl, w0, wst, ya);
    for (int y = ya; y < yb; y++) {
        size_t q = (size_t)(pl->r.y + y) * EPD_PK_S + pl->r.x;
        if (pf && y + 1 < yb) pk_prefetch_row(e, pl, w0, wst, y + 1);
        pk_claim_row(pl, j, B, row, y, k->D + q, w0 + (size_t)y * wst, k->T + q, k);
    }
}

void epd_claim_merge(const epd_claim_plan *pl, epd_claim_band *const *b, int n, epd_claim_out *out)
{
    const int W = pl->r.w;
    epd_claim_band *M = b[0];
    memset(out, 0, sizeof *out);
    for (int c = 0; c < EPD_LUT_CUTS; c++) out->mslot[c] = pl->mown[c] ? pl->mown[c] - 1 : -1;
    for (int k = 0; k < EPD_CL_OWNERS; k++) { out->x0[k] = INT32_MAX; out->y0[k] = out->x1[k] = out->y1[k] = -1; }
    for (int i = 1; i < n; i++) {
        const epd_claim_band *B = b[i];
        for (int x = 0; x < W; x++) { M->ca[x] |= B->ca[x]; M->cd[x] |= B->cd[x]; M->mcol[x] |= B->mcol[x]; }
        if (pl->hown) for (int x = 0; x < W; x++) M->ch[x] |= B->ch[x];
        for (int k = 0; k < 3; k++) {
            if (B->y0[k] >= 0 && (M->y0[k] < 0 || B->y0[k] < M->y0[k])) M->y0[k] = B->y0[k];
            if (B->y1[k] > M->y1[k]) M->y1[k] = B->y1[k];
        }
        for (int c = 0; c < EPD_LUT_CUTS; c++) {
            if (B->mb1[c] < M->mb1[c]) M->mb1[c] = B->mb1[c];
            if (B->mb3[c] > M->mb3[c]) M->mb3[c] = B->mb3[c];
        }
        M->preempted += B->preempted; M->snapped += B->snapped; M->matched += B->matched;
        M->healed += B->healed; M->flash_skipped += B->flash_skipped;
    }
    out->preempted = M->preempted; out->snapped = M->snapped; out->matched = M->matched; out->flash_skipped = M->flash_skipped;
    for (int k = 0; k < 3; k++) { out->y0[k] = M->y0[k]; out->y1[k] = M->y1[k]; }
    ext_cols(out, EPD_CL_A, M->ca, W);
    ext_cols(out, EPD_CL_D, M->cd, W);
    if (pl->hown) ext_cols(out, EPD_CL_H, M->ch, W);
    else { out->x0[EPD_CL_H] = 0; out->x1[EPD_CL_H] = -1; }
    for (int c = 0; c < EPD_LUT_CUTS; c++) {
        int k = EPD_CL_M0 + c;
        if (M->mb3[c] < 0) { out->x0[k] = 0; out->x1[k] = -1; continue; }
        int x0 = 0, x1 = W - 1;
        while (!(M->mcol[x0] & (1u << c))) x0++;
        while (!(M->mcol[x1] & (1u << c))) x1--;
        out->x0[k] = x0; out->y0[k] = M->mb1[c]; out->x1[k] = x1; out->y1[k] = M->mb3[c];
    }
    /* (the heal run's pixels: the ones heal_row took, no other pixel holds its slot) */
    if (pl->hown && out->x1[EPD_CL_H] >= 0) out->healed = M->healed;
}

/* ---- : banded claims --------------------------
 * The rows of a bandable claim are independent : N bands of rows, each
 * into its own epd_claim_band, merged (epd_claim_merge) into what one band
 * makes. The arrays' pass gets a row range of its own (claim_rows_soa: its
 * fast path letter for letter, the bench's band_rows made production); its
 * healed count stays its rule (the heal run's pixels over the merged rect) */

static void claim_rows_soa(epd_engine *e, const epd_claim_plan *pl, const epd_claim_job *j, epd_claim_band *B, int ya, int yb)
{
    static const uint8_t zeros[EPD_W], ones[EPD_W] = { [0 ... EPD_W - 1] = 1 };   /* claim_row's full mask */
    const int W = pl->r.w, split = pl->split, opm = pl->opm, R = pl->R, aclamp = pl->aclamp;
    const uint8_t oa = pl->oa, od = pl->od, hown = pl->hown;
    int npo = 0;
    while (npo < 4 && j->pob[npo] != 255) npo++;
    memset(B->ca, 0, (size_t)W);
    memset(B->cd, 0, (size_t)W);
    if (hown) memset(B->ch, 0, (size_t)W);
    memset(B->mcol, 0, (size_t)W * sizeof B->mcol[0]);
    for (int c = 0; c < EPD_LUT_CUTS; c++) { B->mb1[c] = INT32_MAX; B->mb3[c] = -1; }
    for (int k = 0; k < 3; k++) B->y0[k] = B->y1[k] = -1;
    B->preempted = B->snapped = B->matched = B->healed = B->flash_skipped = 0;
    for (int y = ya; y < yb; y++) {
        size_t o = (size_t)(pl->r.y + y) * EPD_W + pl->r.x;
        uint8_t *b = e->busy + o;
        const uint8_t *tg = pl->tg + (size_t)y * W, *lv = e->level + o;
        if (npo) {
            if (preempt_any(b, e->target + o, tg, W, j->pob))
                B->preempted += (uint64_t)preempt_owners_row(b, e->target + o, tg, e->level + o, e->op + o, e->dc + o, W,
                                                             j->pob, j->prn, opm, R);
        }
        else if (pl->npre) B->preempted += (uint64_t)preempt_row(b, e->target + o, tg, e->level + o, e->op + o, e->dc + o, W,
                                                                  pl->pre, (int8_t (*)[4])pl->rn, opm, R);
        if (pl->snp) B->snapped += (uint64_t)snap_row(b, e->level + o, tg, e->op + o, W, R);
        const uint8_t *fm = pl->fm == 2 ? e->stale + o : pl->fm ? ones : zeros;
        int any = pl->hact ? claim_row_heal(b, e->target + o, tg, e->level + o, e->op + o, e->dc + o, B->ca, B->cd, W, fm, split, oa, od, pl->hf0, pl->hf1)
                           : claim_row(b, e->target + o, tg, lv, B->ca, B->cd, W, fm, split, oa, od);
        int healed = hown && heal_row(b, e->target + o, e->level + o, e->op + o, e->dc + o, tg, B->ch, W, hown, R, pl->hmin, pl->hmax);
        if (healed) {
            if (B->y0[EPD_CL_H] < 0) B->y0[EPD_CL_H] = y;
            B->y1[EPD_CL_H] = y;
        }
        if (any || healed) stale_row(e->stale + o, b, W, oa, pl->va, od, pl->vd, healed ? hown : 0);
        if (pl->fm == 2) B->flash_skipped += (uint64_t)flash_skips(e->stale + o, tg, lv, b, W);
        if (!any) continue;
        if (any & 1) { if (B->y0[EPD_CL_A] < 0) B->y0[EPD_CL_A] = y; B->y1[EPD_CL_A] = y; }
        if (any & 2) { if (B->y0[EPD_CL_D] < 0) B->y0[EPD_CL_D] = y; B->y1[EPD_CL_D] = y; }
        int16_t *dc = e->dc + o;
        uint8_t *opr = e->op + o;
        if (split && !pl->mt) {
            if (any & 1) dc_row_bw(dc, b, tg, lv, W, oa, pl->neta);
            if (any & 2)
                for (int x = 0; x < W; x++)
                    if (b[x] == od) {
                        int d = dc[x] + pl->netd[tg[x] << 5 | lv[x]];
                        dc[x] = (int16_t)(d < -30000 ? -30000 : d > 30000 ? 30000 : d);
                    }
            if (opm) {
                if (any & 1) op_row_bw(opr, b, tg, lv, W, oa, pl->neta, aclamp, R);
                if (any & 2) op_row_rail(opr, b, tg, W, od, pl->rail);
            }
            continue;
        }
        gather_bw_row(b, tg, lv, opr, dc, B->fl, B->cq, B->nrow, W, oa, od, pl->neta, pl->mt, pl->model, R);
        uint16_t rb = 0;
        int nl = 0;
        for (int x = 0; x < W; x++) { B->xl[nl] = (uint16_t)x; nl += B->fl[x] >> 1; }
        uint64_t nm = 0;
        for (int k = 0; k < nl; k++) {
            int x = B->xl[k];
            if (B->fl[x] & 1) {
                int q = B->cq[x];
                uint8_t ow = j->ctab[q].own;
                B->nrow[x] = j->ctab[q].net;
                b[x] = ow ? ow : b[x];
                B->mcol[x] |= j->ctab[q].bit;
                rb |= j->ctab[q].bit;
                nm += ow != 0;
                continue;
            }
            uint8_t w = b[x];
            int i = tg[x] << 5 | lv[x], n = (w == oa ? pl->neta : pl->netd)[i];
            if (pl->mt && w == oa && n && !pl->model && dc[x] && (dc[x] < 0) != (n < 0)) {
                int d = dc[x];
                d = d < -EPD_MT_D ? -EPD_MT_D : d > EPD_MT_D ? EPD_MT_D : d;
                int c = pl->best[i][d + EPD_MT_D];
                if (pl->cap && c < pl->cs - pl->cap) c = pl->cs - pl->cap;
                if (c != pl->cs && pl->mown[c]) {
                    b[x] = pl->mown[c];
                    n = pl->mnet[c][i];
                    B->mcol[x] |= (uint16_t)(1u << c);
                    rb |= (uint16_t)(1u << c);
                    B->matched++;
                }
            }
            int dd = dc[x] + n;
            dc[x] = (int16_t)(dd < -30000 ? -30000 : dd > 30000 ? 30000 : dd);
            if (opm) opr[x] = pl->rail[tg[x]];
        }
        B->matched += nm;
        gather_bw_apply(dc, opr, B->fl, tg, B->nrow, W, opm, aclamp, R);
        for (int c = 0; rb && c < EPD_LUT_CUTS; c++)
            if (rb & (1u << c)) { if (B->mb1[c] == INT32_MAX) B->mb1[c] = y; B->mb3[c] = y; }
    }
}

/* how many bands a claim gets (1: the one-band path) */
static int claim_nbands(const epd_claim_plan *pl)
{
    int n = __atomic_load_n(&epd_claim_bands, __ATOMIC_RELAXED);
    if (n < 0) n = KNOB_DEF(epd_claim_bands);   /* (GLASSINE_CLAIM_BANDS: epd_init; 4 since the run-end cleanup's full-panel claim 53 -> 17 ms p50 on the device) */
    if (n > EPD_CLAIM_BANDS_MAX) n = EPD_CLAIM_BANDS_MAX;
    if (n <= 1 || !epd_claim_bandable(pl) || (long)pl->r.w * pl->r.h < EPD_BAND_MIN_PX || pl->r.h < n) return 1;
    return n;
}

typedef struct {
    epd_engine *e;                              /* the arrays the bands read and write */
    const epd_claim_plan *pl;
    const epd_claim_job *j;
    epd_claim_band *b;                          /* n of them */
    int n, kern;                                /* kern 1: packed, 2: packed on the state in place (W at w0, wst apart) */
    uint16_t *w0;
    size_t wst;
} band_set;
static void band_part(void *arg, int i)
{
    const band_set *c = arg;
    int h = c->pl->r.h, ya = (int)((long)h * i / c->n), yb = (int)((long)h * (i + 1) / c->n);
    if (c->kern == 2) pk_claim_rows(c->e, c->pl, c->j, &c->b[i], ya, yb, c->w0, c->wst);
    else if (c->kern) epd_claim_rows(c->e, c->pl, c->j, &c->b[i], ya, yb);
    else claim_rows_soa(c->e, c->pl, c->j, &c->b[i], ya, yb);
}
static void bands_run(const epd_engine *e, void (*fn)(void *, int), void *arg, int n)
{
    const epd_band_ops *o = e->bops;
    if (n > 1 && o && o->run) o->run(o->ctx, fn, arg, n);
    else if (n > 1 && o && o->start && o->wait) { o->start(o->ctx, fn, arg, n); o->wait(o->ctx); }
    else for (int i = 0; i < n; i++) fn(arg, i);
}
static void bands_merge(const epd_engine *e, const epd_claim_plan *pl, epd_claim_band *b, int n, int kern, epd_claim_out *out)
{
    epd_claim_band *bp[EPD_CLAIM_BANDS_MAX];
    for (int i = 0; i < n; i++) bp[i] = &b[i];
    epd_claim_merge(pl, bp, n, out);
    if (kern || !pl->hown) return;
    out->healed = 0;                            /* (claim_pixels_soa's count) */
    for (int y = out->y0[EPD_CL_H]; out->x1[EPD_CL_H] >= 0 && y <= out->y1[EPD_CL_H]; y++)
        for (int x = out->x0[EPD_CL_H]; x <= out->x1[EPD_CL_H]; x++)
            out->healed += e->busy[(size_t)(pl->r.y + y) * EPD_W + pl->r.x + x] == pl->hown;
}

static epd_claim_job cl_j;                      /* (the drive thread's claims; read-only in the bands) */
static epd_claim_band cl_b[EPD_CLAIM_BANDS_MAX];    /* (one per band: 's static b, per worker) */
static void claim_pixels_packed(epd_engine *e, epd_claim_plan *pl, epd_claim_out *out)
{
    int n = claim_nbands(pl);
    epd_claim_prep(e, pl, &cl_j);
    if (epd_tile_stat_on) tile_stat(e, pl);
    band_set c = { e, pl, &cl_j, cl_b, n, 1, NULL, 0 };
    if (n > 1) { e->banded++; bands_run(e, band_part, &c, n); }
    else epd_claim_rows(e, pl, &cl_j, &cl_b[0], 0, pl->r.h);
    bands_merge(e, pl, cl_b, n, 1, out);
}
static void claim_pixels_soa_bands(epd_engine *e, epd_claim_plan *pl, epd_claim_out *out, int n)
{
    epd_claim_prep(e, pl, &cl_j);
    if (epd_tile_stat_on) tile_stat(e, pl);
    band_set c = { e, pl, &cl_j, cl_b, n, 0, NULL, 0 };
    e->banded++;
    bands_run(e, band_part, &c, n);
    bands_merge(e, pl, cl_b, n, 0, out);
}

/* claim_kernel=check: the packed pass over the rect, then (restored) the
 * arrays' pass, whose result is kept; the six arrays over the rect and the
 * report compared (epd_claim_kernel_diffs, the first ones logged) */
static void pk_rect_io(epd_engine *e, epd_rect r, uint8_t *buf, int save)
{
    uint8_t *arr[5] = { e->busy, e->target, e->level, e->op, e->stale };
    const size_t n = (size_t)r.w * r.h;
    for (int y = 0; y < r.h; y++) {
        size_t o = (size_t)(r.y + y) * EPD_W + r.x, q = (size_t)y * r.w;
        for (int k = 0; k < 5; k++) {
            if (save) memcpy(buf + k * n + q, arr[k] + o, (size_t)r.w); else memcpy(arr[k] + o, buf + k * n + q, (size_t)r.w);
        }
        if (save) memcpy(buf + 5 * n + 2 * q, e->dc + o, (size_t)r.w * 2); else memcpy(e->dc + o, buf + 5 * n + 2 * q, (size_t)r.w * 2);
    }
}
static void claim_pixels_check(epd_engine *e, epd_claim_plan *pl, epd_claim_out *out)
{
    static uint8_t *sv, *pk;                    /* the rect before / the packed result: 7 B per pixel */
    if (!sv) {
        sv = malloc((size_t)EPD_W * EPD_H * 7);
        pk = malloc((size_t)EPD_W * EPD_H * 7);
        if (!sv || !pk) { free(sv); free(pk); sv = pk = NULL; claim_pixels_soa(e, pl, out); return; }
    }
    if (!pl->tg) pl->tg = pend_tg(e, pl->pend);
    const epd_rect r = pl->r;
    static epd_claim_plan p2;
    epd_claim_out po;
    p2 = *pl;
    pk_rect_io(e, r, sv, 1);
    claim_pixels_packed(e, &p2, &po);
    pk_rect_io(e, r, pk, 1);
    pk_rect_io(e, r, sv, 0);
    claim_pixels_soa(e, pl, out);
    pk_rect_io(e, r, sv, 1);
    if (memcmp(sv, pk, (size_t)r.w * r.h * 7) || memcmp(&po, out, sizeof po)) {
        if (epd_claim_kernel_diffs++ < 8)
            fprintf(stderr, "epd-engine: claim_kernel=check: the packed pass differs (rect %d,%d %dx%d split %d mt %d hown %d npre %d fm %d)\n",
                    r.x, r.y, r.w, r.h, pl->split, pl->mt, pl->hown, pl->npre, pl->fm);
    }
}

/* ---- : the claim on D/W/T in place ----------------
 * Not bandable (a lazy plan: slots taken in pixel order; > 4 preemptible
 * runs): the arrays' pass over the rect on a scratch engine's arrays, the
 * rect unpacked into them and packed back (the boundary, for these rare
 * claims only; D's k bits kept). W from w0, rows wst apart */
static epd_engine *pkx;                         /* (the scratch: arrays only; the claim's thread) */
static void pk_claim_whole(epd_engine *e, epd_claim_plan *pl, epd_claim_out *out, uint16_t *w0, size_t wst)
{
    if (!pkx && (!(pkx = calloc(1, sizeof *pkx)) || epd_state_alloc(pkx))) { fprintf(stderr, "epd-engine: no memory for a claim\n"); abort(); }
    if (pl->lazy) memcpy(pkx->run, e->run, sizeof e->run);     /* (slot_free: the free slots) */
    epd_pk *k = e->pk;
    const epd_rect r = pl->r;
    for (int y = 0; y < r.h; y++) {
        size_t o = (size_t)(r.y + y) * EPD_W + r.x, q = (size_t)(r.y + y) * EPD_PK_S + r.x;
        pk_unpack_row(pkx, o, r.w, k->D + q, w0 + (size_t)y * wst, k->T + q);
    }
    claim_pixels_soa(pkx, pl, out);
    for (int y = 0; y < r.h; y++) {
        size_t o = (size_t)(r.y + y) * EPD_W + r.x, q = (size_t)(r.y + y) * EPD_PK_S + r.x;
        uint32_t *D = k->D + q;
        uint16_t *Wr = w0 + (size_t)y * wst;
        for (int x = 0; x < r.w; x++) {
            uint8_t b = pkx->busy[o + x], l = pkx->level[o + x], t = pkx->target[o + x];
            D[x] = (D[x] & 0xF000u) | epd_pk_d(l, pkx->op[o + x], pkx->stale[o + x], pkx->dc[o + x]);
            Wr[x] = epd_pk_w(b, t, l);
            k->T[q + x] = (uint8_t)(t | (k->T[q + x] & 0xE0u));     /* (bits 5..7 kept) */
            pk_mark(k, b, r.y + y);
        }
    }
}

/* the claim on e's D/W/T (packed: the state, W written in place; check: the
 * mirror). claim_kernel is packed whatever it says */
static void claim_pixels_pk(epd_engine *e, epd_claim_plan *pl, epd_claim_out *out)
{
    epd_pk *k = e->pk;
    if (!pl->tg) pl->tg = pend_tg(e, pl->pend);
    uint16_t *w0 = k->W + (size_t)pl->r.y * EPD_PK_S + pl->r.x;
    epd_pk_w_rows(e, pl->r.y, pl->r.y + pl->r.h);
    if (!epd_claim_bandable(pl)) { epd_claim_kernel_soa++; pk_claim_whole(e, pl, out, w0, EPD_PK_S); return; }
    int n = claim_nbands(pl);
    epd_claim_prep(e, pl, &cl_j);
    if (epd_tile_stat_on) tile_stat(e, pl);
    band_set c = { e, pl, &cl_j, cl_b, n, 2, w0, EPD_PK_S };
    if (n > 1) { e->banded++; bands_run(e, band_part, &c, n); }
    else pk_claim_rows(e, pl, &cl_j, &cl_b[0], 0, pl->r.h, w0, EPD_PK_S);
    bands_merge(e, pl, cl_b, n, 1, out);
}

/* claim_state=check: rect r of the arrays vs D/W/T (every field decoded, W
 * from busy/target/level: idle = IDLE or FIN, T = target); the first differing pixel or -1 */
static long pk_rect_diff(const epd_engine *e, epd_rect r, const char **what)
{
    const epd_pk *k = e->pk;
    for (int y = r.y; y < r.y + r.h; y++)
        for (int x = r.x; x < r.x + r.w; x++) {
            size_t i = (size_t)y * EPD_W + x, q = (size_t)y * EPD_PK_S + x;
            uint32_t d = k->D[q];
            const char *f = epd_pk_lev(d, k->W[q], k->T[q]) != e->level[i] ? "level" : ((d >> 5) & 63) != e->op[i] ? "op" :
                            ((d >> 11) & 1) != e->stale[i] ? "stale" : (int16_t)(d >> 16) != e->dc[i] ? "dc" :
                            (k->T[q] & 31) != e->target[i] ? "target" :
                            (e->busy[i] ? k->W[q] != epd_pk_w(e->busy[i], e->target[i], e->level[i]) : !EPD_PK_ISIDLE(k->W[q])) ? "W" : NULL;
            if (f) { *what = f; return (long)i; }
        }
    return -1;
}

static void claim_pixels_arrays(epd_engine *e, epd_claim_plan *pl, epd_claim_out *out);
/* claim_state=check: the arrays' claim (the truth, as arrays would make it),
 * then the same plan on D/W/T, both compared over the rect */
static void claim_pixels_cst(epd_engine *e, epd_claim_plan *pl, epd_claim_out *out)
{
    static epd_claim_plan p2;                   /* (before the arrays' pass: a lazy plan takes its slots there) */
    if (!pl->tg) pl->tg = pend_tg(e, pl->pend);
    p2 = *pl;
    claim_pixels_arrays(e, pl, out);
    epd_claim_out po;
    int ts = epd_tile_stat_on;
    epd_tile_stat_on = 0;                       /* (counted once: the arrays' pass) */
    claim_pixels_pk(e, &p2, &po);
    epd_tile_stat_on = ts;
    const char *what = "report";
    long i = pk_rect_diff(e, pl->r, &what);
    epd_claim_state_checked++;
    if (i >= 0 || memcmp(&po, out, sizeof po)) {
        if (epd_claim_state_diffs++ < 8)
            fprintf(stderr, "epd-engine: claim_state=check: D/W/T differ (%s at %ld,%ld; rect %d,%d %dx%d split %d mt %d hown %d npre %d fm %d lazy %d)\n",
                    what, i < 0 ? -1 : i % EPD_W, i < 0 ? -1 : i / EPD_W, pl->r.x, pl->r.y, pl->r.w, pl->r.h,
                    pl->split, pl->mt, pl->hown, pl->npre, pl->fm, p2.lazy);
    }
}

#ifdef EPD_CLAIM_PROF
static void claim_pixels(epd_engine *e, epd_claim_plan *pl, epd_claim_out *out);
void epd_claim_pixels(epd_engine *e, epd_claim_plan *pl, epd_claim_out *out)
{
    double t0 = claim_prof_now();
    claim_pixels(e, pl, out);
    epd_claim_prof_s += claim_prof_now() - t0;
    epd_claim_prof_n++;
}
static void claim_pixels(epd_engine *e, epd_claim_plan *pl, epd_claim_out *out)
#else
void epd_claim_pixels(epd_engine *e, epd_claim_plan *pl, epd_claim_out *out)
#endif
{
    if (e->cst == 1) claim_pixels_pk(e, pl, out);
    else if (e->cst == 2) claim_pixels_cst(e, pl, out);
    else claim_pixels_arrays(e, pl, out);
}

/* the arrays' claim (claim_state arrays: today's dispatch) */
static void claim_pixels_arrays(epd_engine *e, epd_claim_plan *pl, epd_claim_out *out)
{
    int k = __atomic_load_n(&epd_claim_kernel, __ATOMIC_RELAXED);
    if (k < 0) k = KNOB_DEF(epd_claim_kernel);  /* (GLASSINE_CLAIM_KERNEL: epd_init) */
    if (k && !epd_claim_bandable(pl)) { epd_claim_kernel_soa++; k = 0; }
    if (k == 1) claim_pixels_packed(e, pl, out);
    else if (k == 2) claim_pixels_check(e, pl, out);
    else {
        int n = claim_nbands(pl);
        if (n > 1) claim_pixels_soa_bands(e, pl, out, n); else claim_pixels_soa(e, pl, out);
    }
}

static int job_kernel(void)
{
    int k = __atomic_load_n(&epd_claim_kernel, __ATOMIC_RELAXED);
    if (k < 0) k = KNOB_DEF(epd_claim_kernel);  /* (as epd_claim_pixels) */
    return k;
}

/* run own's rows [y0, y1) on the packed words (finish_row's rule: its
 * pixels' level := their target, busy 0). W := EPD_PK_FIN alone (the
 * level is then T's; D untouched: 2 B/px against finish_row's 3 arrays),
 * a 16-px chunk holding none of own's only read */
__attribute__((optimize("O3")))
static void pk_finish_row(uint16_t *restrict w, int n, uint8_t own)
{
    const uint16_t sl = (uint16_t)((own - 1) << 10);
    int x = 0;
    for (; x + 16 <= n; x += 16) {
#ifdef __aarch64__
        const uint16x8_t S = vdupq_n_u16(sl), H = vdupq_n_u16(0xFC00), TF = vdupq_n_u16(0x3E0), F = vdupq_n_u16(EPD_PK_FIN);
        uint16x8_t u0 = vld1q_u16(w + x), u1 = vld1q_u16(w + x + 8);
        uint16x8_t m0 = vbicq_u16(vceqq_u16(vandq_u16(u0, H), S), vceqq_u16(vandq_u16(u0, TF), TF));
        uint16x8_t m1 = vbicq_u16(vceqq_u16(vandq_u16(u1, H), S), vceqq_u16(vandq_u16(u1, TF), TF));
        if (!vmaxvq_u16(vorrq_u16(m0, m1))) continue;
        vst1q_u16(w + x, vbslq_u16(m0, F, u0));
        vst1q_u16(w + x + 8, vbslq_u16(m1, F, u1));
        continue;
#elif defined(__SSE2__)
        const __m128i S = _mm_set1_epi16((short)sl), H = _mm_set1_epi16((short)0xFC00), TF = _mm_set1_epi16(0x3E0), F = _mm_set1_epi16(EPD_PK_FIN);
        __m128i u0 = _mm_loadu_si128((const __m128i *)(w + x)), u1 = _mm_loadu_si128((const __m128i *)(w + x + 8));
        __m128i m0 = _mm_andnot_si128(_mm_cmpeq_epi16(_mm_and_si128(u0, TF), TF), _mm_cmpeq_epi16(_mm_and_si128(u0, H), S));
        __m128i m1 = _mm_andnot_si128(_mm_cmpeq_epi16(_mm_and_si128(u1, TF), TF), _mm_cmpeq_epi16(_mm_and_si128(u1, H), S));
        if (!_mm_movemask_epi8(_mm_or_si128(m0, m1))) continue;
        _mm_storeu_si128((__m128i *)(w + x), _mm_or_si128(_mm_and_si128(m0, F), _mm_andnot_si128(m0, u0)));
        _mm_storeu_si128((__m128i *)(w + x + 8), _mm_or_si128(_mm_and_si128(m1, F), _mm_andnot_si128(m1, u1)));
        continue;
#else
        uint16_t any = 0;
        for (int k = 0; k < 16; k++) { uint16_t u = w[x + k]; any |= (uint16_t)(!EPD_PK_ISIDLE(u) & ((u & 0xFC00u) == sl)); }
        if (!any) continue;
        for (int k = 0; k < 16; k++) {
            uint16_t u = w[x + k];
            w[x + k] = !EPD_PK_ISIDLE(u) && (u & 0xFC00u) == sl ? (uint16_t)EPD_PK_FIN : u;
        }
#endif
    }
    for (; x < n; x++) {
        uint16_t u = w[x];
        if (!EPD_PK_ISIDLE(u) && (u & 0xFC00u) == sl) w[x] = (uint16_t)EPD_PK_FIN;
    }
}

static int finish_mask_of(void)
{
    int k = __atomic_load_n(&epd_finish_mask, __ATOMIC_RELAXED);
    if (k < 0) k = KNOB_DEF(epd_finish_mask);   /* (GLASSINE_FINISH_MASK=0 turns it off: epd_init) */
    return k;
}

/* epd_advance's finish of run own (its whole rect): only the rows its slot
 * was given pixels in, then its row marks cleared */
static void pk_finish_run(epd_engine *e, epd_rect r, uint8_t own)
{
    epd_pk *k = e->pk;
    uint64_t *m = k->rows[own - 1];
    epd_pk_w_rows(e, r.y, r.y + r.h);
    for (int y = r.y; y < r.y + r.h; y++) {
        if (!(__atomic_load_n(&m[y >> 6], __ATOMIC_RELAXED) >> (y & 63) & 1)) continue;
        size_t q = (size_t)y * EPD_PK_S + r.x;
        pk_finish_row(k->W + q, r.w, own);
    }
    for (int i = 0; i < (EPD_H + 63) / 64; i++) __atomic_store_n(&m[i], 0, __ATOMIC_RELAXED);
}

/* run own's pixels finished over rect r (clipped to the run's) on e's
 * D/W/T (packed: the state, W in place; check: the mirror) */
static void pk_finish_rect(epd_engine *e, epd_rect r, uint8_t own)
{
    epd_pk *k = e->pk;
    epd_pk_w_rows(e, r.y, r.y + r.h);
    for (int y = r.y; y < r.y + r.h; y++) {
        size_t q = (size_t)y * EPD_PK_S + r.x;
        pk_finish_row(k->W + q, r.w, own);
    }
}

void epd_finish_rows_packed(epd_engine *e, const epd_run *r, int own, int y0, int y1)
{
    if (!e->level) {                            /* (packed: the state's words in place) */
        int ya = r->r.y > y0 ? r->r.y : y0, yb = r->r.y + r->r.h < y1 ? r->r.y + r->r.h : y1;
        if (ya < yb) pk_finish_rect(e, (epd_rect){ r->r.x, ya, r->r.w, yb - ya }, (uint8_t)own);
        return;
    }
    static epd_claim_band b;                    /* (its d/w/t rows) */
    int ya = r->r.y > y0 ? r->r.y : y0, yb = r->r.y + r->r.h < y1 ? r->r.y + r->r.h : y1;
    for (int y = ya; y < yb; y++) {
        size_t o = (size_t)y * EPD_W + r->r.x;
        pk_pack_row(e, o, r->r.w, b.d, b.w, b.t);
        pk_finish_row(b.w, r->r.w, (uint8_t)own);
        pk_unpack_row(e, o, r->r.w, b.d, b.w, b.t);
    }
}

void epd_claim_take_slots(const epd_engine *e, epd_claim_plan *pl, const int *cuts, int n)
{
    int ms[EPD_LUT_CUTS];
    for (int c = 0; c < EPD_LUT_CUTS; c++) ms[c] = pl->mown[c] ? pl->mown[c] - 1 : -1;
    for (int k = 0; k < n; k++) {
        int c = cuts[k];
        if (c < 0 || c >= EPD_LUT_CUTS || pl->mown[c] || c == pl->cs) continue;
        int s = slot_free(e, pl->sa, pl->sd, ms, EPD_LUT_CUTS, NULL, 0);
        if (s < 0) break;               /* (none free: none will be, as the lazy pass finds) */
        pl->mown[c] = (uint8_t)(s + 1);
        ms[c] = s;
    }
    pl->lazy = 0;
}

static epd_rect ext_rect(const epd_claim_plan *pl, const epd_claim_out *o, int k)
{
    return (epd_rect){ pl->r.x + o->x0[k], pl->r.y + o->y0[k], o->x1[k] - o->x0[k] + 1, o->y1[k] - o->y0[k] + 1 };
}

/* out NULL (a deferred claim, epd_set_defer): every run the plan has a slot
 * for, over the whole rect, its slots noted in e->dfr (epd_claim_settle makes
 * them exact; the counters come then too) */
static void claim_start(epd_engine *e, epd_pend *p, const epd_claim_plan *pl, const claim_runs *cr, const epd_claim_out *out)
{
    mtab *mt = cr->mt;
    const uint8_t *la = cr->la;
    int nfa = cr->nfa;
    if (out) {
        e->preempted += out->preempted; e->snapped_px += out->snapped; e->matched_px += out->matched;
        e->healed_px += out->healed; e->flash_skipped += out->flash_skipped;
    } else
        for (int k = 0; k < EPD_CL_OWNERS; k++) e->dfr.slot[k] = -1;
    /* matched: the A2 runs of this update end together (matched_lut; never
     * deferred: which cuts took pixels sets the length) */
    int mlen = 0;
    for (int c = 0; mt && out && c < EPD_LUT_CUTS; c++) {
        int nf;
        if (out->mslot[c] < 0 || out->x1[EPD_CL_M0 + c] < 0 || !lut(e, p->full, EPD_A2, c ? mt->n0 - c : 0, &nf)) continue;
        if (nf > mlen) mlen = nf;
    }
    if (e->nonblock) mlen = 0;                  /* the payback runs end on their own */
    if (mlen > nfa) la = matched_lut(e, mt, p->full, pl->cs, mlen, &nfa);
    for (int k = 0; k < 2; k++) {
        int slot = k ? cr->sd : cr->sa, o = k ? EPD_CL_D : EPD_CL_A;
        if ((out && out->x1[o] < 0) || slot < 0) continue;
        epd_rect r = out ? ext_rect(pl, out, o) : pl->r;
        if (!out) e->dfr.slot[o] = slot;
        /* a scroll step's DU part (the pixels coming from grey: A2 has no drive
         * from grey levels, DU takes ~19 frames) holds up no later step: they
         * start around it (device: every scroll from a still page,
         * greys after its cleanup, waited ~220 ms per step for about a second) */
        e->run[slot] = (epd_run){ .r = r, .mode = k ? cr->md : cr->ma, .full = p->full, .frame = 0,
                                  .nframes = k ? cr->nfd : nfa, .lut = k ? cr->ld : la, .id = p->id,
                                  .payback = k && e->nonblock && a2_step(p->mode, p->full) };
        e->nrun++;
    }
    for (int c = 0; mt && c < EPD_LUT_CUTS; c++) {   /* the matched lengths' runs */
        int ms = out ? out->mslot[c] : pl->mown[c] ? pl->mown[c] - 1 : -1;
        if (ms < 0 || (out && out->x1[EPD_CL_M0 + c] < 0)) continue;
        int nf;
        const uint8_t *l = matched_lut(e, mt, p->full, c, mlen, &nf);
        e->run[ms] = (epd_run){ .r = out ? ext_rect(pl, out, EPD_CL_M0 + c) : pl->r, .mode = EPD_A2, .full = p->full, .frame = 0,
                                .nframes = nf, .lut = l, .id = p->id, .payback = e->nonblock && c < pl->cs };
        e->nrun++;
        if (!out) e->dfr.slot[EPD_CL_M0 + c] = ms;
    }
    if (cr->hs >= 0 && (!out || out->x1[EPD_CL_H] >= 0)) {     /* the heal run */
        e->run[cr->hs] = (epd_run){ .r = out ? ext_rect(pl, out, EPD_CL_H) : pl->r, .mode = EPD_A2, .full = 0, .frame = 0,
                                    .nframes = cr->hnf, .lut = cr->hlut, .id = p->id, .payback = 1 };
        e->nrun++;
        if (!out) e->dfr.slot[EPD_CL_H] = cr->hs;
    }
}

int epd_claim_settle(epd_engine *e, int wait)
{
    if (!e->dfr.on) return 0;
    e->dfr.on = 0;                              /* (first: a collect that fails may turn the ops off, which settles) */
    if (e->dfr.px) {
        int r = e->dfr.px->collect(e->dfr.px->ctx, e, &e->dfr.out, wait);
        if (r == 1) { e->dfr.on = 1; return 1; }
        if (r) return 0;                        /* lost: the runs keep the whole rect */
        e->settle_waits += wait != 0;
    }
    const epd_claim_out *o = &e->dfr.out;
    e->preempted += o->preempted; e->snapped_px += o->snapped; e->matched_px += o->matched;
    e->healed_px += o->healed; e->flash_skipped += o->flash_skipped;
    int freed = 0;
    for (int k = 0; k < EPD_CL_OWNERS; k++) {
        int sl = e->dfr.slot[k];
        epd_run *u = sl >= 0 ? &e->run[sl] : NULL;
        if (!u || !u->nframes || u->id != e->dfr.id) continue;
        if (o->x1[k] < 0) { memset(u, 0, sizeof *u); e->nrun--; freed++; continue; }   /* took no pixel */
        epd_rect r = e->dfr.r;
        u->r = (epd_rect){ r.x + o->x0[k], r.y + o->y0[k], o->x1[k] - o->x0[k] + 1, o->y1[k] - o->y0[k] + 1 };
    }
    if (e->dfr.nend >= 0 && e->dfr.nend - freed > e->nrun_max) e->nrun_max = e->dfr.nend - freed;
    return 0;
}

void epd_set_defer(epd_engine *e, int on)
{
    if (!on) epd_claim_settle(e, 1);
    e->defer = on;
}

/* ---- (rule 1): the job (epd_set_overlap) -----
 * A claim decided at engine frame k for frame k + 2 (claim_plan's ahead).
 * Its pass reads the arrays over its rect and writes a shadow of them (the
 * same offsets in a second epd_engine, only its arrays used): frames k and
 * k + 1 are driven from the arrays as they are, and nothing writes them over
 * the rect meanwhile (epd_advance leaves it to the job; epd_schedule starts
 * nothing else; the rare writers join first: epd_job_join). The runs that
 * end before k + 2 (F) are finished in the shadow before the pass. The apply
 * (epd_schedule, on the caller) copies the rows the job changed back and
 * starts its runs: the bytes a pass at k + 2 would make. One job at a time */
struct epd_cjob {
    int on;                                     /* decided, not applied */
    int left, done;                             /* (atomic) parts still running / all ran */
    int async;                                  /* on the band ops' threads (start) */
    uint32_t id;                                /* its pending (e->pend, job set) */
    int split, kept;                            /* epd_schedule's, for the apply */
    unsigned long eff;                          /* effective at fcount >= eff */
    epd_claim_plan pl;
    claim_runs cr;
    epd_claim_job j;
    epd_claim_band b[EPD_CLAIM_BANDS_MAX];
    epd_claim_out out;                          /* (whole: the arrays' pass in one part) */
    int n, kern, whole;                         /* bands; packed; not bandable (npre > 4) */
    int pkd;                                    /* claim_state packed: D/T in place, W into pk->Wsh (no sh) */
    epd_engine *src, *sh;                       /* the engine (read), the shadow (written) */
    int nf, nq;
    epd_rect fr[EPD_MAX_UPD], qr[EPD_MAX_UPD];  /* F's runs; runs that ended later while out (job_qfin) */
    uint8_t fo[EPD_MAX_UPD], qo[EPD_MAX_UPD];   /* ... their busy values */
    uint8_t chg[EPD_H];                         /* rect rows the job changed */
};

static void finish_row(uint8_t *restrict bz, uint8_t *restrict lv, const uint8_t *restrict et, int n, uint8_t own);

static void pk_claim_whole(epd_engine *e, epd_claim_plan *pl, epd_claim_out *out, uint16_t *w0, size_t wst);
static void pk_claim_rows(epd_engine *e, const epd_claim_plan *pl, const epd_claim_job *j, epd_claim_band *B, int ya, int yb,
                          uint16_t *w0, size_t wst);
static void pk_finish_row(uint16_t *restrict w, int n, uint8_t own);

/* packed, part i: its rows of W into Wsh (the rect's, r.w apart), F
 * finished there (FIN), the pass on D/T in place and Wsh. Nothing else
 * writes the rect meanwhile, and nothing reads its D/T (the drive reads W:
 * frame k's until the apply) */
static void job_part_pk(struct epd_cjob *J, int i)
{
    epd_engine *e = J->src;
    epd_pk *k = e->pk;
    const epd_rect r = J->pl.r;
    const int ya = (int)((long)r.h * i / J->n), yb = (int)((long)r.h * (i + 1) / J->n);
    uint16_t *ws = k->Wsh;
    for (int y = ya; y < yb; y++)
        memcpy(ws + (size_t)y * r.w, k->W + (size_t)(r.y + y) * EPD_PK_S + r.x, (size_t)r.w * 2);
    for (int f = 0; f < J->nf; f++) {
        const epd_rect fr = J->fr[f];
        int y0 = fr.y > r.y + ya ? fr.y : r.y + ya, y1 = fr.y + fr.h < r.y + yb ? fr.y + fr.h : r.y + yb;
        int x0 = fr.x > r.x ? fr.x : r.x, x1 = fr.x + fr.w < r.x + r.w ? fr.x + fr.w : r.x + r.w;
        for (int y = y0; x0 < x1 && y < y1; y++) {
            pk_finish_row(ws + (size_t)(y - r.y) * r.w + (x0 - r.x), x1 - x0, J->fo[f]);     /* (D's level: FIN, T's) */
        }
    }
    if (J->whole) pk_claim_whole(e, &J->pl, &J->out, ws, (size_t)r.w);
    else pk_claim_rows(e, &J->pl, &J->j, &J->b[i], ya, yb, ws, (size_t)r.w);
}

/* part i: its rows of the rect into the shadow, F finished there, the pass */
static void job_part(void *arg, int i)
{
    struct epd_cjob *J = arg;
    const epd_engine *e = J->src;
    epd_engine *sh = J->sh;
    const epd_rect r = J->pl.r;
    const int ya = (int)((long)r.h * i / J->n), yb = (int)((long)r.h * (i + 1) / J->n);
    if (J->pkd) { job_part_pk(J, i); goto done; }
    for (int y = ya; y < yb; y++) {
        size_t o = (size_t)(r.y + y) * EPD_W + r.x;
        memcpy(sh->busy + o, e->busy + o, (size_t)r.w); memcpy(sh->target + o, e->target + o, (size_t)r.w);
        memcpy(sh->level + o, e->level + o, (size_t)r.w); memcpy(sh->op + o, e->op + o, (size_t)r.w);
        memcpy(sh->stale + o, e->stale + o, (size_t)r.w); memcpy(sh->dc + o, e->dc + o, (size_t)r.w * 2);
    }
    for (int k = 0; k < J->nf; k++) {
        const epd_rect f = J->fr[k];
        int y0 = f.y > r.y + ya ? f.y : r.y + ya, y1 = f.y + f.h < r.y + yb ? f.y + f.h : r.y + yb;
        int x0 = f.x > r.x ? f.x : r.x, x1 = f.x + f.w < r.x + r.w ? f.x + f.w : r.x + r.w;
        for (int y = y0; x0 < x1 && y < y1; y++) {
            size_t o = (size_t)y * EPD_W + x0;
            finish_row(sh->busy + o, sh->level + o, sh->target + o, x1 - x0, J->fo[k]);
        }
    }
    if (J->whole) claim_pixels_soa(sh, &J->pl, &J->out);
    else if (J->kern) epd_claim_rows(sh, &J->pl, &J->j, &J->b[i], ya, yb);
    else claim_rows_soa(sh, &J->pl, &J->j, &J->b[i], ya, yb);
    for (int y = ya; y < yb; y++) {
        size_t o = (size_t)(r.y + y) * EPD_W + r.x;
        J->chg[y] = memcmp(sh->busy + o, e->busy + o, (size_t)r.w) || memcmp(sh->target + o, e->target + o, (size_t)r.w) ||
                    memcmp(sh->level + o, e->level + o, (size_t)r.w) || memcmp(sh->op + o, e->op + o, (size_t)r.w) ||
                    memcmp(sh->stale + o, e->stale + o, (size_t)r.w) || memcmp(sh->dc + o, e->dc + o, (size_t)r.w * 2);
    }
done:
    if (__atomic_sub_fetch(&J->left, 1, __ATOMIC_ACQ_REL)) return;
    __atomic_store_n(&J->done, 1, __ATOMIC_RELEASE);
    if (J->async && e->bops->done) e->bops->done(e->bops->ctx);
}

static int job_kernel(void);
/* p's claim (planned ahead) goes out as the job */
static void job_start(epd_engine *e, epd_pend *p, const epd_claim_plan *pl, const claim_runs *cr, int ahead)
{
    struct epd_cjob *J = e->job;
    if (!J) {
        J = e->job = calloc(1, sizeof *J);
        int bad = !J;
        if (J && e->cst != 1) bad = !(J->sh = calloc(1, sizeof *J->sh)) || epd_state_alloc(J->sh);    /* (arrays) */
        if (J && e->cst == 1 && !e->pk->Wsh) bad = !(e->pk->Wsh = malloc(EPD_NPX * sizeof *e->pk->Wsh));    /* (packed) */
        if (bad) { fprintf(stderr, "epd-engine: no memory for a claim job\n"); abort(); }
    }
    J->pl = *pl; J->cr = *cr;
    if (!J->pl.tg) J->pl.tg = pend_tg(e, p);
    J->pl.pend = NULL;                          /* (the pending list moves: found by id at the apply) */
    int k = job_kernel();
    J->pkd = e->cst == 1;
    J->whole = !epd_claim_bandable(&J->pl);
    J->kern = !J->whole && (k == 1 || J->pkd);
    if (J->whole && (k || J->pkd)) epd_claim_kernel_soa++;
    int n = __atomic_load_n(&epd_claim_bands, __ATOMIC_RELAXED);
    n = n < 1 ? 1 : n > EPD_CLAIM_BANDS_MAX ? EPD_CLAIM_BANDS_MAX : n;
    J->n = J->whole || J->pl.r.h < n ? 1 : n;
    if (!J->whole) epd_claim_prep(e, &J->pl, &J->j);
    if (epd_tile_stat_on && !J->whole) tile_stat(e, &J->pl);
    J->id = p->id; p->job = 1;
    J->eff = e->fcount + (unsigned long)ahead;
    J->nf = J->nq = 0;
    for (int s = 0; s < EPD_MAX_UPD; s++) {
        const epd_run *u = &e->run[s];
        if (u->nframes && u->frame + ahead >= u->nframes && overlap(u->r, J->pl.r)) { J->fr[J->nf] = u->r; J->fo[J->nf++] = (uint8_t)(s + 1); }
    }
    J->src = e;
    J->on = 1;
    J->left = J->n; J->done = 0;
    const epd_band_ops *o = e->bops;
    J->async = o && o->start && o->wait;
    e->jobs++;
    if (J->async) o->start(o->ctx, job_part, J, J->n);
    else for (int i = 0; i < J->n; i++) job_part(J, i);
}

static int pend_blocked(epd_engine *e, epd_pend *p);
static void job_apply(epd_engine *e)
{
    struct epd_cjob *J = e->job;
    double t0 = sched_prof_now();
    if (J->async) e->bops->wait(e->bops->ctx);  /* (its parts ran: the pool's own bookkeeping) */
    J->on = 0;
    const epd_rect r = J->pl.r;
    const epd_engine *sh = J->sh;
    if (J->pkd) {                               /* (W's rows at last; the queued finishes on the state) */
        epd_pk *k = e->pk;
        epd_pk_w_rows(e, r.y, r.y + r.h);
        for (int y = 0; y < r.h; y++)
            memcpy(k->W + (size_t)(r.y + y) * EPD_PK_S + r.x, k->Wsh + (size_t)y * r.w, (size_t)r.w * 2);
        for (int q = 0; q < J->nq; q++) {
            const epd_rect f = J->qr[q];
            int y0 = f.y > r.y ? f.y : r.y, y1 = f.y + f.h < r.y + r.h ? f.y + f.h : r.y + r.h;
            int x0 = f.x > r.x ? f.x : r.x, x1 = f.x + f.w < r.x + r.w ? f.x + f.w : r.x + r.w;
            if (x0 < x1 && y0 < y1) pk_finish_rect(e, (epd_rect){ x0, y0, x1 - x0, y1 - y0 }, J->qo[q]);
        }
    }
    for (int y = 0; !J->pkd && y < r.h; y++) {
        if (!J->chg[y]) continue;
        size_t o = (size_t)(r.y + y) * EPD_W + r.x;
        memcpy(e->busy + o, sh->busy + o, (size_t)r.w); memcpy(e->target + o, sh->target + o, (size_t)r.w);
        memcpy(e->level + o, sh->level + o, (size_t)r.w); memcpy(e->op + o, sh->op + o, (size_t)r.w);
        memcpy(e->stale + o, sh->stale + o, (size_t)r.w); memcpy(e->dc + o, sh->dc + o, (size_t)r.w * 2);
    }
    for (int k = 0; !J->pkd && k < J->nq; k++) {    /* (ended while out, after its effective frame: late only) */
        const epd_rect f = J->qr[k];
        int y0 = f.y > r.y ? f.y : r.y, y1 = f.y + f.h < r.y + r.h ? f.y + f.h : r.y + r.h;
        int x0 = f.x > r.x ? f.x : r.x, x1 = f.x + f.w < r.x + r.w ? f.x + f.w : r.x + r.w;
        for (int y = y0; x0 < x1 && y < y1; y++) {
            size_t o = (size_t)y * EPD_W + x0;
            finish_row(e->busy + o, e->level + o, e->target + o, x1 - x0, J->qo[k]);
        }
    }
    epd_claim_out out;
    if (J->whole) out = J->out; else bands_merge(e, &J->pl, J->b, J->n, J->kern, &out);
    int i = 0;
    while (i < e->npend && !(e->pend[i].job && e->pend[i].id == J->id)) i++;
    if (i == e->npend) { fprintf(stderr, "epd-engine: the claim job's pending is gone\n"); abort(); }
    epd_pend *p = &e->pend[i];
    p->job = 0;
    for (int q = 0; q < e->npend; q++) if (overlap(e->pend[q].r, r)) e->pend[q].bvalid = 0;
    claim_start(e, p, &J->pl, &J->cr, &out);
    if (J->kept)                    /* (epd_schedule's, as there) */
        for (int s = 0; s < EPD_MAX_UPD; s++)
            if (e->run[s].nframes && e->run[s].id == p->id && !e->run[s].frame) e->run[s].payback = 1;
    int left = 0;
    if (J->split) { e->split_starts++; left = pend_blocked(e, p); }
    if (left) p->kept = 1;
    else {
        pend_drop(e, p);
        memmove(p, p + 1, (size_t)(e->npend - i - 1) * sizeof *p);
        e->npend--;
    }
    double dt = sched_prof_now() - t0;
    e->job_apply_s += dt;
    if (dt > e->job_apply_max) e->job_apply_max = dt;
    if (epd_sched_prof) epd_sched_t.start += dt;
}

/* epd_schedule's top: 1 = applied now, 0 = still out (it starts nothing) */
static int job_due(epd_engine *e)
{
    struct epd_cjob *J = e->job;
    if (e->fcount < J->eff && e->nrun) return 0;         /* (an idle engine: as soon as done) */
    if (!__atomic_load_n(&J->done, __ATOMIC_ACQUIRE)) {
        if (e->overlap != 2) { e->job_late++; return 0; }    /* never waits: the next call */
        if (J->async) e->bops->wait(e->bops->ctx);
    }
    if (e->fcount > J->eff) e->job_slip++;
    job_apply(e);
    return 1;
}

int epd_job_out(const epd_engine *e) { return e->job && e->job->on; }

static void job_free(epd_engine *e)
{
    if (!e->job) return;
    if (e->job->on && e->job->async) e->bops->wait(e->bops->ctx);
    if (e->job->sh) state_free(e->job->sh);
    free(e->job->sh);
    free(e->job);
    e->job = NULL;
}

int epd_job_join(epd_engine *e)
{
    if (!epd_job_out(e)) return 0;
    if (e->job->async && !__atomic_load_n(&e->job->done, __ATOMIC_ACQUIRE)) e->bops->wait(e->bops->ctx);
    e->job_joins++;
    job_apply(e);
    return 1;
}

void epd_set_overlap(epd_engine *e, int on)
{
    if (!on) epd_job_join(e);
    e->overlap = on < 0 ? 0 : on > 2 ? 2 : on;
}

void epd_set_band_ops(epd_engine *e, const epd_band_ops *ops)
{
    epd_job_join(e);
    e->bops = ops;
}

/* epd_advance: run s (just over) finished outside the job's rect; over it
 * the job did (F) or the apply will (queued) */
static void job_finish_around(epd_engine *e, int s, const epd_run *u)
{
    struct epd_cjob *J = e->job;
    const epd_rect j = J->pl.r, r = u->r;
    const uint8_t own = (uint8_t)(s + 1);
    int f = 0;
    while (f < J->nf && J->fo[f] != own) f++;
    if (f == J->nf) { J->qr[J->nq] = r; J->qo[J->nq++] = own; e->job_qfin++; }
    int xa = j.x > r.x ? j.x : r.x, xb = j.x + j.w < r.x + r.w ? j.x + j.w : r.x + r.w;
    if (!e->busy) {                             /* (packed: the state, outside the job's rect) */
        for (int y = r.y; y < r.y + r.h; y++) {
            if (y < j.y || y >= j.y + j.h || xa >= xb) { pk_finish_rect(e, (epd_rect){ r.x, y, r.w, 1 }, own); continue; }
            if (xa > r.x) pk_finish_rect(e, (epd_rect){ r.x, y, xa - r.x, 1 }, own);
            if (xb < r.x + r.w) pk_finish_rect(e, (epd_rect){ xb, y, r.x + r.w - xb, 1 }, own);
        }
        return;
    }
    for (int y = r.y; y < r.y + r.h; y++) {
        size_t o = (size_t)y * EPD_W;
        if (y < j.y || y >= j.y + j.h || xa >= xb) { finish_row(e->busy + o + r.x, e->level + o + r.x, e->target + o + r.x, r.w, own); continue; }
        if (xa > r.x) finish_row(e->busy + o + r.x, e->level + o + r.x, e->target + o + r.x, xa - r.x, own);
        if (xb < r.x + r.w) finish_row(e->busy + o + xb, e->level + o + xb, e->target + o + xb, r.x + r.w - xb, own);
    }
}

/* 1: the claim went out as the job (epd_set_overlap) */
static int claim(epd_engine *e, epd_pend *p, int sa, int ma, int sd, int md)
{
    static epd_claim_plan pl;                   /* (the drive thread only) */
    claim_runs cr;
    epd_claim_out out;
    const int pf = epd_sched_prof;
    double q0 = pf ? sched_prof_now() : 0, q1 = 0;
    int ahead = e->overlap && !e->px && !e->defer && (job_kernel() != 2 || e->cst == 1) && e->cst != 2 &&
                (long)p->r.w * p->r.h >= EPD_BAND_MIN_PX ? 2 : 0;
    if (!claim_plan(e, p, sa, ma, sd, md, &pl, &cr, ahead) ||
        (ahead && pl.lazy && !claim_plan(e, p, sa, ma, sd, md, &pl, &cr, ahead = 0))) {   /* (lazy: inline, now) */
        if (pf) epd_sched_t.plan += sched_prof_now() - q0;
        return 0;
    }
    if (pf) { q1 = sched_prof_now(); epd_sched_t.plan += q1 - q0; epd_sched_t.claims++; epd_sched_t.px += (unsigned long long)pl.r.w * pl.r.h; }
    if (ahead) { job_start(e, p, &pl, &cr, ahead); if (pf) epd_sched_t.pix += sched_prof_now() - q1; return 1; }
    /* deferred (epd_set_defer): the pass started (pixel ops) or done (the
     * CPU), its report kept for epd_claim_settle (the last one settled: the
     * slots claim_plan saw were exact) */
    int dfr = e->defer && !e->dfr.on && !pl.lazy && !(cr.mt && !e->nonblock);
    const epd_pixel_ops *px = e->px;
    if (dfr && px) dfr = px->submit && px->collect && !px->submit(px->ctx, e, &pl) && e->px == px;
    else if (dfr) epd_claim_pixels(e, &pl, &e->dfr.out);
    if (!dfr && (!e->px || !e->px->pixels || e->px->pixels(e->px->ctx, e, &pl, &out)))
        epd_claim_pixels(e, &pl, &out);
    double q2 = pf ? sched_prof_now() : 0;
    if (pf) epd_sched_t.pix += q2 - q1;
    for (int i = 0; i < e->npend; i++) if (overlap(e->pend[i].r, pl.r)) e->pend[i].bvalid = 0;
    if (!dfr) {
        claim_start(e, p, &pl, &cr, &out);
        if (pf) epd_sched_t.start += sched_prof_now() - q2;
        return 0;
    }
    claim_start(e, p, &pl, &cr, NULL);
    e->dfr.on = 1; e->dfr.px = px; e->dfr.r = pl.r; e->dfr.id = p->id; e->dfr.nend = -1;
    e->deferred++;
    if (pf) epd_sched_t.start += sched_prof_now() - q2;
    return 0;
}

/* a row of an update's rect: any pixel in flight (busy) heading elsewhere than
 * the update's target? Branch-free (vectorizes; the old early-exit loop was
 * ~5 % of glassine-epdd-desktop's CPU in a scroll, profile) */
__attribute__((optimize("O3")))
static int blocked_row(const uint8_t *restrict b, const uint8_t *restrict et, const uint8_t *restrict tg, int n)
{
    uint8_t any = 0;
    for (int x = 0; x < n; x++) any |= (uint8_t)((b[x] != 0) & (b[x] < HEAL_B) & (et[x] != tg[x]));
    return any;
}
/* (packed: the same on W, whose busy pixels hold their target) */
__attribute__((optimize("O3")))
static int blocked_row_w(const uint16_t *restrict w, const uint8_t *restrict tg, int n)
{
    uint16_t any = 0;
    for (int x = 0; x < n; x++) {
        uint16_t u = w[x];
        any |= (uint16_t)(!EPD_PK_ISIDLE(u) & ((u >> 10) < HEAL_B - 1) & (((u >> 5) & 31) != tg[x]));
    }
    return any;
}

void epd_pend_query(const epd_engine *e, const epd_pend *p, uint64_t *mask, int *nonbw)
{
    /* (a GPU target fetched once: the levels don't change, the cache does) */
    const uint8_t *t0 = pend_tg((epd_engine *)e, (epd_pend *)p);
    uint64_t m = 0;
    int nb = 0;
    for (int y = 0; !e->busy && y < p->r.h; y++) {     /* (packed) */
        size_t q = (size_t)(p->r.y + y) * EPD_PK_S + p->r.x;
        const uint16_t *w = e->pk->W + q;
        const uint32_t *d = e->pk->D + q;
        const uint8_t *tg = t0 + (size_t)y * p->r.w;
        for (int x = 0; x < p->r.w; x++) {
            int b = EPD_PK_BUSY(w[x]);
            uint8_t lv = (uint8_t)epd_pk_lev(d[x], w[x], e->pk->T[q + x]);
            if (b && b < HEAL_B && ((w[x] >> 5) & 31) != tg[x]) m |= 1ull << (b - 1);
            if (!b && (p->full || tg[x] != lv) && !bw_pix(tg[x], lv)) nb = 1;
        }
    }
    for (int y = 0; e->busy && y < p->r.h; y++) {
        size_t o = (size_t)(p->r.y + y) * EPD_W + p->r.x;
        const uint8_t *b = e->busy + o, *et = e->target + o, *lv = e->level + o, *tg = t0 + (size_t)y * p->r.w;
        for (int x = 0; x < p->r.w; x++) {
            if (b[x] && b[x] < HEAL_B && et[x] != tg[x]) m |= 1ull << (b[x] - 1);
            if (!b[x] && (p->full || tg[x] != lv[x]) && !bw_pix(tg[x], lv[x])) nb = 1;
        }
    }
    *mask = m; *nonbw = nb;
}

/* p waits for a pixel in flight elsewhere: on the CPU's arrays, or (pixel
 * ops) the mask the query found, kept up to date (epd_pend) */
static int pend_blocked_(epd_engine *e, epd_pend *p);
static int pend_blocked(epd_engine *e, epd_pend *p)
{
    if (!epd_sched_prof) return pend_blocked_(e, p);
    double t0 = sched_prof_now();
    int r = pend_blocked_(e, p);
    epd_sched_t.blocked += sched_prof_now() - t0;
    return r;
}
static int pend_blocked_(epd_engine *e, epd_pend *p)
{
    if (e->px) {
        if (!p->bvalid) {
            int nb;
            e->px->query(e->px->ctx, e, p, &p->bmask, &nb);
            p->bvalid = 1;
        }
        return p->bmask != 0;
    }
    for (int y = 0; !e->busy && y < p->r.h; y++)       /* (packed) */
        if (blocked_row_w(e->pk->W + (size_t)(p->r.y + y) * EPD_PK_S + p->r.x, pend_tg(e, p) + (size_t)y * p->r.w, p->r.w)) return 1;
    for (int y = 0; e->busy && y < p->r.h; y++) {
        size_t o = (size_t)(p->r.y + y) * EPD_W + p->r.x;
        if (blocked_row(e->busy + o, e->target + o, pend_tg(e, p) + (size_t)y * p->r.w, p->r.w)) return 1;
    }
    return 0;
}

/* may start around payback pixels (epd_set_nonblock) */
static int splittable(const epd_engine *e, const epd_pend *p)
{
    return e->nonblock && a2_step(p->mode, p->full);
}

int epd_schedule(epd_engine *e)
{
    epd_claim_settle(e, 0);                     /* (normally there: the frame's wait covered it) */
    if (epd_job_out(e) && !job_due(e)) return e->nrun;  /* (nothing starts while a job is out) */
    for (int i = 0; i < e->npend;) {
        epd_pend *p = &e->pend[i];
        int ok = 1, slot = -1;
        for (int j = 0; j < i && ok; j++)       /* never overtake an overlapping earlier one */
            if (overlap(e->pend[j].r, p->r)) { ok = 0; e->wait_order++; }
        /* a pixel still in flight only blocks if it heads somewhere else: one
         * already on its way to this update's level is left to its update (the
         * joints of a pen stroke's pieces; else the pieces run one by one) */
        int blocked = ok && pend_blocked(e, p);
        /* non-blocking payback: blocked only by payback pixels (no other run
         * over its rect) -> start around them */
        int split = 0;
        if (ok && blocked && splittable(e, p) && !p->kept) {   /* kept: its rest starts once, when free */
            /* (a claim deferred in this pass: exact before runs or free slots
             * are read; a query above already waited for the GPU) */
            epd_claim_settle(e, 1);
            split = 1;
            for (int s = 0; split && s < EPD_MAX_UPD; s++)
                if (e->run[s].nframes && !e->run[s].payback && overlap(e->run[s].r, p->r)) split = 0;
        }
        if (ok && blocked && !split) { ok = 0; e->wait_pixel++; }
        if (ok) epd_claim_settle(e, 1);
        for (int s = 0; ok && s < RUN_SLOTS; s++) if (!e->run[s].nframes) { slot = s; break; }
        if (ok && slot < 0) e->wait_slot++;
        if (!ok || slot < 0) { i++; continue; }
        int jb = 0;
        if (EPD_MODE_BASE(p->mode) == EPD_A2_OR_DU) {
            /* per pixel: A2 (10 frames) where black/white -> black/white, DU (22)
             * only where grey is involved — pen ink crossing a ruled line or an
             * older stroke's antialiased edge no longer makes the whole piece DU.
             * Two runs over the same rect, each driving the pixels it owns. */
            int slot2 = -1;
            for (int s = slot + 1; s < RUN_SLOTS; s++) if (!e->run[s].nframes) { slot2 = s; break; }
            if (slot2 >= 0) {
                jb = claim(e, p, slot, EPD_A2, slot2, EPD_DU);
            } else {            /* one slot left: one mode for all */
                int bw = 1;
                if (e->px) {                    /* (fresh: finished runs changed the levels) */
                    int nb;
                    e->px->query(e->px->ctx, e, p, &p->bmask, &nb);
                    p->bvalid = 1;
                    bw = !nb;
                }
                for (int y = 0; !e->px && bw && y < p->r.h; y++) {
                    const uint8_t *tg = pend_tg(e, p) + (size_t)y * p->r.w;
                    if (!e->busy) {             /* (packed) */
                        size_t q = (size_t)(p->r.y + y) * EPD_PK_S + p->r.x;
                        for (int x = 0; x < p->r.w; x++) {
                            uint8_t lv = (uint8_t)epd_pk_lev(e->pk->D[q + x], e->pk->W[q + x], e->pk->T[q + x]);
                            if (EPD_PK_ISIDLE(e->pk->W[q + x]) && (p->full || tg[x] != lv) && !bw_pix(tg[x], lv)) { bw = 0; break; }
                        }
                        continue;
                    }
                    const uint8_t *lv = e->level + (size_t)(p->r.y + y) * EPD_W + p->r.x;
                    for (int x = 0; x < p->r.w; x++)
                        if (!e->busy[(size_t)(p->r.y + y) * EPD_W + p->r.x + x] && (p->full || tg[x] != lv[x]) && !bw_pix(tg[x], lv[x])) { bw = 0; break; }
                }
                jb = claim(e, p, slot, bw ? EPD_A2 : EPD_DU, -1, 0);
            }
        } else
            jb = claim(e, p, slot, EPD_MODE_BASE(p->mode), -1, 0);
        if (jb) { e->job->split = split; e->job->kept = p->kept; break; }   /* (the rest at its apply) */
        if (p->kept)                    /* its stragglers: payback themselves, they hold up no step */
            for (int s = 0; s < EPD_MAX_UPD; s++)
                if (e->run[s].nframes && e->run[s].id == p->id && !e->run[s].frame) e->run[s].payback = 1;
        if (split) {                    /* its payback pixels: later, unless superseded */
            e->split_starts++;
            int left = pend_blocked(e, p);
            if (left) { p->kept = 1; i++; continue; }
        }
        pend_drop(e, p);
        memmove(p, p + 1, (size_t)(e->npend - i - 1) * sizeof *p);
        e->npend--;
    }
    if (e->dfr.on) e->dfr.nend = e->nrun;      /* (nrun_max at the settle: the empty runs not counted) */
    else if (e->nrun > e->nrun_max) e->nrun_max = e->nrun;
    if (e->npend > e->npend_max) e->npend_max = e->npend;
    return e->nrun;
}

void epd_render_begin(epd_engine *e, uint32_t *words, int dirty[2])
{
    for (int l = dirty[0]; l < dirty[1]; l++)
        for (uint32_t *w = words + (size_t)l * EPD_LINE, *end = w + EPD_LINE; w < end; w++) *w &= ~0xFFFFu;
    dirty[0] = EPD_LINES; dirty[1] = 0;
    for (int s = 0; s < EPD_MAX_UPD; s++) {
        epd_run *r = &e->run[s];
        if (!r->nframes) continue;
        int l0 = DATA_LINE0 + EPD_H - r->r.y - r->r.h, l1 = DATA_LINE0 + EPD_H - r->r.y;   /* mirrored */
        if (l0 < dirty[0]) dirty[0] = l0;
        if (l1 > dirty[1]) dirty[1] = l1;
    }
}

/* panel rows [y0, y1) of every running update; parts on disjoint row ranges
 * write disjoint words, so they may run in parallel */
/* packed: from W, what the GPU's fs reads (slot, target, level; the
 * table's index is its low 10 bits) — under a job (drive_overlap) D and T
 * over its rect are frame k + 2's already, W is frame k's until the apply */
static void render_rows_w(const epd_engine *e, uint32_t *words, int y0, int y1)
{
    for (int s = 0; s < EPD_MAX_UPD; s++) {
        const epd_run *r = &e->run[s];
        if (!r->nframes) continue;
        const uint8_t *lf = r->lut + (size_t)r->frame * 1024;
        int ya = r->r.y > y0 ? r->r.y : y0, yb = r->r.y + r->r.h < y1 ? r->r.y + r->r.h : y1;
        int x0 = r->r.x, x1 = r->r.x + r->r.w;
        const uint16_t sl = (uint16_t)s;
        for (int y = ya; y < yb; y++) {
            uint32_t *base = words + DATA0 + (size_t)(EPD_H - 1 - y) * EPD_LINE;  /* row mirror */
            const uint16_t *W = e->pk->W + (size_t)y * EPD_PK_S;
            for (int x = x0; x < x1;) {
                int xe = (x | 7) + 1 < x1 ? (x | 7) + 1 : x1;  /* (to the end of x's 8-px word) */
                uint32_t v = 0;
                for (; x < xe; x++) {
                    uint16_t u = W[x];
                    if (!EPD_PK_ISIDLE(u) && (u >> 10) == sl) v |= (uint32_t)(lf[u & 1023] & 3) << (2 * (7 - (x & 7)));
                }
                if (v) base[(x - 1) >> 3] |= v;
            }
        }
    }
}

void epd_render_rows(epd_engine *e, uint32_t *words, int y0, int y1)
{
    if (!e->busy) { render_rows_w(e, words, y0, y1); return; }
    for (int s = 0; s < EPD_MAX_UPD; s++) {
        epd_run *r = &e->run[s];
        if (!r->nframes) continue;
        const uint8_t *lf = r->lut + (size_t)r->frame * 1024;
        int ya = r->r.y > y0 ? r->r.y : y0, yb = r->r.y + r->r.h < y1 ? r->r.y + r->r.h : y1;
        int x0 = r->r.x, x1 = r->r.x + r->r.w;
        int xa = (x0 + 7) & ~7, xb = x1 & ~7;       /* whole 8-pixel words */
        if (xa > xb) xa = xb = x1;
        const uint8_t own = (uint8_t)(s + 1);
        uint64_t own8;
        memset(&own8, own, sizeof own8);
        for (int y = ya; y < yb; y++) {
            uint32_t *base = words + DATA0 + (size_t)(EPD_H - 1 - y) * EPD_LINE;  /* row mirror */
            const uint8_t *tg = e->target + (size_t)y * EPD_W, *lv = e->level + (size_t)y * EPD_W;
            const uint8_t *bz = e->busy + (size_t)y * EPD_W;   /* drive only the pixels this update owns */
            for (int x = x0; x < xa; x++)
                if (bz[x] == own) base[x >> 3] |= (uint32_t)(lf[tg[x] << 5 | lv[x]] & 3) << (2 * (7 - (x & 7)));
            for (int x = xa; x < xb; x += 8) {
                const uint8_t *t = tg + x, *v = lv + x;
                uint64_t b8;
                memcpy(&b8, bz + x, sizeof b8);
                if (b8 != own8) {
                    for (int k = 0; k < 8; k++)
                        if (bz[x + k] == own)
                            base[x >> 3] |= (uint32_t)(lf[t[k] << 5 | v[k]] & 3) << (2 * (7 - k));
                    continue;
                }
                uint32_t w = (uint32_t)(lf[t[0] << 5 | v[0]] & 3) << 14 | (uint32_t)(lf[t[1] << 5 | v[1]] & 3) << 12 |
                             (uint32_t)(lf[t[2] << 5 | v[2]] & 3) << 10 | (uint32_t)(lf[t[3] << 5 | v[3]] & 3) << 8 |
                             (uint32_t)(lf[t[4] << 5 | v[4]] & 3) << 6 | (uint32_t)(lf[t[5] << 5 | v[5]] & 3) << 4 |
                             (uint32_t)(lf[t[6] << 5 | v[6]] & 3) << 2 | (uint32_t)(lf[t[7] << 5 | v[7]] & 3);
                if (w) base[x >> 3] |= w;
            }
            for (int x = xb > xa ? xb : xa; x < x1; x++)
                if (bz[x] == own) base[x >> 3] |= (uint32_t)(lf[tg[x] << 5 | lv[x]] & 3) << (2 * (7 - (x & 7)));
        }
    }
}

void epd_render(epd_engine *e, uint32_t *words, int dirty[2])
{
    epd_render_begin(e, words, dirty);
    epd_render_rows(e, words, 0, EPD_H);
}

/* run own's pixels reach their target. A run owns few of its rect's pixels
 * (a scroll replay: 6 %, matched and heal runs span their whole step), and the
 * old full-rect select rewrote every byte of busy and level: a 16-byte chunk
 * holding none of own's is only read (device profile: finish was
 * 34 % of the drive thread; NEON on arm64, where gcc left the loops scalar) */
__attribute__((optimize("O3")))
static void finish_row(uint8_t *restrict bz, uint8_t *restrict lv, const uint8_t *restrict et, int n, uint8_t own)
{
    int x = 0;
    for (; x + 16 <= n; x += 16) {
#ifdef __aarch64__                              /* (gcc kept the plain loops scalar on arm64) */
        uint8x16_t b = vld1q_u8(bz + x), m = vceqq_u8(b, vdupq_n_u8(own));
        if (!vmaxvq_u8(m)) continue;
        vst1q_u8(lv + x, vbslq_u8(m, vld1q_u8(et + x), vld1q_u8(lv + x)));
        vst1q_u8(bz + x, vbicq_u8(b, m));
#else
        uint8_t any = 0;
        for (int k = 0; k < 16; k++) any |= bz[x + k] == own;      /* (vectorizes on x86: a compare + max) */
        if (!any) continue;
        for (int k = 0; k < 16; k++) {
            uint8_t m = (uint8_t)-(bz[x + k] == own);
            lv[x + k] = (uint8_t)((et[x + k] & m) | (lv[x + k] & ~m));
            bz[x + k] = (uint8_t)(bz[x + k] & ~m);
        }
#endif
    }
    for (; x < n; x++)
        if (bz[x] == own) { lv[x] = et[x]; bz[x] = 0; }
}

void epd_set_pixel_ops(epd_engine *e, const epd_pixel_ops *ops)
{
    if (e->dfr.on && e->dfr.px && e->dfr.px != ops) e->dfr.on = 0;   /* (not collected: the runs keep their rect) */
    e->px = ops;
}

void epd_advance(epd_engine *e, void (*done)(uint32_t id, void *arg), void *arg)
{
    for (int k = 0; e->dfr.on && k < EPD_CL_OWNERS; k++) {     /* (a deferred run ends: exact first) */
        const epd_run *u = e->dfr.slot[k] >= 0 ? &e->run[e->dfr.slot[k]] : NULL;
        if (u && u->nframes && u->id == e->dfr.id && u->frame + 1 >= u->nframes) epd_claim_settle(e, 1);
    }
    e->fcount++;
    uint64_t fin = 0;
    for (int s = 0; s < EPD_MAX_UPD; s++) {
        epd_run *r = &e->run[s];
        if (r->nframes && ++r->frame >= r->nframes) fin |= 1ull << s;
    }
    if (!fin) return;
    const int pf = epd_sched_prof;
    double f0 = pf ? sched_prof_now() : 0;
    int cpu = !e->px || !e->px->finish || e->px->finish(e->px->ctx, e, fin);
    for (int i = 0; i < e->npend; i++) e->pend[i].bmask &= ~fin;
    for (int s = 0; s < EPD_MAX_UPD; s++) {
        epd_run *r = &e->run[s];
        if (!(fin >> s & 1)) continue;
        int around = cpu && epd_job_out(e) && overlap(r->r, e->job->pl.r);   /* (the job's rect is the job's) */
        if (around) job_finish_around(e, s, r);
        for (int y = r->r.y; cpu && !around && e->busy && y < r->r.y + r->r.h; y++) {
            size_t o = (size_t)y * EPD_W + r->r.x;
            finish_row(e->busy + o, e->level + o, e->target + o, r->r.w, (uint8_t)(s + 1));
        }
        if (cpu && !around && e->pk) {                  /* (packed, and check's D/W/T) */
            if (finish_mask_of()) pk_finish_run(e, r->r, (uint8_t)(s + 1));
            else pk_finish_rect(e, r->r, (uint8_t)(s + 1));
        }
        if (pf) { epd_sched_t.fin_runs++; epd_sched_t.fin_px += (unsigned long long)r->r.w * r->r.h; }
        uint32_t id = r->id;
        memset(r, 0, sizeof *r);
        e->nrun--;
        if (done) done(id, arg);
    }
    if (pf) epd_sched_t.finish += sched_prof_now() - f0;
}

int epd_idle(const epd_engine *e)
{
    return !e->nrun && !e->npend;
}

/* a pending update's length once it starts: the cached table, else the
 * shortest waveform it can get (A2) — an estimate is enough for pacing */
int epd_pend_frames_est(const epd_engine *e, int mode, int full)
{
    int t = epd_wbf_temp_index(&e->wbf, e->celsius);
    int m = EPD_MODE_BASE(mode) == EPD_A2_OR_DU ? EPD_A2 : EPD_MODE_BASE(mode);
    int c = m == EPD_DU ? e->cut : 0;
    if (m < 0 || m >= 16 || t < 0 || t >= 64) return 10;
    if (m == EPD_A2 && (mode >> 8) > 0) return mode >> 8;
    int n = e->lutframes[full != 0][m][t][c];
    return n > 0 ? n : 10;
}

static int pend_frames(const epd_engine *e, const epd_pend *p)
{
    return epd_pend_frames_est(e, p->mode, p->full);
}

void epd_op_stats(const epd_engine *e, int limit, double *mean, long *over)
{
    *mean = 0; *over = 0;
    if (!e->op_ready) return;
    double s = 0;
    long n = 0, ov = 0;
    for (size_t i = 0; i < (size_t)EPD_W * EPD_H; i++) {
        int bi = epd_px_busy(e, i), li = epd_px_level(e, i), oi = epd_px_op(e, i);   /* (either state) */
        uint8_t l = (uint8_t)(bi ? epd_px_target(e, i) : li);
        if (l != 0 && l != 30) continue;
        int def = l == 30 ? e->op_r - oi : oi;
        if (bi >= HEAL_B) {                     /* healing: credited at its start, still to come: its own k */
            int k = (li + 1) >> 1, done = e->run[bi - 1].frame;
            def += k > done ? k - done : 0;
        } else if (bi) {                        /* the model is updated at the start: count what is still to run */
            const epd_run *r = &e->run[bi - 1];
            int left = r->nframes - r->frame - 1;
            def += left > 0 ? left : 0;
        }
        s += def; n++;
        ov += def >= limit;
    }
    *mean = n ? s / n : 0;
    *over = ov;
}

void epd_dc_stats(const epd_engine *e, int limit, int *max_abs, long *over)
{
    int m = 0;
    long n = 0;
    for (size_t i = 0; i < (size_t)EPD_W * EPD_H; i++) {
        int v = epd_px_dc(e, i), a = v < 0 ? -v : v;       /* (either state) */
        if (a > m) m = a;
        if (a > limit) n++;
    }
    *max_abs = m;
    *over = n;
}

int epd_frames_until_free(const epd_engine *e, epd_rect r)
{
    if (!clip(&r)) return 0;
    int n = 0;
    for (int s = 0; s < EPD_MAX_UPD; s++) {
        const epd_run *u = &e->run[s];
        if (u->payback) continue;               /* a cut-A2 step starts around them */
        if (u->nframes && overlap(u->r, r) && u->nframes - u->frame > n) n = u->nframes - u->frame;
    }
    /* pending ones over r start after what blocks them (at the latest: after
     * everything running over them) and then run their whole waveform */
    for (int i = 0; i < e->npend; i++) {
        const epd_pend *p = &e->pend[i];
        if (!overlap(p->r, r) || p->kept) continue;     /* kept: its rest waits for payback pixels */
        int b = 0;
        for (int s = 0; s < EPD_MAX_UPD; s++) {
            const epd_run *u = &e->run[s];
            if (u->payback) continue;
            if (u->nframes && overlap(u->r, p->r) && u->nframes - u->frame > b) b = u->nframes - u->frame;
        }
        if (b + pend_frames(e, p) > n) n = b + pend_frames(e, p);
    }
    return n;
}
