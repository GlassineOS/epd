/* epd-gtarget-test — stage 5: update targets made on the GPU
 * (epd-ingest's shadow mirror + layers, epd-gpu's T copy) against the CPU,
 * byte for byte. Needs a GPU with a render node and libgbm (77 = skipped).
 *
 * Part 1, the producer: sway-like frames through the GPU ingest into wlsrc,
 * glassine-epdd-desktop's live commit (extracted, as scan-commit-test) on the CPU shadow
 * and the mirror's commit pass on the GPU (every mode, all or not, any box),
 * CPU writes of wlsrc (stale tiles: the mirror must refuse and take an
 * upload), pen writes (dirty uploads); every target drawn into a layer must
 * equal epd_target_make of the CPU shadow, and the mirror the shadow.
 * Part 2, the engine: two engines fed the same updates from the same shadow,
 * one with CPU targets and the CPU's pixel passes (today), one with GPU
 * targets (handles) and the GPU's pixel passes (gpe) reading T from the
 * layers; runs compared every frame, the whole state every few. Then the
 * check path (the CPU's passes on fetched targets beside the GPU's) and the
 * fallback (every target fetched, the CPU goes on alone).
 * cc ... -DLIVE='"scan-commit.inc"' epd-gtarget-test.c ../epd-ingest.c ../epd-gpu.c ../epd-engine.c ../epd-policy.c
 * epd-gtarget-test <eink_waveform.wbf> [steps] 0 = pass, 77 = skipped */
#define _GNU_SOURCE             /* (the extract's dizzy workers: CPU affinity) */
#include <sched.h>
#include <pthread.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <gbm.h>
#include <EGL/egl.h>
#include "../epd-policy.h"
#include "../epd-ingest.h"
#include "../epd-gpu.h"

#define PW 1872
#define PH 1404
#define TILE 32
#define TW ((PH + TILE - 1) / TILE)
#define TH ((PW + TILE - 1) / TILE)
#define BW PH                           /* the portrait buffer */
#define BH PW
#ifndef DRM_FORMAT_MOD_LINEAR
#define DRM_FORMAT_MOD_LINEAR 0ull
#endif
/* what the extracted scan + commit need (as scan-commit-test.c) */
static box tilebox[TW * TH];
static uint8_t *shadow;
static uint32_t *ebuf;
#include "../epd-dither.h"
static int dither_pattern;              /* glassine-epdd-desktop's dither_pattern=: commit_both switches it */
__attribute__((unused)) static int bw_level(uint8_t g) { return (g >> 4) == 0 || (g >> 4) == 15; }
typedef struct { box b; int bx0, w, h; uint8_t *g; int s; } frame;
static long commit_ebuf(const frame *f, int all, int how) { (void)f; (void)all; (void)how; return 0; }   /* (ebuf is NULL) */
#include <limits.h>
#include <pthread.h>
#include "../epd-dizzy.h"
static pthread_mutex_t buf_mu = PTHREAD_MUTEX_INITIALIZER;     /* (glassine-epdd-desktop's: dizzy_reuse takes it) */
static int dz_reuse;                    /* (glassine-epdd-desktop's, declared above apply_cmd) */
static uint8_t *dz_pg, *dz_pd;
static int dizzy_still_serial;          /* (dizzy: CPU only, never mirrored; not run here) */
static int dizzy_gpu, dizzy_form_layered;   /* (glassine-epdd-desktop's GPU dizzy: off here) */
static uint8_t *wlsrc;                  /* (glassine-epdd-desktop's view of sway's frame: none here) */
static epd_ingest *ing;
static int gt_main(void) { return 0; }
static double tsec(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec / 1e9; }
/* glassine-epdd-desktop's wl_is_view: a frame that reads sway's frame (here: wlsrc, rows PW apart) */
#define wl_is_view(f) (wlsrc && (f)->s == PW && (f)->g == wlsrc + (size_t)(f)->b.x0 * PW + (f)->bx0)
static int upd_acct;                    /* (epd-upd.h's: the extract's commit_census reads it; off) */
#include LIVE
_Static_assert((int)C_ALL == EPD_INGEST_C_ALL && (int)C_BW == EPD_INGEST_C_BW && (int)C_DITHER == EPD_INGEST_C_DITHER &&
               (int)C_THRESH == EPD_INGEST_C_THRESH && (int)C_Q4 == EPD_INGEST_C_Q4, "commit modes");

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; fprintf(stderr, "FAIL: " __VA_ARGS__); fputc('\n', stderr); if (fails > 10) exit(1); } } while (0)
__attribute__((unused)) static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec / 1e9; }

static uint8_t *slurp(const char *p, size_t *n)
{
    FILE *f = fopen(p, "rb");
    if (!f) { perror(p); exit(2); }
    fseek(f, 0, SEEK_END); *n = (size_t)ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t *d = malloc(*n);
    if (fread(d, 1, *n, f) != *n) exit(2);
    fclose(f);
    return d;
}

/* ---- the producer's world: glassine-epdd-desktop's (wlsrc, shadow, the dirty list) ---- */
static uint8_t lut[256], *wlsrc;
static struct gbm_bo *bo;
static int bo_h;
static epd_ingest_rect dirty[256];
static int ndirty;
static long n_gpu_commits, n_up_commits, n_targets;

static void mark(epd_ingest_rect pr)
{
    if (ndirty == 256) { ndirty = 1; dirty[0] = (epd_ingest_rect){ 0, 0, PW, PH }; return; }
    dirty[ndirty++] = pr;
}
static void flush(void)
{
    for (int i = 0; i < ndirty; i++) epd_ingest_mirror_upload(ing, dirty[i], shadow);
    ndirty = 0;
}

/* the buffer: grey v as XRGB (luma (77 + 150 + 29) v >> 8 = v, the LUT is the identity) or random colour */
static uint32_t *px;
static void bo_put(void)
{
    uint32_t st;
    void *md = NULL;
    uint8_t *m = gbm_bo_map(bo, 0, 0, BW, BH, GBM_BO_TRANSFER_WRITE, &st, &md);
    if (!m) { fprintf(stderr, "gbm map\n"); exit(1); }
    for (int y = 0; y < BH; y++) memcpy(m + (size_t)y * st, px + (size_t)y * BW, BW * 4);
    gbm_bo_unmap(bo, md);
}

/* the CPU's convert of buffer rect r into wlsrc (shm frames, the fallback) */
static void cpu_rect(epd_ingest_rect r)
{
    for (int y = r.y; y < r.y + r.h; y++)
        for (int x = r.x; x < r.x + r.w; x++) {
            uint32_t p = px[(size_t)y * BW + x];
            wlsrc[(size_t)x * PW + (PW - 1 - y)] = lut[(((p >> 16) & 0xff) * 77 + ((p >> 8) & 0xff) * 150 + (p & 0xff) * 29) >> 8];
        }
    epd_ingest_stale(ing, r);
}

/* commit of portrait box b of wlsrc as glassine-epdd-desktop's commit_gt does: dirty
 * uploads, the CPU commit, then the GPU's pass (or an upload) */
static void commit_both(box b, int all, int how)
{
    frame f = { b, PW - b.y1, b.y1 - b.y0, b.x1 - b.x0, wlsrc + (size_t)b.x0 * PW + (PW - b.y1), PW };
    static int nc;
    dither_pattern = nc++ % 5 == 3 ? EPD_DITHER_BAYER : EPD_DITHER_BLUE;   /* (both, and the switch) */
    epd_ingest_dither(ing, dither_pattern);
    flush();
    commit(&f, all, how);
    epd_ingest_rect pr = { f.bx0, b.x0, f.w, f.h };
    if (epd_ingest_mirror_commit(ing, pr, all, how) == 0) n_gpu_commits++;
    else { mark(pr); n_up_commits++; }
}

static uint8_t *cmp_a, *cmp_b;
/* a target over pr: the layer read back vs epd_target_make; returns the handle (kept) */
static int target_check(epd_ingest_rect pr, const char *what)
{
    flush();
    int h = epd_ingest_target(ing, pr);
    CHECK(h > 0, "%s: no target", what);
    if (h <= 0) return 0;
    epd_rect r = { pr.x, pr.y, pr.w, pr.h };
    uint8_t *t = epd_target_make(&r, shadow, PW);
    epd_ingest_rect cr = { r.x, r.y, r.w, r.h };
    CHECK(epd_ingest_target_read(ing, h, cr, cmp_a, 2000) == 0, "%s: read", what);
    long bad = 0;
    for (long i = 0; i < (long)r.w * r.h; i++) bad += cmp_a[i] != t[i];
    CHECK(!bad, "%s: target %d,%d %dx%d: %ld px differ", what, r.x, r.y, r.w, r.h, bad);
    free(t);
    n_targets++;
    return h;
}

static void mirror_check(const char *what)
{
    flush();
    CHECK(epd_ingest_mirror_read(ing, cmp_b) == 0, "%s: mirror read", what);
    long bad = 0;
    size_t first = 0;
    for (size_t i = 0; i < (size_t)PW * PH; i++) if (cmp_b[i] != shadow[i]) { if (!bad) first = i; bad++; }
    CHECK(!bad, "%s: the mirror differs from shadow at %ld px (first row %zu col %zu: %d vs %d)", what, bad,
          first / PW, first % PW, cmp_b[first], shadow[first]);
}

static epd_ingest_rect rnd_brect(void)          /* a buffer (portrait) rect */
{
    epd_ingest_rect r;
    r.x = rand() % BW; r.y = rand() % BH;
    r.w = 1 + rand() % (BW - r.x); r.h = 1 + rand() % (BH - r.y);
    if (rand() % 3 == 0) { r.w = 1 + rand() % 70; r.h = 1 + rand() % 70; }
    if (r.x + r.w > BW) r.w = BW - r.x;
    if (r.y + r.h > BH) r.h = BH - r.y;
    return r;
}

static void part1(int iters)
{
    srand(3);
    for (size_t i = 0; i < (size_t)BW * BH; i++) px[i] = (uint32_t)rand() << 1 ^ (uint32_t)rand();
    bo_put();
    epd_ingest_rect whole = { 0, 0, BW, BH };
    CHECK(epd_ingest_frame(ing, bo_h, &whole, 1, wlsrc, 2000) == 0, "first frame");
    for (int it = 0; it < iters; it++) {
        int op = rand() % 100;
        char what[48];
        snprintf(what, sizeof what, "step %d", it);
        if (op < 20) {                          /* a frame through the GPU ingest */
            if (rand() % 4 == 0) {
                for (int k = 0; k < 20000; k++) px[(size_t)(rand() % BH) * BW + rand() % BW] = (uint32_t)rand();
                bo_put();
            }
            epd_ingest_rect r[4];
            int n = 1 + rand() % 4;
            for (int k = 0; k < n; k++) r[k] = rnd_brect();
            CHECK(epd_ingest_frame(ing, bo_h, r, n, wlsrc, 2000) == 0, "%s: frame", what);
        } else if (op < 28) {                   /* the CPU writes wlsrc */
            cpu_rect(rnd_brect());
        } else if (op < 60) {                   /* a commit */
            epd_ingest_rect r = rnd_brect();
            commit_both((box){ r.x, r.y, r.x + r.w, r.y + r.h }, rand() % 4 == 0, rand() % 5);
        } else if (op < 72) {                   /* pen ink */
            int x = rand() % PW, y = rand() % PH, w = 1 + rand() % 9, h = 1 + rand() % 9;
            if (x + w > PW) w = PW - x;
            if (y + h > PH) h = PH - y;
            for (int yy = y; yy < y + h; yy++) memset(shadow + (size_t)yy * PW + x, rand() % 3 ? 0 : rand() & 255, (size_t)w);
            mark((epd_ingest_rect){ x, y, w, h });
        } else {                                /* a target */
            epd_ingest_rect r = rnd_brect();
            epd_ingest_rect pr = { PW - (r.y + r.h), r.x, r.h, r.w };
            int h = target_check(pr, what);
            if (h) epd_ingest_provider(ing)->release(epd_ingest_provider(ing)->ctx, h);
        }
        if (it % 50 == 49) mirror_check(what);
    }
    mirror_check("part 1 end");
    printf("epd-gtarget: producer: %d steps, %ld commits on the GPU, %ld by upload (stale), %ld targets equal\n",
           iters, n_gpu_commits, n_up_commits, n_targets);
}

/* ---- part 2: the engines ---- */
static epd_engine EC, EG;               /* CPU targets + CPU passes (today) / GPU targets + GPU passes */
static epd_gpu *G;
static int check_mode;                  /* EG: the CPU's passes too (on fetched targets), compared */
static long claims, queries, frames, gt_subs, cpu_subs, fetches, bad;
static uint8_t gl[EPD_W * EPD_H], gtg[EPD_W * EPD_H], gb[EPD_W * EPD_H], go[EPD_W * EPD_H], gs[EPD_W * EPD_H];
static int16_t gd[EPD_W * EPD_H];

/* one thread here, two displays: glassine-epdd-desktop has the ingest in the main thread
 * and the gpe in the stream thread, each context current in its own; the
 * test makes the gpe's current before each of its calls (the ingest's own
 * calls make theirs current themselves) */
static EGLDisplay gdpy;
static EGLContext gctx;
static void gcur(void) { if (eglGetCurrentContext() != gctx) eglMakeCurrent(gdpy, EGL_NO_SURFACE, EGL_NO_SURFACE, gctx); }

static int px_claim(void *ctx, epd_engine *e, epd_claim_plan *pl, epd_claim_out *out)
{
    (void)ctx;
    epd_claim_out g;
    gcur();
    int r = epd_gpu_gpe_claim(G, e, pl, check_mode ? &g : out, 2000);
    claims++;
    if (r) { printf("epd-gtarget: the GPU claim failed (%d)\n", r); exit(1); }
    if (check_mode) {
        epd_claim_pixels(e, pl, out);
        if (memcmp(&g, out, sizeof g)) { printf("claim %ld: the GPU's report differs from the CPU's (check)\n", claims); bad++; }
    }
    return 0;
}
static int px_finish(void *ctx, epd_engine *e, uint64_t slots)
{
    (void)ctx;
    gcur();
    epd_gpu_gpe_finish(G, e, slots);
    return check_mode;
}
static void px_query(void *ctx, epd_engine *e, const epd_pend *p, uint64_t *mask, int *nonbw)
{
    (void)ctx;
    gcur();
    if (epd_gpu_gpe_query(G, p, mask, nonbw, 2000)) { printf("epd-gtarget: the GPU query failed\n"); exit(1); }
    queries++;
    if (check_mode) {
        uint64_t m;
        int nb;
        epd_pend_query(e, p, &m, &nb);
        if (m != *mask || nb != *nonbw) { printf("query %ld: the GPU's differs from the CPU's (check)\n", queries); bad++; }
    }
}
static const epd_pixel_ops OPS = { px_claim, px_finish, px_query, NULL, NULL, NULL };
static void t_release(void *ctx, int gt) { (void)ctx; epd_ingest_provider(ing)->release(epd_ingest_provider(ing)->ctx, gt); }
static int t_fetch(void *ctx, int gt, epd_rect r, uint8_t *out) { (void)ctx; fetches++; gcur(); return epd_gpu_gpe_target_read(G, gt, r, out, 2000); }
static const epd_target_ops TOPS = { t_release, t_fetch, NULL };

/* EG's state (on the GPU while its passes run there) vs EC's arrays */
static void compare(const char *when, int on_gpu)
{
    const uint8_t *lv = EG.level, *tg = EG.target, *bz = EG.busy, *op = EG.op, *st = EG.stale;
    const int16_t *dc = EG.dc;
    if (on_gpu) { gcur(); epd_gpu_gpe_read(G, gl, gtg, gb, go, gd, gs); lv = gl; tg = gtg; bz = gb; op = go; st = gs; dc = gd; }
    struct { const char *n; const uint8_t *g, *c; } a[] = {
        { "level", lv, EC.level }, { "busy", bz, EC.busy }, { "stale", st, EC.stale }, { "op", op, EC.op } };
    for (unsigned k = 0; k < sizeof a / sizeof *a; k++) {
        if (k == 3 && !EC.op_ready) continue;
        if (memcmp(a[k].g, a[k].c, sizeof gl)) { printf("%s: %s differs\n", when, a[k].n); bad++; }
    }
    if (memcmp(dc, EC.dc, sizeof gd)) { printf("%s: dc differs\n", when); bad++; }
    for (size_t i = 0; i < sizeof gl; i++)
        if (EC.busy[i] && tg[i] != EC.target[i]) { printf("%s: target differs at %zu,%zu\n", when, i % EPD_W, i / EPD_W); bad++; break; }
    if (bad > 5) exit(1);
}

static void runs_compare(void)
{
    if (EC.npend != EG.npend || EC.nrun != EG.nrun) { printf("frame %ld: pending %d / %d, running %d / %d\n", frames, EC.npend, EG.npend, EC.nrun, EG.nrun); bad++; }
    for (int s = 0; s < EPD_MAX_UPD; s++) {
        const epd_run *a = &EC.run[s], *b = &EG.run[s];
        if (a->nframes != b->nframes || (a->nframes && (a->frame != b->frame || a->id != b->id || a->mode != b->mode ||
            a->payback != b->payback || memcmp(&a->r, &b->r, sizeof a->r)))) {
            printf("frame %ld: run %d differs (id %u/%u, rect %d,%d %dx%d / %d,%d %dx%d, frames %d/%d)\n", frames, s, a->id, b->id,
                   a->r.x, a->r.y, a->r.w, a->r.h, b->r.x, b->r.y, b->r.w, b->r.h, a->nframes, b->nframes);
            bad++;
            break;
        }
    }
    if (bad > 5) exit(1);
}

static int on_gpu = 1;                  /* EG's passes on the GPU (0: the fallback, all CPU) */
static void frame_both(void)
{
    epd_schedule(&EC);
    epd_schedule(&EG);
    runs_compare();
    epd_advance(&EC, NULL, NULL);
    epd_advance(&EG, NULL, NULL);
    frames++;
    if (frames % 20 == 0) { char w[48]; snprintf(w, sizeof w, "frame %ld", frames); compare(w, on_gpu); }
}

/* one update to both engines, from the same shadow: EC a CPU copy, EG a GPU
 * target (main thread) or, gpu = 0, a CPU copy too (the pen thread) */
static void submit_both(epd_rect r, int mode, int full, int gpu)
{
    epd_submit(&EC, r, mode, full, shadow, PW);
    int h = 0;
    if (gpu && on_gpu) { flush(); h = epd_ingest_target(ing, (epd_ingest_rect){ r.x, r.y, r.w, r.h }); }
    if (h) { epd_submit_gt(&EG, r, mode, full, h, shadow, PW); gt_subs++; }
    else { epd_submit(&EG, r, mode, full, shadow, PW); cpu_subs++; }
}

#define PAGE_H 12000
static uint8_t *page;
static void make_page(void)
{
    page = malloc((size_t)BW * PAGE_H);          /* portrait rows of BW */
    memset(page, 255, (size_t)BW * PAGE_H);
    srand(5);
    for (int line = 0; line < PAGE_H / 40; line++)
        for (int x = 40; x < BW - 40; x += 14 + rand() % 6) {
            if (rand() % 7 == 0) { x += 20; continue; }
            int h = 14 + rand() % 6, y0 = line * 40 + 10;
            for (int y = y0; y < y0 + h && y < PAGE_H; y++)
                for (int k = 0; k < 9; k++) { int v = rand() % 5; page[(size_t)y * BW + x + k] = v == 0 ? 0 : v == 1 ? 96 : v == 2 ? 192 : v == 3 ? 37 : 255; }
        }
}

/* the window (portrait rows wy0..wy1 of the screen) showing the page at off, as a sway frame */
static void show(int off, int wy0, int wy1)
{
    for (int y = wy0; y < wy1; y++)
        for (int x = 0; x < BW; x++) { uint32_t v = page[(size_t)(off + y) * BW + x]; px[(size_t)y * BW + x] = 0xff000000u | v << 16 | v << 8 | v; }
    bo_put();
    epd_ingest_rect r = { 0, wy0, BW, wy1 - wy0 };
    CHECK(epd_ingest_frame(ing, bo_h, &r, 1, wlsrc, 2000) == 0, "scroll frame");
}

static void scroll(int steps, int pace)
{
    static const int cuts[] = { 8, 6, 4, 2 };
    static const int hows[] = { C_THRESH, C_DITHER, C_THRESH, C_BW, C_Q4, C_ALL };
    int wy0 = 112, wy1 = 1712, off = 0;      /* (panel columns 160 .. 1760: whole tiles) */          /* portrait rows: panel columns PW - wy1 .. PW - wy0 */
    box b = { 0, wy0, BW, wy1 };
    epd_rect win = { PW - wy1, 0, wy1 - wy0, PH };
    for (int s = 0; s < steps; s++) {
        off = (off + 36 + s % 3 * 12) % (PAGE_H - BH);
        show(off, wy0, wy1);
        int how = hows[s % 6];
        commit_both(b, 0, how);
        int mode = how == C_ALL ? EPD_GC16 : how == C_Q4 ? EPD_DU4 : EPD_MODE_A2LEN(EPD_A2_OR_DU, cuts[s % 4]);
        submit_both(win, mode, how == C_ALL && s % 12 == 5 ? 2 : 0, 1);
        if (s % 5 == 2) {                       /* pen pieces (CPU targets: the pen thread), merging */
            int x = 200 + rand() % 1400, y = 100 + rand() % 1100;
            for (int k = 0; k < 6; k++) {
                int xx = x + k * 5, yy = y + k * 3;
                for (int dy = 0; dy < 4; dy++) memset(shadow + (size_t)(yy + dy) * PW + xx, 0, 4);
                mark((epd_ingest_rect){ xx, yy, 4, 4 });
                submit_both((epd_rect){ xx, yy, 4, 4 }, EPD_A2_OR_DU, 0, 0);
            }
        }
        for (int f = 0; f < pace; f++) frame_both();
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s <wbf> [steps]\n", argv[0]); return 2; }
    int steps = argc > 2 ? atoi(argv[2]) : 120;
    (void)scan;                                     /* (extracted with commit) */
    int fd = open("/dev/dri/renderD128", O_RDWR | O_CLOEXEC);
    struct gbm_device *gd2 = fd >= 0 ? gbm_create_device(fd) : NULL;
    if (!gd2) { printf("epd-gtarget: skipped (no render node)\n"); return 77; }
    for (int i = 0; i < 256; i++) lut[i] = (uint8_t)i;
    if (!(ing = epd_ingest_open(PW, PH, lut))) { printf("epd-gtarget: skipped (no GPU ingest)\n"); return 77; }
    uint64_t mod = DRM_FORMAT_MOD_LINEAR;
    bo = gbm_bo_create_with_modifiers2(gd2, BW, BH, GBM_FORMAT_XRGB8888, &mod, 1, GBM_BO_USE_RENDERING);
    if (!bo) bo = gbm_bo_create(gd2, BW, BH, GBM_FORMAT_XRGB8888, GBM_BO_USE_RENDERING | GBM_BO_USE_LINEAR);
    if (!bo) { printf("epd-gtarget: skipped (no linear GBM buffer)\n"); return 77; }
    bo_h = epd_ingest_import(ing, gbm_bo_get_fd(bo), BW, BH, (int)gbm_bo_get_stride(bo), 0, GBM_FORMAT_XRGB8888, gbm_bo_get_modifier(bo));
    px = malloc((size_t)BW * BH * 4);
    wlsrc = malloc((size_t)PW * PH); shadow = malloc((size_t)PW * PH);
    cmp_a = malloc((size_t)PW * PH); cmp_b = malloc((size_t)PW * PH);
    memset(wlsrc, 255, (size_t)PW * PH);
    srand(1);
    for (size_t i = 0; i < (size_t)PW * PH; i++) shadow[i] = (uint8_t)(rand() % 3 == 0 ? rand() : (rand() & 1) * 255);
    if (bo_h < 0 || epd_ingest_mirror(ing, shadow, EPD_GT_MAX)) { printf("epd-gtarget: skipped (no mirror / exportable layers)\n"); return 77; }
    printf("epd-gtarget: on %s, %d layers\n", epd_ingest_renderer(ing), epd_ingest_provider(ing)->n);
    part1(steps * 4);

    /* part 2 */
    size_t wn;
    uint8_t *wbf = slurp(argv[1], &wn);
    epd_engine *es[2] = { &EC, &EG };
    for (int k = 0; k < 2; k++) {
        if (epd_init(es[k], wbf, wn, 24)) { fprintf(stderr, "wbf\n"); return 1; }
        epd_assume(es[k], 30);
        epd_set_matched(es[k], 1); epd_set_matched_rule(es[k], 2, 3); epd_set_nonblock(es[k], 2);
        epd_set_model(es[k], 2, 0); epd_set_heal(es[k], 2, 2); epd_set_snap(es[k], 1);
    }
    memset(shadow, 255, (size_t)PW * PH);          /* the panel white, as assumed */
    flush();
    epd_ingest_mirror_upload(ing, (epd_ingest_rect){ 0, 0, PW, PH }, shadow);
    make_page();
    if (!(G = epd_gpu_open(&EG)) || epd_gpu_gpe_open(G, &EG)) { printf("epd-gtarget: skipped (no gpe)\n"); return 77; }
    gdpy = eglGetCurrentDisplay(); gctx = eglGetCurrentContext();
    if (epd_gpu_gpe_provider(G, epd_ingest_provider(ing))) { printf("epd-gtarget: FAIL: the layers did not import\n"); return 1; }
    epd_set_pixel_ops(&EG, &OPS);
    epd_set_target_ops(&EG, &TOPS);
    epd_gpu_gpe_drive(G, 1);
    scroll(steps / 2, 30);                        /* slow: each step claims on its own GPU target */
    printf("epd-gtarget: slow scroll: %ld claims, %ld queries, %ld frames\n", claims, queries, frames);
    scroll(steps, 2);
    printf("epd-gtarget: paced scroll: %ld claims, %ld queries, %ld frames\n", claims, queries, frames);
    scroll(steps / 2, 0);                         /* flood: steps pile up, supersede, absorb */
    for (int f = 0; f < 3; f++) frame_both();
    printf("epd-gtarget: flood: %ld claims, %ld frames\n", claims, frames);
    gcur();                                       /* the CPU arrays catch up: its passes run beside the GPU from here */
    epd_gpu_gpe_read(G, EG.level, EG.target, EG.busy, EG.op, EG.dc, EG.stale);
    check_mode = 1;
    scroll(steps / 2, 1);
    printf("epd-gtarget: check (CPU passes on fetched targets): %ld claims, %ld fetches\n", claims, fetches);
    check_mode = 0;
    scroll(steps / 4, 0);                         /* targets waiting, then the GPU goes: the fallback */
    epd_pend_targets_cpu(&EG);
    gcur();
    epd_gpu_gpe_read(G, EG.level, EG.target, EG.busy, EG.op, EG.dc, EG.stale);
    epd_set_pixel_ops(&EG, NULL);
    on_gpu = 0;
    int left = 0;
    for (int i = 0; i < EG.npend; i++) left += EG.pend[i].gt != 0;
    CHECK(!left, "%d targets still on the GPU after epd_pend_targets_cpu", left);
    scroll(steps / 4, 1);
    while (!epd_idle(&EC) || !epd_idle(&EG)) frame_both();
    compare("idle", 0);
    int used = 0;                                 /* every layer released */
    for (int k = 1; k <= epd_ingest_provider(ing)->n; k++) {
        int h = epd_ingest_target(ing, (epd_ingest_rect){ 0, 0, 4, 1 });
        used += h > 0;
    }
    CHECK(used == epd_ingest_provider(ing)->n, "only %d of %d layers free at the end", used, epd_ingest_provider(ing)->n);
    printf("epd-gtarget: engines: %ld GPU-target updates, %ld CPU-target ones, %ld frames, %ld claims, %ld queries, %ld fetches, %ld differences\n",
           gt_subs, cpu_subs, frames, claims, queries, fetches, bad);
    if (bad) fails++;
    printf(fails ? "epd-gtarget: FAILED\n" : "epd-gtarget: ok\n");
    return fails ? 1 : 0;
}
