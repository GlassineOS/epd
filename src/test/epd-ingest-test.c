/* epd-ingest-test — the GPU ingest (epd-ingest.c) against the CPU
 * convert (glassine-epdd-desktop grey_of + wl_rect rotation), byte for byte. Needs a GPU
 * with a render node (host: any Mesa driver; skipped without one): a linear
 * XRGB8888 GBM buffer of the panel size is filled with random pixels (plus
 * the luma edge cases), random damage rects go through epd_ingest_frame, and
 * every pixel the CPU writes must match. Also: a gamma change, a second buffer,
 * ARGB, the column widening (only whole texels outside the rect), the
 * luminance q (epd-grey.h, epd_ingest_mode) against epd_grey_px_mode, timing;
 * GPU dizzy (epd_ingest_dizzy) against epd_dizzy over the same rect, byte
 * for byte, and its time.
 * epd-ingest-test [render node] exit 0 = pass, 77 = skipped */
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <gbm.h>
#include "../epd-ingest.h"
#include "../epd-grey.h"
#include "../epd-dizzy.h"

#define PW 1872
#define PH 1404
#define W PH                            /* the portrait buffer */
#define H PW
#ifndef DRM_FORMAT_MOD_LINEAR
#define DRM_FORMAT_MOD_LINEAR 0ull
#endif
static uint8_t lut[256];
static int lum;                         /* the q: epd_ingest_mode */
static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; fprintf(stderr, "FAIL: " __VA_ARGS__); fputc('\n', stderr); } } while (0)

static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec / 1e9; }
static uint8_t grey_of(uint32_t p) { return epd_grey_px_mode(lut, lum, p); }
static void gamma_lut(double g) { for (int i = 0; i < 256; i++) lut[i] = (uint8_t)lround(255.0 * pow(i / 255.0, g)); }

struct buf { struct gbm_bo *bo; int fd, stride; uint32_t *px; };

static int alloc_buf(struct gbm_device *gd, struct buf *b, uint32_t fourcc)
{
    uint64_t mod = DRM_FORMAT_MOD_LINEAR;
    b->bo = gbm_bo_create_with_modifiers2(gd, W, H, fourcc, &mod, 1, GBM_BO_USE_RENDERING);
    if (!b->bo) b->bo = gbm_bo_create(gd, W, H, fourcc, GBM_BO_USE_RENDERING | GBM_BO_USE_LINEAR);
    if (!b->bo) return -1;
    b->fd = gbm_bo_get_fd(b->bo);
    b->stride = (int)gbm_bo_get_stride(b->bo);
    b->px = malloc((size_t)W * H * 4);
    return b->fd < 0 || !b->px ? -1 : 0;
}

/* the CPU copy of the pixels into the bo (through gbm's map: write once) */
static int fill(struct buf *b, unsigned seed)
{
    srand(seed);
    for (size_t i = 0; i < (size_t)W * H; i++) b->px[i] = (uint32_t)rand() << 1 ^ (uint32_t)rand();
    /* edge cases: black, white, each channel max, luma rounding boundaries */
    static const uint32_t ed[] = { 0, 0xffffff, 0xff0000, 0x00ff00, 0x0000ff, 0x010101, 0xfefefe, 0x808080, 0x7f7f7f, 0xff00ff };
    for (unsigned i = 0; i < sizeof ed / sizeof *ed; i++) b->px[i * 997] = ed[i] | 0xab000000u;
    uint32_t st;
    void *md = NULL;
    uint8_t *m = gbm_bo_map(b->bo, 0, 0, W, H, GBM_BO_TRANSFER_WRITE, &st, &md);
    if (!m) return -1;
    for (int y = 0; y < H; y++) memcpy(m + (size_t)y * st, b->px + (size_t)y * W, W * 4);
    gbm_bo_unmap(b->bo, md);
    return 0;
}

static void cpu_rect(uint8_t *dst, const struct buf *b, epd_ingest_rect r)
{
    for (int py = r.y; py < r.y + r.h; py++)
        for (int x = r.x; x < r.x + r.w; x++) dst[(size_t)x * PW + (PW - 1 - py)] = grey_of(b->px[(size_t)py * W + x]);
}

/* the CPU ingest (epd_ingest_frame_cpu, the buffer's mapping)
 * of the same rects into an image prefilled as the GPU's was: every byte of
 * the panel image equal to the GPU path's gg (the widened columns too, and
 * nothing written elsewhere) */
static long cpu_same(epd_ingest *g, int h, const epd_ingest_rect *r, int n, const uint8_t *gg, const char *what)
{
    static uint8_t *cg;
    if (!cg) cg = malloc((size_t)PW * PH);
    memset(cg, 0x5a, (size_t)PW * PH);
    epd_ingest_times t;
    CHECK(epd_ingest_frame_cpu(g, h, r, n, cg, &t) == 0, "%s: epd_ingest_frame_cpu failed", what);
    CHECK(t.gpu == 0, "%s: the CPU ingest timed a GPU part", what);
    long bad = 0;
    for (size_t i = 0; i < (size_t)PW * PH; i++)
        if (cg[i] != gg[i]) { if (!bad) fprintf(stderr, "  %s: first CPU-ingest diff at row %zu col %zu: gpu %d cpu %d\n", what, i / PW, i % PW, gg[i], cg[i]); bad++; }
    CHECK(!bad, "%s: the CPU ingest differs from the GPU's in %ld px", what, bad);
    return bad;
}

/* the frame's rects through the GPU and the CPU; differences counted where the
 * CPU wrote, and outside the widened columns nothing may be touched */
static long run(epd_ingest *g, int h, const struct buf *b, const epd_ingest_rect *r, int n, const char *what)
{
    static uint8_t *gg, *cc;
    if (!gg) { gg = malloc((size_t)PW * PH); cc = malloc((size_t)PW * PH); }
    memset(gg, 0x5a, (size_t)PW * PH);
    memset(cc, 0x5a, (size_t)PW * PH);
    int rc = epd_ingest_frame(g, h, r, n, gg, 2000);
    CHECK(rc == 0, "%s: epd_ingest_frame failed", what);
    for (int k = 0; k < n; k++) cpu_rect(cc, b, r[k]);
    long bad = 0, stray = 0;
    static uint8_t *mask;
    if (!mask) mask = malloc((size_t)PW * PH);
    memset(mask, 0, (size_t)PW * PH);           /* 1 = the CPU wrote, 2 = widened */
    for (int k = 0; k < n; k++) {
        int c0, c1;
        epd_ingest_cols(g, r[k], &c0, &c1);
        for (int row = r[k].x; row < r[k].x + r[k].w; row++)
            for (int col = c0; col < c1; col++) if (!mask[(size_t)row * PW + col]) mask[(size_t)row * PW + col] = 2;
        for (int row = r[k].x; row < r[k].x + r[k].w; row++)
            for (int col = PW - (r[k].y + r[k].h); col < PW - r[k].y; col++) mask[(size_t)row * PW + col] = 1;
    }
    for (size_t i = 0; i < (size_t)PW * PH; i++) {
        if (mask[i] == 1 && gg[i] != cc[i]) { if (!bad) fprintf(stderr, "  %s: first diff at row %zu col %zu: gpu %d cpu %d\n", what, i / PW, i % PW, gg[i], cc[i]); bad++; }
        if (!mask[i] && gg[i] != 0x5a) stray++;
    }
    CHECK(!bad, "%s: %ld px differ", what, bad);
    CHECK(!stray, "%s: %ld px written outside the widened rects", what, stray);
    bad += cpu_same(g, h, r, n, gg, what);
    return bad;
}

/* sway's grey output : an R8 LINEAR buffer in panel order
 * (PW x PH), q bytes; the ingest's prog8 (the LUT only) against lut8[q] where
 * the rects are, then the CPU ingest against the GPU's bytes ('s gate) */
static uint8_t lut8[256];
struct buf8 { struct gbm_bo *bo; int fd, stride; uint8_t *px; };
static int alloc8(struct gbm_device *gd, struct buf8 *b, unsigned seed)
{
    uint64_t mod = DRM_FORMAT_MOD_LINEAR;
    b->bo = gbm_bo_create_with_modifiers2(gd, PW, PH, GBM_FORMAT_R8, &mod, 1, GBM_BO_USE_RENDERING);
    if (!b->bo) b->bo = gbm_bo_create(gd, PW, PH, GBM_FORMAT_R8, GBM_BO_USE_RENDERING | GBM_BO_USE_LINEAR);
    if (!b->bo || (b->fd = gbm_bo_get_fd(b->bo)) < 0 || !(b->px = malloc((size_t)PW * PH))) return -1;
    b->stride = (int)gbm_bo_get_stride(b->bo);
    srand(seed);
    for (size_t i = 0; i < (size_t)PW * PH; i++) b->px[i] = (uint8_t)rand();
    for (int i = 0; i < 256; i++) b->px[(size_t)i * 4099 % ((size_t)PW * PH)] = (uint8_t)i;      /* every q at least once */
    uint32_t st;
    void *md = NULL;
    uint8_t *m = gbm_bo_map(b->bo, 0, 0, PW, PH, GBM_BO_TRANSFER_WRITE, &st, &md);
    if (!m) return -1;
    for (int y = 0; y < PH; y++) memcpy(m + (size_t)y * st, b->px + (size_t)y * PW, PW);
    gbm_bo_unmap(b->bo, md);
    return 0;
}

static long run8(epd_ingest *g, int h, const struct buf8 *b, const epd_ingest_rect *r, int n, const char *what)
{
    static uint8_t *gg;
    if (!gg) gg = malloc((size_t)PW * PH);
    memset(gg, 0x5a, (size_t)PW * PH);
    CHECK(epd_ingest_frame(g, h, r, n, gg, 2000) == 0, "%s: epd_ingest_frame failed", what);
    long bad = 0;
    for (int k = 0; k < n; k++)                 /* the rect's own pixels: panel cols PW - (y + h) .. PW - y, rows x .. x + w */
        for (int row = r[k].x; row < r[k].x + r[k].w; row++)
            for (int col = PW - (r[k].y + r[k].h); col < PW - r[k].y; col++) {
                uint8_t want = lut8[b->px[(size_t)row * PW + col]];
                if (gg[(size_t)row * PW + col] != want) {
                    if (!bad) fprintf(stderr, "  %s: first R8 diff at row %d col %d: gpu %d want %d\n", what, row, col, gg[(size_t)row * PW + col], want);
                    bad++;
                }
            }
    CHECK(!bad, "%s: %ld R8 px differ from lut8[q]", what, bad);
    return bad + cpu_same(g, h, r, n, gg, what);
}

/* the CPU ingest's time per frame (the mapping; on the host coherent memory) */
static double cpu_ms(epd_ingest *g, int h, epd_ingest_rect r, uint8_t *gr)
{
    double t0 = now();
    for (int i = 0; i < 20; i++) epd_ingest_frame_cpu(g, h, &r, 1, gr, NULL);
    return (now() - t0) * 1e3 / 20;
}

/* GPU dizzy vs its CPU reference (the rect as a whole) on greys gr (PW x
 * PH): the dots differ where? (black and white fixed, as glassine-epdd-desktop's dz_lv).
 * form: the layered shader vs epd_dizzy, the block one vs epd_dizzy_block
 * (the default tables); view: the greys are a view of the ingest's output
 * (seeded there: the block form reads the texture, no upload); lvx: a level
 * table that is not the canonical one (the shaders' LUT path) */
static void dizzy_check(epd_ingest *g, const uint8_t *gr, int x0, int y0, int w, int h, int form, int view, int lvx, const char *what)
{
    uint8_t lv[256];
    for (int v = 0; v < 256; v++) lv[v] = (uint8_t)((v * 30 + 127) / 255 | ((v >> 4) == 0 || (v >> 4) == 15 ? EPD_DZ_FIXED : 0));
    if (lvx) for (int v = 0; v < 256; v++) lv[v] = (uint8_t)(v < 8 ? EPD_DZ_FIXED : (v * 30 + 127) / 255);   /* (white free, a wider black) */
    uint8_t *t = malloc((size_t)w * h), *o = malloc((size_t)w * h);
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) t[(size_t)y * w + x] = lv[gr[(size_t)(y0 + y) * PW + x0 + x]];
    if (form == EPD_GDZ_LAYERED) epd_dizzy(t, w, h, w, x0, y0);
    else epd_dizzy_block(t, w, h, w, x0, y0, NULL);
    memset(o, 0x5a, (size_t)w * h);
    epd_ingest_dizzy_form(g, form);
    if (view == 1) CHECK(epd_ingest_dizzy_seed(g, gr, PW, 0, 0, PW, PH) == 0, "dizzy %s: seed", what);   /* (2: the output as the frames left it) */
    double t0 = now();
    int rc = epd_ingest_dizzy_v(g, gr + (size_t)y0 * PW + x0, PW, x0, y0, w, h, lv, o, w, 2000, view != 0);
    double ms = (now() - t0) * 1e3;
    if (rc < 0) { printf("epd-ingest: dizzy %s: not on this GPU\n", what); free(t); free(o); return; }
    if (form == EPD_GDZ_BLOCK) CHECK(epd_ingest_dizzy_src(g) == (view != 0), "dizzy %s: the greys came from %s", what, view ? "an upload" : "the output");
    long bad = 0;
    for (size_t i = 0; i < (size_t)w * h; i++) {
        uint8_t want = t[i] & EPD_DZ_FIXED ? 0 : t[i];
        if (o[i] != want) { if (!bad) fprintf(stderr, "  dizzy %s: first diff at %zu,%zu: gpu %d cpu %d\n", what, i % w, i / w, o[i], want); bad++; }
    }
    CHECK(!bad, "dizzy %s: %ld px differ", what, bad);
    printf("epd-ingest: dizzy %s %s%s %dx%d: %.2f ms%s\n", form == EPD_GDZ_LAYERED ? "layered" : "block", view ? "view " : "",
           what, w, h, ms, bad ? "" : form == EPD_GDZ_LAYERED ? ", = epd_dizzy" : ", = epd_dizzy_block");
    free(t); free(o);
}

int main(int argc, char **argv)
{
    const char *node = argc > 1 ? argv[1] : "/dev/dri/renderD128";
    int fd = open(node, O_RDWR | O_CLOEXEC);
    struct gbm_device *gd = fd >= 0 ? gbm_create_device(fd) : NULL;
    if (!gd) { printf("epd-ingest: skipped (no %s)\n", node); return 77; }
    gamma_lut(1.8);
    epd_ingest *g = epd_ingest_open(PW, PH, lut);
    if (!g) { printf("epd-ingest: skipped (no GPU ingest)\n"); return 77; }
    printf("epd-ingest: on %s\n", epd_ingest_renderer(g));
    struct buf a, b2;
    if (alloc_buf(gd, &a, GBM_FORMAT_XRGB8888) || fill(&a, 1)) { printf("epd-ingest: skipped (no linear GBM buffer)\n"); return 77; }
    int ha = epd_ingest_import(g, a.fd, W, H, a.stride, 0, GBM_FORMAT_XRGB8888, gbm_bo_get_modifier(a.bo));
    CHECK(ha >= 0, "import XRGB");
    epd_ingest_rect whole = { 0, 0, W, H };
    run(g, ha, &a, &whole, 1, "whole frame");
    srand(7);
    for (int it = 0; it < 20; it++) {             /* random rects, odd sizes, overlapping, clipped */
        epd_ingest_rect r[8];
        int n = 1 + rand() % 8;
        for (int k = 0; k < n; k++) {
            r[k].x = rand() % W; r[k].y = rand() % H;
            r[k].w = 1 + rand() % (W - r[k].x); r[k].h = 1 + rand() % (H - r[k].y);
            if (rand() % 4 == 0) { r[k].w = 1 + rand() % 7; r[k].h = 1 + rand() % 7; }
        }
        char what[32];
        snprintf(what, sizeof what, "random %d", it);
        run(g, ha, &a, r, n, what);
    }
    gamma_lut(1.0);                                 /* a gamma change reaches the shader */
    epd_ingest_lut(g, lut);
    run(g, ha, &a, &whole, 1, "gamma 1.0");
    if (alloc_buf(gd, &b2, GBM_FORMAT_ARGB8888) == 0 && fill(&b2, 2) == 0) {
        int hb = epd_ingest_import(g, b2.fd, W, H, b2.stride, 0, GBM_FORMAT_ARGB8888, gbm_bo_get_modifier(b2.bo));
        CHECK(hb >= 0 && hb != ha, "import ARGB (second buffer)");
        run(g, hb, &b2, &whole, 1, "ARGB whole");
        run(g, ha, &a, &whole, 1, "back to XRGB");
        epd_ingest_drop(g, hb);
    }
    CHECK(epd_ingest_import(g, a.fd, W + 4, H, a.stride, 0, GBM_FORMAT_XRGB8888, 0) < 0, "a wrong size must be refused");
    /* the luminance q, its LUT; then back to luma */
    lum = 1;
    epd_grey_lum_lut(lut, 1.6);
    epd_ingest_lut(g, lut);
    epd_ingest_mode(g, 1);
    run(g, ha, &a, &whole, 1, "luminance whole");
    srand(9);
    for (int it = 0; it < 6; it++) {
        epd_ingest_rect r = { rand() % W, rand() % H, 0, 0 };
        r.w = 1 + rand() % (W - r.x); r.h = 1 + rand() % (H - r.y);
        char what[32];
        snprintf(what, sizeof what, "luminance random %d", it);
        run(g, ha, &a, &r, 1, what);
    }
    lum = 0;
    gamma_lut(1.0);
    epd_ingest_lut(g, lut);
    epd_ingest_mode(g, 0);
    run(g, ha, &a, &whole, 1, "luma again");
    /* timing: full frames, then a scroll-sized band */
    uint8_t *gr = malloc((size_t)PW * PH);
    double t0 = now();
    for (int i = 0; i < 20; i++) epd_ingest_frame(g, ha, &whole, 1, gr, 2000);
    double full = (now() - t0) * 1e3 / 20;
    epd_ingest_rect band = { 0, 300, W, 1400 };
    t0 = now();
    for (int i = 0; i < 20; i++) epd_ingest_frame(g, ha, &band, 1, gr, 2000);
    double bd = (now() - t0) * 1e3 / 20;
    printf("epd-ingest: full frame %.2f ms (%.2f ns/px), scroll band %.2f ms\n", full, full * 1e6 / ((double)W * H), bd);
    printf("epd-ingest: S3 CPU ingest (mapping) XRGB full frame %.2f ms, scroll band %.2f ms\n",
           cpu_ms(g, ha, whole, gr), cpu_ms(g, ha, band, gr));
    /* R8 frames (sway's grey output), the GPU's prog8 and the CPU ingest */
    {
        struct buf8 r8b;
        if (alloc8(gd, &r8b, 11) == 0) {
            int h8 = epd_ingest_import(g, r8b.fd, PW, PH, r8b.stride, 0, GBM_FORMAT_R8, gbm_bo_get_modifier(r8b.bo));
            CHECK(h8 >= 0, "import R8");
            epd_grey_lum_lut(lut8, 1.8);                    /* a non-identity table */
            epd_ingest_lut8(g, lut8);
            if (h8 >= 0) {
                run8(g, h8, &r8b, &whole, 1, "R8 whole frame");
                srand(13);
                for (int it = 0; it < 20; it++) {
                    epd_ingest_rect r[8];
                    int n = 1 + rand() % 8;
                    for (int k = 0; k < n; k++) {
                        r[k].x = rand() % W; r[k].y = rand() % H;
                        r[k].w = 1 + rand() % (W - r[k].x); r[k].h = 1 + rand() % (H - r[k].y);
                        if (rand() % 4 == 0) { r[k].w = 1 + rand() % 7; r[k].h = 1 + rand() % 7; }
                    }
                    char what[32];
                    snprintf(what, sizeof what, "R8 random %d", it);
                    run8(g, h8, &r8b, r, n, what);
                }
                epd_grey_luma_lut(lut8, 1.0);               /* a LUT change reaches both */
                epd_ingest_lut8(g, lut8);
                run8(g, h8, &r8b, &band, 1, "R8 band, luma LUT");
                printf("epd-ingest: S3 CPU ingest (mapping) R8 full frame %.2f ms, scroll band %.2f ms\n",
                       cpu_ms(g, h8, whole, gr), cpu_ms(g, h8, band, gr));
                t0 = now();
                for (int i = 0; i < 20; i++) epd_ingest_frame(g, h8, &whole, 1, gr, 2000);
                printf("epd-ingest: R8 GPU ingest full frame %.2f ms\n", (now() - t0) * 1e3 / 20);
                epd_ingest_drop(g, h8);
            }
        } else printf("epd-ingest: R8: skipped (no R8 LINEAR GBM buffer)\n");
        epd_ingest_frame(g, ha, &whole, 1, gr, 2000);      /* (the dizzy view below: gr = out, all drawn) */
    }
    /* the block dizzy of a real frame's greys read from the ingest's own output (glassine-epdd-desktop's wlsrc case: no upload, the drawn map) */
    dizzy_check(g, gr, 0, 0, PW, PH, EPD_GDZ_BLOCK, 2, 0, "real frame");
    dizzy_check(g, gr, 37, 5, 901, 603, EPD_GDZ_BLOCK, 2, 0, "real frame, odd rect");
    /* dizzy: a photo-like ramp + noise, a UI-like page (white, black text, few greys), odd rects */
    uint8_t *dg = malloc((size_t)PW * PH);
    srand(5);
    for (int y = 0; y < PH; y++)
        for (int x = 0; x < PW; x++) {
            int v = (x * 255 / PW + y * 97 / PH + rand() % 40) % 256;
            dg[(size_t)y * PW + x] = (uint8_t)(y < PH / 2 ? v : (rand() % 9 ? 255 : rand() % 3 ? 0 : 128 + rand() % 64));
        }
    /* the block form (the GPU path's default): uploaded, then as a view of the output (no upload), odd rects, the LUT levels */
    dizzy_check(g, dg, 0, 0, PW, PH, EPD_GDZ_BLOCK, 0, 0, "full panel");
    dizzy_check(g, dg, 0, 0, PW, PH, EPD_GDZ_BLOCK, 0, 0, "full panel again");
    dizzy_check(g, dg, 0, 0, PW, PH, EPD_GDZ_BLOCK, 1, 0, "full panel");
    dizzy_check(g, dg, 37, 5, 901, 603, EPD_GDZ_BLOCK, 1, 0, "odd rect");
    dizzy_check(g, dg, 37, 5, 901, 603, EPD_GDZ_BLOCK, 0, 0, "odd rect");
    dizzy_check(g, dg, 64, 128, 64, 64, EPD_GDZ_BLOCK, 0, 0, "one tile");
    dizzy_check(g, dg, 1, 700, 3, 2, EPD_GDZ_BLOCK, 1, 0, "tiny");
    dizzy_check(g, dg, 1, 700, 3, 2, EPD_GDZ_BLOCK, 0, 0, "tiny");
    dizzy_check(g, dg, PW - 77, PH - 50, 77, 50, EPD_GDZ_BLOCK, 1, 0, "corner");
    dizzy_check(g, dg, PW - 77, PH - 50, 77, 50, EPD_GDZ_BLOCK, 0, 0, "corner");
    dizzy_check(g, dg, 128, 200, 1400, 128, EPD_GDZ_BLOCK, 1, 0, "scroll band");
    /* a rect the caller wrote on the CPU since: the view falls back to the upload */
    epd_ingest_stale(g, (epd_ingest_rect){ 300, 100, 40, 40 });
    {
        uint8_t lv[256];
        (void)lv;
        epd_ingest_dizzy_form(g, EPD_GDZ_BLOCK);
        uint8_t *o = malloc((size_t)PW * PH), lvt[256];
        for (int v = 0; v < 256; v++) lvt[v] = (uint8_t)((v * 30 + 127) / 255 | ((v >> 4) == 0 || (v >> 4) == 15 ? EPD_DZ_FIXED : 0));
        CHECK(epd_ingest_dizzy_v(g, dg, PW, 0, 0, PW, PH, lvt, o, PW, 2000, 1) == 0 && epd_ingest_dizzy_src(g) == 0, "a stale rect must be uploaded");
        free(o);
    }
    dizzy_check(g, dg, 0, 0, PW, PH, EPD_GDZ_BLOCK, 0, 1, "LUT levels");
    dizzy_check(g, dg, 37, 5, 901, 603, EPD_GDZ_BLOCK, 0, 0, "odd rect, ALU levels again");
    /* the layered form (the shader), still = epd_dizzy */
    dizzy_check(g, dg, 0, 0, PW, PH, EPD_GDZ_LAYERED, 0, 0, "full panel");
    dizzy_check(g, dg, 37, 5, 901, 603, EPD_GDZ_LAYERED, 0, 0, "odd rect");
    dizzy_check(g, dg, 1, 700, 3, 2, EPD_GDZ_LAYERED, 0, 0, "tiny");
    dizzy_check(g, dg, PW - 77, PH - 50, 77, 50, EPD_GDZ_LAYERED, 0, 0, "corner");
    free(dg);
    epd_ingest_drop(g, ha);
    epd_ingest_close(g);
    printf(fails ? "epd-ingest: FAILED\n" : "epd-ingest: ok\n");
    return fails ? 1 : 0;
}
