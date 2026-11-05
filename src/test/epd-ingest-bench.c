/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-ingest-bench — : the CPU ingest's kernels
 * (epd-ingest-cpu.h), GL-free: no GPU, no dma-buf (malloc'd stand-ins at
 * wlroots 0006's pitches: R8 2048, XRGB 5632), so it runs on the host, under
 * qemu arm64 (the NEON bytes) and on the device (the Gold-core ms).
 * 1. bytes: every kernel (row = today's, tile, tile in generic C) x parts
 * 1..4 on the pool, against the per-pixel reference (epd_grey_px_mode /
 * lut8[q]), over random widened rects (columns on the 4-px texel grid,
 * any rows) and the full frame: R8, XRGB luma, XRGB luminance; the whole
 * panel compared, the bytes outside the rects poisoned. Then every one of
 * the 2^24 colours through the tile kernel (both q) against the scalar q.
 * 2. ms per full frame, per kernel and parts (the caller + parts-1 workers),
 * and where it goes: src = a memcpy of the frame's source bytes alone,
 * hot = the convert of an L1-resident source (R8: one 1872-byte row
 * 1404 times; XRGB: one 64-row band over and over), frame = the real one.
 * epd-ingest-bench [reps=20] env: CPU=n pins the caller; GLASSINE_INGEST_CPUS
 * (default 1-4) the workers; QUICK=1 fewer rects, no 2^24 sweep, reps 3.
 * exit 0 = all equal, 1 = a difference */
#define _GNU_SOURCE
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../epd-ingest-cpu.h"

#define PW 1872
#define PH 1404
#define ST8 2048                        /* R8: pw rounded up to 256 B (wlroots 0006) */
#define ST32 5632                       /* XRGB: 1404 x 4 rounded up to 256 B */

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; fprintf(stderr, "FAIL: " __VA_ARGS__); fputc('\n', stderr); } } while (0)

static uint32_t rs = 0x9e3779b9u;
static uint32_t rnd(void) { rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5; return rs; }

static uint8_t *m8, *m32, *ga, *gb, lut[256], lut8[256];
static icpu_pool pool;
static uint8_t *bnc[ICPU_BANDS_MAX];

static void reference(int r8, int lum, const icpu_box *b, int n, uint8_t *g)
{
    for (int k = 0; k < n; k++)
        for (int row = b[k].r0; row < b[k].r1; row++)
            for (int c = b[k].c0; c < b[k].c0 + b[k].cw; c++)
                g[(size_t)row * PW + c] = r8 ? lut8[m8[(size_t)row * ST8 + c]]
                                             : epd_grey_px_mode(lut, lum, *(const uint32_t *)(m32 + (size_t)(PW - 1 - c) * ST32 + (size_t)row * 4));
}

static void convert(int kernel, int parts, int r8, int lum, const icpu_box *b, int n, uint8_t *g)
{
    icpu_job J = { .kernel = kernel, .lum = lum, .r8 = r8, .st = r8 ? ST8 : ST32, .pw = PW, .n = n, .parts = parts,
                   .lut = lut, .lut8 = lut8, .m = r8 ? m8 : m32, .grey = g, .b = b };
    for (int i = 0; i < ICPU_BANDS_MAX; i++) J.bounce[i] = bnc[i];
    icpu_pool_run(&pool, parts, icpu_job_part, &J);
}

static const char *kname[] = { "row", "tile", "scalar" };
static const char *fname(int r8, int lum) { return r8 ? "R8" : lum ? "XRGB lum" : "XRGB luma"; }

static void bytes_case(int r8, int lum, const icpu_box *b, int n, const char *what)
{
    memset(ga, 0xA5, (size_t)PW * PH);
    reference(r8, lum, b, n, ga);
    for (int kernel = 0; kernel < 3; kernel++)
        for (int parts = 1; parts <= ICPU_BANDS_MAX; parts++) {
            memset(gb, 0xA5, (size_t)PW * PH);
            convert(kernel, parts, r8, lum, b, n, gb);
            long bad = 0;
            for (size_t i = 0; i < (size_t)PW * PH; i++) bad += ga[i] != gb[i];
            CHECK(!bad, "%s %s: kernel %s parts %d: %ld bytes differ", fname(r8, lum), what, kname[kernel], parts, bad);
        }
}

static icpu_box rbox(void)
{
    icpu_box b;
    int t0 = rnd() % (PW / 4), t1 = t0 + 1 + rnd() % (PW / 4 - t0);
    b.c0 = t0 * 4; b.cw = (t1 - t0) * 4;
    b.r0 = rnd() % PH; b.r1 = b.r0 + 1 + rnd() % (PH - b.r0);
    if (rnd() % 4 == 0) { b.cw = 4 * (1 + rnd() % 6); if (b.c0 + b.cw > PW) b.c0 = PW - b.cw; }     /* thin ones */
    if (rnd() % 4 == 0) { b.r1 = b.r0 + 1 + rnd() % 20; if (b.r1 > PH) b.r1 = PH; }
    return b;
}

static double ms_of(int kernel, int parts, int r8, int lum, int reps)
{
    icpu_box full = { 0, PW, 0, PH };
    convert(kernel, parts, r8, lum, &full, 1, gb);           /* warm */
    double t0 = icpu_now();
    for (int i = 0; i < reps; i++) convert(kernel, parts, r8, lum, &full, 1, gb);
    return (icpu_now() - t0) * 1e3 / reps;
}

/* the convert with its source in L1: R8 one row PH times; XRGB one 64-row band (64 x 1404 px = 360 KB: L2) and,
 * for the tile kernel, one 64 x 64 tile over and over (16 KB) */
static double hot_ms(int kernel, int r8, int lum, int reps)
{
    double t0 = icpu_now();
    for (int i = 0; i < reps; i++) {
        if (r8) {
            /* (source stride 0: every panel row reads source row 0) */
            if (kernel == 0) icpu_row_r8(lut8, m8, 0, gb, PW, 0, PW, 0, PH, bnc[0]);
            else icpu_lut_rows(lut8, gb, PW, m8, 0, PW, PH, kernel == 2);
        } else if (kernel == 0) {
            for (int band = 0; band < PW / 8; band++)
                epd_grey_rect_mode(lut, lum, gb, PW, (const uint32_t *)m32, ST32 / 4, 0, PW - 8 - (band % 8) * 8, PH, 8);
        } else {
            uint8_t t[ICPU_T * ICPU_T] __attribute__((aligned(64)));
            for (int k = 0; k < (PW / ICPU_T) * (PH / ICPU_T); k++) {
                icpu_tile_grey(lut, lum, m32, ST32, 0, 0, ICPU_T, ICPU_T, t, kernel == 2);
                icpu_tile_put(t, ICPU_T, ICPU_T, gb, PW, (k % (PH / ICPU_T)) * ICPU_T, (k / (PH / ICPU_T)) * ICPU_T, kernel == 2);
            }
        }
    }
    return (icpu_now() - t0) * 1e3 / reps;
}

static double src_ms(int r8, int reps)
{
    size_t n = r8 ? (size_t)ST8 * PH : (size_t)ST32 * PW;
    uint8_t *d = malloc(n);
    memcpy(d, r8 ? m8 : m32, n);
    double t0 = icpu_now();
    for (int i = 0; i < reps; i++) { memcpy(d, r8 ? m8 : m32, n); __asm__ volatile("" :: "r"(d) : "memory"); }
    double ms = (icpu_now() - t0) * 1e3 / reps;
    free(d);
    return ms;
}

int main(int argc, char **argv)
{
    int quick = getenv("QUICK") && *getenv("QUICK");
    int reps = argc > 1 ? atoi(argv[1]) : quick ? 3 : 20;
    if (reps < 1) reps = 1;
    if (getenv("CPU")) {
        cpu_set_t cs; CPU_ZERO(&cs); CPU_SET(atoi(getenv("CPU")), &cs);
        if (sched_setaffinity(0, sizeof cs, &cs)) perror("epd-ingest-bench: CPU");
        else printf("epd-ingest-bench: caller pinned to cpu %s\n", getenv("CPU"));
    }
    const char *cpus = getenv("GLASSINE_INGEST_CPUS") ? getenv("GLASSINE_INGEST_CPUS") : "1-4";
    icpu_pool_start(&pool, ICPU_BANDS_MAX - 1, cpus);
    for (int i = 0; i < ICPU_BANDS_MAX; i++) bnc[i] = malloc(ICPU_BOUNCE(PW, PH));
    m8 = malloc((size_t)ST8 * PH);
    m32 = malloc((size_t)ST32 * PW);
    ga = malloc((size_t)PW * PH);
    gb = malloc((size_t)PW * PH);
    for (size_t i = 0; i < (size_t)ST8 * PH; i++) m8[i] = (uint8_t)rnd();
    for (size_t i = 0; i < (size_t)ST32 * PW / 4; i++) ((uint32_t *)m32)[i] = rnd();
    static const uint32_t ed[] = { 0, 0xffffff, 0xff0000, 0x00ff00, 0x0000ff, 0x010101, 0xfefefe, 0x808080, 0x7f7f7f, 0xff00ff };
    for (int i = 0; i < 4000; i++) ((uint32_t *)m32)[rnd() % ((size_t)ST32 * PW / 4)] = ed[i % 10];
    epd_grey_lum_lut(lut8, 1.8);                    /* non-identity */
    printf("epd-ingest-bench: %s, pool of %d workers on cpus %s\n",
#ifdef __aarch64__
           "arm64 (NEON)",
#else
           "generic C",
#endif
           pool.nth, cpus);

    /* 1. bytes */
    for (int fmt = 0; fmt < 3; fmt++) {
        int r8 = fmt == 0, lum = fmt == 2;
        if (lum) epd_grey_lum_lut(lut, 1.8); else epd_grey_luma_lut(lut, 1.8);
        icpu_box full = { 0, PW, 0, PH };
        bytes_case(r8, lum, &full, 1, "full frame");
        int cases = quick ? 6 : 40;
        for (int c = 0; c < cases; c++) {
            icpu_box b[5];
            int n = 1 + rnd() % 5;
            for (int k = 0; k < n; k++) b[k] = rbox();
            /* (rects of one frame may overlap: then the last one's bytes win in every kernel only if they agree:
             * they do, the same source) */
            bytes_case(r8, lum, b, n, "random rects");
        }
    }
    printf("epd-ingest-bench: bytes: 3 kernels x 4 parts x %d cases x 3 formats, %s\n", quick ? 7 : 41, fails ? "DIFFER" : "0 differ");
    if (!quick) {           /* every colour through the tile kernel (the full frame's source, 7 passes) */
        long bad = 0;
        for (int lum = 0; lum < 2; lum++) {
            if (lum) epd_grey_lum_lut(lut, 1.0); else epd_grey_luma_lut(lut, 1.0);     /* identity-ish: q shows */
            for (uint32_t base = 0; base < (1u << 24); base += PW * PH) {
                for (int y = 0; y < PW; y++)
                    for (int x = 0; x < PH; x++) ((uint32_t *)(m32 + (size_t)y * ST32))[x] = (base + (uint32_t)y * PH + x) & 0xffffff;
                icpu_box full = { 0, PW, 0, PH };
                convert(1, 1, 0, lum, &full, 1, gb);
                reference(0, lum, &full, 1, ga);
                for (size_t i = 0; i < (size_t)PW * PH; i++) bad += ga[i] != gb[i];
            }
        }
        CHECK(!bad, "2^24 colours: %ld differ", bad);
        printf("epd-ingest-bench: 2^24 colours x luma + luminance through the tile kernel: %ld differ\n", bad);
        for (size_t i = 0; i < (size_t)ST32 * PW / 4; i++) ((uint32_t *)m32)[i] = rnd();
    }

    /* 2. time */
    epd_grey_luma_lut(lut, 1.8);
    for (int fmt = 0; fmt < 3; fmt++) {
        int r8 = fmt == 0, lum = fmt == 2;
        if (lum) epd_grey_lum_lut(lut, 1.8); else epd_grey_luma_lut(lut, 1.8);
        printf("epd-ingest-bench: %-9s full frame (%.1f MB in): src memcpy %.2f ms\n", fname(r8, lum),
               (r8 ? (double)ST8 * PH : (double)ST32 * PW) / 1e6, src_ms(r8, reps));
        for (int kernel = 0; kernel < 3; kernel++) {
            printf("epd-ingest-bench: %-9s %-6s hot %.2f ms; frame", fname(r8, lum), kname[kernel], hot_ms(kernel, r8, lum, reps));
            for (int parts = 1; parts <= ICPU_BANDS_MAX; parts++) printf(" x%d %.2f", parts, ms_of(kernel, parts, r8, lum, reps));
            printf(" ms\n");
        }
    }
    printf("epd-ingest-bench: pool jobs %lu, parts that found their job spinning %lu\n", pool.jobs, pool.spun);
    icpu_pool_stop(&pool);
    printf(fails ? "epd-ingest-bench: FAILED\n" : "epd-ingest-bench: ok\n");
    return fails ? 1 : 0;
}
