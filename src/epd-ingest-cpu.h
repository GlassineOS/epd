/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-ingest-cpu — : the CPU ingest's convert,
 * cheap. epd_ingest_frame_cpu (epd-ingest.c) reads sway's buffer through its
 * mapping; on the A509 device (Gold core, R8 full frame) that read cost 9.5 ms
 * against a ~1 ms bandwidth floor, the mapping itself reading at malloc speed
 * (heap-bw-probe). Today's row way: a memcpy bounce per row, then epd_grey_lut
 * (R8) or epd_grey_rect_mode over 8 source rows (XRGB). GCC keeps those LUTs
 * on the stack (the four 64-byte tbl tables reloaded per 16 bytes: 16 vector
 * loads per 16 bytes out, arm64 -O2 disassembly) and the XRGB pass
 * writes 8 bytes into each of up to 1404 panel rows per 8 source rows (lines
 * written 8 times, 90 KB of them live: past L1).
 *
 * Here, GL-free (test/epd-ingest-bench runs it without a GPU, and under qemu
 * arm64 for the NEON bytes):
 * - the tile kernel: R8 rows straight from the mapping through the LUT, the
 * table in registers (four independent tbl + an or tree instead of a tbx
 * chain), 64 bytes a step; XRGB in 64 x 64 tiles (16 KB in, 4 KB out = 64
 * whole panel lines): q + LUT into a 4 KB L1 tile in source order (luma
 * fused; luminance's square-root table first, the LUT then in place: both
 * tables do not fit the 32 vector registers), then a 16 x 16 tbl-free
 * byte transpose into the panel. The same q, the same LUT, byte for byte
 * (epd-grey.h's formulas; the edges scalar). Generic C off arm64 (x86:
 * the same tiles, scalar).
 * - the bands: the rects' panel rows split over a small pool (whole rows per
 * part: no shared lines), part 0 on the caller; primed before the START
 * ioctl so the workers' wake-up (1.3-1.4 ms on the device) runs
 * under it; workers spin <= ICPU_SPIN_S for the job, then sleep.
 * - the sync helper: DMA_BUF_IOCTL_SYNC on a thread pinned to one CPU (a
 * Silver core invalidates at ~1/3 of a Gold core's cost per MB, R1
 * measured); START waited for, END posted and not waited for (the CPU
 * never writes the mapping: no dirty line to lose; a later START queues
 * behind it). */
#ifndef EPD_INGEST_CPU_H
#define EPD_INGEST_CPU_H
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <pthread.h>
#include <sched.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/dma-buf.h>
#include "epd-grey.h"

#define ICPU_T 64               /* tile: 64 source rows x 64 source pixels */
#define ICPU_BANDS_MAX 4
#define ICPU_BAND_MIN_PX 300000L        /* below: one part (as 's EPD_BAND_MIN_PX) */
#define ICPU_SPIN_S 0.003       /* a primed worker spins this long for its job, then sleeps */

static inline double icpu_now(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}

static inline void icpu_relax(void)
{
#if defined(__aarch64__)
    __asm__ volatile("yield" ::: "memory");
#elif defined(__x86_64__) || defined(__i386__)
    __builtin_ia32_pause();
#endif
}

/* ---- today's row way, over panel rows [r0, r1) of one widened rect (the
 * bands' parts with ingest_kernel=row; epd_ingest_frame_cpu's own loop is the
 * same code) */
static inline void icpu_row_r8(const uint8_t *lut8, const uint8_t *m, int st, uint8_t *grey, int pw,
                               int c0, int cw, int r0, int r1, uint8_t *bounce)
{
    for (int row = r0; row < r1; row++) {
        memcpy(bounce, m + (size_t)row * st + c0, (size_t)cw);
        epd_grey_lut(lut8, grey + (size_t)row * pw + c0, bounce, cw);
    }
}

static inline void icpu_row_xrgb(const uint8_t *lut, int lum, const uint8_t *m, int st, uint8_t *grey, int pw,
                                 int c0, int cw, int r0, int r1, uint8_t *bounce)
{
    int y = pw - c0 - cw, w = r1 - r0;
    for (int yb = y; yb < y + cw; yb += 8) {
        int bh = y + cw - yb < 8 ? y + cw - yb : 8;
        for (int k = 0; k < bh; k++)
            memcpy(bounce + (size_t)k * w * 4, m + (size_t)(yb + k) * st + (size_t)r0 * 4, (size_t)w * 4);
        epd_grey_rect_mode(lut, lum, grey, pw, (const uint32_t *)bounce, w, r0, yb, w, bh);
    }
}

/* ---- the tile kernel ---- */
#ifdef __aarch64__
/* rows x nb 64-byte blocks through a 256-byte LUT, src rows ss apart, dst
 * rows ds apart (dst may be src). The table stays in v16-v31 for the whole
 * call: GCC reloads tbl tables from the stack at every use, intrinsics or a
 * struct of vectors alike (arm64 -O2 disassembly), 16 vector
 * loads per 64 bytes out. Four lookups per 16 bytes (tbl: out of range -> 0;
 * v ^ 64k lands in range only for the quarter it addresses) or-ed: no tbx
 * chain through one register; three of the four vectors' lookups issued
 * before the first or */
static inline void icpu_lut_asm(const uint8_t *lut, uint8_t *dst, long ds, const uint8_t *src, long ss, long nb, long rows)
{
    if (nb <= 0 || rows <= 0) return;
    __asm__ volatile(
        "ld1 {v16.16b-v19.16b}, [%[t]], #64\n\t"
        "ld1 {v20.16b-v23.16b}, [%[t]], #64\n\t"
        "ld1 {v24.16b-v27.16b}, [%[t]], #64\n\t"
        "ld1 {v28.16b-v31.16b}, [%[t]]\n\t"
        "movi v13.16b, #0x40\n\t"
        "movi v14.16b, #0x80\n\t"
        "movi v15.16b, #0xc0\n\t"
        "0:\n\t"
        "mov x9, %[s]\n\t"
        "mov x10, %[d]\n\t"
        "mov x11, %[nb]\n\t"
        "1:\n\t"
        "ld1 {v0.16b-v3.16b}, [x9], #64\n\t"
        "eor v4.16b, v0.16b, v13.16b\n\t"
        "eor v5.16b, v0.16b, v14.16b\n\t"
        "eor v6.16b, v0.16b, v15.16b\n\t"
        "eor v7.16b, v1.16b, v13.16b\n\t"
        "eor v8.16b, v1.16b, v14.16b\n\t"
        "eor v9.16b, v1.16b, v15.16b\n\t"
        "eor v10.16b, v2.16b, v13.16b\n\t"
        "eor v11.16b, v2.16b, v14.16b\n\t"
        "eor v12.16b, v2.16b, v15.16b\n\t"
        "tbl v0.16b, {v16.16b-v19.16b}, v0.16b\n\t"
        "tbl v4.16b, {v20.16b-v23.16b}, v4.16b\n\t"
        "tbl v5.16b, {v24.16b-v27.16b}, v5.16b\n\t"
        "tbl v6.16b, {v28.16b-v31.16b}, v6.16b\n\t"
        "tbl v1.16b, {v16.16b-v19.16b}, v1.16b\n\t"
        "tbl v7.16b, {v20.16b-v23.16b}, v7.16b\n\t"
        "tbl v8.16b, {v24.16b-v27.16b}, v8.16b\n\t"
        "tbl v9.16b, {v28.16b-v31.16b}, v9.16b\n\t"
        "tbl v2.16b, {v16.16b-v19.16b}, v2.16b\n\t"
        "tbl v10.16b, {v20.16b-v23.16b}, v10.16b\n\t"
        "tbl v11.16b, {v24.16b-v27.16b}, v11.16b\n\t"
        "tbl v12.16b, {v28.16b-v31.16b}, v12.16b\n\t"
        "orr v0.16b, v0.16b, v4.16b\n\t"
        "orr v5.16b, v5.16b, v6.16b\n\t"
        "eor v4.16b, v3.16b, v13.16b\n\t"
        "eor v6.16b, v3.16b, v15.16b\n\t"
        "orr v0.16b, v0.16b, v5.16b\n\t"
        "eor v5.16b, v3.16b, v14.16b\n\t"
        "orr v1.16b, v1.16b, v7.16b\n\t"
        "orr v8.16b, v8.16b, v9.16b\n\t"
        "tbl v3.16b, {v16.16b-v19.16b}, v3.16b\n\t"
        "tbl v4.16b, {v20.16b-v23.16b}, v4.16b\n\t"
        "tbl v5.16b, {v24.16b-v27.16b}, v5.16b\n\t"
        "tbl v6.16b, {v28.16b-v31.16b}, v6.16b\n\t"
        "orr v1.16b, v1.16b, v8.16b\n\t"
        "orr v2.16b, v2.16b, v10.16b\n\t"
        "orr v11.16b, v11.16b, v12.16b\n\t"
        "orr v3.16b, v3.16b, v4.16b\n\t"
        "orr v5.16b, v5.16b, v6.16b\n\t"
        "orr v2.16b, v2.16b, v11.16b\n\t"
        "orr v3.16b, v3.16b, v5.16b\n\t"
        "st1 {v0.16b-v3.16b}, [x10], #64\n\t"
        "subs x11, x11, #1\n\t"
        "b.ne 1b\n\t"
        "add %[s], %[s], %[ss]\n\t"
        "add %[d], %[d], %[ds]\n\t"
        "subs %[rows], %[rows], #1\n\t"
        "b.ne 0b\n\t"
        : [s] "+r"(src), [d] "+r"(dst), [rows] "+r"(rows), [t] "+r"(lut)
        : [ss] "r"(ss), [ds] "r"(ds), [nb] "r"(nb)
        : "x9", "x10", "x11", "v0", "v1", "v2", "v3", "v4", "v5", "v6", "v7", "v8", "v9", "v10", "v11", "v12", "v13",
          "v14", "v15", "v16", "v17", "v18", "v19", "v20", "v21", "v22", "v23", "v24", "v25", "v26", "v27", "v28", "v29",
          "v30", "v31", "memory", "cc");
}
#endif

/* rows x n bytes through a 256-byte LUT (dst may be src: element-wise).
 * scalar: the generic C (also on arm64: ingest_kernel=scalar, the A/B) */
static inline void icpu_lut_rows(const uint8_t *lut, uint8_t *dst, int ds, const uint8_t *src, int ss, int n, int rows,
                                 int scalar)
{
    int i0 = 0;
#ifdef __aarch64__
    if (!scalar) {
        icpu_lut_asm(lut, dst, ds, src, ss, n / 64, rows);
        i0 = n & ~63;
    }
#endif
    (void)scalar;
    if (i0 < n)
        for (int r = 0; r < rows; r++) {
            const uint8_t *s = src + (size_t)r * ss;
            uint8_t *d = dst + (size_t)r * ds;
            for (int i = i0; i < n; i++) d[i] = lut[s[i]];
        }
}

/* pass A: th source rows of tw pixels at (x0, y0) -> the tile t (row = source
 * row, ICPU_T bytes apart), the panel's grey (LUT applied). arm64: table-free
 * passes in intrinsics, every lookup through icpu_lut_rows (the table pinned):
 * luma = q, then the LUT in place; luminance = the channels into three planes,
 * S over them, q from the planes, then the LUT */
static inline void icpu_tile_grey(const uint8_t *lut, int lum, const uint8_t *m, int st, int x0, int y0, int tw, int th,
                                  uint8_t *t, int scalar)
{
#ifdef __aarch64__
    if (!scalar) {
        int n16 = tw & ~15;
        if (!lum) {
            for (int r = 0; r < th; r++) {
                const uint32_t *p = (const uint32_t *)(m + (size_t)(y0 + r) * st) + x0;
                uint8_t *o = t + (size_t)r * ICPU_T;
                int i = 0;
                for (; i < n16; i += 16) {
                    uint8x16x4_t c = vld4q_u8((const uint8_t *)(p + i));         /* B G R X */
                    uint16x8_t lo = vmull_u8(vget_low_u8(c.val[2]), vdup_n_u8(77));
                    lo = vmlal_u8(lo, vget_low_u8(c.val[1]), vdup_n_u8(150));
                    lo = vmlal_u8(lo, vget_low_u8(c.val[0]), vdup_n_u8(29));
                    uint16x8_t hi = vmull_u8(vget_high_u8(c.val[2]), vdup_n_u8(77));
                    hi = vmlal_u8(hi, vget_high_u8(c.val[1]), vdup_n_u8(150));
                    hi = vmlal_u8(hi, vget_high_u8(c.val[0]), vdup_n_u8(29));
                    vst1q_u8(o + i, vcombine_u8(vshrn_n_u16(lo, 8), vshrn_n_u16(hi, 8)));
                }
                for (; i < tw; i++) o[i] = (uint8_t)((((p[i] >> 16) & 0xff) * 77 + ((p[i] >> 8) & 0xff) * 150 + (p[i] & 0xff) * 29) >> 8);
            }
            icpu_lut_rows(lut, t, ICPU_T, t, ICPU_T, tw, th, 0);
            return;
        }
        uint8_t pl[3 * ICPU_T * ICPU_T] __attribute__((aligned(64)));     /* R, G, B */
        uint8_t *pr = pl, *pg = pl + ICPU_T * ICPU_T, *pb = pl + 2 * ICPU_T * ICPU_T;
        for (int r = 0; r < th; r++) {
            const uint32_t *p = (const uint32_t *)(m + (size_t)(y0 + r) * st) + x0;
            size_t o = (size_t)r * ICPU_T;
            int i = 0;
            for (; i < n16; i += 16) {
                uint8x16x4_t c = vld4q_u8((const uint8_t *)(p + i));
                vst1q_u8(pr + o + i, c.val[2]);
                vst1q_u8(pg + o + i, c.val[1]);
                vst1q_u8(pb + o + i, c.val[0]);
            }
            for (; i < tw; i++) { pr[o + i] = (p[i] >> 16) & 0xff; pg[o + i] = (p[i] >> 8) & 0xff; pb[o + i] = p[i] & 0xff; }
        }
        if (tw == ICPU_T && th == ICPU_T) icpu_lut_rows(epd_grey_sq, pl, ICPU_T, pl, ICPU_T, ICPU_T, 3 * ICPU_T, 0);
        else for (int k = 0; k < 3; k++) icpu_lut_rows(epd_grey_sq, pl + k * ICPU_T * ICPU_T, ICPU_T, pl + k * ICPU_T * ICPU_T, ICPU_T, tw, th, 0);
        for (int r = 0; r < th; r++) {
            size_t o = (size_t)r * ICPU_T;
            int i = 0;
            for (; i < n16; i += 16) {      /* (epd_grey8_q's steps, 4 lanes at a time) */
                uint8x16_t sr = vld1q_u8(pr + o + i), sg = vld1q_u8(pg + o + i), sb = vld1q_u8(pb + o + i);
                uint16x8_t r2[2] = { vmull_u8(vget_low_u8(sr), vget_low_u8(sr)), vmull_high_u8(sr, sr) };
                uint16x8_t g2[2] = { vmull_u8(vget_low_u8(sg), vget_low_u8(sg)), vmull_high_u8(sg, sg) };
                uint16x8_t b2[2] = { vmull_u8(vget_low_u8(sb), vget_low_u8(sb)), vmull_high_u8(sb, sb) };
                uint16x4_t q4[4];
                for (int h = 0; h < 4; h++) {
                    uint16x4_t rr = h & 1 ? vget_high_u16(r2[h >> 1]) : vget_low_u16(r2[h >> 1]);
                    uint16x4_t gg = h & 1 ? vget_high_u16(g2[h >> 1]) : vget_low_u16(g2[h >> 1]);
                    uint16x4_t bb = h & 1 ? vget_high_u16(b2[h >> 1]) : vget_low_u16(b2[h >> 1]);
                    uint32x4_t y = vmull_n_u16(rr, EPD_GREY_WR);
                    y = vmlal_n_u16(y, gg, EPD_GREY_WG);
                    y = vmlal_n_u16(y, bb, EPD_GREY_WB);
                    q4[h] = vmovn_u32(epd_grey_isqrt4(vshrq_n_u32(y, 15)));
                }
                vst1q_u8(t + o + i, vcombine_u8(vmovn_u16(vcombine_u16(q4[0], q4[1])), vmovn_u16(vcombine_u16(q4[2], q4[3]))));
            }
            for (; i < tw; i++) {
                uint32_t sr = pr[o + i], sg = pg[o + i], sb = pb[o + i];
                t[o + i] = (uint8_t)epd_grey_isqrt((EPD_GREY_WR * sr * sr + EPD_GREY_WG * sg * sg + EPD_GREY_WB * sb * sb) >> 15);
            }
        }
        icpu_lut_rows(lut, t, ICPU_T, t, ICPU_T, tw, th, 0);       /* q -> grey, in place */
        return;
    }
#endif
    (void)scalar;
    for (int r = 0; r < th; r++) {
        const uint32_t *p = (const uint32_t *)(m + (size_t)(y0 + r) * st) + x0;
        uint8_t *o = t + (size_t)r * ICPU_T;
        for (int i = 0; i < tw; i++) o[i] = epd_grey_px_mode(lut, lum, p[i]);
    }
}

#ifdef __aarch64__
/* 16 x 16 bytes in place: a[j] = column j (trn at 1, 2, 4, 8 bytes; the
 * pairs i, i + s with bit s of i clear, the lower half of each into i) */
static inline void icpu_tr16(uint8x16_t a[16])
{
    for (int i = 0; i < 16; i += 2) {
        uint8x16_t x = a[i], y = a[i + 1];
        a[i] = vtrn1q_u8(x, y); a[i + 1] = vtrn2q_u8(x, y);
    }
    for (int i = 0; i < 16; i++) if (!(i & 2)) {
        uint16x8_t x = vreinterpretq_u16_u8(a[i]), y = vreinterpretq_u16_u8(a[i + 2]);
        a[i] = vreinterpretq_u8_u16(vtrn1q_u16(x, y)); a[i + 2] = vreinterpretq_u8_u16(vtrn2q_u16(x, y));
    }
    for (int i = 0; i < 16; i++) if (!(i & 4)) {
        uint32x4_t x = vreinterpretq_u32_u8(a[i]), y = vreinterpretq_u32_u8(a[i + 4]);
        a[i] = vreinterpretq_u8_u32(vtrn1q_u32(x, y)); a[i + 4] = vreinterpretq_u8_u32(vtrn2q_u32(x, y));
    }
    for (int i = 0; i < 8; i++) {
        uint64x2_t x = vreinterpretq_u64_u8(a[i]), y = vreinterpretq_u64_u8(a[i + 8]);
        a[i] = vreinterpretq_u8_u64(vtrn1q_u64(x, y)); a[i + 8] = vreinterpretq_u8_u64(vtrn2q_u64(x, y));
    }
}
#endif

/* pass B: the tile (th x tw, source order) into the panel: source pixel
 * (x0 + j, y0 + i) -> grey[(x0 + j) * pw + pw - 1 - (y0 + i)] */
static inline void icpu_tile_put(const uint8_t *t, int tw, int th, uint8_t *grey, int pw, int x0, int y0, int scalar)
{
    int th16 = 0, tw16 = 0;
#ifdef __aarch64__
    if (!scalar) {
        th16 = th & ~15; tw16 = tw & ~15;
        for (int i0 = 0; i0 < th16; i0 += 16)
            for (int j0 = 0; j0 < tw16; j0 += 16) {
                uint8x16_t a[16];
                for (int k = 0; k < 16; k++) a[k] = vld1q_u8(t + (size_t)(i0 + 15 - k) * ICPU_T + j0);
                icpu_tr16(a);
                uint8_t *d = grey + (size_t)(x0 + j0) * pw + (pw - 1 - (y0 + i0 + 15));
                for (int j = 0; j < 16; j++) vst1q_u8(d + (size_t)j * pw, a[j]);
            }
    }
#endif
    (void)scalar;
    for (int j = 0; j < tw; j++) {      /* what the blocks left: rows past th16, columns past tw16 */
        uint8_t *d = grey + (size_t)(x0 + j) * pw + (pw - 1 - y0);
        const uint8_t *s = t + j;
        for (int i = j < tw16 ? th16 : 0; i < th; i++) d[-i] = s[(size_t)i * ICPU_T];
    }
}

/* one widened rect's panel rows [r0, r1) (XRGB: its source pixels), panel
 * columns [c0, c0 + cw) (XRGB: source rows pw - c0 - cw ..): the tiles */
static inline void icpu_tile_xrgb(const uint8_t *lut, int lum, const uint8_t *m, int st, uint8_t *grey, int pw,
                                  int c0, int cw, int r0, int r1, int scalar)
{
    uint8_t t[ICPU_T * ICPU_T] __attribute__((aligned(64)));
    int y = pw - c0 - cw;
    (void)scalar;
    for (int ty = y; ty < y + cw; ty += ICPU_T) {
        int th = y + cw - ty < ICPU_T ? y + cw - ty : ICPU_T;
        for (int tx = r0; tx < r1; tx += ICPU_T) {
            int tw = r1 - tx < ICPU_T ? r1 - tx : ICPU_T;
#ifdef __aarch64__
            if (!scalar) {
                icpu_tile_grey(lut, lum, m, st, tx, ty, tw, th, t, 0);
                icpu_tile_put(t, tw, th, grey, pw, tx, ty, 0);
                continue;
            }
#endif
            (void)t;
            /* generic: straight into the tile's 64 panel lines (L1), no second pass */
            for (int r = 0; r < th; r++) {
                const uint32_t *p = (const uint32_t *)(m + (size_t)(ty + r) * st) + tx;
                uint8_t *d = grey + (size_t)tx * pw + (pw - 1 - (ty + r));
                for (int i = 0; i < tw; i++) d[(size_t)i * pw] = epd_grey_px_mode(lut, lum, p[i]);
            }
        }
    }
}

/* kernel: 0 row (today's), 1 tile, 2 tile in generic C. bounce: the row
 * way's (8 rows of the buffer's longer side x 4 bytes) */
static inline void icpu_rect(int kernel, const uint8_t *lut, const uint8_t *lut8, int lum, int r8, const uint8_t *m,
                             int st, uint8_t *grey, int pw, int c0, int cw, int r0, int r1, uint8_t *bounce)
{
    if (r1 <= r0 || cw <= 0) return;
    if (r8 && !lut8) {                  /* the table applied already (sway's GPU): a copy */
        for (int row = r0; row < r1; row++) memcpy(grey + (size_t)row * pw + c0, m + (size_t)row * st + c0, (size_t)cw);
        return;
    }
    if (!kernel) {
        if (r8) icpu_row_r8(lut8, m, st, grey, pw, c0, cw, r0, r1, bounce);
        else icpu_row_xrgb(lut, lum, m, st, grey, pw, c0, cw, r0, r1, bounce);
    } else if (r8) icpu_lut_rows(lut8, grey + (size_t)r0 * pw + c0, pw, m + (size_t)r0 * st + c0, st, cw, r1 - r0, kernel == 2);
    else icpu_tile_xrgb(lut, lum, m, st, grey, pw, c0, cw, r0, r1, kernel == 2);
}

/* part i of n of panel rows [r0, r1): whole tiles (XRGB tile rows) per part */
static inline void icpu_part(int r0, int r1, int i, int n, int *a, int *b)
{
    int nt = (r1 - r0 + ICPU_T - 1) / ICPU_T;
    *a = r0 + nt * i / n * ICPU_T;
    *b = r0 + nt * (i + 1) / n * ICPU_T;
    if (*a > r1) *a = r1;
    if (*b > r1) *b = r1;
}

/* a frame's widened rects ([c0, c0 + cw) x panel rows [r0, r1)) as one job
 * of parts parts: part i converts its share of every rect's rows */
typedef struct { int c0, cw, r0, r1; } icpu_box;
typedef struct icpu_job {
    int kernel, lum, r8, st, pw, n, parts;
    const uint8_t *lut, *lut8, *m;
    uint8_t *grey;
    const icpu_box *b;
    uint8_t *bounce[ICPU_BANDS_MAX];    /* the row way's, one per part */
} icpu_job;
#define ICPU_BOUNCE(pw, ph) ((size_t)8 * 4 * ((ph) > (pw) ? (ph) : (pw)))

static inline void icpu_job_part(void *arg, int i)
{
    const icpu_job *J = arg;
    int lo = 1 << 30, hi = 0, a, b;
    for (int k = 0; k < J->n; k++) {    /* the parts split the rects' rows as a whole: a row is one part's, */
        if (J->b[k].r0 < lo) lo = J->b[k].r0;      /* also where damage rects overlap */
        if (J->b[k].r1 > hi) hi = J->b[k].r1;
    }
    icpu_part(lo, hi, i, J->parts, &a, &b);
    for (int k = 0; k < J->n; k++) {
        int r0 = J->b[k].r0 > a ? J->b[k].r0 : a, r1 = J->b[k].r1 < b ? J->b[k].r1 : b;
        icpu_rect(J->kernel, J->lut, J->lut8, J->lum, J->r8, J->m, J->st, J->grey, J->pw, J->b[k].c0, J->b[k].cw, r0, r1,
                  J->bounce[i]);
    }
}

/* ---- the pool: run(fn, arg, n) = fn(arg, i) for i < n, part 0 on the
 * caller, back when all ran; prime = wake the workers now (they spin for
 * the job ICPU_SPIN_S, then sleep). Workers inherit the starter's policy;
 * pinned to cpus (none: unpinned). One caller thread. */
typedef struct icpu_pool {
    int nth;                            /* workers started (1..nth) */
    pthread_t th[ICPU_BANDS_MAX];
    pthread_mutex_t mu;
    pthread_cond_t go, done;
    unsigned pgen, jgen;                /* wakes; jobs (jgen also read atomically) */
    int jn, left, quit;                 /* left: atomic */
    void (*fn)(void *, int);
    void *arg;
    cpu_set_t cpus;
    int pin;
    unsigned long jobs, spun;           /* jobs run; parts that found the job spinning (primed) */
} icpu_pool;

struct icpu_wk { icpu_pool *p; int k; unsigned pseen, jseen; };    /* the gens at its start (a run right after it is its job) */

static inline void *icpu_worker(void *arg)
{
    struct icpu_wk w = *(struct icpu_wk *)arg;
    free(arg);
    icpu_pool *P = w.p;
    if (P->pin) pthread_setaffinity_np(pthread_self(), sizeof P->cpus, &P->cpus);
    unsigned pseen = w.pseen, jseen = w.jseen;
    for (;;) {
        pthread_mutex_lock(&P->mu);
        while (P->pgen == pseen && P->jgen == jseen && !P->quit) pthread_cond_wait(&P->go, &P->mu);
        pseen = P->pgen;
        int quit = P->quit;
        pthread_mutex_unlock(&P->mu);
        if (quit) break;
        int spun = 0;
        double until = icpu_now() + ICPU_SPIN_S;
        while (__atomic_load_n(&P->jgen, __ATOMIC_ACQUIRE) == jseen) {
            if (icpu_now() > until) break;
            spun = 1;
            icpu_relax();
        }
        pthread_mutex_lock(&P->mu);             /* the job and its gen together */
        unsigned j = P->jgen;
        int jn = P->jn;
        void (*fn)(void *, int) = P->fn;
        void *a = P->arg;
        pthread_mutex_unlock(&P->mu);
        if (j == jseen) continue;               /* primed, no job in time: sleep */
        jseen = j;
        if (w.k >= jn) continue;
        fn(a, w.k);
        if (spun) __atomic_add_fetch(&P->spun, 1, __ATOMIC_RELAXED);
        if (__atomic_sub_fetch(&P->left, 1, __ATOMIC_ACQ_REL) == 0) {
            pthread_mutex_lock(&P->mu);
            pthread_cond_broadcast(&P->done);
            pthread_mutex_unlock(&P->mu);
        }
    }
    return NULL;
}

/* "1-4,6" -> set; 0: none */
static inline int icpu_cpus(const char *s, cpu_set_t *set)
{
    int k = 0;
    CPU_ZERO(set);
    while (s && *s) {
        char *e;
        long a = strtol(s, &e, 10), b = a;
        if (e == s) break;
        if (*e == '-') { s = e + 1; b = strtol(s, &e, 10); if (e == s) break; }
        for (long c = a; c <= b && c < CPU_SETSIZE; c++) if (c >= 0) { CPU_SET((int)c, set); k++; }
        s = *e == ',' ? e + 1 : e;
        if (*e != ',') break;
    }
    return k;
}

/* n workers (parts 1..n), started once (more on a later call); cpus: a list */
static inline int icpu_pool_start(icpu_pool *P, int n, const char *cpus)
{
    if (n > ICPU_BANDS_MAX - 1) n = ICPU_BANDS_MAX - 1;
    if (!P->nth && !P->quit) {
        pthread_mutex_init(&P->mu, NULL);
        pthread_cond_init(&P->go, NULL);
        pthread_cond_init(&P->done, NULL);
        P->pin = icpu_cpus(cpus, &P->cpus) > 0;
    }
    while (P->nth < n) {
        struct icpu_wk *w = malloc(sizeof *w);
        if (!w) break;
        w->p = P; w->k = P->nth + 1; w->pseen = P->pgen; w->jseen = P->jgen;
        if (pthread_create(&P->th[P->nth + 1], NULL, icpu_worker, w)) { free(w); break; }
        P->nth++;
    }
    return P->nth;
}

static inline void icpu_pool_prime(icpu_pool *P)
{
    if (!P->nth) return;
    pthread_mutex_lock(&P->mu);
    P->pgen++;
    pthread_cond_broadcast(&P->go);
    pthread_mutex_unlock(&P->mu);
}

static inline void icpu_pool_run(icpu_pool *P, int n, void (*fn)(void *, int), void *arg)
{
    if (n > P->nth + 1) n = P->nth + 1;
    if (n <= 1) { fn(arg, 0); return; }
    pthread_mutex_lock(&P->mu);
    P->fn = fn; P->arg = arg; P->jn = n;
    __atomic_store_n(&P->left, n - 1, __ATOMIC_RELAXED);
    __atomic_store_n(&P->jgen, P->jgen + 1, __ATOMIC_RELEASE);
    P->pgen++;
    pthread_cond_broadcast(&P->go);
    pthread_mutex_unlock(&P->mu);
    P->jobs++;
    fn(arg, 0);
    double until = icpu_now() + ICPU_SPIN_S;
    while (__atomic_load_n(&P->left, __ATOMIC_ACQUIRE) && icpu_now() < until) icpu_relax();
    if (__atomic_load_n(&P->left, __ATOMIC_ACQUIRE)) {
        pthread_mutex_lock(&P->mu);
        while (__atomic_load_n(&P->left, __ATOMIC_ACQUIRE)) pthread_cond_wait(&P->done, &P->mu);
        pthread_mutex_unlock(&P->mu);
    }
}

static inline void icpu_pool_stop(icpu_pool *P)
{
    if (!P->nth) return;
    pthread_mutex_lock(&P->mu);
    P->quit = 1;
    pthread_cond_broadcast(&P->go);
    pthread_mutex_unlock(&P->mu);
    for (int i = 1; i <= P->nth; i++) pthread_join(P->th[i], NULL);
    P->nth = 0;
}

/* ---- the sync helper: DMA_BUF_IOCTL_SYNC ops in order on one thread ---- */
/* DMA_BUF_IOCTL_SYNC_PARTIAL (GlassineOS/linux 9fbb3532981d, the Android
 * kernels' ABI): the cache maintenance of DMA_BUF_IOCTL_SYNC for the bytes
 * [offset, offset + len) only. Not in the uapi headers: defined here */
struct icpu_sync_partial { __u64 flags; __u32 offset, len; };
#define ICPU_SYNC_PARTIAL _IOW(DMA_BUF_BASE, 2, struct icpu_sync_partial)
/* 1: try the range ioctl; -1: the kernel has none (ENOTTY once: whole-buffer
 * syncs from then on). The switch (ingest_partial) is the caller's: len 0 */
static int icpu_partial = 1;

/* one sync: len 0 = the whole buffer; else the range, or the whole buffer
 * where the kernel has no range ioctl */
static inline void icpu_sync_ioctl(int fd, unsigned flags, unsigned off, unsigned len)
{
    if (len && __atomic_load_n(&icpu_partial, __ATOMIC_RELAXED) > 0) {
        struct icpu_sync_partial p = { flags, off, len };
        if (ioctl(fd, ICPU_SYNC_PARTIAL, &p) == 0) return;
        if (errno == ENOTTY) __atomic_store_n(&icpu_partial, -1, __ATOMIC_RELAXED);
    }
    struct dma_buf_sync sy = { flags };
    ioctl(fd, DMA_BUF_IOCTL_SYNC, &sy);
}

#define ICPU_SYNC_Q 16
typedef struct icpu_sync {
    int up, cpu;
    pthread_t th;
    pthread_mutex_t mu;
    pthread_cond_t work, done;
    struct { int fd; unsigned flags, off, len; } q[ICPU_SYNC_Q];
    unsigned head, tail, fin;           /* queued, taken, finished (fin: atomic) */
    int quit;
    double st_s, st_max, en_s, en_max;  /* the ioctls' own time on the helper */
    unsigned long st_n, en_n;
} icpu_sync;

static inline void *icpu_sync_thread(void *arg)
{
    icpu_sync *S = arg;
    if (S->cpu >= 0) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(S->cpu, &set);
        pthread_setaffinity_np(pthread_self(), sizeof set, &set);
    }
    pthread_mutex_lock(&S->mu);
    for (;;) {
        while (S->tail == S->head && !S->quit) pthread_cond_wait(&S->work, &S->mu);
        if (S->tail == S->head) break;
        int fd = S->q[S->tail % ICPU_SYNC_Q].fd;
        unsigned fl = S->q[S->tail % ICPU_SYNC_Q].flags;
        unsigned off = S->q[S->tail % ICPU_SYNC_Q].off, len = S->q[S->tail % ICPU_SYNC_Q].len;
        S->tail++;
        pthread_mutex_unlock(&S->mu);
        double t0 = icpu_now();
        icpu_sync_ioctl(fd, fl, off, len);
        double dt = icpu_now() - t0;
        pthread_mutex_lock(&S->mu);
        if (fl & DMA_BUF_SYNC_END) { S->en_s += dt; S->en_n++; if (dt > S->en_max) S->en_max = dt; }
        else { S->st_s += dt; S->st_n++; if (dt > S->st_max) S->st_max = dt; }
        __atomic_store_n(&S->fin, S->fin + 1, __ATOMIC_RELEASE);
        pthread_cond_broadcast(&S->done);
    }
    pthread_mutex_unlock(&S->mu);
    return NULL;
}

/* on cpu (-1: unpinned); a running helper keeps its CPU */
static inline int icpu_sync_start(icpu_sync *S, int cpu)
{
    if (S->up) return 0;
    pthread_mutex_init(&S->mu, NULL);
    pthread_cond_init(&S->work, NULL);
    pthread_cond_init(&S->done, NULL);
    S->cpu = cpu;
    S->quit = 0;
    if (pthread_create(&S->th, NULL, icpu_sync_thread, S)) return -1;
    S->up = 1;
    return 0;
}

/* queue one op (len 0: the whole buffer, else the range [off, off + len));
 * wait: until it ran (a short spin, then sleep) */
static inline void icpu_sync_op(icpu_sync *S, int fd, unsigned flags, unsigned off, unsigned len, int wait)
{
    pthread_mutex_lock(&S->mu);
    while (S->head - S->fin >= ICPU_SYNC_Q) pthread_cond_wait(&S->done, &S->mu);
    S->q[S->head % ICPU_SYNC_Q].fd = fd;
    S->q[S->head % ICPU_SYNC_Q].flags = flags;
    S->q[S->head % ICPU_SYNC_Q].off = off;
    S->q[S->head % ICPU_SYNC_Q].len = len;
    unsigned my = ++S->head;
    pthread_cond_signal(&S->work);
    pthread_mutex_unlock(&S->mu);
    if (!wait) return;
    double until = icpu_now() + ICPU_SPIN_S;
    while ((int)(__atomic_load_n(&S->fin, __ATOMIC_ACQUIRE) - my) < 0 && icpu_now() < until) icpu_relax();
    if ((int)(__atomic_load_n(&S->fin, __ATOMIC_ACQUIRE) - my) < 0) {
        pthread_mutex_lock(&S->mu);
        while ((int)(S->fin - my) < 0) pthread_cond_wait(&S->done, &S->mu);
        pthread_mutex_unlock(&S->mu);
    }
}

/* every queued op ran (before an fd closes, a mapping goes, an inline START) */
static inline void icpu_sync_flush(icpu_sync *S)
{
    if (!S->up) return;
    pthread_mutex_lock(&S->mu);
    while (S->fin != S->head) pthread_cond_wait(&S->done, &S->mu);
    pthread_mutex_unlock(&S->mu);
}

static inline void icpu_sync_stop(icpu_sync *S)
{
    if (!S->up) return;
    pthread_mutex_lock(&S->mu);
    S->quit = 1;
    pthread_cond_signal(&S->work);
    pthread_mutex_unlock(&S->mu);
    pthread_join(S->th, NULL);
    S->up = 0;
}

#endif
