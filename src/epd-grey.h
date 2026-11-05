/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-grey — sway's XRGB8888 -> panel-order grey (the grey value q, then the
 * gamma LUT, rotated: portrait row py -> panel column pw-1-py, pixel x ->
 * panel row x), glassine-epdd-desktop's wl convert (40 % of its CPU in the
 * device profile).
 *
 * Two formulas for q (glassine-epdd-desktop grey=,:
 * luma (77 R + 150 G + 29 B) >> 8 on the sRGB values: exact for greys,
 * but saturated colours far too dark (pure green 6 levels)
 * luminance (the default, user ) true luminance in the square-
 * root domain: S[c] = round(sqrt(65025 x the sRGB decode of c))
 * (a byte table: NEON tbl), Y = (6966 S[R]^2 + 23436 S[G]^2 +
 * 2366 S[B]^2) >> 15 (BT.709 weights x 32768), q = round(sqrt(Y))
 * with exact integer rounding (epd_grey_isqrt). epd_grey_lum_lut
 * folds the sRGB encode into the gamma LUT. Against the exact
 * float luminance: within 2 grey values, unbiased (2.7 % of
 * random colours one panel level off, at level edges).
 * Every path computes the same q, byte for byte: this scalar form, the NEON
 * one, epd-ingest's GPU shader (same table, same integer steps).
 * arm64: NEON in 8 x 8 blocks, banded (epd_grey_rect_band) (vld4 deinterleave, widening multiply-add, the
 * LUT (and luminance's S) as four 64-byte tbl/tbx tables, an 8 x 8 byte
 * transpose), the edges scalar; same bytes as the scalar form
 * (test/wl-convert-bench, test/epd-grey-test). */
#ifndef EPD_GREY_H
#define EPD_GREY_H

#include <stddef.h>
#include <math.h>
#include <stdint.h>
#ifdef __aarch64__
#include <arm_neon.h>
#endif

/* S[c] = round(sqrt(65025 x the sRGB decode of c)): c in the square-root domain */
static const uint8_t epd_grey_sq[256] = {
      0,   4,   6,   8,   9,  10,  11,  12,  13,  13,  14,  15,  15,  16,  17,  18,
     18,  19,  20,  21,  21,  22,  23,  24,  24,  25,  26,  27,  27,  28,  29,  30,
     31,  31,  32,  33,  34,  35,  36,  36,  37,  38,  39,  40,  40,  41,  42,  43,
     44,  45,  46,  46,  47,  48,  49,  50,  51,  52,  52,  53,  54,  55,  56,  57,
     58,  59,  60,  60,  61,  62,  63,  64,  65,  66,  67,  68,  69,  69,  70,  71,
     72,  73,  74,  75,  76,  77,  78,  79,  80,  81,  82,  82,  83,  84,  85,  86,
     87,  88,  89,  90,  91,  92,  93,  94,  95,  96,  97,  98,  99, 100, 101, 102,
    103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 112, 113, 114, 115, 116, 117,
    118, 119, 120, 121, 122, 123, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134,
    135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150,
    151, 152, 153, 154, 155, 156, 157, 159, 160, 161, 162, 163, 164, 165, 166, 167,
    168, 169, 170, 171, 172, 173, 174, 175, 177, 178, 179, 180, 181, 182, 183, 184,
    185, 186, 187, 188, 189, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201,
    203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 214, 215, 216, 217, 218, 219,
    220, 221, 222, 223, 225, 226, 227, 228, 229, 230, 231, 232, 234, 235, 236, 237,
    238, 239, 240, 241, 243, 244, 245, 246, 247, 248, 249, 250, 252, 253, 254, 255
};
#define EPD_GREY_WR 6966u       /* BT.709 x 32768 (sum 32768) */
#define EPD_GREY_WG 23436u
#define EPD_GREY_WB 2366u

/* round(sqrt(y)), exact: q is the nearest integer iff q^2 - q < y <= q^2 + q
 * (sqrt(y) = k + 1/2 has no integer y); the float sqrt only starts it */
static inline uint32_t epd_grey_isqrt(uint32_t y)
{
    uint32_t q = (uint32_t)(sqrtf((float)y) + 0.5f);
    if (y > q * q + q) q++;
    else if (q && y <= q * q - q) q--;
    return q;
}

static inline uint32_t epd_grey_q_lum(uint32_t r, uint32_t g, uint32_t b)
{
    uint32_t sr = epd_grey_sq[r], sg = epd_grey_sq[g], sb = epd_grey_sq[b];
    return epd_grey_isqrt((EPD_GREY_WR * sr * sr + EPD_GREY_WG * sg * sg + EPD_GREY_WB * sb * sb) >> 15);
}

static inline uint8_t epd_grey_px_mode(const uint8_t *lut, int lum, uint32_t p)
{
    uint32_t r = (p >> 16) & 0xff, g = (p >> 8) & 0xff, b = p & 0xff;
    return lut[lum ? epd_grey_q_lum(r, g, b) : (r * 77 + g * 150 + b * 29) >> 8];
}

static inline uint8_t epd_grey_px(const uint8_t *lut, uint32_t p) { return epd_grey_px_mode(lut, 0, p); }

/* the gamma LUTs: luma: out = 255 (in/255)^g; luminance: out = 255 (E(q)/255)^g,
 * E = the sRGB encode of (q/255)^2, except that every q a neutral grey v
 * lands on gets luma's value for v (greys keep their levels: 249 of 256
 * exactly at 1.6, the rest 1 apart where two greys share a q) */
static inline void epd_grey_luma_lut(uint8_t lut[256], double gm)
{
    for (int i = 0; i < 256; i++) lut[i] = (uint8_t)lround(255.0 * pow(i / 255.0, gm));
}

static inline void epd_grey_lum_lut(uint8_t lut[256], double gm)
{
    int sum[256] = { 0 }, n[256] = { 0 };
    for (int v = 0; v < 256; v++) {
        uint32_t q = epd_grey_q_lum(v, v, v);
        sum[q] += (int)lround(255.0 * pow(v / 255.0, gm));
        n[q]++;
    }
    for (int q = 0; q < 256; q++) {
        if (n[q]) { lut[q] = (uint8_t)((2 * sum[q] + n[q]) / (2 * n[q])); continue; }
        double y = (q / 255.0) * (q / 255.0);
        double e = y <= 0.0031308 ? 12.92 * y : 1.055 * pow(y, 1 / 2.4) - 0.055;
        lut[q] = (uint8_t)lround(255.0 * pow(e, gm));
    }
}

static inline void epd_grey_rect_scalar_mode(const uint8_t *lut, int lum, uint8_t *dst, int pw, const uint32_t *px,
                                             int stride, int x, int y, int w, int h)
{
    for (int py = y; py < y + h; py++) {
        const uint32_t *row = px + (size_t)(py - y) * stride;
        uint8_t *d = dst + (pw - 1 - py);
        for (int i = 0; i < w; i++) d[(size_t)(x + i) * pw] = epd_grey_px_mode(lut, lum, row[i]);
    }
}

static inline void epd_grey_rect_scalar(const uint8_t *lut, uint8_t *dst, int pw, const uint32_t *px, int stride,
                                        int x, int y, int w, int h)
{
    epd_grey_rect_scalar_mode(lut, 0, dst, pw, px, stride, x, y, w, h);
}

#ifdef __aarch64__
/* a 256-byte table as four 64-byte tbl/tbx tables: tbl: out of range -> 0,
 * tbx: out of range -> keep; i - 64 etc. wrap */
static inline uint8x8_t epd_grey_tbl8(const uint8x16x4_t *t, uint8x8_t i)
{
    uint8x8_t g = vqtbl4_u8(t[0], i);
    g = vqtbx4_u8(g, t[1], vsub_u8(i, vdup_n_u8(64)));
    g = vqtbx4_u8(g, t[2], vsub_u8(i, vdup_n_u8(128)));
    return vqtbx4_u8(g, t[3], vsub_u8(i, vdup_n_u8(192)));
}

/* epd_grey_isqrt on 4 lanes (the same float start, the same corrections) */
static inline uint32x4_t epd_grey_isqrt4(uint32x4_t y)
{
    uint32x4_t q = vcvtq_u32_f32(vaddq_f32(vsqrtq_f32(vcvtq_f32_u32(y)), vdupq_n_f32(0.5f)));
    uint32x4_t q2 = vmulq_u32(q, q);
    uint32x4_t up = vcgtq_u32(y, vaddq_u32(q2, q));                       /* y > q^2 + q: q + 1 */
    uint32x4_t dn = vbicq_u32(vandq_u32(vcleq_u32(y, vsubq_u32(q2, q)), vtstq_u32(q, q)), up);  /* else q && y <= q^2 - q: q - 1 */
    return vaddq_u32(vsubq_u32(q, up), dn);                               /* (masks are -1) */
}

/* 8 pixels' q: luma, or luminance (sq: epd_grey_sq as tbl tables) */
static inline uint8x8_t epd_grey8_q(const uint8x16x4_t *sq, int lum, uint8x8x4_t c)
{
    if (!lum) {
        uint16x8_t l = vmull_u8(c.val[2], vdup_n_u8(77));
        l = vmlal_u8(l, c.val[1], vdup_n_u8(150));
        l = vmlal_u8(l, c.val[0], vdup_n_u8(29));
        return vshrn_n_u16(l, 8);
    }
    uint8x8_t sr = epd_grey_tbl8(sq, c.val[2]), sg = epd_grey_tbl8(sq, c.val[1]), sb = epd_grey_tbl8(sq, c.val[0]);
    uint16x8_t r2 = vmull_u8(sr, sr), g2 = vmull_u8(sg, sg), b2 = vmull_u8(sb, sb);
    uint32x4_t lo = vmull_n_u16(vget_low_u16(r2), EPD_GREY_WR);
    lo = vmlal_n_u16(lo, vget_low_u16(g2), EPD_GREY_WG);
    lo = vmlal_n_u16(lo, vget_low_u16(b2), EPD_GREY_WB);
    uint32x4_t hi = vmull_n_u16(vget_high_u16(r2), EPD_GREY_WR);
    hi = vmlal_n_u16(hi, vget_high_u16(g2), EPD_GREY_WG);
    hi = vmlal_n_u16(hi, vget_high_u16(b2), EPD_GREY_WB);
    uint32x4_t ql = epd_grey_isqrt4(vshrq_n_u32(lo, 15)), qh = epd_grey_isqrt4(vshrq_n_u32(hi, 15));
    return vmovn_u16(vcombine_u16(vmovn_u32(ql), vmovn_u32(qh)));
}

static inline uint8x8_t epd_grey8_mode(const uint8x16x4_t *t, const uint8x16x4_t *sq, int lum, const uint32_t *p)
{
    return epd_grey_tbl8(t, epd_grey8_q(sq, lum, vld4_u8((const uint8_t *)p)));    /* B G R X */
}

static inline uint8x8_t epd_grey8(const uint8x16x4_t *t, const uint32_t *p) { return epd_grey8_mode(t, NULL, 0, p); }
#endif

#ifdef __aarch64__
/* one 8 x 8 block: source rows r[0..7] (8 pixels each) -> 8 panel rows of
 * 8 bytes at o, o + pw, ... (the last source row first) */
static inline void epd_grey_block8(const uint8x16x4_t *t, const uint8x16x4_t *sq, int lum, const uint32_t *r0,
                                   int stride, uint8_t *o, int pw)
{
    uint8x8_t a[8];
    for (int k = 0; k < 8; k++) a[k] = epd_grey8_mode(t, sq, lum, r0 + (size_t)(7 - k) * stride);
    uint8x8x2_t t01 = vtrn_u8(a[0], a[1]), t23 = vtrn_u8(a[2], a[3]);
    uint8x8x2_t t45 = vtrn_u8(a[4], a[5]), t67 = vtrn_u8(a[6], a[7]);
    uint16x4x2_t s0 = vtrn_u16(vreinterpret_u16_u8(t01.val[0]), vreinterpret_u16_u8(t23.val[0]));
    uint16x4x2_t s1 = vtrn_u16(vreinterpret_u16_u8(t01.val[1]), vreinterpret_u16_u8(t23.val[1]));
    uint16x4x2_t s2 = vtrn_u16(vreinterpret_u16_u8(t45.val[0]), vreinterpret_u16_u8(t67.val[0]));
    uint16x4x2_t s3 = vtrn_u16(vreinterpret_u16_u8(t45.val[1]), vreinterpret_u16_u8(t67.val[1]));
    uint32x2x2_t q0 = vtrn_u32(vreinterpret_u32_u16(s0.val[0]), vreinterpret_u32_u16(s2.val[0]));
    uint32x2x2_t q1 = vtrn_u32(vreinterpret_u32_u16(s1.val[0]), vreinterpret_u32_u16(s3.val[0]));
    uint32x2x2_t q2 = vtrn_u32(vreinterpret_u32_u16(s0.val[1]), vreinterpret_u32_u16(s2.val[1]));
    uint32x2x2_t q3 = vtrn_u32(vreinterpret_u32_u16(s1.val[1]), vreinterpret_u32_u16(s3.val[1]));
    vst1_u8(o + 0 * (size_t)pw, vreinterpret_u8_u32(q0.val[0]));
    vst1_u8(o + 1 * (size_t)pw, vreinterpret_u8_u32(q1.val[0]));
    vst1_u8(o + 2 * (size_t)pw, vreinterpret_u8_u32(q2.val[0]));
    vst1_u8(o + 3 * (size_t)pw, vreinterpret_u8_u32(q3.val[0]));
    vst1_u8(o + 4 * (size_t)pw, vreinterpret_u8_u32(q0.val[1]));
    vst1_u8(o + 5 * (size_t)pw, vreinterpret_u8_u32(q1.val[1]));
    vst1_u8(o + 6 * (size_t)pw, vreinterpret_u8_u32(q2.val[1]));
    vst1_u8(o + 7 * (size_t)pw, vreinterpret_u8_u32(q3.val[1]));
}
#endif

/* band: source rows per pass (a multiple of 8). The rotation turns a source
 * row into a panel column, so a pass over 8 rows wrote 8 bytes into each of
 * up to 1404 panel rows 1872 bytes apart (2.6 MB, partial cache lines, a TLB
 * miss per row): memory-bound on the A53 (NEON 10.1 -> 9.4 ns/px
 * only). A band of 64 fills whole 64-byte lines per panel row. */
static inline void epd_grey_rect_band_mode(const uint8_t *lut, int lum, uint8_t *dst, int pw, const uint32_t *px,
                                           int stride, int x, int y, int w, int h, int band)
{
#ifdef __aarch64__
    uint8x16x4_t t[4], sq[4];
    for (int k = 0; k < 4; k++) t[k] = vld1q_u8_x4(lut + 64 * k);
    if (lum) for (int k = 0; k < 4; k++) sq[k] = vld1q_u8_x4(epd_grey_sq + 64 * k);
    int h8 = h & ~7, w8 = w & ~7;
    for (int by = 0; by < h8; by += band) {
        int bh = h8 - by < band ? h8 - by : band;
        for (int bx = 0; bx < w8; bx += 8)
            for (int sb = 0; sb < bh; sb += 8)
                epd_grey_block8(t, sq, lum, px + (size_t)(by + sb) * stride + bx, stride,
                                dst + (pw - 1 - (y + by + sb + 7)) + (size_t)(x + bx) * pw, pw);
        if (w8 < w) epd_grey_rect_scalar_mode(lut, lum, dst, pw, px + (size_t)by * stride + w8, stride, x + w8, y + by, w - w8, bh);
    }
    if (h8 < h) epd_grey_rect_scalar_mode(lut, lum, dst, pw, px + (size_t)h8 * stride, stride, x, y + h8, w, h - h8);
#else
    (void)band;
    epd_grey_rect_scalar_mode(lut, lum, dst, pw, px, stride, x, y, w, h);
#endif
}

static inline void epd_grey_rect_band(const uint8_t *lut, uint8_t *dst, int pw, const uint32_t *px, int stride,
                                      int x, int y, int w, int h, int band)
{
    epd_grey_rect_band_mode(lut, 0, dst, pw, px, stride, x, y, w, h, band);
}

/* a grey row (sway's grey output: q, panel order) through the gamma LUT */
static inline void epd_grey_lut(const uint8_t *lut, uint8_t *dst, const uint8_t *src, int n)
{
    int i = 0;
#ifdef __aarch64__
    uint8x16x4_t t[4];
    for (int k = 0; k < 4; k++) t[k] = vld1q_u8_x4(lut + 64 * k);
    for (; i + 16 <= n; i += 16) {
        uint8x16_t v = vld1q_u8(src + i);
        uint8x16_t g = vqtbl4q_u8(t[0], v);
        g = vqtbx4q_u8(g, t[1], vsubq_u8(v, vdupq_n_u8(64)));
        g = vqtbx4q_u8(g, t[2], vsubq_u8(v, vdupq_n_u8(128)));
        vst1q_u8(dst + i, vqtbx4q_u8(g, t[3], vsubq_u8(v, vdupq_n_u8(192))));
    }
#endif
    for (; i < n; i++) dst[i] = lut[src[i]];
}

#define EPD_GREY_BAND 8         /* as measured; 64 if the A53 bench (wl-convert-bench bandN) says so */
static inline void epd_grey_rect_mode(const uint8_t *lut, int lum, uint8_t *dst, int pw, const uint32_t *px, int stride,
                                      int x, int y, int w, int h)
{
    epd_grey_rect_band_mode(lut, lum, dst, pw, px, stride, x, y, w, h, EPD_GREY_BAND);
}

static inline void epd_grey_rect(const uint8_t *lut, uint8_t *dst, int pw, const uint32_t *px, int stride,
                                 int x, int y, int w, int h)
{
    epd_grey_rect_mode(lut, 0, dst, pw, px, stride, x, y, w, h);
}

#endif
