/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-grey-test — the grey value q (epd-grey.h): luma and luminance
 * the S table against the sRGB decode, epd_grey_isqrt exact, every colour's
 * luminance q against a double reference, fixed colours against the numpy
 * prototype, the gamma LUTs
 * (greys keep their luma levels, monotonic), luma unchanged, and on arm64
 * the NEON rect against the scalar one, byte for byte, both formulas.
 * cc -O2 -Wall -Wextra -o /tmp/epd-grey-test epd-grey-test.c -lm */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../epd-grey.h"

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL: " __VA_ARGS__); printf("\n"); } } while (0)

static double srgb_lin(int c)
{
    double v = c / 255.0;
    return v <= 0.04045 ? v / 12.92 : pow((v + 0.055) / 1.055, 2.4);
}

/* round(sqrt(y)) in double: exact for these y (no y is a half-integer's square) */
static uint32_t ref_isqrt(uint32_t y) { return (uint32_t)floor(sqrt((double)y) + 0.5); }

/* fixed colours and the prototype's q (approx2.py q_of_s) */
static const uint8_t col[][3] = { { 0, 0, 0 }, { 255, 255, 255 }, { 255, 0, 0 }, { 0, 255, 0 }, { 0, 0, 255 }, { 255, 255, 0 }, { 53, 132, 228 }, { 224, 27, 36 }, { 46, 194, 126 }, { 145, 65, 172 }, { 200, 150, 120 }, { 128, 128, 128 }, { 1, 2, 3 }, { 250, 1, 128 }, { 17, 200, 9 }, { 99, 99, 100 } };
static const uint8_t col_q[] = { 0, 255, 118, 216, 69, 246, 122, 104, 162, 91, 152, 118, 6, 119, 164, 90 };

int main(void)
{
    /* S */
    for (int c = 0; c < 256; c++)
        CHECK(epd_grey_sq[c] == (uint8_t)lround(sqrt(65025.0 * srgb_lin(c))), "S[%d] = %d", c, epd_grey_sq[c]);
    /* isqrt over Y's range (and well past it) */
    for (uint32_t y = 0; y <= (1u << 20); y++)
        if (epd_grey_isqrt(y) != ref_isqrt(y)) { CHECK(0, "isqrt(%u) = %u, want %u", y, epd_grey_isqrt(y), ref_isqrt(y)); break; }
    /* every colour: no overflow (Y <= 65025), q = round(sqrt(Y)) */
    long bad = 0;
    for (uint32_t p = 0; p < (1u << 24); p++) {
        uint32_t r = p >> 16, g = (p >> 8) & 0xff, b = p & 0xff;
        uint64_t y = ((uint64_t)EPD_GREY_WR * epd_grey_sq[r] * epd_grey_sq[r] + (uint64_t)EPD_GREY_WG * epd_grey_sq[g] * epd_grey_sq[g]
                      + (uint64_t)EPD_GREY_WB * epd_grey_sq[b] * epd_grey_sq[b]) >> 15;
        if (y > 65025 || epd_grey_q_lum(r, g, b) != ref_isqrt((uint32_t)y)) bad++;
    }
    CHECK(bad == 0, "%ld colours' luminance q off", bad);
    for (size_t i = 0; i < sizeof col_q; i++)
        CHECK(epd_grey_q_lum(col[i][0], col[i][1], col[i][2]) == col_q[i], "q(%d,%d,%d) = %u, want %d", col[i][0], col[i][1],
              col[i][2], epd_grey_q_lum(col[i][0], col[i][1], col[i][2]), col_q[i]);
    /* LUTs: greys keep luma's level (all but a few, 1 apart), ends fixed, monotonic */
    const double gms[] = { 1.0, 1.6, 1.8 };
    for (int k = 0; k < 3; k++) {
        uint8_t la[256], lb[256];
        epd_grey_luma_lut(la, gms[k]);
        epd_grey_lum_lut(lb, gms[k]);
        int same = 0, worst = 0, mono = 1;
        for (int v = 0; v < 256; v++) {
            int d = abs(lb[epd_grey_q_lum(v, v, v)] - la[(v * 77 + v * 150 + v * 29) >> 8]);
            same += d == 0;
            if (d > worst) worst = d;
            if (v && lb[v] < lb[v - 1]) mono = 0;
        }
        CHECK(same >= 240 && worst <= 1, "gamma %.1f: greys equal %d/256, worst %d", gms[k], same, worst);
        CHECK(mono && lb[0] == 0 && lb[255] == 255, "gamma %.1f: LUT not monotonic or ends moved", gms[k]);
        printf("gamma %.1f: greys keep their level %d/256 (else 1 apart)\n", gms[k], same);
        if (gms[k] == 1.6) {   /* the levels of research doc's table (level = grey >> 4) */
            CHECK(lb[epd_grey_q_lum(0, 255, 0)] >> 4 == 12 && lb[epd_grey_q_lum(255, 0, 0)] >> 4 == 5
                  && la[(150 * 255) >> 8] >> 4 == 6 && la[(77 * 255) >> 8] >> 4 == 2, "gamma 1.6: green/red levels");
        }
    }
    /* luma: the old formula */
    uint8_t id[256];
    for (int i = 0; i < 256; i++) id[i] = (uint8_t)i;
    for (uint32_t p = 0; p < (1u << 24); p += 257)
        if (epd_grey_px(id, p) != (((p >> 16) & 0xff) * 77 + ((p >> 8) & 0xff) * 150 + (p & 0xff) * 29) >> 8) { CHECK(0, "luma %06x", p); break; }
    /* the rect: NEON (arm64) vs scalar, odd sizes, both formulas */
    enum { PW = 64, PH = 80, W = 61, H = 75 };
    uint32_t *px = malloc(sizeof(uint32_t) * W * H);
    uint8_t *a = malloc(PW * PH), *b = malloc(PW * PH), lut[256];
    uint32_t s = 12345;
    for (int i = 0; i < W * H; i++) { s = s * 1664525u + 1013904223u; px[i] = s ^ (s >> 13); }
    for (int i = 0; i < 64; i++) px[i] = 0x01010101u * (uint32_t)(i * 4);   /* some greys */
    for (int lum = 0; lum < 2; lum++) {
        (lum ? epd_grey_lum_lut : epd_grey_luma_lut)(lut, 1.6);
        memset(a, 7, PW * PH);
        memset(b, 7, PW * PH);
        epd_grey_rect_scalar_mode(lut, lum, a, PW, px, W, 1, 2, W, H);
        epd_grey_rect_mode(lut, lum, b, PW, px, W, 1, 2, W, H);
        CHECK(!memcmp(a, b, PW * PH), "rect %s: NEON != scalar", lum ? "luminance" : "luma");
        for (int band = 8; band <= 64; band *= 8) {
            memset(b, 7, PW * PH);
            epd_grey_rect_band_mode(lut, lum, b, PW, px, W, 1, 2, W, H, band);
            CHECK(!memcmp(a, b, PW * PH), "rect %s band %d: != scalar", lum ? "luminance" : "luma", band);
        }
    }
#ifdef __aarch64__
    printf("NEON vs scalar: checked\n");
#else
    printf("NEON vs scalar: not this host (arm64 only)\n");
#endif
    free(px); free(a); free(b);
    printf(fails ? "epd-grey: %d FAILED\n" : "epd-grey: ok\n", fails);
    return fails != 0;
}
