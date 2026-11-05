/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-dzblock-test: the block dizzy (epd-dizzy.h epd_dizzy_block, the GPU's
 * form) against the layered and serial dizzy, blue noise and
 * Bayer, by numbers (no GPU, no python):
 * - flat fields (256 x 256, levels 1..29): the tone (mean level error), the
 * low-frequency share of the power spectrum against white noise (1 =
 * white, blue noise ~0.05), the energy at the 8-px grid harmonics against
 * the ring at the same radius (the block seams' signature; Bayer is the
 * positive control at ~100s), the axis / diagonal anisotropy near the
 * principal frequency (worms)
 * - a made-up page (gradient + noise, text-like bars with soft edges, grey
 * patches): the Gaussian-blurred (sigma 1.5) absolute error against the
 * input, in levels, over the page and over its grey pixels only
 * - block: deterministic, every output 0 / 30 (fixed kept), tone within
 * 1.25 levels per band (epd-dizzy-test's bound)
 * epd-dzblock-test [OUTDIR] (OUTDIR: the page's dithers as .pgm)
 * exit 0 = the checks pass; the metrics are printed for the eye */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "../epd-dither.h"
#include "../epd-dizzy.h"

enum { W = 1872, H = 1404, N = 256, NM = 13 };
static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; fprintf(stderr, "FAIL: " __VA_ARGS__); fputc('\n', stderr); } } while (0)
static double now_ms(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec * 1e3 + t.tv_nsec / 1e6; }

/* ---- the dithers, all on levels 0..30 | FIXED in place over a W x H panel buffer at (x0, y0) */
typedef struct { const char *name; int kind; epd_dz_btabs T; } method;
enum { M_LAYERED, M_SERIAL, M_BLOCK, M_BLUE, M_BAYER };
static void run_method(const method *m, uint8_t *t, int w, int h, int stride, int x0, int y0)
{
    switch (m->kind) {
    case M_LAYERED: epd_dizzy(t, w, h, stride, x0, y0); break;
    case M_SERIAL: epd_dizzy_serial(t, w, h, stride, x0, y0); break;
    case M_BLOCK: epd_dizzy_block(t, w, h, stride, x0, y0, &m->T); break;
    default: {
        static uint8_t th[64][64];
        epd_dither_table(th, m->kind == M_BLUE ? EPD_DITHER_BLUE : EPD_DITHER_BAYER);
        for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++) {
                uint8_t *p = t + (size_t)y * stride + x;
                if (!(*p & EPD_DZ_FIXED)) *p = *p * 255 / 30 > th[(y0 + y) & 63][(x0 + x) & 63] ? 30 : 0;
            }
    }
    }
}

/* ---- a 2D FFT of an N x N real field (radix 2), its power spectrum P[v][u], u, v in -N/2 .. N/2 - 1 */
static void fft1(double *re, double *im, int n)
{
    for (int i = 1, j = 0; i < n; i++) {        /* bit reversal */
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) { double t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t; }
    }
    for (int len = 2; len <= n; len <<= 1) {
        double a = -2 * M_PI / len, wr = cos(a), wi = sin(a);
        for (int i = 0; i < n; i += len) {
            double cr = 1, ci = 0;
            for (int k = 0; k < len / 2; k++) {
                double ur = re[i + k], ui = im[i + k], vr = re[i + k + len / 2] * cr - im[i + k + len / 2] * ci,
                       vi = re[i + k + len / 2] * ci + im[i + k + len / 2] * cr;
                re[i + k] = ur + vr; im[i + k] = ui + vi; re[i + k + len / 2] = ur - vr; im[i + k + len / 2] = ui - vi;
                double t = cr * wr - ci * wi; ci = cr * wi + ci * wr; cr = t;
            }
        }
    }
}
static void power2d(const double *f, double *P)   /* P centred: P[(v + N/2) * N + (u + N/2)] */
{
    static double re[N * N], im[N * N], r[N], i_[N];
    memcpy(re, f, sizeof re);
    memset(im, 0, sizeof im);
    for (int y = 0; y < N; y++) fft1(re + (size_t)y * N, im + (size_t)y * N, N);
    for (int x = 0; x < N; x++) {
        for (int y = 0; y < N; y++) { r[y] = re[(size_t)y * N + x]; i_[y] = im[(size_t)y * N + x]; }
        fft1(r, i_, N);
        for (int y = 0; y < N; y++) { re[(size_t)y * N + x] = r[y]; im[(size_t)y * N + x] = i_[y]; }
    }
    for (int v = 0; v < N; v++)
        for (int u = 0; u < N; u++) {
            size_t s = (size_t)v * N + u;
            P[(size_t)((v + N / 2) % N) * N + (u + N / 2) % N] = re[s] * re[s] + im[s] * im[s];
        }
}

/* the flat-field metrics of one dither at level g */
typedef struct { double tone, lf, grid, aniso; } flat_m;
static flat_m flat(const method *m, int g)
{
    static uint8_t t[N * N];
    static double f[N * N], P[N * N];
    memset(t, g, sizeof t);
    run_method(m, t, N, N, N, 0, 0);
    double mean = 0;
    for (size_t i = 0; i < sizeof t; i++) mean += t[i];
    mean /= N * N;
    for (size_t i = 0; i < sizeof t; i++) f[i] = t[i] - mean;           /* (AC only) */
    power2d(f, P);
    double d = mean / 30, fp = sqrt(d < 0.5 ? d : 1 - d) * N;            /* principal frequency, in bins */
    double tot = 0, lo = 0, ax = 0, dg = 0, grid = 0, ring = 0;
    long nt = 0, nl = 0, nax = 0, ndg = 0, ng = 0, nr = 0;
    for (int v = -N / 2; v < N / 2; v++)
        for (int u = -N / 2; u < N / 2; u++) {
            if (!u && !v) continue;
            double p = P[(size_t)(v + N / 2) * N + u + N / 2], r = sqrt((double)u * u + (double)v * v);
            tot += p; nt++;
            if (r < fp / 2) { lo += p; nl++; }
            if (r > fp * 0.7 && r < fp * 1.3) {                           /* the annulus around fp */
                double a = fabs(atan2(fabs((double)v), fabs((double)u)));  /* 0 .. pi/2 */
                if (a < M_PI / 18 || a > M_PI / 2 - M_PI / 18) { ax += p; nax++; }
                else if (fabs(a - M_PI / 4) < M_PI / 18) { dg += p; ndg++; }
            }
            for (int k = 1; k <= 3; k++) {                                /* the 8-px grid's harmonics */
                int h = k * N / 8;
                int on = (abs(u) == h && v == 0) || (abs(v) == h && u == 0);
                if (on) { grid += p; ng++; }
                else if (fabs(r - h) < 0.5) { ring += p; nr++; }
            }
        }
    flat_m r = { mean - g, (lo / nl) / (tot / nt), (grid / ng) / (ring / nr), (ax / nax) / (dg / ndg) };
    return r;
}

/* the page: left a soft gradient with noise, right text-like bars (black, with
 * 2-px grey edges) on white, grey patches, a mid-grey band */
static void page(uint8_t *in)
{
    srand(11);
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            int v;
            if (x < W / 2) v = (x * 30 / (W / 2) + y * 7 / H + rand() % 5 - 2);
            else if (y < H / 3) {                       /* text-like bars, soft edges */
                int bar = (x / 7) % 3 == 0 && (y / 11) % 2 == 0;
                int edge = ((x + 1) / 7) % 3 == 0 || ((x - 1) / 7) % 3 == 0;
                v = bar ? 0 : edge && (y / 11) % 2 == 0 ? 14 : 30;
            } else if (y < 2 * H / 3) v = ((x - W / 2) / 117) * 2 + 1;   /* patches 1 .. 29 */
            else v = 15;
            v = v < 0 ? 0 : v > 30 ? 30 : v;
            in[(size_t)y * W + x] = (uint8_t)(v | (v == 0 || v == 30 ? EPD_DZ_FIXED : 0));
        }
}
static void put(const char *dir, const char *name, const uint8_t *lv)
{
    if (!dir) return;
    char p[512];
    snprintf(p, sizeof p, "%s/%s.pgm", dir, name);
    FILE *f = fopen(p, "wb");
    if (!f) { perror(p); return; }
    fprintf(f, "P5\n%d %d\n255\n", W, H);
    for (size_t i = 0; i < (size_t)W * H; i++) fputc((lv[i] & 31) * 255 / 30, f);
    fclose(f);
}
/* the Gaussian-blurred absolute error, sigma 1.5 (a 7 x 7 kernel), in levels: whole page / grey pixels */
static void blurred_err(const uint8_t *in, const uint8_t *out, double *all, double *grey)
{
    static double k[7], e[W * H], tmp[W * H];
    double s = 0;
    for (int i = 0; i < 7; i++) { k[i] = exp(-(i - 3) * (i - 3) / (2 * 1.5 * 1.5)); s += k[i]; }
    for (int i = 0; i < 7; i++) k[i] /= s;
    for (size_t i = 0; i < (size_t)W * H; i++) e[i] = (double)(out[i] & 31) - (in[i] & 31);
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            double a = 0;
            for (int i = -3; i <= 3; i++) { int xx = x + i < 0 ? 0 : x + i >= W ? W - 1 : x + i; a += k[i + 3] * e[(size_t)y * W + xx]; }
            tmp[(size_t)y * W + x] = a;
        }
    double sa = 0, sg = 0;
    long ng = 0;
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            double a = 0;
            for (int i = -3; i <= 3; i++) { int yy = y + i < 0 ? 0 : y + i >= H ? H - 1 : y + i; a += k[i + 3] * tmp[(size_t)yy * W + x]; }
            sa += fabs(a);
            if (!(in[(size_t)y * W + x] & EPD_DZ_FIXED)) { sg += fabs(a); ng++; }
        }
    *all = sa / ((double)W * H);
    *grey = ng ? sg / ng : 0;
}

int main(int argc, char **argv)
{
    const char *dir = argc > 1 ? argv[1] : NULL;
    /* the block forms: WxH-N-P = block, orders, passes */
    method M[NM] = { { "layered", M_LAYERED }, { "serial", M_SERIAL }, { "blk8x8-8-1", M_BLOCK }, { "blk8x8-8-2", M_BLOCK },
                     { "blk8x8-8-4", M_BLOCK }, { "blk8x8-64-1", M_BLOCK }, { "blk8x4-8-4", M_BLOCK }, { "blk8x8-16-4", M_BLOCK },
                     { "blk8x8-64-4", M_BLOCK }, { "blk8x8-16s4-4", M_BLOCK }, { "blk8x8-32s2-4", M_BLOCK },
                     { "blue", M_BLUE }, { "bayer", M_BAYER } };
    CHECK(!epd_dz_btabs_init(&M[2].T, 8, 8, 8, 1, 0, 1), "tables 8x8 1 pass");
    CHECK(!epd_dz_btabs_init(&M[3].T, 8, 8, 8, 2, 0, 1), "tables 8x8 2 passes");
    CHECK(!epd_dz_btabs_init(&M[4].T, 8, 8, 8, 4, 0, 1), "tables 8x8 4 passes");
    CHECK(!epd_dz_btabs_init(&M[5].T, 8, 8, 64, 1, 0, 1), "tables 8x8 64 orders");
    CHECK(!epd_dz_btabs_init(&M[6].T, 8, 4, 8, 4, 0, 1), "tables 8x4");
    CHECK(!epd_dz_btabs_init(&M[7].T, 8, 8, 16, 4, 0, 1), "tables 8x8 16 orders");
    CHECK(epd_dz_btabs_init(&M[7].T, 4, 4, 8, 1, 0, 1) < 0 && epd_dz_btabs_init(&M[7].T, 8, 8, 3, 1, 0, 1) < 0 &&
          epd_dz_btabs_init(&M[7].T, 8, 8, 8, 3, 0, 1) < 0, "bad shapes refused");
    epd_dz_btabs_init(&M[7].T, 8, 8, 16, 4, 0, 1);
    CHECK(!epd_dz_btabs_init(&M[8].T, 8, 8, 64, 4, 0, 1), "tables 64 orders 4 passes");
    CHECK(!epd_dz_btabs_init(&M[9].T, 8, 8, 64, 4, 0, 4), "tables 16 x 4 symmetries 4 passes");
    CHECK(!epd_dz_btabs_init(&M[10].T, 8, 8, 64, 4, 0, 2), "tables 32 x 2 symmetries 4 passes");
    CHECK(epd_dz_btabs_init(&M[10].T, 8, 4, 64, 2, 0, 8) < 0 && epd_dz_btabs_init(&M[10].T, 8, 8, 8, 2, 0, 3) < 0, "bad symmetries refused");
    epd_dz_btabs_init(&M[10].T, 8, 8, 64, 4, 0, 2);
    CHECK(epd_dz_btabs_get()->passes == 4 && epd_dz_btabs_get()->n == 64 && epd_dz_btabs_get()->nsym == 4 && epd_dz_btabs_get()->npx == 64, "the defaults");
    epd_dizzy_init();
    static const int G[] = { 1, 2, 4, 8, 15, 22, 26, 28, 29 };
    printf("flat fields %dx%d: tone = mean level error; lf = low-frequency power vs white (1); grid = power at the\n"
           "8-px harmonics vs their ring (1 = none); aniso = axis vs diagonal power near fp (1 = isotropic)\n", N, N);
    printf("%-11s", "level");
    for (unsigned i = 0; i < sizeof G / sizeof *G; i++) printf("  %13d", G[i]);
    printf("\n");
    for (int mi = 0; mi < NM; mi++) {
        flat_m r[sizeof G / sizeof *G];
        for (unsigned i = 0; i < sizeof G / sizeof *G; i++) r[i] = flat(&M[mi], G[i]);
        printf("%-11s", M[mi].name);
        for (unsigned i = 0; i < sizeof G / sizeof *G; i++) printf("  t%+5.2f lf%4.2f", r[i].tone, r[i].lf);
        printf("\n%-11s", "");
        for (unsigned i = 0; i < sizeof G / sizeof *G; i++) printf("  g%5.1f  a%4.2f", r[i].grid, r[i].aniso);
        printf("\n");
        if (M[mi].kind == M_BLOCK)
            for (unsigned i = 0; i < sizeof G / sizeof *G; i++) CHECK(fabs(r[i].tone) < 1.25, "%s level %d: tone off by %.2f", M[mi].name, G[i], r[i].tone);
    }
    /* the page */
    static uint8_t in[W * H], out[W * H], out2[W * H];
    page(in);
    put(dir, "page", in);
    printf("\npage: blurred |error| (sigma 1.5), levels: all / grey pixels; the full page's time (one core, host)\n");
    for (int mi = 0; mi < NM; mi++) {
        memcpy(out, in, sizeof out);
        double t0 = now_ms();
        run_method(&M[mi], out, W, H, W, 0, 0);
        double ms = now_ms() - t0, a, g;
        blurred_err(in, out, &a, &g);
        printf("  %-11s all %.3f grey %.3f  (%.1f ms)\n", M[mi].name, a, g, ms);
        put(dir, M[mi].name, out);
        if (M[mi].kind != M_BLOCK) continue;
        for (size_t i = 0; i < sizeof out; i++) {
            if (in[i] & EPD_DZ_FIXED) { if (out[i] != in[i]) { CHECK(0, "%s: a fixed pixel changed", M[mi].name); break; } }
            else if (out[i] != 0 && out[i] != 30) { CHECK(0, "%s: level %d out", M[mi].name, out[i]); break; }
        }
        memcpy(out2, in, sizeof out2);
        run_method(&M[mi], out2, W, H, W, 0, 0);
        CHECK(!memcmp(out, out2, sizeof out), "%s: not deterministic", M[mi].name);
        /* the gradient half's tone per 117-px band */
        for (int b = 0; b < 8; b++) {
            double si = 0, so = 0;
            long n = 0;
            for (int y = 0; y < H; y++)
                for (int x = b * 117; x < (b + 1) * 117; x++) { si += in[(size_t)y * W + x] & 31; so += out[(size_t)y * W + x] & 31; n++; }
            CHECK(fabs(si - so) / n < 1.25, "%s: band %d mean %.2f -> %.2f", M[mi].name, b, si / n, so / n);
        }
        /* an odd rect at an odd place: nothing outside it changes, the inside is 0 / 30 */
        memcpy(out, in, sizeof out);
        epd_dizzy_block(out + (size_t)701 * W + 333, 517, 203, W, 333, 701, &M[mi].T);
        for (size_t i = 0; i < sizeof out; i++) {
            int x = (int)(i % W), y = (int)(i / W), inr = x >= 333 && x < 850 && y >= 701 && y < 904;
            if (!inr && out[i] != in[i]) { CHECK(0, "%s: a pixel outside the rect changed", M[mi].name); break; }
            if (inr && !(in[i] & EPD_DZ_FIXED) && out[i] != 0 && out[i] != 30) { CHECK(0, "%s: rect: level %d", M[mi].name, out[i]); break; }
        }
    }
    printf(fails ? "epd-dzblock: FAILED\n" : "epd-dzblock: ok\n");
    return fails ? 1 : 0;
}
