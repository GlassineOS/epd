/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-dizzy-test: the dithers side by side on a panel picture
 * epd-dizzy-test IN.pgm|- OUTDIR|- [reps]
 * IN: a 1872 x 1404 P5 (glassine-epdd-desktop's "shot": /tmp/glassine-shot-src.pgm; "-": a
 * made-up page, text-like bars over a photo-like gradient, run.sh); its greys
 * as the panel's levels 0..30, then OUTDIR/{blue,bayer,dizzy,serial}.pgm
 * (0 / 255), the same for a grey ramp (ramp-*.pgm), and the dizzy forms' time
 * a full panel and a 512 x 512 rect (reps runs, the median). Checks:
 * - epd_dizzy / epd_dizzy_serial == their plain forms here (ref_*) bit for
 * bit: the picture, the ramp and odd rects (unaligned, 1 px wide / high)
 * - every output 0 / 30; the mean level kept within 1.25 of the input's
 * (ramp: per column band), layered within 2 (it drops the error of a
 * pixel with no later neighbour: ~1.6 levels near the rails)
 * - the same picture twice: the same dots */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "../epd-dither.h"
#include "../epd-dizzy.h"

enum { W = 1872, H = 1404 };

/* the plain forms (the first implementation, ; layered with the
 * lean share rule since the 2x speed-up) */
static int ref_layered(uint8_t *t, int w, int h, int stride, int x0, int y0)
{
    enum { S = 256, TOP = 30 * S, MID = 15 * S };   /* (its units: 1/256 level, the same share to each) */
    if (w <= 0 || h <= 0) return 0;
    int16_t *acc = calloc((size_t)w * h, sizeof *acc);
    if (!acc) return -1;
    const uint16_t *perm = epd_dizzy_perm();
    const uint8_t (*L)[64] = epd_dizzy_layers();
    int tx0 = x0 >> 6, tx1 = (x0 + w + 63) >> 6, ty0 = y0 >> 6, ty1 = (y0 + h + 63) >> 6;
    for (int k = 0; k < 8; k++)
        for (int ty = ty0; ty < ty1; ty++)
            for (int tx = tx0; tx < tx1; tx++)
                for (int i = k << 9; i < (k + 1) << 9; i++) {
                    int px = (tx << 6) + (perm[i] & 63), py = (ty << 6) + (perm[i] >> 6);
                    int x = px - x0, y = py - y0;
                    if (x < 0 || y < 0 || x >= w || y >= h) continue;
                    size_t o = (size_t)y * w + x;
                    uint8_t *tp = t + (size_t)y * stride + x;
                    if (*tp & EPD_DZ_FIXED) continue;
#define FR(xx, yy) (t[(size_t)(yy) * stride + (xx)] & EPD_DZ_FIXED)
                    int v = *tp * S + acc[o];
                    v = v < 0 ? 0 : v > TOP ? TOP : v;
                    int nv = v >= MID ? TOP : 0, err = v - nv;
                    *tp = nv ? 30 : 0;
                    int r = x + 1 < w && L[py & 63][(px + 1) & 63] > k && !FR(x + 1, y), l = x > 0 && L[py & 63][(px - 1) & 63] > k && !FR(x - 1, y);
                    int d = y + 1 < h && L[(py + 1) & 63][px & 63] > k && !FR(x, y + 1), u = y > 0 && L[(py - 1) & 63][px & 63] > k && !FR(x, y - 1);
#undef FR
                    int n = r + l + d + u;
                    if (!n || !err) continue;
                    int sh = (err * ((32768 + n / 2) / n)) >> 15;
#define DZ(c, oo) if (c) acc[oo] = (int16_t)(acc[oo] + sh);
                    DZ(r, o + 1) DZ(l, o - 1) DZ(d, o + w) DZ(u, o - w)
#undef DZ
                }
    free(acc);
    return 0;
}
static int ref_serial(uint8_t *t, int w, int h, int stride, int x0, int y0)
{
    enum { S = 16, TOP = 30 * S, MID = 15 * S };
    if (w <= 0 || h <= 0) return 0;
    int16_t *v = malloc((size_t)w * h * sizeof *v);
    uint8_t *done = calloc((size_t)w * h, 1);
    if (!v || !done) { free(v); free(done); return -1; }
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            uint8_t c = t[(size_t)y * stride + x];
            v[(size_t)y * w + x] = (int16_t)(c * S);
            if (c & EPD_DZ_FIXED) done[(size_t)y * w + x] = 2;     /* (fixed: done from the start, kept) */
        }
    const uint16_t *perm = epd_dizzy_perm();
    int tx0 = x0 >> 6, tx1 = (x0 + w + 63) >> 6, ty0 = y0 >> 6, ty1 = (y0 + h + 63) >> 6;
    for (int ty = ty0; ty < ty1; ty++)
        for (int tx = tx0; tx < tx1; tx++)
            for (int k = 0; k < 4096; k++) {
                int x = (tx << 6) + (perm[k] & 63) - x0, y = (ty << 6) + (perm[k] >> 6) - y0;
                if (x < 0 || y < 0 || x >= w || y >= h) continue;
                size_t o = (size_t)y * w + x;
                if (done[o] == 2) continue;
                int old = v[o], nv = old >= MID ? TOP : 0, err = old - nv;
                v[o] = (int16_t)nv;
                done[o] = 1;
                int n = (x + 1 < w && !done[o + 1]) + (x > 0 && !done[o - 1]) +
                        (y + 1 < h && !done[o + w]) + (y > 0 && !done[o - w]), m = 0;
                if (!n || !err) continue;
#define DZ(c, oo) if (c && !done[oo]) { int q = v[oo] + err * (m + 1) / n - err * m / n; m++; v[oo] = (int16_t)(q < 0 ? 0 : q > TOP ? TOP : q); }
                DZ(x + 1 < w, o + 1) DZ(x > 0, o - 1) DZ(y + 1 < h, o + w) DZ(y > 0, o - w)
#undef DZ
            }
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            if (done[(size_t)y * w + x] != 2) t[(size_t)y * stride + x] = v[(size_t)y * w + x] ? 30 : 0;
    free(v);
    free(done);
    return 0;
}

static double now_ms(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec * 1e3 + t.tv_nsec / 1e6; }
static int cmpd(const void *a, const void *b) { double x = *(const double *)a, y = *(const double *)b; return x < y ? -1 : x > y; }

static void put(const char *dir, const char *name, const uint8_t *lv)
{
    if (!strcmp(dir, "-")) return;
    char p[512];
    snprintf(p, sizeof p, "%s/%s.pgm", dir, name);
    FILE *f = fopen(p, "wb");
    if (!f) { perror(p); exit(1); }
    fprintf(f, "P5\n%d %d\n255\n", W, H);
    for (size_t i = 0; i < (size_t)W * H; i++) fputc(lv[i] * 255 / 30, f);
    fclose(f);
}

static double mean(const uint8_t *lv, int x0, int x1)
{
    double s = 0;
    for (int y = 0; y < H; y++)
        for (int x = x0; x < x1; x++) s += lv[(size_t)y * W + x];
    return s / ((double)H * (x1 - x0));
}

/* the commit's ordered dither (epd_dither_table): white above the threshold */
static void ordered(uint8_t *lv, int pattern)
{
    static uint8_t th[64][64];
    epd_dither_table(th, pattern);
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            uint8_t *p = lv + (size_t)y * W + x;
            *p = *p * 255 / 30 > th[y & 63][x & 63] ? 30 : 0;
        }
}

static int check(const char *what, const uint8_t *in, const uint8_t *out, int bands, double tol)
{
    int bad = 0;
    for (size_t i = 0; i < (size_t)W * H; i++)
        if (out[i] != 0 && out[i] != 30) { printf("%s: level %d at %zu\n", what, out[i], i); return 1; }
    for (int b = 0; b < bands; b++) {
        int x0 = W * b / bands, x1 = W * (b + 1) / bands;
        double mi = mean(in, x0, x1), mo = mean(out, x0, x1);
        if (mo - mi > tol || mi - mo > tol) { printf("%s: band %d mean %.2f -> %.2f\n", what, b, mi, mo); bad = 1; }
    }
    return bad;
}

typedef int (*dz_fn)(uint8_t *, int, int, int, int, int);

/* fast == plain over rect (x, y, w, h) of in */
static int same(const char *what, dz_fn fast, dz_fn ref, const uint8_t *in, int x, int y, int w, int h)
{
    static uint8_t a[W * H], b[W * H];
    memcpy(a, in, sizeof a);
    memcpy(b, in, sizeof b);
    fast(a + (size_t)y * W + x, w, h, W, x, y);
    ref(b + (size_t)y * W + x, w, h, W, x, y);
    if (!memcmp(a, b, sizeof a)) return 0;
    long n = 0;
    for (size_t i = 0; i < sizeof a; i++) n += a[i] != b[i];
    printf("%s: rect %d,%d %dx%d: %ld pixels differ from the plain form\n", what, x, y, w, h, n);
    return 1;
}

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage: %s IN.pgm OUTDIR [reps]\n", argv[0]); return 2; }
    int reps = argc > 3 ? atoi(argv[3]) : 5, w, h, mx;
    uint8_t *src = malloc((size_t)W * H), *a = malloc((size_t)W * H), *ramp = malloc((size_t)W * H);
    if (!strcmp(argv[1], "-")) {
        srand(5);
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++) {
                int g = (x * 255 / W + y * 128 / H) & 255;                  /* the photo */
                if (y % 40 < 16 && (x / 11) % 7 != 0 && x < W / 2) g = rand() % 3 ? 0 : 255;   /* text */
                src[(size_t)y * W + x] = (uint8_t)g;
            }
    } else {
        FILE *f = fopen(argv[1], "rb");
        if (!f || fscanf(f, "P5 %d %d %d", &w, &h, &mx) != 3 || w != W || h != H || mx != 255) { fprintf(stderr, "need a %dx%d P5\n", W, H); return 2; }
        fgetc(f);
        if (fread(src, 1, (size_t)W * H, f) != (size_t)W * H) { fprintf(stderr, "short read\n"); return 2; }
        fclose(f);
    }
    for (size_t i = 0; i < (size_t)W * H; i++) src[i] = (uint8_t)((src[i] * 30 + 127) / 255);
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) ramp[(size_t)y * W + x] = (uint8_t)(x * 31 / W);
    int bad = 0;
    const struct { const char *n; dz_fn fast, ref; } fm[] = { { "dizzy", epd_dizzy, ref_layered }, { "serial", epd_dizzy_serial, ref_serial } };
    const int rects[][4] = { { 0, 0, W, H }, { 600, 300, 512, 512 }, { 37, 5, 301, 77 }, { 1000, 900, 1, 200 },
                             { 3, 1300, 900, 1 }, { 1871, 1403, 1, 1 }, { 64, 64, 64, 64 }, { 1800, 10, 72, 1394 } };
    /* (r 2: the picture with its black and white fixed, as glassine-epdd-desktop hands it over) */
    uint8_t *fix = malloc((size_t)W * H);
    for (size_t i = 0; i < (size_t)W * H; i++) fix[i] = (uint8_t)(src[i] == 0 || src[i] == 30 ? src[i] | EPD_DZ_FIXED : src[i]);
    for (int k = 0; k < 2; k++)
        for (int r = 0; r < 3; r++)
            for (size_t i = 0; i < sizeof rects / sizeof *rects; i++)
                bad |= same(fm[k].n, fm[k].fast, fm[k].ref, r == 2 ? fix : r ? ramp : src, rects[i][0], rects[i][1], rects[i][2], rects[i][3]);
    const struct { const char *n; int pat; dz_fn fn; double tol; } d[] = {
        { "blue", EPD_DITHER_BLUE, NULL, 1.25 }, { "bayer", EPD_DITHER_BAYER, NULL, 1.25 },
        { "dizzy", -1, epd_dizzy, 2.0 }, { "serial", -1, epd_dizzy_serial, 1.25 } };
    for (int k = 0; k < 4; k++)
        for (int r = 0; r < 2; r++) {
            const uint8_t *in = r ? ramp : src;
            memcpy(a, in, (size_t)W * H);
            if (d[k].fn) d[k].fn(a, W, H, W, 0, 0);
            else ordered(a, d[k].pat);
            char nm[64];
            snprintf(nm, sizeof nm, "%s%s", r ? "ramp-" : "", d[k].n);
            put(argv[2], nm, a);
            bad |= check(nm, in, a, r ? 31 : 1, d[k].tol);
        }
    double t[64], u[64], v[64];
    if (reps > 64) reps = 64;
    for (int k = 0; k < 2; k++) {
        for (int i = 0; i < reps; i++) {
            memcpy(a, src, (size_t)W * H);
            double t0 = now_ms();
            fm[k].fast(a, W, H, W, 0, 0);
            t[i] = now_ms() - t0;
            t0 = now_ms();
            fm[k].fast(a + (size_t)300 * W + 600, 512, 512, W, 600, 300);
            u[i] = now_ms() - t0;
            memcpy(a, fix, (size_t)W * H);
            t0 = now_ms();
            fm[k].fast(a, W, H, W, 0, 0);
            v[i] = now_ms() - t0;
        }
        qsort(t, reps, sizeof *t, cmpd);
        qsort(u, reps, sizeof *u, cmpd);
        qsort(v, reps, sizeof *v, cmpd);
        printf("%s: full panel %.1f ms (black/white fixed %.1f), 512 x 512 %.2f ms (median of %d)\n", fm[k].n, t[reps / 2], v[reps / 2], u[reps / 2], reps);
        uint8_t *b = malloc((size_t)W * H);
        memcpy(a, src, (size_t)W * H); fm[k].fast(a, W, H, W, 0, 0);
        memcpy(b, src, (size_t)W * H); fm[k].fast(b, W, H, W, 0, 0);
        if (memcmp(a, b, (size_t)W * H)) { printf("%s: not deterministic\n", fm[k].n); bad = 1; }
        free(b);
    }
    free(src); free(a); free(ramp); free(fix);
    puts(bad ? "FAIL" : "all ok");
    return bad;
}
