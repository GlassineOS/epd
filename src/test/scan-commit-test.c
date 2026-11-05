/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* scan-commit-test — glassine-epdd-desktop's scan + commit (branch-free,
 * vectorized; extracted from glassine-epdd-desktop.c by extract-scan-commit.sh) against the
 * per-pixel reference they replaced (below, verbatim): boxes, "left" counts and
 * the shadow must match for random boxes, data, `all` and every commit mode;
 * then the time of both for a Firefox-sized window scrolled 36 rows.
 * cc ... -include <extracted> scan-commit-test.c ../epd-policy.c (run.sh) */
#define _GNU_SOURCE             /* (the extract's dizzy workers: CPU affinity) */
#include <sched.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "../epd-policy.h"
#include "../epd-dither.h"
#define PW 1872
#define PH 1404
#define TILE 32
#define TW ((PH + TILE - 1) / TILE)
#define TH ((PW + TILE - 1) / TILE)
static box tilebox[TW * TH];
static uint8_t *shadow;
static uint32_t *ebuf;
static const uint8_t bayer8[8][8] = {
    { 0, 32,  8, 40,  2, 34, 10, 42 }, { 48, 16, 56, 24, 50, 18, 58, 26 },
    { 12, 44,  4, 36, 14, 46,  6, 38 }, { 60, 28, 52, 20, 62, 30, 54, 22 },
    { 3, 35, 11, 43,  1, 33,  9, 41 }, { 51, 19, 59, 27, 49, 17, 57, 25 },
    { 15, 47,  7, 39, 13, 45,  5, 37 }, { 63, 31, 55, 23, 61, 29, 53, 21 } };
static int bw_level(uint8_t g) { return (g >> 4) == 0 || (g >> 4) == 15; }
static int dither_pattern;              /* glassine-epdd-desktop's dither_pattern= (both patterns run) */
/* the reference's threshold, straight from the definitions (not epd_dither_table) */
static int ref_thr(int row, int col)
{
    return dither_pattern == EPD_DITHER_BAYER ? bayer8[row & 7][col & 7] * 4 + 2
                                              : epd_bn_rank[(col + 32) & 63][(row + 32) & 63] * 255 >> 12;
}
typedef struct { box b; int bx0, w, h; uint8_t *g; int s; } frame;   /* s: row stride (the live code) */
/* the reference: glassine-epdd-desktop.c before (commit_ebuf = the stock kernel path, kept there) */
static void scan_ref(const frame *f, int all, boxlist *out)
{
    box b = f->b;
    int tx0 = b.x0 / TILE, tx1 = (b.x1 - 1) / TILE, ty0 = b.y0 / TILE, ty1 = (b.y1 - 1) / TILE;
    for (int ty = ty0; ty <= ty1; ty++)
        for (int tx = tx0; tx <= tx1; tx++) tilebox[ty * TW + tx] = (box){0};
    for (int px = b.x0; px < b.x1; px++) {
        const uint8_t *g = f->g + (size_t)(px - b.x0) * f->w, *sh = shadow + (size_t)px * PW + f->bx0;
        box *trow = &tilebox[px / TILE];
        for (int i = 0; i < f->w; i++) {
            if (!all && !((sh[i] ^ g[i]) >> 4)) continue;
            int py = b.y1 - 1 - i;
            box *t = trow + (py / TILE) * TW;
            if (box_empty(*t)) *t = (box){ px, py, px + 1, py + 1 };
            else {
                if (px < t->x0) t->x0 = px;
                if (px >= t->x1) t->x1 = px + 1;
                if (py < t->y0) t->y0 = py;
                if (py >= t->y1) t->y1 = py + 1;
            }
        }
    }
    for (int ty = ty0; ty <= ty1; ty++)
        for (int tx = tx0; tx <= tx1; tx++)
            if (!box_empty(tilebox[ty * TW + tx])) list_add(out, tilebox[ty * TW + tx]);
}
static long commit_ebuf(const frame *f, int all, int how)
{
    long left = 0;
    for (int px = f->b.x0; px < f->b.x1; px++) {
        const uint8_t *gr = f->g + (size_t)(px - f->b.x0) * f->w;
        size_t o0 = (size_t)px * PW + f->bx0;
        for (int i = 0; i < f->w; i++) {
            uint8_t g = gr[i];
            size_t o = o0 + i;
            if (!all && shadow[o] == g) continue;
            if (how == C_BW && ((shadow[o] ^ g) >> 4) && !bw_level(g)) { left++; continue; }
            if (how == C_Q4 && !bw_level(g)) {     /* ordered dither to 0/85/170/255: DU4's levels */
                int s = g * 3, q = s / 255;
                if (s % 255 > ref_thr(px, f->bx0 + i)) q++;
                g = (uint8_t)(q * 85);
                if (shadow[o] == g) continue;
            }
            if ((how == C_DITHER || how == C_THRESH) && !bw_level(g)) {
                g = g > (how == C_THRESH ? 127 : ref_thr(px, f->bx0 + i)) ? 255 : 0;
                if (shadow[o] == g) continue;
            }
            shadow[o] = g;
            if (ebuf) ebuf[o] = 0xFF000000u | g << 16 | g << 8 | g;
        }
    }
    return left;
}
#include <limits.h>
#include <pthread.h>
#include "../epd-dizzy.h"
static pthread_mutex_t buf_mu = PTHREAD_MUTEX_INITIALIZER;     /* (glassine-epdd-desktop's: dizzy_reuse takes it) */
static int dz_reuse;                    /* (glassine-epdd-desktop's, declared above apply_cmd) */
static uint8_t *dz_pg, *dz_pd;
static int dizzy_still_serial;          /* glassine-epdd-desktop's dizzy_still= */
/* (glassine-epdd-desktop's GPU dizzy: no ingest here, so the CPU bands as before) */
static int dizzy_gpu, dizzy_form_layered;
static uint8_t *wlsrc;                  /* (glassine-epdd-desktop's view of sway's frame: none here) */
static void *ing;
static int gt_main(void) { return 0; }
enum { EPD_GDZ_BLOCK, EPD_GDZ_LAYERED };
static int epd_ingest_dizzy_form(void *g, int form) { (void)g; return form; }
static int epd_ingest_dizzy_src(const void *g) { (void)g; return 0; }
static int epd_ingest_dizzy_v(void *g, const uint8_t *gr, int gs, int x0, int y0, int w, int h, const uint8_t *lv, uint8_t *out, int os, int ms, int view)
{
    (void)g; (void)gr; (void)gs; (void)x0; (void)y0; (void)w; (void)h; (void)lv; (void)out; (void)os; (void)ms; (void)view;
    return -1;
}
static double tsec(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec / 1e9; }
/* glassine-epdd-desktop's wl_is_view: a frame that reads sway's frame (here: wlsrc, rows PW apart) */
#define wl_is_view(f) (wlsrc && (f)->s == PW && (f)->g == wlsrc + (size_t)(f)->b.x0 * PW + (f)->bx0)
static int upd_acct;                    /* (epd-upd.h's: the extract's commit_census reads it; off) */
#include LIVE
static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec / 1e9; }
static frame mkframe(box b, uint8_t *g) { return (frame){ b, PW - b.y1, b.y1 - b.y0, b.x1 - b.x0, g, b.y1 - b.y0 }; }
static int same_list(boxlist *a, boxlist *b) { return a->n == b->n && !memcmp(a->b, b->b, sizeof(box) * a->n); }
static uint8_t *P;              /* portrait page, PH wide, long */
#define PAGE 20000
static void page(void)
{
    P = malloc((size_t)PH * PAGE); memset(P, 255, (size_t)PH * PAGE); srand(5);
    for (int line = 0; line < PAGE / 40; line++)
        for (int x = 40; x < PH - 40; x += 14 + rand() % 6) {
            if (rand() % 7 == 0) { x += 20; continue; }
            int h = 14 + rand() % 6, y0 = line * 40 + 10;
            for (int y = y0; y < y0 + h && y < PAGE; y++)
                for (int k = 0; k < 9; k++) { int v = rand() % 4; P[(size_t)y * PH + x + k] = v == 0 ? 0 : v == 1 ? 96 : v == 2 ? 192 : 255; }
        }
}
/* the window at page offset off into frame layout */
static void fill(uint8_t *g, box b, int off)
{
    int w = b.y1 - b.y0;
    for (int px = b.x0; px < b.x1; px++)
        for (int i = 0; i < w; i++) g[(size_t)(px - b.x0) * w + i] = P[(size_t)(off + b.y1 - 1 - i) * PH + px];
}
int main(void)
{
    shadow = malloc((size_t)PW * PH);
    uint8_t *sh2 = malloc((size_t)PW * PH), *g = malloc((size_t)PW * PH), *view = malloc((size_t)PW * PH);
    page();
    int bad = 0;
    srand(11);
    for (int it = 0; it < 3000; it++) {         /* equivalence: random boxes, data, modes */
        box b; b.x0 = rand() % PH; b.y0 = rand() % PW; b.x1 = b.x0 + 1 + rand() % (PH - b.x0); b.y1 = b.y0 + 1 + rand() % (PW - b.y0);
        if (rand() % 3 == 0) { b.x1 = b.x0 + 1 + rand() % (PH - b.x0 < 70 ? PH - b.x0 : 70); b.y1 = b.y0 + 1 + rand() % (PW - b.y0 < 70 ? PW - b.y0 : 70); }
        int kind = rand() % 3;
        size_t n = (size_t)(b.x1 - b.x0) * (b.y1 - b.y0);
        if (it % 50 == 0) for (size_t i = 0; i < (size_t)PW * PH; i++) shadow[i] = (uint8_t)(rand() % 3 == 0 ? rand() : (rand() & 1) * 255);
        for (size_t i = 0; i < n; i++) g[i] = kind == 0 ? (uint8_t)rand() : kind == 1 ? (uint8_t)((rand() & 1) * 255 ^ (rand() % 5 == 0 ? 0x13 : 0)) : (uint8_t)(rand() % 4 * 85);
        if (kind == 2 && rand() % 2) memcpy(g, shadow, n < 4000 ? n : 4000);
        frame f = mkframe(b, g);
        int all = rand() % 5 == 0, how = rand() % 5;
        dither_pattern = it % 3 == 2 ? EPD_DITHER_BAYER : EPD_DITHER_BLUE;
        boxlist la = {0}, lb = {0};
        memcpy(sh2, shadow, (size_t)PW * PH);
        scan_ref(&f, all, &la);
        long l1 = commit_ebuf(&f, all, how);
        uint8_t *keep = shadow; shadow = sh2;
        frame fl = f;                           /* every other case: a view with stride PW (snap_wl's wlsrc) */
        if (it & 1) {
            int off = rand() % (PW - f.w + 1);
            for (int r = 0; r < f.h; r++) memcpy(view + (size_t)r * PW + off, g + (size_t)r * f.w, (size_t)f.w);
            fl.g = view + off; fl.s = PW;
        }
        scan(&fl, all, &lb);
        long l2 = commit(&fl, all, how);
        shadow = keep;
        if (!same_list(&la, &lb) || l1 != l2 || memcmp(shadow, sh2, (size_t)PW * PH)) {
            if (bad++ < 5) fprintf(stderr, "differ it %d box %d,%d-%d,%d all %d how %d: lists %d/%d left %ld/%ld shadow %d\n", it, b.x0, b.y0, b.x1, b.y1, all, how,
                                   la.n, lb.n, l1, l2, memcmp(shadow, sh2, (size_t)PW * PH) != 0);
        }
    }
    printf("scan-commit: equivalence %s (%d of 3000 differ; blue + bayer dither)\n", bad ? "FAILED" : "ok", bad);
    /* the dither: a flat grey's white share (g / 255), and a 50 % grey's black
     * pixels spread evenly over the stagger's groups (rank * G >> 12): blue
     * must be independent of them; the panel, every commit mode's live code */
    for (int pat = 0; pat < 2; pat++) {
        dither_pattern = pat ? EPD_DITHER_BAYER : EPD_DITHER_BLUE;
        box all = { 0, 0, PH, PW };
        double werr = 0;
        for (int gv = 16; gv < 240; gv += 16) {
            memset(g, gv, (size_t)PW * PH);
            memset(shadow, 255, (size_t)PW * PH);
            frame f = mkframe(all, g);
            commit(&f, 1, C_DITHER);
            long w = 0;
            for (size_t i = 0; i < (size_t)PW * PH; i++) w += shadow[i] == 255;
            double e = (double)w / ((size_t)PW * PH) - gv / 255.0;
            if (e < 0) e = -e;
            if (e > werr) werr = e;
        }
        memset(g, 128, (size_t)PW * PH);
        frame f = mkframe(all, g);
        commit(&f, 1, C_DITHER);
        printf("scan-commit: dither %-5s white share vs g/255 worst %.4f; 50 %% grey's black per stagger group:", pat ? "bayer" : "blue", werr);
        if (!pat && werr > 0.005) { fprintf(stderr, "blue dither: white share off by %.4f\n", werr); bad++; }
        for (int G = 2; G <= 8; G *= 2) {
            long n[8] = { 0 }, tot = 0;
            for (int row = 0; row < PH; row++)
                for (int col = 0; col < PW; col++)
                    if (!shadow[(size_t)row * PW + col]) { n[epd_bn_rank[row & 63][col & 63] * G >> 12]++; tot++; }
            double worst = 0;
            printf(" G%d", G);
            for (int k = 0; k < G; k++) {
                double fr = (double)n[k] / tot, d = fr * G - 1;
                printf("%c%.3f", k ? '/' : ' ', fr);
                if (d < 0) d = -d;
                if (d > worst) worst = d;
            }
            if (!pat && worst > 0.10) { fprintf(stderr, "blue dither: G %d group off by %.2f of 1/G\n", G, worst); bad++; }
        }
        printf("\n");
    }
    /* dizzy (dither_pattern=dizzy): the live commit == epd-dizzy.h on the
     * rect's levels in DZ_BAND-row bands, black/white fixed (layered in motion,
     * dizzy_still= otherwise) under
     * commit_bw2_row's rule (black/white targets kept, !all: equal stays);
     * then a flat grey's white share; C_Q4 takes blue noise */
    dither_pattern = EPD_DITHER_DIZZY;
    {
        const box rb[] = { { 0, 0, PH, PW }, { 100, 300, 700, 1000 }, { 5, 7, 6, 1500 }, { 1000, 1800, 1404, 1872 } };
        uint8_t *lv = malloc((size_t)PW * PH);
        int dbad = 0;
        for (int c = 0; c < 4; c++)
            for (int mode = 0; mode < 3; mode++)
                for (int all = 0; all < 2; all++) {
                    box b = rb[c];
                    cm_motion = mode == 0;
                    dizzy_still_serial = mode == 2;
                    fill(g, b, 300 + 40 * c);
                    frame f = mkframe(b, g);
                    for (size_t i = 0; i < (size_t)PW * PH; i++) shadow[i] = (uint8_t)(rand() % 3 == 0 ? rand() : (rand() & 1) * 255);
                    for (int i = 0; i < f.h * f.w && all == 0; i += 7) shadow[(size_t)(f.b.x0 + i / f.w) * PW + f.bx0 + i % f.w] = g[i];
                    memcpy(sh2, shadow, (size_t)PW * PH);
                    for (int i = 0; i < f.h * f.w; i++) lv[i] = (uint8_t)((g[i] * 30 + 127) / 255 | (bw_level(g[i]) ? EPD_DZ_FIXED : 0));
                    for (int r0 = 0; r0 < f.h;) {       /* (the bands: every DZ_BAND-th panel row) */
                        int r1 = ((f.b.x0 + r0) / DZ_BAND + 1) * DZ_BAND - f.b.x0;
                        if (r1 > f.h) r1 = f.h;
                        (mode == 2 ? epd_dizzy_serial : epd_dizzy)(lv + (size_t)r0 * f.w, f.w, r1 - r0, f.w, f.bx0, f.b.x0 + r0);
                        r0 = r1;
                    }
                    for (int i = 0; i < f.h * f.w; i++) {
                        uint8_t *o = sh2 + (size_t)(f.b.x0 + i / f.w) * PW + f.bx0 + i % f.w, v = g[i];
                        if (!all && *o == v) continue;
                        *o = bw_level(v) ? v : (lv[i] & 0x7f) ? 255 : 0;
                    }
                    commit(&f, all, C_DITHER);
                    if (memcmp(shadow, sh2, (size_t)PW * PH)) {
                        if (dbad++ < 5) fprintf(stderr, "dizzy differ: rect %d %s all %d\n", c, mode == 0 ? "motion" : mode == 1 ? "still layered" : "still serial", all);
                    }
                }
        cm_motion = 0;
        dizzy_still_serial = 0;
        double werr[2] = { 0, 0 };
        for (int m = 0; m < 2; m++)
            for (int gv = 16; gv < 240; gv += 16) {
                box all = { 0, 0, PH, PW };
                memset(g, gv, (size_t)PW * PH);
                memset(shadow, 255, (size_t)PW * PH);
                dizzy_still_serial = m;
                frame f = mkframe(all, g);
                commit(&f, 1, C_DITHER);
                long w = 0;
                for (size_t i = 0; i < (size_t)PW * PH; i++) w += shadow[i] == 255;
                double e = (double)w / ((size_t)PW * PH) - gv / 255.0;
                if (e < 0) e = -e;
                if (e > werr[m]) werr[m] = e;
            }
        dizzy_still_serial = 0;
        printf("scan-commit: dizzy == epd-dizzy.h %s (%d of 24 differ); flat grey white share vs g/255 worst layered %.4f serial %.4f\n",
               dbad ? "FAILED" : "ok", dbad, werr[0], werr[1]);
        /* (the algorithm's own loss, epd-dizzy.h: a pixel with no later
         * neighbour drops its error, 1/5 of them in serial's random order and
         * every last-layer one in layered's: ~1.1 / 1.5 levels of 30 here) */
        if (werr[0] > 0.06 || werr[1] > 0.045) { fprintf(stderr, "dizzy: white share off\n"); dbad++; }
        bad += dbad;
        free(lv);
    }
    dither_pattern = EPD_DITHER_BLUE;
    /* timing: a Firefox-sized window scrolled 36 rows per update, each commit mode */
    box win = { 0, 60, PH, 1300 + 60 };
    const char *nm[] = { "all", "bw", "q4", "dither", "thresh" };
    for (int how = 0; how < 5; how++) {
        double t[2][2] = {{0}};
        for (int v = 0; v < 2; v++) {
            fill(g, win, 0); frame f0 = mkframe(win, g);
            memset(shadow, 255, (size_t)PW * PH); commit_ebuf(&f0, 1, C_ALL);
            for (int k = 1; k <= 60; k++) {
                fill(g, win, k * 36); frame f = mkframe(win, g);
                boxlist l = {0};
                double t0 = now();
                if (v) scan(&f, 0, &l); else scan_ref(&f, 0, &l);
                double t1 = now();
                if (v) commit(&f, 0, how); else commit_ebuf(&f, 0, how);
                t[v][0] += t1 - t0; t[v][1] += now() - t1;
            }
        }
        printf("scan-commit: %-7s scan ref %.2f new %.2f ms, commit ref %.2f new %.2f ms per update (%.1f Mpx)\n", nm[how],
               t[0][0] * 1e3 / 60, t[1][0] * 1e3 / 60, t[0][1] * 1e3 / 60, t[1][1] * 1e3 / 60, (double)PH * 1300 / 1e6);
    }
    return bad != 0;
}
