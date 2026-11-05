/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-motion: the shift finder on synthetic pages + the motion state on
 * touch/burst sequences (taps, scrolls, swipes, flings, content alone) */
#include "../epd-motion.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL: " __VA_ARGS__); printf("\n"); } } while (0)

#define PWD 1404
#define PHT 6000
static uint8_t page[PHT][PWD];          /* portrait page, [y][x] */


/* text-ish: lines of "glyphs" (dark blobs of random widths) with grey images
 * between; period > 0: cards repeating every `period` rows (a list) */
static void make_page(int period)
{
    memset(page, 255, sizeof page);
    for (int y = 0; y < PHT; y++) {
        int yy = period ? y % period : y;
        int line = yy / 36, in = yy % 36;
        if (period && yy > period - 24) continue;                       /* card gap */
        if (!period && line % 9 == 8) {                                 /* an image band: greys */
            for (int x = 100; x < 1300; x++) page[y][x] = (uint8_t)(64 + ((x * 7 + y * 3) % 128));
            continue;
        }
        if (in < 8 || in > 28) continue;
        unsigned seed = (unsigned)(period ? (y / period) * 131 + line : y / 36) * 2654435761u;   /* each card its own text */
        int x = 40;
        while (x < 1360) {
            seed = seed * 1103515245u + 12345u;
            int w = 8 + (seed >> 16) % 18, gap = 4 + (seed >> 8) % 6;
            if (((seed >> 4) & 15) == 0) gap += 20;                      /* a word space */
            for (int k = x; k < x + w && k < PWD; k++) page[y][k] = (uint8_t)(((seed >> 3) ^ k) & 1 ? 0 : 40);
            x += w + gap;
        }
    }
}

/* k columns of the window [top, top + n) as glassine-epdd-desktop's frame rows: index j =
 * portrait y reversed. dither: threshold through a 2x2 Bayer (a run's commit) */
static uint8_t rows_o[16][2048], rows_n[16][2048];
static const uint8_t bayer8[8][8] = {           /* glassine-epdd-desktop's (a run's commit) */
    { 0, 32,  8, 40,  2, 34, 10, 42 }, { 48, 16, 56, 24, 50, 18, 58, 26 },
    { 12, 44,  4, 36, 14, 46,  6, 38 }, { 60, 28, 52, 20, 62, 30, 54, 22 },
    { 3, 35, 11, 43,  1, 33,  9, 41 }, { 51, 19, 59, 27, 49, 17, 57, 25 },
    { 15, 47,  7, 39, 13, 45,  5, 37 }, { 63, 31, 55, 23, 61, 29, 53, 21 } };
/* 4 neighbouring columns from x as glassine-epdd-desktop's frame rows (index j = portrait y
 * reversed), averaged (mo_avg4). dither: the old frame is a run's commit, the
 * greys ordered-dithered to black/white in panel coordinates */
static void column(uint8_t *dst, int x, int top, int n, int dither)
{
    static uint8_t c4[4][2048];
    for (int q = 0; q < 4; q++)
        for (int j = 0; j < n; j++) {
            uint8_t g = page[top + n - 1 - j][x + q];
            c4[q][j] = dither ? (g > bayer8[(x + q) & 7][j & 7] * 4 + 2 ? 255 : 0) : g;
        }
    mo_avg4(dst, c4[0], sizeof c4[0], n);
}
static int verbose_shift = 1;
static int find(int k, int n, int top_old, int top_new, int dither_old, double *score)
{
    const uint8_t *o[16], *nw[16];
    for (int r = 0; r < k; r++) {
        int x = 60 + r * (1280 / k);
        column(rows_o[r], x, top_old, n, dither_old);
        column(rows_n[r], x, top_new, n, 0);
        o[r] = rows_o[r]; nw[r] = rows_n[r];
    }
    int s = mo_find_shift(o, nw, k, n, n / 2, score);
    if (verbose_shift) printf("  shift old %d new %d%s: %d score %.2f\n", top_old, top_new, dither_old ? " (dithered old)" : "", s, *score);
    return s;
}

static void shift_tests(void)
{
    double sc;
    int s;
    make_page(0);
    /* the view scrolled down by 37 px: the content moved UP 37 (portrait dy -37) */
    s = find(12, 1700, 1000, 1037, 0, &sc);
    CHECK(s == -37 && sc > 0.8, "scroll 37: shift %d score %.2f", s, sc);
    s = find(12, 1700, 1000, 963, 0, &sc);
    CHECK(s == 37 && sc > 0.8, "scroll back 37: shift %d score %.2f", s, sc);
    s = find(12, 1700, 1000, 1600, 0, &sc);
    CHECK(s == -600 && sc > 0.6, "fling step 600: shift %d score %.2f", s, sc);
    s = find(12, 1700, 1000, 1003, 0, &sc);
    CHECK(s == -3 && sc > 0.5, "creep 3: shift %d score %.2f", s, sc);
    /* run commits: the panel holds a dithered old frame (8 x 8 ordered) --
     * also the first small steps of a scroll from a run's state */
    s = find(12, 1700, 2000, 2120, 1, &sc);
    CHECK(s == -120 && sc >= MO_SHIFT_SCORE, "dithered old, scroll 120: shift %d score %.2f", s, sc);
    s = find(12, 1700, 2000, 2017, 1, &sc);
    CHECK(s == -17 && sc >= MO_SHIFT_SCORE, "dithered old, scroll 17: shift %d score %.2f", s, sc);
    s = find(12, 1700, 2000, 1988, 1, &sc);
    CHECK(s == 12 && sc >= MO_SHIFT_SCORE, "dithered old, scroll back 12: shift %d score %.2f", s, sc);
    s = find(12, 1700, 1000, 4000, 1, &sc);
    CHECK(!s || sc < MO_SHIFT_SCORE, "dithered old, new page: shift %d score %.2f", s, sc);
    /* a new page (page turn, link): no shift */
    s = find(12, 1700, 1000, 4000, 0, &sc);
    CHECK(!s || sc < MO_SHIFT_SCORE, "new page: shift %d score %.2f", s, sc);
    /* a repaint in place (a button pressed): nothing moved */
    {
        const uint8_t *o[12], *nw[12];
        for (int r = 0; r < 12; r++) {
            int x = 60 + r * 106;
            column(rows_o[r], x, 1000, 400, 0);
            column(rows_n[r], x, 1000, 400, 0);
            for (int j = 150; j < 250; j++) rows_n[r][j] = (uint8_t)(255 - rows_n[r][j]);   /* highlight */
            o[r] = rows_o[r]; nw[r] = rows_n[r];
        }
        s = mo_find_shift(o, nw, 12, 400, 200, &sc);
        CHECK(!s || sc < MO_SHIFT_SCORE, "button repaint: shift %d score %.2f", s, sc);
    }
    /* identical: nothing to say */
    s = find(12, 1700, 1000, 1000, 0, &sc);
    CHECK(!s && sc == 0, "identical: shift %d score %.2f", s, sc);
    /* a list of cards (period 224, same layout, own text): the true 50, not 50 - 224 */
    make_page(224);
    s = find(12, 1700, 1000, 1050, 0, &sc);
    CHECK(s == -50 && sc > 0.6, "card list, scroll 50: shift %d score %.2f", s, sc);
    /* cost: 12 rows of a full window */
    make_page(0);
    verbose_shift = 0;
    struct timespec a, b;
    clock_gettime(CLOCK_MONOTONIC, &a);
    find(12, 1762, 1000, 1040, 1, &sc);
    const uint8_t *o[12], *nw[12];
    for (int r = 0; r < 12; r++) { o[r] = rows_o[r]; nw[r] = rows_n[r]; }
    clock_gettime(CLOCK_MONOTONIC, &a);
    for (int i = 0; i < 50; i++) mo_find_shift(o, nw, 12, 1762, 881, &sc);
    clock_gettime(CLOCK_MONOTONIC, &b);
    double ms = ((b.tv_sec - a.tv_sec) * 1e3 + (b.tv_nsec - a.tv_nsec) / 1e6) / 50;
    printf("epd-motion: shift over 12 x 1762 px: %.2f ms per measurement\n", ms);
}

static void state_tests(void)
{
    motion m;
    double v;
    int st;
    const double px = 1.0 / 1872;

    /* a tap with a 3 px jitter, its button repaints: never motion */
    mo_init(&m, 1872);
    mo_down(&m, 1000, 0.3, 0.5);
    mo_move(&m, 1010, 0.3 + 3 * px, 0.5);
    mo_burst(&m, 1040, 1, 0, 0);
    CHECK(mo_state(&m, 1040, &v) == MO_PRESS, "tap: not a press while down");
    mo_up(&m, 1080);
    mo_burst(&m, 1120, 1, 0, 0);
    CHECK(mo_state(&m, 1120, &v) == MO_STILL, "tap: not still after the lift");

    /* a scroll: the app moves the content after 10 px of finger travel (< MO_SLOP);
     * the first shifted frame is the gesture */
    mo_init(&m, 1872);
    mo_down(&m, 1000, 0.3, 0.5);
    mo_burst(&m, 1000, 1, 0, 0);
    mo_move(&m, 1030, 0.3, 0.5 - 10 * px);
    mo_burst(&m, 1050, 1, -20, 0.9);
    st = mo_state(&m, 1050, &v);
    CHECK(st == MO_MOVING && m.by_touch && m.early == 1, "scroll: first shifted frame not moving (%d, by_touch %d, early %lu)", st, m.by_touch, m.early);
    CHECK(fabs(v - 20.0 / 1872 / 0.05) < 1e-6, "scroll: content speed %.3f", v);
    /* the finger rests on the page, nothing shifts: the motion ends */
    mo_burst(&m, 1100, 1, 0, 0);
    st = mo_state(&m, 1100 + MO_HOLD_MS + 10, &v);
    CHECK(st == MO_PRESS, "scroll stopped under a resting finger: %d", st);

    /* a swipe that replaces the page (KOReader): the finger alone */
    mo_init(&m, 1872);
    mo_down(&m, 1000, 0.6, 0.5);
    mo_move(&m, 1020, 0.6 - 0.01, 0.5);
    CHECK(mo_state(&m, 1020, &v) == MO_PRESS, "swipe: moving before MO_SLOP");
    mo_move(&m, 1040, 0.6 - 0.05, 0.5);
    st = mo_state(&m, 1040, &v);
    CHECK(st == MO_MOVING && v > 0.5, "swipe: %d, finger speed %.2f", st, v);
    mo_up(&m, 1060);
    mo_burst(&m, 1100, 1, 0, 0.1);
    CHECK(mo_state(&m, 1100, &v) == MO_MOVING && m.by_touch, "swipe: the page's repaint after the lift is not part of it");
    CHECK(mo_state(&m, 1060 + MO_HOLD_MS + 10, &v) == MO_STILL, "swipe: still moving long after");

    /* content alone (wheel, keys, an animation): one shifted burst may be a
     * layout jump; the second close one is motion */
    mo_init(&m, 1872);
    mo_burst(&m, 1000, 1, 0, 0);
    mo_burst(&m, 1100, 1, 40, 0.9);
    CHECK(mo_state(&m, 1100, &v) == MO_STILL, "content alone: one shift is motion");
    mo_burst(&m, 1180, 1, 40, 0.9);
    st = mo_state(&m, 1180, &v);
    CHECK(st == MO_MOVING && !m.by_touch, "content alone: two shifts not motion (%d, by_touch %d)", st, m.by_touch);
    CHECK(mo_state(&m, 1180 + MO_HOLD_MS + 10, &v) == MO_STILL, "content alone: never ends");
    /* far apart: no */
    mo_init(&m, 1872);
    mo_burst(&m, 1000, 1, 40, 0.9);
    mo_burst(&m, 1000 + MO_ALONE_MS + 100, 1, 40, 0.9);
    CHECK(mo_state(&m, 1000 + MO_ALONE_MS + 100, &v) == MO_STILL, "content alone: two far-apart shifts are motion");

    /* a fling: the finger lifts mid-scroll, the content keeps moving for 2 s */
    mo_init(&m, 1872);
    mo_down(&m, 0, 0.3, 0.8);
    for (int t = 10; t <= 200; t += 10) mo_move(&m, t, 0.3, 0.8 - t * 0.002);
    mo_up(&m, 200);
    int moving_all = 1;
    double last_v = 0;
    for (int t = 250, d = 120; t <= 2250; t += 80, d = d * 9 / 10 + 1) {
        mo_burst(&m, t, 1, -d, 0.9);
        if (mo_state(&m, t, &v) != MO_MOVING) moving_all = 0;
        last_v = v;
    }
    CHECK(moving_all, "fling: the tail stopped counting as motion");
    CHECK(last_v > 0 && last_v < 0.5, "fling: tail speed %.2f", last_v);
    CHECK(!m.by_touch, "fling: the tail 2 s after the lift is still by touch");

    /* a pinch is a gesture at once */
    mo_init(&m, 1872);
    mo_down(&m, 0, 0.5, 0.5);
    mo_pinch(&m, 20);
    CHECK(mo_state(&m, 20, &v) == MO_MOVING, "pinch: not moving");

    /* when a measurement is worth it */
    mo_init(&m, 1872);
    CHECK(!mo_wants_shift(&m, 5000), "idle: wants a shift measurement");
    mo_down(&m, 5000, 0.5, 0.5);
    CHECK(mo_wants_shift(&m, 5000), "finger down: no shift measurement");
}

int main(void)
{
    shift_tests();
    state_tests();
    printf("epd-motion: %s\n", fails ? "FAILED" : "ok");
    return fails != 0;
}
