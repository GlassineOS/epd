/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-matched-test — matched cuts (epd_set_matched) on a replayed scroll:
 * flings over a black/white text page, each step a cut A2 by speed (the
 * policy's steps 8/6/4), the next step submitted when the last is done
 * (panel-paced), a GC16 flash at each fling's end (the run end). Off vs v1
 * (every cut step) vs v2 (epd_set_matched_rule 6, 2: only steps of >= 6
 * frames lengthen, by <= 2; the fast ones must keep their frames):
 * the DC record (max, beyond 60, mean |dc|) and frames per step; the levels
 * after every flash must match. Plus: on with an all-zero record renders
 * the same words as off (nothing opposes yet). Each step comes as 1-4
 * horizontal bands (separate submits, as sway's damage does), split anew per
 * step, so a waiting rest of one step is rarely inside one band of the next
 * (the device run: 9621 order waits, a still image); v3 must not
 * wait on order at all and keeps the fast steps' frames. v4: v3 + healing
 * (epd_set_heal 2, 8: the slowest steps), v5: v4 with the lengths from the panel model
 * (epd_set_model) capped at +2; every engine keeps the model, which judges
 * them all: the mean shortfall while the scroll slows (steps of >= 6 frames, uncut)
 * and right before each fling's flash. v4 must settle cleaner than v3,
 * without slowing the fast steps. v6: the device's evening setting
 * (panel model, uncapped, nonblock, heal 2 from 2); v7: v6 with preemption
 * (nonblock 2): fewer pixels short during the fast steps than v6; v8: v7
 * with the DC record asking for more where it opposes (epd_set_model 2): a
 * smaller record than v7, no more pixels short; v9: v8 with full waveforms
 * from the model's rail (epd_set_snap): a smaller record than v8 (the
 * flashes no longer swing nearly-white pixels from "black"; no larger here:
 * uncapped matched cuts leave no pixel short). Then 3: the device's
 * setting (fixed A2/2 scroll, matched cap 3, heal 2/2, preempt,
 * model 2) with a GC16 at each fling's end, partial (the dirty cleanup) or a
 * flash (a run with cuts), v8 vs v9: v9 leaves fewer pixels short (v8: a
 * partial GC16 skips a pixel its last cut step left half-way, its level being
 * the target) for a record at most 10 % larger (the top-up's frames). 4: one
 * white pixel, an A2/2 to black then a partial GC16 back to white: v8 adds
 * +20 to the record, v9 (snapped: still nearly white) about 0.
 * epd-matched-test WBF [flings] */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../epd-engine.h"

#define PAGE_H 8000
static uint8_t *page, *scr;
#define NE 10
static epd_engine E[NE];

static void make_page(void)
{
    page = malloc((size_t)PAGE_H * EPD_W);
    memset(page, 255, (size_t)PAGE_H * EPD_W);
    srand(7);
    for (int line = 0; line * 40 + 30 < PAGE_H; line++)        /* 40 px lines, 24 px glyphs */
        for (int cx = 60; cx + 14 < EPD_W - 60; cx += 14) {
            if (rand() % 7 == 0) continue;                      /* spaces */
            uint32_t g = (uint32_t)rand() * 2654435761u;
            for (int y = 0; y < 24; y++)
                for (int x = 0; x < 12; x++)
                    if ((g >> ((y / 4) * 3 + x / 4)) & 1) page[(size_t)(line * 40 + 8 + y) * EPD_W + cx + x] = 0;
        }
}

static epd_rect all = { 0, 0, EPD_W, EPD_H };
/* a step, paced as glassine-epdd-desktop paces sway: frames until the area is free for
 * the next step (epd_frames_until_free; with nonblock it leaves payback pixels
 * out); counts the payback pixels still running at that moment into *lag */
static int step_paced(epd_engine *e, long *lag)
{
    int n = 0;
    for (;;) {
        epd_schedule(e);
        if (!epd_frames_until_free(e, all) || n > 400) break;
        epd_advance(e, NULL, NULL);
        n++;
    }
    if (lag && e->nonblock)
        for (int s = 0; s < EPD_MAX_UPD; s++)
            if (e->run[s].nframes && e->run[s].payback) {
                const epd_run *r = &e->run[s];
                for (int y = r->r.y; y < r->r.y + r->r.h; y += 4)       /* sampled: every 4th row */
                    for (int x = r->r.x; x < r->r.x + r->r.w; x++) *lag += e->busy[(size_t)y * EPD_W + x] == s + 1;
            }
    return n;
}

/* run every engine until idle; frames it took */
static int drain(epd_engine *e)
{
    int n = 0;
    while (epd_schedule(e)) { epd_advance(e, NULL, NULL); n++; }
    return n;
}

static void dcs(const epd_engine *e, int *mx, long *over, double *mean)
{
    epd_dc_stats(e, 60, mx, over);
    double s = 0;
    for (size_t i = 0; i < (size_t)EPD_W * EPD_H; i++) s += e->dc[i] < 0 ? -e->dc[i] : e->dc[i];
    *mean = s / ((double)EPD_W * EPD_H);
}

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: epd-matched-test WBF [flings]\n"); return 2; }
    FILE *f = fopen(argv[1], "rb");
    if (!f) { perror(argv[1]); return 2; }
    fseek(f, 0, SEEK_END); size_t n = (size_t)ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t *w = malloc(n);
    if (fread(w, 1, n, f) != n) return 2;
    fclose(f);
    int flings = argc > 2 ? atoi(argv[2]) : 40, bad = 0;
    make_page();
    scr = malloc((size_t)EPD_W * EPD_H);
    for (int k = 0; k < NE; k++) {
        if (epd_init(&E[k], w, n, 24)) { fprintf(stderr, "wbf\n"); return 2; }
        epd_set_matched(&E[k], k > 0);
        epd_set_model(&E[k], k >= 8 ? 2 : k >= 5, 1);
    }
    epd_set_matched_rule(&E[2], 6, 2);          /* v2: the fast steps keep their length */
    epd_set_matched_rule(&E[3], 2, 0);          /* v3: every step pays in full, without waiting */
    epd_set_nonblock(&E[3], 1);
    for (int k = 4; k < 6; k++) { epd_set_matched_rule(&E[k], 2, k == 5 ? 2 : 0); epd_set_nonblock(&E[k], 1); epd_set_heal(&E[k], 2, 8); }
    for (int k = 6; k < NE; k++) { epd_set_matched_rule(&E[k], 2, 0); epd_set_nonblock(&E[k], k >= 7 ? 2 : 1); epd_set_heal(&E[k], 2, 2); }
    epd_set_snap(&E[9], 1);

    /* 1: matched with a zero record = off, word for word */
    {
        static uint32_t wd[2][EPD_WORDS];
        int dirty[2][2] = { { 0, EPD_LINES }, { 0, EPD_LINES } };
        for (int k = 0; k < 2; k++) memcpy(wd[k], E[k].tpl, sizeof wd[k]);
        memcpy(scr, page, (size_t)EPD_W * EPD_H);
        int same = 1;
        for (int k = 0; k < 2; k++) epd_submit(&E[k], all, EPD_MODE_A2LEN(EPD_A2_OR_DU, 4), 0, scr, EPD_W);
        for (;;) {
            int a = epd_schedule(&E[0]), b = epd_schedule(&E[1]);
            if (a != b) { same = 0; break; }
            if (!a) break;
            for (int k = 0; k < 2; k++) { epd_render(&E[k], wd[k], dirty[k]); epd_advance(&E[k], NULL, NULL); }
            if (memcmp(wd[0], wd[1], sizeof wd[0])) { same = 0; break; }
        }
        printf("zero record: %s\n", same ? "same words as off" : "DIFFERENT");
        bad += !same;
        for (int k = 0; k < NE; k++) {              /* back to a clean white page */
            memset(scr, 255, (size_t)EPD_W * EPD_H);
            epd_submit(&E[k], all, EPD_GC16, 1, scr, EPD_W);
            drain(&E[k]);
            memset(E[k].dc, 0, sizeof E[k].dc);
        }
    }

    /* 2: the scroll */
    long steps[NE] = { 0 }, frames[NE] = { 0 }, flash[NE] = { 0 }, fast[NE] = { 0 }, fastn[NE] = { 0 }, lag[NE] = { 0 };
    double slow_sf[NE] = { 0 }, end_sf[NE] = { 0 }, fast_sf[NE] = { 0 };   /* (+ during the fast steps) */
    long fast_over[NE] = { 0 };     /* the model's mean shortfall: slow steps, before the flash */
    long slown = 0, end_over[NE] = { 0 };
    int pos = 0, dir = 1, lev_same = 1;
    srand(11);
    for (int fl = 0; fl < flings; fl++) {
        double v = 40 + rand() % 160;               /* px per step at the start */
        while (v >= 6) {
            int step = (int)v;
            pos += dir * step;
            if (pos < 0) { pos = 0; dir = 1; }
            if (pos > PAGE_H - EPD_H) { pos = PAGE_H - EPD_H; dir = -1; }
            memcpy(scr, page + (size_t)pos * EPD_W, (size_t)EPD_W * EPD_H);
            int len = v > 120 ? 2 : v > 80 ? 4 : v > 40 ? 6 : v > 15 ? 8 : 0;   /* the curve: mostly A2/2; the slowest = the uncut A2 */
            int nb = 1 + rand() % 4, cut[5] = { 0 };                 /* the bands: nb rects, rows 0..EPD_H */
            cut[nb] = EPD_H;
            for (int b = 1; b < nb; b++) cut[b] = cut[b - 1] + 1 + rand() % ((EPD_H - cut[b - 1]) / (nb - b + 1));
            for (int k = 0; k < NE; k++) {
                for (int b = 0; b < nb; b++) {
                    epd_rect band = { 0, cut[b], EPD_W, cut[b + 1] - cut[b] };
                    epd_submit(&E[k], band, EPD_MODE_A2LEN(EPD_A2_OR_DU, len), 0, scr, EPD_W);
                }
                int nf = step_paced(&E[k], &lag[k]);
                frames[k] += nf;
                steps[k]++;
                if (len && len <= 4) { double m; long o; epd_op_stats(&E[k], 3, &m, &o); fast_sf[k] += m; fast_over[k] += o; fast[k] += nf; fastn[k]++; }
                if (!len || len >= 6) { double m; long o; epd_op_stats(&E[k], 3, &m, &o); slow_sf[k] += m; }
            }
            slown += !len || len >= 6;
            v *= 0.85;
        }
        for (int k = 0; k < NE; k++) {
            frames[k] += drain(&E[k]);              /* the fling's last step (and payback) to its end */
            { double m; long o; epd_op_stats(&E[k], 3, &m, &o); end_sf[k] += m; end_over[k] += o; }
            epd_submit(&E[k], all, EPD_GC16, 1, scr, EPD_W); flash[k] += drain(&E[k]);
        }
        for (int k = 1; k < NE; k++) if (memcmp(E[0].level, E[k].level, sizeof E[0].level)) lev_same = 0;
        if (rand() % 3 == 0) dir = -dir;
    }
    static const char *const nm[NE] = { "off", "v1", "v2 6/2", "v3 nonblock", "v4 heal", "v5 model+heal", "v6 panel", "v7 preempt", "v8 panel+dc", "v9 snap" };
    for (int k = 0; k < NE; k++) {
        int mx; long over; double mean;
        dcs(&E[k], &mx, &over, &mean);
        printf("matched %-6s: dc max %d, %ld px beyond 60, mean |dc| %.2f; %.2f frames per step, fast steps (A2/4, A2/2) %.2f (%ld of %ld); flashes %ld frames; %llu px matched, %d runs at most\n",
               nm[k], mx, over, mean, (double)frames[k] / steps[k], fastn[k] ? (double)fast[k] / fastn[k] : 0, fastn[k], steps[k], flash[k],
               (unsigned long long)E[k].matched_px, E[k].nrun_max);
        printf("  %llu order waits, %llu pixel waits, %llu kept updates absorbed\n",
               (unsigned long long)E[k].wait_order, (unsigned long long)E[k].wait_pixel, (unsigned long long)E[k].absorbed);
        printf("  model: mean shortfall %.3f frames while slowing, %.3f before the flash (%ld px >= 3 short per fling); %llu px healed\n",
               slown ? slow_sf[k] / slown : 0, end_sf[k] / flings, end_over[k] / flings, (unsigned long long)E[k].healed_px);
        printf("  fast steps: mean shortfall %.3f frames, %ld px >= 3 short per step; %llu px preempted\n",
               fastn[k] ? fast_sf[k] / fastn[k] : 0, fastn[k] ? fast_over[k] / fastn[k] : 0, (unsigned long long)E[k].preempted);
        if (k >= 3) printf("  %s: %llu split starts; payback pixels still running at a step's end: %.0f per step (%.2f %% of the panel)\n",
                           nm[k], (unsigned long long)E[k].split_starts, 4.0 * lag[k] / steps[k], 400.0 * lag[k] / steps[k] / ((double)EPD_W * EPD_H));
    }
    /* v3 keeps the fast steps' speed too */
    if (fastn[3] && fast[3] > fast[0]) { printf("v3 slowed the fast steps (%ld vs %ld frames)\n", fast[3], fast[0]); bad++; }
    for (int k = 4; k < NE; k++)
        if (fastn[k] && fast[k] > fast[0]) { printf("%s slowed the fast steps (%ld vs %ld frames)\n", nm[k], fast[k], fast[0]); bad++; }
    if (slow_sf[4] >= slow_sf[3]) { printf("v4 did not settle cleaner than v3 (%.3f vs %.3f)\n", slow_sf[4] / slown, slow_sf[3] / slown); bad++; }
    /* v3 never waits on an earlier step's rest */
    if (E[3].wait_order) { printf("v3 waited on order %llu times\n", (unsigned long long)E[3].wait_order); bad++; }
    /* v2 keeps the fast steps' speed */
    if (fastn[2] && fast[2] != fast[0]) { printf("v2 changed the fast steps' frames (%ld vs %ld)\n", fast[2], fast[0]); bad++; }
    /* v7: preemption lets fewer pixels lag in the fast steps */
    if (fast_over[7] >= fast_over[6]) { printf("v7 lagged no less than v6 in the fast steps (%ld vs %ld px)\n", fast_over[7], fast_over[6]); bad++; }
    /* v8: the record pays back, the panel no worse */
    {
        int m7, m8; long o7, o8; double a7, a8;
        dcs(&E[7], &m7, &o7, &a7); dcs(&E[8], &m8, &o8, &a8);
        if (a8 >= a7) { printf("v8 kept no smaller a DC record than v7 (mean |dc| %.2f vs %.2f)\n", a8, a7); bad++; }
        if (fast_over[8] > fast_over[7] * 21 / 20) { printf("v8 lagged more than v7 in the fast steps (%ld vs %ld px)\n", fast_over[8], fast_over[7]); bad++; }
    }
    {
        int m8, m9; long o8, o9; double a8, a9;
        dcs(&E[8], &m8, &o8, &a8); dcs(&E[9], &m9, &o9, &a9);
        if (a9 > a8 || o9 > o8) { printf("v9 kept a larger DC record than v8 (mean |dc| %.2f vs %.2f, %ld vs %ld px beyond 60)\n", a9, a8, o9, o8); bad++; }
        printf("  v9: %llu levels snapped\n", (unsigned long long)E[9].snapped_px);
    }
    /* 3: partial GC16 cleanups (no flash): v8 vs v9 */
    {
        static epd_engine P[2];
        for (int k = 0; k < 2; k++) {
            epd_init(&P[k], w, n, 24);
            epd_set_matched(&P[k], 1); epd_set_model(&P[k], 2, 1); epd_set_matched_rule(&P[k], 2, 3);
            epd_set_nonblock(&P[k], 2); epd_set_heal(&P[k], 2, 2);
            memset(scr, 255, (size_t)EPD_W * EPD_H);
            epd_submit(&P[k], all, EPD_GC16, 1, scr, EPD_W);
            drain(&P[k]);
            memset(P[k].dc, 0, sizeof P[k].dc);
        }
        epd_set_snap(&P[1], 1);
        double sf[2] = { 0 };
        long sh[2] = { 0 };
        int pos2 = 0, dir2 = 1;
        srand(23);
        for (int fl = 0; fl < flings; fl++) {
            double v = 40 + rand() % 160;
            while (v >= 6) {
                pos2 += dir2 * (int)v;
                if (pos2 < 0) { pos2 = 0; dir2 = 1; }
                if (pos2 > PAGE_H - EPD_H) { pos2 = PAGE_H - EPD_H; dir2 = -1; }
                memcpy(scr, page + (size_t)pos2 * EPD_W, (size_t)EPD_W * EPD_H);
                for (int k = 0; k < 2; k++) {
                    epd_submit(&P[k], all, EPD_MODE_A2LEN(EPD_A2_OR_DU, 2), 0, scr, EPD_W);
                    step_paced(&P[k], NULL);
                }
                v *= 0.85;
            }
            for (int k = 0; k < 2; k++) {
                drain(&P[k]);
                epd_submit(&P[k], all, EPD_GC16, fl & 1, scr, EPD_W);    /* partial / flash in turn */
                drain(&P[k]);
                double m; long o;
                epd_op_stats(&P[k], 1, &m, &o);
                sf[k] += m; sh[k] += o;
            }
            if (rand() % 3 == 0) dir2 = -dir2;
        }
        int m[2]; long o[2]; double a[2];
        for (int k = 0; k < 2; k++) dcs(&P[k], &m[k], &o[k], &a[k]);
        for (int k = 0; k < 2; k++)
            printf("device setting, cleanups %s: dc max %d, %ld px beyond 60, mean |dc| %.2f; after each cleanup: mean shortfall %.3f frames, %ld px >= 1 short; %llu snapped, %llu healed\n",
                   k ? "v9 snap" : "v8", m[k], o[k], a[k], sf[k] / flings, sh[k] / flings,
                   (unsigned long long)P[k].snapped_px, (unsigned long long)P[k].healed_px);
        if (a[1] > a[0] * 1.1 || o[1] > o[0]) { printf("device setting: v9's record grew more than 10 %% over v8's\n"); bad++; }
        if (sh[1] >= sh[0]) { printf("device setting: v9 left no fewer pixels short than v8 (%ld vs %ld)\n", sh[1], sh[0]); bad++; }
    }
    /* 4: the short arrival */
    {
        static epd_engine Q[2];
        int d[2];
        epd_rect px = { 100, 100, 8, 8 };
        for (int k = 0; k < 2; k++) {
            epd_init(&Q[k], w, n, 24);
            epd_set_matched(&Q[k], 1); epd_set_model(&Q[k], 2, 1); epd_set_matched_rule(&Q[k], 2, 3);
            epd_set_nonblock(&Q[k], 2); epd_set_heal(&Q[k], 2, 2); epd_set_snap(&Q[k], k);
            memset(scr, 255, (size_t)EPD_W * EPD_H);
            epd_submit(&Q[k], all, EPD_GC16, 1, scr, EPD_W);
            drain(&Q[k]);
            memset(Q[k].dc, 0, sizeof Q[k].dc);
            epd_set_matched(&Q[k], 0);      /* (an A2/2 that nothing lengthens: as a fast fling's) */
            memset(scr, 0, (size_t)EPD_W * EPD_H);
            epd_submit(&Q[k], px, EPD_MODE_A2LEN(EPD_A2, 2), 0, scr, EPD_W);
            drain(&Q[k]);
            memset(scr, 255, (size_t)EPD_W * EPD_H);
            epd_submit(&Q[k], px, EPD_GC16, 0, scr, EPD_W);
            drain(&Q[k]);
            d[k] = Q[k].dc[(size_t)103 * EPD_W + 103];
        }
        printf("short arrival (A2/2 to black, partial GC16 back): dc %+d off, %+d snapped\n", d[0], d[1]);
        if (d[0] < 15 || d[1] < -2 || d[1] > 2) { printf("short arrival: expected about +20 off and 0 snapped\n"); bad++; }
    }
    printf("levels after the flashes: %s\n", lev_same ? "equal" : "DIFFERENT");
    bad += !lev_same;
    printf(bad ? "epd-matched: FAILED\n" : "epd-matched: ok\n");
    return bad != 0;
}
