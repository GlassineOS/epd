/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* the fold (epd_submit): a new update takes in the pending ones it overlaps
 * that have not started, so at most one update per area waits. Against the
 * queue it replaces: the panel ends as the queue would have left it (per
 * pixel the newest update covering it), the mode is the strongest, running
 * updates are left alone, what can't fold keeps its place.
 * epd-fold-test <wbf> [payback]   (payback: glassine-epdd-desktop's defaults) */
#include "epd-engine.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL " __VA_ARGS__); printf("\n"); } else { printf("ok   " __VA_ARGS__); printf("\n"); } } while (0)

static uint8_t *wbf;
static long wbfn;
static int payback;
static uint8_t img[EPD_NPX], img2[EPD_NPX];     /* the frame buffers (shadows) */
static uint8_t expect[EPD_NPX];                 /* the queue's end state: the newest update's level per pixel */
static uint8_t lv[EPD_NPX];

static epd_engine *fresh(void)
{
    static epd_engine *e;
    if (e) { epd_free(e); free(e); }
    e = calloc(1, sizeof *e);
    if (epd_init(e, wbf, wbfn, 24)) { printf("FAIL wbf\n"); exit(1); }
    if (payback) {
        epd_set_matched(e, 1); epd_set_matched_rule(e, 2, 3); epd_set_nonblock(e, 2);
        epd_set_model(e, 2, 0); epd_set_heal(e, 2, 2); epd_set_snap(e, 1);
    }
    memset(expect, 30, sizeof expect);         /* (epd_init assumes white) */
    return e;
}
static void drain(epd_engine *e) { for (int k = 0; k < 100000 && (epd_schedule(e) || !epd_idle(e)); k++) epd_advance(e, NULL, NULL); }
static void frame(epd_engine *e) { epd_schedule(e); epd_advance(e, NULL, NULL); }
/* a picture k: black, two greys, white in 8 px squares, every pixel changes from k to k + 1 */
static void paint(uint8_t *b, int k)
{
    static const uint8_t v[4] = { 0, 96, 192, 255 };
    for (int y = 0; y < EPD_H; y++)
        for (int x = 0; x < EPD_W; x++) b[(size_t)y * EPD_W + x] = v[((x >> 3) + (y >> 3) + k) & 3];
}
static uint32_t sub(epd_engine *e, epd_rect r, int mode, int full, const uint8_t *b)
{
    for (int y = r.y; y < r.y + r.h; y++)
        for (int x = r.x; x < r.x + r.w; x++) expect[(size_t)y * EPD_W + x] = (b[(size_t)y * EPD_W + x] & 0xF0) >> 3;
    return epd_submit(e, r, mode, full, b, EPD_W);
}
static long differ(epd_engine *e)
{
    epd_state_get(e, lv, NULL, NULL, NULL, NULL, NULL);
    long n = 0;
    for (size_t i = 0; i < EPD_NPX; i++) n += lv[i] != expect[i];
    return n;
}
static int overlapping(epd_rect a, epd_rect b) { return a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h; }
/* the pending ones overlapping r */
static int pend_over(const epd_engine *e, epd_rect r)
{
    int n = 0;
    for (int i = 0; i < e->npend; i++) n += overlapping(e->pend[i].r, r) && !e->pend[i].kept;
    return n;
}

/* GPU targets (epd_submit_gt): levels kept per handle, fetched by the CPU's claim */
#define NGT 8
static struct { epd_rect r; uint8_t *lv; int live, fetched; } gts[NGT + 1];
static int gt_release_bad;
static void gt_release(void *c, int gt) { (void)c; if (gt < 1 || gt > NGT || !gts[gt].live) gt_release_bad++; else gts[gt].live = 0; }
static int gt_fetch(void *c, int gt, epd_rect r, uint8_t *out)
{
    (void)c;
    if (gt < 1 || gt > NGT || r.w != gts[gt].r.w || r.h != gts[gt].r.h) return -1;
    memcpy(out, gts[gt].lv, (size_t)r.w * r.h);
    gts[gt].fetched++;
    return 0;
}
static const epd_target_ops gt_ops = { gt_release, gt_fetch, NULL };
static uint32_t sub_gt(epd_engine *e, int h, epd_rect r, int mode, const uint8_t *b)
{
    free(gts[h].lv);
    gts[h].r = r; gts[h].live = 1; gts[h].fetched = 0;
    gts[h].lv = malloc((size_t)r.w * r.h);
    for (int y = 0; y < r.h; y++)
        for (int x = 0; x < r.w; x++) {
            size_t i = (size_t)(r.y + y) * EPD_W + r.x + x;
            gts[h].lv[(size_t)y * r.w + x] = expect[i] = (b[i] & 0xF0) >> 3;
        }
    return epd_submit_gt(e, r, mode, 0, h, b, EPD_W);
}

int main(int c, char **v)
{
    if (c < 2) return 2;
    payback = c > 2 && !strcmp(v[2], "payback");
    FILE *f = fopen(v[1], "rb");
    if (!f) return 2;
    fseek(f, 0, 2); wbfn = ftell(f); rewind(f);
    wbf = malloc(wbfn);
    if (fread(wbf, 1, wbfn, f) != (size_t)wbfn) return 2;
    fclose(f);
    printf("epd-fold (%s)\n", payback ? "payback on" : "payback off");
    const epd_rect all = { 0, 0, EPD_W, EPD_H };

    /* 1: a scroll's stream of overlapping DU updates (the rects' x and width
     * changing, full height) every 4 frames while a DU runs: one waits */
    for (int a2 = 0; a2 < 2; a2++) {
        const int mode = a2 ? EPD_MODE_A2LEN(EPD_A2_OR_DU, 2) : EPD_DU;
        const char *mn = a2 ? "cut A2/2 (A2_OR_DU)" : "DU";
        epd_engine *e = fresh();
        int k = 0;
        paint(img, k++);
        sub(e, all, EPD_GC16, 1, img); drain(e);
        paint(img, k++);
        epd_rect r0 = { 0, 0, 1100, EPD_H };
        uint32_t id0 = sub(e, r0, EPD_DU, 0, img);
        epd_schedule(e);
        int s0 = -1;
        for (int s = 0; s < EPD_MAX_UPD; s++) if (e->run[s].nframes && e->run[s].id == id0) s0 = s;
        CHECK(s0 >= 0 && e->npend == 0, "%s 1: the first update runs", mn);
        if (s0 < 0) continue;
        epd_run run0 = e->run[s0];
        static uint8_t tg0[EPD_NPX];
        epd_state_get(e, NULL, tg0, NULL, NULL, NULL, NULL);
        static uint8_t tg[EPD_NPX];
        int maxover = 0, altered = 0, steps = 0;   /* maxover: waiting, not started (kept: started in part) */
        uint64_t m0 = e->merged + e->absorbed + e->superseded;
        static const int ws[] = { 1064, 1352, 1064, 1516 };
        for (int n = 0; n < 24; n++) {
            paint(img, k++);
            int x = (n % 5) * 3;
            sub(e, (epd_rect){ x, 0, ws[n % 4] - x, EPD_H }, mode, 0, img);
            if (pend_over(e, all) > maxover) maxover = pend_over(e, all);
            for (int j = 0; j < 4; j++) {
                frame(e);
                if (e->run[s0].nframes && e->run[s0].id == id0) {   /* the first one still runs: untouched */
                    steps++;
                    const epd_run *u = &e->run[s0];
                    altered += u->r.x != run0.r.x || u->r.y != run0.r.y || u->r.w != run0.r.w || u->r.h != run0.r.h ||
                               u->mode != run0.mode || u->full != run0.full || u->nframes != run0.nframes || u->lut != run0.lut;
                    epd_state_get(e, NULL, tg, NULL, NULL, NULL, NULL);
                    for (int y = 0; y < EPD_H; y++)
                        altered += memcmp(tg + (size_t)y * EPD_W, tg0 + (size_t)y * EPD_W, (size_t)r0.w) != 0;
                }
            }
        }
        uint64_t m1 = e->merged + e->absorbed + e->superseded;
        paint(img, k++);
        sub(e, (epd_rect){ 0, 0, 1516, EPD_H }, mode, 0, img);    /* (the absorb shows the newest image over its union) */
        drain(e);
        long d = differ(e);
        CHECK(maxover <= 1, "%s 1: 24 overlapping updates, at most %d pending (<= 1)", mn, maxover);
        CHECK(m1 - m0 >= 12, "%s 1: %llu taken in (merged %llu absorbed %llu superseded %llu)", mn,
              (unsigned long long)(m1 - m0), (unsigned long long)e->merged, (unsigned long long)e->absorbed,
              (unsigned long long)e->superseded);
        CHECK(steps > 0 && !altered, "%s 1: the running update untouched over %d frames (%d changes)", mn, steps, altered);
        CHECK(d == 0, "%s 1: drained, the panel = the queue's end (%ld px differ)", mn, d);
        long dl = 0;                            /* the last rect: the last image */
        for (int y = 0; y < EPD_H; y++)
            for (int x = 0; x < 1516; x++) dl += lv[(size_t)y * EPD_W + x] != (img[(size_t)y * EPD_W + x] & 0xF0) >> 3;
        CHECK(dl == 0, "%s 1: the last image shown (%ld px differ)", mn, dl);
    }

    /* 2: modes and flags behind a running update: DU then GC16 -> one GC16 */
    {
        static const struct { int m1, f1, m2, f2, want, wfull, wflash; const char *what; } t[] = {
            { EPD_DU, 0, EPD_GC16, 0, EPD_GC16, 0, 0, "DU then GC16 -> GC16" },
            { EPD_GC16, 0, EPD_DU, 0, EPD_GC16, 0, 0, "GC16 then DU -> GC16" },
            { EPD_GC16, 2, EPD_DU, 0, EPD_GC16, 1, 1, "GC16 flash then DU -> GC16 flash" },
            { EPD_DU, 1, EPD_GC16, 2, EPD_GC16, 1, 0, "DU full then GC16 flash -> GC16 full" },
            { EPD_DU4, 0, EPD_DU, 0, EPD_DU4, 0, 0, "DU4 then DU -> DU4" },
            { EPD_MODE_A2LEN(EPD_A2_OR_DU, 2), 0, EPD_DU, 0, EPD_DU, 0, 0, "cut A2 then DU -> DU" },
            { EPD_DU, 0, EPD_MODE_A2LEN(EPD_A2_OR_DU, 2), 0, EPD_DU, 0, 0, "DU then cut A2 -> DU" },
            { EPD_MODE_A2LEN(EPD_A2, 6), 0, EPD_MODE_A2LEN(EPD_A2_OR_DU, 2), 0, EPD_MODE_A2LEN(EPD_A2_OR_DU, 2), 0, 0,
              "A2/6 then A2_OR_DU/2 -> A2_OR_DU/2" },
            { EPD_MODE_A2LEN(EPD_A2_OR_DU, 0), 0, EPD_MODE_A2LEN(EPD_A2_OR_DU, 4), 0, EPD_MODE_A2LEN(EPD_A2_OR_DU, 4), 0, 0,
              "two A2_OR_DU steps -> the newest length" },
        };
        for (size_t i = 0; i < sizeof t / sizeof *t; i++) {
            epd_engine *e = fresh();
            paint(img, 0); sub(e, all, EPD_GC16, 1, img); drain(e);
            paint(img, 1); sub(e, (epd_rect){ 0, 0, 1200, EPD_H }, EPD_GC16, 0, img); epd_schedule(e);
            paint(img, 2); sub(e, (epd_rect){ 100, 0, 900, EPD_H }, t[i].m1, t[i].f1, img);
            uint64_t ab = e->absorbed;
            paint(img, 3); sub(e, (epd_rect){ 500, 0, 900, EPD_H }, t[i].m2, t[i].f2, img);
            if (e->absorbed != ab || e->npend == 1)     /* (one buffer: the newest image over the union) */
                for (int y = 0; y < EPD_H; y++)
                    for (int x = 100; x < 1400; x++) expect[(size_t)y * EPD_W + x] = (img[(size_t)y * EPD_W + x] & 0xF0) >> 3;
            const epd_pend *p = &e->pend[e->npend - 1];
            CHECK(e->npend == 1 && p->mode == t[i].want && p->full == t[i].wfull && p->flash == t[i].wflash &&
                  p->r.x == 100 && p->r.w == 1300 && p->r.y == 0 && p->r.h == EPD_H,
                  "2: %s (pending %d: mode %d|%d full %d flash %d, %d,%d %dx%d)", t[i].what, e->npend, EPD_MODE_BASE(p->mode),
                  p->mode >> 8, p->full, p->flash, p->r.x, p->r.y, p->r.w, p->r.h);
            drain(e);
            long d = differ(e);
            CHECK(d == 0, "2: %s: drained, %ld px differ", t[i].what, d);
        }
    }

    /* 3: what stays apart: rects that don't overlap; rects that meet at a
     * corner (their box mostly neither's); a pending of another buffer with
     * a gap; a pending whose content is on the GPU only */
    {
        epd_engine *e = fresh();
        paint(img, 0); sub(e, all, EPD_GC16, 1, img); drain(e);
        paint(img, 1); sub(e, all, EPD_DU, 0, img); epd_schedule(e);           /* (runs: everything waits) */
        paint(img, 2);
        uint64_t m0 = e->merged;
        sub(e, (epd_rect){ 0, 0, 500, 500 }, EPD_DU, 0, img);
        sub(e, (epd_rect){ 800, 0, 500, 500 }, EPD_DU, 0, img);
        CHECK(e->npend == 2 && e->merged == m0, "3: two apart: %d pending, %llu merged", e->npend, (unsigned long long)(e->merged - m0));
        sub(e, (epd_rect){ 300, 300, 400, 600 }, EPD_DU, 0, img);              /* box 700 x 900, the rects 1/2 of it */
        CHECK(e->npend == 3 && e->merged == m0, "3: corners apart: %d pending, %llu merged", e->npend, (unsigned long long)(e->merged - m0));
        drain(e);
        long d = differ(e);
        CHECK(d == 0, "3: drained, %ld px differ", d);

        /* another buffer, no gap: folded, each pixel its own update's content
         * (the older one's where the new rect doesn't reach: not the new buffer's) */
        e = fresh();
        paint(img, 0); sub(e, all, EPD_GC16, 1, img); drain(e);
        paint(img, 1); sub(e, all, EPD_DU, 0, img); epd_schedule(e);
        paint(img2, 5); sub(e, (epd_rect){ 0, 0, 600, 800 }, EPD_DU, 0, img2);
        paint(img, 6); sub(e, (epd_rect){ 300, 0, 600, 800 }, EPD_DU, 0, img);
        CHECK(e->npend == 1 && e->pend[0].r.x == 0 && e->pend[0].r.w == 900 && e->pend[0].r.h == 800,
              "3: another buffer, no gap: folded (%d pending, %d,%d %dx%d)", e->npend, e->pend[0].r.x, e->pend[0].r.y,
              e->pend[0].r.w, e->pend[0].r.h);
        /* a gap (the box's corners no rect covers, 1/8 of it at most): only of one buffer */
        sub(e, (epd_rect){ 1000, 0, 300, 300 }, EPD_DU, 0, img2);
        sub(e, (epd_rect){ 1100, 100, 300, 300 }, EPD_DU, 0, img);
        CHECK(e->npend == 3, "3: another buffer, a gap: apart (%d pending)", e->npend);
        sub(e, (epd_rect){ 1500, 0, 300, 300 }, EPD_DU, 0, img);
        sub(e, (epd_rect){ 1550, 50, 300, 300 }, EPD_DU, 0, img);
        const epd_pend *p = &e->pend[e->npend - 1];
        CHECK(e->npend == 4 && p->r.x == 1500 && p->r.y == 0 && p->r.w == 350 && p->r.h == 350,
              "3: one buffer, a gap of 1/25: folded (%d pending, %d,%d %dx%d)", e->npend, p->r.x, p->r.y, p->r.w, p->r.h);
        for (int y = 0; y < 350; y++)           /* (the gap: the buffer's newest image) */
            for (int x = 1500; x < 1850; x++)
                if ((x < 1550 && y >= 300) || (x >= 1800 && y < 50)) expect[(size_t)y * EPD_W + x] = (img[(size_t)y * EPD_W + x] & 0xF0) >> 3;
        drain(e);
        d = differ(e);
        CHECK(d == 0, "3: drained, %ld px differ", d);
    }

    /* 4: the order: a pending that can't fold (another buffer's, its content on
     * the GPU, not covered by the new rect) between an older one and the new
     * one, over the box: nothing folds (else the older content would land on top of it) */
    {
        epd_engine *e = fresh();
        epd_set_target_ops(e, &gt_ops);
        paint(img, 0); sub(e, all, EPD_GC16, 1, img); drain(e);
        paint(img, 1); sub(e, all, EPD_DU, 0, img); epd_schedule(e);
        paint(img, 2); sub(e, (epd_rect){ 0, 0, 800, EPD_H }, EPD_DU, 0, img);
        paint(img2, 3); sub_gt(e, 1, (epd_rect){ 600, 0, 400, EPD_H }, EPD_DU, img2);
        paint(img, 4); sub(e, (epd_rect){ 0, 0, 700, EPD_H }, EPD_DU, 0, img);
        CHECK(e->npend == 3, "4: a GPU-only pending over the box outside the new rect: no fold (%d pending)", e->npend);
        drain(e);
        long d = differ(e);
        CHECK(d == 0, "4: drained, the queue's order kept (%ld px differ)", d);
        /* covered whole by the new rect: dropped unfetched */
        paint(img, 5); sub(e, all, EPD_DU, 0, img); epd_schedule(e);
        paint(img, 6); sub_gt(e, 2, (epd_rect){ 600, 0, 400, EPD_H }, EPD_GC16, img);
        paint(img, 7); sub(e, (epd_rect){ 0, 0, 1400, EPD_H }, EPD_DU, 0, img);
        CHECK(e->npend == 1 && EPD_MODE_BASE(e->pend[0].mode) == EPD_GC16 && !gts[2].fetched && !gts[2].live,
              "4: a GPU-only pending inside the new rect: folded unfetched, released (%d pending, mode %d)", e->npend,
              EPD_MODE_BASE(e->pend[0].mode));
        /* the new one on the GPU only, an older CPU pending inside it: folded, the new handle kept */
        paint(img, 8); sub(e, (epd_rect){ 1500, 0, 200, 300 }, EPD_DU, 0, img);
        sub(e, (epd_rect){ 1500, 0, 200, 300 }, EPD_DU, 0, img);               /* (superseded: same mode) */
        paint(img, 9); sub(e, (epd_rect){ 1550, 0, 100, 200 }, EPD_DU, 0, img);
        paint(img, 10); sub_gt(e, 3, (epd_rect){ 1450, 0, 300, 400 }, EPD_DU, img);
        const epd_pend *p = &e->pend[e->npend - 1];
        CHECK(e->npend == 2 && p->gt == 3 && !p->target && p->r.x == 1450 && p->r.w == 300,
              "4: the new one GPU-only, the older inside it: folded, its handle kept (%d pending, gt %d)", e->npend, p->gt);
        drain(e);
        d = differ(e);
        CHECK(d == 0, "4: drained, %ld px differ", d);
        CHECK(!gt_release_bad && !gts[1].live && !gts[2].live && !gts[3].live, "4: every GPU target released once (%d bad)", gt_release_bad);
    }

    /* 5: a pen stroke's touching pieces (the merge, the absorb, the fold) behind a running update */
    {
        epd_engine *e = fresh();
        paint(img, 0); sub(e, all, EPD_GC16, 1, img); drain(e);
        paint(img, 1); sub(e, all, EPD_DU, 0, img); epd_schedule(e);
        paint(img, 2);
        for (int n = 0; n < 20; n++) {          /* (ink: each piece black, the buffer otherwise as it was) */
            epd_rect r = { 400 + n * 20, 600, 30, 30 };
            for (int y = r.y; y < r.y + r.h; y++) memset(img + (size_t)y * EPD_W + r.x, 0, (size_t)r.w);
            sub(e, r, EPD_MODE_A2LEN(EPD_A2_OR_DU, 0), 0, img);
            frame(e);
        }
        int pp = pend_over(e, (epd_rect){ 400, 600, 430, 30 });
        CHECK(pp <= 1, "5: 20 touching pieces: %d pending", pp);
        drain(e);
        long d = differ(e);
        CHECK(d == 0, "5: drained, %ld px differ", d);
    }
    printf("epd-fold: %s (%d failures)\n", fails ? "FAIL" : "ok", fails);
    return fails != 0;
}
