/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-handover-test — the stagger -> engine hand-back against a panel
 * simulator: every drive frame the stagger (CPU reference) and then the
 * engine produce is integrated per pixel (phase 1 = one pulse toward black,
 * 2 = toward white, clamped to the model's range R), and at the end every
 * pixel the engine believes is at a rail (level 0 or 30, at its target, not
 * busy) must be at that rail in the simulator.
 *
 * epd-handover-test <eink_waveform.wbf>
 *
 * Chromium's URL bar came out inverted on the panel
 * while sway's content, epdd's panel buffer and the engine's levels all said
 * the target was reached; each time a near-full GC16 ("mode base 2 len 0
 * full") had ended a stagger run over it. The stream's sequence for that,
 * from epd-drm.c (stag_take / stag_frame / stag_advance / stag_exit), is
 * replayed here with the parameters (ctl 1, pay 3, A2/2: n 1, G 2),
 * then (stage 0) with the daily config of the
 * engine with glassine-epdd-desktop's payback defaults (matched, rule 2 cap 3, preempt,
 * model 2, heal 2/2, snap) and the stagger as epd-drm.c's stag_enter makes it
 * (ctl 3, K 8, payrule with the engine's matched_cap 3, hammer_top 2,
 * stripes 64, dither 0), over a scroll, a window and a static page.
 * The simulator is crude (no greys, no waveform shape): it only catches
 * gross errors -- a pixel at the wrong rail, i.e. an inversion. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../epd-engine.h"
#include "../epd-stagger.h"

static epd_engine E;
static epd_stag S;
static uint32_t W[EPD_WORDS], F[EPD_WORDS];
static int8_t P[EPD_W * EPD_H];         /* the simulated panel: 0 black .. R white */
static int R;
static long bad;

#define FAIL(...) do { printf("FAIL: " __VA_ARGS__); printf("\n"); bad++; } while (0)

static uint8_t *slurp(const char *p, size_t *n)
{
    FILE *f = fopen(p, "rb");
    if (!f) { perror(p); exit(2); }
    fseek(f, 0, SEEK_END); *n = (size_t)ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t *d = malloc(*n);
    if (fread(d, 1, *n, f) != *n) exit(2);
    fclose(f);
    return d;
}

static int phase_at(const uint32_t *w, int x, int y)
{
    size_t wo = (size_t)(EPD_DATA0 / EPD_LINE + EPD_H - 1 - y) * EPD_LINE + EPD_DATA0 % EPD_LINE + x / 8;
    return (int)((w[wo] >> (14 - 2 * (x % 8))) & 3);
}

static long frames_in, phase3;
static void integrate(const uint32_t *w)
{
    for (int y = 0; y < EPD_H; y++)
        for (int x = 0; x < EPD_W; x++) {
            int ph = phase_at(w, x, y);
            int8_t *p = &P[(size_t)y * EPD_W + x];
            if (ph == 1 && *p > 0) (*p)--;
            else if (ph == 2 && *p < R) (*p)++;
            else if (ph == 3) phase3++;
        }
    frames_in++;
}

/* a text page scrolled by off rows: black dashes on white (levels 0 / 30) */
static void page(uint8_t *lv, epd_rect r, int off)
{
    for (int y = 0; y < r.h; y++)
        for (int x = 0; x < r.w; x++) {
            int yy = (y + off) % 48, xx = x % 40;
            lv[(size_t)y * r.w + x] = (yy >= 20 && yy < 30 && xx < 28) ? 0 : 30;
        }
}

static int fdirty[2];                   /* F's rows the last engine frame drove (the stream's fdirty) */
static void engine_until_idle(void)
{
    int guard = 0;
    while (epd_schedule(&E) && guard++ < 2000) {
        epd_render_begin(&E, F, fdirty);
        epd_render_rows(&E, F, 0, EPD_H);
        integrate(F);
        epd_advance(&E, NULL, NULL);
    }
}

/* the check: the engine's panel model (op: drive frames from black, 0..R)
 * against the simulator, every pixel. More than R/2 apart = the panel is on
 * the other side of where the engine thinks: an inversion. (Small gaps are
 * the simulator's crudeness: no waveform shape) */
static void check(const char *name)
{
    long n = 0, inv = 0, near = 0;
    int ex = -1, ey = -1, dmax = 0;
    for (int y = 0; y < EPD_H; y++)
        for (int x = 0; x < EPD_W; x++) {
            size_t o = (size_t)y * EPD_W + x;
            int d = P[o] - E.op[o];
            if (d < 0) d = -d;
            n++;
            if (d > dmax) dmax = d;
            if (d > R / 2) { inv++; if (ex < 0) { ex = x; ey = y; } }
            else if (d > 1) near++;
        }
    if (inv)
        FAIL("%s: %ld of %ld pixels INVERTED against the engine's panel model (first %d,%d: model %d, simulator %d of %d, level %d)",
             name, inv, n, ex, ey, E.op[(size_t)ey * EPD_W + ex], P[(size_t)ey * EPD_W + ex], R, E.level[(size_t)ey * EPD_W + ex]);
    else printf("%s: ok (%ld frames; %ld pixels 2..%d drive frames off the model, max %d)\n", name, frames_in, near, R / 2, dmax);
}

static void reset_white(void)
{
    epd_assume(&E, 30);
    memset(E.dc, 0, EPD_NPX * sizeof *E.dc);
    memset(P, R, sizeof P);
    frames_in = phase3 = 0;
}

/* a scroll in the stagger, then a GC16 over most of the screen ends it:
 * gc_off = the GC16's content (page offset); late = cut steps that come in
 * during the drain (the engine queues them behind the GC16). daily: the
 * stagger's daily params (ctl and pay ignored); kind 0 the scrolled band,
 * 1 a window (500 x 380) scrolling, 2 static (the band's page once) */
static int daily, kind;
static void scenario(const char *name, int ctl, int pay, int nframes, int gc_off, int late, int full)
{
    reset_white();
    epd_rect r = { 0, 0, EPD_W, 600 };          /* the scrolled area */
    if (kind == 1) r = (epd_rect){ 300, 400, 500, 380 };
    uint8_t *lv = malloc((size_t)r.w * r.h);
    epd_stag_par p = { .G = 2, .n = 1, .ctl = ctl, .pay = pay, .K = 8, .dither = 1, .R = R };
    if (daily)                                  /* (epd-drm.c stag_enter, the daily knobs) */
        p = (epd_stag_par){ .G = 2, .n = 1, .ctl = 3, .K = 8, .dither = 0, .R = R, .payrule = 1,
                            .pcap = E.matched_cap > 0 && E.matched_cap <= EPD_STAG_PCAP ? E.matched_cap : EPD_STAG_PCAP,
                            .htop = 2, .stripe = 64 };
    epd_stag_enter(&S);
    S.f = 3;
    epd_stag_load(&S, &E, &p, r);               /* stag_enter: the first cut's rect */
    page(lv, r, 0);
    epd_stag_target(&S, r, lv);
    int nmax = p.n + (p.ctl >= 2);
    /* STG_ON: a new target every second frame (sway's ~42 Hz into 85 Hz) */
    for (int k = 0; k < nframes; k++) {
        if (k && k % 2 == 0 && kind != 2) {
            page(lv, r, k * 4);
            epd_stag_load(&S, &E, &p, r);
            epd_stag_target(&S, r, lv);
        }
        epd_stag_par q = p;
        q.G = q.n + 1;
        epd_stag_rows(&S, &q, E.tpl, W, 0, EPD_H);
        integrate(W);
        epd_stag_advance(&S);
    }
    /* stag_take: a non-cut update -> DRAIN (drain_left = nmax); the engine queues it */
    epd_rect g = { 0, 0, EPD_W, EPD_H - 60 };
    uint8_t *grey = malloc((size_t)EPD_W * EPD_H);
    memset(grey, 255, (size_t)EPD_W * EPD_H);
    page(lv, r, gc_off);
    for (int y = 0; y < r.h; y++)
        for (int x = 0; x < r.w; x++) grey[(size_t)(r.y + y) * EPD_W + r.x + x] = lv[(size_t)y * r.w + x] ? 255 : 0;
    epd_submit(&E, g, EPD_GC16, full, grey, EPD_W);
    if (late) {                                 /* draining: cut steps go to the engine's queue */
        page(lv, r, gc_off + 4);
        for (int y = 0; y < r.h; y++)
            for (int x = 0; x < r.w; x++) grey[(size_t)(r.y + y) * EPD_W + r.x + x] = lv[(size_t)y * r.w + x] ? 255 : 0;
        epd_submit(&E, r, EPD_MODE_A2LEN(EPD_A2, 2), 0, grey, EPD_W);
    }
    for (int left = nmax; left > 0; left--) {   /* stag_frame (drain) + stag_advance */
        epd_stag_par q = p;
        q.G = q.n + 1;
        q.drain = 1;
        epd_stag_rows(&S, &q, E.tpl, W, 0, EPD_H);
        integrate(W);
        epd_stag_advance(&S);
    }
    long act = epd_stag_active(&S);
    epd_stag_exit(&S, &E);                      /* stag_exit */
    engine_until_idle();
    char nm[160];
    if (daily) snprintf(nm, sizeof nm, "daily %s: %s (%ld still active at the hand-back)", kind == 1 ? "window" : kind == 2 ? "static" : "scroll", name, act);
    else snprintf(nm, sizeof nm, "%s (ctl %d, %ld still active at the hand-back)", name, ctl, act);
    check(nm);
    free(lv); free(grey);
}

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s <eink_waveform.wbf>\n", argv[0]); return 2; }
    size_t n;
    uint8_t *wbf = slurp(argv[1], &n);
    if (epd_init(&E, wbf, n, 24)) { fprintf(stderr, "bad waveform\n"); return 2; }
    epd_set_model(&E, 0, 1);
    epd_assume(&E, 30);
    epd_submit(&E, (epd_rect){ 0, 0, 8, 8 }, EPD_GC16, 1, (uint8_t[64]){ 0 }, 8);   /* (op_prep: op_r) */
    while (epd_schedule(&E)) epd_advance(&E, NULL, NULL);
    R = E.op_r ? E.op_r : 9;
    printf("model range R %d\n", R);
    if (epd_stag_init(&S)) return 2;
    memcpy(W, E.tpl, sizeof W);
    memcpy(F, E.tpl, sizeof F);
    /* the baseline: the engine alone (a GC16 of the page from white) */
    reset_white();
    {
        uint8_t *grey = malloc((size_t)EPD_W * EPD_H), lv[EPD_W];
        memset(grey, 255, (size_t)EPD_W * EPD_H);
        for (int y = 0; y < 600; y++) {
            page(lv, (epd_rect){ 0, y, EPD_W, 1 }, y);
            for (int x = 0; x < EPD_W; x++) grey[(size_t)y * EPD_W + x] = lv[x] ? 255 : 0;
        }
        epd_submit(&E, (epd_rect){ 0, 0, EPD_W, EPD_H - 60 }, EPD_GC16, 1, grey, EPD_W);
        engine_until_idle();
        check("engine GC16 alone");
        free(grey);
    }
    for (int ctl = 0; ctl <= 3; ctl++) {
        if (ctl == 2) continue;                 /* (not recommended, not used) */
        scenario("GC16 of the last target", ctl, 3, 40, 76, 0, 1);
        scenario("GC16 of a newer frame", ctl, 3, 40, 84, 0, 1);
        scenario("GC16 + a cut step during the drain", ctl, 3, 40, 84, 1, 1);
        scenario("GC16 mid-step (odd frame count)", ctl, 3, 41, 80, 0, 1);
        scenario("partial GC16 (not full)", ctl, 3, 40, 84, 0, 0);
    }
    /* the daily config: the engine as glassine-epdd-desktop sets it (payback=on), the
     * stagger as stag_enter (above); last: the engine keeps these settings */
    epd_set_matched(&E, 1);
    epd_set_matched_rule(&E, 2, 3);
    epd_set_nonblock(&E, 2);
    epd_set_model(&E, 2, 0);
    epd_set_heal(&E, 2, 2);
    epd_set_snap(&E, 1);
    daily = 1;
    for (kind = 0; kind <= 2; kind++) {
        scenario("GC16 of the last target", 3, 0, 40, 76, 0, 1);
        scenario("GC16 of a newer frame", 3, 0, 40, 84, 0, 1);
        scenario("GC16 + a cut step during the drain", 3, 0, 40, 84, 1, 1);
        scenario("GC16 mid-step (odd frame count)", 3, 0, 41, 80, 0, 1);
        scenario("partial GC16 (not full)", 3, 0, 40, 84, 0, 0);
    }
    if (phase3) printf("note: %ld pixel-frames with phase 3 (not integrated)\n", phase3);
    printf("%s\n", bad ? "FAILED" : "all ok");
    return bad ? 1 : 0;
}
