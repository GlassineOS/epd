/* epd-stagger-test — staggered cut steps (epd-stagger.h): what the CPU
 * reference does (the definition), then the GPU's pass against it bit for bit.
 *
 * epd-stagger-test <eink_waveform.wbf> [frames per scenario]
 *
 * Part 1 (CPU): one step per pixel (its frames, its group's start frame, the
 * idle frame after, the model and the DC record), the blue-noise dither of a
 * grey, the controller (keeps driving until the model is at the rail, pays
 * the DC record back), drain (no new steps) and the hand-back to the engine.
 * Part 2 (GPU, skipped with exit 77 where there is none): scenarios of moving
 * content (a scrolled text page, flings, page flips) with the period, the
 * step and the controller changed under way; every frame's words compared,
 * the whole state every 8th frame and at the end. Exit 0 = all as expected.
 * The CPU reference steps its rows on every core (STAG_THREADS=N: N; the
 * rows are independent, the result the same).
 * STAG_TIME_ONLY=1: only the GPU timings (a full panel loaded first; ~30 s on
 * the A509: the device benchmark; GLASSINE_STAG_DEFS picks a shader variant).
 * The "daily:" scenarios and timing rows are the device's daily config
 * (glassine-session-sway + epd-drm.c: ctl 3, K 8, @stagger_pay=panel = payrule with
 * the engine's matched_cap 3, hammer_top 2, stagger_stripe 64, dither 0; since
 * the default is stripe 0, the blue noise: "daily-bn:" below);
 * STAG_ONLY="daily:" runs those alone (the default-path load gate,
 * STAG_ONLY=daily adds "daily-bn:", the same at stripe 0 = the blue
 * noise, and "daily-blk:", the same with the stripes as blocks (stagger_stripe=blocks),
 * for 's layout question). The "" lines (STAG_TONLY=daily, STAG_STRIPE=0 for blue noise,
 * STAG_STRIPE=blocks for the blocks, and
 * GLASSINE_STAG_DEFS="#define COLDLIN 1" for the linear cold layout): pass ms per
 * frame, average and maximum. HEAPSTATE (R4, timing/test only):
 * GLASSINE_STAG_DEFS="#define COLDLIN 1\n#define HEAPSTATE 1": the linear cold (D) and the targets (T)
 * in cached heap dma-bufs (ABGR8888, ABGR16161616; the udmabuf stand-in on the host), the
 * same checks and rows ("heapst"); T written by the CPU through its map. */
#define _GNU_SOURCE                 /* (sched_setaffinity: the CPU timing pins its threads) */
#include <sched.h>
#include "heap-shim.h"             /* (HEAPSTATE: no heap = exit 78, before the open) */
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include "../epd-engine.h"
#include "../epd-stagger.h"
#include "../epd-bluenoise.h"
#ifdef EPD_GPU
#include "../epd-gpu.h"
static double now(void);
#define EPD_BP4_TV 1                    /* (bp4_rows times its visits: bp4_tv) */
#endif
#include "../epd-bp4.h"

static epd_engine E;
static epd_stag S;
static uint32_t W[EPD_WORDS];
/* STAG_CTL4=1 (part 2, lanes): the scenarios bp4 takes (G 2, payrule, a
 * 64-row layout) at stagger_ctl 4, the reference = the twin (epd-bp4.h
 * bp4_lane_rows: T4, its masks M4 of layout m4k; arms from E's D, evicts
 * into DSH; its acks in S.ack), not epd_stag_rows */
static int B4;
#ifdef EPD_GPU
static bp4_state T4;
static bp_masks M4;
static int m4k = -1;
static uint32_t *DSH;
static void b4_twin_frame(const epd_stag_par *p);
#else
static void b4_twin_frame(const epd_stag_par *p) { (void)p; }
#endif
static long bad;

#define FAIL(...) do { printf("FAIL: " __VA_ARGS__); printf("\n"); bad++; } while (0)
/* a scenario's cells (pr 4096, stripes 512 with it): log2 w - 3 at bit 13, log2 h at bit 15 */
#define SC_CELL(w, h) (4096 | (__builtin_ctz(w) - 3) << 13 | __builtin_ctz(h) << 15)
/* word bayer (stagger_stripe=wbayer): 8 x 1 cells in Bayer order, pr bit 18 */
#define SC_WBAYER (SC_CELL(8, 1) | 1 << 18)

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

/* the phase of panel pixel (x, y) in frame words w */
static int phase_at(const uint32_t *w, int x, int y)
{
    size_t wo = (size_t)(EPD_DATA0 / EPD_LINE + EPD_H - 1 - y) * EPD_LINE + EPD_DATA0 % EPD_LINE + x / 8;
    return (int)((w[wo] >> (14 - 2 * (x % 8))) & 3);
}

/* epd_stag_rows over rows [y0, y1), split over the cores */
typedef struct { const epd_stag_par *p; int y0, y1; } rows_job;
static void *rows_run(void *a)
{
    const rows_job *j = a;
    epd_stag_rows(&S, j->p, E.tpl, W, j->y0, j->y1);
    return NULL;
}
static void rows(const epd_stag_par *p, int y0, int y1)
{
    static int nt;
    if (!nt) {
        const char *v = getenv("STAG_THREADS");
        nt = v && atoi(v) > 0 ? atoi(v) : (int)sysconf(_SC_NPROCESSORS_ONLN);
        nt = nt < 1 ? 1 : nt > 16 ? 16 : nt;
    }
    if (nt == 1 || y1 - y0 < 64) { epd_stag_rows(&S, p, E.tpl, W, y0, y1); return; }
    pthread_t t[16];
    rows_job j[16];
    int up[16] = { 0 };
    for (int i = 0; i < nt; i++) {
        j[i] = (rows_job){ p, y0 + (y1 - y0) * i / nt, y0 + (y1 - y0) * (i + 1) / nt };
        if (i) up[i] = !pthread_create(&t[i], NULL, rows_run, &j[i]);
        if (i && !up[i]) rows_run(&j[i]);   /* (no thread: here) */
    }
    rows_run(&j[0]);
    for (int i = 1; i < nt; i++) if (up[i]) pthread_join(t[i], NULL);
}

static void frame(const epd_stag_par *p)
{
    if (B4) { b4_twin_frame(p); return; }
    rows(p, 0, EPD_H);
    epd_stag_advance(&S);
}

static int group(int x, int y, int G) { return epd_bn_rank[y & 63][x & 63] * G >> 12; }

/* ---- part 1 ---- */
static void one_step(void)
{
    const int R = E.op_r ? E.op_r : 9;
    epd_assume(&E, 30);
    epd_state_fill(&E, EPD_F_DC, 0);
    epd_stag_par p = { .G = 4, .n = 3, .R = R };
    epd_stag_enter(&S);
    S.f = 5;
    epd_rect r = { 100, 200, 64, 64 };
    uint8_t *lv = calloc((size_t)r.w * r.h, 1);           /* black */
    if (epd_stag_load(&S, &E, &p, r) != (long)r.w * r.h) FAIL("one step: load did not take the rect");
    epd_stag_target(&S, r, lv);
    free(lv);
    int first[64 * 64], cnt[64 * 64], last[64 * 64], other = 0, wrong = 0;
    for (int i = 0; i < 64 * 64; i++) first[i] = -1, cnt[i] = 0, last[i] = -1;
    for (int k = 0; k < 12; k++) {
        uint32_t f = S.f;
        frame(&p);
        for (int y = 0; y < r.h; y++)
            for (int x = 0; x < r.w; x++) {
                int ph = phase_at(W, r.x + x, r.y + y), i = y * 64 + x;
                if (!ph) continue;
                if (ph != 1) wrong++;
                if (first[i] < 0) first[i] = (int)f;
                if (last[i] >= 0 && last[i] != (int)f - 1) wrong++;     /* (one run) */
                last[i] = (int)f;
                cnt[i]++;
            }
        for (int y = 0; y < EPD_H; y += 7)                /* nothing outside r */
            for (int x = 0; x < EPD_W; x += 5)
                if ((x < r.x || x >= r.x + r.w || y < r.y || y >= r.y + r.h) && phase_at(W, x, y)) other++;
    }
    int late = 0, len = 0;
    for (int y = 0; y < r.h; y++)
        for (int x = 0; x < r.w; x++) {
            int i = y * 64 + x, g = group(r.x + x, r.y + y, p.G);
            if (first[i] < 0 || (first[i] - g) % p.G || first[i] - 5 >= p.G) late++;
            if (cnt[i] != p.n) len++;
        }
    if (wrong || other || late || len) FAIL("one step: wrong phase/runs %d, outside %d, wrong start %d, wrong length %d", wrong, other, late, len);
    if (epd_stag_active(&S)) FAIL("one step: %ld pixels still active", epd_stag_active(&S));
    size_t o = (size_t)(r.y + 10) * EPD_W + r.x + 10;
    if (EPD_STAG_LEVEL(S.cold[o]) != 0 || EPD_STAG_DC(S.cold[o]) != -3 || EPD_STAG_OP(S.cold[o]) != R - 3 || !EPD_STAG_STALE(S.cold[o]))
        FAIL("one step: state level %d dc %d op %d stale %d", EPD_STAG_LEVEL(S.cold[o]), EPD_STAG_DC(S.cold[o]), EPD_STAG_OP(S.cold[o]), EPD_STAG_STALE(S.cold[o]));
    /* the next step only after the idle frame: a new target right away */
    uint8_t w30[64 * 64];
    memset(w30, 30, sizeof w30);
    if (epd_stag_load(&S, &E, &p, r)) FAIL("one step: loaded twice");
    epd_stag_target(&S, r, w30);
    int seen = 0;
    for (int k = 0; k < 8; k++) { frame(&p); seen += phase_at(W, r.x + 10, r.y + 10) == 2; }
    if (seen != 3) FAIL("one step: back to white drove %d frames (3)", seen);
    /* exit */
    size_t po = (size_t)(r.y + 100) * EPD_W + r.x;            /* parked (outside r) */
    epd_px_put(&E, po, EPD_F_LEVEL | EPD_F_TARGET | EPD_F_DC, 17, 17, 0, 0, 4, 0);   /* (either claim_state) */
    epd_stag_exit(&S, &E);
    if (epd_px_level(&E, o) != 30 || epd_px_target(&E, o) != 30 || epd_px_busy(&E, o) || epd_px_dc(&E, o) != 0)
        FAIL("exit: level %d target %d busy %d dc %d", epd_px_level(&E, o), epd_px_target(&E, o), epd_px_busy(&E, o), epd_px_dc(&E, o));
    if (epd_px_level(&E, po) != 17 || epd_px_target(&E, po) != 17 || epd_px_dc(&E, po) != 4)
        FAIL("exit: a parked pixel changed: level %d target %d dc %d", epd_px_level(&E, po), epd_px_target(&E, po), epd_px_dc(&E, po));
    epd_px_put(&E, po, EPD_F_LEVEL | EPD_F_TARGET | EPD_F_DC, 30, 30, 0, 0, 0, 0);
    printf("one step: ok (G 4, n 3: starts on the group's frame, 3 drive frames, then idle)\n");
}

static void dither(void)
{
    epd_assume(&E, 30);
    epd_stag_par p = { .G = 2, .n = 1, .dither = 1, .R = 9 };
    epd_stag_enter(&S);
    epd_rect r = { 0, 0, 512, 512 };
    uint8_t *lv = malloc((size_t)r.w * r.h);
    for (int lev = 0; lev <= 30; lev += 6) {
        memset(lv, lev, (size_t)r.w * r.h);
        epd_stag_load(&S, &E, &p, r);
        epd_stag_target(&S, r, lv);
        for (int k = 0; k < 4; k++) frame(&p);
        long white = 0;
        for (int y = 0; y < r.h; y++)
            for (int x = 0; x < r.w; x++) white += EPD_STAG_LEVEL(S.cold[(size_t)y * EPD_W + x]) == 30;
        double fr = (double)white / (r.w * r.h);
        if (fr < lev / 30.0 - 0.01 || fr > lev / 30.0 + 0.01) FAIL("dither: level %d: %.3f white", lev, fr);
        printf("dither: level %2d -> %.3f white\n", lev, fr);
    }
    free(lv);
}

static void controller(void)
{
    const int R = 9;
    epd_assume(&E, 30);
    epd_state_fill(&E, EPD_F_DC, 0);
    epd_stag_par p = { .G = 4, .n = 3, .ctl = 1, .R = R };
    epd_stag_enter(&S);
    epd_rect r = { 300, 300, 16, 16 };
    uint8_t lv[16 * 16];
    memset(lv, 0, sizeof lv);
    epd_stag_load(&S, &E, &p, r);
    epd_stag_target(&S, r, lv);
    size_t o = (size_t)(r.y + 3) * EPD_W + r.x + 3;
    int drive = 0;
    for (int k = 0; k < 30; k++) { frame(&p); drive += phase_at(W, r.x + 3, r.y + 3) == 1; }
    if (drive != R || EPD_STAG_OP(S.cold[o]) != 0 || EPD_STAG_DC(S.cold[o]) != -R)
        FAIL("controller: drove %d frames (%d), op %d dc %d", drive, R, EPD_STAG_OP(S.cold[o]), EPD_STAG_DC(S.cold[o]));
    else printf("controller: white -> black in steps of 3 until the model is there (%d frames)\n", drive);
    /* payback: at black with the record at +5 */
    p.pay = 2;
    S.cold[o] = epd_stag_cold(0, 0, 1, 5);
    S.hot[o] = (uint16_t)epd_stag_hotbits(0, 0, 5, &p);
    drive = 0;
    for (int k = 0; k < 30; k++) { frame(&p); drive += phase_at(W, r.x + 3, r.y + 3) == 1; }
    /* (a step pays up to the whole record: 3, then the 2 left, which is >= pay) */
    if (drive != 5 || EPD_STAG_DC(S.cold[o]) != 0 || EPD_STAG_LEVEL(S.cold[o]) != 0)
        FAIL("controller: paid back %d frames (5), dc %d level %d", drive, EPD_STAG_DC(S.cold[o]), EPD_STAG_LEVEL(S.cold[o]));
    else printf("controller: DC +5 at black paid back to 0 (pay 2: steps of 3 + 2), level unchanged\n");
    /* drain: a new target starts nothing */
    p.drain = 1;
    memset(lv, 30, sizeof lv);
    epd_stag_target(&S, r, lv);
    drive = 0;
    for (int k = 0; k < 20; k++) { frame(&p); drive += phase_at(W, r.x + 3, r.y + 3) != 0; }
    if (drive) FAIL("drain: %d drive frames", drive);
    else printf("drain: no new steps\n");
}

/* ctl 2: payback through the idle slots (A2/2: every frame instead of every
 * other), and never a reversal without an idle frame before it */
static void controller2(void)
{
    const int R = 9;
    epd_assume(&E, 30);
    epd_state_fill(&E, EPD_F_DC, 0);
    epd_rect r = { 300, 300, 16, 16 };
    uint8_t lv[16 * 16];
    int pays[3];
    for (int ctl = 1; ctl <= 2; ctl++) {
        epd_stag_par p = { .G = 2, .n = 1, .ctl = ctl, .pay = 2, .R = R };
        epd_stag_enter(&S);
        memset(lv, 0, sizeof lv);
        epd_stag_load(&S, &E, &p, r);
        epd_stag_target(&S, r, lv);
        for (int y = r.y; y < r.y + r.h; y++)          /* at black, the record at +40 */
            for (int x = r.x; x < r.x + r.w; x++) {
                size_t o = (size_t)y * EPD_W + x;
                S.cold[o] = epd_stag_cold(0, 0, 1, 40);
                S.hot[o] = (uint16_t)epd_stag_hotbits(0, 0, 40, &p);
            }
        int drive = 0, last[16 * 16];
        for (int k = 0; k < 20; k++) {
            frame(&p);
            drive += phase_at(W, r.x + 3, r.y + 3) == 1;
            for (int i = 0; i < 16 * 16; i++) last[i] = phase_at(W, r.x + i % 16, r.y + i / 16);
        }
        pays[ctl] = drive;
        /* now white: the black payback stops, white starts after an idle frame */
        memset(lv, 30, sizeof lv);
        epd_stag_target(&S, r, lv);
        int bad_rev = 0, whites = 0;
        for (int k = 0; k < 24; k++) {
            frame(&p);
            for (int i = 0; i < 16 * 16; i++) {
                int ph = phase_at(W, r.x + i % 16, r.y + i / 16);
                if (ph && last[i] && ph != last[i]) bad_rev++;
                whites += ph == 2;
                last[i] = ph;
            }
        }
        if (bad_rev) FAIL("controller %d: %d reversals without an idle frame", ctl, bad_rev);
        if (!whites) FAIL("controller %d: never drove to white", ctl);
    }
    if (pays[2] != 2 * pays[1] || pays[1] != 10)
        FAIL("controller 2: paid %d frames in 20 (ctl 1: %d; expected 20 and 10)", pays[2], pays[1]);
    else printf("controller 2: payback every frame at A2/2 (%d vs %d of 20), reversals after an idle frame\n", pays[2], pays[1]);
}

/* ctl 3: a pulse instead of the idle frame. White -> black in R frames in a
 * row (ctl 1: R steps, an idle after each); a run of pulses without an idle
 * frame is at most K + G - 1 long (the cycle past K keeps its idle); content
 * flipping every cycle (a 1-px line scrolling by) reverses without an idle */
static int run_max(const int *ph, int n, int *drive)
{
    int run = 0, mx = 0;
    *drive = 0;
    for (int i = 0; i < n; i++) {
        run = ph[i] ? run + 1 : 0;
        if (run > mx) mx = run;
        *drive += ph[i] != 0;
    }
    return mx;
}

static void controller3(void)
{
    const int R = 9, K = 8;
    epd_assume(&E, 30);
    epd_state_fill(&E, EPD_F_DC, 0);
    epd_rect r = { 300, 300, 16, 16 };
    uint8_t lv[16 * 16];
    int px = r.x + 3, py = r.y + 3, at[4] = { 0 };
    size_t o = (size_t)py * EPD_W + px;
    for (int ctl = 1; ctl <= 3; ctl += 2) {
        epd_stag_par p = { .G = 2, .n = 1, .ctl = ctl, .pay = 2, .K = K, .R = R };
        epd_stag_enter(&S);
        memset(lv, 0, sizeof lv);
        epd_stag_load(&S, &E, &p, r);
        epd_stag_target(&S, r, lv);
        int ph[64];
        for (int k = 0; k < 40 && !at[ctl]; k++) {
            frame(&p);
            ph[k] = phase_at(W, px, py);
            if (EPD_STAG_OP(S.cold[o]) == 0 && !at[ctl]) at[ctl] = k + 1;
        }
        if (ctl == 3) {
            int d, mx = run_max(ph, at[ctl], &d);
            if (d != R || mx != R) FAIL("controller 3: white -> black: %d pulses, run %d (%d, %d)", d, mx, R, R);
        }
        /* payback: at black, the record at +40: runs of K + G - 1 at most */
        for (int y = r.y; y < r.y + r.h; y++)
            for (int x = r.x; x < r.x + r.w; x++) {
                size_t q = (size_t)y * EPD_W + x;
                S.cold[q] = epd_stag_cold(0, 0, 1, 40);
                S.hot[q] = (uint16_t)epd_stag_hotbits(0, 0, 40, &p);
            }
        int pph[40], d;
        for (int k = 0; k < 40; k++) { frame(&p); pph[k] = phase_at(W, px, py); }
        int mx = run_max(pph, 40, &d);
        if (ctl == 3 && (mx > K + p.G - 1 || d < 30)) FAIL("controller 3: payback: run %d (<= %d), %d pulses in 40", mx, K + p.G - 1, d);
        if (ctl == 3) printf("controller 3: payback %d pulses in 40, runs <= %d (K %d)\n", d, mx, K);
    }
    if (at[3] > R + 1 || at[1] < 2 * R - 1)
        FAIL("controller 3: white -> black after %d frames (ctl 1: %d)", at[3], at[1]);
    else printf("controller 3: white -> black after %d frames (ctl 1: %d)\n", at[3], at[1]);
    /* the line: black and white by turns every 2 frames (a cycle), ctl 1 vs 3 */
    int got[4] = { 0 };
    for (int ctl = 1; ctl <= 3; ctl += 2) {
        epd_stag_par p = { .G = 2, .n = 1, .ctl = ctl, .K = K, .R = R };
        epd_stag_enter(&S);
        memset(lv, 30, sizeof lv);
        epd_stag_load(&S, &E, &p, r);
        epd_stag_target(&S, r, lv);
        int ph[48], rev = 0, d;
        for (int k = 0; k < 48; k++) {
            if (k % 4 == 0) { memset(lv, k % 8 ? 30 : 0, sizeof lv); epd_stag_target(&S, r, lv); }
            frame(&p);
            ph[k] = phase_at(W, px, py);
            if (k && ph[k] && ph[k - 1] && ph[k] != ph[k - 1]) rev++;
        }
        int mx = run_max(ph, 48, &d);
        got[ctl] = d;
        printf("controller %d: a target flipping every 4 frames: %d pulses in 48, longest run %d, %d direct reversals\n", ctl, d, mx, rev);
        if (ctl == 3 && mx > K + p.G - 1) FAIL("controller 3: line: run %d", mx);
    }
    if (got[3] <= got[1]) FAIL("controller 3: line: %d pulses, not more than ctl 1's %d", got[3], got[1]);
    {                                           /* A2/1 (G 1, n 1, ctl 3): a decision every frame */
        epd_stag_par p = { .G = 1, .n = 1, .ctl = 3, .K = K, .R = R };
        int ph[48], d, a1 = 0;
        epd_stag_enter(&S);
        memset(lv, 30, sizeof lv);
        epd_stag_load(&S, &E, &p, r);
        memset(lv, 0, sizeof lv);
        epd_stag_target(&S, r, lv);
        for (int k = 0; k < 40 && !a1; k++) {
            frame(&p);
            ph[k] = phase_at(W, px, py);
            if (EPD_STAG_OP(S.cold[o]) == 0) a1 = k + 1;
        }
        int mx = run_max(ph, a1, &d);
        if (a1 != R + (R - 1) / K || d != R) FAIL("A2/1: white -> black after %d frames, %d pulses (%d, an idle every K)", a1, d, R);
        else printf("A2/1: white -> black after %d frames (A2/2 ctl 3: %d, ctl 1: %d)\n", a1, at[3], at[1]);
        for (int per = 1; per <= 2; per++) {    /* a target flipping every frame / every 2nd */
            epd_stag_enter(&S);
            memset(lv, 30, sizeof lv);
            epd_stag_load(&S, &E, &p, r);
            epd_stag_target(&S, r, lv);
            int fl = 0, late = 0;
            for (int k = 0; k < 48; k++) {
                if (k % per == 0) { memset(lv, (k / per) % 2 ? 30 : 0, sizeof lv); epd_stag_target(&S, r, lv); }
                frame(&p);
                ph[k] = phase_at(W, px, py);
                int want = (k / per) % 2 ? 2 : 1;   /* (1 black, 2 white) */
                fl += ph[k] == want;
                late += ph[k] && ph[k] != want;
            }
            mx = run_max(ph, 48, &d);
            printf("A2/1: a target flipping every %d frame%s: %d pulses in 48 (%d toward the frame's target, %d not), longest run %d\n",
                   per, per > 1 ? "s" : "", d, fl, late, mx);
            if (late || mx > K) FAIL("A2/1: flips every %d: %d pulses away from the frame's target, run %d (K %d)", per, late, mx, K);
        }
    }
}

#ifdef EPD_GPU
/* ---- part 2: the GPU against the CPU ---- */
static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec / 1e9; }
static uint32_t GW_[EPD_WORDS];
static epd_stag S2;
/* lanes (stagger_state=lanes, ; GLASSINE_CLAIM_STATE=packed
 * GLASSINE_STAGGER_STATE=lanes): the pass on E's D and T; the CPU reference runs
 * px_lane on S, its targets and parked cold state mirrored from T and D */
static int LN;
static uint8_t *TG, *LD, *MK, *PU;  /* the targets staged (EPD_W apart, both modes); lanes: armed marks, the arm mask; a box's content held */
static const epd_rect ALL = { 0, 0, EPD_W, EPD_H };
static void dsy(unsigned fl) { if (LN) epd_dmabuf_sync(E.pk->dfd, fl, 0, 0); }
static void tsy(unsigned fl) { if (LN) epd_dmabuf_sync(E.pk->tfd, fl, 0, 0); }
#define SY_B (DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW)
#define SY_E (DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW)

static int b4_state_cmp(epd_gpu *g, const char *when);
static int state_cmp(epd_gpu *g, const char *when)
{
    if (B4) return b4_state_cmp(g, when);
    epd_gpu_stag_read(g, &S2, (epd_rect){ 0, 0, EPD_W, EPD_H }, 1);
    size_t n = (size_t)EPD_W * EPD_H, dh = 0, dc = 0, first = n;
    for (size_t i = 0; i < n; i++) {
        if (S2.hot[i] != S.hot[i]) { dh++; if (first == n) first = i; }
        if (S2.cold[i] != S.cold[i]) { dc++; if (first == n) first = i; }
    }
    if (dh || dc) {
        FAIL("%s: state differs: hot %zu, cold %zu px; first (%zu, %zu) hot gpu %04x cpu %04x cold gpu %08x cpu %08x", when, dh, dc,
             first % EPD_W, first / EPD_W, S2.hot[first], S.hot[first], S2.cold[first], S.cold[first]);
        if (getenv("STAG_DUMP"))                /* (every differing pixel, up to 64) */
            for (size_t i = 0, k = 0; i < n && k < 64; i++)
                if ((S2.hot[i] != S.hot[i] || S2.cold[i] != S.cold[i]) && ++k)
                    printf("  diff (%zu, %zu): hot gpu %04x cpu %04x cold gpu %08x cpu %08x\n", i % EPD_W, i / EPD_W, S2.hot[i], S.hot[i], S2.cold[i], S.cold[i]);
        return 1;
    }
    if (LN) {                                   /* (lanes: the acks too) */
        size_t na = (size_t)(EPD_W / 8) * EPD_H, da = 0, fa = na;
        for (size_t i = 0; i < na; i++) if (S2.ack[i] != S.ack[i]) { da++; if (fa == na) fa = i; }
        if (da) {
            FAIL("%s: acks differ: %zu words; first word (%zu, %zu) gpu %d cpu %d", when, da, fa % (EPD_W / 8), fa / (EPD_W / 8), S2.ack[fa], S.ack[fa]);
            return 1;
        }
    }
    return 0;
}

/* ---- lanes: the CPU's side (epd-stagger.h lane_*), each T write synced,
 * noted for the GPU (epd_gpu_stag_target) and mirrored into S ---- */
static void ln_mirror(epd_rect r)
{
    if (!LN) return;
    dsy(DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ);
    epd_stag_lane_mirror(&S, &E, r);
    dsy(DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ);
    if (B4) {                                   /* (ctl 4: the twin's request planes) */
        tsy(DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ);
        bp4_lane_pack(&T4, E.pk->T, r);
        tsy(DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ);
    }
}

/* the engine's finished words in r resolved (a run's start; after a paint) */
static void ln_resolve(epd_rect r)
{
    if (!LN) return;
    dsy(SY_B);
    long n = epd_stag_lane_resolve(&E, r);
    dsy(SY_E);
    if (n) epd_pk_w_sync(&E);
    ln_mirror(r);
}

/* the targets of r (TG): runs: S's and the GPU's; lanes: armed (where MK is
 * set, if mk; padded to the 16-px lines), LD marked */
static epd_rect tg_set(epd_gpu *g, epd_rect r, int mk)
{
    if (!LN) {
        for (int y = r.y; y < r.y + r.h; y++) memcpy(S.tgt + (size_t)y * EPD_W + r.x, TG + (size_t)y * EPD_W + r.x, (size_t)r.w);
        epd_gpu_stag_target(g, &S, r);
        return r;
    }
    tsy(SY_B);
    epd_rect pr = epd_stag_lane_arm(&E, r, TG, mk ? MK : NULL);
    tsy(SY_E);
    for (int y = pr.y; y < pr.y + pr.h; y++)
        for (int x = pr.x; x < pr.x + pr.w; x++) { size_t o = (size_t)y * EPD_W + x; if (!mk || MK[o]) LD[o] = 1; }
    epd_gpu_stag_target(g, &S, pr);
    ln_mirror(pr);
    return pr;
}

/* new content over r: runs: loaded first (S and the GPU) */
static void tg_in(epd_gpu *g, const epd_stag_par *p, epd_rect r, int mk)
{
    if (!LN) { epd_stag_load(&S, &E, p, r); epd_gpu_stag_load(g, &S, p, r); }
    tg_set(g, r, mk);
}

/* lanes: r evicted (each word's tog the other of its last ack, the GPU's): the padded rect */
static epd_rect ln_evict(epd_gpu *g, epd_rect r)
{
    epd_gpu_stag_acks(g, S2.ack, 0, EPD_H);
    tsy(SY_B);
    epd_rect pr = epd_stag_lane_evict(&E, r, S2.ack);
    tsy(SY_E);
    epd_gpu_stag_target(g, &S, pr);
    ln_mirror(pr);
    return pr;
}

/* lanes: pr's words not acked yet, the CPU's (*cu) and the GPU's (*gu) */
static void ln_acks(epd_gpu *g, epd_rect pr, long *cu, long *gu)
{
    epd_gpu_stag_acks(g, S2.ack, 0, EPD_H);
    tsy(DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ);
    *cu = epd_stag_lane_unacked(&E, S.ack, pr);
    *gu = epd_stag_lane_unacked(&E, S2.ack, pr);
    tsy(DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ);
}

/* lanes: the acked pr taken over (T := D's level where armed); every pixel
 * of it parked with its level decoded (an even D) */
static void ln_takeover(epd_gpu *g, epd_rect pr, const char *when)
{
    long np = 0, nd = 0;
    size_t fd = 0;
    dsy(DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ);
    for (int y = pr.y; y < pr.y + pr.h; y++)
        for (int x = pr.x; x < pr.x + pr.w; x++) {
            size_t o = (size_t)y * EPD_PK_S + x;
            if (!B4) { np += EPD_STAG_LV(S.hot[(size_t)y * EPD_W + x]) != 3 || (E.pk->D[o] & 1); continue; }
            np += !((T4.park[(size_t)y * BP_NK + x / 32] >> (31 - x % 32)) & 1) || (E.pk->D[o] & 1);
            if (LD[(size_t)y * EPD_W + x] && E.pk->D[o] != DSH[o]) { if (!nd) fd = o; nd++; }   /* (ctl 4: the evicted D, the GPU's == the twin's) */
        }
    if (nd) FAIL("%s: takeover: %ld evicted D words differ, first (%zu, %zu): gpu %08x twin %08x", when, nd, fd % EPD_PK_S, fd / EPD_PK_S, E.pk->D[fd], DSH[fd]);
    tsy(SY_B);
    epd_stag_lane_takeover(&E, pr, LD);
    tsy(SY_E);
    dsy(DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ);
    if (np) FAIL("%s: takeover: %ld pixels not parked or with a budget left", when, np);
    epd_gpu_stag_target(g, &S, pr);
    ln_mirror(pr);
}

/* the engine runs to its end (D/T/W written by the CPU: everything parked) */
static void eng_run(void)
{
    while (E.nrun || E.npend) { epd_schedule(&E); epd_advance(&E, NULL, NULL); }
}

/* 's paints of a box the stagger gave back (both modes): a popup drawn
 * into the engine's state (1), a pen piece (2: a DU), a page turn (4: a GC16
 * over the panel, the next page: *y0); PU := the box's levels after */
static void s5_paint(int s5, epd_rect bx, epd_rect pb, const uint8_t *pg, int *y0, int ph)
{
    dsy(SY_B); tsy(SY_B);
    if (s5 & 1) {
        for (int y = pb.y; y < pb.y + pb.h; y++)
            for (int x = pb.x; x < pb.x + pb.w; x++) {
                size_t o = (size_t)y * EPD_W + x;
                int edge = x == pb.x || y == pb.y || x == pb.x + pb.w - 1 || y == pb.y + pb.h - 1;
                int l = edge ? 0 : ((x - pb.x) / 12 + (y - pb.y) / 12) & 1 ? 30 : 4;
                epd_px_put(&E, o, EPD_F_LEVEL | EPD_F_TARGET, l, l, 0, 0, 0, 0);
            }
    } else {
        uint8_t *img = calloc(EPD_NPX, 1);
        epd_rect r = s5 & 4 ? ALL : bx;
        if (s5 & 4) *y0 = (*y0 + 1404) % (ph - EPD_H);
        for (int y = r.y; y < r.y + r.h; y++)
            for (int x = r.x; x < r.x + r.w; x++) {
                size_t o = (size_t)y * EPD_W + x;
                if (s5 & 4) img[o] = (uint8_t)(pg[(size_t)(*y0 + y) * EPD_W + x] * 255 / 30);
                else img[o] = (x - bx.x) / 2 == (y - bx.y) || (x - bx.x) / 2 == (y - bx.y) + 1 ? 0 : epd_px_level(&E, o) >= 16 ? 255 : 0;
            }
        epd_submit(&E, r, s5 & 4 ? EPD_GC16 : EPD_DU, (s5 & 4) != 0, img, EPD_W);
        eng_run();
        free(img);
    }
    tsy(SY_E); dsy(SY_E);
    epd_pk_w_sync(&E);
    ln_resolve(s5 & 4 ? ALL : pb);
    for (int y = pb.y; y < pb.y + pb.h; y++)
        for (int x = pb.x; x < pb.x + pb.w; x++) PU[(size_t)y * EPD_W + x] = (uint8_t)epd_px_level(&E, (size_t)y * EPD_W + x);
}

/* lanes, the GPU alone (after the timed frames, S not kept): everything
 * evicted, frames until the GPU's acks, taken over */
static void ln_settle_gpu(epd_gpu *g, int R)
{
    epd_stag_par p = { .G = 2, .n = 1, .ctl = 3, .K = 8, .R = R, .payrule = 1, .pcap = 3, .stripe = 64, .lanes = 1 };
    epd_rect pr = ln_evict(g, ALL);
    long u = 1;
    int f = 0;
    for (; f < 40 && u; f++) {
        epd_gpu_stag_frame(g, &p, S.f++, 0);
        epd_gpu_finish(g, 1000);
        epd_gpu_stag_acks(g, S2.ack, 0, EPD_H);
        u = epd_stag_lane_unacked(&E, S2.ack, pr);
    }
    if (u) FAIL("lanes: settle: %ld words not acked after %d frames", u, f);
    tsy(SY_B);
    epd_stag_lane_takeover(&E, pr, LD);
    tsy(SY_E);
    epd_gpu_stag_target(g, &S, pr);
    epd_stag_lanes_park(&S);
    memcpy(S.ack, S2.ack, (size_t)(EPD_W / 8) * EPD_H);
    ln_mirror(ALL);
}

/* ctl 4: the twin's frame (S.f; the masks of p's layout; arms from E's D,
 * evicts into DSH, acks into S.ack) */
static void b4_twin_frame(const epd_stag_par *p)
{
    int mk = p->stripe << 8 ^ p->cellw << 4 ^ p->cellh << 20 ^ p->cellord << 28 ^ p->blocks << 2;
    if (mk != m4k) {
        epd_stag_par q = *p;
        q.G = 2;
        bp_masks_free(&M4);
        if (bp_masks_make(&M4, &q)) exit(2);
        m4k = mk;
    }
    dsy(DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ);
    bp4_lane_rows(&T4, &M4, p, bp4_vm(p), S.f, E.tpl, W, 0, EPD_H, E.pk->D, DSH, S.ack);
    dsy(DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ);
    epd_stag_advance(&S);
}
/* the twin's planes as the GPU's S texels (BP_NK x 7 EPD_H, layer L at rows L EPD_H) */
static void b4_texels(const bp4_state *s, uint32_t *u)
{
    const size_t L = (size_t)4 * BP_NW;
    for (size_t i = 0; i < BP_NW; i++) {
        uint32_t *t = u + 4 * i;
        t[0] = s->lw[i]; t[1] = s->p[i]; t[2] = s->ow[0][i]; t[3] = s->ow[1][i];
        for (int j = 0; j < 4; j++) t[L + j] = s->op[j][i];
        for (int j = 0; j < 4; j++) t[2 * L + j] = s->ac[j][i];
        t[3 * L + 0] = s->ac[4][i]; t[3 * L + 1] = s->pd[i]; t[3 * L + 2] = s->rv[i]; t[3 * L + 3] = s->h8[i];
        for (int j = 0; j < 4; j++) t[4 * L + j] = s->dc[j][i];
        for (int j = 0; j < 4; j++) t[5 * L + j] = s->dc[4 + j][i];
        t[6 * L + 0] = s->st[i]; t[6 * L + 1] = s->park[i]; t[6 * L + 2] = 0; t[6 * L + 3] = 0;
        t[7 * L + 0] = s->kc[0][i]; t[7 * L + 1] = s->kc[1][i]; t[7 * L + 2] = s->kc[2][i]; t[7 * L + 3] = 0;
    }
}
/* ctl 4: the GPU's planes and acks against the twin's */
static int b4_state_cmp(epd_gpu *g, const char *when)
{
    size_t n = (size_t)4 * 8 * BP_NW, fi = 0;
    uint32_t *u = malloc(n * 4), *v = malloc(n * 4);
    if (!u || !v) exit(2);
    epd_gpu_bp4_read(g, u);
    b4_texels(&T4, v);
    long d = 0;
    for (size_t i = 0; i < n; i++) if (u[i] != v[i]) { if (!d) fi = i; d++; }
    if (d) FAIL("%s: ctl 4 planes differ: %ld words; first layer %zu word (%zu, %zu) .%zu: gpu %08x twin %08x", when, d,
                fi / 4 / BP_NW, fi / 4 % BP_NK, fi / 4 / BP_NK % EPD_H, fi % 4, u[fi], v[fi]);
    free(u); free(v);
    epd_gpu_stag_acks(g, S2.ack, 0, EPD_H);
    size_t na = (size_t)BP_GW * EPD_H, da = 0, fa = 0;
    for (size_t i = 0; i < na; i++) if (S2.ack[i] != S.ack[i]) { if (!da) fa = i; da++; }
    if (da) FAIL("%s: ctl 4 acks differ: %zu words; first word (%zu, %zu) gpu %d twin %d", when, da, fa % BP_GW, fa / BP_GW, S2.ack[fa], S.ack[fa]);
    return d || da;
}

/* a text page: rows of glyph-ish blocks, greys at the edges (5-bit levels) */
static uint8_t *page(int h)
{
    uint8_t *p = malloc((size_t)EPD_W * h);
    srand(11);
    for (int y = 0; y < h; y++)
        for (int x = 0; x < EPD_W; x++) {
            int line = y % 48, col = x % 24;
            int ink = line >= 12 && line < 36 && col >= 4 && col < 20 && ((x / 24 * 7 + y / 48 * 13) % 5 != 0);
            int v = ink ? 0 : 30;
            if (ink && (line == 12 || line == 35 || col == 4 || col == 19)) v = 2 * (rand() % 16);
            if (x > 1400 && y % 400 < 200) v = 2 * ((x + y) % 16);     /* a picture: greys */
            p[(size_t)y * EPD_W + x] = (uint8_t)v;
        }
    return p;
}

/* the controllers compared on the CPU (a measure, no pass/fail): a scroll
 * (new content every 3rd frame), then a read pause. Behind: pixel-frames whose
 * panel model is not at its target's rail (a target change shows late);
 * |DC|: the records' mean and max over the panel */
static void quality(int G, int pay, int speed, int frames, int pause, int seed)
{
    const int R = E.op_r ? E.op_r : 9;
    int ph = 1404 * 4;
    uint8_t *pg = page(ph);
    for (int ctl = G == 1 ? 3 : 1; ctl <= 3; ctl++) {     /* (G 1: A2/1, ctl 3 only) */
        epd_stag_par p = { .G = G, .n = G > 1 ? G - 1 : 1, .ctl = ctl, .pay = pay, .K = 8, .dither = 1, .R = R };
        if (getenv("STAG_QTOP")) p.htop = atoi(getenv("STAG_QTOP"));   /* the arrival top-up */
        epd_assume(&E, 30);
        srand(5);                               /* records left by the engine's paints (GC16 +-21 a paint): +-seed */
        for (size_t o = 0; o < (size_t)EPD_W * EPD_H; o++) epd_px_put(&E, o, EPD_F_DC, 0, 0, 0, 0, (int16_t)(seed ? rand() % (2 * seed + 1) - seed : 0), 0);
        epd_stag_enter(&S);
        epd_rect r = { 0, 0, EPD_W, EPD_H };
        epd_stag_load(&S, &E, &p, r);
        int y0 = 0;
        double behind_s = 0, behind_p = 0, dsum = 0, dscroll = 0, dhalf = 0;
        int dmax = 0;
        for (int f = 0; f < frames + pause; f++) {
            if (f < frames && f % 3 == 0) {
                y0 = (y0 + speed) % (ph - EPD_H);
                memcpy(S.tgt, pg + (size_t)y0 * EPD_W, (size_t)EPD_W * EPD_H);
            }
            frame(&p);
            long b = 0;
            for (int y = 0; y < EPD_H; y++)
                for (int x = 0; x < EPD_W; x++) {
                    size_t o = (size_t)y * EPD_W + x;
                    int t = S.tgt[o];
                    int tb = t >= 2 * epd_stag_dqt[y & 63][x & 63], op = EPD_STAG_OP(S.cold[o]);
                    b += tb ? op != R : op != 0;
                }
            if (f < frames) behind_s += b; else behind_p += b;
            if (f == frames - 1 || f == frames + pause / 4 - 1) {
                double ds = 0;
                for (size_t o = 0; o < (size_t)EPD_W * EPD_H; o++) ds += abs(EPD_STAG_DC(S.cold[o]));
                *(f == frames - 1 ? &dscroll : &dhalf) = ds / ((double)EPD_W * EPD_H);
            }
        }
        for (size_t o = 0; o < (size_t)EPD_W * EPD_H; o++) {
            int d = EPD_STAG_DC(S.cold[o]);
            d = d < 0 ? -d : d;
            dsum += d;
            if (d > dmax) dmax = d;
        }
        printf("quality G %d pay %d seed %d, ctl %d: behind %.0f px a frame scrolling, %.0f in the pause; |DC| mean %.2f after the scroll, %.2f a quarter into the pause, %.2f at its end (max %d)\n",
               G, pay, seed, ctl, behind_s / frames, pause ? behind_p / pause : 0, dscroll, dhalf, dsum / ((double)EPD_W * EPD_H), dmax);
    }
    free(pg);
}

/* STAG_COLD_DIAG=1 (CPU): ctl 1 with panel pay, dithered, a full panel
 * scrolled 24 frames then still: per frame, the
 * pixels on their group's frame that take the cold path (not skipped), by
 * why: what keeps a still page's tiles busy (the tile skip) */
static void cold_diag(void)
{
    const int R = E.op_r ? E.op_r : 9, ph = 1404 * 4;
    uint8_t *pg = page(ph);
    epd_stag_par p = { .G = 2, .n = 1, .ctl = 1, .K = 8, .dither = 1, .R = R, .payrule = 1, .pcap = EPD_STAG_PCAP };
    epd_assume(&E, 30);
    epd_stag_enter(&S);
    epd_rect all = { 0, 0, EPD_W, EPD_H };
    epd_stag_load(&S, &E, &p, all);
    int y0 = 0;
    for (int f = 0; f < 24 + 120; f++) {
        if (f < 24 && f % 2 == 0) {
            y0 = (y0 + 17) % (ph - EPD_H);
            memcpy(S.tgt, pg + (size_t)y0 * EPD_W, (size_t)EPD_W * EPD_H);
        }
        if (f >= 24 && (f - 24) % 8 == 0) {     /* before this frame: who goes cold */
            long lvx = 0, nset = 0, bud = 0, phs = 0, tot = 0, tiles = 0;
            int fm = (int)(S.f % (unsigned)p.G);
            unsigned f6 = S.f & 63;
            static uint8_t tb_[(EPD_W / 256 + 1) * (EPD_H / 8 + 1)];
            memset(tb_, 0, sizeof tb_);
            for (int y = 0; y < EPD_H; y++)
                for (int x = 0; x < EPD_W; x++) {
                    size_t o = (size_t)y * EPD_W + x;
                    unsigned h = S.hot[o], hph = h & 3, n = (h >> 2) & 15, age = (f6 - (h >> 6)) & 63;
                    int rank = epd_bn_rank[y & 63][x & 63], pos = fm - (rank * p.G >> 12);
                    if (pos < 0) pos += p.G;
                    if (hph && age < n) { phs++; tb_[(y / 8) * (EPD_W / 256 + 1) + x / 256] = 1; continue; }   /* (driving) */
                    if (pos != 0) continue;
                    int t = S.tgt[o], tb = t >= 2 * epd_stag_dqt[y & 63][x & 63] ? 30 : 0;
                    if (EPD_STAG_LV(h) == (tb ? 1u : 0u) && EPD_STAG_SET(h)) continue;
                    tot++;
                    tb_[(y / 8) * (EPD_W / 256 + 1) + x / 256] = 1;
                    int b, l = epd_stag_ldec(EPD_STAG_LEVEL(S.cold[o]), &b);
                    if (b) bud++;
                    else if (EPD_STAG_LV(h) != (tb ? 1u : 0u)) lvx++;
                    else nset++;
                    (void)l;
                }
            for (unsigned i = 0; i < sizeof tb_; i++) tiles += tb_[i];
            printf("cold diag, still frame %d: cold %ld px (rail with a budget %ld, rail level off %ld, "
                   "model short %ld); driving %ld; busy tiles %ld of %d\n", f - 24, tot, bud,
                   lvx, nset, phs, tiles, (EPD_W / 256 + 1) * (EPD_H / 8 + 1));
        }
        frame(&p);
    }
    free(pg);
}

/* ---- the bit-plane bench (test/epd-bitplane.h): a prototype
 * of the A2 pass at ctl 0 with the state as bit planes, timing only. Off
 * unless STAG_BP_GATE (the gate below) or STAG_BITPLANE + a "fixed:" timing
 * row (gpu_part) asks for it: every other path of this test is unchanged.
 * STAG_BITPLANE="32x1 8x8": the shapes (workgroup WX x WY, one invocation a
 * 32-px plane word; the timing rows use the first); "none": the gate's
 * CPU-only half (today's CPU reference == the bit-plane CPU reference)
 * STAG_BP_LAYOUTS="64 wbayer 0" (the gate's): stripes 64, word Bayer 8 x 1,
 * the blue noise; STAG_BP_GS="2 4" (the gate's G, n = G - 1) */
#include "epd-bitplane.h"
static int bp_layout(epd_stag_par *p, const char *l)
{
    p->stripe = 0; p->cellw = p->cellh = 0; p->cellord = 0; p->blocks = 0;
    if (!strcmp(l, "wbayer")) { p->stripe = 64; p->cellw = 8; p->cellh = 1; p->cellord = EPD_STAG_CORD_WBAYER; }
    else if (!strcmp(l, "64")) p->stripe = 64;
    else if (strcmp(l, "0")) return -1;
    return 0;
}
static const char *bp_lname(const epd_stag_par *p)
{
    return p->stripe <= 0 ? "bluenoise" : p->cellw ? (p->cellord == EPD_STAG_CORD_WBAYER ? "wbayer" : "cells") : p->blocks ? "blocks" : "stripes64";
}
/* the data words of a and b differ: their number (*first: the first) */
static long bp_wdiff(const uint32_t *a, const uint32_t *b, size_t *first)
{
    long n = 0;
    *first = 0;
    for (int y = 0; y < EPD_H; y++) {
        size_t wo = (size_t)(EPD_DATA0 / EPD_LINE + EPD_H - 1 - y) * EPD_LINE + EPD_DATA0 % EPD_LINE;
        for (int x = 0; x < BP_GW; x++) if (a[wo + x] != b[wo + x]) { if (!n) *first = wo + x; n++; }
    }
    return n;
}
#define BP_SHMAX 8
/* the gate: today's CPU reference (epd_stag_rows, runs, ctl 0) == the
 * bit-plane CPU reference == the bit-plane pass on the GPU (each shape),
 * every frame's data words, the state every 8th frame, for each layout and
 * G: a scroll (48 frames, new content every 2nd), still (24), a window (48:
 * every 2nd frame the timing rows' 500 x 380 window or an odd-edged
 * 1500 x 1203 one). 0: all equal */
static int bp_gate(void)
{
    static uint32_t W2[EPD_WORDS], W3[EPD_WORDS];
    const int R = E.op_r ? E.op_r : 9;
    const char *shs = getenv("STAG_BITPLANE"), *ls = getenv("STAG_BP_LAYOUTS"), *gs = getenv("STAG_BP_GS");
    int sw[BP_SHMAX], sh[BP_SHMAX], ns = 0;
    char buf[256];
    bp_break = getenv("STAG_BP_BREAK") ? atoi(getenv("STAG_BP_BREAK")) : 0;   /* (the negative controls: G 4 must differ) */
    snprintf(buf, sizeof buf, "%s", shs && *shs ? shs : "32x1 8x8");
    for (char *t = strtok(buf, " ,"); t && ns < BP_SHMAX; t = strtok(NULL, " ,")) {
        if (!strcmp(t, "none")) continue;
        if (bp_shape_parse(t, &sw[ns], &sh[ns])) { printf("BP: bad shape %s\n", t); return 2; }
        ns++;
    }
    const int ph = 1404 * 4;
    uint8_t *pg = page(ph), *tg = malloc(EPD_NPX);
    bp_state b, bg;
    if (!tg || bp_alloc(&b) || bp_alloc(&bg)) return 2;
    long nfr = 0, nbad = 0, nst = 0;
    char lb[128], gb[64];
    snprintf(lb, sizeof lb, "%s", ls && *ls ? ls : "64 wbayer 0");
    snprintf(gb, sizeof gb, "%s", gs && *gs ? gs : "2 4");
    char *lsv, *gsv;
    for (char *lt = strtok_r(lb, " ,", &lsv); lt; lt = strtok_r(NULL, " ,", &lsv)) {
        char gb2[64];
        snprintf(gb2, sizeof gb2, "%s", gb);
        for (char *gt = strtok_r(gb2, " ,", &gsv); gt; gt = strtok_r(NULL, " ,", &gsv)) {
            int G = atoi(gt);
            epd_stag_par p = { .G = G, .n = G - 1, .ctl = 0, .K = 8, .dither = 0, .R = R };
            if (bp_layout(&p, lt) || !bp_layout_ok(&p)) { printf("BP gate: layout %s G %d: not one the bit planes take\n", lt, G); return 2; }
            bp_masks m;
            if (bp_masks_make(&m, &p)) return 2;
            /* the start: an engine state with greys everywhere (as part 2's), no step in flight */
            epd_stag_enter(&S);
            memset(b.lw, 0, BP_NW * 4); memset(b.lg, 0, BP_NW * 4); memset(b.a, 0, BP_NW * 4);
            for (int y = 0; y < EPD_H; y++)
                for (int x = 0; x < EPD_W; x++) {
                    size_t i = (size_t)y * EPD_W + x;
                    int l = 2 * (int)(i % 16), dc = (int)(i % 7) - 3, op = l >= 30 ? R : R * l / 30;
                    S.cold[i] = epd_stag_cold(l, op, 0, dc);
                    S.hot[i] = (uint16_t)epd_stag_hotbits(l, op, dc, &p);
                    S.ld[i] = 1;
                    bp_set_level(&b, x, y, l, 0);
                }
            S.f = 61;                           /* (the frame counter crosses 64: f6 wraps) */
            int y0 = 0;
            memcpy(tg, pg, EPD_NPX);
            memcpy(S.tgt, tg, EPD_NPX);
            bp_targets(&b, tg, ALL);
#ifdef EPD_GPU
            bp_gpu bp[BP_SHMAX];
            for (int s = 0; s < ns; s++) {
                if (bp_gpu_open(&bp[s], sw[s], sh[s], &m, E.tpl)) { printf("BP gate: shape %dx%d: the GPU setup failed\n", sw[s], sh[s]); return 2; }
                bp_gpu_load(&bp[s], &b);
                bp_gpu_target(&bp[s], tg, ALL);
            }
#endif
            long lbad = 0, lfr = 0;
            for (int f = 0; f < 120 && !lbad; f++) {
                epd_rect r = { 0, 0, 0, 0 };
                if (f < 48) { if (f % 2 == 0) { y0 = (y0 + 17) % (ph - EPD_H); r = ALL; } }
                else if (f >= 72 && f % 2 == 0) {
                    y0 = (y0 + 17) % (ph - EPD_H);
                    r = (f / 2) % 2 ? (epd_rect){ 300, 400, 500, 380 } : (epd_rect){ 37, 101, 1500, 1203 };
                }
                if (r.w) {
                    for (int y = r.y; y < r.y + r.h; y++) {
                        memcpy(tg + (size_t)y * EPD_W + r.x, pg + (size_t)(y0 + y) * EPD_W + r.x, (size_t)r.w);
                        memcpy(S.tgt + (size_t)y * EPD_W + r.x, tg + (size_t)y * EPD_W + r.x, (size_t)r.w);
                    }
                    bp_targets(&b, tg, r);
#ifdef EPD_GPU
                    for (int s = 0; s < ns; s++) bp_gpu_target(&bp[s], tg, r);
#endif
                }
                uint32_t fc = S.f;
                frame(&p);                      /* (today's CPU reference: W) */
                memcpy(W2, E.tpl, sizeof W2);
                bp_rows(&b, &m, fc, E.tpl, W2, 0, EPD_H);
                size_t fw;
                long d = bp_wdiff(W, W2, &fw);
                if (d) { printf("BP gate: %s G %d frame %d: bit-plane CPU: %ld words differ, first line %zu word %zu: ref %08x bp %08x\n",
                                bp_lname(&p), G, f, d, fw / EPD_LINE, fw % EPD_LINE, W[fw], W2[fw]); lbad++; }
                int chk = f % 8 == 7 || f == 119;
                if (chk) {
                    size_t fi;
                    long sd = bp_cmp_ref(&b, &S, &fi);
                    nst++;
                    if (sd) { printf("BP gate: %s G %d frame %d: bit-plane CPU state: %ld px differ, first (%zu, %zu)\n", bp_lname(&p), G, f, sd, fi % EPD_W, fi / EPD_W); lbad++; }
                }
#ifdef EPD_GPU
                for (int s = 0; s < ns; s++) {
                    double pms, dms;
                    bp_gpu_frame(&bp[s], fc, &pms, &dms);
                    memcpy(W3, E.tpl, sizeof W3);
                    bp_gpu_words(&bp[s], W3);
                    d = bp_wdiff(W, W3, &fw);
                    if (d) { printf("BP gate: %s G %d shape %dx%d frame %d: GPU: %ld words differ, first line %zu word %zu: ref %08x gpu %08x\n",
                                    bp_lname(&p), G, sw[s], sh[s], f, d, fw / EPD_LINE, fw % EPD_LINE, W[fw], W3[fw]); lbad++; }
                    if (chk) {
                        bp_gpu_state(&bp[s], &bg);
                        long sd = 0;
                        for (size_t i = 0; i < BP_NW; i++) sd += bg.lw[i] != b.lw[i] || bg.lg[i] != b.lg[i] || bg.a[i] != b.a[i];
                        if (sd) { printf("BP gate: %s G %d shape %dx%d frame %d: GPU state: %ld plane words differ\n", bp_lname(&p), G, sw[s], sh[s], f, sd); lbad++; }
                    }
                    GLenum e = bpgl.glGetError();
                    if (e != GL_NO_ERROR) { printf("BP gate: shape %dx%d: GL error 0x%x\n", sw[s], sh[s], e); lbad++; }
                }
#endif
                lfr++;
            }
#ifdef EPD_GPU
            for (int s = 0; s < ns; s++) bp_gpu_close(&bp[s]);
#endif
            printf("BP gate: %-9s G %d n %d: %ld frames, %s (CPU ref == bit-plane CPU%s)\n", bp_lname(&p), G, G - 1, lfr, lbad ? "DIFFERS" : "equal",
                   ns ? " == GPU shapes" : "; no GPU shapes");
            nfr += lfr; nbad += lbad;
            bp_masks_free(&m);
        }
    }
    printf("BP gate: %ld frames, %ld bad, %ld state checks; shapes", nfr, nbad, nst);
    for (int s = 0; s < ns; s++) printf(" %dx%d", sw[s], sh[s]);
    printf("%s\n", ns ? "" : " none (CPU only)");
    bp_free(&b); bp_free(&bg);
    free(pg); free(tg);
    return nbad ? 1 : 0;
}

/* ---- ctl 3 on bit planes (test/epd-bp3.h): the gate, CPU only
 * for now. Today's CPU reference (epd_stag_rows, runs, ctl 3, payrule 1,
 * pcap 3, htop 2, K 8, G 2, dither 0; its record clamped to +-127 after every
 * frame, counted) == the bit-plane twin, every frame's words, the state every
 * 8th frame. Per layout, R (STAG_BP3_RS, default "9 14") and start (a small
 * record, then a wide one: +-100). Also counts the heavy words (the cold
 * path) per frame of each stretch: the cost estimate's traffic number.
 * STAG_BP3_BREAK=1..4: the negative controls (each must differ). */
#include "epd-bp3.h"
#include "epd-bp3n.h"
#include "epd-tier.h"
#include "epd-bp4.h"
#include "epd-bp4i.h"

/* the CPU timing of the ctl 3 planes : one frame over nt threads,
 * thread i pinned to cpus[i] (when given); vec: the four-word twin */
typedef struct { bp3_state *s; const bp_masks *m; const epd_stag_par *p; uint32_t f; uint32_t *w; size_t o0, o1; int vec, cpu; } bp3c_job;
static void *bp3c_run(void *a)
{
    bp3c_job *j = a;
    if (j->cpu >= 0) { cpu_set_t cs; CPU_ZERO(&cs); CPU_SET(j->cpu, &cs); sched_setaffinity(0, sizeof cs, &cs); }
    if (j->vec) bp3n_rows(j->s, j->m, j->p, j->f, E.tpl, j->w, j->o0, j->o1, NULL);
    else bp3_rows(j->s, j->m, j->p, j->f, E.tpl, j->w, (int)(j->o0 / BP_NK), (int)(j->o1 / BP_NK), NULL);
    return NULL;
}
static void bp3c_frame(bp3_state *s, const bp_masks *m, const epd_stag_par *p, uint32_t f, uint32_t *w, int vec, int nt, const int *cpus, int ncpu)
{
    pthread_t t[16];
    bp3c_job j[16];
    int up[16] = { 0 };
    for (int i = 0; i < nt; i++) {
        size_t a, b;
        if (vec) a = (size_t)BP_NW / 4 * i / nt * 4, b = (size_t)BP_NW / 4 * (i + 1) / nt * 4;
        else a = (size_t)EPD_H * i / nt * BP_NK, b = (size_t)EPD_H * (i + 1) / nt * BP_NK;
        j[i] = (bp3c_job){ s, m, p, f, w, a, b, vec, ncpu ? cpus[i % ncpu] : -1 };
        if (i) up[i] = !pthread_create(&t[i], NULL, bp3c_run, &j[i]);
        if (i && !up[i]) bp3c_run(&j[i]);
    }
    bp3c_run(&j[0]);
    for (int i = 1; i < nt; i++) if (up[i]) pthread_join(t[i], NULL);
}
static int bp3_gate(void)
{
    const char *ls = getenv("STAG_BP_LAYOUTS"), *rs = getenv("STAG_BP3_RS"), *shs = getenv("STAG_BITPLANE");
    bp3_break = getenv("STAG_BP3_BREAK") ? atoi(getenv("STAG_BP3_BREAK")) : 0;
    int coldall = getenv("STAG_BP3_COLD") ? atoi(getenv("STAG_BP3_COLD")) : 0;
    int sw[BP_SHMAX], sh[BP_SHMAX], ns = 0;
    {   char buf[256];
        snprintf(buf, sizeof buf, "%s", shs && *shs ? shs : "none");
        for (char *t = strtok(buf, " ,"); t && ns < BP_SHMAX; t = strtok(NULL, " ,")) {
            if (!strcmp(t, "none")) continue;
            if (bp_shape_parse(t, &sw[ns], &sh[ns])) { printf("BP3: bad shape %s\n", t); return 2; }
            ns++;
        }
        if (ns && !epd_gpu_open(&E)) { printf("BP3 gate: no GPU (EGL): GPU shapes skipped\n"); ns = 0; }
    }
    const int ph = 1404 * 4;
    uint8_t *pg = page(ph), *tg = malloc(EPD_NPX);
    bp3_state b, bn;                            /* (bn: the four-word twin, epd-bp3n.h) */
    if (!tg || bp3_alloc(&b) || bp3_alloc(&bn)) return 2;
    long nfr = 0, nbad = 0, nst = 0, nsat = 0, nvbad = 0;
    char lb[128], rb[64];
    snprintf(lb, sizeof lb, "%s", ls && *ls ? ls : "64 wbayer 0");
    snprintf(rb, sizeof rb, "%s", rs && *rs ? rs : "9 14");
    char *lsv, *rsv;
    for (char *lt = strtok_r(lb, " ,", &lsv); lt; lt = strtok_r(NULL, " ,", &lsv)) {
        char rb2[64];
        snprintf(rb2, sizeof rb2, "%s", rb);
        for (char *rt = strtok_r(rb2, " ,", &rsv); rt; rt = strtok_r(NULL, " ,", &rsv))
        for (int wide = 0; wide < 2; wide++) {
            int R = atoi(rt);
            epd_stag_par p = { .G = 2, .n = 1, .ctl = 3, .K = 8, .payrule = 1, .pcap = 3, .htop = 2, .dither = 0, .R = R };
            if (bp_layout(&p, lt) || !bp3_par_ok(&p)) { printf("BP3 gate: layout %s R %d: not one the planes take\n", lt, R); return 2; }
            bp_masks m;
            if (bp_masks_make(&m, &p)) return 2;
            epd_stag_enter(&S);
            for (int y = 0; y < EPD_H; y++)
                for (int x = 0; x < EPD_W; x++) {
                    size_t i = (size_t)y * EPD_W + x;
                    int l = 2 * (int)(i % 16), op = l >= 30 ? R : R * l / 30;
                    int dc = wide ? (int)((i * 37) % 201) - 100 : (int)(i % 7) - 3;
                    S.cold[i] = epd_stag_cold(l, op, 0, dc);
                    S.hot[i] = (uint16_t)epd_stag_hotbits(l, op, dc, &p);
                    S.ld[i] = 1;
                    bp3_put(&b, x, y, l, 0, op, 0, 0, dc, 0);
                }
            S.f = 61;
            int y0 = 0;
            memcpy(tg, pg, EPD_NPX);
            memcpy(S.tgt, tg, EPD_NPX);
            {   bp_state t = { .tb = b.tb };            /* (epd-bitplane.h's pack: TB alone) */
                bp_targets(&t, tg, ALL); }
            bp3_copy(&bn, &b);
            bp3_gpu gp[BP_SHMAX];
            for (int s = 0; s < ns; s++) {
                if (bp3_gpu_open(&gp[s], sw[s], sh[s], &m, &p, E.tpl, coldall)) { printf("BP3 gate: shape %dx%d: the GPU setup failed\n", sw[s], sh[s]); return 2; }
                bp3_gpu_load(&gp[s], &b, R);
                bp_gpu_target(&gp[s].b, tg, ALL);
            }
            long lbad = 0, lfr = 0, lsat = 0, hv[3] = { 0, 0, 0 }, nf[3] = { 0, 0, 0 };
            for (int f = 0; f < 160 && !lbad; f++) {
                epd_rect r = { 0, 0, 0, 0 };
                int stretch = f < 64 ? 0 : f < 96 ? 1 : 2;      /* scroll, still, window */
                if (stretch == 0) { if (f % 2 == 0) { y0 = (y0 + 17) % (ph - EPD_H); r = ALL; } }
                else if (stretch == 2 && f % 2 == 0) {
                    y0 = (y0 + 17) % (ph - EPD_H);
                    r = (f / 2) % 2 ? (epd_rect){ 300, 400, 500, 380 } : (epd_rect){ 37, 101, 1500, 1203 };
                }
                if (r.w) {
                    for (int y = r.y; y < r.y + r.h; y++) {
                        memcpy(tg + (size_t)y * EPD_W + r.x, pg + (size_t)(y0 + y) * EPD_W + r.x, (size_t)r.w);
                        memcpy(S.tgt + (size_t)y * EPD_W + r.x, tg + (size_t)y * EPD_W + r.x, (size_t)r.w);
                    }
                    bp_state t = { .tb = b.tb };
                    bp_targets(&t, tg, r);
                    memcpy(bn.tb, b.tb, BP_NW * 4);
                    for (int s = 0; s < ns; s++) bp_gpu_target(&gp[s].b, tg, r);
                }
                uint32_t fc = S.f;
                frame(&p);
                lsat += bp3_clamp_ref(&S);
                static uint32_t W2[EPD_WORDS];
                memcpy(W2, E.tpl, sizeof W2);
                long h = 0;
                bp3_rows(&b, &m, &p, fc, E.tpl, W2, 0, EPD_H, &h);
                hv[stretch] += h; nf[stretch]++;
                size_t fw;
                long d;
                if (!bp3_break) {                       /* (the four-word twin == the twin, words and planes, every frame) */
                    static uint32_t W4[EPD_WORDS];
                    memcpy(W4, E.tpl, sizeof W4);
                    bp3n_rows(&bn, &m, &p, fc, E.tpl, W4, 0, BP_NW, NULL);
                    long vd = bp_wdiff(W2, W4, &fw) + bp3_diff(&b, &bn);
                    if (vd) { printf("BP3 gate: %s R %d %s frame %d: the four-word twin differs (%ld)\n", bp_lname(&p), R, wide ? "wide" : "small", f, vd); lbad++; nvbad++; }
                }
                d = bp_wdiff(W, W2, &fw);
                if (d) { printf("BP3 gate: %s R %d %s frame %d: %ld words differ, first line %zu word %zu: ref %08x bp %08x\n",
                                bp_lname(&p), R, wide ? "wide" : "small", f, d, fw / EPD_LINE, fw % EPD_LINE, W[fw], W2[fw]); lbad++; }
                if (f % 8 == 7 || f == 159) {
                    size_t fi;
                    long sd = bp3_cmp_ref(&b, &S, &fi);
                    nst++;
                    if (sd) {
                        int l, bb, op, st, kc, dc, pp;
                        bp3_get(&b, (int)(fi % EPD_W), (int)(fi / EPD_W), &l, &bb, &op, &st, &kc, &dc, &pp);
                        printf("BP3 gate: %s R %d %s frame %d: state: %ld px differ, first (%zu, %zu): ref hot %04x cold %08x; bp l %d b %d op %d st %d kc %d dc %d p %d\n",
                               bp_lname(&p), R, wide ? "wide" : "small", f, sd, fi % EPD_W, fi / EPD_W, S.hot[fi], S.cold[fi], l, bb, op, st, kc, dc, pp);
                        lbad++;
                    }
                }
                for (int s = 0; s < ns; s++) {
                    static uint32_t W3[EPD_WORDS];
                    double pms, dms;
                    bp3_gpu_frame(&gp[s], fc, &pms, &dms);
                    memcpy(W3, E.tpl, sizeof W3);
                    bp_gpu_words(&gp[s].b, W3);
                    d = bp_wdiff(W, W3, &fw);
                    if (d) { printf("BP3 gate: %s R %d shape %dx%d frame %d: GPU: %ld words differ, first line %zu word %zu: ref %08x gpu %08x\n",
                                    bp_lname(&p), R, sw[s], sh[s], f, d, fw / EPD_LINE, fw % EPD_LINE, W[fw], W3[fw]); lbad++; }
                    if (f % 8 == 7 || f == 159) {
                        long gd = bp3_gpu_cmp(&gp[s], &b, R);
                        if (gd) { printf("BP3 gate: %s R %d shape %dx%d frame %d: GPU state: %ld plane words differ\n", bp_lname(&p), R, sw[s], sh[s], f, gd); lbad++; }
                    }
                    GLenum e = bpgl.glGetError();
                    if (e != GL_NO_ERROR) { printf("BP3 gate: shape %dx%d: GL error 0x%x\n", sw[s], sh[s], e); lbad++; }
                }
                lfr++;
            }
            for (int s = 0; s < ns; s++) bp3_gpu_close(&gp[s]);
            printf("BP3 gate: %-9s R %2d %-5s: %ld frames, %s; clamped %ld; heavy words a frame: scroll %.0f, still %.0f, window %.0f (of %d)\n",
                   bp_lname(&p), R, wide ? "wide" : "small", lfr, lbad ? "DIFFERS" : "equal", lsat,
                   nf[0] ? (double)hv[0] / nf[0] : 0.0, nf[1] ? (double)hv[1] / nf[1] : 0.0, nf[2] ? (double)hv[2] / nf[2] : 0.0, BP_NW);
            nfr += lfr; nbad += lbad; nsat += lsat;
            bp_masks_free(&m);
        }
    }
    printf("BP3 gate: %ld frames, %ld bad, %ld state checks, %ld records clamped; GPU shapes", nfr, nbad, nst, nsat);
    for (int s = 0; s < ns; s++) printf(" %dx%d", sw[s], sh[s]);
    printf("%s (cold %s); four-word twin %s%s\n", ns ? "" : " none", coldall ? "all first" : "on heavy words",
           bp3_break ? "not run" : nvbad ? "DIFFERS" : "equal", bp3_break ? " (a negative control: must be bad)" : "");
    bp3_free(&b); bp3_free(&bn);
    free(pg); free(tg);
    return nbad ? 1 : 0;
}

static long gframes, words_bad;
static double t_gpu, t_cpu;

static int gpu_frame(epd_gpu *g, const epd_stag_par *p)
{
    double t0 = now();
    epd_gpu_stag_frame(g, p, S.f, 0);
    epd_gpu_read(g, 0, GW_);
    t_gpu += now() - t0;
    t0 = now();
    frame(p);
    t_cpu += now() - t0;
    gframes++;
    if (memcmp(GW_, W, sizeof W)) {
        size_t i = 0;
        while (GW_[i] == W[i]) i++;
        long n = 0;
        for (size_t j = 0; j < EPD_WORDS; j++) n += GW_[j] != W[j];
        FAIL("frame %u: %ld words differ, first word %zu (line %zu): gpu %08x cpu %08x", S.f - 1, n, i, i / EPD_LINE, GW_[i], W[i]);
        words_bad++;
        return 1;
    }
    return 0;
}

/* the engine's own frames on the same GPU after the stagger's (the hand-back,
 * glassine-epdd-desktop): the stagger binds its textures on the main shader's units (the
 * tile skip's AT on 2 = the LUT's), and the engine's next draws must not use
 * them. the LUT upload hit AT, GL_INVALID_OPERATION, and the
 * device's GPU drive frames went OFF after every tile-skip run. A DU update's
 * frames as epd-drm makes them (epd-gpu-test's step), compared inside its box
 * (the target holds the stagger's frames elsewhere) */
static void hand_back(epd_gpu *g, const char *when)
{
    static uint32_t cw[EPD_WORDS], gw[EPD_WORDS];
    int cd[2] = { EPD_LINES, 0 }, err0 = epd_gpu_error(g), hb = 0, nf = 0;
    while (E.nrun || E.npend) { epd_schedule(&E); epd_advance(&E, NULL, NULL); }
    epd_gpu_state(g, &E, (epd_rect){ 0, 0, EPD_W, EPD_H });
    memcpy(cw, E.tpl, sizeof cw);               /* (the CPU's frames from the template, as epd-gpu-test) */
    epd_rect r = { 304, 200, 896, 700 };
    /* (a whole panel: epd_submit reads grey at r's place in it. The box alone,
     * stride r.w, read past its end: the device's SIGBUS at the end of every
     * timing run) */
    uint8_t *img = calloc((size_t)EPD_W * EPD_H, 1);
    for (int i = 0; i < r.w * r.h; i++) {
        size_t o = (size_t)(r.y + i / r.w) * EPD_W + r.x + i % r.w;
        img[o] = (uint8_t)(i * 37 % 7 ? 0 : 255);
        if (epd_px_level(&E, o) == (img[o] ? 30 : 0)) img[o] = (uint8_t)(255 - img[o]);
    }
    epd_submit(&E, r, EPD_DU, 0, img, EPD_W);
    int w0 = EPD_DATA0 % EPD_LINE + r.x / 8, w1 = EPD_DATA0 % EPD_LINE + (r.x + r.w) / 8;
    int l0 = EPD_DATA0 / EPD_LINE + EPD_H - r.y - r.h, l1 = EPD_DATA0 / EPD_LINE + EPD_H - r.y;
    for (int f = 0; f < 40; f++) {
        uint32_t prev[EPD_MAX_UPD];
        for (int k = 0; k < EPD_MAX_UPD; k++) prev[k] = E.run[k].nframes ? E.run[k].id : 0;
        if (!epd_schedule(&E)) break;
        epd_rect st[2 * EPD_MAX_UPD];
        int nst = epd_gpu_starts(g, &E, prev, st);
        for (int k = 0; k < nst; k++) epd_gpu_state(g, &E, st[k]);
        epd_render(&E, cw, cd);
        epd_gpu_render(g, &E, 0);
        if (epd_gpu_finish(g, 2000)) { FAIL("hand-back %s: the GPU hung", when); break; }
        epd_gpu_read(g, 0, gw);
        int d = 0;
        for (int l = l0; l < l1 && !d; l++)
            for (int w = w0; w < w1; w++) d |= cw[(size_t)l * EPD_LINE + w] != gw[(size_t)l * EPD_LINE + w];
        if (d && getenv("STAG_HBV")) {
            long nw = 0, fw = -1;
            for (int l = l0; l < l1; l++)
                for (int w = w0; w < w1; w++) if (cw[(size_t)l * EPD_LINE + w] != gw[(size_t)l * EPD_LINE + w]) { nw++; if (fw < 0) fw = (long)l * EPD_LINE + w; }
            printf("  hand-back frame %d: %ld words differ, first line %ld word %ld: cpu %08x gpu %08x\n", f, nw, fw / EPD_LINE, fw % EPD_LINE, cw[fw], gw[fw]);
        }
        hb += d;
        nf++;
        epd_gpu_finishing(g, &E);
        epd_advance(&E, NULL, NULL);
    }
    int err = epd_gpu_error(g);
    free(img);
    if (err0) FAIL("hand-back %s: a GL error left: 0x%x", when, err0);
    if (err) FAIL("hand-back %s: the engine's frames: GL error 0x%x", when, err);
    if (!nf) FAIL("hand-back %s: the update never ran", when);
    if (hb) FAIL("hand-back %s: %d of %d engine frames differ from the CPU's", when, hb, nf);
    printf("hand-back %s: %d engine frames, %s\n", when, nf, err0 || err || hb || !nf ? "BAD" : "clean");
}

static void gpu_part(int frames)
{
    epd_gpu *g = epd_gpu_open(&E);
    if (!g) { printf("no GPU: part 2 skipped\n"); exit(bad ? 1 : 77); }
    if (getenv("GLASSINE_STAG_DEFS") && strstr(getenv("GLASSINE_STAG_DEFS"), "HEAPSTATE")) {
        char why[256];
        hs_buf hb;
        if (hs_alloc(&hb, 4096, why, sizeof why)) { printf("no heap (%s): HEAPSTATE skipped\n", why); exit(bad ? 1 : HS_NOHEAP); }
        hs_free(&hb);
    }
    int so = epd_gpu_stag_open(g);
    if (so == -2) { printf("FAIL: GLES 3.1 but the stagger's setup failed (a shader: above)\n"); exit(1); }
    if (so) { printf("no GLES 3.1: part 2 skipped\n"); exit(bad ? 1 : 77); }
    printf("GPU: %s\n", epd_gpu_renderer(g));
    if (getenv("STAG_BP_GATE")) {               /* (the bit-plane bench's gate alone) */
        if (bp_gate()) bad++;
        epd_gpu_close(g);
        return;
    }
    if (epd_stag_init(&S2)) exit(2);
    const int R = E.op_r ? E.op_r : 9;
    int ph = 1404 * 4;
    uint8_t *pg = page(ph);
    struct { const char *name; int G, n, ctl, pay, dith, speed, K, pr; } sc[] = {
        { "scroll A2/2 full stagger", 2, 1, 0, 0, 1, 6, 0, 0 },
        { "scroll A2/4", 4, 3, 0, 0, 1, 12, 0, 0 },
        { "fling A2/8", 8, 7, 0, 0, 0, 60, 0, 0 },
        { "dial: n changes per frame", 6, 3, 0, 0, 1, 20, 0, 0 },
        { "controller A2/3", 3, 2, 1, 0, 1, 15, 0, 0 },
        { "controller + payback, G 16", 16, 15, 1, 3, 1, 9, 0, 0 },
        { "controller + payback A2/2 (the device default)", 2, 1, 1, 3, 1, 6, 0, 0 },
        { "page flips", 5, 4, 1, 2, 1, -1, 0, 0 },
        { "controller 2 + payback A2/2", 2, 1, 2, 3, 1, 6, 0, 0 },
        { "controller 2 + payback A2/4, flips", 4, 3, 2, 2, 1, -1, 0, 0 },
        { "controller 2 + payback G 16", 16, 15, 2, 3, 1, 9, 0, 0 },
        { "controller 3 + payback A2/2", 2, 1, 3, 3, 1, 6, 8, 0 },
        { "controller 3 A2/2, K 2", 2, 1, 3, 0, 1, 6, 2, 0 },
        { "controller 3 + payback A2/4, flips", 4, 3, 3, 2, 1, -1, 8, 0 },
        { "controller 3 + payback A2/8, K 15", 8, 7, 3, 2, 1, 30, 15, 0 },
        { "controller 3 + payback G 16", 16, 15, 3, 3, 1, 9, 15, 0 },
        /* payrule 1 (the panel's pay-on-arrival, cap 3) */
        { "panel pay: controller A2/2", 2, 1, 1, 0, 1, 6, 0, 1 },
        { "panel pay: controller A2/4, flips", 4, 3, 1, 0, 1, -1, 0, 1 },
        { "panel pay: controller G 16", 16, 15, 1, 0, 1, 9, 0, 1 },
        { "panel pay: controller 2 A2/2", 2, 1, 2, 0, 1, 6, 0, 1 },
        { "panel pay: controller 2 A2/4, flips", 4, 3, 2, 0, 1, -1, 0, 1 },
        { "panel pay: controller 2 G 16", 16, 15, 2, 0, 1, 9, 0, 1 },
        { "panel pay: controller 3 A2/2", 2, 1, 3, 0, 1, 6, 8, 1 },
        { "panel pay: controller 3 A2/4, flips", 4, 3, 3, 0, 1, -1, 8, 1 },
        { "panel pay: controller 3 G 16", 16, 15, 3, 0, 1, 9, 15, 1 },
        { "panel pay (pay 3 ignored), cap 1, ctl 3 A2/2", 2, 1, 3, 3, 1, 6, 8, 2 },
        /* a window, then still (pr 64: the tile skip's idle tiles) */
        { "window: A2/2", 2, 1, 0, 0, 1, 6, 0, 64 },
        { "window: ctl A2/2", 2, 1, 1, 0, 1, 6, 0, 64 },
        { "window: ctl 3 + payback A2/4", 4, 3, 3, 2, 1, 12, 8, 64 },
        { "window: panel pay, ctl 1 A2/2", 2, 1, 1, 0, 1, 6, 0, 1 | 64 },
        /* the arrival top-up (pr 128: htop 2) */
        { "top-up: panel pay, ctl 1 A2/2", 2, 1, 1, 0, 1, 6, 0, 1 | 128 },
        { "top-up: panel pay, ctl 2 A2/4, flips", 4, 3, 2, 0, 1, -1, 0, 1 | 128 },
        { "top-up: panel pay, cap 1, ctl 3 A2/2", 2, 1, 3, 0, 1, 6, 8, 2 | 128 },
        /* A2/1 (G 1, n 1, ctl 3: every pixel decides every frame, a pulse a frame) */
        { "A2/1: controller 3 + payback", 1, 1, 3, 3, 1, 6, 8, 0 },
        { "A2/1: controller 3, K 2, flips", 1, 1, 3, 0, 1, -1, 2, 0 },
        { "A2/1: panel pay, controller 3", 1, 1, 3, 0, 1, 6, 8, 1 },
        { "A2/1: panel pay + top-up, window", 1, 1, 3, 0, 1, 6, 8, 1 | 64 | 128 },
        /* phase groups as 64-px column stripes (pr 512; the word pass and px both take them: stw, epd-gpu.c) */
        { "stripes: controller 3 + payback A2/2", 2, 1, 3, 3, 1, 6, 8, 512 },
        { "stripes: panel pay, ctl 3 A2/2", 2, 1, 3, 0, 1, 6, 8, 1 | 512 },
        { "stripes: panel pay, ctl 3 A2/4, flips", 4, 3, 3, 0, 1, -1, 8, 1 | 512 },
        { "stripes: panel pay, ctl 1 A2/2", 2, 1, 1, 0, 1, 6, 0, 1 | 512 },
        { "stripes: panel pay + top-up, window, ctl 3 A2/2", 2, 1, 3, 0, 1, 6, 8, 1 | 64 | 128 | 512 },
        { "stripes: dither A2/3, ctl 3", 3, 2, 3, 0, 1, 15, 8, 1 | 512 },
        /* the stripes as scattered 64 x 64 blocks (pr 2048 with 512: epd_stag_brank's
         * Bayer groups; G 3, 5, 16: its 256 ranks unevenly cut) */
        { "blocks: controller 3 + payback A2/2", 2, 1, 3, 3, 1, 6, 8, 512 | 2048 },
        { "blocks: panel pay, ctl 3 A2/2", 2, 1, 3, 0, 1, 6, 8, 1 | 512 | 2048 },
        { "blocks: panel pay, ctl 3 A2/4, flips", 4, 3, 3, 0, 1, -1, 8, 1 | 512 | 2048 },
        { "blocks: panel pay, ctl 1 A2/2", 2, 1, 1, 0, 1, 6, 0, 1 | 512 | 2048 },
        { "blocks: panel pay + top-up, window, ctl 3 A2/2", 2, 1, 3, 0, 1, 6, 8, 1 | 64 | 128 | 512 | 2048 },
        { "blocks: dither A2/3, ctl 3", 3, 2, 3, 0, 1, 15, 8, 1 | 512 | 2048 },
        { "blocks: controller 2 + payback A2/5", 5, 4, 2, 3, 1, 12, 0, 512 | 2048 },
        { "blocks: panel pay, ctl 3 G 16", 16, 15, 3, 0, 1, 9, 15, 1 | 512 | 2048 },
        { "blocks: A2/1 (G 1: no groups), ctl 3", 1, 1, 3, 0, 1, 6, 8, 1 | 512 | 2048 },
        /* the stripes as blue-noise ranked cells (pr 4096 with 512: epd_stag_cells; the
         * shape by SC_CELL: log2 w - 3 at bit 13, log2 h at bit 15; ). G 3, 5,
         * 16 and the odd shapes: a tile's cells unevenly cut */
        { "cells: controller 3 + payback A2/2, 8x8", 2, 1, 3, 3, 1, 6, 8, 512 | SC_CELL(8, 8) },
        { "cells: panel pay, ctl 3 A2/2, 16x16", 2, 1, 3, 0, 1, 6, 8, 1 | 512 | SC_CELL(16, 16) },
        { "cells: panel pay, ctl 3 A2/4, flips, 32x32", 4, 3, 3, 0, 1, -1, 8, 1 | 512 | SC_CELL(32, 32) },
        { "cells: panel pay, ctl 1 A2/2, 8x4", 2, 1, 1, 0, 1, 6, 0, 1 | 512 | SC_CELL(8, 4) },
        { "cells: panel pay + top-up, window, ctl 3 A2/2, 16x8", 2, 1, 3, 0, 1, 6, 8, 1 | 64 | 128 | 512 | SC_CELL(16, 8) },
        { "cells: dither A2/3, ctl 3, 8x8", 3, 2, 3, 0, 1, 15, 8, 1 | 512 | SC_CELL(8, 8) },
        { "cells: controller 2 + payback A2/5, 64x8", 5, 4, 2, 3, 1, 12, 0, 512 | SC_CELL(64, 8) },
        { "cells: panel pay, ctl 3 G 16, 8x1", 16, 15, 3, 0, 1, 9, 15, 1 | 512 | SC_CELL(8, 1) },
        { "cells: panel pay, ctl 3 G 16, 32x64", 16, 15, 3, 0, 1, 9, 15, 1 | 512 | SC_CELL(32, 64) },
        { "cells: A2/1 (G 1: no groups), ctl 3, 8x8", 1, 1, 3, 0, 1, 6, 8, 1 | 512 | SC_CELL(8, 8) },
        /* word bayer : G 2 (the checkerboard), G 4 flips, G 3 (classes unevenly
         * cut), ctl 1, dither, G 16 */
        { "wbayer: controller 3 + payback A2/2", 2, 1, 3, 3, 1, 6, 8, 512 | SC_WBAYER },
        { "wbayer: panel pay, ctl 3 A2/4, flips", 4, 3, 3, 0, 1, -1, 8, 1 | 512 | SC_WBAYER },
        { "wbayer: panel pay, ctl 3 A2/3", 3, 2, 3, 0, 1, 6, 8, 1 | 512 | SC_WBAYER },
        { "wbayer: panel pay + top-up, window, ctl 1 A2/2", 2, 1, 1, 0, 1, 6, 0, 1 | 64 | 128 | 512 | SC_WBAYER },
        { "wbayer: dither A2/3, ctl 3", 3, 2, 3, 0, 1, 15, 8, 1 | 512 | SC_WBAYER },
        { "wbayer: panel pay, ctl 3 G 16", 16, 15, 3, 0, 1, 9, 15, 1 | 512 | SC_WBAYER },
        /* the device's daily config : ctl 3, K 8, payrule with
         * cap 3 (pr 1: pcap 3 = the engine's matched_cap), htop 2 (pr 128),
         * stripes 64 (pr 512), dither 0; pr 1024: the targets at frame 0 only
         * (a page that comes in once, then settles). Keep them last: the
         * scenarios above go by index (k == 0, 1, 3) */
        { "daily: scroll A2/2", 2, 1, 3, 0, 0, 6, 8, 1 | 128 | 512 },
        { "daily: static", 2, 1, 3, 0, 0, 0, 8, 1 | 128 | 512 | 1024 },
        { "daily: window", 2, 1, 3, 0, 0, 6, 8, 1 | 64 | 128 | 512 },
        /* (not the daily path: its stripes off, the blue-noise groups; ) */
        { "daily-bn: scroll A2/2, stripe 0", 2, 1, 3, 0, 0, 6, 8, 1 | 128 },
        { "daily-bn: window, stripe 0", 2, 1, 3, 0, 0, 6, 8, 1 | 64 | 128 },
        /* (the daily path with the stripes as blocks: stagger_stripe=blocks) */
        { "daily-blk: scroll A2/2, blocks", 2, 1, 3, 0, 0, 6, 8, 1 | 128 | 512 | 2048 },
        { "daily-blk: static, blocks", 2, 1, 3, 0, 0, 0, 8, 1 | 128 | 512 | 1024 | 2048 },
        { "daily-blk: window, blocks", 2, 1, 3, 0, 0, 6, 8, 1 | 64 | 128 | 512 | 2048 },
        /* (the daily path with the stripes as cells: stagger_stripe=cells8, cells16, cells32) */
        { "daily-cell8: scroll A2/2, cells 8x8", 2, 1, 3, 0, 0, 6, 8, 1 | 128 | 512 | SC_CELL(8, 8) },
        { "daily-cell8: static, cells 8x8", 2, 1, 3, 0, 0, 0, 8, 1 | 128 | 512 | 1024 | SC_CELL(8, 8) },
        { "daily-cell8: window, cells 8x8", 2, 1, 3, 0, 0, 6, 8, 1 | 64 | 128 | 512 | SC_CELL(8, 8) },
        { "daily-cell16: scroll A2/2, cells 16x16", 2, 1, 3, 0, 0, 6, 8, 1 | 128 | 512 | SC_CELL(16, 16) },
        { "daily-cell16: static, cells 16x16", 2, 1, 3, 0, 0, 0, 8, 1 | 128 | 512 | 1024 | SC_CELL(16, 16) },
        { "daily-cell16: window, cells 16x16", 2, 1, 3, 0, 0, 6, 8, 1 | 64 | 128 | 512 | SC_CELL(16, 16) },
        { "daily-cell32: scroll A2/2, cells 32x32", 2, 1, 3, 0, 0, 6, 8, 1 | 128 | 512 | SC_CELL(32, 32) },
        { "daily-cell32: static, cells 32x32", 2, 1, 3, 0, 0, 0, 8, 1 | 128 | 512 | 1024 | SC_CELL(32, 32) },
        { "daily-cell32: window, cells 32x32", 2, 1, 3, 0, 0, 6, 8, 1 | 64 | 128 | 512 | SC_CELL(32, 32) },
        { "daily-wbayer: scroll A2/2", 2, 1, 3, 0, 0, 6, 8, 1 | 128 | 512 | SC_WBAYER },
        { "daily-wbayer: static", 2, 1, 3, 0, 0, 0, 8, 1 | 128 | 512 | 1024 | SC_WBAYER },
        { "daily-wbayer: window", 2, 1, 3, 0, 0, 6, 8, 1 | 64 | 128 | 512 | SC_WBAYER },
        /* (pr bits 20-23): the lanes' requests, in both modes
         * (runs: an evict is a drain, the exit and a new enter): arm mid-scroll (8:
         * the right part joins at a third, its edge mid-line: the arm's pad), a
         * popup (1: evicted at a quarter, drawn into the engine's state, back at
         * half), a pen piece in the box (2: the engine's DU there), a run's end
         * and a later page turn (4: everything evicted, a GC16 over the panel,
         * the next run at once) */
        { "s5: arm mid-scroll, stripes", 2, 1, 3, 0, 0, 6, 8, 1 | 128 | 512 | 8 << 20 },
        { "s5: arm mid-scroll, blocks A2/4", 4, 3, 3, 0, 1, 12, 8, 1 | 512 | 2048 | 8 << 20 },
        { "s5: popup, stripes", 2, 1, 3, 0, 0, 6, 8, 1 | 128 | 512 | 1 << 20 },
        { "s5: popup, blocks A2/4", 4, 3, 3, 0, 1, 12, 8, 1 | 512 | 2048 | 1 << 20 },
        { "s5: popup, A2/1 window", 1, 1, 3, 0, 1, 6, 8, 1 | 64 | 1 << 20 },
        { "s5: pen piece, stripes", 2, 1, 3, 0, 0, 6, 8, 1 | 128 | 512 | 2 << 20 },
        { "s5: pen piece, cells 16x16", 2, 1, 3, 0, 0, 6, 8, 1 | 128 | 512 | SC_CELL(16, 16) | 2 << 20 },
        { "s5: run end + page turn, stripes", 2, 1, 3, 0, 0, 6, 8, 1 | 128 | 512 | 4 << 20 },
        { "s5: run end + page turn, A2/1", 1, 1, 3, 0, 1, 6, 8, 1 | 4 << 20 },
        /* (the daily default since stripe 0, the blue noise; a word's lanes in several groups) */
        { "s5: arm mid-scroll, blue noise", 2, 1, 3, 0, 0, 6, 8, 1 | 128 | 8 << 20 },
        { "s5: popup, blue noise", 2, 1, 3, 0, 0, 6, 8, 1 | 128 | 1 << 20 },
        { "s5: popup, wbayer", 2, 1, 3, 0, 0, 6, 8, 1 | 128 | 512 | SC_WBAYER | 1 << 20 },
        { "s5: pen piece, blue noise A2/4", 4, 3, 3, 0, 1, 12, 8, 1 | 2 << 20 },
        { "s5: run end + page turn, blue noise", 2, 1, 3, 0, 0, 6, 8, 1 | 128 | 4 << 20 },
    };
    int tonly = getenv("STAG_TIME_ONLY") != NULL;
    LN = epd_gpu_stag_lanes(g);
    B4 = getenv("STAG_CTL4") && atoi(getenv("STAG_CTL4")) > 0;
    if (B4) {
        if (!LN) { printf("STAG_CTL4: lanes only (GLASSINE_STAGGER_STATE=lanes GLASSINE_CLAIM_STATE=packed)\n"); exit(2); }
        if (epd_gpu_bp4_open(g)) { printf("FAIL: stagger_ctl 4: the GPU's programs (above)\n"); exit(1); }
        epd_gpu_stag_timer = !getenv("STAG_NOTIMER");   /* (the passes' GPU timers per scenario) */
        if (bp4_alloc(&T4) || !(DSH = calloc((size_t)EPD_PK_S * (EPD_H + EPD_PK_DPAD), 4))) exit(2);
        T4.kidle = 1;                           /* (ctl 4's idle: ctl 3's per-pixel K) */
        bp4_lane_park(&T4);
        printf("stagger_ctl 4: the GPU against the twin (epd-bp4.h)\n");
    }
    TG = calloc(EPD_NPX, 1); LD = calloc(EPD_NPX, 1); MK = calloc(EPD_NPX, 1); PU = calloc(EPD_NPX, 1);
    if (!TG || !LD || !MK || !PU) exit(2);
    printf("stagger state: %s\n", LN ? "lanes (the pass on the engine's D and T)" : "runs");
    dsy(SY_B); tsy(SY_B);
    epd_assume(&E, 30);
    if (LN) epd_stag_lanes_park(&S);
    else epd_stag_enter(&S);
    epd_gpu_stag_enter(g, &S);
    /* parked: an engine state with greys and a DC record everywhere, of which
     * only the targets' rects come in (the rest must stay inert) */
    for (size_t i = 0; i < (size_t)EPD_W * EPD_H; i++) {
        epd_px_put(&E, i, EPD_F_LEVEL | EPD_F_DC, 2 * (int)(i % 16), 0, 0, 0, (int)(i % 7) - 3, 0);
    }
    tsy(SY_E); dsy(SY_E);
    ln_resolve(ALL);
    if (!B4 && state_cmp(g, "enter (all parked)")) exit(1);
    int y0 = 0;
    if (tonly) {                                /* the timings only: a full panel of their state loaded */
        epd_stag_par p = { .G = 2, .n = 1, .dither = 1, .R = R };
        epd_rect all = { 0, 0, EPD_W, EPD_H };
        if (LN) {                               /* (lanes: armed with a page) */
            memcpy(TG, pg, EPD_NPX);
            tg_set(g, all, 0);
        } else {
            epd_stag_load(&S, &E, &p, all);
            epd_gpu_stag_load(g, &S, &p, all);
        }
        epd_gpu_finish(g, 1000);
    }
    const char *only = getenv("STAG_ONLY");     /* (a name's substring: those scenarios only) */
    int ndaily = 0;                             /* (daily scenarios run: their hand-back after the table) */
    for (unsigned k = 0; k < (tonly ? 0 : sizeof sc / sizeof *sc); k++) {
        if (only && !strstr(sc[k].name, only)) continue;
        long b0 = bad;
        y0 = 0;                                 /* (each scenario from the page top: the same alone or in the table) */
        epd_stag_par p = { .G = sc[k].G, .n = sc[k].n, .ctl = sc[k].ctl, .pay = sc[k].pay, .K = sc[k].K, .dither = sc[k].dith, .R = R,
                           .payrule = (sc[k].pr & 3) != 0, .pcap = (sc[k].pr & 3) == 2 ? 1 : 3,
                           .htop = sc[k].pr >> 7 & 1 ? 2 : 0, .stripe = sc[k].pr >> 9 & 1 ? 64 : 0, .blocks = sc[k].pr >> 11 & 1,
                           .cellw = sc[k].pr >> 12 & 1 ? 8 << (sc[k].pr >> 13 & 3) : 0, .cellh = sc[k].pr >> 12 & 1 ? 1 << (sc[k].pr >> 15 & 7) : 0,
                           .cellord = sc[k].pr >> 18 & 1 ? EPD_STAG_CORD_WBAYER : 0, .lanes = LN };
        if (B4) {                               /* (ctl 4: the scenarios it takes) */
            if (k == 3 || !bp4_par_ok(&p) || p.ctl != 3) continue;
            p.ctl = 4;
            p.sidle = getenv("STAG_SIDLE") != NULL;     /* (stagger_idle=shared) */
        }
        long odd = 0;                           /* (payrule: pixels seen with a budget) */
        long gl0 = 0;                           /* (greys at entry: a flip from the rails keeps no budget) */
        for (size_t i = 0; i < (size_t)EPD_W * EPD_H; i += 97) { int l = EPD_STAG_LEVEL(S.cold[i]); gl0 += l && l != 30; }
        /* the event (pr bits 20-22, frames >= 80): evs 0 before, 1 evicting
         * (runs: draining), 2 the box held by the engine, 3 back */
        int s5 = sc[k].pr >> 20 & 15, ev = (s5 & 7) && frames >= 80, evs = 0, evf = 0, efc = frames / 2, endev = 0, brk = 0;
        epd_rect bx = s5 & 1 ? (epd_rect){ 905, 403, 333, 211 } : s5 & 2 ? (epd_rect){ 1301, 707, 37, 23 } : ALL;
        epd_rect pb = LN ? epd_stag_lane_pad(bx) : bx, pend = ALL;
        if (LN) { ln_resolve(ALL); memset(MK, 1, EPD_NPX); }   /* (a run's start: the engine's finished words resolved) */
        for (int f = 0; f < frames; f++) {
            if (k == 3) { p.n = 1 + f % 5; p.G = p.n + 1 + (f / 7) % 3; }
            int full = 0;                       /* (new content over the whole panel this frame) */
            if (ev && evs == 0 && f == frames / 4) {
                evs = 1; evf = f;
                if (LN) {
                    pb = ln_evict(g, bx);
                    for (int y = pb.y; y < pb.y + pb.h; y++) memset(MK + (size_t)y * EPD_W + pb.x, 0, (size_t)pb.w);
                } else p.drain = 1;
            }
            if (evs == 1) {
                int ready;
                if (LN) {
                    long cu, gu;
                    ln_acks(g, pb, &cu, &gu);
                    if (!cu != !gu) { FAIL("%s: frame %d: acks: %ld (cpu) / %ld (gpu) words open", sc[k].name, f, cu, gu); brk = 1; break; }
                    ready = !cu;
                } else ready = f - evf >= 20;
                if (!ready && f - evf > 2 * p.G + 24) { FAIL("%s: no acks %d frames after the evict", sc[k].name, f - evf); brk = 1; break; }
                if (ready) {
                    if (LN) ln_takeover(g, pb, sc[k].name);
                    else {
                        if (state_cmp(g, sc[k].name)) { brk = 1; break; }
                        if (epd_stag_active(&S)) FAIL("%s: %ld pixels active after the drain", sc[k].name, epd_stag_active(&S));
                        epd_stag_exit(&S, &E);
                    }
                    s5_paint(s5, bx, pb, pg, &y0, ph);
                    if (!LN) { epd_stag_enter(&S); epd_gpu_stag_enter(g, &S); p.drain = 0; full = 1; }
                    printf("  %s: the box (%d, %d, %d x %d) the engine's %d frames after the %s\n", sc[k].name, pb.x, pb.y, pb.w, pb.h, f - evf, LN ? "evict" : "drain");
                    evs = 2;
                    if (s5 & 4) efc = f;        /* (the page turn: the next run at once) */
                }
            }
            if (evs == 2 && f >= efc) {         /* the box back: its content again (lanes: armed) */
                evs = 3; full = 1;
                if (LN) memset(MK, 1, EPD_NPX);
            }
            int upd = f % 3 == 0 && !((sc[k].pr & 64) && f >= frames / 2 && f > 0) && !((sc[k].pr & 1024) && f > 0);   /* new content every 3rd frame (a 28 fps app) */
            if (upd || full) {
                epd_rect r = { 0, 0, EPD_W, EPD_H };
                if (upd) {
                    if ((sc[k].pr & 64) && f > 0)   /* (pr 64: a moving window only, then still: the tile skip) */
                        r = (epd_rect){ 200 + (f * 13) % 400, 150 + (f * 7) % 300, 500, 380 };
                    if (sc[k].speed < 0) {
                        if (f % 30 == 0) y0 = (y0 + 1404) % (ph - EPD_H);
                    } else
                        y0 = (y0 + sc[k].speed) % (ph - EPD_H);
                    if (k == 1) r = (epd_rect){ 37, 101, 1500, 1203 };   /* a window, odd edges */
                    if (k == 0 && f == 0) r = (epd_rect){ 0, 300, EPD_W, 900 };  /* parked rows above and below, first */
                    if (s5 & 8) r = f < frames / 3 ? (epd_rect){ 0, 0, 901, EPD_H } : ALL;   /* (arm mid-scroll: the rest joins) */
                }
                if (full) r = ALL;
                for (int y = r.y; y < r.y + r.h; y++)
                    memcpy(TG + (size_t)y * EPD_W + r.x, pg + (size_t)(y0 + y) * EPD_W + r.x, (size_t)r.w);
                if (evs == 2)                   /* (the box held: its content stays; lanes: masked anyway) */
                    for (int y = pb.y; y < pb.y + pb.h; y++) memcpy(TG + (size_t)y * EPD_W + pb.x, PU + (size_t)y * EPD_W + pb.x, (size_t)pb.w);
                tg_in(g, &p, r, LN && (evs == 1 || evs == 2 || endev));
            }
            if (f == frames - 20) {             /* the end: lanes: everything evicted; runs: drain */
                if (LN) {
                    pend = ln_evict(g, ALL);
                    memset(MK, 0, EPD_NPX);
                    endev = 1;
                } else p.drain = 1;
            }
            if (gpu_frame(g, &p)) { printf("  (frame %d of %s)\n", f, sc[k].name); brk = 1; break; }
            if (sc[k].pr & 3)                   /* (every frame: a budget lives a frame or two) */
                for (size_t i = 0; i < (size_t)EPD_W * EPD_H; i += 97) odd += EPD_STAG_LEVEL(S.cold[i]) & 1;
            if ((f % 8 == 7 || getenv("STAG_CMP1")) && state_cmp(g, sc[k].name)) { printf("  (state first differs after frame %d of %s)\n", f, sc[k].name); brk = 1; break; }
        }
        if (ev && !brk && evs != 3) FAIL("%s: the event did not finish (state %d)", sc[k].name, evs);
        state_cmp(g, sc[k].name);
        long act = epd_stag_active(&S);
        if (act) FAIL("%s: %ld pixels still active after the drain", sc[k].name, act);
        if (LN && endev) {                      /* lanes: every word acked, the panel taken over */
            long cu, gu;
            ln_acks(g, pend, &cu, &gu);
            if (cu || gu) FAIL("%s: %ld (cpu) / %ld (gpu) words not acked at the end", sc[k].name, cu, gu);
            ln_takeover(g, pend, sc[k].name);
        }
        /* (G 16: n 15 pays need + b in one step, a budget is never kept; page flips
         * in 30 frames alone: one page, no arrival with a budget yet; flips from a
         * panel on the rails: rail to rail pays in the drive, budgets come from greys) */
        /* (pr 1024, static: one page from wherever the panel was, often greys or
         * the same rail: no budget needs to arise) */
        if (!B4 && (sc[k].pr & 3) && !(sc[k].pr & 1024) && sc[k].G < 16 && !odd && (sc[k].speed >= 0 || (frames > 30 && gl0 > 10))) FAIL("%s: no pixel with a payback budget seen", sc[k].name);
        if (B4 && epd_gpu_bp4_ms[3] > 0) {      /* (ctl 4: the passes' GPU timers, as the daemon's stagger_timer=1 line) */
            printf("  %s: ctl 4 %.0f frames, avg pack %.2f visit %.2f drive %.2f ms\n", sc[k].name, epd_gpu_bp4_ms[3],
                   epd_gpu_bp4_ms[0] / epd_gpu_bp4_ms[3], epd_gpu_bp4_ms[1] / epd_gpu_bp4_ms[3], epd_gpu_bp4_ms[2] / epd_gpu_bp4_ms[3]);
            epd_gpu_bp4_ms[0] = epd_gpu_bp4_ms[1] = epd_gpu_bp4_ms[2] = epd_gpu_bp4_ms[3] = 0;
        }
        if (sc[k].pr & 3) printf("%-30s %s (budget pixels seen %ld)\n", sc[k].name, bad == b0 ? "equal" : "DIFFERS", odd);
        else printf("%-30s %s\n", sc[k].name, bad == b0 ? "equal" : "DIFFERS");
        ndaily += !strncmp(sc[k].name, "daily", 5);
    }
    if (ndaily) hand_back(g, "after the daily scenarios");   /* (the engine's frames after a daily-config run, same GPU) */
    if (gframes) printf("GPU vs CPU: %ld frames, %ld bad; GPU %.2f ms a frame (incl. readback), CPU %.2f\n", gframes, words_bad,
           t_gpu / gframes * 1e3, t_cpu / gframes * 1e3);
    /* timing as the stream runs it: a frame, then its wait (epd_gpu_finish);
     * a full-panel target upload every 3rd frame, timed apart */
    if (LN) ln_resolve(ALL);                    /* (the hand-back's engine run: its finished words) */
    for (int G = 2; G <= 8; G *= 2) {
        epd_stag_par p = { .G = G, .n = G - 1, .dither = 1, .R = R, .lanes = LN };
        double tf[60], tu = 0, tfmax = 0, tsum = 0;
        int nu = 0;
        for (int f = 0; f < 60; f++) {
            if (f % 3 == 0) {
                y0 = (y0 + 23) % (ph - EPD_H);
                memcpy(TG, pg + (size_t)y0 * EPD_W, (size_t)EPD_W * EPD_H);
                double t0 = now();
                tg_set(g, (epd_rect){ 0, 0, EPD_W, EPD_H }, 0);
                epd_gpu_finish(g, 1000);
                tu += now() - t0; nu++;
            }
            double t0 = now();
            epd_gpu_stag_frame(g, &p, S.f++, 0);
            epd_gpu_finish(g, 1000);
            tf[f] = now() - t0;
            tsum += tf[f];
            if (tf[f] > tfmax) tfmax = tf[f];
        }
        printf("timing G %d n %d%s: frame %.2f ms avg, %.2f max; full target upload %.2f ms\n", G, G - 1, LN ? " (lanes)" : "",
               tsum / 60 * 1e3, tfmax * 1e3, tu / nu * 1e3);
    }
    {                                           /* GPU time (timer queries) per scenario kind */
        static const struct { const char *n; int G, ctl, pay, scroll, pr; } tc[] = {
            { "A2/2 scroll", 2, 0, 0, 1, 0 }, { "A2/4 scroll", 4, 0, 0, 1, 0 }, { "A2/8 scroll", 8, 0, 0, 1, 0 },
            { "A2/4 static", 4, 0, 0, 0, 0 }, { "ctl A2/4 + pay scroll", 4, 1, 2, 1, 0 },
            { "ctl A2/2 + pay scroll", 2, 1, 3, 1, 0 }, { "ctl 2 A2/2 + pay scroll", 2, 2, 3, 1, 0 },
            { "ctl 3 A2/2 + pay scroll", 2, 3, 3, 1, 0 },
            { "ctl A2/2 scroll", 2, 1, 0, 1, 0 }, { "ctl A2/2 static", 2, 1, 0, 0, 0 },
            /* the device's (ctl 1, stagger_pay=panel) */
            { "dev: ctl A2/2 panel scroll", 2, 1, 0, 1, 1 }, { "dev: ctl A2/2 panel static", 2, 1, 0, 0, 1 },
            /* a 500 x 380 window scrolls, the rest still (scroll 2: the tile skip's case) */
            { "dev: ctl A2/2 panel window", 2, 1, 0, 2, 1 },
            /* the same at ctl 3 (a pulse instead of the idle frame, K 8) */
            { "dev: ctl3 A2/2 panel scroll", 2, 3, 0, 1, 1 }, { "dev: ctl3 A2/2 panel static", 2, 3, 0, 0, 1 },
            /* A2/1 (G 1, n 1, ctl 3) */
            { "dev: A2/1 panel scroll", 1, 3, 0, 1, 1 }, { "dev: A2/1 panel static", 1, 3, 0, 0, 1 },
            { "dev: A2/1 panel window", 1, 3, 0, 2, 1 },
            /* pr 2: the daily config (ctl 3, K 8, payrule cap 3, htop 2, dither 0,
             * stripes 64 unless STAG_STRIPE says otherwise; the "dev:" rows above
             * are not it: cap 7, dither 1, no htop, stripes only by STAG_STRIPE).
             * One line each, pass ms avg / max per frame */
            { "daily: ctl3 A2/2 scroll", 2, 3, 0, 1, 2 }, { "daily: ctl3 A2/2 static", 2, 3, 0, 0, 2 },
            { "daily: ctl3 A2/2 window", 2, 3, 0, 2, 2 },
            /* pr 3, by name only (STAG_TONLY=fixed; ): the daily path's
             * layout (stripes 64 unless STAG_STRIPE says otherwise, dither 0) at
             * ctl 0, no payback: today's pass beside the bit-plane bench
             * (STAG_BITPLANE=<shape>: these rows run the bit-plane pass instead) */
            { "fixed: ctl0 A2/2 scroll", 2, 0, 0, 1, 3 }, { "fixed: ctl0 A2/2 static", 2, 0, 0, 0, 3 },
            { "fixed: ctl0 A2/2 window", 2, 0, 0, 2, 3 },
            { "fixed: ctl0 A2/4 scroll", 4, 0, 0, 1, 3 }, { "fixed: ctl0 A2/4 static", 4, 0, 0, 0, 3 },
            { "fixed: ctl0 A2/4 window", 4, 0, 0, 2, 3 }, };
        bp_gpu bpt;                             /* (the bit-plane pass of the fixed rows: STAG_BITPLANE) */
        int bpon = 0, bpw = 0, bph = 0, bpG = 0;
        bp_state bps = { 0 };
        {
            const char *v = getenv("STAG_BITPLANE");
            char sb[32];
            if (v && *v && strcmp(v, "none")) {
                snprintf(sb, sizeof sb, "%s", v);
                char *t = strtok(sb, " ,");
                if (!t || bp_shape_parse(t, &bpw, &bph)) { printf("STAG_BITPLANE=%s: no shape\n", v); exit(2); }
                if (bp_alloc(&bps)) exit(2);
                bpon = 1;
            }
        }
        epd_gpu_stag_timer = !getenv("STAG_NOTIMER");    /* (STAG_NOTIMER: no timer queries, a lockup hunt) */
        for (unsigned k = 0; k < sizeof tc / sizeof *tc; k++) {
            {                                   /* (STAG_TONLY: a timing name's substring, those only; "!s": all but those) */
                const char *to = getenv("STAG_TONLY");
                if (to && (*to == '!' ? strstr(tc[k].n, to + 1) != NULL : !strstr(tc[k].n, to))) continue;
                if (tc[k].pr == 3 && (!to || *to == '!')) continue;     /* (the fixed rows: by name only) */
            }
            int daily = tc[k].pr == 2, fixed = tc[k].pr == 3, p2 = daily || fixed;
            const char *ss = getenv("STAG_STRIPE");     /* (0, 64, "blocks": stripe 64 as blocks, "cells[N|WxH]": as cells, "wbayer") */
            int sblk = ss && !strcmp(ss, "blocks"), scw = 0, sch = 0, swb = ss && !strcmp(ss, "wbayer");   /* (wbayer: 8 x 1 cells, Bayer order) */
            if (ss && !strncmp(ss, "cells", 5) && epd_stag_cells_parse(ss, &scw, &sch)) { printf("STAG_STRIPE=%s: no cell shape\n", ss); exit(2); }
            if (swb) scw = 8, sch = 1;
            epd_stag_par p = { .G = tc[k].G, .n = tc[k].G > 1 ? tc[k].G - 1 : 1, .ctl = tc[k].ctl, .pay = tc[k].pay, .K = 8, .dither = !p2, .R = R,
                               .payrule = tc[k].pr == 1 || daily, .pcap = daily ? 3 : EPD_STAG_PCAP, .htop = daily ? 2 : 0,
                               .stripe = sblk || scw ? 64 : ss ? atoi(ss) : p2 ? 64 : 0, .blocks = sblk,
                               .cellw = scw, .cellh = sch, .cellord = swb ? EPD_STAG_CORD_WBAYER : 0, .lanes = LN };   /* (the word pass and px) */
            double fmax = 0, fsum = 0;          /* (daily: per frame, the timer's or the wall's) */
            int ftimer = 1;
            memset(epd_gpu_stag_ms, 0, sizeof epd_gpu_stag_ms);
            int bpm = fixed && bpon;            /* (this row on the bit-plane pass) */
            int bpm3 = daily && bpon;           /* (a daily row on the ctl 3 bit-plane pass) */
            if (bpm3) {
                if (!bp3_par_ok(&p)) { printf("BP3 %-24s %s: not one the ctl 3 planes take: skipped\n", tc[k].n, ss ? ss : "64"); continue; }
                if (bpG) { bp_gpu_close(&bpt); bpG = 0; }
                if (getenv("STAG_BP3_CPU")) {   /* (the planes on the CPU, no GPU: STAG_BP3_CPU="s n" the twins, STAG_BP3_NT threads, STAG_BP3_CPUS pins) */
                    char ib[32], nb[64], cb[64];
                    int cpus[16], ncpu = 0;
                    snprintf(cb, sizeof cb, "%s", getenv("STAG_BP3_CPUS") ? getenv("STAG_BP3_CPUS") : "");
                    for (char *t = strtok(cb, " ,"); t && ncpu < 16; t = strtok(NULL, " ,")) cpus[ncpu++] = atoi(t);
                    bp_masks m;
                    bp3_state b3;
                    static uint32_t WC[EPD_WORDS];
                    if (bp3_alloc(&b3) || bp_masks_make(&m, &p)) exit(2);
                    epd_gpu_stag_read(g, &S2, ALL, 1);
                    snprintf(ib, sizeof ib, "%s", getenv("STAG_BP3_CPU"));
                    char *isv, *nsv;
                    for (char *it = strtok_r(ib, " ,", &isv); it; it = strtok_r(NULL, " ,", &isv)) {
                        int vec = *it == 'n';
                        snprintf(nb, sizeof nb, "%s", getenv("STAG_BP3_NT") ? getenv("STAG_BP3_NT") : "1 2 4 8");
                        for (char *nt_ = strtok_r(nb, " ,", &nsv); nt_; nt_ = strtok_r(NULL, " ,", &nsv)) {
                            int nt = atoi(nt_);
                            nt = nt < 1 ? 1 : nt > 16 ? 16 : nt;
                            bp3_from_ref(&b3, &S2);
                            {   bp_state t = { .tb = b3.tb }; bp_targets(&t, TG, ALL); }
                            int yy = y0;
                            double fs = 0, fx = 0, ps = 0;
                            int nup = 0;
                            uint32_t fc = S.f;
                            for (int f = 0; f < 48; f++) {
                                double t0 = now();
                                if ((tc[k].scroll == 1 || tc[k].scroll == 2) && f % 2 == 0) {
                                    epd_rect r = tc[k].scroll == 1 ? ALL : (epd_rect){ 300, 400, 500, 380 };
                                    yy = (yy + 17) % (ph - EPD_H);
                                    for (int y = r.y; y < r.y + r.h; y++)
                                        memcpy(TG + (size_t)y * EPD_W + r.x, pg + (size_t)(yy + y) * EPD_W + r.x, (size_t)r.w);
                                    bp_state t = { .tb = b3.tb };
                                    bp_targets(&t, TG, r);
                                    nup++; ps += now() - t0;
                                }
                                memcpy(WC, E.tpl, sizeof WC);
                                bp3c_frame(&b3, &m, &p, fc++, WC, vec, nt, cpus, ncpu);
                                double d = (now() - t0) * 1e3;
                                fs += d; if (d > fx) fx = d;
                            }
                            printf("BP3 cpu: %s  %s %s threads %d%s%s frame %.2f ms avg, %.2f max; pack %.2f ms per upload (%d) (48 frames, wall clock, incl. the pack)\n",
                                   tc[k].n, bp_lname(&p), vec ? "neon4" : "scalar", nt, ncpu ? " cpus " : "", ncpu ? getenv("STAG_BP3_CPUS") : "",
                                   fs / 48, fx, nup ? ps / nup * 1e3 : 0.0, nup);
                        }
                    }
                    bp_masks_free(&m);
                    bp3_free(&b3);
                    continue;
                }
                bp_masks m;
                bp3_state b3;
                bp3_gpu g3;
                int cold = getenv("STAG_BP3_COLD") ? atoi(getenv("STAG_BP3_COLD")) : 0;
                if (bp3_alloc(&b3) || bp_masks_make(&m, &p) || bp3_gpu_open(&g3, bpw, bph, &m, &p, E.tpl, cold)) { printf("BP3: the GPU setup failed\n"); exit(1); }
                bp_masks_free(&m);
                epd_gpu_stag_read(g, &S2, ALL, 1);      /* (today's pass's state, untimed) */
                bp3_from_ref(&b3, &S2);
                bp3_gpu_load(&g3, &b3, R);
                bp_gpu_target(&g3.b, TG, ALL);
                bpgl.glFinish();
                double dsum = 0, dmax = 0, psum = 0;
                int nup = 0;
                for (int f = 0; f < 48; f++) {
                    if (tc[k].scroll == 1 && f % 2 == 0) {
                        y0 = (y0 + 17) % (ph - EPD_H);
                        memcpy(TG, pg + (size_t)y0 * EPD_W, (size_t)EPD_W * EPD_H);
                        bp_gpu_target(&g3.b, TG, ALL), nup++;
                    } else if (tc[k].scroll == 2 && f % 2 == 0) {
                        epd_rect wr = { 300, 400, 500, 380 };
                        y0 = (y0 + 17) % (ph - EPD_H);
                        for (int y = wr.y; y < wr.y + wr.h; y++)
                            memcpy(TG + (size_t)y * EPD_W + wr.x, pg + (size_t)(y0 + y) * EPD_W + wr.x, (size_t)wr.w);
                        bp_gpu_target(&g3.b, TG, wr), nup++;
                    }
                    double pd, dd, w0 = now();
                    bp3_gpu_frame(&g3, S.f++, &pd, &dd);
                    if (dd < 0) { bpgl.glFinish(); dd = (now() - w0) * 1e3; pd = 0; ftimer = 0; }
                    dsum += dd; psum += pd;
                    if (dd > dmax) dmax = dd;
                    fsum += dd + pd;
                    if (dd + pd > fmax) fmax = dd + pd;
                    if (getenv("STAG_TIME_SERIES")) printf("  BP3 %s f %d: drive %.2f, pack %.2f ms\n", tc[k].n, f, dd, pd);
                }
                GLenum e = bpgl.glGetError();
                printf("BP3 %s  %s shape %dx%d cold %s drive %.2f ms avg, %.2f max; pack %.2f ms per upload (%d); frame %.2f ms avg, %.2f max (48 frames, %s)%s\n",
                       tc[k].n, bp_lname(&p), bpw, bph, cold ? "all" : "heavy", dsum / 48, dmax, nup ? psum / nup : 0.0, nup, fsum / 48, fmax,
                       ftimer ? "GPU timer" : "wall clock", e != GL_NO_ERROR ? " GL ERROR" : "");
                bp3_gpu_close(&g3);
                bp3_free(&b3);
                continue;
            }
            double bdsum = 0, bdmax = 0, bpsum = 0;
            int bnup = 0;
            if (bpm && !bp_layout_ok(&p)) { printf("BP %-24s %s: not a layout the bit planes take: skipped\n", tc[k].n, ss ? ss : "64"); continue; }
            if (bpm && bpG != p.G) {            /* (a new G: its masks; the state from today's pass's, no step in flight; untimed) */
                bp_masks m;
                if (bpG) bp_gpu_close(&bpt);
                if (bp_masks_make(&m, &p) || bp_gpu_open(&bpt, bpw, bph, &m, E.tpl)) { printf("BP: the GPU setup failed\n"); exit(1); }
                bp_masks_free(&m);
                epd_gpu_stag_read(g, &S2, ALL, 1);
                memset(bps.lw, 0, BP_NW * 4); memset(bps.lg, 0, BP_NW * 4); memset(bps.a, 0, BP_NW * 4);
                for (int y = 0; y < EPD_H; y++)
                    for (int x = 0; x < EPD_W; x++) {
                        int b_, l = epd_stag_ldec(EPD_STAG_LEVEL(S2.cold[(size_t)y * EPD_W + x]), &b_);
                        bp_set_level(&bps, x, y, l, 0);
                    }
                bp_gpu_load(&bpt, &bps);
                bp_gpu_target(&bpt, TG, ALL);   /* (its pack goes with the row's first frame: a full panel either way at a scroll) */
                bpgl.glFinish();
                bpG = p.G;
            }
            for (int f = 0; f < 48; f++) {
                if (tc[k].scroll == 1 && f % 2 == 0) { /* a 42 fps scroll */
                    y0 = (y0 + 17) % (ph - EPD_H);
                    memcpy(TG, pg + (size_t)y0 * EPD_W, (size_t)EPD_W * EPD_H);
                    if (bpm) bp_gpu_target(&bpt, TG, ALL), bnup++;
                    else tg_set(g, (epd_rect){ 0, 0, EPD_W, EPD_H }, 0);
                } else if (tc[k].scroll == 2 && f % 2 == 0) {
                    epd_rect wr = { 300, 400, 500, 380 };
                    y0 = (y0 + 17) % (ph - EPD_H);
                    for (int y = wr.y; y < wr.y + wr.h; y++)
                        memcpy(TG + (size_t)y * EPD_W + wr.x, pg + (size_t)(y0 + y) * EPD_W + wr.x, (size_t)wr.w);
                    if (bpm) bp_gpu_target(&bpt, TG, wr), bnup++;
                    else tg_set(g, wr, 0);
                }
                if (bpm) {                      /* the pack (on an upload) + the drive pass, each on the timer */
                    double pd, dd, w0 = now();
                    bp_gpu_frame(&bpt, S.f++, &pd, &dd);
                    if (dd < 0) { bpgl.glFinish(); dd = (now() - w0) * 1e3; pd = 0; ftimer = 0; }
                    bdsum += dd; bpsum += pd;
                    if (dd > bdmax) bdmax = dd;
                    fsum += dd + pd;
                    if (dd + pd > fmax) fmax = dd + pd;
                    if (getenv("STAG_TIME_SERIES")) printf("  BP %s f %d: drive %.2f, pack %.2f ms\n", tc[k].n, f, dd, pd);
                    continue;
                }
                double m0 = epd_gpu_stag_ms[0], m2 = epd_gpu_stag_ms[2], w0 = now();
                epd_gpu_stag_frame(g, &p, S.f++, 0);
                if (getenv("STAG_TIME_SERIES"))         /* (each frame's pass ms: a settling page's curve) */
                    printf("  %s f %d: %.2f ms\n", tc[k].n, f, epd_gpu_stag_ms[0] - m0);
                if (p2) {                       /* (no timer queries: the wall clock to the frame's end) */
                    double d = epd_gpu_stag_ms[0] - m0;
                    if (epd_gpu_stag_ms[2] == m2) { epd_gpu_finish(g, 2000); d = (now() - w0) * 1e3; ftimer = 0; }
                    fsum += d;
                    if (d > fmax) fmax = d;
                }
            }
            if (bpm) {
                printf("BP %-24s %-9s shape %dx%d drive %.2f ms avg, %.2f max; pack %.2f ms per upload (%d); frame %.2f ms avg, %.2f max (48 frames, %s)\n",
                       tc[k].n, bp_lname(&p), bpw, bph, bdsum / 48, bdmax, bnup ? bpsum / bnup : 0.0, bnup, fsum / 48, fmax, ftimer ? "GPU timer" : "wall clock");
                GLenum e = bpgl.glGetError();
                if (e != GL_NO_ERROR) FAIL("BP %s: GL error 0x%x", tc[k].n, e);
                continue;
            }
            if (p2) {
                const char *dv = getenv("GLASSINE_STAG_DEFS");
                char cl[24] = "";
                if (p.cellw) snprintf(cl, sizeof cl, " cell %dx%d%s", p.cellw, p.cellh, epd_gpu_stag_cremap ? "" : " inplace");
                printf("P2 %-24s stripe %-2d%s %-7s pass %.2f ms avg, %.2f max (48 frames, %s)\n", tc[k].n, p.stripe, p.cellw ? cl : p.blocks ? " blk" : "",
                       dv && strstr(dv, "HEAPSTATE") ? "heapst" : dv && strstr(dv, "COLDLIN") ? "coldlin" : "planes", fsum / 48, fmax, ftimer ? "GPU timer" : "wall clock");
                continue;
            }
            printf("GPU time %-27s pass %.2f ms, template pass %.2f ms a frame\n", tc[k].n,
                   epd_gpu_stag_ms[0] / epd_gpu_stag_ms[2], epd_gpu_stag_ms[1] / epd_gpu_stag_ms[2]);
        }
        epd_gpu_stag_timer = 0;
        if (bpG) bp_gpu_close(&bpt);
        if (bpon) bp_free(&bps);
    }
    if (LN) ln_settle_gpu(g, R);                /* (lanes: the timed frames' lanes evicted, taken over) */
    hand_back(g, "after the timed frames");    /* (their tile skip leaves AT bound; the scenarios load too often for it) */
    if (LN) {                                   /* lanes: a run's start and end, a full panel armed */
        epd_stag_par p = { .G = 2, .n = 1, .dither = 1, .R = R, .stripe = 64, .lanes = 1 };
        memcpy(TG, pg, EPD_NPX);
        double t0 = now();
        ln_resolve(ALL);
        double t1 = now();
        tsy(SY_B);
        epd_stag_lane_arm(&E, ALL, TG, NULL);
        tsy(SY_E);
        epd_gpu_stag_target(g, &S, ALL);
        memset(LD, 1, EPD_NPX);
        double t2 = now();
        for (int f = 0; f < 4; f++) epd_gpu_stag_frame(g, &p, S.f++, 0);
        epd_gpu_finish(g, 1000);
        double t3 = now();
        epd_rect pr = ln_evict(g, ALL);
        double t4 = now();
        long u = 1;
        int nf = 0;
        for (; nf < 40 && u; nf++) {
            epd_gpu_stag_frame(g, &p, S.f++, 0);
            epd_gpu_finish(g, 1000);
            epd_gpu_stag_acks(g, S2.ack, 0, EPD_H);
            u = epd_stag_lane_unacked(&E, S2.ack, pr);
        }
        double t5 = now();
        if (u) FAIL("lanes: transitions: %ld words not acked", u);
        tsy(SY_B);
        epd_stag_lane_takeover(&E, pr, LD);
        tsy(SY_E);
        double t6 = now();
        printf("transitions (lanes), a full panel: resolve %.1f ms, arm (T) %.1f, 4 frames %.1f, evict %.1f, %d frames to the acks %.1f, takeover %.1f\n",
               (t1 - t0) * 1e3, (t2 - t1) * 1e3, (t3 - t2) * 1e3, (t4 - t3) * 1e3, nf, (t5 - t4) * 1e3, (t6 - t5) * 1e3);
    } else {                                    /* a run's start and end, a full panel loaded */
        epd_stag_par p = { .G = 2, .n = 1, .dither = 1, .R = R };
        epd_rect all = { 0, 0, EPD_W, EPD_H };
        double t0 = now();
        epd_stag_enter(&S);
        double t1 = now();
        epd_stag_load(&S, &E, &p, all);
        double t2 = now();
        epd_gpu_stag_enter(g, &S);
        epd_gpu_finish(g, 1000);
        double t3 = now();
        epd_gpu_stag_load(g, &S, &p, all);
        epd_gpu_finish(g, 1000);
        double t4 = now();
        epd_gpu_stag_read(g, &S2, all, 0);
        double t5 = now();
        epd_stag_exit(&S2, &E);
        double t6 = now();
        printf("transitions, a full panel: park %.1f ms, load (CPU) %.1f, GPU enter %.1f, GPU load %.1f, read %.1f, exit (CPU) %.1f\n",
               (t1 - t0) * 1e3, (t2 - t1) * 1e3, (t3 - t2) * 1e3, (t4 - t3) * 1e3, (t5 - t4) * 1e3, (t6 - t5) * 1e3);
    }
    if (getenv("STAG_PROF")) {                  /* per pass, a glFinish after each */
        epd_stag_par p = { .G = 4, .n = 3, .dither = 1, .R = R };
        epd_gpu_prof_on = 1;
        memset(epd_gpu_prof, 0, sizeof epd_gpu_prof);
        for (int f = 0; f < 30; f++) epd_gpu_stag_frame(g, &p, S.f++, 0);
        epd_gpu_prof_on = 0;
        printf("profile G 4: compute %.2f ms, template pass %.2f ms a frame\n",
               epd_gpu_prof[EPD_GPU_PROF_CLAIM] / 30 * 1e3, epd_gpu_prof[EPD_GPU_PROF_FIN] / 30 * 1e3);
    }
    free(pg);
    free(TG); free(LD); free(MK); free(PU);
    epd_stag_free(&S2);
    epd_gpu_close(g);
}
#endif

/* payrule 1 (epd-stagger.h): a pixel at a rail going to the other one drives
 * min(max(need, owe), need + cap) in all, owe = the record against that
 * drive; nothing at an idle rail. Returns the drive frames, the end state in
 * *end (the pixel at px, py of a 4x4 block) */
static int pr_R = 9;
static int pr_run(int ctl, int G, int cap, int l, int op, int dc, int t, uint32_t *end)
{
    const int R = pr_R, px = 403, py = 403;
    epd_stag_par p = { .G = G, .n = G - 1, .ctl = ctl, .K = 8, .R = R, .payrule = 1, .pcap = cap };
    epd_stag_enter(&S);
    epd_rect r = { 400, 400, 4, 4 };
    uint8_t lv[16];
    memset(lv, t, sizeof lv);
    for (int y = r.y; y < r.y + r.h; y++)
        for (int x = r.x; x < r.x + r.w; x++) {
            size_t q = (size_t)y * EPD_W + x;
            S.cold[q] = epd_stag_cold(l, op, 0, dc);
            S.hot[q] = (uint16_t)epd_stag_hotbits(l, op, dc, &p);
            S.ld[q] = 1;
        }
    epd_stag_target(&S, r, lv);
    int drive = 0, wrong = 0;
    for (int k = 0; k < 120; k++) {
        frame(&p);
        int ph = phase_at(W, px, py);
        drive += ph != 0;
        wrong += ph && ph != (t ? 2 : 1);
    }
    if (wrong) FAIL("payrule: %d frames away from the target", wrong);
    *end = S.cold[(size_t)py * EPD_W + px];
    return drive;
}

static void payrule(const uint8_t *wbf, size_t wn)
{
    const int R = 9;
    static const struct { int l, op, dc, t, cap, want; const char *what; } c[] = {
        { 30, 9, 5, 0, 3, 9, "white -> black, the record below the need" },
        { 30, 9, 11, 0, 3, 11, "white -> black, the record 2 past the need" },
        { 30, 9, 12, 0, 3, 12, "white -> black, the record at need + cap" },
        { 30, 9, 25, 0, 3, 12, "white -> black, the record far past: need + cap" },
        { 30, 9, -14, 0, 3, 9, "white -> black, a same-way record: the need only" },
        { 0, 0, -13, 30, 3, 12, "black -> white, the record 4 past, cap 3" },
        { 0, 0, -13, 30, 7, 13, "black -> white, cap 7" },
        { 0, 0, -20, 30, 7, 16, "black -> white, cap 7, far past" },
        { 0, 0, -20, 30, 1, 10, "black -> white, cap 1" },
        { 30, 5, 8, 0, 3, 8, "white (model short of the rail) -> black" },
        { 30, 9, -20, 30, 3, 0, "an idle white rail with a record against it: nothing" },
        { 0, 0, 20, 0, 3, 0, "an idle black rail with a record against it: nothing" },
        { 16, 5, 20, 0, 3, 5, "a grey (loaded) -> black: the need only" },
    };
    int ok = 0, n = 0;
    for (int ctl = 1; ctl <= 3; ctl++)
        for (int G = 2; G <= 16; G *= 2)
            for (unsigned k = 0; k < sizeof c / sizeof *c; k++) {
                uint32_t e;
                int d = pr_run(ctl, G, c[k].cap, c[k].l, c[k].op, c[k].dc, c[k].t, &e), sd = c[k].t ? 1 : -1;
                int e_l = EPD_STAG_LEVEL(e), e_dc = EPD_STAG_DC(e), e_op = EPD_STAG_OP(e);
                n++;
                if (d != c[k].want || e_l != c[k].t || e_dc != c[k].dc + sd * d || e_op != (c[k].t ? R : 0) ||
                    (d && !EPD_STAG_STALE(e)))
                    FAIL("payrule ctl %d G %d: %s: drove %d (want %d), end level %d dc %d op %d stale %d", ctl, G, c[k].what, d,
                         c[k].want, e_l, e_dc, e_op, EPD_STAG_STALE(e));
                else ok++;
            }
    printf("payrule: %d of %d cases (ctl 1-3, G 2/4/8/16) drive min(max(need, owe), need + cap), nothing at an idle rail\n", ok, n);
    /* a reversal halfway: the budget is the new arrival's; the drive away is
     * what it drove (the first leg ends where the target flips) */
    {
        epd_stag_par p = { .G = 2, .n = 1, .ctl = 1, .R = R, .payrule = 1, .pcap = 3 };
        epd_stag_enter(&S);
        epd_rect r = { 400, 400, 4, 4 };
        size_t o = (size_t)403 * EPD_W + 403;
        uint8_t lv[16];
        for (int y = r.y; y < r.y + r.h; y++)
            for (int x = r.x; x < r.x + r.w; x++) {
                size_t q = (size_t)y * EPD_W + x;
                S.cold[q] = epd_stag_cold(30, 9, 0, 20);
                S.hot[q] = (uint16_t)epd_stag_hotbits(30, 9, 20, &p);
                S.ld[q] = 1;
            }
        memset(lv, 0, sizeof lv);
        epd_stag_target(&S, r, lv);
        int d1 = 0, d2 = 0;
        for (int k = 0; k < 20; k++) { frame(&p); d1 += phase_at(W, 403, 403) == 1; }     /* 10 pulses: need 9 + 1 of b 3 */
        int odd = EPD_STAG_LEVEL(S.cold[o]) & 1, dcm = EPD_STAG_DC(S.cold[o]);
        memset(lv, 30, sizeof lv);
        epd_stag_target(&S, r, lv);
        for (int k = 0; k < 60; k++) { frame(&p); d2 += phase_at(W, 403, 403) == 2; }
        /* the way back: need 9, owe = -dc = -(20 - 10) < 0: the need only */
        if (d1 != 10 || !odd || dcm != 10 || d2 != 9 || EPD_STAG_LEVEL(S.cold[o]) != 30 || EPD_STAG_DC(S.cold[o]) != 19)
            FAIL("payrule: reversal: drove %d (10), odd %d, dc %d (10); back %d (9), level %d dc %d (19)", d1, odd, dcm, d2,
                 EPD_STAG_LEVEL(S.cold[o]), EPD_STAG_DC(S.cold[o]));
        else printf("payrule: a reversal with budget left: the way back pays only its own arrival's (need 9, a same-way record)\n");
        /* the hand-back: an odd level (a budget left) goes back as its rail */
        S.cold[o] = epd_stag_cold(epd_stag_lenc(0, 2), 0, 1, 5);
        size_t o2 = o - 1;
        S.cold[o2] = epd_stag_cold(epd_stag_lenc(30, 3), 9, 1, -5);
        for (size_t q = (size_t)400 * EPD_W + 400; q < (size_t)404 * EPD_W; q += EPD_W)
            for (size_t x = 0; x < 4; x++) if (q + x != o && q + x != o2) S.cold[q + x] = epd_stag_cold(30, 9, 1, 0);
        epd_stag_exit(&S, &E);
        if (epd_px_level(&E, o) != 0 || epd_px_target(&E, o) != 0 || epd_px_level(&E, o2) != 30 || epd_px_dc(&E, o) != 5 || epd_px_dc(&E, o2) != -5)
            FAIL("payrule: exit: levels %d %d (0 30), dc %d %d", epd_px_level(&E, o), epd_px_level(&E, o2), epd_px_dc(&E, o), epd_px_dc(&E, o2));
        else printf("payrule: exit hands back the rail (an unpaid budget dropped)\n");
        for (int y = 400; y < 404; y++)
            for (int x = 400; x < 404; x++) {
                size_t q = (size_t)y * EPD_W + x;
                epd_px_put(&E, q, EPD_F_LEVEL | EPD_F_TARGET | EPD_F_DC, 30, 30, 0, 0, 0, 0);
            }
        int b, m = 1;
        for (int rail = 0; rail <= 30; rail += 30)
            for (int bb = 0; bb <= EPD_STAG_PCAP; bb++) {
                int l = epd_stag_lenc(rail, bb);
                m &= epd_stag_ldec(l, &b) == rail && b == bb && l >= 0 && l <= 31 && (bb == 0) == !(l & 1);
            }
        if (!m) FAIL("payrule: level encoding does not round-trip");
    }
    /* payrule off: ctl 1-3 with pay as before (the record past the need at an
     * idle rail still pays) -- the default untouched */
    {
        epd_stag_par p = { .G = 2, .n = 1, .ctl = 1, .pay = 2, .R = R };
        int dd = 0;
        epd_stag_enter(&S);
        size_t o = (size_t)403 * EPD_W + 403;
        S.cold[o] = epd_stag_cold(0, 0, 1, 5);
        S.hot[o] = (uint16_t)epd_stag_hotbits(0, 0, 5, &p);
        S.tgt[o] = 0; S.ld[o] = 1;
        for (int k = 0; k < 30; k++) { frame(&p); dd += phase_at(W, 403, 403) == 1; }
        if (dd != 4) FAIL("payrule off: the idle-rail payback drove %d (4: down to the record 1 < pay 2)", dd);
        p.payrule = 1; p.pcap = 3;
        epd_stag_enter(&S);
        S.cold[o] = epd_stag_cold(0, 0, 1, 5);
        S.hot[o] = (uint16_t)epd_stag_hotbits(0, 0, 5, &p);
        S.tgt[o] = 0; S.ld[o] = 1;
        dd = 0;
        for (int k = 0; k < 30; k++) { frame(&p); dd += phase_at(W, 403, 403) != 0; }
        if (dd) FAIL("payrule: the same idle rail (pay 2 set) drove %d (0)", dd);
    }
    /* the engine's matched cut (model 2, cap 3) for the same pixel: a cut A2
     * step whose net drive is the pixel's need (the floor = the need), the
     * record from below the need to past need + cap; the engine's drive =
     * its record's move */
    {
        static epd_engine X;
        if (epd_init(&X, wbf, wn, 24)) { FAIL("payrule: engine init"); return; }
        epd_set_model(&X, 2, 1);
        epd_set_matched_rule(&X, 2, 3);
        epd_rect all = { 0, 0, EPD_W, EPD_H }, px = { 600, 600, 8, 8 };
        static uint8_t scr[EPD_W * EPD_H];
        memset(scr, 255, sizeof scr);
        epd_submit(&X, all, EPD_GC16, 1, scr, EPD_W);
        while (epd_schedule(&X)) epd_advance(&X, NULL, NULL);
        const int XR = X.op_r;
        size_t o = (size_t)603 * EPD_W + 603;
        memset(scr, 0, sizeof scr);
        /* the net of each cut length L (matched off): to find the steps */
        int net[64] = { 0 }, nl = 0;
        epd_set_matched(&X, 0);
        for (int L = 2; L < 64; L++) {
            for (int y = 600; y < 608; y++)
                for (int x = 600; x < 608; x++) { size_t q = (size_t)y * EPD_W + x; epd_px_put(&X, q, EPD_F_LEVEL | EPD_F_TARGET | EPD_F_DC | EPD_F_OP, 30, 30, 0, XR, 0, 0); }
            epd_submit(&X, px, EPD_MODE_A2LEN(EPD_A2, L), 0, scr, EPD_W);
            while (epd_schedule(&X)) epd_advance(&X, NULL, NULL);
            net[L] = -epd_px_dc(&X, o);
            if (net[L] >= XR) { nl = L; break; }
        }
        epd_set_matched(&X, 1);
        pr_R = XR;
        int same = 0, cmp = 0, diff = 0;
        char why[512] = "";
        for (int need = 3; need <= XR - 3; need++) {
            int L = 0;
            for (int k = 2; k <= nl; k++) if (net[k] == need) { L = k; break; }
            if (!L) continue;
            for (int dc = need - 2; dc <= need + 5; dc++) {
                for (int y = 600; y < 608; y++)
                    for (int x = 600; x < 608; x++) { size_t q = (size_t)y * EPD_W + x; epd_px_put(&X, q, EPD_F_LEVEL | EPD_F_TARGET | EPD_F_DC | EPD_F_OP, 30, 30, 0, need, dc, 0); }
                epd_submit(&X, px, EPD_MODE_A2LEN(EPD_A2, L), 0, scr, EPD_W);
                while (epd_schedule(&X)) epd_advance(&X, NULL, NULL);
                int eng = dc - epd_px_dc(&X, o);
                uint32_t e;
                int st = pr_run(1, 2, 3, 30, need, dc, 0, &e);
                (void)R;
                cmp++;
                if (eng == st) same++;
                else {
                    diff++;
                    if (strlen(why) < 400)
                        snprintf(why + strlen(why), sizeof why - strlen(why), " need %d dc %d: engine %d stagger %d;", need, dc, eng, st);
                }
            }
        }
        pr_R = 9;
        printf("payrule vs the engine's matched cut (R %d, cap 3): %d of %d equal%s%s\n", XR, same, cmp, diff ? "; differ:" : "", why);
        if (!cmp) FAIL("payrule: no engine step matched a need");
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s <eink_waveform.wbf> [frames]\n", argv[0]); return 2; }
    setvbuf(stdout, NULL, _IOLBF, 0);   /* (in order with stderr's census lines, GLASSINE_STAG_COUNT) */
    setenv("GLASSINE_BP4_TIMER", "1", 0);    /* (a bench: ctl 4's pass timers on; the daemon keeps them off, epd-gpu.c) */
    epd_stag_dither_set(getenv("STAG_DPAT") && !strcmp(getenv("STAG_DPAT"), "bayer"));   /* (the dither pattern: blue, STAG_DPAT=bayer) */
    size_t n;
    uint8_t *wbf = slurp(argv[1], &n);
    if (epd_init(&E, wbf, n, 24)) { fprintf(stderr, "bad waveform\n"); return 2; }
    epd_set_model(&E, 0, 1);                    /* the model kept (op_r) */
    epd_assume(&E, 30);
    epd_submit(&E, (epd_rect){ 0, 0, 8, 8 }, EPD_GC16, 1, (uint8_t[64]){ 0 }, 8);   /* (op_prep: op_r) */
    while (epd_schedule(&E)) epd_advance(&E, NULL, NULL);
    printf("model range R %d\n", E.op_r);
    if (epd_stag_init(&S)) return 2;
    memcpy(W, E.tpl, sizeof W);
#ifdef EPD_GPU
    if (getenv("STAG_BP4_LANES")) return bp4_lane_gate();   /* (the twin's lanes: armed everywhere == plain, a popup evict, re-arm)*/
    if (getenv("STAG_BP4_GATE")) return bp4_gate();     /* (the two-tier ctl 3 on planes == the tier model, + the host time vs bp3)*/
    if (getenv("STAG_BP4I_GATE")) return bp4i_gate();   /* (bp4 with the planes interleaved == bp4, lockstep; + the time, 1/2/4/8 threads)*/
    if (getenv("STAG_TIER_SIM")) return tier_sim();     /* (ctl 3 in two tiers + op bits, scored against today's: host)*/
    if (getenv("STAG_BP3_GATE")) return bp3_gate();     /* (ctl 3 on bit planes, the CPU gate alone) */
#endif
    if (!getenv("STAG_TIME_ONLY") && !getenv("STAG_GPU_ONLY")) {   /* (STAG_GPU_ONLY: part 2 only, device gates; the host runs part 1) */
        one_step();
        dither();
        controller();
        controller2();
        controller3();
        payrule(wbf, n);
    }
#ifdef EPD_GPU
    if (getenv("STAG_COLD_DIAG")) { cold_diag(); return 0; }
    if (getenv("STAG_QUALITY")) {               /* (a measure) */
        quality(2, 3, 6, 240, 60, 0);
        quality(1, 3, 6, 240, 60, 0);           /* A2/1 */
        quality(2, 3, 6, 240, 60, 42);
        quality(1, 3, 6, 240, 60, 42);
        quality(2, 3, 6, 60, 120, 42);
        quality(4, 3, 12, 240, 60, 42);
    }
    gpu_part(argc > 2 ? atoi(argv[2]) : 120);
#endif
    printf("%s\n", bad ? "FAILED" : "all ok");
    return bad ? 1 : 0;
}
