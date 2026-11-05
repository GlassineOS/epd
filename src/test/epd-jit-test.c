/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-jit-test: the stream thread's JIT render lead (epd-jit.h) in a model of
 * its loop (epd-drm.c thread): frame k+1 renders from exp_evt - lead -
 * JIT_MARGIN (or at once), must finish by frame k's flip event (+0.7 ms, the
 * stats' "late") or slips a whole period. jit_size=off: one decaying max, the
 * start fixed when the wait begins (clock_nanosleep). jit_size=on: per size,
 * predicted from the next frame's work (runs still going + updates pending),
 * re-predicted at every submit during the wait.
 * Render cost = base + full * rows / 1404 (+ an outlier); pen pieces 40 rows,
 * 3 frames; a scroll frame the whole panel. Counts missed flips (late) and pen
 * latency (arrival -> the flip event that shows it first), asserts:
 * A one slow big frame: the pen pieces' lead is back at the small value
 * within 3 frames (old: still high ~35 frames on), no more late frames
 * B a sustained slowdown of the pen frames: the lead follows it at once,
 * same result as the old lead (one size: the same rule)
 * C small -> big transitions (scroll bursts in a pen stream, arriving at any
 * phase): no more late frames than the old lead */
#include <stdio.h>
#include <stdlib.h>
#include "../epd-jit.h"

#define P (1.0 / 85)
#define MARGIN 1.5e-3
#define LATE 0.7e-3
#define ROWS 1404
#define NF 1200
#define MAXP 4096

typedef struct { double at; int y, rows, nf, pen, slow; } piece;
typedef struct { int y, rows, left; } run;

static piece pc[MAXP];
static int npc;
static unsigned rng = 12345;
static double rnd(void) { rng = rng * 1103515245u + 12345u; return ((rng >> 8) & 0xffff) / 65536.0; }

static void add(double at, int y, int rows, int nf, int pen, int slow)
{
    if (npc < MAXP) pc[npc++] = (piece){ at, y, rows, nf, pen, slow };
}
static int cmp(const void *a, const void *b)
{
    double d = ((const piece *)a)->at - ((const piece *)b)->at;
    return d < 0 ? -1 : d > 0;
}

/* scenario cost knobs */
static double c_base = 0.6e-3, c_full = 9e-3, c_slow = 10e-3;
static int slow_from = -1, slow_to = -1;       /* B: frames whose pen cost is x5 */

static int span(run *r, int n, piece **pd, int npd)
{
    int y0 = ROWS, y1 = 0;
    for (int i = 0; i < n; i++) { if (r[i].y < y0) y0 = r[i].y; if (r[i].y + r[i].rows > y1) y1 = r[i].y + r[i].rows; }
    for (int i = 0; i < npd; i++) { if (pd[i]->y < y0) y0 = pd[i]->y; if (pd[i]->y + pd[i]->rows > y1) y1 = pd[i]->y + pd[i]->rows; }
    return y1 > y0 ? y1 - y0 : 0;
}

typedef struct {
    int late, frames, npen;
    double pen_lat;
    double lead[NF];        /* the lead each frame used (s); -1: no small frame */
    int cls[NF];            /* its predicted size at the wait's start */
    int rcls[NF];           /* the size it rendered */
    double pen_lat_win; int npen_win;    /* pen pieces arriving in the window */
} result;

static void sim(int size, result *res, double win0, double win1)
{
    epd_jit j;
    epd_jit_init(&j);
    run rn[64];
    int nr = 0, ip = 0;
    piece *pd[256];
    int npd = 0;
    double t = 0.5e-3, exp_evt = P;
    *res = (result){0};
    for (int k = 0; k < NF; k++) res->lead[k] = -1;
    for (int k = 0; k < NF && ip < npc; k++) {
        int idle = 0;
        while (ip < npc && pc[ip].at <= t) pd[npd++] = &pc[ip++];
        if (!nr && !npd && ip >= npc) break;
        if (!nr && !npd) {                         /* nothing to do: wait for a submit */
            t = pc[ip].at;
            pd[npd++] = &pc[ip++];
            if (t > exp_evt) idle = 1;             /* (the last flip's event passed: not "late") */
        }
        int c = epd_jit_class(span(rn, nr, pd, npd), 0, 0);
        double lead = epd_jit_lead(&j, c, size), start = exp_evt - lead - MARGIN;
        res->lead[k] = lead; res->cls[k] = c;
        if (size)       /* woken by each submit during the wait: predict again */
            while (ip < npc && pc[ip].at < start) {
                t = pc[ip].at > t ? pc[ip].at : t;
                pd[npd++] = &pc[ip++];
                c = epd_jit_class(span(rn, nr, pd, npd), 0, 0);
                lead = epd_jit_lead(&j, c, size);
                start = exp_evt - lead - MARGIN;
            }
        double r0 = start > t ? start : t;
        while (ip < npc && pc[ip].at <= r0) pd[npd++] = &pc[ip++];
        /* epd_schedule: the pending pieces become runs */
        int slow = 0, pen_slow = k >= slow_from && k < slow_to;
        for (int i = 0; i < npd && nr < 64; i++) { rn[nr++] = (run){ pd[i]->y, pd[i]->rows, pd[i]->nf }; slow |= pd[i]->slow; }
        int rows = span(rn, nr, NULL, 0), jc = epd_jit_class(rows, 0, 0);
        double rt = (c_base + c_full * rows / ROWS) * (pen_slow ? 5 : 1) + (slow ? c_slow : 0);
        double fin = r0 + rt, ev;
        epd_jit_add(&j, jc, rt);
        res->rcls[k] = jc;
        if (idle) {
            ev = P * (int)((fin + 0.5e-3) / P + 1);
        } else if (fin > exp_evt + LATE) {
            res->late++;
            ev = P * (int)((fin + 0.5e-3) / P + 1);
        } else
            ev = exp_evt + P;
        for (int i = 0; i < npd; i++)
            if (pd[i]->pen) {
                double l = ev - pd[i]->at;
                res->pen_lat += l; res->npen++;
                if (pd[i]->at >= win0 && pd[i]->at < win1) { res->pen_lat_win += l; res->npen_win++; }
            }
        npd = 0;
        for (int i = 0; i < nr;) if (--rn[i].left <= 0) rn[i] = rn[--nr]; else i++;
        t = (fin > exp_evt ? fin : exp_evt) + 50e-6;
        exp_evt = ev;
        res->frames++;
    }
}

static void pen_stream(double t0, double t1)
{
    int y = 600;
    for (double t = t0; t < t1; t += P * (0.8 + 0.4 * rnd())) {
        y += (int)(rnd() * 21) - 10;
        if (y < 0) y = 0;
        if (y > ROWS - 40) y = ROWS - 40;
        add(t, y, 40, 3, 1, 0);
    }
}

static double avg_lead(const result *r, int k0, int k1, int cls)
{
    double s = 0; int n = 0;
    for (int k = k0; k < k1 && k < r->frames; k++) if (r->lead[k] >= 0 && r->cls[k] == cls) { s += r->lead[k]; n++; }
    return n ? s / n : -1;
}

static int fails;
#define CHECK(c, ...) do { if (!(c)) { printf("FAIL: " __VA_ARGS__); printf("\n"); fails++; } } while (0)

int main(void)
{
    result o, n;
    static result o2, n2;

    /* A: steady pen pieces, one slow whole-panel frame at ~frame 200 */
    npc = 0; rng = 12345;
    pen_stream(0, 6.0);
    double tb = 200 * P + 3e-3;
    add(tb, 0, ROWS, 1, 0, 1);
    qsort(pc, npc, sizeof *pc, cmp);
    sim(0, &o, tb + 3 * P, tb + 40 * P);
    sim(1, &n, tb + 3 * P, tb + 40 * P);
    int kb = -1;            /* the first frame after the big one whose work is small again */
    for (int k = 190; k < n.frames && k < 260; k++) if (n.rcls[k] == 2) kb = k;
    double steady = avg_lead(&n, 100, 190, 0);
    printf("A outlier: steady small lead %.2f ms; after the slow frame (frames %d..+35): old %.2f new %.2f ms; "
           "3 frames on: old %.2f new %.2f ms; 35 on: old %.2f new %.2f ms\n",
           steady * 1e3, kb + 1, avg_lead(&o, kb + 1, kb + 36, 0) * 1e3, avg_lead(&n, kb + 1, kb + 36, 0) * 1e3,
           o.lead[kb + 3] * 1e3, n.lead[kb + 3] * 1e3, o.lead[kb + 35] * 1e3, n.lead[kb + 35] * 1e3);
    printf("A outlier: late old %d new %d of %d frames; pen latency in the 37 frames after: old %.2f new %.2f ms (all: old %.2f new %.2f)\n",
           o.late, n.late, n.frames, o.pen_lat_win / o.npen_win * 1e3, n.pen_lat_win / n.npen_win * 1e3,
           o.pen_lat / o.npen * 1e3, n.pen_lat / n.npen * 1e3);
    CHECK(kb > 0, "A: no big frame seen");
    CHECK(n.lead[kb + 3] <= steady + 0.5e-3, "A: small lead 3 frames after the outlier %.2f ms > steady %.2f + 0.5", n.lead[kb + 3] * 1e3, steady * 1e3);
    CHECK(o.lead[kb + 3] > steady + 5e-3, "A: the old lead did not stay high (%.2f ms): the scenario misses the bug", o.lead[kb + 3] * 1e3);
    CHECK(n.late <= o.late, "A: more late frames (%d > %d)", n.late, o.late);
    CHECK(n.pen_lat_win < o.pen_lat_win / o.npen_win * n.npen_win, "A: pen latency after the outlier not lower");

    /* B: a sustained slowdown of the pen frames (x5, frames 100..200) */
    npc = 0; rng = 777;
    pen_stream(0, 4.0);
    qsort(pc, npc, sizeof *pc, cmp);
    slow_from = 100; slow_to = 200;
    sim(0, &o, 0, 0);
    sim(1, &n, 0, 0);
    slow_from = slow_to = -1;
    double rt_slow = 5 * (c_base + c_full * 40.0 / ROWS);
    printf("B sustained: lead in the slowdown (frames 101..199) old %.2f new %.2f ms (a pen frame takes >= %.2f); late old %d new %d of %d\n",
           avg_lead(&o, 101, 199, 0) * 1e3, avg_lead(&n, 101, 199, 0) * 1e3, rt_slow * 1e3, o.late, n.late, n.frames);
    for (int k = 101; k < 199; k++)
        if (n.cls[k] == 0 && n.lead[k] < rt_slow * 0.95) { CHECK(0, "B: frame %d lead %.2f ms below the slow pen frame %.2f", k, n.lead[k] * 1e3, rt_slow * 1e3); break; }
    CHECK(n.late <= o.late, "B: more late frames (%d > %d)", n.late, o.late);

    /* C: scroll bursts (3 whole-panel frames, one a period, any phase) every
     * 30 frames in a pen stream; and C2 the same with 8 ms scroll frames */
    for (int v = 0; v < 2; v++) {
        npc = 0; rng = 4242 + v;
        pen_stream(0, 12.0);
        for (double t = 30 * P; t < 12.0; t += 30 * P) {
            double ph = rnd() * P;
            for (int i = 0; i < 3; i++) add(t + ph + i * P, 0, ROWS, 2, 0, 0);
        }
        qsort(pc, npc, sizeof *pc, cmp);
        if (v) c_full = 8e-3; else c_full = 9e-3;
        result *ro = v ? &o2 : &o, *rn_ = v ? &n2 : &n;
        sim(0, ro, 0, 0);
        sim(1, rn_, 0, 0);
        printf("C%s transitions (full frame %.1f ms): late old %d new %d of %d frames; pen latency old %.2f new %.2f ms; small lead avg old %.2f new %.2f ms\n",
               v ? "2" : "", (c_base + c_full) * 1e3, ro->late, rn_->late, rn_->frames, ro->pen_lat / ro->npen * 1e3, rn_->pen_lat / rn_->npen * 1e3,
               avg_lead(ro, 0, NF, 0) * 1e3, avg_lead(rn_, 0, NF, 0) * 1e3);
        CHECK(rn_->late <= ro->late, "C%s: more late frames (%d > %d)", v ? "2" : "", rn_->late, ro->late);
    }
    c_full = 9e-3;
    printf(fails ? "epd-jit: %d FAILED\n" : "epd-jit: ok\n", fails);
    return fails != 0;
}
