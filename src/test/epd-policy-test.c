/* epd-policy host tests:
 * 1. parity: the pre-refactor glassine-epdd-desktop decision code (transplanted below with
 * its X/ioctl calls stubbed into an action log) vs epd-policy, on random
 * traces — same actions, same commit modes, same timeouts
 * 2. scenarios: properties the panel must have (no flash outside the window a
 * scroll happens in, page turns flash once, typing DUs then cleans up)
 * build+run: make -C .. test (or: cc -O2 -Wall -I.. ../epd-policy.c epd-policy-test.c) */
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../epd-policy.h"

#define W 1404
#define H 1872
static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void fmt_act(const action *a, char *s, size_t n)
{
    static const char *k[] = { "send", "grab", "graball", "cut", "banner" };
    snprintf(s, n, "%s %d,%d-%d,%d wf%d m%d n%d", k[a->kind], a->b.x0, a->b.y0, a->b.x1, a->b.y1, a->wf, a->mode, a->n);
}

/* ---- the old code (glassine-epdd-desktop.c's main loop before the policy split), I/O stubbed ------------
 * (+ deliberate behaviour changes since, so it stays a baseline) */
static actions *R;                                     /* where the stubs record */
static int burst_auto = 1, clean_full = 0, paused = 0;
static double du_max = 0.03, flash_min = 0.35;
static int cut_auto = 1, cut_fixed = 0, cut_cur = -1, dither_runs = 1;
static int64_t last_du = -1000, NOW;
static int in_run;
static uint8_t ghost[GTW * GTH];
static boxlist dirty, inked;
static int64_t last_ink;
static int run_bursts;
static box run_box, run_win;
static int64_t last_dmg, last_gc16;
static box gc16_box;                    /* (the policy's gc16_box/_t) */
static int64_t gc16_box_t;
static int want_full = 1;

static void rec(action a) { if (R->n < MAXACT) R->a[R->n++] = a; }
static void ghost_note(box b, int du)
{
    if (box_empty(b)) return;
    for (int ty = b.y0 / GTILE; ty <= (b.y1 - 1) / GTILE && ty < GTH; ty++)
        for (int tx = b.x0 / GTILE; tx <= (b.x1 - 1) / GTILE && tx < GTW; tx++) {
            uint8_t *g = &ghost[ty * GTW + tx];
            if (!du) *g = 0;
            else if (*g < 255) (*g)++;
        }
}
static void send(box b, int wf, int mode)
{
    if (wf == WF_DU) last_du = NOW;
    if (wf == WF_GC16) { gc16_box = NOW - gc16_box_t >= GC16_BUSY_MS ? b : box_union(gc16_box, b); gc16_box_t = NOW; }
    if (mode == M_SCREEN) memset(ghost, 0, sizeof ghost);
    else if (wf == WF_DU) ghost_note(b, 1);
    else if (mode == M_AREA) ghost_note(b, 0);
    rec((action){ A_SEND, b, wf, mode, 0 });
}
static void set_cut(int n)
{
    if (n == cut_cur || NOW - last_du < 400) return;
    cut_cur = n;
    rec((action){ A_CUT, {0}, 0, 0, n });
}
static void grab(box b, int all) { rec((action){ all ? A_GRAB_ALL : A_GRAB, b, 0, 0, 0 }); }
static void apply_cmd(const char *cmd)
{
    const char *v;
    if (strstr(cmd, "cut=auto")) cut_auto = 1;
    else if ((v = strstr(cmd, "cut="))) { cut_auto = 0; cut_fixed = atoi(v + 4); }
    if (strstr(cmd, "dither=on")) dither_runs = 1;
    if (strstr(cmd, "dither=off")) dither_runs = 0;
    if (strstr(cmd, "burst=du")) burst_auto = 0;
    if (strstr(cmd, "burst=auto")) burst_auto = 1;
    if (strstr(cmd, "clean=partial")) clean_full = 0;
    if (strstr(cmd, "clean=full")) clean_full = 1;
    if ((v = strstr(cmd, "du_max="))) du_max = atof(v + 7);
    if ((v = strstr(cmd, "flash_min="))) flash_min = atof(v + 10);
}
/* cmd contains w as a whole word ("full", not "clean=full" / "flash_full=") */
static int cmd_word(const char *cmd, const char *w)
{
    size_t n = strlen(w);
    for (const char *c = cmd; (c = strstr(c, w)); c += n)
        if ((c == cmd || c[-1] == ' ' || c[-1] == '\n') && (!c[n] || c[n] == ' ' || c[n] == '\n')) return 1;
    return 0;
}

static int ref_cmd(const char *cmd)
{
    int wake = 0;
    if (cmd_word(cmd, "full")) want_full = 1;
    if (strstr(cmd, "pause")) paused = 1;
    if (strstr(cmd, "resume")) { paused = 0; want_full = 1; }
    if (strstr(cmd, "sleep")) {
        box bar = { 0, H - BAR_H, W, H }, all = { 0, 0, W, H };
        rec((action){ A_GRAB_ALL, all, 0, 0, 0 });
        rec((action){ A_BANNER, bar, 0, 0, 0 });
        send(all, WF_GC16, M_SCREEN);
        last_gc16 = NOW;
        paused = 2;
    }
    if (strstr(cmd, "wake") && paused == 2) { paused = 0; wake = 1; }
    apply_cmd(cmd);
    if (strstr(cmd, "gamma=")) want_full = 1;
    if (strstr(cmd, "cut=")) set_cut(cut_auto ? 0 : cut_fixed);
    return wake;
}
static void ref_wake(box changed)
{
    box bar = { 0, H - BAR_H, W, H };
    send(box_union(bar, changed), WF_GC16, M_PART);
    last_gc16 = NOW;
}
static void ref_ink(box b)
{
    if (!paused && !box_empty(b)) {
        send(b, WF_DU, M_PART);
        list_add(&inked, b);
        last_ink = NOW;
    }
}
static int ref_step(int inking, int pend_n, int64_t first_dmg)
{
    int64_t t = NOW;
    if (inked.n && !inking && t - last_ink >= INK_SETTLE_MS && !paused) {
        for (int i = 0; i < inked.n; i++) list_add(&dirty, inked.b[i]);
        inked.n = 0;
        last_dmg = t;
    }
    if (paused) {
        dirty.n = 0; inked.n = 0; run_bursts = 0; run_box = (box){0}; run_win = (box){0}; in_run = 0;
        return pend_n ? STEP_DROP : STEP_NONE;
    } else if (want_full) {
        box all = { 0, 0, W, H };
        grab(all, 1);
        send(all, WF_GC16, M_SCREEN);
        want_full = 0; last_gc16 = t;
        dirty.n = 0; inked.n = 0; run_bursts = 0; run_box = (box){0}; run_win = (box){0}; in_run = 0;
        return pend_n ? STEP_DROP : STEP_NONE;
    } else if (pend_n && !inking && t - first_dmg >= COALESCE_MS) {
        return STEP_BURST;
    } else if (!pend_n && !inking && !inked.n && (dirty.n || run_bursts) && t - last_dmg >= IDLE_GC_MS) {
        const double screen = (double)W * H;
        if (run_bursts >= RUN_BURSTS && box_area(run_box) / screen > du_max) {
            grab(run_box, 0);
            send(run_box, WF_GC16, M_AREA);
            last_gc16 = t;
            int k = 0;
            for (int i = 0; i < dirty.n; i++)
                if (!box_inside(dirty.b[i], run_box)) dirty.b[k++] = dirty.b[i];
            dirty.n = k;
        }
        if (dirty.n) {
            list_fold(&dirty, 2);
            for (int i = 0; i < dirty.n; i++) {
                grab(dirty.b[i], 0);
                send(dirty.b[i], WF_GC16, clean_full ? M_AREA : M_PART);
            }
            last_gc16 = t;
        }
        boxlist gh = {0};
        for (int ty = 0; ty < GTH; ty++)
            for (int tx = 0; tx < GTW; tx++)
                if (ghost[ty * GTW + tx] >= GHOST_MAX) {
                    box g = { tx * GTILE, ty * GTILE, (tx + 1) * GTILE, (ty + 1) * GTILE };
                    list_add(&gh, box_clip(g, (box){ 0, 0, W, H }));
                }
        list_fold(&gh, 3);
        for (int i = 0; i < gh.n; i++) {
            grab(gh.b[i], 0);
            send(gh.b[i], WF_GC16, M_AREA);
            last_gc16 = t;
        }
        dirty.n = 0;
        run_bursts = 0; run_box = (box){0}; run_win = (box){0};
        set_cut(cut_auto ? 0 : cut_fixed);
        in_run = 0;
    }
    return STEP_NONE;
}
static void ref_gc16_cleans(const boxlist *ch)
{
    for (int i = 0; i < ch->n; i++) {
        int k = 0;
        for (int j = 0; j < dirty.n; j++)
            if (!box_inside(dirty.b[j], ch->b[i])) dirty.b[k++] = dirty.b[j];
        dirty.n = k;
    }
}
static int ref_burst(boxlist ch, box pwin, int pm)
{
    int64_t t = NOW;
    const double screen = (double)W * H;
    int gc16_busy = 0;
    if (t - last_gc16 < GC16_BUSY_MS && t - gc16_box_t < GC16_BUSY_MS)
        for (int i = 0; i < ch.n && !gc16_busy; i++) gc16_busy = !box_empty(box_clip(ch.b[i], gc16_box));
    long area = 0, maxr = 0;
    box all = {0};
    for (int i = 0; i < ch.n; i++) {
        long a = box_area(ch.b[i]);
        area += a;
        if (a > maxr) maxr = a;
        all = box_union(all, ch.b[i]);
    }
    int run = ch.n && burst_auto && pm != PM_QUALITY && (pm == PM_FAST || gc16_busy || in_run);
    if (run) in_run = 1;
    if (run && box_empty(run_win)) run_win = pwin;     /* first window only (was: union) */
    for (int i = 0; run && i < ch.n; i++) run_box = box_union(run_box, box_clip(ch.b[i], run_win));
    if (run && t - last_ink > POST_INK_MS) run_bursts++;
    int du = ch.n && pm != PM_QUALITY && (!burst_auto || run ||
                      !(area / screen >= flash_min || maxr / screen > du_max));
    if (run && cut_auto) set_cut(5);
    int how = !du ? C_ALL : run && dither_runs ? C_DITHER : C_BW;
    if (run) {
        list_fold(&ch, 3);
        for (int i = 0; i < ch.n; i++) { send(ch.b[i], WF_DU, M_PART); list_add(&dirty, ch.b[i]); }
    } else if (ch.n && burst_auto && area / screen >= flash_min) {
        box in = {0}, out = {0};
        for (int i = 0; i < ch.n; i++) {
            in = box_union(in, box_clip(ch.b[i], pwin));
            if (!box_inside(ch.b[i], pwin)) out = box_union(out, ch.b[i]);
        }
        send(box_empty(in) ? all : in, WF_GC16, M_AREA);
        if (!box_empty(in) && !box_empty(out)) send(out, WF_GC16, M_PART);
        last_gc16 = t;
        ref_gc16_cleans(&ch);
    } else if (ch.n && (pm == PM_QUALITY || (burst_auto && maxr / screen > du_max))) {
        send(all, WF_GC16, M_PART);
        last_gc16 = t;
        ref_gc16_cleans(&ch);
    } else if (ch.n) {
        list_fold(&ch, 3);
        for (int i = 0; i < ch.n; i++) { send(ch.b[i], WF_DU, M_PART); list_add(&dirty, ch.b[i]); }
    }
    return how;
}
static int ref_timeout(int inking, int pend_n)
{
    if (inked.n && !inking) return COALESCE_MS;
    if (pend_n && !inking) return COALESCE_MS;
    if (dirty.n || run_bursts) return IDLE_GC_MS;
    return -1;
}

/* ---- trace helpers --------------------------------------------------------- */
static box rbox(int maxw, int maxh)
{
    int w = 1 + rand() % maxw, h = 1 + rand() % maxh;
    int x = rand() % (W - w + 1), y = rand() % (H - h + 1);
    return (box){ x, y, x + w, y + h };
}
static const box wins[] = { { 0, 0, W, H }, { 0, 977, W, H }, { 0, 32, W, 977 }, { 0, 0, W, 60 }, { 300, 400, 1100, 1400 } };

static int same(const actions *a, const actions *b, const char *what, long step)
{
    int ok = a->n == b->n;
    for (int i = 0; ok && i < a->n; i++) ok = !memcmp(&a->a[i], &b->a[i], sizeof(action));
    if (!ok) {
        fails++;
        printf("PARITY FAIL at step %ld (%s): old %d actions, new %d\n", step, what, a->n, b->n);
        char s[128];
        for (int i = 0; i < a->n; i++) { fmt_act(&a->a[i], s, sizeof s); printf("  old %s\n", s); }
        for (int i = 0; i < b->n; i++) { fmt_act(&b->a[i], s, sizeof s); printf("  new %s\n", s); }
    }
    return ok;
}

static void parity(unsigned seed, long steps)
{
    srand(seed);
    burst_auto = 1; clean_full = 0; paused = 0; du_max = 0.03; flash_min = 0.35;
    cut_auto = 1; cut_fixed = 0; cut_cur = -1; dither_runs = 1; last_du = -1000; in_run = 0;
    memset(ghost, 0, sizeof ghost); dirty.n = inked.n = 0; last_ink = 0; run_bursts = 0;
    run_box = run_win = (box){0}; last_dmg = last_gc16 = 0; want_full = 1;
    gc16_box = (box){0}; gc16_box_t = 0;
    policy p;
    pol_init(&p, W, H);
    pol_config(&p, "run_wait=0");       /* the old code: a fixed idle_ms */
    pol_config(&p, "clean_motion=0");   /* and cleanups only after a full quiet */
    pol_config(&p, "busy=0");           /* and no busy-content runs */
    pol_config(&p, "run_tail=0");       /* and every GC16 starts runs */
    pol_config(&p, "ink_redraw=off ink_clean=stroke");     /* and a stroke's cleanup is the old GC16 */
    p.run_ms = 0; p.run_fold = 3; p.run_end = RUN_END_FLASH; p.quiet_ms = 0;   /* the pre-refactor behaviour */
    int64_t now = 1000, first_dmg = 0;
    int pend_n = 0, inking = 0;
    static const char *cmds[] = { "full", "pause", "resume", "sleep", "wake", "cut=5", "cut=auto", "cut=2",
                                  "dither=off", "dither=on", "burst=du", "burst=auto", "clean=full", "clean=partial",
                                  "du_max=0.05", "du_max=0.03", "flash_min=0.3", "flash_min=0.35", "gamma=1.8" };
    for (long s = 0; s < steps && fails < 5; s++) {
        int r = rand() % 100;
        now += r < 60 ? rand() % 40 : r < 90 ? rand() % 300 : rand() % 2000;
        NOW = now;
        actions a = {0}, b = {0};
        R = &a;
        int e = rand() % 100;
        if (e < 3) {
            const char *c = cmds[rand() % (sizeof cmds / sizeof *cmds)];
            int w1 = ref_cmd(c), w2 = pol_cmd(&p, c, now, &b);
            CHECK(w1 == w2, "wake flag differs on %s", c);
            if (w1) { box ch = rand() % 2 ? rbox(W, H) : (box){0}; ref_wake(ch); pol_wake(&p, ch, now, &b); }
            if (!same(&a, &b, c, s)) printf("  cmd %s\n", c);
        } else if (e < 13) {
            inking = rand() % 3 != 0;
            box ib = rbox(40, 40);
            ref_ink(ib); pol_ink(&p, ib, now, &b);
            same(&a, &b, "ink", s);
        } else if (e < 16) {
            inking = 0;
        } else if (e < 18) {
            if (!in_run) set_cut(cut_auto ? 0 : cut_fixed);
            pol_recut(&p, now, &b);
            same(&a, &b, "recut", s);
        } else if (e < 70) {
            if (!pend_n) first_dmg = now;
            pend_n++;
            last_dmg = now; pol_damage(&p, now);
        }
        a.n = b.n = 0;
        int s1 = ref_step(inking, pend_n, first_dmg), s2 = pol_step(&p, now, inking, pend_n, first_dmg, &b);
        CHECK(s1 == s2, "step result %d vs %d at %ld", s1, s2, s);
        same(&a, &b, "step", s);
        if (s1 == STEP_BURST) {
            boxlist ch = {0};
            int n = rand() % 5, big = rand() % 3 == 0;
            for (int i = 0; i < n; i++) list_add(&ch, big ? rbox(W, H) : rbox(200, 120));
            box pwin = wins[rand() % 5];
            int pm = rand() % 3;
            a.n = b.n = 0;
            burst_in in = { now, ch, pwin, pm, 0, 0, BUSY_AUTO, 0, 0 };
            int h1 = ref_burst(ch, pwin, pm), h2 = pol_burst(&p, &in, &b);
            CHECK(h1 == h2, "commit mode %d vs %d at %ld", h1, h2, s);
            same(&a, &b, "burst", s);
        }
        if (s1 != STEP_NONE) pend_n = 0;
        if (!paused) {
            int t1 = ref_timeout(inking, pend_n), t2 = pol_timeout(&p, now, inking, pend_n);
            /* never later than the old code; earlier since the precise wakes of
             * * each one checked harmless by parity2's kind-1 triage */
            CHECK(t1 == t2 || (t2 >= 0 && (t1 < 0 || t2 < t1)), "timeout %d vs %d at step %ld (state %s): later than the old code", t1, t2, s, state_name(p.state));
        }
        CHECK(!!in_run == (p.state == S_RUN) || paused, "run flag %d vs state %s at %ld", in_run, state_name(p.state), s);
    }
}

/* ---- scenarios ------------------------------------------------------------- */
typedef struct { actions log[4096]; int n; } journal;
static void jadd(journal *j, const actions *a) { if (j->n < 4096) j->log[j->n++] = *a; }

/* drive the policy like the daemon's loop: damage -> bursts after COALESCE_MS */
static void idle_until(policy *p, int64_t *now, int64_t until, journal *j)
{
    while (*now < until) {
        *now += 10;
        actions o = {0};
        pol_step(p, *now, 0, 0, 0, &o);
        jadd(j, &o);
    }
}
static int burst_moving;                 /* the motion estimate the next bursts carry */
static int burst_pages;                  /* the app's pages= the next bursts carry */
static int burst_calm;                   /* the shell's calm (burst_in.calm) the next bursts carry */
static int burst_how = -1;               /* the last burst's C_* (pol_burst's return) */
static double burst_speed;
static void burst(policy *p, int64_t *now, boxlist ch, box pwin, int pm, journal *j)
{
    int64_t first = *now;
    pol_input(p, *now);                 /* the scenarios model a user at work */
    pol_damage(p, *now);
    *now += COALESCE_MS;
    actions o = {0};
    int st = pol_step(p, *now, 0, 1, first, &o);
    jadd(j, &o);
    if (st == STEP_BURST) {
        actions b = {0};
        burst_in in = { *now, ch, pwin, pm, burst_moving, burst_speed, BUSY_AUTO, 0, 0, burst_pages, burst_calm };
        burst_how = pol_burst(p, &in, &b);
        jadd(j, &b);
    }
}
static boxlist one(box b) { boxlist l = {0}; list_add(&l, b); return l; }
static int count(const journal *j, int wf, int mode)
{
    int n = 0;
    for (int i = 0; i < j->n; i++)
        for (int k = 0; k < j->log[i].n; k++)
            n += j->log[i].a[k].kind == A_SEND && j->log[i].a[k].wf == wf && j->log[i].a[k].mode == mode;
    return n;
}

static void scenario_firefox_scroll(void)
{
    /* Firefox in the bottom half, terminal on top, panel clock at the top.
     * Touch scrolling = "fast" profile; 3 s of scroll repaints every 50 ms
     * (sometimes the whole viewport), the clock ticks once; then idle. */
    policy p; pol_init(&p, W, H);
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 2000, j);                   /* startup full refresh */
    j->n = 0;
    box ff = { 0, 977, W, H }, view = { 0, 1100, W, H };
    for (int i = 0; i < 60; i++) {
        box b = i % 5 ? (box){ 0, 1100 + (i * 37) % 600, W, 1100 + (i * 37) % 600 + 200 } : view;
        boxlist ch = one(b);
        if (i == 30) list_add(&ch, (box){ 1300, 5, 1400, 50 });     /* clock */
        burst(&p, &now, ch, ff, PM_FAST, j);
        now += 20;
    }
    idle_until(&p, &now, now + 2000, j);
    CHECK(p.state == S_IDLE, "firefox scroll: ends idle, is %s", state_name(p.state));
    CHECK(count(j, WF_GC16, M_SCREEN) == 0, "firefox scroll: full-screen flash");
    for (int i = 0; i < j->n; i++)
        for (int k = 0; k < j->log[i].n; k++) {
            action *a = &j->log[i].a[k];
            if (a->kind == A_SEND && a->mode != M_PART)
                CHECK(box_inside(a->b, ff), "firefox scroll: flash %d,%d-%d,%d outside firefox",
                      a->b.x0, a->b.y0, a->b.x1, a->b.y1);
        }
    CHECK(count(j, WF_GC16, M_AREA) <= 1, "firefox scroll: expected at most 1 end flash, got %d", count(j, WF_GC16, M_AREA));
    free(j);
}

/* burst every 20 ms of damage during a run: throttled to one DU update per run_ms */
static void scenario_throttle(void)
{
    policy p; pol_init(&p, W, H);
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 2000, j);
    j->n = 0;
    box ff = { 0, 977, W, H };
    int64_t first = 0, end = now + 1000;
    int pend = 0;
    while (now < end) {
        now += 5;
        if (now % 20 == 0) { if (!pend) first = now; pend++; pol_damage(&p, now); pol_input(&p, now); }
        actions o = {0};
        int st = pol_step(&p, now, 0, pend, first, &o);
        jadd(j, &o);
        if (st == STEP_BURST) {
            boxlist ch = {0};
            list_add(&ch, (box){ 0, 1100, W, 1300 }); list_add(&ch, (box){ 0, 1700, W, 1800 });
            actions b = {0};
            burst_in in = { now, ch, ff, PM_FAST, 0, 0, BUSY_AUTO, 0, 0 };
            pol_burst(&p, &in, &b);
            jadd(j, &b);
        }
        if (st != STEP_NONE) pend = 0;
        if (pend) {
            int t = pol_timeout(&p, now, 0, pend);
            CHECK(t >= COALESCE_MS && t <= p.run_ms, "throttle: timeout %d", t);
        }
    }
    int du = count(j, WF_DU, M_PART);
    CHECK(du >= 8 && du <= 11, "throttle: %d DU updates in 1 s (want ~1 per %d ms)", du, p.run_ms);
    free(j);
}

/* pace=engine: damage every 20 ms in a run, an engine that takes 113 ms per
 * full-area update (A2, 10 frames at 85 Hz) — bursts come when the engine is
 * free (never one that would wait in its queue), soon after it is, and the
 * fixed run_ms no longer caps them; pace=fixed keeps the old cap */
static void scenario_pace(void)
{
    for (int fixed = 0; fixed <= 1; fixed++) {
        policy p; pol_init(&p, W, H);
        if (fixed) pol_config(&p, "pace=fixed");
        int64_t now = 1000;
        journal *j = calloc(1, sizeof *j);
        idle_until(&p, &now, 2000, j);
        j->n = 0;
        box ff = { 0, 977, W, H };
        int64_t first = 0, end = now + 2000, free_at = 0, late_max = 0;
        int pend = 0, bursts = 0, queued = 0;
        while (now < end) {
            now += 1;
            if (now % 20 == 0) { if (!pend) first = now; pend++; pol_damage(&p, now); pol_input(&p, now); }
            pol_engine(&p, p.state == S_RUN && pend ? (free_at > now ? (int)(free_at - now) : 0) : -1);
            actions o = {0};
            int st = pol_step(&p, now, 0, pend, first, &o);
            jadd(j, &o);
            if (st == STEP_BURST) {
                if (p.state == S_RUN) {
                    if (free_at > now) queued++;
                    else if (bursts > 2 && now - free_at > late_max) late_max = now - free_at;
                }
                boxlist ch = {0};
                list_add(&ch, (box){ 0, 1000, W, 1800 });
                actions b = {0};
                burst_in in = { now, ch, ff, PM_FAST, 0, 0, BUSY_AUTO, 0, 0 };
                pol_burst(&p, &in, &b);
                jadd(j, &b);
                free_at = (free_at > now ? free_at : now) + 113;
                bursts++;
            }
            if (st != STEP_NONE) pend = 0;
            if (pend && p.state == S_RUN && !fixed) {
                int t = pol_timeout(&p, now, 0, pend);
                CHECK(t <= (free_at > now ? free_at - now : 0) + COALESCE_MS, "pace: timeout %d past the engine (%lld)", t,
                      (long long)(free_at - now));
            }
        }
        if (fixed) {
            CHECK(bursts >= 17 && bursts <= 21, "pace=fixed: %d bursts in 2 s (want ~1 per run_ms)", bursts);
        } else {
            CHECK(queued == 0, "pace=engine: %d bursts read while the engine was busy", queued);
            CHECK(late_max <= 20, "pace=engine: a burst came %lld ms after the engine was free", (long long)late_max);
            CHECK(bursts >= 15 && bursts <= 19, "pace=engine: %d bursts in 2 s (want one per 113 ms)", bursts);
        }
        free(j);
    }
}

/* motion=adaptive: a run with the finger at speed(t) screens/s (finger down
 * until lift_ms, then a fling), damage every 20 ms until end_ms; each run
 * burst's step goes to lv[] (from its sends). Returns the number of bursts;
 * *flash = run-end flashes (GC16 M_AREA) after the run */
static const char *adaptive_cfg;       /* extra settings for adaptive_run */
static int adaptive_dither;            /* burst_in.dither (an app's own dither=) */
static int a2_n_max;                    /* the shortest A2 cut seen (adaptive_run) */
static int send_level(const action *a)
{
    if (a->kind != A_SEND) return -1;
    if (a->wf == WF_GC16) return a->mode == M_PART ? LV_GC16 : -1;
    if (a->wf == WF_DU4) return LV_DU4;
    if (a->n && (!a2_n_max || a->n < a2_n_max)) a2_n_max = a->n;
    return a->n == 8 ? LV_A2_8 : a->n == 6 ? LV_A2_6 : a->n == 4 ? LV_A2_4 : a->n == 2 ? LV_A2_2 : LV_A2;
}
static int adaptive_run(double (*speed)(int64_t), int64_t lift_ms, int64_t end_ms, int engine, int *lv, int *flash, int *how_last)
{
    policy p; pol_init(&p, W, H);
    pol_config(&p, "motion=adaptive");
    if (adaptive_cfg) pol_config(&p, adaptive_cfg);
    p.has_engine = engine;
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 2000, j);
    j->n = 0;
    int64_t t0 = now, first = 0;
    int pend = 0, n = 0;
    box ff = { 0, 977, W, H };
    while (now < t0 + end_ms) {
        now += 2;
        int64_t t = now - t0;
        pol_finger(&p, now, t < lift_ms, t < lift_ms ? speed(t) : 0);
        if (now % 20 == 0) { if (!pend) first = now; pend++; pol_damage(&p, now); if (t < lift_ms) pol_input(&p, now); }
        actions o = {0};
        int st = pol_step(&p, now, 0, pend, first, &o);
        if (st == STEP_BURST) {
            boxlist ch = {0};
            list_add(&ch, (box){ 0, 1000, W, 1800 });
            actions b = {0};
            burst_in in = { now, ch, ff, PM_FAST, 0, 0, BUSY_AUTO, adaptive_dither, 0 };
            int how = pol_burst(&p, &in, &b);
            if (how_last) *how_last = how;
            int l = -1;
            for (int k = 0; k < b.n; k++) if (send_level(&b.a[k]) > l) l = send_level(&b.a[k]);
            if (n < 512) lv[n++] = l;
        }
        if (st != STEP_NONE) pend = 0;
    }
    j->n = 0;
    idle_until(&p, &now, now + 2000, j);
    *flash = count(j, WF_GC16, M_AREA);
    CHECK(p.state == S_IDLE, "adaptive: not idle after the run (%s)", state_name(p.state));
    free(j);
    return n;
}
static double v_slow(int64_t t) { (void)t; return 0.05; }
static double v_fast(int64_t t) { (void)t; return 6; }
static double v_noisy(int64_t t) { return (t / 20) % 2 ? 0.8 * 1.15 : 0.8 * 0.87; }   /* around the A2 | A2/8 line */
static double v_ramp(int64_t t) { return 0.05 * pow(120, t / 1500.0); }                 /* 0.05 -> 6 */
static double v_flick(int64_t t) { (void)t; return 4; }
static double v_kick(int64_t t) { return t < 600 ? 0.05 : 6; }                          /* slow, then a sudden flick */
/* bursts after the kick until the fastest step */
static int kick_lag(const int *lv, int n, int at)
{
    for (int i = at; i < n; i++) if (lv[i] == LV_A2_4) return i - at;
    return 99;
}

static void scenario_adaptive(void)
{
    int lv[512], fl, how, n;
    /* a slow reading drag: GC16 steps (real greys), no cut, no end flash */
    n = adaptive_run(v_slow, 1500, 1500, 1, lv, &fl, &how);
    int bad = 0;
    for (int i = 0; i < n; i++) bad += lv[i] != LV_GC16;
    CHECK(n >= 3 && !bad && fl == 0 && how == C_ALL, "adaptive slow: %d bursts, %d not GC16, %d flashes", n, bad, fl);
    /* a fast flick: A2 cut to 4 frames, dithered; the run ends in a flash */
    n = adaptive_run(v_fast, 1500, 1500, 1, lv, &fl, &how);
    bad = 0;
    for (int i = 0; i < n; i++) bad += lv[i] != LV_A2_4;
    CHECK(n >= 10 && !bad && fl >= 1 && how == C_DITHER, "adaptive fast: %d bursts, %d not A2/4, %d flashes", n, bad, fl);
    /* noise around a threshold: the hysteresis holds the step */
    n = adaptive_run(v_noisy, 2000, 2000, 1, lv, &fl, &how);
    int changes = 0;
    for (int i = 1; i < n; i++) changes += lv[i] != lv[i - 1];
    CHECK(changes <= 2, "adaptive noisy: %d step changes in %d bursts", changes, n);
    /* speeding up: the steps only go up */
    n = adaptive_run(v_ramp, 1500, 1500, 1, lv, &fl, &how);
    int down = 0;
    for (int i = 1; i < n; i++) down += lv[i] < lv[i - 1];
    CHECK(!down && lv[0] <= LV_DU4 && lv[n - 1] >= LV_A2_6, "adaptive ramp: %d steps down, first %d last %d", down, lv[0], lv[n - 1]);
    /* a fling: fast while the finger moves, then the steps come down as the
     * fling slows (the content keeps moving), never back up */
    n = adaptive_run(v_flick, 300, 2500, 1, lv, &fl, &how);
    down = 0;
    int up = 0;
    for (int i = 1; i < n; i++) { down += lv[i] < lv[i - 1]; up += lv[i] > lv[i - 1] && i > 2; }
    CHECK(lv[1] >= LV_A2_6 && down >= 2 && !up && lv[n - 1] <= LV_A2, "adaptive fling: first %d last %d, %d down %d up", lv[1], lv[n - 1], down, up);
    /* steps=: the cut lengths are settings (a flick reaches the last one) */
    adaptive_cfg = "steps=4,3,2"; a2_n_max = 0;
    n = adaptive_run(v_fast, 1500, 1500, 1, lv, &fl, &how);
    CHECK(a2_n_max == 2, "adaptive steps=4,3,2: shortest cut %d", a2_n_max);
    adaptive_cfg = "steps=4,6,2"; a2_n_max = 0;          /* not falling: ignored */
    n = adaptive_run(v_fast, 1500, 1500, 1, lv, &fl, &how);
    CHECK(a2_n_max == 4, "adaptive steps=4,6,2 should be refused: shortest cut %d", a2_n_max);
    /* LV_A2_2: off by default (a flick ends at A2/4); a 6th ladder value
     * reaches it; a 5-value ladder still applies and leaves it off; steps=
     * takes its length as a 4th value */
    /* an app's own dither=: off dots a run with dither=on; its pattern dithers with dither=off */
    adaptive_dither = -1;
    adaptive_run(v_fast, 1500, 1500, 1, lv, &fl, &how);
    CHECK(how == C_THRESH, "app dither=off: run commit %d, want thresholded", how);
    adaptive_cfg = "dither=off"; adaptive_dither = 1;
    adaptive_run(v_fast, 1500, 1500, 1, lv, &fl, &how);
    CHECK(how == C_DITHER, "app's own pattern with dither=off: run commit %d, want dithered", how);
    adaptive_dither = 0;
    adaptive_run(v_fast, 1500, 1500, 1, lv, &fl, &how);
    CHECK(how == C_THRESH, "dither=off, no app's own: run commit %d", how);
    adaptive_cfg = NULL; a2_n_max = 0;
    n = adaptive_run(v_fast, 1500, 1500, 1, lv, &fl, &how);
    CHECK(a2_n_max == 4 && lv[n - 1] == LV_A2_4, "adaptive A2/2 default off: shortest cut %d, last step %d", a2_n_max, lv[n - 1]);
    adaptive_cfg = "ladder=0.12,0.3,0.8,1.6,3.2,4.5"; a2_n_max = 0;
    n = adaptive_run(v_fast, 1500, 1500, 1, lv, &fl, &how);
    bad = 0;
    for (int i = 0; i < n; i++) bad += lv[i] != LV_A2_2;
    CHECK(n >= 10 && !bad && a2_n_max == 2 && fl >= 1, "adaptive A2/2 ladder: %d of %d bursts not A2/2, shortest %d, %d flashes", bad, n, a2_n_max, fl);
    adaptive_cfg = "ladder=0.01,0.02,0.03,0.04,0.05"; a2_n_max = 0;
    n = adaptive_run(v_noisy, 1500, 1500, 1, lv, &fl, &how);
    bad = 0;
    for (int i = 0; i < n; i++) bad += lv[i] != LV_A2_4;
    CHECK(n >= 10 && !bad, "adaptive 5-value ladder: %d of %d bursts not A2/4 (refused, or A2/2 not off)", bad, n);
    adaptive_cfg = "ladder=0.12,0.3,0.8,1.6,3.2,4.5 steps=8,6,4,3"; a2_n_max = 0;
    n = adaptive_run(v_fast, 1500, 1500, 1, lv, &fl, &how);
    CHECK(a2_n_max == 3, "adaptive steps=8,6,4,3: shortest cut %d", a2_n_max);
    adaptive_cfg = NULL;
    /* attack=jump: a sudden flick reaches the fastest step at the next burst;
     * fused takes longer (the prior damps the rise) */
    int kick = -1, lag_f, lag_j;
    adaptive_cfg = NULL;
    n = adaptive_run(v_kick, 1500, 1500, 1, lv, &fl, &how);
    for (int i = 0; i < n && kick < 0; i++) if (lv[i] != LV_GC16) kick = i;
    lag_f = kick < 0 ? 99 : kick_lag(lv, n, kick);
    adaptive_cfg = "attack=jump";
    n = adaptive_run(v_kick, 1500, 1500, 1, lv, &fl, &how);
    kick = -1;
    for (int i = 0; i < n && kick < 0; i++) if (lv[i] != LV_GC16) kick = i;
    lag_j = kick < 0 ? 99 : kick_lag(lv, n, kick);
    CHECK(lag_j == 0 && lag_f > 0, "adaptive attack: fastest step %d bursts after the kick with jump, %d fused", lag_j, lag_f);
    /* the way down stays damped under jump: the fling still steps down, never up */
    n = adaptive_run(v_flick, 300, 2500, 1, lv, &fl, &how);
    down = up = 0;
    for (int i = 1; i < n; i++) { down += lv[i] < lv[i - 1]; up += lv[i] > lv[i - 1] && i > 2; }
    CHECK(down >= 2 && !up, "adaptive attack=jump fling: %d down %d up", down, up);
    /* mmin=: a slow drag uses the floor step, not GC16; a2 is not a2_8 */
    static const struct { const char *cfg; int lv; } fl_t[] = { { "mmin=du4", LV_DU4 }, { "mmin=a2", LV_A2 }, { "mmin=a2_6", LV_A2_6 } };
    for (unsigned k = 0; k < sizeof fl_t / sizeof *fl_t; k++) {
        adaptive_cfg = fl_t[k].cfg;
        n = adaptive_run(v_slow, 1500, 1500, 1, lv, &fl, &how);
        bad = 0;
        for (int i = 0; i < n; i++) bad += lv[i] != fl_t[k].lv;
        CHECK(n >= 3 && !bad, "adaptive %s: %d of %d bursts not step %d", fl_t[k].cfg, bad, n, fl_t[k].lv);
    }
    adaptive_cfg = "mmin=du4";                           /* a floor never caps: a flick still cuts */
    n = adaptive_run(v_fast, 1500, 1500, 1, lv, &fl, &how);
    CHECK(lv[n - 1] == LV_A2_4, "adaptive mmin=du4 flick: last step %d", lv[n - 1]);
    /* mmax=: a ceiling (how far adaptive goes); mmin=mmax: one fixed paint */
    adaptive_cfg = "ladder=0.12,0.3,0.8,1.6,3.2,4.5 mmax=a2_4";
    n = adaptive_run(v_fast, 1500, 1500, 1, lv, &fl, &how);
    bad = 0;
    for (int i = 0; i < n; i++) bad += lv[i] > LV_A2_4;
    CHECK(n >= 3 && !bad && lv[n - 1] == LV_A2_4, "adaptive mmax=a2_4 flick: %d bursts past A2/4, last %d", bad, lv[n - 1]);
    static const char *const fixed[] = { "mmin=a2_2 mmax=a2_2", "mmin=a2_6 mmax=a2_6" };
    for (int k = 0; k < 2; k++)
        for (int sp = 0; sp < 2; sp++) {
            adaptive_cfg = fixed[k];
            n = adaptive_run(sp ? v_fast : v_slow, 1500, 1500, 1, lv, &fl, &how);
            bad = 0;
            for (int i = 0; i < n; i++) bad += lv[i] != (k ? LV_A2_6 : LV_A2_2);
            CHECK(n >= 3 && !bad, "fixed %s (%s): %d of %d bursts not that step", fixed[k], sp ? "fast" : "slow", bad, n);
        }
    /* curve=off: any motion is the range's fast end, slow or fast */
    static const struct { const char *cfg; int lv; } cv[] = {
        { "mmin=gc16 mmax=a2_2 curve=off", LV_A2_2 }, { "mmin=gc16 mmax=a2_4 curve=off", LV_A2_4 },
        { "mmin=a2 mmax=a2_6 curve=off", LV_A2_6 } };
    for (unsigned k = 0; k < sizeof cv / sizeof *cv; k++)
        for (int sp = 0; sp < 2; sp++) {
            adaptive_cfg = cv[k].cfg;
            n = adaptive_run(sp ? v_fast : v_slow, 1500, 1500, 1, lv, &fl, &how);
            bad = 0;
            for (int i = 0; i < n; i++) bad += lv[i] != cv[k].lv;
            CHECK(n >= 3 && !bad, "%s (%s): %d of %d bursts not step %d", cv[k].cfg, sp ? "fast" : "slow", bad, n, cv[k].lv);
        }
    adaptive_cfg = NULL;
    /* no engine (/dev/ebc): binary — plain DU, no cuts */
    n = adaptive_run(v_fast, 1500, 1500, 0, lv, &fl, &how);
    bad = 0;
    for (int i = 0; i < n; i++) bad += lv[i] != LV_A2;
    CHECK(!bad, "adaptive without engine: %d bursts not plain DU", bad);
}

/* a short scroll ends in a partial GC16 (no flash); a long one in a flash */
static void scenario_run_end(void)
{
    for (int steps = 5; steps <= 60; steps += 55) {
        policy p; pol_init(&p, W, H);
        int64_t now = 1000;
        journal *j = calloc(1, sizeof *j);
        idle_until(&p, &now, 2000, j);
        j->n = 0;
        box ff = { 0, 977, W, H };
        for (int i = 0; i < steps; i++) {
            burst(&p, &now, one((box){ 0, 1100, W, 1800 }), ff, PM_FAST, j);
            now += 80;
        }
        j->n = 0;
        idle_until(&p, &now, now + 1500, j);
        int fl = count(j, WF_GC16, M_AREA), pa = count(j, WF_GC16, M_PART);
        if (steps < GHOST_MAX) CHECK(fl == 0 && pa == 1, "short scroll end: %d flashes, %d partials", fl, pa);
        else CHECK(fl == 1 && pa == 0, "long scroll end: %d flashes, %d partials", fl, pa);
        free(j);
    }
}

/* the run tail (a trace of Chromium after a fling): the run
 * ends in its GC16; ~240 ms later the overlay scrollbar fades out (6 small
 * frames, 25 ms apart, nothing measured moving), ~740 ms after the end the
 * scroll-end re-raster (most of the window). No new run, no second flash.
 * run_tail=0 (the old rule): the fade restarts a run. A finger down: a run. */
static int tail_log;
static void tail_logf(const char *fmt, ...)
{
    if (!tail_log) return;
    va_list ap; va_start(ap, fmt); vprintf(fmt, ap); va_end(ap);
}
static int tail_case(const char *cfg, int finger, int *flashes, int *partials)
{
    policy p; pol_init(&p, W, H);
    p.log = tail_logf;
    if (cfg) pol_config(&p, cfg);
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 2000, j);
    box ff = { 0, 60, W, H };
    for (int i = 0; i < 40; i++) {
        burst(&p, &now, one((box){ 0, 300 + (i * 31) % 900, W, 300 + (i * 31) % 900 + 400 }), ff, PM_FAST, j);
        now += 25;
    }
    while (p.state != S_IDLE && now < 10000) idle_until(&p, &now, now + 10, j);
    j->n = 0;
    int ran = 0;
    tail_log = getenv("TAIL_LOG") != NULL;
    if (tail_log) printf("-- tail (%s) at %lld, run end %lld\n", cfg ? cfg : "default", (long long)now, (long long)p.last_run_end);
    idle_until(&p, &now, now + 240, j);
    if (finger == 2) pol_wheel(&p, now);                /* a touchpad scroll (held wheel_hold_ms) */
    else if (finger) pol_finger(&p, now, 1, 0);
    for (int i = 0; i < 6; i++) {
        burst(&p, &now, one((box){ 1390, 160, 1398, 600 }), ff, PM_AUTO, j);
        ran |= p.state == S_RUN;
        idle_until(&p, &now, now + 25, j);
    }
    if (finger == 1) pol_finger(&p, now, 0, 0);
    idle_until(&p, &now, now + 350, j);
    burst(&p, &now, one((box){ 60, 160, 1400, 1500 }), ff, PM_AUTO, j);
    ran |= p.state == S_RUN;
    idle_until(&p, &now, now + 3000, j);
    tail_log = 0;
    *flashes = count(j, WF_GC16, M_AREA) + count(j, WF_GC16, M_SCREEN);
    *partials = count(j, WF_GC16, M_PART);
    free(j);
    return ran;
}
/* a game drawing on after a run end (OpenMW traces): large
 * frames every 60 ms. Old: a GC16 partial each for the whole 2 s tail (~33),
 * queued on the same pixels. Now the tail ends after TAIL_FAST_N of them and
 * the content runs. Returns the GC16 partials before the run (-1: no run) */
static int tail_anim(void)
{
    policy p; pol_init(&p, W, H);
    p.log = tail_logf;
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 2000, j);
    box ff = { 0, 60, W, H };
    for (int i = 0; i < 40; i++) {
        burst(&p, &now, one((box){ 0, 300 + (i * 31) % 900, W, 300 + (i * 31) % 900 + 400 }), ff, PM_FAST, j);
        now += 25;
    }
    while (p.state != S_IDLE && now < 10000) idle_until(&p, &now, now + 10, j);
    j->n = 0;
    int pa = -1;
    tail_log = getenv("TAIL_LOG") != NULL;
    if (tail_log) printf("-- tail_anim at %lld, run end %lld\n", (long long)now, (long long)p.last_run_end);
    idle_until(&p, &now, now + 40, j);
    for (int i = 0; i < 30 && pa < 0; i++) {
        burst(&p, &now, one((box){ 0, 104, W, H }), (box){ 0, 104, W, H }, PM_AUTO, j);
        if (p.state == S_RUN) pa = count(j, WF_GC16, M_PART);
        idle_until(&p, &now, now + 60, j);
    }
    tail_log = 0;
    free(j);
    return pa;
}
/* a foreign GC16 ends the tail (a quality app's repaint, 550 ms after the run
 * end); a cleanup after it must not restart it (not
 * intended — the cleanup marked every GC16 so far as the tail's own). Returns
 * whether the late re-raster still went the tail's way (its GC16 marked ours;
 * inside run_tail, no run between) */
static int tail_foreign(int fade)
{
    policy p; pol_init(&p, W, H);
    pol_config(&p, "busy=0");          /* (the scroll before looks busy: its rule would restart a run) */
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 2000, j);
    box ff = { 0, 60, W, H };
    for (int i = 0; i < 40; i++) {
        burst(&p, &now, one((box){ 0, 300 + (i * 31) % 900, W, 300 + (i * 31) % 900 + 400 }), ff, PM_FAST, j);
        now += 25;
    }
    while (p.state != S_IDLE && now < 10000) idle_until(&p, &now, now + 10, j);
    idle_until(&p, &now, now + GC16_BUSY_MS + 50, j);                     /* (the run end's GC16 landed) */
    burst(&p, &now, one((box){ 100, 200, 300, 260 }), ff, PM_QUALITY, j);    /* the foreign GC16 */
    CHECK(p.state != S_RUN && p.last_gc16 > p.last_own_gc16, "tail_foreign: the quality repaint was no foreign GC16 (%s)", state_name(p.state));
    idle_until(&p, &now, now + GC16_BUSY_MS + 100, j);                    /* (no longer in flight: no run) */
    if (fade) burst(&p, &now, one((box){ 1390, 160, 1398, 600 }), ff, PM_AUTO, j);   /* a small change, its cleanup */
    idle_until(&p, &now, now + 350, j);
    CHECK(p.state == S_IDLE, "tail_foreign: state %s before the re-raster", state_name(p.state));
    burst(&p, &now, one((box){ 60, 160, 1400, 1500 }), ff, PM_AUTO, j);
    int tailed = p.last_own_gc16 == p.last_gc16 && p.last_gc16 >= now - COALESCE_MS;
    free(j);
    return tailed;
}
static void scenario_run_tail(void)
{
    int ta = tail_anim();
    CHECK(ta >= 0 && ta < TAIL_FAST_N, "run tail: a game's frames every 60 ms got %d GC16 partials before a run (want < %d; -1 = no run)", ta, TAIL_FAST_N);
    int fl, pa;
    int ran = tail_case(NULL, 0, &fl, &pa);
    CHECK(!ran, "run tail: a still frame after the run end restarted a run");
    CHECK(fl == 0, "run tail: %d flashes after the run end", fl);
    CHECK(pa >= 1, "run tail: the late frames never got their GC16");
    ran = tail_case("run_tail=0", 0, &fl, &pa);
    CHECK(ran, "run_tail=0: the old rule (a GC16 this recent starts a run) is gone");
    ran = tail_case(NULL, 1, &fl, &pa);
    CHECK(ran, "run tail: a finger down after the run end started no run");
    ran = tail_case(NULL, 2, &fl, &pa);                 /* (one "engaged" since the wheel too) */
    CHECK(ran, "run tail: a wheel scroll after the run end started no run");
    int nf = tail_foreign(0), wf = tail_foreign(1);
    CHECK(!nf, "run tail: a foreign GC16 did not end the tail");
    CHECK(!wf, "run tail: a cleanup after a foreign GC16 restarted the tail");
}

/* Rnote after a stroke (capture): the pen DUs the stroke, ~0.46 s
 * after the lift the app's first re-render (GC16-sized: 752x474), ~50 ms later
 * a second one under that GC16, ~70 ms later a third. Old: GC16 partial, the
 * second started a run (cut A2), the run end flashed = 3 refreshes, a flash.
 * ink_redraw=du: DUs, then ONE GC16 partial after the last re-render.
 * `second`: a second stroke starts 150 ms after the second re-render (writing
 * on); `outside`: the third re-render also changes something away from the ink.
 * Returns the GC16 refreshes after the lift (steps that sent a GC16; a cleanup
 * may send one per dirty box), *flash = flashes (M_AREA), *du = DUs */
static void rnote_burst(policy *p, int64_t now, boxlist ch, journal *j)
{
    pol_damage(p, now);
    actions o = {0};
    int st = pol_step(p, now + COALESCE_MS, 0, 1, now, &o);
    jadd(j, &o);
    if (st == STEP_BURST) {
        actions b = {0};
        burst_in in = { now + COALESCE_MS, ch, { 0, 110, W, 1872 }, PM_AUTO, 0, 0, BUSY_AUTO, 0, 0 };
        pol_burst(p, &in, &b);
        jadd(j, &b);
    }
}
static void rnote_stroke(policy *p, int64_t *now, int x0, int y0, journal *j)
{
    for (int i = 0; i < 44; i++) {                  /* ~130 ms, 3 ms apart, down-right */
        int x = x0 + i * 17, y = y0 + i * 11;
        pol_inked(p, (box){ x, y, x + 24, y + 30 }, *now);
        actions o = {0};
        pol_step(p, *now, 1, 0, 0, &o);
        jadd(j, &o);
        *now += 3;
    }
}
static int rnote_wheel;                 /* a wheel event before the second re-render */
static int rnote_case(const char *cfg, int second, int outside, int *flash, int *du)
{
    policy p; pol_init(&p, W, H);
    p.has_engine = 1;
    pol_config(&p, "cut=auto dither=on run_ms=100 idle_ms=250 quiet_ms=2000 quiet_after=5000 "
                   "mmin=gc16 attack=jump motion=adaptive mmax=a2_2 still=range curve=off");   /* the device's eink.conf */
    if (cfg) pol_config(&p, cfg);
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 3000, j);
    int64_t d0 = now;                               /* the app's damage from the stroke's start, held */
    pol_damage(&p, now);
    rnote_stroke(&p, &now, 450, 440, j);
    j->n = 0;                                       /* after the lift */
    box sb = { 450, 439, 1205, 943 };               /* the stroke's changed pixels */
    now += 460;                                     /* (the daemon's read waited: no steps, the damage pending) */
    {
        actions o = {0};
        int st = pol_step(&p, now, 0, 1, d0, &o);
        jadd(j, &o);
        CHECK(st == STEP_BURST, "rnote: no read of the first re-render");
        actions b = {0};
        burst_in in = { now, one(sb), { 0, 110, W, 1872 }, PM_AUTO, 0, 0, BUSY_AUTO, 0, 0 };
        pol_burst(&p, &in, &b);
        jadd(j, &b);
    }
    now += COALESCE_MS;
    idle_until(&p, &now, now + 37, j);
    if (rnote_wheel) pol_wheel(&p, now);
    rnote_burst(&p, now, one(sb), j); now += COALESCE_MS;
    if (second) {
        idle_until(&p, &now, now + 150, j);
        rnote_stroke(&p, &now, 300, 1000, j);
        box s2 = { 300, 999, 1055, 1503 };
        idle_until(&p, &now, now + 460, j);
        rnote_burst(&p, now, one(s2), j); now += COALESCE_MS;
    } else {
        idle_until(&p, &now, now + 60, j);      /* (its damage; the device read it 0.36 s later, the engine busy) */
        boxlist ch = one((box){ 451, 440, 1207, 945 });
        if (outside == 1) list_add(&ch, (box){ 1300, 120, 1380, 180 });   /* a toolbar button: small, a DU too */
        if (outside == 2) list_add(&ch, (box){ 0, 1300, W, 1800 });       /* a big change elsewhere */
        rnote_burst(&p, now, ch, j); now += COALESCE_MS;
    }
    idle_until(&p, &now, now + 3000, j);
    CHECK(p.state == S_IDLE, "rnote: ends %s", state_name(p.state));
    *flash = count(j, WF_GC16, M_AREA);
    *du = count(j, WF_DU, M_PART);
    int g = 0;                                      /* GC16 refreshes: the steps that sent one */
    for (int i = 0; i < j->n; i++) {
        int any = 0;
        for (int k = 0; k < j->log[i].n; k++) any |= j->log[i].a[k].kind == A_SEND && j->log[i].a[k].wf == WF_GC16;
        g += any;
    }
    if (getenv("RNOTE_V")) {
        char s[128];
        printf("rnote %s second %d outside %d:\n", cfg ? cfg : "(default)", second, outside);
        for (int i = 0; i < j->n; i++)
            for (int k = 0; k < j->log[i].n; k++) { fmt_act(&j->log[i].a[k], s, sizeof s); printf("  %s\n", s); }
    }
    free(j);
    return g;
}
static void scenario_rnote_ink(void)
{
    int fl, du, g;
    /* default : ink_redraw=du + ink_clean=damage — the
     * re-renders are DUs, no GC16 at all for one stroke (the ink debt waits) */
    g = rnote_case(NULL, 0, 0, &fl, &du);
    CHECK(g == 0 && fl == 0 && du == 3, "rnote default: %d GC16s %d flashes %d DUs (want 0 / 0 / 3)", g, fl, du);
    g = rnote_case(NULL, 1, 0, &fl, &du);           /* writing on */
    CHECK(g == 0 && fl == 0, "rnote default, two strokes: %d GC16s %d flashes (want 0 / 0)", g, fl);
    g = rnote_case(NULL, 0, 1, &fl, &du);           /* + a small change away from the ink (the undo button) */
    CHECK(g == 0 && fl == 0 && du == 4, "rnote default, a button beside: %d GC16s %d flashes %d DUs (want 0 / 0 / 4)", g, fl, du);
    g = rnote_case(NULL, 0, 2, &fl, &du);           /* + a big change elsewhere: the sized choice, as before */
    CHECK(du == 2 && g >= 1, "rnote default, a big change elsewhere: %d DUs (want 2: the third read sized), %d GC16s", du, g);
    /* ink_clean=stroke: the GC16 partial after each stroke (ink_redraw=du alone) */
    g = rnote_case("ink_clean=stroke", 0, 0, &fl, &du);
    CHECK(g == 1 && fl == 0 && du == 3, "rnote ink_clean=stroke: %d GC16s %d flashes %d DUs (want 1 / 0 / 3)", g, fl, du);
    g = rnote_case("ink_clean=stroke", 1, 0, &fl, &du);  /* writing on: one partial at the end, no flash */
    CHECK(g == 1 && fl == 0, "rnote ink_clean=stroke, two strokes: %d GC16s %d flashes (want 1 / 0)", g, fl);
    g = rnote_case("ink_clean=stroke", 0, 1, &fl, &du);
    CHECK(g == 1 && fl == 0 && du == 4, "rnote ink_clean=stroke, a button beside: %d GC16s %d flashes %d DUs (want 1 / 0 / 4)", g, fl, du);
    g = rnote_case("ink_redraw=off ink_clean=stroke", 0, 0, &fl, &du);
    CHECK(g == 2 && fl == 1, "rnote ink_redraw=off ink_clean=stroke: %d GC16s %d flashes (want the old 2 / 1)", g, fl);
    /* a touchpad scroll right after the stroke: the second change is no ink
     * re-render is engaged (one "engaged" since the wheel too) */
    rnote_wheel = 1;
    g = rnote_case(NULL, 0, 0, &fl, &du);
    rnote_wheel = 0;
    CHECK(du < 3, "rnote + a wheel scroll: %d DUs, %d GC16s (want the second change no ink re-render: < 3 DUs)", du, g);
}

/* ink_clean=damage: handwriting (strokes without app re-renders) over lines;
 * the one GC16 partial only once enough tiles are inked AND the pen rested
 * INK_DEBT_PAUSE_MS; a page turn's GC16 clears the debt for free; strokes over
 * one spot get it by the ghost rule */
static int ink_lines(policy *p, int64_t *now, int lines, int y0, journal *j)
{
    int strokes = 0;
    for (int l = 0; l < lines; l++)
        for (int x = 40; x + 60 < W; x += 70, strokes++) {     /* a letter: ~60x50, 12 pieces */
            for (int i = 0; i < 12; i++) {
                actions o = {0};
                int y = y0 + l * 64 + (i & 3) * 10;
                pol_inked(p, (box){ x + i * 4, y, x + i * 4 + 12, y + 14 }, *now);
                pol_step(p, *now, 1, 0, 0, &o);
                jadd(j, &o);
                *now += 8;
            }
            idle_until(p, now, *now + 120, j);                /* between letters: the pen lifts */
        }
    return strokes;
}
static void scenario_ink_debt(void)
{
    policy p; pol_init(&p, W, H);
    p.has_engine = 1;
    pol_config(&p, "cut=auto dither=on run_ms=100 idle_ms=250 quiet_ms=2000 quiet_after=5000 "
                   "mmin=gc16 attack=jump motion=adaptive mmax=a2_2 still=range curve=off");
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 3000, j);
    j->n = 0;
    int st = ink_lines(&p, &now, 2, 200, j);           /* two lines: below the threshold */
    idle_until(&p, &now, now + 10000, j);
    CHECK(count(j, WF_GC16, M_PART) == 0 && count(j, WF_GC16, M_AREA) == 0,
          "ink debt: %d strokes on 2 lines, %d tiles: %d GC16 partials %d flashes (want none)", st, p.ink_tiles,
          count(j, WF_GC16, M_PART), count(j, WF_GC16, M_AREA));
    CHECK(count(j, WF_DU, M_PART) >= 1, "ink debt: %d DUs for %d strokes (the strokes' cleanup: a DU)", count(j, WF_DU, M_PART), st);
    j->n = 0;
    ink_lines(&p, &now, 5, 400, j);                     /* five more: over the threshold */
    int64_t lift = now;
    idle_until(&p, &now, lift + INK_DEBT_PAUSE_MS - 200, j);
    CHECK(count(j, WF_GC16, M_PART) == 0, "ink debt: a GC16 before the pen rested %d ms", INK_DEBT_PAUSE_MS);
    idle_until(&p, &now, lift + INK_DEBT_PAUSE_MS + 500, j);
    CHECK(count(j, WF_GC16, M_AREA) == 0, "ink debt: %d flashes (want none)", count(j, WF_GC16, M_AREA));
    CHECK(count(j, WF_GC16, M_PART) >= 1 && p.ink_debt_cleans == 1 && !p.ink_debt.n && !p.ink_tiles,
          "ink debt: %d GC16 partials, %lu debt cleans, %d boxes / %d tiles left (want one clean, nothing left)",
          count(j, WF_GC16, M_PART), p.ink_debt_cleans, p.ink_debt.n, p.ink_tiles);
    /* a page turn (the app's whole-window GC16) clears the debt: no extra GC16 */
    j->n = 0;
    ink_lines(&p, &now, 7, 200, j);
    idle_until(&p, &now, now + 500, j);
    {
        actions o = {0};
        pol_damage(&p, now);
        int s2 = pol_step(&p, now + COALESCE_MS, 0, 1, now, &o);
        if (s2 == STEP_BURST) {
            burst_in in = { now + COALESCE_MS, one((box){ 0, 110, W, 1872 }), { 0, 110, W, 1872 }, PM_AUTO, 0, 0, BUSY_AUTO, 0, 0 };
            pol_burst(&p, &in, &o);
        }
        jadd(j, &o);
    }
    unsigned long dc = p.ink_debt_cleans;
    idle_until(&p, &now, now + 6000, j);
    CHECK(p.ink_debt_cleans == dc && !p.ink_debt.n, "ink debt: a page turn's GC16 left %d boxes, %lu more debt cleans (want 0 / 0)",
          p.ink_debt.n, p.ink_debt_cleans - dc);
    /* strokes over strokes in one place: the ghost rule, without the area */
    policy q; pol_init(&q, W, H);
    pol_config(&q, "cut=auto idle_ms=250");
    now = 1000;
    j->n = 0;
    for (int k = 0; k < 20; k++) {
        for (int i = 0; i < 6; i++) {
            actions o = {0};
            pol_inked(&q, (box){ 600 + i * 3, 600, 620 + i * 3, 620 }, now);
            pol_step(&q, now, 1, 0, 0, &o);
            jadd(j, &o);
            now += 8;
        }
        idle_until(&q, &now, now + 400, j);
    }
    CHECK(q.ink_tiles < INK_DEBT_AREA * GTW * GTH, "ink debt: the overwrite case is small (%d tiles)", q.ink_tiles);
    idle_until(&q, &now, now + INK_DEBT_PAUSE_MS + 500, j);
    CHECK(q.ink_debt_cleans == 1 && count(j, WF_GC16, M_AREA) == 0, "ink debt: 20 strokes over one spot: %lu debt cleans %d flashes (want 1 / 0)",
          q.ink_debt_cleans, count(j, WF_GC16, M_AREA));
    free(j);
}

/* pol_timeout wakes the daemon for every wait pol_step has: a stroke's ink
 * cleanup (inkd/inkr only, after POST_INK_MS) and the ink debt's GC16 (before:
 * both returned -1, epdd slept up to 5 s) */
static void scenario_ink_timeout(void)
{
    policy p; pol_init(&p, W, H);
    actions o = {0};
    pol_step(&p, 0, 0, 0, 0, &o);                        /* (pol_init's want_full) */
    box ink = { 400, 400, 600, 500 };
    pol_inked(&p, ink, 1000);
    o.n = 0; pol_step(&p, 1100, 0, 0, 0, &o);            /* inked -> inkd */
    burst_in in = { 1500, one((box){ 450, 420, 550, 480 }), { 0, 110, W, 1872 }, PM_AUTO, 0, 0, BUSY_AUTO, 0, 0 };
    o.n = 0; pol_burst(&p, &in, &o);                     /* the app's re-render of the stroke */
    int t = pol_timeout(&p, 1500, 0, 0);
    int64_t fire = -1;
    for (int64_t now = 1510; now <= 7000 && fire < 0; now += 10) {
        o.n = 0; pol_step(&p, now, 0, 0, 0, &o);
        if (!p.inkd.n && !p.inkr.n) fire = now;
    }
    CHECK(fire > 0 && t > 0 && t <= fire - 1500, "ink timeout: after a stroke + redraw pol_timeout %d, the cleanup at +%ld",
          t, (long)(fire - 1500));
    /* the ink debt: strokes over one spot, then the pen rests */
    policy q; pol_init(&q, W, H);
    pol_config(&q, "cut=auto idle_ms=250");
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    for (int k = 0; k < 20; k++) {
        for (int i = 0; i < 6; i++) {
            o.n = 0;
            pol_inked(&q, (box){ 600 + i * 3, 600, 620 + i * 3, 620 }, now);
            pol_step(&q, now, 1, 0, 0, &o);
            now += 8;
        }
        idle_until(&q, &now, now + 400, j);
    }
    idle_until(&q, &now, now + 1500, j);                 /* (the strokes' cleanup) */
    int64_t due = q.c.ink_t + INK_DEBT_PAUSE_MS;
    t = pol_timeout(&q, now, 0, 0);
    CHECK(q.ink_debt.n && q.ink_debt_cleans == 0 && t > 0 && t <= due - now + COALESCE_MS,
          "ink timeout: a pending ink debt (%d boxes) gets pol_timeout %d (due in %ld ms)", q.ink_debt.n, t, (long)(due - now));
    if (t > 0) { now += t; o.n = 0; pol_step(&q, now, 0, 0, 0, &o); }
    CHECK(q.ink_debt_cleans == 1, "ink timeout: stepping at the timeout ran %lu ink debt cleans (want 1)", q.ink_debt_cleans);
    free(j);
}

static void scenario_page_turn(void)
{
    /* KOReader (auto profile) turns a page: whole window changes, no GC16 in
     * flight -> one flash, no DU, nothing left to clean */
    policy p; pol_init(&p, W, H);
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 3000, j);
    j->n = 0;
    box win = { 0, 0, W, H };
    for (int i = 0; i < 3; i++) {
        burst(&p, &now, one((box){ 20, 20, W - 20, H - 20 }), win, PM_AUTO, j);
        idle_until(&p, &now, now + 3000, j);
    }
    CHECK(count(j, WF_GC16, M_AREA) == 3, "page turn: %d flashes for 3 turns", count(j, WF_GC16, M_AREA));
    CHECK(count(j, WF_DU, M_PART) == 0, "page turn: DU used");
    CHECK(p.state == S_IDLE, "page turn: ends %s", state_name(p.state));
    free(j);
}

/* pages= (KOReader): page changes are GC16 partials, every Nth a
 * flash; a GC16 in flight (a menu's highlight) starts no run; a measured
 * shift (moving) still runs */
static void scenario_pages(void)
{
    policy p; pol_init(&p, W, H);
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 3000, j);
    j->n = 0;
    box win = { 0, 0, W, H };
    burst_pages = PAGES_FLASH;
    for (int i = 0; i < 2 * PAGES_FLASH; i++) {
        burst(&p, &now, one((box){ 20, 20, W - 20, H - 20 }), win, PM_AUTO, j);
        idle_until(&p, &now, now + 3000, j);
    }
    CHECK(count(j, WF_GC16, M_AREA) == 2, "pages: %d flashes for %d turns (want 2)", count(j, WF_GC16, M_AREA), 2 * PAGES_FLASH);
    CHECK(count(j, WF_GC16, M_PART) == 2 * PAGES_FLASH - 2, "pages: %d partials (want %d)", count(j, WF_GC16, M_PART), 2 * PAGES_FLASH - 2);
    CHECK(count(j, WF_DU, M_PART) == 0, "pages: DU used");
    /* a menu tap: the row's GC16, the menu 40 ms later while it is in flight */
    j->n = 0;
    burst(&p, &now, one((box){ 0, 400, W, 490 }), win, PM_AUTO, j);
    now += 40;
    burst(&p, &now, one((box){ 0, 0, W, 1000 }), win, PM_AUTO, j);
    CHECK(p.state != S_RUN && count(j, WF_DU, M_PART) == 0, "pages: menu tap became a run (%s)", state_name(p.state));
    idle_until(&p, &now, now + 3000, j);
    /* a pan (a measured shift) runs as before */
    j->n = 0;
    burst_moving = 1;
    burst(&p, &now, one((box){ 0, 1000, W, 1800 }), win, PM_AUTO, j);
    CHECK(p.state == S_RUN && count(j, WF_DU, M_PART) == 1, "pages: a shifted burst was no run (%s)", state_name(p.state));
    burst_moving = 0;
    burst_pages = 0;
    free(j);
}

static void scenario_typing(void)
{
    policy p; pol_init(&p, W, H);
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 3000, j);
    j->n = 0;
    box term = { 0, 32, W, 977 };
    for (int i = 0; i < 10; i++) {
        burst(&p, &now, one((box){ 40 + i * 20, 500, 60 + i * 20, 530 }), term, PM_AUTO, j);
        now += 150;
    }
    CHECK(p.state == S_CLEANUP, "typing: expected cleanup pending, is %s", state_name(p.state));
    idle_until(&p, &now, now + 1000, j);
    CHECK(count(j, WF_DU, M_PART) == 10, "typing: %d DUs", count(j, WF_DU, M_PART));
    CHECK(count(j, WF_GC16, M_PART) >= 1, "typing: no cleanup");
    CHECK(count(j, WF_GC16, M_AREA) + count(j, WF_GC16, M_SCREEN) == 0, "typing: flashed");
    CHECK(p.state == S_IDLE, "typing: ends %s", state_name(p.state));
    free(j);
}

/* calm (the shell's bar and popups): a tap's highlight under a
 * finger is no press — with curve=off + still=range (the device's config: a press
 * = the fast end, the grey smudge saw)
 * an uncut thresholded DU, its GC16 partial later, never a run or a flash;
 * a large shell change a GC16 partial, no flash; a moving finger no run */
static void scenario_calm(void)
{
    policy p; pol_init(&p, W, H);
    p.has_engine = 1;
    pol_config(&p, "mmin=gc16 attack=jump motion=adaptive mmax=a2_2 still=range curve=off");   /* the device's eink.conf */
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 3000, j);
    box bar = { 0, 0, W, 104 };
    burst_calm = 1;
    j->n = 0;
    burst(&p, &now, one((box){ 116, 16, 340, 88 }), bar, PM_AUTO, j);
    int cut = -1;
    for (int i = 0; i < j->n; i++)
        for (int k = 0; k < j->log[i].n; k++)
            if (j->log[i].a[k].kind == A_SEND && j->log[i].a[k].wf == WF_DU) cut = j->log[i].a[k].n;
    CHECK(count(j, WF_DU, M_PART) == 1 && cut == 0, "calm tap: not one uncut DU (DUs %d, cut %d)", count(j, WF_DU, M_PART), cut);
    CHECK(p.state != S_RUN, "calm tap: a run");
    CHECK(burst_how == C_BW, "calm tap: commit %d, not C_BW (black/white only, greys at the GC16: never dithered)", burst_how);
    idle_until(&p, &now, now + 3000, j);
    CHECK(count(j, WF_GC16, M_PART) >= 1, "calm tap: no GC16 cleanup");
    CHECK(count(j, WF_GC16, M_AREA) + count(j, WF_GC16, M_SCREEN) == 0, "calm tap: flashed");
    /* the same tap without calm: a press = the cut floor (what saw) */
    burst_calm = 0;
    j->n = 0;
    burst(&p, &now, one((box){ 116, 16, 340, 88 }), bar, PM_PRESS, j);
    cut = -1;
    for (int i = 0; i < j->n; i++)
        for (int k = 0; k < j->log[i].n; k++)
            if (j->log[i].a[k].kind == A_SEND && j->log[i].a[k].wf == WF_DU) cut = j->log[i].a[k].n;
    CHECK(cut > 0, "no calm: the press is not the cut floor (cut %d) - this test's premise", cut);
    idle_until(&p, &now, now + 3000, j);
    /* a large change in a popup: a GC16 partial, no flash */
    burst_calm = 1;
    j->n = 0;
    box pop = { 24, 120, W - 24, 1800 };
    burst(&p, &now, one((box){ 48, 200, W - 48, 1700 }), pop, PM_AUTO, j);
    CHECK(count(j, WF_GC16, M_PART) == 1 && count(j, WF_GC16, M_AREA) + count(j, WF_GC16, M_SCREEN) == 0,
          "calm large: not one GC16 partial (part %d, area %d)", count(j, WF_GC16, M_PART), count(j, WF_GC16, M_AREA));
    CHECK(count(j, WF_DU, M_PART) == 0, "calm large: a DU");
    idle_until(&p, &now, now + 3000, j);
    /* a swipe in the overview (finger moving): still no run */
    burst_moving = 1;
    j->n = 0;
    for (int i = 0; i < 6; i++) { burst(&p, &now, one((box){ 48, 200 + i * 10, W - 48, 600 + i * 10 }), pop, PM_AUTO, j); now += 40; }
    CHECK(p.state != S_RUN, "calm swipe: a run (%s)", state_name(p.state));
    burst_moving = 0;
    burst_calm = 0;
    free(j);
}

/* finger down on a large repaint (scroll start): DU at once, no GC16, no run;
 * the tap's result after release (auto) is a normal flash */
static void scenario_press(void)
{
    policy p; pol_init(&p, W, H);
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 3000, j);
    j->n = 0;
    box ff = { 0, 977, W, H };
    burst(&p, &now, one((box){ 0, 1000, W, 1800 }), ff, PM_PRESS, j);
    CHECK(count(j, WF_DU, M_PART) == 1 && count(j, WF_GC16, M_PART) + count(j, WF_GC16, M_AREA) == 0,
          "press: first large repaint not a single DU");
    CHECK(p.state == S_CLEANUP, "press: started a run (%s)", state_name(p.state));
    now += 100;
    j->n = 0;
    burst(&p, &now, one((box){ 0, 1000, W, 1800 }), ff, PM_AUTO, j);
    CHECK(count(j, WF_GC16, M_AREA) == 1, "press: tap result not a flash");
    free(j);
    /* a cut floor (fixed A2/2): the press paints the run's own cut (an uncut A2
     * held the first run steps behind it); a tap's cleanup stays a partial */
    pol_init(&p, W, H);
    p.has_engine = 1;
    pol_config(&p, "motion=adaptive mmin=a2_2 mmax=a2_2");
    now = 1000;
    j = calloc(1, sizeof *j);
    idle_until(&p, &now, 3000, j);
    j->n = 0;
    burst(&p, &now, one((box){ 0, 1000, W, 1800 }), ff, PM_PRESS, j);
    int cutn = -1;
    for (int i = 0; i < j->n; i++)
        for (int k = 0; k < j->log[i].n; k++)
            if (j->log[i].a[k].kind == A_SEND && j->log[i].a[k].wf == WF_DU) cutn = j->log[i].a[k].n;
    CHECK(cutn == p.a2len[3], "press fixed A2/2: DU cut %d, not A2/2's %d", cutn, p.a2len[3]);
    CHECK(p.state == S_CLEANUP, "press fixed A2/2: started a run (%s)", state_name(p.state));
    j->n = 0;
    idle_until(&p, &now, now + 3000, j);
    CHECK(count(j, WF_GC16, M_PART) >= 1 && count(j, WF_GC16, M_AREA) + count(j, WF_GC16, M_SCREEN) == 0,
          "press fixed A2/2: cleanup not a partial");
    free(j);
}

/* epd-motion's estimate: a moving burst is a run whatever the profile's
 * touch mode (the first shifted frame of a scroll, a wheel scroll), a still one
 * of the same size is a repaint; the estimate's speed picks the adaptive step */
static void scenario_moving(void)
{
    policy p; pol_init(&p, W, H);
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    box ff = { 0, 977, W, H };
    idle_until(&p, &now, 3000, j);
    j->n = 0;
    burst_moving = 1;
    burst(&p, &now, one((box){ 0, 1000, W, 1800 }), ff, PM_AUTO, j);
    CHECK(p.state == S_RUN && count(j, WF_DU, M_PART) == 1 && count(j, WF_GC16, M_AREA) == 0,
          "moving: a moving burst is not a run (%s)", state_name(p.state));
    burst_moving = 0;
    idle_until(&p, &now, now + 4000, j);
    j->n = 0;
    burst(&p, &now, one((box){ 0, 1000, W, 1800 }), ff, PM_AUTO, j);
    CHECK(p.state != S_RUN && count(j, WF_GC16, M_AREA) == 1, "moving: a still repaint became a run (%s)", state_name(p.state));
    burst(&p, &now, one((box){ 0, 1000, W, 1800 }), ff, PM_QUALITY, j);
    free(j);
    /* quality apps stay quality */
    pol_init(&p, W, H);
    now = 1000;
    j = calloc(1, sizeof *j);
    idle_until(&p, &now, 3000, j);
    j->n = 0;
    burst_moving = 1;
    burst(&p, &now, one((box){ 0, 1000, W, 1800 }), ff, PM_QUALITY, j);
    CHECK(p.state != S_RUN, "moving: a quality app ran");
    burst_moving = 0;
    free(j);
    /* the speed: slow content -> GC16 steps, fast -> a cut A2 */
    for (int fast = 0; fast < 2; fast++) {
        pol_init(&p, W, H);
        p.has_engine = 1;
        pol_config(&p, "motion=adaptive");
        now = 1000;
        j = calloc(1, sizeof *j);
        idle_until(&p, &now, 3000, j);
        burst_moving = 1; burst_speed = fast ? 3.0 : 0.05;
        for (int i = 0; i < 6; i++) { j->n = 0; burst(&p, &now, one((box){ 0, 1000, W, 1800 }), ff, PM_AUTO, j); now += 150; }   /* past run_ms */
        int cutn = -1, gc = count(j, WF_GC16, M_PART);
        for (int i = 0; i < j->n; i++)
            for (int k = 0; k < j->log[i].n; k++)
                if (j->log[i].a[k].kind == A_SEND && j->log[i].a[k].wf == WF_DU) cutn = j->log[i].a[k].n;
        if (fast) CHECK(cutn > 0, "moving fast: last step not a cut A2 (%d)", cutn);   /* attack=fused ramps up */
        else CHECK(gc == 1 && cutn < 0, "moving slow: last step not GC16 (gc %d cut %d)", gc, cutn);
        burst_moving = 0; burst_speed = 0;
        free(j);
    }
}

/* a ticker (one small line changing every 100 ms) under a scrolled page: the
 * scroll's DU wait for their cleanup; the ticker must not hold it back
 * (clean_motion), and with clean_motion=0 it does (the old rule) */
static int ticker_run_at(const char *cfg, int period, box tk)
{
    policy p; pol_init(&p, W, H);
    if (cfg) pol_config(&p, cfg);
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    box ff = { 0, 110, W, H };
    idle_until(&p, &now, 3000, j);
    burst_moving = 1;
    for (int i = 0; i < 8; i++) { burst(&p, &now, one((box){ 0, 200, W, 1700 }), ff, PM_AUTO, j); now += 120; }
    burst_moving = 0;
    j->n = 0;
    for (int64_t end = now + 3000; now < end;) {    /* 3 s of the ticker */
        burst(&p, &now, one(tk), ff, PM_AUTO, j);
        idle_until(&p, &now, now + period - COALESCE_MS, j);
    }
    int gc = count(j, WF_GC16, M_PART) + count(j, WF_GC16, M_AREA) + count(j, WF_GC16, M_SCREEN);
    int dirty_left = p.dirty.n;
    long big_left = 0;
    for (int i = 0; i < p.dirty.n; i++) if (box_area(p.dirty.b[i]) > 1000000) big_left++;   /* the page's */
    free(j);
    (void)dirty_left;
    return gc * 100 + (int)big_left;
}
static int ticker_run(const char *cfg) { return ticker_run_at(cfg, 100, (box){ 0, 1740, W, 1770 }); }
static void scenario_ticker(void)
{
    /* faster than the old ring (16 bursts) covered in one wait: the area counts */
    int r = ticker_run_at(NULL, 30, (box){ 0, 1740, W, 1770 });
    CHECK(r / 100 >= 1 && r % 100 == 0, "ticker every 30 ms: the cleanup held back (%d GC16, %d big dirty left)", r / 100, r % 100);
    /* larger than the old 5 % cap (a video, ~11 %), still small against the page */
    r = ticker_run_at(NULL, 40, (box){ 300, 1500, 1100, 1870 });
    CHECK(r / 100 >= 1 && r % 100 == 0, "video-sized motion: the cleanup held back (%d GC16, %d big dirty left)", r / 100, r % 100);
    /* two small tickers far apart: their union is small, their bounding box isn't */
    {
        policy p; pol_init(&p, W, H);
        int64_t now = 1000;
        journal *j = calloc(1, sizeof *j);
        box ff = { 0, 110, W, H };
        idle_until(&p, &now, 3000, j);
        burst_moving = 1;
        for (int i = 0; i < 8; i++) { burst(&p, &now, one((box){ 0, 200, W, 1700 }), ff, PM_AUTO, j); now += 120; }
        burst_moving = 0;
        j->n = 0;
        for (int64_t end = now + 3000; now < end;) {
            boxlist l = {0};
            l.b[l.n++] = (box){ 0, 120, 200, 150 };
            l.b[l.n++] = (box){ 1200, 1840, W, 1870 };
            burst(&p, &now, l, ff, PM_AUTO, j);
            idle_until(&p, &now, now + 100 - COALESCE_MS, j);
        }
        CHECK(p.early_cleans >= 1, "two tickers apart: no early cleanup");
        free(j);
    }
    r = ticker_run(NULL);
    CHECK(r / 100 >= 1 && r % 100 == 0, "ticker: the scroll's cleanup held back (%d GC16, %d big dirty left)", r / 100, r % 100);
    r = ticker_run("clean_motion=0");
    CHECK(r / 100 == 0, "ticker, clean_motion=0: %d GC16 (the old rule should wait)", r / 100);
    /* a slow scroll is no ticker: the whole window changes, no early cleanup */
    policy p; pol_init(&p, W, H);
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    box ff = { 0, 110, W, H };
    idle_until(&p, &now, 3000, j);
    burst_moving = 1;
    for (int i = 0; i < 4; i++) { burst(&p, &now, one((box){ 0, 200, W, 1700 }), ff, PM_AUTO, j); now += 120; }
    burst_moving = 0;
    j->n = 0;
    for (int i = 0; i < 20; i++) { burst(&p, &now, one((box){ 0, 200, W, 1700 }), ff, PM_AUTO, j); idle_until(&p, &now, now + 140, j); }
    CHECK(p.early_cleans == 0, "slow scroll: %lu early cleanups", p.early_cleans);
    free(j);
}

/* still=range: a still repaint paints at the range's low end (an app forced to
 * A2); the low end == the high end: one mode for everything, no cleanups */
static int sends(const journal *j, int wf) { int n = 0; for (int m = 0; m < 4; m++) n += count(j, wf, m); return n; }
static void scenario_still_range(void)
{
    box ff = { 0, 110, W, H };
    /* still=auto (default): a medium repaint is a GC16 */
    policy p; pol_init(&p, W, H);
    p.has_engine = 1;
    pol_config(&p, "motion=adaptive mmin=a2 mmax=a2_2");
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 3000, j);
    j->n = 0;
    burst(&p, &now, one((box){ 0, 300, W, 900 }), ff, PM_AUTO, j);
    CHECK(sends(j, WF_GC16) == 1 && sends(j, WF_DU) == 0, "still=auto: medium repaint not a GC16");
    free(j);
    /* still=range, low end A2: an A2, its partial GC16 cleanup after the quiet */
    pol_init(&p, W, H);
    p.has_engine = 1;
    pol_config(&p, "motion=adaptive mmin=a2 mmax=a2_2 still=range");
    now = 1000;
    j = calloc(1, sizeof *j);
    idle_until(&p, &now, 3000, j);
    j->n = 0;
    burst(&p, &now, one((box){ 0, 300, W, 900 }), ff, PM_AUTO, j);
    burst(&p, &now, one((box){ 0, 110, W, H }), ff, PM_AUTO, j);   /* even a whole-window repaint */
    CHECK(sends(j, WF_GC16) == 0 && sends(j, WF_DU) == 2, "still=range a2: GC16 %d DU %d", sends(j, WF_GC16), sends(j, WF_DU));
    j->n = 0;
    idle_until(&p, &now, now + 3000, j);
    CHECK(count(j, WF_GC16, M_PART) >= 1 && count(j, WF_GC16, M_AREA) == 0, "still=range a2: cleanup not a partial");
    free(j);
    /* low end DU4 */
    pol_init(&p, W, H);
    p.has_engine = 1;
    pol_config(&p, "motion=adaptive mmin=du4 mmax=a2_2 still=range");
    now = 1000;
    j = calloc(1, sizeof *j);
    idle_until(&p, &now, 3000, j);
    j->n = 0;
    burst(&p, &now, one((box){ 0, 300, W, 900 }), ff, PM_AUTO, j);
    CHECK(sends(j, WF_DU4) == 1 && sends(j, WF_GC16) == 0, "still=range du4: not a DU4");
    free(j);
    /* one mode (a2_4 .. a2_4): stills, presses, runs all A2/4, no GC16 at all */
    pol_init(&p, W, H);
    p.has_engine = 1;
    pol_config(&p, "motion=adaptive mmin=a2_4 mmax=a2_4 still=range");
    now = 1000;
    j = calloc(1, sizeof *j);
    idle_until(&p, &now, 3000, j);
    j->n = 0;
    burst(&p, &now, one((box){ 0, 300, W, 900 }), ff, PM_AUTO, j);
    burst(&p, &now, one((box){ 0, 1000, W, 1800 }), ff, PM_PRESS, j);
    burst_moving = 1;
    for (int i = 0; i < 6; i++) { burst(&p, &now, one((box){ 0, 200, W, 1700 }), ff, PM_AUTO, j); now += 60; }
    burst_moving = 0;
    burst(&p, &now, one((box){ 0, 110, W, H }), ff, PM_AUTO, j);
    idle_until(&p, &now, now + 4000, j);
    int bad = 0;
    for (int i = 0; i < j->n; i++)
        for (int k = 0; k < j->log[i].n; k++)
            if (j->log[i].a[k].kind == A_SEND && (j->log[i].a[k].wf != WF_DU || j->log[i].a[k].n != p.a2len[2])) bad++;
    CHECK(!bad && sends(j, WF_DU) >= 6, "one mode a2_4: %d paints not A2/4 (%d DU)", bad, sends(j, WF_DU));
    CHECK(p.dirty.n == 0 && p.state == S_IDLE, "one mode: cleanup not dropped (%d dirty, %s)", p.dirty.n, state_name(p.state));
    free(j);
    /* curve=off, gc16 .. a2_2: a still is GC16, a press A2/2 (motion), a slow drag A2/2 */
    pol_init(&p, W, H);
    p.has_engine = 1;
    pol_config(&p, "motion=adaptive mmin=gc16 mmax=a2_2 still=range curve=off");
    now = 1000;
    j = calloc(1, sizeof *j);
    idle_until(&p, &now, 3000, j);
    j->n = 0;
    burst(&p, &now, one((box){ 0, 300, W, 900 }), ff, PM_AUTO, j);
    CHECK(sends(j, WF_GC16) == 1 && sends(j, WF_DU) == 0, "curve=off still: GC16 %d DU %d", sends(j, WF_GC16), sends(j, WF_DU));
    j->n = 0;
    burst(&p, &now, one((box){ 0, 1000, W, 1800 }), ff, PM_PRESS, j);
    burst_moving = 1; burst_speed = 0.05;
    for (int i = 0; i < 6; i++) { burst(&p, &now, one((box){ 0, 200, W, 1700 }), ff, PM_AUTO, j); now += 60; }
    burst_moving = 0; burst_speed = 0;
    bad = 0;
    int du = 0;
    for (int i = 0; i < j->n; i++)
        for (int k = 0; k < j->log[i].n; k++)
            if (j->log[i].a[k].kind == A_SEND) { du += j->log[i].a[k].wf == WF_DU; bad += j->log[i].a[k].wf != WF_DU || j->log[i].a[k].n != p.a2len[3]; }
    CHECK(!bad && du >= 3, "curve=off motion: %d paints not A2/2 (%d DU)", bad, du);
    free(j);
    /* one mode GC16: small changes are GC16 partials too, never a DU */
    pol_init(&p, W, H);
    p.has_engine = 1;
    pol_config(&p, "motion=adaptive mmin=gc16 mmax=gc16 still=range");
    now = 1000;
    j = calloc(1, sizeof *j);
    idle_until(&p, &now, 3000, j);
    j->n = 0;
    burst(&p, &now, one((box){ 100, 300, 200, 330 }), ff, PM_AUTO, j);
    burst(&p, &now, one((box){ 0, 1000, W, 1800 }), ff, PM_PRESS, j);
    idle_until(&p, &now, now + 3000, j);
    CHECK(sends(j, WF_DU) + sends(j, WF_DU4) == 0 && count(j, WF_GC16, M_PART) == 2, "one mode gc16: DU %d GC16 partial %d",
          sends(j, WF_DU) + sends(j, WF_DU4), count(j, WF_GC16, M_PART));
    free(j);
}

static void scenario_sleep(void)
{
    policy p; pol_init(&p, W, H);
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 2000, j);
    actions o = {0};
    pol_cmd(&p, "sleep", now, &o);
    CHECK(p.state == S_ASLEEP && o.n == 3 && o.a[0].kind == A_GRAB_ALL && o.a[1].kind == A_BANNER &&
          o.a[2].kind == A_SEND && o.a[2].mode == M_SCREEN, "sleep: page grabbed, banner, full refresh");
    o.n = 0;
    CHECK(pol_step(&p, now + 100, 0, 3, now, &o) == STEP_DROP && !o.n, "asleep: damage dropped");
    pol_ink(&p, (box){ 1, 1, 9, 9 }, now, &o);
    {   /* pol_inked: the same state as pol_ink, no action */
        policy q1, q2;
        actions oa = {0};
        pol_init(&q1, 1404, 1872); pol_init(&q2, 1404, 1872);
        pol_ink(&q1, (box){ 50, 50, 90, 70 }, 1000, &oa);
        pol_inked(&q2, (box){ 50, 50, 90, 70 }, 1000);
        CHECK(oa.n == 1 && q1.inked.n == 1 && q2.inked.n == 1 && q1.state == q2.state &&
              q1.c.ink_t == q2.c.ink_t && q1.last_du == q2.last_du && !memcmp(q1.ghost, q2.ghost, sizeof q1.ghost),
              "pol_inked differs from pol_ink");
    }
    CHECK(!o.n, "asleep: no ink");
    CHECK(pol_cmd(&p, "wake", now, &o) == 1 && p.state == S_IDLE, "wake");
    free(j);
}

/* a terminal repainting on its own every 700 ms (htop, a clock): after
 * quiet_after ms without input the repaints collect into one update per
 * quiet_ms; a touch lifts that at once */
static void scenario_quiet(void)
{
    policy p; pol_init(&p, W, H);
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 2000, j);
    pol_input(&p, now);
    int64_t start = now, first = 0;
    int pend = 0, bursts_active = 0, bursts_quiet = 0, max_wait_touch = -1;
    int64_t touch_at = start + 30000, touch_dmg = 0;
    while (now < start + 40000) {
        now += 10;
        int repaint = (now - start) % 700 == 0;
        if (now == touch_at) { pol_input(&p, now); touch_dmg = now; repaint = 1; }
        if (repaint) { if (!pend) first = now; pend++; pol_damage(&p, now); }
        actions o = {0};
        int st = pol_step(&p, now, 0, pend, first, &o);
        jadd(j, &o);
        if (st == STEP_BURST) {
            actions b = {0};
            burst_in in = { now, one((box){ 0, 100, W, 900 }), (box){ 0, 32, W, 977 }, PM_AUTO, 0, 0, BUSY_AUTO, 0, 0 };
            pol_burst(&p, &in, &b);
            jadd(j, &b);
            if (now - start < p.quiet_after) bursts_active++;
            else if (now < touch_at) bursts_quiet++;
            if (touch_dmg && now >= touch_dmg && max_wait_touch < 0) max_wait_touch = (int)(now - touch_dmg);
        }
        if (st != STEP_NONE) pend = 0;
        if (pend) {
            int t = pol_timeout(&p, now, 0, pend);
            CHECK(t >= COALESCE_MS && t <= p.quiet_ms, "quiet: timeout %d", t);
        }
    }
    /* 5 s active: every repaint (~7); then 25 s quiet: one per 2 s (~12, not ~36) */
    CHECK(bursts_active >= 6 && bursts_active <= 8, "quiet: %d bursts while active (want every repaint)", bursts_active);
    CHECK(bursts_quiet >= 11 && bursts_quiet <= 13, "quiet: %d bursts in 25 s idle (want ~1 per %d ms)", bursts_quiet, p.quiet_ms);
    CHECK(max_wait_touch >= 0 && max_wait_touch <= COALESCE_MS + 10, "quiet: repaint after a touch waited %d ms", max_wait_touch);
    free(j);
}

/* a video playing with nobody touching (choppy ~5 s after the
 * last touch, smooth while a finger stayed down): frames every gap_ms over
 * vid, the last input at the start. Returns the GC16s while it plays (after
 * the first 2 s), *end_ms = how long after the last frame the cleanup came
 * (-1: none within 3 s). cfg: pol_config line; app: the profile's busy= */
static int vr_late;     /* video_run: bursts 10-18 s in (quiet_after long past) */
static int video_run(const char *cfg, int app, int gap_ms, box vid, int *end_ms)
{
    policy p; pol_init(&p, W, H);
    pol_config(&p, "quiet_ms=0");       /* the engine's setting (glassine-epdd-desktop) */
    if (cfg) pol_config(&p, cfg);
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 2000, j);
    pol_input(&p, now);
    int64_t start = now, first = 0, stop = start + 20000, last_frame = 0;
    int pend = 0, gc_play = 0;
    *end_ms = -1; vr_late = 0;
    while (now < stop + 3000) {
        now += 10;
        if (now < stop && (now - start) % gap_ms == 0) {
            if (!pend) first = now;
            pend++; pol_damage(&p, now); last_frame = now;
        }
        actions o = {0};
        int st = pol_step(&p, now, 0, pend, first, &o);
        if (st == STEP_BURST) {
            burst_in in = { now, one(vid), (box){ 0, 32, W, 977 }, PM_AUTO, 0, 0, app, 0, 0 };
            pol_burst(&p, &in, &o);
            if (now - start >= 10000 && now - start < 18000) vr_late++;
        }
        if (st != STEP_NONE) pend = 0;
        for (int k = 0; k < o.n; k++)
            if (o.a[k].kind == A_SEND && o.a[k].wf == WF_GC16) {
                if (now < stop && now - start >= 2000) gc_play++;
                else if (now >= stop && *end_ms < 0) *end_ms = (int)(now - last_frame);
            }
    }
    free(j);
    return gc_play;
}

static void scenario_busy(void)
{
    box vid = { 100, 300, 1300, 975 };          /* a 16:9 video, ~31 % of the screen */
    box small = { 600, 40, 800, 72 };           /* a clock, ~0.2 % */
    int end, n;
    /* busy off (before): a frame gap > idle_ms with no input ends the run at
     * every frame, each with a GC16 (the choppiness) */
    n = video_run("busy=0", BUSY_AUTO, 300, vid, &end);
    CHECK(n >= 20, "busy=0: video at 300 ms gaps had %d GC16s in 18 s (the old behaviour: many)", n);
    /* detected: one run for the whole video, its cleanup once it stops */
    n = video_run(NULL, BUSY_AUTO, 300, vid, &end);
    CHECK(n == 0, "busy: %d GC16s while the video played (want none)", n);
    CHECK(end >= 0 && end <= 1300, "busy: cleanup %d ms after the last frame (want <= run_wait_max + a bit)", end);
    n = video_run(NULL, BUSY_AUTO, 40, vid, &end);    /* 25 fps */
    CHECK(n == 0, "busy: %d GC16s while a 25 fps video played", n);
    int full = vr_late;
    /* the device's eink.conf has @quiet_ms=2000: busy content is not "quiet",
     * its frames keep their rate long after the last touch */
    n = video_run("quiet_ms=2000", BUSY_AUTO, 40, vid, &end);
    CHECK(n == 0, "busy, quiet_ms=2000: %d GC16s while a 25 fps video played", n);
    CHECK(vr_late >= full * 9 / 10 && vr_late > 20, "busy, quiet_ms=2000: %d bursts 10-18 s in (quiet_ms=0: %d; want the same)", vr_late, full);
    /* the app opted out: as before */
    n = video_run(NULL, BUSY_OFF, 300, vid, &end);
    CHECK(n >= 20, "busy=off app: %d GC16s (want the old behaviour)", n);
    /* not busy: a clock ticking fast but tiny; a big repaint only every 700 ms */
    n = video_run(NULL, BUSY_AUTO, 300, small, &end);
    CHECK(n >= 20, "busy: a tiny 300 ms repaint counted as busy (%d GC16s)", n);
    n = video_run(NULL, BUSY_AUTO, 700, vid, &end);
    CHECK(n >= 15, "busy: a big repaint every 700 ms counted as busy (%d GC16s)", n);
    /* busy=on: the app's repaints are motion whatever their rate or size */
    n = video_run(NULL, BUSY_ON, 700, vid, &end);
    CHECK(n == 0, "busy=on app: %d GC16s at 700 ms gaps (want none)", n);
}

/* an animation under engine pacing (an OpenMW sample: a moving
 * sky in a 702x884 window, nobody touching): the app draws every 14 ms, but in
 * a run epdd answers sway's frame only when the engine can take the next burst
 * (ahead=predict), so no damage comes while the engine drives a burst. A burst
 * keeps the engine busy eng_ms. Old: the run ended idle_ms after its first
 * burst whenever eng_ms > idle_ms, then a GC16 per frame through the run tail.
 * Returns the GC16s while it plays (after the first 2 s); *runs = run starts;
 * *end_ms = the cleanup's GC16 after the last frame (-1: none within 3 s) */
static int pa_always;    /* paced_anim: the app's motion=always */
static int paced_anim(const char *cfg, int eng_ms, int *runs, int *end_ms)
{
    policy p; pol_init(&p, W, H);
    pol_config(&p, "quiet_ms=0");
    if (cfg) pol_config(&p, cfg);
    p.has_engine = 1;
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 2000, j);
    pol_input(&p, now);
    box win = { 100, 300, 802, 1184 };
    int64_t start = now, stop = start + 20000, free_at = 0, first = 0;
    int64_t last_frame = 0;
    int pend = 0, gc = 0, was_run = 0;
    *runs = 0; *end_ms = -1;
    while (now < stop + 3000) {
        now += 2;
        int ew = now < free_at ? (int)(free_at - now) : 0;
        int in_run = p.state == S_RUN;
        /* the app's frame: drawn every 14 ms, its damage seen at once outside
         * runs, in a run only once the engine is free (the held DONE) */
        if (now < stop && (now - start) % 14 == 0 && (!in_run || ew == 0) && !pend) { first = now; pend = 1; pol_damage(&p, now); last_frame = now; }
        pol_engine(&p, in_run ? ew : -1);
        actions o = {0};
        int st = pol_step(&p, now, 0, pend, first, &o);
        if (st == STEP_BURST) {
            burst_in in = { now, one(win), (box){ 0, 0, W, H }, PM_AUTO, 0, 0, BUSY_AUTO, 0, pa_always };
            pol_burst(&p, &in, &o);
            free_at = now + eng_ms;
        }
        if (st != STEP_NONE) pend = 0;
        if (p.state == S_RUN && !was_run) (*runs)++;
        was_run = p.state == S_RUN;
        for (int k = 0; k < o.n; k++)
            if (o.a[k].kind == A_SEND && o.a[k].wf == WF_GC16) {
                if (now < stop && now - start >= 2000) gc++;
                else if (now >= stop && *end_ms < 0) *end_ms = (int)(now - last_frame);
            }
    }
    free(j);
    return gc;
}
static void scenario_paced_anim(void)
{
    int runs, n, end;
    n = paced_anim(NULL, 470, &runs, &end);
    CHECK(n == 0 && runs == 1, "paced animation (engine 470 ms > idle_ms): %d GC16s, %d runs in 20 s (want 0, 1)", n, runs);
    CHECK(end >= 0 && end <= 470 + 1300, "paced animation (engine 470 ms): cleanup %d ms after the last frame", end);
    n = paced_anim(NULL, 40, &runs, &end);
    CHECK(n == 0 && runs == 1, "paced animation (engine 40 ms): %d GC16s, %d runs (want 0, 1)", n, runs);
    CHECK(end >= 0 && end <= 40 + 1300, "paced animation (engine 40 ms): cleanup %d ms after the last frame", end);
    if (getenv("PACED_LOG")) printf("paced animation: cleanup %d ms after the last frame (engine 40 ms)\n", end);
    /* a motion=always game ("always in motion not reliable":
     * the same hold ended its runs, a cleanup GC16 mid-play) */
    pa_always = 1;
    n = paced_anim(NULL, 470, &runs, &end);
    CHECK(n == 0 && runs == 1, "paced always-in-motion app (engine 470 ms): %d GC16s, %d runs in 20 s (want 0, 1)", n, runs);
    CHECK(end >= 0 && end <= 470 + 1300, "paced always-in-motion app: cleanup %d ms after the last frame", end);
    pa_always = 0;
}

/* a video that starts long after the last touch (FreeTube: one
 * GC16 per 2 s until a finger came): quiet_ms=2000 paced the reads to one per
 * 2 s, too few for busy_seen. Frames every gap_ms over b from 8 s after the
 * last input for 12 s; returns the bursts in its last 6 s, *gc = GC16s then */
static int late_video(int gap_ms, box b, int *gc)
{
    policy p; pol_init(&p, W, H);
    pol_config(&p, "quiet_ms=2000 quiet_after=5000");
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 2000, j);
    pol_input(&p, now);
    int64_t start = now + 8000, stop = start + 12000, first = 0;
    int pend = 0, late = 0;
    *gc = 0;
    while (now < stop) {
        now += 1;
        if (now >= start && (now - start) % gap_ms == 0) { if (!pend) first = now; pend++; pol_damage(&p, now); }
        actions o = {0};
        int st = pol_step(&p, now, 0, pend, first, &o);
        if (st == STEP_BURST) {
            burst_in in = { now, one(b), (box){ 0, 0, W, H }, PM_AUTO, 0, 0, BUSY_AUTO, 0, 0 };
            pol_burst(&p, &in, &o);
            if (now - start >= 6000) late++;
        }
        if (st != STEP_NONE) pend = 0;
        for (int k = 0; k < o.n; k++)
            if (o.a[k].kind == A_SEND && o.a[k].wf == WF_GC16 && now - start >= 6000) (*gc)++;
    }
    free(j);
    return late;
}

static void scenario_late_video(void)
{
    box vid = { 739, 702, 1133, 1404 };         /* the harness's mpv window, ~10 % of the screen */
    box clock = { 600, 40, 800, 72 };
    int gc, n;
    n = late_video(40, vid, &gc);               /* 25 fps */
    CHECK(n >= 50 && gc == 0, "late video 25 fps: %d bursts in 6 s, %d GC16 (want a run: ~60 at run_ms, none)", n, gc);
    n = late_video(150, vid, &gc);              /* ~7 fps */
    CHECK(n >= 30 && gc == 0, "late video 7 fps: %d bursts in 6 s, %d GC16 (want ~40, none)", n, gc);
    /* still quiet: a clock at 1/s, a small fast repaint (a spinner) at 25/s, a big repaint every 1.5 s */
    n = late_video(1000, clock, &gc);
    CHECK(n <= 4, "late clock: %d bursts in 6 s (want quiet_ms pacing: <= 4)", n);
    n = late_video(40, clock, &gc);
    CHECK(n <= 4, "late spinner: %d bursts in 6 s (want quiet_ms pacing: <= 4)", n);
    n = late_video(1500, vid, &gc);
    CHECK(n <= 4, "late slow big repaint: %d bursts in 6 s (want quiet_ms pacing: <= 4)", n);
}

/* a game (OpenMW) with nobody touching: a busy scene (half the screen every
 * 100 ms) for 2 s, then a slow one (a 160 px figure moving, 150 ms apart) for
 * 6 s, then nothing. The device's eink.conf (Fluid: mmin=gc16 mmax=a2_2
 * curve=off still=range). Returns the GC16s while the game draws; *cut2 = run
 * bursts sent as A2/2, *other = DUs at any other cut, *end_ms = the cleanup
 * after the last frame (-1: none) */
static int game_run(int always, const char *cfg, int *cut2, int *other, int *end_ms)
{
    policy p; pol_init(&p, W, H);
    p.has_engine = 1;
    pol_config(&p, "quiet_ms=2000 motion=adaptive mmin=gc16 mmax=a2_2 curve=off still=range");
    if (cfg) pol_config(&p, cfg);
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 2000, j);
    box win = { 0, 0, W, H };
    int64_t start = now, slow = start + 2000, stop = start + 8000, first = 0, last_frame = 0;
    int pend = 0, gc = 0, k2 = 0, ko = 0;
    box ch = {0};
    *end_ms = -1;
    while (now < stop + 3000) {
        now += 10;
        int gap = now < slow ? 100 : 150;
        if (now < stop && (now - start) % gap == 0) {
            if (now < slow) ch = (box){ 0, 400, W, 1336 };
            else { int x = 300 + (int)((now - slow) / 150) * 4; ch = (box){ x, 900, x + 160, 1060 }; }
            if (!pend) first = now;
            pend++; pol_damage(&p, now); last_frame = now;
        }
        actions o = {0};
        int st = pol_step(&p, now, 0, pend, first, &o);
        if (st == STEP_BURST) {
            burst_in in = { now, one(ch), win, PM_AUTO, 0, 0, BUSY_AUTO, 0, always };
            pol_burst(&p, &in, &o);
        }
        if (st != STEP_NONE) pend = 0;
        for (int i = 0; i < o.n; i++) {
            if (o.a[i].kind != A_SEND) continue;
            if (o.a[i].wf == WF_GC16) {
                if (now < stop) gc++;
                else if (*end_ms < 0) *end_ms = (int)(now - last_frame);
            } else if (now < stop) {
                if (o.a[i].wf == WF_DU && o.a[i].n == 2) k2++; else ko++;
            }
        }
    }
    free(j);
    *cut2 = k2; *other = ko;
    return gc;
}

static void scenario_always(void)
{
    int k2, ko, end, n;
    /* before: the slow scene left the run (cleanup despite motion: the figure
     * is small against the run's area) and its changes went the sized way */
    n = game_run(0, NULL, &k2, &ko, &end);
    CHECK(n >= 1, "game, no motion=always: %d GC16s while it drew (the collapse: want some)", n);
    /* motion=always: one run at the fluidity (A2/2) all along, the cleanup once it stops */
    n = game_run(1, NULL, &k2, &ko, &end);
    CHECK(n == 0, "motion=always: %d GC16s while the game drew (want none)", n);
    CHECK(ko == 0 && k2 >= 50, "motion=always: %d A2/2 bursts, %d other DUs (want all A2/2)", k2, ko);
    CHECK(end >= 0 && end <= 1300, "motion=always: cleanup %d ms after the last frame", end);
    /* the speed curve on (an older eink.conf): the slow scene still paints the fast end */
    n = game_run(1, "curve=on", &k2, &ko, &end);
    CHECK(n == 0 && ko == 0, "motion=always, curve=on: %d GC16s, %d DUs not A2/2", n, ko);
    /* Clear (mmax=a2_8): its own cut */
    n = game_run(1, "mmax=a2_8", &k2, &ko, &end);
    CHECK(n == 0 && k2 == 0 && ko >= 50, "motion=always, Clear: %d GC16s, %d A2/2, %d A2/8", n, k2, ko);
}

/* whole frames (sway): in a run the read starts as the damage arrives; X (and
 * sway outside a run) keeps the COALESCE_MS wait. Returns the ms from the
 * damage to the burst for frame 3 of a "fast" run (frame 1 starts the run) */
static int whole_wait(int whole, int pm)
{
    policy p; pol_init(&p, W, H);
    pol_config(&p, "debounce_ms=0");   /* (here: the coalesce itself) */
    p.whole = whole;
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 3000, j);
    box ff = { 0, 110, W, H };
    int got = -1;
    for (int f = 0; f < 3; f++) {
        now += pm == PM_FAST ? 200 : 900;   /* past run_ms; outside a run past GC16_BUSY_MS too */
        int64_t first = now;
        pol_input(&p, now);             /* (typing: not quiet_ms's idle throttle) */
        pol_damage(&p, now);
        for (int64_t t = now; t < now + 50; t++) {
            actions o = {0};
            int to = pol_timeout(&p, t, 0, 1);
            if (pol_step(&p, t, 0, 1, first, &o) == STEP_BURST) {
                box b = pm == PM_FAST ? (box){ 0, 300, W, 900 } : (box){ 100, 400, 140, 420 };   /* a cursor blink */
                burst_in in = { t, one(b), ff, pm, 0, 0, BUSY_AUTO, 0, 0 };
                pol_burst(&p, &in, &o);
                if (f == 2) got = (int)(t - first);
                if (f == 2 && pm != PM_FAST) CHECK(p.state != S_RUN, "whole_wait: the cursor blink started a run");
                CHECK(to >= 0, "whole_wait: negative timeout %d", to);
                break;
            }
        }
    }
    free(j);
    return got;
}

static void scenario_whole(void)
{
    int a = whole_wait(1, PM_FAST), b = whole_wait(0, PM_FAST), c = whole_wait(1, PM_AUTO);
    CHECK(a == 0, "whole frames in a run: read after %d ms (want at once)", a);
    CHECK(b == COALESCE_MS, "X damage in a run: read after %d ms (want %d)", b, COALESCE_MS);
    CHECK(c == COALESCE_MS, "whole frames outside a run: read after %d ms (want %d)", c, COALESCE_MS);
}

/* the dead band's true frame (pol_true): no run, however it falls; a cleanup
 * that waits keeps its time and takes the areas along; at rest they get a
 * cleanup of their own (partial, no flash); the next app frame is judged as if
 * nothing had come */
static void scenario_true_frame(void)
{
    box a = { 100, 200, 600, 700 }, far = { 800, 1200, 1300, 1700 }, all = { 0, 0, W, H };
    {   /* a run waits for its end: the same end, at the same time */
        int64_t end[2];
        for (int k = 0; k < 2; k++) {
            policy p; pol_init(&p, W, H);
            int64_t now = 1000;
            journal *j = calloc(1, sizeof *j);
            idle_until(&p, &now, 2000, j);
            for (int i = 0; i < 6; i++) { burst(&p, &now, one(a), all, PM_FAST, j); now += 80; }
            CHECK(p.state == S_RUN, "true frame: the run did not start");
            int64_t dmg = p.last_dmg;
            now += 100;
            if (k) {
                CHECK(pol_true(&p, &(boxlist){ .b = { a }, .n = 1 }) == 1, "true frame in a run: not taken");
                CHECK(p.state == S_RUN && p.last_dmg == dmg, "true frame in a run: %s, last_dmg moved %d",
                      state_name(p.state), (int)(p.last_dmg - dmg));
            }
            j->n = 0;
            while (p.state != S_IDLE && now < 5000) idle_until(&p, &now, now + 1, j);
            end[k] = now;
            CHECK(count(j, WF_GC16, M_PART) + count(j, WF_GC16, M_AREA) >= 1, "true frame: the run end is gone");
            free(j);
        }
        CHECK(end[0] == end[1], "true frame: the run ended at %d, without it %d", (int)end[1], (int)end[0]);
    }
    {   /* at rest: one partial, not a run, and quiet again; the area outside any window too */
        policy p; pol_init(&p, W, H);
        int64_t now = 1000;
        journal *j = calloc(1, sizeof *j);
        idle_until(&p, &now, 3000, j);
        j->n = 0;
        pol_true(&p, &(boxlist){ .b = { far }, .n = 1 });
        CHECK(p.state == S_CLEANUP, "true frame at rest: %s", state_name(p.state));
        idle_until(&p, &now, now + 1000, j);
        CHECK(count(j, WF_GC16, M_PART) == 1 && count(j, WF_GC16, M_AREA) == 0 && p.state == S_IDLE,
              "true frame at rest: %d partials %d flashes, ends %s", count(j, WF_GC16, M_PART), count(j, WF_GC16, M_AREA),
              state_name(p.state));
        free(j);
    }
    {   /* nothing differed: nothing happens */
        policy p; pol_init(&p, W, H);
        int64_t now = 1000;
        journal *j = calloc(1, sizeof *j);
        idle_until(&p, &now, 3000, j);
        j->n = 0;
        pol_true(&p, &(boxlist){ .n = 0 });
        idle_until(&p, &now, now + 1000, j);
        CHECK(p.state == S_IDLE && count(j, WF_GC16, M_PART) == 0, "true frame without a change: %s", state_name(p.state));
        free(j);
    }
    {   /* a cleanup's GC16 is in flight when the true frame lands: still no run */
        policy p; pol_init(&p, W, H);
        int64_t now = 1000;
        journal *j = calloc(1, sizeof *j);
        idle_until(&p, &now, 3000, j);
        burst(&p, &now, one((box){ 100, 200, 300, 300 }), all, PM_AUTO, j);
        idle_until(&p, &now, now + 300, j);
        pol_true(&p, &(boxlist){ .b = { a }, .n = 1 });
        CHECK(p.state != S_RUN, "true frame under a GC16: %s", state_name(p.state));
        idle_until(&p, &now, now + 1000, j);
        CHECK(p.state == S_IDLE, "true frame under a GC16: ends %s", state_name(p.state));
        free(j);
    }
    {   /* single mode: no cleanup, the caller paints them */
        policy p; pol_init(&p, W, H);
        pol_config(&p, "still=range mmin=a2 mmax=a2 motion=adaptive");
        p.has_engine = 1;
        CHECK(pol_true(&p, &(boxlist){ .b = { a }, .n = 1 }) == 0 && !p.dirty.n, "true frame, single mode: taken");
    }
}

/* window changes (pol_window): what they uncover or cover gets one flash */
static void scenario_windows(void)
{
    box popup = { 300, 500, 1100, 1200 };               /* ~21 % of the screen */
    /* a popup vanishes: the window below repaints -> a partial GC16 at once,
     * then one local flash of that area at the cleanup */
    {
        policy p; pol_init(&p, W, H);
        int64_t now = 1000;
        journal *j = calloc(1, sizeof *j);
        idle_until(&p, &now, 3000, j);
        j->n = 0;
        pol_window(&p, popup, now);
        burst(&p, &now, one(popup), (box){ 0, 0, W, H }, PM_AUTO, j);
        idle_until(&p, &now, now + 1500, j);
        CHECK(count(j, WF_GC16, M_PART) == 1 && count(j, WF_GC16, M_AREA) == 1 && count(j, WF_DU, M_PART) == 0,
              "popup gone: %d partials %d flashes %d DU", count(j, WF_GC16, M_PART), count(j, WF_GC16, M_AREA),
              count(j, WF_DU, M_PART));
        CHECK(p.state == S_IDLE && !p.stale.n, "popup gone: ends %s, %d stale", state_name(p.state), p.stale.n);
        free(j);
    }
    /* workspace switch: 3 tiles repaint one after another, 120 ms apart. Before:
     * the 2nd and 3rd became a motion run (a GC16 was in flight): dithered DU,
     * and the run's end GC16 covered only the first window */
    {
        policy p; pol_init(&p, W, H);
        int64_t now = 1000;
        journal *j = calloc(1, sizeof *j);
        idle_until(&p, &now, 3000, j);
        j->n = 0;
        box t[3] = { { 0, 82, W, 700 }, { 0, 700, W, 1300 }, { 0, 1300, W, H } };
        for (int i = 0; i < 3; i++) pol_window(&p, t[i], now);
        for (int i = 0; i < 3; i++) {
            burst(&p, &now, one(t[i]), t[i], PM_AUTO, j);
            now += 90;
        }
        idle_until(&p, &now, now + 1500, j);
        CHECK(count(j, WF_DU, M_PART) == 0, "workspace switch: %d DU", count(j, WF_DU, M_PART));
        CHECK(count(j, WF_GC16, M_AREA) >= 1, "workspace switch: no flash");
        box fl = {0};
        for (int i = 0; i < j->n; i++)
            for (int k = 0; k < j->log[i].n; k++)
                if (j->log[i].a[k].kind == A_SEND && j->log[i].a[k].mode == M_AREA) fl = box_union(fl, j->log[i].a[k].b);
        CHECK(box_inside((box){ 0, 82, W, H }, fl), "workspace switch: flash %d,%d-%d,%d misses a tile",
              fl.x0, fl.y0, fl.x1, fl.y1);
        CHECK(p.state == S_IDLE, "workspace switch: ends %s", state_name(p.state));
        free(j);
    }
    /* a workspace switch while a motion run is going (a repaint under the GC16
     * started it): the new windows still come as GC16, not dithered DU */
    {
        policy p; pol_init(&p, W, H);
        int64_t now = 1000;
        journal *j = calloc(1, sizeof *j);
        idle_until(&p, &now, 3000, j);
        burst(&p, &now, one((box){ 0, 82, W, 700 }), (box){ 0, 82, W, 700 }, PM_AUTO, j);   /* a GC16 */
        now += 100;
        burst(&p, &now, one((box){ 0, 600, W, 650 }), (box){ 0, 82, W, 700 }, PM_AUTO, j);  /* under it -> run */
        CHECK(p.state == S_RUN, "run setup: %s", state_name(p.state));
        j->n = 0;
        now += 100;
        box t = { 0, 82, W, H };
        pol_window(&p, t, now);
        burst(&p, &now, one(t), t, PM_AUTO, j);
        CHECK(count(j, WF_DU, M_PART) == 0, "switch inside a run: %d DU", count(j, WF_DU, M_PART));
        idle_until(&p, &now, now + 1500, j);
        CHECK(p.state == S_IDLE, "switch inside a run: ends %s", state_name(p.state));
        free(j);
    }
    /* a GC16 in one window, then within GC16_BUSY_MS
     * a repaint of another one (a button tap: the app losing focus, then the
     * page beside it): one GC16 each, no run (was: DU + run + end flash) */
    {
        policy p; pol_init(&p, W, H);
        int64_t now = 1000;
        journal *j = calloc(1, sizeof *j);
        idle_until(&p, &now, 3000, j);
        box l = { 0, 82, W / 2, H }, r = { W / 2 + 4, 82, W, H };
        burst(&p, &now, one(l), l, PM_AUTO, j);
        now += 100;
        j->n = 0;
        burst(&p, &now, one(r), r, PM_AUTO, j);
        CHECK(p.state != S_RUN && count(j, WF_DU, M_PART) == 0, "GC16 elsewhere: %s, %d DU", state_name(p.state), count(j, WF_DU, M_PART));
        idle_until(&p, &now, now + 1500, j);
        CHECK(count(j, WF_GC16, M_PART) + count(j, WF_GC16, M_AREA) == 1, "GC16 elsewhere: %d GC16s for one repaint",
              count(j, WF_GC16, M_PART) + count(j, WF_GC16, M_AREA));
        /* the same window again within GC16_BUSY_MS: content faster than GC16, a run */
        idle_until(&p, &now, now + 3000, j);
        burst(&p, &now, one(l), l, PM_AUTO, j);
        now += 100;
        burst(&p, &now, one((box){ 0, 400, W / 2, 500 }), l, PM_AUTO, j);
        CHECK(p.state == S_RUN, "GC16 under it: %s, want a run", state_name(p.state));
        free(j);
    }
    /* a DU'd stroke, then the app's own GC16 repaint over it: the cleanup has
     * nothing left there (was: GC16'd twice within 250 ms, "renders twice") */
    {
        policy p; pol_init(&p, W, H);
        int64_t now = 1000;
        journal *j = calloc(1, sizeof *j);
        idle_until(&p, &now, 3000, j);
        burst(&p, &now, one((box){ 100, 500, 140, 540 }), (box){ 0, 82, W / 2, H }, PM_AUTO, j);   /* small: DU */
        CHECK(count(j, WF_DU, M_PART) == 1, "stroke: %d DU", count(j, WF_DU, M_PART));
        now += 30;
        burst(&p, &now, one((box){ 0, 300, W / 2 - 40, 900 }), (box){ 0, 82, W / 2, H }, PM_QUALITY, j);   /* the app's GC16 */
        j->n = 0;
        idle_until(&p, &now, now + 1500, j);
        CHECK(count(j, WF_GC16, M_PART) + count(j, WF_GC16, M_AREA) == 0, "stroke under the app's GC16: %d more GC16s",
              count(j, WF_GC16, M_PART) + count(j, WF_GC16, M_AREA));
        CHECK(p.state == S_IDLE, "stroke under the app's GC16: ends %s", state_name(p.state));
        free(j);
    }
    /* a window change nobody repaints (mapped and unmapped at once): nothing */
    {
        policy p; pol_init(&p, W, H);
        int64_t now = 1000;
        journal *j = calloc(1, sizeof *j);
        idle_until(&p, &now, 3000, j);
        j->n = 0;
        pol_window(&p, popup, now);
        idle_until(&p, &now, now + CAND_MS + 500, j);
        burst(&p, &now, one((box){ 350, 550, 400, 600 }), (box){ 0, 0, W, H }, PM_AUTO, j);   /* later typing there */
        idle_until(&p, &now, now + 1500, j);
        CHECK(count(j, WF_GC16, M_AREA) == 0, "unrepainted window change: %d flashes", count(j, WF_GC16, M_AREA));
        free(j);
    }
    /* the repaint was big enough to flash at once: no second flash */
    {
        policy p; pol_init(&p, W, H);
        int64_t now = 1000;
        journal *j = calloc(1, sizeof *j);
        idle_until(&p, &now, 3000, j);
        j->n = 0;
        box all = { 0, 82, W, H };
        pol_window(&p, all, now);
        burst(&p, &now, one(all), all, PM_AUTO, j);
        idle_until(&p, &now, now + 1500, j);
        CHECK(count(j, WF_GC16, M_AREA) == 1, "big window change: %d flashes", count(j, WF_GC16, M_AREA));
        free(j);
    }
}

/* a popup (override-redirect) maps and paints, no flash (the quick
 * settings card is 36 % of the screen, over flash_min, and blinked), nothing
 * left for the cleanup: popup=du (default) = DU at once + the GC16 partial
 * right behind; popup=fast = one GC16 partial; popup=gc16 = the old way */
static void scenario_popup(void)
{
    box card = { 540, 100, 1388, 1218 };
    for (int fast = 2; fast >= 0; fast--) {         /* 2 popup=du (default), 1 popup=fast, 0 popup=gc16 */
        policy p; pol_init(&p, W, H);
        if (!fast) pol_config(&p, "popup=gc16");
        if (fast == 1) pol_config(&p, "popup=fast");
        int64_t now = 1000;
        journal *j = calloc(1, sizeof *j);
        idle_until(&p, &now, 3000, j);
        j->n = 0;
        pol_popup(&p, card, now);
        now += 150;                         /* GTK builds and draws it */
        burst(&p, &now, one(card), (box){ 0, 0, W, H }, PM_AUTO, j);
        if (fast == 1)
            CHECK(j->n >= 2 && j->log[1].n >= 1 && j->log[1].a[0].kind == A_SEND && j->log[1].a[0].wf == WF_GC16 &&
                  j->log[1].a[0].mode == M_PART && count(j, WF_DU, M_PART) == 0, "popup: not one GC16 partial");
        if (fast == 2) {
            CHECK(j->n >= 2 && j->log[1].n >= 3 && j->log[1].a[0].kind == A_SEND && j->log[1].a[0].wf == WF_DU &&
                  j->log[1].a[1].kind == A_GRAB && j->log[1].a[2].wf == WF_GC16 && j->log[1].a[2].mode == M_PART,
                  "popup: not DU, grab, GC16 partial in one burst");
        }
        /* it repaints once more 200 ms later (a switch's state): no motion run */
        now += 200;
        burst(&p, &now, one((box){ 600, 200, 700, 260 }), (box){ 0, 0, W, H }, PM_AUTO, j);
        CHECK(p.state != S_RUN, "popup%s: second paint started a run", fast ? "" : "=gc16");
        idle_until(&p, &now, now + 1500, j);
        if (fast == 1)
            CHECK(count(j, WF_GC16, M_AREA) == 0 && count(j, WF_DU, M_PART) == 0 && count(j, WF_GC16, M_PART) == 2,
                  "popup: %d flashes %d DU %d partials", count(j, WF_GC16, M_AREA), count(j, WF_DU, M_PART),
                  count(j, WF_GC16, M_PART));
        else if (fast == 2)
            CHECK(count(j, WF_GC16, M_AREA) == 0 && count(j, WF_DU, M_PART) == 2 && count(j, WF_GC16, M_PART) == 2,
                  "popup: %d flashes %d DU %d partials", count(j, WF_GC16, M_AREA), count(j, WF_DU, M_PART),
                  count(j, WF_GC16, M_PART));
        else
            CHECK(count(j, WF_GC16, M_AREA) == 1, "popup=gc16: %d flashes", count(j, WF_GC16, M_AREA));
        CHECK(p.state == S_IDLE, "popup: ends %s", state_name(p.state));
        /* after POPUP_MS its area is ordinary again: a small change there is a DU */
        j->n = 0;
        now += POPUP_MS;
        burst(&p, &now, one((box){ 600, 200, 640, 240 }), (box){ 0, 0, W, H }, PM_AUTO, j);
        CHECK(count(j, WF_DU, M_PART) == 1 && count(j, WF_GC16, M_PART) == 0, "popup: stays fast after POPUP_MS");
        free(j);
    }
}

/* a popup replaces another (quick settings -> app drawer, one shell
 * window resized): the drawer paints DU + GC16 and settles -- no stale flash
 * of the closed card's area over it (~1 s, after its paint), and its second
 * paint, which carries the bar button beside it, is no DU run */
static void scenario_popup_swap(void)
{
    box card = { 480, 108, 1400, 1432 }, drawer = { 4, 108, 1400, 1188 }, button = { 1300, 16, 1384, 88 };
    policy p; pol_init(&p, W, H);
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 3000, j);
    pol_popup(&p, card, now);
    now += 100;
    burst(&p, &now, one(card), (box){ 0, 0, W, H }, PM_AUTO, j);
    idle_until(&p, &now, now + 3000, j);
    j->n = 0;
    pol_window(&p, card, now);                          /* the card goes ... */
    pol_popup(&p, drawer, now);                         /* ... the drawer comes */
    now += 60;
    burst(&p, &now, one(drawer), (box){ 0, 0, W, H }, PM_AUTO, j);
    now += 60;                                          /* its content + the bar button */
    boxlist two = one((box){ 480, 126, 1392, 1182 });
    list_add(&two, button);
    burst(&p, &now, two, (box){ 0, 0, W, H }, PM_AUTO, j);
    CHECK(p.state != S_RUN, "popup swap: second paint started a run");
    idle_until(&p, &now, now + 1500, j);
    /* DU + GC16 for the drawer, then for its two rects */
    CHECK(count(j, WF_GC16, M_AREA) == 0 && count(j, WF_DU, M_PART) == 3 && count(j, WF_GC16, M_PART) == 3,
          "popup swap: %d flashes %d DU %d partials", count(j, WF_GC16, M_AREA), count(j, WF_DU, M_PART),
          count(j, WF_GC16, M_PART));
    CHECK(p.state == S_IDLE && !p.stale.n, "popup swap: ends %s, %d stale", state_name(p.state), p.stale.n);
    /* what the card left uncovered below the drawer repaints: that one gets its flash */
    j->n = 0;
    now += 100;
    burst(&p, &now, one((box){ 480, 1188, 1400, 1432 }), (box){ 0, 0, W, H }, PM_AUTO, j);
    idle_until(&p, &now, now + 1500, j);
    CHECK(count(j, WF_GC16, M_AREA) + count(j, WF_GC16, M_PART) >= 1, "popup swap: the uncovered strip got no GC16");
    free(j);
}

/* a popup closes over a "fast" app (Firefox under the quick settings):
 * what it uncovered repaints in ONE GC16 partial -- no DU run, no stale flash;
 * the app repaints in two bursts, wider than the card */
static void scenario_popup_close(void)
{
    box card = { 480, 108, 1400, 1432 };
    policy p; pol_init(&p, W, H);
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 3000, j);
    pol_popup(&p, card, now);
    now += 100;
    burst(&p, &now, one(card), (box){ 0, 0, W, H }, PM_FAST, j);
    idle_until(&p, &now, now + 3000, j);
    j->n = 0;
    pol_popup_gone(&p, card, now);
    now += 60;
    burst(&p, &now, one((box){ 0, 108, W, 1432 }), (box){ 0, 108, W, H }, PM_FAST, j);
    now += 80;
    burst(&p, &now, one((box){ 20, 126, 1392, 400 }), (box){ 0, 108, W, H }, PM_FAST, j);
    CHECK(p.state != S_RUN, "popup close: started a run");
    idle_until(&p, &now, now + 2000, j);
    CHECK(count(j, WF_GC16, M_AREA) == 0 && count(j, WF_DU, M_PART) == 0 && count(j, WF_GC16, M_PART) == 2,
          "popup close: %d flashes %d DU %d partials", count(j, WF_GC16, M_AREA), count(j, WF_DU, M_PART),
          count(j, WF_GC16, M_PART));
    CHECK(p.state == S_IDLE && !p.stale.n, "popup close: ends %s, %d stale", state_name(p.state), p.stale.n);
    /* later the app is itself again: a scroll there is DU */
    j->n = 0;
    now += POPUP_MS;
    burst(&p, &now, one((box){ 100, 500, 1300, 900 }), (box){ 0, 108, W, H }, PM_FAST, j);
    CHECK(count(j, WF_DU, M_PART) == 1, "popup close: the app is not fast again after POPUP_MS");
    free(j);
}

/* a tooltip closes at a touch-down that starts a scroll (GNOME
 * Software): the stroke's frames are the run's cut A2, not GC16 partials of
 * "what the popup uncovered" */
static void scenario_popup_close_scroll(void)
{
    box tip = { 448, 1068, 592, 1126 };
    policy p; pol_init(&p, W, H);
    pol_engine(&p, 0);
    p.has_engine = 1; p.adaptive = 1; p.m_curve = 0; p.m_max = LV_A2_2;
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 3000, j);
    pol_popup(&p, tip, now);
    now += 100;
    burst(&p, &now, one(tip), (box){ 0, 110, W, H }, PM_AUTO, j);
    idle_until(&p, &now, now + 3000, j);
    j->n = 0;
    pol_finger(&p, now, 1, 0);                          /* the touch-down closes it */
    pol_popup_gone(&p, tip, now);
    for (int k = 0; k < 8; k++) {
        now += 50;
        pol_finger(&p, now, 1, 0.8);
        burst_moving = k > 0;
        burst(&p, &now, one((box){ 0, 110, W, H }), (box){ 0, 110, W, H }, PM_FAST, j);
    }
    burst_moving = 0;
    CHECK(count(j, WF_GC16, M_PART) == 0 && count(j, WF_GC16, M_AREA) == 0,
          "popup close + scroll: %d GC16 partials %d flashes during the stroke", count(j, WF_GC16, M_PART), count(j, WF_GC16, M_AREA));
    CHECK(count(j, WF_DU, M_PART) >= 8, "popup close + scroll: %d DU steps, want 8", count(j, WF_DU, M_PART));
    free(j);
}

/* run_wait (device log): Firefox on a heavy page, a frame every
 * ~550 ms with the finger down (touch=fast), then its kinetic tail after the
 * lift, frames 550..1050 ms apart. Wanted: the run holds over the whole
 * scroll (no GC16 while it moves), one cleanup at the end, no full flash;
 * run_wait=0 shows the old thrash. A held finger keeps a run open however
 * slow the app; with no input for RW_INPUT_MS a blinking cursor gets its
 * cleanup at idle_ms again. */
static int gc16s(const journal *j) { return count(j, WF_GC16, M_PART) + count(j, WF_GC16, M_AREA) + count(j, WF_GC16, M_SCREEN); }
static void frames_to(policy *p, int64_t *now, int64_t until, journal *j)   /* time passes, no damage */
{
    while (*now < until) {
        *now += 10;
        actions o = {0};
        pol_step(p, *now, 0, 0, 0, &o);
        jadd(j, &o);
    }
}
static void scenario_run_wait(void)
{
    static const int tail[] = { 560, 600, 650, 720, 800, 900, 1050 };
    for (int old = 0; old <= 1; old++) {
        policy p; pol_init(&p, W, H);
        pol_config(&p, "idle_ms=400");          /* the device's eink.conf */
        if (old) pol_config(&p, "run_wait=0");
        int64_t now = 1000;
        journal *j = calloc(1, sizeof *j);
        idle_until(&p, &now, 2000, j);
        j->n = 0;
        box ff = { 0, 110, W, H }, pg = { 0, 300, W, 1760 };
        pol_finger(&p, now, 1, 1.0);
        for (int i = 0; i < 3; i++) { burst(&p, &now, one(pg), ff, PM_FAST, j); frames_to(&p, &now, now + 550, j); }
        pol_finger(&p, now, 0, 0);
        int during = 0;
        for (int i = 0; i < 7; i++) {
            burst(&p, &now, one(pg), ff, PM_AUTO, j);
            during = gc16s(j);
            frames_to(&p, &now, now + tail[i], j);
        }
        int mid = gc16s(j);
        idle_until(&p, &now, now + 2000, j);
        int end = gc16s(j) - mid;
        if (old) CHECK(mid >= 4, "run_wait=0: the old thrash expected (>= 4 GC16 while moving), got %d", mid);
        else {
            CHECK(during == 0 && mid == 0, "slow scroll: %d GC16 while it moved", mid);
            CHECK(end == 1, "slow scroll: one cleanup at the end, got %d", end);
            CHECK(count(j, WF_GC16, M_SCREEN) == 0, "slow scroll: a full-screen flash");
            CHECK(p.state == S_IDLE, "slow scroll: ends idle, is %s", state_name(p.state));
        }
        free(j);
    }
    {   /* a held finger: 1.5 s between frames, no cleanup until the lift */
        policy p; pol_init(&p, W, H);
        int64_t now = 1000;
        journal *j = calloc(1, sizeof *j);
        idle_until(&p, &now, 2000, j);
        j->n = 0;
        box ff = { 0, 110, W, H }, pg = { 0, 300, W, 1500 };
        pol_finger(&p, now, 1, 0.5);
        for (int i = 0; i < 3; i++) { burst(&p, &now, one(pg), ff, PM_FAST, j); frames_to(&p, &now, now + 1500, j); }
        CHECK(gc16s(j) == 0, "finger held: %d GC16 with the finger down", gc16s(j));
        pol_finger(&p, now, 0, 0);
        idle_until(&p, &now, now + 1500, j);
        CHECK(gc16s(j) == 1, "finger held: one cleanup after the lift, got %d", gc16s(j));
        free(j);
    }
    {   /* after a scroll, no input: a cursor blinking every 500 ms is cleaned at idle_ms */
        policy p; pol_init(&p, W, H);
        int64_t now = 1000;
        journal *j = calloc(1, sizeof *j);
        idle_until(&p, &now, 2000, j);
        box ff = { 0, 110, W, H };
        for (int i = 0; i < 5; i++) { burst(&p, &now, one((box){ 0, 300, W, 1500 }), ff, PM_FAST, j); now += 50; }
        frames_to(&p, &now, now + RW_INPUT_MS, j);
        j->n = 0;
        for (int i = 0; i < 8; i++) {           /* no pol_input: nobody is touching */
            int64_t first = now;
            pol_damage(&p, now);
            now += COALESCE_MS;
            actions o = {0};
            if (pol_step(&p, now, 0, 1, first, &o) == STEP_BURST) {
                jadd(j, &o);
                actions b = {0};
                burst_in in = { now, one((box){ 100, 400, 104, 430 }), ff, PM_AUTO, 0, 0, BUSY_AUTO, 0, 0 };
                pol_burst(&p, &in, &b);
                jadd(j, &b);
            }
            frames_to(&p, &now, now + 500, j);
        }
        CHECK(gc16s(j) >= 1, "cursor blink with no input: never cleaned");
        free(j);
    }
}

/* the mouse pointer (pol_pointer): while it moves, cut A2s at the
 * range's fast end (A2/2 by default; a dithered DU without the engine), no
 * run, no GC16; at rest ONE GC16 partial after idle_ms, no flash; a hover
 * repaint (a tooltip at the pointer) right after that GC16 is no run; its DUs
 * count once per tile for ghosting; run_cut never set */
static int pointer_moves(policy *p, int64_t *now, int n, int x0, journal *j, int *how, int *bad)
{
    box old = {0};
    int runs = 0;
    for (int i = 0; i < n; i++) {
        box nb = { x0 + i * 6, 600, x0 + i * 6 + 48, 648 };     /* a 48 px pointer, 6 px a step */
        boxlist ch = {0};
        list_add(&ch, box_union(old, nb));
        old = nb;
        actions o = {0};
        *how = pol_pointer(p, &ch, *now, &o);
        jadd(j, &o);
        for (int k = 0; k < o.n; k++)
            if (o.a[k].kind == A_SEND && (o.a[k].wf != WF_DU || o.a[k].mode != M_PART)) (*bad)++;
        runs += p->state == S_RUN;
        frames_to(p, now, *now + 25, j);
        runs += p->state == S_RUN;
    }
    return runs;
}
static int min_cut(const journal *j)
{
    int m = 99;
    for (int i = 0; i < j->n; i++)
        for (int k = 0; k < j->log[i].n; k++)
            if (j->log[i].a[k].kind == A_SEND && j->log[i].a[k].wf == WF_DU && j->log[i].a[k].n < m) m = j->log[i].a[k].n;
    return m;
}
/* a touchpad's two-finger scroll (Chromium): swipes 0.3-1.2 s
 * apart, each its own run ended by a full-window GC16 the next one waited on.
 * Wheel events hold the run as a finger down does (wheel_hold_ms) */
static int wheel_swipes(int hold, int *ended)
{
    policy p; pol_init(&p, W, H);
    char cf[64]; snprintf(cf, sizeof cf, "motion=adaptive wheel_hold_ms=%d", hold);
    pol_config(&p, cf);
    p.has_engine = 1;
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    box win = { 0, 100, W, H };
    idle_until(&p, &now, 3000, j);
    j->n = 0;
    *ended = 0;
    burst_moving = 1;
    for (int s = 0; s < 4; s++) {
        for (int k = 0; k < 8; k++) {                   /* a swipe: wheel events + moving frames */
            pol_wheel(&p, now);
            burst(&p, &now, one((box){ 0, 200, W, H - 100 }), win, PM_AUTO, j);
            frames_to(&p, &now, now + 30, j);
        }
        burst_moving = 0;
        int was = p.state == S_RUN;
        frames_to(&p, &now, now + 600, j);              /* the fingers lift, set down again */
        *ended += was && p.state != S_RUN;
        burst_moving = 1;
    }
    burst_moving = 0;
    int g = gc16s(j);
    frames_to(&p, &now, now + 3000, j);                 /* the scroll is over: its cleanup */
    CHECK(p.state != S_RUN, "wheel (hold %d): still a run after 3 s", hold);
    free(j);
    return g;
}
static void scenario_wheel(void)
{
    int ended, g = wheel_swipes(WHEEL_HOLD_MS, &ended);
    CHECK(ended == 0 && g == 0, "wheel swipes 600 ms apart: %d run ends, %d GC16 between them (want 0, 0)", ended, g);
    g = wheel_swipes(0, &ended);
    CHECK(ended >= 1, "wheel_hold_ms=0: %d run ends between swipes (want the old behaviour, >= 1)", ended);
}
/* the wakes are the deadlines (step 2b): the cleanup's quiet is
 * counted from the last damage, not idle_ms again from the last look; a wheel
 * scroll's hold wakes when it ends */
/* Content.stalled (a stall must not end the run with a GC16
 * and a new ramp): a run's frames stop while the last one still moved fast (an
 * app missing frames in a fling, nobody touching) -> the run holds stall_ms; a
 * stop that decays (slower frames first) ends at the usual wait */
static int stall_run(const char *cfg, int resume, int decay, int64_t *end_gc16)
{
    policy p; pol_init(&p, W, H);
    pol_config(&p, cfg);
    p.has_engine = 1;
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 2000, j);
    box win = { 0, 100, W, H };
    burst_moving = 1; burst_speed = 2.0;
    for (int k = 0; k < 10; k++) { burst(&p, &now, one((box){ 0, 200, W, H - 100 }), win, PM_AUTO, j); now += 30; }
    int mid = 0;
    j->n = 0;
    int64_t stop = p.last_dmg;                        /* (from the last frame's damage) */
    frames_to(&p, &now, now + 700, j);                  /* the app misses 700 ms of frames */
    mid = gc16s(j);
    if (resume) for (int k = 0; k < 5; k++) { burst(&p, &now, one((box){ 0, 200, W, H - 100 }), win, PM_AUTO, j); now += 30; }
    if (decay) {
        burst_speed = 0.1;
        for (int k = 0; k < 3; k++) { burst(&p, &now, one((box){ 0, 200, W, 400 }), win, PM_AUTO, j); now += 30; }
    }
    burst_moving = 0; burst_speed = 0;
    if (resume || decay) { j->n = 0; stop = p.last_dmg; }
    *end_gc16 = -1;
    for (int64_t until = now + 3000; now < until && *end_gc16 < 0; ) {
        frames_to(&p, &now, now + 10, j);
        if (gc16s(j)) *end_gc16 = now - stop;
    }
    free(j);
    return mid;
}
static void scenario_stall(void)
{
    int64_t e;
    int mid = stall_run("motion=adaptive", 1, 1, &e);
    CHECK(mid == 0, "stall: %d GC16 during a 700 ms frame stall at speed (want 0: the run holds)", mid);
    CHECK(e >= 0 && e < 700, "stall: the decayed stop's end GC16 %lld ms after the last frame (want the usual wait, < 700)", (long long)e);
    mid = stall_run("motion=adaptive stall_ms=0", 1, 1, &e);
    CHECK(mid > 0, "stall_ms=0: no GC16 during the stall (the old behaviour ends the run there)");
    stall_run("motion=adaptive", 0, 0, &e);             /* an abrupt stop at speed (a list's end): held, then ended */
    CHECK(e >= STALL_MS && e <= STALL_MS + 50, "stall: an abrupt stop at speed ends %lld ms after (want ~%d)", (long long)e, STALL_MS);
}
/* the first still frame's debounce: on by default (11-01); waits for quiet, capped; the exemptions */
static int db_step(policy *p, int64_t now, int64_t first)
{
    static actions out;
    out.n = 0;
    return pol_step(p, now, 0, 1, first, &out);
}
static policy *db_new(const char *cfg)
{
    static policy p;
    pol_init(&p, W, H);
    pol_config(&p, cfg);
    p.whole = 1;
    p.want_full = 0;
    p.state = S_IDLE;
    p.c.ink_t = -100000;
    return &p;
}
static void scenario_debounce(void)
{
    box o;
    policy *p = db_new("motion=adaptive");
    int64_t t = 10000;
    pol_damage(p, t);
    CHECK(pol_debounce_left(p, t) == 150, "debounce: holds 150 ms by default");
    p = db_new("motion=adaptive debounce_ms=0");
    pol_damage(p, t);
    CHECK(pol_debounce_left(p, t) == 0, "debounce_ms=0: no hold");
    CHECK(db_step(p, t + COALESCE_MS, t) == STEP_BURST, "debounce off: the first frame does not paint at the coalesce");
    p = db_new("motion=adaptive debounce_drag=1");
    pol_damage(p, t);
    CHECK(pol_debounce_left(p, t) == 0, "a handle held: the drag bar's first frame does not wait");
    pol_config(p, "debounce_drag=0");
    pol_damage(p, t + 1000);
    CHECK(pol_debounce_left(p, t + 1000) == 150, "the handle let go: the debounce is back");

    p = db_new("debounce_ms=70");
    pol_damage(p, t);
    CHECK(pol_debounce_left(p, t) == 70, "debounce: left %d, want 70", pol_debounce_left(p, t));
    CHECK(db_step(p, t + 20, t) != STEP_BURST, "debounce: painted during the wait");
    CHECK(pol_timeout(p, t + 20, 0, 1) == 50, "debounce: wakes in %d ms, want 50", pol_timeout(p, t + 20, 0, 1));
    pol_damage(p, t + 40);
    CHECK(db_step(p, t + 100, t) != STEP_BURST, "debounce: new damage did not extend the wait");
    CHECK(db_step(p, t + 110, t) == STEP_BURST, "debounce: no paint after the quiet");
    CHECK(p->db_t0 == 0, "debounce: the hold outlives its paint");

    p = db_new("debounce_ms=70");                       /* endless damage: the cap */
    for (int i = 0; i < 5; i++) pol_damage(p, t + i * 30);
    CHECK(db_step(p, t + 149, t) != STEP_BURST, "debounce: the cap came early");
    CHECK(db_step(p, t + 150, t) == STEP_BURST, "debounce: no paint at the cap under steady damage");

    p = db_new("debounce_ms=70");                       /* no still screen: no hold */
    pol_damage(p, t);
    p->db_t0 = 0;
    pol_damage(p, t + 100);
    CHECK(pol_debounce_left(p, t + 100) == 0, "debounce: held inside the gap (recent damage)");
    p = db_new("debounce_ms=70");
    p->last_burst = t - 100;
    pol_damage(p, t);
    CHECK(pol_debounce_left(p, t) == 0, "debounce: held inside the gap (recent paint)");

    p = db_new("debounce_ms=70 osk_up=1");              /* the keyboard knob */
    pol_damage(p, t);
    CHECK(pol_debounce_left(p, t) == 0, "debounce: held with the keyboard up");
    p = db_new("debounce_ms=70 osk_up=1 debounce_osk=1");
    pol_damage(p, t);
    CHECK(pol_debounce_left(p, t) == 70, "debounce_osk=1: not held with the keyboard up");
    p = db_new("debounce_ms=70");
    pol_damage(p, t);
    pol_config(p, "osk_up=1");
    CHECK(pol_debounce_left(p, t + 1) == 0, "debounce: a keyboard coming up keeps the hold");

    p = db_new("debounce_ms=70");                       /* a scroll, the pen: never held */
    pol_finger(p, t, 1, 1.0);
    pol_damage(p, t + 1);
    CHECK(pol_debounce_left(p, t + 1) == 0, "debounce: held under a moving finger");
    p = db_new("debounce_ms=70");
    p->c.ink_t = t - 10;
    pol_damage(p, t);
    CHECK(pol_debounce_left(p, t) == 0, "debounce: held right after pen ink");
    p = db_new("debounce_ms=70");
    pol_popup(p, (box){ 100, 100, 700, 700 }, t - 1);
    pol_damage(p, t);
    CHECK(pol_debounce_left(p, t) == 0, "debounce: held a popup's paint");

    p = db_new("debounce_ms=70");                       /* the tap: its area at once, the rest held */
    pol_tap(p, t, 1, 500, 800);
    pol_finger(p, t, 1, 0);
    pol_damage(p, t + 5);
    CHECK(pol_debounce_clip(p, (box){ 0, 0, W, H }, t + 5, &o) && o.x0 == 400 && o.y0 == 700 && o.x1 == 600 && o.y1 == 900,
          "debounce: whole-window damage not clipped to the tap (%d,%d-%d,%d)", o.x0, o.y0, o.x1, o.y1);
    CHECK(!pol_debounce_clip(p, (box){ 900, 1400, 1000, 1500 }, t + 5, &o), "debounce: damage far from the tap passed");
    p->db_tap = 1;
    CHECK(db_step(p, t + 5 + COALESCE_MS, t + 5) == STEP_BURST, "debounce: the tap's damage waits");
    p->db_tap = 0;
    CHECK(p->db_t0 != 0, "debounce: the tap's paint ended the hold");
    CHECK(db_step(p, t + 20, t + 5) != STEP_BURST, "debounce: the rest painted with the tap's");
    CHECK(db_step(p, t + 75, t + 5) == STEP_BURST, "debounce: the rest never painted");
    pol_tap(p, t + 100, 0, 0, 0);                       /* a tap's life after its lift */
    p->db_t0 = t + 350; p->last_dmg = t + 350;
    CHECK(pol_debounce_clip(p, (box){ 0, 0, W, H }, t + 350, &o), "debounce: tap dead inside debounce_tap_ms");
    p->db_t0 = t + 450; p->last_dmg = t + 450;
    CHECK(!pol_debounce_clip(p, (box){ 0, 0, W, H }, t + 450, &o), "debounce: tap alive after debounce_tap_ms");
}
static void scenario_wakes(void)
{
    policy p; pol_init(&p, W, H);
    pol_config(&p, "motion=adaptive");                 /* (run_wait=0, the old fixed idle_ms, holds nothing) */
    p.has_engine = 1;
    int64_t now = 1000;
    journal *j = calloc(1, sizeof *j);
    idle_until(&p, &now, 2000, j);
    burst_moving = 1;
    for (int k = 0; k < 6; k++) { burst(&p, &now, one((box){ 0, 200, W, H - 100 }), (box){ 0, 100, W, H }, PM_AUTO, j); now += 30; }
    burst_moving = 0;
    CHECK(p.state == S_RUN, "wakes: no run (%s)", state_name(p.state));
    int64_t last = p.last_dmg;
    now = last + 200;
    pol_input(&p, now);                                 /* (a key: the daemon looks, nothing to read) */
    int t = pol_timeout(&p, now, 0, 0);
    CHECK(t == p.idle_ms - 200, "wakes: timeout %d 200 ms into the quiet (want the rest, %d)", t, p.idle_ms - 200);
    pol_wheel(&p, now);                                 /* a touchpad scroll holds the run */
    now += p.wheel_hold - 100;
    t = pol_timeout(&p, now, 0, 0);
    CHECK(t == 100, "wakes: timeout %d 100 ms before the wheel hold ends (want 100)", t);
    free(j);
}
static void scenario_pointer(void)
{
    for (int engine = 1; engine >= 0; engine--) {
        policy p; pol_init(&p, W, H);
        pol_config(&p, "motion=adaptive ptr_rest_ms=1000");
        p.has_engine = engine;
        int64_t now = 1000;
        journal *j = calloc(1, sizeof *j);
        idle_until(&p, &now, 2000, j);
        j->n = 0;
        int how = -1, bad = 0;
        int runs = pointer_moves(&p, &now, 40, 300, j, &how, &bad);
        CHECK(runs == 0 && bad == 0 && gc16s(j) == 0, "pointer moving (engine %d): %d steps in a run, %d sends not DU, %d GC16",
              engine, runs, bad, gc16s(j));
        CHECK(min_cut(j) == (engine ? 2 : 0), "pointer moving (engine %d): A2 cut %d (want %d)", engine, min_cut(j), engine ? 2 : 0);
        CHECK(how == C_DITHER, "pointer moving (engine %d): commit %d (want C_DITHER)", engine, how);
        CHECK(!p.run_cut && p.state == S_CLEANUP, "pointer moving (engine %d): run_cut %d, state %s", engine, p.run_cut, state_name(p.state));
        /* rest: nothing before ptr_rest (> idle_ms), then one GC16 partial */
        j->n = 0;
        frames_to(&p, &now, now + p.ptr_rest - 40, j);
        CHECK(gc16s(j) == 0, "pointer resting (engine %d): GC16 before ptr_rest", engine);
        frames_to(&p, &now, now + 300, j);
        CHECK(count(j, WF_GC16, M_PART) == 1 && count(j, WF_GC16, M_AREA) == 0 && count(j, WF_GC16, M_SCREEN) == 0,
              "pointer at rest (engine %d): %d GC16 partials, %d flashes, %d full (want 1, 0, 0)", engine,
              count(j, WF_GC16, M_PART), count(j, WF_GC16, M_AREA), count(j, WF_GC16, M_SCREEN));
        CHECK(p.state == S_IDLE, "pointer at rest (engine %d): state %s", engine, state_name(p.state));
        /* a tooltip appears at the pointer right after its cleanup GC16 */
        j->n = 0;
        now += 40;
        burst(&p, &now, one((box){ 450, 620, 800, 700 }), (box){ 0, 0, W, H }, PM_AUTO, j);   /* (over its last place) */
        CHECK(p.state != S_RUN, "hover repaint after the pointer's GC16 (engine %d): a run", engine);
        free(j);
    }
    {   /* a touchpad's short pauses : moves with 400 ms rests (> idle_ms, < ptr_rest),
         * a hover repaint in between: no cleanup GC16 until the pointer really rests */
        policy p; pol_init(&p, W, H);
        pol_config(&p, "motion=adaptive ptr_rest_ms=1000");
        p.has_engine = 1;
        int64_t now = 1000;
        journal *j = calloc(1, sizeof *j);
        idle_until(&p, &now, 2000, j);
        j->n = 0;
        int how, bad = 0;
        for (int k = 0; k < 4; k++) {
            pointer_moves(&p, &now, 8, 300 + k * 60, j, &how, &bad);
            if (k == 1) burst(&p, &now, one((box){ 300, 580, 700, 640 }), (box){ 0, 0, W, H }, PM_AUTO, j);
            frames_to(&p, &now, now + 400, j);
        }
        CHECK(gc16s(j) == 0, "touchpad pauses: %d GC16 while the pointer pauses < ptr_rest (want 0)", gc16s(j));
        int t = pol_timeout(&p, now, 0, 0);
        CHECK(t > 0 && t <= p.ptr_rest, "touchpad pauses: timeout %d (want the rest left, <= %d)", t, p.ptr_rest);
        frames_to(&p, &now, now + p.ptr_rest, j);
        CHECK(gc16s(j) >= 1 && p.state == S_IDLE, "touchpad pauses: %d GC16 after the rest, state %s (want >= 1, idle)",
              gc16s(j), state_name(p.state));
        pol_config(&p, "ptr_rest_ms=0");
        CHECK(p.ptr_rest == 0, "ptr_rest_ms=0: %d", p.ptr_rest);
        free(j);
    }
    {   /* a hover highlight while the pointer still moves (its DUs out, no GC16 yet): no run either */
        policy p; pol_init(&p, W, H);
        pol_config(&p, "motion=adaptive ptr_rest_ms=1000");
        p.has_engine = 1;
        int64_t now = 1000;
        journal *j = calloc(1, sizeof *j);
        idle_until(&p, &now, 2000, j);
        int how, bad = 0;
        pointer_moves(&p, &now, 10, 300, j, &how, &bad);
        burst(&p, &now, one((box){ 300, 580, 700, 640 }), (box){ 0, 0, W, H }, PM_AUTO, j);
        CHECK(p.state != S_RUN, "hover highlight under a moving pointer: a run");
        free(j);
    }
    {   /* ghosting: N crossings of one tile count N, not 2N; the trail's tile flash comes at GHOST_MAX */
        policy p; pol_init(&p, W, H);
        pol_config(&p, "motion=adaptive ptr_rest_ms=1000");
        p.has_engine = 1;
        int64_t now = 1000;
        journal *j = calloc(1, sizeof *j);
        idle_until(&p, &now, 2000, j);
        box b = { 640, 640, 660, 660 };         /* inside tile (20, 20) */
        int tile = (640 / GTILE) * GTW + 640 / GTILE;
        for (int i = 0; i < GHOST_MAX - 1; i++) {
            boxlist ch = one(b);
            actions o = {0};
            pol_pointer(&p, &ch, now, &o);
            now += 30;
        }
        CHECK(p.ghost[tile] == GHOST_MAX - 1, "pointer ghost count %d after %d crossings (want once each)", p.ghost[tile], GHOST_MAX - 1);
        j->n = 0;
        frames_to(&p, &now, now + p.ptr_rest + 300, j);    /* (the cleanup waits for the pointer's rest) */
        CHECK(count(j, WF_GC16, M_AREA) == 0, "pointer: a tile flash below GHOST_MAX");
        for (int i = 0; i < 2; i++) { boxlist ch = one(b); actions o = {0}; pol_pointer(&p, &ch, now, &o); now += 30; }
        j->n = 0;
        frames_to(&p, &now, now + p.ptr_rest + 300, j);    /* (the cleanup waits for the pointer's rest) */
        CHECK(count(j, WF_GC16, M_AREA) == 1, "pointer: %d tile flashes past GHOST_MAX (want 1)", count(j, WF_GC16, M_AREA));
        free(j);
    }
}

int main(void)
{
    for (unsigned seed = 1; seed <= 200 && fails < 5; seed++) parity(seed, 5000);
    printf("parity: %s (200 traces x 5000 steps)\n", fails ? "FAILED" : "ok");
    int f0 = fails;
    scenario_firefox_scroll();
    scenario_page_turn();
    scenario_pages();
    scenario_typing();
    scenario_sleep();
    scenario_press();
    scenario_calm();
    scenario_moving();
    scenario_ticker();
    scenario_still_range();
    scenario_throttle();
    scenario_pace();
    scenario_adaptive();
    scenario_run_end();
    scenario_run_tail();
    scenario_quiet();
    scenario_windows();
    scenario_true_frame();
    scenario_popup();
    scenario_popup_swap();
    scenario_popup_close();
    scenario_popup_close_scroll();
    scenario_run_wait();
    scenario_busy();
    scenario_paced_anim();
    scenario_late_video();
    scenario_always();
    scenario_whole();
    scenario_rnote_ink();
    scenario_ink_debt();
    scenario_ink_timeout();
    scenario_pointer();
    scenario_wheel();
    scenario_wakes();
    scenario_stall();
    scenario_debounce();
    printf("scenarios: %s\n", fails > f0 ? "FAILED" : "ok");
    return fails != 0;
}
