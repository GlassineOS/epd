/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-policy — see epd-policy.h. Pure: no I/O, no clock, no X. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "epd-policy.h"

/* ---- geometry ------------------------------------------------------------ */
int box_empty(box b) { return b.x1 <= b.x0 || b.y1 <= b.y0; }
box box_union(box a, box b)
{
    if (box_empty(a)) return b;
    if (box_empty(b)) return a;
    return (box){ a.x0 < b.x0 ? a.x0 : b.x0, a.y0 < b.y0 ? a.y0 : b.y0,
                  a.x1 > b.x1 ? a.x1 : b.x1, a.y1 > b.y1 ? a.y1 : b.y1 };
}
box box_clip(box a, box c)
{
    box r = { a.x0 > c.x0 ? a.x0 : c.x0, a.y0 > c.y0 ? a.y0 : c.y0,
              a.x1 < c.x1 ? a.x1 : c.x1, a.y1 < c.y1 ? a.y1 : c.y1 };
    return box_empty(r) ? (box){0} : r;
}
int box_inside(box a, box c) { return a.x0 >= c.x0 && a.y0 >= c.y0 && a.x1 <= c.x1 && a.y1 <= c.y1; }
long box_area(box b) { return box_empty(b) ? 0 : (long)(b.x1 - b.x0) * (b.y1 - b.y0); }
static int box_near(box a, box b)
{
    return a.x0 < b.x1 + MERGE_PX && b.x0 < a.x1 + MERGE_PX &&
           a.y0 < b.y1 + MERGE_PX && b.y0 < a.y1 + MERGE_PX;
}

/* A list, not one bounding box: the union of a stroke and a far-away scrollbar
 * would be half the screen. */
void list_add(boxlist *l, box b)
{
    for (int i = 0; i < l->n; i++)
        if (box_near(l->b[i], b)) {
            b = box_union(b, l->b[i]);
            l->b[i] = l->b[--l->n];         /* removed; rescan all with the grown box */
            i = -1;
        }
    if (l->n == MAXBOX) {
        /* too fragmented: fold b into the rectangle whose union grows least */
        int best = 0;
        long bestgrow = -1;
        for (int i = 0; i < l->n; i++) {
            long grow = box_area(box_union(l->b[i], b)) - box_area(l->b[i]);
            if (bestgrow < 0 || grow < bestgrow) { bestgrow = grow; best = i; }
        }
        box u = box_union(l->b[best], b);
        l->b[best] = l->b[--l->n];
        list_add(l, u);                     /* may now touch others: re-merge */
        return;
    }
    l->b[l->n++] = b;
}

void list_fold(boxlist *l, int max)
{
    while (l->n > max) {
        int bi = 0, bj = 1;
        long best = -1;
        for (int i = 0; i < l->n; i++)
            for (int j = i + 1; j < l->n; j++) {
                long g = box_area(box_union(l->b[i], l->b[j])) - box_area(l->b[i]) - box_area(l->b[j]);
                if (best < 0 || g < best) { best = g; bi = i; bj = j; }
            }
        l->b[bi] = box_union(l->b[bi], l->b[bj]);
        l->b[bj] = l->b[--l->n];
    }
}

const char *state_name(int s)
{
    static const char *n[] = { "idle", "cleanup", "run", "paused", "asleep" };
    return s >= 0 && s <= S_ASLEEP ? n[s] : "?";
}

/* ---- actions ------------------------------------------------------------- */
static void act(actions *o, action a)
{
    if (o->n < MAXACT) o->a[o->n++] = a;
}
static void grab(actions *o, box b) { act(o, (action){ A_GRAB, b, 0, 0, 0 }); }

/* count DUs / reset on flashes, per tile touched */
static void ghost_note(policy *p, box b, int du)
{
    if (box_empty(b)) return;
    for (int ty = b.y0 / GTILE; ty <= (b.y1 - 1) / GTILE && ty < GTH; ty++)
        for (int tx = b.x0 / GTILE; tx <= (b.x1 - 1) / GTILE && tx < GTW; tx++) {
            uint8_t *g = &p->ghost[ty * GTW + tx];
            if (!du) *g = 0;
            else if (*g < 255) (*g)++;
        }
}

/* drop the boxes of l inside b (a flash drove every pixel there) */
static void list_drop_inside(boxlist *l, box b)
{
    int k = 0;
    for (int i = 0; i < l->n; i++)
        if (!box_inside(l->b[i], b)) l->b[k++] = l->b[i];
    l->n = k;
}

/* the ink debt's tiles (ink_clean=damage): mark what the pen touched, clear
 * what a GC16 drove (tiles wholly inside b) */
static void ink_tiles_mark(policy *p, box b)
{
    if (box_empty(b)) return;
    for (int ty = b.y0 / GTILE; ty <= (b.y1 - 1) / GTILE && ty < GTH; ty++)
        for (int tx = b.x0 / GTILE; tx <= (b.x1 - 1) / GTILE && tx < GTW; tx++)
            if (!p->ink_tile[ty * GTW + tx]) { p->ink_tile[ty * GTW + tx] = 1; p->ink_tiles++; }
}
static void ink_tiles_clear(policy *p, box b)
{
    if (!p->ink_tiles || box_empty(b)) return;
    for (int ty = (b.y0 + GTILE - 1) / GTILE; (ty + 1) * GTILE <= b.y1 && ty < GTH; ty++)
        for (int tx = (b.x0 + GTILE - 1) / GTILE; (tx + 1) * GTILE <= b.x1 && tx < GTW; tx++)
            if (p->ink_tile[ty * GTW + tx]) { p->ink_tile[ty * GTW + tx] = 0; p->ink_tiles--; }
}

static void send_n(policy *p, actions *o, box b, int wf, int mode, int n, int64_t now);
static void send(policy *p, actions *o, box b, int wf, int mode, int64_t now) { send_n(p, o, b, wf, mode, 0, now); }
/* a GC16 and whose it is (the run tail's bookkeeping, statechart doc):
 * G_FOREIGN ends the tail, G_OWN (the run end's, the tail's) keeps it,
 * G_NONE (a run's own GC16 step) is neither */
enum { G_NONE, G_FOREIGN, G_OWN };
static void send_gc16(policy *p, actions *o, box b, int mode, int who, int64_t now)
{
    send_n(p, o, b, WF_GC16, mode, 0, now);
    if (who != G_NONE) p->last_gc16 = now;
    if (who == G_OWN) p->last_own_gc16 = now;
}
static void send_n(policy *p, actions *o, box b, int wf, int mode, int n, int64_t now)
{
    if (wf == WF_DU) p->last_du = now;
    if (wf == WF_GC16) {
        p->gc16_box = now - p->gc16_box_t >= GC16_BUSY_MS ? b : box_union(p->gc16_box, b);
        p->gc16_box_t = now;
        list_drop_inside(&p->ink_debt, b);      /* (its greys are exact now) */
        list_drop_inside(&p->inkd, b);
        list_drop_inside(&p->inkr, b);
        ink_tiles_clear(p, b);
    }
    if (mode == M_SCREEN) { memset(p->ghost, 0, sizeof p->ghost); p->cand.n = p->stale.n = 0; }
    else if (wf == WF_DU) ghost_note(p, b, 1);
    else if (mode == M_AREA) {
        ghost_note(p, b, 0);
        list_drop_inside(&p->cand, b);
        list_drop_inside(&p->stale, b);
    }
    act(o, (action){ A_SEND, b, wf, mode, n });
}

/* the driver's DU frame cut; never changed while a DU may still be running
 * (the driver rebuilds the DU LUT in place) */
static void cut(policy *p, actions *o, int n, int64_t now)
{
    if (n == p->cut_cur || now - p->last_du < CUT_SAFE_MS) return;
    p->cut_cur = n;
    act(o, (action){ A_CUT, {0}, 0, 0, n });
}
static int cut_rest(const policy *p) { return p->cut_auto ? 0 : p->cut_fixed; }

/* forget everything pending (full refresh, pause, sleep) */
static void forget(policy *p)
{
    p->dirty.n = 0; p->inked.n = 0; p->ink_rec.n = 0; p->ink_held = 0;
    p->inkd.n = 0; p->inkr.n = 0; p->ink_debt.n = 0;
    memset(p->ink_tile, 0, sizeof p->ink_tile); p->ink_tiles = 0;
    p->cand.n = 0; p->stale.n = 0; p->popup.n = 0; p->ptr_dirty.n = 0;
    p->run_bursts = 0; p->run_box = (box){0}; p->run_win = (box){0};
    p->run_cut = 0; p->run_always = 0;
    p->m_w = 0; p->m_level = LV_GC16; p->m_down_since = 0; p->stall_t = 0;
}

static int awake(const policy *p) { return p->state != S_PAUSED && p->state != S_ASLEEP; }
static void to(policy *p, int s)
{
    if (s != p->state && p->log) p->log("state %s -> %s\n", state_name(p->state), state_name(s));
    p->state = s;
}
/* something now waits for a cleanup */
static void pending(policy *p) { if (p->state == S_IDLE) to(p, S_CLEANUP); }

/* ---- events -------------------------------------------------------------- */
void pol_init(policy *p, int W, int H)
{
    memset(p, 0, sizeof *p);
    p->W = W; p->H = H;
    p->banner = (box){ 0, H - BAR_H, W, H };
    p->burst_auto = 1; p->cut_auto = 1; p->dither_runs = 1;
    p->du_max = 0.03; p->flash_min = 0.35;
    p->idle_ms = IDLE_GC_MS;
    p->run_ms = 100; p->run_fold = 1; p->run_end = RUN_END_PARTIAL;
    p->pace = 1; p->eng_wait = -1;
    static const double lad[LV_N - 1] = { 0.12, 0.3, 0.8, 1.6, 3.2, LV_OFF };
    memcpy(p->lad, lad, sizeof lad);
    p->a2len[0] = 8; p->a2len[1] = 6; p->a2len[2] = 4; p->a2len[3] = 2;
    p->m_max = LV_A2_2;
    p->m_curve = 1;
    p->m_mu = log(M_STILL); p->c.touch.lift_t = -100000;
    p->clean_motion = CLEAN_DEBT_RATIO;
    p->busy_min = 0.03;
    p->last_busy = -100000;
    p->last_big = -100000;
    p->quiet_ms = 2000; p->quiet_after = 5000;
    p->run_tail = RUN_TAIL_MS;
    p->run_wait = 1.5; p->run_wait_max = 1200;
    p->ptr_rest = PTR_REST_MS;
    p->wheel_hold = WHEEL_HOLD_MS; p->c.wheel_t = -100000;
    p->debounce_ms = 150; p->debounce_gap = 500; p->debounce_cap = 150;   /*, 11-01 (eye test) */
    p->debounce_tap_ms = 300; p->debounce_tap_px = 100;
    p->stall_ms = STALL_MS;
    p->popup_fast = 2;                  /* popup=du (GC16 alone faded in) */
    p->popup_dither = 1;
    p->ink_redraw = 1;                  /* (no flash per stroke) */
    p->ink_clean = 1;
    p->last_popup = -100000;
    p->last_gone = -100000;
    p->cut_cur = -1;
    p->last_du = -1000;
    p->last_restack = -100000;
    p->want_full = 1;
    p->state = S_IDLE;
}

void pol_config(policy *p, const char *cmd)
{
    const char *cm = strstr(cmd, "clean_motion=");
    if (cm) p->clean_motion = atof(cm + 13);
    if ((cm = strstr(cmd, "busy=")) && (cm == cmd || cm[-1] == ' ' || cm[-1] == '\n')) {
        p->busy_min = atof(cm + 5);
        if (p->busy_min < 0) p->busy_min = 0;
    }
    const char *v;
    if (strstr(cmd, "cut=auto")) p->cut_auto = 1;
    else if ((v = strstr(cmd, "cut="))) { p->cut_auto = 0; p->cut_fixed = atoi(v + 4); }
    if (strstr(cmd, "dither=on")) p->dither_runs = 1;
    if (strstr(cmd, "dither=off")) p->dither_runs = 0;
    if (strstr(cmd, "burst=du")) p->burst_auto = 0;
    if (strstr(cmd, "burst=auto")) p->burst_auto = 1;
    if (strstr(cmd, "clean=partial")) p->clean_full = 0;
    if (strstr(cmd, "clean=full")) p->clean_full = 1;
    if ((v = strstr(cmd, "du_max="))) p->du_max = atof(v + 7);
    if ((v = strstr(cmd, "flash_min="))) p->flash_min = atof(v + 10);
    if ((v = strstr(cmd, "debounce_ms="))) { p->debounce_ms = atoi(v + 12); if (p->debounce_ms < 0) p->debounce_ms = 0; }
    if ((v = strstr(cmd, "debounce_gap="))) p->debounce_gap = atoi(v + 13);
    if ((v = strstr(cmd, "debounce_cap="))) p->debounce_cap = atoi(v + 13);
    if ((v = strstr(cmd, "debounce_tap_ms="))) p->debounce_tap_ms = atoi(v + 16);
    if ((v = strstr(cmd, "debounce_tap_px="))) p->debounce_tap_px = atoi(v + 16);
    if ((v = strstr(cmd, "debounce_osk="))) p->debounce_osk = atoi(v + 13) != 0;
    if ((v = strstr(cmd, "osk_up="))) p->osk_up = atoi(v + 7) != 0;
    if ((v = strstr(cmd, "idle_ms="))) { p->idle_ms = atoi(v + 8); if (p->idle_ms < 50) p->idle_ms = 50; }
    if ((v = strstr(cmd, "run_ms="))) p->run_ms = atoi(v + 7);
    if ((v = strstr(cmd, "run_wait="))) { p->run_wait = atof(v + 9); if (p->run_wait < 0) p->run_wait = 0; }
    if ((v = strstr(cmd, "run_wait_max="))) p->run_wait_max = atoi(v + 13);
    if ((v = strstr(cmd, "ptr_rest_ms="))) { p->ptr_rest = atoi(v + 12); if (p->ptr_rest < 0) p->ptr_rest = 0; }
    if ((v = strstr(cmd, "wheel_hold_ms="))) { p->wheel_hold = atoi(v + 14); if (p->wheel_hold < 0) p->wheel_hold = 0; }
    if ((v = strstr(cmd, "stall_ms="))) { p->stall_ms = atoi(v + 9); if (p->stall_ms < 0) p->stall_ms = 0; }
    if ((v = strstr(cmd, "run_fold="))) { p->run_fold = atoi(v + 9); if (p->run_fold < 1) p->run_fold = 1; }
    if (strstr(cmd, "motion=adaptive")) p->adaptive = 1;
    if (strstr(cmd, "motion=binary")) p->adaptive = 0;
    if ((v = strstr(cmd, "ladder="))) {
        double l[LV_N - 1];
        int k = sscanf(v + 7, "%lf,%lf,%lf,%lf,%lf,%lf", &l[0], &l[1], &l[2], &l[3], &l[4], &l[5]);
        if (k == LV_N - 2) l[LV_N - 2] = LV_OFF;     /* the five of before LV_A2_2: it stays off */
        int ok = k >= LV_N - 2 && l[0] > 0;
        for (int i = 1; ok && i < LV_N - 1; i++) ok = l[i] > l[i - 1];
        if (ok) memcpy(p->lad, l, sizeof l);
    }
    if ((v = strstr(cmd, "steps="))) {
        int a[4] = { 0, 0, 0, 2 };
        int k = sscanf(v + 6, "%d,%d,%d,%d", &a[0], &a[1], &a[2], &a[3]), ok = k >= 3;
        for (int i = 0; ok && i < 4; i++) ok = a[i] >= 2 && a[i] <= 10 && (!i || a[i] <= a[i - 1]);
        if (ok) memcpy(p->a2len, a, sizeof a);
    }
    if (strstr(cmd, "still=range")) p->still_range = 1;
    if (strstr(cmd, "still=auto")) p->still_range = 0;
    if (strstr(cmd, "attack=jump")) p->m_attack = 1;
    if (strstr(cmd, "attack=fused")) p->m_attack = 0;
    if (strstr(cmd, "curve=off")) p->m_curve = 0;
    if (strstr(cmd, "curve=on")) p->m_curve = 1;
    for (int mx = 0; mx < 2; mx++)
        if ((v = strstr(cmd, mx ? "mmax=" : "mmin="))) {
            static const char *const nm[LV_N] = { "gc16", "du4", "a2", "a2_8", "a2_6", "a2_4", "a2_2" };
            v += 5;
            size_t n = strcspn(v, " ,;\n");
            for (int i = 0; i < LV_N; i++)
                if (strlen(nm[i]) == n && !strncmp(v, nm[i], n)) *(mx ? &p->m_max : &p->m_min) = i;
        }
    if (strstr(cmd, "pace=engine")) p->pace = 1;
    if (strstr(cmd, "pace=fixed")) p->pace = 0;
    if (strstr(cmd, "run_end=flash")) p->run_end = RUN_END_FLASH;
    if (strstr(cmd, "run_end=partial")) p->run_end = RUN_END_PARTIAL;
    if ((v = strstr(cmd, "run_tail="))) { p->run_tail = atoi(v + 9); if (p->run_tail < 0) p->run_tail = 0; }
    if ((v = strstr(cmd, "quiet_ms="))) { p->quiet_ms = atoi(v + 9); if (p->quiet_ms < 0) p->quiet_ms = 0; }
    if ((v = strstr(cmd, "quiet_after="))) { p->quiet_after = atoi(v + 12); if (p->quiet_after < 0) p->quiet_after = 0; }
    if (strstr(cmd, "popup=fast")) p->popup_fast = 1;
    if (strstr(cmd, "popup=du")) p->popup_fast = 2;
    if (strstr(cmd, "popup=gc16")) p->popup_fast = 0;
    /* popup=du's DU: greys dithered to dots, or thresholded to clean black/white
     * (the GC16 behind then swings only the true greys) */
    if (strstr(cmd, "popupdu=dither")) p->popup_dither = 1;
    if (strstr(cmd, "popupdu=bw")) p->popup_dither = 0;
    if (strstr(cmd, "ink_redraw=du")) p->ink_redraw = 1;
    if (strstr(cmd, "ink_redraw=off")) p->ink_redraw = 0;
    if (strstr(cmd, "ink_clean=damage")) p->ink_clean = 1;
    if (strstr(cmd, "ink_clean=stroke")) p->ink_clean = 0;
}

/* cmd contains w as a whole word ("full", not "clean=full" / "flash_full=") */
static int cmd_word(const char *cmd, const char *w)
{
    size_t n = strlen(w);
    for (const char *c = cmd; (c = strstr(c, w)); c += n)
        if ((c == cmd || c[-1] == ' ' || c[-1] == '\n') && (!c[n] || c[n] == ' ' || c[n] == '\n')) return 1;
    return 0;
}

int pol_cmd(policy *p, const char *cmd, int64_t now, actions *out)
{
    int wake = 0;
    if (cmd_word(cmd, "full")) p->want_full = 1;
    if (strstr(cmd, "pause")) { forget(p); to(p, S_PAUSED); }
    if (strstr(cmd, "resume")) { to(p, S_IDLE); p->want_full = 1; }
    if (strstr(cmd, "sleep")) {
        /* the panel keeps its image without power: say it is asleep. glassine-powerd
         * suspends once the banner is on the panel (mainline: the engine holds
         * a wakeup source until its frames are sent; stock: ~1 s); updates stay
         * off until "wake" */
        box all = { 0, 0, p->W, p->H };
        act(out, (action){ A_GRAB_ALL, all, 0, 0, 0 });     /* the page as it is now */
        act(out, (action){ A_BANNER, p->banner, 0, 0, 0 });
        send_gc16(p, out, all, M_SCREEN, G_FOREIGN, now);   /* a full refresh, the banner in it */
        forget(p);
        to(p, S_ASLEEP);
    }
    if (strstr(cmd, "wake") && p->state == S_ASLEEP) { to(p, S_IDLE); wake = 1; }
    pol_config(p, cmd);
    if (strstr(cmd, "gamma=")) p->want_full = 1;
    if (strstr(cmd, "cut=")) cut(p, out, cut_rest(p), now);
    return wake;
}

void pol_banner(policy *p, box b)
{
    box all = { 0, 0, p->W, p->H };
    b = box_clip(b, all);
    p->banner = box_empty(b) ? (box){ 0, p->H - BAR_H, p->W, p->H } : b;
}

void pol_wake(policy *p, box changed, int64_t now, actions *out)
{
    send_gc16(p, out, box_union(p->banner, changed), M_PART, G_FOREIGN, now);
}

void pol_engine(policy *p, int wait_ms) { p->eng_wait = wait_ms; }
void pol_finger(policy *p, int64_t now, int down, double v)
{
    if (p->c.touch.down && !down) { p->c.touch.lift_t = now; p->c.touch.lift_v = p->c.touch.v; }
    p->c.touch.down = down;
    p->c.touch.v = down ? v : 0;
}

/* motion=adaptive: fuse this burst's evidence into the speed estimate, then
 * move the step (see epd-policy.h) */
static int motion_step(policy *p, int64_t t, double speed)
{
    double w0 = p->m_w * exp(-(double)(t - p->m_t) / M_TAU_MS);
    double sw = w0, sm = w0 * p->m_mu, w, mu;
    if (t - p->c.ink_t < POST_INK_MS) { w = 3; mu = log(M_STILL); }
    else if (speed > 0) { w = 4; mu = log(speed > M_STILL ? speed : M_STILL); }   /* the motion estimate (content first) */
    else if (p->c.touch.down) { w = 4; mu = log(p->c.touch.v > M_STILL ? p->c.touch.v : M_STILL); }
    else if (t - p->c.touch.lift_t < M_LIFT_MS) {     /* after a lift: a fling slowing down (or the content stopped) */
        double d = exp(-(double)(t - p->c.touch.lift_t) / M_FLING_MS), v = p->c.touch.lift_v * d;
        w = 2 * d + 0.5; mu = log(v > M_STILL ? v : M_STILL);
    } else { w = 1; mu = log(M_NOFINGER); }
    sw += w; sm += w * mu;
    if (p->m_attack && mu > p->m_mu) { sw = w; sm = w * mu; }   /* attack=jump: speeding up is believed at once */
    p->m_mu = sm / sw;
    p->m_w = sw < M_WMAX ? sw : M_WMAX;
    p->m_t = t;
    int up = 0, dn = 0;
    for (int k = 0; k < LV_N - 1; k++) {
        if (p->m_mu > log(p->lad[k]) + M_HYST / 2) up = k + 1;
        if (p->m_mu > log(p->lad[k]) - M_HYST / 2) dn = k + 1;
    }
    if (up > p->m_level) { p->m_level = up; p->m_down_since = 0; }
    else if (dn < p->m_level) {
        if (!p->m_down_since) p->m_down_since = t;
        else if (t - p->m_down_since >= M_DOWN_MS) { p->m_level = dn; p->m_down_since = 0; }
    } else p->m_down_since = 0;
    int lv = p->m_level > p->m_min ? p->m_level : p->m_min;
    if (lv > p->m_max) lv = p->m_max > p->m_min ? p->m_max : p->m_min;     /* (mmin wins a clash) */
    if (p->log) p->log("motion %.2f screens/s (w %.1f) -> step %d\n", exp(p->m_mu), p->m_w, lv);
    return lv;
}
static int wheel_held(const policy *p, int64_t now);
/* a contact, pen or wheel in use, a popup's paint due: the first frame must not wait */
static int db_busy(const policy *p, int64_t now)
{
    return p->c.drag || wheel_held(p, now) || (p->c.touch.down && p->c.touch.v >= DB_FINGER_V) ||
           now - p->c.ink_t < DB_INK_MS;
}
static int db_may_start(const policy *p, int64_t now)
{
    return p->debounce_ms > 0 && p->whole && p->state != S_RUN && awake(p) && (!p->osk_up || p->debounce_osk) &&
           !db_busy(p, now) && now - p->last_dmg >= p->debounce_gap && now - p->last_burst >= p->debounce_gap &&
           (!p->popup.n || now - p->last_popup >= POPUP_MS) && now - p->last_gone >= POPUP_MS;
}
int pol_debounce_left(const policy *p, int64_t now)
{
    if (!p->db_t0 || p->debounce_ms <= 0 || db_busy(p, now) || (p->osk_up && !p->debounce_osk)) return 0;
    int64_t end = p->last_dmg + p->debounce_ms, cap = p->db_t0 + p->debounce_cap;
    if (cap < end) end = cap;
    return end > now ? (int)(end - now) : 0;
}
void pol_tap(policy *p, int64_t now, int down, int x, int y)
{
    p->tap_on = down;
    if (down) { p->tap_x = x; p->tap_y = y; } else p->tap_up = now;
}
int pol_debounce_clip(const policy *p, box dmg, int64_t now, box *out)
{
    if (pol_debounce_left(p, now) <= 0) return 0;
    if (!p->tap_on && (!p->tap_up || now - p->tap_up > p->debounce_tap_ms)) return 0;
    int r = p->debounce_tap_px;
    box c = box_clip(dmg, (box){ p->tap_x - r, p->tap_y - r, p->tap_x + r, p->tap_y + r });
    if (box_empty(c)) return 0;
    *out = c;
    return 1;
}

void pol_damage(policy *p, int64_t now)
{
    if (db_may_start(p, now)) { p->db_t0 = now; if (p->log) p->log("debounce: holding the first frame\n"); }
    int64_t g = now - p->last_dmg;
    if (p->last_dmg > 0 && g >= RW_MIN_GAP_MS && g <= p->run_wait_max) {
        p->rw_t[p->rw_i] = now; p->rw_gap[p->rw_i] = (int)g;
        p->rw_i = (p->rw_i + 1) % RW_N;
    }
    p->last_dmg = now;
}

/* how long a quiet spell ends what is dirty (see RW_N in epd-policy.h);
 * -1: not now (a finger is down in a run) */
static int clean_wait_run(const policy *p, int64_t now);
/* a wheel scroll (touchpad, mouse) in progress: its "finger down" (no finger
 * to lift, so the hold: two-finger swipes come 0.3-1.2 s apart, each
 * swipe was its own run, ended by a full-window GC16 flash the next one waited on) */
static int wheel_held(const policy *p, int64_t now) { return p->wheel_hold > 0 && now - p->c.wheel_t < p->wheel_hold; }
/* the run is held whatever the quiet: a finger on the glass, a scroll on the pad */
static int holds_run(const policy *p, int64_t now) { return p->c.touch.down || wheel_held(p, now); }
static int clean_wait(const policy *p, int64_t now)
{
    int w = clean_wait_run(p, now);
    /* ink_redraw: an app re-renders a stroke more than once (Rnote: ~0.46 s
     * after the lift, again ~50 ms and ~0.4 s later); the one cleanup comes
     * after the last of them */
    if (w >= 0 && p->ink_held) {
        int64_t e = p->c.ink_t + POST_INK_MS - p->last_dmg;
        if (e > w) w = (int)e;
    }
    return w;
}
/* Content.stalled (statechart doc): the run's last frame still moved fast and
 * the frames stopped. Input during the gap already holds the run (holds_run);
 * this covers a fling or an animation after the fingers lifted: an app that
 * misses frames at speed is no stop (a stop decays to zero first) */
static int stalled(const policy *p) { return p->state == S_RUN && p->stall_ms > 0 && p->stall_t > 0; }
static int clean_wait_run1(const policy *p, int64_t now);
static int clean_wait_run(const policy *p, int64_t now)
{
    int w = clean_wait_run1(p, now);
    if (w >= 0 && stalled(p) && w < p->stall_ms) w = p->stall_ms;
    return w;
}
static int clean_wait_run1(const policy *p, int64_t now)
{
    int64_t act = p->c.input_t > p->last_busy ? p->c.input_t : p->last_busy;     /* busy content = input */
    if (p->run_wait <= 0 || p->state != S_RUN || now - act >= RW_INPUT_MS) return p->idle_ms;
    if (holds_run(p, now)) return -1;
    int g = 0;
    for (int i = 0; i < RW_N; i++)
        if (p->rw_gap[i] > g && now - p->rw_t[i] <= RW_SPAN_MS) g = p->rw_gap[i];
    int w = (int)(p->run_wait * g);
    if (w > p->run_wait_max) w = p->run_wait_max;
    return w > p->idle_ms ? w : p->idle_ms;
}
void pol_input(policy *p, int64_t now) { p->c.input_t = now; }
void pol_wheel(policy *p, int64_t now) { p->c.wheel_t = now; p->c.input_t = now; }

void pol_window(policy *p, box b, int64_t now)
{
    b = box_clip(b, (box){ 0, 0, p->W, p->H });
    if (!awake(p) || b.x1 - b.x0 < WIN_MIN || b.y1 - b.y0 < WIN_MIN) return;
    list_add(&p->cand, b);
    p->last_restack = now;
    if (p->log) p->log("window change %d,%d-%d,%d\n", b.x0, b.y0, b.x1, b.y1);
}

void pol_popup(policy *p, box b, int64_t now)
{
    b = box_clip(b, (box){ 0, 0, p->W, p->H });
    if (!awake(p) || b.x1 - b.x0 < WIN_MIN || b.y1 - b.y0 < WIN_MIN) return;
    if (now - p->last_popup >= POPUP_MS) p->popup.n = 0;
    list_add(&p->popup, b);
    p->last_popup = now;
    if (p->log) p->log("popup %d,%d-%d,%d\n", b.x0, b.y0, b.x1, b.y1);
}

/* a popup disappearing: what it uncovered repaints in one GC16 partial
 * as a window change it got a DU run in a "fast" app, then a stale flash, then
 * a GC16 partial: ~2 s). Everything that changes right after, not only inside
 * the popup's box: the app below repaints wider than the uncovered part. */
void pol_popup_gone(policy *p, box b, int64_t now)
{
    b = box_clip(b, (box){ 0, 0, p->W, p->H });
    if (!awake(p) || b.x1 - b.x0 < WIN_MIN || b.y1 - b.y0 < WIN_MIN) return;
    p->last_gone = now;
    p->last_restack = now;
    if (p->log) p->log("popup gone %d,%d-%d,%d\n", b.x0, b.y0, b.x1, b.y1);
}

/* a workspace snapshot was painted with a flash and the apps have settled: the
 * whole panel is clean, so what the next read finds (a scrollbar that faded, a
 * handle) is a small correction — a GC16 partial of exactly that, like a
 * popup's paint, no stale flash for the window changes of the switch */
void pol_settled(policy *p, int64_t now)
{
    p->cand.n = 0; p->stale.n = 0;
    p->popup.n = 0;
    pol_popup(p, (box){ 0, 0, p->W, p->H }, now);
}

/* while a tile handle is dragged, what else changes (glassine-shell's ratio pill
 * beside the bar, the snap rail) is motion too: dithered DU, no GC16 per step;
 * the drag's end is corrected in one GC16 (pol_settled) */
void pol_motion(policy *p, int on) { p->c.drag = on; }

/* a popup's paint: a rect inside a popup mapped within POPUP_MS, the rest only
 * small ones beside it (glassine-shell repaints the bar button that opened it in the
 * same burst, that made the drawer's second paint a DU run) */
static int in_popup(const policy *p, const boxlist *ch, int64_t now)
{
    if (!p->popup.n || now - p->last_popup >= POPUP_MS) return 0;
    int inside = 0;
    for (int i = 0; i < ch->n; i++) {
        int in = 0;
        for (int k = 0; k < p->popup.n && !in; k++) in = box_inside(ch->b[i], p->popup.b[k]);
        if (in) inside = 1;
        else if (box_area(ch->b[i]) > POPUP_BESIDE) return 0;
    }
    return inside;
}

/* the pen inked b: what it inked since a POST_INK_MS gap (ink_redraw) */
static void ink_note(policy *p, box b, int64_t now)
{
    if (now - p->c.ink_t >= POST_INK_MS) p->ink_rec.n = 0;
    list_add(&p->ink_rec, b);
    p->c.ink_t = now;
    ink_tiles_mark(p, b);
}

/* the changed rects inside what the pen just inked (+- INK_AA_PX), the others
 * small ones (<= du_max: a DU anyway; Rnote its undo button and
 * scrollbar change with every stroke); at least one inside */
static int in_ink(const policy *p, const boxlist *ch)
{
    const double screen = (double)p->W * p->H;
    int inside = 0;
    for (int i = 0; i < ch->n; i++) {
        int in = 0;
        for (int k = 0; k < p->ink_rec.n && !in; k++) {
            box r = p->ink_rec.b[k];
            in = box_inside(ch->b[i], (box){ r.x0 - INK_AA_PX, r.y0 - INK_AA_PX, r.x1 + INK_AA_PX, r.y1 + INK_AA_PX });
        }
        if (in) inside = 1;
        else if (box_area(ch->b[i]) / screen > p->du_max) return 0;
    }
    return inside;
}

void pol_ink(policy *p, box b, int64_t now, actions *out)
{
    if (!awake(p) || box_empty(b)) return;
    if (p->ink_clean) {                 /* (no ghost count per piece: pol_inked) */
        p->last_du = now;
        act(out, (action){ A_SEND, b, WF_DU, M_PART, 0 });
    } else
        send(p, out, b, WF_DU, M_PART, now);
    list_add(&p->inked, b);
    ink_note(p, b, now);
    pending(p);
}

void pol_inked(policy *p, box b, int64_t now)
{
    if (!awake(p) || box_empty(b)) return;
    p->last_du = now;               /* as send does for the DU the pen thread sent */
    /* (ink_clean=damage: a stroke counts once, by its cleanup DU, not each of
     * its overlapping pieces: 6 strokes over a spot made 30 counts = a tile
     * flash mid-writing) */
    if (!p->ink_clean) ghost_note(p, b, 1);
    list_add(&p->inked, b);
    ink_note(p, b, now);
    pending(p);
}

void pol_recut(policy *p, int64_t now, actions *out)
{
    if (p->state != S_RUN) cut(p, out, cut_rest(p), now);
}

/* any tile of b DU'd >= GHOST_MAX times since its last flash */
static int ghosted(const policy *p, box b)
{
    for (int ty = b.y0 / GTILE; ty <= (b.y1 - 1) / GTILE && ty < GTH; ty++)
        for (int tx = b.x0 / GTILE; tx <= (b.x1 - 1) / GTILE && tx < GTW; tx++)
            if (p->ghost[ty * GTW + tx] >= GHOST_MAX) return 1;
    return 0;
}

/* the least time between two bursts right now: run_ms in a motion run; with no
 * input (touch, pen ink) for quiet_after ms, quiet_ms — whatever repaints on its
 * own (clock, htop, a progress bar) then collects into one update per quiet_ms
 * instead of one per repaint (a terminal cost ~47 ms of kernel CPU every
 * 1-1.5 s). The next touch lifts it at once. */
static int engine_paced(const policy *p) { return p->state == S_RUN && p->pace && p->eng_wait >= 0; }
/* large changes arriving densely: not "quiet" (see QUIET_DENSE_N) */
static int quiet_lifted(const policy *p, int64_t now)
{
    if (p->busy_min <= 0 || now - p->last_big > QUIET_DENSE_MS + p->quiet_ms) return 0;
    int n = 0;
    for (int i = 0; i < RW_N; i++)
        if (p->rw_t[i] && now - p->rw_t[i] <= QUIET_DENSE_MS) n++;
    return n >= QUIET_DENSE_N;
}
/* the user's last input or pen ink (quiet pacing, the ink debt's pause) */
static int64_t last_active(const policy *p) { return p->c.input_t > p->c.ink_t ? p->c.input_t : p->c.ink_t; }
static int min_gap(const policy *p, int64_t now)
{
    int gap = p->state != S_RUN ? 0 : engine_paced(p) ? RUN_FLOOR_MS : p->run_ms;
    int64_t input = last_active(p);
    /* busy content in a run = input (as clean_wait): a video kept its run but
     * fell to one frame per quiet_ms 5 s after the last touch */
    if (p->state == S_RUN && p->last_busy > input) input = p->last_busy;
    if (p->quiet_ms > gap && now - input >= p->quiet_after && !quiet_lifted(p, now)) gap = p->quiet_ms;
    return gap;
}

/* ms until a burst may be read (0 = now): the gap since the last one and, in an
 * engine-paced run, until the engine can start it — read earlier, the frame
 * waits in the queue and is replaced by the next one (or shows old content) */
static int hold_ms(const policy *p, int64_t now)
{
    int64_t w = min_gap(p, now) - (now - p->last_burst);
    if (engine_paced(p) && p->eng_wait > w) w = p->eng_wait;
    return w > 0 ? (int)w : 0;
}
static int throttled(const policy *p, int64_t now) { return hold_ms(p, now) > 0; }
int pol_quiet_held(const policy *p, int64_t now)
{
    if (p->state == S_RUN || p->quiet_ms <= 0) return 0;
    int64_t input = last_active(p);
    return now - input >= p->quiet_after && !quiet_lifted(p, now) && hold_ms(p, now) > 0;
}
/* damage -> read wait (COALESCE_MS; none for whole frames in a run) */
static int coalesce(const policy *p) { return p->whole && p->state == S_RUN ? 0 : COALESCE_MS; }

/* still=range with both ends of the range together: one mode for everything */
/* a level's DU frame count: the cut steps' (steps=), 0 = the uncut DU / DU4 / GC16 */
static int a2n(const policy *p, int lv) { return lv >= LV_A2_8 && lv < LV_N ? p->a2len[lv - LV_A2_8] : 0; }

static int single(const policy *p)
{
    return p->still_range && p->adaptive && p->has_engine && p->m_min == p->m_max;
}

/* the range's fast end (mmin wins a clash, as in motion_step) */
static int top_step(const policy *p) { return p->m_max > p->m_min ? p->m_max : p->m_min; }

/* a repaint at step lv outside a run (still=range, a press with a floor):
 * no run_cut, so its cleanup stays a partial */
static int floor_paint(policy *p, actions *out, boxlist *ch, int lv, int64_t t)
{
    list_fold(ch, p->run_fold);
    for (int i = 0; i < ch->n; i++) {
        if (lv == LV_GC16) { send_gc16(p, out, ch->b[i], M_PART, G_FOREIGN, t); continue; }
        send_n(p, out, ch->b[i], lv == LV_DU4 ? WF_DU4 : WF_DU, M_PART, a2n(p, lv), t);
        if (lv != LV_DU4) ghost_note(p, ch->b[i], 1);
        list_add(&p->dirty, ch->b[i]);
    }
    if (lv == LV_GC16) return C_ALL;
    pending(p);
    return lv == LV_DU4 ? C_Q4 : p->dith ? C_DITHER : C_THRESH;
}

/* the exact area of the union of n boxes: a sweep over the x edges, the y
 * intervals merged per slab (n <= ~64: microseconds) */
static long union_area(const box *b, int n)
{
    int xs[2 * MV_N * MV_K + 2 * MAXBOX + 4], nx = 0;
    if (n > MV_N * MV_K + MAXBOX + 2) n = MV_N * MV_K + MAXBOX + 2;
    for (int i = 0; i < n; i++) if (!box_empty(b[i])) { xs[nx++] = b[i].x0; xs[nx++] = b[i].x1; }
    for (int i = 1; i < nx; i++) for (int j = i; j > 0 && xs[j] < xs[j - 1]; j--) { int t = xs[j]; xs[j] = xs[j - 1]; xs[j - 1] = t; }
    long a = 0;
    for (int k = 0; k + 1 < nx; k++) {
        int x0 = xs[k], x1 = xs[k + 1];
        if (x1 == x0) continue;
        int y0[MV_N * MV_K + MAXBOX + 2], y1[MV_N * MV_K + MAXBOX + 2], ny = 0;
        for (int i = 0; i < n; i++)
            if (!box_empty(b[i]) && b[i].x0 <= x0 && b[i].x1 >= x1) {
                int j = ny++;
                for (; j > 0 && y0[j - 1] > b[i].y0; j--) { y0[j] = y0[j - 1]; y1[j] = y1[j - 1]; }
                y0[j] = b[i].y0; y1[j] = b[i].y1;
            }
        long h = 0;
        for (int i = 0, cy0 = 0, cy1 = 0, open = 0; i <= ny; i++) {
            if (i < ny && open && y0[i] <= cy1) { if (y1[i] > cy1) cy1 = y1[i]; continue; }
            if (open) h += cy1 - cy0;
            if (i < ny) { cy0 = y0[i]; cy1 = y1[i]; open = 1; }
        }
        a += h * (x1 - x0);
    }
    return a;
}

/* the idle cleanup: what DU left out (greys, pen previews) is brought in right
 * before the GC16 that can reach it */
static void cleanup(policy *p, int64_t now, actions *out)
{
    const double screen = (double)p->W * p->H;
    if (p->state == S_RUN) p->last_run_end = now;
    /* the cleanup's GC16s are the tail's own (the scrollbar fade's cleanup keeps
     * it) only at a run end or while the tail still holds: after a foreign GC16
     * had ended it, a cleanup restarted it (not intended) */
    int own = p->state == S_RUN || p->last_gc16 <= p->last_own_gc16;
    int who = own ? G_OWN : G_FOREIGN;
    /* one mode for everything (single): no optical cleanup — what waits is
     * dropped, the ghost counts stay (another app's cleanup takes them). The
     * engine's DC healing is not a cleanup: it rides the paints as before */
    if (single(p)) {
        if (p->log) p->log("single mode: cleanup dropped (%d dirty, %d stale)\n", p->dirty.n, p->stale.n);
        p->stale.n = 0;
        p->dirty.n = 0; p->ink_held = 0; p->inkd.n = 0; p->inkr.n = 0; p->ptr_dirty.n = 0;
        p->run_bursts = 0; p->run_box = (box){0}; p->run_win = (box){0};
        p->run_cut = 0; p->run_always = 0;
        p->m_w = 0; p->m_level = LV_GC16; p->m_down_since = 0;
        p->stall_t = 0;
        to(p, S_IDLE);
        cut(p, out, cut_rest(p), now);
        return;
    }
    /* window changes: a local flash where the old content was replaced (it
     * also covers the run end and DU cleanups inside it) */
    if (p->stale.n) {
        list_fold(&p->stale, 3);
        for (int i = 0; i < p->stale.n; i++) {
            box b = p->stale.b[i];
            if (p->log) p->log("stale flash %d,%d-%d,%d\n", b.x0, b.y0, b.x1, b.y1);
            grab(out, b);
            list_drop_inside(&p->dirty, b);
            list_drop_inside(&p->ptr_dirty, b);
            if (box_inside(p->run_box, b)) { p->run_bursts = 0; p->run_box = (box){0}; }
            send_gc16(p, out, b, M_AREA, who, now);
        }
        p->stale.n = 0;
    }
    /* a motion run ends in ONE GC16 over what it touched inside its window:
     * a partial one (~0.5 s, no flash: brings the dithered greys to their real
     * level), or — where the area is ghosted (GHOST_MAX DUs since its last
     * flash: ~3 s of scrolling), or with run_end=flash — a flash (~1 s, drives
     * every pixel; a partial skips those the controller believes are right) */
    if ((p->run_bursts >= RUN_BURSTS && box_area(p->run_box) / screen > p->du_max) || (p->run_cut && !box_empty(p->run_box))) {
        int flash = p->run_end == RUN_END_FLASH || ghosted(p, p->run_box) || p->run_cut;
        grab(out, p->run_box);
        send_gc16(p, out, p->run_box, flash ? M_AREA : M_PART, who, now);
        int k = 0;                      /* DU areas it doesn't cover: partial below */
        for (int i = 0; i < p->dirty.n; i++)
            if (!box_inside(p->dirty.b[i], p->run_box)) p->dirty.b[k++] = p->dirty.b[i];
        p->dirty.n = k;
        list_drop_inside(&p->ptr_dirty, p->run_box);
    }
    /* ink_clean=damage: the strokes' cleanup is a DU of the app's content (it
     * replaces the pen's previews, no flash); the greys go to the ink debt */
    if (p->inkd.n || p->inkr.n) {
        for (int i = 0; i < p->inkr.n; i++) list_drop_inside(&p->inkd, p->inkr.b[i]);   /* (re-rendered since) */
        list_fold(&p->inkd, 2);
        for (int i = 0; i < p->inkd.n; i++) {
            grab(out, p->inkd.b[i]);
            send(p, out, p->inkd.b[i], WF_DU, M_PART, now);
            list_add(&p->ink_debt, p->inkd.b[i]);
        }
        for (int i = 0; i < p->inkr.n; i++) list_add(&p->ink_debt, p->inkr.b[i]);
        if (p->log) p->log("ink cleanup: %d DU%s (no GC16), the ink debt %d box%s, %d tiles of pen\n", p->inkd.n,
                           p->inkd.n == 1 ? "" : "s", p->ink_debt.n, p->ink_debt.n == 1 ? "" : "es", p->ink_tiles);
        p->inkd.n = p->inkr.n = 0;
    }
    if (p->dirty.n) {
        list_fold(&p->dirty, 2);
        for (int i = 0; i < p->dirty.n; i++) {
            grab(out, p->dirty.b[i]);
            send_gc16(p, out, p->dirty.b[i], p->clean_full || p->run_cut ? M_AREA : M_PART, who, now);
            list_drop_inside(&p->ptr_dirty, p->dirty.b[i]);
        }
    }
    /* tiles DU'd often since their last flash: clear the ghosting there only */
    boxlist gh = {0};
    for (int ty = 0; ty < GTH; ty++)
        for (int tx = 0; tx < GTW; tx++)
            if (p->ghost[ty * GTW + tx] >= GHOST_MAX)
                list_add(&gh, box_clip((box){ tx * GTILE, ty * GTILE, (tx + 1) * GTILE, (ty + 1) * GTILE },
                                       (box){ 0, 0, p->W, p->H }));
    list_fold(&gh, 3);
    for (int i = 0; i < gh.n; i++) {
        if (p->log) p->log("ghost clear %d,%d-%d,%d\n", gh.b[i].x0, gh.b[i].y0, gh.b[i].x1, gh.b[i].y1);
        grab(out, gh.b[i]);
        send_gc16(p, out, gh.b[i], M_AREA, who, now);
        list_drop_inside(&p->ptr_dirty, gh.b[i]);
    }
    /* the pointer's paints (pol_pointer): ONE GC16 partial where it went, the
     * pointer crisp at rest; sent past send: not gc16_box, not last_gc16 (a
     * hover repaint right after it must not see a GC16 in flight = a run) */
    if (p->ptr_dirty.n) {
        list_fold(&p->ptr_dirty, 2);
        for (int i = 0; i < p->ptr_dirty.n; i++) {
            grab(out, p->ptr_dirty.b[i]);
            act(out, (action){ A_SEND, p->ptr_dirty.b[i], WF_GC16, M_PART, 0 });
        }
        p->ptr_dirty.n = 0;
    }
    p->dirty.n = 0; p->ink_held = 0;
    p->run_bursts = 0; p->run_box = (box){0}; p->run_win = (box){0};
    p->run_cut = 0; p->run_always = 0;
    p->m_w = 0; p->m_level = LV_GC16; p->m_down_since = 0;
    p->stall_t = 0;
    to(p, S_IDLE);
    cut(p, out, cut_rest(p), now);
    if (own) p->last_own_gc16 = p->last_gc16;  /* (also with nothing sent: a run end keeps a foreign GC16 of the run as the tail's) */
}

/* damage goes on, but over a small area against what waits for the cleanup
 * (clean_motion x the area): everything outside the changes of the last w ms
 * has been quiet that long -> clean now (see MV_N in epd-policy.h) */
static int small_motion(policy *p, int64_t now, int w)
{
    if (p->clean_motion <= 0) return 0;
    if (now - p->last_burst > w) return 0;      /* (then the quiet rule decides) */
    if (w > (MV_N - 1) * MV_BUCKET_MS) return 0;   /* the buckets don't reach back that far */
    box mv[MV_N * MV_K];
    int n = 0;
    for (int i = 0; i < MV_N; i++)
        if (p->mv_n[i] && p->mv_t[i] + MV_BUCKET_MS > now - w)
            for (int k = 0; k < p->mv_n[i]; k++) mv[n++] = p->mv_b[i][k];
    long ma = union_area(mv, n);
    if (!ma) return 0;
    box debt[MAXBOX * 2 + 1];
    int nd = 0;
    debt[nd++] = p->run_box;
    for (int i = 0; i < p->dirty.n; i++) debt[nd++] = p->dirty.b[i];
    for (int i = 0; i < p->stale.n; i++) debt[nd++] = p->stale.b[i];
    long da = union_area(debt, nd);
    if ((double)da < p->clean_motion * ma) return 0;
    if (p->log) p->log("cleanup despite motion: %ld px waiting, only %ld px moving (clean_motion=%g)\n",
                       da, ma, p->clean_motion);
    return 1;
}

/* a burst with changes: busy content (see BUSY_N in epd-policy.h)? The motion
 * buckets must hold this burst already */
static int busy_seen(policy *p, int64_t t)
{
    p->bz_t[p->bz_i] = t;
    p->bz_i = (p->bz_i + 1) % BUSY_N;
    int64_t oldest = p->bz_t[p->bz_i];          /* the slot written BUSY_N bursts ago */
    if (p->busy_min <= 0 || !oldest || t - oldest > BUSY_SPAN_MS) return 0;
    box mv[MV_N * MV_K];
    int n = 0;
    for (int i = 0; i < MV_N; i++)
        if (p->mv_n[i] && p->mv_t[i] + MV_BUCKET_MS > t - BUSY_SPAN_MS)
            for (int k = 0; k < p->mv_n[i]; k++) mv[n++] = p->mv_b[i][k];
    return union_area(mv, n) >= p->busy_min * p->W * p->H;
}

/* the ink debt is worth a GC16 now (INK_DEBT_PAUSE_MS in epd-policy.h) */
static int ink_debt_due(const policy *p)
{
    if (p->ink_tiles >= INK_DEBT_AREA * GTW * GTH) return 1;
    for (int i = 0; i < p->ink_debt.n; i++) {
        box b = p->ink_debt.b[i];
        for (int ty = b.y0 / GTILE; ty <= (b.y1 - 1) / GTILE && ty < GTH; ty++)
            for (int tx = b.x0 / GTILE; tx <= (b.x1 - 1) / GTILE && tx < GTW; tx++)
                if (p->ghost[ty * GTW + tx] >= INK_DEBT_GHOST) return 1;
    }
    return 0;
}

static void ink_debt_clean(policy *p, int64_t now, actions *out)
{
    list_fold(&p->ink_debt, 2);
    if (p->log) p->log("ink debt: one GC16 partial over %d box%s (%ld px)\n", p->ink_debt.n,
                       p->ink_debt.n == 1 ? "" : "es", union_area(p->ink_debt.b, p->ink_debt.n));
    boxlist d = p->ink_debt;
    for (int i = 0; i < d.n; i++) {
        grab(out, d.b[i]);
        send_gc16(p, out, d.b[i], M_PART, G_FOREIGN, now);   /* (send drops it from the debt) */
    }
    p->ink_debt.n = 0;
    memset(p->ink_tile, 0, sizeof p->ink_tile); p->ink_tiles = 0;
    p->last_gc16 = now;             /* (also with no box: due by ink_tiles alone) */
    p->ink_debt_cleans++;
}

/* the idle cleanup has work (pol_step runs it, pol_timeout wakes for it) */
static int clean_waits(const policy *p)
{
    return p->dirty.n || p->inkd.n || p->inkr.n || p->run_bursts || p->stale.n || p->ptr_dirty.n;
}
/* the pen is up and nothing of its strokes waits (previews, their cleanup, a re-render) */
static int pen_quiet(const policy *p, int inking) { return !inking && !p->inked.n && !p->inkd.n && !p->inkr.n; }
/* the ink debt waits for its GC16: due at ink_debt_at if ink_debt_due */
static int ink_debt_waits(const policy *p, int inking, int pend_n)
{
    return p->ink_debt.n && p->state != S_RUN && !pend_n && pen_quiet(p, inking);
}
/* when the quiet a cleanup waits for began: the last damage, in an engine-paced
 * run when the engine was free (eng_held), and the pointer's rest first */
static int64_t quiet_from(const policy *p)
{
    int64_t q = p->eng_held > p->last_dmg && p->state == S_RUN ? p->eng_held : p->last_dmg;
    if (p->last_ptr > q) q = p->last_ptr;
    return q;
}
/* the deadlines pol_step fires and pol_timeout wakes for, each computed once
 * (doc): ms until due, <= 0 = due */
/* CLEAN: the cleanup's quiet w (clean_wait, >= 0) has passed */
static int64_t clean_left(const policy *p, int64_t now, int w) { return w - (now - quiet_from(p)); }
/* ... and not before the pointer rested */
static int64_t ptr_left(const policy *p, int64_t now) { return p->ptr_rest - (now - p->last_ptr); }
static int ptr_rested(const policy *p, int64_t now) { return ptr_left(p, now) <= 0; }
static int64_t ink_debt_at(const policy *p)
{
    return last_active(p) + INK_DEBT_PAUSE_MS;
}
/* INK_DEBT's deadline: the pen paused INK_DEBT_PAUSE_MS (as clean_left: <= 0 = due) */
static int64_t ink_debt_left(const policy *p, int64_t now) { return ink_debt_at(p) - now; }

int pol_step(policy *p, int64_t now, int inking, int pend_n, int64_t first_dmg, actions *out)
{
    switch (p->state) {
    case S_PAUSED:
    case S_ASLEEP:
        return pend_n ? STEP_DROP : STEP_NONE;
    case S_IDLE:
    case S_CLEANUP:
    case S_RUN:
        /* pen previews: once the pen is up, the cleanup replaces them with X's
         * content (the shadow holds the black preview, so they compare changed) */
        if (p->inked.n && !inking && now - p->c.ink_t >= INK_SETTLE_MS) {
            for (int i = 0; i < p->inked.n; i++) list_add(p->ink_clean ? &p->inkd : &p->dirty, p->inked.b[i]);
            p->inked.n = 0;
            p->last_dmg = now;
        }
        if (p->want_full) {
            box all = { 0, 0, p->W, p->H };
            act(out, (action){ A_GRAB_ALL, all, 0, 0, 0 });
            send_gc16(p, out, all, M_SCREEN, G_FOREIGN, now);
            p->want_full = 0;
            forget(p);
            to(p, S_IDLE);
            return pend_n ? STEP_DROP : STEP_NONE;
        }
        if (pend_n && !inking && now - first_dmg >= coalesce(p) && !throttled(p, now) &&
            (p->db_tap || pol_debounce_left(p, now) <= 0)) {
            if (!p->db_tap) p->db_t0 = 0;
            return STEP_BURST;
        }
        if (p->cand.n && now - p->last_restack >= CAND_MS) p->cand.n = 0;
        /* an engine-paced run: while the engine can't take the next burst, epdd
         * holds sway's frame (ahead=predict), so no damage comes. That quiet is
         * ours, not the app's: it counts from when the engine was free, up to
         * ENG_HOLD_MAX_MS after the last damage (OpenMW's moving sky:
         * every run ended 400 ms after its first A2, whose engine time was
         * ~470 ms, then 2 s of tail GC16 partials per frame, 760 of 855 updates
         * superseded) */
        int eng_busy = engine_paced(p) && p->eng_wait > 0 && now - p->last_dmg < ENG_HOLD_MAX_MS;
        if (eng_busy) p->eng_held = now;
        if (!eng_busy && p->state != S_IDLE && !pend_n && !inking && !p->inked.n && clean_waits(p) && ptr_rested(p, now)) {
            int w = clean_wait(p, now);
            if (w >= 0 && clean_left(p, now, w) <= 0) {
                if (w > p->idle_ms && p->log) p->log("run end after %d ms quiet (the app's frame gap)\n", w);
                cleanup(p, now, out);
            } else if (w >= 0 && !p->run_always && small_motion(p, now, w)) {
                p->early_cleans++;
                cleanup(p, now, out);
            }
        }
        if (ink_debt_waits(p, inking, pend_n) && ink_debt_left(p, now) <= 0 && ink_debt_due(p))
            ink_debt_clean(p, now, out);
        return STEP_NONE;
    }
    return STEP_NONE;
}

/* One flush per burst: every SEND_UPDATE costs a fixed ~41 ms however small
 * and they run in sequence. Classified by the changed area (sum of the
 * tight rects):
 * run (below): dithered DU, folded to <= 3
 * >= flash_min: one flash of the window being repainted
 * any rect > du_max, or a "quality" app: one GC16 partial of everything
 * else DU, folded to <= 3, cleaned at idle
 * A motion run starts with a burst while a GC16 is in flight (dragging,
 * scrolling, animation: a flash per step was unbearable, colliding GC16s left
 * windows half-painted) or in a "fast" app, and STAYS a run until the
 * idle cleanup (else each scroll step > GC16_BUSY_MS after the last flash
 * flashed again). "quality" apps never get DU. */
/* a burst committed in exact greys and sent GC16: the DU'd boxes inside its
 * changed rects hold those greys now, the cleanup need not GC16 them again
 * (after a pen stroke the app's own GC16 repaint and the cleanup's
 * 250 ms later both went over the stroke: "renders twice") */
static void gc16_cleans(policy *p, const boxlist *ch)
{
    for (int i = 0; i < ch->n; i++) { list_drop_inside(&p->dirty, ch->b[i]); list_drop_inside(&p->ptr_dirty, ch->b[i]); }
    if (p->state == S_CLEANUP && !clean_waits(p) && !p->inked.n)
        to(p, S_IDLE);                      /* (nothing left to clean) */
}

/* pol_burst's facts, each spelled once */
/* repaints after a window change (the windows of a workspace one after
 * another, what a popup uncovered): not motion, even inside a run */
static int restacked(const policy *p, int64_t t) { return t - p->last_restack < RESTACK_MS || t - p->last_popup < RESTACK_MS; }
/* a GC16 still in flight where this burst changes things: content faster
 * than GC16. Elsewhere it is no motion (a button tap repainted
 * the app that lost focus with a GC16, then the settings page beside it
 * 100 ms later went DU + run + end flash: two visible refreshes) */
static int gc16_over(const policy *p, const boxlist *ch, int64_t t)
{
    int over = 0;
    if (t - p->last_gc16 < GC16_BUSY_MS && t - p->gc16_box_t < GC16_BUSY_MS)
        for (int i = 0; i < ch->n && !over; i++) over = !box_empty(box_clip(ch->b[i], p->gc16_box));
    return over;
}
/* a large change (busy_min of the panel): a game's or a video's frame, not a scrollbar */
static int large(const policy *p, const boxlist *ch)
{
    return ch->n && p->busy_min > 0 && union_area(ch->b, ch->n) >= p->busy_min * p->W * p->H;
}
/* within run_tail of a run's end */
static int tail_window(const policy *p, int64_t t) { return p->run_tail > 0 && p->last_run_end > 0 && t - p->last_run_end < p->run_tail; }
/* no GC16 since but our own (the run end's, the tail's) */
static int tail_own(const policy *p) { return p->last_gc16 <= p->last_own_gc16; }
/* the sized choices apply (burst=auto, not a quality app) */
static int auto_paint(const policy *p, int pm) { return p->burst_auto && pm != PM_QUALITY; }
/* the user is engaged: a handle drag, a finger down, a wheel / touchpad scroll,
 * or content epd-motion measures moving. One definition:
 * the ink re-render and the run tail lacked the wheel, so a touchpad scroll
 * there was a still frame) */
static int engaged(const policy *p, const burst_in *in, int64_t t) { return p->c.drag || p->c.touch.down || wheel_held(p, t) || in->moving; }
/* ... except for a popup's paint: GTK opens a menu on the PRESS, so the finger
 * is still down (and the content may look moving) when it paints; there only a
 * drag or a wheel scroll keeps it from the popup paint (kept apart on purpose,
 */
static int engaged_popup(const policy *p, int64_t t) { return p->c.drag || wheel_held(p, t); }

/* pol_burst's rows,
 * in priority order: the first whose guard holds wins. POPUP and GONE come
 * before INK_RE (the doc's table lists INK_RE first; the code's order is this) */
enum { R_NONE, R_POPUP, R_GONE, R_INK_RE, R_RUN, R_PRESS, R_STILL_FLOOR, R_TAIL_LARGE,
       R_PAGE, R_CALM_LARGE, R_FLASH, R_GC16, R_DU };
/* a burst's facts: what the rows' guards read (the prologue computes them) */
typedef struct {
    int gc16_busy, busy, tail;
    int flash;                  /* area/screen >= flash_min */
    int wide;                   /* the largest rect / screen > du_max */
} burst_facts;

/* which row a burst is: pure, no state changes */
static int classify(const policy *p, const burst_in *in, const boxlist *ch, const burst_facts *f, int64_t t)
{
    int pm = in->pm;
    if (!ch->n) return R_NONE;
    /* what changed where a window just (dis)appeared gets its flash at the cleanup;
     * not under a popup's paint: the popup covers it (the quick settings
     * just closed flashed the app drawer opened over them, ~1 s, after its paint) */
    if (!engaged_popup(p, t) && p->popup_fast && auto_paint(p, pm) && in_popup(p, ch, t)) return R_POPUP;
    /* not while a finger is down or the content moves (GNOME
     * Software: a tooltip shown after each lift closed at the next touch-down,
     * and that stroke's frames went out as a second of full-window GC16s; the
     * scroll's run end repaints what the popup uncovered anyway) */
    if (!engaged(p, in, t) && p->state != S_RUN && auto_paint(p, pm) && t - p->last_gone < POPUP_MS) return R_GONE;
    /* ink_redraw=du: the app's re-render of the stroke the pen just drew (the
     * changed pixels inside what it inked, within POST_INK_MS of the ink) is
     * not motion, even under the GC16 of its first re-render or in a run: a DU,
     * the one GC16 partial after the last re-render (Rnote GC16
     * partial, its second re-render under it started a run = cut A2, run-end
     * flash: 3 refreshes per stroke) */
    if (p->ink_redraw && auto_paint(p, pm) && pm != PM_FAST &&
        !engaged(p, in, t) && t - p->c.ink_t < POST_INK_MS && in_ink(p, ch)) return R_INK_RE;
    /* calm (the shell's bar and popups, "all interaction
     * here should be Calm"): a tap's highlight was a press = dithered DU or the
     * cut-A2 floor (a grey smudge); never a run, a press or the still floor */
    if (!in->calm && auto_paint(p, pm) &&
        (in->always || pm == PM_FAST || p->c.drag || in->moving ||
         (!restacked(p, t) && (((f->gc16_busy || f->busy) && !f->tail) || p->state == S_RUN)))) return R_RUN;
    /* finger down: fast like a run, so the first scroll step shows at once —
     * a GC16 here (the first repaint of a scroll is large) queued every later
     * DU behind it for ~0.5-1 s */
    if (!in->calm && p->burst_auto && pm == PM_PRESS) return R_PRESS;
    /* still=range: a repaint outside a run paints at the range's low end (an
     * app forced to A2; a GC16 low end keeps the sized choice below, except
     * that one mode for everything never DUs) */
    if (!in->calm && p->still_range && p->adaptive && p->has_engine && auto_paint(p, pm) &&
        (p->m_min != LV_GC16 || (single(p) && !f->flash))) return R_STILL_FLOOR;
    if (p->burst_auto && f->flash && f->tail) return R_TAIL_LARGE;       /* the run end just flashed: no second flash */
    if (p->burst_auto && f->flash && in->pages && (p->pages_n + 1) % (unsigned)in->pages) return R_PAGE;   /* a page change, no flash */
    if (in->calm && f->flash) return R_CALM_LARGE;                       /* a large shell change, no flash */
    if (p->burst_auto && f->flash) return R_FLASH;
    if (pm == PM_QUALITY || (p->burst_auto && f->wide)) return R_GC16;
    return R_DU;
}

/* DU of each (folded) rect, left for the cleanup */
static void du_dirty(policy *p, actions *out, boxlist *ch, int fold, int64_t t)
{
    list_fold(ch, fold);
    for (int i = 0; i < ch->n; i++) {
        send(p, out, ch->b[i], WF_DU, M_PART, t);
        list_add(&p->dirty, ch->b[i]);
    }
    pending(p);
}

int pol_burst(policy *p, const burst_in *in, actions *out)
{
    const double screen = (double)p->W * p->H;
    boxlist ch = in->ch;
    int64_t t = in->now;
    int pm = in->pm;
    p->dith = in->dither ? in->dither > 0 : p->dither_runs;
    if (!awake(p)) return C_ALL;
    p->last_burst = t;
    if (ch.n) {                                 /* the motion area (small_motion) */
        int64_t bt = t - t % MV_BUCKET_MS;
        int bi = (int)((t / MV_BUCKET_MS) % MV_N);
        if (p->mv_t[bi] != bt) { p->mv_t[bi] = bt; p->mv_n[bi] = 0; }
        box *mb = p->mv_b[bi];
        for (int i = 0; i < ch.n; i++) {
            int k = 0;
            while (k < p->mv_n[bi] && !box_inside(ch.b[i], mb[k])) k++;
            if (k < p->mv_n[bi]) continue;
            if (p->mv_n[bi] < MV_K) { mb[p->mv_n[bi]++] = ch.b[i]; continue; }
            long best = -1; int bk = 0;               /* full: into the box it grows least */
            for (k = 0; k < MV_K; k++) {
                long g = box_area(box_union(mb[k], ch.b[i])) - box_area(mb[k]);
                if (best < 0 || g < best) { best = g; bk = k; }
            }
            mb[bk] = box_union(mb[bk], ch.b[i]);
        }
    }
    /* Content: a frame moving fast arms the stall hold, a slower or still one
     * in the run's window (the decay) disarms it; elsewhere (a clock) nothing */
    if (ch.n) {
        if (in->moving && in->speed >= STALL_V) p->stall_t = t;
        else {
            int in_run = box_empty(p->run_win);
            for (int i = 0; i < ch.n && !in_run; i++) in_run = !box_empty(box_clip(ch.b[i], p->run_win));
            if (in_run) p->stall_t = 0;
        }
    }
    burst_facts f = {0};
    f.gc16_busy = gc16_over(p, &ch, t);
    int big = large(p, &ch);
    /* the tail after a run's end GC16 (Chromium after a fling: the
     * overlay scrollbar fading out ~240 ms after the end flash, then the
     * scroll-end re-raster, restarted the stagger and a second flash): the
     * GC16 just sent is our own, not motion. Still frames there take the sized
     * choice below (a large one a partial, no second flash); a finger, measured
     * motion or a handle drag runs as before. Busy content too waits for the
     * tail's end: the fade's frames plus the re-raster looked busy (6 bursts in
     * 2 s, the scroll's own changes still in the motion area) */
    /* ... but not a game or video drawing on (OpenMW traces:
     * a large frame every ~60 ms through the tail got a GC16 partial each,
     * 500-900 ms of panel each, queued on the same pixels; the next run's
     * first DU waited behind them, sway's frame was held, the run ended, and
     * so on, ~16 times in 45 s). Large = busy_min (the fade's scrollbar is
     * not); several in a row, each soon after the last (the re-raster is one) */
    if (big) {
        p->tf_n = t - p->tf_t <= TAIL_FAST_GAP_MS ? p->tf_n + 1 : 1;
        p->tf_t = t;
    } else if (ch.n) p->tf_n = 0;
    f.tail = tail_window(p, t) && tail_own(p) && !engaged(p, in, t) && p->tf_n < TAIL_FAST_N;
    if (p->tf_n == TAIL_FAST_N && tail_window(p, t) && p->log)
        p->log("run tail: continuous content (%d large frames <= %d ms apart): no tail\n", TAIL_FAST_N, TAIL_FAST_GAP_MS);
    /* pages=: a GC16 in flight is no motion without a content shift (a menu
     * tap's highlight, then the menu) */
    if (in->pages && !in->moving) f.gc16_busy = 0;
    /* busy content keeps (and after a cleanup restarts) its run with no input */
    if (big) p->last_big = t;
    if (ch.n && !in->calm && (in->always || in->busy != BUSY_OFF)) {
        f.busy = busy_seen(p, t) || in->busy == BUSY_ON || in->always;
        if (in->pages && !in->moving && in->busy != BUSY_ON && !in->always) f.busy = 0;   /* (page turns in a row look busy) */
        if (f.busy) {
            if (p->log && t - p->last_busy > BUSY_SPAN_MS) p->log("busy content: motion without input\n");
            p->last_busy = t;
            p->busy_bursts++;
        }
    }
    long area = 0, maxr = 0;
    box all = {0};
    for (int i = 0; i < ch.n; i++) {
        long a = box_area(ch.b[i]);
        area += a;
        if (a > maxr) maxr = a;
        all = box_union(all, ch.b[i]);
    }
    f.flash = area / screen >= p->flash_min;
    f.wide = maxr / screen > p->du_max;
    int row = classify(p, in, &ch, &f, t);
    /* the commit for the rows that keep it (binary unless the sized choice is a GC16) */
    int how = row == R_NONE || pm == PM_QUALITY || (p->burst_auto && (f.flash || f.wide)) ? C_ALL : C_BW;
    if (row == R_POPUP)
        for (int k = 0; k < p->popup.n; k++) list_drop_inside(&p->stale, p->popup.b[k]);
    if (row != R_POPUP && row != R_GONE)
        for (int i = 0; i < ch.n; i++)
            for (int k = 0; k < p->cand.n; k++) {
                box s = box_clip(ch.b[i], p->cand.b[k]);
                if (!box_empty(s)) {
                    if (p->log) p->log("stale %d,%d-%d,%d\n", s.x0, s.y0, s.x1, s.y1);
                    list_add(&p->stale, s);
                    pending(p);
                }
            }
    if (f.tail && (f.gc16_busy || f.busy) && row != R_NONE && row != R_POPUP && row != R_GONE && row != R_INK_RE && row != R_RUN && p->log) p->log("run tail: no run (the end GC16 is ours)\n");
    /* pages=: every burst that gets this far counts (a page or the flash after them) */
    if ((row == R_PAGE || row == R_CALM_LARGE || row == R_FLASH) && p->burst_auto && f.flash && in->pages) ++p->pages_n;
    switch (row) {
    case R_NONE: return how;
    /* a popup's paint, no flash of the rest of the panel: a dithered DU (the
     * card at once), the GC16 partial of its exact greys right behind
     * (popup=du, default); popup=fast skips the DU (one transition, but a GC16
     * fades black/white in; du looked better). Not a run, nothing
     * left for the cleanup. */
    case R_POPUP:
        list_fold(&ch, 3);
        if (p->popup_fast == 2)                 /* popup=du: dithered DU first (A2 on the engine) */
            for (int i = 0; i < ch.n; i++) send(p, out, ch.b[i], WF_DU, M_PART, t);
        for (int i = 0; i < ch.n; i++) {
            if (p->popup_fast == 2) grab(out, ch.b[i]);
            send_gc16(p, out, ch.b[i], M_PART, G_FOREIGN, t);
        }
        if (p->log) p->log(p->popup_fast == 2 ? "popup paint: DU + GC16\n" : "popup paint: GC16\n");
        return p->popup_fast == 2 ? (p->popup_dither ? C_DITHER : C_THRESH) : C_ALL;
    /* what a popup uncovered: one GC16 partial, no DU (popup=du's DU there left
     * grey ghosts), no run, nothing left for the cleanup */
    case R_GONE:
        list_fold(&ch, 3);
        for (int i = 0; i < ch.n; i++) {
            list_drop_inside(&p->stale, ch.b[i]);
            send_gc16(p, out, ch.b[i], M_PART, G_FOREIGN, t);
        }
        if (p->log) p->log("popup gone paint: GC16\n");
        return C_ALL;
    case R_INK_RE:
        list_fold(&ch, 3);
        for (int i = 0; i < ch.n; i++) {
            send(p, out, ch.b[i], WF_DU, M_PART, t);
            list_add(p->ink_clean ? &p->inkr : &p->dirty, ch.b[i]);
        }
        p->ink_held = 1;
        pending(p);
        if (p->log) p->log("ink redraw: DU, the GC16 after the last one\n");
        return C_BW;
    case R_RUN:
        to(p, S_RUN);
            /* its end flash is clipped to the window it happens in (the union
             * with everything else drawn meanwhile, e.g. the panel clock, flashed
             * nearly the whole screen). Handwriting (app repaints right after pen
             * ink) must not end in a flash. */
            /* the window the run STARTED in: a panel clock tick mid-scroll made the
             * union span panel..Firefox, terminal between included; DUs
             * elsewhere are cleaned by the partial GC16 of `dirty` */
        if (box_empty(p->run_win)) p->run_win = in->pwin;
        for (int i = 0; i < ch.n; i++) p->run_box = box_union(p->run_box, box_clip(ch.b[i], p->run_win));
        if (t - p->c.ink_t > POST_INK_MS) p->run_bursts++;
        if (in->always) p->run_always = 1;
        if (p->cut_auto) cut(p, out, 5, t);
        if (p->adaptive && p->has_engine) {
            int lv = motion_step(p, t, in->speed);
            if (!p->m_curve || in->always) lv = top_step(p);    /* curve=off, motion=always: the fast end */
            p->lvl_n[lv]++;
            list_fold(&ch, p->run_fold);
            for (int i = 0; i < ch.n; i++) {
                if (lv == LV_GC16) { send_gc16(p, out, ch.b[i], M_PART, G_NONE, t); continue; }   /* exact: nothing to clean */
                send_n(p, out, ch.b[i], lv == LV_DU4 ? WF_DU4 : WF_DU, M_PART, a2n(p, lv), t);
                if (a2n(p, lv)) { ghost_note(p, ch.b[i], 1); p->run_cut = 1; }   /* cuts ghost twice as fast */
                list_add(&p->dirty, ch.b[i]);
            }
            pending(p);
            return lv == LV_GC16 ? C_ALL : lv == LV_DU4 ? C_Q4 : p->dith ? C_DITHER : C_THRESH;
        }
        du_dirty(p, out, &ch, p->run_fold, t);
        return p->dith ? C_DITHER : C_BW;
    case R_PRESS:
        /* a press with a cut floor (mmin above A2, e.g. fixed A2/2): the run's own
         * paint. The uncut A2 here (10 frames, no payback) held the first run steps
         * on its pixels and the reads behind them (fixed A2/2 still
         * ramped up at a scroll's start). One mode for everything: the press too.
         * curve=off: a press is motion, so the fast end */
        if (p->adaptive && p->has_engine && !p->m_curve)
            return floor_paint(p, out, &ch, top_step(p), t);
        if (p->adaptive && p->has_engine && (p->m_min > LV_A2 || single(p)))
            return floor_paint(p, out, &ch, p->m_min, t);
        du_dirty(p, out, &ch, p->run_fold, t);
        return p->dith ? C_DITHER : C_BW;
    case R_STILL_FLOOR:
        return floor_paint(p, out, &ch, p->m_min, t);
    case R_TAIL_LARGE:
        send_gc16(p, out, all, M_PART, G_OWN, t);
        gc16_cleans(p, &ch);
        return how;
    case R_PAGE:
        send_gc16(p, out, all, M_PART, G_FOREIGN, t);
        gc16_cleans(p, &ch);
        if (p->log) p->log("page: GC16 partial (%u of %d before a flash)\n", p->pages_n % (unsigned)in->pages, in->pages);
        return how;
    case R_CALM_LARGE:
        send_gc16(p, out, all, M_PART, G_FOREIGN, t);
        gc16_cleans(p, &ch);
        return how;
    case R_FLASH: {
        p->pages_n = 0;
            /* flash only the window being repainted; whatever changed elsewhere in
             * the same burst (terminal cursor, panel clock) gets a non-flashing
             * partial GC16 */
        box inb = {0}, outb = {0};
        for (int i = 0; i < ch.n; i++) {
            inb = box_union(inb, box_clip(ch.b[i], in->pwin));
            if (!box_inside(ch.b[i], in->pwin)) outb = box_union(outb, ch.b[i]);
        }
        send_gc16(p, out, box_empty(inb) ? all : inb, M_AREA, G_FOREIGN, t);
        if (!box_empty(inb) && !box_empty(outb)) send_gc16(p, out, outb, M_PART, G_FOREIGN, t);
        gc16_cleans(p, &ch);
        return how;
    }
    case R_GC16:
        send_gc16(p, out, all, M_PART, G_FOREIGN, t);
        gc16_cleans(p, &ch);
        return how;
    default:                                    /* R_DU */
        du_dirty(p, out, &ch, 3, t);
        return how;
    }
}

int pol_pointer(policy *p, boxlist *ch, int64_t now, actions *out)
{
    if (!awake(p) || !ch->n) return C_ALL;
    p->dith = p->dither_runs;
    int lv = p->adaptive && p->has_engine ? top_step(p) : LV_A2;
    if (lv == LV_GC16) lv = LV_A2;          /* (a GC16-only range: still no GC16 under a moving pointer) */
    list_fold(ch, 2);
    for (int i = 0; i < ch->n; i++) {
        send_n(p, out, ch->b[i], lv == LV_DU4 ? WF_DU4 : WF_DU, M_PART, a2n(p, lv), now);    /* (ghost: once) */
        list_add(&p->ptr_dirty, ch->b[i]);
    }
    p->last_ptr = now;
    pending(p);
    if (p->log) p->log("pointer: %s %d box%s\n", lv == LV_DU4 ? "DU4" : a2n(p, lv) ? "cut A2" : "DU", ch->n, ch->n == 1 ? "" : "es");
    return lv == LV_DU4 ? C_Q4 : p->dith ? C_DITHER : C_THRESH;
}

int pol_timeout(const policy *p, int64_t now, int inking, int pend_n)
{
    if (!awake(p)) return pend_n ? COALESCE_MS : -1;
    if (p->inked.n && !inking) return COALESCE_MS;
    if (pend_n && !inking) {
        int w = hold_ms(p, now), c = coalesce(p), d = pol_debounce_left(p, now);
        if (d > w) w = d;
        return w > c ? w : c;
    }
    int t = -1;
    if (clean_waits(p)) {
        int w = clean_wait(p, now);
        if (w < 0) {
            /* held: a finger down looks again every idle_ms (its lift wakes the
             * daemon too); a wheel scroll's hold ends by time: wake then */
            t = p->idle_ms;
            if (!p->c.touch.down) {
                int64_t h = p->c.wheel_t + p->wheel_hold - now;
                if (h < t) t = h > COALESCE_MS ? (int)h : COALESCE_MS;
            }
        } else {
            /* the cleanup's own deadline (until w == idle_ms looked
             * again a whole idle_ms later: a run ended up to idle_ms late) */
            int left = (int)clean_left(p, now, w);
            t = left > COALESCE_MS ? left : COALESCE_MS;
        }
        int pr = (int)ptr_left(p, now);                         /* (pol_step: the pointer's rest) */
        if (t >= 0 && pr > t) t = pr;
    }
    if (ink_debt_waits(p, inking, pend_n) && ink_debt_due(p)) {     /* (pol_step's ink debt clean) */
        int64_t left = ink_debt_left(p, now);
        int d = left > COALESCE_MS ? (int)left : COALESCE_MS;
        if (t < 0 || d < t) t = d;
    }
    return t;
}
