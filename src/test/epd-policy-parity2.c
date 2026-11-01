/* parity2: the live epd-policy against the FROZEN copy (test/ref, see its README),
 * over the whole pol_* API, with random traces under a set of eink.conf-like
 * profiles. Compares outputs only: actions (in order), C_*, STEP_*,
 * pol_timeout, pol_quiet_held, and the state where the daemon reads it
 * (RUN / PAUSED / ASLEEP; IDLE vs CLEANUP is relaxed). A divergence is shrunk
 * to a minimal op list and printed.
 * epd-policy-parity2 [seeds [steps]] PARITY2_SELFTEST=1: proves it finds a difference */
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../epd-policy.h"
#include "ref/ref-api.h"

#define W 1404
#define H 1872

/* the profiles: what eink.conf / an app's range sets (pol_config), plus the daemon's fields */
static const struct { const char *name, *cfg; int engine, whole; } profiles[] = {
    { "daily",        "", 1, 1 },
    { "daily-x",      "", 1, 0 },
    { "no-engine",    "", 0, 0 },
    { "fixed-paint",  "mmin=a2_4 mmax=a2_4", 1, 1 },
    { "curve-off",    "curve=off", 1, 1 },
    { "still-range",  "still=range mmin=a2_8", 1, 1 },
    { "popup-gc16",   "popup=gc16", 1, 1 },
    { "popup-du",     "popup=du popupdu=dither", 1, 1 },
    { "ink-stroke",   "ink_clean=stroke ink_redraw=off", 1, 1 },
    { "ink-redraw",   "ink_redraw=du", 1, 1 },
    { "busy-off",     "busy=0", 1, 1 },
    { "tail-off",     "run_tail=0", 1, 1 },
    { "pace-fixed",   "pace=fixed", 1, 1 },
    { "quiet-0",      "quiet_ms=0", 1, 1 },
    { "run-end-flash","run_end=flash run_wait=0", 1, 1 },
    { "ptr-wheel",    "ptr_rest_ms=1000 wheel_hold_ms=400", 1, 1 },
    { "binary",       "motion=binary", 1, 1 },
};
#define NPROF ((int)(sizeof profiles / sizeof *profiles))

/* ctl lines (pol_cmd) and eink.conf words (pol_config) the trace may send */
static const char *cmds[] = { "full", "pause", "resume", "sleep", "wake", "cut=5", "cut=auto", "cut=2",
                              "dither=off", "dither=on", "burst=du", "burst=auto", "clean=full", "clean=partial",
                              "du_max=0.05", "du_max=0.03", "flash_min=0.3", "flash_min=0.35", "gamma=1.8" };
static const char *cfgs[] = { "idle_ms=250", "idle_ms=400", "run_ms=100", "run_ms=0", "run_fold=1", "run_fold=3",
                              "run_wait=1.5", "run_wait=0", "run_wait_max=600", "clean_motion=0.5", "busy=0.3",
                              "motion=adaptive", "still=auto", "still=range", "attack=jump", "attack=fused",
                              "curve=on", "curve=off", "run_end=partial", "run_tail=2000", "quiet_ms=300",
                              "quiet_after=5000", "popup=fast", "ink_clean=damage", "ink_redraw=off",
                              "ptr_rest_ms=0", "wheel_hold_ms=1000", "mmax=a2_2", "mmin=a2" };
#define N(a) ((int)(sizeof a / sizeof *a))

enum { O_NONE, O_CMD, O_CFG, O_FINGER, O_WHEEL, O_INPUT, O_DAMAGE, O_WINDOW, O_POPUP, O_GONE, O_SETTLED,
       O_MOTION, O_INK, O_INKED, O_INK_END, O_RECUT, O_ENGINE, O_POINTER, O_RANGE, O_FIELDS, O_BANNER, O_KINDS };
static const char *op_names[] = { "none", "cmd", "cfg", "finger", "wheel", "input", "damage", "window", "popup",
                                  "gone", "settled", "motion", "ink", "inked", "ink_end", "recut", "engine",
                                  "pointer", "range", "fields", "banner" };
typedef struct {
    int kind;
    int64_t now;
    int i1, i2, i3;
    double v;
    box b;
    boxlist ch;                         /* the burst's (and the pointer's) changes */
    box pwin;
    int pm, moving, busy, dither, always, pages, calm;
    double speed;
} op;

static box rbox(int w, int h)
{
    int x = rand() % W, y = rand() % H;
    int bw = 1 + rand() % w, bh = 1 + rand() % h;
    return (box){ x, y, x + bw > W ? W : x + bw, y + bh > H ? H : y + bh };
}
static const box wins[] = { { 0, 0, W, H }, { 0, 104, W, H }, { 0, 977, W, H }, { 0, 104, W, 977 }, { 300, 400, 1100, 1400 } };

static void gen(op *o, long n, unsigned seed)
{
    srand(seed);
    int64_t now = 1000;
    for (long s = 0; s < n; s++) {
        op x = {0};
        int r = rand() % 100;
        now += r < 60 ? rand() % 40 : r < 90 ? rand() % 300 : r < 98 ? rand() % 2000 : 2000 + rand() % 6000;
        x.now = now;
        int e = rand() % 100;
        x.kind = e < 2 ? O_CMD : e < 4 ? O_CFG : e < 9 ? O_FINGER : e < 13 ? O_WHEEL : e < 18 ? O_INPUT :
                 e < 55 ? O_DAMAGE : e < 57 ? O_WINDOW : e < 59 ? O_POPUP : e < 61 ? O_GONE : e < 62 ? O_SETTLED :
                 e < 63 ? O_MOTION : e < 69 ? O_INK : e < 71 ? O_INKED : e < 73 ? O_INK_END : e < 74 ? O_RECUT :
                 e < 78 ? O_ENGINE : e < 85 ? O_POINTER : e < 86 ? O_RANGE : e < 87 ? O_FIELDS : e < 88 ? O_BANNER : O_NONE;
        if (x.kind == O_WHEEL && getenv("PARITY2_NO_WHEEL")) x.kind = O_NONE;   /* (a change confined to the wheel: 0 without it) */
        x.i1 = rand(); x.i2 = rand(); x.i3 = rand();
        x.v = (rand() % 100) / 25.0;
        x.b = rand() % 4 ? rbox(400, 300) : rbox(W, H);
        int nb = rand() % 5, big = rand() % 3 == 0;
        for (int i = 0; i < nb; i++) list_add(&x.ch, big ? rbox(W, H) : rbox(200, 120));
        x.pwin = wins[rand() % N(wins)];
        x.pm = rand() % 4;
        x.moving = rand() % 3 == 0;
        x.speed = x.moving ? (rand() % 100) / 20.0 : 0;
        x.busy = rand() % 3;
        x.dither = rand() % 2;
        x.always = rand() % 8 == 0;
        x.pages = (int[]){ 0, 0, 0, 1, 6 }[rand() % 5];
        x.calm = rand() % 10 == 0;
        o[s] = x;
    }
}

static ref_box RB(box b) { return (ref_box){ b.x0, b.y0, b.x1, b.y1 }; }
static ref_list RL(const boxlist *l)
{
    ref_list r = {0};
    r.n = l->n;
    for (int i = 0; i < l->n; i++) r.b[i] = RB(l->b[i]);
    return r;
}
static int same_acts(const actions *a, const ref_acts *b)
{
    if (a->n != b->n) return 0;
    for (int i = 0; i < a->n; i++) {
        const action *x = &a->a[i];
        const ref_act *y = &b->a[i];
        if (x->kind != y->kind || x->wf != y->wf || x->mode != y->mode || x->n != y->n ||
            x->b.x0 != y->b.x0 || x->b.y0 != y->b.y0 || x->b.x1 != y->b.x1 || x->b.y1 != y->b.y1) return 0;
    }
    return 1;
}
static void print_acts(const char *who, const actions *a, const ref_acts *b)
{
    int n = a ? a->n : b->n;
    printf("    %s: %d actions\n", who, n);
    for (int i = 0; i < n; i++) {
        int k, wf, mode, nn; box x;
        if (a) { k = a->a[i].kind; wf = a->a[i].wf; mode = a->a[i].mode; nn = a->a[i].n; x = a->a[i].b; }
        else { k = b->a[i].kind; wf = b->a[i].wf; mode = b->a[i].mode; nn = b->a[i].n;
               x = (box){ b->a[i].b.x0, b->a[i].b.y0, b->a[i].b.x1, b->a[i].b.y1 }; }
        printf("      kind %d wf %d mode %d n %d  %d,%d-%d,%d\n", k, wf, mode, nn, x.x0, x.y0, x.x1, x.y1);
    }
}
static int strict_state(int s) { return s == S_RUN || s == S_PAUSED || s == S_ASLEEP ? s : -1; }

/* the daemon's run_range (glassine-epdd-desktop.c): an app's own range swapped in for its bursts */
typedef struct { int saved, mmin, mmax, still, curve; } range_save;
static void range_live(policy *p, range_save *s, int lo, int hi, int curve)
{
    if (s->saved) { p->m_min = s->mmin; p->m_max = s->mmax; p->still_range = s->still; p->m_curve = s->curve; s->saved = 0; }
    if (lo >= 0) {
        *s = (range_save){ 1, p->m_min, p->m_max, p->still_range, p->m_curve };
        p->m_min = lo; p->m_max = hi; p->still_range = 1;
        if (curve >= 0) p->m_curve = curve;
    }
}
static void range_ref(ref_pol *r, range_save *s, int lo, int hi, int curve)
{
    if (s->saved) { ref_set(r, RF_M_MIN, s->mmin); ref_set(r, RF_M_MAX, s->mmax); ref_set(r, RF_STILL_RANGE, s->still);
                    ref_set(r, RF_M_CURVE, s->curve); s->saved = 0; }
    if (lo >= 0) {
        *s = (range_save){ 1, ref_get(r, RF_M_MIN), ref_get(r, RF_M_MAX), ref_get(r, RF_STILL_RANGE), ref_get(r, RF_M_CURVE) };
        ref_set(r, RF_M_MIN, lo); ref_set(r, RF_M_MAX, hi); ref_set(r, RF_STILL_RANGE, 1);
        if (curve >= 0) ref_set(r, RF_M_CURVE, curve);
    }
}

/* what the traces reached (the generator must exercise the policy, or parity2 proves nothing) */
static struct { long st[8], c[8], wf[4][4], swf[4][4]; } cov;

/* the policy's own log lines, by their text up to ':' / '%' / '(': which paths the traces took */
#define NTALLY 64
static struct { char key[40]; long n; } tally[NTALLY];
static int tallying;
static void tally_log(const char *fmt, ...)
{
    if (!tallying) return;
    char k[40];
    size_t n = strcspn(fmt, ":%(\n");
    if (n >= sizeof k) n = sizeof k - 1;
    memcpy(k, fmt, n); k[n] = 0;
    while (n && k[n - 1] == ' ') k[--n] = 0;
    for (int i = 0; i < NTALLY; i++) {
        if (!tally[i].key[0]) { strcpy(tally[i].key, k); tally[i].n = 1; return; }
        if (!strcmp(tally[i].key, k)) { tally[i].n++; return; }
    }
}
static long tally_of(const char *k)
{
    for (int i = 0; i < NTALLY && tally[i].key[0]; i++) if (!strcmp(tally[i].key, k)) return tally[i].n;
    return 0;
}

static long kind1;                     /* benign earlier wakes (kind 1) */

static int selftest;                    /* the live side gets an extra word: parity2 must see it */

/* runs ops[0..n) where keep[i]; returns the index of the first divergence (-1: none);
 * verbose: prints what differs */
static long run(const op *ops, long n, const char *keep, int prof, int verbose)
{
    policy p; pol_init(&p, W, H);
    ref_pol *r = ref_new(W, H);
    pol_config(&p, profiles[prof].cfg); ref_config(r, profiles[prof].cfg);
    if (selftest) pol_config(&p, selftest == 2 ? "popup=gc16" : "idle_ms=260");   /* 1: a timeout differs, 2: actions */
    p.has_engine = profiles[prof].engine; ref_set(r, RF_HAS_ENGINE, profiles[prof].engine);
    p.whole = profiles[prof].whole; ref_set(r, RF_WHOLE, profiles[prof].whole);
    p.log = tally_log; ref_set(r, RF_LOG, 1);
    tallying = !keep;
    range_save sl = {0}, sr = {0};
    int pend_n = 0, inking = 0;
    int64_t first_dmg = 0;
    long bad = -1;
    char what[64] = "";
    static actions a;
    static ref_acts b;
    for (long s = 0; s < n && bad < 0; s++) {
        if (keep && !keep[s]) continue;
        const op *o = &ops[s];
        int64_t now = o->now;
        a.n = 0; b.n = 0;
        int v1 = 0, v2 = 0, acts = 0;
        switch (o->kind) {
        case O_CMD: {
            /* (half of them resume / wake: a pause or sleep would hold most of the trace) */
            const char *c = o->i2 % 2 ? (o->i3 % 2 ? "resume" : "wake") : cmds[o->i1 % N(cmds)];
            v1 = pol_cmd(&p, c, now, &a); v2 = ref_cmd(r, c, now, &b); acts = 1;
            if (v1 && v2 && same_acts(&a, &b)) {
                box ch = o->i2 % 2 ? o->b : (box){0};
                a.n = 0; b.n = 0;
                pol_wake(&p, ch, now, &a); ref_wake(r, RB(ch), now, &b);
            }
            snprintf(what, sizeof what, "cmd %s", c);
            break;
        }
        case O_CFG: {
            const char *c = cfgs[o->i1 % N(cfgs)];
            pol_config(&p, c); ref_config(r, c);
            snprintf(what, sizeof what, "cfg %s", c);
            break;
        }
        case O_FINGER: { int d = o->i1 % 3 != 0; pol_finger(&p, now, d, d ? o->v : 0); ref_finger(r, now, d, d ? o->v : 0); break; }
        case O_WHEEL: pol_wheel(&p, now); ref_wheel(r, now); break;
        case O_INPUT: pol_input(&p, now); ref_input(r, now); break;
        case O_DAMAGE:
            if (!pend_n) first_dmg = now;
            pend_n++;
            pol_damage(&p, now); ref_damage(r, now);
            break;
        case O_WINDOW: pol_window(&p, o->b, now); ref_window(r, RB(o->b), now); break;
        case O_POPUP: pol_popup(&p, o->b, now); ref_popup(r, RB(o->b), now); break;
        case O_GONE: pol_popup_gone(&p, o->b, now); ref_popup_gone(r, RB(o->b), now); break;
        case O_SETTLED: pol_settled(&p, now); ref_settled(r, now); break;
        case O_MOTION: pol_motion(&p, o->i1 % 2); ref_motion(r, o->i1 % 2); break;
        case O_INK:
            inking = o->i1 % 3 != 0;
            pol_ink(&p, o->b, now, &a); ref_ink(r, RB(o->b), now, &b); acts = 1;
            break;
        case O_INKED: pol_inked(&p, o->b, now); ref_inked(r, RB(o->b), now); break;
        case O_INK_END: inking = 0; break;
        case O_RECUT: pol_recut(&p, now, &a); ref_recut(r, now, &b); acts = 1; break;
        case O_ENGINE: { int w = (int[]){ -1, 0, 0, 40, 120, 600 }[o->i1 % 6]; pol_engine(&p, w); ref_engine(r, w); break; }
        case O_POINTER: {
            boxlist ch = o->ch; ref_list rc = RL(&o->ch);
            v1 = pol_pointer(&p, &ch, now, &a); v2 = ref_pointer(r, &rc, now, &b); acts = 1;
            /* the list is an output too (the daemon reads it back) */
            int lsame = ch.n == rc.n;
            for (int i = 0; lsame && i < ch.n; i++)
                lsame = ch.b[i].x0 == rc.b[i].x0 && ch.b[i].y0 == rc.b[i].y0 && ch.b[i].x1 == rc.b[i].x1 && ch.b[i].y1 == rc.b[i].y1;
            if (!lsame) { bad = s; snprintf(what, sizeof what, "pointer list %d vs %d boxes", ch.n, rc.n); }
            break;
        }
        case O_RANGE: {
            int lo = o->i1 % 3 == 0 ? -1 : (int)(o->i2 % LV_N), hi = lo < 0 ? -1 : lo + (int)(o->i3 % (LV_N - lo));
            int curve = o->i3 % 3 - 1;
            range_live(&p, &sl, lo, hi, curve); range_ref(r, &sr, lo, hi, curve);
            break;
        }
        case O_FIELDS:
            if (o->i1 % 2) { p.want_full = 1; ref_set(r, RF_WANT_FULL, 1); }
            else { p.cut_cur = -1; ref_set(r, RF_CUT_CUR, -1); }
            break;
        case O_BANNER: pol_banner(&p, o->b); ref_banner(r, RB(o->b)); break;
        }
        if (bad >= 0) break;
        if (!*what || (o->kind != O_CMD && o->kind != O_CFG)) snprintf(what, sizeof what, "%s", op_names[o->kind]);
        if (acts && (v1 != v2 || !same_acts(&a, &b))) { bad = s; break; }
        /* the daemon's loop: a step after every event, the burst it asks for, the timeout */
        a.n = 0; b.n = 0;
        int s1 = pol_step(&p, now, inking, pend_n, first_dmg, &a), s2 = ref_step(r, now, inking, pend_n, first_dmg, &b);
        if (s1 != s2 || !same_acts(&a, &b)) { bad = s; snprintf(what, sizeof what, "step %d vs %d", s1, s2); break; }
        if (!keep) for (int i = 0; i < a.n; i++) if (a.a[i].kind == A_SEND) cov.swf[a.a[i].wf & 3][a.a[i].mode & 3]++;
        if (s1 == STEP_BURST) {
            burst_in in = { now, o->ch, o->pwin, o->pm, o->moving, o->speed, o->busy, o->dither, o->always, o->pages, o->calm };
            ref_burst_in ri = { now, RL(&o->ch), RB(o->pwin), o->pm, o->moving, o->speed, o->busy, o->dither, o->always,
                                o->pages, o->calm };
            a.n = 0; b.n = 0;
            int h1 = pol_burst(&p, &in, &a), h2 = ref_burst(r, &ri, &b);
            if (!keep) { cov.c[h1 & 7]++; for (int i = 0; i < a.n; i++) if (a.a[i].kind == A_SEND) cov.wf[a.a[i].wf & 3][a.a[i].mode & 3]++; }
            if (h1 != h2 || !same_acts(&a, &b)) { bad = s; snprintf(what, sizeof what, "burst C %d vs %d", h1, h2); break; }
        }
        if (s1 != STEP_NONE) pend_n = 0;
        int t1 = pol_timeout(&p, now, inking, pend_n), t2 = ref_timeout(r, now, inking, pend_n);
        if (t1 != t2) {
            /* kind 1, the live side wakes earlier: benign if a step of both at that wake
             * (no events between) does the same (doc); a later wake (kind 2) is a bug */
            int early = t1 >= 0 && (t2 < 0 || t1 < t2), ok = 0;
            if (early) {
                policy pc = p;
                ref_pol *rc = ref_clone(r);
                actions ea = {0}; ref_acts eb = {0};
                int e1 = pol_step(&pc, now + t1, inking, pend_n, first_dmg, &ea), e2 = ref_step(rc, now + t1, inking, pend_n, first_dmg, &eb);
                ok = e1 == e2 && same_acts(&ea, &eb) && strict_state(pc.state) == strict_state(ref_get(rc, RF_STATE));
                ref_free(rc);
            }
            if (!ok) { bad = s; snprintf(what, sizeof what, "timeout %d vs %d (%s)", t1, t2, early ? "kind 1: earlier, NOT benign" : "kind 2: LATER"); break; }
            if (!keep) kind1++;
        }
        int q1 = pol_quiet_held(&p, now), q2 = ref_quiet_held(r, now);
        if (q1 != q2) { bad = s; snprintf(what, sizeof what, "quiet_held %d vs %d", q1, q2); break; }
        if (!keep) cov.st[p.state & 7]++;
        if (strict_state(p.state) != strict_state(ref_get(r, RF_STATE))) {
            bad = s; snprintf(what, sizeof what, "state %s vs %s", state_name(p.state), ref_state_name(r)); break;
        }
    }
    if (bad >= 0 && verbose) {
        printf("  op %ld (%s) at %lld: %s\n", bad, op_names[ops[bad].kind], (long long)ops[bad].now, what);
        print_acts("live", &a, NULL);
        print_acts("frozen", NULL, &b);
    }
    ref_free(r);
    return bad;
}

/* shrinks a diverging trace: drops ops while it still diverges; prints the rest */
static void shrink(const op *ops, long n, int prof)
{
    char *keep = malloc(n);
    memset(keep, 1, n);
    long last = run(ops, n, keep, prof, 0);
    for (long i = n - 1; i > last; i--) keep[i] = 0;           /* nothing after the first divergence matters */
    for (int pass = 0; pass < 3; pass++) {
        int changed = 0;
        for (long i = 0; i <= last; i++) {
            if (!keep[i]) continue;
            keep[i] = 0;
            long b = run(ops, n, keep, prof, 0);
            if (b >= 0) { last = b; changed = 1; } else keep[i] = 1;
        }
        if (!changed) break;
    }
    int k = 0;
    for (long i = 0; i <= last; i++) k += keep[i];
    printf("  minimal trace (%d ops, profile %s):\n", k, profiles[prof].name);
    for (long i = 0; i <= last; i++)
        if (keep[i]) {
            const op *o = &ops[i];
            printf("    [%ld] t=%lld %s i1=%d v=%.2f b=%d,%d-%d,%d ch=%d pm=%d mov=%d busy=%d pages=%d calm=%d\n", i,
                   (long long)o->now, op_names[o->kind], o->i1, o->v, o->b.x0, o->b.y0, o->b.x1, o->b.y1, o->ch.n, o->pm,
                   o->moving, o->busy, o->pages, o->calm);
        }
    run(ops, n, keep, prof, 1);
    free(keep);
}

int main(int argc, char **argv)
{
    int seeds = argc > 1 ? atoi(argv[1]) : 100;
    long steps = argc > 2 ? atol(argv[2]) : 5000;
    selftest = getenv("PARITY2_SELFTEST") ? atoi(getenv("PARITY2_SELFTEST")) : 0;
    op *ops = malloc(steps * sizeof *ops);
    int diverged = 0;
    for (int pr = 0; pr < NPROF && diverged < 3; pr++)
        for (int sd = 1; sd <= seeds && diverged < 3; sd++) {
            gen(ops, steps, (unsigned)(sd * 7919 + pr));
            long b = run(ops, steps, NULL, pr, 1);
            if (b >= 0) {
                printf("DIVERGE profile %s seed %d\n", profiles[pr].name, sd);
                shrink(ops, steps, pr);
                diverged++;
            }
        }
    free(ops);
    if (selftest) {
        printf("parity2 selftest: %s\n", diverged ? "ok (the difference was found)" : "FAILED (no difference found)");
        return diverged ? 0 : 1;
    }
    printf("parity2: %s (%d profiles x %d seeds x %ld steps; %ld benign earlier wakes)\n", diverged ? "FAILED" : "ok", NPROF, seeds, steps, kind1);
    printf("  steps by state: idle %ld cleanup %ld run %ld paused %ld asleep %ld\n", cov.st[S_IDLE], cov.st[S_CLEANUP],
           cov.st[S_RUN], cov.st[S_PAUSED], cov.st[S_ASLEEP]);
    printf("  bursts by commit: all %ld bw %ld dither %ld thresh %ld; sends DU part %ld, GC16 part %ld screen %ld area %ld, DU4 %ld\n",
           cov.c[C_ALL], cov.c[C_BW], cov.c[C_DITHER], cov.c[C_THRESH], cov.wf[WF_DU][M_PART], cov.wf[WF_GC16][M_PART],
           cov.wf[WF_GC16][M_SCREEN], cov.wf[WF_GC16][M_AREA], cov.wf[WF_DU4][M_PART] + cov.wf[WF_DU4][M_AREA]);
    printf("  step sends (cleanups): GC16 part %ld screen %ld area %ld, DU part %ld\n", cov.swf[WF_GC16][M_PART],
           cov.swf[WF_GC16][M_SCREEN], cov.swf[WF_GC16][M_AREA], cov.swf[WF_DU][M_PART]);
    printf("  paths:");
    for (int i = 0; i < NTALLY && tally[i].key[0]; i++) printf(" [%s] %ld", tally[i].key, tally[i].n);
    printf("\n");
    static const char *must[] = { "popup paint", "popup gone paint", "run tail", "page", "busy content", "single mode",
                                  "ink debt", "stale flash", "ink cleanup", "cleanup despite motion", "run end after" };
    for (int i = 0; i < N(must); i++)
        if (!tally_of(must[i])) { printf("parity2: FAILED (no trace took the path \"%s\")\n", must[i]); return 1; }
    if (!cov.st[S_RUN] || !cov.wf[WF_GC16][M_AREA] || !cov.c[C_DITHER]) { printf("parity2: FAILED (the traces never reached a run / a flash / a dithered burst)\n"); return 1; }
    return diverged ? 1 : 0;
}
