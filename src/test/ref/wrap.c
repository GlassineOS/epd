/* the frozen policy (epd-policy.c/.h in this directory, verbatim) under ref_*
 * names, behind ref-api.h's plain types. Every external symbol of the frozen
 * file is renamed here so it links beside the live epd-policy.c. */
#define box_empty fz_box_empty
#define box_union fz_box_union
#define box_clip fz_box_clip
#define box_inside fz_box_inside
#define box_area fz_box_area
#define list_add fz_list_add
#define list_fold fz_list_fold
#define state_name fz_state_name
#define pol_init fz_pol_init
#define pol_config fz_pol_config
#define pol_cmd fz_pol_cmd
#define pol_banner fz_pol_banner
#define pol_wake fz_pol_wake
#define pol_engine fz_pol_engine
#define pol_finger fz_pol_finger
#define pol_damage fz_pol_damage
#define pol_input fz_pol_input
#define pol_wheel fz_pol_wheel
#define pol_window fz_pol_window
#define pol_settled fz_pol_settled
#define pol_motion fz_pol_motion
#define pol_popup fz_pol_popup
#define pol_popup_gone fz_pol_popup_gone
#define pol_ink fz_pol_ink
#define pol_inked fz_pol_inked
#define pol_recut fz_pol_recut
#define pol_step fz_pol_step
#define pol_burst fz_pol_burst
#define pol_pointer fz_pol_pointer
#define pol_timeout fz_pol_timeout
#define pol_quiet_held fz_pol_quiet_held
#define pol_debounce_left fz_pol_debounce_left
#define pol_debounce_clip fz_pol_debounce_clip
#define pol_tap fz_pol_tap
#include "epd-policy.c"
#include <stdlib.h>
#include "ref-api.h"

struct ref_pol { policy p; };
static void nolog(const char *fmt, ...) { (void)fmt; }

static box bx(ref_box b) { return (box){ b.x0, b.y0, b.x1, b.y1 }; }
static ref_box rb(box b) { return (ref_box){ b.x0, b.y0, b.x1, b.y1 }; }
static void out_acts(const actions *a, ref_acts *o)
{
    o->n = a->n;
    for (int i = 0; i < a->n; i++)
        o->a[i] = (ref_act){ a->a[i].kind, rb(a->a[i].b), a->a[i].wf, a->a[i].mode, a->a[i].n };
}
static boxlist in_list(const ref_list *l)
{
    boxlist b = {0};
    b.n = l->n;
    for (int i = 0; i < l->n; i++) b.b[i] = bx(l->b[i]);
    return b;
}

ref_pol *ref_new(int W, int H) { ref_pol *r = calloc(1, sizeof *r); pol_init(&r->p, W, H); return r; }
ref_pol *ref_clone(const ref_pol *r) { ref_pol *c = malloc(sizeof *c); *c = *r; return c; }
void ref_free(ref_pol *r) { free(r); }
int ref_get(const ref_pol *r, int f)
{
    switch (f) {
    case RF_STATE: return r->p.state;
    case RF_HAS_ENGINE: return r->p.has_engine;
    case RF_WHOLE: return r->p.whole;
    case RF_WANT_FULL: return r->p.want_full;
    case RF_CUT_CUR: return r->p.cut_cur;
    case RF_M_MIN: return r->p.m_min;
    case RF_M_MAX: return r->p.m_max;
    case RF_STILL_RANGE: return r->p.still_range;
    case RF_M_CURVE: return r->p.m_curve;
    }
    abort();
}
void ref_set(ref_pol *r, int f, int v)
{
    switch (f) {
    case RF_HAS_ENGINE: r->p.has_engine = v; return;
    case RF_WHOLE: r->p.whole = v; return;
    case RF_WANT_FULL: r->p.want_full = v; return;
    case RF_CUT_CUR: r->p.cut_cur = v; return;
    case RF_M_MIN: r->p.m_min = v; return;
    case RF_M_MAX: r->p.m_max = v; return;
    case RF_STILL_RANGE: r->p.still_range = v; return;
    case RF_M_CURVE: r->p.m_curve = v; return;
    case RF_LOG: r->p.log = v ? nolog : NULL; return;
    }
    abort();
}
const char *ref_state_name(const ref_pol *r) { return state_name(r->p.state); }

void ref_config(ref_pol *r, const char *cmd) { pol_config(&r->p, cmd); }
int ref_cmd(ref_pol *r, const char *cmd, int64_t now, ref_acts *o)
{ actions a = {0}; int w = pol_cmd(&r->p, cmd, now, &a); out_acts(&a, o); return w; }
void ref_banner(ref_pol *r, ref_box b) { pol_banner(&r->p, bx(b)); }
void ref_wake(ref_pol *r, ref_box b, int64_t now, ref_acts *o)
{ actions a = {0}; pol_wake(&r->p, bx(b), now, &a); out_acts(&a, o); }
void ref_engine(ref_pol *r, int w) { pol_engine(&r->p, w); }
void ref_finger(ref_pol *r, int64_t now, int down, double v) { pol_finger(&r->p, now, down, v); }
void ref_damage(ref_pol *r, int64_t now) { pol_damage(&r->p, now); }
void ref_input(ref_pol *r, int64_t now) { pol_input(&r->p, now); }
void ref_wheel(ref_pol *r, int64_t now) { pol_wheel(&r->p, now); }
void ref_window(ref_pol *r, ref_box b, int64_t now) { pol_window(&r->p, bx(b), now); }
void ref_settled(ref_pol *r, int64_t now) { pol_settled(&r->p, now); }
void ref_motion(ref_pol *r, int on) { pol_motion(&r->p, on); }
void ref_popup(ref_pol *r, ref_box b, int64_t now) { pol_popup(&r->p, bx(b), now); }
void ref_popup_gone(ref_pol *r, ref_box b, int64_t now) { pol_popup_gone(&r->p, bx(b), now); }
void ref_ink(ref_pol *r, ref_box b, int64_t now, ref_acts *o)
{ actions a = {0}; pol_ink(&r->p, bx(b), now, &a); out_acts(&a, o); }
void ref_inked(ref_pol *r, ref_box b, int64_t now) { pol_inked(&r->p, bx(b), now); }
void ref_recut(ref_pol *r, int64_t now, ref_acts *o)
{ actions a = {0}; pol_recut(&r->p, now, &a); out_acts(&a, o); }
int ref_step(ref_pol *r, int64_t now, int inking, int pend_n, int64_t first, ref_acts *o)
{ actions a = {0}; int s = pol_step(&r->p, now, inking, pend_n, first, &a); out_acts(&a, o); return s; }
int ref_burst(ref_pol *r, const ref_burst_in *i, ref_acts *o)
{
    burst_in in = { i->now, in_list(&i->ch), bx(i->pwin), i->pm, i->moving, i->speed, i->busy, i->dither,
                    i->always, i->pages, i->calm };
    actions a = {0};
    int h = pol_burst(&r->p, &in, &a);
    out_acts(&a, o);
    return h;
}
int ref_pointer(ref_pol *r, ref_list *ch, int64_t now, ref_acts *o)
{
    boxlist l = in_list(ch);
    actions a = {0};
    int v = pol_pointer(&r->p, &l, now, &a);
    ch->n = l.n;
    for (int i = 0; i < l.n; i++) ch->b[i] = rb(l.b[i]);
    out_acts(&a, o);
    return v;
}
int ref_timeout(const ref_pol *r, int64_t now, int inking, int pend_n) { return pol_timeout(&r->p, now, inking, pend_n); }
int ref_quiet_held(const ref_pol *r, int64_t now) { return pol_quiet_held(&r->p, now); }
