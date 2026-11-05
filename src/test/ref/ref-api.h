/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* parity2's view of the FROZEN policy (test/ref/epd-policy.{c,h}: verbatim
 * copies at the last approved behaviour, see README.md; never edit them). Plain types only: this
 * header sees neither policy header, so the live and the frozen structs can
 * differ. wrap.c translates; when the live API changes, wrap.c changes, the
 * frozen files never. */
#ifndef REF_API_H
#define REF_API_H
#include <stdint.h>

#define REF_MAXB 12                     /* the frozen MAXBOX */
#define REF_MAXA 48                     /* the frozen MAXACT */
typedef struct { int x0, y0, x1, y1; } ref_box;
typedef struct { ref_box b[REF_MAXB]; int n; } ref_list;
typedef struct { int kind; ref_box b; int wf, mode, n; } ref_act;
typedef struct { ref_act a[REF_MAXA]; int n; } ref_acts;
typedef struct { int64_t now; ref_list ch; ref_box pwin; int pm, moving; double speed;
                 int busy, dither, always, pages, calm; } ref_burst_in;
typedef struct ref_pol ref_pol;

/* the fields the daemon reads or writes directly (epd-policy statechart doc) */
enum { RF_STATE, RF_HAS_ENGINE, RF_WHOLE, RF_WANT_FULL, RF_CUT_CUR, RF_M_MIN, RF_M_MAX,
       RF_STILL_RANGE, RF_M_CURVE, RF_LOG /* set: a no-op log (symmetry with a logging live side) */ };

ref_pol *ref_new(int W, int H);
ref_pol *ref_clone(const ref_pol *r);     /* (parity2's kind-1 triage: a step on a copy) */
void ref_free(ref_pol *r);
int ref_get(const ref_pol *r, int field);
void ref_set(ref_pol *r, int field, int v);
const char *ref_state_name(const ref_pol *r);

void ref_config(ref_pol *r, const char *cmd);
int ref_cmd(ref_pol *r, const char *cmd, int64_t now, ref_acts *out);
void ref_banner(ref_pol *r, ref_box b);
void ref_wake(ref_pol *r, ref_box b, int64_t now, ref_acts *out);
void ref_engine(ref_pol *r, int wait_ms);
void ref_finger(ref_pol *r, int64_t now, int down, double v);
void ref_damage(ref_pol *r, int64_t now);
void ref_input(ref_pol *r, int64_t now);
void ref_wheel(ref_pol *r, int64_t now);
void ref_window(ref_pol *r, ref_box b, int64_t now);
void ref_settled(ref_pol *r, int64_t now);
void ref_motion(ref_pol *r, int on);
void ref_popup(ref_pol *r, ref_box b, int64_t now);
void ref_popup_gone(ref_pol *r, ref_box b, int64_t now);
void ref_ink(ref_pol *r, ref_box b, int64_t now, ref_acts *out);
void ref_inked(ref_pol *r, ref_box b, int64_t now);
void ref_recut(ref_pol *r, int64_t now, ref_acts *out);
int ref_step(ref_pol *r, int64_t now, int inking, int pend_n, int64_t first_dmg, ref_acts *out);
int ref_burst(ref_pol *r, const ref_burst_in *in, ref_acts *out);
int ref_pointer(ref_pol *r, ref_list *ch, int64_t now, ref_acts *out);
int ref_timeout(const ref_pol *r, int64_t now, int inking, int pend_n);
int ref_quiet_held(const ref_pol *r, int64_t now);
#endif
