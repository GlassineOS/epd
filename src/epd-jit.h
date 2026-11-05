/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-jit.h - the stream thread's just-in-time render lead (epd-drm.c,
 * jit=on): frame k+1 is rendered starting rt_hi + JIT_MARGIN before frame k's
 * flip event, rt_hi = the recent worst render (+ copy) time.
 *
 * jit_size=off (the old one): ONE decaying max over every
 * frame, 0.98 a frame (deliberately conservative: a missed flip costs a whole
 * frame). One slow frame (a scroll's whole-panel frame, a
 * stagger frame, a GPU hiccup) then kept every later frame's render ~that
 * much early: 0.98^35 ~ 1/2, so a +20 ms frame still held +10 ms after 35
 * frames. A pen piece submitted in that window waited a whole frame more
 * (step 1).
 *
 * jit_size=on (default): the same decaying max (0.98 per sample), kept per
 * work size: the rows of the frame's runs (render and copy go by whole panel
 * lines) — up to EPD_JIT_SMALL rows (pen pieces), up to EPD_JIT_MID, larger
 * (or a raw frame), and stagger frames apart. The lead is predicted from the
 * NEXT frame's work: the runs still going plus the updates pending, so a big
 * update already submitted starts its frame early by its own size's value,
 * however many small frames came before (a size's max decays only with its own
 * samples). The stream thread also wakes on a submit during the JIT wait and
 * re-predicts: a big update arriving mid-wait starts at once instead of at the
 * small lead. A size with no sample yet borrows the next larger size's value,
 * else the old single max (always kept). Cheap: a union of row spans + a few
 * compares per frame. */
#ifndef EPD_JIT_H
#define EPD_JIT_H

#define EPD_JIT_SMALL 176       /* rows: 1/8 of the panel's 1404 (a pen piece, a cursor) */
#define EPD_JIT_MID 702         /* rows: half the panel */
#define EPD_JIT_STG 3           /* the stagger's frames */
#define EPD_JIT_N 4
#define EPD_JIT_DECAY 0.98
#define EPD_JIT_FLOOR 1e-3      /* s */
#define EPD_JIT_INIT 3e-3       /* s */

typedef struct {
    double one;                 /* s: the old single decaying max (jit_size=off; the fallback) */
    double hi[EPD_JIT_N];       /* s: per size */
    int seen[EPD_JIT_N];
} epd_jit;

static inline void epd_jit_init(epd_jit *j)
{
    j->one = EPD_JIT_INIT;
    for (int i = 0; i < EPD_JIT_N; i++) { j->hi[i] = EPD_JIT_INIT; j->seen[i] = 0; }
}

/* a frame's size: rows = the row span of its runs (+ pending updates);
 * stg = a stagger frame; raw = an epd_stream_raw frame (counted large) */
static inline int epd_jit_class(int rows, int stg, int raw)
{
    if (stg) return EPD_JIT_STG;
    if (raw) return 2;
    return rows <= EPD_JIT_SMALL ? 0 : rows <= EPD_JIT_MID ? 1 : 2;
}

/* s: the render lead of a frame of class c (size: jit_size=on) */
static inline double epd_jit_lead(const epd_jit *j, int c, int size)
{
    if (!size) return j->one;
    if (j->seen[c]) return j->hi[c];
    if (c < EPD_JIT_STG)
        for (int i = c + 1; i < EPD_JIT_STG; i++) if (j->seen[i]) return j->hi[i];
    return j->one;
}

static inline double epd_jit_max(double hi, double rt)
{
    hi = rt > hi * EPD_JIT_DECAY ? rt : hi * EPD_JIT_DECAY;     /* decaying max */
    return hi < EPD_JIT_FLOOR ? EPD_JIT_FLOOR : hi;
}

/* a frame of class c took rt s to render + copy */
static inline void epd_jit_add(epd_jit *j, int c, double rt)
{
    j->one = epd_jit_max(j->one, rt);
    j->hi[c] = j->seen[c] ? epd_jit_max(j->hi[c], rt) : (rt < EPD_JIT_FLOOR ? EPD_JIT_FLOOR : rt);
    j->seen[c] = 1;
}

#endif
