/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-bp4.h — stagger_ctl 4 (:
 * the two-tier ctl 3 on bit planes (variant m<M>is of test/epd-tier.h:
 * payback in the maintenance visit, the hammer floor at the arrival, the K
 * idle as a shared mask, op exact in 4 bits, dc saturating at +-127), the
 * representation of the GPU-owned lanes under stagger_state=lanes. This is
 * the CPU twin: the reference the GPU pass (epd-gpu.c, ctl 4) is gated and
 * checked against, and the CPU fallback. G 2 only (fm = f & 1).
 *
 * Layout: plane word k of row y = pixels 32 k .. 32 k + 31, pixel 32 k + j at
 * bit 31 - j (MSB first, as the wire word: pixel i of a wire word at shift
 * 14 - 2 i); BP_NK = 59 words a row, the last one holds 16 pixels (its low
 * half is never in a group: inert). Wire word 4 k + q = byte 3 - q of a plane
 * word, spread to 2 bits a pixel.
 *
 * Per frame (every word): LW, P, TB; OP[4] = q, the frames left to the
 * level's rail (op = LW ? R - q : q; a step counts it down), OW[2] (the owed
 * count), AC[5] (the accumulator, signed), PD (an arrival waits for its
 * payback), RV (it was a reversal), H8 (its hammer floor was set at the
 * arrival), ST (stale, the cold word's bit 11). The visit (the rows y % M ==
 * cycle % M, on the cycle's first frame): DC[8], signed. No greys (the level
 * is a rail).
 *
 * Lanes (bp4_lane_rows): PARK (the lane is the CPU's: its D is the truth), and
 * the requests of T's bits 5-7 as planes ARM, EV (evict), TOG (packed from T
 * by bp4_lane_pack where T changed). On a word's group frame (M0): a parked
 * lane with ARM and no EV takes its D (level >= 16 white, an odd payrule level
 * its rail; q from op clamped to 0..R; dc clamped to +-127; ST from stale; the
 * rest 0) and decides in the same frame; a loaded lane with EV parks (its step
 * is done at its next group frame at G 2: no wait) and writes its final D
 * (the accumulator folded, level LW ? 30 : 0, op, stale, k 0, dc); a byte
 * (8 lanes) that M0 hits, all 8 parked and an EV among them, acks 1 | TOG << 1
 * (TOG: its first EV pixel's) — epd-stagger.c rows_lanes' protocol, unchanged. */
#ifndef EPD_BP4_H
#define EPD_BP4_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "epd-engine.h"
#include "epd-stagger.h"
#include "epd-bluenoise.h"

#define BP_NK ((EPD_W + 31) / 32)               /* plane words a row (59) */
#define BP_NW (BP_NK * EPD_H)
#define BP_GW (EPD_W / 8)                       /* wire words a row (234) */
_Static_assert(EPD_W % 16 == 0 && EPD_W % 32 == 16, "the last plane word holds 16 pixels");

/* the parameters ctl 4 takes (else ctl 3 runs): G 2 n 1, payrule, pcap <= 3,
 * R 1..15, no dither, and a layout whose groups repeat every 64 rows
 * (stripes, cells W x 1, the blue noise) */
static inline int bp4_par_ok(const epd_stag_par *p)
{
    if (p->G != 2 || p->n != 1 || !p->payrule || p->dither || p->R < 1 || p->R > 15 || p->pcap < 0 || p->pcap > 3) return 0;
    if (p->stripe <= 0) return 1;
    if (p->cellw) return p->cellh == 1;
    return !p->blocks;
}
static inline int bp4_vm(const epd_stag_par *p) { return p->vm > 0 ? p->vm : 4; }

/* the group masks: gm[(g * 64 + (y & 63)) * BP_NK + k] (G * 64 rows) */
typedef struct { int G; uint32_t *gm; } bp_masks;

static inline int bp_masks_make(bp_masks *m, const epd_stag_par *p)
{
    m->G = p->G;
    m->gm = calloc((size_t)p->G * 64 * BP_NK, 4);
    if (!m->gm) return -1;
    for (int r = 0; r < 64; r++)
        for (int x = 0; x < EPD_W; x++) {
            int g = epd_stag_grank(p, x, r, epd_bn_rank[r & 63][x & 63]) * p->G >> 12;
            m->gm[((size_t)g * 64 + r) * BP_NK + x / 32] |= 1u << (31 - x % 32);
        }
    return 0;
}
static inline void bp_masks_free(bp_masks *m) { free(m->gm); m->gm = NULL; }

/* 8 bits to 16 (bit m to bit 2 m), two bytes at once (bits 0-7 and 16-23) */
static inline uint32_t bp_spread2(uint32_t v)
{
    v = (v | v << 4) & 0x0F0F0F0Fu;
    v = (v | v << 2) & 0x33333333u;
    return (v | v << 1) & 0x55555555u;
}
/* plane word pair -> wire words 4 k .. 4 k + 3 (q: byte 3 - q) */
static inline uint32_t bp_wire(uint32_t wh, uint32_t bk, int q)
{
    unsigned sh = 24 - 8 * (unsigned)q;
    uint32_t s = bp_spread2(((wh >> sh) & 255u) | ((bk >> sh) & 255u) << 16);
    return (s & 0xFFFFu) << 1 | s >> 16;
}

typedef struct {
    uint32_t *lw, *p, *tb, *pd, *rv, *h8;
    uint32_t *op[4], *ow[2], *ac[5], *dc[8];
    uint32_t *st, *park, *arm, *ev, *tog;       /* (ST and the lanes': not in bp4_planes) */
    uint32_t *kc[3];                            /* (kidle: the full cycles since the last idle, ctl 3's K / 2) */
    int kidle;                                  /* 1: ctl 3's per-pixel K idle (kc); 0: the shared idle mask (the plain gates) */
} bp4_state;
#define BP4_NPL 25                              /* the controller's planes (bp4_planes) */
#define BP4_NALL 33
static inline void bp4_planes(const bp4_state *s, uint32_t **pl)
{
    uint32_t *all[BP4_NPL] = { s->lw, s->p, s->tb, s->pd, s->rv, s->h8, s->op[0], s->op[1], s->op[2], s->op[3], s->ow[0], s->ow[1],
                               s->ac[0], s->ac[1], s->ac[2], s->ac[3], s->ac[4], s->dc[0], s->dc[1], s->dc[2], s->dc[3], s->dc[4], s->dc[5], s->dc[6], s->dc[7] };
    memcpy(pl, all, sizeof all);
}
static inline uint32_t **bp4_slot(bp4_state *s, int i)
{
    uint32_t **all[BP4_NALL] = { &s->lw, &s->p, &s->tb, &s->pd, &s->rv, &s->h8, &s->op[0], &s->op[1], &s->op[2], &s->op[3], &s->ow[0], &s->ow[1],
                                 &s->ac[0], &s->ac[1], &s->ac[2], &s->ac[3], &s->ac[4], &s->dc[0], &s->dc[1], &s->dc[2], &s->dc[3], &s->dc[4], &s->dc[5], &s->dc[6], &s->dc[7],
                                 &s->st, &s->park, &s->arm, &s->ev, &s->tog, &s->kc[0], &s->kc[1], &s->kc[2] };
    return all[i];
}
static inline int bp4_alloc(bp4_state *s)
{
    memset(s, 0, sizeof *s);
    for (int i = 0; i < BP4_NALL; i++) if (!(*bp4_slot(s, i) = calloc(BP_NW, 4))) return -1;
    return 0;
}
static inline void bp4_free(bp4_state *s)
{
    for (int i = 0; i < BP4_NALL; i++) free(*bp4_slot(s, i));
    memset(s, 0, sizeof *s);
}

/* ---- n-bit bit-sliced arithmetic, two's complement (n <= 10) ---- */
typedef struct { uint32_t v[10]; } bp4_v;
static inline bp4_v bp4_c(int c, int n) { bp4_v r; for (int i = 0; i < n; i++) r.v[i] = (c >> i) & 1 ? ~0u : 0; return r; }
static inline bp4_v bp4_ld(uint32_t *const *pl, int np, size_t o, int n, int sgn)   /* (np planes, extended to n) */
{
    bp4_v r;
    for (int i = 0; i < n; i++) r.v[i] = i < np ? pl[i][o] : sgn ? r.v[np - 1] : 0;
    return r;
}
static inline bp4_v bp4_add(bp4_v a, bp4_v b, int n)
{
    bp4_v r; uint32_t c = 0;
    for (int i = 0; i < n; i++) { uint32_t x = a.v[i] ^ b.v[i]; r.v[i] = x ^ c; c = (a.v[i] & b.v[i]) | (x & c); }
    return r;
}
static inline bp4_v bp4_sub(bp4_v a, bp4_v b, int n)
{
    bp4_v r; uint32_t c = ~0u;
    for (int i = 0; i < n; i++) { uint32_t nb = ~b.v[i], x = a.v[i] ^ nb; r.v[i] = x ^ c; c = (a.v[i] & nb) | (x & c); }
    return r;
}
static inline bp4_v bp4_sel(uint32_t m, bp4_v a, bp4_v b, int n) { bp4_v r; for (int i = 0; i < n; i++) r.v[i] = (a.v[i] & m) | (b.v[i] & ~m); return r; }
static inline uint32_t bp4_nz(bp4_v a, int n) { uint32_t r = 0; for (int i = 0; i < n; i++) r |= a.v[i]; return r; }
static inline uint32_t bp4_neg(bp4_v a, int n) { return a.v[n - 1]; }
static inline uint32_t bp4_lt(bp4_v a, bp4_v b, int n) { return bp4_neg(bp4_sub(a, b, n), n); }   /* (signed; no overflow at the widths used) */
static inline uint32_t bp4_eqc(const bp4_v *a, int c, int n) { uint32_t e = ~0u; for (int i = 0; i < n; i++) e &= (c >> i) & 1 ? a->v[i] : ~a->v[i]; return e; }

/* the cycle's visit of one word: the accumulator folds into dc (8 bits,
 * saturating at +-127), and an arrival that has reached its rail gets its
 * payback. Needs pcap <= 3 (bp3_par_ok) and so hc <= 3, paid <= hc. */
static inline void bp4_visit_word(bp4_state *s, const epd_stag_par *p, size_t o)
{
    const int hc = p->htop < p->pcap ? p->htop : p->pcap;
    uint32_t a[5], anz = 0;
    for (int i = 0; i < 5; i++) anz |= a[i] = s->ac[i][o];
    uint32_t pd = s->pd[o];
    if (!anz && !pd) return;                    /* (nothing to fold, nothing waits) */
    uint32_t d[8];
    for (int i = 0; i < 8; i++) d[i] = s->dc[i][o];
    if (anz) {                                  /* d += sext(acc); an overflow or -128 saturates */
        uint32_t c = 0, sm[8];
        for (int i = 0; i < 8; i++) {
            uint32_t bb = a[i < 5 ? i : 4], x = d[i] ^ bb;
            sm[i] = x ^ c;
            c = (d[i] & bb) | (x & c);
        }
        uint32_t ovf = ~(d[7] ^ a[4]) & (sm[7] ^ d[7]);
        uint32_t m128 = sm[7] & ~(sm[0] | sm[1] | sm[2] | sm[3] | sm[4] | sm[5] | sm[6]) & ~ovf;
        uint32_t neg = d[7];                    /* (an overflow keeps the operands' sign) */
        d[0] = sm[0] | ovf | m128;
        for (int i = 1; i < 7; i++) d[i] = (sm[i] & ~ovf) | (~neg & ovf);
        d[7] = (sm[7] & ~ovf) | (neg & ovf);
        for (int i = 0; i < 8; i++) s->dc[i][o] = d[i];
        for (int i = 0; i < 5; i++) s->ac[i][o] = 0;
    }
    uint32_t go = pd & ~(s->op[0][o] | s->op[1][o] | s->op[2][o] | s->op[3][o]);   /* (at its rail: q == 0) */
    if (!go) return;
    /* owe = lw ? -d : d, only as far as clamp(owe + paid, 0, pcap) needs it:
     * >= 3, <= -3, or its exact value in -2..2 (d's low 3 bits) */
    uint32_t lw = s->lw[o];
    uint32_t ge3 = ~d[7] & (d[6] | d[5] | d[4] | d[3] | d[2] | (d[1] & d[0]));
    uint32_t le3 = d[7] & ~(d[6] & d[5] & d[4] & d[3] & d[2] & d[1]);
    uint32_t big = (lw & le3) | (~lw & ge3), small = (lw & ge3) | (~lw & le3);
    uint32_t n0 = d[0], n1 = d[1] ^ d[0], n2 = d[2] ^ (d[1] | d[0]);    /* (-d, 3 bits: ~d + 1) */
    uint32_t x0 = (lw & n0) | (~lw & d[0]), x1 = (lw & n1) | (~lw & d[1]), x2 = (lw & n2) | (~lw & d[2]);
    x0 = (x0 & ~big & ~small) | big | small;    /* 3 = 011, -3 = 101 */
    x1 = (x1 & ~big & ~small) | big;
    x2 = (x2 & ~big & ~small) | small;
    uint32_t h8 = s->h8[o], w0 = s->ow[0][o], w1 = s->ow[1][o];
    bp4_v t = { { x0, x1, x2, x2 } }, paid, z = bp4_c(0, 4), pc = bp4_c(p->pcap, 4), hcv = bp4_c(hc, 4);
    {   bp4_v ow = { { w0, w1, 0, 0 } };
        paid = bp4_sel(h8, bp4_sub(hcv, ow, 4), z, 4); }
    t = bp4_add(t, paid, 4);                    /* (-3..6: fits 4 bits signed) */
    bp4_v bv = bp4_sel(bp4_neg(t, 4) | ~bp4_nz(t, 4), z, bp4_sel(bp4_lt(t, pc, 4), t, pc, 4), 4);
    if (p->htop > 0) bv = bp4_sel(s->rv[o] & bp4_lt(bv, hcv, 4), hcv, bv, 4);
    bv = bp4_sub(bv, paid, 4);
    bv = bp4_sel(bp4_neg(bv, 4), z, bv, 4);
    s->ow[0][o] = (bv.v[0] & go) | (w0 & ~go);
    s->ow[1][o] = (bv.v[1] & go) | (w1 & ~go);
    s->pd[o] &= ~go; s->rv[o] &= ~go; s->h8[o] &= ~go;
}

/* one word's decisions on its group frame (M0: the lanes deciding, MI: the
 * other group, on its idle frame; idle: the shared K idle). Updates the
 * state; the word's drive (the wire's white = drive & LW, black = drive & ~LW) */
static inline uint32_t bp4_word(bp4_state *s, const epd_stag_par *p, size_t o, uint32_t M0, uint32_t MI, uint32_t idle)
{
    const int R = p->R, hc = p->htop < p->pcap ? p->htop : p->pcap;
    uint32_t lw = s->lw[o], P = s->p[o], tb = s->tb[o];
    uint32_t drv = MI & P;
    uint32_t q0 = s->op[0][o], q1 = s->op[1][o], q2 = s->op[2][o], q3 = s->op[3][o], b0 = s->ow[0][o], b1 = s->ow[1][o];
    uint32_t arrive = M0 & (lw ^ tb), qnz = q0 | q1 | q2 | q3;
    if (M0 && !(M0 & ((lw ^ tb) | b0 | b1 | qnz))) {   /* (light: nothing to do) */
        s->p[o] = P & ~M0;
        if (s->kidle && !p->sidle) for (int i = 0; i < 3; i++) s->kc[i][o] &= ~M0;
        return drv;
    }
    if (!M0) return drv;
    /* need: q (frames to the level's rail), or R - q at an arrival (the far rail) */
    uint32_t n[4], qv[4] = { q0, q1, q2, q3 }, br = 0;
    for (int i = 0; i < 4; i++) {               /* R - q, 4 bits (q <= R: no borrow out) */
        uint32_t r = ((unsigned)R >> i) & 1 ? ~0u : 0, d = r ^ qv[i] ^ br;
        br = (~r & (qv[i] | br)) | (qv[i] & br);
        n[i] = (d & arrive) | (qv[i] & ~arrive);
    }
    uint32_t n1 = n[0] | n[1] | n[2] | n[3], n2 = n[1] | n[2] | n[3];
    uint32_t h8 = p->htop > 0 ? arrive & qnz : 0;          /* a reversal: it had not reached its rail */
    uint32_t c0 = (hc & 1) ? h8 : 0, c1 = (hc & 2) ? h8 : 0;
    b0 = (c0 & arrive) | (b0 & ~arrive); b1 = (c1 & arrive) | (b1 & ~arrive);
    uint32_t bnz = b0 | b1;
    uint32_t act = M0 & (arrive | n1 | bnz);
    uint32_t ge1 = n1 | bnz, ge2 = n2 | b1 | (n1 & bnz);
    if (s->kidle && !p->sidle) {                             /* ctl 3's K (epd-stagger.c px): a full cycle idles when 2 kc + 2 > K, kc >= K / 2 */
        unsigned T = (unsigned)p->K / 2;
        uint32_t br = 0;
        for (int i = 0; i < 3; i++) { uint32_t a_ = s->kc[i][o], b_ = (T >> i) & 1 ? ~0u : 0; br = (~a_ & b_) | (~(a_ ^ b_) & br); }
        idle = ~br;
    }
    uint32_t nn2 = ge2 & ~idle, nn1 = ge1 & ~nn2;
    /* pay = max(nn - need, 0): need 0 pays nn, need 1 pays 1 of a 2 */
    uint32_t p1 = (~n1 & nn1) | (n1 & ~n2 & nn2), p2 = ~n1 & nn2;
    uint32_t r0 = b0 ^ p1, bw = ~b0 & p1, r1 = b1 ^ p2 ^ bw;
    /* q' = max(need - nn, 0) */
    uint32_t qn[4], bo = 0, sb[4] = { nn1, nn2, 0, 0 };
    for (int i = 0; i < 4; i++) {
        uint32_t d = n[i] ^ sb[i] ^ bo;
        bo = (~n[i] & (sb[i] | bo)) | (sb[i] & bo);
        qn[i] = d;
    }
    for (int i = 0; i < 4; i++) qn[i] &= ~bo;  /* (a borrow out: below 0) */
    /* acc += tb ? nn : -nn, 5 bits */
    uint32_t nnz = nn1 | nn2, v[5] = { nn1, nn2 | (~tb & nn1), ~tb & nnz, ~tb & nnz, ~tb & nnz }, cy = 0;
    for (int i = 0; i < 5; i++) {
        uint32_t a_ = s->ac[i][o], x = a_ ^ v[i];
        uint32_t sum = x ^ cy;
        cy = (a_ & v[i]) | (x & cy);
        s->ac[i][o] = (sum & act) | (a_ & ~act);
    }
    for (int i = 0; i < 4; i++) s->op[i][o] = (qn[i] & act) | (qv[i] & ~act);
    s->ow[0][o] = (r0 & act) | (s->ow[0][o] & ~act);
    s->ow[1][o] = (r1 & act) | (s->ow[1][o] & ~act);
    s->pd[o] |= arrive;
    s->rv[o] = (s->rv[o] & ~arrive) | (arrive & qnz);
    s->h8[o] = (s->h8[o] & ~arrive) | h8;
    s->st[o] |= act & (nnz | arrive);           /* (px: a step or a new level makes it stale) */
    lw = (lw & ~act) | (tb & act);
    s->lw[o] = lw;
    s->p[o] = (P & ~M0) | (act & nn2);
    if (s->kidle && !p->sidle) {                             /* kc: + 1 on a full cycle, else 0 (the word's deciders) */
        uint32_t inc = act & nn2, c = inc;
        for (int i = 0; i < 3; i++) { uint32_t a_ = s->kc[i][o], sm = a_ ^ c; c = a_ & c; s->kc[i][o] = (sm & inc) | (a_ & ~M0); }
    }
    return drv | (act & ge1);
}

/* the shared K idle: word k of row y idles on cycle c when (c + (k + y) % 5) % 5 == 4 */
static inline uint32_t bp4_idle(uint32_t c, int k, int y) { return (c + (uint32_t)(k + y) % 5) % 5 == 4 ? ~0u : 0; }
/* the row's wire offset in an EPD_WORDS frame */
static inline size_t bp4_wo(int y) { return (size_t)(EPD_DATA0 / EPD_LINE + EPD_H - 1 - y) * EPD_LINE + EPD_DATA0 % EPD_LINE; }
static inline int bp4_visits(int M, uint32_t f, int y) { return M > 0 && f % 2 == 0 && (uint32_t)y % (uint32_t)M == (f / 2) % (uint32_t)M; }

#ifdef EPD_BP4_TV
static double *bp4_tv;                          /* (tests: when set, the visits' time is added here) */
#endif
/* frame f (cycle f / 2) over rows [y0, y1): the visit (M > 0, the cycle's
 * first frame, its rows), then the decisions and the drive; the words.
 * Every lane loaded (no lanes: runs, or the gate) */
static inline void bp4_rows(bp4_state *s, const bp_masks *m, const epd_stag_par *p, int M, uint32_t f, const uint32_t *tpl, uint32_t *words, int y0, int y1)
{
    int fm = (int)(f & 1), fi = fm ^ 1;
    for (int y = y0; y < y1; y++) {
        const uint32_t *m0 = m->gm + ((size_t)fm * 64 + (y & 63)) * BP_NK, *mi = m->gm + ((size_t)fi * 64 + (y & 63)) * BP_NK;
        size_t row = (size_t)y * BP_NK, wo = bp4_wo(y);
        if (bp4_visits(M, f, y)) {              /* (the row's visit first: words are independent) */
#ifdef EPD_BP4_TV
            double t0 = bp4_tv ? now() : 0;
#endif
            for (int k = 0; k < BP_NK; k++) bp4_visit_word(s, p, row + k);
#ifdef EPD_BP4_TV
            if (bp4_tv) *bp4_tv += now() - t0;
#endif
        }
        for (int k = 0; k < BP_NK; k++) {
            uint32_t drv = bp4_word(s, p, row + k, m0[k], mi[k], bp4_idle(f / 2, k, y)), lw = s->lw[row + k];
            for (int q = 0; q < 4 && 4 * k + q < BP_GW; q++)
                words[wo + 4 * k + q] = tpl[wo + 4 * k + q] | bp_wire(drv & lw, drv & ~lw, q);
        }
    }
}

/* ---- lanes ---- */

/* T's bytes over r (panel pixels; T rows EPD_PK_S apart) into TB, ARM, EV,
 * TOG of every plane word r touches (whole words: T holds every pixel) */
static inline void bp4_lane_pack(bp4_state *s, const uint8_t *T, epd_rect r)
{
    if (r.w <= 0 || r.h <= 0) return;
    for (int y = r.y; y < r.y + r.h; y++)
        for (int k = r.x / 32; k <= (r.x + r.w - 1) / 32; k++) {
            const uint8_t *t = T + (size_t)y * EPD_PK_S + 32 * k;
            uint32_t tb = 0, a = 0, e = 0, g = 0;
            for (int j = 0; j < 32 && 32 * k + j < EPD_W; j++) {
                unsigned b = t[j], bit = 1u << (31 - j);
                tb |= (b & 31) >= 16 ? bit : 0;
                a |= b & EPD_PK_ARM ? bit : 0;
                e |= b & EPD_PK_EVICT ? bit : 0;
                g |= b & EPD_PK_TOG ? bit : 0;
            }
            size_t o = (size_t)y * BP_NK + k;
            s->tb[o] = tb; s->arm[o] = a; s->ev[o] = e; s->tog[o] = g;
        }
}
/* every lane parked, nothing requested (a run's start, and after a loss) */
static inline void bp4_lane_park(bp4_state *s)
{
    for (int i = 0; i < BP4_NALL; i++) memset(*bp4_slot(s, i), 0, BP_NW * 4);
    memset(s->park, 0xFF, BP_NW * 4);
    for (int y = 0; y < EPD_H; y++) s->park[(size_t)y * BP_NK + BP_NK - 1] = 0xFFFF0000u;   /* (past the panel: no lane) */
}

static inline void bp4_setbit(uint32_t *pl, size_t o, uint32_t bit, int v) { pl[o] = v ? pl[o] | bit : pl[o] & ~bit; }
/* arm: the lanes of a (one word, row y) take their D (rows EPD_PK_S apart) */
static inline void bp4_lane_arm(bp4_state *s, const epd_stag_par *p, size_t o, int y, int k, uint32_t a, const uint32_t *D)
{
    for (uint32_t m = a; m; ) {
        int j = __builtin_clz(m);
        m &= ~(0x80000000u >> j);
        uint32_t bit = 1u << (31 - j), c = D[(size_t)y * EPD_PK_S + 32 * k + j];
        int l = EPD_STAG_LEVEL(c), lw = l >= 15;           /* (a grey >= 16 white; an odd payrule level: its rail, epd_stag_ldec) */
        int op = EPD_STAG_OP(c), dc = EPD_STAG_DC(c);
        op = op > p->R ? p->R : op;
        int q = lw ? p->R - op : op;
        dc = dc > 127 ? 127 : dc < -127 ? -127 : dc;
        bp4_setbit(s->lw, o, bit, lw);
        bp4_setbit(s->st, o, bit, EPD_STAG_STALE(c));
        for (int i = 0; i < 4; i++) bp4_setbit(s->op[i], o, bit, (q >> i) & 1);
        for (int i = 0; i < 8; i++) bp4_setbit(s->dc[i], o, bit, (dc >> i) & 1);
    }
    s->p[o] &= ~a; s->pd[o] &= ~a; s->rv[o] &= ~a; s->h8[o] &= ~a;
    for (int i = 0; i < 3; i++) s->kc[i][o] &= ~a;
    for (int i = 0; i < 2; i++) s->ow[i][o] &= ~a;
    for (int i = 0; i < 5; i++) s->ac[i][o] &= ~a;
}
/* evict: the lanes of e park, their final D written (the accumulator folded,
 * saturating at +-127; k 0); their planes cleared (the visit skips them) */
static inline void bp4_lane_evict(bp4_state *s, const epd_stag_par *p, size_t o, int y, int k, uint32_t e, uint32_t *D)
{
    for (uint32_t m = e; m; ) {
        int j = __builtin_clz(m), sh = 31 - j, q = 0, ac = 0, dc = 0;
        m &= ~(1u << sh);
        for (int i = 0; i < 4; i++) q |= (int)((s->op[i][o] >> sh) & 1) << i;
        for (int i = 0; i < 5; i++) ac |= (int)((s->ac[i][o] >> sh) & 1) << i;
        for (int i = 0; i < 8; i++) dc |= (int)((s->dc[i][o] >> sh) & 1) << i;
        dc = (int)(int8_t)dc + (ac >= 16 ? ac - 32 : ac);
        dc = dc > 127 ? 127 : dc < -127 ? -127 : dc;
        for (int i = 0; i < 8; i++) bp4_setbit(s->dc[i], o, 1u << sh, (dc >> i) & 1);
        int lw = (int)((s->lw[o] >> sh) & 1);
        D[(size_t)y * EPD_PK_S + 32 * k + j] = epd_stag_cold(lw ? 30 : 0, lw ? p->R - q : q, (int)((s->st[o] >> sh) & 1), dc);
    }
    s->p[o] &= ~e; s->pd[o] &= ~e; s->rv[o] &= ~e; s->h8[o] &= ~e;
    for (int i = 0; i < 3; i++) s->kc[i][o] &= ~e;
    for (int i = 0; i < 2; i++) s->ow[i][o] &= ~e;
    for (int i = 0; i < 5; i++) s->ac[i][o] &= ~e;
    s->park[o] |= e;
}
/* the acks of word o (row y) after its frame: per byte M0 hit, all 8 parked,
 * an EV among them (ack rows EPD_W / 8 apart) */
static inline void bp4_lane_ack(const bp4_state *s, size_t o, int y, int k, uint32_t M0, uint8_t *ack)
{
    uint32_t pk = s->park[o], ev = s->ev[o];
    for (int q = 0; q < 4 && 4 * k + q < BP_GW; q++) {
        unsigned sh = 24 - 8 * (unsigned)q;
        uint32_t eb = (ev >> sh) & 255u;
        if (!((M0 >> sh) & 255u) || ((pk >> sh) & 255u) != 255u || !eb) continue;
        unsigned fb = 31 - (unsigned)__builtin_clz(eb);                  /* (the first pixel: the byte's top bit set) */
        ack[(size_t)y * BP_GW + 4 * k + q] = (uint8_t)(1 | ((s->tog[o] >> (sh + fb)) & 1) << 1);
    }
}

/* frame f over rows [y0, y1) under lanes: the visit, then per word arm (from
 * Din), evict (into Dout), the decisions of the loaded lanes, the acks
 * (ack: NULL none), the drive; the words */
static inline void bp4_lane_rows(bp4_state *s, const bp_masks *m, const epd_stag_par *p, int M, uint32_t f, const uint32_t *tpl, uint32_t *words,
                          int y0, int y1, const uint32_t *Din, uint32_t *Dout, uint8_t *ack)
{
    int fm = (int)(f & 1), fi = fm ^ 1;
    for (int y = y0; y < y1; y++) {
        const uint32_t *m0 = m->gm + ((size_t)fm * 64 + (y & 63)) * BP_NK, *mi = m->gm + ((size_t)fi * 64 + (y & 63)) * BP_NK;
        size_t row = (size_t)y * BP_NK, wo = bp4_wo(y);
        if (bp4_visits(M, f, y))
            for (int k = 0; k < BP_NK; k++) bp4_visit_word(s, p, row + k);
        for (int k = 0; k < BP_NK; k++) {
            size_t o = row + k;
            uint32_t M0 = m0[k], pk = s->park[o], ev = s->ev[o];
            uint32_t a = M0 & pk & s->arm[o] & ~ev, e = M0 & ~pk & ev;
            if (a) { bp4_lane_arm(s, p, o, y, k, a, Din); s->park[o] = pk &= ~a; }
            if (e) { bp4_lane_evict(s, p, o, y, k, e, Dout); pk |= e; }
            uint32_t drv = bp4_word(s, p, o, M0 & ~pk, mi[k], bp4_idle(f / 2, k, y)), lw = s->lw[o];
            if (ack && M0 && ev) bp4_lane_ack(s, o, y, k, M0, ack);
            for (int q = 0; q < 4 && 4 * k + q < BP_GW; q++)
                words[wo + 4 * k + q] = tpl[wo + 4 * k + q] | bp_wire(drv & lw, drv & ~lw, q);
        }
    }
}

#endif
