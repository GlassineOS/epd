/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-bp3.h — the stagger's A2 pass at ctl 3 (payrule, the panel's payback,
 * the K idle, the rail hammer) with the state as bit planes: the CPU twin of
 * (stage 2: these planes are the state). Test code, included by
 * epd-stagger-test.c after epd-bitplane.h (its layout, masks, wire spread).
 *
 * Semantics: exactly epd-stagger.c px at ctl 3, payrule 1, G 2, dither 0, no
 * drain, every lane armed (runs), R <= 15, pcap <= 3, K <= 9 — except the DC
 * record saturates at +-127 (the gate clamps the reference the same way and
 * counts the pixels it had to clamp).
 * Per pixel (bit j of word k = pixel 32 k + j, MSB first, as epd-bitplane.h):
 * hot LW level white (and the step's direction), LG level grey (an
 * engine grey, b 0), P the last decision pulsed whole (nn == 2: the
 * second pulse on the idle frame, and the kin gate), ST stale
 * cold B[2] the payback budget b, OP[4] the panel model, KC[3] k / 2 (k is
 * always a multiple of G), DC[8] the record, two's complement
 * A word is light on a frame when no deciding lane (M0) has cold work: then
 * only the hot planes move. */
#ifndef EPD_BP3_H
#define EPD_BP3_H

#define BP3_NB 10                               /* the intermediates' width (signed) */
typedef struct {
    uint32_t *lw, *lg, *p, *st, *tb;
    uint32_t *b[2], *op[4], *kc[3], *dc[8];
} bp3_state;

static inline int bp3_alloc(bp3_state *s)
{
    uint32_t **all[] = { &s->lw, &s->lg, &s->p, &s->st, &s->tb, &s->b[0], &s->b[1], &s->op[0], &s->op[1], &s->op[2], &s->op[3],
                         &s->kc[0], &s->kc[1], &s->kc[2], &s->dc[0], &s->dc[1], &s->dc[2], &s->dc[3], &s->dc[4], &s->dc[5], &s->dc[6], &s->dc[7] };
    for (size_t i = 0; i < sizeof all / sizeof *all; i++) if (!(*all[i] = calloc(BP_NW, 4))) return -1;
    return 0;
}
static inline void bp3_free(bp3_state *s)
{
    free(s->lw); free(s->lg); free(s->p); free(s->st); free(s->tb);
    for (int i = 0; i < 2; i++) free(s->b[i]);
    for (int i = 0; i < 4; i++) free(s->op[i]);
    for (int i = 0; i < 3; i++) free(s->kc[i]);
    for (int i = 0; i < 8; i++) free(s->dc[i]);
    memset(s, 0, sizeof *s);
}
#define BP3_NPL 22                              /* the planes, TB included */
static inline void bp3_planes(const bp3_state *s, uint32_t **pl)
{
    uint32_t *all[BP3_NPL] = { s->lw, s->lg, s->p, s->st, s->tb, s->b[0], s->b[1], s->op[0], s->op[1], s->op[2], s->op[3],
                               s->kc[0], s->kc[1], s->kc[2], s->dc[0], s->dc[1], s->dc[2], s->dc[3], s->dc[4], s->dc[5], s->dc[6], s->dc[7] };
    memcpy(pl, all, sizeof all);
}
static inline void bp3_copy(bp3_state *d, const bp3_state *s)
{
    uint32_t *a[BP3_NPL], *b[BP3_NPL];
    bp3_planes(d, a); bp3_planes(s, b);
    for (int i = 0; i < BP3_NPL; i++) memcpy(a[i], b[i], BP_NW * 4);
}
static inline long bp3_diff(const bp3_state *x, const bp3_state *y)     /* (plane words that differ) */
{
    uint32_t *a[BP3_NPL], *b[BP3_NPL];
    long d = 0;
    bp3_planes(x, a); bp3_planes(y, b);
    for (int i = 0; i < BP3_NPL; i++) for (size_t o = 0; o < BP_NW; o++) d += a[i][o] != b[i][o];
    return d;
}
static inline int bp3_par_ok(const epd_stag_par *p)
{
    if (p->G != 2 || p->ctl != 3 || !p->payrule || p->dither || p->drain || p->lanes > 1) return 0;
    if (p->R < 1 || p->R > 15 || p->pcap < 0 || p->pcap > 3 || p->K < 2 || p->K > 9) return 0;
    if (p->stripe <= 0) return 1;
    if (p->cellw) return p->cellh == 1;
    return !p->blocks;
}

/* one pixel's state, as the reference's fields */
static inline void bp3_put(bp3_state *s, int x, int y, int l, int b, int op, int st, int kc, int dc, int pp)
{
    size_t o = (size_t)y * BP_NK + x / 32;
    uint32_t m = 1u << (31 - x % 32);
#define BP3_SET(pl, v) ((pl)[o] = (v) ? (pl)[o] | m : (pl)[o] & ~m)
    BP3_SET(s->lw, l == 30); BP3_SET(s->lg, l != 0 && l != 30); BP3_SET(s->p, pp); BP3_SET(s->st, st);
    for (int i = 0; i < 2; i++) BP3_SET(s->b[i], (b >> i) & 1);
    for (int i = 0; i < 4; i++) BP3_SET(s->op[i], (op >> i) & 1);
    for (int i = 0; i < 3; i++) BP3_SET(s->kc[i], (kc >> i) & 1);
    for (int i = 0; i < 8; i++) BP3_SET(s->dc[i], (dc >> i) & 1);
#undef BP3_SET
}
static inline int bp3_bit(const uint32_t *pl, size_t o, unsigned sh) { return (int)((pl[o] >> sh) & 1); }
static inline void bp3_get(const bp3_state *s, int x, int y, int *l, int *b, int *op, int *st, int *kc, int *dc, int *pp)
{
    size_t o = (size_t)y * BP_NK + x / 32;
    unsigned sh = 31 - x % 32;
    *l = bp3_bit(s->lg, o, sh) ? -1 : bp3_bit(s->lw, o, sh) ? 30 : 0;      /* (-1: a grey, its value is the engine's) */
    *b = *op = *kc = *dc = 0;
    for (int i = 0; i < 2; i++) *b |= bp3_bit(s->b[i], o, sh) << i;
    for (int i = 0; i < 4; i++) *op |= bp3_bit(s->op[i], o, sh) << i;
    for (int i = 0; i < 3; i++) *kc |= bp3_bit(s->kc[i], o, sh) << i;
    for (int i = 0; i < 8; i++) *dc |= bp3_bit(s->dc[i], o, sh) << i;
    *dc = (int)(int8_t)*dc;
    *st = bp3_bit(s->st, o, sh);
    *pp = bp3_bit(s->p, o, sh);
}

/* ---- bit-sliced arithmetic: BP3_NB planes, two's complement, 32 lanes ---- */
typedef struct { uint32_t v[BP3_NB]; } bp3_v;
static inline bp3_v bp3_c(int c) { bp3_v r; for (int i = 0; i < BP3_NB; i++) r.v[i] = (c >> i) & 1 ? ~0u : 0; return r; }
static inline bp3_v bp3_add(bp3_v a, bp3_v b)
{
    bp3_v r; uint32_t c = 0;
    for (int i = 0; i < BP3_NB; i++) { uint32_t x = a.v[i] ^ b.v[i]; r.v[i] = x ^ c; c = (a.v[i] & b.v[i]) | (x & c); }
    return r;
}
static inline bp3_v bp3_sub(bp3_v a, bp3_v b)
{
    bp3_v r; uint32_t c = ~0u;                  /* a + ~b + 1 */
    for (int i = 0; i < BP3_NB; i++) { uint32_t nb = ~b.v[i], x = a.v[i] ^ nb; r.v[i] = x ^ c; c = (a.v[i] & nb) | (x & c); }
    return r;
}
static inline bp3_v bp3_sel(uint32_t m, bp3_v a, bp3_v b) { bp3_v r; for (int i = 0; i < BP3_NB; i++) r.v[i] = (a.v[i] & m) | (b.v[i] & ~m); return r; }
static inline uint32_t bp3_neg(bp3_v a) { return a.v[BP3_NB - 1]; }                 /* the lanes < 0 */
static inline uint32_t bp3_nz(bp3_v a) { uint32_t r = 0; for (int i = 0; i < BP3_NB; i++) r |= a.v[i]; return r; }
static inline uint32_t bp3_lt(bp3_v a, bp3_v b) { return bp3_neg(bp3_sub(a, b)); }  /* (no overflow at these widths) */
static inline bp3_v bp3_from(uint32_t *const *pl, int n, size_t o, int sgn)
{
    bp3_v r;
    for (int i = 0; i < BP3_NB; i++) r.v[i] = i < n ? pl[i][o] : sgn ? pl[n - 1][o] : 0;
    return r;
}

/* the gate's negative controls (STAG_BP3_BREAK): 1 no K idle, 2 b forced 0
 * (no payback), 3 the hammer off, 4 the record cut to 4 planes */
static int bp3_break;

/* frame f's words of rows [y0, y1) (template | phases), the state stepped;
 * *heavy (if set): the words that took the cold path */
static void bp3_rows(bp3_state *s, const bp_masks *m, const epd_stag_par *p, uint32_t f, const uint32_t *tpl, uint32_t *words, int y0, int y1, long *heavy)
{
    int fm = (int)(f & 1), fi = fm ^ 1;
    const bp3_v Rc = bp3_c(p->R), pc = bp3_c(p->pcap), hc = bp3_c(p->htop < p->pcap ? p->htop : p->pcap), one = bp3_c(1), two = bp3_c(2),
                kh = bp3_c(p->K / 2), r127 = bp3_c(127), m127 = bp3_c(-127), zero = bp3_c(0);
    const uint32_t R = (uint32_t)p->R;
    for (int y = y0; y < y1; y++) {
        const uint32_t *m0 = m->gm + ((size_t)fm * 64 + (y & 63)) * BP_NK, *mi = m->gm + ((size_t)fi * 64 + (y & 63)) * BP_NK;
        size_t row = (size_t)y * BP_NK, wo = (size_t)(EPD_DATA0 / EPD_LINE + EPD_H - 1 - y) * EPD_LINE + EPD_DATA0 % EPD_LINE;
        for (int k = 0; k < BP_NK; k++) {
            size_t o = row + k;
            uint32_t M0 = m0[k], MI = mi[k], lw = s->lw[o], lg = s->lg[o], P = s->p[o], tb = s->tb[o];
            uint32_t arrive = M0 & (lg | (lw ^ tb));
            uint32_t bnz = s->b[0][o] | s->b[1][o];
            /* op at the level's rail: OP == R (white) or OP == 0 (black) */
            uint32_t eqR = ~0u, eq0 = ~0u;
            for (int i = 0; i < 4; i++) { uint32_t q = s->op[i][o]; eqR &= (R >> i) & 1 ? q : ~q; eq0 &= ~q; }
            uint32_t oprail = (lw & eqR) | (~lw & eq0);
            uint32_t cold = M0 & (arrive | bnz | ~oprail);
            uint32_t drv;
            if (!cold) {                                /* light: the second pulses; a deciding lane ends its step */
                drv = MI & P;
                s->p[o] = P & ~M0;
            } else {
                if (heavy) (*heavy)++;
                bp3_v op = bp3_from(s->op, 4, o, 0), dc = bp3_from(s->dc, bp3_break == 4 ? 4 : 8, o, 1), B = bp3_from(s->b, 2, o, 0),
                      kc = bp3_from(s->kc, 3, o, 0);
                bp3_v need = bp3_sel(tb, bp3_sub(Rc, op), op);
                bp3_v owe = bp3_sel(tb, bp3_sub(zero, dc), dc);
                bp3_v x = bp3_sub(owe, need);
                uint32_t xpos = ~bp3_neg(x) & bp3_nz(x);
                bp3_v bx = bp3_sel(xpos, bp3_sel(bp3_lt(x, pc), x, pc), zero);
                uint32_t frail = arrive & ~lg;
                if (p->htop > 0 && bp3_break != 3) {    /* the hammer: a reversal (need < R) gets min(htop, pcap) */
                    uint32_t hm = frail & bp3_lt(need, Rc);
                    bx = bp3_sel(hm & bp3_lt(bx, hc), hc, bx);
                }
                bp3_v b = bp3_sel(frail, bx, bp3_sel(arrive, zero, B));        /* (an arriving grey: 0; no arrival: its own) */
                if (bp3_break == 2) b = zero;
                bp3_v want = bp3_add(need, b);
                uint32_t ge1 = bp3_nz(want), ge2 = ~bp3_lt(want, two);
                bp3_v kin = bp3_sel(P, kc, zero);
                uint32_t kfull = bp3_break == 1 ? 0 : ~bp3_lt(kin, kh);
                uint32_t nn2 = ge2 & ~kfull;
                bp3_v nn = bp3_sel(nn2, two, bp3_sel(ge1, one, zero));
                bp3_v kc2 = bp3_sel(nn2, bp3_add(kin, one), zero);
                bp3_v pay = bp3_sub(nn, need);
                pay = bp3_sel(bp3_neg(pay), zero, pay);
                bp3_v b2 = bp3_sub(b, pay);
                bp3_v up = bp3_add(op, nn), dn = bp3_sub(op, nn);
                bp3_v op2 = bp3_sel(tb, bp3_sel(bp3_lt(Rc, up), Rc, up), bp3_sel(bp3_neg(dn), zero, dn));
                bp3_v dsum = bp3_sel(tb, bp3_add(dc, nn), bp3_sub(dc, nn));
                bp3_v dc2 = bp3_sel(bp3_lt(r127, dsum), r127, bp3_sel(bp3_lt(dsum, m127), m127, dsum));
                uint32_t keep = ~cold;
#define BP3_BL(pl, v) ((pl)[o] = ((v) & cold) | ((pl)[o] & keep))
                for (int i = 0; i < 2; i++) BP3_BL(s->b[i], b2.v[i]);
                for (int i = 0; i < 4; i++) BP3_BL(s->op[i], op2.v[i]);
                for (int i = 0; i < 3; i++) BP3_BL(s->kc[i], kc2.v[i]);
                for (int i = 0; i < 8; i++) BP3_BL(s->dc[i], dc2.v[i]);
                BP3_BL(s->st, s->st[o] | ge1 | arrive);
                BP3_BL(s->lw, tb);
                BP3_BL(s->lg, 0u);
#undef BP3_BL
                s->p[o] = (P & ~M0) | (cold & nn2);
                drv = (cold & ge1) | (MI & P);
                lw = s->lw[o];
            }
            for (int q = 0; q < 4 && 4 * k + q < BP_GW; q++)
                words[wo + 4 * k + q] = tpl[wo + 4 * k + q] | bp_wire(drv & lw, drv & ~lw, q);
        }
    }
}

/* the reference's state (runs: hot, cold; the record already clamped to
 * +-127) against the planes: the differing pixels (*first: the first) */
static long bp3_cmp_ref(const bp3_state *s, const epd_stag *ref, size_t *first)
{
    long d = 0;
    *first = (size_t)-1;
    for (int y = 0; y < EPD_H; y++)
        for (int x = 0; x < EPD_W; x++) {
            size_t i = (size_t)y * EPD_W + x;
            unsigned h = ref->hot[i];
            uint32_t c = ref->cold[i];
            int rb, rl = epd_stag_ldec(EPD_STAG_LEVEL(c), &rb), rk = EPD_STAG_K(c);
            int l, b, op, st, kc, dc, pp;
            bp3_get(s, x, y, &l, &b, &op, &st, &kc, &dc, &pp);
            int rpp = (h & 3) && ((h >> 2) & 15) == 2;
            int bad = (rl == 0 || rl == 30 ? l != rl || b != rb : l != -1) || op != EPD_STAG_OP(c) || st != EPD_STAG_STALE(c) ||
                      dc != EPD_STAG_DC(c) || pp != rpp || (rk & 1) || kc != rk / 2 ||
                      ((h & 3) && (h & 3) != (rl == 30 ? 2u : 1u));
            if (bad) { if (!d) *first = i; d++; }
        }
    return d;
}

/* the planes from a reference state (runs: hot, cold; the record clamped) */
static inline void bp3_from_ref(bp3_state *s, const epd_stag *ref)
{
    for (int y = 0; y < EPD_H; y++)
        for (int x = 0; x < EPD_W; x++) {
            size_t i = (size_t)y * EPD_W + x;
            unsigned h = ref->hot[i];
            uint32_t c = ref->cold[i];
            int b, l = epd_stag_ldec(EPD_STAG_LEVEL(c), &b), dc = EPD_STAG_DC(c);
            dc = dc > 127 ? 127 : dc < -127 ? -127 : dc;
            bp3_put(s, x, y, l, b, EPD_STAG_OP(c), EPD_STAG_STALE(c), EPD_STAG_K(c) / 2, dc, (h & 3) && ((h >> 2) & 15) == 2);
        }
}

/* clamp the reference's records to +-127 (the planes' range): the pixels it changed */
static long bp3_clamp_ref(epd_stag *ref)
{
    long n = 0;
    for (size_t i = 0; i < EPD_NPX; i++) {
        int dc = EPD_STAG_DC(ref->cold[i]);
        if (dc > 127 || dc < -127) {
            ref->cold[i] = (ref->cold[i] & 0xFFFFu) | (uint32_t)(uint16_t)(dc > 0 ? 127 : -127) << 16;
            n++;
        }
    }
    return n;
}

#ifdef EPD_GPU
/* ---- the GPU pass. The state: one RGBA32UI texture of BP_NK x
 * 6 EPD_H, six layers of EPD_H rows (a texel = 4 planes of a plane word):
 * 0 H LW LG P CW (CW: cold work pending = b != 0 or op off its level's rail)
 * 1 B0 B1 OP0 OP1
 * 2 OP2 OP3 KC0 KC1
 * 3 KC2 ST DC0 DC1
 * 4 DC2 DC3 DC4 DC5
 * 5 DC6 DC7 0 0
 * A light word (no deciding lane arrives or has CW) reads layer 0 alone (and
 * writes it if P changes); a heavy word reads and writes all six. BP3COLD 1:
 * every layer fetched first, whatever the word (no fetch behind a branch). */
static const char *bp3_cs =
    "precision highp int; precision highp usampler2D; precision highp uimage2D; precision highp image2D;\n"
    "layout(local_size_x = WX, local_size_y = WY) in;\n"
    "uniform usampler2D SS, TBS, GMT, TP;\n"
    "uniform int g0, gi;\n"
    "uniform ivec4 par;\n"              /* (R, pcap, min(htop, pcap) or 0, K / 2) */
    "layout(rgba32ui, binding = 0) writeonly uniform uimage2D S;\n"
    "layout(rgba8, binding = 2) writeonly uniform highp image2D F;\n"
    "#define NB 10\n"
    "#define LH " BP_STR(EPD_H) "\n"
    "uint wire(uint wh, uint bk, uint sh) {\n"
    "  uint v = ((wh >> sh) & 255u) | ((bk >> sh) & 255u) << 16;\n"
    "  v = (v | v << 4) & 0x0F0F0F0Fu; v = (v | v << 2) & 0x33333333u; v = (v | v << 1) & 0x55555555u;\n"
    "  return (v & 0xFFFFu) << 1 | v >> 16;\n"
    "}\n"
    "void vc(out uint r[NB], int c) { for (int i = 0; i < NB; i++) r[i] = ((c >> i) & 1) != 0 ? 0xFFFFFFFFu : 0u; }\n"
    "void vadd(in uint a[NB], in uint b[NB], out uint r[NB]) { uint c = 0u; for (int i = 0; i < NB; i++) { uint x = a[i] ^ b[i]; r[i] = x ^ c; c = (a[i] & b[i]) | (x & c); } }\n"
    "void vsub(in uint a[NB], in uint b[NB], out uint r[NB]) { uint c = 0xFFFFFFFFu; for (int i = 0; i < NB; i++) { uint nb = ~b[i], x = a[i] ^ nb; r[i] = x ^ c; c = (a[i] & nb) | (x & c); } }\n"
    "void vsel(uint m, in uint a[NB], in uint b[NB], out uint r[NB]) { for (int i = 0; i < NB; i++) r[i] = (a[i] & m) | (b[i] & ~m); }\n"
    "uint vnz(in uint a[NB]) { uint r = 0u; for (int i = 0; i < NB; i++) r |= a[i]; return r; }\n"
    "uint vlt(in uint a[NB], in uint b[NB]) { uint d[NB]; vsub(a, b, d); return d[NB - 1]; }\n"
    "void main() {\n"
    "  ivec2 c = ivec2(gl_GlobalInvocationID.xy);\n"
    "  if (c.x >= " BP_STR(BP_NK) " || c.y >= LH) return;\n"
    "  int line = " BP_STR(EPD_DATA0) " / " BP_STR(EPD_LINE) " + LH - 1 - c.y, col = " BP_STR(EPD_DATA0) " % " BP_STR(EPD_LINE) " + 4 * c.x;\n"
    "  bool full = c.x < " BP_STR(BP_NK) " - 1;\n"
    "  uvec4 h = texelFetch(SS, c, 0);\n"
    "  uint tb = texelFetch(TBS, c, 0).r;\n"
    "  uint m0 = texelFetch(GMT, ivec2(c.x, g0 + (c.y & 63)), 0).r, mi = texelFetch(GMT, ivec2(c.x, gi + (c.y & 63)), 0).r;\n"
    "  uint t0 = texelFetch(TP, ivec2(col, line), 0).r, t1 = texelFetch(TP, ivec2(col + 1, line), 0).r;\n"
    "  uint t2 = texelFetch(TP, ivec2(col + 2, line), 0).r, t3 = texelFetch(TP, ivec2(col + 3, line), 0).r;\n"
    "  uint lw = h.x, lg = h.y, P = h.z, cw = h.w;\n"
    "  uint arrive = m0 & (lg | (lw ^ tb)), cold = m0 & (arrive | cw);\n"
    "  uvec4 c1, c2, c3, c4, c5;\n"
    "#if BP3COLD\n"
    "  c1 = texelFetch(SS, c + ivec2(0, LH), 0); c2 = texelFetch(SS, c + ivec2(0, 2 * LH), 0); c3 = texelFetch(SS, c + ivec2(0, 3 * LH), 0);\n"
    "  c4 = texelFetch(SS, c + ivec2(0, 4 * LH), 0); c5 = texelFetch(SS, c + ivec2(0, 5 * LH), 0);\n"
    "#endif\n"
    "  uint drv;\n"
    "  uvec4 h2;\n"
    "  if (cold == 0u) {\n"
    "    drv = mi & P;\n"
    "    h2 = uvec4(lw, lg, P & ~m0, cw);\n"
    "    if (h2 != h) imageStore(S, c, h2);\n"
    "  } else {\n"
    "#if !BP3COLD\n"
    "    c1 = texelFetch(SS, c + ivec2(0, LH), 0); c2 = texelFetch(SS, c + ivec2(0, 2 * LH), 0); c3 = texelFetch(SS, c + ivec2(0, 3 * LH), 0);\n"
    "    c4 = texelFetch(SS, c + ivec2(0, 4 * LH), 0); c5 = texelFetch(SS, c + ivec2(0, 5 * LH), 0);\n"
    "#endif\n"
    "    uint op[NB], dc[NB], B[NB], kc[NB], zero[NB], one[NB], two[NB], Rc[NB], pc[NB], hc[NB], kh[NB], r127[NB], m127[NB];\n"
    "    vc(zero, 0); vc(one, 1); vc(two, 2); vc(Rc, par.x); vc(pc, par.y); vc(hc, par.z); vc(kh, par.w); vc(r127, 127); vc(m127, -127);\n"
    "    for (int i = 0; i < NB; i++) { op[i] = 0u; B[i] = 0u; kc[i] = 0u; }\n"
    "    B[0] = c1.x; B[1] = c1.y; op[0] = c1.z; op[1] = c1.w; op[2] = c2.x; op[3] = c2.y; kc[0] = c2.z; kc[1] = c2.w; kc[2] = c3.x;\n"
    "    uint st = c3.y;\n"
    "    dc[0] = c3.z; dc[1] = c3.w; dc[2] = c4.x; dc[3] = c4.y; dc[4] = c4.z; dc[5] = c4.w; dc[6] = c5.x; dc[7] = c5.y; dc[8] = c5.y; dc[9] = c5.y;\n"
    "    uint ro[NB], need[NB], nd[NB], owe[NB], x[NB], t[NB], bx[NB], b[NB], want[NB], kin[NB], nn[NB], k2[NB], pay[NB], b2[NB], up[NB], dn[NB], o2[NB], ds[NB], d2[NB];\n"
    "    vsub(Rc, op, ro); vsel(tb, ro, op, need);\n"
    "    vsub(zero, dc, nd); vsel(tb, nd, dc, owe);\n"
    "    vsub(owe, need, x);\n"
    "    uint xpos = ~x[NB - 1] & vnz(x);\n"
    "    vsel(vlt(x, pc), x, pc, t); vsel(xpos, t, zero, bx);\n"
    "    uint frail = arrive & ~lg;\n"
    "    if (par.z > 0) { uint hm = frail & vlt(need, Rc) & vlt(bx, hc); vsel(hm, hc, bx, t); bx = t; }\n"
    "    vsel(arrive, zero, B, t); vsel(frail, bx, t, b);\n"
    "    vadd(need, b, want);\n"
    "    uint ge1 = vnz(want), ge2 = ~vlt(want, two);\n"
    "    vsel(P, kc, zero, kin);\n"
    "    uint nn2 = ge2 & vlt(kin, kh);\n"
    "    vsel(ge1, one, zero, t); vsel(nn2, two, t, nn);\n"
    "    vadd(kin, one, t); vsel(nn2, t, zero, k2);\n"
    "    vsub(nn, need, t); vsel(t[NB - 1], zero, t, pay);\n"
    "    vsub(b, pay, b2);\n"
    "    vadd(op, nn, up); vsub(op, nn, dn);\n"
    "    vsel(vlt(Rc, up), Rc, up, t); vsel(dn[NB - 1], zero, dn, x); vsel(tb, t, x, o2);\n"
    "    vadd(dc, nn, t); vsub(dc, nn, x); vsel(tb, t, x, ds);\n"
    "    vsel(vlt(ds, m127), m127, ds, t); vsel(vlt(r127, ds), r127, t, d2);\n"
    "    uint k = cold, n = ~cold;\n"
    "#define BL(o, v) (((v) & k) | ((o) & n))\n"
    "    uint lw2 = BL(lw, tb), lg2 = lg & n, st2 = BL(st, st | ge1 | arrive);\n"
    "    uint B0 = BL(c1.x, b2[0]), B1 = BL(c1.y, b2[1]);\n"
    "    uint O0 = BL(c1.z, o2[0]), O1 = BL(c1.w, o2[1]), O2 = BL(c2.x, o2[2]), O3 = BL(c2.y, o2[3]);\n"
    "    uint eqR = 0xFFFFFFFFu, eq0 = 0xFFFFFFFFu, oo[4] = uint[4](O0, O1, O2, O3);\n"
    "    for (int i = 0; i < 4; i++) { eqR &= ((par.x >> i) & 1) != 0 ? oo[i] : ~oo[i]; eq0 &= ~oo[i]; }\n"
    "    uint cw2 = B0 | B1 | ~((lw2 & eqR) | (~lw2 & eq0));\n"
    "    h2 = uvec4(lw2, lg2, (P & ~m0) | (k & nn2), cw2);\n"
    "    drv = (k & ge1) | (mi & P);\n"
    "    lw = lw2;\n"
    "    uvec4 n1 = uvec4(B0, B1, O0, O1), n2 = uvec4(O2, O3, BL(c2.z, k2[0]), BL(c2.w, k2[1]));\n"
    "    uvec4 n3 = uvec4(BL(c3.x, k2[2]), st2, BL(c3.z, d2[0]), BL(c3.w, d2[1]));\n"
    "    uvec4 n4 = uvec4(BL(c4.x, d2[2]), BL(c4.y, d2[3]), BL(c4.z, d2[4]), BL(c4.w, d2[5])), n5 = uvec4(BL(c5.x, d2[6]), BL(c5.y, d2[7]), 0u, 0u);\n"
    "    if (h2 != h) imageStore(S, c, h2);\n"
    "    if (n1 != c1) imageStore(S, c + ivec2(0, LH), n1);\n"
    "    if (n2 != c2) imageStore(S, c + ivec2(0, 2 * LH), n2);\n"
    "    if (n3 != c3) imageStore(S, c + ivec2(0, 3 * LH), n3);\n"
    "    if (n4 != c4) imageStore(S, c + ivec2(0, 4 * LH), n4);\n"
    "    if (n5 != c5) imageStore(S, c + ivec2(0, 5 * LH), n5);\n"
    "  }\n"
    "  uint wh = drv & lw, bk = drv & ~lw;\n"
    "  imageStore(F, ivec2(col, line), unpackUnorm4x8(t0 | wire(wh, bk, 24u)));\n"
    "  imageStore(F, ivec2(col + 1, line), unpackUnorm4x8(t1 | wire(wh, bk, 16u)));\n"
    "  if (full) {\n"
    "    imageStore(F, ivec2(col + 2, line), unpackUnorm4x8(t2 | wire(wh, bk, 8u)));\n"
    "    imageStore(F, ivec2(col + 3, line), unpackUnorm4x8(t3 | wire(wh, bk, 0u)));\n"
    "  }\n"
    "}\n";

typedef struct {
    bp_gpu b;                                   /* (its TB, T8, GMT, TP, F, pack; its S/U/pdrv/pload unused) */
    GLuint pdrv, pload, S, U, fbS;
    GLint u_g0, u_gi, u_par;
    int par[4];
} bp3_gpu;

/* the load: S from U (6 EPD_H rows) */
static const char *bp3_load_cs =
    "precision highp int; precision highp usampler2D; precision highp uimage2D;\n"
    "layout(local_size_x = 32, local_size_y = 4) in;\n"
    "uniform usampler2D U;\n"
    "layout(rgba32ui, binding = 0) writeonly uniform uimage2D S;\n"
    "void main() {\n"
    "  ivec2 c = ivec2(gl_GlobalInvocationID.xy);\n"
    "  if (c.x >= " BP_STR(BP_NK) " || c.y >= 6 * " BP_STR(EPD_H) ") return;\n"
    "  imageStore(S, c, texelFetch(U, c, 0));\n"
    "}\n";

static int bp3_gpu_open(bp3_gpu *g, int wx, int wy, const bp_masks *m, const epd_stag_par *p, const uint32_t *tpl, int coldall)
{
    memset(g, 0, sizeof *g);
    if (bp_gpu_open(&g->b, wx, wy, m, tpl)) return -1;
    char defs[160];
    snprintf(defs, sizeof defs, "#define WX %d\n#define WY %d\n#define BP3COLD %d\n", wx, wy, coldall);
    if (!(g->pdrv = bp_prog(bp3_cs, defs, "ctl3 drive")) || !(g->pload = bp_prog(bp3_load_cs, "", "ctl3 load"))) return -1;
    bpgl.glUseProgram(g->pdrv);
    bpgl.glUniform1i(bpgl.glGetUniformLocation(g->pdrv, "SS"), BPU_SS);
    bpgl.glUniform1i(bpgl.glGetUniformLocation(g->pdrv, "TBS"), BPU_TB);
    bpgl.glUniform1i(bpgl.glGetUniformLocation(g->pdrv, "GMT"), BPU_GM);
    bpgl.glUniform1i(bpgl.glGetUniformLocation(g->pdrv, "TP"), BPU_TP);
    g->u_g0 = bpgl.glGetUniformLocation(g->pdrv, "g0");
    g->u_gi = bpgl.glGetUniformLocation(g->pdrv, "gi");
    g->u_par = bpgl.glGetUniformLocation(g->pdrv, "par");
    int hc = p->htop > 0 ? (p->htop < p->pcap ? p->htop : p->pcap) : 0;
    if (bp3_break == 3) hc = 0;
    g->par[0] = p->R; g->par[1] = bp3_break == 2 ? 0 : p->pcap; g->par[2] = hc; g->par[3] = bp3_break == 1 ? 15 : p->K / 2;
    bpgl.glUseProgram(g->pload);
    bpgl.glUniform1i(bpgl.glGetUniformLocation(g->pload, "U"), BPU_U);
    g->S = bp_tex(GL_RGBA32UI, BP_NK, 6 * EPD_H);
    g->U = bp_tex(GL_RGBA32UI, BP_NK, 6 * EPD_H);
    if (!(g->fbS = bp_fbo(g->S))) { fprintf(stderr, "bitplane ctl3: no readback framebuffer\n"); return -1; }
    GLenum e = bpgl.glGetError();
    if (e != GL_NO_ERROR) { fprintf(stderr, "bitplane ctl3: GL error 0x%x in setup\n", e); return -1; }
    return 0;
}
static void bp3_gpu_close(bp3_gpu *g)
{
    bpgl.glDeleteTextures(1, &g->S); bpgl.glDeleteTextures(1, &g->U);
    bpgl.glDeleteFramebuffers(1, &g->fbS);
    bpgl.glDeleteProgram(g->pdrv); bpgl.glDeleteProgram(g->pload);
    bp_gpu_close(&g->b);
    memset(g, 0, sizeof *g);
}
/* the per-lane CW plane of word o */
static inline uint32_t bp3_cw(const bp3_state *s, size_t o, int R)
{
    uint32_t eqR = ~0u, eq0 = ~0u;
    for (int i = 0; i < 4; i++) { uint32_t q = s->op[i][o]; eqR &= (R >> i) & 1 ? q : ~q; eq0 &= ~q; }
    return s->b[0][o] | s->b[1][o] | ~((s->lw[o] & eqR) | (~s->lw[o] & eq0));
}
static void bp3_texels(const bp3_state *s, int R, uint32_t *u)
{
    for (size_t i = 0; i < BP_NW; i++) {
        uint32_t *t = u + 4 * i;
        size_t L = (size_t)4 * BP_NW;
        t[0] = s->lw[i]; t[1] = s->lg[i]; t[2] = s->p[i]; t[3] = bp3_cw(s, i, R);
        t[L + 0] = s->b[0][i]; t[L + 1] = s->b[1][i]; t[L + 2] = s->op[0][i]; t[L + 3] = s->op[1][i];
        t[2 * L + 0] = s->op[2][i]; t[2 * L + 1] = s->op[3][i]; t[2 * L + 2] = s->kc[0][i]; t[2 * L + 3] = s->kc[1][i];
        t[3 * L + 0] = s->kc[2][i]; t[3 * L + 1] = s->st[i]; t[3 * L + 2] = s->dc[0][i]; t[3 * L + 3] = s->dc[1][i];
        t[4 * L + 0] = s->dc[2][i]; t[4 * L + 1] = s->dc[3][i]; t[4 * L + 2] = s->dc[4][i]; t[4 * L + 3] = s->dc[5][i];
        t[5 * L + 0] = s->dc[6][i]; t[5 * L + 1] = s->dc[7][i]; t[5 * L + 2] = 0; t[5 * L + 3] = 0;
    }
}
static void bp3_gpu_load(bp3_gpu *g, const bp3_state *s, int R)
{
    uint32_t *u = malloc((size_t)BP_NW * 16 * 6);
    if (!u) return;
    bp3_texels(s, R, u);
    bp_bind(BPU_U, g->U);
    bpgl.glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, BP_NK, 6 * EPD_H, GL_RGBA_INTEGER, GL_UNSIGNED_INT, u);
    free(u);
    bpgl.glUseProgram(g->pload);
    bpgl.glBindImageTexture(0, g->S, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32UI);
    bpgl.glDispatchCompute((BP_NK + 31) / 32, (6 * EPD_H + 3) / 4, 1);
    bpgl.glMemoryBarrier(GL_ALL_BARRIER_BITS);
}
/* frame f: the pending pack, then the ctl 3 drive pass (timed: *pms, *dms; -1 no timer); words in F */
static void bp3_gpu_frame(bp3_gpu *g, uint32_t f, double *pms, double *dms)
{
    bp_gpu *b = &g->b;
    int tm = bpgl.genq != NULL, pk = b->npk;
    *pms = 0; *dms = -1;
    if (pk) {
        bpgl.glUseProgram(b->ppack);
        bp_bind(BPU_T8, b->T8);
        bpgl.glBindImageTexture(1, b->TB, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R32UI);
        bpgl.glUniform4i(b->u_box, b->pk[0], b->pk[1], b->pk[2], b->pk[3]);
        if (tm) bpgl.beginq(0x88BF, b->tq[0]);
        bpgl.glDispatchCompute((GLuint)(b->pk[1] - b->pk[0] + 31) / 32, (GLuint)(b->pk[3] - b->pk[2] + 3) / 4, 1);
        if (tm) bpgl.endq(0x88BF);
        bpgl.glMemoryBarrier(GL_ALL_BARRIER_BITS);
        b->npk = 0;
    }
    int fm = (int)(f & 1);
    bpgl.glUseProgram(g->pdrv);
    bpgl.glUniform1i(g->u_g0, fm * 64);
    bpgl.glUniform1i(g->u_gi, (fm ^ 1) * 64);
    bpgl.glUniform4i(g->u_par, g->par[0], g->par[1], g->par[2], g->par[3]);
    bp_bind(BPU_SS, g->S);
    bp_bind(BPU_TB, b->TB);
    bp_bind(BPU_GM, b->GMT);
    bp_bind(BPU_TP, b->TP);
    bpgl.glBindImageTexture(0, g->S, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32UI);
    bpgl.glBindImageTexture(2, b->F, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA8);
    if (tm) bpgl.beginq(0x88BF, b->tq[1]);
    bpgl.glDispatchCompute((GLuint)(BP_NK + b->wx - 1) / (GLuint)b->wx, (GLuint)(EPD_H + b->wy - 1) / (GLuint)b->wy, 1);
    if (tm) bpgl.endq(0x88BF);
    bpgl.glMemoryBarrier(GL_ALL_BARRIER_BITS);
    if (tm) {
        uint64_t ns = 0;
        if (pk) { bpgl.getq(b->tq[0], 0x8866, &ns); *pms = ns / 1e6; }
        ns = 0;
        bpgl.getq(b->tq[1], 0x8866, &ns);
        *dms = ns / 1e6;
    }
}
/* S against s (the CPU twin, its CW derived): the differing plane words */
static long bp3_gpu_cmp(bp3_gpu *g, const bp3_state *s, int R)
{
    size_t n = (size_t)BP_NW * 4 * 6;
    uint32_t *u = malloc(n * 4), *v = malloc(n * 4);
    if (!u || !v) { free(u); free(v); return -1; }
    bpgl.glFinish();
    bpgl.glBindFramebuffer(GL_FRAMEBUFFER, g->fbS);
    bpgl.glReadPixels(0, 0, BP_NK, 6 * EPD_H, GL_RGBA_INTEGER, GL_UNSIGNED_INT, u);
    bpgl.glBindFramebuffer(GL_FRAMEBUFFER, 0);
    bp3_texels(s, R, v);
    long d = 0;
    for (size_t i = 0; i < n; i++) d += u[i] != v[i];
    free(u); free(v);
    return d;
}
#endif

#endif
