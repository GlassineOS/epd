/* epd-bp3n.h — bp3_rows four plane words at a time (gcc vector extensions:
 * NEON on arm64), for the CPU timing of Same semantics as
 * bp3_rows, bit for bit: a group of four words takes the cold path when any
 * of them has cold work, and the cold mask keeps the others' state (the
 * light path is the cold path with cold == 0). Works on the planes as one
 * flat array of BP_NW words (BP_NW % 4 == 0); a group may span two rows, so
 * the masks are gathered and the wire words stored per word. */
#ifndef EPD_BP3N_H
#define EPD_BP3N_H

typedef uint32_t bp3n_u __attribute__((vector_size(16)));
typedef struct { bp3n_u v[BP3_NB]; } bp3n_v;

static inline bp3n_u bp3n_ld(const uint32_t *p) { bp3n_u r; memcpy(&r, p, sizeof r); return r; }
static inline void bp3n_st(uint32_t *p, bp3n_u v) { memcpy(p, &v, sizeof v); }
static inline int bp3n_any(bp3n_u x) { return (x[0] | x[1] | x[2] | x[3]) != 0; }
static inline bp3n_v bp3n_c(int c) { bp3n_v r; for (int i = 0; i < BP3_NB; i++) r.v[i] = (bp3n_u){ 0, 0, 0, 0 } - (bp3n_u){ (c >> i) & 1, (c >> i) & 1, (c >> i) & 1, (c >> i) & 1 }; return r; }
static inline bp3n_v bp3n_add(bp3n_v a, bp3n_v b)
{
    bp3n_v r; bp3n_u c = { 0, 0, 0, 0 };
    for (int i = 0; i < BP3_NB; i++) { bp3n_u x = a.v[i] ^ b.v[i]; r.v[i] = x ^ c; c = (a.v[i] & b.v[i]) | (x & c); }
    return r;
}
static inline bp3n_v bp3n_sub(bp3n_v a, bp3n_v b)
{
    bp3n_v r; bp3n_u c = ~(bp3n_u){ 0, 0, 0, 0 };
    for (int i = 0; i < BP3_NB; i++) { bp3n_u nb = ~b.v[i], x = a.v[i] ^ nb; r.v[i] = x ^ c; c = (a.v[i] & nb) | (x & c); }
    return r;
}
static inline bp3n_v bp3n_sel(bp3n_u m, bp3n_v a, bp3n_v b) { bp3n_v r; for (int i = 0; i < BP3_NB; i++) r.v[i] = (a.v[i] & m) | (b.v[i] & ~m); return r; }
static inline bp3n_u bp3n_neg(bp3n_v a) { return a.v[BP3_NB - 1]; }
static inline bp3n_u bp3n_nz(bp3n_v a) { bp3n_u r = { 0, 0, 0, 0 }; for (int i = 0; i < BP3_NB; i++) r |= a.v[i]; return r; }
static inline bp3n_u bp3n_lt(bp3n_v a, bp3n_v b) { return bp3n_neg(bp3n_sub(a, b)); }
static inline bp3n_v bp3n_from(uint32_t *const *pl, int n, size_t o, int sgn)
{
    bp3n_v r;
    for (int i = 0; i < BP3_NB; i++) r.v[i] = i < n ? bp3n_ld(pl[i] + o) : sgn ? r.v[n - 1] : (bp3n_u){ 0, 0, 0, 0 };
    return r;
}

/* frame f's words of plane words [o0, o1) (multiples of 4), the state stepped */
static void bp3n_rows(bp3_state *s, const bp_masks *m, const epd_stag_par *p, uint32_t f, const uint32_t *tpl, uint32_t *words, size_t o0, size_t o1, long *heavy)
{
    int fm = (int)(f & 1), fi = fm ^ 1;
    const bp3n_v Rc = bp3n_c(p->R), pc = bp3n_c(p->pcap), hc = bp3n_c(p->htop < p->pcap ? p->htop : p->pcap), one = bp3n_c(1), two = bp3n_c(2),
                 kh = bp3n_c(p->K / 2), r127 = bp3n_c(127), m127 = bp3n_c(-127), zero = bp3n_c(0);
    const uint32_t R = (uint32_t)p->R;
    const bp3n_u Z = { 0, 0, 0, 0 };
    for (size_t o = o0; o < o1; o += 4) {
        bp3n_u M0, MI;
        for (int j = 0; j < 4; j++) {
            size_t y = (o + j) / BP_NK, k = (o + j) % BP_NK;
            M0[j] = m->gm[((size_t)fm * 64 + (y & 63)) * BP_NK + k];
            MI[j] = m->gm[((size_t)fi * 64 + (y & 63)) * BP_NK + k];
        }
        bp3n_u lw = bp3n_ld(s->lw + o), lg = bp3n_ld(s->lg + o), P = bp3n_ld(s->p + o), tb = bp3n_ld(s->tb + o);
        bp3n_u arrive = M0 & (lg | (lw ^ tb));
        bp3n_u bnz = bp3n_ld(s->b[0] + o) | bp3n_ld(s->b[1] + o);
        bp3n_u eqR = ~Z, eq0 = ~Z;
        for (int i = 0; i < 4; i++) { bp3n_u q = bp3n_ld(s->op[i] + o); eqR &= (R >> i) & 1 ? q : ~q; eq0 &= ~q; }
        bp3n_u oprail = (lw & eqR) | (~lw & eq0);
        bp3n_u cold = M0 & (arrive | bnz | ~oprail);
        bp3n_u drv;
        if (!bp3n_any(cold)) {
            drv = MI & P;
            bp3n_st(s->p + o, P & ~M0);
        } else {
            if (heavy) for (int j = 0; j < 4; j++) *heavy += cold[j] != 0;
            bp3n_v op = bp3n_from(s->op, 4, o, 0), dc = bp3n_from(s->dc, 8, o, 1), B = bp3n_from(s->b, 2, o, 0), kc = bp3n_from(s->kc, 3, o, 0);
            bp3n_v need = bp3n_sel(tb, bp3n_sub(Rc, op), op);
            bp3n_v owe = bp3n_sel(tb, bp3n_sub(zero, dc), dc);
            bp3n_v x = bp3n_sub(owe, need);
            bp3n_u xpos = ~bp3n_neg(x) & bp3n_nz(x);
            bp3n_v bx = bp3n_sel(xpos, bp3n_sel(bp3n_lt(x, pc), x, pc), zero);
            bp3n_u frail = arrive & ~lg;
            if (p->htop > 0) {
                bp3n_u hm = frail & bp3n_lt(need, Rc);
                bx = bp3n_sel(hm & bp3n_lt(bx, hc), hc, bx);
            }
            bp3n_v b = bp3n_sel(frail, bx, bp3n_sel(arrive, zero, B));
            bp3n_v want = bp3n_add(need, b);
            bp3n_u ge1 = bp3n_nz(want), ge2 = ~bp3n_lt(want, two);
            bp3n_v kin = bp3n_sel(P, kc, zero);
            bp3n_u kfull = ~bp3n_lt(kin, kh);
            bp3n_u nn2 = ge2 & ~kfull;
            bp3n_v nn = bp3n_sel(nn2, two, bp3n_sel(ge1, one, zero));
            bp3n_v kc2 = bp3n_sel(nn2, bp3n_add(kin, one), zero);
            bp3n_v pay = bp3n_sub(nn, need);
            pay = bp3n_sel(bp3n_neg(pay), zero, pay);
            bp3n_v b2 = bp3n_sub(b, pay);
            bp3n_v up = bp3n_add(op, nn), dn = bp3n_sub(op, nn);
            bp3n_v op2 = bp3n_sel(tb, bp3n_sel(bp3n_lt(Rc, up), Rc, up), bp3n_sel(bp3n_neg(dn), zero, dn));
            bp3n_v dsum = bp3n_sel(tb, bp3n_add(dc, nn), bp3n_sub(dc, nn));
            bp3n_v dc2 = bp3n_sel(bp3n_lt(r127, dsum), r127, bp3n_sel(bp3n_lt(dsum, m127), m127, dsum));
            bp3n_u keep = ~cold;
#define BP3N_BL(pl, v) bp3n_st((pl) + o, ((v) & cold) | (bp3n_ld((pl) + o) & keep))
            for (int i = 0; i < 2; i++) BP3N_BL(s->b[i], b2.v[i]);
            for (int i = 0; i < 4; i++) BP3N_BL(s->op[i], op2.v[i]);
            for (int i = 0; i < 3; i++) BP3N_BL(s->kc[i], kc2.v[i]);
            for (int i = 0; i < 8; i++) BP3N_BL(s->dc[i], dc2.v[i]);
            BP3N_BL(s->st, bp3n_ld(s->st + o) | ge1 | arrive);
            BP3N_BL(s->lw, tb);
            BP3N_BL(s->lg, Z);
#undef BP3N_BL
            bp3n_st(s->p + o, (P & ~M0) | (cold & nn2));
            drv = (cold & ge1) | (MI & P);
            lw = bp3n_ld(s->lw + o);
        }
        for (int j = 0; j < 4; j++) {
            size_t y = (o + j) / BP_NK, k = (o + j) % BP_NK;
            size_t wo = (size_t)(EPD_DATA0 / EPD_LINE + EPD_H - 1 - y) * EPD_LINE + EPD_DATA0 % EPD_LINE;
            for (int q = 0; q < 4 && 4 * k + q < BP_GW; q++)
                words[wo + 4 * k + q] = tpl[wo + 4 * k + q] | bp_wire(drv[j] & lw[j], drv[j] & ~lw[j], q);
        }
    }
}
#endif
