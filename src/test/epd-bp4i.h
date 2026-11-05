/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-bp4i.h — epd-bp4.h's two-tier ctl 3 with the planes interleaved: the
 * same per-frame and visit logic, the state of a plane word together (its
 * cache lines) instead of 25 arrays. Test code, included by
 * epd-stagger-test.c after epd-bp4.h. Three groups, each a base + a stride
 * (in words), so one body serves every layout:
 * hot: lw p pd rv h8 op[4] ow[2] ac[5] (16 words: one 64 B line);
 * tb (written by bp_targets as a plane); dc[8] (the visit only).
 * Layouts (STAG_BP4I_MEM): a25 = all 25 in one struct (100 B), a32 = the
 * same padded to 128 B, h16 = hot structs + a tb plane + dc structs (32 B).
 * STAG_BP4I_GATE=1: bp4 and bp4i in lockstep (the frames of bp4_gate),
 * every frame's words and the full state every 8th frame equal; the time a
 * frame, bp4i over 1/2/4/8 threads (STAG_BP4I_NT, STAG_BP4I_CPUS pins).
 * STAG_BP4I_BREAK=1|2: flip one acc | lw bit at frame 3 (must differ). */
#ifndef EPD_BP4I_H
#define EPD_BP4I_H

enum { BI_LW, BI_P, BI_PD, BI_RV, BI_H8, BI_OP, BI_OW = BI_OP + 4, BI_AC = BI_OW + 2, BI_NH = BI_AC + 5 };   /* (the hot words) */
enum { BI_A25, BI_A32, BI_H16, BI_NK };
static const char *const bi_kname[BI_NK] = { "a25", "a32", "h16" };
typedef struct {
    uint32_t *h, *tb, *dc;                      /* (word o: h[o * hs + BI_*], tb[o * ts], dc[o * ds + i]) */
    void *mem[3];
    int kind;
} bp4i_state;
static const int bi_hs[BI_NK] = { 25, 32, 16 }, bi_ts[BI_NK] = { 25, 32, 1 }, bi_ds[BI_NK] = { 25, 32, 8 };
static inline int bp4i_alloc(bp4i_state *s, int kind)
{
    memset(s, 0, sizeof *s);
    s->kind = kind;
    if (kind != BI_H16) {
        size_t n = (size_t)BP_NW * bi_hs[kind] * 4;
        if (!(s->mem[0] = aligned_alloc(64, n))) return -1;
        memset(s->mem[0], 0, n);                /* (aligned_alloc does not zero) */
        s->h = s->mem[0]; s->tb = s->h + BI_NH; s->dc = s->h + BI_NH + 1;
        return 0;
    }
    size_t nh = (size_t)BP_NW * 16 * 4, nd = (size_t)BP_NW * 8 * 4;
    if (!(s->mem[0] = aligned_alloc(64, nh)) || !(s->mem[1] = calloc(BP_NW, 4)) || !(s->mem[2] = aligned_alloc(64, nd))) return -1;
    memset(s->mem[0], 0, nh); memset(s->mem[2], 0, nd);
    s->h = s->mem[0]; s->tb = s->mem[1]; s->dc = s->mem[2];
    return 0;
}
static inline void bp4i_free(bp4i_state *s) { for (int i = 0; i < 3; i++) free(s->mem[i]); memset(s, 0, sizeof *s); }
/* bp4's planes in bp4_planes order (lw p tb pd rv h8 op ow ac dc) -> the word's slot */
static inline uint32_t *bp4i_slot(const bp4i_state *s, int pl, size_t o)
{
    const int hs = bi_hs[s->kind], ts = bi_ts[s->kind], ds = bi_ds[s->kind];
    static const int hi[BP4_NPL] = { BI_LW, BI_P, -1, BI_PD, BI_RV, BI_H8, BI_OP, BI_OP + 1, BI_OP + 2, BI_OP + 3, BI_OW, BI_OW + 1,
                                     BI_AC, BI_AC + 1, BI_AC + 2, BI_AC + 3, BI_AC + 4 };
    if (pl == 2) return s->tb + o * ts;
    if (pl >= 17) return s->dc + o * ds + (pl - 17);
    return s->h + o * hs + hi[pl];
}
static inline void bp4i_from(bp4i_state *s, const bp4_state *b)
{
    uint32_t *pl[BP4_NPL];
    bp4_planes(b, pl);
    for (int i = 0; i < BP4_NPL; i++) for (size_t o = 0; o < BP_NW; o++) *bp4i_slot(s, i, o) = pl[i][o];
}
static inline void bp4i_tb_from(bp4i_state *s, const uint32_t *tb)     /* (after bp_targets on a plane) */
{
    const int ts = bi_ts[s->kind];
    for (size_t o = 0; o < BP_NW; o++) s->tb[o * ts] = tb[o];
}
static inline long bp4i_cmp(const bp4i_state *s, const bp4_state *b)   /* (the words that differ, all planes) */
{
    uint32_t *pl[BP4_NPL];
    long d = 0;
    bp4_planes(b, pl);
    for (int i = 0; i < BP4_NPL; i++) for (size_t o = 0; o < BP_NW; o++) d += *bp4i_slot(s, i, o) != pl[i][o];
    return d;
}

/* bp4_visit_word, the word's groups at h, t, dc */
static inline __attribute__((always_inline)) void bp4i_visit_word(uint32_t *h, uint32_t *dcp, const epd_stag_par *p)
{
    const int hc = p->htop < p->pcap ? p->htop : p->pcap;
    uint32_t a[5], anz = 0;
    for (int i = 0; i < 5; i++) anz |= a[i] = h[BI_AC + i];
    uint32_t pd = h[BI_PD];
    if (!anz && !pd) return;                    /* (nothing to fold, nothing waits) */
    uint32_t d[8];
    for (int i = 0; i < 8; i++) d[i] = dcp[i];
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
        for (int i = 0; i < 8; i++) dcp[i] = d[i];
        for (int i = 0; i < 5; i++) h[BI_AC + i] = 0;
    }
    uint32_t go = pd & ~(h[BI_OP] | h[BI_OP + 1] | h[BI_OP + 2] | h[BI_OP + 3]);   /* (at its rail: q == 0) */
    if (!go) return;
    uint32_t lw = h[BI_LW];
    uint32_t ge3 = ~d[7] & (d[6] | d[5] | d[4] | d[3] | d[2] | (d[1] & d[0]));
    uint32_t le3 = d[7] & ~(d[6] & d[5] & d[4] & d[3] & d[2] & d[1]);
    uint32_t big = (lw & le3) | (~lw & ge3), small = (lw & ge3) | (~lw & le3);
    uint32_t n0 = d[0], n1 = d[1] ^ d[0], n2 = d[2] ^ (d[1] | d[0]);    /* (-d, 3 bits: ~d + 1) */
    uint32_t x0 = (lw & n0) | (~lw & d[0]), x1 = (lw & n1) | (~lw & d[1]), x2 = (lw & n2) | (~lw & d[2]);
    x0 = (x0 & ~big & ~small) | big | small;    /* 3 = 011, -3 = 101 */
    x1 = (x1 & ~big & ~small) | big;
    x2 = (x2 & ~big & ~small) | small;
    uint32_t h8 = h[BI_H8], w0 = h[BI_OW], w1 = h[BI_OW + 1];
    bp4_v t = { { x0, x1, x2, x2 } }, paid, z = bp4_c(0, 4), pc = bp4_c(p->pcap, 4), hcv = bp4_c(hc, 4);
    {   bp4_v ow = { { w0, w1, 0, 0 } };
        paid = bp4_sel(h8, bp4_sub(hcv, ow, 4), z, 4); }
    t = bp4_add(t, paid, 4);                    /* (-3..6: fits 4 bits signed) */
    bp4_v bv = bp4_sel(bp4_neg(t, 4) | ~bp4_nz(t, 4), z, bp4_sel(bp4_lt(t, pc, 4), t, pc, 4), 4);
    if (p->htop > 0) bv = bp4_sel(h[BI_RV] & bp4_lt(bv, hcv, 4), hcv, bv, 4);
    bv = bp4_sub(bv, paid, 4);
    bv = bp4_sel(bp4_neg(bv, 4), z, bv, 4);
    h[BI_OW] = (bv.v[0] & go) | (w0 & ~go);
    h[BI_OW + 1] = (bv.v[1] & go) | (w1 & ~go);
    h[BI_PD] &= ~go; h[BI_RV] &= ~go; h[BI_H8] &= ~go;
}

/* bp4_rows over rows [y0, y1); tv (when set): the visits' time added */
static inline __attribute__((always_inline)) void bp4i_rows_g(bp4i_state *s, const bp_masks *m, const epd_stag_par *p, int M, uint32_t f, const uint32_t *tpl, uint32_t *words, int y0, int y1, double *tv,
                                                              const int HS, const int TS, const int DS)
{
    int fm = (int)(f & 1), fi = fm ^ 1;
    uint32_t c = f / 2;
    const int R = p->R, hc = p->htop < p->pcap ? p->htop : p->pcap;
    for (int y = y0; y < y1; y++) {
        const uint32_t *m0 = m->gm + ((size_t)fm * 64 + (y & 63)) * BP_NK, *mi = m->gm + ((size_t)fi * 64 + (y & 63)) * BP_NK;
        size_t row = (size_t)y * BP_NK, wo = (size_t)(EPD_DATA0 / EPD_LINE + EPD_H - 1 - y) * EPD_LINE + EPD_DATA0 % EPD_LINE;
        if (M > 0 && f % 2 == 0 && (uint32_t)y % (uint32_t)M == c % (uint32_t)M) {   /* (the row's visit first: words are independent) */
            double t0 = tv ? now() : 0;
            for (int k = 0; k < BP_NK; k++) bp4i_visit_word(s->h + (row + k) * HS, s->dc + (row + k) * DS, p);
            if (tv) *tv += now() - t0;
        }
        for (int k = 0; k < BP_NK; k++) {
            size_t o = row + k;
            uint32_t *h = s->h + o * HS;
            uint32_t M0 = m0[k], MI = mi[k], lw = h[BI_LW], P = h[BI_P], tb = s->tb[o * TS];
            uint32_t idle = (c + (uint32_t)(k + y) % 5) % 5 == 4 ? ~0u : 0;   /* (the shared idle: a word's phase) */
            uint32_t drv = MI & P;
            uint32_t q0 = h[BI_OP], q1 = h[BI_OP + 1], q2 = h[BI_OP + 2], q3 = h[BI_OP + 3], b0 = h[BI_OW], b1 = h[BI_OW + 1];
            uint32_t arrive = M0 & (lw ^ tb), qnz = q0 | q1 | q2 | q3;
            if (M0 && !(M0 & ((lw ^ tb) | b0 | b1 | qnz))) { h[BI_P] = P & ~M0; M0 = 0; }   /* (light: nothing to do) */
            if (M0) {
                uint32_t n[4], qv[4] = { q0, q1, q2, q3 }, br = 0;
                for (int i = 0; i < 4; i++) {           /* R - q, 4 bits (q <= R: no borrow out) */
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
                uint32_t nn2 = ge2 & ~idle, nn1 = ge1 & ~nn2;
                uint32_t p1 = (~n1 & nn1) | (n1 & ~n2 & nn2), p2 = ~n1 & nn2;
                uint32_t r0 = b0 ^ p1, bw = ~b0 & p1, r1 = b1 ^ p2 ^ bw;
                uint32_t qn[4], bo = 0, sb[4] = { nn1, nn2, 0, 0 };
                for (int i = 0; i < 4; i++) {
                    uint32_t d = n[i] ^ sb[i] ^ bo;
                    bo = (~n[i] & (sb[i] | bo)) | (sb[i] & bo);
                    qn[i] = d;
                }
                for (int i = 0; i < 4; i++) qn[i] &= ~bo;              /* (a borrow out: below 0) */
                uint32_t nnz = nn1 | nn2, v[5] = { nn1, nn2 | (~tb & nn1), ~tb & nnz, ~tb & nnz, ~tb & nnz }, cy = 0;
                for (int i = 0; i < 5; i++) {
                    uint32_t a_ = h[BI_AC + i], x = a_ ^ v[i];
                    uint32_t sum = x ^ cy;
                    cy = (a_ & v[i]) | (x & cy);
                    h[BI_AC + i] = (sum & act) | (a_ & ~act);
                }
                for (int i = 0; i < 4; i++) h[BI_OP + i] = (qn[i] & act) | (qv[i] & ~act);
                h[BI_OW] = (r0 & act) | (h[BI_OW] & ~act);
                h[BI_OW + 1] = (r1 & act) | (h[BI_OW + 1] & ~act);
                h[BI_PD] |= arrive;
                h[BI_RV] = (h[BI_RV] & ~arrive) | (arrive & qnz);
                h[BI_H8] = (h[BI_H8] & ~arrive) | h8;
                lw = (lw & ~act) | (tb & act);
                h[BI_LW] = lw;
                h[BI_P] = (P & ~M0) | (act & nn2);
                drv |= act & ge1;
            }
            for (int q = 0; q < 4 && 4 * k + q < BP_GW; q++)
                words[wo + 4 * k + q] = tpl[wo + 4 * k + q] | bp_wire(drv & lw, drv & ~lw, q);
        }
    }
}
static void bp4i_rows(bp4i_state *s, const bp_masks *m, const epd_stag_par *p, int M, uint32_t f, const uint32_t *tpl, uint32_t *words, int y0, int y1, double *tv)
{
    switch (s->kind) {                          /* (the strides as constants) */
    case BI_A25: bp4i_rows_g(s, m, p, M, f, tpl, words, y0, y1, tv, 25, 25, 25); break;
    case BI_A32: bp4i_rows_g(s, m, p, M, f, tpl, words, y0, y1, tv, 32, 32, 32); break;
    default:     bp4i_rows_g(s, m, p, M, f, tpl, words, y0, y1, tv, 16, 1, 8); break;
    }
}

/* one frame over nt threads (rows split), thread i pinned to cpus[i] when given */
typedef struct { bp4i_state *s; const bp_masks *m; const epd_stag_par *p; int M; uint32_t f; uint32_t *w; int y0, y1, cpu; double *tv; } bp4i_job;
static void *bp4i_run(void *a)
{
    bp4i_job *j = a;
    if (j->cpu >= 0) { cpu_set_t cs; CPU_ZERO(&cs); CPU_SET(j->cpu, &cs); sched_setaffinity(0, sizeof cs, &cs); }
    bp4i_rows(j->s, j->m, j->p, j->M, j->f, E.tpl, j->w, j->y0, j->y1, j->tv);
    return NULL;
}
static void bp4i_frame(bp4i_state *s, const bp_masks *m, const epd_stag_par *p, int M, uint32_t f, uint32_t *w, int nt, const int *cpus, int ncpu, double *tv)
{
    pthread_t t[16];
    bp4i_job j[16];
    int up[16] = { 0 };
    for (int i = 0; i < nt; i++) {
        j[i] = (bp4i_job){ s, m, p, M, f, w, EPD_H * i / nt, EPD_H * (i + 1) / nt, ncpu ? cpus[i % ncpu] : -1, nt == 1 ? tv : NULL };
        if (i) up[i] = !pthread_create(&t[i], NULL, bp4i_run, &j[i]);
        if (i && !up[i]) bp4i_run(&j[i]);
    }
    bp4i_run(&j[0]);
    for (int i = 1; i < nt; i++) if (up[i]) pthread_join(t[i], NULL);
}

/* the gate: bp4i == bp4 in lockstep on bp4_gate's frames (every frame's
 * words, the full state every 8th frame); then the time a frame of both,
 * bp4 one thread, bp4i on nt threads; the visits' time at nt 1 */
static int bp4i_gate(void)
{
    const char *ls = getenv("STAG_BP_LAYOUTS"), *rs = getenv("STAG_BP3_RS"), *ms = getenv("STAG_TIER_M");
    const char *ks = getenv("STAG_BP4I_MEM"), *ns = getenv("STAG_BP4I_NT"), *cs = getenv("STAG_BP4I_CPUS");
    const int ph = 1404 * 4, M = ms ? atoi(ms) : 4, brk = getenv("STAG_BP4I_BREAK") ? atoi(getenv("STAG_BP4I_BREAK")) : 0;
    int cpus[16], ncpu = 0;
    if (cs) { char cb[128]; snprintf(cb, sizeof cb, "%s", cs); for (char *t = strtok(cb, " ,"); t && ncpu < 16; t = strtok(NULL, " ,")) cpus[ncpu++] = atoi(t); }
    uint8_t *pg = page(ph), *tg = malloc(EPD_NPX);
    bp4_state b4;
    bp_masks m;
    static uint32_t W4[EPD_WORDS], WI[EPD_WORDS];
    long nbad = 0, nfr = 0;
    if (!tg || bp4_alloc(&b4)) return 2;
    char lb[128], rb[64], kb[64], nb[64], *lsv, *rsv, *ksv, *nsv;
    snprintf(lb, sizeof lb, "%s", ls && *ls ? ls : "64 wbayer 0");
    snprintf(rb, sizeof rb, "%s", rs && *rs ? rs : "9 14");
    for (char *lt = strtok_r(lb, " ,", &lsv); lt; lt = strtok_r(NULL, " ,", &lsv)) {
        char rb2[64];
        snprintf(rb2, sizeof rb2, "%s", rb);
        for (char *rt = strtok_r(rb2, " ,", &rsv); rt; rt = strtok_r(NULL, " ,", &rsv))
        for (int wide = 0; wide < 2; wide++) {
            int R = atoi(rt);
            epd_stag_par p = { .G = 2, .n = 1, .ctl = 3, .K = 8, .payrule = 1, .pcap = 3, .htop = 2, .dither = 0, .R = R };
            if (bp_layout(&p, lt) || !bp3_par_ok(&p) || bp_masks_make(&m, &p)) { printf("BP4I: layout %s R %d: not taken\n", lt, R); return 2; }
            snprintf(kb, sizeof kb, "%s", ks && *ks ? ks : "a25 a32 h16");
            for (char *kt = strtok_r(kb, " ,", &ksv); kt; kt = strtok_r(NULL, " ,", &ksv)) {
                int kind = -1;
                for (int i = 0; i < BI_NK; i++) if (!strcmp(kt, bi_kname[i])) kind = i;
                if (kind < 0) { printf("BP4I: no layout %s\n", kt); return 2; }
                snprintf(nb, sizeof nb, "%s", ns && *ns ? ns : "1 2 4 8");
                for (char *nt_ = strtok_r(nb, " ,", &nsv); nt_; nt_ = strtok_r(NULL, " ,", &nsv)) {
                    int nt = atoi(nt_);
                    nt = nt < 1 ? 1 : nt > 16 ? 16 : nt;
                    bp4i_state bi;
                    if (bp4i_alloc(&bi, kind)) return 2;
                    /* the start: bp4_gate's (planes zeroed first: its set is read-modify-write) */
                    {   uint32_t *pl[BP4_NPL]; bp4_planes(&b4, pl); for (int i = 0; i < BP4_NPL; i++) memset(pl[i], 0, BP_NW * 4); }
                    memcpy(tg, pg, EPD_NPX);
                    for (int y = 0; y < EPD_H; y++)
                        for (int x = 0; x < EPD_W; x++) {
                            size_t i = (size_t)y * EPD_W + x;
                            int l = tg[i] >= 16, dc = wide ? (int)((i * 37) % 201) - 100 : (int)(i % 7) - 3;
                            if (dc > 127) dc = 127;
                            if (dc < -127) dc = -127;
                            size_t o = (size_t)y * BP_NK + x / 32;
                            uint32_t bit = 1u << (31 - x % 32);
                            if (l) b4.lw[o] |= bit;
                            for (int j = 0; j < 8; j++) if ((dc >> j) & 1) b4.dc[j][o] |= bit;
                        }
                    {   bp_state t = { .tb = b4.tb }; bp_targets(&t, tg, ALL); }
                    bp4i_from(&bi, &b4);
                    int y0 = 0;
                    long lbad = 0;
                    double t4[3] = { 0, 0, 0 }, ti[3] = { 0, 0, 0 }, v4[3] = { 0, 0, 0 }, vi[3] = { 0, 0, 0 };
                    int nf[3] = { 0, 0, 0 };
                    uint32_t fc = 61;
                    for (int f = 0; f < 160 && lbad < 5; f++, fc++) {
                        epd_rect r = { 0, 0, 0, 0 };
                        int stretch = f < 64 ? 0 : f < 96 ? 1 : 2;
                        if (stretch == 0) { if (f % 2 == 0) { y0 = (y0 + 17) % (ph - EPD_H); r = ALL; } }
                        else if (stretch == 2 && f % 2 == 0) {
                            y0 = (y0 + 17) % (ph - EPD_H);
                            r = (f / 2) % 2 ? (epd_rect){ 300, 400, 500, 380 } : (epd_rect){ 37, 101, 1500, 1203 };
                        }
                        if (r.w) {
                            for (int y = r.y; y < r.y + r.h; y++)
                                memcpy(tg + (size_t)y * EPD_W + r.x, pg + (size_t)(y0 + y) * EPD_W + r.x, (size_t)r.w);
                            bp_state t = { .tb = b4.tb };
                            bp_targets(&t, tg, r);
                            bp4i_tb_from(&bi, b4.tb);
                        }
                        for (int pass = 0; pass < 2; pass++) {   /* (the order alternates: neither always gets the warm masks) */
                            if ((pass ^ f) & 1) {
                                memcpy(W4, E.tpl, sizeof W4);
                                double a = now();
                                bp4_tv = &v4[stretch];
                                bp4_rows(&b4, &m, &p, M, fc, E.tpl, W4, 0, EPD_H);
                                bp4_tv = NULL;
                                t4[stretch] += now() - a;
                            } else {
                                memcpy(WI, E.tpl, sizeof WI);
                                double a = now();
                                bp4i_frame(&bi, &m, &p, M, fc, WI, nt, cpus, ncpu, &vi[stretch]);
                                ti[stretch] += now() - a;
                            }
                        }
                        nf[stretch]++;
                        if (brk && f == 3) bi.h[(size_t)700 * BP_NK * bi_hs[kind] + (brk == 1 ? BI_AC : BI_LW)] ^= 1;   /* (the negative control: must differ) */
                        if (memcmp(W4, WI, sizeof W4)) {
                            size_t fw = 0;
                            while (W4[fw] == WI[fw]) fw++;
                            printf("BP4I gate: %s R %d %s %s nt %d frame %d: the words differ, first %zu\n", bp_lname(&p), R, wide ? "wide" : "small", kt, nt, f, fw);
                            lbad++;
                        }
                        if (f % 8 == 7 || f == 159) {
                            long sd = bp4i_cmp(&bi, &b4);
                            if (sd) { printf("BP4I gate: %s R %d %s %s nt %d frame %d: state: %ld words differ\n", bp_lname(&p), R, wide ? "wide" : "small", kt, nt, f, sd); lbad++; }
                        }
                        nfr++;
                    }
                    if (lbad) printf("BP4I gate: %-9s R %2d %-5s %s nt %d: DIFFERS (no timing)\n", bp_lname(&p), R, wide ? "wide" : "small", kt, nt);
                    else if (nt == 1)
                        printf("BP4I gate: %-9s R %2d %-5s %s nt 1: equal; ms a frame, this host: scroll %.2f vs bp4 %.2f, still %.2f vs %.2f, window %.2f vs %.2f; the visits: %.2f / %.2f / %.2f vs %.2f / %.2f / %.2f\n",
                               bp_lname(&p), R, wide ? "wide" : "small", kt,
                               ti[0] / nf[0] * 1e3, t4[0] / nf[0] * 1e3, ti[1] / nf[1] * 1e3, t4[1] / nf[1] * 1e3, ti[2] / nf[2] * 1e3, t4[2] / nf[2] * 1e3,
                               vi[0] / nf[0] * 1e3, vi[1] / nf[1] * 1e3, vi[2] / nf[2] * 1e3, v4[0] / nf[0] * 1e3, v4[1] / nf[1] * 1e3, v4[2] / nf[2] * 1e3);
                    else
                        printf("BP4I gate: %-9s R %2d %-5s %s nt %d: equal; ms a frame, this host%s%s: scroll %.2f, still %.2f, window %.2f (bp4 one thread %.2f / %.2f / %.2f)\n",
                               bp_lname(&p), R, wide ? "wide" : "small", kt, nt, ncpu ? ", cpus " : "", ncpu ? cs : "",
                               ti[0] / nf[0] * 1e3, ti[1] / nf[1] * 1e3, ti[2] / nf[2] * 1e3, t4[0] / nf[0] * 1e3, t4[1] / nf[1] * 1e3, t4[2] / nf[2] * 1e3);
                    nbad += lbad;
                    bp4i_free(&bi);
                }
            }
            bp_masks_free(&m);
        }
    }
    printf("BP4I gate: %ld frames, %ld bad (M %d)\n", nfr, nbad, M);
    bp4_free(&b4);
    free(pg); free(tg);
    return nbad ? 1 : 0;
}
#endif
