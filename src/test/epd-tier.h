/* epd-tier.h — ctl 3 split into a per-frame tier and a maintenance tier, with
 * op at fewer bits: a scalar per-pixel model scored against today's ctl 3
 * (the two-tier proposal). Test code, included by
 * epd-stagger-test.c after epd-bp3n.h; host only, no GPU.
 *
 * A variant (STAG_TIER_V, tokens like "m4hio3"):
 * m<M> maintenance period in cycles: 0 = today's rule (the budget from dc
 * at the arrival, dc added every step); M >= 1: steps add into a small
 * accumulator, and each pixel's row is visited every M cycles: the
 * accumulator folds into dc, and a pending arrival that has reached
 * its rail gets its payback b = clamp(owe, 0, pcap) then (owe after
 * the need is driven = owe - need at the arrival: the same amount)
 * h the hammer in the maintenance visit (default: at the arrival, as a
 * floor on the owed count; it is cheap there)
 * i the K idle as a shared mask (a word's cycle (c + phase) % 5 == 4 is
 * short) instead of each pixel's pulse count k
 * s dc saturates at +-127 (the planes' 8-bit record)
 * o<B> op in B bits: L = min(2^B, R + 1) levels across 0..R frames; steps
 * and the need are rounded with an ordered dither (no stored carry)
 * Each variant's drive feeds an exact panel model (op 0..R, dc unbounded):
 * frames short at the end, dc, total drive, agreement with today's frames. */
#ifndef EPD_TIER_H
#define EPD_TIER_H

typedef struct {
    char name[24];
    int M, ham_maint, idle_mask, opbits, dcsat;
    int L;                                      /* (op levels) */
    double u;                                   /* (frames a level) */
    uint8_t *lvl, *opv, *owed, *k, *fl;         /* fl: 1 P, 2 pending, 4 reversal, 8 the hammer floor set at it */
    int8_t *acc;
    int16_t *dc;
    uint8_t *opt;                               /* the exact model */
    int16_t *dct;
    int8_t *lastdir;                            /* (the last frame's drive: the planes' gate) */
    long agree, drives, accmax, dcmax;
} tier_v;

static int tier_parse(tier_v *v, const char *t, int R)
{
    memset(v, 0, sizeof *v);
    snprintf(v->name, sizeof v->name, "%s", t);
    v->opbits = 4;
    for (const char *c = t; *c; c++) {
        if (*c == 'm') v->M = (int)strtol(c + 1, (char **)&c, 10), c--;
        else if (*c == 'o') v->opbits = (int)strtol(c + 1, (char **)&c, 10), c--;
        else if (*c == 'h') v->ham_maint = 1;
        else if (*c == 'i') v->idle_mask = 1;
        else if (*c == 's') v->dcsat = 1;
        else return -1;
    }
    v->L = 1 << v->opbits;
    if (v->L > R + 1) v->L = R + 1;
    v->u = (double)R / (v->L - 1);
    return 0;
}
static int tier_alloc(tier_v *v)
{
    v->lvl = calloc(EPD_NPX, 1); v->opv = calloc(EPD_NPX, 1); v->owed = calloc(EPD_NPX, 1); v->k = calloc(EPD_NPX, 1);
    v->fl = calloc(EPD_NPX, 1); v->acc = calloc(EPD_NPX, 1); v->dc = calloc(EPD_NPX, 2); v->opt = calloc(EPD_NPX, 1);
    v->dct = calloc(EPD_NPX, 2); v->lastdir = calloc(EPD_NPX, 1);
    return v->lastdir && v->lvl && v->opv && v->owed && v->k && v->fl && v->acc && v->dc && v->opt && v->dct ? 0 : -1;
}
static void tier_free(tier_v *v)
{
    free(v->lvl); free(v->opv); free(v->owed); free(v->k); free(v->fl); free(v->acc); free(v->dc); free(v->opt); free(v->dct); free(v->lastdir);
}
/* the ordered dither in [0, 1) for pixel i at cycle c */
static inline double tier_d(size_t i, uint32_t c)
{
    unsigned x = (unsigned)(i % EPD_W), y = (unsigned)(i / EPD_W);
    return (((x * 7 + y * 13 + c * 5) & 15) + 0.5) / 16;
}
static inline int tier_need(const tier_v *v, int lvl_to, int opv, size_t i, uint32_t c)
{
    int lv = lvl_to ? v->L - 1 - opv : opv;
    if (v->L > 0 && v->u == 1.0) return lv;
    return (int)(lv * v->u + tier_d(i, c + 7));
}

/* one pixel's decision on its group frame: nn (0..2) toward tb */
static int tier_decide(tier_v *v, const epd_stag_par *p, size_t i, int tb, uint32_t c)
{
    int lvl = v->lvl[i], P = v->fl[i] & 1, sd = tb ? 1 : -1;
    int need = tier_need(v, tb, v->opv[i], i, c);
    int b = v->owed[i];
    if (lvl != tb) {                            /* an arrival */
        int rev = need < p->R, hc = p->htop < p->pcap ? p->htop : p->pcap;
        if (!v->M) {                            /* today: b from dc now */
            int x = -v->dc[i] * sd - need;
            b = x <= 0 ? 0 : x < p->pcap ? x : p->pcap;
            if (p->htop > 0 && rev && b < hc) b = hc;
        } else {
            int fl8 = !v->ham_maint && p->htop > 0 && rev;
            b = fl8 ? hc : 0;
            v->fl[i] = (uint8_t)((v->fl[i] & ~14) | 2 | (rev ? 4 : 0) | (fl8 ? 8 : 0));
        }
    } else if (need == 0 && b == 0) {
        v->fl[i] &= (uint8_t)~1;                /* nothing to do (today's hot skip: k untouched) */
        return 0;
    }
    int want = need + b, nn = want < p->G ? want : p->G;
    int kin = P ? v->k[i] : 0;
    if (nn == p->G) {
        if (v->idle_mask) { unsigned ph = (unsigned)((i % EPD_W) / 32 + i / EPD_W) % 5; if ((c + ph) % 5 == 4) nn = p->G - 1; }
        else if (kin + nn > p->K) nn = p->G - 1;
    }
    v->k[i] = (uint8_t)(nn == p->G ? kin + nn : 0);
    b -= nn > need ? nn - need : 0;
    v->owed[i] = (uint8_t)(b < 0 ? 0 : b);
    /* op: nn frames as levels, rounded by the dither */
    int dl = v->u == 1.0 ? nn : (int)(nn / v->u + tier_d(i, c));
    int o = v->opv[i] + sd * dl;
    v->opv[i] = (uint8_t)(o < 0 ? 0 : o > v->L - 1 ? v->L - 1 : o);
    if (v->M) {
        int a = v->acc[i] + sd * nn;
        v->acc[i] = (int8_t)a;
        if (abs(a) > v->accmax) v->accmax = abs(a);
    } else {
        int d = v->dc[i] + sd * nn, dm = v->dcsat ? 127 : EPD_STAG_DCMAX;
        v->dc[i] = (int16_t)(d < -dm ? -dm : d > dm ? dm : d);
    }
    v->lvl[i] = (uint8_t)tb;
    v->fl[i] = (uint8_t)((v->fl[i] & ~1) | (nn == p->G));
    return nn;
}
static void tier_visit(tier_v *v, const epd_stag_par *p, size_t i, uint32_t c)
{
    int d = v->dc[i] + v->acc[i], dm = v->dcsat ? 127 : EPD_STAG_DCMAX;
    v->dc[i] = (int16_t)(d < -dm ? -dm : d > dm ? dm : d);
    v->acc[i] = 0;
    if ((v->fl[i] & 2) && tier_need(v, v->lvl[i], v->opv[i], i, c) == 0) {
        int sd = v->lvl[i] ? 1 : -1, x = -v->dc[i] * sd, hc = p->htop < p->pcap ? p->htop : p->pcap;
        /* today's b = max(clamp(owe - need), hammer); the hammer floor set at
         * the arrival (fl 8) has paid hc - owed of it already, and dc has them */
        int paid = (v->fl[i] & 8) ? hc - v->owed[i] : 0;
        x += paid;
        int b = x <= 0 ? 0 : x < p->pcap ? x : p->pcap;
        if (p->htop > 0 && (v->fl[i] & 4) && b < hc) b = hc;
        b -= paid;
        v->owed[i] = (uint8_t)(b > 0 ? b : 0);
        v->fl[i] &= (uint8_t)~14;
    }
}
static inline void tier_phys(uint8_t *opt, int16_t *dct, size_t i, int dir, int R, long *dcmax)
{
    int o = opt[i] + dir;
    opt[i] = (uint8_t)(o < 0 ? 0 : o > R ? R : o);
    int d = dct[i] + dir;
    dct[i] = (int16_t)(d < -30000 ? -30000 : d > 30000 ? 30000 : d);
    if (abs(d) > *dcmax) *dcmax = abs(d);
}

/* one variant through one frame (a thread a variant) */
typedef struct { tier_v *t; const epd_stag_par *p; const bp_masks *m; const uint8_t *tg; const int8_t *rd; uint32_t fc; } tier_job;
static void *tier_run(void *a)
{
    tier_job *j = a;
    tier_v *t = j->t;
    uint32_t fc = j->fc, c = fc / 2;
    int fm = (int)(fc & 1), R = j->p->R;
    for (int y = 0; y < EPD_H; y++) {
        const uint32_t *m0 = j->m->gm + ((size_t)fm * 64 + (y & 63)) * BP_NK;
        int visit = t->M && fc % 2 == 0 && (uint32_t)y % (uint32_t)t->M == c % (uint32_t)t->M;
        for (int x = 0; x < EPD_W; x++) {
            size_t i = (size_t)y * EPD_W + x;
            if (visit) tier_visit(t, j->p, i, c);
            int dir = 0, dec = (m0[x / 32] >> (31 - x % 32)) & 1;
            if (dec) { if (tier_decide(t, j->p, i, j->tg[i] >= 16, c)) dir = t->lvl[i] ? 1 : -1; }
            else if (t->fl[i] & 1) dir = t->lvl[i] ? 1 : -1;
            if (dir) { tier_phys(t->opt, t->dct, i, dir, R, &t->dcmax); t->drives++; }
            t->agree += dir == j->rd[i];
            t->lastdir[i] = (int8_t)dir;
        }
    }
    return NULL;
}

static void tier_report(const char *name, const uint8_t *opt, const int16_t *dct, const uint8_t *tg, int R, long agree, long drives, long rdrives,
                        long dcmax, long accmax, long nframes)
{
    double sh = 0, adc = 0;
    long sh2 = 0, dc60 = 0;
    for (size_t i = 0; i < EPD_NPX; i++) {
        int rail = tg[i] >= 16 ? R : 0, s = abs(opt[i] - rail);
        sh += s; sh2 += s >= 2;
        adc += abs(dct[i]); dc60 += abs(dct[i]) > 60;
    }
    printf("  %-10s agree %6.2f %%  drive %5.3f x  short %.3f fr (>=2: %ld)  |dc| mean %.2f max %ld >60 %ld%s",
           name, 100.0 * agree / ((double)EPD_NPX * nframes), rdrives ? (double)drives / rdrives : 1.0, sh / EPD_NPX, sh2,
           adc / EPD_NPX, dcmax, dc60, accmax ? "" : "\n");
    if (accmax) printf("  acc max %ld\n", accmax);
}

static int tier_sim(void)
{
    const char *ls = getenv("STAG_BP_LAYOUTS"), *rs = getenv("STAG_BP3_RS"), *vs = getenv("STAG_TIER_V");
    const int ph = 1404 * 4;
    uint8_t *pg = page(ph), *tg = malloc(EPD_NPX);
    char lb[128], rb[64], vb[256];
    snprintf(lb, sizeof lb, "%s", ls && *ls ? ls : "64 wbayer 0");
    snprintf(rb, sizeof rb, "%s", rs && *rs ? rs : "9 14");
    snprintf(vb, sizeof vb, "%s", vs && *vs ? vs : "m0 m0o3 m0o2 m1 m2 m4 m8 m4h m4i m4hi m4hio3 m4hio2");
    tier_v V[24];
    int nv = 0;
    uint8_t *ropt = malloc(EPD_NPX);
    int8_t *rd = malloc(EPD_NPX);
    int16_t *rdct = malloc(EPD_NPX * 2);
    if (!tg || !ropt || !rdct || !rd) return 2;
    char *lsv, *rsv, *vsv;
    for (char *lt = strtok_r(lb, " ,", &lsv); lt; lt = strtok_r(NULL, " ,", &lsv)) {
        char rb2[64];
        snprintf(rb2, sizeof rb2, "%s", rb);
        for (char *rt = strtok_r(rb2, " ,", &rsv); rt; rt = strtok_r(NULL, " ,", &rsv))
        for (int wide = 0; wide < 2; wide++) {
            int R = atoi(rt);
            epd_stag_par p = { .G = 2, .n = 1, .ctl = 3, .K = 8, .payrule = 1, .pcap = 3, .htop = 2, .dither = 0, .R = R };
            if (bp_layout(&p, lt) || !bp3_par_ok(&p)) { printf("TIER: layout %s R %d: not taken\n", lt, R); return 2; }
            bp_masks m;
            if (bp_masks_make(&m, &p)) return 2;
            char vb2[256];
            snprintf(vb2, sizeof vb2, "%s", vb);
            nv = 0;
            for (char *vt = strtok_r(vb2, " ,", &vsv); vt && nv < 24; vt = strtok_r(NULL, " ,", &vsv)) {
                if (tier_parse(&V[nv], vt, R)) { printf("TIER: bad variant %s\n", vt); return 2; }
                if (tier_alloc(&V[nv])) return 2;
                nv++;
            }
            int y0 = 0;
            memcpy(tg, pg, EPD_NPX);
            epd_stag_enter(&S);
            for (size_t i = 0; i < EPD_NPX; i++) {
                int l = tg[i] >= 16 ? 30 : 0, op = l ? R : 0;
                int dc = wide ? (int)((i * 37) % 201) - 100 : (int)(i % 7) - 3;
                S.cold[i] = epd_stag_cold(l, op, 0, dc);
                S.hot[i] = (uint16_t)epd_stag_hotbits(l, op, dc, &p);
                S.ld[i] = 1;
                ropt[i] = (uint8_t)op; rdct[i] = (int16_t)dc;
                for (int v = 0; v < nv; v++) {
                    V[v].lvl[i] = (uint8_t)(l != 0); V[v].opv[i] = (uint8_t)(l ? V[v].L - 1 : 0); V[v].dc[i] = (int16_t)dc;
                    V[v].opt[i] = (uint8_t)op; V[v].dct[i] = (int16_t)dc;
                }
            }
            S.f = 61;
            memcpy(S.tgt, tg, EPD_NPX);
            long rdr = 0, rdcmax = 0, nfr = 0;
            const int NF = 256;                 /* 64 scroll, 32 still, 64 window, 96 settle */
            for (int f = 0; f < NF; f++) {
                epd_rect r = { 0, 0, 0, 0 };
                if (f < 64) { if (f % 2 == 0) { y0 = (y0 + 17) % (ph - EPD_H); r = ALL; } }
                else if (f >= 96 && f < 160 && f % 2 == 0) {
                    y0 = (y0 + 17) % (ph - EPD_H);
                    r = (f / 2) % 2 ? (epd_rect){ 300, 400, 500, 380 } : (epd_rect){ 37, 101, 1500, 1203 };
                }
                for (int y = r.y; y < r.y + r.h; y++) {
                    memcpy(tg + (size_t)y * EPD_W + r.x, pg + (size_t)(y0 + y) * EPD_W + r.x, (size_t)r.w);
                    memcpy(S.tgt + (size_t)y * EPD_W + r.x, tg + (size_t)y * EPD_W + r.x, (size_t)r.w);
                }
                uint32_t fc = S.f;
                frame(&p);
                for (int y = 0; y < EPD_H; y++)         /* (today's drive, scored by the exact model) */
                    for (int x = 0; x < EPD_W; x++) {
                        size_t i = (size_t)y * EPD_W + x;
                        int rp = phase_at(W, x, y), rdir = rp == 2 ? 1 : rp == 1 ? -1 : 0;
                        rd[i] = (int8_t)rdir;
                        if (rdir) { tier_phys(ropt, rdct, i, rdir, R, &rdcmax); rdr++; }
                    }
                pthread_t th[24];
                tier_job jb[24];
                for (int v = 0; v < nv; v++) {
                    jb[v] = (tier_job){ &V[v], &p, &m, tg, rd, fc };
                    if (pthread_create(&th[v], NULL, tier_run, &jb[v])) tier_run(&jb[v]), th[v] = 0;
                }
                for (int v = 0; v < nv; v++) if (th[v]) pthread_join(th[v], NULL);
                nfr++;
            }
            printf("TIER %s R %d %s start record (%d frames; today's ctl 3: drive %ld)\n", bp_lname(&p), R, wide ? "wide" : "small", NF, rdr);
            tier_report("today", ropt, rdct, tg, R, (long)EPD_NPX * nfr, rdr, rdr, rdcmax, 0, nfr);
            for (int v = 0; v < nv; v++) {
                tier_report(V[v].name, V[v].opt, V[v].dct, tg, R, V[v].agree, V[v].drives, rdr, V[v].dcmax, V[v].accmax, nfr);
                tier_free(&V[v]);
            }
            bp_masks_free(&m);
        }
    }
    free(pg); free(tg); free(ropt); free(rdct); free(rd);
    return 0;
}
#endif
