/* epd-claim-bench — : the claim's pixel pass
 * (epd_claim_pixels) single-threaded vs a prototype that runs it in K row
 * bands on K threads, on realistic engine states, bit for bit. No production
 * change: this file includes ../epd-engine.c (its static row helpers) and
 * takes every claim through the engine's pixel-ops hook (epd_set_pixel_ops:
 * `pixels`, with the CPU reference query), where it snapshots the rect's
 * state, runs the production pass, restores, runs the banded pass, and
 * compares busy/target/level/op/dc/stale over the rect and the report
 * (epd_claim_out) byte for byte (the order alternates per claim: cache
 * warmth). Lazy plans (fewer free slots than cuts: slots taken in pixel
 * order) stay serial (counted). With PROF=1 (single thread) the production
 * pass's row helpers are timed one by one (Amdahl split; ~25 ns per timed call).
 * epd-claim-bench <wbf> [-t THREADS] [-c CPULIST] [-s scenario] [-n frames]
 * scenarios: scroll (1872x1300 A2_OR_DU cut steps, payback defaults; the
 * daily scroll), window (a 1404x1000 window, the same), bwscroll (the
 * scroll's page dithered to black/white, as a motion run's A2 steps get),
 * page (full-panel GC16 page turns, partial; every 4th a flash of the
 * whole panel, full=1), pen (64x64 DU pieces along strokes, 4 every 3
 * frames), all (each in turn). CPULIST: e.g. 1-4 (Note Air 1: the Gold cores) or
 * 0,5-7 (Silver); the caller runs band 0 on the first CPU, worker i on the next.
 * KERNEL=packed : the other side is the packed
 * pass (epd_claim_prep/_rows/_merge in THREADS bands; -t 1 = production's
 * claim_kernel=packed) instead of the banded array pass, compared against the
 * arrays' pass (claim_pixels_soa) the same way; every finishing run's rows
 * too (epd_finish_rows_packed vs finish_row, the finish hook); and per claim
 * the boundary alone (pack + unpack of the rect, the tax while both exist). The
 * engine's claim_kernel (GLASSINE_CLAIM_KERNEL) only picks lazy / npre > 4 claims'
 * side here, both are run either way.
 * PROD=1 : the other side is production's
 * epd_claim_pixels with epd_claim_bands = THREADS on this pool (the band
 * ops; the kernel by GLASSINE_CLAIM_KERNEL; claims under EPD_BAND_MIN_PX one
 * band), against claim_pixels_soa. */
#define _GNU_SOURCE
#include <pthread.h>
#include <sched.h>
#include "../epd-engine.c"

static int nthr = 4, cpus[16], ncpu;
static int g_prof, g_kern, g_prod, g_s8, g_s8prof;

/* ---- the banded pass ---------------------------------------------------- */
typedef struct {
    uint8_t ca[EPD_W], cd[EPD_W], ch[EPD_W], cls[EPD_W], fl[EPD_W], cq[EPD_W];
    int8_t nrow[EPD_W];
    uint16_t mcol[EPD_W], xl[EPD_W + 1];
    int mb1[EPD_LUT_CUTS], mb3[EPD_LUT_CUTS];
    int y0[3], y1[3];
    uint64_t preempted, snapped, matched, flash_skipped;
} band_t;
static band_t bands[16];

/* what every band reads (made once per claim, serially, as production does) */
static struct {
    epd_engine *e;
    const epd_claim_plan *pl;
    struct { uint8_t own; int8_t net; uint16_t bit; } ctab[256];
    uint8_t pob[4];
    int8_t prn[4][4];
    int npo;
} J;
static const uint8_t zeros_b[EPD_W];
static uint8_t ones_b[EPD_W];

static void band_rows(band_t *B, int ya, int yb)
{
    epd_engine *e = J.e;
    const epd_claim_plan *pl = J.pl;
    const int W = pl->r.w, split = pl->split, opm = pl->opm, R = pl->R, aclamp = pl->aclamp;
    const uint8_t oa = pl->oa, od = pl->od, hown = pl->hown;
    memset(B->ca, 0, (size_t)W); memset(B->cd, 0, (size_t)W);
    if (hown) memset(B->ch, 0, (size_t)W);
    memset(B->mcol, 0, (size_t)W * sizeof B->mcol[0]);
    for (int c = 0; c < EPD_LUT_CUTS; c++) { B->mb1[c] = INT32_MAX; B->mb3[c] = -1; }
    for (int k = 0; k < 3; k++) B->y0[k] = B->y1[k] = -1;
    B->preempted = B->snapped = B->matched = B->flash_skipped = 0;
    for (int y = ya; y < yb; y++) {
        size_t o = (size_t)(pl->r.y + y) * EPD_W + pl->r.x;
        uint8_t *b = e->busy + o;
        const uint8_t *tg = pl->tg + (size_t)y * W, *lv = e->level + o;
        if (J.npo) {
            if (preempt_any(b, e->target + o, tg, W, J.pob))
                B->preempted += (uint64_t)preempt_owners_row(b, e->target + o, tg, e->level + o, e->op + o, e->dc + o, W,
                                                             J.pob, J.prn, opm, R);
        }
        else if (pl->npre) B->preempted += (uint64_t)preempt_row(b, e->target + o, tg, e->level + o, e->op + o, e->dc + o, W,
                                                                  pl->pre, (int8_t (*)[4])pl->rn, opm, R);
        if (pl->snp) B->snapped += (uint64_t)snap_row(b, e->level + o, tg, e->op + o, W, R);
        const uint8_t *fm = pl->fm == 2 ? e->stale + o : pl->fm ? ones_b : zeros_b;
        int any = pl->hact ? claim_row_heal(b, e->target + o, tg, e->level + o, e->op + o, e->dc + o, B->ca, B->cd, W, fm, split, oa, od, pl->hf0, pl->hf1)
                           : claim_row(b, e->target + o, tg, lv, B->ca, B->cd, W, fm, split, oa, od);
        int healed = hown && heal_row(b, e->target + o, e->level + o, e->op + o, e->dc + o, tg, B->ch, W, hown, R, pl->hmin, pl->hmax);
        if (healed) {
            if (B->y0[EPD_CL_H] < 0) B->y0[EPD_CL_H] = y;
            B->y1[EPD_CL_H] = y;
        }
        if (any || healed) stale_row(e->stale + o, b, W, oa, pl->va, od, pl->vd, healed ? hown : 0);
        if (pl->fm == 2) B->flash_skipped += (uint64_t)flash_skips(e->stale + o, tg, lv, b, W);
        if (!any) continue;
        if (any & 1) { if (B->y0[EPD_CL_A] < 0) B->y0[EPD_CL_A] = y; B->y1[EPD_CL_A] = y; }
        if (any & 2) { if (B->y0[EPD_CL_D] < 0) B->y0[EPD_CL_D] = y; B->y1[EPD_CL_D] = y; }
        int16_t *dc = e->dc + o;
        uint8_t *opr = e->op + o;
        if (split && !pl->mt) {
            if (any & 1) dc_row_bw(dc, b, tg, lv, W, oa, pl->neta);
            if (any & 2)
                for (int x = 0; x < W; x++)
                    if (b[x] == od) {
                        int d = dc[x] + pl->netd[tg[x] << 5 | lv[x]];
                        dc[x] = (int16_t)(d < -30000 ? -30000 : d > 30000 ? 30000 : d);
                    }
            if (opm) {
                if (any & 1) op_row_bw(opr, b, tg, lv, W, oa, pl->neta, aclamp, R);
                if (any & 2) op_row_rail(opr, b, tg, W, od, pl->rail);
            }
            continue;
        }
        gather_bw_row(b, tg, lv, opr, dc, B->fl, B->cq, B->nrow, W, oa, od, pl->neta, pl->mt, pl->model, R);
        uint16_t rb = 0;
        int nl = 0;
        for (int x = 0; x < W; x++) { B->xl[nl] = (uint16_t)x; nl += B->fl[x] >> 1; }
        uint64_t nm = 0;
        for (int j = 0; j < nl; j++) {
            int x = B->xl[j];
            if (B->fl[x] & 1) {
                int k = B->cq[x];
                uint8_t ow = J.ctab[k].own;
                B->nrow[x] = J.ctab[k].net;
                b[x] = ow ? ow : b[x];
                B->mcol[x] |= J.ctab[k].bit;
                rb |= J.ctab[k].bit;
                nm += ow != 0;
                continue;
            }
            uint8_t w = b[x];
            int i = tg[x] << 5 | lv[x], n = (w == oa ? pl->neta : pl->netd)[i];
            if (pl->mt && w == oa && n && !pl->model && dc[x] && (dc[x] < 0) != (n < 0)) {
                int d = dc[x];
                d = d < -EPD_MT_D ? -EPD_MT_D : d > EPD_MT_D ? EPD_MT_D : d;
                int c = pl->best[i][d + EPD_MT_D];
                if (pl->cap && c < pl->cs - pl->cap) c = pl->cs - pl->cap;
                if (c != pl->cs && pl->mown[c]) {
                    b[x] = pl->mown[c];
                    n = pl->mnet[c][i];
                    B->mcol[x] |= (uint16_t)(1u << c);
                    rb |= (uint16_t)(1u << c);
                    B->matched++;
                }
            }
            int dd = dc[x] + n;
            dc[x] = (int16_t)(dd < -30000 ? -30000 : dd > 30000 ? 30000 : dd);
            if (opm) opr[x] = pl->rail[tg[x]];
        }
        B->matched += nm;
        gather_bw_apply(dc, opr, B->fl, tg, B->nrow, W, opm, aclamp, R);
        for (int c = 0; rb && c < EPD_LUT_CUTS; c++)
            if (rb & (1u << c)) { if (B->mb1[c] == INT32_MAX) B->mb1[c] = y; B->mb3[c] = y; }
    }
}

/* the pool: band i on worker i (0 = the caller) */
static pthread_t wth[16];
static pthread_mutex_t wmu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t wgo = PTHREAD_COND_INITIALIZER, wdone = PTHREAD_COND_INITIALIZER;
static unsigned wgen;
static int wleft, wn;
static int wy0, wy1;
static void (*wjob)(int i, int n);

static void job_band(int i, int n)
{
    int a = wy0 + (int)((long)(wy1 - wy0) * i / n), b = wy0 + (int)((long)(wy1 - wy0) * (i + 1) / n);
    band_rows(&bands[i], a, b);
}
static int wcpu[16];                    /* where each band's thread ran (sched_getcpu: the pin checked) */
static void job_empty(int i, int n) { (void)n; wcpu[i] = sched_getcpu(); }

static void pin(int cpu)
{
    if (cpu < 0) return;
    cpu_set_t s;
    CPU_ZERO(&s);
    CPU_SET(cpu, &s);
    if (pthread_setaffinity_np(pthread_self(), sizeof s, &s)) fprintf(stderr, "epd-claim-bench: pin to cpu %d failed\n", cpu);
}

static void *worker(void *arg)
{
    int i = (int)(long)arg;
    pin(ncpu ? cpus[i % ncpu] : -1);
    pthread_mutex_lock(&wmu);
    unsigned seen = wgen;
    for (;;) {
        while (wgen == seen) pthread_cond_wait(&wgo, &wmu);
        seen = wgen;
        if (i >= wn) continue;
        pthread_mutex_unlock(&wmu);
        wjob(i, wn);
        pthread_mutex_lock(&wmu);
        if (!--wleft) pthread_cond_signal(&wdone);
    }
    return NULL;
}

static void pool_run(void (*job)(int, int), int n)
{
    pthread_mutex_lock(&wmu);
    wjob = job; wn = n; wleft = n - 1; wgen++;
    pthread_cond_broadcast(&wgo);
    pthread_mutex_unlock(&wmu);
    job(0, n);
    pthread_mutex_lock(&wmu);
    while (wleft) pthread_cond_wait(&wdone, &wmu);
    pthread_mutex_unlock(&wmu);
}

static double tnow(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec / 1e9; }

/* epd_claim_pixels, banded: the serial prologue as production, the rows in
 * nthr bands, then the merge (ORs of the column marks, min/max of the row
 * extents, sums) and the serial epilogue as production */
static double ser_s;            /* the serial parts' time (prologue + merge + epilogue) */
static void claim_banded(epd_engine *e, epd_claim_plan *pl, epd_claim_out *out)
{
    double t0 = tnow();
    const int W = pl->r.w;
    if (!pl->tg) pl->tg = pend_tg(e, pl->pend);
    memset(out, 0, sizeof *out);
    for (int c = 0; c < EPD_LUT_CUTS; c++) out->mslot[c] = pl->mown[c] ? pl->mown[c] - 1 : -1;
    for (int k = 0; k < EPD_CL_OWNERS; k++) { out->x0[k] = INT32_MAX; out->y0[k] = out->x1[k] = out->y1[k] = -1; }
    if (pl->mt)
        for (int k = 0; k < 4; k++) {
            int i = ((k & 2) ? 30 : 0) << 5 | ((k & 1) ? 30 : 0);
            for (int q = 0; q < 64; q++) {
                int c = q < 2 * EPD_MT_D + 1 ? pl->best[i][q] : pl->cs;
                if (pl->model && c > pl->cs) c = pl->cs;
                if (pl->cap && c < pl->cs - pl->cap) c = pl->cs - pl->cap;
                int hit = c != pl->cs && pl->mown[c];
                J.ctab[k << 6 | q].own = hit ? pl->mown[c] : 0;
                J.ctab[k << 6 | q].net = hit ? pl->mnet[c][i] : pl->neta[i];
                J.ctab[k << 6 | q].bit = (uint16_t)(hit ? 1u << c : 0);
            }
        }
    memset(J.pob, 255, sizeof J.pob);
    memset(J.prn, 0, sizeof J.prn);
    J.npo = 0;
    if (pl->npre && pl->npre <= 4)
        for (int v = 1; v < 256 && J.npo < pl->npre; v++)
            if (pl->pre[v]) { J.pob[J.npo] = (uint8_t)v; memcpy(J.prn[J.npo], pl->rn[pl->pre[v]], 4); J.npo++; }
    J.e = e; J.pl = pl;
    int n = nthr > pl->r.h ? pl->r.h : nthr;
    if (n < 1) n = 1;
    wy0 = 0; wy1 = pl->r.h;
    double t1 = tnow();
    if (n == 1) job_band(0, 1); else pool_run(job_band, n);
    double t2 = tnow();
    /* merge */
    band_t *M = &bands[0];
    for (int i = 1; i < n; i++) {
        band_t *B = &bands[i];
        for (int x = 0; x < W; x++) { M->ca[x] |= B->ca[x]; M->cd[x] |= B->cd[x]; M->mcol[x] |= B->mcol[x]; }
        if (pl->hown) for (int x = 0; x < W; x++) M->ch[x] |= B->ch[x];
        for (int k = 0; k < 3; k++) {
            if (B->y0[k] >= 0 && (M->y0[k] < 0 || B->y0[k] < M->y0[k])) M->y0[k] = B->y0[k];
            if (B->y1[k] > M->y1[k]) M->y1[k] = B->y1[k];
        }
        for (int c = 0; c < EPD_LUT_CUTS; c++) {
            if (B->mb1[c] < M->mb1[c]) M->mb1[c] = B->mb1[c];
            if (B->mb3[c] > M->mb3[c]) M->mb3[c] = B->mb3[c];
        }
        M->preempted += B->preempted; M->snapped += B->snapped; M->matched += B->matched; M->flash_skipped += B->flash_skipped;
    }
    out->preempted = M->preempted; out->snapped = M->snapped; out->matched = M->matched; out->flash_skipped = M->flash_skipped;
    for (int k = 0; k < 3; k++) { out->y0[k] = M->y0[k]; out->y1[k] = M->y1[k]; }
    ext_cols(out, EPD_CL_A, M->ca, W);
    ext_cols(out, EPD_CL_D, M->cd, W);
    if (pl->hown) ext_cols(out, EPD_CL_H, M->ch, W);
    else { out->x0[EPD_CL_H] = 0; out->x1[EPD_CL_H] = -1; }
    int mb[EPD_LUT_CUTS][4];
    for (int c = 0; c < EPD_LUT_CUTS; c++) { mb[c][0] = INT32_MAX; mb[c][1] = M->mb1[c]; mb[c][2] = -1; mb[c][3] = M->mb3[c]; }
    for (int c = 0; c < EPD_LUT_CUTS; c++) {
        if (mb[c][3] < 0) continue;
        int x0 = 0, x1 = W - 1;
        while (!(M->mcol[x0] & (1u << c))) x0++;
        while (!(M->mcol[x1] & (1u << c))) x1--;
        mb[c][0] = x0; mb[c][2] = x1;
    }
    for (int c = 0; c < EPD_LUT_CUTS; c++) {
        int k = EPD_CL_M0 + c;
        if (mb[c][2] < 0) { out->x0[k] = 0; out->x1[k] = -1; continue; }
        out->x0[k] = mb[c][0]; out->y0[k] = mb[c][1]; out->x1[k] = mb[c][2]; out->y1[k] = mb[c][3];
    }
    if (pl->hown)
        for (int y = out->y0[EPD_CL_H]; out->x1[EPD_CL_H] >= 0 && y <= out->y1[EPD_CL_H]; y++)
            for (int x = out->x0[EPD_CL_H]; x <= out->x1[EPD_CL_H]; x++)
                out->healed += e->busy[(size_t)(pl->r.y + y) * EPD_W + pl->r.x + x] == pl->hown;
    ser_s += (t1 - t0) + (tnow() - t2);
}

/* ---- the packed pass (KERNEL=packed): the engine's banding-ready entries --- */
static epd_claim_job PJ;
static epd_claim_band pbands[16];
static const epd_claim_plan *ppl;
static void job_pk(int i, int n)
{
    int a = wy0 + (int)((long)(wy1 - wy0) * i / n), b = wy0 + (int)((long)(wy1 - wy0) * (i + 1) / n);
    epd_claim_rows(J.e, ppl, &PJ, &pbands[i], a, b);
}
static void claim_packed(epd_engine *e, epd_claim_plan *pl, epd_claim_out *out)
{
    double t0 = tnow();
    epd_claim_prep(e, pl, &PJ);
    J.e = e; ppl = pl;
    int n = nthr > pl->r.h ? pl->r.h : nthr;
    if (n < 1) n = 1;
    wy0 = 0; wy1 = pl->r.h;
    double t1 = tnow();
    if (n == 1) job_pk(0, 1); else pool_run(job_pk, n);
    double t2 = tnow();
    epd_claim_band *bp[16];
    for (int i = 0; i < n; i++) bp[i] = &pbands[i];
    epd_claim_merge(pl, bp, n, out);
    ser_s += (t1 - t0) + (tnow() - t2);
}
/* ---- PROD=1 : production's banded pass ----------------
 * epd_claim_pixels with epd_claim_bands = THREADS and the band ops on this
 * pool (the kernel: GLASSINE_CLAIM_KERNEL), against claim_pixels_soa */
static void (*bo_fn)(void *, int);
static void *bo_arg;
static void job_ops(int i, int n) { (void)n; bo_fn(bo_arg, i); }
static void bo_run(void *ctx, void (*fn)(void *, int), void *arg, int n)
{
    (void)ctx;
    bo_fn = fn; bo_arg = arg;
    if (n > nthr) { for (int i = 0; i < n; i++) fn(arg, i); return; }
    pool_run(job_ops, n);
}
static const epd_band_ops bench_bops = { bo_run, NULL, NULL, NULL, NULL };
static void claim_prod(epd_engine *e, epd_claim_plan *pl, epd_claim_out *out)
{
    epd_claim_bands = nthr;
    epd_claim_pixels(e, pl, out);
    epd_claim_bands = 1;
}
static double tax_s;                    /* the boundary alone (pack + unpack of every rect row) */
static void tax(epd_engine *e, epd_rect r)
{
    static epd_claim_band b;
    double t0 = tnow();
    for (int y = 0; y < r.h; y++) {
        size_t o = (size_t)(r.y + y) * EPD_W + r.x;
        pk_pack_row(e, o, r.w, b.d, b.w, b.t);
        pk_unpack_row(e, o, r.w, b.d, b.w, b.t);
    }
    tax_s += tnow() - t0;
}

/* ---- the production pass, its row helpers timed (PROF=1) ---------------- */
enum { RH_PREEMPT, RH_SNAP, RH_CLAIM, RH_HEAL, RH_STALE, RH_FSKIP, RH_SPLIT, RH_GATHER_A, RH_GATHER_B, RH_GATHER_C, RH_N };
static const char *const rh_name[RH_N] = { "preempt_row", "snap_row", "claim_row", "heal_row", "stale_row", "flash_skips",
                                           "dc/op_row (split)", "gather_bw_row (A)", "cut table (B)", "gather_bw_apply (C)" };
static double rh_s[RH_N];
/* band_rows with each helper timed (one band, K=1): the same writes */
static void band_rows_prof(band_t *B, int ya, int yb)
{
    epd_engine *e = J.e;
    const epd_claim_plan *pl = J.pl;
    const int W = pl->r.w, split = pl->split, opm = pl->opm, R = pl->R, aclamp = pl->aclamp;
    const uint8_t oa = pl->oa, od = pl->od, hown = pl->hown;
    memset(B->ca, 0, (size_t)W); memset(B->cd, 0, (size_t)W);
    if (hown) memset(B->ch, 0, (size_t)W);
    memset(B->mcol, 0, (size_t)W * sizeof B->mcol[0]);
    for (int c = 0; c < EPD_LUT_CUTS; c++) { B->mb1[c] = INT32_MAX; B->mb3[c] = -1; }
    for (int k = 0; k < 3; k++) B->y0[k] = B->y1[k] = -1;
    B->preempted = B->snapped = B->matched = B->flash_skipped = 0;
    double t = tnow(), u;
#define LAP(k) do { u = tnow(); rh_s[k] += u - t; t = u; } while (0)
    for (int y = ya; y < yb; y++) {
        size_t o = (size_t)(pl->r.y + y) * EPD_W + pl->r.x;
        uint8_t *b = e->busy + o;
        const uint8_t *tg = pl->tg + (size_t)y * W, *lv = e->level + o;
        t = tnow();
        if (J.npo) {
            if (preempt_any(b, e->target + o, tg, W, J.pob))
                B->preempted += (uint64_t)preempt_owners_row(b, e->target + o, tg, e->level + o, e->op + o, e->dc + o, W,
                                                             J.pob, J.prn, opm, R);
        }
        else if (pl->npre) B->preempted += (uint64_t)preempt_row(b, e->target + o, tg, e->level + o, e->op + o, e->dc + o, W,
                                                                  pl->pre, (int8_t (*)[4])pl->rn, opm, R);
        LAP(RH_PREEMPT);
        if (pl->snp) B->snapped += (uint64_t)snap_row(b, e->level + o, tg, e->op + o, W, R);
        LAP(RH_SNAP);
        const uint8_t *fm = pl->fm == 2 ? e->stale + o : pl->fm ? ones_b : zeros_b;
        int any = pl->hact ? claim_row_heal(b, e->target + o, tg, e->level + o, e->op + o, e->dc + o, B->ca, B->cd, W, fm, split, oa, od, pl->hf0, pl->hf1)
                           : claim_row(b, e->target + o, tg, lv, B->ca, B->cd, W, fm, split, oa, od);
        LAP(RH_CLAIM);
        int healed = hown && heal_row(b, e->target + o, e->level + o, e->op + o, e->dc + o, tg, B->ch, W, hown, R, pl->hmin, pl->hmax);
        if (healed) {
            if (B->y0[EPD_CL_H] < 0) B->y0[EPD_CL_H] = y;
            B->y1[EPD_CL_H] = y;
        }
        LAP(RH_HEAL);
        if (any || healed) stale_row(e->stale + o, b, W, oa, pl->va, od, pl->vd, healed ? hown : 0);
        LAP(RH_STALE);
        if (pl->fm == 2) B->flash_skipped += (uint64_t)flash_skips(e->stale + o, tg, lv, b, W);
        LAP(RH_FSKIP);
        if (!any) continue;
        if (any & 1) { if (B->y0[EPD_CL_A] < 0) B->y0[EPD_CL_A] = y; B->y1[EPD_CL_A] = y; }
        if (any & 2) { if (B->y0[EPD_CL_D] < 0) B->y0[EPD_CL_D] = y; B->y1[EPD_CL_D] = y; }
        int16_t *dc = e->dc + o;
        uint8_t *opr = e->op + o;
        if (split && !pl->mt) {
            if (any & 1) dc_row_bw(dc, b, tg, lv, W, oa, pl->neta);
            if (any & 2)
                for (int x = 0; x < W; x++)
                    if (b[x] == od) {
                        int d = dc[x] + pl->netd[tg[x] << 5 | lv[x]];
                        dc[x] = (int16_t)(d < -30000 ? -30000 : d > 30000 ? 30000 : d);
                    }
            if (opm) {
                if (any & 1) op_row_bw(opr, b, tg, lv, W, oa, pl->neta, aclamp, R);
                if (any & 2) op_row_rail(opr, b, tg, W, od, pl->rail);
            }
            LAP(RH_SPLIT);
            continue;
        }
        gather_bw_row(b, tg, lv, opr, dc, B->fl, B->cq, B->nrow, W, oa, od, pl->neta, pl->mt, pl->model, R);
        LAP(RH_GATHER_A);
        uint16_t rb = 0;
        int nl = 0;
        for (int x = 0; x < W; x++) { B->xl[nl] = (uint16_t)x; nl += B->fl[x] >> 1; }
        uint64_t nm = 0;
        for (int j = 0; j < nl; j++) {
            int x = B->xl[j];
            if (B->fl[x] & 1) {
                int k = B->cq[x];
                uint8_t ow = J.ctab[k].own;
                B->nrow[x] = J.ctab[k].net;
                b[x] = ow ? ow : b[x];
                B->mcol[x] |= J.ctab[k].bit;
                rb |= J.ctab[k].bit;
                nm += ow != 0;
                continue;
            }
            uint8_t w = b[x];
            int i = tg[x] << 5 | lv[x], n = (w == oa ? pl->neta : pl->netd)[i];
            if (pl->mt && w == oa && n && !pl->model && dc[x] && (dc[x] < 0) != (n < 0)) {
                int d = dc[x];
                d = d < -EPD_MT_D ? -EPD_MT_D : d > EPD_MT_D ? EPD_MT_D : d;
                int c = pl->best[i][d + EPD_MT_D];
                if (pl->cap && c < pl->cs - pl->cap) c = pl->cs - pl->cap;
                if (c != pl->cs && pl->mown[c]) {
                    b[x] = pl->mown[c];
                    n = pl->mnet[c][i];
                    B->mcol[x] |= (uint16_t)(1u << c);
                    rb |= (uint16_t)(1u << c);
                    B->matched++;
                }
            }
            int dd = dc[x] + n;
            dc[x] = (int16_t)(dd < -30000 ? -30000 : dd > 30000 ? 30000 : dd);
            if (opm) opr[x] = pl->rail[tg[x]];
        }
        B->matched += nm;
        LAP(RH_GATHER_B);
        gather_bw_apply(dc, opr, B->fl, tg, B->nrow, W, opm, aclamp, R);
        for (int c = 0; rb && c < EPD_LUT_CUTS; c++)
            if (rb & (1u << c)) { if (B->mb1[c] == INT32_MAX) B->mb1[c] = y; B->mb3[c] = y; }
        LAP(RH_GATHER_C);
    }
#undef LAP
}
static void job_band_prof(int i, int n) { (void)n; band_rows_prof(&bands[i], wy0, wy1); }

/* ---- the hook ------------------------------------------------------------ */
static struct {
    unsigned long claims, lazy, bad, px;
    double prod_s, band_s, prod_max, band_max;
} C[8];
static int cur_sc;
static uint8_t *snap_b[6];          /* busy, target, level, op, stale; dc (2 B) */
static uint8_t *res_b[6];

static void rect_copy(epd_engine *e, epd_rect r, uint8_t **to, int save)
{
    uint8_t *arr[5] = { e->busy, e->target, e->level, e->op, e->stale };
    for (int y = 0; y < r.h; y++) {
        size_t o = (size_t)(r.y + y) * EPD_W + r.x, d = (size_t)y * r.w;
        for (int k = 0; k < 5; k++) {
            if (save) memcpy(to[k] + d, arr[k] + o, (size_t)r.w); else memcpy(arr[k] + o, to[k] + d, (size_t)r.w);
        }
        if (save) memcpy(to[5] + 2 * d, (uint8_t *)(e->dc + o), (size_t)r.w * 2);
        else memcpy((uint8_t *)(e->dc + o), to[5] + 2 * d, (size_t)r.w * 2);
    }
}

static int hook_pixels(void *ctx, epd_engine *e, epd_claim_plan *pl, epd_claim_out *out)
{
    (void)ctx;
    if (!pl->tg) pl->tg = pend_tg(e, pl->pend);
    epd_rect r = pl->r;
    C[cur_sc].claims++;
    C[cur_sc].px += (unsigned long)r.w * r.h;
    if (pl->lazy || epd_tile_stat_on || (g_kern && !epd_claim_bandable(pl))) { C[cur_sc].lazy++; epd_claim_pixels(e, pl, out); return 0; }
    static epd_claim_plan p2;
    p2 = *pl;
    epd_claim_out o2;
    rect_copy(e, r, snap_b, 1);
    int band_first = C[cur_sc].claims & 1;
    double tp = 0, tb = 0;
    for (int pass = 0; pass < 2; pass++) {
        int band = pass == 0 ? band_first : !band_first;
        if (pass) rect_copy(e, r, snap_b, 0);
        double t0 = tnow();
        if (band) { if (g_prod) claim_prod(e, &p2, &o2); else if (g_kern) claim_packed(e, &p2, &o2); else claim_banded(e, &p2, &o2); }
        else if (g_kern || g_prod) claim_pixels_soa(e, pl, out);
        else epd_claim_pixels(e, pl, out);
        double dt = tnow() - t0;
        if (band) tb = dt; else tp = dt;
        if (pass == 0) rect_copy(e, r, res_b, 1);       /* the first pass's result */
    }
    /* compare: the arrays now hold the second pass's result, res_b the first's */
    rect_copy(e, r, snap_b, 1);                         /* (snap_b: the second's) */
    size_t n = (size_t)r.w * r.h;
    int bad = 0;
    for (int k = 0; k < 5; k++) bad |= memcmp(snap_b[k], res_b[k], n) != 0;
    bad |= memcmp(snap_b[5], res_b[5], 2 * n) != 0;
    bad |= memcmp(out, &o2, sizeof o2) != 0;
    if (bad) {
        if (C[cur_sc].bad++ < 3) {
            fprintf(stderr, "claim %lu differs (rect %d,%d %dx%d mt %d split %d hown %d npre %d fm %d):", C[cur_sc].claims, r.x, r.y, r.w, r.h,
                    pl->mt, pl->split, pl->hown, pl->npre, pl->fm);
            for (int k = 0; k < 6; k++) fprintf(stderr, " %d", k < 5 ? memcmp(snap_b[k], res_b[k], n) != 0 : memcmp(snap_b[5], res_b[5], 2 * n) != 0);
            fprintf(stderr, " out %d\n", memcmp(out, &o2, sizeof o2) != 0);
        }
        /* keep the production result */
        if (!band_first) { rect_copy(e, r, res_b, 0); }
        else { rect_copy(e, r, snap_b, 0); }
    }
    if (g_kern) tax(e, r);
    C[cur_sc].prod_s += tp; C[cur_sc].band_s += tb;
    if (tp > C[cur_sc].prod_max) C[cur_sc].prod_max = tp;
    if (tb > C[cur_sc].band_max) C[cur_sc].band_max = tb;
    return 0;
}

/* =1 : the engine in claim_state=check
 * (the arrays + a D/W/T mirror, both finished by epd_advance): per bandable
 * claim the arrays' pass (claim_pixels_soa) vs production's claim on D/W/T
 * in place (claim_pixels_pk: W written directly, THREADS bands on this
 * pool), every field compared over the rect (pk_rect_diff) and the report;
 * the whole panel once more at the scenario's end. The order alternates
 * per claim (cache warmth). Not bandable (lazy, npre > 4): the engine's own
 * check (counted) */
static unsigned long s8_whole_bad;
/* S8PROF=1: the in-place claim of one band, timed by part (prep = w_begin +
 * epd_claim_prep, init = pk_band_init, sweep = the row kernels, rest =
 * pk_claim_row's other work (pass B, the split's gather), merge); the
 * timers' own cost is left in */
enum { S8P_PREP, S8P_INIT, S8P_SWEEP, S8P_REST, S8P_MERGE, S8P_N };
static const char *const s8p_name[S8P_N] = { "prep", "init", "sweep", "rest (pass B / split gather)", "merge" };
static double s8p[S8P_N];
static unsigned long s8p_rows;
static int (*s8_inner)(PK_ROW_ARGS);
static int s8_timed(PK_ROW_ARGS)
{
    double t = tnow();
    int r = s8_inner(PK_ROW_PASS, c);
    s8p[S8P_SWEEP] += tnow() - t;
    return r;
}
static void s8_prof_pk(epd_engine *e, epd_claim_plan *pl, epd_claim_out *out)
{
    epd_pk *k = e->pk;
    double t0 = tnow();
    if (!pl->tg) pl->tg = pend_tg(e, pl->pend);
    uint16_t *w0 = k->W + (size_t)pl->r.y * EPD_PK_S + pl->r.x;
    epd_pk_w_begin(e);
    epd_claim_prep(e, pl, &cl_j);
    double t1 = tnow();
    s8_inner = pk_band_init(pl, &cl_j, &cl_b[0]);
    double t2 = tnow();
    double sw = s8p[S8P_SWEEP];
    for (int y = 0; y < pl->r.h; y++) {
        size_t q = (size_t)(pl->r.y + y) * EPD_PK_S + pl->r.x;
        pk_claim_row(pl, &cl_j, &cl_b[0], s8_timed, y, k->D + q, w0 + (size_t)y * EPD_PK_S, k->T + q, k);
    }
    double t3 = tnow();
    bands_merge(e, pl, cl_b, 1, 1, out);
    double t4 = tnow();
    s8p[S8P_PREP] += t1 - t0; s8p[S8P_INIT] += t2 - t1;
    s8p[S8P_REST] += (t3 - t2) - (s8p[S8P_SWEEP] - sw); s8p[S8P_MERGE] += t4 - t3;
    s8p_rows += (unsigned long)pl->r.h;
}

static int hook_s8(void *ctx, epd_engine *e, epd_claim_plan *pl, epd_claim_out *out)
{
    (void)ctx;
    if (!pl->tg) pl->tg = pend_tg(e, pl->pend);
    epd_rect r = pl->r;
    C[cur_sc].claims++;
    C[cur_sc].px += (unsigned long)r.w * r.h;
    if (!epd_claim_bandable(pl)) {
        unsigned long d0 = epd_claim_state_diffs;
        C[cur_sc].lazy++;
        claim_pixels_cst(e, pl, out);
        if (epd_claim_state_diffs != d0) C[cur_sc].bad++;
        return 0;
    }
    static epd_claim_plan p2;
    p2 = *pl;
    epd_claim_out o2;
    int pk_first = C[cur_sc].claims & 1;
    double tp = 0, tb = 0;
    for (int pass = 0; pass < 2; pass++) {
        int pk = pass == 0 ? pk_first : !pk_first;
        double t0 = tnow();
        if (pk && g_s8prof) s8_prof_pk(e, &p2, &o2);
        else if (pk) { epd_claim_bands = nthr; claim_pixels_pk(e, &p2, &o2); epd_claim_bands = 1; }
        else claim_pixels_soa(e, pl, out);
        double dt = tnow() - t0;
        if (pk) tb = dt; else tp = dt;
    }
    const char *what = "report";
    long i = pk_rect_diff(e, r, &what);
    if (i >= 0 || memcmp(out, &o2, sizeof o2)) {
        if (C[cur_sc].bad++ < 3)
            fprintf(stderr, "claim %lu differs (%s at %ld,%ld; rect %d,%d %dx%d mt %d split %d hown %d npre %d fm %d)\n", C[cur_sc].claims,
                    what, i < 0 ? -1 : i % EPD_W, i < 0 ? -1 : i / EPD_W, r.x, r.y, r.w, r.h, pl->mt, pl->split, pl->hown, pl->npre, pl->fm);
    }
    C[cur_sc].prod_s += tp; C[cur_sc].band_s += tb;
    if (tp > C[cur_sc].prod_max) C[cur_sc].prod_max = tp;
    if (tb > C[cur_sc].band_max) C[cur_sc].band_max = tb;
    return 0;
}

/* KERNEL=packed: every finishing run's rows, finish_row vs the packed
 * finish, over a copy (the arrays' result kept); 0 = done */
static unsigned long fin_runs, fin_bad;
static int hook_finish(void *ctx, epd_engine *e, uint64_t fin)
{
    (void)ctx;
    for (int s = 0; s < EPD_MAX_UPD; s++) {
        if (!(fin >> s & 1)) continue;
        const epd_run *r = &e->run[s];
        if (r->r.w <= 0 || r->r.h <= 0) continue;
        fin_runs++;
        rect_copy(e, r->r, snap_b, 1);
        epd_finish_rows_packed(e, r, s + 1, 0, EPD_H);
        rect_copy(e, r->r, res_b, 1);
        rect_copy(e, r->r, snap_b, 0);
        for (int y = r->r.y; y < r->r.y + r->r.h; y++) {
            size_t o = (size_t)y * EPD_W + r->r.x;
            finish_row(e->busy + o, e->level + o, e->target + o, r->r.w, (uint8_t)(s + 1));
        }
        rect_copy(e, r->r, snap_b, 1);
        size_t n = (size_t)r->r.w * r->r.h;
        int bad = 0;
        for (int k = 0; k < 5; k++) bad |= memcmp(snap_b[k], res_b[k], n) != 0;
        bad |= memcmp(snap_b[5], res_b[5], 2 * n) != 0;
        if (bad && fin_bad++ < 3) fprintf(stderr, "finish of run %d differs (rect %d,%d %dx%d)\n", s, r->r.x, r->r.y, r->r.w, r->r.h);
    }
    return 0;
}

/* PROF=1: the production pass alone, its helpers timed (one thread) */
static int hook_prof(void *ctx, epd_engine *e, epd_claim_plan *pl, epd_claim_out *out)
{
    (void)ctx;
    if (!pl->tg) pl->tg = pend_tg(e, pl->pend);
    C[cur_sc].claims++;
    C[cur_sc].px += (unsigned long)pl->r.w * pl->r.h;
    if (pl->lazy) { C[cur_sc].lazy++; epd_claim_pixels(e, pl, out); return 0; }
    /* the banded code with one band and timed helpers: the same writes as
     * production (checked by the default mode); its report via claim_banded's
     * epilogue would double the prologue, so: production's report, banded's rows */
    double t0 = tnow();
    J.e = e; J.pl = pl;
    /* (prologue as claim_banded) */
    if (pl->mt)
        for (int k = 0; k < 4; k++) {
            int i = ((k & 2) ? 30 : 0) << 5 | ((k & 1) ? 30 : 0);
            for (int q = 0; q < 64; q++) {
                int c = q < 2 * EPD_MT_D + 1 ? pl->best[i][q] : pl->cs;
                if (pl->model && c > pl->cs) c = pl->cs;
                if (pl->cap && c < pl->cs - pl->cap) c = pl->cs - pl->cap;
                int hit = c != pl->cs && pl->mown[c];
                J.ctab[k << 6 | q].own = hit ? pl->mown[c] : 0;
                J.ctab[k << 6 | q].net = hit ? pl->mnet[c][i] : pl->neta[i];
                J.ctab[k << 6 | q].bit = (uint16_t)(hit ? 1u << c : 0);
            }
        }
    memset(J.pob, 255, sizeof J.pob);
    memset(J.prn, 0, sizeof J.prn);
    J.npo = 0;
    if (pl->npre && pl->npre <= 4)
        for (int v = 1; v < 256 && J.npo < pl->npre; v++)
            if (pl->pre[v]) { J.pob[J.npo] = (uint8_t)v; memcpy(J.prn[J.npo], pl->rn[pl->pre[v]], 4); J.npo++; }
    epd_rect r = pl->r;
    rect_copy(e, r, snap_b, 1);
    wy0 = 0; wy1 = r.h;
    job_band_prof(0, 1);
    rect_copy(e, r, snap_b, 0);                         /* (undone: production below makes the state) */
    double t1 = tnow();
    epd_claim_pixels(e, pl, out);
    C[cur_sc].prod_s += tnow() - t1;
    C[cur_sc].band_s += t1 - t0;
    return 0;
}

static void query(void *ctx, epd_engine *e, const epd_pend *p, uint64_t *mask, int *nonbw)
{
    (void)ctx;
    epd_pend_query(e, p, mask, nonbw);
}

/* ---- scenarios (the engine's daily payback config, 24 C) ------------------ */
static epd_engine E;
#define PAGE_H 20000
static uint8_t *page, *pagebw;

static void make_page(void)
{
    page = malloc((size_t)EPD_W * PAGE_H);
    pagebw = malloc((size_t)EPD_W * PAGE_H);
    memset(page, 255, (size_t)EPD_W * PAGE_H);
    srand(5);
    for (int line = 0; line < PAGE_H / 40; line++)      /* text lines: glyph boxes, antialiased */
        for (int x = 40; x < EPD_W - 40; x += 14 + rand() % 6) {
            if (rand() % 7 == 0) { x += 20; continue; }
            int h = 14 + rand() % 6, y0 = line * 40 + 10;
            for (int y = y0; y < y0 + h && y < PAGE_H; y++)
                for (int k = 0; k < 9; k++) {
                    int v = rand() % 4;
                    page[(size_t)y * EPD_W + x + k] = v == 0 ? 0 : v == 1 ? 96 : v == 2 ? 192 : 255;
                }
        }
    for (int y = 0; y < PAGE_H; y++)                    /* every 25th band: a grey picture */
        if ((y / 700) % 5 == 3)
            for (int x = 200; x < EPD_W - 200; x++) page[(size_t)y * EPD_W + x] = (uint8_t)((x * 3 + y * 5) & 255);
    for (size_t i = 0; i < (size_t)EPD_W * PAGE_H; i++) pagebw[i] = page[i] < 144 ? 0 : 255;
}

static void payback(void)
{
    epd_set_matched(&E, 1);
    epd_set_matched_rule(&E, 2, 3);
    epd_set_nonblock(&E, 2);
    epd_set_model(&E, 2, 0);
    epd_set_heal(&E, 2, 2);
    epd_set_snap(&E, 1);
}

static void run(const char *sc, int frames)
{
    static const int cuts[] = { 8, 6, 4, 2 };
    int scroll = !strcmp(sc, "scroll") || !strcmp(sc, "bwscroll"), window = !strcmp(sc, "window");
    int pagec = !strcmp(sc, "page"), pen = !strcmp(sc, "pen");
    const uint8_t *pg = !strcmp(sc, "bwscroll") ? pagebw : page;
    epd_rect win = window ? (epd_rect){ 234, 202, 1404, 1000 } : (epd_rect){ 0, 60, EPD_W, 1300 };
    int off = 0, nsub = 0;
    double ts = 0;
    unsigned long nf = 0;
    static uint8_t canvas[EPD_W * EPD_H];
    memset(canvas, 255, sizeof canvas);
    for (int f = 0; f < frames; f++) {
        if (scroll || window) {
            if (epd_frames_until_free(&E, win) <= 1 && E.npend == 0) {
                off = (off + 36) % (PAGE_H - win.h);
                const uint8_t *g = pg + (size_t)off * EPD_W - (size_t)win.y * EPD_W;
                epd_rect r = win;
                uint8_t *t = epd_target_make(&r, g, EPD_W);
                if (t) epd_submit_target(&E, r, EPD_MODE_A2LEN(EPD_A2_OR_DU, cuts[nsub % 4]), 0, t, g, EPD_W);
                nsub++;
            }
        } else if (pagec) {
            if (!E.nrun && !E.npend) {                  /* a page turn: the next page, the whole panel */
                off = (off + 1404) % (PAGE_H - EPD_H);
                const uint8_t *g = page + (size_t)off * EPD_W;
                epd_rect r = { 0, 0, EPD_W, EPD_H };
                uint8_t *t = epd_target_make(&r, g, EPD_W);
                if (t) epd_submit_target(&E, r, EPD_GC16, nsub % 4 == 3 ? 1 : 0, t, g, EPD_W);
                nsub++;
            }
        } else if (pen && f % 3 == 0) {                 /* 4 touching pieces along strokes */
            for (int k = 0; k < 4; k++) {
                int s = nsub++, stroke = s / 60, i = s % 60;
                int x = 100 + (stroke * 137) % 1500 + i * 20, y = 100 + (stroke * 211) % 1100 + (i * 13) % 200;
                if (x + 64 > EPD_W) x = EPD_W - 64;
                if (y + 64 > EPD_H) y = EPD_H - 64;
                for (int yy = y + 28; yy < y + 36; yy++) for (int xx = x; xx < x + 64; xx++) canvas[(size_t)yy * EPD_W + xx] = (stroke & 1) ? 0 : 128;
                epd_rect r = { x, y, 64, 64 };
                const uint8_t *g = canvas;
                uint8_t *t = epd_target_make(&r, g, EPD_W);
                if (t) epd_submit_target(&E, r, EPD_MODE_A2LEN(EPD_A2_OR_DU, 0), 0, t, g, EPD_W);
            }
        }
        double t0 = tnow();
        epd_schedule(&E);
        ts += tnow() - t0;
        epd_advance(&E, NULL, NULL);
        nf++;
    }
    (void)ts; (void)nf;
}

static int parse_cpus(const char *s)
{
    int n = 0;
    while (*s && n < 16) {
        char *e;
        long a = strtol(s, &e, 10), b = a;
        if (e == s) break;
        if (*e == '-') { s = e + 1; b = strtol(s, &e, 10); }
        for (long c = a; c <= b && n < 16; c++) cpus[n++] = (int)c;
        s = *e == ',' ? e + 1 : e;
    }
    return n;
}

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s <wbf> [-t THREADS] [-c CPULIST] [-s scroll|window|bwscroll|page|pen|all] [-n frames]\n", argv[0]); return 2; }
    const char *scen = "all";
    int frames = 1200;
    for (int i = 2; i < argc; i++) {
        if (!strcmp(argv[i], "-t") && i + 1 < argc) nthr = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-c") && i + 1 < argc) ncpu = parse_cpus(argv[++i]);
        else if (!strcmp(argv[i], "-s") && i + 1 < argc) scen = argv[++i];
        else if (!strcmp(argv[i], "-n") && i + 1 < argc) frames = atoi(argv[++i]);
    }
    if (nthr < 1) nthr = 1;
    if (nthr > 16) nthr = 16;
    g_prof = getenv("PROF") != NULL;
    g_kern = getenv("KERNEL") && !strcmp(getenv("KERNEL"), "packed");
    g_prod = getenv("PROD") != NULL;
    g_s8 = getenv("S8") != NULL;
    g_s8prof = g_s8 && getenv("S8PROF") != NULL;
    if (g_prod || g_s8) g_kern = 0;
    if (g_s8) { g_prod = 0; g_prof = 0; setenv("GLASSINE_CLAIM_STATE", "check", 1); }
    epd_claim_bands = 1;                /* (PROD: per claim, the band side only) */
    memset(ones_b, 1, sizeof ones_b);
    pin(ncpu ? cpus[0] : -1);
    for (int i = 1; i < nthr; i++) pthread_create(&wth[i], NULL, worker, (void *)(long)i);
    for (int k = 0; k < 6; k++) { snap_b[k] = malloc((size_t)EPD_W * EPD_H * 2); res_b[k] = malloc((size_t)EPD_W * EPD_H * 2); }
    FILE *f = fopen(argv[1], "rb");
    if (!f) { perror(argv[1]); return 2; }
    fseek(f, 0, SEEK_END);
    size_t wn = (size_t)ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *wbf = malloc(wn);
    if (fread(wbf, 1, wn, f) != wn) return 2;
    fclose(f);
    make_page();
    /* the pool's dispatch alone (pool_run of an empty job) */
    double pd = 0;
    if (nthr > 1) {
        for (int i = 0; i < 200; i++) { double t0 = tnow(); pool_run(job_empty, nthr); pd += tnow() - t0; }
        pd /= 200;
    }
    printf("epd-claim-bench: %d thread%s, cpus", nthr, nthr > 1 ? "s" : "");
    for (int i = 0; i < ncpu; i++) printf(" %d", cpus[i]);
    if (!ncpu) printf(" (unpinned)");
    if (nthr == 1) wcpu[0] = sched_getcpu();
    printf("; ran on");
    for (int i = 0; i < nthr; i++) printf(" %d", wcpu[i]);
    printf("; pool dispatch %.3f ms%s\n", pd * 1e3, g_prof ? "; PROF: helper times (one thread)" : "");
    static const char *const all[] = { "scroll", "bwscroll", "window", "page", "pen" };
    int bad = 0;
    for (int k = 0; k < 5; k++) {
        if (strcmp(scen, "all") && strcmp(scen, all[k])) continue;
        cur_sc = k;
        memset(&E, 0, sizeof E);
        if (epd_init(&E, wbf, wn, 24)) { fprintf(stderr, "wbf\n"); return 1; }
        epd_assume(&E, 30);
        payback();
        static epd_pixel_ops ops;
        ops = (epd_pixel_ops){ .pixels = g_s8 ? hook_s8 : g_prof ? hook_prof : hook_pixels, .query = query,
                               .finish = g_kern && !g_prof ? hook_finish : NULL };
        epd_set_pixel_ops(&E, &ops);
        if (g_prod || g_s8) epd_set_band_ops(&E, &bench_bops);
        if (g_s8 && E.cst != 2) { fprintf(stderr, "S8: the engine is not in claim_state=check\n"); return 1; }
        memset(rh_s, 0, sizeof rh_s);
        ser_s = 0; tax_s = 0; fin_runs = fin_bad = 0;
        memset(s8p, 0, sizeof s8p); s8p_rows = 0;
#ifdef EPD_PK_PROF
        epd_pk_prof_mt = epd_pk_prof_tail = 0;
#endif
        memset(&epd_sched_t, 0, sizeof epd_sched_t);
        epd_sched_prof = 1;
        run(all[k], frames);
        epd_sched_prof = 0;
        typeof(C[0]) *c = &C[k];
        if (g_s8) {                                     /* (the whole panel: the finishes too) */
            const char *what = "";
            long i = pk_rect_diff(&E, (epd_rect){ 0, 0, EPD_W, EPD_H }, &what);
            if (i >= 0) { s8_whole_bad++; fprintf(stderr, "%s: the panel differs at the end (%s at %ld,%ld)\n", all[k], what, i % EPD_W, i / EPD_W); }
        }
        unsigned long nb = c->claims - c->lazy;
        if (g_prof) {
            double tot = 0;
            for (int h = 0; h < RH_N; h++) tot += rh_s[h];
            printf("%-9s %lu claims (%lu lazy), %.0f kpx avg; production pass %.3f ms avg; helpers (timed, sum %.3f ms):", all[k],
                   c->claims, c->lazy, c->claims ? c->px / 1e3 / c->claims : 0, c->claims ? c->prod_s * 1e3 / c->claims : 0,
                   nb ? tot * 1e3 / nb : 0);
            for (int h = 0; h < RH_N; h++) if (rh_s[h] > 0) printf(" %s %.0f%%", rh_name[h], tot > 0 ? 100 * rh_s[h] / tot : 0);
            printf("\n");
        } else {
            printf("%-9s %lu claims (%lu lazy: serial), %.0f kpx avg; %s %.3f ms avg (max %.2f), %s x%d %.3f ms (max %.2f) = %.2fx; "
                   "serial in banded %.3f ms; %lu differ\n", all[k], c->claims, c->lazy, c->claims ? c->px / 1e3 / c->claims : 0,
                   g_kern || g_prod || g_s8 ? "soa" : "production", nb ? c->prod_s * 1e3 / nb : 0, c->prod_max * 1e3,
                   g_s8 ? "D/W/T in place" : g_prod ? "production banded" : g_kern ? "packed" : "banded", nthr,
                   nb ? c->band_s * 1e3 / nb : 0, c->band_max * 1e3,
                   c->band_s > 0 ? c->prod_s / c->band_s : 0, nb ? ser_s * 1e3 / nb : 0, c->bad);
            if (g_s8prof && nb) {
                double tot = 0;
                for (int h = 0; h < S8P_N; h++) tot += s8p[h];
                printf("          S8PROF (one band, timers in): %.1f rows per claim;", (double)s8p_rows / nb);
                for (int h = 0; h < S8P_N; h++) printf(" %s %.4f ms (%.0f%%)", s8p_name[h], s8p[h] * 1e3 / nb, tot > 0 ? 100 * s8p[h] / tot : 0);
#ifdef EPD_PK_PROF
                printf("; in the sweep: matched-cut pass %.4f ms, scalar tail %.4f ms, so the 16-px steps %.4f ms",
                       epd_pk_prof_mt * 1e3 / nb, epd_pk_prof_tail * 1e3 / nb, (s8p[S8P_SWEEP] - epd_pk_prof_mt - epd_pk_prof_tail) * 1e3 / nb);
#else
                printf("; (build with -DEPD_PK_PROF for the sweep's split)");
#endif
                printf("\n");
            }
            if (g_kern)
                /* (the tax is timed on one thread over the whole rect: "without it" only means something at -t 1) */
                printf("          packed: the S1 boundary (pack + unpack, one thread) %.3f ms per claim, the pass without it ~%.3f ms%s; finish: %lu runs, %lu differ\n",
                       nb ? tax_s * 1e3 / nb : 0, nb ? (c->band_s - (nthr > 1 ? tax_s / nthr : tax_s)) * 1e3 / nb : 0,
                       nthr > 1 ? " (tax / threads, est.)" : "", fin_runs, fin_bad);
        }
        /* epd_schedule's other parts (per claim; pix includes this bench's double pass) */
        printf("          per claim: plan %.3f ms, start %.3f ms, blocked %.3f ms; advance finish %.3f ms per frame (%llu kpx finished)\n",
               c->claims ? epd_sched_t.plan * 1e3 / c->claims : 0, c->claims ? epd_sched_t.start * 1e3 / c->claims : 0,
               c->claims ? epd_sched_t.blocked * 1e3 / c->claims : 0, epd_sched_t.finish * 1e3 / frames,
               (unsigned long long)(epd_sched_t.fin_px / 1000));
        bad += c->bad + (int)fin_bad + (int)s8_whole_bad;
        s8_whole_bad = 0;
        epd_set_pixel_ops(&E, NULL);
        epd_free(&E);
    }
    printf(bad ? "epd-claim-bench: FAIL (%d claims differ)\n" : g_s8 ? "epd-claim-bench: ok (S8a: D/W/T in place == soa, bit for bit)\n"
           : g_prod ? "epd-claim-bench: ok (production banded == soa, bit for bit)\n"
           : g_kern ? "epd-claim-bench: ok (packed == soa, bit for bit)\n"
                                                                       : "epd-claim-bench: ok (banded == production, bit for bit)\n", bad);
    return bad != 0;
}
