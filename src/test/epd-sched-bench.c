/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-sched-bench — where the drive thread's CPU goes in a scroll
 * (host replay, no device). The real engine (epd-engine.c) at 24 C, a
 * Firefox-sized window scrolled: each update is the page shifted by `step`
 * rows (mostly white, black text, antialiased grey edges), A2_OR_DU cut to 6
 * frames as motion=adaptive does. Per drive frame the stream thread's CPU work
 * (epd_schedule + epd_advance) is timed, and the main thread's epd_submit.
 * epd-sched-bench <wbf> [mode] mode: paced (the next update when the
 * engine is free there, pace=engine) | flood (one every 4 frames, the pile-up
 * of the run: waits order 14046, pend 19); payback: glassine-epdd-desktop's
 * defaults since (matched from 2 cap 3, preempt, panel model 2,
 * heal 2 from 2, snap) and the cut cycling A2/8, /6, /4, /2 as a fling does
 * (what payback costs the hand-off and the stream thread)
 * a 4th arg `bw` = the page dithered to
 * black/white (what a motion run's A2 steps get on the device: the default
 * page's antialiased greys make ~70 % of the claimed pixels DU ones); `mix` = a grey page (gradients, grey
 * blocks) and the modes cycled (A2_OR_DU cut, plain A2 cut and uncut, DU,
 * partial GC16, a flash now and then): the claim pass's fallbacks (non-bw
 * pixels, matched cuts, preempts) covered. HASH=1: every drive frame is
 * rendered and hashed (FNV-1a 64 over the words), the engine state (busy,
 * target, level, dc, op, stale, the runs, the counters) every 8th frame and
 * at the end: one line "hash <16 hex>" that an optimization of the pass must
 * not change (byte-identical); HASHV=<file>: the running hash per frame. Built with -DEPD_CLAIM_PROF the pass is timed
 * (epd_claim_pixels per frame and per claim). SCHEDPROF=1: epd_sched_prof on
 * ('s timers; the hash must not change).
 * GLASSINE_CLAIM_BANDS=N bands the claims (the
 * hash must not change); THREADS=N runs the bands on N pool threads (else
 * one after the other). --effective 2 (anywhere in the arguments): the
 * effective-frame reference, epd_set_overlap 2 — a big claim decided at
 * frame k goes out as a job and applies at k + 2 (waited for there); with
 * THREADS the job runs on the pool meanwhile (drive_overlap's code path;
 * the hash must equal the reference's without). The state is hashed only
 * while no job is out (its rect is the job's). */
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "../epd-engine.h"

/* the band pool (THREADS): part i of a run on thread i (0 = the caller; a
 * start: all on the threads) */
#define POOL_MAX 8
static pthread_mutex_t pmu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t pgo = PTHREAD_COND_INITIALIZER, pdone = PTHREAD_COND_INITIALIZER;
static unsigned pgen;
static int pleft, pn, pbase, nthreads;
static void (*pfn)(void *, int);
static void *parg;
static void *pworker(void *a)
{
    int t = (int)(long)a;
    pthread_mutex_lock(&pmu);
    unsigned seen = pgen;
    for (;;) {
        while (pgen == seen) pthread_cond_wait(&pgo, &pmu);
        seen = pgen;
        int i = t - pbase;              /* (start: thread t takes part t; run: part t + 1) */
        if (i < 0 || i >= pn) continue;
        pthread_mutex_unlock(&pmu);
        pfn(parg, i);
        pthread_mutex_lock(&pmu);
        if (!--pleft) pthread_cond_broadcast(&pdone);
    }
    return NULL;
}
static void pool_go(void (*fn)(void *, int), void *arg, int n, int caller)
{
    pthread_mutex_lock(&pmu);
    pfn = fn; parg = arg; pn = n; pbase = caller ? -1 : 0;
    pleft = caller ? n - 1 : n;
    pgen++;
    pthread_cond_broadcast(&pgo);
    pthread_mutex_unlock(&pmu);
}
static void pool_wait(void *ctx)
{
    (void)ctx;
    pthread_mutex_lock(&pmu);
    while (pleft) pthread_cond_wait(&pdone, &pmu);
    pthread_mutex_unlock(&pmu);
}
static void pool_run(void *ctx, void (*fn)(void *, int), void *arg, int n)
{
    if (n - 1 > nthreads) { for (int i = 0; i < n; i++) fn(arg, i); return; }
    pool_go(fn, arg, n, 1);
    fn(arg, 0);
    pool_wait(ctx);
}
static void pool_start(void *ctx, void (*fn)(void *, int), void *arg, int n)
{
    (void)ctx;
    if (n > nthreads) { for (int i = 0; i < n; i++) fn(arg, i); return; }
    pool_go(fn, arg, n, 0);
}
static const epd_band_ops pool_ops = { pool_run, pool_start, pool_wait, NULL, NULL };

static epd_engine E;
static uint32_t words[EPD_WORDS];
static uint64_t H = 1469598103934665603ull;
static void hash(const void *p, size_t n)
{
    const uint8_t *b = p;
    uint64_t h = H;
    for (size_t i = 0; i < n; i++) { h ^= b[i]; h *= 1099511628211ull; }
    H = h;
}
/* (word-wise: the frames and state are large) */
static void hash64(const void *p, size_t n)
{
    const uint8_t *b = p;
    uint64_t h = H;
    size_t i = 0;
    for (; i + 8 <= n; i += 8) { uint64_t w; memcpy(&w, b + i, 8); h ^= w; h *= 1099511628211ull; h ^= h >> 29; }
    H = h;
    hash(b + i, n - i);
}
/* (the six fields decoded from either state, so claim_state=packed
 * hashes the same bytes as arrays) */
static uint8_t hb[EPD_NPX], ht[EPD_NPX], hl[EPD_NPX], ho[EPD_NPX], hs[EPD_NPX];
static int16_t hd[EPD_NPX];
static void hash_state(void)
{
    epd_state_get(&E, hl, ht, hb, ho, hd, hs);
    hash64(hb, sizeof hb); hash64(ht, sizeof ht); hash64(hl, sizeof hl);
    hash64(hd, sizeof hd); hash64(ho, sizeof ho); hash64(hs, sizeof hs);
    for (int s = 0; s < EPD_MAX_UPD; s++) {
        const epd_run *u = &E.run[s];
        int v[8] = { u->r.x, u->r.y, u->r.w, u->r.h, u->mode, u->frame, u->nframes, u->payback };
        hash(v, sizeof v);
    }
    uint64_t c[] = { E.preempted, E.snapped_px, E.matched_px, E.healed_px, E.flash_skipped, (uint64_t)E.nrun,
                     (uint64_t)E.npend, E.wait_order, E.wait_pixel, E.wait_slot, E.superseded };
    hash(c, sizeof c);
}
#ifdef EPD_CLAIM_PROF
extern double epd_claim_prof_s;
extern unsigned long epd_claim_prof_n, epd_claim_prof_px, epd_claim_prof_fast;
#endif
static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec / 1e9; }

static uint8_t *slurp(const char *p, size_t *n)
{
    FILE *f = fopen(p, "rb");
    if (!f) { perror(p); exit(2); }
    fseek(f, 0, SEEK_END); *n = (size_t)ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t *d = malloc(*n);
    if (fread(d, 1, *n, f) != *n) exit(2);
    fclose(f);
    return d;
}

#define PAGE_H 20000
static uint8_t *page;                   /* a long page, EPD_W wide */

static void make_page(void)
{
    page = malloc((size_t)EPD_W * PAGE_H);
    memset(page, 255, (size_t)EPD_W * PAGE_H);
    srand(5);
    if (getenv("MIXPAGE"))                              /* (set by mix) a grey page: gradients and grey blocks */
        for (int y = 0; y < PAGE_H; y++)
            for (int x = 0; x < EPD_W; x++) {
                int band = (y / 300) % 4;
                uint8_t v = band == 0 ? (uint8_t)(x * 255 / EPD_W) : band == 1 ? (uint8_t)(((x / 40 + y / 40) & 1) ? 128 : 255)
                          : band == 2 ? (uint8_t)((y * 7 + x) & 255) : 255;
                page[(size_t)y * EPD_W + x] = v;
            }
    for (int line = 0; line < PAGE_H / 40; line++)      /* text lines: glyph boxes */
        for (int x = 40; x < EPD_W - 40; x += 14 + rand() % 6) {
            if (rand() % 7 == 0) { x += 20; continue; }
            int h = 14 + rand() % 6, y0 = line * 40 + 10;
            for (int y = y0; y < y0 + h && y < PAGE_H; y++)
                for (int k = 0; k < 9; k++) {
                    int v = rand() % 4;                 /* black core, grey antialias */
                    page[(size_t)y * EPD_W + x + k] = v == 0 ? 0 : v == 1 ? 96 : v == 2 ? 192 : 255;
                }
        }
    if (getenv("BWPAGE"))                               /* (set by bw) dithered to black/white, as a motion run's frames */
        for (size_t i = 0; i < (size_t)EPD_W * PAGE_H; i++) page[i] = page[i] < 144 ? 0 : 255;
}

int main(int argc, char **argv)
{
    int effective = 0;
    for (int i = 1; i + 1 < argc; i++)                  /* (--effective N: out of the positional arguments) */
        if (!strcmp(argv[i], "--effective")) {
            effective = atoi(argv[i + 1]);
            memmove(argv + i, argv + i + 2, (size_t)(argc - i - 1) * sizeof *argv);
            argc -= 2; i--;
        }
    if (argc < 2) { fprintf(stderr, "usage: %s <wbf> [paced|flood] [payback|-] [mix|bw]\n", argv[0]); return 2; }
    int flood = argc > 2 && !strcmp(argv[2], "flood");
    int payback = argc > 3 && !strcmp(argv[3], "payback");
    int mix = argc > 4 && !strcmp(argv[4], "mix");
    int dohash = getenv("HASH") != NULL;
    FILE *hv = dohash && getenv("HASHV") ? fopen(getenv("HASHV"), "w") : NULL;   /* (the running hash per frame: where it diverges) */
    int bw = argc > 4 && !strcmp(argv[4], "bw");
    if (mix) setenv("MIXPAGE", "1", 1);
    if (bw) setenv("BWPAGE", "1", 1);
    size_t wn;
    uint8_t *wbf = slurp(argv[1], &wn);
    if (epd_init(&E, wbf, wn, 24)) { fprintf(stderr, "wbf\n"); return 1; }
    epd_assume(&E, 30);
    epd_template(words);
    epd_tile_stat_on = getenv("TILESTAT") != NULL;
    epd_sched_prof = getenv("SCHEDPROF") != NULL;
    if (effective && effective != 2) { fprintf(stderr, "--effective: only 2 (the engine's k + 2)\n"); return 2; }
    nthreads = getenv("THREADS") ? atoi(getenv("THREADS")) : 0;
    if (nthreads > POOL_MAX) nthreads = POOL_MAX;
    for (int t = 0; t < nthreads; t++) {
        pthread_t th;
        pthread_create(&th, NULL, pworker, (void *)(long)t);
    }
    if (nthreads > 0) epd_set_band_ops(&E, &pool_ops);
    if (effective) epd_set_overlap(&E, 2);
    make_page();
    /* TBITS=1 (packed): T's bit 5 (the lanes' TOG) set to
     * a pattern before the run; every claim, finish and put must keep it and
     * nothing reads it: the hash is the run's without it, and the bits are
     * checked at the end. TBITS=2: bits 5..7 (ARM and EVICT too): kept, but
     * the hash differs where the engine leaves armed/evicted lanes alone
     * (op_prep's skip) */
    const int tbits = getenv("TBITS") && E.pk;
    const uint32_t tmask = tbits && atoi(getenv("TBITS")) == 2 ? 7 : 1;
    if (tbits)
        for (int y = 0; y < EPD_H; y++)
            for (int x = 0; x < EPD_W; x++) {
                size_t q = (size_t)y * EPD_PK_S + x;
                E.pk->T[q] = (uint8_t)((E.pk->T[q] & 31) | ((((uint32_t)q * 2654435761u) >> 29) & tmask) << 5);
            }
    if (payback) {
        epd_set_matched(&E, 1);
        epd_set_matched_rule(&E, 2, 3);
        epd_set_nonblock(&E, 2);
        epd_set_model(&E, getenv("MODEL") ? atoi(getenv("MODEL")) : 2, 0);   /* (MODEL=0|1: the claim pass's other want rules) */
        epd_set_heal(&E, 2, 2);
        epd_set_snap(&E, 1);
    }
    static const int cuts[] = { 8, 6, 4, 2 };
    epd_rect win = { 0, 60, EPD_W, 1300 };
    int step = 36, off = 0, frames = 85 * 20;          /* 20 s at 85 Hz */
    double t_sched = 0, t_adv = 0, t_sub = 0, sched_max = 0, t_make = 0, sub_max = 0, adv_max = 0;
    long nsub = 0, maxpend = 0;
    int mode = EPD_MODE_A2LEN(EPD_A2_OR_DU, 6);
    for (int f = 0; f < frames; f++) {
        int want = flood ? f % 4 == 0 : epd_frames_until_free(&E, win) <= 1 && E.npend == 0;
        if (want) {
            off = (off + step) % (PAGE_H - win.h);
            if (payback) mode = EPD_MODE_A2LEN(EPD_A2_OR_DU, cuts[nsub % 4]);
            int full = 0;
            if (mix) {                                  /* the fallbacks: other modes, a flash now and then */
                static const int mm[] = { EPD_A2_OR_DU, EPD_A2, EPD_A2_OR_DU, EPD_A2, EPD_DU, EPD_A2_OR_DU, EPD_GC16, EPD_A2 };
                int k = (int)(nsub % 8), c = cuts[(nsub / 8) % 4];
                mode = mm[k] == EPD_A2_OR_DU || mm[k] == EPD_A2 ? EPD_MODE_A2LEN(mm[k], k == 7 ? 0 : c) : mm[k];
                full = nsub % 23 == 22 ? 2 : nsub % 37 == 36 ? 1 : 0;
            }
            const uint8_t *g = page + (size_t)off * EPD_W - (size_t)win.y * EPD_W + win.x;
            epd_rect r = win;
            double t0 = now();
            uint8_t *t = epd_target_make(&r, g, EPD_W);
            double tm = now();
            if (t) epd_submit_target(&E, r, mode, full, t, g, EPD_W);
            double te = now();
            t_make += tm - t0; t_sub += te - tm;
            if (te - tm > sub_max) sub_max = te - tm;
            nsub++;
        }
        double t0 = now();
        epd_schedule(&E);
        double t1 = now(), ta = t1;
        if (dohash) {                           /* the drive frame, as the stream thread renders it (untimed) */
            static int dirty[2];
            epd_render(&E, words, dirty);
            hash64(words, sizeof words);
            if (hv) fprintf(hv, "%d %016llx\n", f, (unsigned long long)H);
            ta = now();
        }
        epd_advance(&E, NULL, NULL);
        double t2 = now();
        t_sched += t1 - t0; t_adv += t2 - ta;
        if (t2 - ta > adv_max) adv_max = t2 - ta;
        if (t1 - t0 > sched_max) sched_max = t1 - t0;
        if (E.npend > maxpend) maxpend = E.npend;
        if (dohash && f % 8 == 7 && !epd_job_out(&E)) hash_state();
    }
    if (epd_job_join(&E) && dohash) hash(&frames, sizeof frames);     /* (one out at the end: applied, marked) */
    if (effective || E.banded)
        printf("S4: %llu banded claims; %llu jobs, late %llu, slipped %llu, joined %llu, queued finishes %llu; apply %.3f ms avg (max %.2f)\n",
               (unsigned long long)E.banded, (unsigned long long)E.jobs, (unsigned long long)E.job_late,
               (unsigned long long)E.job_slip, (unsigned long long)E.job_joins, (unsigned long long)E.job_qfin,
               E.jobs ? E.job_apply_s * 1e3 / (double)E.jobs : 0, E.job_apply_max * 1e3);
    printf("%s%s: %ld updates in %d frames; per frame: schedule %.3f ms (max %.2f), advance %.3f ms (max %.2f); "
           "per update: target copy %.2f ms, submit %.2f ms (max %.2f); waits order %llu pixel %llu slot %llu, peak pend %ld, superseded %llu\n",
           flood ? "flood" : "paced", payback ? " payback" : "", nsub, frames, t_sched * 1e3 / frames, sched_max * 1e3,
           t_adv * 1e3 / frames, adv_max * 1e3, nsub ? t_make * 1e3 / nsub : 0,
           nsub ? t_sub * 1e3 / nsub : 0, sub_max * 1e3, (unsigned long long)E.wait_order, (unsigned long long)E.wait_pixel,
           (unsigned long long)E.wait_slot, maxpend, (unsigned long long)E.superseded);
    if (getenv("GLASSINE_CLAIM_KERNEL"))             /* (the claims the packed pass left to the arrays', and check's differences) */
        printf("claim_kernel %s: %lu claims on the arrays' pass (lazy / npre > 4), %lu differ\n", getenv("GLASSINE_CLAIM_KERNEL"),
               epd_claim_kernel_soa, epd_claim_kernel_diffs);
    if (E.cst)
        printf("claim_state %s: %lu claims on the arrays' pass (lazy / npre > 4), %lu checked, %lu differ\n",
               epd_claim_state_name(&E), epd_claim_kernel_soa, epd_claim_state_checked, epd_claim_state_diffs);
    if (dohash) { hash_state(); printf("hash %016llx\n", (unsigned long long)H); }
    if (tbits) {
        long nb = 0;
        for (int y = 0; y < EPD_H; y++)
            for (int x = 0; x < EPD_W; x++) {
                size_t q = (size_t)y * EPD_PK_S + x;
                nb += (E.pk->T[q] >> 5) != ((((uint32_t)q * 2654435761u) >> 29) & tmask);
            }
        printf("TBITS: T's bits 5..7 %s (%ld of %d pixels changed)\n", nb ? "LOST" : "kept", nb, EPD_W * EPD_H);
        if (nb) return 1;
    }
#ifdef EPD_CLAIM_PROF
    printf("claim pass: %lu claims, %.3f ms per frame, %.3f ms per claim; matched %llu preempted %llu snapped %llu healed %llu",
           epd_claim_prof_n, epd_claim_prof_s * 1e3 / frames, epd_claim_prof_n ? epd_claim_prof_s * 1e3 / epd_claim_prof_n : 0,
           (unsigned long long)E.matched_px, (unsigned long long)E.preempted, (unsigned long long)E.snapped_px,
           (unsigned long long)E.healed_px);
    if (epd_claim_prof_px) printf("; gather px %lu, fast %.1f %%", epd_claim_prof_px, 100.0 * epd_claim_prof_fast / epd_claim_prof_px);
    printf("\n");
#endif
    if (epd_tile_stat_on) {
        const epd_tile_stats *s = &epd_tile_stat;
        printf("tiles: %llu claims, %.1f %% of their pixels change;", (unsigned long long)s->claims,
               s->px ? 100.0 * s->px_chg / s->px : 0);
        static const int ts[EPD_TS_N] = { 16, 32, 64 };
        for (int k = 0; k < EPD_TS_N; k++)
            printf(" %dpx: %.1f %% need the pass (changed %.1f, busy %.1f, flash %.1f, heal %.1f)", ts[k],
                   s->tiles[k] ? 100.0 * s->work[k] / s->tiles[k] : 0, s->tiles[k] ? 100.0 * s->chg[k] / s->tiles[k] : 0,
                   s->tiles[k] ? 100.0 * s->busy[k] / s->tiles[k] : 0, s->tiles[k] ? 100.0 * s->flash[k] / s->tiles[k] : 0,
                   s->tiles[k] ? 100.0 * s->heal[k] / s->tiles[k] : 0);
        printf("\n");
    }
    return 0;
}
