/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-gpe-test — the engine's per-pixel state on the GPU
 * (epd-gpu.c gpe) against the CPU engine. The CPU engine runs as always; its
 * pixel ops (epd_set_pixel_ops) also run the GPU's claim and finish passes on
 * the same plans, then compare what the claim reports (run rects, counters)
 * and the whole state (level, target, busy, model, DC record, stale) after
 * every claim and every frame. Host or device (any GLES 3 GPU, surfaceless
 * EGL).
 *
 * epd-gpe-test <eink_waveform.wbf> [frames per scenario]
 *
 * Scenarios, with glassine-epdd-desktop's payback defaults (matched from 2 cap 3, preempt,
 * panel model 2, heal 2 from 2, snap): a paced scroll and a flood (A2_OR_DU,
 * cuts A2/8, /6, /4, /2), then random updates of every mode (partial, full,
 * flash; GC16, GL16, DU, DU4, A2, A2_OR_DU, cut or not) over a random page.
 * GPE_ASYNC=1: deferred claims (epd_set_defer): the GPU's pass is submitted,
 * the CPU's done at once, the reports compared when the engine collects the
 * GPU's (and the whole state then, as above).
 * GPE_PEN=1: the pen scenario only (pen_stroke: ~29 small DU
 * claims running), its cost per claim on an "epd-gpe: pen" line (with
 * GPE_PROF=1 the fixed cost per pass). On a difference the GPU state is made
 * anew from the CPU's (epd_gpu_gpe_reseed, never an upload into S/D after
 * they were stored into) and the run goes on; GPE_INJECT=n makes one
 * at the n-th compare (the recovery's own test).
 * Exit 0 = equal throughout. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "../epd-engine.h"
#include "../epd-gpu.h"

static epd_engine E;
static epd_gpu *G;
static long claims, lazies, frames, bad;
static long cov[8];                     /* claims with: preempt, heal in flight, heal run, snap, flash, split, matched, model rescale */
static double t_gpu, t_cpu, t_gpu_max;
static int pen;                          /* GPE_PEN: the pen scenario only (pen_stroke) */
static double t_psub, t_pcol, t_q;      /* (pen: the sync claims' submit / collect; every query) */
static uint8_t gl[EPD_W * EPD_H], gt[EPD_W * EPD_H], gb[EPD_W * EPD_H], go[EPD_W * EPD_H], gs[EPD_W * EPD_H];
static int16_t gd[EPD_W * EPD_H];

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

/* the whole state, GPU vs CPU: the first difference of each array.
 * GPE_INJECT=n: the n-th compare finds one made (the CPU's DC record of
 * pixel 0 moved by one): the recovery below, then equal again */
static long compares, inject;
static void compare(const char *when)
{
    epd_gpu_gpe_read(G, gl, gt, gb, go, gd, gs);
    if (inject && ++compares == inject) E.dc[0] += E.dc[0] < 30000 ? 1 : -1;
    struct { const char *n; const uint8_t *g, *c; } a[] = {
        { "level", gl, E.level }, { "target", gt, E.target }, { "busy", gb, E.busy }, { "op", go, E.op }, { "stale", gs, E.stale } };
    int any = 0;
    for (unsigned k = 0; k < sizeof a / sizeof *a; k++) {
        if (!memcmp(a[k].g, a[k].c, sizeof gl)) continue;
        /* target only matters where busy (the CPU leaves old targets, so does the GPU: still compared) */
        size_t i = 0, n = 0;
        while (a[k].g[i] == a[k].c[i]) i++;
        for (size_t j = 0; j < sizeof gl; j++) n += a[k].g[j] != a[k].c[j];
        printf("%s: %s differs at %zu px, first (%zu, %zu): gpu %d cpu %d (level %d target %d busy %d op %d dc %d)\n",
               when, a[k].n, n, i % EPD_W, i / EPD_W, a[k].g[i], a[k].c[i], E.level[i], E.target[i], E.busy[i], E.op[i], E.dc[i]);
        any = 1;
    }
    if (memcmp(gd, E.dc, sizeof gd)) {
        size_t i = 0, n = 0;
        while (gd[i] == E.dc[i]) i++;
        for (size_t j = 0; j < sizeof gl; j++) n += gd[j] != E.dc[j];
        printf("%s: dc differs at %zu px, first (%zu, %zu): gpu %d cpu %d\n", when, n, i % EPD_W, i / EPD_W, gd[i], E.dc[i]);
        any = 1;
    }
    if (any && ++bad > 5) { printf("epd-gpe: too many differences\n"); exit(1); }
    /* go on from equal states: S and D made anew from the CPU's (H2:
     * an upload into them, image-store targets, can be lost on the A509) */
    if (any && epd_gpu_gpe_reseed(G, &E)) { printf("epd-gpe: re-seeding the GPU state failed\n"); exit(1); }
}

static long every = 1;             /* GPE_EVERY */

/* GPE_DEBUG=2: the GPU hang count (dmesg) after each pass, the first passes that hung */
static int hang_count(void)
{
    FILE *f = popen("dmesg 2>/dev/null | grep -c 'hangcheck recover'", "r");
    int n = -1;
    if (f) { if (fscanf(f, "%d", &n) != 1) n = -1; pclose(f); }
    return n;
}
static int hang_last = -1, hang_shown;
static void hang_hook(const char *what)
{
    int n = hang_count();
    if (hang_last >= 0 && n > hang_last && hang_shown++ < 20) printf("gpe pass %s: %d new GPU hang(s)\n", what, n - hang_last);
    else if (hang_shown < 20 && getenv("GPE_PASSES")) printf("gpe pass %s: no hang\n", what);
    hang_last = n;
}

static void cover(const epd_claim_plan *pl)
{
    cov[0] += pl->npre > 0; cov[1] += pl->hact; cov[2] += pl->hown != 0; cov[3] += pl->snp;
    cov[4] += pl->fm == 2; cov[5] += pl->split; cov[6] += pl->mt; cov[7] += pl->op_init == 2;
}

/* the GPU's report (go2) vs the CPU's (out) of claim pl (r: its rect) */
static void report_cmp(const epd_claim_out *go2, const epd_claim_out *out, epd_rect r, const char *what)
{
    if (memcmp(go2, out, sizeof *go2)) {
        printf("claim %ld (rect %d,%d %dx%d%s): report differs\n", claims, r.x, r.y, r.w, r.h, what);
        for (int k = 0; k < EPD_CL_OWNERS; k++)
            if (go2->x0[k] != out->x0[k] || go2->x1[k] != out->x1[k] || go2->y0[k] != out->y0[k] || go2->y1[k] != out->y1[k])
                printf("  owner %d: gpu %d..%d x %d..%d, cpu %d..%d x %d..%d\n", k, go2->x0[k], go2->x1[k], go2->y0[k], go2->y1[k],
                       out->x0[k], out->x1[k], out->y0[k], out->y1[k]);
        printf("  counters gpu pre %llu snap %llu matched %llu healed %llu fskip %llu, cpu %llu %llu %llu %llu %llu\n",
               (unsigned long long)go2->preempted, (unsigned long long)go2->snapped, (unsigned long long)go2->matched,
               (unsigned long long)go2->healed, (unsigned long long)go2->flash_skipped,
               (unsigned long long)out->preempted, (unsigned long long)out->snapped, (unsigned long long)out->matched,
               (unsigned long long)out->healed, (unsigned long long)out->flash_skipped);
        bad++;
    }
    char w[64];
    snprintf(w, sizeof w, "after claim %ld", claims);
    if (claims % every == 0) compare(w);
}

static int px_claim(void *ctx, epd_engine *e, epd_claim_plan *pl, epd_claim_out *out)
{
    (void)ctx;
    epd_claim_out go2;
    cover(pl);
    double t0 = now();
    int lazy = pl->lazy, r;
    if (pen) {                                  /* (the submit and the wait apart) */
        r = epd_gpu_gpe_submit(G, e, pl, 2000);
        double ts = now();
        if (r) epd_gpu_gpe_cancel(G);
        else if ((r = epd_gpu_gpe_collect(G, &go2, 2000))) epd_gpu_gpe_cancel(G);
        t_psub += ts - t0; t_pcol += now() - ts;
    } else
        r = epd_gpu_gpe_claim(G, e, pl, &go2, 2000);
    lazies += lazy;
    double t1 = now();
    epd_claim_pixels(e, pl, out);
    t_cpu += now() - t1;
    claims++;
    if (r) { printf("epd-gpe: the GPU claim failed (%d)\n", r); exit(1); }
    t_gpu += t1 - t0;
    if (t1 - t0 > t_gpu_max) t_gpu_max = t1 - t0;
    char w[64];
    snprintf(w, sizeof w, " oa %d od %d mt %d hown %d npre %d", pl->oa, pl->od, pl->mt, pl->hown, pl->npre);
    report_cmp(&go2, out, pl->r, w);
    return 0;
}

/* GPE_ASYNC: the GPU's pass submitted, the CPU's done now (its report kept
 * for the collect); collect: the GPU's report, compared */
static epd_claim_out cpu_out;
static epd_rect sub_r;
static long submits, collects, polls_early;
static double t_sub, t_col, t_col_max;
static int px_submit(void *ctx, epd_engine *e, epd_claim_plan *pl)
{
    (void)ctx;
    cover(pl);
    double t0 = now();
    int r = epd_gpu_gpe_submit(G, e, pl, 2000);
    double t1 = now();
    if (r) { printf("epd-gpe: the GPU submit failed (%d)\n", r); exit(1); }
    epd_claim_pixels(e, pl, &cpu_out);
    t_cpu += now() - t1;
    t_sub += t1 - t0;
    sub_r = pl->r;
    claims++; submits++;
    return 0;
}

static int px_collect(void *ctx, epd_engine *e, epd_claim_out *out, int wait)
{
    (void)ctx; (void)e;
    double t0 = now();
    int r = epd_gpu_gpe_collect(G, out, wait ? 2000 : 0);
    if (r == 1) { polls_early++; return 1; }
    if (r) { printf("epd-gpe: the GPU collect failed (%d)\n", r); exit(1); }
    double dt = now() - t0;
    t_col += dt;
    if (dt > t_col_max) t_col_max = dt;
    collects++;
    report_cmp(out, &cpu_out, sub_r, wait ? " (collected, waited)" : " (collected)");
    return 0;
}

static int px_finish(void *ctx, epd_engine *e, uint64_t slots)
{
    (void)ctx;
    epd_gpu_gpe_finish(G, e, slots);
    return 1;                                   /* the CPU does them too */
}

static long queries, stale_masks, drive_frames, drive_bad;

static void px_query(void *ctx, epd_engine *e, const epd_pend *p, uint64_t *mask, int *nonbw)
{
    (void)ctx;
    uint64_t gm;
    int gn;
    double t0 = now();
    if (epd_gpu_gpe_query(G, p, &gm, &gn, 2000)) { printf("epd-gpe: the GPU query failed\n"); exit(1); }
    t_q += now() - t0;
    epd_pend_query(e, p, mask, nonbw);
    queries++;
    if (gm != *mask || gn != *nonbw) {
        printf("query %ld (rect %d,%d %dx%d): gpu mask %016llx nonbw %d, cpu %016llx %d\n", queries, p->r.x, p->r.y, p->r.w, p->r.h,
               (unsigned long long)gm, gn, (unsigned long long)*mask, *nonbw);
        bad++;
    }
}

/* GPE_ASYNC: the scenario's claims deferred / settled inside an epd_schedule
 * pass (a second claim there: a wait for the first's report) */
static const char *phase(void)
{
    static char b[64];
    static long d0, w0;
    b[0] = 0;
    if (E.defer) snprintf(b, sizeof b, " (%ld deferred, %ld settled inside a pass)", (long)E.deferred - d0, (long)E.settle_waits - w0);
    d0 = (long)E.deferred; w0 = (long)E.settle_waits;
    return b;
}

static const epd_pixel_ops OPS = { px_claim, px_finish, px_query, NULL, NULL, NULL };
static const epd_pixel_ops OPS_ASYNC = { px_claim, px_finish, px_query, NULL, px_submit, px_collect };

/* every pending's kept mask still what a fresh query finds (epd_pend) */
static void check_masks(void)
{
    for (int i = 0; i < E.npend; i++) {
        const epd_pend *p = &E.pend[i];
        if (!p->bvalid) continue;
        uint64_t m;
        int nb;
        epd_pend_query(&E, p, &m, &nb);
        if (m != p->bmask) {
            printf("frame %ld: pending %d kept mask %016llx, fresh %016llx\n", frames, i, (unsigned long long)p->bmask, (unsigned long long)m);
            if (++stale_masks > 5) exit(1);
        }
    }
}

/* this frame from the engine state on the GPU (epd_gpu_gpe_drive) vs epd_render */
static uint32_t cpu_words[EPD_WORDS], gpu_words[EPD_WORDS];
static int cpu_dirty[2];
static void check_drive(void)
{
    if (!E.nrun) return;
    epd_render(&E, cpu_words, cpu_dirty);
    epd_gpu_render(G, &E, 0);
    epd_gpu_read(G, 0, gpu_words);
    drive_frames++;
    if (memcmp(cpu_words, gpu_words, sizeof cpu_words)) {
        int k = 0;
        while (cpu_words[k] == gpu_words[k]) k++;
        printf("frame %ld: drive frame differs, first word %d: gpu %08x cpu %08x\n", frames, k, gpu_words[k], cpu_words[k]);
        if (++drive_bad > 5) exit(1);
    }
}

#define PAGE_H 20000
static uint8_t *page;

static void make_page(void)
{
    page = malloc((size_t)EPD_W * PAGE_H);
    memset(page, 255, (size_t)EPD_W * PAGE_H);
    srand(5);
    for (int line = 0; line < PAGE_H / 40; line++)
        for (int x = 40; x < EPD_W - 40; x += 14 + rand() % 6) {
            if (rand() % 7 == 0) { x += 20; continue; }
            int h = 14 + rand() % 6, y0 = line * 40 + 10;
            for (int y = y0; y < y0 + h && y < PAGE_H; y++)
                for (int k = 0; k < 9; k++) {
                    int v = rand() % 4;
                    page[(size_t)y * EPD_W + x + k] = v == 0 ? 0 : v == 1 ? 96 : v == 2 ? 192 : 255;
                }
        }
}

static long pen_peak;
static void frame(void)
{
    epd_schedule(&E);
    if (E.nrun > pen_peak) pen_peak = E.nrun;
    if (frames % every == 0) { check_masks(); check_drive(); }
    epd_advance(&E, NULL, NULL);
    frames++;
    if (frames % (50 * every) == 0) { char w[64]; snprintf(w, sizeof w, "frame %ld", frames); compare(w); }
}

static void scroll(int nf, int flood)
{
    static const int cuts[] = { 8, 6, 4, 2 };
    epd_rect win = { 0, 60, EPD_W, 1300 };
    int off = 0, n = 0;
    for (int f = 0; f < nf; f++) {
        int want = flood ? f % 4 == 0 : epd_frames_until_free(&E, win) <= 1 && E.npend == 0;
        if (want) {
            off = (off + 36) % (PAGE_H - win.h);
            epd_submit(&E, win, EPD_MODE_A2LEN(EPD_A2_OR_DU, cuts[n++ % 4]), 0,
                       page + (size_t)off * EPD_W - (size_t)win.y * EPD_W + win.x, EPD_W);
        }
        frame();
    }
}

static void randoms(int nf)
{
    static uint8_t img[EPD_W * EPD_H];
    srand(11);
    static const int modes[] = { EPD_GC16, EPD_GL16, EPD_DU, EPD_DU4, EPD_A2, EPD_A2_OR_DU };
    for (int f = 0; f < nf; f++) {
        if (rand() % 3 == 0) {
            int off = rand() % (PAGE_H - EPD_H);
            memcpy(img, page + (size_t)off * EPD_W, sizeof img);
            for (int k = rand() % 4; k > 0; k--) {          /* some grey blocks */
                int x = rand() % (EPD_W - 200), y = rand() % (EPD_H - 200), v = rand() % 256;
                for (int yy = y; yy < y + 150; yy++) memset(img + (size_t)yy * EPD_W + x, v, 150);
            }
            int w = 16 + rand() % (EPD_W - 16), h = 16 + rand() % (EPD_H - 16);
            epd_rect r = { rand() % (EPD_W - w + 1), rand() % (EPD_H - h + 1), w, h };
            int m = modes[rand() % 6], full = rand() % 8 == 0 ? 1 + rand() % 2 : 0;
            if ((m == EPD_A2 || m == EPD_A2_OR_DU) && rand() % 2) m = EPD_MODE_A2LEN(m, 2 + rand() % 7);
            epd_submit(&E, r, m, full, img, EPD_W);
        }
        frame();
    }
}

/* run slots nearly all taken (small DU updates in the band above the scroll
 * window) while matched scroll steps start: lazy plans (the GPU's probe) */
static void scarce(int rounds)
{
    static uint8_t img[EPD_W * EPD_H];
    static const int cuts[] = { 8, 6, 4 };
    epd_rect win = { 0, 60, EPD_W, 1300 };
    while (!epd_idle(&E)) frame();              /* (the random phase's backlog) */
    for (int k = 0; k < rounds; k++) {
        memcpy(img, page + (size_t)(k * 97 % (PAGE_H - EPD_H)) * EPD_W, sizeof img);
        for (int i = 0; i < 58; i++)                /* 58 x 18x20 DU pieces 12 px apart (> EPD_MERGE_GAP: not merged), grey: long runs */
            for (int y = 20; y < 40; y++) memset(img + (size_t)y * EPD_W + i * 30, (k & 1) ? 96 : 192, 18);
        for (int i = 0; i < 58; i++) epd_submit(&E, (epd_rect){ i * 30, 20, 18, 20 }, EPD_DU, 0, img, EPD_W);
        frame();
        epd_submit(&E, win, EPD_MODE_A2LEN(EPD_A2_OR_DU, cuts[k % 3]), 0, img, EPD_W);
        for (int f = 0; f < 12; f++) frame();
    }
}

/* GPE_PEN=1 (the pen gate): this scenario only. A thick
 * fast pen stroke as the pen path hands it in: 64x64 DU pieces, PEN_P3 every
 * 3 frames, each the box around the ink since the previous piece (discs of
 * radius PEN_R, a piece every PEN_STEP px along a scribble over the panel).
 * Each piece from its own buffer (PEN_BUF copies of the same ink): epd_submit
 * merges touching pending pieces of one buffer, and the probe wants one
 * small claim a piece ('s thick fast stroke: 29 updates running at
 * once). The last third erases along the same path (white over the ink).
 * The payback defaults, as the other scenarios */
static long pen_pieces;
#define PEN_P3 4                        /* pieces every 3 frames (2, 1, 1): with DU's ~22 frames
                                         * at 24 C, ~29 running */
#define PEN_STEP 22
#define PEN_R 9
#define PEN_BUF 3
#define PEN_BOX 64
static uint8_t pbuf[PEN_BUF][EPD_W * EPD_H];

static void pen_disc(int cx, int cy, int r, uint8_t v)
{
    for (int y = cy - r; y <= cy + r; y++)
        for (int x = cx - r; x <= cx + r; x++)
            if (x >= 0 && y >= 0 && x < EPD_W && y < EPD_H && (x - cx) * (x - cx) + (y - cy) * (y - cy) <= r * r)
                for (int b = 0; b < PEN_BUF; b++) pbuf[b][(size_t)y * EPD_W + x] = v;
}

static void pen_stroke(int nf)
{
    for (int b = 0; b < PEN_BUF; b++) memset(pbuf[b], 255, sizeof pbuf[b]);
    /* the scribble in integers (the same path on every machine): steps of
     * (17, 14) = 22 px, a quarter turn every 23 steps, bouncing off a 60 px
     * margin; the erase goes over the same path again */
    int x = 300, y = 300, vx = 17, vy = 14, n = 0, erase = 0;
    for (int f = 0; f < nf; f++) {
        if (!erase && f >= nf * 2 / 3) { erase = 1; x = 300; y = 300; vx = 17; vy = 14; n = 0; }
        for (int k = 0; k < PEN_P3 / 3 + (f % 3 < PEN_P3 % 3); k++) {
            if (++n % 23 == 0) { int t = vx; vx = -vy; vy = t; }
            if (x + vx < 60 || x + vx > EPD_W - 60) vx = -vx;
            if (y + vy < 60 || y + vy > EPD_H - 60) vy = -vy;
            for (int s = 0; s <= 8; s++)                    /* the segment's ink */
                pen_disc(x + vx * s / 8, y + vy * s / 8, erase ? PEN_R + 2 : PEN_R, erase ? 255 : 0);
            int bx = x + vx / 2 - PEN_BOX / 2, by = y + vy / 2 - PEN_BOX / 2;
            epd_submit(&E, (epd_rect){ bx, by, PEN_BOX, PEN_BOX }, EPD_DU, 0, pbuf[pen_pieces++ % PEN_BUF], EPD_W);
            x += vx; y += vy;
        }
        frame();
    }
}

int main(int argc, char **argv)
{
    /* GPE_EVERY=N: the whole-state compare, kept masks and drive frame only
     * every N-th claim / frame (a slow GPU: the device); GPE_PROF=1: per-pass times */
    if (getenv("GPE_EVERY")) every = atoi(getenv("GPE_EVERY")) > 0 ? atoi(getenv("GPE_EVERY")) : 1;
    epd_gpu_prof_on = getenv("GPE_PROF") != NULL;
    epd_gpu_gpe_debug = getenv("GPE_DEBUG") != NULL;
    epd_gpu_gpe_noatom = getenv("GPE_NOATOM") != NULL;
    if (getenv("GPE_DEBUG") && atoi(getenv("GPE_DEBUG")) >= 2) { epd_gpu_gpe_debug_hook = hang_hook; hang_last = hang_count(); }
    if (argc < 2) { fprintf(stderr, "usage: %s <wbf> [frames]\n", argv[0]); return 2; }
    int nf = argc > 2 ? atoi(argv[2]) : 600;
    size_t wn;
    uint8_t *wbf = slurp(argv[1], &wn);
    if (epd_init(&E, wbf, wn, 24)) { fprintf(stderr, "wbf\n"); return 1; }
    epd_assume(&E, 30);
    epd_set_matched(&E, 1);
    epd_set_matched_rule(&E, 2, 3);
    epd_set_nonblock(&E, 2);
    epd_set_model(&E, 2, 0);
    epd_set_heal(&E, 2, 2);
    epd_set_snap(&E, 1);
    make_page();
    if (!(G = epd_gpu_open(&E)) || epd_gpu_gpe_open(G, &E)) { fprintf(stderr, "epd-gpe: no GPU\n"); return 77; }
    if (getenv("GPE_SELFTEST")) return epd_gpu_gpe_selftest(G) ? 1 : 0;
    int async = getenv("GPE_ASYNC") && atoi(getenv("GPE_ASYNC"));
    pen = getenv("GPE_PEN") && atoi(getenv("GPE_PEN"));
    inject = getenv("GPE_INJECT") ? atol(getenv("GPE_INJECT")) : 0;
    epd_set_pixel_ops(&E, async ? &OPS_ASYNC : &OPS);
    epd_set_defer(&E, async);
    epd_gpu_gpe_drive(G, 1);
    memcpy(cpu_words, E.tpl, sizeof cpu_words);
    compare("start");
    if (pen) {
        pen_stroke(nf);
        printf("epd-gpe: pen stroke: %ld claims, %ld frames%s\n", claims, frames, phase());
    } else {
        scroll(nf, 0);
        printf("epd-gpe: paced scroll: %ld claims, %ld frames%s\n", claims, frames, phase());
        scroll(nf, 1);
        printf("epd-gpe: flood: %ld claims, %ld frames%s\n", claims, frames, phase());
        randoms(nf);
        printf("epd-gpe: random updates: %ld claims, %ld frames%s\n", claims, frames, phase());
        scarce(nf / 20);
        printf("epd-gpe: slots scarce: %ld claims (%ld lazy), %ld frames%s\n", claims, lazies, frames, phase());
        E.celsius = 8;                              /* colder: other tables, the model rescaled */
        scroll(nf / 2, 0);
        randoms(nf / 2);
        printf("epd-gpe: at 8 C: %ld claims, %ld frames%s\n", claims, frames, phase());
    }
    while (!epd_idle(&E)) frame();
    epd_claim_settle(&E, 1);
    compare("idle");
    printf("epd-gpe: %ld queries, %ld kept masks stale; %ld drive frames from the GPU state, %ld differ\n",
           queries, stale_masks, drive_frames, drive_bad);
    printf("epd-gpe: claims with preempt %ld, heal in flight %ld, heal run %ld, snap %ld, flash %ld, split %ld, matched %ld, model rescale %ld\n",
           cov[0], cov[1], cov[2], cov[3], cov[4], cov[5], cov[6], cov[7]);
    printf("epd-gpe on %s: %ld claims (%ld lazy: slots from a GPU probe), %ld frames, %ld differences; claim GPU %.2f ms avg (max %.2f, readback incl.), CPU %.2f ms avg\n",
           epd_gpu_renderer(G), claims, lazies, frames, bad, claims - lazies ? t_gpu * 1e3 / (claims - lazies) : 0, t_gpu_max * 1e3,
           claims ? t_cpu * 1e3 / claims : 0);
    printf("epd-gpe: shape %s\n", epd_gpu_gpe_shape(G));
    if (async)
        printf("epd-gpe: async: %ld deferred, %ld collected (%ld inside a pass, %ld polls before done); submit %.2f ms avg, collect %.3f ms avg (max %.2f)\n",
               (long)E.deferred, collects, (long)E.settle_waits, polls_early, submits ? t_sub * 1e3 / submits : 0,
               collects ? t_col * 1e3 / collects : 0, t_col_max * 1e3);
    if (epd_gpu_prof_on) {
        static const char *nm[EPD_GPU_PROF_N] = { "upload", "probe", "claim", "reduce", "wait", "map", "finish", "query" };
        printf("epd-gpe: per pass (ms per claim; finish/query: total s):");
        for (int k = 0; k < EPD_GPU_PROF_N; k++)
            if (k < EPD_GPU_PROF_FIN) printf(" %s %.2f", nm[k], claims ? epd_gpu_prof[k] * 1e3 / claims : 0);
            else printf(" %s %.2f s", nm[k], epd_gpu_prof[k]);
        printf("\n");
    }
    if (pen) {                                  /* the cost of one small claim */
        long nc = claims - lazies;
        printf("epd-gpe: pen: %ld pieces (%dx%d DU, %d every 3 frames), %ld claims, %ld running at most (engine peak %d); ",
               pen_pieces, PEN_BOX, PEN_BOX, PEN_P3, claims, pen_peak, E.nrun_max);
        if (async)
            printf("submit CPU %.3f ms avg, collect %.3f ms avg", submits ? t_sub * 1e3 / submits : 0, collects ? t_col * 1e3 / collects : 0);
        else
            printf("claim %.3f ms avg (max %.3f) = submit %.3f%s + collect %.3f", nc ? t_gpu * 1e3 / nc : 0, t_gpu_max * 1e3,
                   claims ? t_psub * 1e3 / claims : 0, epd_gpu_prof_on ? " (GPE_PROF: incl. a finish per pass)" : " CPU",
                   claims ? t_pcol * 1e3 / claims : 0);
        printf("; CPU claim %.3f ms; %ld queries %.3f ms avg\n", claims ? t_cpu * 1e3 / claims : 0, queries, queries ? t_q * 1e3 / queries : 0);
        if (epd_gpu_prof_on) {
            const double *q = epd_gpu_prof;
            double fx = q[EPD_GPU_PROF_UPLOAD] + q[EPD_GPU_PROF_CLAIM] + q[EPD_GPU_PROF_REDUCE] + q[EPD_GPU_PROF_WAIT] + q[EPD_GPU_PROF_READ];
            printf("epd-gpe: pen fixed cost: %.3f ms a claim (upload %.3f + claim %.3f + reduce %.3f + wait %.3f + map %.3f; probe %.3f), "
                   "x %.2f a frame = %.2f ms; query %.3f ms each\n",
                   claims ? fx * 1e3 / claims : 0, claims ? q[EPD_GPU_PROF_UPLOAD] * 1e3 / claims : 0,
                   claims ? q[EPD_GPU_PROF_CLAIM] * 1e3 / claims : 0, claims ? q[EPD_GPU_PROF_REDUCE] * 1e3 / claims : 0,
                   claims ? q[EPD_GPU_PROF_WAIT] * 1e3 / claims : 0, claims ? q[EPD_GPU_PROF_READ] * 1e3 / claims : 0,
                   claims ? q[EPD_GPU_PROF_PROBE] * 1e3 / claims : 0, PEN_P3 / 3.0, claims ? fx * 1e3 / claims * PEN_P3 / 3 : 0,
                   queries ? q[EPD_GPU_PROF_QUERY] * 1e3 / queries : 0);
        }
    }
    if (bad || stale_masks || drive_bad) return 1;
    printf("epd-gpe: ok\n");
    return 0;
}
