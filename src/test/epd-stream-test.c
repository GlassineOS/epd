#ifdef LIVE
#define _GNU_SOURCE             /* (the extract: CPU affinity) */
#endif
/* epd-stream-test — epd-engine + epd-drm on the Note Air 1 under the mainline kernel.
 *
 * epd-stream-test <wbf> <hv 0|1> [scene] [celsius] [hold seconds, default 2]
 *
 * Every scene ends where it started (all white) and every pixel goes
 * white -> X -> white with the same waveform, so the panel is left DC-balanced.
 * card GC16: 16 grey bars + a black bar (768 x 478 at the panel origin), erase
 * squares DU: four 200 x 200 squares submitted 60 ms apart (concurrent runs +
 * one collision: the 4th overlaps the 1st), then all erased
 * full GC16 full-screen grey ramp, erase (render cost worst case)
 * a2 A2 (EPD_A2_OR_DU) squares with a warm link, then erased: latency
 * hv=0 is a dry run: the rails stay off and nothing on the panel changes.
 * celsius: default = battery temperature (the MAX17135 has no thermistor).
 *
 * Built with -DEPD_HOST (test/run.sh) the stream runs against epd-drm.c's
 * fake DRM device (a pipe + an 85 Hz timer: see epd-drm.c "host harness") and
 * the scene `host` is the hand-off inbox suite :
 * order 200 submits back to back reach the engine in submit order, with
 * the ids the submitters got
 * noblock with epd_schedule made 20 ms slow (a long claim under the lock)
 * while a run is going, a submit takes < 5 ms on the inbox path
 * and is shown to wait > 10 ms on the lock path (handoff=lock)
 * wake a submit to a drive thread in its idle wait (link up, timed
 * 50 ms; link down, untimed) is applied within 20 ms: no lost
 * wakeup across many rounds (a lost one shows as ~50 ms or a hang)
 * free_ms epd_stream_free_ms right after a submit counts the message not
 * yet applied (same value as the lock path's engine read), tracks
 * the run, and is 0 once idle
 * idle epd_stream_wait_idle right after a submit returns only once the
 * update ran (the inbox counts as busy)
 * knobs epd_stream_set_cut through the inbox applies (DU frames shrink)
 * frames the same scripted scene on both paths flips byte-identical frame
 * sequences (GC16 card, 4 DU squares at once, 8 touching pen
 * pieces = merges from grey, a superseding DU, the erase)
 * Built with -DEPD_HOST -DEPD_GPU (the fake device's buffers are udmabufs the
 * GPU imports) the scene `stagger` runs the stagger with stagger=check on the
 * host GPU : a cut A2/3 scroll, a GC16 popup, more scroll;
 * GLASSINE_CLAIM_STATE=packed GLASSINE_STAGGER_STATE=lanes for the lanes
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <time.h>
#include <unistd.h>
#include "../epd-drm.h"

static uint8_t grey[EPD_W * EPD_H];

#if defined(EPD_HOST) && defined(LIVE)
/* glassine-epdd-desktop's scan + commit + the job, extracted
 * (extract-scan-commit.sh; -DLIVE=<file>), with the stubs scan-commit-test
 * gives them; the `update` case below drives them into the real stream */
#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include "../epd-policy.h"
#include "../epd-dither.h"
#include "../epd-dizzy.h"
#include "../epd-upd.h"
#define PW EPD_W
#define PH EPD_H
#define TILE 32
#define TW ((PH + TILE - 1) / TILE)
#define TH ((PW + TILE - 1) / TILE)
static box tilebox[TW * TH];
static uint8_t *shadow;
static uint32_t *ebuf;
static int bw_level(uint8_t g) { return (g >> 4) == 0 || (g >> 4) == 15; }
static int dither_pattern = EPD_DITHER_BLUE;
typedef struct { box b; int bx0, w, h; uint8_t *g; int s; } frame;
static pthread_mutex_t buf_mu = PTHREAD_MUTEX_INITIALIZER;
static int dz_reuse;
static uint8_t *dz_pg, *dz_pd;
static int dizzy_still_serial;
static int dizzy_gpu, dizzy_form_layered;
static uint8_t *wlsrc;
static void *ing;
static int gt_main(void) { return 0; }
enum { EPD_GDZ_BLOCK, EPD_GDZ_LAYERED };
static int epd_ingest_dizzy_form(void *g, int form) { (void)g; return form; }
static int epd_ingest_dizzy_src(const void *g) { (void)g; return 0; }
static int epd_ingest_dizzy_v(void *g, const uint8_t *gr, int gs, int x0, int y0, int w, int h, const uint8_t *lv, uint8_t *out, int os, int ms, int view)
{
    (void)g; (void)gr; (void)gs; (void)x0; (void)y0; (void)w; (void)h; (void)lv; (void)out; (void)os; (void)ms; (void)view;
    return -1;
}
static double tsec(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec / 1e9; }
static long commit_ebuf(const frame *f, int all, int how) { (void)f; (void)all; (void)how; return 0; }   /* (no ebuf here) */
/* glassine-epdd-desktop's wl_is_view: a frame that reads sway's frame (here: wlsrc, rows PW apart) */
#define wl_is_view(f) (wlsrc && (f)->s == PW && (f)->g == wlsrc + (size_t)(f)->b.x0 * PW + (f)->bx0)
#include LIVE
#endif

#ifdef EPD_HOST
#define CHECK(c, ...) do { if (!(c)) { fprintf(stderr, "epd-stream host: FAIL " __VA_ARGS__); fprintf(stderr, "\n"); fails++; } } while (0)
static int fails;

/* the flip hook: FNV-1a of every flipped buffer, in order */
#define HMAX 4096
static uint64_t hashes[HMAX];
static int nhash;
static void on_flip(uint32_t fb, const uint32_t *w, int lines)
{
    (void)fb;
    uint64_t h = 1469598103934665603ull;
    const uint8_t *b = (const uint8_t *)w;
    for (size_t i = 0; i < (size_t)lines * EPD_LINE * 4; i++) { h ^= b[i]; h *= 1099511628211ull; }
    if (nhash < HMAX) hashes[nhash] = h;
    nhash++;
}

/* the applied hook: ids and rects in order, the time of the last one */
#define AMAX 1024
static uint32_t aid[AMAX];
static epd_rect arect[AMAX];
static int napplied;
static double t_applied;
static double hnow(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec / 1e9;
}
static void on_applied(uint32_t id, int kind, epd_rect r)
{
    (void)kind;
    if (napplied < AMAX) { aid[napplied] = id; arect[napplied] = r; }
    __atomic_store_n(&napplied, napplied + 1, __ATOMIC_SEQ_CST);
    t_applied = hnow();
}
static int applied_n(void) { return __atomic_load_n(&napplied, __ATOMIC_SEQ_CST); }

static void hfill(epd_rect r, int v)
{
    for (int y = r.y; y < r.y + r.h; y++) memset(grey + (size_t)y * EPD_W + r.x, v, r.w);
}

static epd_stream *hopen(const char *wbf, int inbox)
{
    epd_stream_inbox = inbox;
    epd_stream_qos = -2;                    /* (no cpu_dma_latency hold on the host) */
    epd_stream_host_flip = on_flip;
    epd_stream_host_applied = on_applied;
    epd_stream_host_slow_sched_us = 0;
    nhash = 0; napplied = 0;
    memset(grey, 0xff, sizeof grey);
    epd_stream *s = epd_stream_open(wbf, 24, 1);
    if (!s) { fprintf(stderr, "epd-stream host: open failed\n"); exit(1); }
    return s;
}

/* the scripted scene of `frames`. Deterministic on both paths: a single
 * submit starts at the next schedule whenever it lands; every burst goes in
 * behind a GC16 "blocker" over its area that was submitted just before, so
 * its updates wait (pixels in flight heading elsewhere) and all start in the
 * same schedule once the blocker ends, however the burst fell against the
 * drive thread's lock windows (the first run split a burst on the lock path:
 * the first submit woke the thread, whose schedule held the next ones one
 * frame; on the inbox path they drained together — a race both paths have,
 * with a smaller window on the inbox path) */
static void blocker(epd_stream *s, epd_rect r)
{
    hfill(r, 0x80);
    epd_stream_submit(s, r, EPD_GC16, 0, grey, EPD_W);
}

static void scene_frames(epd_stream *s)
{
    epd_rect card = { 0, 0, 768, 478 };
    for (int i = 0; i < 16; i++) hfill((epd_rect){ 16 + i * 46, 40, 46, 300 }, i * 17);
    hfill((epd_rect){ 16, 380, 736, 60 }, 0);
    epd_stream_submit(s, card, EPD_GC16, 0, grey, EPD_W);
    epd_stream_wait_idle(s);
    blocker(s, (epd_rect){ 300, 600, 1000, 300 });
    epd_rect q[4] = { { 300, 600, 200, 200 }, { 700, 600, 200, 200 }, { 1100, 600, 200, 200 }, { 400, 700, 200, 200 } };
    for (int i = 0; i < 4; i++) { hfill(q[i], 0); epd_stream_submit(s, q[i], EPD_DU, 0, grey, EPD_W); }
    epd_stream_wait_idle(s);
    blocker(s, (epd_rect){ 180, 990, 400, 100 });
    for (int i = 0; i < 8; i++) {           /* a pen stroke: touching pieces, merged from grey */
        epd_rect p = { 200 + i * 24, 1000 + i * 6, 30, 30 };
        hfill(p, 0);
        epd_stream_submit(s, p, EPD_A2_OR_DU, 0, grey, EPD_W);
    }
    epd_stream_wait_idle(s);
    blocker(s, (epd_rect){ 1400, 200, 200, 200 });
    hfill((epd_rect){ 1400, 200, 200, 200 }, 0);
    epd_stream_submit(s, (epd_rect){ 1450, 250, 100, 100 }, EPD_DU, 0, grey, EPD_W);
    epd_stream_submit(s, (epd_rect){ 1400, 200, 200, 200 }, EPD_DU, 0, grey, EPD_W);   /* supersedes the first */
    epd_stream_wait_idle(s);
    memset(grey, 0xff, sizeof grey);
    epd_stream_submit(s, (epd_rect){ 0, 0, EPD_W, EPD_H }, EPD_GC16, 0, grey, EPD_W);
    epd_stream_wait_idle(s);
}

#ifdef LIVE
/* update : a scripted burst sequence through glassine-epdd-desktop's own
 * scan + commit (+ the levels of the big sends) into the stream, once per
 * placement: update_job=main (one pass), the stream's workers (gold + fifo),
 * the job's own pool at fifo and at nice, and the workers with worker_rt=nice.
 * Every step: two frames (a 1300 x 1672 window scrolled by 36 rows, and a
 * small one that stays a single pass) scanned and committed in one of the 5
 * commit modes, the changed boxes sent (A2 cut steps, every 4th step a GC16)
 * behind a blocker so they all start together; then idle. Byte-identical =
 * the boxes (order too), the "left" counts, the shadow after every step and
 * the flipped frames, against main. GLASSINE_UPD_STEPS: steps (default 24);
 * GLASSINE_UPD_ONLY=v: main and variant v only (TSAN) */
#define UPD_V 6
static const char *const upd_vname[UPD_V] = { "main", "workers", "own fifo", "own nice", "workers worker_rt=nice", "workers + S4 jobs" };
static upd_pool upd_tp;
static void upd_tw(void *c, void (*fn)(void *, int), void *arg, int n) { epd_stream_upd_run(c, fn, arg, n); }
static uint64_t fnv(uint64_t h, const void *p, size_t n)
{
    const uint8_t *b = p;
    for (size_t i = 0; i < n; i++) { h ^= b[i]; h *= 1099511628211ull; }
    return h;
}
static uint8_t upd_bgrey[EPD_W * EPD_H];
static void update_suite(const char *wbf)
{
    const char *se = getenv("GLASSINE_UPD_STEPS"), *so = getenv("GLASSINE_UPD_ONLY");
    int steps = se ? atoi(se) : 24, only = so ? atoi(so) : -1;
    if (steps < 1) steps = 1;
    if (steps > 256) steps = 256;
    static uint64_t step_h[UPD_V][256];
    static uint64_t seq[UPD_V][HMAX];
    int nseq[UPD_V];
    double ms[UPD_V], lvms[UPD_V];
    uint8_t *src = malloc((size_t)PW * PH);
    shadow = malloc((size_t)PW * PH);
    memset(upd_bgrey, 0x80, sizeof upd_bgrey);
    static const int hows[5] = { C_ALL, C_BW, C_DITHER, C_THRESH, C_Q4 };
    int wrt0 = epd_stream_worker_rt;
    for (int v = 0; v < UPD_V; v++) {
        if (v && only > 0 && v != only) continue;
        epd_stream_worker_rt = v == 4;
        /* v 5: the update jobs beside 's claim jobs on the same workers (drive_overlap, 4 bands): the
         * ordering under load; its frames depend on the jobs' timing (late/slipped), so only the bytes
         * of the update job are compared, plus: no hang, every pixel white at the end */
        int ov0 = epd_stream_drive_overlap, nb0 = epd_claim_bands;
        if (v == 5) { epd_stream_drive_overlap = 1; epd_claim_bands = 4; }
        epd_stream *s = hopen(wbf, 1);
        upd_run = NULL; upd_ctx = NULL;
        if (v == 1 || v == 4 || v == 5) { upd_run = upd_tw; upd_ctx = s; }
        if (v == 2 || v == 3) { upd_pool_config(&upd_tp, NULL, 0, v == 3); upd_run = upd_pool_runner; upd_ctx = &upd_tp; }
        epd_stream_set_upd_runner(s, upd_run, upd_ctx);
        memset(shadow, 0xff, (size_t)PW * PH);
        memset(src, 0xff, (size_t)PW * PH);
        ms[v] = lvms[v] = 0;
        for (int k = 0; k < steps; k++) {
            /* the content: a scrolled page of grey text bars + a ramp column, in panel layout */
            for (int y = 60; y < 1360; y++) {
                uint8_t *r = src + (size_t)y * PW;
                int yy = y + k * 36;
                for (int x = 100; x < 1772; x++) {
                    int line = (yy / 23) % 7, col = (x / 11 + yy / 3) % 13;
                    r[x] = (line < 5 && col < 9) ? (uint8_t)(((x + yy) * 37 + k) & 0xff) : x < 160 ? (uint8_t)(yy & 0xff) : 0xff;
                }
            }
            for (int y = 200; y < 260; y++)
                for (int x = 1782; x < 1862; x++) src[(size_t)y * PW + x] = (uint8_t)((x * 5 + y * 3 + k * 40) & 0xff);
            /* two frames, portrait boxes (x = panel row, y = PW - panel column), the view snap_wl makes */
            box fb[2] = { { 60, PW - 1772, 1360, PW - 100 }, { 200, PW - 1862, 260, PW - 1782 } };
            frame fr[2];
            for (int i = 0; i < 2; i++) {
                box b = fb[i];
                fr[i] = (frame){ b, PW - b.y1, b.y1 - b.y0, b.x1 - b.x0, NULL, PW };
                fr[i].g = src + (size_t)b.x0 * PW + fr[i].bx0;
            }
            int how = hows[k % 5];
            cm_motion = 1;
            double t0 = tsec();
            boxlist ch = { 0 };
            for (int i = 0; i < 2; i++) {
                if (v) scan_job(&fr[i], 0, &ch); else scan(&fr[i], 0, &ch);
            }
            long left = 0;
            for (int i = 0; i < 2; i++) left += v ? commit_rows_job(&fr[i], 0, how) : commit(&fr[i], 0, how);
            double t1 = tsec();
            /* the sends: the changed boxes' union behind a blocker, then each box */
            box u = { 0 };
            for (int i = 0; i < ch.n; i++) u = box_union(u, ch.b[i]);
            if (!box_empty(u)) epd_stream_submit(s, (epd_rect){ PW - u.y1, u.x0, u.y1 - u.y0, u.x1 - u.x0 }, EPD_GC16, 0, upd_bgrey, EPD_W);
            double t2 = tsec();
            for (int i = 0; i < ch.n; i++) {
                box b = ch.b[i];
                int mode = k % 4 == 3 ? EPD_GC16 : EPD_MODE_A2LEN(EPD_A2_OR_DU, 4);
                epd_stream_submit(s, (epd_rect){ PW - b.y1, b.x0, b.y1 - b.y0, b.x1 - b.x0 }, mode, 0, shadow, PW);
            }
            double t3 = tsec();
            ms[v] += (t1 - t0 + t3 - t2) * 1e3;
            lvms[v] += (t3 - t2) * 1e3;
            uint64_t h = fnv(1469598103934665603ull, ch.b, sizeof ch.b[0] * (size_t)ch.n);
            h = fnv(h, &ch.n, sizeof ch.n);
            h = fnv(h, &left, sizeof left);
            step_h[v][k] = fnv(h, shadow, (size_t)PW * PH);
            epd_stream_wait_idle(s);
        }
        memset(shadow, 0xff, (size_t)PW * PH);
        epd_stream_submit(s, (epd_rect){ 0, 0, EPD_W, EPD_H }, EPD_GC16, 0, shadow, PW);
        epd_stream_wait_idle(s);
        nseq[v] = nhash < HMAX ? nhash : HMAX;
        memcpy(seq[v], hashes, (size_t)nseq[v] * sizeof hashes[0]);
        long nw = 0;
        if (v == 5) {
            static int16_t dc[EPD_W * EPD_H];
            static uint8_t lv[EPD_W * EPD_H];
            int r;
            epd_stream_dc_map(s, dc, lv, NULL, &r);
            for (size_t i = 0; i < sizeof lv; i++) nw += lv[i] != 30;
            char s4[256];
            epd_stream_s4(s, s4, sizeof s4);
            fprintf(stderr, "epd-stream host: update: workers + S4 jobs: %s; %ld pixels not white\n", s4, nw);
            CHECK(strstr(s4, " 0 jobs") == NULL && nw == 0, "update(workers + S4 jobs): %s, %ld pixels not white", s4, nw);
        }
        epd_stream_drive_overlap = ov0; epd_claim_bands = nb0;
        epd_stream_set_upd_runner(s, NULL, NULL);
        unsigned long long j, w, c, sr;
        epd_stream_upd_stats(s, &j, &w, &c, &sr);
        epd_stream_close(s);
        int same = 1, first = -1;
        for (int k = 0; k < steps && same; k++) if (step_h[v][k] != step_h[0][k]) { same = 0; first = k; }
        int fsame = nseq[v] == nseq[0];
        for (int i = 0; fsame && i < nseq[v]; i++) if (seq[v][i] != seq[0][i]) fsame = 0;
        uint64_t fd = fnv(1469598103934665603ull, seq[v], (size_t)nseq[v] * sizeof seq[v][0]);
        uint64_t sd = fnv(1469598103934665603ull, step_h[v], (size_t)steps * sizeof step_h[v][0]);
        fprintf(stderr, "epd-stream host: update: %-22s steps %016llx flips %016llx\n", upd_vname[v], (unsigned long long)sd, (unsigned long long)fd);
        if (getenv("GLASSINE_UPD_NOFLIPS") || v == 5) fsame = 1;     /* (TSAN: its timing splits bursts; the bytes stay compared) */
        fprintf(stderr, "epd-stream host: update: %-22s %d steps, %d flips, scan+commit+sends %.2f ms/step (blocker excluded; sends %.2f); "
                "workers' jobs %llu (parts %llu workers / %llu caller, %llu serial), own pool's jobs %llu (parts %llu helpers / %llu caller): %s\n",
                upd_vname[v], steps, nseq[v], ms[v] / steps, lvms[v] / steps, j, w, c, sr, upd_tp.jobs, upd_tp.hparts, upd_tp.cparts,
                !v ? "the reference" : same && fsame ? "byte-identical" : "DIFFER");
        if (v) CHECK(same && fsame, "update(%s): differs from main (first step %d, flips %s)", upd_vname[v], first, fsame ? "same" : "differ");
        if (v == 1 || v == 4 || v == 5) CHECK(j > 0, "update(%s): no jobs on the workers", upd_vname[v]);
    }
    epd_stream_worker_rt = wrt0;
    upd_run = NULL; upd_ctx = NULL;
    upd_pool_stop(&upd_tp);
    free(src); free(shadow); shadow = NULL;
}
#endif

static int host_suite(const char *wbf)
{
    double t_suite = hnow();
    epd_stream *s;
    epd_stats st;

    /* order */
    s = hopen(wbf, 1);
    uint32_t got[200];
    epd_rect sent[200];
    for (int i = 0; i < 200; i++) {
        sent[i] = (epd_rect){ (i % 40) * 40 + 8, (i / 40) * 40 + 8, 16, 16 };
        hfill(sent[i], 0);
        got[i] = epd_stream_submit(s, sent[i], EPD_DU, 0, grey, EPD_W);
        CHECK(got[i] != 0, "order: submit %d refused", i);
    }
    epd_stream_wait_idle(s);
    CHECK(applied_n() == 200, "order: %d applied of 200", applied_n());
    for (int i = 0; i < 200 && i < applied_n(); i++) {
        CHECK(aid[i] == got[i], "order: applied %d has id %u, submit got %u", i, aid[i], got[i]);
        CHECK(!memcmp(&arect[i], &sent[i], sizeof sent[i]), "order: applied %d is not submit %d's rect", i, i);
    }
    epd_stream_stats(s, &st);
    CHECK(st.updates == 200, "order: %llu updates of 200", (unsigned long long)st.updates);
    CHECK(st.inbox_n == 200 && st.inbox_full == 0, "order: inbox_n %llu full %llu", (unsigned long long)st.inbox_n, (unsigned long long)st.inbox_full);
    fprintf(stderr, "epd-stream host: order: 200 submits in order, ids match, %llu frames, drained at most %llu a time\n",
            (unsigned long long)st.frames, (unsigned long long)st.inbox_max);
    epd_stream_close(s);

    /* noblock: both paths, the same scene */
    double maxw[2] = { 0, 0 }, sumw[2] = { 0, 0 };
    for (int inbox = 1; inbox >= 0; inbox--) {
        s = hopen(wbf, inbox);
        epd_stream_host_slow_sched_us = 20000;
        hfill((epd_rect){ 0, 0, EPD_W, 700 }, 0);
        epd_stream_submit(s, (epd_rect){ 0, 0, EPD_W, 700 }, EPD_DU, 0, grey, EPD_W);
        usleep(30000);                      /* the run is going: schedule + render every frame */
        for (int i = 0; i < 60; i++) {
            epd_rect r = { 20 + i * 30, 800, 24, 24 };
            hfill(r, 0);
            double t0 = hnow();
            epd_stream_submit(s, r, EPD_DU, 0, grey, EPD_W);
            double w = (hnow() - t0) * 1e3;
            sumw[inbox] += w;
            if (w > maxw[inbox]) maxw[inbox] = w;
            usleep(3000);
        }
        epd_stream_wait_idle(s);
        epd_stream_stats(s, &st);
        CHECK(st.updates == 61, "noblock(%s): %llu updates of 61", inbox ? "inbox" : "lock", (unsigned long long)st.updates);
        epd_stream_close(s);
    }
    fprintf(stderr, "epd-stream host: noblock: a submit while epd_schedule takes 20 ms: inbox avg %.2f max %.2f ms, lock avg %.2f max %.2f ms\n",
            sumw[1] / 60, maxw[1], sumw[0] / 60, maxw[0]);
    CHECK(maxw[1] < 5, "noblock: an inbox submit waited %.2f ms", maxw[1]);
    CHECK(maxw[0] > 10, "noblock: the lock path never waited (max %.2f ms): the test would not see a regression", maxw[0]);

    /* wake: link up (timed idle wait, 50 ms) and link down (untimed) */
    s = hopen(wbf, 1);
    double wmax = 0, wsum = 0;
    int rounds = 40;
    for (int i = 0; i < rounds; i++) {
        epd_rect r = { 100 + (i % 10) * 40, 100 + (i / 10) * 40, 16, 16 };
        hfill(r, i & 1 ? 0xff : 0);
        int n0 = applied_n();
        double t0 = hnow();
        epd_stream_submit(s, r, EPD_A2_OR_DU, 0, grey, EPD_W);
        for (int k = 0; applied_n() == n0 && k < 2000; k++) usleep(100);    /* (200 ms cap: a hang fails below) */
        double w = (t_applied - t0) * 1e3;
        CHECK(applied_n() > n0, "wake: round %d not applied within 200 ms", i);
        if (w > wmax) wmax = w;
        wsum += w;
        epd_stream_wait_idle(s);
    }
    __atomic_store_n(&epd_stream_link_warm_ms, 0, __ATOMIC_RELAXED);
    usleep(700000);                         /* the linger: link down, the untimed wait */
    for (int i = 0; i < 3; i++) {
        epd_rect r = { 600, 100, 16, 16 };
        hfill(r, i & 1 ? 0xff : 0);
        int n0 = applied_n();
        double t0 = hnow();
        epd_stream_submit(s, r, EPD_A2_OR_DU, 0, grey, EPD_W);
        for (int k = 0; applied_n() == n0 && k < 2000; k++) usleep(100);
        double w = (t_applied - t0) * 1e3;
        CHECK(applied_n() > n0, "wake(down): round %d not applied within 200 ms", i);
        if (w > wmax) wmax = w;
        wsum += w;
        epd_stream_wait_idle(s);
        usleep(600000);
    }
    /* a keepalive alone (pen hover) to the fully-down, untimed wait: the link comes up */
    double kmax = 0;
    for (int i = 0; i < 3; i++) {
        epd_stream_stats(s, &st);
        uint64_t links = st.links;
        double t0 = hnow();
        epd_stream_keepalive(s, 300);
        for (int k = 0; k < 2000; k++) { epd_stream_stats(s, &st); if (st.links > links) break; usleep(100); }
        double w = (hnow() - t0) * 1e3;
        CHECK(st.links > links, "wake(keepalive): round %d: no link up within 200 ms", i);
        if (w > kmax) kmax = w;
        usleep(900000);                     /* its 300 ms hold + the linger: down again */
    }
    __atomic_store_n(&epd_stream_link_warm_ms, 30000, __ATOMIC_RELAXED);
    fprintf(stderr, "epd-stream host: wake: %d rounds, submit -> applied avg %.2f max %.2f ms; keepalive -> link up max %.1f ms\n",
            rounds + 3, wsum / (rounds + 3), wmax, kmax);
    CHECK(wmax < 20, "wake: a submit waited %.2f ms for the drive thread (a lost wakeup = its 50 ms timed wait)", wmax);
    epd_stream_close(s);

    /* free_ms: right after a submit (the message still in the inbox), mid-run, idle */
    int fm[2][3];
    for (int inbox = 1; inbox >= 0; inbox--) {
        s = hopen(wbf, inbox);
        epd_rect r = { 300, 300, 200, 200 };
        hfill(r, 0);
        epd_stream_submit(s, r, EPD_DU, 0, grey, EPD_W);     /* (the first DU fills the engine's table) */
        epd_stream_wait_idle(s);
        hfill(r, 0xff);
        epd_stream_submit(s, r, EPD_DU, 0, grey, EPD_W);
        fm[inbox][0] = epd_stream_free_ms(s, r);
        usleep(120000);
        fm[inbox][1] = epd_stream_free_ms(s, r);
        epd_stream_wait_idle(s);
        fm[inbox][2] = epd_stream_free_ms(s, r);
        epd_stream_close(s);
    }
    fprintf(stderr, "epd-stream host: free_ms: after the submit %d / %d ms, 120 ms in %d / %d, idle %d / %d (inbox / lock)\n",
            fm[1][0], fm[0][0], fm[1][1], fm[0][1], fm[1][2], fm[0][2]);
    CHECK(fm[1][0] > 0 && fm[1][0] == fm[0][0], "free_ms right after the submit: inbox %d vs lock %d ms", fm[1][0], fm[0][0]);
    CHECK(fm[1][1] > 0 && fm[1][1] < fm[1][0] && abs(fm[1][1] - fm[0][1]) <= 24, "free_ms mid-run: inbox %d vs lock %d ms", fm[1][1], fm[0][1]);
    CHECK(fm[1][2] == 0 && fm[0][2] == 0, "free_ms idle: %d / %d", fm[1][2], fm[0][2]);

    /* idle: wait_idle right after the submit */
    s = hopen(wbf, 1);
    hfill((epd_rect){ 500, 500, 100, 100 }, 0);
    epd_stream_submit(s, (epd_rect){ 500, 500, 100, 100 }, EPD_DU, 0, grey, EPD_W);
    epd_stream_wait_idle(s);
    epd_stream_stats(s, &st);
    CHECK(applied_n() == 1 && st.frames >= 10, "idle: wait_idle returned with %d applied, %llu frames", applied_n(), (unsigned long long)st.frames);
    fprintf(stderr, "epd-stream host: idle: wait_idle after a submit waited for %llu frames\n", (unsigned long long)st.frames);

    /* knobs: the cut through the inbox */
    uint64_t f0, f1;
    hfill((epd_rect){ 500, 500, 100, 100 }, 0xff);
    epd_stream_stats_reset(s);
    epd_stream_submit(s, (epd_rect){ 500, 500, 100, 100 }, EPD_DU, 0, grey, EPD_W);
    epd_stream_wait_idle(s);
    epd_stream_stats(s, &st);
    f0 = st.frames;
    epd_stream_set_cut(s, 4);
    hfill((epd_rect){ 500, 500, 100, 100 }, 0);
    epd_stream_stats_reset(s);
    epd_stream_submit(s, (epd_rect){ 500, 500, 100, 100 }, EPD_DU, 0, grey, EPD_W);
    epd_stream_wait_idle(s);
    epd_stream_stats(s, &st);
    f1 = st.frames;
    epd_stream_set_cut(s, 0);
    hfill((epd_rect){ 500, 500, 100, 100 }, 0xff);
    epd_stream_submit(s, (epd_rect){ 500, 500, 100, 100 }, EPD_DU, 0, grey, EPD_W);
    epd_stream_wait_idle(s);
    fprintf(stderr, "epd-stream host: knobs: DU %llu frames, cut 4: %llu\n", (unsigned long long)f0, (unsigned long long)f1);
    CHECK(f1 + 4 == f0, "knobs: set_cut(4) through the inbox: %llu -> %llu frames", (unsigned long long)f0, (unsigned long long)f1);
    epd_stream_close(s);

    /* frames: the scene on both paths */
    static uint64_t seq[2][HMAX];
    int nseq[2];
    uint64_t fr[2];
    for (int inbox = 1; inbox >= 0; inbox--) {
        s = hopen(wbf, inbox);
        scene_frames(s);
        epd_stream_stats(s, &st);
        fr[inbox] = st.frames;
        nseq[inbox] = nhash < HMAX ? nhash : HMAX;
        memcpy(seq[inbox], hashes, (size_t)nseq[inbox] * sizeof hashes[0]);
        CHECK(st.merged >= 1 && st.superseded >= 1, "frames(%s): merged %llu superseded %llu (the scene should merge and supersede)",
              inbox ? "inbox" : "lock", (unsigned long long)st.merged, (unsigned long long)st.superseded);
        epd_stream_close(s);
    }
    int same = nseq[0] == nseq[1];
    int first = -1;
    for (int i = 0; same && i < nseq[0]; i++) if (seq[0][i] != seq[1][i]) { same = 0; first = i; }
    fprintf(stderr, "epd-stream host: frames: inbox %d flips (%llu drive frames), lock %d (%llu): %s\n", nseq[1], (unsigned long long)fr[1],
            nseq[0], (unsigned long long)fr[0], same ? "byte-identical" : "DIFFER");
    CHECK(same, "frames: the two paths flipped different frames (first difference at flip %d)", first);

    /* overlap : the scene, then a scroll burst (full-window cut
     * steps 30 ms apart over running ones: jobs, finishes around them,
     * preempts) and a white GC16, with drive_overlap on and 4 bands on the
     * workers: no hang (wait_idle returns: the job's wakeup), jobs went out,
     * nothing joined, every pixel white at the end */
    {
        int ov0 = epd_stream_drive_overlap, nb0 = epd_claim_bands;
        epd_stream_drive_overlap = 1;
        epd_claim_bands = 4;
        s = hopen(wbf, 1);
        epd_stream_set_nonblock(s, 2);
        scene_frames(s);
        for (int i = 0; i < 12; i++) {
            for (int y = 60; y < 1360; y++)
                for (int x = 0; x < EPD_W; x++) grey[(size_t)y * EPD_W + x] = ((x / 9 + y / 13 + i * 3) % 5) ? 0xff : 0;
            epd_stream_submit(s, (epd_rect){ 0, 60, EPD_W, 1300 }, EPD_MODE_A2LEN(EPD_A2_OR_DU, 6), 0, grey, EPD_W);
            usleep(30000);
        }
        memset(grey, 0xff, sizeof grey);
        epd_stream_submit(s, (epd_rect){ 0, 0, EPD_W, EPD_H }, EPD_GC16, 0, grey, EPD_W);
        double tw = hnow();
        epd_stream_wait_idle(s);
        char s4[256];
        epd_stream_s4(s, s4, sizeof s4);
        unsigned long long jobs = 0, joined = 1;
        const char *j = strstr(s4, " jobs");
        while (j && j > s4 && j[-1] >= '0' && j[-1] <= '9') j--;
        if (j) jobs = strtoull(j, NULL, 10);
        if ((j = strstr(s4, "joined "))) joined = strtoull(j + 7, NULL, 10);
        static int16_t dc[EPD_W * EPD_H];
        static uint8_t lv[EPD_W * EPD_H];
        int r;
        epd_stream_dc_map(s, dc, lv, NULL, &r);
        long nw = 0;
        for (size_t i = 0; i < sizeof lv; i++) nw += lv[i] != 30;
        fprintf(stderr, "epd-stream host: overlap: %s; idle after %.0f ms; %ld pixels not white\n", s4, (hnow() - tw) * 1e3, nw);
        CHECK(jobs > 0 && joined == 0 && nw == 0, "overlap: jobs %llu joined %llu, %ld pixels not white", jobs, joined, nw);
        epd_stream_close(s);
        epd_stream_drive_overlap = ov0;
        epd_claim_bands = nb0;
    }

#ifdef LIVE
    update_suite(wbf);
#endif
    fprintf(stderr, "epd-stream host: %s (%d failures, %.1f s)\n", fails ? "FAIL" : "ok", fails, hnow() - t_suite);
    return fails != 0;
}

#ifdef EPD_GPU
/* scene `stagger` (-DEPD_GPU): GPU drive frames, the stagger
 * with stagger=check: a scroll of cut A2/3 steps 30 ms apart, a GC16 popup in
 * the middle (a non-cut update: the run ends), more scroll, a white GC16.
 * Both stagger_state=runs and =lanes (GLASSINE_CLAIM_STATE=packed
 * GLASSINE_STAGGER_STATE=lanes): runs started and ended, no check difference
 * (frames, state, self, loads), no evict waited out (lanes), every pixel
 * white at the end */
static void scroll(epd_stream *s, int n, int ph)
{
    for (int i = 0; i < n; i++) {
        for (int y = 60; y < 1360; y++)
            for (int x = 0; x < EPD_W; x++) grey[(size_t)y * EPD_W + x] = ((x / 11 + y / 17 + (i + ph) * 4) % 6) ? 0xff : 0;
        epd_stream_submit(s, (epd_rect){ 0, 60, EPD_W, 1300 }, EPD_MODE_A2LEN(EPD_A2, 3), 0, grey, EPD_W);
        usleep(30000);
    }
}

static int stagger_suite(const char *wbf)
{
    double t0 = hnow();
    epd_stream_gpu = 1;
    epd_stream_stagger = 1;
    epd_stream_stag_check = 1;
    if (getenv("STAG_CTL4")) {              /* (stagger_ctl 4: its parameters; "stagger_ctl 4: bit planes" in the log = it ran) */
        epd_stream_stag_ctl = 4;
        epd_stream_stag_payrule = 1;
        epd_stream_stag_cap = 3;
        if (getenv("STAG_STRIPE")) epd_stream_stag_stripe = atoi(getenv("STAG_STRIPE"));
        if (getenv("STAG_VERBOSE")) epd_stream_verbose = 1;
    }
    epd_stream *s = hopen(wbf, 1);
    if (epd_stream_gpu_dead(s)) { fprintf(stderr, "epd-stream host: stagger: no GPU drive frames (skipped)\n"); epd_stream_close(s); return 0; }
    hfill((epd_rect){ 1800, 1380, 40, 20 }, 0);     /* (the link up first: before it the inbox's steps arrive at once) */
    epd_stream_submit(s, (epd_rect){ 1800, 1380, 40, 20 }, EPD_DU, 0, grey, EPD_W);
    epd_stream_wait_idle(s);
    scroll(s, 20, 0);
    hfill((epd_rect){ 400, 400, 600, 400 }, 0x40);
    epd_stream_submit(s, (epd_rect){ 400, 400, 600, 400 }, EPD_GC16, 0, grey, EPD_W);
    usleep(200000);
    scroll(s, 20, 3);
    epd_stream_wait_idle(s);
    usleep(300000);                         /* (the quiet end of the last run) */
    memset(grey, 0xff, sizeof grey);
    epd_stream_submit(s, (epd_rect){ 0, 0, EPD_W, EPD_H }, EPD_GC16, 0, grey, EPD_W);
    epd_stream_wait_idle(s);
    epd_stats st;
    epd_stream_stats(s, &st);
    static int16_t dc[EPD_W * EPD_H];
    static uint8_t lv[EPD_W * EPD_H];
    int r;
    epd_stream_dc_map(s, dc, lv, NULL, &r);
    long nw = 0;
    for (size_t i = 0; i < sizeof lv; i++) nw += lv[i] != 30;
    const char *ls = getenv("GLASSINE_STAGGER_STATE");
    fprintf(stderr, "epd-stream host: stagger (%s): %llu runs (%llu exits), %llu steps, %llu frames; check %llu frames (%llu bad, %llu px), "
            "state bad %llu, self %llu (%llu bad), loads %llu (%llu bad); lanes: %llu arms, acks after %llu frames in all, %llu late, syncs %.2f ms; "
            "ends quiet %llu other %llu; pool: %llu arms, %llu run-end passes; %ld pixels not white, %.1f s\n",
            ls ? ls : "runs", (unsigned long long)st.stag_entries, (unsigned long long)st.stag_exits, (unsigned long long)st.stag_subs,
            (unsigned long long)st.stag_frames, (unsigned long long)st.stag_chk_frames, (unsigned long long)st.stag_chk_frames_bad,
            (unsigned long long)st.stag_chk_px_bad, (unsigned long long)st.stag_chk_state_bad, (unsigned long long)st.stag_chk_self,
            (unsigned long long)st.stag_chk_self_bad, (unsigned long long)st.stag_chk_loads, (unsigned long long)st.stag_chk_loads_bad,
            (unsigned long long)st.stag_lane_arms, (unsigned long long)st.stag_lane_ack_frames, (unsigned long long)st.stag_lane_ack_late,
            st.stag_lane_sync_ms_sum, (unsigned long long)st.stag_end_quiet, (unsigned long long)st.stag_end_other,
            (unsigned long long)st.stag_lane_par, (unsigned long long)st.stag_lane_xpar, nw, hnow() - t0);
    CHECK(st.stag_entries >= 2 && st.stag_exits == st.stag_entries, "stagger: %llu runs, %llu exits", (unsigned long long)st.stag_entries,
          (unsigned long long)st.stag_exits);
    CHECK(st.stag_chk_frames > 0 && !st.stag_chk_frames_bad && !st.stag_chk_state_bad && !st.stag_chk_self_bad && !st.stag_chk_loads_bad,
          "stagger: check differences");
    CHECK(!st.stag_lane_ack_late, "stagger: %llu evicts waited out", (unsigned long long)st.stag_lane_ack_late);
    CHECK(!ls || strcmp(ls, "lanes") || st.stag_lane_arms > 0, "stagger: lanes asked for, no arms");
    CHECK(nw == 0, "stagger: %ld pixels not white", nw);
    epd_stream_close(s);
    fprintf(stderr, "epd-stream host: %s (%d failures)\n", fails ? "FAIL" : "ok", fails);
    return fails != 0;
}
#endif
#endif

static int battery_celsius(void)
{
    DIR *d = opendir("/sys/class/power_supply");
    struct dirent *de;
    int c = 24;
    while (d && (de = readdir(d))) {
        char p[320], b[32];
        if (!strstr(de->d_name, "bat")) continue;
        snprintf(p, sizeof p, "/sys/class/power_supply/%s/temp", de->d_name);
        FILE *f = fopen(p, "r");
        if (!f) continue;
        if (fgets(b, sizeof b, f)) { c = atoi(b) / 10; fprintf(stderr, "temperature %d C from %s\n", c, de->d_name); }
        fclose(f);
        break;
    }
    if (d) closedir(d);
    return c;
}

static double now(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec / 1e9;
}

static void fill(epd_rect r, int v)
{
    for (int y = r.y; y < r.y + r.h; y++) memset(grey + (size_t)y * EPD_W + r.x, v, r.w);
}

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage: %s <wbf> <hv 0|1> [card|squares|full] [celsius]\n", argv[0]); return 2; }
    const char *scene = argc > 3 ? argv[3] : "card";
#ifdef EPD_HOST
    if (!strcmp(scene, "host")) return host_suite(argv[1]);
#ifdef EPD_GPU
    if (!strcmp(scene, "stagger")) return stagger_suite(argv[1]);
#endif
#endif
    int hv = atoi(argv[2]);
    int c = argc > 4 && atoi(argv[4]) ? atoi(argv[4]) : battery_celsius();
    int hold = argc > 5 ? atoi(argv[5]) : 2;
    epd_stream_verbose = 1;
    epd_stream *s = epd_stream_open(argv[1], c, hv);
    if (!s) return 1;
    memset(grey, 0xff, sizeof grey);
    double t0 = now();

    if (!strcmp(scene, "card")) {
        epd_rect r = { 0, 0, 768, 478 };
        for (int i = 0; i < 16; i++) fill((epd_rect){ 16 + i * 46, 40, 46, 300 }, i * 17);
        fill((epd_rect){ 16, 380, 736, 60 }, 0);
        epd_stream_submit(s, r, EPD_GC16, 0, grey, EPD_W);
        epd_stream_wait_idle(s);
        fprintf(stderr, "card on after %.0f ms\n", (now() - t0) * 1e3);
        sleep(hold);
        fill(r, 0xff);
        epd_stream_submit(s, r, EPD_GC16, 0, grey, EPD_W);
    } else if (!strcmp(scene, "squares")) {
        epd_rect q[4] = { { 300, 300, 200, 200 }, { 700, 300, 200, 200 }, { 1100, 300, 200, 200 }, { 400, 400, 200, 200 } };
        for (int i = 0; i < 4; i++) {
            fill(q[i], 0);
            epd_stream_submit(s, q[i], EPD_DU, 0, grey, EPD_W);
            usleep(60000);
        }
        epd_stream_wait_idle(s);
        fprintf(stderr, "squares on after %.0f ms\n", (now() - t0) * 1e3);
        sleep(hold);
        memset(grey, 0xff, sizeof grey);
        epd_stream_submit(s, (epd_rect){ 300, 300, 1000, 300 }, EPD_DU, 0, grey, EPD_W);
    } else if (!strcmp(scene, "a2")) {
        /* the WBF's A2 (10 frames) via A2_OR_DU: three black squares, one at
         * a time with the link kept warm (first frame without the wake-up),
         * held, then erased the same way; every pixel W -> B -> W in A2 */
        epd_rect q[3] = { { 300, 900, 160, 160 }, { 600, 900, 160, 160 }, { 900, 900, 160, 160 } };
        epd_stream_keepalive(s, 3000);
        usleep(150000);
        for (int i = 0; i < 3; i++) {
            double t = now();
            fill(q[i], 0);
            epd_stream_submit(s, q[i], EPD_A2_OR_DU, 0, grey, EPD_W);
            epd_stream_wait_idle(s);
            fprintf(stderr, "square %d: %.0f ms\n", i, (now() - t) * 1e3);
        }
        sleep(hold);
        epd_stream_keepalive(s, 3000);
        usleep(150000);
        memset(grey, 0xff, sizeof grey);
        double t = now();
        epd_stream_submit(s, (epd_rect){ 300, 900, 760, 160 }, EPD_A2_OR_DU, 0, grey, EPD_W);
        epd_stream_wait_idle(s);
        fprintf(stderr, "erase: %.0f ms\n", (now() - t) * 1e3);
    } else if (!strcmp(scene, "full")) {
        for (int y = 0; y < EPD_H; y++)
            for (int x = 0; x < EPD_W; x++) grey[(size_t)y * EPD_W + x] = (x * 16 / EPD_W) * 17;
        epd_stream_submit(s, (epd_rect){ 0, 0, EPD_W, EPD_H }, EPD_GC16, 0, grey, EPD_W);
        epd_stream_wait_idle(s);
        fprintf(stderr, "ramp on after %.0f ms\n", (now() - t0) * 1e3);
        sleep(hold);
        memset(grey, 0xff, sizeof grey);
        epd_stream_submit(s, (epd_rect){ 0, 0, EPD_W, EPD_H }, EPD_GC16, 0, grey, EPD_W);
    } else { fprintf(stderr, "unknown scene %s\n", scene); epd_stream_close(s); return 2; }

    epd_stream_wait_idle(s);
    epd_stats st;
    epd_stream_stats(s, &st);
    epd_stream_close(s);
    printf("%s hv=%d %d C: %.0f ms, %llu updates, %llu frames, %llu late, %llu links, %llu rail drops, render max %.1f ms avg %.1f ms\n",
           scene, hv, c, (now() - t0) * 1e3, (unsigned long long)st.updates, (unsigned long long)st.frames,
           (unsigned long long)st.late, (unsigned long long)st.links, (unsigned long long)st.rail_drops,
           st.render_ms_max, st.frames ? st.render_ms_sum / st.frames : 0);
    return 0;
}
