/* epd-tpool-test — the update targets' pool (epd_target_alloc /
 * epd_target_free, epd-engine.c) changes nothing on the wire. A flood of
 * scroll steps (supersedes, non-blocking payback absorbs, the glassine-epdd-desktop payback
 * defaults) plus pen-like pieces (merges) and random rects of many sizes runs
 * twice through the real engine: target_pool=0 (malloc/free each, as before)
 * and target_pool=4 with the pool seeded with poisoned (0xA5) blocks, so a
 * reader that looked at a block before writing it would show. Every frame's
 * wire words and the final engine state are hashed; both runs must match.
 * Then a micro-benchmark of epd_target_make + epd_target_free (the hand-off's
 * "copy" and the drop) for a full-panel and a 31 % rect, pool off vs on; with
 * MUSL=1 glibc's mmap threshold is pinned at 128 KB and its trim off-limits,
 * which EMULATES musl's mallocng (a fresh mmap per block >= ~128 KB, munmap at
 * free) -- glibc's own dynamic threshold hides the cost after the first free.
 * epd-tpool-test <wbf> [frames] (MUSL=1: the emulation for the bench) */
#include <malloc.h>
#include <sys/resource.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "../epd-engine.h"

static epd_engine E;
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

#define PAGE_H 6000
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

static uint64_t fnv(uint64_t h, const void *p, size_t n)
{
    const uint8_t *b = p;
    for (size_t i = 0; i < n; i++) h = (h ^ b[i]) * 0x100000001b3ull;
    return h;
}

static uint32_t words[EPD_WORDS];

/* one run; returns the hash of every frame's words + the final state */
static uint64_t run(const uint8_t *wbf, size_t wn, int frames, unsigned long *nsub)
{
    memset(&E, 0, sizeof E);
    if (epd_init(&E, wbf, wn, 24)) { fprintf(stderr, "wbf\n"); exit(1); }
    epd_assume(&E, 30);
    epd_set_matched(&E, 1);
    epd_set_matched_rule(&E, 2, 3);
    epd_set_nonblock(&E, 2);
    epd_set_model(&E, 2, 0);
    epd_set_heal(&E, 2, 2);
    epd_set_snap(&E, 1);
    memset(words, 0, sizeof words);
    int dirty[2] = { 0, EPD_LINES };
    static const int cuts[] = { 8, 6, 4, 2 };
    epd_rect win = { 0, 60, EPD_W, 1300 };
    int off = 0;
    uint64_t h = 0xcbf29ce484222325ull;
    unsigned seed = 77;
    *nsub = 0;
    for (int f = 0; f < frames; f++) {
        seed = seed * 1103515245u + 12345u;
        int k = (int)(seed >> 16) % 8;
        if (f >= frames / 2) {                  /* a video: one 31 % rect, a new picture every 2 frames (supersedes) */
            if (f % 2 == 0) {
                const uint8_t *g = page + (size_t)((f * 11) % (PAGE_H - EPD_H)) * EPD_W;
                epd_rect r = { 300, 150, 786, 1053 };
                uint8_t *t = epd_target_make(&r, g, EPD_W);
                if (t) epd_submit_target(&E, r, EPD_GC16, 0, t, g, EPD_W);
                (*nsub)++;
            }
        } else if (f % 5 == 0) {                       /* a scroll step (a flood: supersedes, absorbs) */
            off = (off + 36) % (PAGE_H - EPD_H);
            const uint8_t *g = page + (size_t)off * EPD_W;
            epd_rect r = win;
            uint8_t *t = epd_target_make(&r, g, EPD_W);
            if (t) epd_submit_target(&E, r, EPD_MODE_A2LEN(EPD_A2_OR_DU, cuts[*nsub % 4]), 0, t, g, EPD_W);
            (*nsub)++;
        }
        if (f < frames / 2 && k < 2 && f % 6 == 0) {  /* a random rect of a random size: GC16 or DU */
            seed = seed * 1103515245u + 12345u;
            int w = 16 + (int)(seed >> 8) % (EPD_W - 16), hh = 16 + (int)(seed >> 3) % (EPD_H - 16);
            seed = seed * 1103515245u + 12345u;
            epd_rect r = { (int)(seed >> 9) % (EPD_W - w + 1), (int)(seed >> 4) % (EPD_H - hh + 1), w, hh };
            const uint8_t *g = page + (size_t)((f * 7) % (PAGE_H - EPD_H)) * EPD_W;
            uint8_t *t = epd_target_make(&r, g, EPD_W);
            if (t) epd_submit_target(&E, r, k == 0 ? EPD_GC16 : EPD_DU, f % 64 == 0, t, g, EPD_W);
            (*nsub)++;
        }
        if (f < frames / 2 && k == 5) {                           /* pen pieces: small touching rects (merges) */
            const uint8_t *g = page + (size_t)((f * 13) % (PAGE_H - EPD_H)) * EPD_W;
            int x = 200 + (f % 50) * 6, y = 300 + (f % 37) * 5;
            for (int p = 0; p < 4; p++) {
                epd_rect r = { x + p * 6, y + p * 3, 12, 12 };
                uint8_t *t = epd_target_make(&r, g, EPD_W);
                if (t) epd_submit_target(&E, r, EPD_DU, 0, t, g, EPD_W);
                (*nsub)++;
            }
        }
        epd_schedule(&E);
        if (getenv("TRACE") && f % 250 == 0) fprintf(stderr, "f %d pend %d run %d order %llu pixel %llu slot %llu\n", f, E.npend, E.nrun, (unsigned long long)E.wait_order, (unsigned long long)E.wait_pixel, (unsigned long long)E.wait_slot);
        epd_render(&E, words, dirty);
        h = fnv(h, words, sizeof words);
        epd_advance(&E, NULL, NULL);
    }
    h = fnv(h, E.level, EPD_NPX);
    h = fnv(h, E.target, EPD_NPX);
    h = fnv(h, E.busy, EPD_NPX);
    h = fnv(h, E.dc, EPD_NPX * sizeof *E.dc);
    h = fnv(h, &E.superseded, sizeof E.superseded);
    h = fnv(h, &E.merged, sizeof E.merged);
    h = fnv(h, &E.absorbed, sizeof E.absorbed);
    unsigned long long st[3] = { E.superseded, E.merged, E.absorbed };
    fprintf(stderr, "  (superseded %llu, merged %llu, absorbed %llu, pending at the end %d)\n", st[0], st[1], st[2], E.npend);
    for (int i = 0; i < E.npend; i++) epd_target_free(E.pend[i].target);
    E.npend = 0;
    epd_free(&E);
    return h;
}

/* the hand-off's copy (epd_target_make) and its drop, n times */
static void bench(const char *what, epd_rect r0, int n)
{
    const uint8_t *g = page;
    double tm = 0, tf = 0;
    struct rusage u0, u1;
    getrusage(RUSAGE_SELF, &u0);
    for (int i = 0; i < n; i++) {
        epd_rect r = r0;
        double t0 = now();
        uint8_t *t = epd_target_make(&r, g + (size_t)(i % 64) * EPD_W, EPD_W);
        double t1 = now();
        epd_target_free(t);
        tf += now() - t1; tm += t1 - t0;
    }
    getrusage(RUSAGE_SELF, &u1);
    unsigned long hit, miss;
    int kept;
    epd_target_pool_stats(&hit, &miss, &kept);
    printf("  %-22s target_pool=%d: make %.3f ms, free %.3f ms, %.0f page faults (avg of %d; pool %lu reused, %lu fresh)\n", what,
           epd_target_pool, tm * 1e3 / n, tf * 1e3 / n, (double)(u1.ru_minflt - u0.ru_minflt) / n, n, hit, miss);
}

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s <wbf> [frames]\n", argv[0]); return 2; }
    int frames = argc > 2 ? atoi(argv[2]) : 1200;
    int musl = getenv("MUSL") != NULL;
    if (musl) {                                 /* glibc behaving like musl's mallocng (see the top) */
#ifdef __GLIBC__
        mallopt(M_MMAP_THRESHOLD, 128 * 1024);
        mallopt(M_TRIM_THRESHOLD, 128 * 1024);
#endif
    }
    size_t wn;
    uint8_t *wbf = slurp(argv[1], &wn);
    make_page();
    unsigned long n0, n1;
    epd_target_pool = 0;
    uint64_t a = run(wbf, wn, frames, &n0);
    epd_target_pool = 4;
    {                                           /* the pool seeded with poison, of several sizes */
        static const size_t sz[] = { (size_t)EPD_W * EPD_H, (size_t)EPD_W * 1300, 600 * 1024, 200 * 1024 };
        uint8_t *b[4];
        for (int i = 0; i < 4; i++) { b[i] = malloc(sz[i]); memset(b[i], 0xA5, sz[i]); }
        for (int i = 0; i < 4; i++) epd_target_free(b[i]);
    }
    uint64_t b = run(wbf, wn, frames, &n1);
    unsigned long hit, miss;
    int kept;
    epd_target_pool_stats(&hit, &miss, &kept);
    printf("epd-tpool: %d frames, %lu updates: pool off %016llx, pool on %016llx (%lu reused, %lu fresh, %d kept): %s\n",
           frames, n0, (unsigned long long)a, (unsigned long long)b, hit, miss, kept, a == b && n0 == n1 ? "same" : "DIFFER");
    if (a != b || n0 != n1) return 1;
    printf("bench (%s malloc):\n", musl ? "musl-like (glibc, mmap threshold 128 KB = EMULATION)" : "glibc default");
    epd_rect full = { 0, 0, EPD_W, EPD_H }, part = { 0, 0, (int)(EPD_W * 0.42), EPD_H * 3 / 4 };  /* 31 %: 42 % x 75 % */
    for (int on = 0; on < 2; on++) {
        epd_target_pool = on ? 4 : 0;
        if (!on) { epd_target_free(epd_target_alloc(1 << 20)); }   /* (drains: 0 frees the kept ones at the next free) */
        bench("full panel 2.63 MB", full, 200);
        bench("31 % rect 0.81 MB", part, 200);
    }
    return 0;
}
