/* epd-pack-test — part 2: the vectorized CPU halves of the engine hand-off
 * (epd_target_make) and the GPU state pack (epd_gpu_pack_state) against the
 * old per-pixel loops, on random state, plus their time per full panel. No GPU.
 * epd-pack-test (exit 0 = identical) */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "../epd-engine.h"
#include "../epd-gpu.h"

#define IDLE 1023
static epd_engine E;
static uint16_t got[EPD_W * EPD_H + 8], want[EPD_W * EPD_H + 8];
static uint8_t grey[EPD_W * EPD_H];
static int fails;
static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec / 1e9; }

static int ref_pack(uint16_t *s, epd_rect r)       /* epd-gpu.c before part 2 */
{
    if (r.x < 0) { r.w += r.x; r.x = 0; }
    if (r.y < 0) { r.h += r.y; r.y = 0; }
    if (r.x + r.w > EPD_W) r.w = EPD_W - r.x;
    if (r.y + r.h > EPD_H) r.h = EPD_H - r.y;
    if (r.w <= 0 || r.h <= 0) return 0;
    int x0 = r.x & ~7, x1 = (r.x + r.w + 7) & ~7, n = 0;
    for (int y = r.y; y < r.y + r.h; y++) {
        size_t o = (size_t)y * EPD_W;
        for (int x = x0; x < x1; x++, n++) {
            int b = E.busy[o + x];
            s[n] = b ? (uint16_t)((b - 1) << 10 | E.target[o + x] << 5 | E.level[o + x]) : IDLE;
        }
    }
    return n;
}

int main(void)
{
    if (epd_state_alloc(&E)) return 1;      /* (the arrays are allocated) */
    srand(7);
    for (size_t i = 0; i < sizeof grey; i++) {
        grey[i] = (uint8_t)rand();
        E.busy[i] = rand() % 3 ? 0 : (uint8_t)(1 + rand() % 64);
        E.target[i] = (uint8_t)(rand() % 31);
        E.level[i] = (uint8_t)(rand() % 31);
    }
    epd_rect rs[] = { { 0, 0, EPD_W, EPD_H }, { 3, 5, 17, 9 }, { -4, -4, 20, 20 }, { 1860, 1390, 40, 40 },
                      { 101, 77, 1, 1 }, { 0, 0, 0, 5 }, { 500, 300, 640, 480 } };
    for (unsigned k = 0; k < sizeof rs / sizeof *rs; k++) {
        epd_rect r = rs[k];
        memset(got, 0xAA, sizeof got); memset(want, 0x55, sizeof want);
        int n = ref_pack(want, r);
        int ok = epd_gpu_pack_state(got, &E, &r);
        if (!n != !ok || (n && (r.w * r.h != n || memcmp(got, want, (size_t)n * 2)))) {
            printf("FAIL pack rect %u (%d,%d %dx%d)\n", k, rs[k].x, rs[k].y, rs[k].w, rs[k].h); fails++;
        }
        /* the hand-off copy: (g >> 4) * 2 */
        epd_rect q = rs[k];
        uint8_t *t = epd_target_make(&q, grey, EPD_W);
        if (t) {
            for (int y = 0; y < q.h && !fails; y++)
                for (int x = 0; x < q.w; x++)
                    if (t[(size_t)y * q.w + x] != (grey[(size_t)(q.y + y) * EPD_W + q.x + x] >> 4) << 1) {
                        printf("FAIL target rect %u at %d,%d\n", k, x, y); fails++; break;
                    }
            free(t);
        }
    }
    /* time per full panel (host CPU: relative only, the A53 decides) */
    epd_rect all = { 0, 0, EPD_W, EPD_H };
    double t0 = now();
    for (int i = 0; i < 20; i++) ref_pack(want, all);
    double t1 = now();
    for (int i = 0; i < 20; i++) { epd_rect r = all; epd_gpu_pack_state(got, &E, &r); }
    double t2 = now();
    for (int i = 0; i < 20; i++) { epd_rect r = all; free(epd_target_make(&r, grey, EPD_W)); }
    double t3 = now();
    printf("pack: old %.2f ms, new %.2f ms per panel; target make %.2f ms\n",
           (t1 - t0) * 50, (t2 - t1) * 50, (t3 - t2) * 50);
    printf(fails ? "epd-pack: FAILED\n" : "epd-pack: ok\n");
    return fails != 0;
}
