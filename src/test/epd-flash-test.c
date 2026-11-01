/* epd_submit full = 2 (a flash of only the pixels that need one, e->stale):
 * static black UI is left alone (the shell bar took -2 DC frames at every
 * run-end flash), scrolled pixels are flashed, full = 1 drives all */
#include "epd-engine.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static void drain(epd_engine *e) { while (epd_schedule(e) || !epd_idle(e)) epd_advance(e, NULL, NULL); }
static uint8_t img[EPD_W * EPD_H];
static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL " __VA_ARGS__); printf("\n"); } else { printf("ok   " __VA_ARGS__); printf("\n"); } } while (0)
int main(int c, char **v)
{
    if (c < 2) return 2;
    FILE *f = fopen(v[1], "rb"); fseek(f, 0, 2); long n = ftell(f); rewind(f);
    uint8_t *w = malloc(n); fread(w, 1, n, f); fclose(f);
    epd_engine *e = calloc(1, sizeof *e);
    if (epd_init(e, w, n, 28)) return 2;
    epd_set_matched(e, 1); epd_set_matched_rule(e, 2, 3); epd_set_nonblock(e, 2);
    epd_set_heal(e, 2, 2); epd_set_model(e, 2, 0);
    for (int y = 0; y < EPD_H; y++) for (int x = 0; x < EPD_W; x++) img[(size_t)y * EPD_W + x] = x < 200 ? 0 : 255;  /* a black bar */
    epd_rect all = { 0, 0, EPD_W, EPD_H };
    epd_submit(e, all, EPD_GC16, 1, img, EPD_W); drain(e);     /* full: everything exact */
    memset(e->dc, 0, EPD_NPX * sizeof *e->dc);
    size_t B = 100 * EPD_W + 100, T = 500 * EPD_W + 900;
    int bar0 = e->dc[B];
    epd_submit(e, all, EPD_GC16, 2, img, EPD_W); drain(e);
    CHECK(e->dc[B] == bar0, "flash over an untouched black bar: dc %+d", e->dc[B] - bar0);
    CHECK(e->flash_skipped >= (unsigned long)(EPD_W * EPD_H) - 10, "flash left %llu of %d px alone", (unsigned long long)e->flash_skipped, EPD_W * EPD_H);
    /* scroll steps over the content area (x >= 200): text appears, A2/2 */
    for (int k = 0; k < 6; k++) {
        for (int y = 400; y < 600; y++) for (int x = 800; x < 1000; x++) img[(size_t)y * EPD_W + x] = ((x + y + k * 7) / 9) & 1 ? 0 : 255;
        epd_rect r = { 200, 0, EPD_W - 200, EPD_H };
        epd_submit(e, r, EPD_MODE_A2LEN(EPD_A2, 2), 0, img, EPD_W); drain(e);
    }
    CHECK(e->stale[T] == 1, "a pixel the scroll drove is stale");
    CHECK(e->stale[B] == 0, "the bar stays exact");
    unsigned long long s0 = e->flash_skipped;
    int b1 = e->dc[B];
    epd_submit(e, all, EPD_GC16, 2, img, EPD_W);
    epd_schedule(e);
    CHECK(e->busy[T] != 0, "the flash drives the scrolled pixel");
    CHECK(e->busy[B] == 0, "the flash leaves the bar");
    drain(e);
    CHECK(e->stale[T] == 0, "after the flash the scrolled pixel is exact");
    CHECK(e->dc[B] == b1, "bar dc unchanged by the run-end flash");
    printf("     second flash left %llu px alone\n", e->flash_skipped - s0);
    /* a full (1) refresh still drives everything */
    epd_submit(e, all, EPD_GC16, 1, img, EPD_W); epd_schedule(e);
    CHECK(e->busy[B] != 0, "a full refresh still drives the bar");
    drain(e);
    return fails != 0;
}
