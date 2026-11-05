/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* a scroll from a still page with greys (after its GC16 cleanup): A2 has no
 * drive from grey, so those pixels go DU (~19 frames); that part must not
 * hold up the next step (device: ~220 ms per step for ~1 s), and
 * every pixel must still end at the last step's content */
#include "epd-engine.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static void drain(epd_engine *e) { while (epd_schedule(e) || !epd_idle(e)) epd_advance(e, NULL, NULL); }
static uint8_t page[EPD_W * 4000], img[EPD_W * EPD_H];
int main(int c, char **v)
{
    if (c < 2) return 2;
    FILE *f = fopen(v[1], "rb"); fseek(f, 0, 2); long n = ftell(f); rewind(f);
    uint8_t *w = malloc(n); fread(w, 1, n, f); fclose(f);
    epd_engine *e = calloc(1, sizeof *e);
    if (epd_init(e, w, n, 28)) return 2;
    epd_set_matched(e, 1); epd_set_matched_rule(e, 2, 3); epd_set_nonblock(e, 2);
    epd_set_heal(e, 2, 2); epd_set_model(e, 2, 0);
    /* a page (panel order: scrolling = shifting along x) of text with grey antialiasing */
    srand(1);
    for (int x = 0; x < 4000; x++) for (int y = 0; y < EPD_H; y++) {
        int line = (x / 40) % 2, g = rand() % 100;
        page[(size_t)y * 4000 + x] = !line ? 255 : g < 20 ? 0 : g < 30 ? 128 : g < 35 ? 200 : 255;
    }
    for (int y = 0; y < EPD_H; y++) memcpy(img + (size_t)y * EPD_W, page + (size_t)y * 4000, EPD_W);
    epd_rect all = { 0, 0, EPD_W, EPD_H };
    epd_submit(e, all, EPD_GC16, 1, img, EPD_W); drain(e);      /* still page with greys */
    int grey = 0, worst = 0;   /* targets dithered to black/white, as in a run */
    for (int step = 1; step <= 12; step++) {
        for (int y = 0; y < EPD_H; y++) for (int x = 0; x < EPD_W; x++) {
            uint8_t g = page[(size_t)y * 4000 + x + step * 20];
            img[(size_t)y * EPD_W + x] = grey ? g : g < 128 ? 0 : g < 160 ? ((x + y) & 1 ? 0 : 255) : 255;
        }
        epd_submit(e, all, EPD_MODE_A2LEN(EPD_A2_OR_DU, 2), 0, img, EPD_W);
        int fr = 0;
        for (;;) { epd_schedule(e); if (!epd_frames_until_free(e, all) || fr > 400) break; epd_advance(e, NULL, NULL); fr++; }
        int du = 0;
        for (int s = 0; s < EPD_MAX_UPD; s++) if (e->run[s].nframes && EPD_MODE_BASE(e->run[s].mode) == EPD_DU) du++;
        (void)du;
        if (fr > worst) worst = fr;
    }
    drain(e);
    long bad = 0;
    for (size_t i = 0; i < (size_t)EPD_W * EPD_H; i++) bad += e->level[i] != (img[i] >> 3 & 0x1e);
    printf("grey start: steps free after <= %d frames, %ld px not at the last content\n", worst, bad);
    return worst > 3 || bad;
}
