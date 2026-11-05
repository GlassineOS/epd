/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* glassine-epd-show IMAGE.pgm [waveform_mode [BAR TEXT]]
 * Show an 8-bit binary PGM (P5) on the Note Air 1 e-ink panel via the stock kernel's
 * /dev/ebc interface. The sequence (our TWRP backend uses it too):
 *
 * open fb0, open+mmap /dev/ebc (pixel-exact 1872x1404x4), SEND_UPDATE (init),
 * FBIOBLANK powerdown+unblank, then the content SEND_UPDATE.
 * Only ioctl 0x700c is used — unknown /dev/ebc ioctls panic this kernel.
 * The panel is landscape 1872x1404. A portrait 1404x1872 PGM is rotated
 * 90 deg (direction: GLASSINE_EPD_ROT=cw|ccw, default ccw — confirmed upright on HW).
 * waveform_mode default 2 (GC16). BAR TEXT: a black bar with that text (capitals,
 * epd-glyphs.h) across the bottom of the picture as seen, like glassine-epdd-desktop's
 * SLEEPING bar; glassine-splash's shutdown screen says POWERED OFF with it.
 * Mainline kernel (no /dev/ebc): the same picture through glassine-epdd-desktop's userspace
 * engine (epd-engine.c + epd-drm.c): an INIT flash to white, then a full update
 * in the given mode; the panel levels are left in /run/glassine-epdd-desktop.levels. It
 * must finish before glassine-epdd-desktop opens the panel (glassine-x starts after glassine-splash). */
#include <errno.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include "epd-engine.h"
#include "epd-drm.h"
#include "epd-glyphs.h"

#define EBC_SEND_UPDATE 0x700c
#define W 1872
#define H 1404

struct onyx_epdc_update {
    int32_t y, x, width, height;
    int32_t waveform_mode, update_mode, marker, dither;
    uint32_t flags;
    int32_t reserved;
};

static int send(int fd, int mode, int marker)
{
    struct onyx_epdc_update u = {0};
    u.width = W; u.height = H;
    u.waveform_mode = mode;
    u.update_mode = 1;        /* full */
    u.marker = marker;
    u.dither = 0x1000;        /* full-screen value */
    u.flags = 0x20000;
    if (ioctl(fd, EBC_SEND_UPDATE, &u) < 0) { perror("SEND_UPDATE"); return -1; }
    return 0;
}

static uint8_t *read_pgm(const char *path, int *w, int *h)
{
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return NULL; }
    int maxv;
    if (fscanf(f, "P5 %d %d %d", w, h, &maxv) != 3 || maxv != 255) {
        fprintf(stderr, "%s: need binary 8-bit PGM (P5, maxval 255)\n", path);
        fclose(f); return NULL;
    }
    fgetc(f);
    size_t n = (size_t)*w * *h;
    uint8_t *p = malloc(n);
    if (!p || fread(p, 1, n, f) != n) { fprintf(stderr, "%s: short read\n", path); fclose(f); free(p); return NULL; }
    fclose(f);
    return p;
}

/* the bar across the bottom of the image as seen (portrait: its last rows) */
#define BAR_H 72
static void draw_bar(uint8_t *img, int w, int h, const char *text)
{
    int len = (int)strlen(text);
    for (int y = h - BAR_H; y < h; y++)
        for (int x = 0; x < w; x++)
            img[(size_t)y * w + x] = epd_bar_ink(text, len, 6, w, BAR_H, x, y - (h - BAR_H)) ? 0xff : 0x00;
}

static int battery_celsius(void)             /* as glassine-epdd-desktop's */
{
    static const char *p[] = { "/sys/class/power_supply/qcom-battery/temp", "/sys/class/power_supply/battery/temp" };
    for (unsigned i = 0; i < sizeof p / sizeof *p; i++) {
        FILE *f = fopen(p[i], "r");
        char b[32];
        if (!f) continue;
        int ok = fgets(b, sizeof b, f) != NULL;
        fclose(f);
        if (ok) return atoi(b) / 10;
    }
    return 24;
}

/* panel pixel (x, y) of the image, rotated as for /dev/ebc */
static uint8_t px(const uint8_t *img, int portrait, int ccw, int x, int y)
{
    if (!portrait) return img[y * W + x];
    return ccw ? img[y + (W - 1 - x) * H] : img[(H - 1 - y) + x * H];
}

static int show_engine(const uint8_t *img, int portrait, int ccw, int mode)
{
    const char *hv = getenv("GLASSINE_EPDD_HV");
    epd_stream *s = epd_stream_open("/waveform/eink_waveform.wbf", battery_celsius(), hv ? atoi(hv) : 1);
    if (!s) { fprintf(stderr, "glassine-epd-show: no /dev/ebc and no engine\n"); return 1; }
    uint8_t *g = malloc((size_t)W * H);
    if (!g) return 1;
    epd_rect r = { 0, 0, W, H };
    memset(g, 0xff, (size_t)W * H);
    epd_stream_submit(s, r, EPD_INIT, 1, g, W);
    epd_stream_wait_idle(s);            /* a pending update may still read g (merges) */
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) g[y * W + x] = px(img, portrait, ccw, x, y);
    epd_stream_submit(s, r, mode, 1, g, W);
    epd_stream_save_levels(s, "/run/glassine-epdd-desktop.levels");      /* waits for both */
    epd_stream_close(s);
    free(g);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s IMAGE.pgm [waveform_mode [BAR TEXT]]\n", argv[0]); return 2; }
    int mode = argc > 2 ? atoi(argv[2]) : 2;
    const char *rot = getenv("GLASSINE_EPD_ROT");
    int ccw = !(rot && !strcmp(rot, "cw"));
    int iw, ih;
    uint8_t *img = read_pgm(argv[1], &iw, &ih);
    if (!img) return 1;
    int portrait = (iw == H && ih == W);
    if (!portrait && !(iw == W && ih == H)) {
        fprintf(stderr, "image must be %dx%d or %dx%d, got %dx%d\n", W, H, H, W, iw, ih);
        return 1;
    }
    if (argc > 3) draw_bar(img, iw, ih, argv[3]);

    if (access("/dev/ebc", F_OK)) return show_engine(img, portrait, ccw, mode);

    int fb = open("/dev/fb0", O_RDWR);
    if (fb < 0) fb = open("/dev/graphics/fb0", O_RDWR);
    if (fb < 0) { perror("open fb0"); return 1; }
    int ebc = open("/dev/ebc", O_RDWR);
    if (ebc < 0) { perror("open /dev/ebc"); return 1; }
    size_t sz = (size_t)W * H * 4;
    uint32_t *buf = mmap(NULL, sz, PROT_READ | PROT_WRITE, MAP_SHARED, ebc, 0);
    if (buf == MAP_FAILED) { perror("mmap /dev/ebc"); return 1; }

    memset(buf, 0xFF, sz);
    send(ebc, 2, 0);
    ioctl(fb, FBIOBLANK, FB_BLANK_POWERDOWN);
    ioctl(fb, FBIOBLANK, FB_BLANK_UNBLANK);

    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            uint8_t g = px(img, portrait, ccw, x, y);
            buf[y * W + x] = 0xFF000000u | g << 16 | g << 8 | g;
        }
    int rc = send(ebc, mode, 1);
    munmap(buf, sz);
    close(ebc);
    close(fb);
    return rc ? 1 : 0;
}
