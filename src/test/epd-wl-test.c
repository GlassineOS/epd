/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-wl-test — the lazy frame path of epd-wl.c : with a damage
 * callback set, a frame's damage is reported at its arrival and its pixels are
 * converted only by epd_wl_flush / epd_wl_done; in runs glassine-epdd-desktop answers NEXT
 * (sway draws on, the buffer stays locked) and a frame replaced before a read
 * is answered DONE unread, its damage merged into the newer frame's convert.
 * A fake backend (this process, the other end of the socket) sends HELLO, shm
 * buffers and frames; checked: what the backend gets back (NEXT/DONE), the
 * damage and convert calls, and that the converted picture is the newest
 * buffer's. Also: a backend without HELLO (NEXT falls back to DONE), and no
 * damage callback (the old eager path). The pointer : CURSOR_IMG /
 * CURSOR_POS into one callback per dispatch, the box and sprite turned
 * portrait the way the frames' rects are (both layouts). exit 0 = pass */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/un.h>
#include "../epd-wl.h"
#include "../glassine-epd-output-proto.h"

#define W 64
#define H 48
static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; fprintf(stderr, "FAIL: " __VA_ARGS__); fputc('\n', stderr); } } while (0)

static uint32_t img[W * H];             /* what the convert delivered */
static int converts, dmgs, dmg_px;
static void rect_fn(void *ctx, const uint32_t *px, int stride, int x, int y, int w, int h)
{
    (void)ctx;
    converts++;
    for (int r = 0; r < h; r++) memcpy(&img[(y + r) * W + x], px + (size_t)r * stride, w * 4);
}
static epd_wl_rect last_dmg;
static void dmg_fn(void *ctx, const epd_wl_rect *r, int n)
{
    (void)ctx;
    dmgs++;
    for (int k = 0; k < n; k++) dmg_px += r[k].w * r[k].h;
    if (n) last_dmg = r[n - 1];
}
/* grey (R8 panel-order) rects: the rect, and its pixels = the test pattern */
static int greys, gr_ok;
static epd_wl_rect gr;
static void grey_fn(void *ctx, const uint8_t *g, int stride, int x, int y, int w, int h)
{
    (void)ctx;
    greys++;
    gr = (epd_wl_rect){ x, y, w, h };
    gr_ok = 1;
    for (int r = 0; r < h; r++)
        for (int c = 0; c < w; c++)
            if (g[(size_t)r * stride + c] != (uint8_t)((x + c) * 7 + (y + r) * 3)) gr_ok = 0;
}

/* the pointer callback: how often, the last one (its sprite copied) */
static int curs;
static epd_wl_cursor cur;
static uint32_t cur_px[16 * 16];
static void cur_fn(void *ctx, const epd_wl_cursor *c)
{
    (void)ctx;
    curs++;
    cur = *c;
    if (c->on && c->r.w * c->r.h <= 16 * 16) memcpy(cur_px, c->px, (size_t)c->r.w * c->r.h * 4);
}

static int be;                          /* the fake backend's socket */
static uint32_t *buf[3];
static void be_send(const struct glassine_epd_msg *m, int fd)
{
    struct iovec iov = { (void *)m, sizeof *m };
    union { char b[CMSG_SPACE(sizeof(int))]; struct cmsghdr a; } c;
    struct msghdr mh = { .msg_iov = &iov, .msg_iovlen = 1 };
    if (fd >= 0) {
        mh.msg_control = c.b; mh.msg_controllen = sizeof c.b;
        struct cmsghdr *cm = CMSG_FIRSTHDR(&mh);
        cm->cmsg_level = SOL_SOCKET; cm->cmsg_type = SCM_RIGHTS; cm->cmsg_len = CMSG_LEN(sizeof(int));
        memcpy(CMSG_DATA(cm), &fd, sizeof fd);
    }
    if (sendmsg(be, &mh, 0) < 0) perror("sendmsg");
}
/* the answers that came back since the last call: "D3 N4 ..." */
static void answers(char *o, size_t len)
{
    o[0] = 0;
    struct glassine_epd_msg m;
    while (recv(be, &m, sizeof m, MSG_DONTWAIT) > 0) {
        size_t l = strlen(o);
        snprintf(o + l, len - l, "%s%c%u", l ? " " : "", m.type == GLASSINE_EPD_DONE ? 'D' : m.type == GLASSINE_EPD_NEXT ? 'N' : '?', m.seq);
    }
}
static void frame(epd_wl *s, uint32_t id, uint32_t seq, int n, const struct glassine_epd_rect *r)
{
    struct glassine_epd_msg m = { .type = GLASSINE_EPD_FRAME, .id = id, .seq = seq, .nrect = n };
    memcpy(m.rect, r, n * sizeof *r);
    be_send(&m, -1);
    usleep(2000);
    epd_wl_dispatch(s);
}
/* buffer b holds the whole current picture: pattern p in r (the rest as before) */
static uint32_t pic[W * H];
static void paint(int b, struct glassine_epd_rect r, uint32_t p)
{
    for (int y = r.y; y < r.y + r.h; y++)
        for (int x = r.x; x < r.x + r.w; x++) pic[y * W + x] = p + (uint32_t)(y * W + x);
    memcpy(buf[b], pic, sizeof pic);
}

static epd_wl *setup(const char *path, int hello, int lazy)
{
    epd_wl *s = epd_wl_open(path, W, H, rect_fn, NULL);
    if (!s) { fprintf(stderr, "epd-wl-test: no socket\n"); exit(1); }
    if (lazy) epd_wl_set_damage(s, dmg_fn, NULL);
    be = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
    struct sockaddr_un sa = { .sun_family = AF_UNIX };
    snprintf(sa.sun_path, sizeof sa.sun_path, "%s", path);
    if (connect(be, (struct sockaddr *)&sa, sizeof sa)) { perror("connect"); exit(1); }
    usleep(2000);
    epd_wl_dispatch(s);                 /* accept */
    /* hello 2: also LUM (the grey output by luminance) */
    if (hello) { struct glassine_epd_msg m = { .type = GLASSINE_EPD_HELLO, .seq = GLASSINE_EPD_CAP_NEXT | (hello == 2 ? GLASSINE_EPD_CAP_LUM : 0) }; be_send(&m, -1); }
    for (int b = 0; b < 3; b++) {
        int fd = memfd_create("epd-wl-test", MFD_CLOEXEC);
        if (fd < 0 || ftruncate(fd, W * H * 4)) { perror("memfd"); exit(1); }
        buf[b] = mmap(NULL, W * H * 4, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        struct glassine_epd_msg m = { .type = GLASSINE_EPD_BUF, .id = b + 1, .width = W, .height = H, .stride = W * 4,
                                 .format = 0x34325258u };
        be_send(&m, fd);
        close(fd);
    }
    usleep(2000);
    epd_wl_dispatch(s);
    memset(img, 0, sizeof img); memset(pic, 0, sizeof pic);
    converts = dmgs = dmg_px = 0;
    return s;
}
static void teardown(epd_wl *s)
{
    close(be);
    epd_wl_close(s);
    for (int b = 0; b < 3; b++) munmap(buf[b], W * H * 4);
}

int main(void)
{
    char path[64], a[128];
    snprintf(path, sizeof path, "/tmp/epd-wl-test.%d", (int)getpid());
    struct glassine_epd_rect all = { 0, 0, W, H }, r2 = { 4, 4, 10, 8 }, r3 = { 30, 20, 12, 6 }, r4 = { 50, 2, 6, 30 };

    /* lazy, a backend with NEXT */
    epd_wl *s = setup(path, 1, 1);
    CHECK(epd_wl_caps(s) == GLASSINE_EPD_CAP_NEXT, "caps after HELLO: %u", epd_wl_caps(s));
    paint(0, all, 0x100000);
    frame(s, 1, 1, 0, NULL);            /* the whole buffer */
    CHECK(dmgs == 1 && dmg_px == W * H && converts == 0, "frame 1: damage %d (%d px), converts %d (want 1, %d, 0)", dmgs, dmg_px, converts, W * H);
    epd_wl_next(s);                     /* a run's early answer */
    answers(a, sizeof a);
    CHECK(!strcmp(a, "N1"), "frame 1 answered '%s' (want N1)", a);
    paint(1, r2, 0x200000);
    frame(s, 2, 2, 1, &r2);             /* replaces frame 1 before a read */
    answers(a, sizeof a);
    CHECK(!strcmp(a, "D1"), "frame 2's arrival answered '%s' (want D1: frame 1 unread)", a);
    CHECK(dmgs == 2 && dmg_px == W * H + 10 * 8 && converts == 0, "frame 2: damage %d (%d px), converts %d (want 2, %d, 0)", dmgs, dmg_px, converts, W * H + 80);
    epd_wl_flush(s);                    /* a read */
    CHECK(converts > 0 && !memcmp(img, buf[1], sizeof img), "flush: %d converts, picture %s buffer 2's", converts, memcmp(img, buf[1], sizeof img) ? "!=" : "==");
    answers(a, sizeof a);
    CHECK(!a[0], "flush of an unanswered frame answered '%s' (want nothing)", a);
    int c = converts;
    epd_wl_done(s);
    answers(a, sizeof a);
    CHECK(!strcmp(a, "D2") && converts == c, "done after the read: '%s', %d more converts (want D2, 0)", a, converts - c);
    CHECK(epd_wl_skipped(s) == 1, "skipped count (want 1)");
    /* two NEXT'ed frames, the second replaces the first, then a read: one
     * convert of the newest buffer over both damages, its DONE from the flush */
    paint(2, r3, 0x300000);
    frame(s, 3, 3, 1, &r3);
    epd_wl_next(s);
    paint(0, r4, 0x400000);             /* buffer 1 again: the whole picture (as sway's swapchain) */
    frame(s, 1, 4, 1, &r4);
    epd_wl_next(s);
    answers(a, sizeof a);
    CHECK(!strcmp(a, "N3 D3 N4"), "frames 3, 4: '%s' (want N3 D3 N4)", a);
    c = converts;
    memset(img, 0, sizeof img);         /* so the check sees only this convert */
    memcpy(img, buf[1], sizeof img);    /* what the panel source had: frame 2 */
    epd_wl_flush(s);
    answers(a, sizeof a);
    CHECK(!strcmp(a, "D4"), "the flush of a NEXT'ed frame answered '%s' (want D4)", a);
    CHECK(converts > c && !memcmp(img, buf[0], sizeof img), "frames 3+4 converted %s buffer 1's picture (%d converts)",
          memcmp(img, buf[0], sizeof img) ? "!=" : "==", converts - c);
    epd_wl_flush(s);
    epd_wl_done(s);
    answers(a, sizeof a);
    CHECK(!a[0], "a second flush / done answered '%s' (want nothing)", a);
    teardown(s);

    /* lazy, an older backend (no HELLO): NEXT falls back to DONE, converted then */
    s = setup(path, 0, 1);
    CHECK(epd_wl_caps(s) == 0, "caps without HELLO: %u", epd_wl_caps(s));
    paint(0, all, 0x500000);
    frame(s, 1, 1, 0, NULL);
    CHECK(converts == 0, "old backend: converted at arrival (%d)", converts);
    epd_wl_next(s);
    answers(a, sizeof a);
    CHECK(!strcmp(a, "D1") && converts > 0 && !memcmp(img, buf[0], sizeof img), "old backend: next -> '%s', converts %d (want D1, > 0, the picture)", a, converts);
    teardown(s);

    /* no damage callback: the old eager path (converted at arrival) */
    s = setup(path, 2, 0);
    CHECK(epd_wl_caps(s) == (GLASSINE_EPD_CAP_NEXT | GLASSINE_EPD_CAP_LUM), "caps with LUM: %u", epd_wl_caps(s));
    paint(0, all, 0x600000);
    frame(s, 1, 1, 0, NULL);
    CHECK(converts > 0 && dmgs == 0 && !memcmp(img, buf[0], sizeof img), "eager: converts %d, damage calls %d (want > 0, 0)", converts, dmgs);
    epd_wl_next(s);
    answers(a, sizeof a);
    CHECK(!strcmp(a, "D1"), "eager: next -> '%s' (want D1)", a);
    teardown(s);

    /* an R8 buffer in panel order (H x W, sway's grey output).
     * Its FRAME rects are panel rects; the damage callback gets them turned
     * back to portrait (panel col c, row r = portrait (r, H - 1 - c)), the
     * grey callback the panel rect and its pixels */
    {
        epd_wl *g = epd_wl_open(path, W, H, rect_fn, NULL);
        epd_wl_set_damage(g, dmg_fn, NULL);
        epd_wl_set_grey(g, grey_fn);
        be = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
        struct sockaddr_un sa = { .sun_family = AF_UNIX };
        snprintf(sa.sun_path, sizeof sa.sun_path, "%s", path);
        if (connect(be, (struct sockaddr *)&sa, sizeof sa)) { perror("connect"); exit(1); }
        usleep(2000);
        epd_wl_dispatch(g);
        int st = H + 8;                 /* a padded stride, as GBM's */
        int fd = memfd_create("epd-wl-test-r8", MFD_CLOEXEC);
        if (fd < 0 || ftruncate(fd, st * W)) { perror("memfd"); exit(1); }
        uint8_t *gb = mmap(NULL, st * W, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        for (int y = 0; y < W; y++)
            for (int x = 0; x < H; x++) gb[y * st + x] = (uint8_t)(x * 7 + y * 3);
        struct glassine_epd_msg m = { .type = GLASSINE_EPD_BUF, .id = 9, .width = H, .height = W, .stride = st, .format = 0x20203852u };
        be_send(&m, fd);
        close(fd);
        struct glassine_epd_msg bad = { .type = GLASSINE_EPD_BUF, .id = 10, .width = W, .height = H, .stride = W, .format = 0x20203852u };
        int fd2 = memfd_create("epd-wl-test-bad", MFD_CLOEXEC);
        if (fd2 < 0 || ftruncate(fd2, W * H)) { perror("memfd"); exit(1); }
        be_send(&bad, fd2);                 /* R8 in portrait size: refused */
        close(fd2);
        usleep(2000);
        epd_wl_dispatch(g);
        dmgs = dmg_px = 0; greys = 0;
        struct glassine_epd_rect pr = { 5, 7, 11, 13 };     /* panel cols 5..15, rows 7..19 */
        frame(g, 9, 1, 1, &pr);
        /* portrait: x = rows 7..19 -> x 7, w 13; y = H - 16 .. H - 5 -> y H - 16, h 11 */
        CHECK(dmgs == 1 && last_dmg.x == 7 && last_dmg.y == H - 16 && last_dmg.w == 13 && last_dmg.h == 11,
              "grey damage %d,%d %dx%d (want 7,%d 13x11)", last_dmg.x, last_dmg.y, last_dmg.w, last_dmg.h, H - 16);
        epd_wl_flush(g);
        CHECK(greys == 1 && gr.x == 5 && gr.y == 7 && gr.w == 11 && gr.h == 13 && gr_ok,
              "grey rect %d: %d,%d %dx%d, pixels %s (want 1: 5,7 11x13, ok)", greys, gr.x, gr.y, gr.w, gr.h, gr_ok ? "ok" : "wrong");
        frame(g, 10, 2, 0, NULL);       /* the refused buffer: answered, nothing converted */
        epd_wl_done(g);
        CHECK(greys == 1, "a refused R8 buffer was converted");
        close(be);
        epd_wl_close(g);
        munmap(gb, st * W);
    }
    /* the pointer: a sprite in the backend's memfd (two slots), its hotspot,
     * its place; one callback per dispatch with the newest state */
    {
        s = setup(path, 1, 1);
        epd_wl_set_cursor(s, cur_fn, NULL);
        int fd = memfd_create("epd-wl-test-cur", MFD_CLOEXEC);
        if (fd < 0 || ftruncate(fd, 2 * GLASSINE_EPD_CUR_SLOT)) { perror("memfd"); exit(1); }
        uint8_t *cm = mmap(NULL, 2 * GLASSINE_EPD_CUR_SLOT, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        uint32_t *s1 = (uint32_t *)(cm + GLASSINE_EPD_CUR_SLOT), *s0 = (uint32_t *)cm;
        for (int v = 0; v < 3; v++)
            for (int u = 0; u < 4; u++) { s1[v * 6 + u] = 0xff000000u | v << 8 | u; s0[v * 4 + u] = 0x80000000u | v << 8 | u; }
        curs = 0;
        struct glassine_epd_msg m = { .type = GLASSINE_EPD_CURSOR_IMG, .width = 4, .height = 3, .stride = 24, .offset = GLASSINE_EPD_CUR_SLOT,
                                 .format = GLASSINE_EPD_CUR_PORTRAIT, .rect = { { 1, 2, 0, 0 } } };
        be_send(&m, fd);
        struct glassine_epd_msg pos = { .type = GLASSINE_EPD_CURSOR_POS, .rect = { { 10, 20, 0, 0 } } };
        be_send(&pos, -1);
        usleep(2000);
        epd_wl_dispatch(s);
        int px_ok = 1;
        for (int v = 0; v < 3; v++)
            for (int u = 0; u < 4; u++) px_ok &= cur_px[v * 4 + u] == (0xff000000u | v << 8 | u);
        CHECK(curs == 1 && cur.on && cur.img && cur.r.x == 9 && cur.r.y == 18 && cur.r.w == 4 && cur.r.h == 3 && px_ok,
              "pointer IMG+POS: %d calls, on %d img %d, %d,%d %dx%d, sprite %s (want 1, 1 1, 9,18 4x3, ok)",
              curs, cur.on, cur.img, cur.r.x, cur.r.y, cur.r.w, cur.r.h, px_ok ? "ok" : "wrong");
        pos.rect[0].x = 12; be_send(&pos, -1);
        pos.rect[0].x = 14; be_send(&pos, -1);  /* two moves, one dispatch: the newest */
        usleep(2000);
        epd_wl_dispatch(s);
        CHECK(curs == 2 && cur.on && !cur.img && cur.moved && cur.r.x == 13, "pointer POS x2: %d calls, img %d moved %d, x %d (want 2, 0 1, 13)", curs, cur.img, cur.moved, cur.r.x);
        epd_wl_dispatch(s);
        CHECK(curs == 2, "a dispatch without pointer news called back (%d)", curs);
        /* the grey output: sprite and place in panel order; the box must be the
         * one a FRAME rect over the same panel pixels gets */
        m.format = GLASSINE_EPD_CUR_PANEL; m.offset = 0; m.stride = 16; m.rect[0] = (struct glassine_epd_rect){ 0, 0, 0, 0 };
        be_send(&m, -1);
        pos.rect[0] = (struct glassine_epd_rect){ 5, 7, 0, 0 };
        be_send(&pos, -1);
        usleep(2000);
        epd_wl_dispatch(s);
        /* the R8 buffer for the comparison frame */
        int gfd = memfd_create("epd-wl-test-r8c", MFD_CLOEXEC);
        if (gfd < 0 || ftruncate(gfd, H * W)) { perror("memfd"); exit(1); }
        struct glassine_epd_msg gm = { .type = GLASSINE_EPD_BUF, .id = 7, .width = H, .height = W, .stride = H, .format = 0x20203852u };
        be_send(&gm, gfd);
        close(gfd);
        epd_wl_cursor pc = cur;
        struct glassine_epd_rect pr = { 5, 7, 4, 3 };
        dmgs = 0;
        usleep(2000);
        epd_wl_dispatch(s);
        frame(s, 7, 9, 1, &pr);
        px_ok = 1;
        for (int v = 0; v < 3; v++)             /* panel (u, v) = portrait (v, w - 1 - u): rows of h = 3 */
            for (int u = 0; u < 4; u++) px_ok &= cur_px[(3 - u) * 3 + v] == (0x80000000u | v << 8 | u);
        CHECK(pc.on && pc.img && pc.panel && pc.r.x == last_dmg.x && pc.r.y == last_dmg.y && pc.r.w == last_dmg.w && pc.r.h == last_dmg.h &&
              pc.r.x == 7 && pc.r.y == H - 9 && px_ok,
              "pointer, panel order: %d,%d %dx%d (a FRAME of the same pixels: %d,%d %dx%d; want 7,%d 3x4), sprite %s",
              pc.r.x, pc.r.y, pc.r.w, pc.r.h, last_dmg.x, last_dmg.y, last_dmg.w, last_dmg.h, H - 9, px_ok ? "ok" : "wrong");
        epd_wl_done(s);
        m.width = 0; m.height = 0;              /* hidden */
        be_send(&m, -1);
        usleep(2000);
        epd_wl_dispatch(s);
        CHECK(!cur.on, "pointer width 0: still on");
        m.width = 4; m.height = 3; m.stride = 16;
        be_send(&m, -1);
        usleep(2000);
        epd_wl_dispatch(s);
        CHECK(cur.on, "pointer shown again: off");
        int n = curs;
        close(be);                              /* the backend goes: so does its pointer */
        usleep(2000);
        epd_wl_dispatch(s);
        CHECK(curs == n + 1 && !cur.on, "backend gone: %d calls, on %d (want 1, 0)", curs - n, cur.on);
        epd_wl_close(s);
        for (int b = 0; b < 3; b++) munmap(buf[b], W * H * 4);
        munmap(cm, 2 * GLASSINE_EPD_CUR_SLOT);
        close(fd);
    }
    unlink(path);

    if (fails) { fprintf(stderr, "epd-wl: %d failures\n", fails); return 1; }
    printf("epd-wl: ok\n");
    return 0;
}
