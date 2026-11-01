/* epd-comp-test.c — the compositor (epd-comp.c) against a per-pixel reference
 * on a known scene, on a real X server (Xvfb: run by epd-comp-test.sh).
 *
 * Scene (portrait W x H, bottom to top): a full-screen desktop, an opaque app
 * window with a bordered child-less frame, an ARGB override-redirect popup
 * with alpha under and over the cut, a shaped window whose pixmap is black
 * outside its shape, and an unmapped window. Checked:
 * - comp_read over the whole screen and over odd sub-rects == the reference,
 * in the panel frame layout g[(px - x0) * h + (y1 - 1 - py)];
 * - damage: drawing into a window yields a dirty box covering the drawing;
 * - unmap / move: old and new areas dirty, the result follows;
 * - gpu mode on an X without DRI3 / with the marker left over: falls back to
 * the cpu backend (the marker path is compiled in by the script). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/extensions/shape.h>
#include "../epd-comp.h"

#define W 1404
#define H 1872
#define PW 1872
#define PH 1404

static int fails;
static int lenient(Display *d, XErrorEvent *e) { (void)d; (void)e; return 0; }   /* as glassine-epdd-desktop */
#define CHECK(c, ...) do { if (!(c)) { fails++; fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__); fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } } while (0)

static Display *dpy;
static Window root;
static uint8_t lut[256];
static uint8_t ref[H][W];                       /* the reference screen, grey */

static unsigned luma(unsigned r, unsigned g, unsigned b) { return (r * 77 + g * 150 + b * 29) >> 8; }

/* reference painting: an opaque rect of colour rgb */
static void ref_fill(int x, int y, int w, int h, unsigned r, unsigned g, unsigned b)
{
    for (int yy = y; yy < y + h; yy++)
        for (int xx = x; xx < x + w; xx++)
            if (xx >= 0 && yy >= 0 && xx < W && yy < H) ref[yy][xx] = lut[luma(r, g, b)];
}

/* premultiplied ARGB pixel over the reference, as the compositor: a < 128 left
 * out (cut 0.5), else un-premultiplied and opaque */
static void ref_argb(int x, int y, int w, int h, unsigned a, unsigned r, unsigned g, unsigned b)
{
    if (a < 128) return;
    unsigned R = (r * 510 + a) / (2 * a), G = (g * 510 + a) / (2 * a), B = (b * 510 + a) / (2 * a);
    if (R > 255) R = 255;
    if (G > 255) G = 255;
    if (B > 255) B = 255;
    ref_fill(x, y, w, h, R, G, B);
}

static void fill(Window w, GC gc, unsigned long px, int x, int y, int ww, int hh)
{
    XSetForeground(dpy, gc, px);
    XFillRectangle(dpy, w, gc, x, y, ww, hh);
}

static Window mk(int x, int y, int w, int h, int bw, int depth, Visual *vis, int override)
{
    XSetWindowAttributes a = { 0 };
    unsigned long m = CWOverrideRedirect | CWBorderPixel | CWColormap;
    a.override_redirect = override;
    a.border_pixel = 0;
    a.colormap = XCreateColormap(dpy, root, vis, AllocNone);
    return XCreateWindow(dpy, root, x, y, w, h, bw, depth, InputOutput, vis, m, &a);
}

static void pump(comp *c, boxlist *d)
{
    XSync(dpy, False);
    while (XPending(dpy)) {
        XEvent ev;
        XNextEvent(dpy, &ev);
        comp_event(c, &ev, d);
    }
}

/* compare rect b of the screen */
static int check_rect(comp *c, box b, const char *what)
{
    int w = b.y1 - b.y0, h = b.x1 - b.x0, bad = 0;
    uint8_t *g = malloc((size_t)w * h);
    if (comp_read(c, b, g) < 0) { CHECK(0, "%s: comp_read failed", what); free(g); return 1; }
    for (int py = b.y0; py < b.y1; py++)
        for (int px = b.x0; px < b.x1; px++) {
            uint8_t v = g[(size_t)(px - b.x0) * w + (b.y1 - 1 - py)];
            if (v != ref[py][px]) {
                if (bad < 5) fprintf(stderr, "  %s: (%d,%d) got %u want %u\n", what, px, py, v, ref[py][px]);
                bad++;
            }
        }
    if (bad) {
        int hist[256] = { 0 };
        for (int py = b.y0; py < b.y1; py++)
            for (int px = b.x0; px < b.x1; px++) hist[g[(size_t)(px - b.x0) * w + (b.y1 - 1 - py)]]++;
        for (int i = 0; i < 256; i++) if (hist[i]) fprintf(stderr, "    got %u: %d px\n", i, hist[i]);
    }
    CHECK(!bad, "%s: %d pixels differ", what, bad);
    free(g);
    return bad;
}

int main(void)
{
    if (!(dpy = XOpenDisplay(NULL))) { fprintf(stderr, "no X\n"); return 2; }
    root = DefaultRootWindow(dpy);
    XSetErrorHandler(lenient);
    int scr = DefaultScreen(dpy);
    CHECK(DisplayWidth(dpy, scr) == W && DisplayHeight(dpy, scr) == H, "screen must be %dx%d", W, H);
    XSelectInput(dpy, root, SubstructureNotifyMask);
    for (int i = 0; i < 256; i++) lut[i] = (uint8_t)(255 - (255 - i) * 7 / 8);   /* not the identity */

    Visual *v24 = DefaultVisual(dpy, scr);
    XVisualInfo vi32;
    CHECK(XMatchVisualInfo(dpy, scr, 32, TrueColor, &vi32), "no 32-bit visual");

    /* white where no window */
    memset(ref, lut[255], sizeof ref);

    comp *c = comp_open(dpy, root, W, H, PW, PH);
    CHECK(c != NULL, "comp_open");
    if (!c) return 1;
    CHECK(!strcmp(comp_renderer(c), "cpu"), "renderer %s", comp_renderer(c));
    comp_set_lut(c, lut);
    boxlist d = { 0 };

    /* desktop: full screen, grey 200, a darker band */
    Window desk = mk(0, 0, W, H, 0, 24, v24, 0);
    /* app: opaque, border 5 (border pixel 0 = black) */
    Window app = mk(100, 200, 400, 300, 5, 24, v24, 0);
    /* ARGB popup, override-redirect */
    Window pop = mk(300, 400, 200, 150, 0, 32, vi32.visual, 1);
    /* shaped: pixmap all black, shape = two rects */
    Window shp = mk(800, 900, 300, 300, 0, 24, v24, 0);
    XRectangle sr[2] = { { 0, 0, 100, 50 }, { 150, 100, 150, 200 } };
    XShapeCombineRectangles(dpy, shp, ShapeBounding, 0, 0, sr, 2, ShapeSet, Unsorted);
    /* unmapped: never shows */
    Window hid = mk(0, 0, 500, 500, 0, 24, v24, 0);
    (void)hid;
    XMapWindow(dpy, desk);
    XMapWindow(dpy, app);
    XMapWindow(dpy, pop);
    XMapWindow(dpy, shp);
    pump(c, &d);
    CHECK(d.n > 0, "mapping gave no dirty boxes");

    GC gc = XCreateGC(dpy, desk, 0, NULL);
    GC gc32 = XCreateGC(dpy, pop, 0, NULL);
    fill(desk, gc, 0xc8c8c8, 0, 0, W, H);
    fill(desk, gc, 0x404080, 0, 1000, W, 37);
    fill(app, gc, 0x0a141e, 0, 0, 400, 300);
    fill(app, gc, 0xffffff, 17, 23, 33, 11);
    /* popup: left half alpha 0x40 (left out), right half alpha 0xc0 */
    fill(pop, gc32, 0x40101010, 0, 0, 100, 150);
    fill(pop, gc32, 0xc0603020, 100, 0, 100, 150);
    fill(shp, gc, 0x000000, 0, 0, 300, 300);
    fill(shp, gc, 0x80ff80, 0, 0, 100, 50);
    fill(shp, gc, 0x2080ff, 150, 100, 150, 200);
    d.n = 0;
    pump(c, &d);

    ref_fill(0, 0, W, H, 0xc8, 0xc8, 0xc8);
    ref_fill(0, 1000, W, 37, 0x40, 0x40, 0x80);
    ref_fill(100, 200, 410, 310, 0, 0, 0);           /* border */
    ref_fill(105, 205, 400, 300, 0x0a, 0x14, 0x1e);
    ref_fill(105 + 17, 205 + 23, 33, 11, 0xff, 0xff, 0xff);
    ref_argb(300, 400, 100, 150, 0x40, 0x10, 0x10, 0x10);
    ref_argb(400, 400, 100, 150, 0xc0, 0x60, 0x30, 0x20);
    ref_fill(800, 900, 100, 50, 0x80, 0xff, 0x80);
    ref_fill(950, 1000, 150, 200, 0x20, 0x80, 0xff);

    check_rect(c, (box){ 0, 0, W, H }, "full screen");
    check_rect(c, (box){ 97, 198, 511, 513 }, "around the app");
    check_rect(c, (box){ 299, 399, 501, 551 }, "the popup");
    check_rect(c, (box){ 790, 890, 1111, 1213 }, "the shaped window");
    check_rect(c, (box){ 3, 1001, 4, 1002 }, "one pixel");
    check_rect(c, (box){ 120, 220, 160, 240 }, "inside the app (covered skip)");

    /* damage: a new rect in the app */
    d.n = 0;
    fill(app, gc, 0x808080, 200, 100, 50, 40);
    pump(c, &d);
    box want = { 105 + 200, 205 + 100, 105 + 250, 205 + 140 };
    int covered = 0;
    for (int i = 0; i < d.n; i++) if (box_inside(want, d.b[i])) covered = 1;
    CHECK(covered, "damage box does not cover the drawing (%d boxes)", d.n);
    ref_fill(want.x0, want.y0, 50, 40, 0x80, 0x80, 0x80);
    check_rect(c, (box){ 100, 200, 510, 510 }, "after drawing");

    /* move the popup: both areas dirty, the desktop shows again */
    d.n = 0;
    XMoveWindow(dpy, pop, 600, 1300);
    pump(c, &d);
    int old_dirty = 0, new_dirty = 0;
    for (int i = 0; i < d.n; i++) {
        if (box_inside((box){ 300, 400, 500, 550 }, d.b[i])) old_dirty = 1;
        if (box_inside((box){ 600, 1300, 800, 1450 }, d.b[i])) new_dirty = 1;
    }
    CHECK(old_dirty && new_dirty, "move: old %d new %d", old_dirty, new_dirty);
    ref_fill(300, 400, 200, 150, 0xc8, 0xc8, 0xc8);   /* the desktop under it */
    ref_fill(100, 200, 410, 310, 0, 0, 0);            /* the app over it again */
    ref_fill(105, 205, 400, 300, 0x0a, 0x14, 0x1e);
    ref_fill(105 + 17, 205 + 23, 33, 11, 0xff, 0xff, 0xff);
    ref_fill(want.x0, want.y0, 50, 40, 0x80, 0x80, 0x80);
    ref_argb(600, 1300, 100, 150, 0x40, 0x10, 0x10, 0x10);
    ref_argb(700, 1300, 100, 150, 0xc0, 0x60, 0x30, 0x20);
    check_rect(c, (box){ 0, 0, W, H }, "after the move");

    /* unmap the app: the desktop */
    d.n = 0;
    XUnmapWindow(dpy, app);
    pump(c, &d);
    ref_fill(100, 200, 410, 310, 0xc8, 0xc8, 0xc8);
    check_rect(c, (box){ 90, 190, 520, 520 }, "after unmap");

    /* resize the shaped window's shape away: nothing of it */
    XShapeCombineRectangles(dpy, shp, ShapeBounding, 0, 0, NULL, 0, ShapeSet, Unsorted);
    pump(c, &d);
    ref_fill(800, 900, 300, 300, 0xc8, 0xc8, 0xc8);
    ref_fill(800, 1000, 300, 37, 0x40, 0x40, 0x80);  /* the desktop's band */
    check_rect(c, (box){ 790, 890, 1111, 1213 }, "shaped to nothing");

    /* a window destroyed between the damage and the read */
    XDestroyWindow(dpy, pop);
    XSync(dpy, False);
    ref_fill(600, 1300, 200, 150, 0xc8, 0xc8, 0xc8);
    {   /* its named pixmap still holds the last picture: either is right, no error */
        uint8_t *g = malloc(220 * 170);
        CHECK(comp_read(c, (box){ 590, 1290, 810, 1460 }, g) == 0, "destroyed, events not read yet");
        free(g);
    }
    pump(c, &d);
    check_rect(c, (box){ 0, 0, W, H }, "after destroy");

    char st[256];
    comp_stats(c, st, sizeof st);
    fprintf(stderr, "%s\n", st);
    comp_close(c);

    /* gpu mode here (Xvfb: no DRI3 / llvmpipe): the cpu composites */
    setenv("GLASSINE_EPDD_COMP", "gpu", 1);
    c = comp_open(dpy, root, W, H, PW, PH);
    CHECK(c && !strcmp(comp_renderer(c), "cpu"), "gpu mode without a GPU: %s", c ? comp_renderer(c) : "NULL");
    if (c) {
        comp_set_lut(c, lut);
        pump(c, &d);
        check_rect(c, (box){ 0, 0, W, H }, "gpu mode fallback");
        comp_close(c);
    }
    CHECK(access(getenv("GPU_MARKER_PATH"), F_OK) != 0, "marker left behind by a fallback");
    /* the marker from an unclean end: gpu refused, cpu */
    FILE *f = fopen(getenv("GPU_MARKER_PATH"), "w");
    if (f) fclose(f);
    c = comp_open(dpy, root, W, H, PW, PH);
    CHECK(c && !strcmp(comp_renderer(c), "cpu"), "marker present: %s", c ? comp_renderer(c) : "NULL");
    if (c) comp_close(c);
    CHECK(access(getenv("GPU_MARKER_PATH"), F_OK) == 0, "the marker must stay until deleted");
    setenv("GLASSINE_EPDD_COMP", "off", 1);
    CHECK(comp_open(dpy, root, W, H, PW, PH) == NULL, "off");

    fprintf(stderr, fails ? "epd-comp-test: %d FAILED\n" : "epd-comp-test: all passed\n", fails);
    return fails ? 1 : 0;
}
