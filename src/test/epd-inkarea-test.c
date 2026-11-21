/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-inkarea-test — whose stroke a pen contact is (epd-inkarea.h), as glassine-epdd-desktop's
 * pen thread asks it: the start decides whether a stroke inks and whose it is; a stroke
 * started in a writing region inks over its window's visible region, past the region's
 * edge; one started in another ink area inks in its owner's areas only; one started
 * outside every area never inks. Parity: for blocks without regions, every report the
 * stroke rule inks the per-point rule it replaced inked too, and a stroke that stays in
 * one area inks exactly as before. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../epd-inkarea.h"

#define PH 1404                 /* portrait px, as the daemon */
#define PW 1872
#define CW (PH / IA_C)
#define CH (PW / IA_C)

static ia_block blk;
static uint8_t slot[CW * CH], own[CW * CH];
static uint64_t keys[IA_MAXO + 1];
static int fails;

/* the daemon's predicate (ink_rect_inks): S 1/2, or a per-app area of an ink=on app */
static int inks(const ia_rect *r, void *data)
{
    (void)data;
    return r->sdk || !strcmp(r->app, "inkon");
}

/* a block (lines separated by \n) into the masks, as ink_line_cmd + ink_rebuild */
static void load(const char *text)
{
    char line[512];
    const char *t = text;
    ia_begin(&blk);
    while (*t) {
        size_t n = strcspn(t, "\n");
        snprintf(line, sizeof line, "%.*s", (int)n, t);
        t += n + (t[n] == '\n');
        if (!strcmp(line, "ink begin") || !strcmp(line, "ink end") || !strncmp(line, "ink brush ", 10)) continue;
        if (!ia_line(&blk, line, (box){ 0, 0, PH, PW })) { printf("FAIL: line not taken: %s\n", line); fails++; }
    }
    ia_owners(&blk, own, CW, CH, inks, NULL);
    memcpy(keys, blk.key, sizeof keys);
    memset(slot, 0, sizeof slot);
    for (int i = 0; i < blk.n; i++)
        if (!blk.r[i].vis && inks(&blk.r[i], NULL)) ia_fill(slot, CW, CH, blk.r[i].b, (uint8_t)(1 + blk.r[i].br));
}

/* the rule the daemon had: a report inks where the mask has an area */
static int old_rule(int x, int y)
{
    long o = ia_cell(x, y, CW, CH);
    return o >= 0 && slot[o];
}

/* a contact as the pen thread runs it */
typedef struct { int st; ia_owner ow; } contact;
static int report(contact *c, int x, int y)
{
    long o = ia_cell(x, y, CW, CH);
    if (!c->st) c->st = o >= 0 && ia_start(slot[o], own[o], keys, &c->ow) ? 1 : -1;
    return c->st > 0 && o >= 0 && ia_cont(slot[o], own[o], keys, &c->ow);
}

/* a straight piece of a stroke, 2 px apart: the reports that ink */
static int drag(contact *c, int x0, int y0, int x1, int y1, int *total)
{
    int n = abs(x1 - x0) > abs(y1 - y0) ? abs(x1 - x0) : abs(y1 - y0), in = 0;
    n = n / 2 + 1;
    for (int i = 0; i <= n; i++) in += report(c, x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n);
    if (total) *total = n + 1;
    return in;
}

/* every report of the piece inks */
static int every(contact *c, int x0, int y0, int x1, int y1)
{
    int n, in = drag(c, x0, y0, x1, y1, &n);
    return in == n;
}

static void expect(int got, int want, const char *what)
{
    if (got != want) { printf("FAIL: %s: %d, expected %d\n", what, got, want); fails++; }
    else printf("ok: %s\n", what);
}

/* window "planner" (region mode) at the top, a keyboard popup over it, an ink=on window and a
 * plain one below. The planner's visible region is its box less the popup. */
static const char *A =
    "ink begin\n"
    "ink brush 1 0 0.00 0.00 1.000 1 0.00 0.00 1.000\n"
    "ink win 812.5\n"
    "ink r 2 1 400 400 200 200 input-method\n"
    "ink win 700.11\n"
    "ink vis 0 0 1404 400\n"
    "ink vis 0 400 400 200\n"
    "ink vis 600 400 804 200\n"
    "ink vis 0 600 1404 600\n"
    "ink r 3 1 800 800 400 300 planner\n"
    "ink r 1 2 100 100 300 200 planner\n"
    "ink win 700.40\n"
    "ink 0 0 1200 1404 400 inkon\n"
    "ink win 700.52\n"
    "ink 0 0 1600 1404 272 plain\n"
    "ink end\n";

/* the same windows, listed in another order; the region moved right */
static const char *A2 =
    "ink begin\n"
    "ink win 700.40\n"
    "ink 0 0 1200 1404 400 inkon\n"
    "ink win 700.11\n"
    "ink vis 0 0 1404 400\n"
    "ink vis 0 400 400 200\n"
    "ink vis 600 400 804 200\n"
    "ink vis 0 600 1404 600\n"
    "ink r 1 2 300 100 300 200 planner\n"
    "ink win 812.5\n"
    "ink r 2 1 400 400 200 200 input-method\n"
    "ink end\n";

/* the planner gone */
static const char *A3 =
    "ink begin\n"
    "ink win 700.40\n"
    "ink 0 0 1200 1404 400 inkon\n"
    "ink end\n";

/* an older compositor: no win lines, no regions */
static const char *OLD =
    "ink begin\n"
    "ink r 1 1 0 0 700 500 a\n"
    "ink r 2 1 700 0 704 500 b\n"
    "ink 0 0 500 1404 500 inkon\n"
    "ink 0 0 1000 1404 872 plain\n"
    "ink end\n";

/* what sway's ink-popup test (case 10) sends for a window with one region; the
 * token and the numbers as glassine-ink-area.c writes them */
static const char *SWAY =
    "ink begin\n"
    "ink brush 1 0 0.00 0.00 1.000 1 0.00 0.00 1.000\n"
    "ink win 41.3\n"
    "ink vis 0 0 800 600\n"
    "ink r 1 2 20 20 200 100 ink-region\n"
    "ink end\n";

int main(void)
{
    contact c;

    load(A);
    expect(blk.no, 4, "A: four owners");
    /* 1: started in the region, out past its edge over the window, over the popup, back, into
     * the window below */
    c = (contact){ 0 };
    expect(every(&c, 200, 200, 380, 200), 1, "1 in the region: every report inks");
    expect(every(&c, 400, 200, 1300, 200), 1, "1 past the region's edge, over its window: inks");
    expect(drag(&c, 500, 450, 560, 560, NULL), 0, "1 over the popup that covers the window: no ink");
    expect(every(&c, 700, 700, 1000, 1000), 1, "1 back on the window (another area of it too): inks");
    expect(drag(&c, 700, 1300, 700, 1500, NULL), 0, "1 on the window below: no ink");
    expect(report(&c, 300, 300), 1, "1 back in the region: inks");
    /* 2: a drag from the window (no area there) into the region */
    c = (contact){ 0 };
    expect(drag(&c, 50, 50, 250, 250, NULL), 0, "2 started outside every area, into the region: no ink");
    expect(drag(&c, 250, 250, 1000, 1000, NULL), 0, "2 and nowhere after");
    /* 3: started in the planner's own canvas (S 1): its areas only */
    c = (contact){ 0 };
    expect(every(&c, 900, 900, 1190, 900), 1, "3 in the app's area: inks");
    expect(drag(&c, 1210, 900, 1390, 900, NULL), 0, "3 past the area's edge: no ink");
    expect(every(&c, 1100, 950, 900, 950), 1, "3 back in: inks");
    expect(report(&c, 200, 200), 1, "3 in the same window's region (its area too): inks");
    expect(drag(&c, 100, 1300, 1300, 1300, NULL), 0, "3 into the ink=on window below: no ink");
    /* 4: started in the ink=on window below, up into the planner's region */
    c = (contact){ 0 };
    expect(every(&c, 100, 1300, 100, 1500), 1, "4 the ink=on window: inks");
    expect(drag(&c, 200, 1190, 200, 150, NULL), 0, "4 up into the planner and its region: no ink");
    /* 5: started in the popup: popup only */
    c = (contact){ 0 };
    expect(every(&c, 420, 420, 580, 580), 1, "5 the popup's area: inks");
    expect(drag(&c, 620, 620, 700, 700, NULL), 0, "5 off the popup onto the window under it: no ink");
    /* 6: the plain window (no ink=on, no areas): nothing */
    c = (contact){ 0 };
    expect(drag(&c, 100, 1700, 1300, 1700, NULL), 0, "6 the plain window: no ink");
    expect(drag(&c, 100, 1300, 100, 1400, NULL), 0, "6 and not in the ink=on window it crosses into");
    /* 7: new areas mid-stroke (sway resends): the owner is the same window by its key */
    c = (contact){ 0 };
    expect(report(&c, 200, 200), 1, "7 started in the region");
    load(A2);
    expect(every(&c, 700, 250, 1300, 250), 1, "7 owners renumbered, the region moved: the stroke goes on");
    expect(drag(&c, 700, 1300, 700, 1500, NULL), 0, "7 still not on the other window");
    load(A3);
    expect(report(&c, 700, 250), 0, "7 the window gone: no ink");
    /* 8: an older compositor's block: one owner per app, the start rule all the same */
    load(OLD);
    c = (contact){ 0 };
    expect(every(&c, 100, 100, 690, 100), 1, "8 old block: area a inks");
    expect(drag(&c, 710, 100, 1300, 100, NULL), 0, "8 old block: not in b, another app's area");
    c = (contact){ 0 };
    expect(drag(&c, 100, 1100, 100, 400, NULL), 0, "8 old block: from a plain window into ink=on and a: no ink");
    /* 9: sway's block */
    load(SWAY);
    expect(blk.no == 1 && blk.n == 2 && blk.r[0].vis && blk.r[1].sdk == 2 && blk.r[1].br == 1 &&
           !strcmp(blk.r[1].app, "ink-region"), 1, "9 sway's lines: one owner, its visible region, a region with brush 1");
    c = (contact){ 0 };
    expect(every(&c, 50, 50, 210, 110), 1, "9 in the region: inks");
    expect(every(&c, 230, 110, 790, 590), 1, "9 past it, over the window: inks");
    c = (contact){ 0 };
    expect(drag(&c, 300, 300, 100, 60, NULL), 0, "9 from the window into the region: no ink");

    /* parity with the per-point rule, random blocks without regions (old and new form) */
    srand(7);
    int sub = 0, same = 0, strokes = 0;
    for (int it = 0; it < 400; it++) {
        char text[8192];
        int len = snprintf(text, sizeof text, "ink begin\n");
        int nw = 1 + rand() % 6, wl = rand() % 2;
        for (int w = 0; w < nw; w++) {
            if (wl) len += snprintf(text + len, sizeof text - len, "ink win %d.%d\n", 100 + w, w * 3);
            int nr = 1 + rand() % 3;
            for (int r = 0; r < nr; r++) {
                int x = rand() % PH, y = rand() % PW, ww = 4 + rand() % 500, hh = 4 + rand() % 500;
                int s = rand() % 3 == 0 ? 0 : 1;
                len += snprintf(text + len, sizeof text - len, "ink r %d %d %d %d %d %d %s\n", rand() % 4, s, x, y, ww, hh,
                                s ? (w % 2 ? "app b" : "app a") : (rand() % 2 ? "inkon" : "plain"));
            }
        }
        snprintf(text + len, sizeof text - len, "ink end\n");
        load(text);
        for (int k = 0; k < 20; k++) {
            /* a random stroke: no report the new rule inks that the old one didn't */
            c = (contact){ 0 };
            int x = rand() % PH, y = rand() % PW;
            for (int s = 0; s < 40; s++) {
                int nx = x + rand() % 81 - 40, ny = y + rand() % 81 - 40;
                if (report(&c, nx, ny) && !old_rule(nx, ny)) sub++;
                x = nx; y = ny;
            }
            /* a stroke inside one rect that inks: the same verdict at every report */
            int i = rand() % blk.n;
            box b = blk.r[i].b;
            if (b.x1 - b.x0 < 8 || b.y1 - b.y0 < 8 || !inks(&blk.r[i], NULL)) continue;
            c = (contact){ 0 };
            strokes++;
            for (int s = 0; s < 30; s++) {
                int px = b.x0 + rand() % (b.x1 - b.x0), py = b.y0 + rand() % (b.y1 - b.y0);
                long o = ia_cell(px, py, CW, CH);
                /* (a later rect over this one, or the cell's centre outside: the old rule decides there too) */
                if (o < 0 || own[o] != own[ia_cell(b.x0 + (b.x1 - b.x0) / 2, b.y0 + (b.y1 - b.y0) / 2, CW, CH)]) continue;
                if (report(&c, px, py) != old_rule(px, py) && c.st > 0) same++;
            }
        }
    }
    expect(sub, 0, "parity: without regions the stroke rule inks only where the per-point rule did");
    expect(same, 0, "parity: a stroke inside one area inks as before");
    printf("(%d strokes inside one area)\n", strokes);
    if (fails) { printf("epd-inkarea-test: %d FAILED\n", fails); return 1; }
    printf("epd-inkarea-test: all passed\n");
    return 0;
}
