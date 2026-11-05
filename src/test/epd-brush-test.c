/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* host test of epd-brush.c (the pen-preview brush renderer; epd-brush.h).
 * epd-brush-test the checks
 * epd-brush-test bench + us per dab (8/16/32/64 px disc, 32 px mask, dither on) */
#include "../epd-brush.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

extern long epdb_stats_dabs;
void epdb_sha256(const uint8_t *m, size_t len, uint8_t out[32]);

#define W 600
#define H 400
static uint8_t grey[W * H], halo[W * H], thr[64 * 64];
static uint32_t xrgb[W * H];
static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void clear(uint8_t v)
{
    memset(grey, v, sizeof grey);
    for (int i = 0; i < W * H; i++) xrgb[i] = 0xFF000000u | v * 0x010101u;
    memset(halo, 0, sizeof halo);
}
static epdb_target tgt(int dither, int hal)
{
    return (epdb_target){ grey, xrgb, W, H, W, hal ? halo : NULL, dither ? thr : NULL, 64, 64 };
}
static epd_brush *mk(const epdb_desc *d)
{
    static uint8_t buf[NB_HDR + 64 * 64 * 16];
    char err[128] = "";
    size_t n = epdb_encode(d, buf);
    epd_brush *b = epdb_load(buf, n, err, sizeof err);
    if (!b) printf("load: %s\n", err);
    return b;
}
static epdb_point pt(float x, float y, float p, double t) { return (epdb_point){ x, y, p, 0, 0, 0, t }; }

/* a stroke through pts (n points, 2.8 ms apart) */
static epdb_box stroke(const epd_brush *b, float dmin, float dmax, float pexp, const epdb_point *pts, int n,
                       uint32_t seed, const epdb_target *t, epdb_box *hb)
{
    epdb_stroke s;
    epdb_box acc = {0, 0, 0, 0};
    epdb_stroke_begin(&s, &pts[0], seed);
    for (int i = 1; i < n; i++) {
        epdb_box r = epdb_segment(b, dmin, dmax, pexp, &s, &pts[i], t, hb);
        if (r.x0 < r.x1) {
            if (acc.x0 >= acc.x1) acc = r;
            else {
                if (r.x0 < acc.x0) acc.x0 = r.x0;
                if (r.y0 < acc.y0) acc.y0 = r.y0;
                if (r.x1 > acc.x1) acc.x1 = r.x1;
                if (r.y1 > acc.y1) acc.y1 = r.y1;
            }
        }
    }
    return acc;
}

/* ---- the old renderer (glassine-epdd-desktop.c ink_disc / ink_line), on grey only ---- */
static void old_disc(uint8_t *g, int x, int y, float r)
{
    int ri = (int)ceilf(r);
    float rr = r * r + r * 0.5f;
    for (int dy = -ri; dy <= ri; dy++)
        for (int dx = -ri; dx <= ri; dx++) {
            if (dx * dx + dy * dy > rr) continue;
            int bx = x + dx, by = y + dy;
            if (bx < 0 || bx >= W || by < 0 || by >= H) continue;
            g[by * W + bx] = 0;
        }
}
static void old_line(uint8_t *g, int x0, int y0, int x1, int y1, float r)
{
    int dx = abs(x1 - x0), dy = abs(y1 - y0), n = dx > dy ? dx : dy;
    int ri = (int)ceilf(r), step = ri > 1 ? ri / 2 : 1;
    for (int i = 0; i <= n; i += step)
        old_disc(g, x0 + (x1 - x0) * i / (n ? n : 1), y0 + (y1 - y0) * i / (n ? n : 1), r);
    old_disc(g, x1, y1, r);
}

static int count_below(const uint8_t *g, int v)
{
    int n = 0;
    for (int i = 0; i < W * H; i++) n += g[i] < v;
    return n;
}

/* the extents of non-white pixels */
static void extent(int *ex, int *ey)
{
    int x0 = W, x1 = -1, y0 = H, y1 = -1;
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            if (grey[y * W + x] < 255) {
                if (x < x0) x0 = x;
                if (x > x1) x1 = x;
                if (y < y0) y0 = y;
                if (y > y1) y1 = y;
            }
    *ex = x1 - x0 + 1;
    *ey = y1 - y0 + 1;
}

static void t_file(void)
{
    /* SHA-256 known answers */
    uint8_t d[32];
    epdb_sha256((const uint8_t *)"abc", 3, d);
    CHECK(d[0] == 0xba && d[1] == 0x78 && d[31] == 0xad, "sha256(abc)");
    const char *m56 = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
    epdb_sha256((const uint8_t *)m56, 56, d);
    CHECK(d[0] == 0x24 && d[1] == 0x8d && d[31] == 0xc1, "sha256(56 bytes)");
    epdb_sha256((const uint8_t *)"", 0, d);
    CHECK(d[0] == 0xe3 && d[31] == 0x55, "sha256(empty)");

    /* roundtrip with every field set */
    static uint8_t tip[16 * 16 * 3], buf[NB_HDR + 64 * 64 * 16], buf2[sizeof buf];
    for (size_t i = 0; i < sizeof tip; i++) tip[i] = (uint8_t)(i * 37);
    epdb_desc s;
    epdb_desc_default(&s);
    s.kind = NB_INK; s.blend = NB_LIGHTEN; s.grey = 176; s.opacity = 100; s.angle_mode = NB_ANG_TILT;
    s.scatter = 4; s.hardness = 0.3f; s.spacing = 0.6f; s.elongation = 0.4f; s.angle_deg = -30;
    s.pos_jitter = 0.6f; s.size_jitter = 0.2f; s.op_jitter = 0.5f;
    s.curves_present = 0x0b;
    for (int c = 0; c < NB_NCURVE; c++) for (int i = 0; i < 16; i++) s.curve[c][i] = (uint8_t)(c * 50 + i * 3);
    s.tip_size = 16; s.tip_variants = 3; s.tip = tip;
    size_t n = epdb_encode(&s, buf);
    CHECK(n == NB_HDR + sizeof tip, "encode length %zu", n);
    CHECK(!memcmp(buf, "EPDB", 4) && buf[4] == 1 && buf[5] == 0, "magic/version");
    CHECK(buf[8] == 176 && buf[9] == 100 && buf[11] == 4 && buf[26] == 0x0b, "bytes");
    CHECK(buf[14] + 256 * buf[15] == 614, "spacing 0.6 -> %d", buf[14] + 256 * buf[15]);
    CHECK((int16_t)(buf[18] | buf[19] << 8) == -30 * 64, "angle");
    for (int i = 0; i < 16; i++) CHECK(buf[28 + 32 + i] == 0, "absent curve 2 written as 0");
    char id[17], id2[17];
    epdb_id(buf, n, id);
    CHECK(strlen(id) == 16 && strspn(id, "0123456789abcdef") == 16, "id %s", id);
    char err[128];
    epd_brush *b = epdb_load(buf, n, err, sizeof err);
    CHECK(b && epdb_kind(b) == NB_INK, "load: %s", b ? "" : err);
    CHECK(epdb_ref(b) == b, "ref");
    epdb_unref(b);
    epdb_unref(b);
    size_t n2 = epdb_encode(&s, buf2);
    epdb_id(buf2, n2, id2);
    CHECK(n2 == n && !memcmp(buf, buf2, n) && !strcmp(id, id2), "deterministic encode/id");
    buf2[100] ^= 1;
    epdb_id(buf2, n2, id2);
    CHECK(strcmp(id, id2), "id follows content");

    /* clamping */
    epdb_desc c;
    epdb_desc_default(&c);
    c.spacing = 10; c.elongation = 0; c.hardness = 3; c.pos_jitter = 9; c.scatter = 20; c.angle_deg = 450;
    c.size_jitter = -1; c.op_jitter = NAN;
    n = epdb_encode(&c, buf);
    CHECK(n == NB_HDR, "disc length");
    CHECK(buf[14] + 256 * buf[15] == 4096 && buf[16] + 256 * buf[17] == 3277 && buf[12] + 256 * buf[13] == 65535,
          "clamp spacing/elongation/hardness");
    CHECK(buf[20] + 256 * buf[21] == 2048 && buf[11] == 8, "clamp jitter/scatter");
    CHECK((int16_t)(buf[18] | buf[19] << 8) == 90 * 64, "angle wraps 450 -> 90");
    CHECK(buf[22] == 0 && buf[23] == 0 && buf[24] == 0 && buf[25] == 0, "clamp size/op jitter");
    b = epdb_load(buf, n, err, sizeof err);
    CHECK(b != NULL, "clamped loads: %s", err);
    epdb_unref(b);
    c.kind = 3;
    CHECK(epdb_encode(&c, buf) == 0, "encode rejects kind 3");
    epdb_desc_default(&c);
    c.tip_size = 65; c.tip = tip;
    CHECK(epdb_encode(&c, buf) == 0, "encode rejects tip 65");
    c.tip_size = 8; c.tip = NULL;
    CHECK(epdb_encode(&c, buf) == 0, "encode rejects a NULL tip");
    c.tip = tip; c.tip_variants = 17;
    CHECK(epdb_encode(&c, buf) == 0, "encode rejects 17 variants");

    /* rejection, one field at a time (from the valid roundtrip file) */
    n = epdb_encode(&s, buf);
    struct { int off; uint8_t v; const char *what; } bad[] = {
        { 0, 'X', "magic" }, { 4, 2, "version" }, { 5, 1, "version hi" }, { 6, 3, "kind" }, { 7, 3, "blend" },
        { 10, 3, "angle mode" }, { 11, 9, "scatter" }, { 14, 19, "spacing low" }, { 15, 16, "spacing high" },
        { 17, 0, "elongation" }, { 21, 8, "pos jitter" }, { 26, 0x10, "curves present" }, { 27, 1, "reserved 27" },
        { 28 + 32 + 5, 1, "absent curve set" }, { 92, 65, "tip size" }, { 93, 0, "variants 0" },
        { 93, 17, "variants 17" }, { 94, 1, "reserved 94" }, { 95, 1, "reserved 95" },
    };
    for (size_t i = 0; i < sizeof bad / sizeof *bad; i++) {
        memcpy(buf2, buf, n);
        if (bad[i].off == 14) buf2[15] = 0;            /* spacing 19 */
        if (bad[i].off == 15) buf2[14] = 1;            /* spacing 4097 */
        buf2[bad[i].off] = bad[i].v;
        b = epdb_load(buf2, n, err, sizeof err);
        CHECK(!b, "accepted bad %s", bad[i].what);
        epdb_unref(b);
    }
    CHECK(!epdb_load(buf, n - 1, err, sizeof err), "short by one");
    CHECK(!epdb_load(buf, n + 1, err, sizeof err), "long by one");
    CHECK(!epdb_load(buf, 50, err, sizeof err), "truncated header");
    CHECK(!epdb_load(NULL, 0, err, sizeof err), "NULL");
    /* the disc must say 1 variant */
    epdb_desc_default(&c);
    n = epdb_encode(&c, buf);
    buf[93] = 2;
    CHECK(!epdb_load(buf, n, err, sizeof err), "disc with 2 variants");
    /* a 64x64x16 tip: the largest file */
    static uint8_t big[64 * 64 * 16];
    c.tip = big; c.tip_size = 64; c.tip_variants = 16;
    n = epdb_encode(&c, buf);
    b = epdb_load(buf, n, err, sizeof err);
    CHECK(n == NB_HDR + sizeof big && b, "largest file");
    epdb_unref(b);
    epd_brush *e = epdb_disc(1), *p = epdb_disc(0);
    CHECK(epdb_kind(e) == NB_ERASE && epdb_kind(p) == NB_INK, "builtin discs");
    epdb_unref(e);
    epdb_unref(p);
    printf("file: ok\n");
}

static void t_old(void)
{
    /* a wavy polyline, integer points, ~3..6 px apart; d 3..7 by pressure (the default brush=3-7) */
    static uint8_t ref[W * H];
    epdb_point pts[400];
    int n = 0;
    for (int i = 0; i < 400; i++) {
        float x = 20 + i * 1.4f, y = 200 + 120 * sinf(i * 0.03f);
        pts[n++] = pt(roundf(x), roundf(y), 0.5f + 0.5f * sinf(i * 0.05f), i * 0.0028);
    }
    for (int dmax = 7; dmax <= 31; dmax += 12) {
        float dmin = dmax == 7 ? 3 : dmax / 2;
        memset(ref, 255, sizeof ref);
        for (int i = 1; i < n; i++) {
            float d = dmin + (dmax - dmin) * pts[i].pressure;
            old_line(ref, (int)pts[i - 1].x, (int)pts[i - 1].y, (int)pts[i].x, (int)pts[i].y, d / 2);
        }
        clear(255);
        epd_brush *b = epdb_disc(0);
        epdb_target t = tgt(0, 0);
        epdb_box bx = stroke(b, dmin, dmax, 1, pts, n, 1, &t, NULL);
        int a = count_below(ref, 128), c = count_below(grey, 128), x = 0, outside = 0;
        for (int i = 0; i < W * H; i++) {
            x += (ref[i] < 128) != (grey[i] < 128);
            if (grey[i] < 255 && (i % W < bx.x0 || i % W >= bx.x1 || i / W < bx.y0 || i / W >= bx.y1)) outside++;
        }
        int okv = 1;
        for (int i = 0; i < W * H; i++) okv &= grey[i] == 0 || grey[i] == 255;
        printf("old vs new (d %g-%d): old %d px, new %d px (%+.2f%%), differ %d px (%.2f%%)\n",
               dmin, dmax, a, c, 100.0 * (c - a) / a, x, 100.0 * x / a);
        CHECK(fabs((double)(c - a) / a) < 0.03, "pixel count within 3%%");
        CHECK((double)x / a < 0.10, "symmetric difference < 10%% (thin lines: all edge)");
        CHECK(okv, "binary output");
        CHECK(!outside, "box covers all ink (%d outside)", outside);
        int xok = 1;
        for (int i = 0; i < W * H; i++) xok &= xrgb[i] == (0xFF000000u | grey[i] * 0x010101u);
        CHECK(xok, "xrgb == grey");
        epdb_unref(b);
    }
}

static void random_bg(unsigned seed)
{
    srand(seed);
    for (int i = 0; i < W * H; i++) { grey[i] = (uint8_t)rand(); xrgb[i] = 0xFF000000u | grey[i] * 0x010101u; }
    memset(halo, 0, sizeof halo);
}

static void scribble(epdb_point *pts, int n, unsigned seed)
{
    srand(seed);
    float x = 300, y = 200;
    for (int i = 0; i < n; i++) {
        x += (rand() % 21) - 10; y += (rand() % 21) - 10;
        if (x < -20) x = -20;
        if (x > W + 20) x = W + 20;
        if (y < -20) y = -20;
        if (y > H + 20) y = H + 20;
        pts[i] = (epdb_point){ x, y, (rand() % 100) / 99.0f, (rand() % 121) - 60.0f, (rand() % 121) - 60.0f, 1,
                               i * 0.0028 };
    }
}

static void t_blend(void)
{
    static uint8_t before[W * H];
    static uint8_t tip[24 * 24];
    for (int i = 0; i < 24 * 24; i++) tip[i] = (uint8_t)rand();
    epdb_point pts[300];
    scribble(pts, 300, 7);
    for (int mode = 0; mode < 6; mode++) {        /* darken disc/mask, lighten, erase, replace check, NONE */
        epdb_desc d;
        epdb_desc_default(&d);
        d.hardness = 0.4f; d.grey = 128; d.opacity = 200; d.scatter = 2; d.pos_jitter = 0.5f;
        d.elongation = 0.5f; d.angle_mode = NB_ANG_DIRECTION;
        if (mode == 1) { d.tip = tip; d.tip_size = 24; d.tip_variants = 1; }
        if (mode == 2) d.blend = NB_LIGHTEN;
        if (mode == 3) d.kind = NB_ERASE;
        if (mode == 4) { d.blend = NB_REPLACE; d.grey = 77; d.opacity = 255; d.hardness = 1; d.scatter = 0; }
        if (mode == 5) d.kind = NB_NONE;
        epd_brush *b = mk(&d);
        random_bg(mode + 1);
        memcpy(before, grey, sizeof before);
        epdb_target t = tgt(0, 0);
        epdb_box bx = stroke(b, 4, 30, 1, pts, 300, 5, &t, NULL);
        int dark = 0, light = 0, changed = 0, outside = 0, x77 = 0, xbad = 0;
        for (int i = 0; i < W * H; i++) {
            dark += grey[i] < before[i];
            light += grey[i] > before[i];
            changed += grey[i] != before[i];
            x77 += grey[i] == 77;
            xbad += xrgb[i] != (0xFF000000u | grey[i] * 0x010101u);
            if (grey[i] != before[i] && (i % W < bx.x0 || i % W >= bx.x1 || i / W < bx.y0 || i / W >= bx.y1)) outside++;
        }
        CHECK(!xbad, "mode %d: xrgb follows grey", mode);
        CHECK(!outside, "mode %d: %d changed px outside the box", mode, outside);
        if (mode <= 1) CHECK(changed > 1000 && !light, "darken: %d lightened (of %d changed)", light, changed);
        if (mode == 2 || mode == 3) CHECK(changed > 1000 && !dark, "lighten/erase: %d darkened (of %d)", dark, changed);
        if (mode == 4) CHECK(x77 > 1000, "replace writes its grey");
        if (mode == 5) CHECK(!changed && bx.x0 >= bx.x1, "ink kind none draws nothing");
        epdb_unref(b);
    }
    /* no-op on white: erase and lighten write nothing, not even xrgb */
    for (int mode = 0; mode < 3; mode++) {
        epdb_desc d;
        epdb_desc_default(&d);
        if (mode == 0) d.kind = NB_ERASE;
        if (mode == 1) { d.blend = NB_LIGHTEN; d.grey = 200; d.hardness = 0.2f; }
        if (mode == 2) { d.grey = 255; }                    /* white ink, darken */
        epd_brush *b = mk(&d);
        memset(grey, 255, sizeof grey);
        for (int i = 0; i < W * H; i++) xrgb[i] = 0x12345678u;
        for (int dither = 0; dither < 2; dither++) {
            epdb_target t = tgt(dither, 0);
            epdb_box bx = stroke(b, 4, 30, 1, pts, 300, 5, &t, NULL);
            int bad = 0;
            for (int i = 0; i < W * H; i++) bad += grey[i] != 255 || xrgb[i] != 0x12345678u;
            CHECK(!bad && bx.x0 >= bx.x1, "no-op on white (mode %d dither %d): %d written", mode, dither, bad);
        }
        epdb_unref(b);
    }
    printf("blend: ok\n");
}

static void t_dither(void)
{
    epdb_point pts[300];
    scribble(pts, 300, 9);
    epdb_desc d;
    epdb_desc_default(&d);
    d.grey = 140; d.hardness = 0.3f; d.opacity = 180;
    epd_brush *b = mk(&d);
    clear(255);
    epdb_target t = tgt(1, 0);
    stroke(b, 6, 40, 1, pts, 300, 3, &t, NULL);
    int bad = 0, black = 0, xb = 0;
    for (int i = 0; i < W * H; i++) {
        bad += grey[i] != 0 && grey[i] != 255;
        black += grey[i] == 0;
        xb += xrgb[i] != (0xFF000000u | grey[i] * 0x010101u);
    }
    CHECK(!bad && black > 500 && !xb, "dither: %d non-binary, %d black, %d xrgb mismatches", bad, black, xb);
    epdb_unref(b);
    /* the threshold map is panel-position keyed: one flat grey dab = the map's pattern */
    epdb_desc_default(&d);
    d.grey = 100; d.blend = NB_REPLACE;
    b = mk(&d);
    clear(255);
    epdb_stroke s;
    epdb_point p0 = pt(300, 200, 1, 0);
    epdb_stroke_begin(&s, &p0, 1);
    epdb_segment(b, 40, 40, 1, &s, &p0, &t, NULL);
    int mism = 0;
    for (int y = 190; y < 210; y++)
        for (int x = 290; x < 310; x++)
            mism += grey[y * W + x] != (100 > thr[(y % 64) * 64 + x % 64] ? 255 : 0);
    CHECK(!mism, "dither follows thresh[(y%%th)*tw + x%%tw] (%d off)", mism);
    epdb_unref(b);
    /* the dither never undoes the blend: darken over greys (an app's
     * antialiased stroke) and over its own earlier dabs only darkens,
     * lighten only lightens (textured dabs whitened pixels the
     * previous dab had sent to black -> the engine held every pen piece) */
    for (int bl = NB_DARKEN; bl <= NB_LIGHTEN; bl++) {
        epdb_desc_default(&d);
        d.grey = bl == NB_DARKEN ? 77 : 200; d.blend = (uint8_t)bl; d.hardness = 0.2f; d.opacity = 200;
        b = mk(&d);
        for (int i = 0; i < W * H; i++) grey[i] = (uint8_t)(60 + (i * 7 + i / W * 13) % 140);
        for (int i = 0; i < W * H; i++) xrgb[i] = 0xFF000000u | grey[i] * 0x010101u;
        static uint8_t before[W * H];
        memcpy(before, grey, sizeof before);
        stroke(b, 20, 50, 1, pts, 300, 3, &t, NULL);
        int wrong = 0, changed = 0;
        for (int i = 0; i < W * H; i++) {
            wrong += bl == NB_DARKEN ? grey[i] > before[i] : grey[i] < before[i];
            changed += grey[i] != before[i];
        }
        CHECK(!wrong && changed > 500, "dither + %s over greys: %d pixels went the wrong way (%d changed)",
              bl == NB_DARKEN ? "darken" : "lighten", wrong, changed);
        epdb_unref(b);
    }
    printf("dither: ok\n");
}

static void t_halo(void)
{
    static uint8_t g1[W * H];
    static uint32_t x1[W * H];
    epdb_desc d;
    epdb_desc_default(&d);
    d.hardness = 0.1f;
    epd_brush *b = mk(&d);
    epdb_point p0 = pt(300.3f, 200.6f, 1, 0);
    for (int dither = 0; dither < 2; dither++) {
        /* one dab without the halo */
        clear(255);
        epdb_target t = tgt(dither, 0);
        epdb_stroke s;
        epdb_stroke_begin(&s, &p0, 1);
        epdb_segment(b, 30, 30, 1, &s, &p0, &t, NULL);
        memcpy(g1, grey, sizeof g1);
        memcpy(x1, xrgb, sizeof x1);
        /* with: the core now, the rest pending */
        clear(255);
        t = tgt(dither, 1);
        epdb_box hb;
        epdb_stroke_begin(&s, &p0, 1);
        epdb_box cb = epdb_segment(b, 30, 30, 1, &s, &p0, &t, &hb);
        int core = 0, pend = 0, full = 0, both = 0;
        for (int i = 0; i < W * H; i++) {
            core += grey[i] != 255;
            pend += halo[i] != 0;
            full += g1[i] != 255;
            both += halo[i] && grey[i] != 255;
        }
        CHECK(core > 0 && pend > 0 && core < full && !both, "halo split: core %d, pending %d, all %d", core, pend, full);
        CHECK(cb.x0 < cb.x1 && hb.x0 < hb.x1 && hb.x0 <= cb.x0 && hb.x1 >= cb.x1, "halo box around the core");
        epdb_box fb = epdb_halo_flush(&t, hb);
        int left = 0, diff = 0;
        for (int i = 0; i < W * H; i++) { left += halo[i] != 0; diff += grey[i] != g1[i] || xrgb[i] != x1[i]; }
        CHECK(!left && fb.x0 < fb.x1, "flush clears the halo");
        /* a pending level 254 (the 255 cap) is the only allowed difference */
        CHECK(diff <= (dither ? 0 : full / 50), "flushed = drawn at once (dither %d): %d px differ", dither, diff);
        epdb_box fb2 = epdb_halo_flush(&t, hb);
        CHECK(fb2.x0 >= fb2.x1, "second flush writes nothing");
    }
    epdb_unref(b);
    printf("halo: ok\n");
}

static void t_angle(void)
{
    epdb_desc d;
    epdb_desc_default(&d);
    d.elongation = 0.2f;
    int ex, ey;
    /* tilt: the major axis along the tilt direction */
    d.angle_mode = NB_ANG_TILT;
    epd_brush *b = mk(&d);
    epdb_target t = tgt(0, 0);
    for (int k = 0; k < 3; k++) {
        clear(255);
        epdb_point p = { 300, 200, 1, k == 0 ? 0 : k == 1 ? 40 : 0, k == 2 ? 40 : 0, 1, 0 };
        if (k == 0) p.has_tilt = 0;
        epdb_stroke s;
        epdb_stroke_begin(&s, &p, 1);
        epdb_segment(b, 60, 60, 1, &s, &p, &t, NULL);
        extent(&ex, &ey);
        if (k < 2) CHECK(ex > 3 * ey, "tilt %d (x / none): extent %dx%d", k, ex, ey);
        else CHECK(ey > 3 * ex, "tilt along y: extent %dx%d", ex, ey);
    }
    epdb_unref(b);
    /* direction: the first dab already follows the first segment */
    d.angle_mode = NB_ANG_DIRECTION;
    b = mk(&d);
    for (int k = 0; k < 2; k++) {
        clear(255);
        epdb_point p0 = pt(300, 200, 1, 0), p1 = pt(k ? 300 : 302, k ? 202 : 200, 1, 0.003);
        epdb_stroke s;
        epdb_stroke_begin(&s, &p0, 1);
        epdb_segment(b, 60, 60, 1, &s, &p1, &t, NULL);
        extent(&ex, &ey);
        if (!k) CHECK(ex > 3 * ey, "direction x: %dx%d", ex, ey);
        else CHECK(ey > 3 * ex, "direction y: %dx%d", ex, ey);
    }
    epdb_unref(b);
    /* a mask: a horizontal bar turned 90 degrees stands upright */
    static uint8_t bar[32 * 32];
    for (int y = 14; y < 18; y++) memset(bar + y * 32, 255, 32);
    epdb_desc_default(&d);
    d.tip = bar; d.tip_size = 32;
    for (int k = 0; k < 2; k++) {
        d.angle_deg = k ? 90 : 0;
        b = mk(&d);
        clear(255);
        epdb_point p0 = pt(300, 200, 1, 0);
        epdb_stroke s;
        epdb_stroke_begin(&s, &p0, 1);
        epdb_segment(b, 64, 64, 1, &s, &p0, &t, NULL);
        extent(&ex, &ey);
        if (!k) CHECK(ex >= 62 && ex <= 66 && ey >= 7 && ey <= 9, "mask bar 0: %dx%d", ex, ey);
        else CHECK(ey >= 62 && ey <= 66 && ex >= 7 && ex <= 9, "mask bar 90: %dx%d", ex, ey);
        epdb_unref(b);
    }
    printf("angle: ok\n");
}

static void t_scatter(void)
{
    static uint8_t g1[W * H];
    static uint8_t tip[16 * 16 * 4];
    for (size_t i = 0; i < sizeof tip; i++) tip[i] = (uint8_t)(rand() & 0xff);
    epdb_point pts[200];
    scribble(pts, 200, 11);
    epdb_desc d;
    epdb_desc_default(&d);
    d.scatter = 4; d.pos_jitter = 0.6f; d.size_jitter = 0.5f; d.op_jitter = 0.8f; d.opacity = 120; d.grey = 60;
    d.tip = tip; d.tip_size = 16; d.tip_variants = 4;
    epd_brush *b = mk(&d);
    epdb_target t = tgt(0, 0);
    clear(255);
    stroke(b, 8, 24, 1, pts, 200, 1234, &t, NULL);
    memcpy(g1, grey, sizeof g1);
    clear(255);
    stroke(b, 8, 24, 1, pts, 200, 1234, &t, NULL);
    CHECK(!memcmp(g1, grey, sizeof g1), "same seed, same pixels");
    clear(255);
    stroke(b, 8, 24, 1, pts, 200, 99, &t, NULL);
    CHECK(memcmp(g1, grey, sizeof g1), "another seed, other pixels");
    epdb_unref(b);
    printf("scatter: ok\n");
}

static void t_cap(void)
{
    epd_brush *b = epdb_disc(0);
    epdb_target t = tgt(0, 0);
    clear(255);
    /* a 64 px pen, one report 500 px further: <= NB_MAXDABS (+ the first) */
    epdb_point p0 = pt(10, 200, 1, 0), p1 = pt(510, 200, 1, 0.0028);
    epdb_stroke s;
    epdb_stroke_begin(&s, &p0, 1);
    long d0 = epdb_stats_dabs;
    epdb_segment(b, 64, 64, 1, &s, &p1, &t, NULL);
    long nd = epdb_stats_dabs - d0;
    CHECK(nd >= NB_MAXDABS - 1 && nd <= NB_MAXDABS + 1, "64 px dabs per call: %ld", nd);
    /* a 4 px pen, 120 px in one report: no gap along the line (the old ink_line didn't gap) */
    clear(255);
    p0 = pt(10, 100, 1, 0); p1 = pt(130, 160, 1, 0.0028);
    epdb_stroke_begin(&s, &p0, 1);
    d0 = epdb_stats_dabs;
    epdb_segment(b, 4, 4, 1, &s, &p1, &t, NULL);
    nd = epdb_stats_dabs - d0;
    int gaps = 0;
    for (int i = 0; i <= 120; i++) gaps += grey[(100 + i / 2) * W + 10 + i] != 0;
    CHECK(!gaps, "4 px pen: %d gaps on the line (%ld dabs)", gaps, nd);
    /* scatter counts too */
    epdb_desc d;
    epdb_desc_default(&d);
    d.scatter = 8; d.pos_jitter = 1;
    epd_brush *sb = mk(&d);
    p0 = pt(10, 200, 1, 0); p1 = pt(510, 200, 1, 0.0028);
    epdb_stroke_begin(&s, &p0, 1);
    d0 = epdb_stats_dabs;
    epdb_segment(sb, 64, 64, 1, &s, &p1, &t, NULL);
    nd = epdb_stats_dabs - d0;
    CHECK(nd <= 2 * 9, "scatter 8 at 64 px: %ld dabs (one step + the first)", nd);
    /* pressure -> diameter: pexp and the pressure_size curve */
    clear(255);
    p0 = pt(300, 200, 0, 0);
    epdb_stroke_begin(&s, &p0, 1);
    epdb_segment(b, 10, 50, 2, &s, &p0, &t, NULL);       /* pressure 0: 10 px */
    int ex, ey;
    extent(&ex, &ey);
    CHECK(ex >= 10 && ex <= 12, "pressure 0 -> dmin: %d", ex);
    clear(255);
    p0.pressure = 0.5f;
    epdb_stroke_begin(&s, &p0, 1);
    epdb_segment(b, 10, 50, 2, &s, &p0, &t, NULL);       /* 10 + 40 * 0.25 = 20 */
    extent(&ex, &ey);
    CHECK(ex >= 20 && ex <= 22, "pressure 0.5 ^2 -> 20: %d", ex);
    epdb_desc_default(&d);
    d.curves_present = 1 << NB_CURVE_P_SIZE;
    for (int i = 0; i < 16; i++) d.curve[NB_CURVE_P_SIZE][i] = 255;    /* always full */
    epd_brush *cb = mk(&d);
    clear(255);
    epdb_stroke_begin(&s, &p0, 1);
    epdb_segment(cb, 10, 50, 2, &s, &p0, &t, NULL);
    extent(&ex, &ey);
    CHECK(ex >= 50 && ex <= 52, "pressure_size curve replaces pexp: %d", ex);
    epdb_unref(cb);
    /* velocity_size: 128 (x1) at rest .. 255 (~x2) at speed; a fast stroke inks wider */
    epdb_desc_default(&d);
    d.curves_present = 1 << NB_CURVE_V_SIZE;
    for (int i = 0; i < 16; i++) d.curve[NB_CURVE_V_SIZE][i] = (uint8_t)(128 + i * 127 / 15);
    cb = mk(&d);
    int wide[2];
    for (int k = 0; k < 2; k++) {
        clear(255);
        float v = k ? 3000 : 30;                                /* panel px/s */
        epdb_stroke_begin(&s, &(epdb_point){ 100, 200, 1, 0, 0, 0, 0 }, 1);
        for (int i = 1; i <= 100; i++) {
            epdb_point q = { 100 + i * 4.0f, 200, 1, 0, 0, 0, i * 4.0 / v };
            epdb_segment(cb, 10, 10, 1, &s, &q, &t, NULL);
        }
        int n = 0;
        for (int y = 0; y < H; y++) n += grey[y * W + 450] == 0;
        wide[k] = n;
    }
    CHECK(wide[0] >= 10 && wide[0] <= 12 && wide[1] >= 16, "velocity_size: slow %d px, fast %d px", wide[0], wide[1]);
    epdb_unref(cb);
    /* pressure_opacity: half opacity on a darken grey 0 = level ~128 */
    epdb_desc_default(&d);
    d.curves_present = 1 << NB_CURVE_P_OPACITY;
    memset(d.curve[NB_CURVE_P_OPACITY], 128, 16);
    cb = mk(&d);
    clear(255);
    p0 = pt(300, 200, 1, 0);
    epdb_stroke_begin(&s, &p0, 1);
    epdb_segment(cb, 20, 20, 1, &s, &p0, &t, NULL);
    CHECK(grey[200 * W + 300] >= 125 && grey[200 * W + 300] <= 129, "pressure_opacity: %d", grey[200 * W + 300]);
    epdb_unref(cb);
    epdb_unref(sb);
    epdb_unref(b);
    printf("cap + dynamics: ok\n");
}

/* edges and big dabs (chunks > 256 px): stays inside, the box too (run under ASan too) */
static void t_clip(void)
{
    static uint8_t tip[40 * 40 * 2];
    for (size_t i = 0; i < sizeof tip; i++) tip[i] = (uint8_t)(i * 13);
    const float at[][2] = { { -10, -10 }, { W + 5, H + 5 }, { W - 1, 0 }, { 0, H - 1 }, { W / 2, H / 2 }, { -400, 50 } };
    int bad = 0, drawn = 0;
    for (int kind = 0; kind < 4; kind++)
        for (int hal = 0; hal < 2; hal++)
            for (int dither = 0; dither < 2; dither++) {
                epdb_desc d;
                epdb_desc_default(&d);
                if (kind == 1) d.hardness = 0.3f;
                if (kind == 2) { d.elongation = 0.3f; d.angle_deg = 33; d.hardness = 0.6f; }
                if (kind == 3) { d.tip = tip; d.tip_size = 40; d.tip_variants = 2; d.angle_deg = -71; d.elongation = 0.7f; }
                epd_brush *b = mk(&d);
                for (size_t j = 0; j < sizeof at / sizeof *at; j++)
                    for (float dia = 1; dia <= 700; dia *= 3.3f) {
                        clear(255);
                        epdb_target t = tgt(dither, hal);
                        epdb_stroke s;
                        epdb_point p = pt(at[j][0], at[j][1], 1, 0);
                        epdb_box hb;
                        epdb_stroke_begin(&s, &p, 1);
                        epdb_box bx = epdb_segment(b, dia, dia, 1, &s, &p, &t, &hb);
                        if (bx.x0 < bx.x1) {
                            drawn++;
                            bad += bx.x0 < 0 || bx.y0 < 0 || bx.x1 > W || bx.y1 > H;
                        }
                        if (hal && hb.x0 < hb.x1) {
                            bad += hb.x0 < 0 || hb.y0 < 0 || hb.x1 > W || hb.y1 > H;
                            epdb_halo_flush(&t, (epdb_box){ -5, -5, W + 5, H + 5 });
                        }
                    }
                epdb_unref(b);
            }
    CHECK(!bad && drawn > 100, "clip: %d boxes outside (%d drawn)", bad, drawn);
    /* random bytes never load, random mutations of a valid file never crash */
    static uint8_t buf[NB_HDR + 40 * 40 * 2];
    epdb_desc d;
    epdb_desc_default(&d);
    d.tip = tip; d.tip_size = 40; d.tip_variants = 2;
    size_t n = epdb_encode(&d, buf);
    srand(5);
    int loaded = 0;
    for (int i = 0; i < 20000; i++) {
        static uint8_t m[sizeof buf];
        memcpy(m, buf, n);
        int k = 1 + rand() % 3;
        while (k--) m[rand() % NB_HDR] = (uint8_t)rand();
        epd_brush *b = epdb_load(m, n, NULL, 0);
        if (b) {
            loaded++;
            clear(255);
            epdb_target t = tgt(rand() & 1, 0);
            epdb_point pts[3] = { pt(50, 50, 0.5f, 0), pt(80, 70, 1, 0.003), pt(120, 60, 0.2f, 0.006) };
            stroke(b, 2, 40, 1.5f, pts, 3, 1, &t, NULL);
            epdb_unref(b);
        }
    }
    /* garbage points: NaN / huge coordinates, tilt, times, diameters */
    {
        epd_brush *b = epdb_disc(0);
        epdb_target t = tgt(1, 0);
        epdb_stroke s;
        epdb_point p0 = pt(10, 10, 1, 0);
        epdb_stroke_begin(&s, &p0, 1);
        epdb_point g[] = { { NAN, 5, 1, 0, 0, 0, 0 }, { 1e30f, 5, 1, 0, 0, 0, 0 }, { 5, 5, NAN, NAN, NAN, 1, NAN },
                           { 50, 50, 1, 0, 0, 0, 1e300 }, { 9e5f, -9e5f, 1, 0, 0, 0, 0 }, { 60, 60, 1, 0, 0, 0, -5 } };
        for (size_t i = 0; i < sizeof g / sizeof *g; i++) {
            epdb_segment(b, 4, 8, 1, &s, &g[i], &t, NULL);
            epdb_segment(b, NAN, INFINITY, NAN, &s, &g[i], &t, NULL);
        }
        epdb_unref(b);
    }
    printf("clip + fuzz: ok (%d of 20000 mutations valid)\n", loaded);
}

static double now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

/* us per dab: an ink dab, then the same dab erasing it (every dab changes its pixels) */
static void bench_one(const char *name, epd_brush *b, epd_brush *e, float d, int dither)
{
    epdb_target t = tgt(dither, 0);
    int reps = (int)(2e6 / (d * d)) + 200;
    clear(255);
    epdb_stroke s;
    long d0 = epdb_stats_dabs;
    double t0 = now();
    for (int r = 0; r < reps; r++) {
        epdb_point p = pt(100 + (r % 7) * 50.3f, 100 + (r % 5) * 40.7f, 1, 0);
        epdb_stroke_begin(&s, &p, (uint32_t)r + 1);
        epdb_segment(b, d, d, 1, &s, &p, &t, NULL);
        epdb_stroke_begin(&s, &p, (uint32_t)r + 1);
        epdb_segment(e, d, d, 1, &s, &p, &t, NULL);
    }
    double dt = now() - t0;
    long nd = epdb_stats_dabs - d0;
    printf("  %-20s d %2.0f %s: %6.2f us/dab\n", name, d, dither ? "dither" : "      ", dt * 1e6 / nd);
}

static void pair(epdb_desc *d, epd_brush **b, epd_brush **e)
{
    *b = mk(d);
    d->kind = NB_ERASE;
    *e = mk(d);
}

static void bench(void)
{
    epd_brush *b[4], *e[4];
    const char *nm[4] = { "hard disc", "soft disc grey", "hard ellipse 0.3", "32 px mask, 4 var" };
    epdb_desc d;
    epdb_desc_default(&d);
    pair(&d, &b[0], &e[0]);
    epdb_desc_default(&d);
    d.hardness = 0.5f; d.grey = 100;
    pair(&d, &b[1], &e[1]);
    epdb_desc_default(&d);
    d.elongation = 0.3f; d.angle_deg = 30;
    pair(&d, &b[2], &e[2]);
    static uint8_t tip[32 * 32 * 4];
    for (size_t i = 0; i < sizeof tip; i++) tip[i] = (uint8_t)(i * 97 % 251);
    epdb_desc_default(&d);
    d.tip = tip; d.tip_size = 32; d.tip_variants = 4; d.angle_deg = 20;
    pair(&d, &b[3], &e[3]);
    printf("bench:\n");
    for (int dither = 0; dither < 2; dither++)
        for (int k = 0; k < 4; k++)
            for (int sz = 8; sz <= 64; sz *= 2) bench_one(nm[k], b[k], e[k], (float)sz, dither);
    for (int k = 0; k < 4; k++) { epdb_unref(b[k]); epdb_unref(e[k]); }
}

int main(int argc, char **argv)
{
    srand(1);
    for (int i = 0; i < 64 * 64; i++) thr[i] = (uint8_t)(rand() % 255);
    t_file();
    t_old();
    t_blend();
    t_dither();
    t_halo();
    t_angle();
    t_scatter();
    t_cap();
    t_clip();
    if (argc > 1 && !strcmp(argv[1], "bench")) bench();
    printf(fails ? "epd-brush: %d FAILED\n" : "epd-brush: ok\n", fails);
    return fails != 0;
}
