/* epd-brush: declarative pen-preview brushes, the renderer (epd-brush.h is
 * the contract; "Brushes").
 *
 * A dab = one stamp of the tip at a point: a disc (hardness: linear falloff
 * from hardness*r to r, through a per-brush coverage profile indexed by the
 * squared normalized distance: no sqrt per pixel) or one of the app's masks
 * (nearest, rotated by inverse mapping in 16.16 fixed point). Both get the
 * elongation (minor/major) and the angle. Coverage x opacity = alpha; the
 * pixel's new level = old mixed toward blend(old, ink) by alpha, read from
 * and written back to the shadow (and ebuf), dithered to 0/255 when the
 * target has a threshold map. A pixel whose level would not change is not
 * written at all. The round hard disc at full coverage (today's pen) takes a
 * span fast path: one sqrt per row.
 *
 * The dab budget: NB_MAXDABS is counted in 64 px dabs (4096 px each), a
 * smaller dab costs its area (at least 256 px): a 64 px brush gets 8 dabs per
 * pen report, a 4 px pen line never gaps on a fast stroke (the old ink_line
 * drew every step too). Past the budget the spacing grows. */
#include "epd-brush.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef M_PI                    /* strict C11 (sway builds with -std=c11 -D_POSIX_C_SOURCE) */
#define M_PI 3.14159265358979323846
#endif

#define NPROF 1024                      /* disc profile entries over q = d^2/R^2 in [0,1) */

struct epd_brush {
    int refs;
    uint8_t kind, blend, ink, angle_mode, scatter, curves, tsize, tvar;
    int op256;                          /* opacity 0..256 */
    int round_hard;                     /* hard disc: the span path when round */
    float hardness, spacing, elong, angle;  /* angle in radians */
    float pj, sj, oj;
    uint8_t curve[NB_NCURVE][16];
    uint16_t prof[NPROF];               /* disc coverage 0..256 */
    uint8_t *tip;                       /* tsize^2 * tvar (NULL: disc) */
};

/* test hook (not in the header): dabs stamped since start */
long epdb_stats_dabs;

/* ---- the file --------------------------------------------------------------- */
void epdb_desc_default(epdb_desc *d)
{
    memset(d, 0, sizeof *d);
    d->kind = NB_INK;
    d->blend = NB_DARKEN;
    d->grey = 0;
    d->opacity = 255;
    d->angle_mode = NB_ANG_FIXED;
    d->hardness = 1;
    d->spacing = 0.25f;
    d->elongation = 1;
    d->tip_variants = 1;
}

static float clampf(float v, float lo, float hi)
{
    if (!(v >= lo)) return lo;          /* (NaN: lo) */
    return v > hi ? hi : v;
}
static void put16(uint8_t *p, unsigned v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static unsigned get16(const uint8_t *p) { return p[0] | (unsigned)p[1] << 8; }

size_t epdb_encode(const epdb_desc *d, uint8_t *out)
{
    if (!d || !out) return 0;
    if (d->kind > NB_NONE || d->blend > NB_REPLACE || d->angle_mode > NB_ANG_TILT) return 0;
    unsigned ts = d->tip_size, tv = ts ? d->tip_variants : 1;
    if (ts > NB_MAXTIP || (ts && (tv < 1 || tv > NB_MAXVAR || !d->tip))) return 0;
    memset(out, 0, NB_HDR);
    memcpy(out, NB_MAGIC, 4);
    put16(out + 4, NB_VERSION);
    out[6] = d->kind;
    out[7] = d->blend;
    out[8] = d->grey;
    out[9] = d->opacity;
    out[10] = d->angle_mode;
    out[11] = d->scatter > 8 ? 8 : d->scatter;
    put16(out + 12, (unsigned)lrintf(clampf(d->hardness, 0, 1) * 65535));
    put16(out + 14, (unsigned)lrintf(clampf(d->spacing, 0.02f, 4) * 1024));
    put16(out + 16, (unsigned)lrintf(clampf(d->elongation, 0.05f, 1) * 65535));
    float a = clampf(d->angle_deg, -1e6f, 1e6f);
    a = fmodf(a, 360);
    if (a >= 180) a -= 360;
    if (a < -180) a += 360;
    put16(out + 18, (unsigned)(uint16_t)(int16_t)lrintf(a * 64));
    put16(out + 20, (unsigned)lrintf(clampf(d->pos_jitter, 0, 2) * 1024));
    put16(out + 22, (unsigned)lrintf(clampf(d->size_jitter, 0, 1) * 65535));
    put16(out + 24, (unsigned)lrintf(clampf(d->op_jitter, 0, 1) * 65535));
    out[26] = d->curves_present & ((1 << NB_NCURVE) - 1);
    for (int c = 0; c < NB_NCURVE; c++)
        if (out[26] >> c & 1) memcpy(out + 28 + 16 * c, d->curve[c], 16);
    out[92] = (uint8_t)ts;
    out[93] = (uint8_t)tv;
    size_t n = (size_t)ts * ts * tv;
    if (n) memcpy(out + NB_HDR, d->tip, n);
    return NB_HDR + n;
}

/* SHA-256 (FIPS 180-4), small and self-contained */
static const uint32_t sha_k[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2 };
#define ROR(x, n) ((x) >> (n) | (x) << (32 - (n)))
static void sha_block(uint32_t h[8], const uint8_t *p)
{
    uint32_t w[64], a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], k = h[7];
    for (int i = 0; i < 16; i++) w[i] = (uint32_t)p[4 * i] << 24 | p[4 * i + 1] << 16 | p[4 * i + 2] << 8 | p[4 * i + 3];
    for (int i = 16; i < 64; i++) {
        uint32_t s0 = ROR(w[i - 15], 7) ^ ROR(w[i - 15], 18) ^ w[i - 15] >> 3;
        uint32_t s1 = ROR(w[i - 2], 17) ^ ROR(w[i - 2], 19) ^ w[i - 2] >> 10;
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    for (int i = 0; i < 64; i++) {
        uint32_t t1 = k + (ROR(e, 6) ^ ROR(e, 11) ^ ROR(e, 25)) + ((e & f) ^ (~e & g)) + sha_k[i] + w[i];
        uint32_t t2 = (ROR(a, 2) ^ ROR(a, 13) ^ ROR(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
        k = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e; h[5] += f; h[6] += g; h[7] += k;
}

/* (also used by the test: the full digest) */
void epdb_sha256(const uint8_t *m, size_t len, uint8_t out[32])
{
    uint32_t h[8] = { 0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19 };
    size_t i = 0;
    for (; i + 64 <= len; i += 64) sha_block(h, m + i);
    uint8_t t[128] = {0};
    size_t r = len - i;
    if (r) memcpy(t, m + i, r);
    t[r] = 0x80;
    size_t tl = r + 1 + 8 <= 64 ? 64 : 128;
    uint64_t bits = (uint64_t)len * 8;
    for (int j = 0; j < 8; j++) t[tl - 1 - j] = (uint8_t)(bits >> (8 * j));
    sha_block(h, t);
    if (tl == 128) sha_block(h, t + 64);
    for (int j = 0; j < 8; j++) {
        out[4 * j] = (uint8_t)(h[j] >> 24); out[4 * j + 1] = (uint8_t)(h[j] >> 16);
        out[4 * j + 2] = (uint8_t)(h[j] >> 8); out[4 * j + 3] = (uint8_t)h[j];
    }
}

void epdb_id(const uint8_t *file, size_t len, char id[17])
{
    static const char hx[] = "0123456789abcdef";
    uint8_t d[32];
    epdb_sha256(file, len, d);
    for (int i = 0; i < 8; i++) { id[2 * i] = hx[d[i] >> 4]; id[2 * i + 1] = hx[d[i] & 15]; }
    id[16] = 0;
}

/* ---- prepared brushes -------------------------------------------------------- */
static void prep_profile(epd_brush *b)
{
    float h = b->hardness;
    for (int k = 0; k < NPROF; k++) {
        float f = sqrtf((k + 0.5f) / NPROF), c;
        if (h >= 0.9999f || f <= h) c = 1;
        else c = (1 - f) / (1 - h);
        int v = (int)lrintf(c * 256);
        b->prof[k] = (uint16_t)(v < 0 ? 0 : v > 256 ? 256 : v);
    }
}

#define BAD(...) do { if (err && errlen) snprintf(err, errlen, __VA_ARGS__); return NULL; } while (0)
epd_brush *epdb_load(const uint8_t *f, size_t len, char *err, size_t errlen)
{
    if (!f || len < NB_HDR) BAD("short file (%zu bytes)", len);
    if (memcmp(f, NB_MAGIC, 4)) BAD("bad magic");
    if (get16(f + 4) != NB_VERSION) BAD("version %u", get16(f + 4));
    if (f[6] > NB_NONE) BAD("ink kind %u", f[6]);
    if (f[7] > NB_REPLACE) BAD("blend %u", f[7]);
    if (f[10] > NB_ANG_TILT) BAD("angle mode %u", f[10]);
    if (f[11] > 8) BAD("scatter count %u", f[11]);
    unsigned sp = get16(f + 14), el = get16(f + 16), pj = get16(f + 20);
    if (sp < 20 || sp > 4096) BAD("spacing %u/1024", sp);
    if (el < 3276) BAD("elongation %u/65535", el);
    if (pj > 2048) BAD("position jitter %u/1024", pj);
    if (f[26] >> NB_NCURVE) BAD("curves present 0x%02x", f[26]);
    if (f[27]) BAD("reserved byte 27");
    for (int c = 0; c < NB_NCURVE; c++)
        if (!(f[26] >> c & 1))
            for (int i = 0; i < 16; i++)
                if (f[28 + 16 * c + i]) BAD("curve %d not present but set", c);
    unsigned ts = f[92], tv = f[93];
    if (ts > NB_MAXTIP) BAD("tip size %u", ts);
    if (ts ? tv < 1 || tv > NB_MAXVAR : tv != 1) BAD("tip variants %u", tv);
    if (f[94] || f[95]) BAD("reserved bytes 94-95");
    size_t want = NB_HDR + (size_t)ts * ts * tv;
    if (len != want) BAD("length %zu, want %zu", len, want);

    epd_brush *b = calloc(1, sizeof *b);
    if (!b) BAD("out of memory");
    b->refs = 1;
    b->kind = f[6];
    b->blend = b->kind == NB_ERASE ? NB_REPLACE : f[7];
    b->ink = b->kind == NB_ERASE ? 255 : f[8];
    b->op256 = f[9] + (f[9] >> 7);
    b->angle_mode = f[10];
    b->scatter = f[11];
    b->hardness = get16(f + 12) / 65535.0f;
    b->spacing = sp / 1024.0f;
    b->elong = el / 65535.0f;
    b->angle = (int16_t)get16(f + 18) / 64.0f * (float)M_PI / 180;
    b->pj = pj / 1024.0f;
    b->sj = get16(f + 22) / 65535.0f;
    b->oj = get16(f + 24) / 65535.0f;
    b->curves = f[26];
    memcpy(b->curve, f + 28, sizeof b->curve);
    b->tsize = (uint8_t)ts;
    b->tvar = (uint8_t)tv;
    if (ts) {
        b->tip = malloc((size_t)ts * ts * tv);
        if (!b->tip) { free(b); BAD("out of memory"); }
        memcpy(b->tip, f + NB_HDR, (size_t)ts * ts * tv);
    }
    b->round_hard = !ts && b->hardness >= 0.9999f;
    prep_profile(b);
    return b;
}

epd_brush *epdb_disc(int erase)
{
    epdb_desc d;
    uint8_t buf[NB_HDR];
    epdb_desc_default(&d);
    if (erase) { d.kind = NB_ERASE; d.grey = 255; d.blend = NB_REPLACE; }
    size_t n = epdb_encode(&d, buf);
    return epdb_load(buf, n, NULL, 0);
}

epd_brush *epdb_ref(epd_brush *b)
{
    if (b) __atomic_fetch_add(&b->refs, 1, __ATOMIC_RELAXED);
    return b;
}

void epdb_unref(epd_brush *b)
{
    if (b && __atomic_sub_fetch(&b->refs, 1, __ATOMIC_ACQ_REL) == 0) {
        free(b->tip);
        free(b);
    }
}

int epdb_kind(const epd_brush *b) { return b ? b->kind : NB_NONE; }

/* ---- stamping ----------------------------------------------------------------
 * A dab goes row by row, each row in chunks of up to CHUNK px: the coverage
 * (scalar: the profile / mask lookups), then branch-free passes that
 * vectorize: the new levels, the dither, the first and last pixel that
 * changes, the write (the shadow, and ebuf where the level changed). */
#define CHUNK 256
typedef struct {
    const epdb_target *t;
    int ink, blend;
    epdb_box box, hbox;
} ctx;

static void box_add(epdb_box *b, int x0, int y0, int x1, int y1)
{
    if (x0 >= x1 || y0 >= y1) return;
    if (b->x0 >= b->x1) { *b = (epdb_box){ x0, y0, x1, y1 }; return; }
    if (x0 < b->x0) b->x0 = x0;
    if (y0 < b->y0) b->y0 = y0;
    if (x1 > b->x1) b->x1 = x1;
    if (y1 > b->y1) b->y1 = y1;
}

static inline int blend_px(int o, int ink, int blend)
{
    return blend == NB_DARKEN ? (o < ink ? o : ink) : blend == NB_LIGHTEN ? (o > ink ? o : ink) : ink;
}
static inline int mix_px(int o, int tg, int a) { return o + (((tg - o) * a + 128) >> 8); }

/* the new levels nl of g[0..n) at alpha cov*a (cov NULL: full coverage) */
static inline __attribute__((always_inline))
void levels_bl(uint8_t *restrict nl, const uint8_t *restrict g, const uint16_t *restrict cov, int a, int n,
               int ink, const int bl)
{
    if (!cov && a == 256) {
        for (int i = 0; i < n; i++) nl[i] = (uint8_t)blend_px(g[i], ink, bl);
    } else if (!cov) {
        for (int i = 0; i < n; i++) nl[i] = (uint8_t)mix_px(g[i], blend_px(g[i], ink, bl), a);
    } else {
        for (int i = 0; i < n; i++) {
            int al = (cov[i] * a + 128) >> 8;
            nl[i] = (uint8_t)mix_px(g[i], blend_px(g[i], ink, bl), al);
        }
    }
}

/* the dither: thresh row tr, starting at column xi (wraps at tw) */
static void dither_row(uint8_t *restrict nl, const uint8_t *restrict tr, int tw, int xi, int n)
{
    while (n > 0) {
        int k = tw - xi < n ? tw - xi : n;
        for (int i = 0; i < k; i++) nl[i] = nl[i] > tr[xi + i] ? 255 : 0;
        nl += k; n -= k; xi = 0;
    }
}

/* one pixel at coverage cov (0..256) x alpha a (0..256), with the halo */
static void put_halo(ctx *c, int x, int y, size_t i, int cov, int a)
{
    const epdb_target *t = c->t;
    int al = (cov * a + 128) >> 8;
    if (al <= 0) return;                /* below 1/256 */
    int o = t->grey[i], n = mix_px(o, blend_px(o, c->ink, c->blend), al);
    if (cov < 128) {
        if (n == o) return;
        int h = t->halo[i];
        if (h) {
            int p = h - 1 == 254 ? 255 : h - 1;
            if (c->blend == NB_DARKEN) n = n < p ? n : p;
            else if (c->blend == NB_LIGHTEN) n = n > p ? n : p;
        }
        t->halo[i] = (uint8_t)((n > 254 ? 254 : n) + 1);
        box_add(&c->hbox, x, y, x + 1, y + 1);
        return;
    }
    t->halo[i] = 0;                     /* the core supersedes a pending level */
    if (t->thresh) {
        n = n > t->thresh[(size_t)(y % t->th) * t->tw + x % t->tw] ? 255 : 0;
        if (c->blend == NB_DARKEN && n > o) n = o;          /* (as apply_row: the dither never undoes the blend) */
        else if (c->blend == NB_LIGHTEN && n < o) n = o;
    }
    if (n == o) return;
    t->grey[i] = (uint8_t)n;
    if (t->xrgb) t->xrgb[i] = 0xFF000000u | (uint32_t)n * 0x010101u;
    box_add(&c->box, x, y, x + 1, y + 1);
}

/* the shadow, and ebuf where the level changes */
static void write_row(uint8_t *restrict g, uint32_t *restrict xr, const uint8_t *restrict nl, int n)
{
    for (int i = 0; i < n; i++) {
        uint8_t v = nl[i];
        xr[i] = v != g[i] ? 0xFF000000u | v * 0x010101u : xr[i];
        g[i] = v;
    }
}

/* pixels [x0, x0+n) of row y (n <= CHUNK), coverage cov (NULL: full) x alpha a */
static void apply_row(ctx *c, int y, int x0, int n, const uint16_t *cov, int a)
{
    const epdb_target *t = c->t;
    size_t r = (size_t)y * t->stride + x0;
    uint8_t *g = t->grey + r;
    if (t->halo) {
        for (int i = 0; i < n; i++) put_halo(c, x0 + i, y, r + i, cov ? cov[i] : 256, a);
        return;
    }
    uint8_t nl[CHUNK];
    switch (c->blend) {
    case NB_DARKEN: levels_bl(nl, g, cov, a, n, c->ink, NB_DARKEN); break;
    case NB_LIGHTEN: levels_bl(nl, g, cov, a, n, c->ink, NB_LIGHTEN); break;
    default: levels_bl(nl, g, cov, a, n, c->ink, NB_REPLACE); break;
    }
    if (t->thresh) {
        dither_row(nl, t->thresh + (size_t)(y % t->th) * t->tw, t->tw, x0 % t->tw, n);
        /* the dither must not undo the blend: a darken dab over a grey (an
         * app's antialiased edge) mixes toward its ink, and the threshold
         * can round that up to white — a white speckle in an older stroke,
         * and a pixel that now heads elsewhere than the app's own update in
         * flight (the engine holds the pen's piece until that one is done) */
        if (c->blend == NB_DARKEN) for (int i = 0; i < n; i++) nl[i] = nl[i] < g[i] ? nl[i] : g[i];
        else if (c->blend == NB_LIGHTEN) for (int i = 0; i < n; i++) nl[i] = nl[i] > g[i] ? nl[i] : g[i];
    }
    int f = 0, l = n - 1;
    while (f < n && nl[f] == g[f]) f++;
    if (f == n) return;
    while (nl[l] == g[l]) l--;
    if (t->xrgb) write_row(g + f, t->xrgb + r + f, nl + f, l - f + 1);
    else memcpy(g + f, nl + f, (size_t)(l - f + 1));
    box_add(&c->box, x0 + f, y, x0 + l + 1, y + 1);
}

/* floor / ceil to int without libm calls (x86-64 baseline has no roundss) */
static inline int ifloor(float v) { int i = (int)v; return i - (v < (float)i); }
static inline int iceil(float v) { int i = (int)v; return i + (v > (float)i); }

static void clip_rows(const epdb_target *t, float cy, float ey, int *y0, int *y1)
{
    *y0 = iceil(cy - ey);
    *y1 = ifloor(cy + ey);
    if (*y0 < 0) *y0 = 0;
    if (*y1 > t->h - 1) *y1 = t->h - 1;
}

typedef struct { float cx, cy, d, ang; int a, var; } dab;

static void stamp(const epd_brush *b, ctx *c, const dab *k)
{
    const epdb_target *t = c->t;
    if (k->a <= 0) return;
    epdb_stats_dabs++;
    float r = k->d * 0.5f;
    if (r < 0.5f) r = 0.5f;
    if (!(k->cx > -2 * r - 2 && k->cx < t->w + 2 * r + 2 && k->cy > -2 * r - 2 && k->cy < t->h + 2 * r + 2))
        return;                         /* off the panel (or NaN) */
    float cs = cosf(k->ang), sn = sinf(k->ang);
    uint16_t cov[CHUNK];
    int y0, y1;
    if (!b->tip) {
        /* the disc: R^2 = r^2 + r/2 (ink_disc's edge, r 1 = the old 3x3 + 4
         * tips). q = (u/Ra)^2 + (v/Rb)^2 < 1, u along the major axis; per
         * row q(dx) = A dx^2 + B dx + C: the span from its roots, q by
         * second differences inside it */
        float rb = r * b->elong;
        float Ra = sqrtf(r * r + r * 0.5f), Rb = sqrtf(rb * rb + rb * 0.5f);
        float cx = k->cx, cy = k->cy;
        if (b->round_hard && b->elong >= 0.9999f) { cx = (float)ifloor(cx + 0.5f); cy = (float)ifloor(cy + 0.5f); }   /* (integer centers: ink_disc) */
        float ua = cs / Ra, ub = sn / Ra, va = -sn / Rb, vb = cs / Rb;
        float A = ua * ua + va * va, B1 = 2 * (ua * ub + va * vb), C1 = ub * ub + vb * vb;
        float ey = sqrtf(Ra * Ra * sn * sn + Rb * Rb * cs * cs), i2a = 0.5f / A;
        clip_rows(t, cy, ey, &y0, &y1);
        for (int y = y0; y <= y1; y++) {
            float dy = y - cy, B = B1 * dy, C = C1 * dy * dy - 1, disc = B * B - 4 * A * C;
            if (disc < 0) continue;
            float sq = sqrtf(disc);
            int xa = iceil(cx + (-B - sq) * i2a), xb = ifloor(cx + (-B + sq) * i2a);
            if (xa < 0) xa = 0;
            if (xb > t->w - 1) xb = t->w - 1;
            for (int x = xa; x <= xb; x += CHUNK) {
                int n = xb - x + 1 < CHUNK ? xb - x + 1 : CHUNK;
                if (b->round_hard) { apply_row(c, y, x, n, NULL, k->a); continue; }
                float dx = x - cx, q = (A * dx + B) * dx + C + 1, dq = A * (2 * dx + 1) + B, a2 = 2 * A;
                for (int i = 0; i < n; i++, q += dq, dq += a2) {
                    int j = (int)(q * NPROF);
                    j = j < 0 ? 0 : j > NPROF - 1 ? NPROF - 1 : j;
                    cov[i] = b->prof[j];
                }
                apply_row(c, y, x, n, cov, k->a);
            }
        }
        return;
    }
    /* a mask: the side = the diameter along the major axis, x elongation
     * along the minor one; mask coords (16.16) of the pixel offset */
    int S = b->tsize;
    float rm = r * b->elong;
    float ex = fabsf(r * cs) + fabsf(rm * sn), ey = fabsf(r * sn) + fabsf(rm * cs);
    int x0 = iceil(k->cx - ex), x1 = ifloor(k->cx + ex);
    if (x0 < 0) x0 = 0;
    if (x1 > t->w - 1) x1 = t->w - 1;
    clip_rows(t, k->cy, ey, &y0, &y1);
    float su = S / (2 * r) * 65536, sv = S / (2 * rm) * 65536, half = S * 0.5f * 65536;
    int32_t mxa = (int32_t)lrintf(cs * su), mya = (int32_t)lrintf(-sn * sv);
    const uint8_t *m = b->tip + (size_t)k->var * S * S;
    uint32_t lim = (uint32_t)S << 16;
    for (int y = y0; y <= y1; y++) {
        float dy = y - k->cy;
        /* the row's pixels inside the mask square: mx and my in [0, S) are
         * two slabs in x (one pixel of margin; the lookup still checks) */
        float pu = (dy * sn - k->cx * cs) * su + half, pv = (dy * cs + k->cx * sn) * sv + half;
        float gu = cs * su, gv = -sn * sv, lo = (float)x0, hi = (float)x1;
        if (fabsf(gu) > 1e-3f) {
            float e0 = -pu / gu, e1 = ((float)lim - pu) / gu;
            if (e0 > e1) { float tmp = e0; e0 = e1; e1 = tmp; }
            if (e0 - 1 > lo) lo = e0 - 1;
            if (e1 + 1 < hi) hi = e1 + 1;
        } else if (pu < 0 || pu >= (float)lim) continue;
        if (fabsf(gv) > 1e-3f) {
            float e0 = -pv / gv, e1 = ((float)lim - pv) / gv;
            if (e0 > e1) { float tmp = e0; e0 = e1; e1 = tmp; }
            if (e0 - 1 > lo) lo = e0 - 1;
            if (e1 + 1 < hi) hi = e1 + 1;
        } else if (pv < 0 || pv >= (float)lim) continue;
        int xs = iceil(lo), xe = ifloor(hi);
        for (int x = xs; x <= xe; x += CHUNK) {
            int n = xe - x + 1 < CHUNK ? xe - x + 1 : CHUNK, any = 0;
            float dx = x - k->cx;
            int32_t mx = (int32_t)lrintf((dx * cs + dy * sn) * su + half);
            int32_t my = (int32_t)lrintf((dy * cs - dx * sn) * sv + half);
            for (int i = 0; i < n; i++, mx += mxa, my += mya) {
                int v = (uint32_t)mx < lim && (uint32_t)my < lim ? m[(my >> 16) * S + (mx >> 16)] : 0;
                cov[i] = (uint16_t)(v + (v >> 7));
                any |= v;
            }
            if (any) apply_row(c, y, x, n, cov, k->a);
        }
    }
}

/* ---- the stroke ------------------------------------------------------------- */
static uint32_t rnd(epdb_stroke *s)
{
    uint32_t x = s->seed;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    return s->seed = x;
}
static float rndf(epdb_stroke *s) { return (rnd(s) >> 8) * (1.0f / 16777216); }

static float curve(const uint8_t *c, float x)    /* 0..255 */
{
    x = clampf(x, 0, 1) * 15;
    int i = (int)x;
    if (i >= 15) return c[15];
    return c[i] + (c[i + 1] - c[i]) * (x - i);
}

void epdb_stroke_begin(epdb_stroke *s, const epdb_point *p, uint32_t seed)
{
    memset(s, 0, sizeof *s);
    s->seed = seed ? seed : 0x9E3779B9u;
    s->lx = p->x;
    s->ly = p->y;
    s->lt = p->t;
    s->n = 1;
}

epdb_box epdb_segment(const epd_brush *b, float dmin, float dmax, float pexp,
                      epdb_stroke *s, const epdb_point *p, const epdb_target *t,
                      epdb_box *halo_box)
{
    ctx c = { t, 0, 0, {0, 0, 0, 0}, {0, 0, 0, 0} };
    if (halo_box) *halo_box = c.hbox;
    if (!(fabsf(p->x) < 1e6f && fabsf(p->y) < 1e6f)) return c.box;    /* (NaN, garbage: ignored) */
    float dx = p->x - s->lx, dy = p->y - s->ly, L = sqrtf(dx * dx + dy * dy);
    double dt = p->t - s->lt;
    if (dt > 1e-4 && dt < 1e6) {
        float vi = (float)(L / dt), kk = (float)(dt / (dt + 0.03));    /* ~30 ms smoothing */
        s->vel += (vi - s->vel) * kk;
    }
    if (L > 0.5f || (s->n == 1 && L > 1e-3f)) s->dir = atan2f(dy, dx);
    float sx = s->lx, sy = s->ly;
    int first = s->n == 1;
    s->lx = p->x; s->ly = p->y; s->lt = p->t;
    if (s->n < 1 << 30) s->n++;
    if (!b || b->kind == NB_NONE || !t || !t->grey) return c.box;
    c.ink = b->ink;
    c.blend = b->blend;

    /* dynamics at this report */
    float pr = clampf(p->pressure, 0, 1), ps;
    if (b->curves >> NB_CURVE_P_SIZE & 1) ps = curve(b->curve[NB_CURVE_P_SIZE], pr) / 255;
    else ps = !(pexp > 0) || pexp == 1 ? pr : powf(pr, pexp);
    float D = dmin + (dmax - dmin) * ps;
    if (b->curves >> NB_CURVE_V_SIZE & 1) D *= curve(b->curve[NB_CURVE_V_SIZE], s->vel / 4000) / 128;
    float tx = p->has_tilt ? clampf(p->tilt_x, -90, 90) : 0, ty = p->has_tilt ? clampf(p->tilt_y, -90, 90) : 0;
    if (b->curves >> NB_CURVE_T_SIZE & 1)
        D *= curve(b->curve[NB_CURVE_T_SIZE], sqrtf(tx * tx + ty * ty) / 90) / 128;
    if (!(D > 0.5f)) D = 0.5f;
    if (D > 4096) D = 4096;
    float op = b->op256;
    if (b->curves >> NB_CURVE_P_OPACITY & 1) op *= curve(b->curve[NB_CURVE_P_OPACITY], pr) / 255;
    float ang = b->angle;
    if (b->angle_mode == NB_ANG_DIRECTION) ang += s->dir;
    else if (b->angle_mode == NB_ANG_TILT && (tx != 0 || ty != 0)) ang += atan2f(ty, tx);

    /* the step and the budget */
    float step = b->spacing * D;
    if (step < 1) step = 1;
    float ext = D * (1 + b->sj) * (b->tip ? 1.42f : 1) + 2;
    float cost = (ext * ext < 256 ? 256 : ext * ext) * (1 + b->scatter);
    float nd = (s->dist + L) / step + first;
    float budget = (float)NB_MAXDABS * 4096;
    if (nd * cost > budget) {
        float steps = budget / cost - first;    /* spaced steps we can afford */
        if (steps < 1) steps = 1;
        float st2 = (s->dist + L) / steps;
        if (st2 > step) step = st2;
    }

    dab k = { 0, 0, D, ang, (int)op, 0 };
    float pos = 0;
    for (int fi = first;; fi = 0) {
        if (!fi) {
            float need = step - s->dist;
            if (need < 0) need = 0;
            if (pos + need > L) { s->dist += L - pos; break; }
            pos += need;
            s->dist = 0;
        }
        float f = L > 0 ? pos / L : 0;
        k.cx = sx + dx * f;
        k.cy = sy + dy * f;
        k.d = D;
        k.a = (int)op;
        k.var = b->tvar > 1 ? (int)(rnd(s) % b->tvar) : 0;
        stamp(b, &c, &k);
        float mx = k.cx, my = k.cy;
        for (int j = 0; j < b->scatter; j++) {
            float ox, oy;
            int g = 0;
            do { ox = 2 * rndf(s) - 1; oy = 2 * rndf(s) - 1; } while (ox * ox + oy * oy > 1 && ++g < 8);
            k.cx = mx + ox * b->pj * D;
            k.cy = my + oy * b->pj * D;
            k.d = D * (1 + b->sj * (2 * rndf(s) - 1));
            k.a = (int)(op * (1 - b->oj * rndf(s)));
            k.var = b->tvar > 1 ? (int)(rnd(s) % b->tvar) : 0;
            stamp(b, &c, &k);
        }
        if (fi) s->dist = 0;
    }
    if (halo_box) *halo_box = c.hbox;
    return c.box;
}

epdb_box epdb_halo_flush(const epdb_target *t, epdb_box bx)
{
    epdb_box out = {0, 0, 0, 0};
    if (!t || !t->halo) return out;
    if (bx.x0 < 0) bx.x0 = 0;
    if (bx.y0 < 0) bx.y0 = 0;
    if (bx.x1 > t->w) bx.x1 = t->w;
    if (bx.y1 > t->h) bx.y1 = t->h;
    for (int y = bx.y0; y < bx.y1; y++) {
        size_t r = (size_t)y * t->stride;
        int first = -1, last = -1;
        for (int x = bx.x0; x < bx.x1; x++) {
            int h = t->halo[r + x];
            if (!h) continue;
            t->halo[r + x] = 0;
            int n = h - 1 == 254 ? 255 : h - 1, o = t->grey[r + x];
            if (t->thresh) n = n > t->thresh[(size_t)(y % t->th) * t->tw + x % t->tw] ? 255 : 0;
            if (n == o) continue;
            t->grey[r + x] = (uint8_t)n;
            if (t->xrgb) t->xrgb[r + x] = 0xFF000000u | (uint32_t)n * 0x010101u;
            if (first < 0) first = x;
            last = x;
        }
        if (first >= 0) box_add(&out, first, y, last + 1, y + 1);
    }
    return out;
}
