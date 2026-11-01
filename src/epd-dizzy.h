/* epd-dizzy.h - "dizzy" dithering (Liam Appelbe: liamappelbe.medium.com/
 * dizzy-dithering-2ae76dbceba1, gist c2c76ad1): error diffusion in a random
 * order, each pixel's error split equally over its 4 neighbours not yet done
 * (no scan direction: no worms, no grid). Two forms:
 * epd_dizzy_serial: the gist's, one pixel after the other (the order a
 * fixed shuffle of a 64 x 64 tile, the tiles in raster order); a pixel
 * clamps at each add. The desktop daemon's stills (dither_pattern=dizzy)
 * epd_dizzy (layered): the article's parallel form, the shuffle cut in 8
 * layers of 512; layer by layer, a pixel snaps and shares its error with
 * its neighbours in later layers (same-layer neighbours trade nothing); the
 * received error summed, clamped when its layer comes. The desktop daemon's motion
 * runs. A pixel with all 4 neighbours in earlier layers (every last-layer
 * pixel) keeps its error: near the rails a ramp loses up to ~1.6 levels
 * (epd-dizzy-test), one reason serial looks a little better (by eye)
 *
 * Both on the panel's levels 0..30 (black .. white) to 0 / 30, in place over a
 * rect. A pixel with EPD_DZ_FIXED set (glassine-epdd-desktop: the frame's pure black and
 * white, which the commit keeps as they are) takes no part: it neither shares
 * nor receives error (it is not a neighbour) and stays as it is; a 64 x 64
 * tile without a free pixel is skipped whole (a UI page pays for its greys
 * only). A pixel with no free later neighbour keeps its error. Over a
 * rect (t: its rows, stride bytes apart; x0, y0 its panel position: the order
 * is fixed by the panel position, so a still picture dithers the same every
 * time). Error stays inside the rect. Integer: serial in 1/16 levels, the
 * m-th share of err over n neighbours (R, L, D, U order)
 * err (m + 1) / n - err m / n (nothing lost; C's truncation); layered in
 * 1/256 levels, err / n to each (the remainder dropped, see epd_dizzy).
 *
 * Speed : a neighbour's "later" is fixed by the tile position
 * (layered: its layer; serial: its shuffle index, or the tile it is in), so
 * both read a 64 x 64 table of neighbour masks and split the error without a
 * divide; layered runs as one sweep down the rect (layer k on row s - k at step
 * s: everything a pixel receives is in by then), its error in a 16-row ring.
 * The plain forms these must match bit for bit are in test/epd-dizzy-test.c.
 * Separate rects may run at once (work buffers per thread) once
 * epd_dizzy_init has built the tables. */
#ifndef EPD_DIZZY_H
#define EPD_DIZZY_H
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

enum { EPD_DZ_S = 16, EPD_DZ_TOP = 30 * EPD_DZ_S, EPD_DZ_MID = 15 * EPD_DZ_S };
enum { EPD_DZ_R = 1, EPD_DZ_L = 2, EPD_DZ_D = 4, EPD_DZ_U = 8 };
enum { EPD_DZ_FIXED = 0x80 };           /* (in t: the pixel stays as it is) */

/* the rect's tiles (64 x 64, panel-aligned) that hold a free pixel:
 * f[(ty - ty0) * ntx + (tx - tx0)]; returns f, NULL when out of memory */
static inline uint8_t *epd_dz_tiles(const uint8_t *t, int w, int h, int stride, int x0, int y0, int *ntx)
{
    static __thread uint8_t *f;
    static __thread size_t cap;
    int tx0 = x0 >> 6, tx1 = (x0 + w + 63) >> 6, ty0 = y0 >> 6, ty1 = (y0 + h + 63) >> 6;
    size_t n = (size_t)(tx1 - tx0) * (ty1 - ty0);
    if (n > cap) {
        uint8_t *nf = realloc(f, n);
        if (!nf) return NULL;
        f = nf;
        cap = n;
    }
    memset(f, 0, n);
    *ntx = tx1 - tx0;
    for (int y = 0; y < h; y++) {
        const uint8_t *tr = t + (size_t)y * stride;
        uint8_t *fr = f + (size_t)(((y0 + y) >> 6) - ty0) * *ntx;
        for (int tx = tx0; tx < tx1; tx++) {
            if (fr[tx - tx0]) continue;
            int a = (tx << 6) - x0, b = a + 64;
            if (a < 0) a = 0;
            if (b > w) b = w;
            uint8_t all = EPD_DZ_FIXED;
            for (int x = a; x < b; x++) all &= tr[x];
            fr[tx - tx0] = !all;
        }
    }
    return f;
}

/* the shuffle: perm[i] = a tile position (y << 6 | x), its layer i >> 9 */
static inline const uint16_t *epd_dizzy_perm(void)
{
    static uint16_t perm[4096];
    static int built;
    if (!built) {                       /* (Fisher-Yates, a fixed seed: the same layers every run) */
        uint32_t s = 0x9E3779B9u;
        for (int i = 0; i < 4096; i++) perm[i] = (uint16_t)i;
        for (int i = 4095; i > 0; i--) {
            s = s * 1664525u + 1013904223u;
            int j = (int)((uint64_t)s * (uint64_t)(i + 1) >> 32);
            uint16_t v = perm[i]; perm[i] = perm[j]; perm[j] = v;
        }
        built = 1;
    }
    return perm;
}

/* layer[y & 63][x & 63] (0..7) */
static inline const uint8_t (*epd_dizzy_layers(void))[64]
{
    static uint8_t lay[64][64];
    static int built;
    if (!built) {
        const uint16_t *p = epd_dizzy_perm();
        for (int i = 0; i < 4096; i++) lay[p[i] >> 6][p[i] & 63] = (uint8_t)(i >> 9);
        built = 1;
    }
    return (const uint8_t (*)[64])lay;
}

/* the tables: the neighbour masks (later neighbours, EPD_DZ_*), layered and
 * serial; layered's x positions of layer k in tile row y (xs[y][k][0..xn)); the
 * split: share m of |err| over n = q + ex[n][|err| % n][m], q = |err| / n */
typedef struct {
    uint8_t lm[64][64], sm[64][64];
    uint8_t xn[64][8], xs[64][8][64];
    uint8_t ex[5][4][4];
    uint8_t pc[16];
} epd_dizzy_tabs;
static inline const epd_dizzy_tabs *epd_dizzy_tabs_get(void)
{
    static epd_dizzy_tabs T;
    static int built;
    if (!built) {
        const uint16_t *p = epd_dizzy_perm();
        const uint8_t (*L)[64] = epd_dizzy_layers();
        static uint16_t ix[64][64];
        for (int i = 0; i < 4096; i++) ix[p[i] >> 6][p[i] & 63] = (uint16_t)i;
        for (int y = 0; y < 64; y++)
            for (int x = 0; x < 64; x++) {
                int k = L[y][x], i = ix[y][x];
                T.lm[y][x] = (uint8_t)((L[y][(x + 1) & 63] > k) * EPD_DZ_R | (L[y][(x - 1) & 63] > k) * EPD_DZ_L |
                                       (L[(y + 1) & 63][x] > k) * EPD_DZ_D | (L[(y - 1) & 63][x] > k) * EPD_DZ_U);
                /* serial: the next tile right / below is later, left / above earlier */
                T.sm[y][x] = (uint8_t)((x == 63 || ix[y][x + 1] > i) * EPD_DZ_R | (x > 0 && ix[y][x - 1] > i) * EPD_DZ_L |
                                       (y == 63 || ix[y + 1][x] > i) * EPD_DZ_D | (y > 0 && ix[y - 1][x] > i) * EPD_DZ_U);
                T.xs[y][k][T.xn[y][k]++] = (uint8_t)x;
            }
        for (int n = 1; n <= 4; n++)
            for (int r = 0; r < n; r++)
                for (int m = 0; m < n; m++) T.ex[n][r][m] = (uint8_t)((m + 1) * r / n - m * r / n);
        for (int b = 0; b < 16; b++) T.pc[b] = (uint8_t)((b & 1) + (b >> 1 & 1) + (b >> 2 & 1) + (b >> 3 & 1));
        built = 1;
    }
    return &T;
}

/* the split, branch-free: share to direction d (R L D U) of |err| e over
 * mask mk = ((q & bm[mk][d]) + sx[mk][r][d]), q = e * rc[n] >> 16 (exact for
 * e <= 480), r = e - q n; a direction not in mk gets 0 (the work buffers have
 * a 1-pixel border, so adding 0 there is harmless) */
typedef struct { int16_t bm[16][4]; uint8_t sx[16][4][4], n[16]; int32_t rc[5]; } epd_dz_split;
static inline const epd_dz_split *epd_dz_split_get(void)
{
    static epd_dz_split S;
    static int built;
    if (!built) {
        const epd_dizzy_tabs *T = epd_dizzy_tabs_get();
        for (int mk = 0; mk < 16; mk++) {
            int n = T->pc[mk];
            S.n[mk] = (uint8_t)n;
            for (int r = 0; r < 4; r++)
                for (int d = 0, m = 0; d < 4; d++) {
                    int on = mk >> d & 1;
                    S.bm[mk][d] = (int16_t)-on;
                    S.sx[mk][r][d] = (uint8_t)(on && r < n ? T->ex[n][r][m] : 0);
                    m += on;
                }
        }
        S.rc[0] = 0; S.rc[1] = 65536; S.rc[2] = 32768; S.rc[3] = 21846; S.rc[4] = 16384;
        built = 1;
    }
    return &S;
}
/* the four shares of err over mk into sh[4] (signed) */
static inline void epd_dz_shares(const epd_dz_split *S, int err, int mk, int sh[4])
{
    int sg = err >> 31, e = (err ^ sg) - sg, n = S->n[mk];
    int q = (e * S->rc[n]) >> 16, r = (e - q * n) & -(n != 0);
    for (int d = 0; d < 4; d++) {
        int v = (q & S->bm[mk][d]) + S->sx[mk][r][d];
        sh[d] = (v ^ sg) - sg;
    }
}

static inline int16_t *epd_dz_buf(size_t n, int which)
{
    static __thread int16_t *b[2];      /* (per thread: glassine-epdd-desktop runs bands in parallel) */
    static __thread size_t cap[2];
    if (n > cap[which]) {
        int16_t *nb = realloc(b[which], n * sizeof *nb);
        if (!nb) return NULL;
        b[which] = nb;
        cap[which] = n;
    }
    return b[which];
}

/* layered's per-position list: layer k of tile row y, its x and its later
 * neighbours (EPD_DZ_*); dl[mk]: 32768 / the neighbours in mk (0 for none) */
typedef struct { uint8_t xx, mk; } epd_dz_le;
typedef struct { epd_dz_le e[64][8][64]; int32_t dl[16]; } epd_dz_ltabs;
static inline const epd_dz_ltabs *epd_dz_ltabs_get(void)
{
    static epd_dz_ltabs L;
    static int built;
    if (!built) {
        const epd_dizzy_tabs *T = epd_dizzy_tabs_get();
        for (int y = 0; y < 64; y++)
            for (int k = 0; k < 8; k++)
                for (int j = 0; j < T->xn[y][k]; j++) {
                    int xx = T->xs[y][k][j];
                    L.e[y][k][j] = (epd_dz_le){ (uint8_t)xx, T->lm[y][xx] };
                }
        for (int mk = 0; mk < 16; mk++) L.dl[mk] = T->pc[mk] ? (32768 + T->pc[mk] / 2) / T->pc[mk] : 0;
        built = 1;
    }
    return &L;
}

/* the tables, built once: call before the first epd_dizzy* from more than one
 * thread (the lazy build is not thread-safe: two first calls at once corrupt
 * the layer lists for good) */
static inline void epd_dizzy_init(void)
{
    (void)epd_dz_ltabs_get();
    (void)epd_dz_split_get();
}

/* layered; returns 0, -1 when out of memory (t unchanged). Its own units
 * (1/256 level, EPD_DZ_LS): each later free neighbour gets err * dl[n] >> 15
 * (the same share each, the rounding dropped: < 1/64 level a pixel), which is
 * what made it 2x faster than the exact split (A73, a full panel
 * 154 -> 75 ms on one gold core) */
enum { EPD_DZ_LS = 256, EPD_DZ_LTOP = 30 * EPD_DZ_LS, EPD_DZ_LMID = 15 * EPD_DZ_LS };
static inline int epd_dizzy(uint8_t *t, int w, int h, int stride, int x0, int y0)
{
    if (w <= 0 || h <= 0) return 0;
    const epd_dz_ltabs *L = epd_dz_ltabs_get();
    const epd_dizzy_tabs *T = epd_dizzy_tabs_get();
    size_t wp = (size_t)w + 2;                              /* (a 1-pixel border left and right) */
    int16_t *ring = epd_dz_buf(16 * wp, 0);
    if (!ring) return -1;
    memset(ring, 0, 16 * wp * sizeof *ring);
    int tx0 = x0 >> 6, tx1 = (x0 + w + 63) >> 6, ty0 = y0 >> 6, ntx;
    const uint8_t *tf = epd_dz_tiles(t, w, h, stride, x0, y0, &ntx);
    if (!tf) return -1;
    for (int s = 0; s < h + 7; s++) {
        if (s + 1 < h && s >= 1) memset(ring + (size_t)((s + 1) & 15) * wp, 0, wp * sizeof *ring);
        for (int k = 0; k < 8; k++) {
            int y = s - k;
            if (y < 0 || y >= h) continue;
            int py = y0 + y, yy = py & 63, n_ = T->xn[yy][k];
            const epd_dz_le *es = L->e[yy][k];
            int16_t *a = ring + (size_t)(y & 15) * wp + 1, *ad = ring + (size_t)((y + 1) & 15) * wp + 1,
                    *au = ring + (size_t)((y - 1) & 15) * wp + 1;
            uint8_t *tr = t + (size_t)y * stride;
            const uint8_t *tfr = tf + (size_t)((py >> 6) - ty0) * ntx - tx0;
            int inner = y > 0 && y < h - 1;
            for (int tx = tx0; tx < tx1; tx++) {
                if (!tfr[tx]) continue;             /* (no free pixel in the tile) */
                int base = (tx << 6) - x0;
                if (inner && base > 0 && base + 64 < w) {   /* (all 4 neighbours inside the rect) */
                    uint8_t *tb = tr + base;
                    int16_t *ab = a + base, *adb = ad + base, *aub = au + base;
                    for (int j = 0; j < n_; j++) {
                        int xx = es[j].xx, c = tb[xx];
                        if (c & EPD_DZ_FIXED) continue;
                        int v = c * EPD_DZ_LS + ab[xx];
                        v = v < 0 ? 0 : v > EPD_DZ_LTOP ? EPD_DZ_LTOP : v;
                        /* (masks, no branches: black or white is a coin
                         * toss in a dither, the A73 mispredicted half of them) */
                        int hi = -(v >= EPD_DZ_LMID);
                        tb[xx] = (uint8_t)(hi & 30);
                        int err = v - (hi & EPD_DZ_LTOP);
                        int mk = es[j].mk & ~((tb[xx + 1] >> 7) | (tb[xx - 1] >> 7) << 1 |
                                              (tb[xx + stride] >> 7) << 2 | (tb[xx - stride] >> 7) << 3);
                        int sh = (err * L->dl[mk]) >> 15;
                        ab[xx + 1] = (int16_t)(ab[xx + 1] + (sh & -(mk & 1)));
                        ab[xx - 1] = (int16_t)(ab[xx - 1] + (sh & -(mk >> 1 & 1)));
                        adb[xx] = (int16_t)(adb[xx] + (sh & -(mk >> 2 & 1)));
                        aub[xx] = (int16_t)(aub[xx] + (sh & -(mk >> 3 & 1)));
                    }
                    continue;
                }
                for (int j = 0; j < n_; j++) {      /* (a tile on the rect's edge) */
                    int x = base + es[j].xx;
                    if (x < 0 || x >= w) continue;
                    int c = tr[x];
                    if (c & EPD_DZ_FIXED) continue;
                    int mk = es[j].mk;
                    if (x + 1 >= w || tr[x + 1] & EPD_DZ_FIXED) mk &= ~EPD_DZ_R;
                    if (x == 0 || tr[x - 1] & EPD_DZ_FIXED) mk &= ~EPD_DZ_L;
                    if (y + 1 >= h || tr[x + stride] & EPD_DZ_FIXED) mk &= ~EPD_DZ_D;
                    if (y == 0 || tr[x - stride] & EPD_DZ_FIXED) mk &= ~EPD_DZ_U;
                    int v = c * EPD_DZ_LS + a[x];
                    v = v < 0 ? 0 : v > EPD_DZ_LTOP ? EPD_DZ_LTOP : v;
                    int hi = v >= EPD_DZ_LMID;
                    tr[x] = (uint8_t)(hi ? 30 : 0);
                    int sh = ((v - (hi ? EPD_DZ_LTOP : 0)) * L->dl[mk]) >> 15;
                    if (mk & EPD_DZ_R) a[x + 1] = (int16_t)(a[x + 1] + sh);
                    if (mk & EPD_DZ_L) a[x - 1] = (int16_t)(a[x - 1] + sh);
                    if (mk & EPD_DZ_D) ad[x] = (int16_t)(ad[x] + sh);
                    if (mk & EPD_DZ_U) au[x] = (int16_t)(au[x] + sh);
                }
            }
        }
    }
    return 0;
}

/* serial (the gist's); returns 0, -1 when out of memory (t unchanged) */
static inline int epd_dizzy_serial(uint8_t *t, int w, int h, int stride, int x0, int y0)
{
    if (w <= 0 || h <= 0) return 0;
    const epd_dizzy_tabs *T = epd_dizzy_tabs_get();
    const epd_dz_split *S = epd_dz_split_get();
    size_t wp = (size_t)w + 2;                              /* (a 1-pixel border all round) */
    int16_t *v0 = epd_dz_buf(wp * ((size_t)h + 2), 1);
    if (!v0) return -1;
    int16_t *v = v0 + wp + 1;
    memset(v0, 0, wp * sizeof *v0);
    memset(v0 + wp * ((size_t)h + 1), 0, wp * sizeof *v0);
    for (int y = 0; y < h; y++) {
        const uint8_t *tr = t + (size_t)y * stride;
        int16_t *vr = v + (size_t)y * wp;
        vr[-1] = vr[w] = 0;
        for (int x = 0; x < w; x++) vr[x] = (int16_t)(tr[x] & EPD_DZ_FIXED ? -1 : tr[x] * EPD_DZ_S);
    }
    const uint16_t *perm = epd_dizzy_perm();
    int tx0 = x0 >> 6, tx1 = (x0 + w + 63) >> 6, ty0 = y0 >> 6, ty1 = (y0 + h + 63) >> 6, ntx;
    const uint8_t *tf = epd_dz_tiles(t, w, h, stride, x0, y0, &ntx);
    if (!tf) return -1;
    for (int ty = ty0; ty < ty1; ty++)
        for (int tx = tx0; tx < tx1; tx++) {
            if (!tf[(size_t)(ty - ty0) * ntx + (tx - tx0)]) continue;
            int bx = (tx << 6) - x0, by = (ty << 6) - y0;
            int edge = bx <= 0 || by <= 0 || bx + 64 >= w || by + 64 >= h;
            for (int k = 0; k < 4096; k++) {
                int pp = perm[k], xx = pp & 63, yy = pp >> 6, x = bx + xx, y = by + yy;
                int mk = T->sm[yy][xx];
                if (edge) {
                    if (x < 0 || y < 0 || x >= w || y >= h) continue;
                    mk &= ~((x == 0 ? EPD_DZ_L : 0) | (x == w - 1 ? EPD_DZ_R : 0) |
                            (y == 0 ? EPD_DZ_U : 0) | (y == h - 1 ? EPD_DZ_D : 0));
                }
                int16_t *o = v + (size_t)y * wp + x;
                int old = *o, hi = -(old >= EPD_DZ_MID), sh[4];
                if (old < 0) continue;              /* (fixed) */
                int err = old - (EPD_DZ_TOP & hi);
                *o = (int16_t)(EPD_DZ_TOP & hi);
                if (!err) continue;
                if ((mk & EPD_DZ_R) && o[1] < 0) mk &= ~EPD_DZ_R;
                if ((mk & EPD_DZ_L) && o[-1] < 0) mk &= ~EPD_DZ_L;
                if ((mk & EPD_DZ_D) && o[wp] < 0) mk &= ~EPD_DZ_D;
                if ((mk & EPD_DZ_U) && o[-(ptrdiff_t)wp] < 0) mk &= ~EPD_DZ_U;
                epd_dz_shares(S, err, mk, sh);
#define EPD_DZ_ADDS(p, d) do { int c_ = *(p) + sh[d]; if (sh[d]) *(p) = (int16_t)(c_ < 0 ? 0 : c_ > EPD_DZ_TOP ? EPD_DZ_TOP : c_); } while (0)   /* (a 0 share leaves a fixed -1 alone) */
                EPD_DZ_ADDS(o + 1, 0); EPD_DZ_ADDS(o - 1, 1); EPD_DZ_ADDS(o + wp, 2); EPD_DZ_ADDS(o - wp, 3);
#undef EPD_DZ_ADDS
            }
        }
    for (int y = 0; y < h; y++) {
        uint8_t *tr = t + (size_t)y * stride;
        const int16_t *vr = v + (size_t)y * wp;
        for (int x = 0; x < w; x++)
            if (vr[x] >= 0) tr[x] = (uint8_t)(vr[x] ? 30 : 0);
    }
    return 0;
}

/* ---- block dizzy : the GPU's form (epd-ingest.c) ----
 * Error diffusion in a random order inside small panel-aligned blocks (8 x 8
 * by default), every block the work of one GPU invocation in registers: no
 * shared memory, no barriers (the layered form's barriers cost
 * 232-417 ms a panel on the A509). Inside a block it is the serial dizzy (the
 * gist's): a pixel snaps and gives err / n to each of its free 4-neighbours
 * (not fixed, inside the rect) that come later -- later in the block's order,
 * or in a block of a later pass (below) -- the layered form's lean share
 * (EPD_DZ_LS units); a pixel with none gives all of it to the next free pixel
 * in the order (the next two tried), so only the last free pixel's error is
 * dropped: a block's tone is right to half a dot (0.23 levels of its mean at
 * 64 px; the layered form loses ~0.69).
 * Across blocks: the blocks are coloured in `passes` (1, 2: a checkerboard,
 * 4: a 2 x 2 cell, pass = (by & 1) * 2 + (bx & 1)) and run pass by pass; an
 * edge pixel gives to its neighbour across the seam when that block's pass is
 * later (the share goes into the receiver's per-side slot, added to its
 * accumulator when its pass starts), so every seam has error flowing one way
 * across it, like the layers of the layered form. With 1 pass the seams are
 * walls (test/epd-dzblock-test measures the 8-px grid they leave).
 * N orders, one per class of block; a block's class is a fixed shuffle of the
 * 64 x 64 panel tile's block positions, so the pattern has the tile's 64-px
 * period, like blue noise and the layered form (8 orders repeat 8 block
 * patterns on the 8-px lattice: visible in the spectrum, hence the passes).
 * The GPU shader is this very loop unrolled (epd-ingest.c gdz_block_src reads
 * these tables), bit for bit: a free pixel's accumulator is its level * 256 +
 * EPD_DZB_BIAS, a fixed or outside one 0 (received error can't take a free
 * one under EPD_DZB_FREE). Not the layered form's dots. */
enum { EPD_DZB_MAXPX = 64, EPD_DZB_MAXNB = 128, EPD_DZB_MAXORD = 64, EPD_DZB_MAXSIDE = 16,
       EPD_DZB_BIAS = 1 << 16, EPD_DZB_FREE = 1 << 15 };
enum { EPD_DZB_R, EPD_DZB_L, EPD_DZB_D, EPD_DZB_U };   /* sides, as EPD_DZ_R.. (bit = 1 << side) */
typedef struct {
    int bw, bh, npx;                            /* the block, bw x bh = npx <= 64 */
    int nbx, nby, nb;                           /* blocks across / down a 64 x 64 tile, their count */
    int n, slots;                               /* orders (classes); blocks of a class in a tile */
    int passes;                                 /* 1, 2 or 4 (see above) */
    int aligned;                                /* every class in one pass (c % passes); see the deal in _init */
    int nsym;                                   /* the orders are n / nsym base shuffles times nsym symmetries of the block (epd_dzb_sym) */
    int jit;                                    /* a zero-sum jitter: +jv to the first, -jv to the second pixel of the order (both free), jv in [-jit, jit] by block position */
    int16_t jv[EPD_DZB_MAXNB];
    uint8_t ord[EPD_DZB_MAXORD][EPD_DZB_MAXPX]; /* ord[c][k]: the k-th pixel of class c, y * bw + x */
    uint8_t cls[EPD_DZB_MAXNB];                 /* cls[by * nbx + bx]: the block's class */
    uint8_t cpos[EPD_DZB_MAXORD][EPD_DZB_MAXNB];/* cpos[c][s]: class c's s-th block, by * nbx + bx */
    int32_t dl[5];                              /* 32768 / n */
} epd_dz_btabs;
static inline uint32_t epd_dzb_rnd(uint32_t *s) { *s = *s * 1664525u + 1013904223u; return *s; }
/* a block's pass (its colour in the pass order) */
static inline int epd_dzb_pass(const epd_dz_btabs *T, int bx, int by)
{
    return T->passes == 1 ? 0 : T->passes == 2 ? (bx + by) & 1 : (by & 1) * 2 + (bx & 1);
}
/* the block symmetries: 0 as is, 1 x mirrored, 2 y mirrored, 3 both, 4..7 the same transposed (square blocks only) */
static inline int epd_dzb_sym(int s, int x, int y, int bw, int bh)
{
    if (s & 4) { int t = x; x = y; y = t; }
    if (s & 1) x = bw - 1 - x;
    if (s & 2) y = bh - 1 - y;
    return y * bw + x;
}
/* -1: a shape this can't do (bw 8 | 16, bh 4 | 8, npx <= 64; n a power of 2 dividing the blocks of a tile; passes 1 | 2 | 4) */
static inline int epd_dz_btabs_init(epd_dz_btabs *T, int bw, int bh, int n, int passes, int jit, int nsym)
{
    if ((bw != 8 && bw != 16) || (bh != 4 && bh != 8) || bw * bh > EPD_DZB_MAXPX) return -1;
    if (passes != 1 && passes != 2 && passes != 4) return -1;
    if ((nsym != 1 && nsym != 2 && nsym != 4 && nsym != 8) || (nsym == 8 && bw != bh) || n % nsym) return -1;
    int nbx = 64 / bw, nby = 64 / bh, nb = nbx * nby;
    if (n < 1 || n > EPD_DZB_MAXORD || (n & (n - 1)) || nb % n || nb > EPD_DZB_MAXNB) return -1;
    memset(T, 0, sizeof *T);
    T->bw = bw; T->bh = bh; T->npx = bw * bh; T->nbx = nbx; T->nby = nby; T->nb = nb; T->n = n; T->slots = nb / n;
    T->passes = passes;
    T->jit = jit;
    T->nsym = nsym;
    for (int c = 0; c < n / nsym; c++) {        /* the base orders (Fisher-Yates, a fixed seed per class) */
        uint32_t s = 0x9E3779B9u ^ (uint32_t)(c * 0x85EBCA6B + bw * 131 + bh);
        for (int i = 0; i < T->npx; i++) T->ord[c][i] = (uint8_t)i;
        for (int i = T->npx - 1; i > 0; i--) {
            int j = (int)((uint64_t)epd_dzb_rnd(&s) * (uint64_t)(i + 1) >> 32);
            uint8_t v = T->ord[c][i]; T->ord[c][i] = T->ord[c][j]; T->ord[c][j] = v;
        }
    }
    for (int c = n / nsym; c < n; c++)          /* their images under the symmetries */
        for (int i = 0; i < T->npx; i++) {
            int p = T->ord[c % (n / nsym)][i], x = p % bw, y = p / bw;
            T->ord[c][i] = (uint8_t)epd_dzb_sym(c / (n / nsym), x, y, bw, bh);
        }
    uint8_t p[EPD_DZB_MAXNB];
    uint32_t s = 0xC2B2AE35u ^ (uint32_t)(nb * 7 + n);
    for (int i = 0; i < nb; i++) p[i] = (uint8_t)i;
    for (int i = nb - 1; i > 0; i--) {
        int j = (int)((uint64_t)epd_dzb_rnd(&s) * (uint64_t)(i + 1) >> 32);
        uint8_t v = p[i]; p[i] = p[j]; p[j] = v;
    }
    /* the deal: class c gets `slots` of the shuffled positions; with n >= passes
     * all of a class's blocks are in one pass (c % passes: the GPU then runs a
     * class as one dispatch), else a class spans the passes */
    T->aligned = n % passes == 0;
    for (int c = 0, k = 0; c < n; c++)
        for (int j = 0; j < T->slots; j++) {
            if (T->aligned) { while (epd_dzb_pass(T, p[k] % nbx, p[k] / nbx) != c % passes) k = (k + 1) % nb; }
            T->cls[p[k]] = (uint8_t)c;
            T->cpos[c][j] = p[k];
            p[k] = p[--nb];                     /* (taken) */
            if (nb) k %= nb;
        }
    nb = T->nb;
    for (int i = 0; i < nb; i++) { uint32_t r = epd_dzb_rnd(&s) >> 16; T->jv[i] = (int16_t)((int)(r * (uint32_t)(2 * jit + 1) >> 16) - jit); }
    for (int k = 1; k <= 4; k++) T->dl[k] = (32768 + k / 2) / k;
    return 0;
}
/* the default tables (8 x 8, 16 base orders x the 4 mirror symmetries, 4 passes: what the GPU form runs; the transposes are not on the GPU), built once */
static inline const epd_dz_btabs *epd_dz_btabs_get(void)
{
    static epd_dz_btabs T;
    static int built;
    if (!built) { epd_dz_btabs_init(&T, 8, 8, 64, 4, 0, 4); built = 1; }
    return &T;
}
/* the sides a block gives on (the neighbour's pass is later) */
static inline int epd_dzb_gives(const epd_dz_btabs *T, int bx, int by)
{
    int p = epd_dzb_pass(T, bx, by), m = 0;
    if (epd_dzb_pass(T, bx + 1, by) > p) m |= 1 << EPD_DZB_R;
    if (epd_dzb_pass(T, bx - 1, by) > p) m |= 1 << EPD_DZB_L;
    if (epd_dzb_pass(T, bx, by + 1) > p) m |= 1 << EPD_DZB_D;
    if (epd_dzb_pass(T, bx, by - 1) > p) m |= 1 << EPD_DZB_U;
    return m;
}
/* the pixel of side s at index i (the row for R / L, the column for D / U) */
static inline int epd_dzb_side_px(const epd_dz_btabs *T, int s, int i)
{
    return s == EPD_DZB_R ? i * T->bw + T->bw - 1 : s == EPD_DZB_L ? i * T->bw : s == EPD_DZB_D ? (T->bh - 1) * T->bw + i : i;
}
/* one block, the reference the shader is unrolled from. a[]: the block's
 * accumulators (level * 256 + BIAS + what earlier passes gave; 0 fixed /
 * outside); gives: the sides whose neighbours come later; ofree[s]: bit i =
 * the pixel across side s at index i is free; out[s][i]: what side s's pixel
 * i gives across (every pixel of a giving side writes it, 0 when nothing);
 * dots: bit y * bw + x */
static inline void epd_dzb_block(const epd_dz_btabs *T, int c, int32_t *a, int gives, const uint32_t ofree[4],
                                 int32_t out[4][EPD_DZB_MAXSIDE], uint64_t *dots)
{
    const uint8_t *o = T->ord[c];
    int bw = T->bw, bh = T->bh, npx = T->npx;
    uint64_t d = 0;
    for (int k = 0; k < npx; k++) {
        int p = o[k], x = p % bw, y = p / bw;
        int free = a[p] >= EPD_DZB_FREE;                    /* (else fixed, or outside the rect) */
        int v = a[p] - EPD_DZB_BIAS;
        v = v < 0 ? 0 : v > EPD_DZ_LTOP ? EPD_DZ_LTOP : v;
        int hi = v >= EPD_DZ_LMID, err = v - (hi ? EPD_DZ_LTOP : 0);
        if (free) d |= (uint64_t)hi << p;
        /* the later free 4-neighbours: in the block (later in the order) and across a giving side */
        int later[4], m = 0, os[4], nos = 0;                /* (os: the sides given across) */
        static const int dx[4] = { 1, -1, 0, 0 }, dy[4] = { 0, 0, 1, -1 };
        for (int s = 0; s < 4; s++) {
            int nx = x + dx[s], ny = y + dy[s];
            if (nx >= 0 && nx < bw && ny >= 0 && ny < bh) {
                int q = ny * bw + nx, kk = 0;
                while (o[kk] != q) kk++;
                if (kk > k && a[q] >= EPD_DZB_FREE) later[m++] = q;
            } else if (gives & 1 << s) {
                int i = s < 2 ? y : x;
                if (ofree[s] >> i & 1) os[nos++] = s;
                out[s][i] = 0;                              /* (set below when given) */
            }
        }
        if (!free) continue;
        int n = m + nos;
        if (n) {
            int sh = (err * T->dl[n]) >> 15;
            for (int i = 0; i < m; i++) a[later[i]] += sh;
            for (int i = 0; i < nos; i++) out[os[i]][os[i] < 2 ? y : x] = sh;
        } else if (k + 1 < npx && a[o[k + 1]] >= EPD_DZB_FREE) a[o[k + 1]] += err;
        else if (k + 2 < npx && a[o[k + 2]] >= EPD_DZB_FREE) a[o[k + 2]] += err;
    }
    *dots = d;
}
/* block dizzy over a rect (as epd_dizzy: levels 0..30 | EPD_DZ_FIXED in
 * place, x0 / y0 the panel position); T NULL = the defaults. -1: out of memory */
static inline int epd_dizzy_block(uint8_t *t, int w, int h, int stride, int x0, int y0, const epd_dz_btabs *T)
{
    if (w <= 0 || h <= 0) return 0;
    if (!T) T = epd_dz_btabs_get();
    int bw = T->bw, bh = T->bh;
    int bx0 = x0 / bw, bx1 = (x0 + w + bw - 1) / bw, by0 = y0 / bh, by1 = (y0 + h + bh - 1) / bh, nbx = bx1 - bx0;
    /* the shares given across seams: per block of the rect, 4 sides x MAXSIDE int32 */
    size_t nin = (size_t)nbx * (by1 - by0) * 4 * EPD_DZB_MAXSIDE;
    int32_t *in = (int32_t *)epd_dz_buf(nin * 2, 1);       /* (its cells are int16: 2 a value) */
    if (!in) return -1;
    memset(in, 0, nin * sizeof *in);
#define LV(px, py) ((px) >= x0 && (px) < x0 + w && (py) >= y0 && (py) < y0 + h ? t[(size_t)((py) - y0) * stride + (px) - x0] : EPD_DZ_FIXED)
    for (int pass = 0; pass < T->passes; pass++)
        for (int by = by0; by < by1; by++)
            for (int bx = bx0; bx < bx1; bx++) {
                if (epd_dzb_pass(T, bx, by) != pass) continue;
                int c = T->cls[(by % T->nby) * T->nbx + bx % T->nbx], gives = epd_dzb_gives(T, bx, by);
                int32_t a[EPD_DZB_MAXPX], *me = in + ((size_t)(by - by0) * nbx + (bx - bx0)) * 4 * EPD_DZB_MAXSIDE;
                int any = 0;
                for (int p = 0; p < T->npx; p++) {
                    uint8_t l = LV(bx * bw + p % bw, by * bh + p / bw);
                    a[p] = l & EPD_DZ_FIXED ? 0 : l * EPD_DZ_LS + EPD_DZB_BIAS;
                    any |= !(l & EPD_DZ_FIXED);
                }
                if (!any) continue;
                if (T->jit) {                       /* (the zero-sum jitter, when both pixels are free) */
                    int p0 = T->ord[c][0], p1 = T->ord[c][1], jv = T->jv[(by % T->nby) * T->nbx + bx % T->nbx];
                    if (a[p0] >= EPD_DZB_FREE && a[p1] >= EPD_DZB_FREE) { a[p0] += jv; a[p1] -= jv; }
                }
                uint32_t ofree[4] = { 0, 0, 0, 0 };
                int32_t out[4][EPD_DZB_MAXSIDE];
                for (int s = 0; s < 4; s++) {
                    int len = s < 2 ? bh : bw;
                    for (int i = 0; i < len; i++) {
                        int p = epd_dzb_side_px(T, s, i), x = p % bw, y = p / bw;
                        int qx = bx * bw + x + (s == EPD_DZB_R) - (s == EPD_DZB_L), qy = by * bh + y + (s == EPD_DZB_D) - (s == EPD_DZB_U);
                        if (gives & 1 << s) ofree[s] |= (uint32_t)!(LV(qx, qy) & EPD_DZ_FIXED) << i;
                        else if (T->passes > 1 && a[p] >= EPD_DZB_FREE) a[p] += me[s * EPD_DZB_MAXSIDE + i];   /* (what the earlier neighbour gave) */
                    }
                }
                uint64_t d;
                epd_dzb_block(T, c, a, gives, ofree, out, &d);
                for (int s = 0; s < 4; s++) {               /* into the later neighbours' slots (the opposite side) */
                    if (!(gives & 1 << s)) continue;
                    int nbxx = bx + (s == EPD_DZB_R) - (s == EPD_DZB_L), nbyy = by + (s == EPD_DZB_D) - (s == EPD_DZB_U);
                    if (nbxx < bx0 || nbxx >= bx1 || nbyy < by0 || nbyy >= by1) continue;
                    int32_t *nb = in + ((size_t)(nbyy - by0) * nbx + (nbxx - bx0)) * 4 * EPD_DZB_MAXSIDE + (s ^ 1) * EPD_DZB_MAXSIDE;
                    for (int i = 0; i < (s < 2 ? bh : bw); i++) nb[i] = out[s][i];
                }
                for (int p = 0; p < T->npx; p++) {
                    int x = bx * bw + p % bw - x0, y = by * bh + p / bw - y0;
                    if (x < 0 || x >= w || y < 0 || y >= h) continue;
                    uint8_t *tp = t + (size_t)y * stride + x;
                    if (!(*tp & EPD_DZ_FIXED)) *tp = (uint8_t)((d >> p & 1) * 30);
                }
            }
#undef LV
    return 0;
}

#endif
