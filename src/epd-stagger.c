/* epd-stagger — staggered cut steps: the CPU reference (epd-stagger.h) */
#include <stdlib.h>
#include <string.h>
#include "epd-stagger.h"
#include "epd-bluenoise.h"
#include "epd-dither.h"
#ifdef __aarch64__
#include <arm_neon.h>
#elif defined(__SSE2__)
#include <emmintrin.h>
#endif

uint8_t epd_stag_dqt[64][64];
unsigned epd_stag_dgen;
void epd_stag_dither_set(int pattern)
{
    for (int y = 0; y < 64; y++)
        for (int x = 0; x < 64; x++) epd_stag_dqt[y][x] = (uint8_t)epd_stag_dq(epd_dither_rank(x, y, pattern));
    epd_stag_dgen++;
}

int epd_stag_cells_ok(int w, int h)
{
    return (w == 8 || w == 16 || w == 32 || w == 64) && h >= 1 && h <= 64 && !(h & (h - 1)) && (64 / w) * (64 / h) >= 2;
}

int epd_stag_cells_parse(const char *s, int *w, int *h)
{
    if (strncmp(s, "cells", 5)) return -1;
    s += 5;
    int a = 8, b;
    if (*s >= '0' && *s <= '9') a = (int)strtol(s, (char **)&s, 10);
    b = a;
    if (*s == 'x' && s[1] >= '0' && s[1] <= '9') b = (int)strtol(s + 1, (char **)&s, 10);
    if (!epd_stag_cells_ok(a, b)) return -1;
    *w = a, *h = b;
    return 0;
}

static const uint16_t *cell_tabs[2][4][7];      /* [ord][log2 w - 3][log2 h] */

/* the key cell k (j << tx | i, row j, column i) of tile (px, py) is sorted
 * by: blue noise its rank; wbayer the top 4 bits of the Bayer index of its
 * (cell column, cell row), then k (raster order: consecutive places stay
 * near each other in a class) */
static int cell_key(int ord, int px, int py, int tx, int ty, int k)
{
    int cx = px << tx | (k & ((1 << tx) - 1)), cy = py << ty | k >> tx;
    if (ord == EPD_STAG_CORD_WBAYER) return (epd_stag_brank(cx, cy) >> 8) << 9 | k;
    return epd_bn_rank[cy][cx];
}

const uint16_t *epd_stag_cells(int w, int h, int cord)
{
    if (!epd_stag_cells_ok(w, h) || cord < 0 || cord > EPD_STAG_CORD_WBAYER) return NULL;
    int wl = __builtin_ctz((unsigned)w), hl = __builtin_ctz((unsigned)h);
    const uint16_t **slot = &cell_tabs[cord][wl - 3][hl];
    const uint16_t *t = __atomic_load_n(slot, __ATOMIC_ACQUIRE);
    if (t) return t;
    uint16_t *n = malloc(2 * 4096 * sizeof *n);
    if (!n) return NULL;
    int tx = 6 - wl, ty = 6 - hl, nc = 1 << (tx + ty);      /* cells a tile: 1 << tx across, 1 << ty down */
    for (int py = 0; py < 1 << hl; py++)                    /* the tiles of the 64 x 64 cell period */
        for (int px = 0; px < 1 << wl; px++) {
            int ord[512];
            for (int k = 0; k < nc; k++) {                  /* the tile's cells by their keys (insertion sort; keys unique) */
                int key = cell_key(cord, px, py, tx, ty, k), m = k;
                while (m > 0 && cell_key(cord, px, py, tx, ty, ord[m - 1]) > key) {
                    ord[m] = ord[m - 1];
                    m--;
                }
                ord[m] = k;
            }
            for (int s = 0; s < nc; s++) {
                int k = ord[s];
                n[(py << ty | k >> tx) * 64 + (px << tx | (k & ((1 << tx) - 1)))] = (uint16_t)(s << (wl + hl));   /* s << (12 - log2 nc) */
                n[4096 + (py << ty | s >> tx) * 64 + (px << tx | (s & ((1 << tx) - 1)))] = (uint16_t)k;
            }
        }
    const uint16_t *exp = NULL;
    if (!__atomic_compare_exchange_n(slot, &exp, n, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
        free(n);                                            /* (another thread's first) */
        return exp;
    }
    return n;
}

#define NPX ((size_t)EPD_W * EPD_H)

int epd_stag_init(epd_stag *s)
{
    memset(s, 0, sizeof *s);
    s->hot = calloc(NPX, sizeof *s->hot);
    s->cold = calloc(NPX, sizeof *s->cold);
    s->tgt = calloc(NPX, 1);
    s->ld = calloc(NPX, 1);
    s->ack = calloc(NPX / 8, 1);
    if (!s->hot || !s->cold || !s->tgt || !s->ld || !s->ack) { epd_stag_free(s); return -1; }
    return 0;
}

void epd_stag_free(epd_stag *s)
{
    free(s->hot); free(s->cold); free(s->tgt); free(s->ld); free(s->ack);
    s->hot = NULL; s->cold = NULL; s->tgt = NULL; s->ld = NULL; s->ack = NULL;
}

void epd_stag_lanes_park(epd_stag *s)
{
    for (size_t i = 0; i < NPX; i++) s->hot[i] = EPD_STAG_PARKL;
    memset(s->ack, 1, NPX / 8);
}

void epd_stag_enter(epd_stag *s)
{
    for (size_t i = 0; i < NPX; i++) s->hot[i] = EPD_STAG_PARKED;
    memset(s->cold, 0, NPX * sizeof *s->cold);
    memset(s->tgt, 0, NPX);
    memset(s->ld, 0, NPX);
}

void epd_stag_repark(epd_stag *s, epd_rect r)
{
    if (r.x < 0) { r.w += r.x; r.x = 0; }
    if (r.y < 0) { r.h += r.y; r.y = 0; }
    if (r.x + r.w > EPD_W) r.w = EPD_W - r.x;
    if (r.y + r.h > EPD_H) r.h = EPD_H - r.y;
    for (int y = r.y; y < r.y + r.h && r.w > 0; y++) {
        size_t o = (size_t)y * EPD_W + r.x;
        for (int x = 0; x < r.w; x++) s->hot[o + x] = EPD_STAG_PARKED;
        memset(s->cold + o, 0, (size_t)r.w * sizeof *s->cold);
        memset(s->tgt + o, 0, (size_t)r.w);
        memset(s->ld + o, 0, (size_t)r.w);
    }
}

long epd_stag_load(epd_stag *s, const epd_engine *e, const epd_stag_par *p, epd_rect r)
{
    if (r.x < 0) { r.w += r.x; r.x = 0; }
    if (r.y < 0) { r.h += r.y; r.y = 0; }
    if (r.x + r.w > EPD_W) r.w = EPD_W - r.x;
    if (r.y + r.h > EPD_H) r.h = EPD_H - r.y;
    if (r.w <= 0 || r.h <= 0) return 0;
    int keep = e->op_ready && e->op_r == p->R;
    long n = 0;
    for (int y = r.y; y < r.y + r.h; y++) {
        size_t o = (size_t)y * EPD_W + r.x;
        if (!memchr(s->ld + o, 0, (size_t)r.w)) continue;     /* (a scroll: the same rect again) */
        for (size_t i = o; i < o + (size_t)r.w; i++) {
            if (s->ld[i]) continue;
            /* (claim_state packed reads D: the same fields) */
            int l = epd_px_level(e, i), dc = epd_px_dc(e, i);
            int op = keep ? epd_px_op(e, i) : l >= 30 ? p->R : p->R * l / 30;
            s->cold[i] = epd_stag_cold(l, op, epd_px_stale(e, i), dc);
            s->hot[i] = (uint16_t)epd_stag_hotbits(l, op, dc, p);
            s->tgt[i] = (uint8_t)l;
            s->ld[i] = 2;
            n++;
        }
    }
    return n;
}

void epd_stag_exit(const epd_stag *s, epd_engine *e)
{
    int keep = e->op_ready;
    for (size_t i = 0; i < NPX; i++) {
        if (!s->ld[i]) continue;
        uint32_t c = s->cold[i];
        int b;
        int l = epd_stag_ldec(EPD_STAG_LEVEL(c), &b);      /* (payrule: an unpaid rest dropped) */
        if (e->pk) {                            /* (packed: D/W/T; check: both) */
            epd_px_put(e, i, EPD_F_LEVEL | EPD_F_TARGET | EPD_F_BUSY | (keep ? EPD_F_OP : 0) | EPD_F_DC | EPD_F_STALE,
                       l, l, 0, (int)EPD_STAG_OP(c), (int16_t)EPD_STAG_DC(c), (int)EPD_STAG_STALE(c));
            continue;
        }
        e->level[i] = e->target[i] = (uint8_t)l;
        e->busy[i] = 0;
        if (keep) e->op[i] = (uint8_t)EPD_STAG_OP(c);
        e->dc[i] = (int16_t)EPD_STAG_DC(c);
        e->stale[i] = (uint8_t)EPD_STAG_STALE(c);
    }
}

void epd_stag_target(epd_stag *s, epd_rect r, const uint8_t *levels)
{
    for (int y = 0; y < r.h; y++)
        memcpy(s->tgt + (size_t)(r.y + y) * EPD_W + r.x, levels + (size_t)y * r.w, r.w);
}

/* one pixel: its phase this frame; the state stepped (the GPU's pass does the
 * same per pixel: epd-gpu.c stag_cs) */
static inline unsigned px(uint16_t *hp, uint32_t *cp, int t, int dq, int grank, int fm, unsigned f6, const epd_stag_par *p)
{
    unsigned h = *hp, ph = h & 3, n = (h >> 2) & 15, age = (f6 - (h >> 6)) & 63;
    int pos = fm - (grank * p->G >> 12);    /* (grank: the group's rank; dq: the dither's, epd_stag_dqt) */
    if (pos < 0) pos += p->G;
    /* ctl 2: a step may also follow its predecessor without the idle frame
     * (age == n), but only one driving the same way (a reversal waits a cycle);
     * ctl 3: any step (k counts the pulses without an idle frame) */
    if (pos == 0 && (ph == 0 || age > n || (p->ctl >= 2 && age == n))) {   /* its group's frame, its step done */
        int tb = p->dither ? (t >= 2 * dq ? 30 : 0) : t >= 16 ? 30 : 0;
        int pr0 = p->payrule && p->ctl;
        if (p->drain || (EPD_STAG_LV(h) == (tb ? 1u : 0u) &&
                         (!p->ctl || (EPD_STAG_SET(h) && !(p->pay && !pr0 && EPD_STAG_DUE(h)))))) {
            *hp = (uint16_t)(h & ~3u);                  /* nothing to do (the hot bits say so) */
            return 0;
        }
        uint32_t c = *cp;
        int l = EPD_STAG_LEVEL(c), op = EPD_STAG_OP(c), st = EPD_STAG_STALE(c), dc = EPD_STAG_DC(c), b = 0;
        int sd = tb ? 1 : -1, nn = 0, k = 0, pr = p->payrule && p->ctl;
        if (pr) {                                       /* payrule: the budget (epd-stagger.h) */
            l = epd_stag_ldec(l, &b);
            if (l != tb) {                              /* an arrival starts: b from it (a rail only) */
                int x = -dc * sd - (tb ? p->R - op : op);
                b = (l != 0 && l != 30) || x <= 0 ? 0 : x < p->pcap ? x : p->pcap;
                if (p->htop > 0 && (l == 0 || l == 30) && (tb ? p->R - op : op) < p->R) {   /* the top-up: a reversal */
                    int t = p->htop < p->pcap ? p->htop : p->pcap;
                    b = b > t ? b : t;
                }
            }
        }
        if (!p->ctl) nn = tb != l ? p->n : 0;
        else if (p->ctl == 3) {
            int need = tb ? p->R - op : op, owe = -dc * sd;
            int want = pr ? need + b : need > 0 ? need : p->pay && owe >= p->pay ? owe : 0;
            int kin = ph && age == n ? EPD_STAG_K(c) : 0;   /* pulses since its last idle frame */
            int Gc = p->G < 16 ? p->G : 15;            /* (the 4-bit n field) */
            nn = want < Gc ? want : Gc;
            if (nn == p->G && kin + nn > p->K) nn = p->G - 1;   /* an idle frame at this cycle's end (G 1: the frame) */
            k = nn == p->G ? kin + nn : 0;
        } else if (pr) {
            int need = tb ? p->R - op : op, cap = need > 0 || p->ctl != 2 ? p->n : (p->G < 16 ? p->G : 15);
            nn = need + b < cap ? need + b : cap;
        } else {
            int need = tb ? p->R - op : op;
            if (need > 0) nn = need < p->n ? need : p->n;
            else if (p->pay && -dc * sd >= p->pay) {
                int cap = p->ctl == 2 ? (p->G < 16 ? p->G : 15) : p->n;    /* ctl 2: payback through the idle slot too (15: the n field) */
                nn = -dc * sd < cap ? -dc * sd : cap;
            }
        }
        if (p->ctl == 2 && ph && age == n && nn && ph != (tb ? 2u : 1u))
            return 0;                                   /* the idle frame first (state unchanged) */
        if (nn || tb != l) st = 1;
        if (pr) {                                       /* the frames past the need pay b off */
            int need = tb ? p->R - op : op;
            b -= nn > need ? nn - need : 0;
        }
        op += sd * nn;
        op = op < 0 ? 0 : op > p->R ? p->R : op;
        dc += sd * nn;
        dc = dc < -EPD_STAG_DCMAX ? -EPD_STAG_DCMAX : dc > EPD_STAG_DCMAX ? EPD_STAG_DCMAX : dc;
        l = pr ? epd_stag_lenc(tb, b) : tb;
        h = (nn ? (tb ? 2u : 1u) | (unsigned)nn << 2 | f6 << 6 : 0) | epd_stag_hotbits(l, op, dc, p);
        *hp = (uint16_t)h;
        *cp = epd_stag_cold(l, op, st, dc) | (uint32_t)k << 12;
        return nn ? h & 3 : 0;
    }
    return ph && age < n ? ph : 0;
}

/* lanes: one pixel (the GPU: epd-gpu.c stag_cs LANES); *pk: parked after it */
static inline unsigned px_lane(uint16_t *hp, uint32_t *cp, unsigned tb, int dq, int grank, int fm, unsigned f6, const epd_stag_par *p,
                               int *sm, int *pk)
{
    unsigned h = *hp, ph = h & 3, n = (h >> 2) & 15, age = (f6 - (h >> 6)) & 63;
    int pos = fm - (grank * p->G >> 12);
    if (pos < 0) pos += p->G;
    int done = pos == 0 && (ph == 0 || age > n || (p->ctl >= 2 && age == n)), parked = EPD_STAG_LV(h) == 3;
    *sm = done;
    if (parked && (!done || (tb & (EPD_PK_ARM | EPD_PK_EVICT)) != EPD_PK_ARM)) { *pk = 1; return 0; }
    if (parked) {                                   /* arm: the engine's state, its hot bits (stag_load_cs's) */
        uint32_t c = *cp;
        *hp = (uint16_t)epd_stag_hotbits(EPD_STAG_LEVEL(c), EPD_STAG_OP(c), EPD_STAG_DC(c), p);
    } else if (done && (tb & EPD_PK_EVICT)) {       /* evict: the final cold state, parked */
        uint32_t c = *cp;
        int b, l = epd_stag_ldec(EPD_STAG_LEVEL(c), &b);
        *cp = (c & ~31u) | (uint32_t)l;
        *hp = (uint16_t)EPD_STAG_PARKL;
        *pk = 1;
        return 0;
    }
    *pk = 0;
    return px(hp, cp, (int)(tb & 31), dq, grank, fm, f6, p);
}

static void rows_lanes(epd_stag *s, const epd_stag_par *p, const uint32_t *tpl, uint32_t *words, int y0, int y1)
{
    int fm = (int)(s->f % (unsigned)p->G);
    unsigned f6 = s->f & 63;
    const uint16_t *ct = p->stripe > 0 && p->cellw ? epd_stag_cells(p->cellw, p->cellh, p->cellord) : NULL;
    for (int y = y0; y < y1; y++) {
        size_t o = (size_t)y * EPD_W, wo = (size_t)(EPD_DATA0 / EPD_LINE + EPD_H - 1 - y) * EPD_LINE + EPD_DATA0 % EPD_LINE;
        const uint16_t *rk = epd_bn_rank[y & 63];
        const uint8_t *dr = epd_stag_dqt[y & 63];
        for (int gx = 0; gx < EPD_W / 8; gx++) {
            uint32_t w = tpl[wo + gx];
            int anysm = 0, npk = 0, ev = -1;
            for (int i = 0; i < 8; i++) {
                int x = gx * 8 + i, sm, pk;
                unsigned tb = s->tgt[o + x];
                if (ev < 0 && (tb & EPD_PK_EVICT)) ev = (tb & EPD_PK_TOG) != 0;
                w |= px_lane(s->hot + o + x, s->cold + o + x, tb, dr[x & 63], ct ? epd_stag_crank(ct, p, x, y) : epd_stag_grank(p, x, y, rk[x & 63]),
                             fm, f6, p, &sm, &pk) << (14 - 2 * i);
                anysm |= sm;
                npk += pk;
            }
            if (anysm && npk == 8 && ev >= 0) s->ack[(size_t)y * (EPD_W / 8) + gx] = (uint8_t)(1 | ev << 1);
            words[wo + gx] = w;
        }
    }
}

void epd_stag_rows(epd_stag *s, const epd_stag_par *p, const uint32_t *tpl, uint32_t *words, int y0, int y1)
{
    if (p->lanes) { rows_lanes(s, p, tpl, words, y0, y1); return; }
    int fm = (int)(s->f % (unsigned)p->G);
    unsigned f6 = s->f & 63;
    const uint16_t *ct = p->stripe > 0 && p->cellw ? epd_stag_cells(p->cellw, p->cellh, p->cellord) : NULL;   /* (cells: the table once) */
    for (int y = y0; y < y1; y++) {
        size_t o = (size_t)y * EPD_W, wo = (size_t)(EPD_DATA0 / EPD_LINE + EPD_H - 1 - y) * EPD_LINE + EPD_DATA0 % EPD_LINE;
        const uint16_t *rk = epd_bn_rank[y & 63];
        const uint8_t *dr = epd_stag_dqt[y & 63];
        for (int gx = 0; gx < EPD_W / 8; gx++) {
            uint32_t w = tpl[wo + gx];
            for (int i = 0; i < 8; i++) {
                int x = gx * 8 + i;
                int r = rk[x & 63];
                w |= px(s->hot + o + x, s->cold + o + x, s->tgt[o + x], dr[x & 63], ct ? epd_stag_crank(ct, p, x, y) : epd_stag_grank(p, x, y, r), fm, f6, p) << (14 - 2 * i);
            }
            words[wo + gx] = w;
        }
    }
}

/* ---- lanes: the CPU's side of the requests (epd-stagger.h) ---- */
static epd_rect clip(epd_rect r)
{
    if (r.x < 0) { r.w += r.x; r.x = 0; }
    if (r.y < 0) { r.h += r.y; r.y = 0; }
    if (r.x + r.w > EPD_W) r.w = EPD_W - r.x;
    if (r.y + r.h > EPD_H) r.h = EPD_H - r.y;
    if (r.w < 0) r.w = 0;
    if (r.h < 0) r.h = 0;
    return r;
}

epd_rect epd_stag_lane_pad(epd_rect r)
{
    r = clip(r);
    if (r.w <= 0 || r.h <= 0) return (epd_rect){ 0, 0, 0, 0 };
    int x0 = r.x & ~15, x1 = (r.x + r.w + 15) & ~15;
    if (x1 > EPD_W) x1 = EPD_W;
    return (epd_rect){ x0, r.y, x1 - x0, r.h };
}

/* (16 px = one 64-B line of D: a chunk with no FIN word is left unwritten, a
 * chunk with one is resolved branch-free. Only chunks holding a FIN word are
 * stored to, and an armed lane holds none, so no store touches a line of the
 * GPU's. a run's start, after the engine's first cut finished
 * the whole window, spent ~27 ms here per pixel) */
__attribute__((optimize("O3")))
static long resolve_chunk(uint32_t *restrict d, uint16_t *restrict w, const uint8_t *restrict t, int n)
{
    long c = 0;
    for (int i = 0; i < n; i++) c += w[i] == EPD_PK_FIN;
    if (!c) return 0;
    for (int i = 0; i < n; i++) {
        uint32_t m = (uint32_t)-(uint32_t)(w[i] == EPD_PK_FIN);
        d[i] = (d[i] & ~(m & 31u)) | (t[i] & 31u & m);
        w[i] = (uint16_t)((w[i] & ~m) | (EPD_PK_IDLE & m));
    }
    return c;
}

/* one whole 16-px line: does it hold a FIN word (a vector test) */
static inline int line_fin(const uint16_t *w)
{
#ifdef __aarch64__
    const uint16x8_t F = vdupq_n_u16(EPD_PK_FIN);
    return vmaxvq_u16(vorrq_u16(vceqq_u16(vld1q_u16(w), F), vceqq_u16(vld1q_u16(w + 8), F))) != 0;
#elif defined(__SSE2__)
    const __m128i F = _mm_set1_epi16((short)EPD_PK_FIN);
    return _mm_movemask_epi8(_mm_or_si128(_mm_cmpeq_epi16(_mm_loadu_si128((const __m128i *)w), F),
                                          _mm_cmpeq_epi16(_mm_loadu_si128((const __m128i *)(w + 8)), F))) != 0;
#else
    int c = 0;
    for (int i = 0; i < 16; i++) c |= w[i] == EPD_PK_FIN;
    return c;
#endif
}

/* one whole 16-px line holding a FIN word: resolve_chunk's rule as a vector
 * blend */
__attribute__((optimize("O3")))
static long resolve_line(uint32_t *restrict d, uint16_t *restrict w, const uint8_t *restrict t)
{
#ifdef __aarch64__
    const uint16x8_t F = vdupq_n_u16(EPD_PK_FIN), I = vdupq_n_u16(EPD_PK_IDLE);
    const uint32x4_t L = vdupq_n_u32(31);
    uint16x8_t w0 = vld1q_u16(w), w1 = vld1q_u16(w + 8);
    uint16x8_t m[2] = { vceqq_u16(w0, F), vceqq_u16(w1, F) };
    vst1q_u16(w, vbslq_u16(m[0], I, w0));
    vst1q_u16(w + 8, vbslq_u16(m[1], I, w1));
    uint8x16_t tb = vld1q_u8(t);
    uint16x8_t tw[2] = { vmovl_u8(vget_low_u8(tb)), vmovl_u8(vget_high_u8(tb)) };
    for (int h = 0; h < 2; h++) {
        uint32x4_t ml = vreinterpretq_u32_s32(vmovl_s16(vreinterpret_s16_u16(vget_low_u16(m[h]))));
        uint32x4_t mh = vreinterpretq_u32_s32(vmovl_s16(vreinterpret_s16_u16(vget_high_u16(m[h]))));
        uint32_t *dp = d + 8 * h;
        vst1q_u32(dp, vbslq_u32(vandq_u32(ml, L), vmovl_u16(vget_low_u16(tw[h])), vld1q_u32(dp)));
        vst1q_u32(dp + 4, vbslq_u32(vandq_u32(mh, L), vmovl_u16(vget_high_u16(tw[h])), vld1q_u32(dp + 4)));
    }
    return vaddvq_u16(vaddq_u16(vshrq_n_u16(m[0], 15), vshrq_n_u16(m[1], 15)));
#elif defined(__SSE2__)
    const __m128i F = _mm_set1_epi16((short)EPD_PK_FIN), I = _mm_set1_epi16(EPD_PK_IDLE), L = _mm_set1_epi32(31), Z = _mm_setzero_si128();
    __m128i w0 = _mm_loadu_si128((const __m128i *)w), w1 = _mm_loadu_si128((const __m128i *)(w + 8));
    __m128i m[2] = { _mm_cmpeq_epi16(w0, F), _mm_cmpeq_epi16(w1, F) };
    _mm_storeu_si128((__m128i *)w, _mm_or_si128(_mm_and_si128(m[0], I), _mm_andnot_si128(m[0], w0)));
    _mm_storeu_si128((__m128i *)(w + 8), _mm_or_si128(_mm_and_si128(m[1], I), _mm_andnot_si128(m[1], w1)));
    __m128i tb = _mm_loadu_si128((const __m128i *)t);
    __m128i tw[2] = { _mm_unpacklo_epi8(tb, Z), _mm_unpackhi_epi8(tb, Z) };
    for (int h = 0; h < 2; h++) {
        __m128i ms[2] = { _mm_and_si128(_mm_unpacklo_epi16(m[h], m[h]), L), _mm_and_si128(_mm_unpackhi_epi16(m[h], m[h]), L) };
        __m128i ts[2] = { _mm_unpacklo_epi16(tw[h], Z), _mm_unpackhi_epi16(tw[h], Z) };
        for (int j = 0; j < 2; j++) {
            __m128i *dp = (__m128i *)(d + 8 * h + 4 * j), dv = _mm_loadu_si128(dp);
            _mm_storeu_si128(dp, _mm_or_si128(_mm_and_si128(ms[j], ts[j]), _mm_andnot_si128(ms[j], dv)));
        }
    }
    return __builtin_popcount((unsigned)_mm_movemask_epi8(_mm_packs_epi16(m[0], m[1])));
#else
    return resolve_chunk(d, w, t, 16);
#endif
}

long epd_stag_lane_resolve(epd_engine *e, epd_rect r)
{
    epd_pk *k = e->pk;
    long n = 0;
    int wm = 0;
    r = clip(r);
    /* (one pass, a 16-px line at a time: a line with no FIN word only read,
     * W's rows marked before the first store. the scalar
     * form with a row pre-scan was ~10 ms of a run's start) */
    for (int y = r.y; y < r.y + r.h; y++) {
        size_t q = (size_t)y * EPD_PK_S;
        int x1 = r.x + r.w;
        for (int x = r.x; x < x1; ) {
            int c = 16 - (x & 15);      /* (to the next 16-px line) */
            if (c > x1 - x) c = x1 - x;
            uint16_t *w = k->W + q + x;
            int has = c == 16 ? line_fin(w) : 0;
            if (c < 16) for (int i = 0; i < c; i++) has |= w[i] == EPD_PK_FIN;
            if (has) {
                if (!wm) { epd_pk_w_rows(e, r.y, r.y + r.h); wm = 1; }
                n += c == 16 ? resolve_line(k->D + q + x, w, k->T + q + x) : resolve_chunk(k->D + q + x, w, k->T + q + x, c);
            }
            x += c;
        }
    }
    return n;
}

epd_rect epd_stag_lane_arm(epd_engine *e, epd_rect r, const uint8_t *tgt, const uint8_t *mask)
{
    epd_pk *k = e->pk;
    epd_rect c = clip(r);
    r = epd_stag_lane_pad(r);
    for (int y = r.y; y < r.y + r.h; y++) {
        size_t o = (size_t)y * EPD_W, q = (size_t)y * EPD_PK_S;
        for (int x = r.x; x < r.x + r.w; x++) {
            if (mask && !mask[o + x]) continue;
            int in = x >= c.x && x < c.x + c.w;             /* (the pad: its own target) */
            k->T[q + x] = (uint8_t)(((in ? tgt[o + x] : k->T[q + x]) & 31) | EPD_PK_ARM);
        }
    }
    return r;
}

epd_rect epd_stag_lane_evict(epd_engine *e, epd_rect r, const uint8_t *ack)
{
    epd_pk *k = e->pk;
    r = epd_stag_lane_pad(r);
    /* (a word's 8 bytes at once: the pad keeps whole words; the
     * per-pixel form was most of a run end's ~15 ms evict) */
    const uint64_t lo = 0x1F1F1F1F1F1F1F1Full, ones = 0x0101010101010101ull;
    for (int y = r.y; y < r.y + r.h; y++) {
        uint8_t *t = k->T + (size_t)y * EPD_PK_S;
        const uint8_t *a = ack + (size_t)y * (EPD_W / 8);
        for (int x = r.x; x < r.x + r.w; x += 8) {
            uint64_t rq = (uint64_t)(EPD_PK_EVICT | ((a[x / 8] >> 1) & 1 ? 0 : EPD_PK_TOG)), v;   /* (the word's other tog) */
            memcpy(&v, t + x, 8);
            v = (v & lo) | rq * ones;
            memcpy(t + x, &v, 8);
        }
    }
    return r;
}

long epd_stag_lane_unacked(const epd_engine *e, const uint8_t *ack, epd_rect r)
{
    const epd_pk *k = e->pk;
    long n = 0;
    r = clip(r);
    for (int y = r.y; y < r.y + r.h; y++)
        for (int x = r.x / 8; x < (r.x + r.w + 7) / 8; x++) {
            uint8_t t = k->T[(size_t)y * EPD_PK_S + (size_t)x * 8];
            n += ack[(size_t)y * (EPD_W / 8) + x] != (uint8_t)(1 | (t & EPD_PK_TOG ? 2 : 0));
        }
    return n;
}

void epd_stag_lane_takeover(epd_engine *e, epd_rect r, uint8_t *ld)
{
    epd_pk *k = e->pk;
    r = clip(r);
    for (int y = r.y; y < r.y + r.h; y++) {
        size_t o = (size_t)y * EPD_W, q = (size_t)y * EPD_PK_S;
        uint8_t *t = k->T + q;
        const uint32_t *d = k->D + q;
        if (!ld) {
            for (int x = r.x; x < r.x + r.w; x++) t[x] &= 31;
            continue;
        }
        /* (branch-free, then ld's row cleared: the loop vectorizes) */
        uint8_t *l = ld + o;
        for (int x = r.x; x < r.x + r.w; x++) {
            uint8_t m = (uint8_t)-(l[x] != 0);
            t[x] = (uint8_t)(((uint8_t)d[x] & m) | (t[x] & (uint8_t)~m)) & 31;
        }
        memset(l + r.x, 0, (size_t)r.w);
    }
}

void epd_stag_lane_mirror(epd_stag *s, const epd_engine *e, epd_rect r)
{
    const epd_pk *k = e->pk;
    r = clip(r);
    for (int y = r.y; y < r.y + r.h; y++) {
        size_t o = (size_t)y * EPD_W, q = (size_t)y * EPD_PK_S;
        memcpy(s->tgt + o + r.x, k->T + q + r.x, (size_t)r.w);
        for (int x = r.x; x < r.x + r.w; x++)
            if (EPD_STAG_LV(s->hot[o + x]) == 3) s->cold[o + x] = k->D[q + x];
    }
}

long epd_stag_active(const epd_stag *s)
{
    long n = 0;
    unsigned f6 = s->f & 63;
    for (size_t i = 0; i < NPX; i++) {
        unsigned h = s->hot[i];
        n += (h & 3) && ((f6 - (h >> 6)) & 63) <= ((h >> 2) & 15);
    }
    return n;
}
