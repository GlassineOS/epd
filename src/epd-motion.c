/* epd-motion — see epd-motion.h */
#include "epd-motion.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

void mo_init(motion *m, double screen_h)
{
    memset(m, 0, sizeof *m);
    m->screen_h = screen_h > 0 ? screen_h : 1872;
    m->up_t = m->evidence_t = m->burst_t = m->shift_t = INT64_MIN / 2;
}

static void evidence(motion *m, int64_t t)
{
    m->moving = 1;
    m->evidence_t = t;
}

void mo_down(motion *m, int64_t t, double x, double y)
{
    m->down = 1;
    m->pinch = m->finger_gesture = 0;
    m->x0 = m->x = x;
    m->y0 = m->y = y;
    m->fv = 0;
    m->ft = m->fmoved = t;
}

void mo_move(motion *m, int64_t t, double x, double y)
{
    if (!m->down) return;
    double dx = x - m->x, dy = y - m->y;
    if (t > m->ft) {
        double v = sqrt(dx * dx + dy * dy) / ((t - m->ft) / 1e3);
        m->fv = m->fv * 0.6 + v * 0.4;
    }
    if (dx || dy) m->fmoved = t;
    m->x = x; m->y = y; m->ft = t;
    if (!m->finger_gesture && (fabs(x - m->x0) > MO_SLOP || fabs(y - m->y0) > MO_SLOP)) m->finger_gesture = 1;
    if (m->finger_gesture) evidence(m, t);
}

void mo_pinch(motion *m, int64_t t)
{
    m->pinch = m->finger_gesture = 1;
    evidence(m, t);
}

void mo_up(motion *m, int64_t t)
{
    if (!m->down) return;
    m->down = 0;
    m->up_t = t;
    if (m->finger_gesture) evidence(m, t);   /* a fling: the content goes on */
}

void mo_burst(motion *m, int64_t t, int measured, int shift, double score)
{
    m->bursts++;
    int shifted = measured && shift && score >= MO_SHIFT_SCORE;
    if (shifted) {
        m->shifts++;
        int64_t dt = t - m->burst_t;
        double v = dt > 0 && dt <= MO_SHIFT_DT_MS ? fabs((double)shift) / m->screen_h / (dt / 1e3) : 0;
        if (v > 0) m->cv = t - m->shift_t <= MO_SHIFT_DT_MS && m->cv > 0 ? m->cv * 0.5 + v * 0.5 : v;
        else m->cv = 0;
        /* without a finger one shifted burst may be a layout jump (a page
         * pushing its content down while it loads): two close ones are motion */
        if (m->down || t - m->evidence_t <= MO_HOLD_MS || t - m->shift_t <= MO_ALONE_MS) {
            if (!m->moving && m->down && !m->finger_gesture) m->early++;
            evidence(m, t);
        }
        m->shift_t = t;
        m->last_shift = shift;
    }
    m->burst_t = t;
}

int mo_state(motion *m, int64_t t, double *v)
{
    if (m->moving && !m->down && t - m->evidence_t > MO_HOLD_MS) m->moving = 0;
    if (m->moving && m->down && !m->finger_gesture && t - m->evidence_t > MO_HOLD_MS) m->moving = 0;   /* a scroll that stopped under a resting finger */
    m->by_touch = m->moving && (m->down || t - m->up_t <= MO_HOLD_MS);
    double s = 0;
    if (m->moving) {
        if (t - m->shift_t <= MO_SHIFT_DT_MS && m->cv > 0) s = m->cv;          /* what the panel has to show */
        else if (m->down && t - m->fmoved < MO_FRESH_MS) s = m->fv;             /* the finger leads */
    }
    if (v) *v = s;
    return m->moving ? MO_MOVING : m->down ? MO_PRESS : MO_STILL;
}

int mo_wants_shift(const motion *m, int64_t t)
{
    return m->down || m->moving || t - m->evidence_t <= 2 * MO_HOLD_MS || t - m->shift_t <= MO_ALONE_MS;
}

/* ---- the shift ---------------------------------------------------------- */
void mo_avg4(uint8_t *dst, const uint8_t *r, long stride, int n)
{
    const uint8_t *a = r, *b = r + stride, *c = r + 2 * stride, *d = r + 3 * stride;
    for (int j = 0; j < n; j++) dst[j] = (uint8_t)((a[j] + b[j] + c[j] + d[j] + 2) >> 2);
}

#define MO_K_MAX 32
#define MO_N_MAX 2048

/* sum |a[j] - b[j]|, j < n (n <= MO_N_MAX). arm64: explicit NEON -- gcc -O2
 * left the scalar loop as is, 5.7 ms per measurement on the A53 (the x86 host
 * vectorized it itself: 0.16 ms). 16-bit lanes take 2 x 255 per step: up to
 * 128 steps = 2048 px */
#if defined(__aarch64__)
#include <arm_neon.h>
static unsigned sad_row(const uint8_t *a, const uint8_t *b, int n)
{
    uint16x8_t acc = vdupq_n_u16(0);
    int j = 0;
    for (; j + 16 <= n; j += 16) acc = vpadalq_u8(acc, vabdq_u8(vld1q_u8(a + j), vld1q_u8(b + j)));
    unsigned sum = vaddlvq_u16(acc);
    for (; j < n; j++) sum += (unsigned)abs((int)a[j] - (int)b[j]);
    return sum;
}
#else
static unsigned sad_row(const uint8_t *a, const uint8_t *b, int n)
{
    unsigned sum = 0;
    for (int j = 0; j < n; j++) sum += (unsigned)abs((int)a[j] - (int)b[j]);
    return sum;
}
#endif

/* mean |new[j] - old[j + s]| over the overlap, all rows (x256 fixed point);
 * -1 when the overlap is under half the row */
static long mismatch(const uint8_t *const *o, const uint8_t *const *nw, int k, int n, int s)
{
    int j0 = s < 0 ? -s : 0, j1 = s > 0 ? n - s : n;
    if (j1 - j0 < n / 2) return -1;
    long sum = 0;
    for (int r = 0; r < k; r++) sum += sad_row(nw[r] + j0, o[r] + s + j0, j1 - j0);
    return sum * 256 / ((long)k * (j1 - j0));
}

int mo_find_shift(const uint8_t *const *oldr, const uint8_t *const *newr, int k, int n, int smax, double *score)
{
    *score = 0;
    if (k <= 0 || n < 64) return 0;
    if (k > MO_K_MAX) k = MO_K_MAX;
    if (n > MO_N_MAX) n = MO_N_MAX;
    if (smax > n / 2) smax = n / 2;
    /* coarse: rows averaged over 4 (dithered old pixels average to their grey) */
    static uint8_t co[MO_K_MAX][MO_N_MAX / 4], cn[MO_K_MAX][MO_N_MAX / 4];
    const uint8_t *cop[MO_K_MAX], *cnp[MO_K_MAX];
    int n4 = n / 4;
    for (int r = 0; r < k; r++) {
        for (int j = 0; j < n4; j++) {
            const uint8_t *a = oldr[r] + 4 * j, *b = newr[r] + 4 * j;
            co[r][j] = (uint8_t)((a[0] + a[1] + a[2] + a[3] + 2) >> 2);
            cn[r][j] = (uint8_t)((b[0] + b[1] + b[2] + b[3] + 2) >> 2);
        }
        cop[r] = co[r]; cnp[r] = cn[r];
    }
    long still = mismatch(cop, cnp, k, n4, 0);
    if (still <= 0) return 0;                   /* nothing differs here */
    int s4max = smax / 4, best = 0;
    static long mms[MO_N_MAX / 4 + 1];
    long bm = still;
    for (int s = -s4max; s <= s4max; s++) {
        long mm = s ? mismatch(cop, cnp, k, n4, s) : still;
        mms[s + s4max] = mm;
        if (!s) continue;
        /* the smallest |s| among (near) equals: a repeating list's period would alias */
        if (mm >= 0 && (mm * 20 < bm * 19 || (mm <= bm && abs(s) < abs(best)))) { bm = mm; best = s; }
    }
    if (!best) return 0;
    /* the confidence (coarse pass: at full resolution dithered old pixels would
     * count as mismatch): how far the best shift beats standing still AND the
     * runner-up away from it. Text lines have a pitch, so two unrelated pages
     * still line up their blank gaps at some shift (a page turn scored 0.61
     * against standing still alone); a true scroll also matches the glyphs */
    long second = -1;
    for (int s = -s4max; s <= s4max; s++) {
        long mm = mms[s + s4max];
        if (mm < 0 || abs(s - best) <= 2) continue;
        if (second < 0 || mm < second) second = mm;
    }
    double a = 1.0 - (double)bm / (double)still, b = second > 0 ? 1.0 - (double)bm / (double)second : a;
    *score = a < b ? a : b;
    if (*score <= 0) { *score = 0; return 0; }
    /* the fine pass only places the shift: full resolution around 4 x the coarse one */
    int fb = 0;
    long fm = -1;
    for (int s = 4 * best - 4; s <= 4 * best + 4; s++) {
        if (!s || s < -smax || s > smax) continue;
        long mm = mismatch(oldr, newr, k, n, s);
        if (mm >= 0 && (fm < 0 || mm < fm)) { fm = mm; fb = s; }
    }
    if (fm < 0) { *score = 0; return 0; }
    return fb;
}
