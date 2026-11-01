/* epd-motion — is the screen moving, and how fast: ONE estimate for glassine-epdd-desktop's
 * paint policy (user, "a stone age finger touch screen check at the
 * front and complex statistics in the back" -> the motion detection one layer up).
 *
 * Until now a burst counted as a gesture only once the finger had moved
 * TOUCH_MOVE (0.02 of the axis, 37 px); before that it was a "press". The first
 * frames of every scroll were painted as presses, and a scroll without a finger
 * (a fling's tail after 500 ms, the wheel, keys) was seen only through "a GC16
 * was recent". Here two kinds of evidence meet:
 * the finger down/up, slot-0 positions, a second finger (pinch); a finger
 * that moved MO_SLOP is a gesture (swipes that REPLACE the page,
 * KOReader's page turns, show no shift)
 * the content per burst: did the changed area SHIFT against what the panel
 * shows (mo_find_shift)? A scroll's first frame shifts, whatever
 * the finger did; a tap's repaint (a button, a new page) does not
 * States: MO_STILL, MO_PRESS (finger down, nothing moving yet: a tap or the
 * moment before a scroll), MO_MOVING (latched: until neither evidence came for
 * MO_HOLD_MS). Speed: screen heights per second, from the content when it was
 * measured (what the panel has to show), else the finger, else 0 (unknown: the
 * policy's own fling decay applies).
 *
 * No clock, no I/O: the caller passes the time (ms). Host test:
 * test/epd-motion-test.c. */
#ifndef EPD_MOTION_H
#define EPD_MOTION_H
#include <stdint.h>

#define MO_SLOP 0.02            /* finger travel (screen heights) that makes a gesture (was TOUCH_MOVE) */
#define MO_HOLD_MS 500          /* moving lasts this long after the last evidence (was TOUCH_HOLD_MS) */
#define MO_FRESH_MS 60          /* a finger speed older than this is "resting" */
#define MO_SHIFT_SCORE 0.5      /* mo_find_shift score that counts as a shift */
#define MO_SHIFT_DT_MS 400      /* a content speed needs the previous burst at most this long ago */
#define MO_ALONE_MS 300         /* no finger: two shifted bursts this close start the motion */

enum { MO_STILL, MO_PRESS, MO_MOVING };

typedef struct {
    double screen_h;            /* content shift px per screen height (portrait: 1872) */
    /* the finger (slot 0) */
    int down, pinch;
    double x0, y0, x, y;        /* screen heights */
    double fv;                  /* finger speed, screen heights/s (smoothed) */
    int64_t ft, fmoved;         /* last report, last report that moved */
    int finger_gesture;         /* moved MO_SLOP since down (or pinched) */
    /* the content */
    int64_t burst_t;            /* the last burst (what the panel shows dates from it) */
    int64_t shift_t;            /* the last shifted burst */
    double cv;                  /* content speed, screen heights/s */
    int last_shift;             /* px, portrait y (+ = content moved down) */
    /* the result */
    int moving;
    int64_t evidence_t;         /* the last evidence of motion */
    int by_touch;               /* the motion involves the finger (down, or lifted < MO_HOLD_MS ago) */
    int64_t up_t;
    unsigned long shifts, bursts, early;   /* stats: shifted bursts, bursts, gestures found before MO_SLOP */
} motion;

void mo_init(motion *m, double screen_h);
/* the finger: positions in screen heights (x scaled to the same unit) */
void mo_down(motion *m, int64_t t, double x, double y);
void mo_move(motion *m, int64_t t, double x, double y);
void mo_pinch(motion *m, int64_t t);
void mo_up(motion *m, int64_t t);
/* a burst: measured = mo_find_shift ran, shift its result (px, portrait y),
 * score its confidence (0..1) */
void mo_burst(motion *m, int64_t t, int measured, int shift, double score);
/* MO_*; *v = speed (screen heights/s, 0 = unknown) */
int mo_state(motion *m, int64_t t, double *v);
/* whether a burst now is worth a shift measurement (finger down, or moving) */
int mo_wants_shift(const motion *m, int64_t t);

/* the shift between what the panel shows and the new frame, along k sampled
 * rows: oldr[i]/newr[i] point at n pixels each (one portrait column; index
 * j = portrait y reversed, as in glassine-epdd-desktop's frames). Finds s in [-smax, smax]
 * with new[j] ~ old[j + s] (coarse at 1/4, then refined); returns s (0 = none)
 * and *score = 1 - best/median mismatch (0 when nothing differs or no shift
 * beats standing still). Old pixels may be dithered (runs commit dithered):
 * the coarse pass averages 4. */
int mo_find_shift(const uint8_t *const *oldr, const uint8_t *const *newr, int k, int n, int smax, double *score);
/* dst[j] = the mean of 4 rows at r, r + stride, r + 2 stride, r + 3 stride: the
 * caller's columns for mo_find_shift. With the 1/4 averaging along the row the
 * coarse pass compares 4 x 4 blocks: a run's 8 x 8 ordered dither (the panel
 * buffer during a run) averages out (device: the first shifted frames
 * of a scroll scored 0.2-0.45 against single dithered columns). */
void mo_avg4(uint8_t *dst, const uint8_t *r, long stride, int n);

#endif
