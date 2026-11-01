/* epd-policy — the e-ink refresh policy of glassine-epdd-desktop as a pure state machine.
 *
 * No X, no /dev/ebc, no clock: glassine-epdd-desktop feeds events (a burst of changed
 * rectangles, pen ink, time passing, FIFO commands) with the current time and
 * executes the actions returned (grab X pixels into the panel buffer, send an
 * update, set the DU frame cut, draw the sleep banner). The same code runs on the
 * host in test/epd-policy-test.c (scenario tests + trace replay).
 *
 * States (policy.state):
 * IDLE nothing on the panel needs attention
 * CLEANUP DU'd areas / pen previews wait for their GC16 cleanup (after
 * IDLE_GC_MS without damage)
 * RUN motion run (scrolling, dragging, a "fast" app): bursts go dithered DU
 * with the frame cut; ends at the idle cleanup with ONE flash over what
 * it touched inside its window
 * PAUSED "pause": the panel belongs to someone else (glassine-greytest)
 * Window changes (pol_window: a window mapped, unmapped, moved or resized): the
 * area it covered or now covers is a candidate; pixels there that change in the
 * next bursts are "stale" — the panel moved them from unrelated old content,
 * with a partial GC16 or DU, which leaves the old content's ghost (a vanished
 * popup on the window below, the last workspace's windows under the new ones).
 * The idle cleanup gives stale areas one local flash, unless a flash already
 * covered them; bursts right after a window change never start a motion run
 * (the windows repainting one after another after a workspace switch are not
 * scrolling).
 * Popups (pol_popup: an override-redirect window mapped — menus, the panel
 * popups): never a flash of the whole panel (the quick settings card is
 * >= flash_min of the screen and blinked). Default "popup=du": a dithered DU
 * first (A2 on the engine: ~120 ms, a black/white card at once), the GC16
 * partial of the real greys right behind. "popup=fast": the GC16 partial alone
 * (one transition, but GC16 walks even black/white pixels through its
 * greys: popups faded in; du looked better). "popup=gc16": no popup rule.
 * ASLEEP "sleep" (glassine-powerd suspend): banner shown, no updates until "wake"
 * A pending full refresh ("full", start, gamma change, resume) is a request flag,
 * served at the next step in any awake state.
 *
 * The structure as a statechart:
 * Output (the states above, policy.state), Contact (policy.c: drag, touch,
 * wheel, pen, input; each a sub-region read through the predicates engaged,
 * holds_run, pen_quiet ...), Panel facts (the GC16 in flight, gc16_over;
 * whose the last GC16 was, send_gc16(who) -> the run tail). A burst is no state:
 * classify picks one row of the decision table and the row paints.
 *
 * Coordinates: portrait X screen pixels (W x H = 1404 x 1872), half-open boxes.
 */
#ifndef EPD_POLICY_H
#define EPD_POLICY_H
#include <stdint.h>

/* ---- geometry ------------------------------------------------------------ */
typedef struct { int x0, y0, x1, y1; } box;
#define MAXBOX 12
#define MERGE_PX 24             /* merge rectangles closer than this */
typedef struct { box b[MAXBOX]; int n; } boxlist;

int box_empty(box b);
box box_union(box a, box b);
box box_clip(box a, box c);
int box_inside(box a, box c);
long box_area(box b);
void list_add(boxlist *l, box b);       /* add, merging with everything near it */
void list_fold(boxlist *l, int max);    /* merge cheapest pairs until <= max */

/* ---- tuning -------------------------------------------------------------- */
/* damage -> read: wait this long for the rest of a repaint. 30 -> 10;
 * with the compositor apps present whole frames into their pixmaps, and a
 * half-drawn read heals (the rest of the drawing is new damage, re-read) */
#define COALESCE_MS 10
/* ... except in a motion run when the damage comes in whole frames (sway's
 * glassine-epd output: one composited frame per dispatch, nothing more to wait
 * for; policy.whole). Chromium scroll A/B: epdd's loop was ~55 ms
 * per frame in motion, 10 of them this wait */
#define IDLE_GC_MS 250          /* was 600: an A/B by feel */
#define PTR_REST_MS 0           /* the pointer: no cleanup before it rested this long (ptr_rest_ms=; 1000 made the
                                 * pointer's trails linger off) */
#define STALL_MS 1000           /* a run whose last frame still moved fast (>= STALL_V) holds this long
                                 * through a pause of the app's frames (stall_ms=; 0 = off;
                                 * a stall must not end the run with a GC16 and a new ramp) */
#define STALL_V 0.3             /* screens/s: "still fast" (a fling that ends decays below it first) */
#define WHEEL_HOLD_MS 1000      /* a wheel / touchpad scroll holds its run this long after a wheel event, as a
                                 * finger down does (wheel_hold_ms=; swipe gaps 0.3-1.2 s) */
#define GC16_BUSY_MS 500        /* a GC16 this recent, under a change => it starts a motion run */
#define RUN_TAIL_MS 2000        /* run_tail: ... except our own run-end GC16 (still frames) */
#define TAIL_FAST_N 3           /* run_tail: this many large frames in a row ... */
#define TAIL_FAST_GAP_MS 150    /* ... each this soon after the last = continuous content: the tail ends */
#define GHOST_MAX 30            /* DUs on a tile before it gets a clearing flash */
#define RUN_BURSTS 2            /* run bursts before the run ends in a flash */
#define DB_FINGER_V 0.05         /* screens/s: above it a finger is scrolling, not tapping (the debounce) */
#define DB_INK_MS 1000            /* the pen inked this recently: no debounce */
#define INK_SETTLE_MS 90          /* was COALESCE_MS * 3 when that was 30 */
#define POST_INK_MS 1000        /* app repaints this soon after pen ink are its re-render of the stroke */
#define INK_AA_PX 8             /* ink_redraw: a re-render may reach this far past the inked boxes (AA, width) */
/* ink_clean=damage (default since "stock Boox doesn't GC16
 * after every pen stroke"): a stroke's cleanup is a DU of the app's content (no
 * flash); its greys (anti-aliasing) wait as ink debt for ONE GC16 partial once
 * the pen has rested INK_DEBT_PAUSE_MS and the debt is worth it: the ghost
 * tiles (GTILE) the pen touched since then, INK_DEBT_AREA of the screen's (the
 * tiles, not the strokes' boxes: one diagonal stroke's box is 15 % of the
 * screen, its tiles ~2 %), or a tile of it DU'd INK_DEBT_GHOST times since its
 * last flash (strokes over strokes, erasing). Any other GC16 over it (a page
 * turn, a window change) clears it for free */
#define INK_DEBT_PAUSE_MS 3000
#define INK_DEBT_AREA 0.15      /* ~390 of the 2596 tiles: four or five full lines of handwriting */
#define INK_DEBT_GHOST 15       /* half GHOST_MAX: before the tile's own clearing flash */
#define CUT_SAFE_MS 400         /* no DU this recent before changing the frame cut */
#define BAR_H 72                /* sleep banner height (bottom of the screen): the fallback
                                 * when there is no sleep pill (pol_banner) */
#define GTILE 32                /* ghost accounting tile */
/* run_wait: a run ends after the app's own frame gap, not a fixed idle_ms
 * (scroll log: Firefox on a heavy page drew a frame every ~550 ms,
 * so every frame was a run of its own and ended in a 78 % GC16 400 ms later,
 * 16 times in 8 s; its kinetic tail, frames up to 1050 ms apart, ended the run
 * and flashed the whole screen). The gaps between damage events of the last
 * RW_SPAN_MS count (RW_MIN_GAP_MS..run_wait_max); within RW_INPUT_MS of the
 * last input a run waits run_wait x the largest of them, at most run_wait_max,
 * never less than idle_ms, and never ends with a finger down */
#define RW_N 6
#define RW_SPAN_MS 3000
#define RW_MIN_GAP_MS 20
#define RW_INPUT_MS 5000
/* busy content: an app that repaints a sizeable area at a steady rate on its
 * own (a video, an animation, a slideshow, a build log) is motion like a
 * finger: its run waits for its own frame gap (run_wait), not idle_ms
 * (video went choppy ~5 s after the last touch and a
 * finger on the screen kept it smooth -- RW_INPUT_MS after the input the run
 * ended at every frame gap > idle_ms, GC16 cleanup, the stagger off, a new
 * run). Busy = the last BUSY_N bursts with changes within BUSY_SPAN_MS (>= 3
 * a second) AND their changes of the last BUSY_SPAN_MS cover >= busy_min of
 * the screen (the exact union: a clock, a cursor, a progress bar never do).
 * Per app: eink.conf busy=on (every repaint is motion) / busy=off (never) */
#define BUSY_N 6
/* quiet pacing (quiet_ms) is lifted while damage comes at least QUIET_DENSE_N
 * times within QUIET_DENSE_MS and the last burst changed >= busy_min of the
 * screen: paced to one read per quiet_ms, busy content never showed the
 * BUSY_N bursts busy_seen needs, so a video started 5 s after the last touch
 * stayed at one GC16 per 2 s until a finger came (FreeTube) */
#define QUIET_DENSE_N 3
#define QUIET_DENSE_MS 1000
#define BUSY_SPAN_MS 2000
enum { BUSY_AUTO = 0, BUSY_ON = 1, BUSY_OFF = 2 };
#define RESTACK_MS 1000         /* a window change this recent: bursts are repaints, not a run */
/* the cleanup despite ongoing damage (user, "a single horizontally
 * scrolling line can prevent the repaint for the entire screen no matter how
 * damaged it is" -- a ticker, a video timer, GNOME Software's carousel held back
 * every cleanup while A2 piled DC into the rest, max 553). The changes are
 * collected in time buckets (MV_N x MV_BUCKET_MS, up to MV_K boxes each); when
 * the area the changes of the quiet wait cover (the exact union, not a bounding
 * box) is small against what waits for the cleanup (the union of the dirty
 * boxes, the run's area and the stale ones: >= CLEAN_DEBT_RATIO x), the rest
 * has been quiet long enough: the cleanup runs now ("clean_motion=F": the
 * ratio, 0 = off). No size cap and no burst count: a fast ticker counts by the
 * area it covers, not by how often it paints */
#define MV_N 16
#define MV_BUCKET_MS 100
#define MV_K 4
#define CLEAN_DEBT_RATIO 4.0
#define CAND_MS 2000            /* window-change candidates expire unless repainted by then */
#define WIN_MIN 16              /* smaller windows (1-pixel helpers) don't count */
#define RUN_FLOOR_MS 24         /* pace=engine: bursts at least this far apart (two drive frames) */
#define ENG_HOLD_MAX_MS 1500    /* pace=engine: a run's quiet starts when the engine is free, at most this long after the last damage */
#define POPUP_MS 1000           /* a popup mapped this recently paints fast (pol_popup) */
#define POPUP_BESIDE 16384      /* px: changes outside the popup that still ride its paint */
#define GTW ((1404 + GTILE - 1) / GTILE)
#define GTH ((1872 + GTILE - 1) / GTILE)

/* ---- actions ------------------------------------------------------------- */
enum { WF_DU = 1, WF_GC16 = 2, WF_DU4 = 3 };   /* WF_DU4: engine only (motion=adaptive) */
/* partial / full-screen flash / flash of just the area (the driver widens every
 * flashing update to the whole panel: glassine-epdd-desktop does M_AREA as two partials) */
enum { M_PART = 0, M_SCREEN = 1, M_AREA = 2 };
enum { A_SEND, A_GRAB, A_GRAB_ALL, A_CUT, A_BANNER };
/* A_GRAB: copy X's pixels of b into the panel buffer (changed ones);
 * A_GRAB_ALL: every pixel of b; A_SEND: SEND_UPDATE b with wf/mode;
 * A_CUT: set cut_frame_num = n; A_BANNER: draw the "SLEEPING" bar into the buffer */
/* n: A_CUT the cut; A_SEND with WF_DU: an A2 cut to n frames (0 = none; engine only) */
typedef struct { int kind; box b; int wf, mode, n; } action;
#define MAXACT 48
typedef struct { action a[MAXACT]; int n; } actions;

/* how a burst's pixels enter the panel buffer (see glassine-epdd-desktop.c commit) */
enum { C_ALL = 0, C_BW = 1, C_DITHER = 2, C_THRESH = 3,   /* C_THRESH: greys to black/white at mid grey */
       C_Q4 = 4 };                                          /* C_Q4: greys dithered to DU4's 4 levels */

/* ---- motion=adaptive: the paint of a motion run follows how fast it moves ----
 * A speed estimate (screen heights per second, in log space) fused from the
 * evidence at every run burst, Bayesian style: each source is a Gaussian
 * estimate with a weight, the posterior is the weighted mean, and the last
 * posterior is the prior with its weight decaying over M_TAU_MS (the damping):
 * finger moving (glassine-epdd-desktop's touch read, pol_finger) weight 4, its speed
 * finger lifted (a fling: the content goes on) weight 2.5 -> 0.5 over
 * M_FLING_MS, the lift
 * speed decaying (to "still")
 * finger resting weight 4, "still"
 * no finger (wheel, keys, animation) weight 1, the A2 step
 * pen ink in the last second weight 3, "still"
 * The step: up at once past a threshold + M_HYST/2 (log units), down only
 * after M_DOWN_MS below its threshold - M_HYST/2 (Schmitt trigger + hold): a
 * noisy speed near a threshold does not flip the waveform. Steps (24 C):
 * LV_GC16 GC16 partial, 16 greys 38 frames ~2 steps/s (reading drag)
 * LV_DU4 DU4, greys dithered to 4 24 frames ~3.5
 * LV_A2 dithered A2/DU (= binary) 10 ~8.5
 * LV_A2_8 / LV_A2_6 / LV_A2_4 A2 cut to 8 / 6 / 4 frames ~10 / 14 / 21
 * LV_A2_2 A2 cut to 2 frames (1 drive frame: faint) ~42 by the panel; our
 * ~42 ms of CPU per update caps it near 24
 * (the lengths: steps=a,b,c[,d], 2..10 frames, falling;
 * a frame is 11.8 ms, the last one idle)
 * A run that used a cut A2 ends in a flash of what it touched (the cut leaves
 * pixels short of the level the engine records; a partial GC16 would skip them).
 * Thresholds between the steps: ladder=a,b,c,d,e[,f] (screens/s); f (into
 * LV_A2_2) is off (never reached) unless given.
 * attack=jump: rising evidence replaces the estimate at once (the prior only
 * damps the way down); attack=fused (default) fuses both ways.
 * mmin=gc16|du4|a2|a2_8|a2_6|a2_4|a2_2: the lowest step a run uses (mmin=du4 skips
 * GC16, mmin=a2 skips GC16 and DU4); mmax= (same names) the highest, the
 * fastest paint a run may reach (mmax=a2_4: never A2/2; mmin=mmax: one fixed
 * paint); the estimate itself is not clamped.
 * curve=off: no ladder — any motion (a run, a press) paints at mmax, a still
 * at mmin (with still=range); the estimate is still kept for the log. */
enum { LV_GC16, LV_DU4, LV_A2, LV_A2_8, LV_A2_6, LV_A2_4, LV_A2_2, LV_N };
#define LV_OFF 1e9              /* a ladder threshold never reached (screens/s) */
#define M_TAU_MS 250.0
#define M_WMAX 6.0
#define M_HYST 0.35
#define M_DOWN_MS 300
#define M_FLING_MS 600.0
#define M_STILL 0.03            /* screens/s: "not moving" */
#define M_NOFINGER 1.2          /* screens/s: no finger evidence -> the LV_A2 step */
#define M_LIFT_MS 10000         /* a lift this recent is the evidence (the fling's decay), not "no finger" */

/* per-app refresh mode (E-ink Center). PM_PRESS: a finger is down but not (yet)
 * a gesture — the start of a scroll, or a button press: dithered DU at once,
 * WITHOUT starting a motion run (a tap's result must not become a DU) */
enum { PM_AUTO = 0, PM_QUALITY = 1, PM_FAST = 2, PM_PRESS = 3 };

enum { RUN_END_FLASH = 0, RUN_END_PARTIAL = 1 };
enum { S_IDLE, S_CLEANUP, S_RUN, S_PAUSED, S_ASLEEP };
const char *state_name(int s);

typedef struct {
    /* config (FIFO / eink.conf "@" commands) */
    int W, H;
    int burst_auto, clean_full, cut_auto, cut_fixed, dither_runs;
    int dith;                   /* this burst's: dither_runs or the app's own (pol_burst) */
    box gc16_box;               /* the GC16s sent within GC16_BUSY_MS of gc16_box_t (union) */
    int64_t gc16_box_t;
    int idle_ms;                /* no damage this long: the cleanup (IDLE_GC_MS; "idle_ms=") */
    int run_ms;                 /* runs, pace=fixed (and /dev/ebc): at most one burst per run_ms */
    int pace;                   /* runs: 1 = pace=engine (default): the next burst when the engine
                                 * can start it (pol_engine), at least RUN_FLOOR_MS apart; 0 = fixed */
    int eng_wait;               /* pol_engine: ms until a burst read now could start; -1 unknown */
    int64_t eng_held;           /* pace=engine run: the last step the engine could not take a burst (epdd held sway's frame) */
    int run_fold;               /* runs: DU updates per burst (each SEND_UPDATE = ~47 ms kernel CPU) */
    int run_tail;               /* ms after a run's end GC16: a frame without motion (no finger,
                                   nothing measured moving, not busy) restarts no run; 0 = off */
    int64_t last_run_end;       /* when cleanup last ended a run */
    unsigned pages_n;           /* pages=: page changes since the last flash of one */
    int tf_n; int64_t tf_t;     /* run_tail: large frames in a row TAIL_FAST_GAP_MS apart, the last one */
    int64_t last_own_gc16;      /* the last GC16 the cleanup or a tail frame sent */
    int run_end;                /* RUN_END_FLASH: local flash; RUN_END_PARTIAL: partial GC16 unless ghosted */
    int quiet_ms;               /* no input for quiet_after ms: at most one burst per quiet_ms
                                 * (clocks, htop, progress bars; 0 = off). Each update costs
                                 * ~47 ms of kernel CPU in the driver, whatever its size */
    int quiet_after;
    double run_wait;            /* "run_wait=F" (1.5; 0 = a fixed idle_ms) */
    int run_wait_max;           /* "run_wait_max=MS" (1200) */
    int popup_fast;             /* pol_popup areas: 2 DU + GC16 (popup=du, default), 1 one GC16 partial, 0 off */
    int ink_redraw;             /* ink_redraw=du: an app's re-render inside what the pen just inked is a DU,
                                 * one GC16 partial after POST_INK_MS (no gc16_busy run); default on
                                 * off: the old sized choice */
    int ink_held;               /* the dirty list holds such a DU (its cleanup waits for POST_INK_MS) */
    int ink_clean;              /* ink_clean=damage (1, default): see INK_DEBT_PAUSE_MS; =stroke (0): a
                                 * GC16 partial after every stroke (the old behaviour) */
    int popup_dither;           /* popup=du's DU: 1 dithered greys (popupdu=dither, default), 0 thresholded (popupdu=bw) */
    double du_max, flash_min;
    int adaptive;               /* motion=adaptive (engine only: has_engine) */
    int has_engine;             /* the caller has the mainline engine (WF_DU4, A2 cuts) */
    int whole;                  /* damage arrives as whole frames (sway): no coalesce wait in a run */
    double lad[LV_N - 1];       /* step thresholds, screens/s */
    int a2len[4];               /* frames of the four cut steps (steps=a,b,c[,d]; 2..10, falling) */
    int m_attack;               /* attack=jump: rising evidence taken at once */
    int m_curve;                /* curve=on (default): a run's step follows the speed (the ladder);
                                 * curve=off: every run and press paints m_max, whatever the speed
                                 * (stills keep m_min with still=range) */
    int m_min;                  /* mmin=: lowest step of a run (LV_*) */
    int m_max;                  /* mmax=: highest (fastest) step of a run (LV_*; default LV_A2_2) */
    int still_range;            /* still=range: repaints outside runs paint at m_min too (still=auto:
                                 * sized as before); m_min == m_max then = ONE mode for everything,
                                 * no optical cleanups (single: runs, presses, stills; popups stay) */
    /* state */
    struct {                    /* Contact (statechart doc): what is doing, one sub-region each */
        int drag;               /* pol_motion: a drag glassine-epdd-desktop paints itself is on */
        struct { int down; double v, lift_v; int64_t lift_t; } touch;  /* pol_finger (v, lift_v: screens/s) */
        int64_t wheel_t;        /* the last wheel event (pol_wheel: a touchpad's two-finger scroll, a mouse wheel) */
        int64_t ink_t;          /* the last pen ink (pol_ink) */
        int64_t input_t;        /* the last input event (pol_input) */
    } c;
    double m_mu, m_w;           /* speed estimate: log screens/s, its weight */
    int64_t m_t, m_down_since;
    int m_level;
    int run_cut;                /* this run used a cut A2 */
    int run_always;             /* this run holds a motion=always app's bursts (no small_motion cleanup) */
    unsigned long lvl_n[LV_N];  /* run updates per step (stats) */
    int state;
    int want_full;
    box banner;                 /* what "sleep" draws over the page (pol_banner; default the bar) */
    boxlist dirty;              /* DU areas awaiting their GC16 cleanup */
    boxlist inked;              /* pen previews to replace by X content */
    boxlist ink_rec;            /* what the pen inked since a POST_INK_MS gap (ink_redraw) */
    boxlist inkd;               /* ink_clean=damage: the pen's previews, for the stroke's DU cleanup */
    boxlist inkr;               /* ... the app's re-renders of them (DU'd already: content) */
    boxlist ink_debt;           /* ... then their greys, for the damage-gated GC16 partial */
    uint8_t ink_tile[GTW * GTH];    /* tiles the pen touched since the ink debt was last cleaned */
    int ink_tiles;              /* ... how many */
    unsigned long ink_debt_cleans;  /* stats */
    int run_bursts;             /* run bursts since the last cleanup */
    box run_box, run_win;       /* what the run touched / the window(s) it ran in */
    int64_t last_dmg, last_gc16, last_du, last_burst;
    int cut_cur;
    uint8_t ghost[GTW * GTH];   /* DUs per tile since its last flash */
    boxlist cand;               /* areas of recent window changes (pol_window) */
    boxlist stale;              /* changed inside cand since: a flash at the cleanup */
    int64_t last_restack;
    int64_t rw_t[RW_N];         /* recent damage gaps: when, how long */
    int rw_gap[RW_N], rw_i;
    double clean_motion;        /* CLEAN_DEBT_RATIO; "clean_motion=F", 0 = off */
    double busy_min;            /* "busy=F": screen share of busy content (0.03; 0 = off) */
    int64_t bz_t[BUSY_N];       /* the last bursts with changes (busy_seen) */
    int bz_i;
    int64_t last_busy;          /* the last busy burst: counts as input for run_wait */
    int64_t last_big;           /* the last burst whose changes cover >= busy_min of the screen (quiet_lifted) */
    unsigned long busy_bursts;  /* stats */
    int64_t mv_t[MV_N];         /* the changes per MV_BUCKET_MS bucket: which bucket (its start) */
    box mv_b[MV_N][MV_K];       /* where (merged down to MV_K boxes) */
    int mv_n[MV_N];
    unsigned long early_cleans; /* stats: cleanups the small ongoing motion did not hold back */
    boxlist ptr_dirty;          /* pol_pointer's paints awaiting their GC16 (cleanup) */
    int64_t last_ptr;           /* the last pol_pointer paint: the cleanup's quiet counts from it too */
    int ptr_rest;               /* no cleanup until the pointer rested this long ("ptr_rest_ms=";
                                 * a touchpad pauses often for ~250 ms (idle_ms), the cleanup's GC16 partial
                                 * then held the next pointer move 450-800 ms) */
    int wheel_hold;             /* "wheel_hold_ms=": the run holds this long after one, as under a finger */
    int stall_ms;               /* "stall_ms=": see STALL_MS */
    int64_t stall_t;            /* Content: the run's last frame moved >= STALL_V (when; 0 = it slowed or stood) */
    boxlist popup;              /* popups mapped within POPUP_MS */
    /* the first still frame's debounce (pol_debounce_left; sway frames only) */
    int debounce_ms, debounce_gap, debounce_cap, debounce_tap_ms, debounce_tap_px, debounce_osk, osk_up;
    int64_t db_t0;              /* the hold's first damage; 0 = none */
    int db_tap;                 /* this step reads the damage near the tap: the hold stays */
    int tap_x, tap_y, tap_on;   /* the last contact (pol_tap, portrait px); on: tap_up not reached */
    int64_t tap_up;             /* when it lifted */
    int64_t last_popup;
    int64_t last_gone;          /* a popup unmapped (pol_popup_gone) */
    void (*log)(const char *fmt, ...);   /* optional verbose log */
} policy;

/* a burst: the rectangles whose visible pixels changed, the window under the
 * largest one (whole screen if none) and the refresh mode that applies;
 * moving/speed: the motion estimate (epd-motion: the finger AND whether the
 * content shifted) -- moving starts/continues a run like a "fast" profile, a
 * speed > 0 (screen heights/s) is the adaptive step's evidence before the
 * finger's (0 = unknown: pol_finger's evidence as before) */
/* busy: the app's eink.conf busy= (BUSY_AUTO: detected, see BUSY_N);
 * dither: the app's own dither= (0: dither=on|off; 1 dithered, -1 not);
 * always: the app's motion=always (see ALWAYS below);
 * calm: the shell's own surfaces (bar, popups): never motion — no press, no run,
 * no still floor; small changes a thresholded DU (cleaned later), large a GC16 partial;
 * pages: the app's pages= (see PAGES below; 0 = off) */
typedef struct { int64_t now; boxlist ch; box pwin; int pm; int moving; double speed; int busy; int dither; int always; int pages; int calm; } burst_in;
/* pages=on|N (an app's eink.conf line, default off; KOReader, from a host study
 * of its frames): an app that REPLACES its page rather than moving it. A swipe
 * page turn kept epd-motion "moving" for MO_HOLD_MS on the finger alone, and a
 * menu tap's second frame came while the highlight's GC16 was in flight: both
 * became a faint A2 run plus its end flash (2-3 visible refreshes). For such an
 * app (glassine-epdd-desktop passes moving only on a measured content shift):
 * - a GC16 in flight or busy content (busy=auto) starts no run without a shift
 * - a large change (>= flash_min) is a GC16 partial, no flash; every Nth one
 * (pages=N, on = PAGES_FLASH) flashes, to clear the partials' ghosts
 * A pan or scroll (a measured shift) runs as before. */
#define PAGES_FLASH 6
/* motion=always (an app's eink.conf line; glassine-settings > Apps > "Always in
 * motion"; "a slow scene in Morrowind collapses into
 * GC16"): every repaint of the app is a run burst at its fluidity (the range's
 * fast end, no speed ladder), with or without input, after a window change or
 * a run's tail; the run is busy content (it waits for the app's own frame gap)
 * and never takes the cleanup despite motion (small_motion: a slow scene's
 * small changes against the run's area). Its cleanup comes once the app is
 * really quiet. Without it a slow scene was still content: the speed estimate
 * fell, small_motion cleaned up the run, and the next changes went the sized
 * way (a GC16 partial past du_max) */

void pol_init(policy *p, int W, int H);
/* settings only (GLASSINE_EPDD env, "@" lines of eink.conf):
 * cut=auto|N dither=on|off burst=auto|du clean=partial|full du_max=F flash_min=F
 * run_ms=N run_fold=N run_end=flash|partial quiet_ms=N quiet_after=N popup=fast|du|gc16
 * pace=engine|fixed motion=binary|adaptive curve=on|off ladder=a,b,c,d,e[,f] steps=a,b,c[,d]
 * run_wait=F run_wait_max=MS (a run ends after F x the app's frame gap; 0 = idle_ms)
 * ptr_rest_ms=MS (no cleanup until the pointer rested this long; PTR_REST_MS)
 * wheel_hold_ms=MS (a run holds this long after a wheel event; WHEEL_HOLD_MS, 0 = off)
 * stall_ms=MS (a run whose last frame moved fast holds this long through a frame pause; STALL_MS, 0 = off)
 * debounce_ms=MS (0 = off, see pol_debounce_left) debounce_gap=MS debounce_cap=MS debounce_tap_ms=MS
 * debounce_tap_px=PX debounce_osk=0|1 (1: also with the on-screen keyboard up) osk_up=0|1 (glassine-osk)
 * busy=F (busy content's screen share, see BUSY_N; 0 = off)
 * ink_redraw=du|off (default du) ink_clean=damage|stroke (default damage: INK_DEBT_PAUSE_MS) */
void pol_config(policy *p, const char *cmd);
/* FIFO command: full pause resume sleep wake gamma= + the settings above.
 * "sleep": the page is grabbed whole, the banner drawn over it, then one full
 * GC16 (design round 8: a clean page to sleep on).
 * Returns 1 on "wake" from sleep: the caller grabs the banner area (p->banner, all pixels)
 * and the changed rest of the screen, then calls pol_wake */
int pol_cmd(policy *p, const char *cmd, int64_t now, actions *out);
/* the area "sleep" draws over the page (glassine-epdd-desktop: the sleep pill's box, from
 * glassine-sleep-pill); an empty box = the plain bar at the bottom */
void pol_banner(policy *p, box b);
/* after "wake": the banner area plus whatever changed meanwhile */
void pol_wake(policy *p, box changed, int64_t now, actions *out);
/* the engine's view before pol_step / pol_timeout: ms until a burst read now
 * could start on the panel (the caller has taken off what the read and hand-off
 * take); -1 = no engine (stock /dev/ebc: runs keep run_ms). with a
 * fixed run_ms, 40 % of a scroll's updates were read, converted and committed,
 * then replaced in the engine queue before they could start */
void pol_engine(policy *p, int wait_ms);
/* the first finger: down, and its speed in screen heights/s (0 = not moving) */
void pol_finger(policy *p, int64_t now, int down, double v);
void pol_damage(policy *p, int64_t now);                        /* X reported damage */
void pol_input(policy *p, int64_t now);                         /* touch / pen / key: is active */
/* the debounce: a still screen's first damage (nothing painted or damaged for
 * debounce_gap) waits until no damage came for debounce_ms, at most debounce_cap
 * from its start, so a short transition paints once. Off with debounce_ms=0.
 * Not while a finger drags, a wheel scrolls, the pen just inked, a popup paints,
 * or (debounce_osk=0) the on-screen keyboard is up. Damage near a live tap
 * (pol_debounce_clip) and on the shell's surfaces (glassine-epdd-desktop) goes on at once: set db_tap for that step. */
int pol_debounce_left(const policy *p, int64_t now);            /* ms still held; 0 = none */
void pol_tap(policy *p, int64_t now, int down, int x, int y);   /* a contact at portrait px x,y; its lift */
int pol_debounce_clip(const policy *p, box dmg, int64_t now, box *out);    /* the part of dmg near a live tap, during a hold */
void pol_wheel(policy *p, int64_t now);                         /* a wheel event (mouse, touchpad): a scroll's finger */
void pol_window(policy *p, box b, int64_t now);                 /* a window (un)mapped / moved: its area */
void pol_settled(policy *p, int64_t now);
void pol_motion(policy *p, int on);                             /* a handle drag: everything is motion */                       /* a snapshot was just painted: the next read corrects it */
void pol_popup(policy *p, box b, int64_t now);
void pol_popup_gone(policy *p, box b, int64_t now);             /* a popup / menu unmapped: its area */                  /* a popup / menu mapped: its area */
void pol_ink(policy *p, box b, int64_t now, actions *out);     /* pen preview drawn at b */
/* the same, for a preview already sent by the caller (glassine-epdd-desktop's pen thread) */
void pol_inked(policy *p, box b, int64_t now);
/* frame cut back to its resting value (after config changes), unless in a run */
void pol_recut(policy *p, int64_t now, actions *out);
/* once per loop iteration. Returns STEP_BURST when the caller should build the
 * pending burst now (pend_n rectangles, oldest damage at first_dmg) and call
 * pol_burst; STEP_DROP when the pending damage is void (paused / full refresh). */
enum { STEP_NONE, STEP_BURST, STEP_DROP };
int pol_step(policy *p, int64_t now, int inking, int pend_n, int64_t first_dmg, actions *out);
/* returns how to commit the burst's pixels (C_*); the caller commits, then runs out */
int pol_burst(policy *p, const burst_in *in, actions *out);
/* the mouse pointer (sway: glassine-epdd-desktop stamps the sprite into its
 * reads and paints where it moved itself): ch = what changed under the
 * pointer's old and new place. Painted at the fast end of the motion range
 * ("faint + instant": top_step with motion=adaptive on the engine,
 * else a dithered DU; never GC16), its GC16 partial at the idle cleanup once
 * the pointer rests (one, no flash: crisp at rest). Separate from pol_burst
 * on purpose: no motion buckets, frame gaps, busy or tail bookkeeping, never
 * a run, no run_cut. Decisions:
 * - the pointer's cleanup GC16 is not armed as gc16_box (nor last_gc16): a
 * hover repaint (tooltip, highlight) right after it is a sized repaint,
 * not "content faster than GC16" = a run
 * - its DUs count once per tile for ghosting (pol_burst counts cut A2s
 * twice): a pointer's 1-2 drive frames per crossing ghost less than a
 * scroll's cut steps, and its GC16 partial at rest takes the rest; trails
 * clear by the normal GHOST_MAX tile flash (~30 crossings, not ~15)
 * - the quiet before the cleanup counts from the last pointer paint too
 * (last_ptr; not last_dmg: the app's frame gaps stay the app's)
 * Returns how to commit (C_*), as pol_burst */
int pol_pointer(policy *p, boxlist *ch, int64_t now, actions *out);
/* ms until pol_step has something to do (-1: nothing scheduled) */
int pol_timeout(const policy *p, int64_t now, int inking, int pend_n);
/* the next read waits for quiet pacing (quiet_ms, no input for quiet_after):
 * glassine-epdd-desktop answers sway's frames meanwhile, so an app keeps its own frame rate
 * (else it drew at the read rate, one frame per quiet_ms, and its damage could
 * never look dense: QUIET_DENSE_N) */
int pol_quiet_held(const policy *p, int64_t now);

#endif
