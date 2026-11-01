/* epd-stagger — staggered cut steps (the idea:
 *.
 *
 * A cut A2 step of n drive frames, but every pixel starts its steps on its own
 * frames: pixel p belongs to phase group g(p) of G (a 64 x 64 blue-noise rank
 * map, epd-bluenoise.h: group g = ranks [g * 4096 / G, (g + 1) * 4096 / G)) and
 * may start a step on the frames f with f mod G == g. So 1/G of the panel
 * takes the newest content on every frame: the panel samples motion at the
 * frame rate whatever the cut, and the work per frame is even.
 *
 * An A2 black/white transition is one phase for all its drive frames (2 to
 * white, 1 to black; the WBF, , every temperature) and A2 drives no
 * grey. So a step is (phase, n) from its start frame: no table. Per pixel:
 * hot (16 bit, read every frame): phase 2 | n 4 << 2 | start frame mod 64 << 6
 * | what the cold state implies: level class 2 << 12 (0 black, 1 white,
 * 2 a grey) | model at that rail 1 << 14 | DC payback due 1 << 15 (with
 * the pay of the step that set it)
 * cold (32 bit, only for a pixel that has something to do on its group's
 * frame): level 5 | model 6 << 5 | stale 1 << 11 | (ctl 3) pulses since
 * the last idle frame 4 << 12 | DC record (int16) << 16
 * and the target level (8 bit, the newest content). One frame, per pixel:
 * age = (f - start) mod 64; it drives `phase` while age < n, then idles.
 * On its group's frame, a pixel whose step is done (age > n: its idle frame
 * is behind it) takes the target: black/white (greys: blue-noise dithered,
 * epd_stag_dq, or thresholded at 16), and starts a step toward it:
 * ctl 0 (levels 1 + 2): n frames if the target is not its level (a cut
 * step as today, only staggered; the dial = n and G set per frame);
 * ctl 1 (level 3, a controller): toward the target while the panel model
 * is short of that rail (at most n frames a step: it keeps driving until
 * it gets there, with any cut), and with pay > 0, a pixel at its rail
 * whose DC record opposes that rail's drive and is >= pay frames gets
 * up to n frames toward its own rail (it pays back without a change).
 * ctl 2 : ctl 1, but a payback step may take up to G frames
 * (its idle slot too: at G = n + 1 twice the payback of A2/2), and a
 * step may start on the group's frame right after its predecessor's
 * last drive frame (no idle frame between) if it drives the same way;
 * a reversal there waits for the next group frame (the idle first).
 * Content steps stay n frames + idle, so the cut's look is unchanged.
 * ctl 3 ("pulse instead of idle"): ctl 1, but a step
 * (toward the target or payback) may drive up to G frames, its whole
 * cycle to the group's next frame: at A2/2 a pulse every frame. A
 * reversal needs no idle frame first (the WBF's GC16 reverses frame to
 * frame 488 times; every waveform ends on an idle frame). k (cold bits
 * 12..15) counts the pulses of its whole-cycle steps since its last
 * idle frame; a cycle that would take it past K keeps an idle frame at
 * its end (G - 1 pulses), so a run of pulses is K + G - 1 at most. The
 * drain (ctl 2, 3) lasts G frames: every pixel ends on an idle.
 * drain: no new steps (running ones end): the hand-back to the slot engine.
 * A pixel whose level class is the target's and (ctl 1) whose model is at
 * that rail with no payback due has nothing to do: its hot bits say so, its
 * cold state is not read (in a scroll most pixels on a frame: the GPU's
 * traffic).
 * The model and the DC record move by the drive (+ to white), as the slot
 * engine's (epd_set_model: 0 black .. R white; the record clamped +-30000).
 *
 * payrule 1 ("one payback rule"): the slot engine's panel mode
 * (matched cuts, model 2) pays DC back only with an arrival: a black/white
 * pixel stepping to the other rail drives the longer of what it needs and
 * the record opposing that drive, at most cap frames past what it needs, and
 * nothing at an idle rail. ctl >= 1 with payrule takes the same: when a
 * pixel at a rail starts a step toward the other one, its payback budget is
 * b = min(max(owe - need, 0), cap) (owe = the record against the new rail's
 * drive, need = the model's distance to it, both at that step); its steps
 * then drive need + b in all (each at most what its ctl allows a step) and
 * the frames past need count off b. The record only drops by those frames,
 * so owe - need stays b's source: an arrival drives min(max(need, owe),
 * need + cap). The engine's matched cut also has a floor (the step's own
 * length) and a ceiling (the uncut A2); the stagger's controller has neither
 * (it drives exactly what the model needs, the cut sets only the steps'
 * size). b lives in the cold level while it is > 0: an odd level (never a
 * real one: those are even) = a rail with b frames still to pay, black
 * 2b - 1, white 31 - 2b (b <= 7: cap <= 7); its hot class is "grey", so the
 * pixel takes the cold path on its next group frame. The hand-back gives the
 * rail (an unpaid rest is dropped, as at a drain). pay and the due bit are
 * not used; payback frames are stale like the arrival's (the engine marks a
 * matched pixel by its step).
 *
 * This file: the CPU reference (the definition; tests compare the GPU's
 * frames and state to it bit for bit) and the hand-over to and from the slot
 * engine. The layout is the GPU's: rows of EPD_W pixels, so a wire word's 8
 * pixels are one 16-byte hot texel, two 16-byte cold texels, one 8-byte target
 * texel. */
#ifndef EPD_STAGGER_H
#define EPD_STAGGER_H

#include <stdint.h>
#include "epd-engine.h"

#define EPD_STAG_GMAX 16            /* phase groups */
#define EPD_STAG_NMAX 15            /* drive frames of a step */
#define EPD_STAG_DCMAX 30000
#define EPD_STAG_PCAP 7             /* payrule's cap: the odd levels hold b <= 7 */

typedef struct {
    int G;                          /* phase groups, 1..EPD_STAG_GMAX */
    int n;                          /* drive frames of a step, 1..EPD_STAG_NMAX */
    int ctl;                        /* 0 fixed steps, 1 the controller, 2 + payback through the idle slots, 3 pulses instead of idles,
                                     * 4 ctl 3 on bit planes (epd-bp4.h: lanes, G 2, the GPU; not epd_stag_rows) */
    int K;                          /* ctl 3: the most pulses without an idle frame (G..15; below G: every cycle keeps one) */
    int pay;                        /* ctl 1: DC payback from this many frames (0 off; payrule 0) */
    int payrule;                    /* 1: the engine's pay-on-arrival instead (above; ctl >= 1) */
    int pcap;                       /* payrule 1: the most payback frames past the need (0..EPD_STAG_PCAP) */
    int dither;                     /* greys: 1 blue-noise dither, 0 threshold at 16 */
    int drain;                      /* no new steps */
    int R;                          /* the model's range (the uncut A2's net drive, op_r) */
    /* the rail hammer = the arrival top-up (payrule; an optical reset, not a
     * DC payback: the DC record never decides it): a pixel
     * arriving at a rail with a partial history -- from a grey, or from a rail
     * it had not wholly left (its need < R: a reversal) -- gets htop frames
     * past the need into it (the budget b, max with the DC's; <= pcap). Frames
     * into a rail leave it at the rail (blind epdlab) */
    int htop;
    /* phase groups as column stripes : 0 the blue-noise ranks
     * (epd_bn_rank, fine-grained); W > 0: stripes W pixels wide, stripe s in
     * group (epd_stag_srank(s) * G) >> 12 -- the bit-reversed stripe index, so
     * every G interleaves (G 2: alternate stripes). The dither thresholds stay
     * the blue noise's. The GPU (px) draws only the frame's group stripes */
    int stripe;
    /* stripe > 0: scattered blocks instead of column stripes (2;
     * default off): the stripes' cells cut into W x W blocks (the word
     * engine's workgroup tile at SWY 64), block (bx, by) in group
     * (epd_stag_brank(bx, by) * G) >> 12: a 16 x 16 Bayer pattern over the
     * block grid (G 2: a checkerboard). No long seams; a wave still one
     * group. The px engine takes them as column stripes (not supported) */
    int blocks;
    /* stripe > 0, cellw > 0: cells instead (default off; before
     * blocks): cellw x cellh pixel cells (cellw 8, 16, 32, 64: whole words;
     * cellh 1..64; powers of two; at least 2 cells a 64 x 64 tile), cell
     * (cx, cy) in group (epd_stag_crank * G) >> 12: its place among the cells
     * of its 64 x 64 tile ordered by their blue-noise ranks
     * epd_bn_rank[cy & 63][cx & 63] (epd_stag_cells). A word one group, and a
     * tile N / G cells of each group for G dividing N, so the GPU's word pass
     * can run a tile's cells in that order (CELLR): every wave one group, as
     * with stripes. Other sizes: no cells (stripes or blocks). px: stripes */
    int cellw, cellh;
    /* the cells' order in a tile (epd_stag_cells; 0: by blue-noise rank;
     * EPD_STAG_CORD_WBAYER, stagger_stripe=wbayer with 8 x 1 cells,
     * by the 4 x 4 Bayer class of (word, row), epd_stag_brank's top 4
     * bits, then raster order, so G 2: a checkerboard of 8 x 1 words, G 4, 8,
     * 16 the finer classes; per-pixel fine in look, a word one group) */
    int cellord;
    /* stagger_state=lanes (0: runs, today's): the targets'
     * bytes carry the requests (EPD_PK_ARM / EVICT / TOG, epd-engine.h), a
     * lane not in the stagger is parked (hot EPD_STAG_PARKL): on its group's
     * frame a parked lane with arm (no evict) takes its cold state (the
     * engine's D) and decides as a loaded one; a lane with evict whose step is
     * done parks, its cold level decoded (payrule's budget dropped); a word
     * whose 8 lanes are parked and that holds an evict on its group's frame
     * acks it (ack 1 | tog << 1, tog: its first evicted byte's) */
    int lanes;
    /* ctl 4: the maintenance visit's period in cycles (epd-bp4.h: the
     * payback and the DC fold; 0: 4) */
    int vm;
    /* ctl 4: the idle as the shared word-diagonal mask (1: cheaper, the old
     * look) instead of ctl 3's per-pixel K (0, epd-bp4.h kc) */
    int sidle;
} epd_stag_par;

/* the cells' shape is one the tables take (epd_stag_par.cellw, cellh) */
int epd_stag_cells_ok(int w, int h);
/* "cells" (8 x 8), "cellsN" (N x N), "cellsWxH" at s: 0 and the shape, -1
 * (not one, or not a shape epd_stag_cells_ok takes) */
int epd_stag_cells_parse(const char *s, int *w, int *h);
enum { EPD_STAG_CORD_BN = 0, EPD_STAG_CORD_WBAYER = 1 };
/* the tables of the cell shape w x h (NULL: not a valid one), built on first
 * use (any thread), never freed. [i] for i = (cy & 63) * 64 + (cx & 63): cell
 * (cx, cy)'s rank s << (12 - log2 N) (s its place in its tile, 0..N - 1; N =
 * (64 / w) (64 / h) cells a tile; the cell grid repeats every 64 cells);
 * [4096 + i] the inverse, laid out like the forward half: at (tile column
 * << tx | s % (1 << tx), tile row << ty | s >> tx), tx = log2 (64 / w), ty =
 * log2 (64 / h), the tile's cell (j << tx | i) of place s. ord: the order
 * (epd_stag_par.cellord) */
const uint16_t *epd_stag_cells(int w, int h, int ord);

static inline int epd_stag_srank(int s)     /* (the 5 bits of s reversed, << 7: 0..3968) */
{
    s &= 31;
    return ((s & 1) << 4 | (s & 2) << 2 | (s & 4) | (s & 8) >> 2 | (s & 16) >> 4) << 7;
}
/* block (bx, by)'s rank: the 16 x 16 Bayer index, << 4 (0..4080). a = bx ^ by,
 * b = by (4 bits each), bit i of a -> bit 7 - 2i, bit i of b -> bit 6 - 2i:
 * the finest bits on top, so the first G ranks' groups interleave at the
 * finest scale (G 2: (bx ^ by) & 1; G 4: a 2 x 2 cell; G 16: 4 x 4). The
 * same arithmetic in epd-gpu.c stag_cs (BLOCKS) */
static inline int epd_stag_brank(int bx, int by)
{
    int a = (bx ^ by) & 15, b = by & 15;
    return ((a & 1) << 7 | (b & 1) << 6 | (a & 2) << 4 | (b & 2) << 3 | (a & 4) << 1 | (b & 4) | (a & 8) >> 2 | (b & 8) >> 3) << 4;
}
/* the rank that picks pixel (x, y)'s group (r: its blue-noise rank) */
static inline int epd_stag_crank(const uint16_t *ct, const epd_stag_par *p, int x, int y)
{
    return ct[((y >> __builtin_ctz((unsigned)p->cellh)) & 63) * 64 + ((x >> __builtin_ctz((unsigned)p->cellw)) & 63)];
}
static inline int epd_stag_grank(const epd_stag_par *p, int x, int y, int r)
{
    if (p->stripe <= 0) return r;
    if (p->cellw) {
        const uint16_t *ct = epd_stag_cells(p->cellw, p->cellh, p->cellord);
        if (ct) return epd_stag_crank(ct, p, x, y);
    }
    if (p->stripe == 64)                    /* (the knob's only width: shifts, not divisions; x, y >= 0) */
        return p->blocks ? epd_stag_brank(x >> 6, y >> 6) : epd_stag_srank(x >> 6);
    return p->blocks ? epd_stag_brank(x / p->stripe, y / p->stripe) : epd_stag_srank(x / p->stripe);
}

typedef struct {
    uint16_t *hot;                  /* EPD_W * EPD_H each */
    uint32_t *cold;
    uint8_t *tgt;
    /* per pixel: 0 parked (not in this run: inert, never driven, not given
     * back), 1 loaded, 2 loaded since the GPU last took the loads */
    uint8_t *ld;
    uint32_t f;                     /* frames rendered */
    uint8_t *ack;                   /* lanes: per word (EPD_W / 8 x EPD_H) the last ack (1 | tog << 1; 1: parked since the start) */
} epd_stag;
/* lanes: a parked lane's hot state (level class 3: never a target's; inert) */
#define EPD_STAG_PARKL (3u << 12)
/* the hot state of a parked pixel: level class black, model "at the rail",
 * target black: nothing to do in any mode */
#define EPD_STAG_PARKED (1u << 14)        /* (epd-gpu.c stag_load_cs: 0x40004000u) */

int epd_stag_init(epd_stag *s);     /* 0 / -1 (no memory) */
/* lanes: every lane parked, every word's ack 1 (the start: nothing armed) */
void epd_stag_lanes_park(epd_stag *s);
void epd_stag_free(epd_stag *s);

static inline uint32_t epd_stag_cold(int level, int op, int stale, int dc)
{
    return (uint32_t)(level & 31) | (uint32_t)(op & 63) << 5 | (uint32_t)(stale & 1) << 11 | (uint32_t)(uint16_t)dc << 16;
}
#define EPD_STAG_LEVEL(c) ((int)((c) & 31))
#define EPD_STAG_OP(c) ((int)(((c) >> 5) & 63))
#define EPD_STAG_STALE(c) ((int)(((c) >> 11) & 1))
#define EPD_STAG_DC(c) ((int)(int16_t)((c) >> 16))
#define EPD_STAG_K(c) ((int)(((c) >> 12) & 15))
#define EPD_STAG_LV(h) (((h) >> 12) & 3u)
#define EPD_STAG_SET(h) (((h) >> 14) & 1u)
#define EPD_STAG_DUE(h) (((h) >> 15) & 1u)          /* (ctl 1 pay without payrule; epd_stag_hotbits) */
/* payrule: a rail (0 / 30) with b > 0 payback frames still to come as a cold
 * level, and back (b 0: the level itself) */
static inline int epd_stag_lenc(int rail, int b) { return b <= 0 ? rail : rail ? 31 - 2 * b : 2 * b - 1; }
static inline int epd_stag_ldec(int l, int *b)
{
    if (!(l & 1)) { *b = 0; return l; }
    if (l < 15) { *b = (l + 1) / 2; return 0; }
    *b = (31 - l) / 2;
    return 30;
}
/* the blue-noise dither: a pixel of rank r goes white from level 2 * dq(r)
 * up (dq 1..15: target t > (r + 1/2) / 4096 of white, for even t) */
static inline int epd_stag_dq(int r) { return (r * 30 + 15) / 8192 + 1; }
/* the stagger's dither: dq of each pixel [y & 63][x & 63] from the engine's
 * pattern (epd-dither.h epd_dither_rank: the same dots at a cut's handover,
 * and the blue noise's independent of the stagger's groups); a pattern change
 * bumps epd_stag_dgen (the GPU rebuilds its tables) */
extern uint8_t epd_stag_dqt[64][64];
extern unsigned epd_stag_dgen;
void epd_stag_dither_set(int pattern);
static inline void epd_stag_dither_need(void) { if (!epd_stag_dgen) epd_stag_dither_set(0); }
/* the hot bits of a cold state: level class, model at the rail, payback due
 * (ctl 1 with pay > 0 and no payrule: the DC record opposes the rail's drive
 * by >= pay; payrule pays only on arrivals: no due bit) */
static inline unsigned epd_stag_hotbits(int l, int op, int dc, const epd_stag_par *p)
{
    unsigned lv = l == 0 ? 0u : l == 30 ? 1u : 2u;
    unsigned set = lv == 0 ? op == 0 : lv == 1 ? op == p->R : 0;
    int owe = lv == 1 ? -dc : dc;
    unsigned due = lv < 2 && p->ctl && !p->payrule && p->pay > 0 && owe >= p->pay;
    return lv << 12 | set << 14 | due << 15;
}

/* a run starts: every pixel parked (the slot engine idle: no run in flight;
 * its state is read pixel by pixel as the run's targets reach them) */
void epd_stag_enter(epd_stag *s);
/* the same when only the pixels of r can be loaded (the last run's loads:
 * outside them everything is still parked since an epd_stag_enter) */
void epd_stag_repark(epd_stag *s, epd_rect r);
/* the pixels of r not loaded yet: the engine's state (frozen while the
 * stagger runs) — level, model (from the level if e keeps none or another
 * range than p->R), DC record, stale; no step; target = level (p: R and the
 * hot bits' pay). The number loaded */
long epd_stag_load(epd_stag *s, const epd_engine *e, const epd_stag_par *p, epd_rect r);
/* the loaded pixels back into the engine (every step done: drained): level =
 * target = the level reached, busy 0, model (if e keeps one), DC record,
 * stale; parked ones are left as they were */
void epd_stag_exit(const epd_stag *s, epd_engine *e);
/* the newest content over r (loaded first: epd_stag_load): 5-bit levels, r.w
 * per row */
void epd_stag_target(epd_stag *s, epd_rect r, const uint8_t *levels);
/* frame s->f's data words of panel rows [y0, y1) into words (an EPD_WORDS
 * frame: those words = template | phases; nothing else touched), the state
 * stepped; then epd_stag_advance once per frame */
void epd_stag_rows(epd_stag *s, const epd_stag_par *p, const uint32_t *tpl, uint32_t *words, int y0, int y1);
static inline void epd_stag_advance(epd_stag *s) { s->f++; }
/* lanes : the CPU's side, on the engine's D/W/T (claim_state
 * packed; rects in panel pixels, buffers EPD_W apart). D is the CPU's where a
 * lane is parked, the GPU's where it is armed (ownership by the 64-B line =
 * 16 pixels: evict pads to them). The caller syncs (epd_dmabuf_sync: T END|
 * WRITE after arm/evict/takeover, D END|WRITE after resolve, D START|READ
 * before takeover) and notes the rect for the GPU (epd_gpu_stag_target).
 * The CPU writes D only where a whole 16-px line is parked: inside an acked
 * evict's (padded) rect, or anywhere while nothing is armed; arm pads too,
 * so a line is never part armed, part parked (a cache-line write-back of the
 * CPU's would clobber the GPU's stores of the same line).
 * lane_pad: r widened to 16-px columns (clipped)
 * lane_resolve: FIN words of r resolved (D's level := T's, W idle; the W
 * write opened with epd_pk_w_begin): before an arm, while no lane in r's
 * lines is armed (a run's start). The number resolved (> 0: D written)
 * lane_arm: T := tgt | ARM over r, the pad to lane_pad(r) T := its own
 * target | ARM, both where mask (NULL: everywhere) is set (a mask must be
 * 16-px aligned: an evict's rects): the padded rect
 * lane_evict: T := its target | EVICT | TOG over lane_pad(r) (that rect
 * returned), TOG per word the other of its last ack's (ack: the GPU's,
 * epd_gpu_stag_acks, read since that word's last evict): an ack of an
 * earlier evict never passes for this one, and the CPU never writes A
 * lane_unacked: words of r whose ack is not 1 | its T's TOG (0: all parked)
 * lane_takeover: r acked: T := D's level where ld is set (ld cleared),
 * elsewhere only the request bits cleared
 * lane_mirror: the CPU reference's targets := T over r, its cold := D where
 * its lane is parked (after the CPU wrote them) */
epd_rect epd_stag_lane_pad(epd_rect r);
long epd_stag_lane_resolve(epd_engine *e, epd_rect r);
epd_rect epd_stag_lane_arm(epd_engine *e, epd_rect r, const uint8_t *tgt, const uint8_t *mask);
epd_rect epd_stag_lane_evict(epd_engine *e, epd_rect r, const uint8_t *ack);
long epd_stag_lane_unacked(const epd_engine *e, const uint8_t *ack, epd_rect r);
void epd_stag_lane_takeover(epd_engine *e, epd_rect r, uint8_t *ld);
void epd_stag_lane_mirror(epd_stag *s, const epd_engine *e, epd_rect r);
/* pixels with a step in flight after frame s->f - 1 (tests) */
long epd_stag_active(const epd_stag *s);

#endif
