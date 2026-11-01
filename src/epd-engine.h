/* epd-engine — the Note Air 1 e-ink software TCON in userspace (for the mainline
 * kernel, where no Onyx /dev/ebc driver exists).
 *
 * Pure computation, no I/O: the frame generator,
 * plus a scheduler with these update semantics:
 *
 * - an update = a panel rect + a waveform mode; the target grey levels are
 * copied at submit;
 * - updates on disjoint pixels run concurrently, one shared drive frame;
 * - an update touching pixels still in flight waits until they are done
 * and never overtakes an earlier pending update
 * it overlaps;
 * - per pixel: index = (new << 5) | old (5-bit levels, new = (grey >> 4) * 2),
 * phase = LUT[frame][index], LUT diagonal zeroed (unchanged = never driven).
 * The caller renders one drive frame at a time into a 285 x 1423 word buffer
 * (control template + 2-bit phases in data bits 0..15) and sends it exactly once.
 *
 * Panel coordinates: 1872 x 1404 landscape.
 * The Note Air 1 needs the row mirror (test card): panel row y goes out
 * on wire line 1403 - y.
 */
#ifndef EPD_ENGINE_H
#define EPD_ENGINE_H

#include <stddef.h>
#include <stdint.h>

#define EPD_W 1872
#define EPD_H 1404
#define EPD_LINE 285            /* words per wire line */
#define EPD_LINES 1423          /* wire lines per frame */
#define EPD_WORDS (EPD_LINE * EPD_LINES)
#define EPD_DATA0 1469          /* word of panel col 0 on the wire line of panel row EPD_H - 1 (line 5, word 44) */
#define EPD_MAX_UPD 64          /* concurrently running updates (a thick fast pen stroke peaked at 29; busy[] holds slot + 1 <= 255) */
#define EPD_MAX_PENDING 256
#define EPD_MERGE_GAP 8                     /* px: pieces this close count as touching */
#define EPD_MERGE_MAX (512 * 512)           /* px: never grow a merged update beyond */
#define EPD_MAX_CUT 5           /* the cut's range (DU, epd_set_cut) */
#define EPD_LUT_CUTS 9          /* cut tables cached per mode: 0..8 frames off */

/* WBF modes (mode_version 0x19 order, spec) */
enum { EPD_INIT = 0, EPD_DU = 1, EPD_GC16 = 2, EPD_GL16 = 3, EPD_A2 = 6, EPD_DU4 = 7 };
/* pseudo-mode: decided when the update starts (the old levels are known then):
 * A2 (10 frames at 24 C, the WBF's real A2, not slot
 * 22) if every pixel goes black/white -> black/white, else DU. A2 leaves more
 * ghosting than DU, which the caller's cleanups (GC16) take away, as for DU.
 * They do NOT take away DC. No waveform is balanced per transition, only over
 * a pair of the same mode (24 C net drive frames: A2 white<-black +9 /
 * black<-white -9, cut A2 +-(len - 1), DU and GC16 +-21; a full GC16 drives a
 * black pixel that stays black -2). Mixing them leaves the difference: A2 to
 * black and GC16 back to white = +12, and each full flash adds -2 to a black
 * pixel. The DC record counts it (epd_dc_stats); matched cuts
 * (epd_set_matched) steer it back (test/epd-matched-test). */
#define EPD_A2_OR_DU 100
/* a cut A2 (motion=adaptive, the fast steps of a scroll): mode | len << 8 =
 * A2 shortened to len frames (its last one zeroed, as the DU cut does:
 * len - 1 drive frames) for EPD_A2 and the A2 part of EPD_A2_OR_DU; len 0 =
 * uncut. The pixels stop short of black/white, but the engine records the
 * target: the policy must drive the area fully afterwards (a flash). */
#define EPD_MODE_A2LEN(mode, len) ((mode) | (len) << 8)
#define EPD_MODE_BASE(mode) ((mode) & 0xff)

typedef struct {
    const uint8_t *d;
    size_t n;
    int modes, temps, frame_rate;
    uint8_t ranges[64];
    uint32_t mode_table;
} epd_wbf;

int epd_wbf_load(epd_wbf *w, const uint8_t *data, size_t n);        /* 0 or -1 */
int epd_wbf_temp_index(const epd_wbf *w, int celsius);
/* frames of (mode, temp index): *out = malloc'd nframes * 1024 phases,
 * [frame][new * 32 + old], diagonal zeroed. Returns nframes or -1. */
int epd_wbf_frames(const epd_wbf *w, int mode, int temp, uint8_t **out);

void epd_template(uint32_t *words);         /* EPD_WORDS: control template */

typedef struct { int x, y, w, h; } epd_rect;

typedef struct {
    epd_rect r;
    int mode, full;
    int frame, nframes;
    const uint8_t *lut;                     /* nframes * 1024, owned by the cache */
    uint32_t id;
    int payback;                            /* matched pixels longer than their step (epd_set_nonblock) */
} epd_run;

typedef struct {
    epd_rect r;
    int mode, full;
    uint8_t *target;                        /* w * h 5-bit levels, owned */
    uint32_t id;
    const uint8_t *src;                     /* submit's grey buffer + stride: merge only */
    int stride;                             /* pieces of the same frame buffer */
    int kept;                               /* started in part, its rest waits for payback pixels */
    int flash;                              /* full == 2 at submit: only stale or changed pixels (full = 1) */
    /* with pixel ops (epd_set_pixel_ops): the run slots whose pixels over r
     * head elsewhere than its target (it waits for them); found by the query
     * when bvalid is 0, kept up to date by claims over r (bvalid 0) and
     * finishing runs (their bit cleared) */
    uint64_t bmask;
    int bvalid;
    /* stage 5: the target made on the GPU (a handle of the
     * caller's, epd_submit_gt; 0 none). target is then NULL until the CPU
     * needs the levels (epd_target_ops fetch: once, kept beside the handle) */
    int gt;
    int job;                                /* its claim is out as a job (epd_set_overlap): submit leaves it alone */
} epd_pend;

/* the owners a pass reports extents for: the step's run (A2, or all), its DU
 * part (A2_OR_DU split), the heal run, the matched lengths (cut c) */
enum { EPD_CL_A, EPD_CL_D, EPD_CL_H, EPD_CL_M0, EPD_CL_OWNERS = EPD_CL_M0 + EPD_LUT_CUTS };
typedef struct {
    /* per owner: the rect columns [x0, x1] and rows [y0, y1] where it took
     * pixels (x1 < 0: none) */
    int x0[EPD_CL_OWNERS], x1[EPD_CL_OWNERS], y0[EPD_CL_OWNERS], y1[EPD_CL_OWNERS];
    int mslot[EPD_LUT_CUTS];                /* the matched cuts' run slots (-1: none) */
    uint64_t preempted, snapped, matched, healed, flash_skipped;
} epd_claim_out;

typedef struct {
    epd_wbf wbf;
    int celsius;
    /* the per-pixel state, EPD_W apart (claim_state arrays and check; NULL
     * under packed, where pk's D/W/T are the only state: epd_px_*) */
    uint8_t *level;                         /* 5-bit level each pixel shows (old) */
    uint8_t *target;                        /* 5-bit target while in flight */
    uint8_t *busy;                          /* 0 idle, else run slot + 1 */
    epd_run run[EPD_MAX_UPD];
    int nrun;
    epd_pend pend[EPD_MAX_PENDING];
    int npend;
    uint32_t next_id;
    uint64_t superseded;                    /* pending updates dropped, see epd_submit */
    uint64_t merged;                        /* submits merged into a pending update */
    /* why pending updates waited (per epd_schedule pass and update): an earlier
     * overlapping pending one / a pixel in flight elsewhere / no free run slot */
    uint64_t wait_order, wait_pixel, wait_slot;
    int nrun_max, npend_max;                /* peaks since epd_stats_reset */
    int cut;                                /* DU frame cut, see epd_set_cut */
    uint8_t *lutcache[2][16][64][EPD_LUT_CUTS];      /* [full][mode][temp][cut] */
    int lutframes[2][16][64][EPD_LUT_CUTS];
    /* per pixel: net drive frames (+ one polarity, - the other) of every
     * transition started, clamped: the DC an unbalanced waveform (A2, cuts)
     * leaves on the film (stats only, epd_dc_stats) */
    int16_t *dc;
    int matched;                            /* see epd_set_matched */
    int matched_from, matched_cap;          /* see epd_set_matched_rule */
    uint64_t matched_px;                    /* pixels given a matched length */
    int nonblock;                           /* see epd_set_nonblock */
    uint64_t split_starts;                  /* updates started around payback pixels (stats) */
    uint64_t absorbed;                      /* waiting (kept) updates taken into a newer step */
    uint64_t preempted;                     /* payback pixels taken by a newer step (nonblock 2) */
    void *mtab[2][64];                      /* [full][temp]: matched cuts' tables (claim) */
    /* the panel model (epd_set_model): per pixel where it really is, in drive
     * frames from black (0) to white (op_r = the uncut A2's net drive) */
    uint8_t *op;
    int op_r, op_tidx, op_ready, op_track;
    int model;                              /* matched lengths from: 0 the DC record, 1 the panel model */
    int heal_min, heal_from, heal_len;      /* see epd_set_heal, epd_set_heal_len */
    uint64_t healed_px;                     /* pixels given a heal drive (stats) */
    int snap;                               /* see epd_set_snap */
    uint64_t snapped_px;                    /* levels snapped to the model's rail (stats) */
    uint8_t *heallut[9];                    /* heal of k drive frames: k + 1 frames x 1024 */
    /* per pixel: 1 = driven by something other than an exact GC16/GL16 since
     * its last one (cuts, A2, DU, heal: may hold a ghost); a flash (full 2)
     * skips the 0 ones that do not change (static black UI took -2 DC frames
     * at every run-end flash: the shell bar) */
    uint8_t *stale;
    uint64_t flash_skipped;                 /* pixels a flash left alone (stats) */
    uint32_t tpl[EPD_WORDS];
    const struct epd_pixel_ops *px;         /* epd_set_pixel_ops */
    const struct epd_target_ops *tops;      /* epd_set_target_ops */
    /* epd_set_defer: the claim whose report is still to come (on), its runs
     * started over its whole rect r (slot per owner, -1 none); px: the ops
     * that collect it (NULL: out is the CPU's, done); nend: nrun at the end
     * of its epd_schedule (nrun_max once exact; -1 not yet) */
    int defer;
    struct {
        int on;
        const struct epd_pixel_ops *px;
        epd_rect r;
        uint32_t id;
        int slot[EPD_CL_OWNERS];
        int nend;
        epd_claim_out out;
    } dfr;
    uint64_t deferred, settle_waits;        /* claims deferred / settled with a wait (stats) */
    /* banded claims and the job (epd_set_band_ops, epd_set_overlap) */
    const struct epd_band_ops *bops;
    int overlap;
    struct epd_cjob *job;
    unsigned long fcount;                   /* epd_advance calls (the effective frame's clock) */
    uint64_t jobs, job_late, job_slip, job_joins, job_qfin, banded;   /* (stats) */
    double job_apply_s, job_apply_max;      /* the apply step on the caller (s) */
    /* claim_state (epd_claim_state, read at epd_init):
     * 0 arrays (the six arrays above), 1 packed (pk's D/W/T only), 2 check
     * (both: the arrays the truth, D/W/T kept beside them and compared per
     * claim) */
    int cst;
    struct epd_pk *pk;                      /* packed and check: D/W/T (NULL: arrays) */
} epd_engine;

/* A claim (an update starting, epd_schedule) in three parts, so its per-pixel
 * part can run elsewhere (on the GPU, epd-gpe): the plan (run
 * slots, tables; CPU), the pixel pass over the rect (epd_claim_pixels on the
 * CPU), and the runs started from what the pass took (CPU). */
#define EPD_RUN_SLOTS (EPD_MAX_UPD - 2)     /* the last two slots: heal runs */
#define EPD_HEAL_B (EPD_RUN_SLOTS + 1)      /* busy >= this: a heal run's pixel */
#define EPD_MT_D 31                         /* matched cuts: DC record range of the table */
typedef struct {
    epd_rect r;                             /* the update's rect (clipped) */
    const uint8_t *tg;                      /* its target levels, r.w per row (NULL: only on the GPU, gt) */
    int gt;                                 /* the pending's GPU target (epd_pend gt; 0 none) */
    epd_pend *pend;                         /* the pending (epd_claim_pixels fetches tg from it when NULL) */
    int fm;                                 /* also drives unchanged pixels: 0 no, 1 all (full), 2 the stale ones (flash) */
    int split;                              /* A2_OR_DU: black/white -> black/white to oa, the rest to od */
    uint8_t oa, od;                         /* their busy values (slot + 1; od 0 = none) */
    uint8_t va, vd;                         /* e->stale of a pixel they take (1: not an exact mode) */
    int8_t neta[1024], netd[1024];          /* their net drive per transition (the DC record) */
    /* matched cuts (mt): the step's cut cs; per cut its busy value (0 = none:
     * the pixel keeps the step's length), net drive and the best-cut table */
    int mt, cs, cap, model;
    uint8_t mown[EPD_LUT_CUTS];
    /* lazy: fewer free slots than cuts the pass may pick: the pass takes one
     * per cut on first use (sa, sd: not those), so the cuts that occur first
     * get them (the GPU pass can't: it takes the CPU's in that case) */
    int lazy, sa, sd;
    const int8_t (*mnet)[1024];
    const uint8_t (*best)[2 * EPD_MT_D + 1];
    int opm, R, aclamp;                     /* the panel model kept; op_r; oa's A2 moves the model by its net */
    /* op_prep before the pass changed the whole panel's model (not only the
     * rect): 1 started from the levels, 2 rescaled from op_old to R */
    int op_init, op_old;
    uint8_t rail[32];                       /* model value of a level reached */
    uint8_t hown;                           /* the heal run's busy value (0 = no healing) */
    int hmin, hmax;
    int hact;                               /* a heal run in flight: its pixels may be taken (claim_row_heal) */
    uint8_t hf0, hf1;                       /* the two heal slots' frames so far */
    int npre;                               /* preemptible payback runs (nonblock 2): pre[busy] = 1.. */
    uint8_t pre[256];
    int8_t rn[EPD_RUN_SLOTS + 1][4];        /* their net drive still to come per (to, from) black/white */
    int snp;                                /* snap (epd_set_snap) before the claim */
} epd_claim_plan;
/* the pixel pass of a claim: busy, target, level, model, DC record and stale
 * over pl->r as the plan says; the extents and counters into out */
void epd_claim_pixels(epd_engine *e, epd_claim_plan *pl, epd_claim_out *out);
/* (tests): how much of each claim needs the pixel pass, per tile of
 * EPD_TS_N sizes (16, 32, 64 px square, on the panel grid). A tile needs it if
 * any pixel is busy, has a target other than its level, is in a flash (all,
 * or the stale ones) or starts a heal; the rest of a claim's rect would be
 * left as it is. Counted before the pass, only while epd_tile_stat_on */
#define EPD_TS_N 3
typedef struct {
    uint64_t claims, px, px_chg;                    /* claims, their pixels, pixels whose target differs */
    uint64_t tiles[EPD_TS_N], work[EPD_TS_N];       /* tiles touched / needing the pass */
    uint64_t chg[EPD_TS_N], busy[EPD_TS_N], flash[EPD_TS_N], heal[EPD_TS_N];   /* by first reason */
} epd_tile_stats;
extern int epd_tile_stat_on;
extern epd_tile_stats epd_tile_stat;
/* (epd-drm.c drive_prof): where epd_schedule's and epd_advance's
 * time goes, summed while epd_sched_prof is set (CLOCK_MONOTONIC, s; the
 * caller reads and zeroes them per frame; off: one branch per claim). pix:
 * the pixel passes (epd_claim_pixels or the pixel ops), plan: claim_plan,
 * start: claim_start + the pendings' blocked flags, blocked: pend_blocked's
 * passes (blocked_row or the query), finish: epd_advance's finish_row passes
 * (fin_px: the finished runs' rect pixels, fin_runs: the runs) */
typedef struct {
    double pix, plan, start, blocked, finish;
    unsigned long claims, fin_runs;
    unsigned long long px, fin_px;
} epd_sched_times;
extern int epd_sched_prof;
extern epd_sched_times epd_sched_t;
/* a lazy plan's matched slots, given the cuts its pixels want in the order the
 * lazy pass meets them (raster order): taken as that pass would, then the
 * plan is no longer lazy (the GPU's pass: a probe finds the order) */
void epd_claim_take_slots(const epd_engine *e, epd_claim_plan *pl, const int *cuts, int n);
/* The per-pixel parts done elsewhere (on the GPU). pixels: a
 * claim's pixel pass, 0 = done (else epd_claim_pixels does it); finish: the
 * runs in slots (bit s = run[s], still set) end in epd_advance, 0 = their
 * pixels are done (else the CPU does them). NULL = all on the CPU. */
typedef struct epd_pixel_ops {
    int (*pixels)(void *ctx, epd_engine *e, epd_claim_plan *pl, epd_claim_out *out);
    int (*finish)(void *ctx, epd_engine *e, uint64_t slots);
    /* for pending p: *mask = the slots < EPD_HEAL_B with pixels over p->r in
     * flight elsewhere than p's target; *nonbw = 1 if a pixel there that is
     * idle and changes (any, if p->full) is not black/white -> black/white.
     * With pixel ops set, query is required (the CPU's arrays may be stale) */
    void (*query)(void *ctx, epd_engine *e, const epd_pend *p, uint64_t *mask, int *nonbw);
    void *ctx;
    /* deferred claims (epd_set_defer; NULL = pixels, synchronously): submit
     * starts plan pl's pass, 0 = started (else pixels or the CPU does it);
     * collect gives its report into out: 0, 1 = not done yet (wait 0 only),
     * -1 lost (the runs keep their whole rect, the counters are lost) */
    int (*submit)(void *ctx, epd_engine *e, epd_claim_plan *pl);
    int (*collect)(void *ctx, epd_engine *e, epd_claim_out *out, int wait);
} epd_pixel_ops;
/* the claim pass on the unified state's packed
 * words instead of the six arrays, per pixel
 * D 32 bit level 5 | op 6 << 5 | stale 1 << 11 | k 4 << 12 | dc 16 << 16
 * (the stagger's cold word, epd_stag_cold; k is the stagger's,
 * kept as found)
 * W 16 bit (busy - 1) << 10 | target << 5 | level, EPD_PK_IDLE for busy 0
 * (the GPU's st code, epd-gpu.c state_row)
 * T 8 bit target (as e->target, every pixel; the request bits 0)
 * epd_claim_kernel: 0 the arrays' pass (soa, default), 1 the packed pass
 * (packed), 2 both, the arrays' result kept and the packed one compared
 * (check: epd_claim_kernel_diffs); -1 = GLASSINE_CLAIM_KERNEL=soa|packed|check at
 * the first claim. Read at every claim: live. Until the arrays stay the
 * engine's state: the packed pass packs each row of the rect from them and
 * unpacks it back (in L1: the boundary, gone once D/W/T persist). Lazy
 * plans (slots taken in pixel order) and > 4 preemptible runs take the
 * arrays' pass (epd_claim_kernel_soa counts them). */
#define EPD_PK_IDLE 1023
extern int epd_claim_kernel;
extern unsigned long epd_claim_kernel_soa, epd_claim_kernel_diffs;
static inline uint32_t epd_pk_d(int level, int op, int stale, int dc)
{
    return (uint32_t)(level & 31) | (uint32_t)(op & 63) << 5 | (uint32_t)(stale & 1) << 11 | (uint32_t)(uint16_t)dc << 16;
}
static inline uint16_t epd_pk_w(int busy, int target, int level)
{
    return busy ? (uint16_t)((busy - 1) << 10 | (target & 31) << 5 | (level & 31)) : (uint16_t)EPD_PK_IDLE;
}
/* 's finish (the A73's gate: the finish at <= 1.5x the arrays'):
 * a finished run's pixel gets W = EPD_PK_FIN, D untouched: its level is T's
 * (level := target, as finish_row), D's level bits stale until a write of the
 * pixel's D (a claim over it, epd_px_put) resolves them. Any code with target
 * field 31 is idle (targets are 0..30: never a run's code; its LUT entries are
 * zero, epd-gpu.c / epd_render); EPD_PK_IDLE: idle, D's level valid */
#define EPD_PK_FIN (0x3E0 | 30)
#define EPD_PK_ISIDLE(w) (((w) & 0x3E0) == 0x3E0)
#define EPD_PK_BUSY(w) (EPD_PK_ISIDLE(w) ? 0 : ((w) >> 10) + 1)
static inline int epd_pk_lev(uint32_t d, uint16_t w, uint8_t t) { return w == EPD_PK_FIN ? t & 31 : (int)(d & 31); }
/* D/W/T as the engine's state. claim_state
 * (epd_claim_state; -1 = GLASSINE_CLAIM_STATE=arrays|packed|check at epd_init,
 * not live): arrays (default) = the six arrays, today's code; packed = D, W,
 * T only (the claim on the words in place, claim_kernel packed forced; the
 * finish, the render, the queries on the words; epd-gpu imports W, so
 * drive_state heap is forced); check = both, the arrays the truth and D/W/T
 * kept beside them by the same steps, compared per claim over its rect
 * (epd_claim_state_diffs; no jobs, as claim_kernel=check). Rows EPD_PK_S
 * pixels apart (D 7680, W 3840, T 1920 bytes: imports them as they are).
 * W: the engine's allocation, a dma-buf (/dev/dma_heap/system, else a
 * udmabuf) when it can get one (wfd >= 0: epd_gpu_open imports it), else
 * malloc (GLASSINE_CLAIM_STATE_W=malloc forces it: epd-gpu then packs its st
 * texture from W, the tex fallback); D, T: malloc (not imported before). Under a job (drive_overlap)
 * its bands write D and T in place and W into Wsh (the rect's rows, r.w
 * apart): W, which the drive reads (the GPU's fs, the CPU's epd_render),
 * changes only at the apply (frame k + 2); D/T over the rect are then k + 2
 * already, so a reader of them outside epd_schedule / epd_advance joins
 * first (epd_job_join) */
#define EPD_PK_S 1920
#define EPD_PK_DPAD 128
/* T's request bits (; the stagger's, kept by every engine
 * store): tog (the evict request's parity, echoed by the ack), arm (a parked
 * lane takes D and starts), evict (a lane parks, its final D) */
#define EPD_PK_TOG 0x20u
#define EPD_PK_ARM 0x40u
#define EPD_PK_EVICT 0x80u
/* stagger_state (epd_stagger_state; -1 = GLASSINE_STAGGER_STATE=lanes at epd_init,
 * else runs; not live): lanes = D and T in dma-bufs (claim_state packed) */
extern int epd_stagger_state;
/* DMA_BUF_IOCTL_SYNC over bytes [off, off + len) of fd: the ACK's range
 * ioctl (DMA_BUF_IOCTL_SYNC_PARTIAL) when present (epd_dmabuf_partial: -1
 * not probed, 0 none, 1 used; GLASSINE_DMABUF_PARTIAL=0 never), else the whole
 * buffer; fd < 0: nothing. 0 / -1. Totals: epd_dmabuf_sync_s, _n */
int epd_dmabuf_sync_at(const char *file, int line, int fd, unsigned flags, size_t off, size_t len);
#define epd_dmabuf_sync(fd, flags, off, len) epd_dmabuf_sync_at(__FILE__, __LINE__, (fd), (flags), (off), (len))
/* per call site (which syncs carry the stagger's
 * 62-916 ms a run): calls, time, bytes asked (len 0 = the whole buffer, counted
 * apart); epd_dmabuf_sync_sites writes one line a site to f (a FILE *), the
 * costliest first, and resets them */
void epd_dmabuf_sync_sites(void *f);
extern int epd_dmabuf_partial;
extern double epd_dmabuf_sync_s;
extern unsigned long epd_dmabuf_sync_n;
#define EPD_NPX ((size_t)EPD_W * EPD_H)
typedef struct epd_pk {
    uint32_t *D;
    uint16_t *W;
    uint8_t *T;
    int wfd;                                /* W's dma-buf (-1: malloc) */
    size_t wsize;
    const char *wsrc;                       /* dma-heap, udmabuf, malloc */
    int wdirty;                             /* W written since the last END|WRITE (epd_pk_w_sync) */
    int wy0, wy1;                           /* its rows (the START|WRITE ones; the END's range) */
    uint16_t *Wsh;                          /* the job's W (packed): the rect's rows */
    /* (stagger_state=lanes): D and T as dma-bufs (dfd,
     * tfd; -1: malloc) the stagger's pass imports (D: EPD_PK_DPAD rows more,
     * the GPU's scratch) */
    int dfd, tfd;
    size_t dsize, tsize;
    const char *dtsrc;
    /* rows[s][y / 64] bit y % 64: row y may hold slot s's pixels (set where a
     * claim, pk_put or a claim's arrays pass gives a pixel busy s + 1; cleared
     * when epd_advance finishes run s; a set bit for a row without one is only
     * a wasted scan). The finish reads only those rows (finish_mask) */
    uint64_t rows[EPD_MAX_UPD][(EPD_H + 63) / 64];
} epd_pk;
extern int epd_claim_state;
/* claim_prefetch (; epd_claim_prefetch, -1 = GLASSINE_CLAIM_PREFETCH=1
 * at the first claim, default off): the in-place claim prefetches row y + 1's
 * lines of D, W, T and the target before row y (a device A/B: rows are 7.5 KB
 * apart, a pen rect's rows each start a new stream) */
extern int epd_claim_prefetch;
/* finish_mask (; epd_finish_mask, -1 = GLASSINE_FINISH_MASK at the
 * first finish, default 1 = on, 0 = off: every row of the run's rect read):
 * epd_advance's packed finish reads only the rows the run's slot was given
 * pixels in (epd_pk.rows). The same words either way */
extern int epd_finish_mask;
/* claim_skip (; epd_claim_skip, -1 = GLASSINE_CLAIM_SKIP=1 at the
 * first claim, default off): the sweep's 16-px step returns early when no
 * lane can change (all idle, not FIN, at their target, no heal candidate; not
 * in a flash plan's kinds): it writes the row buffers' values only. The same
 * words and report either way (a quiet step's full pass stores nothing) */
extern int epd_claim_skip;
extern unsigned long epd_claim_state_diffs, epd_claim_state_checked;
const char *epd_claim_state_name(const epd_engine *e);
/* the allocation for e->cst (epd_init does it; a calloc'ed engine without
 * epd_init: a test's); 0 / -1 */
int epd_state_alloc(epd_engine *e);
/* W was written by the CPU (START|WRITE before the first write) / the
 * drive takes it: END|WRITE if written (epd-gpu, before a frame reads W) */
void epd_pk_w_begin(const epd_engine *e);                 /* (all rows) */
void epd_pk_w_rows(const epd_engine *e, int y0, int y1);   /* rows [y0, y1) only (range-synced) */
void epd_pk_w_sync(const epd_engine *e);
static inline size_t epd_pk_at(size_t i) { return i / EPD_W * EPD_PK_S + i % EPD_W; }
/* one pixel (i = y * EPD_W + x) in either state (check: the arrays) */
static inline int epd_px_level(const epd_engine *e, size_t i)
{
    if (e->level) return e->level[i];
    size_t q = epd_pk_at(i);
    return epd_pk_lev(e->pk->D[q], e->pk->W[q], e->pk->T[q]);
}
static inline int epd_px_target(const epd_engine *e, size_t i) { return e->target ? e->target[i] : e->pk->T[epd_pk_at(i)] & 31; }
static inline int epd_px_busy(const epd_engine *e, size_t i) { return e->busy ? e->busy[i] : (int)EPD_PK_BUSY(e->pk->W[epd_pk_at(i)]); }
static inline int epd_px_op(const epd_engine *e, size_t i) { return e->op ? e->op[i] : (int)((e->pk->D[epd_pk_at(i)] >> 5) & 63); }
static inline int epd_px_stale(const epd_engine *e, size_t i) { return e->stale ? e->stale[i] : (int)((e->pk->D[epd_pk_at(i)] >> 11) & 1); }
static inline int epd_px_dc(const epd_engine *e, size_t i) { return e->dc ? e->dc[i] : (int16_t)(e->pk->D[epd_pk_at(i)] >> 16); }
/* fields of one pixel set (both states; W kept to busy/target/level) */
enum { EPD_F_LEVEL = 1, EPD_F_TARGET = 2, EPD_F_BUSY = 4, EPD_F_OP = 8, EPD_F_DC = 16, EPD_F_STALE = 32, EPD_F_ALL = 63 };
void epd_px_put(epd_engine *e, size_t i, unsigned what, int level, int target, int busy, int op, int dc, int stale);
/* the whole panel as six planes (EPD_W apart; a NULL plane skipped) / set
 * from them / one field := v everywhere (both states) */
void epd_state_get(const epd_engine *e, uint8_t *level, uint8_t *target, uint8_t *busy, uint8_t *op, int16_t *dc, uint8_t *stale);
void epd_state_set(epd_engine *e, const uint8_t *level, const uint8_t *target, const uint8_t *busy, const uint8_t *op,
                   const int16_t *dc, const uint8_t *stale);
void epd_state_fill(epd_engine *e, unsigned what, int v);
/* busy 0, dc 0, stale 1 (a GPU state lost) */
void epd_state_reset(epd_engine *e);
/* banding-ready (bands them; one band = the whole pass): what every band
 * reads, made once per claim (epd_claim_prep), and a band's scratch + report.
 * epd_claim_rows does rows [ya, yb) of pl->r (relative), any split of
 * [0, r.h) in any order or in parallel; epd_claim_merge the bands' reports
 * into out (as epd_claim_pixels makes it). Not lazy plans, not npre > 4
 * (epd_claim_bandable) */
typedef struct {
    struct { uint8_t own; int8_t net; uint16_t bit; } ctab[256];   /* the matched cut table (fast gather) */
    uint8_t pob[4];                         /* the preemptible runs' busy values (255: none) */
    int8_t prn[4][4];
    uint8_t l30;                            /* 30 (the white level, read at run time: see pk_rail) */
} epd_claim_job;
typedef struct {
    uint32_t d[EPD_W];                      /* the row's D, W, T (packed from the arrays) */
    uint16_t w[EPD_W];
    uint8_t t[EPD_W];
    uint8_t ca[EPD_W], cd[EPD_W], ch[EPD_W], fl[EPD_W], cq[EPD_W];
    int8_t nrow[EPD_W];
    uint16_t mcol[EPD_W], xl[EPD_W + 1];
    int mb1[EPD_LUT_CUTS], mb3[EPD_LUT_CUTS];
    int y0[3], y1[3];                       /* EPD_CL_A, _D, _H */
    uint64_t preempted, snapped, matched, healed, flash_skipped;
    struct {                                /* (a row's last < 16 px as one vector step: pk_tail16) */
        uint32_t d[16];
        uint16_t w[16];
        uint8_t t[16], tg[16], ca[16], cd[16], ch[16], fl[16], cq[16];
        int8_t nrow[16];
    } tl;
} epd_claim_band;
int epd_claim_bandable(const epd_claim_plan *pl);
void epd_claim_prep(epd_engine *e, epd_claim_plan *pl, epd_claim_job *j);
void epd_claim_rows(epd_engine *e, const epd_claim_plan *pl, const epd_claim_job *j, epd_claim_band *b, int ya, int yb);
void epd_claim_merge(const epd_claim_plan *pl, epd_claim_band *const *b, int n, epd_claim_out *out);
/* banded claims. epd_claim_bands = N > 1:
 * a bandable claim (epd_claim_bandable) of >= EPD_BAND_MIN_PX pixels runs
 * its rows in N bands (either kernel: the arrays' pass through its own row
 * range, the packed one through epd_claim_rows), merged as one band would
 * make it: the same bytes (test/epd-claim-bench, epd-sched-bench HASH=1).
 * <= 1: one band. -1 (the default) = GLASSINE_CLAIM_BANDS at the first claim, else 4.
 * The bands run where epd_set_band_ops says: run(fn, arg, n) = fn(arg, i)
 * for every i, part 0 on the caller, back when all are done; start = the
 * same on other threads, back at once (wait: until they are done; done:
 * called by the last part, any thread). NULL = one after the other on the
 * caller (the same bytes) */
#define EPD_CLAIM_BANDS_MAX 4
#define EPD_BAND_MIN_PX 300000L             /* below: one band (the device's pool dispatch ~1.4 ms) */
extern int epd_claim_bands;
typedef struct epd_band_ops {
    void (*run)(void *ctx, void (*fn)(void *arg, int i), void *arg, int n);
    void (*start)(void *ctx, void (*fn)(void *arg, int i), void *arg, int n);
    void (*wait)(void *ctx);
    void (*done)(void *ctx);
    void *ctx;
} epd_band_ops;
void epd_set_band_ops(epd_engine *e, const epd_band_ops *ops);
/* decisions effective at frame k + 2 (drive_overlap). on 1:
 * a bandable claim of >= EPD_BAND_MIN_PX pixels decided in epd_schedule at
 * engine frame k (epd_advance's count) is planned for frame k + 2 (the runs
 * ending at k or k + 1 counted as finished, the preempted runs' debit and the
 * heal runs' frames as at k + 2) and goes out as a job: its pass runs on a
 * shadow copy of its rect (the arrays are read, never written) on the band
 * ops' threads (start) while frames k, k + 1 are driven from the arrays as
 * they are. epd_schedule applies it (the shadow rect copied back, its runs
 * started) at the first call with fcount >= k + 2, or earlier once nothing
 * runs, if it is done; not done = held (job_late; never waits). Until then
 * epd_schedule starts nothing else; epd_advance's finish passes leave the
 * job's rect to the job (a run ending later than k + 1 while it is out is
 * finished over it at the apply: job_qfin). on 2: the same, but epd_schedule
 * waits for the job at its effective frame (the reference: epd-sched-bench
 * --effective 2). 0 (default): today's (a job out is applied first) */
void epd_set_overlap(epd_engine *e, int on);
/* the job's decision applied now (waited for): before the arrays are written
 * outside epd_schedule / epd_advance (assume, a state brought down from the
 * GPU, a stagger exit, levels loaded). 1 = there was one */
int epd_job_join(epd_engine *e);
int epd_job_out(const epd_engine *e);       /* a job decided, not yet applied */
/* a finished run's rows [y0, y1) (panel) on the packed words, run own ('s;
 * until then the arrays' finish_row runs, test/epd-claim-bench compares) */
void epd_finish_rows_packed(epd_engine *e, const epd_run *r, int own, int y0, int y1);
/* the query on the CPU's arrays (reference; tests) */
void epd_pend_query(const epd_engine *e, const epd_pend *p, uint64_t *mask, int *nonbw);
void epd_set_pixel_ops(epd_engine *e, const epd_pixel_ops *ops);
/* deferred claims: a claim's runs start at once over its whole
 * rect (every owner the plan has a slot for) and get their exact rects (the
 * empty ones freed) and its counters when its report is there: settled at the
 * next epd_schedule (looked at, not waited for), before any decision there
 * that reads runs or free slots, and before one of its runs could end in
 * epd_advance (waited for). Drive frames are the same (a run drives only the
 * pixels it owns); epd_frames_until_free and epd_idle may see the over-
 * estimate for a frame, nrun_max and the counters come a frame later. Not
 * deferred: lazy plans, matched ones without nonblock. Without pixel ops
 * (the CPU) the pass is done at once and only its report waits (tests).
 * on = 0 settles first. */
void epd_set_defer(epd_engine *e, int on);
/* the deferred claim's report taken in (wait 0: only if there; 1 = still
 * waiting); 0 when none is left. Callers: before the state is read or
 * replaced elsewhere (the GPU's state brought down, the ops changed) */
int epd_claim_settle(epd_engine *e, int wait);
/* stage 5: targets made on the GPU (the caller's handles, > 0;
 * epd_submit_gt). release: the engine is done with one (any thread, under
 * the caller's lock); fetch: its levels over r into out (r.w per row), 0 /
 * -1 (then white is assumed: a GPU that hung, its contents lost). Only the
 * CPU's pixel paths fetch (check mode, a CPU fallback), in the thread that
 * runs epd_schedule */
typedef struct epd_target_ops {
    void (*release)(void *ctx, int gt);
    int (*fetch)(void *ctx, int gt, epd_rect r, uint8_t *out);
    void *ctx;
} epd_target_ops;
void epd_set_target_ops(epd_engine *e, const epd_target_ops *ops);
/* every pending's GPU target fetched to the CPU now and its handle released
 * (before the GPU that reads them goes away) */
void epd_pend_targets_cpu(epd_engine *e);

int epd_init(epd_engine *e, const uint8_t *wbf, size_t n, int celsius);
void epd_free(epd_engine *e);
/* The frame cut (0..5): DU updates that START from now on
 * drop that many frames off the end and zero the last kept frame
 * */
void epd_set_cut(epd_engine *e, int cut);
/* Matched cuts (off by default): in a cut A2 (EPD_MODE_A2LEN, the fast steps
 * of a scroll) a pixel whose transition opposes its DC record gets the length
 * (2 .. the uncut A2) whose net drive cancels the record best, instead of the
 * step's; one that adds to it keeps the step's length. Each length is a run
 * of its own (an ordinary cut A2: renderers unchanged), so an update may take
 * up to EPD_LUT_CUTS more run slots; with none free, a pixel keeps the step's
 * length. For updates that start from now on. */
void epd_set_matched(epd_engine *e, int on);
/* Where matched cuts may lengthen (v2): only in cut steps of at least `from`
 * frames, and by at most `cap` frames over the step (0 = no limit). The fast
 * steps keep their length and leave their DC to the slower ones, which pay it
 * back a little each pass. epd_init: from 2, cap 0 (v1: every cut step, any
 * length). */
void epd_set_matched_rule(epd_engine *e, int from, int cap);
/* Non-blocking payback (off by default): the pixels a matched cut made longer
 * than their step run as payback runs that do not hold up the next step. A
 * partial A2 / A2_OR_DU update (a scroll step, cut or not) that only payback
 * pixels block starts at once on its other pixels; the payback ones stay
 * pending (kept) and start once, with the newest content, when free. A newer
 * such step takes in every kept or not yet started one it overlaps (its rect
 * grows to the union, the content from its own image), and supersedes one it
 * contains whatever their cut lengths. It still waits for the step before it
 * (any other run over its rect), so a step never tears. The matched runs then
 * end on their own frames (no padding), and epd_frames_until_free leaves
 * payback runs and kept updates out (the pacing). */
void epd_set_nonblock(epd_engine *e, int on);
/* on = 2: as 1, and a black/white pixel still paying back (an A2 payback run)
 * whose content changes is taken by the new A2 step at once instead of
 * waiting: the frames it will not get are credited back to the panel model
 * and the DC record, its level becomes where it was heading, and the step
 * drives what it still needs (with epd_set_model's optical lengths). A fast
 * fling with 1 left ~1/3 of the panel showing older content. */
/* The panel model: where each pixel really is, in drive frames between black
 * (0) and white (op_r: the uncut A2's net drive at this temperature). A full
 * waveform (GC16, DU, the uncut A2) puts a pixel at its target; a cut A2 moves
 * it by its net drive only, clamped at black and white — a settled black pixel
 * that a 1-frame cut "turns white" is still nearly black (level[] assumes every
 * drive reached its target; this record does not). optical = 1: matched cuts
 * (epd_set_matched) lengthen a changing black/white pixel to what it needs to
 * reach its target (never shorter than its step), instead of cancelling the DC
 * record. optical = 2: as 1, and where the DC record opposes the drive and
 * asks for more, the longer of the two (the extra frames only drive a pixel
 * already at its rail: they pay the DC back without changing what it shows;
 * 1 alone let the record drift). track = 1: keep the record without using it (tests, stats). The
 * record is kept while optical, track or healing is on; switched on, it starts
 * from the levels (every pixel settled). */
void epd_set_model(epd_engine *e, int optical, int track);
/* Healing (needs epd_set_nonblock and acts through the model): in a partial
 * A2 / A2_OR_DU / DU update that is uncut or cut to >= `from` frames, every
 * black/white pixel of its rect that is idle and does not change, but sits
 * >= `min` frames short of its level, gets 1, 2, 4 or 8 drive frames toward
 * that level (the A2's own drive phase; no more than the step drives: len - 1
 * for a cut step, the rest the next time) as a payback run: it holds up no step,
 * a later update over it starts around it. Pixels only pay back when they
 * change otherwise, so the leftovers of a fast scroll stayed while it slowed
 * down (user). min 0 = off. */
void epd_set_heal(epd_engine *e, int min, int from);
/* heal runs of up to n drive frames (1..8) whatever the step's length: a fast
 * step's len - 1 (A2/2: 1) was below min, so the fastest steps never healed.
 * They are payback runs and preempted when their pixels change, so a longer
 * one holds up nothing. 0 (default) = len - 1 */
void epd_set_heal_len(epd_engine *e, int n);
/* Full waveforms from where the pixel is (needs the model): a GC16 / GL16 / DU
 * / DU4 update reads each pixel's "from" level as the level it was last driven
 * toward, but a cut or preempted A2 leaves a black/white pixel short of it
 * (A2/2 "to black" moves a white pixel 1 of op_r frames). The full waveform
 * then swings the whole way back (+21 against the -1 it arrived with: the
 * DC map, scrolled text) or, in a partial update whose target is
 * that level, skips a pixel that is still half white. On: an idle black/white
 * pixel whose target is the other rail and whose model is nearer that rail is
 * taken as there (its level snapped before the claim: a partial update leaves
 * it alone, a full one drives it rail to itself), and (with epd_set_nonblock)
 * a heal run of up to op_r / 2 drive frames gives it, and every idle
 * black/white pixel of the rect >= 1 frame short, what it is still short of
 * (a partial GC16 left those short: 0.6 frames on average after each cleanup
 * in the test). 0 = off. */
void epd_set_snap(epd_engine *e, int on);
/* the model's view of black/white pixels: mean shortfall (frames; a pixel in
 * flight also counts its run's frames still to come) and how many are >=
 * `limit` frames short of their level (0, 0 while it is not kept) */
void epd_op_stats(const epd_engine *e, int limit, double *mean, long *over);
/* Forget what the panel shows: assume every pixel at `level5` (0..30). */
void epd_assume(epd_engine *e, int level5);
/* Queue an update of rect r. An earlier update still pending (not started)
 * of the same mode and fullness that lies entirely inside r, with no pending
 * update in between overlapping it, is dropped: r overwrites all of it
 * (fast scrolling queues no stale frames). Queue (clipped to the panel) to the 8-bit greys in
 * `grey` (row stride `stride` bytes, addressed in panel coordinates, i.e.
 * grey[y * stride + x] for the rect's pixels). full:
 * unchanged pixels are driven too (the flash); partial updates skip them, and
 * a partial update that changes nothing never runs. full = 2: a flash of
 * only the pixels that need one: changed, or driven by anything but an exact
 * GC16/GL16 since their last one (e->stale); the rest is left as it is.
 * Returns its id, 0 if empty or the queue is full. */
uint32_t epd_submit(epd_engine *e, epd_rect r, int mode, int full, const uint8_t *grey, int stride);
/* epd_submit in two halves, so a caller can make the target (the copy of the
 * rect's levels: the expensive part, ~2.6 Mpx for a scroll) outside its lock:
 * epd_target_make clips *r and returns the malloc'd levels (NULL: empty or no
 * memory); epd_submit_target queues them (taking ownership; grey/stride are
 * only read to merge pen pieces). Same result as epd_submit. */
uint8_t *epd_target_make(epd_rect *r, const uint8_t *grey, int stride);
/* the same, its rows in `parts` parts through run(ctx, fn, arg, n) (fn(arg, k)
 * for every k < n, back when all ran; epd_stream_upd_run's shape) */
uint8_t *epd_target_make_par(epd_rect *r, const uint8_t *grey, int stride,
                             void (*run)(void *, void (*)(void *, int), void *, int), void *ctx, int parts);
/* the targets' buffers (epd_target_make, the merges, pend_tg): blocks of
 * 128 KB and more are pooled (epd_target_pool of them, 0 = off; default 4)
 * instead of mapped and unmapped per update (musl). epd_target_free takes any
 * malloc block; a pooled one may also go to free. Stats: since the last call */
extern int epd_target_pool;
uint8_t *epd_target_alloc(size_t n);
void epd_target_free(void *p);
void epd_target_pool_stats(unsigned long *hit, unsigned long *miss, int *kept);
uint32_t epd_submit_target(epd_engine *e, epd_rect r, int mode, int full, uint8_t *t, const uint8_t *grey, int stride);
/* the same with a GPU target: handle gt (epd_target_ops) holds the levels of
 * grey over r, made by the caller; the engine owns it from here (released
 * when the update is gone, also when it is dropped at once: 0). A merge or
 * absorb makes the union on the CPU from grey (as epd_submit_target) and
 * releases the handles it replaces */
uint32_t epd_submit_gt(epd_engine *e, epd_rect r, int mode, int full, int gt, const uint8_t *grey, int stride);
/* Start what can start. Returns the number of running updates (0 = nothing
 * to send). */
int epd_schedule(epd_engine *e);
/* Render the next drive frame of all running updates into `words` (a buffer
 * that currently holds the control template, except for data bits in `*dirty`
 * rows left by its previous use, which are cleared first). `dirty` is updated
 * to the rows written now ([y0, y1) in wire lines; empty = y0 >= y1). */
void epd_render(epd_engine *e, uint32_t *words, int dirty[2]);
/* epd_render in pieces, for several threads: begin (clear + dirty), then
 * epd_render_rows over panel row ranges that together cover [0, EPD_H) */
void epd_render_begin(epd_engine *e, uint32_t *words, int dirty[2]);
void epd_render_rows(epd_engine *e, uint32_t *words, int y0, int y1);
/* The frame rendered last was sent: advance, finish updates (their pixels'
 * levels become the targets). Returns ids of finished updates via cb. */
void epd_advance(epd_engine *e, void (*done)(uint32_t id, void *arg), void *arg);
int epd_idle(const epd_engine *e);          /* nothing running or pending */
/* drive frames until no update over r is in flight or queued (0 = free now):
 * an update submitted over r now waits about this long before it starts.
 * Pending updates count with an estimate of their length. */
int epd_frames_until_free(const epd_engine *e, epd_rect r);
/* a pending update's length once it starts (epd_frames_until_free's estimate)
 * for a mode (EPD_MODE_A2LEN: the length) and full: the cached table, else the
 * shortest it can get (10) — epd-drm publishes it for the lockless free_ms */
int epd_pend_frames_est(const epd_engine *e, int mode, int full);
/* the DC record: largest |net| frames of any pixel, and how many pixels are
 * beyond `limit` */
void epd_dc_stats(const epd_engine *e, int limit, int *max_abs, long *over);

#endif
