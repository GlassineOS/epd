/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-drm — streams epd-engine drive frames to the Note Air 1 panel on the mainline
 * kernel (panel-onyx-na1-link: DSI command mode, one DRM page flip = one wire
 * frame, TC358762 + MAX17135 sequenced by the driver's enable/disable).
 *
 * A thread owns the engine and the DRM device:
 * idle -> an update is submitted -> link up (SETCRTC on a neutral all-zero
 * frame = an idle bus; the driver raises the rails, checked via hv_on)
 * -> one page flip per drive frame, frame k+1 rendered while k is on the
 * wire -> nothing left for `linger_ms` -> neutral frame, link down (rails
 * down). The driver's hv_max_ms backstop is raised to 30 s; a stream that
 * runs longer is cut into link cycles (rails down/up between two frames), and
 * if the rails ever drop mid-stream the link is brought up again.
 * Frames are rendered into cached memory; only changed wire lines are copied
 * into the (write-combined) scanout buffers.
 */
#ifndef EPD_DRM_H
#define EPD_DRM_H

#include <stdint.h>
#include "epd-engine.h"
#include "epd-gtarget.h"

typedef struct epd_stream epd_stream;

typedef struct {
    uint64_t frames, late, links, updates, rail_drops, dead_links, slow_frames, slow_reads;   /* slow_reads: slow but read late (we were busy) */
    uint64_t warm_wakes;                    /* links that came up from warm idle (rails only), of links */
    double render_ms_max, render_ms_sum;    /* render + copy, per frame */
    double copy_ms_max, copy_ms_sum;        /* the copy into the scanout buffer alone */
    uint64_t words_copied;
    uint64_t gpu_frames, gpu_checked;       /* drive frames the GPU rendered / of those, checked equal to the CPU's */
    uint64_t gpu_late;                      /* GPU frames that took > GPU_LATE_MS (and finished) */
    uint64_t gpu_resets;                    /* GPU recovers that dropped one of our jobs (a full flash each) */
    uint64_t gpu_revives;                   /* abandoned (hung) GPUs back after their frame finished (gpu_revive) */
    double lag_ms_max;                      /* flip call after the previous flip event, back-to-back frames */
    double evt_age_ms_sum;                  /* flip event: arrival - its timestamp */
    double evt_age_ms_max;
    double gpu_ms_sum, gpu_ms_max;          /* GPU frames: state upload + render + finish */
    double check_ms_sum, check_ms_max;      /* gpu=check: a sampled frame's readback + compare */
    uint64_t state_n, state_px;             /* GPU per-pixel state packs (run starts/ends) and their pixels */
    double state_ms_sum, state_ms_max;      /* their CPU time (pack + upload call) */
    uint64_t evt_n;
    /* jit: frames that came to the JIT wait, by size (epd-jit.h: small,
     * mid, large, stagger) and the lead they used */
    uint64_t jit_n[4];
    double jit_lead_ms_sum[4];
    /* the step timeline: the hand-off (epd_stream_submit) split
     * into the target copy (outside the lock), the wait for the lock and the
     * engine's epd_submit_target under it; the stream thread's epd_schedule
     * and epd_advance (+ GPU finishing) under the lock, per rendered frame */
    uint64_t sub_n, sched_n;
    double sub_make_ms_sum, sub_make_ms_max, sub_lock_ms_sum, sub_lock_ms_max, sub_tgt_ms_sum, sub_tgt_ms_max;
    double sched_ms_sum, sched_ms_max, adv_ms_sum, adv_ms_max;
    /* the engine state on the GPU (epd_stream_gpe): claims done there (lazy:
     * slots from a probe), their time incl. the readback; blocked tests;
     * check-mode whole-state compares; syncs down to the CPU's arrays */
    uint64_t gpe_claims, gpe_lazy, gpe_queries, gpe_checked, gpe_syncs;
    double gpe_ms_sum, gpe_ms_max, gpe_chk_ms_sum, gpe_chk_ms_max, gpe_sync_ms_sum, gpe_sync_ms_max;
    /* GLASSINE_GPE_ASYNC=1 (deferred claims, epd_set_defer): claims deferred (their
     * gpe_ms: the submit only), reports collected, the collects' time (the
     * wait for the GPU incl.), settles inside an epd_schedule pass */
    uint64_t gpe_deferred, gpe_collects, gpe_settle_waits;
    double gpe_col_ms_sum, gpe_col_ms_max;
    uint64_t gt_subs;                       /* stage 5: submits with a GPU target (epd_stream_submit_gt) */
    /* staggered cut steps (epd_stream_stagger): starts and ends,
     * their time (enter: the state up; exit: read back into the engine), the
     * cut steps taken, the frames (one GPU pass + finish each) */
    uint64_t stag_chk_runs, stag_chk_frames, stag_chk_frames_bad, stag_chk_px_bad, stag_chk_state_bad;   /* stagger=check */
    uint64_t stag_chk_self, stag_chk_self_bad, stag_chk_drift;    /* its self-consistency (STG_SELF_EVERY) */
    uint64_t stag_chk_loads, stag_chk_loads_bad;                  /* its loads that (did not) land */
    uint64_t stag_entries, stag_exits, stag_subs, stag_frames, stag_held, stag_flushed, stag_end_quiet, stag_end_other;
    uint64_t stag_dc_px, stag_dc_in, stag_dc_out;   /* the runs' pixels: sum of |DC record| before (the engine's) and after */
    /* stagger_state=lanes : rects armed, frames from the
     * evicts to their acks (summed over the runs), evicts taken over without
     * all acks, the runs' dma-buf sync time (ms, summed) */
    uint64_t stag_warms, stag_warm_px, stag_first;
    uint64_t stag_lane_arms, stag_lane_ack_frames, stag_lane_ack_late, stag_lane_dskip, stag_lane_finarmed, stag_lane_par;
    /* ... run ends' evicts / takeovers on the pool, the evicts' time (ms) */
    uint64_t stag_lane_xpar, stag_evicts;
    double stag_lane_sync_ms_sum, stag_evict_ms_sum, stag_evict_ms_max;
    double stag_warm_ms_sum, stag_warm_ms_max;
    double stag_enter_ms_sum, stag_enter_ms_max, stag_exit_ms_sum, stag_exit_ms_max, stag_ms_sum, stag_ms_max, stag_sub_ms_sum, stag_sub_ms_max,
           stag_enter_cpu_ms_sum, stag_exit_read_ms_sum;
    /* the engine's (epd-engine.h) */
    uint64_t merged, superseded, wait_order, wait_pixel, wait_slot;
    int nrun_max, npend_max;
    /* the hand-off inbox (epd_stream_inbox): messages the drive
     * thread applied, the most in one drain, appends that waited for a slot
     * (the ring full: the drive thread stuck in link work), GPU-target submits
     * taken on a stale flag and turned into CPU copies */
    uint64_t inbox_n, inbox_max, inbox_full, gt_fallback;
} epd_stats;

/* hv: require the rails (1; the driver's hv parameter is set) or run without
 * them (0; dry run, nothing on the panel changes). */
epd_stream *epd_stream_open(const char *wbf_path, int celsius, int hv);
uint32_t epd_stream_submit(epd_stream *s, epd_rect r, int mode, int full, const uint8_t *grey, int stride);
/* ms until an update submitted over r now could start (what runs and waits
 * over r finishes; 0 = at once): glassine-epdd-desktop reads a motion run's next frame
 * just in time for it instead of every run_ms */
/* stage 5: update targets made on the GPU (epd-gtarget.h). The
 * producer's layers, registered with the gpe in the stream thread once it
 * runs (NULL: none). Then epd_stream_submit_gt takes a handle instead of the
 * grey copy: the update's id (EPD_GT_DROPPED: taken, but nothing to do), or
 * 0 = refused (the gpe is off or the layers did not import yet): the handle
 * is still the caller's, who submits with epd_stream_submit instead. Before
 * the gpe or the GPU goes, pending GPU targets become CPU copies. */
#define EPD_GT_DROPPED 0xffffffffu
void epd_stream_set_gt(epd_stream *s, const struct epd_gt_provider *pv);
uint32_t epd_stream_submit_gt(epd_stream *s, epd_rect r, int mode, int full, int gt, const uint8_t *grey, int stride);
/* the last epd_stream_submit's parts in ms: target copy, lock wait, epd_submit_target */
void epd_stream_last_submit(epd_stream *s, double *make_ms, double *lock_ms, double *tgt_ms);
int epd_stream_free_ms(epd_stream *s, epd_rect r);
/* the engine's DC record (epd_dc_stats): largest |net drive frames|, pixels beyond limit */
void epd_stream_dc(epd_stream *s, int limit, int *max_abs, long *over);
/* the DC record (panel order, EPD_W x EPD_H) and the levels, copied under the lock */
/* op (may be NULL): the panel model, drive frames from black 0..*r (op_r) */
void epd_stream_dc_map(epd_stream *s, int16_t *dc, uint8_t *level, uint8_t *op, int *r);
/* matched cuts (epd_set_matched) on/off; pixels given a matched length so far */
void epd_stream_set_matched(epd_stream *s, int on);
void epd_stream_set_matched_rule(epd_stream *s, int from, int cap);    /* epd_set_matched_rule */
void epd_stream_set_nonblock(epd_stream *s, int on);                   /* epd_set_nonblock */
unsigned long long epd_stream_split_starts(epd_stream *s);
unsigned long long epd_stream_preempted(epd_stream *s); /* payback pixels taken by a newer step (preempt) */
unsigned long long epd_stream_flash_skipped(epd_stream *s); /* pixels flashes left alone (not stale, unchanged) */
unsigned long long epd_stream_absorbed(epd_stream *s);  /* waiting rests taken into a newer step (nonblock) */
void epd_stream_set_model(epd_stream *s, int optical, int track);   /* epd_set_model */
void epd_stream_set_heal(epd_stream *s, int min, int from);         /* epd_set_heal */
void epd_stream_set_heal_len(epd_stream *s, int n);                 /* epd_set_heal_len */
unsigned long long epd_stream_healed(epd_stream *s);                /* pixels healed so far */
void epd_stream_set_snap(epd_stream *s, int on);                    /* epd_set_snap */
unsigned long long epd_stream_snapped(epd_stream *s);               /* levels snapped so far */
void epd_stream_shortfall(epd_stream *s, int limit, double *mean, long *over);  /* epd_op_stats */
unsigned long long epd_stream_matched_px(epd_stream *s);
void epd_stream_assume(epd_stream *s, int level5);    /* only while idle */
void epd_stream_set_temp(epd_stream *s, int celsius);
void epd_stream_set_cut(epd_stream *s, int cut);
/* keep the link and rails up for ms (<= 5000) from now, bringing them up at
 * once if down: the first frame of the next update then goes out without the
 * ~75 ms bridge wake + rail power-good (call it on pen/touch input) */
void epd_stream_park(epd_stream *s, int on);   /* 1: sleep/pause: link fully down now, no warm idle */
void epd_stream_keepalive(epd_stream *s, int ms);
void epd_stream_wait_idle(epd_stream *s);
int epd_stream_save_levels(epd_stream *s, const char *path);   /* waits for idle */
int epd_stream_load_levels(epd_stream *s, const char *path);
void epd_stream_stats(epd_stream *s, epd_stats *out);
void epd_stream_stats_reset(epd_stream *s);           /* all counters and peaks to 0 */
/* CPU seconds the stream's threads used since they started: the drive thread,
 * the render workers together (where epdd's CPU goes) */
void epd_stream_cpu(epd_stream *s, double *drive_s, double *workers_s);
void epd_stream_close(epd_stream *s);                 /* waits for idle, link down */
/* raw drive frames (glassine-epd-bench): once the engine is idle, every frame is fn's
 * instead of the engine's until fn returns 0. fn gets the frame with the
 * last raw frame's drive bits cleared (the template's control bits kept) and
 * ORs its 2-bit phases (1 toward black, 2 toward white) into the low 16 bits
 * of the words it drives, widening dirty (wire lines [0], [1]) to cover them;
 * it runs in the stream thread, the lock not held. Its last frame (the one it
 * returns 0 for) must drive nothing: the panel may see the last frame again.
 * The engine's updates wait meanwhile. CPU frames only: -1 with the GPU on
 * (epd_stream_gpu) or a raw run already set; 0 set. epd_stream_wait_idle
 * returns once it ended. */
int epd_stream_raw(epd_stream *s, int (*fn)(void *arg, uint32_t *words, int dirty[2]), void *arg);
/* 1: the GPU hung and was abandoned (epd-drm.c gpu_abandon): until it comes
 * back (gpu_revive: its timed-out frame finished) or a reboot, the process
 * must not exec or exit (freeing the GPU's buffers would hang) */
void epd_stream_set_hv(epd_stream *s, int hv);    /* 0: dry run; from the next link up */
int epd_stream_gpu_dead(epd_stream *s);
/* 1 (once): the GPU hung while the engine state was on it; the levels are
 * assumed white now: the caller should flash the whole screen */
int epd_stream_levels_lost(epd_stream *s);
/* epd_stream_gpu: before epd_stream_open: 1 drive frames on the GPU (-DEPD_GPU builds), 2 + checked against the CPU,
                                   3 only the GPU's scanout buffers, the CPU renders (diagnosis) */
/* epd_stream_gpe: before epd_stream_open, with epd_stream_gpu 1: the engine's per-pixel state
                                   on the GPU (1), checked against the CPU's (2); see epd-drm.c */
/* staggered cut steps (see epd-drm.c; -DEPD_GPU, GPU drive frames,
 * gpe off): 1 on (default 0); ctl 1 the controller, pay its payback threshold */
/* epd_stream_stag_check: stagger=check: see epd-drm.c */
/* epd_stream_stag_direct: stagger_direct=: see epd-drm.c */
/* epd_stream_stag_warm: stagger_warm=on|off: see epd-drm.c (stag_warm) */
/* epd_stream_stag_entry: stagger_entry=first|engine: see epd-drm.c (stag_take) */
/* epd_stream_stag_stripe: stagger_stripe= */
/* epd_stream_stag_blocks: stagger_stripe=blocks */
/* epd_stream_stag_cellw, epd_stream_stag_cellh: stagger_stripe=cells[W[xH]] */
/* epd_stream_stag_cellord: stagger_stripe=wbayer: EPD_STAG_CORD_WBAYER */
/* epd_stream_stag_htop: hammer_top= */
/* epd_stream_gpu_hang_test: gpu_hang_test=1 (a test of the GPU's revive) */
/* epd_stream_stag_a21: stagger_a21= */
/* epd_stream_jit: 1: render each frame just before it is due (default) */
/* epd_stream_jit_size: 1: its lead by the next frame's size (epd-jit.h, default) / 0: one decaying max */
/* 1 a submit, a GPU-target submit and
 * the engine knobs (cut, temp, matched, nonblock, model, heal, snap) are
 * messages the drive thread applies in order at the top of its loop; the
 * submitter never waits for the stream lock (which the drive thread holds
 * through epd_schedule, 10-16 ms per hand-off before). Then epd_stream_submit
 * returns the stream's own id for the update (nonzero; the engine's ids stay
 * inside), epd_stream_submit_gt takes or refuses by the drive thread's last
 * published state, and epd_stream_free_ms reads a published view instead of
 * the engine. 0: the old direct path (live: glassine-epdd-desktop handoff=inbox|lock) */
#ifdef EPD_HOST
/* the host harness (-DEPD_HOST, test/epd-stream-test.c): every flipped scanout
 * buffer, every submit message applied (kind: 1 CPU target, 2 GPU target), and
 * a sleep inside epd_schedule's locked stretch (a long claim) */
extern void (*epd_stream_host_flip)(uint32_t fb, const uint32_t *words, int lines);
extern void (*epd_stream_host_applied)(uint32_t id, int kind, epd_rect r);
extern int epd_stream_host_slow_sched_us;
#endif
/* knobs; defaults = the old behaviour (see epd-drm.c) */
/* epd_stream_nwork: 1..4 threads per frame's render/copy (default 4) */
/* epd_stream_rt_umin: uclamp.min of those threads, -1 = kernel RT default */
/* epd_stream_link_log: 1: log every link transition with its time */
/* epd_stream_link_warm_ms: warm idle after the linger (rails down, link up), ms; 0 off */
/* epd_stream_qos: -1 global cpu_dma_latency 0; -2 none; N: stream CPUs only, N us */
void epd_stream_set_rt_umin(epd_stream *s, int v);     /* applies to the running threads */
/* drive_overlap 1 = the engine's
 * epd_set_overlap(1): big claims go out as a job on the workers, effective
 * two frames on, the stream thread never waits for one (default 0; read at
 * the stream thread's loop top). With it or the engine's epd_claim_bands > 1
 * the stream thread runs at FIFO 51. epd_stream_s4: the counters as a line */
void epd_stream_s4(epd_stream *s, char *b, size_t n);

/* the update job.
 * epd_stream_upd_run: fn(arg, k) for every k < n, the caller taking parts
 * too, back when all ran. Parts are taken one at a time from a counter by the
 * caller and by idle workers; a worker serves a band part or a render/copy
 * part first (checked between update parts), so a claim or a frame waits at
 * most one update part. The stream thread never waits for an update job. A
 * second caller while a job runs (or n <= 1): every part on the caller.
 * epd_stream_set_upd_runner: the runner epd_stream_submit's target copy (the
 * levels) uses for rects of >= EPD_UPD_LEVELS_PX in EPD_UPD_LEVELS_PARTS row
 * parts (NULL: one pass, today). epd_stream_upd_fifo: 1 while the daemon runs
 * update jobs at SCHED_FIFO (the stream thread then at 51, above them) */
void epd_stream_upd_run(epd_stream *s, void (*fn)(void *, int), void *arg, int n);
typedef void (*epd_upd_runner)(void *ctx, void (*fn)(void *, int), void *arg, int n);
void epd_stream_set_upd_runner(epd_stream *s, epd_upd_runner run, void *ctx);
#define EPD_UPD_LEVELS_PX (256 * 1024)
#define EPD_UPD_LEVELS_PARTS 16
/* jobs run, parts taken by workers / by the caller, jobs refused (busy: serial) */
void epd_stream_upd_stats(epd_stream *s, unsigned long long *jobs, unsigned long long *wparts,
                          unsigned long long *cparts, unsigned long long *serial);
/* worker_rt: the frame workers' policy, 0 = SCHED_FIFO 50 (today), 1 =
 * SCHED_OTHER nice EPD_WORKER_NICE (rt_umin then not applied: a CFS thread
 * keeps the CFS default clamp). Live: a worker re-applies it at its next job */
#define EPD_WORKER_NICE (-10)

/* the drive thread's per-frame timing, default off
 * (off: a branch on the drive thread's own flag per timed spot). on: 1 / 0 /
 * -1 unchanged; every_s: the stderr summary's period (0 unchanged; default
 * 5); csv: NULL unchanged, "" or "off" closed, else a file one CSV row per
 * loop iteration goes to (opened by the drive thread, truncated) plus
 * <csv>.freq (the CPU frequency sampler's rows). A message through the inbox:
 * applied at the drive thread's next drain. While on, a non-RT helper thread
 * samples every cpufreq policy's scaling_cur_freq at 20 Hz (no sysfs read on
 * the FIFO thread). Field list and record types: epd-drm.c, "drive_prof". */
void epd_stream_prof(epd_stream *s, int on, int every_s, const char *csv);
/* the summary now, and the newest records (up to 1024 loop iterations + 256
 * end-to-end update records) as CSV into path (NULL: /run/glassine-epdd-desktop-frames.csv);
 * glassine-epdd-desktop's "stats" */
void epd_stream_prof_dump(epd_stream *s, const char *path);
/* end-to-end stamps (CLOCK_MONOTONIC s, 0 = not known) the next submits from
 * this thread carry: [0] the damage arrived, [1] its producer's fence seen,
 * [2] ingested (in the main thread's frame), [3] scanned, [4] committed.
 * Thread-local; NULL clears. Used only while drive_prof is on */
#define EPD_ORG_N 5
void epd_stream_set_origin(const double *org);
/* env (harness): GLASSINE_DRIVE_PROF=1 or =<csv path> turns it on at open */

#endif
