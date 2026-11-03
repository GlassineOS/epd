/* epd-drm — see epd-drm.h */
#define _GNU_SOURCE             /* CPU affinity */
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>
#ifdef EPD_HOST
#include <linux/udmabuf.h>
#endif
#include <libdrm/drm.h>
#include <libdrm/drm_mode.h>
#include <libdrm/drm_fourcc.h>
#include "epd-drm.h"
#include "epd-jit.h"
#include "epd-prof.h"
#include "epd-profpath.h"
#ifdef EPD_GPU
#include "epd-gpu.h"
#include "epd-bp4.h"
#include <linux/dma-buf.h>
#include "epd-bluenoise.h"
#endif

#define PARAMS "/sys/module/panel_onyx_na1_link/parameters/"
#define NBUF 3
#define LINGER_MS 400
#define CYCLE_MS 25000          /* < the 30 s hv_max_ms we set (kernels without hv_kick) */
#define KICK_MS 5000            /* hv_kick at most this often (<< hv_max_ms) */
#define KEEPALIVE_MAX_MS 5000   /* rails up without drive: bounded */
#define NREND 4                 /* render/copy threads incl. the stream thread (the most) */
#define NWORK 5                 /* threads: the stream thread + its workers (4 band parts off the stream thread) */

/* knobs (runtime, see epd-drm.h). Defaults: rt_umin = the kernel's RT
 * default and qos global. rt_umin 0 + qos cpu:0 (the defaults for a while)
 * saved power and kept the engine's frames on time in the scroll/pen
 * benchmark, but in real use the device felt far slower (an A/B by feel: "night and day";
 * late frames 2.4 % vs 0.2 %): X, the apps and the grab/composition then run on
 * CPUs that deep-idle between events and ramp from low clocks. qos=cpu:600 / nwork=2
 * added late frames. */
int epd_stream_nwork = NREND;   /* threads a frame's render/copy is split over, 1..NREND */
/* drive_overlap (epd-drm.h); claim_bands is the
 * engine's epd_claim_bands. Either on: the stream thread at FIFO 51 (above
 * the workers' 50: a band never delays a flip on its CPU) */
int epd_stream_drive_overlap;
/* update jobs at SCHED_FIFO (the stream thread at 51 then); the frame
 * workers' policy (epd-drm.h) */
int epd_stream_upd_fifo;
int epd_stream_worker_rt;
int epd_stream_rt_umin = -1;    /* uclamp.min of the stream + worker threads; -1: the
                                   kernel default for RT (sched_util_clamp_min_rt_default) */
/* warm idle (step 1): after the linger the rails go
 * down but the DRM pipe and the TC358762 bridge stay up this long (ms), so the
 * next update or pen hover only raises the rails (driver param "rails", kernel
 * 0051) instead of a whole modeset + bridge init; then the full link down.
 * 0 = the old behaviour (full down after the linger). Parked (sleep / pause /
 * close): always the full down, at once. */
int epd_stream_link_warm_ms = 30000;
int epd_stream_link_log;        /* link_log=on: every link transition with its time (also -v) */
#define LINK_LOG (epd_stream_verbose || epd_stream_link_log)
#define LINK_TS() fprintf(stderr, "%.3f ", now())    /* CLOCK_MONOTONIC s (glassine-epdd-desktop's "mark" clock /1000) */
int epd_stream_qos = -1;        /* CPU wakeup latency hold while the link is up: -1 global
                                   cpu_dma_latency 0 (every CPU WFI only); -2 none; N >= 0:
                                   only the stream's CPUs (pin_fast_cpus), resume latency
                                   N us (0 = none tolerated) */

/* drive frames on the GPU (epd-gpu.c; builds with -DEPD_GPU): 0 off, 1 on, 2 check
 * (the CPU renders every frame too; every GPU_CHECK_EVERY-th GPU frame is read
 * back and compared before its flip; a difference sends the CPU frame and turns
 * the GPU off), 3 pad: only the GPU's
 * scanout buffers (EPD_GPU_EXTRA_LINES more lines, a dma-buf), the CPU renders.
 * the first check read the whole frame back through the write-combined
 * scanout mapping, ~450 ms a frame: the link held each frame that long and the
 * panel came out INVERTED (over-driven). Dry runs also crashed the SoC (900E):
 * a DPU IRQ read unclocked registers while the link cycled (kernel patch 0012).
 * The rows keep the display's own pitch; the GPU sees the buffer flat (epd-gpu.h) */
int epd_stream_gpu;
#define GPU_CHECK_EVERY 16       /* check mode: every 16th GPU frame is read back (~7 ms)
                                 * and compared; the CPU still renders every frame */
#define STR_MS_(x) #x " ms"
#define STR_MS(x) STR_MS_(x)
#define GPU_LATE_MS 100         /* a frame this slow is late (counted: gpu late) */
#define GPU_SLOW_RUN 8          /* this many late frames in a row: GPU drive frames off (CPU) */
#define GPU_HUNG_MS 2000        /* a frame not done after this: the GPU is hung -> abandoned.
                                 * Was GPU_LATE_MS until under a GLES sway the one
                                 * ring is shared (sway, the ingest, apps), a drive frame waited
                                 * > 100 ms behind them, no kernel hang, and the GPU was
                                 * abandoned until a reboot (CPU drive frames: ~28 ms CPU each).
                                 * A late frame only flips later (the panel pauses, nothing is
                                 * driven twice); a real hang never signals, so it is
                                 * still caught */
#define GPU_DEVFREQ "/sys/class/devfreq/5000000.gpu/"

/* the engine's per-pixel state on the GPU (epd-gpu.c gpe; only
 * with GPU drive frames): 0 off, 1 on — claims, finishes and blocked tests are
 * GPU passes, the drive frames read that state (no state packs); the CPU's
 * arrays are synced down only for the readers (DC map, shortfall, saved
 * levels) and before the CPU takes over — 2 check: the CPU does every pixel
 * pass too, each claim's report and blocked test compared, every
 * GPE_CHECK_EVERY-th claim the whole state read back; a difference turns the
 * gpe off (the CPU's state goes on). gpu=check makes it 2 (the CPU renders
 * from its own arrays then) */
int epd_stream_gpe;
#define GPE_CHECK_EVERY 64
#define GPE_DOWN 1              /* gpe_req: the CPU's arrays from the GPU's state */
#define GPE_UP 2                /* the GPU's state from the CPU's arrays */

/* staggered cut steps (epd-stagger.h; GPU drive frames, gpe off):
 * 0 off, 1 on. A motion run's cut A2 steps (partial EPD_MODE_A2LEN, len >= 2)
 * then go to the stagger instead of the engine: every pixel steps on its own
 * blue-noise phase group (n = len - 1 drive frames, G = len groups: each frame
 * 1/G of the panel starts a step, so the panel takes a new target every
 * frame instead of every len frames), one GPU pass a frame. It starts once a
 * cut step reached the engine and the engine is idle (its state moves over),
 * and ends (its state back to the engine) when anything else is submitted
 * (that waits in the engine's queue: no new steps for the longest step,
 * then the engine goes on) or when the targets stopped changing. ctl: the
 * controller (steps only as long as the panel model needs to reach the
 * rail; pay > 0: and DC payback steps of the record >= pay) */
int epd_stream_stagger;
/* stagger=check (diagnosis, the inverted URL bar): the CPU
 * reference steps a copy of the stagger's state in lockstep with the GPU
 * (its inputs taken where the GPU's are uploaded); every STG_CHECK_EVERY-th
 * frame the GPU's drive frame is read back and compared over the loaded
 * rows, and at each hand-back the state read back. Differences are logged
 * (up to STG_CHECK_LOG lines a run), nothing is changed. Costs a CPU
 * render of the loaded rows per frame (frames come late: a pause) */
int epd_stream_stag_check;
/* stagger_direct=0|1 (-1: GLASSINE_STAG_DIRECT / default 1), live: whether stagger
 * frames image-store into the scanout target (a second texture on the same
 * dma-buf EGLImage) or go through its framebuffer like the engine's frames.
 * the suspect for the stagger's model != panel (the two views
 * alias one buffer behind Mesa's back: one view's caches can hold stale lines) */
int epd_stream_stag_direct = -1;
#define STG_CHECK_EVERY 4
#define STG_CHECK_LOG 12
/* and every STG_SELF_EVERY-th frame (a compared one) the self-consistency:
 * the GPU's whole state read back before the frame, the CPU reference steps
 * ONE frame from it (the same targets and parameters), against the GPU's
 * frame -- the GPU stepping wrong from its own state (inputs on the GPU that
 * differ, a pass that differs in the stream) vs a state that drifted from the
 * reference earlier (then: when, from the state compare). ~75 ms a time */
#define STG_SELF_EVERY 16
int epd_stream_stag_ctl = 3;          /* stagger_ctl=: 3 = a pulse instead of the idle frame (the default) */
int epd_stream_stag_k = 8;          /* ctl 3: the most pulses without an idle frame */
int epd_stream_stag_sidle = 0;      /* ctl 4: stagger_idle=shared (1) | pixel (0) */
int epd_stream_stag_pay;
/* payrule 1: the engine's pay-on-arrival (epd-stagger.h; default off), its
 * cap (-1: the engine's matched_cap; 0: none = the most the levels hold) */
int epd_stream_stag_payrule, epd_stream_stag_cap = -1;
int epd_stream_stag_stripe = 0;          /* stagger_stripe=: 0 the blue-noise groups (the default), 64 px column stripes (the earlier default) */
int epd_stream_stag_blocks;              /* stagger_stripe=blocks: the stripes as scattered 64 x 64 blocks (stripe 64; 2, default off) */
int epd_stream_stag_cellw, epd_stream_stag_cellh;   /* stagger_stripe=cells[W[xH]]: the stripes as blue-noise ranked cells (stripe 64; , default off: 0) */
int epd_stream_stag_cellord;            /* stagger_stripe=wbayer: 8 x 1 cells in Bayer order (default off: 0 = blue-noise order) */
/* hammer_top=: the rail hammer, an optical reset (epd_stag_par.htop): an
 * arrival at black / white gets 2 frames past its need into the rail. On by
 * default; 0 = off (glassine-settings: Rail hammer) */
int epd_stream_stag_htop = 2;
int epd_stream_gpu_hang_test;  /* gpu_hang_test=1: the next GPU frame not waited for = a hang (gpu_revive's test) */
/* stagger_a21=on (ctl 3, default off): A2/1 -- a run's 1-frame steps (A2/2)
 * at G 1 instead of G = n + 1 = 2: every pixel decides on every panel frame
 * (a pulse or an idle; an idle after K pulses), so each pixel follows each
 * new compositor frame at 85 Hz instead of 42.5 */
int epd_stream_stag_a21;
/* the stagger stays on this long after the targets last changed (ms): an app
 * draws every few panel frames, a scroll's frames come 30-100 ms apart */
int epd_stream_stag_linger = 250;
enum { STG_OFF, STG_ON, STG_DRAIN };
#define STG_HOLD 8                  /* held rects (more: they go to the engine) */
#define STG_PQ 16                   /* lanes: armed rects waiting for the next upload (more: merged) */
#define STG_ACK_MAX 64              /* lanes: frames an evict may wait for its acks (then taken over anyway) */

int epd_stream_verbose;
int epd_stream_jit = 1;
int epd_stream_jit_size = 1;
#define JIT_MARGIN 1.5e-3       /* s: render must finish this long before the flip event */
/* the hand-off inbox. 1: a submit (and the engine
 * knobs the policy sets per update) is a message the drive thread applies at
 * the top of its loop; the submitter never takes the stream lock. 0: the old
 * direct path (the submitter takes s->mu, which the drive thread holds
 * through epd_schedule: 10-16 ms per hand-off on a scroll). glassine-epdd-desktop
 * handoff=inbox|lock, live */
int epd_stream_inbox = 1;          /* (default on, after an A/B on the device) */
#define INBOX_N 256             /* messages = EPD_MAX_PENDING (a pen stroke's pieces come ~2-3 a frame; full: the submitter waits) */
#ifdef EPD_HOST
void (*epd_stream_host_flip)(uint32_t fb, const uint32_t *words, int lines);
void (*epd_stream_host_applied)(uint32_t id, int kind, epd_rect r);
int epd_stream_host_slow_sched_us;
#endif

struct scan { uint32_t fb; uint8_t *map; uint32_t pitch; int dirty[2]; int dmabuf; size_t size; };

/* an inbox message: a submit (the target already made, ours until applied),
 * a submit with a GPU target (the handle, and its provider at the submit:
 * released through that one if the drive thread has to fall back to a CPU
 * copy), or an engine knob with its arguments. kind: IN_* */
enum { IN_SUBMIT = 1, IN_SUBMIT_GT, IN_CUT, IN_TEMP, IN_MATCHED, IN_MATCHED_RULE, IN_NONBLOCK, IN_MODEL, IN_HEAL, IN_HEAL_LEN, IN_SNAP, IN_KEEPALIVE, IN_PROF };
struct inmsg {
    int kind;
    epd_rect r;
    int mode, full;
    uint8_t *t;
    int gt;
    const epd_gt_provider *pv;
    const uint8_t *grey;
    int stride;
    int a, b;
    long long ns;               /* IN_KEEPALIVE: hold the link until (CLOCK_MONOTONIC ns) */
    double make_ms, lock_ms;    /* the submitter's parts (stats) */
    uint32_t id;
    double tapp;                /* (drive_prof) appended to the inbox (CLOCK_MONOTONIC s) */
    char *str;                  /* IN_PROF: the CSV path (strdup'd; the drive thread frees it) or NULL */
    double org[EPD_ORG_N];      /* (drive_prof) the submitter's stamps for the update's end-to-end record: epd_stream_set_origin */
};

/* the engine as epd_frames_until_free sees it, for epd_stream_free_ms without
 * the lock: the runs (payback ones left out) with the frames they have left,
 * the pendings (kept ones left out) with their length once started, and the
 * length a mode would get now (the messages not applied yet are pendings too) */
struct eng_view {
    struct { epd_rect r; int left; } run[EPD_MAX_UPD];
    int nrun;
    struct { epd_rect r; int frames; } pend[EPD_MAX_PENDING];
    int npend;
    int est[2][16];             /* [full][base mode]: a pending's frames (A2 with a length: the length) */
    int stag_on;                /* STG_ON: a new target every frame, nothing to wait for */
    int hz;
};

struct epd_stream {
    epd_engine e;
    uint8_t *wbf;
    int fd, hv;
    uint32_t crtc, conn;
    struct drm_mode_modeinfo mode;
    struct scan buf[NBUF], neutral;
    uint32_t *frame;            /* cached render target, EPD_LINE words per line */
    int fdirty[2];
    int (*raw_fn)(void *, uint32_t *, int *);   /* epd_stream_raw: set, and */
    void *raw_arg;
    int raw_on;                 /* its frames going out (the engine was idle) */
    int link, linkframes, qos, hv_max_ms_was;
    int warm;                 /* warm idle: CRTC + bridge up, rails down, no frames (link = 0) */
    double warm_since;
    int park;                   /* epd_stream_park: no warm idle, no keepalive, link fully down at once */
    int no_rails;               /* the kernel has no "rails" parameter (before 0051): no warm idle */
    double link_since;
    int kick_fd;                /* PARAMS hv_kick (patch 0050), -1: none, cycle every CYCLE_MS */
    int cycle_ms;               /* link cycle: CYCLE_MS, or just under hv_cap_ms with hv_kick */
    double kick_at;             /* the rails timeout was last (re)armed */
    double flip_t;              /* when the pending flip was issued */
    double evt_age;             /* last flip event: arrival - its timestamp = transfer time */
    int evt_queued;             /* ... and it was already queued when we began to wait: we
                                 * were busy, its "arrival" is our late read (an upper bound) */
    int link_fails;             /* consecutive dead links (see LINK_DEAD_MS) */
    int slow_run;               /* consecutive frames slower than LINK_SLOW_MS */
    int gpu_slow;               /* consecutive GPU frames slower than GPU_LATE_MS */
    double link_retry_at;       /* no link-up attempt before this */
    double slept_s;             /* boottime - monotonic at the last link up: grows by the time suspended */
    pthread_t th, work[NWORK];
    pid_t tid[NWORK];           /* [0] the stream thread, [i] worker i (0: not started) */
    cpu_set_t qos_set;          /* per-CPU resume latency held on these (qos N) */
    int qos_cpus;
    /* worker pool: job `job` runs as job(s, i) on part i of jn (<= NWORK); only
     * workers 1..jn-1 are woken, each on its own condition */
    pthread_mutex_t wmu;
    pthread_cond_t wgo[NWORK], wdone;
    int jn;
    void (*job)(epd_stream *, int);
    unsigned gen;
    int wleft, wquit;
    struct scan *jbuf;
    int jy0, jy1;
    /* band parts (the engine's epd_band_ops): worker i runs bfn(barg, i - 1 + boff)
     * for i <= bn; bleft of them still running (a render meanwhile: on the stream
     * thread alone, pool_run) */
    void (*bfn)(void *, int);
    void *barg;
    int bn, boff, bleft;
    unsigned bgen;
    int bdone;                  /* (atomic) a job's last part ran: drive_wait does not sleep */
    epd_band_ops bops;
    /* update job (epd_stream_upd_run): parts unext.. of un, uleft not yet
     * run, uhelp workers inside; uon/ubusy/the counters atomic, ufn/uarg set
     * before uon. umu/ucv: the caller's wait for the last part / helper.
     * urun/uctx: epd_stream_submit's target copy runner */
    void (*ufn)(void *, int);
    void *uarg;
    int un, unext, uleft, uhelp, uon, ubusy;
    pthread_mutex_t umu;
    pthread_cond_t ucv;
    unsigned long long ujobs, uwparts, ucparts, userial;    /* (atomic) */
    epd_upd_runner urun;
    void *uctx;
    int prio, overlap_on;       /* the stream thread's FIFO priority; drive_overlap as applied */
    pthread_mutex_t mu;
    pthread_cond_t wake, idle;
    int quit, busy;
    /* the hand-off inbox (epd_stream_inbox; see "the hand-off inbox" below):
     * a ring the submitters append to under imu, the drive thread drains
     * under mu at the top of its loop */
    pthread_mutex_t imu;
    pthread_cond_t ispace;      /* the ring was full: a slot came free */
    struct inmsg *in;           /* INBOX_N */
    int ihead;
    int icount;                 /* also read without imu (atomic): the drive thread's sleep check */
    int asleep;                 /* (atomic) the drive thread is about to wait / waits on `wake`: see wake_drive */
    uint32_t sub_id;            /* the ids the inbox path returns (the engine's stay inside) */
    long long hold_ns;          /* keepalive: link (and rails) up until then (CLOCK_MONOTONIC ns; atomic) */
    double in_make, in_lock;    /* the last submit's parts (under imu; epd_stream_last_submit) */
    long long in_tgt_ns;        /* ... and its engine time (atomic; the drive thread) */
    uint64_t ifull;             /* appends that waited for a slot (under imu) */
    int prof_on;                /* drive_prof : the drive thread's own flag (IN_PROF) */
    struct dprof *dp;           /* ... its tables (allocated at the first enable, kept) */
    /* the engine's view for epd_stream_free_ms without the lock: published by
     * the drive thread after every change (seqlock vseq: odd while written) */
    struct eng_view *view;
    unsigned vseq;
    int gt_ok;                  /* (atomic) GPU update targets taken now (gt_on, no stagger run) */
    int wl_fd, wul_fd;          /* /sys/power/wake_lock, wake_unlock; -1: none */
    double wl_armed;            /* when the wakeup source was last (re)armed */
    int hv_want;                /* epd_stream_set_hv: applied at the next link up; -1 none */
    epd_stats st;
    int levels_lost;            /* the GPU hung with the engine state on it (epd_stream_levels_lost) */
#ifdef EPD_GPU
    int gpe;                    /* epd_stream_gpe in force: 0 off, 1 on, 2 check */
    int gpe_req;                /* GPE_DOWN / GPE_UP for the stream thread (gpe_sync) */
    pthread_cond_t gpe_cv;      /* ... done */
    epd_pixel_ops px;           /* the engine's pixel ops: ctx = this stream */
    epd_claim_out gpe_cpu_out;  /* check, deferred claims: the CPU's report, for the collect */
    int gpe_quiet;              /* a settle while the gpe goes: a failed collect only lets go */
    uint8_t *gpe_chk;           /* check: the GPU's whole state, read back */
    epd_gpu *gpu;               /* NULL: the CPU renders */
    int gtarget[NBUF];          /* epd-gpu target of buf[i] */
    int gpu_full;               /* upload all per-pixel state before the next frame */
    char gpu_min_was[32];       /* devfreq min_freq to restore at link down */
    uint32_t *gpu_words;        /* check mode: a GPU frame read back */
    int gpu_dead;               /* gpu_abandon: until gpu_revive (or a reboot) */
    epd_gpu *gpu_lost;          /* the abandoned GPU, kept for gpu_revive */
    struct scan gbuf[NBUF];     /* its scanout buffers, while the CPU's are in buf */
    struct scan cbuf[NBUF];     /* the CPU's, kept for the next abandon (0 fb: none yet) */
    double revive_at, revive_gap;   /* when gpu_revive looks next, and the gap after that */
#define GPU_RFD_MAX 8
    int rfd[GPU_RFD_MAX], nrfd;     /* gpu_reset_seen: Mesa's render-node fds, */
    uint64_t rfault[GPU_RFD_MAX];   /* their fault counts last seen, */
    double rfd_scan;                /* when /proc/self/fd was last read */
    /* stage 5: GPU update targets (epd_stream_set_gt) */
    const epd_gt_provider *gt_pv;   /* the producer's layers (NULL: none) */
    int gt_req;                 /* register gt_pv with the gpe (stream thread) */
    int gt_on;                  /* registered: epd_stream_submit_gt takes handles */
    epd_target_ops tops;        /* the engine's: release to the producer, fetch through epd-gpu */
    /* the staggered steps (epd_stream_stagger) */
    struct {
        int state;              /* STG_OFF, STG_ON, STG_DRAIN */
        int arm;                /* a cut step went to the engine: start once it is idle */
        int gl;                 /* epd_gpu_stag_open done (-1: failed, never again) */
        int glok;               /* it was opened: a gpu_revive brings it back (gl -1 then = the GPU went) */
        epd_stag S;             /* S.tgt: the targets (the submitters write them, under mu) */
        epd_stag_par p;         /* ctl, pay, R (n and G: per frame from n) */
        int n;                  /* the latest step's drive frames (len - 1) */
        epd_rect dirty;         /* targets changed since the last upload (w 0: none) */
        epd_rect lrect;         /* pixels loaded since the GPU last took loads (w 0: none) */
        epd_rect ldbox;         /* every pixel loaded in this run (the rest are parked) */
        int quiet;              /* frames since the targets last changed */
        int nmax;               /* the longest step started (the drain's length; ctl 2, 3: G) */
        int a21;                /* this run: A2/1 (epd_stream_stag_a21 with ctl 3) */
        int drain_left;
        /* armed: the cut steps wait here (not in the engine, so it gets idle
         * and the stagger starts); anything else first gives them to the engine */
        uint8_t *hold;          /* EPD_W x EPD_H levels (malloc'd once) */
        epd_rect hr[STG_HOLD];  /* the held rects (exact: a gap between two is not theirs) */
        int nhr;
        double arm_t;           /* when the engine took the cut that armed it */
        int parked;             /* S all parked but ldbox (an entry done) */
        int hmode;
        const uint8_t *hgrey;   /* the submitter's grey (the engine merges from it) */
        int hstride;
        int hn;                 /* steps held */
        /* stagger=check: the CPU reference's copy, its frame, the GPU's read back */
        epd_stag C;
        int cok;                /* C follows this run (0: off, or lost) */
        int cgl;                /* C allocated (-1: no memory) */
        uint32_t *cw, *gw;      /* the CPU's / the GPU's words of the last frame */
        int cread;              /* gw holds this frame's GPU words */
        int clog;               /* lines logged this run */
        epd_rect cbox;          /* the rows C rendered this frame (ldbox then) */
        epd_stag X;             /* the GPU's state read back (self-consistency) */
        uint32_t *xw;           /* the CPU's step of X */
        int xread;              /* xw holds this frame's self-step */
        uint32_t f0;            /* C.f at the run's entry (the self-check's early frames) */
        epd_stag_par fp;        /* this frame's parameters, taken under the lock at the upload */
        int lctl;               /* the last run's ctl (into or out of 4: the lanes re-parked) */
        int c4warn;             /* "stagger_ctl 4 not available" said */
        /* stagger=check at ctl 4: C4 the twin (epd-bp4.h; its acks in C.ack),
         * M4 its masks (layout m4k), dsh the D it evicts (0xffffffff: not
         * evicted this run) */
        int c4;                 /* this run's check follows C4 (cgl: allocated) */
        bp4_state C4;
        bp_masks M4;
        int m4k;
        uint32_t *dsh;
        /* stagger_state=lanes (epd_gpu_stag_lanes after the
         * open): the A2 pass on the engine's D and T. A cut step's targets are
         * staged in S.tgt (| ARM, padded to 16-px lines, S.ld marks the armed
         * pixels) and its rect queued (pq); stag_upload writes them into T
         * between frames. The run's end (DRAIN) evicts ldbox (evr) at the next
         * upload; the frames after it look for the acks (evf frames so far),
         * then the takeover (T := D's level where armed) ends the run: no
         * state moves either way */
        int lanes, lrepark;     /* lanes: H to be re-parked at the next entry (after a loss) */
        epd_rect pq[STG_PQ];
        int npq, evd, evf;      /* evd: the evict is in T */
        epd_rect evr;
        double sync0;           /* epd_dmabuf_sync_s at the entry */
    } stg;
#endif
};

/* claim_state packed : a running job's bands write D/T in
 * place, so whoever reads or writes them from outside the engine joins it
 * first (arrays and check: the job writes its shadow only, nothing to wait) */
static void pk_quiet(epd_stream *s)
{
    if (s->e.cst == 1) epd_job_join(&s->e);
}

/* the calling thread onto the CPUs of the highest cpu_capacity (mainline
 * without cpufreq: the silver cluster, 1.38 GHz vs gold's 600 MHz). Nothing
 * if all are equal or sysfs doesn't say. */
static int fast_cpus(cpu_set_t *set)
{
    int cap[16], n = 0, hi = 0, lo = 1 << 30, k = 0;
    CPU_ZERO(set);
    for (; n < 16; n++) {
        char p[64];
        snprintf(p, sizeof p, "/sys/devices/system/cpu/cpu%d/cpu_capacity", n);
        FILE *f = fopen(p, "r");
        if (!f) break;
        if (fscanf(f, "%d", &cap[n]) != 1) cap[n] = 0;
        fclose(f);
        if (cap[n] > hi) hi = cap[n];
        if (cap[n] < lo) lo = cap[n];
    }
    if (!n || hi == lo) return 0;
    for (int i = 0; i < n; i++) if (cap[i] == hi) { CPU_SET(i, set); k++; }
    return k;
}

static void pin_fast_cpus(void)
{
    cpu_set_t set;
    if (fast_cpus(&set)) pthread_setaffinity_np(pthread_self(), sizeof set, &set);
}

/* uclamp.min of one thread, policy and priority kept (musl has no wrapper) */
struct epd_sched_attr {
    uint32_t size, sched_policy;
    uint64_t sched_flags;
    int32_t sched_nice;
    uint32_t sched_priority;
    uint64_t sched_runtime, sched_deadline, sched_period;
    uint32_t sched_util_min, sched_util_max;
};

static int set_umin(pid_t tid, int v)
{
    struct epd_sched_attr a = { .size = sizeof a };
    a.sched_flags = 0x08 | 0x10 | 0x20;     /* KEEP_POLICY | KEEP_PARAMS | UTIL_CLAMP_MIN */
    a.sched_util_min = v < 0 ? (uint32_t)-1 : (uint32_t)(v > 1024 ? 1024 : v);   /* -1: reset */
    return syscall(SYS_sched_setattr, tid, &a, 0);
}

void epd_stream_set_rt_umin(epd_stream *s, int v)
{
    epd_stream_rt_umin = v;
    for (int i = 0; i < NWORK; i++)
        if (s->tid[i] && set_umin(s->tid[i], v) && epd_stream_verbose)
            perror("epd: sched_setattr uclamp.min");
}

/* thread start: FIFO 50 (callers), the uclamp knob, pinned; tid for later changes */
static void rt_thread_setup(epd_stream *s, int i)
{
    s->tid[i] = (pid_t)syscall(SYS_gettid);
    if (epd_stream_rt_umin >= 0) set_umin(s->tid[i], epd_stream_rt_umin);
    pin_fast_cpus();
}

static void cpu_qos(const cpu_set_t *set, const char *v)
{
    for (int c = 0; c < CPU_SETSIZE; c++) {
        if (!CPU_ISSET(c, set)) continue;
        char p[96];
        snprintf(p, sizeof p, "/sys/devices/system/cpu/cpu%d/power/pm_qos_resume_latency_us", c);
        int fd = open(p, O_WRONLY | O_CLOEXEC);
        if (fd < 0 || write(fd, v, strlen(v)) < 0) { if (epd_stream_verbose) perror(p); }
        if (fd >= 0) close(fd);
    }
}

static double now(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec / 1e9;
}

static void acc(double *sum, double *max, double v)
{
    *sum += v;
    if (v > *max) *max = v;
}

static void inbox_put(epd_stream *s, struct inmsg *m);
static uint32_t apply_msg(epd_stream *s, struct inmsg *m);

/* ---- drive_prof : the drive loop's split, opt-in ----
 * One record per loop iteration, closed at the top of the next one. Types:
 * stagger a stagger frame (ctl 3 run) enter the frame of a run start (stag_enter: the load)
 * exit the stagger frame that ended it engine an engine LUT frame on the GPU (jit class mid/whole)
 * pen an engine GPU frame of class pen (<= 176 rows)
 * heal an engine GPU frame whose runs are all heal/payback slots
 * cpu an engine frame the CPU rendered (no GPU, or it failed)
 * raw an epd_stream_raw frame idle an iteration that rendered nothing (flip waits, idle waits)
 * Fields (ms unless named otherwise):
 * inbox inbox_drain (all of the iteration's; incl. its view_publish) msgs messages drained (count)
 * jit time asleep in the JIT wait lead its predicted render lead (the first prediction)
 * sched epd_schedule (0 on stagger/raw frames: not called there)
 * pix its pixel passes (epd_claim_pixels) ser claim_plan + claim_start blocked pend_blocked passes
 * claims claims started (count) ckpx their rects, kpx
 * publish the loop's own view_publish calls
 * upload stagger: stag_upload (load at a run start, target rects); engine GPU: gpu_state + epd_gpu_starts
 * pack ... of which the CPU pack (state_row) in epd_gpu_state
 * submit the CPU's time to issue the GPU frame (epd_gpu_render incl. its LUT upload / epd_gpu_stag_frame)
 * fence the wait for it (epd_gpu_finish)
 * cpur CPU render + copy (render + publish; raw: raw_fn + publish; incl. a failed GPU frame's redo)
 * adv the advance block (stag_advance / epd_gpu_finishing + epd_advance)
 * fin ... epd_advance's finish_row passes fkpx the finished runs' rects, kpx
 * flipw wait_flip_by (the previous frame's flip event)
 * flip link up/down + the flip ioctl slack ms from this frame ready to its deadline (the
 * previous flip's expected event; < 0 = it missed it) lag flip lag after the event
 * total the iteration's wall time minus the idle wait wait the idle wait (drive_wait at the bottom)
 * CSV columns: t_s,type,cpu,khz,cls,ev,tid,<fields>; cls = epd_jit_class (0 pen 1 mid 2 whole 3 stagger),
 * ev bits: 1 run start, 2 run end, 4 a claim pass ran, 8 late, 16 heal-only. khz: the newest 20 Hz
 * sample of the record's CPU's policy (the sampler thread; -1 none).
 * End-to-end records (type "upd-engine" / "upd-stagger", one per submit from a thread that set
 * epd_stream_set_origin, the first drive frame that carries it): arr_fence fence_ing ing_scan
 * scan_commit commit_app (main thread), app_drain (the inbox), drain_sched (to the start of the
 * schedule of the frame whose run started it; stagger: that frame's start), sched_flip (to its flip),
 * app_flip, arr_flip (0 where a stamp is unknown). Merged submits keep the first one's stamps;
 * superseded/absorbed ones expire after 5 s ("lost"). */
enum { DT_STAG, DT_ENTER, DT_EXIT, DT_ENG, DT_PEN, DT_HEAL, DT_CPU, DT_RAW, DT_IDLE, DT_N };
static const char *const dt_name[DT_N] = { "stagger", "enter", "exit", "engine", "pen", "heal", "cpu", "raw", "idle" };
enum { DF_INBOX, DF_MSGS, DF_JIT, DF_LEAD, DF_SCHED, DF_PIX, DF_SER, DF_BLOCK, DF_CLAIMS, DF_CKPX, DF_PUBLISH, DF_UPLOAD,
       DF_PACK, DF_SUBMIT, DF_FENCE, DF_CPUR, DF_ADV, DF_FIN, DF_FKPX, DF_FLIPW, DF_FLIP, DF_SLACK, DF_LAG, DF_TOTAL,
       DF_WAIT, DF_N };
static const char *const df_name[DF_N] = { "inbox", "msgs", "jit", "lead", "sched", "pix", "ser", "blocked", "claims", "ckpx",
                                           "publish", "upload", "pack", "submit", "fence", "cpur", "adv", "fin", "fkpx",
                                           "flipw", "flip", "slack", "lag", "total", "wait" };
enum { UT_ENG, UT_STAG, UT_N };
static const char *const ut_name[UT_N] = { "upd-engine", "upd-stagger" };
enum { UF_ARR_FENCE, UF_FENCE_ING, UF_ING_SCAN, UF_SCAN_COMMIT, UF_COMMIT_APP, UF_APP_DRAIN, UF_DRAIN_SCHED, UF_SCHED_FLIP,
       UF_APP_FLIP, UF_ARR_FLIP, UF_N };
static const char *const uf_name[UF_N] = { "arr_fence", "fence_ing", "ing_scan", "scan_commit", "commit_app", "app_drain",
                                           "drain_sched", "sched_flip", "app_flip", "arr_flip" };
#define DP_RAW 1024
#define DP_URAW 256
#define DP_TRK 64
#define DP_CPUS 16
enum { EV_ENTER = 1, EV_EXIT = 2, EV_CLAIM = 4, EV_LATE = 8, EV_HEAL = 16 };
struct dprof {
    epd_prof tab, upd;
    float cur[DF_N];
    double it0;                 /* the iteration's start */
    int rendered, stg, raw, gpu_done, cls, ev;
    int every;                  /* s */
    FILE *csv;
    char *dump;                 /* a dump asked for (path; "" = the default) */
    int tid;
    struct { double t; int type, cpu, cls, ev; long khz; float v[DF_N]; } raw_[DP_RAW];
    unsigned long nraw;
    struct { double t; int type; float v[UF_N]; } uraw[DP_URAW];
    unsigned long nuraw;
    struct { uint32_t id; int st, stag; double org[EPD_ORG_N], app, drain, sched; } trk[DP_TRK];
    unsigned long lost, trk_full;
    /* the period's extras */
    unsigned long cpu_n[DT_N][DP_CPUS], khz_n[DT_N];
    double khz_sum[DT_N];
    unsigned long drains, msgs, msgs_max, eng_frames, claim_frames, adv_frames;
    uint64_t ifull_last;
};

/* the CPU frequency sampler (not the FIFO thread: no sysfs reads there):
 * every cpufreq policy at 20 Hz, the newest kHz per CPU, rows to <csv>.freq */
static long fq_khz[DP_CPUS] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
static int fq_run;
static pthread_t fq_th;
static pthread_mutex_t fq_mu = PTHREAD_MUTEX_INITIALIZER;
static char fq_path[256];       /* (fq_mu) the CSV to (re)open, "" none */
static int fq_gen;              /* (fq_mu) bumped when fq_path changes */
static int fq_every = 5;        /* s: the proc summary's period (drive_prof's every) */

/* the proc summary (the sampler thread, never the FIFO one), every fq_every s:
 * every glassine-epdd-desktop thread's CPU time and run-queue wait from
 * /proc/self/task/N/schedstat (deltas), its policy/rtprio and last CPU
 * (/proc/self/task/N/stat), each core's busy share (/proc/stat) and the CPU
 * pressure (/proc/pressure/cpu some/full total deltas). Do RT threads starve
 * the pipeline: a SCHED_OTHER thread with a large wait next to busy FIFO
 * threads on the same cores says yes */
#define PQ_T 96
static struct { int tid; unsigned long long run, wait; } pq_prev[PQ_T];
static int pq_n;
static unsigned long long pq_cpu[DP_CPUS][2], pq_psi[2];       /* busy, total jiffies; some, full us */
static double pq_t;

static int pq_read(const char *path, char *b, size_t n)
{
    FILE *f = fopen(path, "re");
    if (!f) return -1;
    size_t k = fread(b, 1, n - 1, f);
    fclose(f);
    b[k] = 0;
    return (int)k;
}

static void pq_report(void)
{
    double t = now(), dt = t - pq_t;
    int first = pq_t == 0;
    pq_t = t;
    /* threads */
    struct { int tid, cpu, prio, pol; char comm[24]; double run, wait; } th[PQ_T];
    int nth = 0, nprev = 0;
    static struct { int tid; unsigned long long run, wait; } cur[PQ_T];
    DIR *d = opendir("/proc/self/task");
    for (struct dirent *de; d && (de = readdir(d)) && nth < PQ_T;) {
        int tid = atoi(de->d_name);
        if (tid <= 0) continue;
        char p[64], b[1024];
        unsigned long long run = 0, wait = 0;
        snprintf(p, sizeof p, "/proc/self/task/%d/schedstat", tid);
        if (pq_read(p, b, sizeof b) <= 0 || sscanf(b, "%llu %llu", &run, &wait) != 2) continue;
        snprintf(p, sizeof p, "/proc/self/task/%d/stat", tid);
        if (pq_read(p, b, sizeof b) <= 0) continue;
        char *l = strchr(b, '('), *r = strrchr(b, ')');
        if (!l || !r) continue;
        int k = nth;
        snprintf(th[k].comm, sizeof th[k].comm, "%.*s", (int)(r - l - 1 < 23 ? r - l - 1 : 23), l + 1);
        /* after ") ": field 3 (state) on; processor 39, rt_priority 40, policy 41 */
        char *q = r + 2;
        int fld = 3;
        th[k].cpu = th[k].prio = th[k].pol = -1;
        for (char *tok = strtok(q, " "); tok; tok = strtok(NULL, " "), fld++) {
            if (fld == 39) th[k].cpu = atoi(tok);
            else if (fld == 40) th[k].prio = atoi(tok);
            else if (fld == 41) { th[k].pol = atoi(tok); break; }
        }
        unsigned long long pr = 0, pw = 0;
        int seen = 0;
        for (int i = 0; i < pq_n; i++) if (pq_prev[i].tid == tid) { pr = pq_prev[i].run; pw = pq_prev[i].wait; seen = 1; break; }
        th[k].tid = tid;
        th[k].run = seen ? (run - pr) / 1e6 : run / 1e6;
        th[k].wait = seen ? (wait - pw) / 1e6 : wait / 1e6;
        cur[nprev].tid = tid; cur[nprev].run = run; cur[nprev].wait = wait; nprev++;
        nth++;
    }
    if (d) closedir(d);
    memcpy(pq_prev, cur, sizeof cur[0] * nprev);
    pq_n = nprev;
    /* cores */
    char b[8192];
    double share[DP_CPUS];
    int ncpu = 0;
    if (pq_read("/proc/stat", b, sizeof b) > 0)
        for (char *ln = strstr(b, "\ncpu"); ln; ln = strstr(ln + 1, "\ncpu")) {
            int c;
            unsigned long long v[8] = { 0 };
            if (sscanf(ln + 1, "cpu%d %llu %llu %llu %llu %llu %llu %llu %llu", &c, &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7]) < 5) continue;
            if (c < 0 || c >= DP_CPUS) continue;
            unsigned long long tot = 0;
            for (int i = 0; i < 8; i++) tot += v[i];
            unsigned long long busy = tot - v[3] - v[4];
            share[c] = tot > pq_cpu[c][1] ? 100.0 * (busy - pq_cpu[c][0]) / (tot - pq_cpu[c][1]) : 0;
            pq_cpu[c][0] = busy; pq_cpu[c][1] = tot;
            if (c + 1 > ncpu) ncpu = c + 1;
        }
    /* pressure */
    unsigned long long ps[2] = { 0, 0 };
    int psi = pq_read("/proc/pressure/cpu", b, sizeof b) > 0;
    if (psi) {
        char *s1 = strstr(b, "some"), *s2 = strstr(b, "full"), *tt;
        if (s1 && (tt = strstr(s1, "total="))) ps[0] = strtoull(tt + 6, NULL, 10);
        if (s2 && (tt = strstr(s2, "total="))) ps[1] = strtoull(tt + 6, NULL, 10);
    }
    double some = (ps[0] - pq_psi[0]) / 1e4 / dt, full = (ps[1] - pq_psi[1]) / 1e4 / dt;   /* us -> % of the interval */
    pq_psi[0] = ps[0]; pq_psi[1] = ps[1];
    if (first) return;          /* (the baseline) */
    char line[4096];
    int o = snprintf(line, sizeof line, "epd-prof threads (%.1f s; run/wait ms, policy prio, last cpu):", dt);
    for (int pass = 0; pass < nth && o < (int)sizeof line - 96; pass++) {      /* (by run time, most first) */
        int best = -1;
        for (int i = 0; i < nth; i++) if (th[i].tid && (best < 0 || th[i].run > th[best].run)) best = i;
        if (best < 0 || th[best].run + th[best].wait < 0.05) break;
        static const char *const pn[] = { "OTHER", "FIFO", "RR", "BATCH", "?", "IDLE", "DL" };
        o += snprintf(line + o, sizeof line - o, " %s/%d %.1f/%.1f %s%s%.0d cpu%d;", th[best].comm, th[best].tid, th[best].run,
                      th[best].wait, th[best].pol >= 0 && th[best].pol < 7 ? pn[th[best].pol] : "?",
                      th[best].prio > 0 ? " " : "", th[best].prio, th[best].cpu);
        th[best].tid = 0;
    }
    fprintf(stderr, "%s\n", line);
    o = snprintf(line, sizeof line, "epd-prof cores busy %%:");
    for (int c = 0; c < ncpu; c++) o += snprintf(line + o, sizeof line - o, " cpu%d %.0f", c, share[c]);
    if (psi) snprintf(line + o, sizeof line - o, "; cpu pressure some %.1f %% full %.1f %%", some, full);
    fprintf(stderr, "%s\n", line);
}

static void *fq_thread(void *arg)
{
    (void)arg;
    struct sched_param sp = { 0 };
    pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp);   /* (never RT: it reads sysfs) */
    int pol[DP_CPUS], npol = 0;
    unsigned cpus[DP_CPUS] = { 0 };
    for (int i = 0; i < 64 && npol < DP_CPUS; i++) {
        char p[96], b[128];
        snprintf(p, sizeof p, "/sys/devices/system/cpu/cpufreq/policy%d/affected_cpus", i);
        FILE *f = fopen(p, "re");
        if (!f) continue;
        unsigned m = 0;
        if (fgets(b, sizeof b, f))
            for (char *q = b; *q;) {
                char *e;
                long c = strtol(q, &e, 10);
                if (e == q) { q++; continue; }
                if (c >= 0 && c < DP_CPUS) m |= 1u << c;
                q = e;
            }
        fclose(f);
        pol[npol] = i; cpus[npol++] = m;
    }
    FILE *out = NULL;
    int gen = -1;
    while (__atomic_load_n(&fq_run, __ATOMIC_ACQUIRE)) {
        pthread_mutex_lock(&fq_mu);
        if (gen != fq_gen) {
            gen = fq_gen;
            if (out) fclose(out);
            out = fq_path[0] ? epd_prof_fopen(fq_path) : NULL;     /* (a control line named it: see epd-profpath.h) */
            if (out) fprintf(out, "t_s,policy,cpus,khz\n");
            else if (fq_path[0]) perror(fq_path);
        }
        pthread_mutex_unlock(&fq_mu);
        double t = now();
        for (int k = 0; k < npol; k++) {
            char p[96], b[32];
            snprintf(p, sizeof p, "/sys/devices/system/cpu/cpufreq/policy%d/scaling_cur_freq", pol[k]);
            FILE *f = fopen(p, "re");
            long v = f && fgets(b, sizeof b, f) ? atol(b) : -1;
            if (f) fclose(f);
            for (int c = 0; c < DP_CPUS; c++) if (cpus[k] >> c & 1) __atomic_store_n(&fq_khz[c], v, __ATOMIC_RELAXED);
            if (out) fprintf(out, "%.6f,%d,%#x,%ld\n", t, pol[k], cpus[k], v);
        }
        if (out) fflush(out);
        if (t - pq_t >= __atomic_load_n(&fq_every, __ATOMIC_RELAXED)) pq_report();
        usleep(50000);
    }
    if (out) fclose(out);
    return NULL;
}

static void fq_set(int on, const char *csv)        /* (the caller's thread: glassine-epdd-desktop's main) */
{
    if (csv) {
        pthread_mutex_lock(&fq_mu);
        if (*csv && strcmp(csv, "off")) snprintf(fq_path, sizeof fq_path, "%s.freq", csv); else fq_path[0] = 0;
        fq_gen++;
        pthread_mutex_unlock(&fq_mu);
    }
    if (on == 1 && !fq_run) {
        pq_t = 0; pq_n = 0;             /* (a new baseline) */
        __atomic_store_n(&fq_run, 1, __ATOMIC_RELEASE);
        pthread_attr_t a;
        pthread_attr_init(&a);
        pthread_attr_setinheritsched(&a, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&a, SCHED_OTHER);
        if (pthread_create(&fq_th, &a, fq_thread, NULL)) __atomic_store_n(&fq_run, 0, __ATOMIC_RELEASE);
        pthread_attr_destroy(&a);
    } else if (on == 0 && fq_run) {
        __atomic_store_n(&fq_run, 0, __ATOMIC_RELEASE);
        pthread_join(fq_th, NULL);
        for (int c = 0; c < DP_CPUS; c++) __atomic_store_n(&fq_khz[c], -1L, __ATOMIC_RELAXED);
    }
}

static __thread double origin_tl[EPD_ORG_N];

void epd_stream_set_origin(const double *org)
{
    for (int k = 0; k < EPD_ORG_N; k++) origin_tl[k] = org ? org[k] : 0;
}

#define PF(s) __builtin_expect((s)->prof_on, 0)
#define PADD(s, f, v) do { if (PF(s)) (s)->dp->cur[f] += (float)(v); } while (0)

static void dp_csv_head_to(FILE *f)
{
    fprintf(f, "t_s,type,cpu,khz,cls,ev,tid");
    for (int k = 0; k < DF_N; k++) fprintf(f, ",%s", df_name[k]);
    fprintf(f, "\n");
}
static void dp_csv_head(struct dprof *d)
{
    if (d->csv) dp_csv_head_to(d->csv);
}

/* IN_PROF on the drive thread, under mu */
static void prof_apply(epd_stream *s, struct inmsg *m)
{
    struct dprof *d = s->dp;
    if (!d) { free(m->str); return; }
    if (m->mode == 1) {                 /* a dump */
        free(d->dump);
        d->dump = m->str ? m->str : strdup("");
        return;
    }
    if (m->str) {
        if (d->csv) { fclose(d->csv); d->csv = NULL; }
        if (*m->str && strcmp(m->str, "off")) {
            d->csv = epd_prof_fopen(m->str);    /* (a control line named it: only a file in EPD_PROF_DIR) */
            if (!d->csv) perror(m->str);
            else { setvbuf(d->csv, NULL, _IOFBF, 1 << 16); dp_csv_head(d); }
        }
        free(m->str);
    }
    if (m->b > 0) d->every = m->b;
    if (m->a == 1 && !s->prof_on) {
        epd_prof_init(&d->tab, DT_N, dt_name, DF_N, df_name);
        epd_prof_init(&d->upd, UT_N, ut_name, UF_N, uf_name);
        memset(d->cur, 0, sizeof d->cur);
        d->it0 = now();
        d->rendered = d->stg = d->raw = d->gpu_done = d->ev = 0;
        d->cls = -1;
        d->tid = (int)syscall(SYS_gettid);
        if (!d->every) d->every = 5;
        memset(d->trk, 0, sizeof d->trk);
        memset(&epd_sched_t, 0, sizeof epd_sched_t);
#ifdef EPD_GPU
        epd_gpu_prof_pack = 0;
#endif
        d->ifull_last = s->ifull;
        s->prof_on = 1;
        epd_sched_prof = 1;
        fprintf(stderr, "epd: drive_prof on (every %d s%s), drive thread tid %d\n", d->every, d->csv ? ", CSV" : "", d->tid);
    } else if (m->a == 0 && s->prof_on) {
        s->prof_on = 0;
        epd_sched_prof = 0;
        if (d->csv) fflush(d->csv);
        fprintf(stderr, "epd: drive_prof off\n");
    }
}

/* an end-to-end record starts: a submit applied (drive thread, under mu) */
static void prof_track(epd_stream *s, uint32_t id, int stag, const struct inmsg *m)
{
    struct dprof *d = s->dp;
    double t = now();
    int fr = -1;
    for (int i = 0; i < DP_TRK; i++) {
        if (d->trk[i].st && d->trk[i].id == id && d->trk[i].stag == stag) return;   /* merged: the first one's stamps */
        if (d->trk[i].st && t - d->trk[i].app > 5) { d->trk[i].st = 0; d->lost++; }
        if (!d->trk[i].st && fr < 0) fr = i;
    }
    if (fr < 0) { d->trk_full++; return; }
    d->trk[fr].id = id; d->trk[fr].stag = stag; d->trk[fr].st = 1;
    memcpy(d->trk[fr].org, m->org, sizeof m->org);
    d->trk[fr].app = m->tapp;
    d->trk[fr].drain = t;
}

/* after epd_schedule (t0 = its start): the tracked engine updates that started now */
static void prof_started(epd_stream *s, double t0, int stg)
{
    struct dprof *d = s->dp;
    for (int i = 0; i < DP_TRK; i++) {
        if (d->trk[i].st != 1) continue;
        if (d->trk[i].stag) { if (stg) { d->trk[i].st = 2; d->trk[i].sched = t0; } continue; }
        for (int k = 0; !stg && k < EPD_MAX_UPD; k++)
            if (s->e.run[k].nframes && !s->e.run[k].frame && s->e.run[k].id == d->trk[i].id) { d->trk[i].st = 2; d->trk[i].sched = t0; break; }
    }
}

static float dms(double a, double b) { return a > 0 && b > 0 ? (float)((b - a) * 1e3) : 0; }

/* the frame flipped at t: the started ones are done */
static void prof_flipped(epd_stream *s, double t)
{
    struct dprof *d = s->dp;
    for (int i = 0; i < DP_TRK; i++) {
        if (d->trk[i].st != 2) continue;
        const double *o = d->trk[i].org;
        float v[UF_N] = { dms(o[0], o[1]), dms(o[1], o[2]), dms(o[2], o[3]), dms(o[3], o[4]), dms(o[4], d->trk[i].app),
                          dms(d->trk[i].app, d->trk[i].drain), dms(d->trk[i].drain, d->trk[i].sched),
                          dms(d->trk[i].sched, t), dms(d->trk[i].app, t), dms(o[0], t) };
        int ty = d->trk[i].stag ? UT_STAG : UT_ENG;
        epd_prof_add(&d->upd, ty, v);
        unsigned long k = d->nuraw++ % DP_URAW;
        d->uraw[k].t = t; d->uraw[k].type = ty;
        memcpy(d->uraw[k].v, v, sizeof v);
        d->trk[i].st = 0;
    }
}

/* the iteration's record (the top of the next one, under mu) */
static void prof_close(epd_stream *s)
{
    struct dprof *d = s->dp;
    double t = now();
    float *v = d->cur;
    v[DF_TOTAL] = (float)((t - d->it0) * 1e3) - v[DF_WAIT];
    int ty;
    if (!d->rendered) ty = DT_IDLE;
    else if (d->stg) ty = d->ev & EV_ENTER ? DT_ENTER : d->ev & EV_EXIT ? DT_EXIT : DT_STAG;
    else if (d->raw) ty = DT_RAW;
    else if (!d->gpu_done) ty = DT_CPU;
    else if (d->ev & EV_HEAL) ty = DT_HEAL;
    else ty = d->cls == 0 ? DT_PEN : DT_ENG;
    v[DF_CKPX] /= 1e3f;
    v[DF_FKPX] /= 1e3f;
    int cpu = sched_getcpu();
    long khz = cpu >= 0 && cpu < DP_CPUS ? __atomic_load_n(&fq_khz[cpu], __ATOMIC_RELAXED) : -1;
    epd_prof_add(&d->tab, ty, v);
    if (cpu >= 0 && cpu < DP_CPUS) d->cpu_n[ty][cpu]++;
    if (khz > 0) { d->khz_sum[ty] += khz; d->khz_n[ty]++; }
    if (d->rendered && !d->stg && !d->raw) {
        d->eng_frames++;
        if (v[DF_CLAIMS] > 0) d->claim_frames++; else d->adv_frames++;
    }
    unsigned long k = d->nraw++ % DP_RAW;
    d->raw_[k].t = d->it0; d->raw_[k].type = ty; d->raw_[k].cpu = cpu; d->raw_[k].cls = d->cls; d->raw_[k].ev = d->ev;
    d->raw_[k].khz = khz;
    memcpy(d->raw_[k].v, v, sizeof d->raw_[k].v);
    if (d->csv) {
        char ex[96];
        snprintf(ex, sizeof ex, "%d,%ld,%d,%d,%d", cpu, khz, d->cls, d->ev, d->tid);
        epd_prof_csv_row(&d->tab, d->csv, d->it0, ty, ex, v);
    }
    memset(d->cur, 0, sizeof d->cur);
    d->rendered = d->stg = d->raw = d->gpu_done = d->ev = 0;
    d->cls = -1;
    d->it0 = t;
}

#define PROF_DUMP_DEFAULT "/run/glassine-epdd-desktop-frames.csv"
static void prof_dump_file(struct dprof *d, const char *path)
{
    /* (the default is the daemon's own; a path given is confined like the CSVs) */
    FILE *f = !strcmp(path, PROF_DUMP_DEFAULT) ? epd_prof_open_file(path) : epd_prof_fopen(path);
    if (!f) { perror(path); return; }
    dp_csv_head_to(f);
    unsigned long n = d->nraw < DP_RAW ? d->nraw : DP_RAW;
    for (unsigned long i = d->nraw - n; i < d->nraw; i++) {
        const typeof(d->raw_[0]) *r = &d->raw_[i % DP_RAW];
        char ex[96];
        snprintf(ex, sizeof ex, "%d,%ld,%d,%d,%d", r->cpu, r->khz, r->cls, r->ev, d->tid);
        epd_prof_csv_row(&d->tab, f, r->t, r->type, ex, r->v);
    }
    fprintf(f, "\n");
    fprintf(f, "t_s,type");
    for (int k = 0; k < UF_N; k++) fprintf(f, ",%s", uf_name[k]);
    fprintf(f, "\n");
    n = d->nuraw < DP_URAW ? d->nuraw : DP_URAW;
    for (unsigned long i = d->nuraw - n; i < d->nuraw; i++)
        epd_prof_csv_row(&d->upd, f, d->uraw[i % DP_URAW].t, d->uraw[i % DP_URAW].type, NULL, d->uraw[i % DP_URAW].v);
    fclose(f);
    fprintf(stderr, "epd: drive_prof: %lu frames + %lu update records into %s\n", d->nraw < DP_RAW ? d->nraw : DP_RAW,
            d->nuraw < DP_URAW ? d->nuraw : DP_URAW, path);
}

/* the summary when due (or a dump asked for); mu NOT held */
static void prof_report(epd_stream *s, int force)
{
    struct dprof *d = s->dp;
    if (!force && !d->dump && now() - d->tab.t0 < d->every) return;
    char exs[DT_N][160];
    const char *ex[DT_N];
    for (int t = 0; t < DT_N; t++) {
        int o = 0;
        exs[t][0] = 0;
        unsigned long n = 0;
        for (int c = 0; c < DP_CPUS; c++) n += d->cpu_n[t][c];
        for (int c = 0; c < DP_CPUS && n; c++)
            if (d->cpu_n[t][c]) o += snprintf(exs[t] + o, sizeof exs[t] - o, "%scpu%d %.0f%%", o ? " " : "(", c, 100.0 * d->cpu_n[t][c] / n);
        if (o && d->khz_n[t]) o += snprintf(exs[t] + o, sizeof exs[t] - o, ", %.0f MHz", d->khz_sum[t] / d->khz_n[t] / 1e3);
        if (o) snprintf(exs[t] + o, sizeof exs[t] - o, ")");
        ex[t] = exs[t];
    }
    epd_prof_print(&d->tab, stderr, "drive", ex);
    epd_prof_print(&d->upd, stderr, "update", NULL);
    uint64_t ifl = s->ifull;    /* (a racy read of a counter: stats only) */
    if (ifl < d->ifull_last) d->ifull_last = 0;      /* ("stats" zeroed it) */
    fprintf(stderr, "epd-prof drive: tid %d; inbox %lu drains, %.2f msgs per drain (max %lu), inbox_full %llu; "
            "engine frames %lu: %lu with a claim pass (%.0f %%), %lu advance-only; e2e lost %lu, table full %lu\n",
            d->tid, d->drains, d->drains ? (double)d->msgs / d->drains : 0, d->msgs_max, (unsigned long long)(ifl - d->ifull_last),
            d->eng_frames, d->claim_frames, d->eng_frames ? 100.0 * d->claim_frames / d->eng_frames : 0, d->adv_frames,
            d->lost, d->trk_full);
    d->ifull_last = ifl;
    memset(d->cpu_n, 0, sizeof d->cpu_n); memset(d->khz_n, 0, sizeof d->khz_n); memset(d->khz_sum, 0, sizeof d->khz_sum);
    d->drains = d->msgs = d->msgs_max = d->eng_frames = d->claim_frames = d->adv_frames = 0;
    if (d->csv) fflush(d->csv);
    if (d->dump) {
        prof_dump_file(d, *d->dump ? d->dump : PROF_DUMP_DEFAULT);
        free(d->dump);
        d->dump = NULL;
    }
}

/* the engine's parts of this frame, from epd_sched_t (then zeroed) */
static void prof_take_sched(epd_stream *s)
{
    float *v = s->dp->cur;
    v[DF_PIX] += (float)(epd_sched_t.pix * 1e3);
    v[DF_SER] += (float)((epd_sched_t.plan + epd_sched_t.start) * 1e3);
    v[DF_BLOCK] += (float)(epd_sched_t.blocked * 1e3);
    v[DF_CLAIMS] += (float)epd_sched_t.claims;
    v[DF_CKPX] += (float)epd_sched_t.px;
    v[DF_FIN] += (float)(epd_sched_t.finish * 1e3);
    v[DF_FKPX] += (float)epd_sched_t.fin_px;
    if (epd_sched_t.claims) s->dp->ev |= EV_CLAIM;
#ifdef EPD_GPU
    v[DF_PACK] += (float)(epd_gpu_prof_pack * 1e3);
    epd_gpu_prof_pack = 0;
#endif
    memset(&epd_sched_t, 0, sizeof epd_sched_t);
}

void epd_stream_prof(epd_stream *s, int on, int every_s, const char *csv)
{
    if (!s->dp && on == 1 && !(s->dp = calloc(1, sizeof *s->dp))) { fprintf(stderr, "epd: drive_prof: no memory\n"); return; }
    if (!s->dp) return;                 /* (never on: nothing to turn off) */
    if (every_s > 0) __atomic_store_n(&fq_every, every_s, __ATOMIC_RELAXED);
    fq_set(on, csv);
    struct inmsg m = { .kind = IN_PROF, .a = on, .b = every_s, .str = csv ? strdup(csv) : NULL };
    if (epd_stream_inbox) { inbox_put(s, &m); return; }
    pthread_mutex_lock(&s->mu);
    apply_msg(s, &m);
    pthread_cond_signal(&s->wake);
    pthread_mutex_unlock(&s->mu);
}

void epd_stream_prof_dump(epd_stream *s, const char *path)
{
    if (!s->dp) return;
    struct inmsg m = { .kind = IN_PROF, .mode = 1, .a = -1, .str = strdup(path ? path : "") };
    if (epd_stream_inbox) { inbox_put(s, &m); return; }
    pthread_mutex_lock(&s->mu);
    apply_msg(s, &m);
    pthread_cond_signal(&s->wake);
    pthread_mutex_unlock(&s->mu);
}


#ifndef EPD_HOST
static int xioctl(int fd, unsigned long req, void *arg)
{
    int r;
    do r = ioctl(fd, req, arg); while (r < 0 && (errno == EINTR || errno == EAGAIN));
    return r;
}

static int param_get(const char *name)
{
    char b[16] = "", p[160];
    snprintf(p, sizeof p, PARAMS "%s", name);
    FILE *f = fopen(p, "r");
    if (!f) return -1;
    if (!fgets(b, sizeof b, f)) b[0] = 0;
    fclose(f);
    if (b[0] == 'Y') return 1;
    if (b[0] == 'N') return 0;
    return atoi(b);
}

static int param_set(const char *name, const char *v)
{
    char p[160];
    snprintf(p, sizeof p, PARAMS "%s", name);
    FILE *f = fopen(p, "w");
    if (!f) return -1;
    int r = fputs(v, f) < 0;
    return fclose(f) || r ? -1 : 0;
}

/* the driver's rails alone (kernel 0051): 0 or -errno */
static int rails_set(int on)
{
    int f = open(PARAMS "rails", O_WRONLY | O_CLOEXEC);
    if (f < 0) return -errno;
    int r = write(f, on ? "1" : "0", 1) == 1 ? 0 : -errno;
    close(f);
    return r;
}
#else
/* ---- host harness (test/epd-stream-test.c, -DEPD_HOST; ) ---------
 * No kernel, no panel: the DRM device is a pipe. A page flip queues one
 * flip-complete event for the next 85 Hz tick (a timer thread writes it into
 * the pipe, so poll/read in wait_flip_by are the real ones), SETCRTC turns the
 * fake pipe on and off (hv_on = the hv parameter while on; the rails parameter
 * alone for warm idle), the dumb buffers are calloc'd, the driver parameters a
 * few ints. The drive loop, the link state machine, the JIT wait, the worker
 * pool and the inbox are the real code. epd_stream_host_flip sees every
 * flipped buffer (the test hashes the frame sequence), epd_stream_host_applied
 * every message the drive thread applied, epd_stream_host_slow_sched_us makes
 * epd_schedule slow (a long claim) to show what a submit waits for. */
#define HOST_BUFS 16
static struct {
    int rd, wr;                 /* the pipe: flip events */
    pthread_t th;
    int th_on;
    pthread_mutex_t mu;
    int quit, crtc_on, hv, hv_on;
    uint32_t flip_fb;           /* the flip pending for the next tick (0: none) */
    double t0, period;
    int nhandle;
    uint8_t *bufs[HOST_BUFS];   /* handle k: bufs[k - 1] (fb id = handle) */
    int fds[HOST_BUFS];         /* its udmabuf (-1: calloc'd; PRIME_HANDLE_TO_FD fails then: the CPU renders) */
    size_t sizes[HOST_BUFS];
} H = { .mu = PTHREAD_MUTEX_INITIALIZER };

/* a scanout buffer the GPU can import (scene `stagger`): a
 * udmabuf over a memfd, mapped; -1 = none */
static int host_udmabuf(size_t n, uint8_t **map)
{
    int m = memfd_create("glassine-epd-host-scan", MFD_ALLOW_SEALING | MFD_CLOEXEC), u, fd = -1;
    if (m < 0) return -1;
    if (!ftruncate(m, (off_t)n) && !fcntl(m, F_ADD_SEALS, F_SEAL_SHRINK) && (u = open("/dev/udmabuf", O_RDWR | O_CLOEXEC)) >= 0) {
        struct udmabuf_create c = { .memfd = (unsigned)m, .flags = UDMABUF_FLAGS_CLOEXEC, .offset = 0, .size = n };
        fd = ioctl(u, UDMABUF_CREATE, &c);
        close(u);
    }
    close(m);
    if (fd < 0) return -1;
    void *p = mmap(0, n, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (p == MAP_FAILED) { close(fd); return -1; }
    *map = p;
    return fd;
}

static void *host_tick(void *a)
{
    (void)a;
    for (long k = 1;; k++) {
        double t = H.t0 + k * H.period;
        struct timespec ts = { (time_t)t, (long)((t - (time_t)t) * 1e9) };
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        pthread_mutex_lock(&H.mu);
        int quit = H.quit;
        uint32_t fb = H.flip_fb;
        H.flip_fb = 0;
        pthread_mutex_unlock(&H.mu);
        if (quit) break;
        if (fb) {
            struct drm_event_vblank ev = { .base = { DRM_EVENT_FLIP_COMPLETE, sizeof ev } };
            ev.tv_sec = (uint32_t)(time_t)t;
            ev.tv_usec = (uint32_t)((t - (time_t)t) * 1e6);
            if (write(H.wr, &ev, sizeof ev) < 0) {}
        }
    }
    return NULL;
}

static int host_open(void)
{
    int p[2];
    if (pipe2(p, O_CLOEXEC)) return -1;
    H.rd = p[0]; H.wr = p[1];
    H.quit = 0; H.crtc_on = 0; H.hv_on = 0; H.flip_fb = 0; H.nhandle = 0;
    H.period = 1.0 / 85;
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    H.t0 = t.tv_sec + t.tv_nsec / 1e9;
    H.th_on = !pthread_create(&H.th, NULL, host_tick, NULL);
    return H.rd;
}

static void host_close(void)
{
    pthread_mutex_lock(&H.mu);
    H.quit = 1;
    pthread_mutex_unlock(&H.mu);
    if (H.th_on) pthread_join(H.th, NULL);
    H.th_on = 0;
    close(H.rd); close(H.wr);
    for (int i = 0; i < H.nhandle; i++) {
        if (H.fds[i] >= 0) { munmap(H.bufs[i], H.sizes[i]); close(H.fds[i]); } else free(H.bufs[i]);
        H.bufs[i] = NULL;
    }
    H.nhandle = 0;
}

static int xioctl(int fd, unsigned long req, void *arg)
{
    (void)fd;
    switch (req) {
    case DRM_IOCTL_MODE_GETRESOURCES: {
        struct drm_mode_card_res *r = arg;
        if (r->crtc_id_ptr) ((uint32_t *)(uintptr_t)r->crtc_id_ptr)[0] = 1;
        if (r->connector_id_ptr) ((uint32_t *)(uintptr_t)r->connector_id_ptr)[0] = 2;
        if (r->encoder_id_ptr) ((uint32_t *)(uintptr_t)r->encoder_id_ptr)[0] = 3;
        r->count_fbs = 0; r->count_crtcs = 1; r->count_connectors = 1; r->count_encoders = 1;
        return 0; }
    case DRM_IOCTL_MODE_GETCONNECTOR: {
        struct drm_mode_get_connector *c = arg;
        if (c->modes_ptr) {
            struct drm_mode_modeinfo *m = (void *)(uintptr_t)c->modes_ptr;
            memset(m, 0, sizeof *m);
            m->hdisplay = EPD_LINE; m->vdisplay = EPD_LINES; m->vrefresh = 85;
        }
        if (c->encoders_ptr) ((uint32_t *)(uintptr_t)c->encoders_ptr)[0] = 3;
        c->count_modes = 1; c->count_encoders = 1; c->count_props = 0;
        return 0; }
    case DRM_IOCTL_MODE_CREATE_DUMB: {
        struct drm_mode_create_dumb *c = arg;
        if (H.nhandle >= HOST_BUFS) { errno = ENOMEM; return -1; }
        c->pitch = c->width * 4;
        c->size = (uint64_t)c->pitch * c->height;
        size_t n = ((size_t)c->size + 4095) & ~(size_t)4095;
        H.sizes[H.nhandle] = n;
        H.fds[H.nhandle] = host_udmabuf(n, &H.bufs[H.nhandle]);
        if (H.fds[H.nhandle] < 0) H.bufs[H.nhandle] = calloc(1, c->size);
        c->handle = (uint32_t)++H.nhandle;
        return 0; }
    case DRM_IOCTL_PRIME_HANDLE_TO_FD: {
        struct drm_prime_handle *p = arg;
        if (p->handle < 1 || p->handle > (uint32_t)H.nhandle || H.fds[p->handle - 1] < 0) { errno = ENOTTY; return -1; }
        p->fd = fcntl(H.fds[p->handle - 1], F_DUPFD_CLOEXEC, 0);
        return p->fd < 0 ? -1 : 0; }
    case DRM_IOCTL_MODE_MAP_DUMB: { struct drm_mode_map_dumb *m = arg; m->offset = m->handle; return 0; }
    case DRM_IOCTL_MODE_ADDFB2: { struct drm_mode_fb_cmd2 *a = arg; a->fb_id = a->handles[0]; return 0; }
    case DRM_IOCTL_MODE_SETCRTC: {
        struct drm_mode_crtc *c = arg;
        pthread_mutex_lock(&H.mu);
        H.crtc_on = c->mode_valid;
        H.flip_fb = 0;
        H.hv_on = H.crtc_on ? H.hv : 0;
        pthread_mutex_unlock(&H.mu);
        return 0; }
    case DRM_IOCTL_MODE_PAGE_FLIP: {
        struct drm_mode_crtc_page_flip *p = arg;
        pthread_mutex_lock(&H.mu);
        int ok = H.crtc_on && !H.flip_fb && p->fb_id >= 1 && p->fb_id <= (uint32_t)H.nhandle;
        if (ok) H.flip_fb = p->fb_id;
        pthread_mutex_unlock(&H.mu);
        if (!ok) { errno = EBUSY; return -1; }
        if (epd_stream_host_flip) epd_stream_host_flip(p->fb_id, (const uint32_t *)H.bufs[p->fb_id - 1], EPD_LINES);
        return 0; }
    default:
        errno = ENOTTY;
        return -1;
    }
}

static int param_get(const char *name)
{
    if (!strcmp(name, "hv_on")) { pthread_mutex_lock(&H.mu); int v = H.hv_on; pthread_mutex_unlock(&H.mu); return v; }
    if (!strcmp(name, "hv_max_ms")) return 30000;
    return -1;
}

static int param_set(const char *name, const char *v)
{
    if (!strcmp(name, "hv")) {
        pthread_mutex_lock(&H.mu);
        H.hv = atoi(v) != 0;
        if (H.crtc_on) H.hv_on = H.hv;
        pthread_mutex_unlock(&H.mu);
        return 0;
    }
    return strcmp(name, "hv_max_ms") ? -1 : 0;
}

static int rails_set(int on)
{
    pthread_mutex_lock(&H.mu);
    H.hv_on = on && H.hv;
    pthread_mutex_unlock(&H.mu);
    return 0;
}
#endif

static int make_scan(epd_stream *s, struct scan *b, int gpu)
{
    struct drm_mode_create_dumb c = { .width = EPD_LINE, .height = EPD_LINES, .bpp = 32 };
#ifdef EPD_GPU
    if (gpu) c.height += EPD_GPU_EXTRA_LINES;       /* never scanned out */
#endif
    if (xioctl(s->fd, DRM_IOCTL_MODE_CREATE_DUMB, &c)) return -1;
    b->dmabuf = -1;
    if (gpu) {                  /* for the GPU to render into */
        struct drm_prime_handle ph = { .handle = c.handle, .flags = DRM_CLOEXEC | DRM_RDWR };
        if (!xioctl(s->fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &ph)) b->dmabuf = ph.fd;
    }
    struct drm_mode_map_dumb m = { .handle = c.handle };
    if (xioctl(s->fd, DRM_IOCTL_MODE_MAP_DUMB, &m)) return -1;
#ifdef EPD_HOST
    b->map = H.bufs[c.handle - 1];          /* (the fake device's own memory) */
#else
    b->map = mmap(0, c.size, PROT_READ | PROT_WRITE, MAP_SHARED, s->fd, m.offset);
    if (b->map == MAP_FAILED) return -1;
#endif
    b->pitch = c.pitch;
    b->size = c.size;
    memset(b->map, 0, c.size);
    /* XBGR8888: SSPP unpack 0x03010002; with the DSI host's
     * GRB order (DT qcom,cmd-rgb-swap-grb) the wire carries the words as-is */
    struct drm_mode_fb_cmd2 a = { .width = EPD_LINE, .height = EPD_LINES, .pixel_format = DRM_FORMAT_XBGR8888 };
    a.handles[0] = c.handle; a.pitches[0] = c.pitch;
    if (xioctl(s->fd, DRM_IOCTL_MODE_ADDFB2, &a)) return -1;
    b->fb = a.fb_id;
    b->dirty[0] = 0; b->dirty[1] = EPD_LINES;      /* all lines still to be written */
    return 0;
}

static int setup_drm(epd_stream *s)
{
#ifdef EPD_HOST
    s->fd = host_open();
#else
    s->fd = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
#endif
    if (s->fd < 0) { perror("epd: /dev/dri/card0"); return -1; }
    struct drm_mode_card_res r = { 0 };
    uint32_t crtcs[8], conns[8], encs[8];
    if (xioctl(s->fd, DRM_IOCTL_MODE_GETRESOURCES, &r) || r.count_crtcs < 1 || r.count_crtcs > 8 ||
        r.count_connectors < 1 || r.count_connectors > 8 || r.count_encoders > 8) return -1;
    r.crtc_id_ptr = (uintptr_t)crtcs; r.connector_id_ptr = (uintptr_t)conns; r.encoder_id_ptr = (uintptr_t)encs;
    r.count_fbs = 0;
    if (xioctl(s->fd, DRM_IOCTL_MODE_GETRESOURCES, &r)) return -1;
    struct drm_mode_get_connector c = { .connector_id = conns[0] };
    struct drm_mode_modeinfo modes[8];
    uint32_t ce[8];
    if (xioctl(s->fd, DRM_IOCTL_MODE_GETCONNECTOR, &c) || c.count_modes < 1 || c.count_modes > 8 || c.count_encoders > 8) return -1;
    c.modes_ptr = (uintptr_t)modes; c.encoders_ptr = (uintptr_t)ce; c.count_props = 0;
    if (xioctl(s->fd, DRM_IOCTL_MODE_GETCONNECTOR, &c)) return -1;
    if (modes[0].hdisplay != EPD_LINE || modes[0].vdisplay != EPD_LINES) {
        fprintf(stderr, "epd: mode %ux%u is not the Note Air 1 wire link\n", modes[0].hdisplay, modes[0].vdisplay);
        return -1;
    }
    s->mode = modes[0];
    s->conn = c.connector_id;
    s->crtc = crtcs[0];
    for (int i = 0; i < NBUF; i++) if (make_scan(s, &s->buf[i], epd_stream_gpu)) return -1;
    if (make_scan(s, &s->neutral, 0)) return -1;           /* stays all zero */
    return 0;
}

/* While frames stream, the event -> flip turnaround must fit in ~0.7 ms, but
 * waking from the deep cpuidle states takes up to 2.3 ms (cpu-sleep-0-1 on
 * mainline sdm660): hold the PM QoS CPU latency at 0 (WFI only) while the
 * link is up. */
static void gpu_clock_hold(epd_stream *s, int on);

static void qos_hold(epd_stream *s, int on)
{
    gpu_clock_hold(s, on);
    if (on && s->qos < 0 && !s->qos_cpus && epd_stream_qos >= 0) {
        char v[16];                     /* sysfs: "n/a" = none tolerated, "0" = no limit */
        if (epd_stream_qos == 0) strcpy(v, "n/a"); else snprintf(v, sizeof v, "%d", epd_stream_qos);
        s->qos_cpus = fast_cpus(&s->qos_set);
        if (!s->qos_cpus) { CPU_ZERO(&s->qos_set); for (int c = 0; c < 8; c++) CPU_SET(c, &s->qos_set); s->qos_cpus = 8; }
        cpu_qos(&s->qos_set, v);
    } else if (on && s->qos < 0 && !s->qos_cpus && epd_stream_qos == -1) {
        int32_t v = 0;
        s->qos = open("/dev/cpu_dma_latency", O_WRONLY | O_CLOEXEC);
        if (s->qos >= 0 && write(s->qos, &v, sizeof v) != sizeof v) { close(s->qos); s->qos = -1; }
        if (s->qos < 0 && epd_stream_verbose) fprintf(stderr, "epd: no cpu_dma_latency hold\n");
    } else if (!on) {
        if (s->qos >= 0) close(s->qos);     /* closing releases the request */
        s->qos = -1;
        if (s->qos_cpus) cpu_qos(&s->qos_set, "0");
        s->qos_cpus = 0;
    }
}

/* the GPU's devfreq governor ramps far too late for a frame every 11.8 ms
 * (epd-gpu-test: 43 ms frames): its top clock is held while the link is up.
 * A hold that an exit never released (a crash, a kill, a 900E: the
 * device sat at min = max = 430 MHz for 6 days) must not become the "was" of
 * the next one: a min already at the top restores to the lowest available */
#ifdef EPD_GPU
static void gpu_lowest_freq(char *out, size_t n)
{
    char buf[256] = "";
    FILE *f = fopen(GPU_DEVFREQ "available_frequencies", "r");
    if (f) { if (!fgets(buf, sizeof buf, f)) buf[0] = 0; fclose(f); }
    unsigned long lo = 0;
    for (char *p = buf, *e; *p; p = e) {
        unsigned long v = strtoul(p, &e, 10);
        if (e == p) break;
        if (v && (!lo || v < lo)) lo = v;
    }
    if (lo) snprintf(out, n, "%lu\n", lo);
}
#endif
static void gpu_clock_hold(epd_stream *s, int on)
{
#ifdef EPD_GPU
    if (!s->gpu) return;
    if (on && !s->gpu_min_was[0]) {
        char top[32] = "";
        FILE *f = fopen(GPU_DEVFREQ "min_freq", "r");
        if (f && !fgets(s->gpu_min_was, sizeof s->gpu_min_was, f)) s->gpu_min_was[0] = 0;
        if (f) fclose(f);
        if ((f = fopen(GPU_DEVFREQ "max_freq", "r"))) { if (!fgets(top, sizeof top, f)) top[0] = 0; fclose(f); }
        if (s->gpu_min_was[0] && top[0] && strtoul(s->gpu_min_was, NULL, 10) >= strtoul(top, NULL, 10)) {
            char lo[32] = "";
            gpu_lowest_freq(lo, sizeof lo);
            if (lo[0] && strtoul(lo, NULL, 10) < strtoul(top, NULL, 10)) {
                fprintf(stderr, "epd: GPU min_freq was left at the top (%lu): restores to %lu\n",
                        strtoul(s->gpu_min_was, NULL, 10), strtoul(lo, NULL, 10));
                snprintf(s->gpu_min_was, sizeof s->gpu_min_was, "%s", lo);
            }
        }
        if (s->gpu_min_was[0] && top[0] && (f = fopen(GPU_DEVFREQ "min_freq", "w"))) { fputs(top, f); fclose(f); }
    } else if (!on && s->gpu_min_was[0]) {
        FILE *f = fopen(GPU_DEVFREQ "min_freq", "w");
        if (f) { fputs(s->gpu_min_was, f); fclose(f); }
        s->gpu_min_was[0] = 0;
    }
#else
    (void)s; (void)on;
#endif
}

/* A flip's event arrives <= ~24 ms after the flip (next 11.76 ms tick + the
 * ~11 ms transfer). Much later means the link is not delivering frames: ,
 * the DSI PLL failed to lock after the radio came up, the host did not power
 * on, the driver still raised the rails, and every kickoff timed out (~88 ms)
 * for 25 s: HV on the panel with no valid frames (undefined drive, the panel
 * pulsed and ended too light). Never again: such a link goes down at once. */
#define LINK_DEAD_MS 150     /* a latency spike with radio load: <= ~60 ms */
/* ...and a frame's transfer (event arrival - event timestamp) is ~11 ms at the
 * full link rate. after the PLL failure the link came back at ~1/4
 * rate (~41 ms per frame, ~20 Hz): every waveform phase held ~4x too long,
 * the panel over-driven (too light). A slower link is a bad link too. */
#define LINK_SLOW_MS 20
#define LINK_SLOW_RUN 3       /* consecutive slow frames (single ones: IRQ/wakeup latency under WiFi load) */
#define LINK_RETRY_MS 2000
/* right after a resume the DSI PLL failed to lock (twice; caught
 * above) and once a link passed the one-frame check, then a kickoff timed out
 * with the rails up. The first link after a resume proves itself longer. */
#define LINK_RESUME_FRAMES 5
#define LINK_MAX_FAILS 5
/* given up: one more try this often (10-01: with the DSI runtime-PM leak fixed,
 * kernel 0033, a PLL wedged after a resume is reset once the MDSS GDSC powers
 * off, so the link can come back; slow, a wedged MDSS must not be hammered) */
#define LINK_GIVEUP_RETRY_MS 60000

static int wait_flip_by(epd_stream *s, double *t, double deadline);
static int flip(epd_stream *s, uint32_t fb);
static void link_kill(epd_stream *s, const char *why);
static void log_temp_range(epd_stream *s, int c);

/* seconds spent suspended since boot (CLOCK_BOOTTIME counts them, MONOTONIC not) */
static double slept(void)
{
    struct timespec b, m;
    clock_gettime(CLOCK_BOOTTIME, &b);
    clock_gettime(CLOCK_MONOTONIC, &m);
    return (b.tv_sec - m.tv_sec) + (b.tv_nsec - m.tv_nsec) / 1e9;
}

static int link_wake(epd_stream *s);
static int link_confirm(epd_stream *s);

static int link_up(epd_stream *s)
{
    if (now() < s->link_retry_at) return -1;            /* given up: see link_kill */
    int want = __atomic_exchange_n(&s->hv_want, -1, __ATOMIC_ACQ_REL);
    if (want >= 0 && want != s->hv) {           /* the link is down here: the rails are off */
        if (param_set("hv", want ? "1" : "0")) fprintf(stderr, "epd: hv=%d not set\n", want);
        else { s->hv = want; fprintf(stderr, "epd: hv %d (%s)\n", want, want ? "panel driven" : "dry run"); }
    }
    if (s->warm) {              /* warm idle: rails only (else it fell back: CRTC off, cold below) */
        int r = link_wake(s);
        if (r != -1) return r;
    }
    double t0 = now();
    qos_hold(s, 1);
    struct drm_mode_crtc c = { .crtc_id = s->crtc, .fb_id = s->neutral.fb, .count_connectors = 1,
                               .set_connectors_ptr = (uintptr_t)&s->conn, .mode = s->mode, .mode_valid = 1 };
    if (xioctl(s->fd, DRM_IOCTL_MODE_SETCRTC, &c)) { perror("epd: SETCRTC"); qos_hold(s, 0); return -1; }
    double t_set = now();
    int on = param_get("hv_on");
    if (on != s->hv) {
        /* the DSI host did not power on (rails stay off): a dead link too.
         * 10-01 (8097bb01): uncounted, this cycled the CRTC 3x per frame for
         * as long as the PLL stayed wedged; then the SoC froze (900E) */
        char why[64];
        snprintf(why, sizeof why, "link up but hv_on = %d (want %d)", on, s->hv);
        link_kill(s, why);
        return -1;
    }
    s->link = 1;
    s->linkframes = 0;
    s->link_since = now();
    s->kick_at = s->link_since;         /* rails up: the driver's timeout armed */
    s->st.links++;
    /* prove the link delivers frames before anything is drawn: one neutral frame */
    double t;
    if (flip(s, s->neutral.fb) || wait_flip_by(s, &t, s->flip_t + LINK_DEAD_MS / 1e3)) {
        link_kill(s, "link-up check: no flip event");
        return -1;
    }
    /* after a resume: more neutral frames, every one on time */
    double sl = slept();
    if (sl - s->slept_s > 0.5) {
        for (int k = 1; k < LINK_RESUME_FRAMES; k++) {
            if (s->evt_age > LINK_SLOW_MS) {
                char why[80];
                snprintf(why, sizeof why, "link-up check after resume: frame %d took %.1f ms", k, s->evt_age);
                link_kill(s, why);
                return -1;
            }
            if (flip(s, s->neutral.fb) || wait_flip_by(s, &t, s->flip_t + LINK_DEAD_MS / 1e3)) {
                link_kill(s, "link-up check after resume: no flip event");
                return -1;
            }
        }
        fprintf(stderr, "epd: link up after a resume (%.0f s asleep): %d neutral frames on time\n",
                sl - s->slept_s, LINK_RESUME_FRAMES);
    }
    s->slept_s = sl;
    if (link_confirm(s)) return -1;
    if (LINK_LOG) LINK_TS();
    if (LINK_LOG)
        fprintf(stderr, "epd: link up (hv_on %d): cold %.1f ms (modeset incl. bridge + rails %.1f, check frames %.1f)\n",
                on, (now() - t0) * 1e3, (t_set - t0) * 1e3, (now() - t_set) * 1e3);
    return 0;
}

/* slow: confirm with more neutral frames (a single slow one can be latency) */
static int link_confirm(epd_stream *s)
{
    double t;
    for (int k = 1; s->evt_age > LINK_SLOW_MS; k++) {
        if (k >= LINK_SLOW_RUN) {
            char why[64];
            snprintf(why, sizeof why, "link-up check: %d frames took > %d ms (last %.1f)", k, LINK_SLOW_MS, s->evt_age);
            link_kill(s, why);
            return -1;
        }
        if (flip(s, s->neutral.fb) || wait_flip_by(s, &t, s->flip_t + LINK_DEAD_MS / 1e3)) {
            link_kill(s, "link-up check: no flip event");
            return -1;
        }
    }
    s->slow_run = 0;
    s->link_fails = 0;
    return 0;
}

/* CRTC off from warm idle: the bridge goes down too (full link down) */
static void warm_down(epd_stream *s, const char *why)
{
    struct drm_mode_crtc o = { .crtc_id = s->crtc };
    if (xioctl(s->fd, DRM_IOCTL_MODE_SETCRTC, &o)) perror("epd: SETCRTC off");
    s->warm = 0;
    if (LINK_LOG) LINK_TS();
    if (LINK_LOG) fprintf(stderr, "epd: link down from warm idle after %.1f s (%s), hv_on %d\n",
                                    now() - s->warm_since, why, param_get("hv_on"));
}

/* idle with the link up: rails down, CRTC + bridge stay (warm idle); no
 * rails parameter or it failed: the full link down */
static void link_down(epd_stream *s);
static int wait_flip(epd_stream *s, double *t);
static void link_warm(epd_stream *s)
{
    double t, t0 = now();
    if (!flip(s, s->neutral.fb)) wait_flip(s, &t);      /* the tested ending: a neutral frame */
    int r = rails_set(0), on = param_get("hv_on");
    if (r || on != 0) {
        if (!s->no_rails) fprintf(stderr, "epd: no warm idle (rails=0: %s, hv_on %d): full link downs\n",
                                  r ? strerror(-r) : "ok", on);
        s->no_rails = 1;
        link_down(s);
        return;
    }
    s->link = 0;
    s->warm = 1;
    s->warm_since = now();
    qos_hold(s, 0);
    if (LINK_LOG) LINK_TS();
    if (LINK_LOG) fprintf(stderr, "epd: link warm: rails down in %.1f ms, hv_on %d (full down in %d ms)\n",
                                    (now() - t0) * 1e3, on, epd_stream_link_warm_ms);
}

/* warm idle -> link up: the rails (another thread: the driver's write blocks
 * for the ramp + power good, ~40 ms) while a neutral frame proves the link,
 * so the proof costs nothing. The proof frame overlaps the ramp: all zero,
 * i.e. the same constant wire the bridge holds between frames. HV meets the
 * drive frames only after the proof, as on a cold link up (where the rails
 * come up in enable, before its first frame). 0 up; -1 not possible, CRTC
 * off, go cold; -2 dead link (link_kill). */
struct rails_job { int ret; double ms; };
static void *rails_up_job(void *a)
{
    struct rails_job *j = a;
    double t0 = now();
    j->ret = rails_set(1);
    j->ms = (now() - t0) * 1e3;
    return NULL;
}

static int link_wake(epd_stream *s)
{
    double t, t0 = now(), idle = t0 - s->warm_since;
    if (slept() - s->slept_s > 0.5) {
        warm_down(s, "a suspend since");          /* the resume restored the CRTC: start clean */
        return -1;
    }
    s->warm = 0;
    qos_hold(s, 1);
    if (flip(s, s->neutral.fb)) {
        struct drm_mode_crtc o = { .crtc_id = s->crtc };
        if (xioctl(s->fd, DRM_IOCTL_MODE_SETCRTC, &o)) perror("epd: SETCRTC off");
        qos_hold(s, 0);
        return -1;
    }
    struct rails_job j = { 0, 0 };
    pthread_t th;
    int threaded = 0;
    if (s->hv) {
        if (pthread_create(&th, NULL, rails_up_job, &j)) rails_up_job(&j);
        else threaded = 1;
    }
    int late = wait_flip_by(s, &t, s->flip_t + LINK_DEAD_MS / 1e3);
    double t_proof = now();
    if (threaded) pthread_join(th, NULL);
    int on = param_get("hv_on");
    if (late) {
        link_kill(s, "warm wake: no flip event");
        return -2;
    }
    if (j.ret) {
        /* refused (e.g. the link clocks stopped, the pipe went down behind us): cold */
        fprintf(stderr, "epd: warm wake: rails=1 refused (%s): cold link up\n", strerror(-j.ret));
        struct drm_mode_crtc o = { .crtc_id = s->crtc };
        if (xioctl(s->fd, DRM_IOCTL_MODE_SETCRTC, &o)) perror("epd: SETCRTC off");
        qos_hold(s, 0);
        return -1;
    }
    if (on != s->hv) {
        char why[64];
        snprintf(why, sizeof why, "warm wake: hv_on = %d (want %d)", on, s->hv);
        link_kill(s, why);
        return -2;
    }
    s->link = 1;
    s->linkframes = 0;
    s->link_since = now();
    s->kick_at = s->link_since;         /* rails up: the driver's timeout armed */
    s->st.links++;
    s->st.warm_wakes++;
    if (link_confirm(s)) return -2;
    if (LINK_LOG) LINK_TS();
    if (LINK_LOG)
        fprintf(stderr, "epd: link up (hv_on %d): warm %.1f ms (rails %.1f, proof frame %.1f) after %.1f s warm idle\n",
                on, (now() - t0) * 1e3, j.ms, (t_proof - t0) * 1e3, idle);
    return 0;
}

static int wait_flip(epd_stream *s, double *t)
{
    return wait_flip_by(s, t, s->flip_t + 1.0);
}

/* the pending flip's event, or -1 once `deadline` (now seconds) has passed */
static int wait_flip_by(epd_stream *s, double *t, double deadline)
{
    struct pollfd p = { s->fd, POLLIN, 0 };
    s->evt_queued = poll(&p, 1, 0) > 0;
    for (;;) {
        int ms = (int)((deadline - now()) * 1e3 + 0.999);
        if (ms < 0) ms = 0;
        if (poll(&p, 1, ms) <= 0) return -1;
        char b[256];
        ssize_t n = read(s->fd, b, sizeof b);
        for (ssize_t o = 0; o + (ssize_t)sizeof(struct drm_event) <= n;) {
            struct drm_event *ev = (void *)(b + o);
            if (ev->type == DRM_EVENT_FLIP_COMPLETE) {
                /* the time it ARRIVED: the event's own timestamp is ~one transfer
                 * (~11 ms) earlier (every frame then looked 11 ms late, and
                 * the just-in-time render aimed a whole period too early) */
                struct drm_event_vblank *vb = (void *)ev;
                *t = now();
                s->evt_age = (*t - (vb->tv_sec + vb->tv_usec / 1e6)) * 1e3;
                s->st.evt_age_ms_sum += s->evt_age;
                if (s->evt_age > s->st.evt_age_ms_max) s->st.evt_age_ms_max = s->evt_age;
                s->st.evt_n++;
                return 0;
            }
            o += ev->length;
        }
    }
}

static int flip(epd_stream *s, uint32_t fb)
{
    struct drm_mode_crtc_page_flip pf = { .crtc_id = s->crtc, .fb_id = fb, .flags = DRM_MODE_PAGE_FLIP_EVENT };
    if (xioctl(s->fd, DRM_IOCTL_MODE_PAGE_FLIP, &pf)) { perror("epd: PAGE_FLIP"); return -1; }
    s->flip_t = now();
    return 0;
}

/* the link is not delivering frames: CRTC off (rails down) NOW, no neutral
 * frame (it would not arrive either), retry later, give up after a few */
static void link_kill(epd_stream *s, const char *why)
{
    struct drm_mode_crtc o = { .crtc_id = s->crtc };
    if (xioctl(s->fd, DRM_IOCTL_MODE_SETCRTC, &o)) perror("epd: SETCRTC off");
    s->link = 0;
    s->warm = 0;
    qos_hold(s, 0);
    s->st.dead_links++;
    s->link_fails++;
    s->link_retry_at = now() + (s->link_fails >= LINK_MAX_FAILS ? LINK_GIVEUP_RETRY_MS : LINK_RETRY_MS) / 1e3;
    fprintf(stderr, "epd: DEAD LINK (%s): link + rails down, hv_on %d, failure %d/%d%s\n", why,
            param_get("hv_on"), s->link_fails, LINK_MAX_FAILS,
            s->link_fails >= LINK_MAX_FAILS ? ": GIVING UP, the panel is no longer driven (one try a minute)" : "");
}

/* restart the driver's hv_max_ms rails timeout. Only from the stream thread,
 * right after a driven frame's flip event: a wedged epdd (GPU wait, deadlock)
 * stops kicking and the backstop fires; the kernel's hv_cap_ms bounds even a
 * kicker that is stuck in a loop. A failed write: back to the link cycle */
static void hv_kick(epd_stream *s, double t)
{
    if (pwrite(s->kick_fd, "1", 1, 0) == 1) { s->kick_at = t; return; }
    fprintf(stderr, "epd: hv_kick: %s: back to a link cycle every %d s\n", strerror(errno), CYCLE_MS / 1000);
    close(s->kick_fd);
    s->kick_fd = -1;
    s->cycle_ms = CYCLE_MS;
}

static void link_down(epd_stream *s)
{
    double t;
    if (!flip(s, s->neutral.fb)) wait_flip(s, &t);      /* the tested ending: a neutral frame */
    struct drm_mode_crtc o = { .crtc_id = s->crtc };
    if (xioctl(s->fd, DRM_IOCTL_MODE_SETCRTC, &o)) perror("epd: SETCRTC off");
    s->link = 0;
    qos_hold(s, 0);
    if (LINK_LOG) LINK_TS();
    if (LINK_LOG) fprintf(stderr, "epd: link down (hv_on %d)\n", param_get("hv_on"));
}

/* copy the frame's changed lines (+ what this buffer still holds from its
 * last use) into scanout buffer b */
static void *worker(void *arg);

/* run job on jn parts (part 0 on the calling thread) and wait */
static void pool_run(epd_stream *s, void (*job)(epd_stream *, int))
{
    pthread_mutex_lock(&s->wmu);
    int n = epd_stream_nwork < 1 ? 1 : epd_stream_nwork > NREND ? NREND : epd_stream_nwork;
    s->jn = n;
    if (s->bleft) {             /* (a claim job has the workers: every part here, no wait for it) */
        pthread_mutex_unlock(&s->wmu);
        for (int i = 0; i < n; i++) job(s, i);
        return;
    }
    s->job = job;
    s->wleft = n - 1;
    __atomic_add_fetch(&s->gen, 1, __ATOMIC_RELEASE);   /* (an update helper reads it between parts) */
    for (int i = 1; i < n; i++) pthread_cond_signal(&s->wgo[i]);
    pthread_mutex_unlock(&s->wmu);
    job(s, 0);
    pthread_mutex_lock(&s->wmu);
    while (s->wleft) pthread_cond_wait(&s->wdone, &s->wmu);
    pthread_mutex_unlock(&s->wmu);
}

/* worker_rt (epd-drm.h): FIFO 50 + rt_umin, or SCHED_OTHER at
 * EPD_WORKER_NICE with the CFS default clamp; the affinity (pin_fast_cpus,
 * rt_thread_setup) is the same in both */
static int set_umin(pid_t tid, int v);
static void worker_policy(epd_stream *s, int i, int rt)
{
    if (!rt) {
        struct sched_param sp = { .sched_priority = 50 };
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
        if (epd_stream_rt_umin >= 0 && s->tid[i]) set_umin(s->tid[i], epd_stream_rt_umin);
        return;
    }
    struct sched_param sp = { .sched_priority = 0 };
    pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp);
    if (s->tid[i]) set_umin(s->tid[i], -1);         /* (the CFS default: no RT boost) */
    if (setpriority(PRIO_PROCESS, (id_t)syscall(SYS_gettid), EPD_WORKER_NICE) && epd_stream_verbose)
        perror("epd: worker nice");
}

/* an update job has parts left (wmu held) */
static int upd_avail(epd_stream *s)
{
    return __atomic_load_n(&s->uon, __ATOMIC_ACQUIRE) &&
           __atomic_load_n(&s->unext, __ATOMIC_RELAXED) < __atomic_load_n(&s->un, __ATOMIC_RELAXED);
}
static void upd_note(epd_stream *s)
{
    pthread_mutex_lock(&s->umu);
    pthread_cond_broadcast(&s->ucv);
    pthread_mutex_unlock(&s->umu);
}
/* a worker's share of the update job: parts until none is left or the stream
 * thread wants it (a band part or a render/copy part: gen/bgen moved) */
static void upd_help(epd_stream *s, unsigned seen, unsigned bseen)
{
    __atomic_add_fetch(&s->uhelp, 1, __ATOMIC_SEQ_CST);
    if (__atomic_load_n(&s->uon, __ATOMIC_SEQ_CST)) {
        void (*fn)(void *, int) = s->ufn;
        void *arg = s->uarg;
        int n = __atomic_load_n(&s->un, __ATOMIC_RELAXED);
        while (__atomic_load_n(&s->gen, __ATOMIC_ACQUIRE) == seen && __atomic_load_n(&s->bgen, __ATOMIC_ACQUIRE) == bseen) {
            int k = __atomic_fetch_add(&s->unext, 1, __ATOMIC_RELAXED);
            if (k >= n) break;
            fn(arg, k);
            __atomic_add_fetch(&s->uwparts, 1, __ATOMIC_RELAXED);
            if (__atomic_sub_fetch(&s->uleft, 1, __ATOMIC_ACQ_REL) == 0) upd_note(s);
        }
    }
    if (__atomic_sub_fetch(&s->uhelp, 1, __ATOMIC_SEQ_CST) == 0) upd_note(s);
}

static void *worker(void *arg)
{
    epd_stream *s = arg;
    /* a part of every frame: as urgent as the stream thread (a helper preempted
     * by X or the app made the frame late); worker_rt=nice: SCHED_OTHER */
    int rt = __atomic_load_n(&epd_stream_worker_rt, __ATOMIC_RELAXED) != 0;
    if (!rt) {
        struct sched_param sp = { .sched_priority = 50 };
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    }
    pthread_mutex_lock(&s->wmu);
    int i = s->wleft++ + 1;         /* part index, assigned at start */
    /* both before the opener can go on: wleft is what it waits for, and a claim posted
     * during the setup below would count as seen and never run (the stream hangs in band_wait) */
    unsigned seen = s->gen, bseen = s->bgen;
    pthread_mutex_unlock(&s->wmu);
    rt_thread_setup(s, i);
    if (rt) worker_policy(s, i, rt);
    pthread_mutex_lock(&s->wmu);
    pthread_cond_broadcast(&s->wdone);
    for (;;) {
        while (s->gen == seen && s->bgen == bseen && !upd_avail(s) && !s->wquit) pthread_cond_wait(&s->wgo[i], &s->wmu);
        if (s->wquit) break;
        int want = __atomic_load_n(&epd_stream_worker_rt, __ATOMIC_RELAXED) != 0;
        if (want != rt) {           /* (worker_rt changed: at this job) */
            rt = want;
            pthread_mutex_unlock(&s->wmu);
            worker_policy(s, i, rt);
            pthread_mutex_lock(&s->wmu);
        }
        if (s->bgen != bseen) {     /* (a band part) */
            bseen = s->bgen;
            if (i > s->bn) continue;
            int k = i - 1 + s->boff;
            pthread_mutex_unlock(&s->wmu);
            s->bfn(s->barg, k);
            pthread_mutex_lock(&s->wmu);
            if (!--s->bleft) pthread_cond_broadcast(&s->wdone);
            continue;
        }
        if (s->gen != seen) {
            seen = s->gen;
            if (i >= s->jn) continue;   /* not part of this job (spurious wakeup) */
            pthread_mutex_unlock(&s->wmu);
            s->job(s, i);
            pthread_mutex_lock(&s->wmu);
            if (!--s->wleft) pthread_cond_signal(&s->wdone);
            continue;
        }
        if (upd_avail(s)) {         /* (update parts, last in line) */
            pthread_mutex_unlock(&s->wmu);
            upd_help(s, seen, bseen);
            pthread_mutex_lock(&s->wmu);
        }
    }
    pthread_mutex_unlock(&s->wmu);
    return NULL;
}

void epd_stream_upd_run(epd_stream *s, void (*fn)(void *, int), void *arg, int n)
{
    if (!s || n <= 1 || __atomic_exchange_n(&s->ubusy, 1, __ATOMIC_ACQUIRE)) {
        if (s && n > 1) __atomic_add_fetch(&s->userial, 1, __ATOMIC_RELAXED);
        for (int k = 0; k < n; k++) fn(arg, k);
        return;
    }
    s->ufn = fn; s->uarg = arg;
    __atomic_store_n(&s->un, n, __ATOMIC_RELAXED);
    __atomic_store_n(&s->unext, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&s->uleft, n, __ATOMIC_RELAXED);
    __atomic_store_n(&s->uon, 1, __ATOMIC_SEQ_CST);
    __atomic_add_fetch(&s->ujobs, 1, __ATOMIC_RELAXED);
    /* the wake, without wmu: the caller (SCHED_OTHER, maybe on a Silver core)
     * never holds the lock the drive thread takes for its frames (a preempted
     * holder would stall it). A worker between its upd_avail check and its
     * wait misses this one wake and sits the job out: help is best effort,
     * the caller runs every part nobody took */
    for (int i = 1; i < NWORK; i++) pthread_cond_signal(&s->wgo[i]);
    for (;;) {
        int k = __atomic_fetch_add(&s->unext, 1, __ATOMIC_RELAXED);
        if (k >= n) break;
        fn(arg, k);
        __atomic_add_fetch(&s->ucparts, 1, __ATOMIC_RELAXED);
        __atomic_sub_fetch(&s->uleft, 1, __ATOMIC_ACQ_REL);
    }
    pthread_mutex_lock(&s->umu);
    while (__atomic_load_n(&s->uleft, __ATOMIC_ACQUIRE)) pthread_cond_wait(&s->ucv, &s->umu);
    __atomic_store_n(&s->uon, 0, __ATOMIC_SEQ_CST);
    while (__atomic_load_n(&s->uhelp, __ATOMIC_SEQ_CST)) pthread_cond_wait(&s->ucv, &s->umu);
    pthread_mutex_unlock(&s->umu);
    __atomic_store_n(&s->ubusy, 0, __ATOMIC_RELEASE);
}

void epd_stream_set_upd_runner(epd_stream *s, epd_upd_runner run, void *ctx)
{
    pthread_mutex_lock(&s->umu);
    __atomic_store_n(&s->uctx, ctx, __ATOMIC_RELAXED);
    __atomic_store_n(&s->urun, run, __ATOMIC_RELEASE);
    pthread_mutex_unlock(&s->umu);
}

void epd_stream_upd_stats(epd_stream *s, unsigned long long *jobs, unsigned long long *wparts,
                          unsigned long long *cparts, unsigned long long *serial)
{
    *jobs = __atomic_load_n(&s->ujobs, __ATOMIC_RELAXED);
    *wparts = __atomic_load_n(&s->uwparts, __ATOMIC_RELAXED);
    *cparts = __atomic_load_n(&s->ucparts, __ATOMIC_RELAXED);
    *serial = __atomic_load_n(&s->userial, __ATOMIC_RELAXED);
}

/* ---- : the engine's band ops on this pool -----------------
 * run: part 0 here, 1..n-1 on workers 1..n-1, back when done (claim_bands
 * without overlap: the stream thread waits for its own claim, as today).
 * start: parts 0..n-1 on workers 1..n, back at once (drive_overlap: the job);
 * wait: until they ran; done: the job's last part, from its worker */
static void band_go(epd_stream *s, void (*fn)(void *, int), void *arg, int nw, int off)
{
    s->bfn = fn; s->barg = arg; s->bn = nw; s->boff = off; s->bleft = nw;
    __atomic_add_fetch(&s->bgen, 1, __ATOMIC_RELEASE);  /* (as gen) */
    for (int i = 1; i <= nw; i++) pthread_cond_signal(&s->wgo[i]);
}
static void band_wait(void *ctx)
{
    epd_stream *s = ctx;
    pthread_mutex_lock(&s->wmu);
    while (s->bleft) pthread_cond_wait(&s->wdone, &s->wmu);
    pthread_mutex_unlock(&s->wmu);
}
static void band_run(void *ctx, void (*fn)(void *, int), void *arg, int n)
{
    epd_stream *s = ctx;
    pthread_mutex_lock(&s->wmu);
    if (s->bleft || n > NWORK) { pthread_mutex_unlock(&s->wmu); for (int i = 0; i < n; i++) fn(arg, i); return; }
    band_go(s, fn, arg, n - 1, 1);
    pthread_mutex_unlock(&s->wmu);
    fn(arg, 0);
    band_wait(s);
}
static void band_start(void *ctx, void (*fn)(void *, int), void *arg, int n)
{
    epd_stream *s = ctx;
    pthread_mutex_lock(&s->wmu);
    if (s->bleft || n > NWORK - 1) { pthread_mutex_unlock(&s->wmu); for (int i = 0; i < n; i++) fn(arg, i); return; }
    band_go(s, fn, arg, n, 0);
    pthread_mutex_unlock(&s->wmu);
}
static void wake_drive(epd_stream *s);
static void band_done(void *ctx)
{
    epd_stream *s = ctx;
    __atomic_store_n(&s->bdone, 1, __ATOMIC_SEQ_CST);
    wake_drive(s);
}

static void part(int y0, int y1, int i, int n, int *a, int *b)
{
    *a = y0 + (int)((long)(y1 - y0) * i / n);
    *b = y0 + (int)((long)(y1 - y0) * (i + 1) / n);
}

/* render: panel rows between the running updates' extremes, split evenly */
static void job_render(epd_stream *s, int i)
{
    int a, b;
    part(s->jy0, s->jy1, i, s->jn, &a, &b);
    epd_render_rows(&s->e, s->frame, a, b);
}

/* copy: wire lines [jy0, jy1) into jbuf, split evenly */
static void job_copy(epd_stream *s, int i)
{
    int a, b;
    part(s->jy0, s->jy1, i, s->jn, &a, &b);
    struct scan *d = s->jbuf;
    for (int y = a; y < b; y++)
        memcpy(d->map + (size_t)y * d->pitch, s->frame + (size_t)y * EPD_LINE, EPD_LINE * 4);
}

static void render(epd_stream *s)
{
    epd_render_begin(&s->e, s->frame, s->fdirty);
    s->jy0 = EPD_H; s->jy1 = 0;
    for (int k = 0; k < EPD_MAX_UPD; k++) {
        epd_run *r = &s->e.run[k];
        if (!r->nframes) continue;
        if (r->r.y < s->jy0) s->jy0 = r->r.y;
        if (r->r.y + r->r.h > s->jy1) s->jy1 = r->r.y + r->r.h;
    }
    if (s->jy0 < s->jy1) pool_run(s, job_render);
}

/* copy the frame's changed lines (+ what this buffer still holds from its
 * last use) into scanout buffer b */
static void publish(epd_stream *s, struct scan *b)
{
    int y0 = s->fdirty[0], y1 = s->fdirty[1];
    if (y0 >= y1) { y0 = b->dirty[0]; y1 = b->dirty[1]; }
    else if (b->dirty[0] < b->dirty[1]) {
        if (b->dirty[0] < y0) y0 = b->dirty[0];
        if (b->dirty[1] > y1) y1 = b->dirty[1];
    }
    if (y0 < y1) {
        s->jbuf = b; s->jy0 = y0; s->jy1 = y1;
        pool_run(s, job_copy);
        s->st.words_copied += (uint64_t)(y1 - y0) * EPD_LINE;      /* stats: read under mu, benign */
    }
    b->dirty[0] = s->fdirty[0]; b->dirty[1] = s->fdirty[1];
}

#ifdef EPD_GPU
static void gpe_start(epd_stream *s);
static void gpe_off(epd_stream *s, const char *why);
static void gpe_lost(epd_stream *s);
static int gpe_down(epd_stream *s);
static void gt_drop(epd_stream *s);

/* a GPU frame in scanout buffer nb read back (after its fence). On the host
 * the fake device's buffers are udmabufs we mapped (cached): read there —
 * radeonsi's glReadPixels of an imported udmabuf faults (SIGBUS) */
static void scan_read(epd_stream *s, int nb, uint32_t *words)
{
#ifdef EPD_HOST
    struct scan *b = &s->buf[nb];
    epd_dmabuf_sync(b->dmabuf, DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ, 0, 0);
    for (int l = 0; l < EPD_LINES; l++) memcpy(words + (size_t)l * EPD_LINE, b->map + (size_t)l * b->pitch, EPD_LINE * 4);
    epd_dmabuf_sync(b->dmabuf, DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ, 0, 0);
#else
    epd_gpu_read(s->gpu, s->gtarget[nb], words);
#endif
}

/* in the stream thread (the GL context lives there): the scanout buffers
 * become the GPU's render targets; NULL on failure (the CPU renders) */
static void gpu_start(epd_stream *s)
{
    epd_gpu *g = epd_gpu_open(&s->e);
    for (int i = 0; g && i < NBUF; i++)
        if (s->buf[i].dmabuf < 0 || (s->gtarget[i] = epd_gpu_target_dmabuf(g, s->buf[i].dmabuf, (int)s->buf[i].pitch, s->buf[i].size)) < 0) {
            epd_gpu_close(g);
            g = NULL;
        }
    s->gpu = g;
    fprintf(stderr, "epd: drive frames on %s%s%s%s\n", g ? epd_gpu_renderer(g) : "the CPU (no GPU)",
            g && epd_stream_gpu == 2 ? ", checked against the CPU" : "",
            g && epd_gpu_drive_state > 0 ? ", state " : "", g && epd_gpu_drive_state > 0 ? epd_gpu_drive_state_of(g) : "");
    if (g && epd_stream_gpe) gpe_start(s);
}

/* the GPU is dropped (error, hang, a checked frame differed): the CPU renders
 * from the next frame on; its frame cache and every buffer are redone whole */
static void gpu_stop(epd_stream *s, const char *why)
{
    fprintf(stderr, "epd: GPU drive frames OFF (%s): the CPU renders\n", why);
    if (s->gpe) gpe_off(s, "the GPU is dropped");   /* (its state synced down first) */
    gpu_clock_hold(s, 0);
    epd_gpu_close(s->gpu);
    s->gpu = NULL;
    if (epd_stream_gpu != 2) {          /* check mode kept the CPU cache current */
        memcpy(s->frame, s->e.tpl, sizeof(uint32_t) * EPD_WORDS);
        s->fdirty[0] = s->fdirty[1] = 0;
    }
    for (int i = 0; i < NBUF; i++) { s->buf[i].dirty[0] = 0; s->buf[i].dirty[1] = EPD_LINES; }
}

/* the GPU did not finish a frame (hung; 09-29: a CP fault after the
 * kernel's recover left no fence ever signalling, the stream waited in glFinish
 * for good and the panel driver's 30 s backstop cut the rails). Nothing of the
 * GPU's may be freed now: closing its context or the buffers it rendered into
 * waits for the dead fences in the kernel, uninterruptibly (Xorg sat in
 * msm_gem_close until the hung-task panic). So the context and the
 * scanout buffers are left as they are, the CPU gets fresh buffers (no GPU
 * fence on them: a flip does not wait), and the process must not exit or
 * reexec until the GPU comes back (gpu_revive) or a reboot (epd_stream_gpu_dead). */
static void gpu_abandon(epd_stream *s, const char *why)
{
    fprintf(stderr, "epd: GPU HUNG (%s): abandoned, the CPU renders until its frame finishes "
            "(no reexec, no exit meanwhile: freeing its buffers would hang)\n", why);
    gpu_clock_hold(s, 0);
    if (s->gpe) {               /* (nothing of it can be read now) */
        s->gpu_dead = 1;        /* (gt_fetch: white) */
        gt_drop(s);
        epd_set_pixel_ops(&s->e, NULL);         /* (a deferred claim: dropped, its runs keep their rect) */
        s->e.defer = 0;
        if (s->gpe == 1) gpe_lost(s);
        s->gpe = 0;
        pthread_cond_broadcast(&s->gpe_cv);
    }
    s->gpu_lost = s->gpu;       /* kept, not closed (gpu_revive) */
    s->gpu = NULL;
    s->gpu_dead = 1;
    s->revive_gap = 2;
    s->revive_at = now() + s->revive_gap;
    for (int i = 0; i < NBUF; i++) {
        struct scan n = s->cbuf[i];
        if (!n.fb && make_scan(s, &n, 0)) { fprintf(stderr, "epd: no CPU buffer %d: keeping the GPU's\n", i); s->gbuf[i].fb = 0; continue; }
        s->gbuf[i] = s->buf[i];
        s->cbuf[i] = n;
        s->buf[i] = n;
    }
    memcpy(s->frame, s->e.tpl, sizeof(uint32_t) * EPD_WORDS);
    s->fdirty[0] = s->fdirty[1] = 0;
    for (int i = 0; i < NBUF; i++) { s->buf[i].dirty[0] = 0; s->buf[i].dirty[1] = EPD_LINES; }
}

/* under the lock, no frame ready or in flight: the abandoned GPU back once
 * its timed-out frame finished (a GPU starved by another
 * process's long jobs finishes late, and was lost until a reboot; the
 * stagger with it). Looks every 2 s, then 4, 8 .. 60. A dead CP (09-29:
 * the fence never signals) stays on the CPU */
static void gpu_revive(epd_stream *s)
{
    double t = now();
    if (t < s->revive_at) return;
    s->revive_gap = s->revive_gap < 30 ? s->revive_gap * 2 : 60;
    s->revive_at = t + s->revive_gap;
    if (!epd_gpu_revive(s->gpu_lost)) return;
    for (int i = 0; i < NBUF; i++) {
        if (!s->gbuf[i].fb) continue;           /* (no CPU buffer was made: it stayed) */
        s->buf[i] = s->gbuf[i];
        s->buf[i].dirty[0] = 0; s->buf[i].dirty[1] = EPD_LINES;
    }
    s->gpu = s->gpu_lost;
    s->gpu_lost = NULL;
    s->gpu_dead = 0;
    s->gpu_full = 1;            /* every pixel's state up again, every target drawn whole */
    s->nrfd = 0; s->rfd_scan = 0;   /* (the hang's fault counts: from now) */
    if (s->stg.glok && s->stg.gl < 0) s->stg.gl = 1;
    if (s->link) gpu_clock_hold(s, 1);
    s->st.gpu_revives++;
    fprintf(stderr, "epd: GPU drive frames again (its timed-out frame finished)\n");
    if (epd_stream_gpe) {
        epd_gpu_gpe_upload(s->gpu, &s->e, (epd_rect){ 0, 0, EPD_W, EPD_H });
        gpe_start(s);
    }
}

/* GPU resets (seen: two recoveries in 8.7 h, each after a Chromium
 * fault on ring 2, each "offending task: glassine-epdd-desktop"). msm's recover skips the
 * submit at the head of the active ring and signals its fence as done: no GL
 * error, glFinish returns, so the frame that job drew went out garbled (or
 * the stagger's state pass was lost) and nothing here noticed. Tiles that
 * never healed and inverted rectangles until a full refresh followed. The
 * kernel charges the skipped submit to its process's VM: MSM_PARAM_FAULTS on
 * a drm file = global faults + that file's VM faults. Our contexts' files are
 * Mesa's own (EGL gives no fd), so they are found as the render nodes in
 * /proc/self/fd (rescanned every 2 s) and read once a GPU frame: a count that
 * rose = a job was dropped. On the A509 (no per-process page tables: one VM
 * for all) every file reads the same, device-wide count (device
 * epdd's 6 and sway's 8 render fds all 4 = 2 hangs x 2 recovers, each hang's
 * second recover "offending task: glassine-epdd-desktop"), so any process's GPU fault
 * means a full flash here; one where only the other's job went would not
 * need it (the recover replays the rest), but costs only a flash */
struct glassine_msm_param { uint32_t pipe, param; uint64_t value; };
#define GLASSINE_MSM_GET_PARAM DRM_IOWR(DRM_COMMAND_BASE + 0x00, struct glassine_msm_param)
static int fd_faults(int fd, uint64_t *v)
{
    struct glassine_msm_param p = { 0x10 /* MSM_PIPE_3D0 */, 0x09 /* MSM_PARAM_FAULTS */, 0 };
    if (ioctl(fd, GLASSINE_MSM_GET_PARAM, &p)) return -1;
    *v = p.value;
    return 0;
}

static void gpu_fd_scan(epd_stream *s)
{
    DIR *d = opendir("/proc/self/fd");
    if (!d) return;
    int fds[GPU_RFD_MAX], n = 0;
    uint64_t v[GPU_RFD_MAX];
    struct dirent *de;
    while ((de = readdir(d)) && n < GPU_RFD_MAX) {
        if (de->d_name[0] < '0' || de->d_name[0] > '9') continue;
        int fd = atoi(de->d_name);
        if (fd == dirfd(d)) continue;
        char p[48], l[64];
        snprintf(p, sizeof p, "/proc/self/fd/%d", fd);
        ssize_t k = readlink(p, l, sizeof l - 1);
        if (k <= 0) continue;
        l[k] = 0;
        uint64_t f;
        if (strncmp(l, "/dev/dri/renderD", 16) || fd_faults(fd, &f)) continue;
        int j = 0;
        while (j < s->nrfd && s->rfd[j] != fd) j++;
        /* known: its count stays (a rise since is the check's to see); new or
         * fallen (another file under the old number): from now */
        v[n] = j < s->nrfd && s->rfault[j] <= f ? s->rfault[j] : f;
        fds[n++] = fd;
    }
    closedir(d);
    memcpy(s->rfd, fds, sizeof(int) * n);
    memcpy(s->rfault, v, sizeof(uint64_t) * n);
    s->nrfd = n;
}

/* under the lock, after a frame the GPU rendered: 1 if one of our jobs was
 * dropped since the last look */
static int gpu_reset_seen(epd_stream *s)
{
    double t = now();
    if (!s->rfd_scan || t - s->rfd_scan > 2.0) { s->rfd_scan = t; gpu_fd_scan(s); }
    int hit = 0;
    for (int i = 0; i < s->nrfd; i++) {
        uint64_t f;
        if (fd_faults(s->rfd[i], &f)) continue;
        if (f > s->rfault[i]) {
            fprintf(stderr, "epd: GPU faults on fd %d: %llu -> %llu\n", s->rfd[i],
                    (unsigned long long)s->rfault[i], (unsigned long long)f);
            hit = 1;
        }
        s->rfault[i] = f;
    }
    return hit;
}

/* one of our GPU jobs was dropped: what the panel shows is unknown (a drive
 * frame may have gone out garbled; the stagger's or the gpe's state on the
 * GPU may miss a pass). The GPU stays (its context is fine after the
 * recover); everything it held is dropped, every pixel stale and assumed
 * white, and glassine-epdd-desktop flashes the whole screen from a fresh read
 * (epd_stream_levels_lost) */
static void stag_lost(epd_stream *s);
static void gpu_reset(epd_stream *s)
{
    epd_engine *e = &s->e;
    epd_job_join(e);                /* (before the state is replaced) */
    s->st.gpu_resets++;
    if (s->stg.state != STG_OFF) stag_lost(s);
    /* (the GPU context survives a recover, so the stagger comes back: before,
     * only gpu_revive (an abandon) undid stag_lost's gl = -1, and one app's
     * hang (10-29: Chromium's GPU process) left the A2 pass off for the rest
     * of the boot, scroll event age 13.6 -> 31 ms. At most 3 times a boot: a
     * reset our own pass keeps causing stays off) */
    static int nback;                   /* (a process's; a reexec starts over, as the stats do) */
    if (s->stg.glok && s->stg.gl < 0 && nback < 3) {
        s->stg.gl = 1;
        fprintf(stderr, "epd: staggered steps back after a GPU reset (%d of 3)\n", ++nback);
    }
    if (s->gpe) {
        gt_drop(s);
        epd_set_pixel_ops(e, NULL);
        e->defer = 0;
        s->gpe = 0;
        pthread_cond_broadcast(&s->gpe_cv);
        fprintf(stderr, "epd: gpe off (a GPU reset)\n");
    }
    epd_state_reset(e);         /* (busy 0, dc 0, stale 1: either claim_state) */
    epd_assume(e, 30);
    s->gpu_full = 1;
    for (int i = 0; i < NBUF; i++) { s->buf[i].dirty[0] = 0; s->buf[i].dirty[1] = EPD_LINES; }
    s->levels_lost = 1;
    fprintf(stderr, "epd: GPU reset: one of our GPU jobs was dropped, the panel is unknown: assumed white, a full flash follows\n");
}

/* epd_gpu_state, timed: the per-pixel state of r packed on the CPU and
 * uploaded (a full-screen run start and end each pack ~2.6 Mpx) */
static void gpu_state(epd_stream *s, epd_rect r)
{
    double t0 = now();
    epd_gpu_state(s->gpu, &s->e, r);
    double ms = (now() - t0) * 1e3;
    s->st.state_n++;
    s->st.state_px += (uint64_t)(r.w > 0 ? r.w : 0) * (uint64_t)(r.h > 0 ? r.h : 0);
    s->st.state_ms_sum += ms;
    if (ms > s->st.state_ms_max) s->st.state_ms_max = ms;
}

/* the running updates' ids, to find what epd_schedule starts */
static void run_ids(epd_stream *s, uint32_t *ids)
{
    for (int k = 0; k < EPD_MAX_UPD; k++) ids[k] = s->e.run[k].nframes ? s->e.run[k].id : 0;
}

/* frame into scanout buffer b on the GPU (after epd_schedule; `was` = the ids
 * before it). 0 = done; -1 = not done, the CPU must render it (GPU dropped) */
static int gpu_frame(epd_stream *s, int nb, const uint32_t *was)
{
    struct scan *b = &s->buf[nb];
    double tu = PF(s) ? now() : 0;
    if (!s->gpe) {              /* (gpe: the drive frame reads the engine state itself) */
        if (s->gpu_full) { gpu_state(s, (epd_rect){ 0, 0, EPD_W, EPD_H }); s->gpu_full = 0; }
        epd_rect st[2 * EPD_MAX_UPD];           /* started now: their pixels' state */
        int nst = epd_gpu_starts(s->gpu, &s->e, was, st);
        for (int k = 0; k < nst; k++) gpu_state(s, st[k]);
    }
    double t0 = now();
    PADD(s, DF_UPLOAD, (t0 - tu) * 1e3);
    epd_gpu_render(s->gpu, &s->e, s->gtarget[nb]);
    double tsub = PF(s) ? now() : 0;
    PADD(s, DF_SUBMIT, (tsub - t0) * 1e3);
    if (epd_stream_gpu_hang_test && (epd_stream_gpu_hang_test = 0, epd_gpu_finish(s->gpu, 0))) {
        gpu_abandon(s, "a test: gpu_hang_test");
        return -1;
    }
    int fin = epd_gpu_finish(s->gpu, GPU_HUNG_MS);
    PADD(s, DF_FENCE, (now() - tsub) * 1e3);
    if (fin) {
        gpu_abandon(s, "a frame not done after " STR_MS(GPU_HUNG_MS));
        return -1;              /* the CPU renders this frame, into a fresh buffer */
    }
    double ms = (now() - t0) * 1e3;
    s->st.gpu_ms_sum += ms;
    if (ms > s->st.gpu_ms_max) s->st.gpu_ms_max = ms;
    int err = epd_gpu_error(s->gpu);
    if (ms > GPU_LATE_MS) {
        s->st.gpu_late++;
        if (epd_stream_verbose) fprintf(stderr, "epd: GPU frame late: %.0f ms\n", ms);
    }
    s->gpu_slow = ms > GPU_LATE_MS ? s->gpu_slow + 1 : 0;
    if (err || s->gpu_slow >= GPU_SLOW_RUN) {
        char why[64];
        if (err) snprintf(why, sizeof why, "GL error 0x%x", err);
        else snprintf(why, sizeof why, "%d frames in a row took > " STR_MS(GPU_LATE_MS) " (last %.0f)", s->gpu_slow, ms);
        s->gpu_slow = 0;
        gpu_stop(s, why);
        return -1;          /* (this frame finished: the CPU renders it again, into the same buffer) */
    }
    if (epd_stream_gpu == 2) {                  /* check: the CPU's frame, compared */
        render(s);                              /* keeps the CPU frame current */
        if (s->st.gpu_frames % GPU_CHECK_EVERY == 0) {
            double c0 = now();                  /* check time = readback + compare */
            if (!s->gpu_words && !(s->gpu_words = malloc(sizeof(uint32_t) * EPD_WORDS))) return -1;
            scan_read(s, nb, s->gpu_words);
            for (int y = 0; y < EPD_LINES; y++)
                if (memcmp(s->gpu_words + (size_t)y * EPD_LINE, s->frame + (size_t)y * EPD_LINE, EPD_LINE * 4)) {
                    char why[64];
                    snprintf(why, sizeof why, "checked frame differs on wire line %d", y);
                    gpu_stop(s, why);
                    return -1;          /* the CPU's frame goes out instead (publish) */
                }
            s->st.gpu_checked++;
            double cm = (now() - c0) * 1e3;
            s->st.check_ms_sum += cm;
            if (cm > s->st.check_ms_max) s->st.check_ms_max = cm;
        }
    }
    b->dirty[0] = s->fdirty[0]; b->dirty[1] = s->fdirty[1];     /* (check mode: for the next compare) */
    s->st.gpu_frames++;
    return 0;
}

/* ---- the engine state on the GPU (epd_stream_gpe) ------------------------
 * The pixel ops run in the stream thread (epd_schedule / epd_advance, under
 * the lock), where the GL context is. A GPU that fails one: dropped
 * (gpu_stop: its state synced down first) or, hung, abandoned (gpu_abandon:
 * the state is lost, gpe_lost) — either way the CPU does that pass at once */
static int gpe_failed(epd_stream *s, int r, const char *what)
{
    char why[80];
    snprintf(why, sizeof why, "%s %s", what, r == -2 ? "not done after " STR_MS(GPU_HUNG_MS) : "failed");
    if (r == -2) gpu_abandon(s, why);
    else gpu_stop(s, why);
    return 1;
}

static uint64_t gpe_nchk;           /* check: claims since the gpe started (every GPE_CHECK_EVERY-th: gpe_compare) */

/* check: the GPU's whole state vs the CPU's arrays; 1 = differs (gpe off) */
static int gpe_compare(epd_stream *s)
{
    size_t n = (size_t)EPD_W * EPD_H;
    if (!s->gpe_chk && !(s->gpe_chk = malloc(n * 7))) return 0;
    uint8_t *lv = s->gpe_chk, *tg = lv + n, *bz = tg + n, *op = bz + n, *st = op + n;
    int16_t *dc = (int16_t *)(st + n);
    double t0 = now();
    if (epd_gpu_finish(s->gpu, GPU_HUNG_MS)) { gpe_failed(s, -2, "a state readback"); return 1; }
    epd_gpu_gpe_read(s->gpu, lv, tg, bz, op, dc, st);
    const epd_engine *e = &s->e;
    const struct { const char *name; const void *g, *c; size_t sz; } f[] = {
        { "level", lv, e->level, 1 }, { "busy", bz, e->busy, 1 }, { "stale", st, e->stale, 1 },
        { "dc", dc, e->dc, 2 }, { "op", op, e->op, 1 },
    };
    for (unsigned k = 0; k < sizeof f / sizeof f[0]; k++) {
        if (k == 4 && !e->op_ready) continue;           /* (no model yet: op undefined) */
        if (!memcmp(f[k].g, f[k].c, n * f[k].sz)) continue;
        size_t i = 0;
        while (!memcmp((const uint8_t *)f[k].g + i * f[k].sz, (const uint8_t *)f[k].c + i * f[k].sz, f[k].sz)) i++;
        char why[96];
        snprintf(why, sizeof why, "the state differs: %s at %zu,%zu", f[k].name, i % EPD_W, i / EPD_W);
        gpe_off(s, why);
        return 1;
    }
    for (size_t i = 0; i < n; i++)              /* target: only where in flight */
        if (e->busy[i] && tg[i] != e->target[i]) {
            char why[96];
            snprintf(why, sizeof why, "the state differs: target at %zu,%zu", i % EPD_W, i / EPD_W);
            gpe_off(s, why);
            return 1;
        }
    s->st.gpe_checked++;
    acc(&s->st.gpe_chk_ms_sum, &s->st.gpe_chk_ms_max, (now() - t0) * 1e3);
    return 0;
}

static int px_claim(void *ctx, epd_engine *e, epd_claim_plan *pl, epd_claim_out *out)
{
    epd_stream *s = ctx;
    epd_claim_out g;
    int lazy = pl->lazy;
    double t0 = now();
    int r = epd_gpu_gpe_claim(s->gpu, e, pl, s->gpe == 2 ? &g : out, GPU_HUNG_MS);
    if (r) return gpe_failed(s, r, "a claim");  /* the CPU claims (on synced / reset arrays) */
    acc(&s->st.gpe_ms_sum, &s->st.gpe_ms_max, (now() - t0) * 1e3);
    s->st.gpe_claims++;
    s->st.gpe_lazy += lazy;
    if (s->gpe == 2) {                          /* (a lazy plan has its slots now: the CPU takes the same) */
        epd_claim_pixels(e, pl, out);
        if (memcmp(&g, out, sizeof g)) {
            char why[96];
            snprintf(why, sizeof why, "a claim's report differs (rect %d,%d %dx%d)", pl->r.x, pl->r.y, pl->r.w, pl->r.h);
            gpe_off(s, why);
        } else if (++gpe_nchk % GPE_CHECK_EVERY == 0)
            gpe_compare(s);
    }
    return 0;
}

/* GLASSINE_GPE_ASYNC: the claim's passes started (the engine collects the report
 * at its settle: epd_set_defer). Check: the CPU's pass now, its report kept */
static int px_submit(void *ctx, epd_engine *e, epd_claim_plan *pl)
{
    epd_stream *s = ctx;
    double t0 = now();
    int r = epd_gpu_gpe_submit(s->gpu, e, pl, GPU_HUNG_MS);
    if (r) { epd_gpu_gpe_cancel(s->gpu); return gpe_failed(s, r, "a claim"); }
    acc(&s->st.gpe_ms_sum, &s->st.gpe_ms_max, (now() - t0) * 1e3);
    s->st.gpe_claims++;
    if (s->gpe == 2) epd_claim_pixels(e, pl, &s->gpe_cpu_out);
    return 0;
}

static int px_collect(void *ctx, epd_engine *e, epd_claim_out *out, int wait)
{
    epd_stream *s = ctx;
    (void)e;
    double t0 = now();
    int r = epd_gpu_gpe_collect(s->gpu, out, wait ? GPU_HUNG_MS : 0);
    if (r == 1) return 1;
    if (r) {                                    /* (the runs keep the claim's rect: harmless) */
        epd_gpu_gpe_cancel(s->gpu);
        if (!s->gpe_quiet) gpe_failed(s, r, "a claim's report");
        return -1;
    }
    acc(&s->st.gpe_col_ms_sum, &s->st.gpe_col_ms_max, (now() - t0) * 1e3);
    s->st.gpe_collects++;
    if (s->gpe == 2) {
        if (memcmp(&s->gpe_cpu_out, out, sizeof *out)) {
            *out = s->gpe_cpu_out;              /* (the CPU's arrays are the truth from here) */
            if (!s->gpe_quiet) gpe_off(s, "a deferred claim's report differs");
        } else if (++gpe_nchk % GPE_CHECK_EVERY == 0 && !s->gpe_quiet)
            gpe_compare(s);
    }
    return 0;
}

/* the deferred claim's report taken in before the gpe's state goes (a GPU
 * that fails it now: gpe_down / the caller finds that) */
static void gpe_settle(epd_stream *s)
{
    s->gpe_quiet = 1;
    epd_claim_settle(&s->e, 1);
    s->gpe_quiet = 0;
}

static int px_finish(void *ctx, epd_engine *e, uint64_t slots)
{
    epd_stream *s = ctx;
    epd_gpu_gpe_finish(s->gpu, e, slots);       /* (a failure shows at the next wait) */
    return s->gpe == 2;                         /* check: the CPU's arrays too */
}

static void px_query(void *ctx, epd_engine *e, const epd_pend *p, uint64_t *mask, int *nonbw)
{
    epd_stream *s = ctx;
    int r = epd_gpu_gpe_query(s->gpu, p, mask, nonbw, GPU_HUNG_MS);
    if (r) { gpe_failed(s, r, "a blocked test"); epd_pend_query(e, p, mask, nonbw); return; }
    s->st.gpe_queries++;
    if (s->gpe == 2) {
        uint64_t m;
        int nb;
        epd_pend_query(e, p, &m, &nb);
        if (m != *mask || nb != *nonbw) {
            char why[112];
            snprintf(why, sizeof why, "a blocked test differs (rect %d,%d %dx%d: %016llx vs %016llx)",
                     p->r.x, p->r.y, p->r.w, p->r.h, (unsigned long long)*mask, (unsigned long long)m);
            gpe_off(s, why);
            *mask = m; *nonbw = nb;
        }
    }
}

/* ---- stage 5: GPU update targets (the stream thread, under the lock) ---- */
static void gt_release(void *ctx, int gt)
{
    epd_stream *s = ctx;
    if (s->gt_pv) s->gt_pv->release(s->gt_pv->ctx, gt);
}

/* the engine's CPU paths need a GPU target's levels (check mode, a fallback):
 * read through epd-gpu; -1 (white is assumed) once the GPU is gone or hung */
static int gt_fetch(void *ctx, int gt, epd_rect r, uint8_t *out)
{
    epd_stream *s = ctx;
    if (!s->gpu || s->gpu_dead || !s->gt_on) return -1;
    int rr = epd_gpu_gpe_target_read(s->gpu, gt, r, out, GPU_HUNG_MS);
    if (rr) fprintf(stderr, "epd: a GPU target could not be read (%d): white assumed there\n", rr);
    return rr;
}

/* register the producer's layers with the gpe (gpe on, a provider set) */
static void gt_service(epd_stream *s)
{
    if (!s->gt_req || !s->gpe || !s->gpu || s->gpu_dead) return;
    s->gt_req = 0;
    if (s->gt_on) {             /* a new provider (or none): the old one's targets become CPU copies first */
        epd_pend_targets_cpu(&s->e);
        epd_gpu_gpe_provider(s->gpu, NULL);
        s->gt_on = 0;
    }
    s->tops =(epd_target_ops){ gt_release, gt_fetch, s };
    epd_set_target_ops(&s->e, &s->tops);
    s->gt_on = s->gt_pv && !epd_gpu_gpe_provider(s->gpu, s->gt_pv);
    fprintf(stderr, "epd: GPU update targets %s\n", s->gt_on ? "on" : s->gt_pv ? "OFF (the layers did not import)" : "off");
}

/* before the gpe or the GPU goes: every pending's GPU target becomes a CPU
 * copy (fetched while the GPU still reads) and its handle is released; the
 * next gpe start registers the layers again */
static void gt_drop(epd_stream *s)
{
    if (!s->gt_on) return;
    epd_pend_targets_cpu(&s->e);
    if (s->gpu && !s->gpu_dead) epd_gpu_gpe_provider(s->gpu, NULL);
    s->gt_on = 0;
    s->gt_req = s->gt_pv != NULL;
}

static void gpe_start(epd_stream *s)
{
    epd_job_join(&s->e);            /* (pixel ops end jobs) */
    if (epd_gpu_gpe_open(s->gpu, &s->e)) { fprintf(stderr, "epd: no engine state on the GPU (setup failed)\n"); return; }
    s->gpe = epd_stream_gpu == 2 ? 2 : epd_stream_gpe == 2 ? 2 : 1;
    /* GLASSINE_GPE_ASYNC=1: deferred claims (no wait for the claim's
     * report, collected at the next epd_schedule) */
    const char *as = getenv("GLASSINE_GPE_ASYNC");
    int async = as && atoi(as) > 0;
    s->px = (epd_pixel_ops){ px_claim, px_finish, px_query, s, async ? px_submit : NULL, async ? px_collect : NULL };
    epd_set_pixel_ops(&s->e, &s->px);
    epd_set_defer(&s->e, async);
    epd_gpu_gpe_drive(s->gpu, 1);
    fprintf(stderr, "epd: engine state on the GPU%s%s\n", s->gpe == 2 ? ", checked against the CPU" : "",
            async ? ", claims deferred" : "");
    gt_service(s);
}

/* the CPU's arrays := the GPU's state (stream thread); 0 ok, -1 hung (abandoned) */
static int gpe_down(epd_stream *s)
{
    epd_job_join(&s->e);
    gpe_settle(s);
    double t0 = now();
    if (epd_gpu_finish(s->gpu, GPU_HUNG_MS)) { gpe_failed(s, -2, "a state readback"); return -1; }
    epd_engine *e = &s->e;
    epd_gpu_gpe_read(s->gpu, e->level, e->target, e->busy, e->op, e->dc, e->stale);
    s->st.gpe_syncs++;
    acc(&s->st.gpe_sync_ms_sum, &s->st.gpe_sync_ms_max, (now() - t0) * 1e3);
    return 0;
}

/* the gpe ends, the GPU stays (drive frames from packed codes again) */
static void gpe_off(epd_stream *s, const char *why)
{
    fprintf(stderr, "epd: engine state on the GPU OFF (%s): the CPU's\n", why);
    gpe_settle(s);              /* (the deferred claim's report, while it can be read) */
    gt_drop(s);                 /* (the targets first: the GPU still reads them) */
    if (s->gpe == 1 && gpe_down(s)) return;     /* (hung: gpu_abandon did the rest) */
    epd_set_defer(&s->e, 0);
    epd_set_pixel_ops(&s->e, NULL);
    s->gpe = 0;
    epd_gpu_gpe_drive(s->gpu, 0);
    s->gpu_full = 1;            /* every code packed from the arrays */
    pthread_cond_broadcast(&s->gpe_cv);
}

/* hung with the state on it: what the panel shows is unknown. Nothing in
 * flight any more, every pixel stale and assumed white; glassine-epdd-desktop flashes the
 * whole screen (epd_stream_levels_lost) */
static void gpe_lost(epd_stream *s)
{
    epd_engine *e = &s->e;
    epd_job_join(e);
    epd_state_reset(e);
    epd_assume(e, 30);
    s->levels_lost = 1;
    fprintf(stderr, "epd: the panel's levels were on the hung GPU: assumed white, a full flash follows\n");
}

/* the stream thread, under the lock: what gpe_sync asked (UP before DOWN: a
 * writer's arrays go up before a later reader's come down) */
static void gpe_service(epd_stream *s)
{
    if (!s->gpe_req) return;
    if (s->gpe == 1 && s->gpu) {
        if (s->gpe_req & GPE_UP) epd_gpu_gpe_upload(s->gpu, &s->e, (epd_rect){ 0, 0, EPD_W, EPD_H });
        if ((s->gpe_req & GPE_DOWN) && s->gpe == 1) gpe_down(s);
    }
    s->gpe_req = 0;
    pthread_cond_broadcast(&s->gpe_cv);
}
#endif

/* under the lock, any thread but the stream's: the CPU's arrays current
 * (GPE_DOWN) / the GPU's state from them (GPE_UP), with the gpe on */
static void gpe_sync(epd_stream *s, int what)
{
#ifdef EPD_GPU
    if (s->gpe != 1) return;
    s->gpe_req |= what;
    pthread_cond_signal(&s->wake);
    /* timed: a GPU dropped in a frame (gpu_frame runs unlocked) turns the gpe
     * off without the lock, its broadcast may come between test and wait */
    while (s->gpe == 1 && (s->gpe_req & what)) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += 100 * 1000000L;
        if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
        pthread_cond_timedwait(&s->gpe_cv, &s->mu, &ts);
    }
#else
    (void)s; (void)what;
#endif
}

#ifdef EPD_GPU
/* ---- staggered cut steps (epd_stream_stagger) ---------------------------- */
static int stag_cut(int mode, int full)
{
    int b = EPD_MODE_BASE(mode), len = mode >> 8;
    return !full && len >= 2 && len - 1 <= EPD_STAG_NMAX && (b == EPD_A2 || b == EPD_A2_OR_DU);
}

static void rect_union(epd_rect *d, epd_rect r)
{
    if (d->w <= 0) { *d = r; return; }
    int x0 = d->x < r.x ? d->x : r.x, y0 = d->y < r.y ? d->y : r.y;
    int x1 = d->x + d->w > r.x + r.w ? d->x + d->w : r.x + r.w, y1 = d->y + d->h > r.y + r.h ? d->y + d->h : r.y + r.h;
    *d = (epd_rect){ x0, y0, x1 - x0, y1 - y0 };
}

static uint32_t stag_id(epd_stream *s)
{
    uint32_t id = s->e.next_id++;
    if (!s->e.next_id) s->e.next_id = 1;
    return id ? id : s->e.next_id++;
}

static int rect_in(epd_rect a, epd_rect b)     /* a inside b */
{
    return a.x >= b.x && a.y >= b.y && a.x + a.w <= b.x + b.w && a.y + a.h <= b.y + b.h;
}

/* under the lock: the held cut steps to the engine (an update per held rect,
 * the newest step's mode), disarmed */
static void stag_hold_flush(epd_stream *s)
{
    s->stg.arm = 0;
    for (int k = 0; k < s->stg.nhr; k++) {
        epd_rect r = s->stg.hr[k];
        uint8_t *t = epd_target_alloc((size_t)r.w * r.h);
        if (!t) continue;
        for (int y = 0; y < r.h; y++)
            memcpy(t + (size_t)y * r.w, s->stg.hold + (size_t)(r.y + y) * EPD_W + r.x, r.w);
        epd_submit_target(&s->e, r, s->stg.hmode, 0, t, s->stg.hgrey, s->stg.hstride);
        s->st.stag_flushed++;
    }
    s->stg.nhr = 0;
}

/* ---- lanes (stagger_state=lanes) ---- */
static int dclean_all = -1;     /* (GLASSINE_DCLEAN_ALL=1: lane_take cleans the whole box, the old way) */
static void ln_sync(int fd, unsigned fl, epd_rect r, size_t row)
{
    if (r.h > 0) epd_dmabuf_sync(fd, fl, (size_t)r.y * row, (size_t)r.h * row);
}

/* the queue full: one rect, the gap pixels' staged targets from T (S.tgt is
 * exact only inside the queued rects) */
static void lane_pq_merge(epd_stream *s)
{
    epd_rect u = { 0, 0, 0, 0 };
    for (int i = 0; i < s->stg.npq; i++) rect_union(&u, s->stg.pq[i]);
    const uint8_t *T = s->e.pk->T;
    for (int y = u.y; y < u.y + u.h; y++)
        for (int x = u.x; x < u.x + u.w; x++) {
            int in = 0;
            for (int i = 0; i < s->stg.npq && !in; i++) {
                epd_rect q = s->stg.pq[i];
                in = x >= q.x && x < q.x + q.w && y >= q.y && y < q.y + q.h;
            }
            if (!in) s->stg.S.tgt[(size_t)y * EPD_W + x] = T[(size_t)y * EPD_PK_S + x];
        }
    s->stg.pq[0] = u;
    s->stg.npq = 1;
}

/* under the lock: a cut step's targets over r (t, ts apart) armed: staged in
 * S.tgt over r padded to its 16-px lines (the pad keeps its own target),
 * S.ld marked, the rect queued for the upload. The engine's finished words
 * there resolved first, D's lines cleaned before any arm can be seen (the
 * CPU's until the GPU takes a lane) */
/* rows [y0, y1) of an arm (lane_take): resolve, stage, mark; what the rows
 * need of D's clean into o (the rows are a part's own: no shared write) */
struct lane_part { int dy0, dy1, resolved; uint64_t finarmed; };
static void lane_rows(epd_stream *s, epd_rect pr, epd_rect r, const uint8_t *t, size_t ts, int all, int y0, int y1, struct lane_part *o)
{
    epd_engine *e = &s->e;
    epd_pk *k = e->pk;
    *o = (struct lane_part){ EPD_H, 0, 0, 0 };
    /* (row by row: the pad keeps its own target, r takes t's;
     * the per-pixel form cost ~25 ms an arm over a full window) */
    int mx0 = r.x > pr.x ? r.x : pr.x, mx1 = r.x + r.w < pr.x + pr.w ? r.x + r.w : pr.x + pr.w;
    if (mx1 < mx0) mx1 = mx0;
    for (int y = y0; y < y1; y++) {
        uint8_t *tg = s->stg.S.tgt + (size_t)y * EPD_W, *ld = s->stg.S.ld + (size_t)y * EPD_W;
        const uint8_t *Tr = k->T + (size_t)y * EPD_PK_S, *tr = t + (size_t)(y - r.y) * ts;
        int fresh = memchr(ld + pr.x, 0, (size_t)pr.w) != NULL;
        /* (FIN words resolved on fresh rows only: an armed lane is the GPU's
         * and the engine finishes nothing there; the whole-box
         * resolve was 3.5 ms an arm) */
        if (fresh && epd_stag_lane_resolve(e, (epd_rect){ pr.x, y, pr.w, 1 }) > 0) o->resolved = 1;
        if (!fresh && s->stg.cok) {     /* (stagger=check: the rule above, counted; must stay 0) */
            const uint16_t *w = k->W + (size_t)y * EPD_PK_S;
            for (int x = pr.x; x < pr.x + pr.w; x++) o->finarmed += w[x] == EPD_PK_FIN;
        }
        for (int x = pr.x; x < mx0; x++) tg[x] = (uint8_t)(((ld[x] ? tg[x] : Tr[x]) & 31) | EPD_PK_ARM);
        for (int x = mx0; x < mx1; x++) tg[x] = (uint8_t)((tr[x - r.x] & 31) | EPD_PK_ARM);
        for (int x = mx1; x < pr.x + pr.w; x++) tg[x] = (uint8_t)(((ld[x] ? tg[x] : Tr[x]) & 31) | EPD_PK_ARM);
        memset(ld + pr.x, 1, (size_t)pr.w);
        if (s->stg.cok) memset(s->stg.C.ld + (size_t)y * EPD_W + pr.x, 1, (size_t)pr.w);
        if (fresh || all) { if (y < o->dy0) o->dy0 = y; o->dy1 = y + 1; }
    }
}

/* lane_take's rows on the render pool (a run's start: the held steps over a
 * whole window, ~20 ms on one CPU, memory-bound; ) */
#define LANE_PAR_ROWS 256       /* (an arm this tall or taller) */
static struct { epd_rect pr, r; const uint8_t *t; size_t ts; int all; struct lane_part p[NREND]; } lane_pj;
static void lane_job(epd_stream *s, int i)
{
    int n = s->jn, h = lane_pj.pr.h, y0 = lane_pj.pr.y + h * i / n, y1 = lane_pj.pr.y + h * (i + 1) / n;
    struct lane_part *q = &lane_pj.p[i];
    lane_rows(s, lane_pj.pr, lane_pj.r, lane_pj.t, lane_pj.ts, lane_pj.all, y0, y1, q);
    if (q->dy1 > q->dy0)        /* (D's clean of the part's own rows, here: ~4 ms on one CPU) */
        ln_sync(s->e.pk->dfd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE, ((epd_rect){ lane_pj.pr.x, q->dy0, lane_pj.pr.w, q->dy1 - q->dy0 }), EPD_PK_S * 4);
}

/* under the lock: a cut step's targets over r (t, ts apart) armed: staged in
 * S.tgt over r padded to its 16-px lines (the pad keeps its own target),
 * S.ld marked, the rect queued for the upload. The engine's finished words
 * there resolved first, D's lines cleaned before any arm can be seen (the
 * CPU's until the GPU takes a lane). par: tall ones split over the render
 * pool (stag_enter: the drive thread, no frame rendering) */
static void lane_take(epd_stream *s, epd_rect r, const uint8_t *t, size_t ts, int par)
{
    epd_rect pr = epd_stag_lane_pad(r);
    if (pr.w <= 0 || pr.h <= 0) return;
    epd_engine *e = &s->e;
    epd_pk *k = e->pk;
    if (dclean_all < 0) { const char *v = getenv("GLASSINE_DCLEAN_ALL"); dclean_all = v && atoi(v) == 1; }
    pk_quiet(s);
    /* D's lines cleaned only where the CPU may have written them since their
     * last clean: rows with a pixel not armed yet (an armed lane's D is the
     * GPU's; the CPU never writes it until the takeover; a resolve writes D
     * on fresh rows only), all of pr while an evict is out (the whole-box
     * clean was 3.07 ms a frame in a scroll) */
    int all = s->stg.evd || dclean_all;
    struct lane_part o;
    if (par && pr.h >= LANE_PAR_ROWS && epd_stream_nwork > 1) {
        /* (W's rows marked here, once: a part's resolve then only reads
         * wy0/wy1, and the END below pairs this START. Only with a fresh row:
         * a mid-run arm over armed rows resolves nothing, no W sync) */
        int fr = 0;
        for (int y = pr.y; y < pr.y + pr.h && !fr; y++) fr = memchr(s->stg.S.ld + (size_t)y * EPD_W + pr.x, 0, (size_t)pr.w) != NULL;
        if (fr) epd_pk_w_rows(e, pr.y, pr.y + pr.h);
        lane_pj.pr = pr; lane_pj.r = r; lane_pj.t = t; lane_pj.ts = ts; lane_pj.all = all;
        pool_run(s, lane_job);
        o = (struct lane_part){ EPD_H, 0, fr, 0 };
        int dn = 0;
        for (int i = 0; i < s->jn; i++) {
            dn += lane_pj.p[i].dy1 > lane_pj.p[i].dy0;
            o.finarmed += lane_pj.p[i].finarmed;
        }
        if (!dn) s->st.stag_lane_dskip++;       /* (D cleaned by the parts: o.dy stays empty) */
        s->st.stag_lane_par++;
    } else {
        lane_rows(s, pr, r, t, ts, all, pr.y, pr.y + pr.h, &o);
        if (o.dy1 > o.dy0) ln_sync(k->dfd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE, ((epd_rect){ pr.x, o.dy0, pr.w, o.dy1 - o.dy0 }), EPD_PK_S * 4);
        else s->st.stag_lane_dskip++;
    }
    s->st.stag_lane_finarmed += o.finarmed;
    if (o.resolved) epd_pk_w_sync(e);
    rect_union(&s->stg.ldbox, pr);
    if (s->stg.npq == STG_PQ) lane_pq_merge(s);
    s->stg.pq[s->stg.npq++] = pr;
    s->st.stag_lane_arms++;
}

/* under the lock, between frames: the queued arms into T */
static void lane_pq_flush(epd_stream *s)
{
    epd_pk *k = s->e.pk;
    for (int i = 0; i < s->stg.npq; i++) {
        epd_rect r = s->stg.pq[i];
        ln_sync(k->tfd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE, r, EPD_PK_S);
        for (int y = r.y; y < r.y + r.h; y++)
            memcpy(k->T + (size_t)y * EPD_PK_S + r.x, s->stg.S.tgt + (size_t)y * EPD_W + r.x, (size_t)r.w);
        ln_sync(k->tfd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE, r, EPD_PK_S);
        epd_gpu_stag_target(s->gpu, &s->stg.S, r);
        if (s->stg.cok) epd_stag_lane_mirror(&s->stg.C, &s->e, r);
        if (s->stg.cok && s->stg.c4) bp4_lane_pack(&s->stg.C4, k->T, r);
    }
    s->stg.npq = 0;
}

/* the run end's row work (evict, takeover) on the render pool, a box this
 * tall or taller (on one CPU ~15 ms the evict, ~22 ms the
 * takeover over a full window); the parts own their rows (T, S.tgt, S.ld) and
 * sync their own ranges */
static struct { epd_rect b; int take; } lane_xj;
static int lane_xpar = -1;      /* (GLASSINE_LANE_XPAR=0: the run end on one CPU, the old way; mid-run arms too) */
static __thread int on_drive;   /* (the drive thread: pool_run is its own; a direct-path submitter never runs it) */
static int lane_xpar_on(void)
{
    if (lane_xpar < 0) { const char *v = getenv("GLASSINE_LANE_XPAR"); lane_xpar = !v || atoi(v) != 0; }
    return lane_xpar && on_drive;
}
static void lane_xrows(epd_stream *s, epd_rect b)
{
    epd_pk *k = s->e.pk;
    if (lane_xj.take) {                 /* (D's lines the GPU wrote, invalidated once: nothing of the CPU's is dirty there) */
        ln_sync(k->dfd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ, b, EPD_PK_S * 4);
        ln_sync(k->tfd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE, b, EPD_PK_S);
        epd_stag_lane_takeover(&s->e, b, s->stg.S.ld);
        ln_sync(k->tfd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE, b, EPD_PK_S);
        ln_sync(k->dfd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ, b, EPD_PK_S * 4);
    } else {
        ln_sync(k->tfd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE, b, EPD_PK_S);
        b = epd_stag_lane_evict(&s->e, b, s->stg.S.ack);
        ln_sync(k->tfd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE, b, EPD_PK_S);
    }
    for (int y = b.y; y < b.y + b.h; y++)          /* (S.tgt follows T: the self-check steps from it) */
        memcpy(s->stg.S.tgt + (size_t)y * EPD_W + b.x, k->T + (size_t)y * EPD_PK_S + b.x, (size_t)b.w);
}
static void lane_xjob(epd_stream *s, int i)
{
    epd_rect b = lane_xj.b;
    int y0 = b.y + b.h * i / s->jn, y1 = b.y + b.h * (i + 1) / s->jn;
    if (y1 > y0) lane_xrows(s, (epd_rect){ b.x, y0, b.w, y1 - y0 });
}
static void lane_xrun(epd_stream *s, epd_rect b, int take)
{
    lane_xj.b = b;
    lane_xj.take = take;
    if (b.h >= LANE_PAR_ROWS && epd_stream_nwork > 1 && lane_xpar_on()) {
        pool_run(s, lane_xjob);
        s->st.stag_lane_xpar++;
    } else
        lane_xrows(s, b);
}

/* under the lock, between frames (the run's end): the run's box evicted, each
 * word's tog the other of its last ack */
static void lane_evict(epd_stream *s)
{
    lane_pq_flush(s);
    s->stg.evd = 1;
    s->stg.evf = 0;
    epd_rect b = s->stg.ldbox;
    s->stg.evr = b;
    if (b.w <= 0 || b.h <= 0) return;
    double t0 = now();
    epd_gpu_stag_acks(s->gpu, s->stg.S.ack, b.y, b.y + b.h);
    epd_rect pr = epd_stag_lane_pad(b);  /* (what the evict writes: the pad keeps whole words) */
    lane_xrun(s, pr, 0);
    s->stg.evr = pr;
    epd_gpu_stag_target(s->gpu, &s->stg.S, pr);
    if (s->stg.cok) epd_stag_lane_mirror(&s->stg.C, &s->e, pr);
    if (s->stg.cok && s->stg.c4) bp4_lane_pack(&s->stg.C4, s->e.pk->T, pr);
    s->st.stag_evicts++;
    acc(&s->st.stag_evict_ms_sum, &s->st.stag_evict_ms_max, (now() - t0) * 1e3);
}

/* under the lock, after a frame's fence: the evict's words all acked */
static int lane_acked(epd_stream *s)
{
    epd_rect b = s->stg.evr;
    if (b.w <= 0 || b.h <= 0) return 1;
    epd_gpu_stag_acks(s->gpu, s->stg.S.ack, b.y, b.y + b.h);
    return !epd_stag_lane_unacked(&s->e, s->stg.S.ack, b);
}

/* under the lock (a submitter): a cut step's target t over r into the
 * stagger (its id; t freed), or 0: the engine takes it. While armed (the
 * stagger waits for the engine to get idle) cut steps are held; anything
 * else gives them to the engine first. Anything else submitted while the
 * stagger runs ends it (the drain) */
static uint32_t stag_take(epd_stream *s, epd_rect r, int mode, int full, uint8_t *t, const uint8_t *grey, int stride)
{
    int cut = epd_stream_stagger && stag_cut(mode, full);
    if (!cut) {
        if (s->stg.arm) stag_hold_flush(s);
        if (s->stg.state == STG_ON) {
            s->stg.state = STG_DRAIN; s->stg.drain_left = s->stg.nmax; s->st.stag_end_other++;
            /* (most runs end this way mid-scroll; which update does it) */
            fprintf(stderr, "epd: stagger run ended by an update: mode base %d len %d%s over %d,%d %dx%d\n",
                    EPD_MODE_BASE(mode), mode >> 8, full ? " full" : "", r.x, r.y, r.w, r.h);
        }
        return 0;
    }
    if (s->stg.state == STG_OFF) {
        if (!s->gpu || s->gpe || s->stg.gl < 0) return 0;
        /* a scroll's first cut: the engine at once (no entry's wait before the
         * first response); it arms, the next ones are held until the engine is
         * idle. An arm older than the linger is a new scroll's first cut again */
        double tn = now();
        if (!s->stg.arm || (!s->stg.nhr && tn - s->stg.arm_t > epd_stream_stag_linger / 1e3)) {
            s->stg.arm = 1;
            s->stg.arm_t = tn;
            return 0;
        }
        if (!s->stg.hold && !(s->stg.hold = malloc((size_t)EPD_W * EPD_H))) return 0;
        int k = 0;
        while (k < s->stg.nhr && !rect_in(r, s->stg.hr[k])) k++;
        if (k == s->stg.nhr) {                  /* new: it replaces the ones inside it */
            int m = 0;
            for (int j = 0; j < s->stg.nhr; j++)
                if (!rect_in(s->stg.hr[j], r)) s->stg.hr[m++] = s->stg.hr[j];
            s->stg.nhr = m;
            if (m == STG_HOLD) stag_hold_flush(s);
            s->stg.hr[s->stg.nhr++] = r;
        }
        s->stg.arm = 1;
        for (int y = 0; y < r.h; y++)
            memcpy(s->stg.hold + (size_t)(r.y + y) * EPD_W + r.x, t + (size_t)y * r.w, r.w);
        epd_target_free(t);
        s->stg.hmode = mode;
        s->stg.hgrey = grey;
        s->stg.hstride = stride;
        s->stg.hn = (mode >> 8) - 1;
        s->st.stag_held++;
        return stag_id(s);
    }
    if (s->stg.state != STG_ON) return 0;   /* (draining: the engine queues it, then goes on) */
    if (s->stg.lanes) {                     /* (lanes: armed, T at the next upload; a tall one on the
                                               pool from the drive thread's inbox, ~3.9 ms on one CPU) */
        lane_take(s, r, t, (size_t)r.w, lane_xpar_on());
        epd_target_free(t);
        s->stg.n = (mode >> 8) - 1;
        s->stg.quiet = 0;
        s->st.stag_subs++;
        return stag_id(s);
    }
    pk_quiet(s);
    if (epd_stag_load(&s->stg.S, &s->e, &s->stg.p, r)) {      /* (the engine's state is frozen meanwhile) */
        rect_union(&s->stg.lrect, r);
        rect_union(&s->stg.ldbox, r);
    }
    epd_stag_target(&s->stg.S, r, t);
    if (s->stg.cok) {                   /* (C takes them now, the GPU at the next upload: */
        epd_stag_load(&s->stg.C, &s->e, &s->stg.p, r);       /* C renders there too) */
        epd_stag_target(&s->stg.C, r, t);
    }
    epd_target_free(t);
    rect_union(&s->stg.dirty, r);
    s->stg.n = (mode >> 8) - 1;
    s->stg.quiet = 0;
    s->st.stag_subs++;
    return stag_id(s);
}

/* the phase groups of a run's n-frame steps: n + 1 (A2/1: 1 for n 1) */
static int stag_G(const epd_stream *s, int n) { return s->stg.a21 && n == 1 ? 1 : n + 1; }

/* the frame's parameters (stag_frame; stag_check_render: the same) */
static epd_stag_par stag_par_now(const epd_stream *s)
{
    epd_stag_par p = s->stg.p;
    p.n = p.ctl == 4 ? 1 : s->stg.n;   /* (ctl 4: one-frame steps, G 2; epd-bp4.h) */
    p.G = stag_G(s, p.n);
    p.drain = s->stg.state == STG_DRAIN && !s->stg.lanes;  /* (lanes: the evict ends the lanes, word by word) */
    p.lanes = s->stg.lanes;
    return p;
}

/* stagger=check: C = the CPU's state at the entry (the GPU's is the same: its
 * enter parks every pixel and takes S's targets and loads) */
static void stag_check_enter(epd_stream *s)
{
    s->stg.cok = 0;
    if (!epd_stream_stag_check || s->stg.cgl < 0) return;
    if (!s->stg.cgl) {
        if (epd_stag_init(&s->stg.C) || epd_stag_init(&s->stg.X) || !(s->stg.cw = malloc(sizeof(uint32_t) * EPD_WORDS)) ||
            !(s->stg.gw = malloc(sizeof(uint32_t) * EPD_WORDS)) || !(s->stg.xw = malloc(sizeof(uint32_t) * EPD_WORDS))) {
            fprintf(stderr, "epd: stagger check: no memory\n");
            s->stg.cgl = -1;
            return;
        }
        s->stg.cgl = 1;
    }
    size_t np = (size_t)EPD_W * EPD_H;
    if (s->stg.lanes) {                 /* (lanes: every lane parked (the GPU's too), C's cold and targets from D and T; the queued arms at the upload) */
        epd_stag_lanes_park(&s->stg.C);
        epd_gpu_stag_acks(s->gpu, s->stg.C.ack, 0, EPD_H);
        epd_dmabuf_sync(s->e.pk->dfd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ, 0, 0);
        epd_stag_lane_mirror(&s->stg.C, &s->e, (epd_rect){ 0, 0, EPD_W, EPD_H });
        epd_dmabuf_sync(s->e.pk->dfd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ, 0, 0);
        s->stg.c4 = 0;
        if (s->stg.p.ctl == 4) {        /* (ctl 4: the twin, every lane parked, its requests from T) */
            if (!s->stg.dsh && (bp4_alloc(&s->stg.C4) ||
                                !(s->stg.dsh = malloc(sizeof(uint32_t) * EPD_PK_S * (EPD_H + EPD_PK_DPAD))))) {
                fprintf(stderr, "epd: stagger check: no memory (ctl 4)\n");
                return;
            }
            s->stg.C4.kidle = 1;           /* (ctl 4's idle: ctl 3's per-pixel K) */
            bp4_lane_park(&s->stg.C4);
            bp4_lane_pack(&s->stg.C4, s->e.pk->T, (epd_rect){ 0, 0, EPD_W, EPD_H });
            memset(s->stg.dsh, 0xff, sizeof(uint32_t) * EPD_PK_S * (EPD_H + EPD_PK_DPAD));
            s->stg.c4 = 1;
        }
    } else {
        memcpy(s->stg.C.hot, s->stg.S.hot, np * sizeof *s->stg.C.hot);
        memcpy(s->stg.C.cold, s->stg.S.cold, np * sizeof *s->stg.C.cold);
        memcpy(s->stg.C.tgt, s->stg.S.tgt, np);
    }
    memcpy(s->stg.C.ld, s->stg.S.ld, np);
    s->stg.C.f = s->stg.S.f;
    memcpy(s->stg.cw, s->e.tpl, sizeof(uint32_t) * EPD_WORDS);
    memcpy(s->stg.xw, s->e.tpl, sizeof(uint32_t) * EPD_WORDS);
    s->stg.xread = 0;
    s->stg.f0 = s->stg.S.f;
    s->stg.cok = 1;
    s->stg.cread = 0;
    s->stg.clog = 0;
    s->st.stag_chk_runs++;
}

static void stag_check_log(epd_stream *s, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void stag_check_log(epd_stream *s, const char *fmt, ...)
{
    if (s->stg.clog++ >= STG_CHECK_LOG) return;
    va_list a;
    va_start(a, fmt);
    vfprintf(stderr, fmt, a);
    va_end(a);
}

/* under the lock, right after stag_upload (the GPU's inputs for this frame
 * are C's now): the CPU's frame over the loaded rows, C stepped */
static void stag_check_render(epd_stream *s)
{
    if (!s->stg.cok) return;
    epd_rect b = s->stg.cbox = s->stg.ldbox;
    s->stg.xread = 0;
    if (s->stg.c4) {                    /* ctl 4: the twin's frame (no self-step: the planes are not read back) */
        const epd_stag_par *p = &s->stg.fp;
        int mk = p->stripe << 8 ^ p->cellw << 4 ^ p->cellh << 20 ^ p->cellord << 28 ^ p->blocks << 2;
        if (!s->stg.M4.gm || mk != s->stg.m4k) {
            bp_masks_free(&s->stg.M4);
            if (bp_masks_make(&s->stg.M4, p)) { fprintf(stderr, "epd: stagger check: no memory (ctl 4)\n"); s->stg.cok = 0; return; }
            s->stg.m4k = mk;
        }
        if (b.w > 0) {
            epd_pk *k = s->e.pk;
            epd_dmabuf_sync(k->dfd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ, 0, 0);
            bp4_lane_rows(&s->stg.C4, &s->stg.M4, p, bp4_vm(p), s->stg.S.f, s->e.tpl, s->stg.cw, b.y, b.y + b.h, k->D, s->stg.dsh, s->stg.C.ack);
            epd_dmabuf_sync(k->dfd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ, 0, 0);
        }
        epd_stag_advance(&s->stg.C);
        return;
    }
    uint32_t k = s->stg.C.f - s->stg.f0;  /* frames since the entry: 0 = its state as loaded */
    if (b.w > 0 && (k < 4 || k == 7 || (s->stg.C.f + 1) % STG_SELF_EVERY == 0)) {
        size_t np = (size_t)EPD_W * EPD_H;
        epd_stag *X = &s->stg.X;
        epd_gpu_stag_read(s->gpu, X, (epd_rect){ 0, 0, EPD_W, EPD_H }, 1);   /* (the uploads are in: the GPU's inputs) */
        if (s->stg.lanes) memcpy(X->tgt, s->stg.C.tgt, np);     /* (lanes: T, as C mirrors it) */
        else memcpy(X->tgt, s->stg.S.tgt, np);
        memcpy(X->ld, s->stg.C.ld, np);
        X->f = s->stg.C.f;
        /* the state drift against the reference, before this frame */
        long n = 0, nl = 0, nop = 0, ndc = 0, nh = 0;
        int ex = -1, ey = -1;
        for (int y = b.y; y < b.y + b.h; y++)
            for (int x = b.x; x < b.x + b.w; x++) {
                size_t o = (size_t)y * EPD_W + x;
                if (!s->stg.C.ld[o]) continue;
                uint32_t g = X->cold[o], c = s->stg.C.cold[o];
                int hd = X->hot[o] != s->stg.C.hot[o];
                if (g == c && !hd) continue;
                n++; nh += hd;
                nl += EPD_STAG_LEVEL(g) != EPD_STAG_LEVEL(c);
                nop += EPD_STAG_OP(g) != EPD_STAG_OP(c);
                ndc += EPD_STAG_DC(g) != EPD_STAG_DC(c);
                if (ex < 0) { ex = x; ey = y; }
            }
        if (n) {
            s->st.stag_chk_drift++;
            size_t o = (size_t)ey * EPD_W + ex;
            fprintf(stderr, "epd: stagger self-check: frame %u (%u since the entry): the GPU's state differs from the reference's in %ld pixels "
                    "(level %ld, op %ld, dc %ld, hot %ld); first %d,%d: GPU cold %08x hot %04x, CPU cold %08x hot %04x, tgt %d\n",
                    s->stg.C.f, k, n, nl, nop, ndc, nh, ex, ey, X->cold[o], X->hot[o], s->stg.C.cold[o], s->stg.C.hot[o], s->stg.S.tgt[o]);
        } else if (k < 4 || k == 7)
            fprintf(stderr, "epd: stagger self-check: frame %u (%u since the entry): state equal (%d,%d %dx%d)\n",
                    s->stg.C.f, k, b.x, b.y, b.w, b.h);
        epd_stag_rows(X, &s->stg.fp, s->e.tpl, s->stg.xw, b.y, b.y + b.h);
        s->stg.xread = 1;
    }
    if (b.w > 0) epd_stag_rows(&s->stg.C, &s->stg.fp, s->e.tpl, s->stg.cw, b.y, b.y + b.h);
    epd_stag_advance(&s->stg.C);
}

static int stag_phase(const uint32_t *w, int x, int y)
{
    size_t wo = (size_t)(EPD_DATA0 / EPD_LINE + EPD_H - 1 - y) * EPD_LINE + EPD_DATA0 % EPD_LINE + x / 8;
    return (int)((w[wo] >> (14 - 2 * (x % 8))) & 3);
}

/* under the lock (stag_advance): this frame's GPU words against the CPU's */
static void stag_check_frame(epd_stream *s)
{
    if (!s->stg.cok || !s->stg.cread) return;
    s->stg.cread = 0;
    s->st.stag_chk_frames++;
    epd_rect b = s->stg.cbox;
    if (s->stg.xread) {                 /* the self-step: the GPU's frame from its own state */
        s->stg.xread = 0;
        s->st.stag_chk_self++;
        long xn = 0;
        int xx = -1, xy = -1, xg = 0, xc = 0;
        for (int y = b.y; y < b.y + b.h; y++)
            for (int x = b.x; x < b.x + b.w; x++) {
                int g = stag_phase(s->stg.gw, x, y), c = stag_phase(s->stg.xw, x, y);
                if (g == c) continue;
                if (!xn++) { xx = x; xy = y; xg = g; xc = c; }
            }
        if (xn) {
            s->st.stag_chk_self_bad++;
            fprintf(stderr, "epd: stagger self-check: frame %u: the GPU's frame differs from a CPU step of ITS OWN state in %ld pixels "
                    "(first %d,%d: GPU %d, CPU %d; state cold %08x hot %04x tgt %d)\n", s->stg.C.f - 1, xn, xx, xy, xg, xc,
                    s->stg.X.cold[(size_t)xy * EPD_W + xx], s->stg.X.hot[(size_t)xy * EPD_W + xx], s->stg.S.tgt[(size_t)xy * EPD_W + xx]);
        } else if (epd_stream_verbose)
            fprintf(stderr, "epd: stagger self-check: frame %u: the GPU's frame = a CPU step of its own state\n", s->stg.C.f - 1);
    }
    if (s->stg.c4 && b.w > 0) {         /* (ctl 4: the GPU's acks against the twin's) */
        epd_gpu_stag_acks(s->gpu, s->stg.X.ack, b.y, b.y + b.h);
        long na = 0;
        size_t fa = 0;
        for (size_t i = (size_t)b.y * BP_GW; i < (size_t)(b.y + b.h) * BP_GW; i++)
            if (s->stg.X.ack[i] != s->stg.C.ack[i]) { if (!na) fa = i; na++; }
        if (na) {
            s->st.stag_chk_frames_bad++;
            stag_check_log(s, "epd: stagger check: frame %u: ctl 4: %ld ack bytes differ (first byte %zu row %zu: GPU %d, twin %d)\n",
                           s->stg.C.f - 1, na, fa % BP_GW, fa / BP_GW, s->stg.X.ack[fa], s->stg.C.ack[fa]);
        }
    }
    long n = 0;
    int ex = -1, ey = -1, eg = 0, ec = 0;
    for (int y = b.y; y < b.y + b.h; y++)
        for (int x = b.x; x < b.x + b.w; x++) {
            int g = stag_phase(s->stg.gw, x, y), c = stag_phase(s->stg.cw, x, y);
            if (g == c) continue;
            if (!n++) { ex = x; ey = y; eg = g; ec = c; }
        }
    if (!n) return;
    s->st.stag_chk_frames_bad++;
    s->st.stag_chk_px_bad += (uint64_t)n;
    uint32_t c = s->stg.C.cold[(size_t)ey * EPD_W + ex];
    stag_check_log(s, "epd: stagger check: frame %u: %ld pixels' phases differ (first %d,%d: GPU %d, CPU %d; "
                   "CPU state level %d op %d dc %d tgt %d)\n", s->stg.C.f - 1, n, ex, ey, eg, ec,
                   EPD_STAG_LEVEL(c), EPD_STAG_OP(c), EPD_STAG_DC(c), s->stg.C.tgt[(size_t)ey * EPD_W + ex]);
}

/* under the lock (stag_exit at ctl 4, before the takeover): the D the GPU
 * evicted against the twin's (the pixels the twin evicted this run) */
static void stag_check_exit4(epd_stream *s, epd_rect b)
{
    s->stg.cok = 0;
    epd_pk *k = s->e.pk;
    long n = 0, ne = 0;
    size_t fo = 0;
    epd_dmabuf_sync(k->dfd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ, 0, 0);
    for (int y = b.y; y < b.y + b.h; y++)
        for (int x = b.x; x < b.x + b.w; x++) {
            size_t o = (size_t)y * EPD_PK_S + x;
            if (s->stg.dsh[o] == 0xffffffffu) continue;
            ne++;
            if (k->D[o] != s->stg.dsh[o]) { if (!n) fo = o; n++; }
        }
    uint32_t g = k->D[fo];
    epd_dmabuf_sync(k->dfd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ, 0, 0);
    if (!n) {
        if (epd_stream_verbose) fprintf(stderr, "epd: stagger check: ctl 4 hand-back D equal (%ld evicted pixels, %d,%d %dx%d)\n", ne, b.x, b.y, b.w, b.h);
        return;
    }
    s->st.stag_chk_state_bad++;
    s->stg.clog = 0;                    /* (always said) */
    stag_check_log(s, "epd: stagger check: ctl 4 HAND-BACK D DIFFERS: %ld of %ld evicted pixels in %d,%d %dx%d; first %zu,%zu: GPU %08x, twin %08x\n",
                   n, ne, b.x, b.y, b.w, b.h, fo % EPD_PK_S, fo / EPD_PK_S, g, s->stg.dsh[fo]);
}

/* under the lock (stag_exit, after the GPU's state was read back into S):
 * the loaded pixels' state against C's */
static void stag_check_exit(epd_stream *s)
{
    if (!s->stg.cok) return;
    s->stg.cok = 0;
    epd_rect b = s->stg.ldbox;
    long n = 0, nl = 0, nop = 0, ndc = 0, nld = 0;
    int ex = -1, ey = -1;
    for (int y = b.y; y < b.y + b.h; y++)
        for (int x = b.x; x < b.x + b.w; x++) {
            size_t o = (size_t)y * EPD_W + x;
            if (!s->stg.C.ld[o] != !s->stg.S.ld[o]) nld++;
            if (!s->stg.C.ld[o]) continue;
            uint32_t g = s->stg.S.cold[o], c = s->stg.C.cold[o];
            if (g == c) continue;
            n++;
            nl += EPD_STAG_LEVEL(g) != EPD_STAG_LEVEL(c);
            nop += EPD_STAG_OP(g) != EPD_STAG_OP(c);
            ndc += EPD_STAG_DC(g) != EPD_STAG_DC(c);
            if (ex < 0) { ex = x; ey = y; }
        }
    if (!n && !nld) {
        if (epd_stream_verbose) fprintf(stderr, "epd: stagger check: hand-back state equal (%d,%d %dx%d)\n", b.x, b.y, b.w, b.h);
        return;
    }
    s->st.stag_chk_state_bad++;
    uint32_t g = ex >= 0 ? s->stg.S.cold[(size_t)ey * EPD_W + ex] : 0, c = ex >= 0 ? s->stg.C.cold[(size_t)ey * EPD_W + ex] : 0;
    /* read again: agreeing now = the read was wrong (taken: the hand-back
     * healed), still differing = the GPU's state is */
    long ra = 0;
    epd_gpu_stag_read(s->gpu, &s->stg.X, b, 0);
    for (int y = b.y; y < b.y + b.h; y++)
        for (int x = b.x; x < b.x + b.w; x++) {
            size_t o = (size_t)y * EPD_W + x;
            if (!s->stg.C.ld[o] || s->stg.S.cold[o] == s->stg.C.cold[o]) continue;
            ra += s->stg.X.cold[o] == s->stg.C.cold[o];
        }
    if (ra == n)
        for (int y = b.y; y < b.y + b.h; y++)
            memcpy(s->stg.S.cold + (size_t)y * EPD_W + b.x, s->stg.X.cold + (size_t)y * EPD_W + b.x, (size_t)b.w * sizeof *s->stg.S.cold);
    s->stg.clog = 0;                    /* (always said) */
    stag_check_log(s, "epd: stagger check: HAND-BACK STATE DIFFERS: %ld pixels (level %ld, op %ld, dc %ld; loaded sets %ld) in %d,%d %dx%d; "
                   "first %d,%d: GPU level %d op %d dc %d, CPU level %d op %d dc %d; a re-read agrees in %ld of them%s\n", n, nl, nop, ndc, nld,
                   b.x, b.y, b.w, b.h, ex, ey, EPD_STAG_LEVEL(g), EPD_STAG_OP(g), EPD_STAG_DC(g), EPD_STAG_LEVEL(c), EPD_STAG_OP(c), EPD_STAG_DC(c),
                   ra, ra == n ? " (the read was wrong: the re-read taken)" : "");
}

/* the GPU's load of r (S.ld 2 -> 1); stagger=check: then whether it landed:
 * the taken pixels' flagged values in the upload texture, and their cold
 * state read back against S's (the entry's loads did not land) */
static void stag_gpu_load(epd_stream *s, epd_rect r, const char *what)
{
    if (!s->stg.cok) {
        epd_gpu_stag_load(s->gpu, &s->stg.S, &s->stg.p, r);
        return;
    }
    if (r.x < 0) { r.w += r.x; r.x = 0; }
    if (r.y < 0) { r.h += r.y; r.y = 0; }
    if (r.x + r.w > EPD_W) r.w = EPD_W - r.x;
    if (r.y + r.h > EPD_H) r.h = EPD_H - r.y;
    if (r.w <= 0 || r.h <= 0) return;
    epd_rect wr = { r.x & ~7, r.y, ((r.x + r.w + 7) & ~7) - (r.x & ~7), r.h };   /* (the load takes whole words) */
    uint8_t *ld = s->stg.X.ld;          /* (scratch: the self-check fills it later) */
    for (int y = wr.y; y < wr.y + wr.h; y++)
        memcpy(ld + (size_t)y * EPD_W + wr.x, s->stg.S.ld + (size_t)y * EPD_W + wr.x, (size_t)wr.w);
    epd_gpu_stag_load(s->gpu, &s->stg.S, &s->stg.p, r);
    long n = 0;
    for (int y = wr.y; y < wr.y + wr.h; y++)
        for (int x = wr.x; x < wr.x + wr.w; x++) n += ld[(size_t)y * EPD_W + x] == 2;
    if (!n) return;
    long up = epd_gpu_stag_load_probe(s->gpu, wr, ld);
    epd_gpu_stag_read(s->gpu, &s->stg.X, wr, 0);
    long ok = 0;
    int ex = -1, ey = -1;
    for (int y = wr.y; y < wr.y + wr.h; y++)
        for (int x = wr.x; x < wr.x + wr.w; x++) {
            size_t o = (size_t)y * EPD_W + x;
            if (ld[o] != 2) continue;
            if (s->stg.X.cold[o] == s->stg.S.cold[o]) ok++;
            else if (ex < 0) { ex = x; ey = y; }
        }
    s->st.stag_chk_loads++;
    if (ok == n) {
        if (epd_stream_verbose) fprintf(stderr, "epd: stagger check: %s load: %ld pixels, all landed\n", what, n);
        return;
    }
    s->st.stag_chk_loads_bad++;
    size_t o = (size_t)ey * EPD_W + ex;
    s->stg.clog = 0;                    /* (always said) */
    stag_check_log(s, "epd: stagger check: %s LOAD DID NOT LAND: %ld of %ld pixels wrong in C (%d,%d %dx%d), the upload holds %ld of them "
                   "(upload GL error %#x); first %d,%d: GPU cold %08x, CPU %08x\n", what, n - ok, n, wr.x, wr.y, wr.w, wr.h, up,
                   epd_gpu_stag_load_err(s->gpu), ex, ey, s->stg.X.cold[o], s->stg.S.cold[o]);
}

/* the stream thread, under the lock, the engine idle: its state into the stagger */
static void stag_enter(epd_stream *s)
{
    double t0 = now();
    if (!s->stg.gl) {
        if (epd_stag_init(&s->stg.S) || epd_gpu_stag_open(s->gpu)) {
            fprintf(stderr, "epd: staggered steps not available (GLES 3.1 / memory): the engine does the steps\n");
            s->stg.gl = -1;
            stag_hold_flush(s);
            return;
        }
        s->stg.gl = 1;
        s->stg.glok = 1;
        s->stg.lanes = epd_gpu_stag_lanes(s->gpu);
        if (s->stg.lanes) fprintf(stderr, "epd: staggered steps: stagger_state=lanes (the A2 pass on the engine's D and T)\n");
    }
    int pcap = epd_stream_stag_cap < 0 ? s->e.matched_cap : epd_stream_stag_cap;
    /* .dither 0: the stagger dithers nothing : the commit owns the
     * dither (glassine-epdd-desktop commit, dither= / dither_pattern=) and hands it black
     * and white; a grey that still arrives (dither=off: C_BW keeps a grey's
     * old value) is thresholded at 16 */
    s->stg.p = (epd_stag_par){ .G = 2, .n = 1, .ctl = epd_stream_stag_ctl, .pay = epd_stream_stag_pay, .K = epd_stream_stag_k, .sidle = epd_stream_stag_sidle,
                               .dither = 0, .R = s->e.op_r ? s->e.op_r : 9, .payrule = epd_stream_stag_payrule != 0,
                               .pcap = pcap <= 0 || pcap > EPD_STAG_PCAP ? EPD_STAG_PCAP : pcap, .htop = epd_stream_stag_htop,
                               .stripe = epd_stream_stag_stripe, .blocks = epd_stream_stag_blocks,
                               .cellw = epd_stream_stag_cellw, .cellh = epd_stream_stag_cellh,
                               .cellord = epd_stream_stag_cellord };
    if (s->stg.p.ctl > 4) s->stg.p.ctl = 4;
    if (s->stg.p.ctl == 4 && !(s->stg.lanes && bp4_par_ok(&s->stg.p) && !epd_gpu_bp4_open(s->gpu))) {
        if (!s->stg.c4warn) fprintf(stderr, "epd: stagger_ctl 4 needs stagger_state=lanes, stagger_pay=panel, a cap <= 3 and a "
                                    "64-row layout (stripes, cells w x 1, blue noise) and the GPU's programs: ctl 3 runs\n");
        s->stg.c4warn = 1;
        s->stg.p.ctl = 3;
    }
    if ((s->stg.p.ctl == 4) != (s->stg.lctl == 4)) s->stg.lrepark = 1;    /* (the other state: parked, Q packed from T) */
    s->stg.lctl = s->stg.p.ctl;
    s->stg.a21 = epd_stream_stag_a21 && s->stg.p.ctl == 3;
    if (s->stg.lanes) {                 /* lanes: the held steps armed; nothing moves */
        if (s->stg.lrepark) { epd_gpu_stag_lanes_park(s->gpu); s->stg.lrepark = 0; }
        s->stg.n = 1;
        s->stg.ldbox = (epd_rect){ 0, 0, 0, 0 };
        s->stg.npq = 0;
        s->stg.evd = 0;
        s->stg.sync0 = epd_dmabuf_sync_s;
        s->stg.cok = 0;                 /* (lane_take marks C.ld once check is on) */
        for (int k = 0; k < s->stg.nhr; k++) {
            epd_rect h = s->stg.hr[k];
            lane_take(s, h, s->stg.hold + (size_t)h.y * EPD_W + h.x, EPD_W, 1);
            s->stg.n = s->stg.hn;
        }
        s->st.stag_enter_cpu_ms_sum += (now() - t0) * 1e3;
        s->stg.nhr = 0;
        s->stg.arm = 0;
        stag_check_enter(s);            /* (C.ld from S.ld: the arms above) */
        s->stg.lrect = (epd_rect){ 0, 0, 0, 0 };
        s->stg.state = STG_ON;
        s->stg.dirty = (epd_rect){ 0, 0, 0, 0 };
        s->stg.quiet = 0;
        s->stg.nmax = s->stg.n + (s->stg.p.ctl >= 2);
        double ms = (now() - t0) * 1e3;
        s->st.stag_entries++;
        acc(&s->st.stag_enter_ms_sum, &s->st.stag_enter_ms_max, ms);
        if (epd_stream_verbose) fprintf(stderr, "epd: staggered steps on, lanes (%.2f ms)\n", ms);
        return;
    }
    /* every pixel parked; the held steps' rect loaded (the engine's state) and
     * targeted: the run's first targets */
    if (s->stg.parked) epd_stag_repark(&s->stg.S, s->stg.ldbox);
    else epd_stag_enter(&s->stg.S);
    s->stg.parked = 1;
    s->stg.n = 1;
    s->stg.ldbox = (epd_rect){ 0, 0, 0, 0 };
    if (s->stg.nhr) pk_quiet(s);
    for (int k = 0; k < s->stg.nhr; k++) {
        epd_rect h = s->stg.hr[k];
        epd_stag_load(&s->stg.S, &s->e, &s->stg.p, h);
        for (int y = 0; y < h.h; y++)
            memcpy(s->stg.S.tgt + (size_t)(h.y + y) * EPD_W + h.x, s->stg.hold + (size_t)(h.y + y) * EPD_W + h.x, h.w);
        s->stg.n = s->stg.hn;
        rect_union(&s->stg.ldbox, h);
    }
    s->st.stag_enter_cpu_ms_sum += (now() - t0) * 1e3;
    s->stg.nhr = 0;
    s->stg.arm = 0;
    stag_check_enter(s);                /* (before the GPU's load turns S.ld 2 -> 1) */
    epd_gpu_stag_enter(s->gpu, &s->stg.S);
    if (s->stg.ldbox.w > 0) stag_gpu_load(s, s->stg.ldbox, "entry");     /* (only ld 2 taken) */
    s->stg.lrect = (epd_rect){ 0, 0, 0, 0 };
    s->stg.state = STG_ON;
    s->stg.dirty = (epd_rect){ 0, 0, 0, 0 };
    s->stg.quiet = 0;
    s->stg.nmax = s->stg.n + (s->stg.p.ctl >= 2);   /* (ctl 2, 3: a step may last G = n + 1 frames) */
    double ms = (now() - t0) * 1e3;
    s->st.stag_entries++;
    acc(&s->st.stag_enter_ms_sum, &s->st.stag_enter_ms_max, ms);
    if (epd_stream_verbose) fprintf(stderr, "epd: staggered steps on (%.1f ms)\n", ms);
}

/* under the lock: back to the engine (no step running) */
static void lane_exit(epd_stream *s)
{
    double t0 = now();
    lane_pq_flush(s);                   /* (none: a drain takes no new arms; a quit's exit) */
    epd_rect b = s->stg.evd ? s->stg.evr : epd_stag_lane_pad(s->stg.ldbox);
    if (b.w > 0 && b.h > 0) {
        pk_quiet(s);
        if (s->stg.cok && s->stg.c4) stag_check_exit4(s, b);
        else if (s->stg.cok) {          /* (stagger=check: D's armed pixels against C's, before the takeover) */
            epd_gpu_stag_read(s->gpu, &s->stg.S, b, 0);
            stag_check_exit(s);
        }
        lane_xrun(s, b, 1);
        epd_gpu_stag_target(s->gpu, &s->stg.S, b);
    }
    s->stg.cok = 0;
    s->stg.state = STG_OFF;
    s->stg.evd = 0;
    s->stg.ldbox = (epd_rect){ 0, 0, 0, 0 };
    s->gpu_full = 1;
    double ms = (now() - t0) * 1e3, sy = (epd_dmabuf_sync_s - s->stg.sync0) * 1e3;
    s->st.stag_exits++;
    s->st.stag_lane_ack_frames += (uint64_t)s->stg.evf;
    s->st.stag_lane_sync_ms_sum += sy;
    acc(&s->st.stag_exit_ms_sum, &s->st.stag_exit_ms_max, ms);
    if (s->st.stag_exits <= 5 || s->st.stag_exits % 20 == 0 || epd_stream_verbose)
        fprintf(stderr, "epd: staggered steps off, lanes: run %llu, %d,%d %dx%d evicted, acked after %d frames, takeover %.2f ms, "
                "the run's syncs %.2f ms (%s)\n", (unsigned long long)s->st.stag_exits, b.x, b.y, b.w, b.h, s->stg.evf, ms, sy,
                epd_dmabuf_partial == 1 ? "range" : "whole buffer");
}

static void stag_exit(epd_stream *s)
{
    if (s->stg.lanes) { lane_exit(s); return; }
    double t0 = now();
    epd_gpu_stag_read(s->gpu, &s->stg.S, s->stg.ldbox, 0);     /* (only loaded pixels go back) */
    s->st.stag_exit_read_ms_sum += (now() - t0) * 1e3;
    stag_check_exit(s);
    pk_quiet(s);
    {                                   /* the run's DC: each loaded pixel's record before (the engine's, frozen meanwhile) and after */
        epd_rect b = s->stg.ldbox;
        uint64_t n = 0, di = 0, dq = 0;
        for (int y = b.y; y < b.y + b.h; y++)
            for (int x = b.x; x < b.x + b.w; x++) {
                size_t o = (size_t)y * EPD_W + x;
                if (!s->stg.S.ld[o]) continue;
                int a = epd_px_dc(&s->e, o), c = EPD_STAG_DC(s->stg.S.cold[o]);
                n++;
                di += (uint64_t)(a < 0 ? -a : a);
                dq += (uint64_t)(c < 0 ? -c : c);
            }
        s->st.stag_dc_px += n;
        s->st.stag_dc_in += di;
        s->st.stag_dc_out += dq;
    }
    epd_stag_exit(&s->stg.S, &s->e);
    s->stg.state = STG_OFF;
    s->gpu_full = 1;
    double ms = (now() - t0) * 1e3;
    s->st.stag_exits++;
    acc(&s->st.stag_exit_ms_sum, &s->st.stag_exit_ms_max, ms);
    if (epd_stream_verbose) fprintf(stderr, "epd: staggered steps off (%.1f ms)\n", ms);
}

/* under the lock: the GPU went with the stagger's state (dropped or hung):
 * the loaded pixels' targets taken as reached, stale (the run end's cleanup
 * fixes what they are not); parked ones were never touched */
static void stag_lost(epd_stream *s)
{
    epd_engine *e = &s->e;
    pk_quiet(s);
    for (size_t i = 0; i < (size_t)EPD_W * EPD_H; i++)
        if (s->stg.S.ld[i])
            epd_px_put(e, i, EPD_F_LEVEL | EPD_F_TARGET | EPD_F_STALE, s->stg.S.tgt[i] & 31, s->stg.S.tgt[i] & 31, 0, 0, 0, 1);
    if (s->stg.lanes) {                 /* (lanes: no request left in T, every lane the CPU's; H re-parked at the next entry) */
        epd_pk *k = e->pk;
        epd_dmabuf_sync(k->tfd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE, 0, 0);
        for (size_t q = 0; q < (size_t)EPD_PK_S * EPD_H; q++) k->T[q] &= 31;
        epd_dmabuf_sync(k->tfd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE, 0, 0);
        for (size_t i = 0; i < (size_t)EPD_W * EPD_H; i++) { s->stg.S.ld[i] = 0; s->stg.S.tgt[i] &= 31; }
        s->stg.npq = 0;
        s->stg.evd = 0;
        s->stg.ldbox = (epd_rect){ 0, 0, 0, 0 };
        s->stg.lrepark = 1;
    }
    epd_state_fill(e, EPD_F_BUSY, 0);
    e->op_ready = 0;
    s->stg.state = STG_OFF;
    s->stg.gl = -1;
    s->stg.cok = 0;
    fprintf(stderr, "epd: the GPU went during staggered steps: their targets taken as reached\n");
}

/* under the lock, before a stagger frame: the changed targets up */
static void stag_upload(epd_stream *s)
{
    if (s->stg.lanes) {                 /* lanes: the queued arms into T; the run's end: its evict */
        lane_pq_flush(s);
        if (s->stg.state == STG_DRAIN && !s->stg.evd) lane_evict(s);
        s->stg.fp = stag_par_now(s);
        stag_check_render(s);
        return;
    }
    if (s->stg.lrect.w > 0) {
        stag_gpu_load(s, s->stg.lrect, "run");
        s->stg.lrect.w = 0;
    }
    if (s->stg.dirty.w > 0) {
        epd_gpu_stag_target(s->gpu, &s->stg.S, s->stg.dirty);
        s->stg.dirty.w = 0;
    }
    s->stg.fp = stag_par_now(s);        /* (stag_frame runs unlocked: a submit meanwhile waits for the next frame) */
    stag_check_render(s);
}

/* unlocked: the stagger's next frame into buffer nb. 0 = done; -1 = the GPU
 * is gone (stag_lost after relocking; the CPU renders the engine's frame) */
static int stag_frame(epd_stream *s, int nb)
{
    epd_stag_par p = s->stg.fp;
    if (!p.drain && p.n + (p.ctl >= 2) > s->stg.nmax) s->stg.nmax = p.n + (p.ctl >= 2);
    double t0 = now();
    if (epd_stream_stag_direct >= 0) epd_gpu_stag_set_direct(s->gpu, epd_stream_stag_direct);
    epd_gpu_stag_frame(s->gpu, &p, s->stg.S.f, s->gtarget[nb]);
    double tsub = now();
    acc(&s->st.stag_sub_ms_sum, &s->st.stag_sub_ms_max, (tsub - t0) * 1e3);
    PADD(s, DF_SUBMIT, (tsub - t0) * 1e3);
    if (epd_stream_gpu_hang_test && (epd_stream_gpu_hang_test = 0, epd_gpu_finish(s->gpu, 0))) {
        gpu_abandon(s, "a test in staggered steps: gpu_hang_test");
        return -1;
    }
    int fin = epd_gpu_finish(s->gpu, GPU_HUNG_MS);
    PADD(s, DF_FENCE, (now() - tsub) * 1e3);
    if (fin) {
        gpu_abandon(s, "a staggered frame not done after " STR_MS(GPU_HUNG_MS));
        return -1;
    }
    int err = epd_gpu_error(s->gpu);
    if (err) {
        char why[48];
        snprintf(why, sizeof why, "GL error 0x%x in a staggered frame", err);
        gpu_stop(s, why);
        return -1;
    }
    double ms = (now() - t0) * 1e3;
    if (s->stg.cok && (s->stg.C.f % STG_CHECK_EVERY == 0 || s->stg.xread)) {     /* (C.f: this frame's + 1) */
        scan_read(s, nb, s->stg.gw);
        s->stg.cread = 1;
    }
    s->st.stag_frames++;
    acc(&s->st.stag_ms_sum, &s->st.stag_ms_max, ms);
    s->buf[nb].dirty[0] = 0;            /* (the whole data area: a CPU frame after it clears it) */
    s->buf[nb].dirty[1] = EPD_LINES;
    return 0;
}

/* under the lock, the frame rendered: the stagger's frame count on; ON ->
 * DRAIN when the engine has something queued, the targets stopped changing
 * or the stream quits; a drain done -> the engine */
static void stag_advance(epd_stream *s)
{
    stag_check_frame(s);
    s->stg.S.f++;
    s->stg.quiet++;
    if (s->stg.state == STG_ON) {
        int n = s->stg.p.ctl == 4 ? 1 : s->stg.n, G = stag_G(s, n);
        /* level 1: after the last change every pixel took its step (its group's
         * frame within G, then n frames); the controller steps on until the
         * model is at the rail (R / n steps, G apart), payback after it */
        int bound = s->stg.p.ctl ? G * ((s->stg.p.R + n - 1) / n + 2) + (s->stg.p.payrule ? s->stg.p.pcap * G : s->stg.p.pay ? 4 * G : 0) : G + s->stg.nmax;
        int linger = epd_stream_stag_linger * 85 / 1000;  /* (frames: 85 Hz) */
        if (bound < linger) bound = linger;
        if (s->stg.quiet > bound) s->st.stag_end_quiet++;
        if (s->e.npend || s->quit || s->stg.quiet > bound || !epd_stream_stagger) {
            s->stg.state = STG_DRAIN;
            s->stg.drain_left = s->stg.p.ctl || s->e.npend || s->quit ? s->stg.nmax : 0;
        }
    } else if (s->stg.lanes) {          /* lanes: the evict's acks (after this frame's fence); none after STG_ACK_MAX: taken over */
        if (s->stg.evd && (lane_acked(s) || ++s->stg.evf > STG_ACK_MAX)) {
            if (s->stg.evf > STG_ACK_MAX) {
                s->st.stag_lane_ack_late++;
                fprintf(stderr, "epd: staggered steps: no acks %d frames after the evict: taken over\n", STG_ACK_MAX);
            }
            stag_exit(s);
        }
        return;
    } else
        s->stg.drain_left--;
    if (s->stg.state == STG_DRAIN && s->stg.drain_left <= 0) stag_exit(s);
}
#endif

/* Pipeline: frame k+1 is rendered and copied while frame k is on the wire,
 * and flipped the moment k's flip event arrives. The event comes at the end
 * of the transfer (~11.06 ms into the 11.76 ms period at 85 Hz), so anything
 * done between the event and the flip must fit in ~0.7 ms or the frame waits
 * a whole period. Three buffers: the one on the wire, the one sent before it,
 * and the one being prepared. */
/* Busy = frames queued or on the wire. While busy the stream holds the wakeup
 * source "glassine-epdd-desktop", so a suspender that does the wakeup_count handshake
 * (glassine-powerd) waits for the panel instead of freezing us mid-update: the
 * sleep banner was frozen in the queue and drawn at resume. The lock has a
 * timeout (a crash must not keep the device awake forever) and is re-armed
 * while busy. No CONFIG_PM_WAKELOCKS (no /sys/power/wake_lock): nothing. */
#define WL_NAME "glassine-epdd-desktop"
#define WL_TIMEOUT "5000000000"     /* ns; re-armed every WL_REARM_S while busy */
#define WL_REARM_S 1.0

static void set_busy(epd_stream *s, int b)   /* under s->mu */
{
    if (s->wl_fd >= 0) {
        double t = now();
        if (b && (!s->busy || t - s->wl_armed > WL_REARM_S)) {
            if (write(s->wl_fd, WL_NAME " " WL_TIMEOUT, sizeof WL_NAME " " WL_TIMEOUT - 1) < 0) {}
            s->wl_armed = t;
        } else if (!b && s->busy && s->wul_fd >= 0) {
            if (write(s->wul_fd, WL_NAME, sizeof WL_NAME - 1) < 0) {}
        }
    }
    s->busy = b;
    if (!b) pthread_cond_broadcast(&s->idle);
}

/* jit_size: the row span of the next frame's work (under s->mu) — the runs
 * still going and the updates pending (render and copy go by whole lines) */
static int jit_rows(const epd_engine *e)
{
    int y0 = EPD_H, y1 = 0;
    for (int k = 0; k < EPD_MAX_UPD; k++)
        if (e->run[k].nframes) {
            if (e->run[k].r.y < y0) y0 = e->run[k].r.y;
            if (e->run[k].r.y + e->run[k].r.h > y1) y1 = e->run[k].r.y + e->run[k].r.h;
        }
    for (int k = 0; k < e->npend; k++) {
        if (e->pend[k].r.y < y0) y0 = e->pend[k].r.y;
        if (e->pend[k].r.y + e->pend[k].r.h > y1) y1 = e->pend[k].r.y + e->pend[k].r.h;
    }
    return y1 > y0 ? y1 - y0 : 0;
}

/* ---- the hand-off inbox -------------------------
 * The submitters (glassine-epdd-desktop's main thread, holding sway's frame callback, and
 * the pen thread) took s->mu for stag_take + epd_submit_target, and the drive
 * thread holds it from the top of its loop through epd_schedule (9-12 ms a
 * frame at 85 Hz: the claims) and epd_advance: the wait was 10-16 ms per
 * hand-off, up to ~80, on a GNOME Software scroll (device) — all
 * of it input -> ink latency and sway frames not read. Now a submit, and every
 * engine knob the policy sets per update (cut, temp, ...), is a message in a
 * ring: appended under imu (held for a ~100-byte copy), applied by the drive
 * thread in order at the top of its loop, right before epd_schedule. That is
 * where a lock-path submit became visible as well (the drive thread held mu
 * from the loop top through the schedule; submits landed only in its unlocked
 * stretches: render, flip wait, link work, sleep), so every epd_schedule sees
 * the same set of updates as before, in the same order, and renders the same
 * frames. The merge/supersede step (epd_submit_target reads the submitter's
 * grey for a union's content) now runs in the drive thread: it reads the
 * shadow while the main thread may be writing it — the same pattern as
 * stag_hold_flush's hgrey, and within the engine's contract (the union takes
 * "the newest image"; bytes are whole, a torn row is a mix of two images,
 * and every write to the shadow gets its own submit).
 * Waking a sleeping drive thread without taking mu: the thread sets
 * `asleep` (under mu) before its last look at the inbox and then waits (which
 * lets go of mu); a submitter appends, then tries mu (trylock): it gets it
 * once the thread waits (signal), or while the thread runs unlocked (a
 * harmless signal); when the thread holds mu it is awake and drains at its
 * loop top — unless asleep is set, i.e. it is between its last look and the
 * wait: then the submitter yields until trylock succeeds. The two stores
 * (count, asleep) and two loads are sequentially consistent, so one side
 * always sees the other: no lost wakeup.
 * epd_stream_free_ms (the policy, every loop iteration) reads a view the
 * drive thread publishes after every engine change (seqlock, no mutex: a
 * preempted reader must not hold up the SCHED_FIFO thread), plus the messages
 * not applied yet. The engine's rare readers (DC map, stats, save/load,
 * assume, raw) still take mu. epd_stream_inbox = 0: the old direct path, the
 * same apply_msg under mu in the submitter (a device A/B). */
static long long now_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1000000000LL + t.tv_nsec;
}

static int inbox_pending(const epd_stream *s)
{
    return __atomic_load_n(&s->icount, __ATOMIC_SEQ_CST);
}

/* the engine's view published (under mu, the drive thread) */
static void view_publish(epd_stream *s)
{
    struct eng_view *v = s->view;
    const epd_engine *e = &s->e;
    __atomic_store_n(&s->vseq, s->vseq + 1, __ATOMIC_RELAXED);     /* odd: being written */
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    v->nrun = 0;
    for (int k = 0; k < EPD_MAX_UPD; k++) {
        const epd_run *u = &e->run[k];
        if (!u->nframes || u->payback) continue;
        v->run[v->nrun].r = u->r;
        v->run[v->nrun].left = u->nframes - u->frame;
        v->nrun++;
    }
    v->npend = 0;
    for (int i = 0; i < e->npend; i++) {
        const epd_pend *p = &e->pend[i];
        if (p->kept) continue;
        v->pend[v->npend].r = p->r;
        v->pend[v->npend].frames = epd_pend_frames_est(e, p->mode, p->full);
        v->npend++;
    }
    for (int f = 0; f < 2; f++)
        for (int m = 0; m < 16; m++) v->est[f][m] = epd_pend_frames_est(e, m, f);
    v->stag_on = 0;
#ifdef EPD_GPU
    v->stag_on = s->stg.state == STG_ON;
    __atomic_store_n(&s->gt_ok, s->gt_on && s->stg.state == STG_OFF, __ATOMIC_RELEASE);
#endif
    v->hz = s->mode.vrefresh ? s->mode.vrefresh : 85;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    __atomic_store_n(&s->vseq, s->vseq + 1, __ATOMIC_RELAXED);     /* even: done */
}

static void view_read(const epd_stream *s, struct eng_view *v)
{
    for (;;) {
        unsigned a = __atomic_load_n(&s->vseq, __ATOMIC_SEQ_CST);
        if (a & 1) { sched_yield(); continue; }
        memcpy(v, s->view, sizeof *v);
        __atomic_thread_fence(__ATOMIC_SEQ_CST);
        if (__atomic_load_n(&s->vseq, __ATOMIC_SEQ_CST) == a) return;
    }
}

static int rects_overlap(epd_rect a, epd_rect b)
{
    return a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
}

static int rect_clip(epd_rect *r)
{
    if (r->x < 0) { r->w += r->x; r->x = 0; }
    if (r->y < 0) { r->h += r->y; r->y = 0; }
    if (r->x + r->w > EPD_W) r->w = EPD_W - r->x;
    if (r->y + r->h > EPD_H) r->h = EPD_H - r->y;
    return r->w > 0 && r->h > 0;
}

/* epd_frames_until_free on the view, the inbox's submits as pendings */
static int view_frames_free(const struct eng_view *v, epd_rect r, const struct inmsg *in, int nin)
{
    if (!rect_clip(&r)) return 0;
    if (v->stag_on) return 0;
    int n = 0;
    for (int k = 0; k < v->nrun; k++)
        if (rects_overlap(v->run[k].r, r) && v->run[k].left > n) n = v->run[k].left;
    for (int i = 0; i < v->npend + nin; i++) {
        epd_rect q;
        int frames;
        if (i < v->npend) { q = v->pend[i].r; frames = v->pend[i].frames; }
        else {
            const struct inmsg *m = &in[i - v->npend];
            if (m->kind != IN_SUBMIT && m->kind != IN_SUBMIT_GT) continue;
            q = m->r;
            if (!rect_clip(&q)) continue;
            int b = EPD_MODE_BASE(m->mode), len = m->mode >> 8;
            if (b == EPD_A2_OR_DU) b = EPD_A2;
            frames = b == EPD_A2 && len > 0 ? len : b >= 0 && b < 16 ? v->est[m->full != 0][b] : 10;
        }
        if (!rects_overlap(q, r)) continue;
        int b = 0;
        for (int k = 0; k < v->nrun; k++)
            if (rects_overlap(v->run[k].r, q) && v->run[k].left > b) b = v->run[k].left;
        if (b + frames > n) n = b + frames;
    }
    return n;
}

/* a submitter: the drive thread out of its wait (see above; never blocks on mu) */
static void wake_drive(epd_stream *s)
{
    for (;;) {
        if (!pthread_mutex_trylock(&s->mu)) {
            pthread_cond_signal(&s->wake);
            pthread_mutex_unlock(&s->mu);
            return;
        }
        if (!__atomic_load_n(&s->asleep, __ATOMIC_SEQ_CST)) return;     /* awake: it drains at its loop top */
        sched_yield();
    }
}

/* the drive thread's wait on `wake` (under mu): not with messages in the inbox */
static void drive_wait(epd_stream *s, const struct timespec *ts)
{
    __atomic_store_n(&s->asleep, 1, __ATOMIC_SEQ_CST);
    if (!inbox_pending(s) && !__atomic_load_n(&s->bdone, __ATOMIC_SEQ_CST)) {   /* (a job done: apply it first) */
        if (ts) pthread_cond_timedwait(&s->wake, &s->mu, ts);
        else pthread_cond_wait(&s->wake, &s->mu);
    }
    __atomic_store_n(&s->asleep, 0, __ATOMIC_SEQ_CST);
}

static void set_temp_now(epd_stream *s, int c)   /* under mu */
{
    int was = epd_wbf_temp_index(&s->e.wbf, s->e.celsius);
    s->e.celsius = c;           /* used by updates that start from now on */
    if (epd_wbf_temp_index(&s->e.wbf, c) != was) log_temp_range(s, c);
}

/* one message into the engine (under mu; the drive thread, or the submitter
 * on the direct path). A submit's id: the engine's (direct), the message's
 * (inbox); 0 = dropped */
static void prof_apply(epd_stream *s, struct inmsg *m);
static void prof_track(epd_stream *s, uint32_t id, int stag, const struct inmsg *m);
static uint32_t apply_msg(epd_stream *s, struct inmsg *m)
{
    int took_stag = 0;
    uint32_t id = 0;
    double t0 = now();
    switch (m->kind) {
    case IN_SUBMIT:
#ifdef EPD_GPU
        id = stag_take(s, m->r, m->mode, m->full, m->t, m->grey, m->stride);
        took_stag = id != 0;
#endif
        if (!id) id = epd_submit_target(&s->e, m->r, m->mode, m->full, m->t, m->grey, m->stride);
        break;
    case IN_SUBMIT_GT:
#ifdef EPD_GPU
        if (s->gt_on && s->stg.state == STG_OFF) {
            id = epd_submit_gt(&s->e, m->r, m->mode, m->full, m->gt, m->grey, m->stride);
            s->st.gt_subs++;
            if (!id) id = EPD_GT_DROPPED;   /* taken, nothing to do (the handle released by the engine) */
        } else {
            /* taken on a stale flag (the gpe or the layers went, a stagger run
             * began since): the CPU copy from the submitter's grey instead, the
             * handle back to the provider it came from */
            uint8_t *t = epd_target_make(&m->r, m->grey, m->stride);
            if (m->pv && m->pv->release) m->pv->release(m->pv->ctx, m->gt);
            s->st.gt_fallback++;
            if (t) {
                id = stag_take(s, m->r, m->mode, m->full, t, m->grey, m->stride);
                if (!id) id = epd_submit_target(&s->e, m->r, m->mode, m->full, t, m->grey, m->stride);
            }
        }
#endif
        break;
    case IN_CUT: epd_set_cut(&s->e, m->a); break;
    case IN_TEMP: set_temp_now(s, m->a); break;
    case IN_MATCHED: epd_set_matched(&s->e, m->a); break;
    case IN_MATCHED_RULE: epd_set_matched_rule(&s->e, m->a, m->b); break;
    case IN_NONBLOCK: epd_set_nonblock(&s->e, m->a); break;
    case IN_MODEL: epd_set_model(&s->e, m->a, m->b); break;
    case IN_HEAL: epd_set_heal(&s->e, m->a, m->b); break;
    case IN_HEAL_LEN: epd_set_heal_len(&s->e, m->a); break;
    case IN_SNAP: epd_set_snap(&s->e, m->a); break;
    case IN_PROF: prof_apply(s, m); break;
    case IN_KEEPALIVE: {
        /* (a message, not a bare atomic store + wake: only the inbox count is
         * in the sleep check's Dekker pairing — a store the drive thread read
         * just before its untimed wait, with wake_drive's trylock failing on
         * the still-held mu, would have been missed until the next submit) */
        long long was = __atomic_load_n(&s->hold_ns, __ATOMIC_RELAXED);
        if (m->ns > was) __atomic_store_n(&s->hold_ns, m->ns, __ATOMIC_RELEASE);
        break; }
    }
    if (m->kind == IN_SUBMIT || m->kind == IN_SUBMIT_GT) {
        double tgt = (now() - t0) * 1e3;
        if (id && id != EPD_GT_DROPPED) { s->st.updates++; set_busy(s, 1); }
        if (PF(s) && id && id != EPD_GT_DROPPED) prof_track(s, id, took_stag, m);
        s->st.sub_n++;
        acc(&s->st.sub_make_ms_sum, &s->st.sub_make_ms_max, m->make_ms);
        acc(&s->st.sub_lock_ms_sum, &s->st.sub_lock_ms_max, m->lock_ms);
        acc(&s->st.sub_tgt_ms_sum, &s->st.sub_tgt_ms_max, tgt);
        __atomic_store_n(&s->in_tgt_ns, (long long)(tgt * 1e6), __ATOMIC_RELAXED);
#ifdef EPD_HOST
        if (epd_stream_host_applied) epd_stream_host_applied(id, m->kind, m->r);
#endif
    }
    return id;
}

/* the drive thread, under mu: every message, in order; then the view. The
 * messages stay in the ring until the view that has them is published, so
 * epd_stream_free_ms (which reads the ring first, then the view) always sees
 * a submit in one of the two: the first run of the host test caught it in
 * neither (taken out of the ring, not yet applied: free_ms 0) */
static void inbox_drain_(epd_stream *s);
static void inbox_drain(epd_stream *s)
{
    if (!PF(s)) { inbox_drain_(s); return; }
    double t0 = now();
    inbox_drain_(s);
    s->dp->cur[DF_INBOX] += (float)((now() - t0) * 1e3);
}
static void inbox_drain_(epd_stream *s)
{
    static struct inmsg m[INBOX_N];     /* (the drive thread only; ~26 KB, not on a musl thread's 128 KB stack) */
    pthread_mutex_lock(&s->imu);
    int n = s->icount;
    for (int i = 0; i < n; i++) m[i] = s->in[(s->ihead + i) % INBOX_N];
    pthread_mutex_unlock(&s->imu);
    for (int i = 0; i < n; i++) apply_msg(s, &m[i]);
    view_publish(s);
    if (!n) return;
    pthread_mutex_lock(&s->imu);
    s->ihead = (s->ihead + n) % INBOX_N;
    __atomic_store_n(&s->icount, s->icount - n, __ATOMIC_SEQ_CST);
    pthread_cond_broadcast(&s->ispace);
    pthread_mutex_unlock(&s->imu);
    s->st.inbox_n += n;
    if ((uint64_t)n > s->st.inbox_max) s->st.inbox_max = n;
    if (PF(s)) {
        s->dp->cur[DF_MSGS] += n;
        s->dp->drains++; s->dp->msgs += n;
        if ((unsigned long)n > s->dp->msgs_max) s->dp->msgs_max = n;
    }
    /* every submit dropped (nothing became busy): the idle waiters, who also
     * wait for the inbox to empty, look again */
    if (!s->busy) pthread_cond_broadcast(&s->idle);
}

/* a submitter: the message into the ring (waits for a slot when full: the
 * drive thread is stuck in link work; the inbox never drops), then the wake */
static void inbox_put(epd_stream *s, struct inmsg *m)
{
    double t1 = now();
    pthread_mutex_lock(&s->imu);
    while (s->icount >= INBOX_N) { s->ifull++; pthread_cond_wait(&s->ispace, &s->imu); }
    m->lock_ms = (now() - t1) * 1e3;
    m->tapp = t1;
    if (m->kind == IN_SUBMIT || m->kind == IN_SUBMIT_GT) {
        s->in_make = m->make_ms; s->in_lock = m->lock_ms;
        m->id = s->sub_id++;
        if (!s->sub_id) s->sub_id = 1;
    }
    s->in[(s->ihead + s->icount) % INBOX_N] = *m;
    __atomic_store_n(&s->icount, s->icount + 1, __ATOMIC_SEQ_CST);
    pthread_mutex_unlock(&s->imu);
    wake_drive(s);
}

/* a knob: through the inbox (in order with the submits), or at once under mu */
static void knob(epd_stream *s, int kind, int a, int b)
{
    struct inmsg m = { .kind = kind, .a = a, .b = b };
    if (epd_stream_inbox) { inbox_put(s, &m); return; }
    pthread_mutex_lock(&s->mu);
    apply_msg(s, &m);
    pthread_mutex_unlock(&s->mu);
}

/* switches, at the loop top (under mu): drive_overlap into the engine
 * (off: a job out applied first); the stream thread at FIFO 51 while a band
 * switch is on, 50 (today) else */
static void band_switches(epd_stream *s)
{
    int ov = __atomic_load_n(&epd_stream_drive_overlap, __ATOMIC_RELAXED) != 0;
    int nb = __atomic_load_n(&epd_claim_bands, __ATOMIC_RELAXED);
    if (ov != s->overlap_on) { epd_set_overlap(&s->e, ov); s->overlap_on = ov; }
    int prio = ov || nb > 1 || __atomic_load_n(&epd_stream_upd_fifo, __ATOMIC_RELAXED) ? 51 : 50;   /* (above FIFO update jobs too) */
    if (prio == s->prio) return;
    struct sched_param sp = { .sched_priority = prio };
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) && epd_stream_verbose)
        fprintf(stderr, "epd: no SCHED_FIFO %d for the stream thread\n", prio);
    s->prio = prio;
}

static void *thread(void *arg)
{
    epd_stream *s = arg;
    /* a flip issued >0.7 ms after the event waits a whole period: stay ahead
     * of ordinary tasks (needs root / CAP_SYS_NICE; best effort) */
    struct sched_param sp = { .sched_priority = 50 };
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) && epd_stream_verbose)
        fprintf(stderr, "epd: no SCHED_FIFO for the stream thread\n");
    rt_thread_setup(s, 0);
    s->prio = 50;
#ifdef EPD_GPU
    on_drive = 1;
    if (epd_stream_gpu == 1 || epd_stream_gpu == 2) gpu_start(s);
    uint32_t was[EPD_MAX_UPD];
#endif
    double last_active = 0, prev_t = 0;
    /* just in time: frame k+1 is rendered as late as it can be (right before
     * k's flip event, minus the recent worst render time and a margin), not
     * the moment k was flipped — an update submitted meanwhile (pen ink) goes
     * out one frame (~12 ms) sooner. The worst render time: epd-jit.h (by the
     * next frame's size, jit_size=on) */
    double period = 1.0 / (s->mode.vrefresh ? s->mode.vrefresh : 85), exp_evt = 0;
    epd_jit jit;
    epd_jit_init(&jit);
    int nb = 0, pending = 0, drop = 0, cont = 0;
    struct scan *ready = NULL;
    pthread_mutex_lock(&s->mu);
    for (;;) {
        int stg = 0;
        if (PF(s)) prof_close(s);      /* (drive_prof: the last iteration's record) */
        __atomic_store_n(&s->bdone, 0, __ATOMIC_SEQ_CST);  /* (epd_schedule below looks at the job) */
        band_switches(s);
        inbox_drain(s);         /* the submits since: into the engine before anything looks at it */
#ifdef EPD_GPU
        gpe_service(s);
        if (s->gpu_lost && !ready && !pending) gpu_revive(s);
        gt_service(s);
        if (s->stg.arm && (!s->gpu || s->gpe || s->stg.gl < 0 || !epd_stream_stagger || s->quit)) stag_hold_flush(s);
        if (s->stg.state == STG_OFF && s->stg.arm && s->stg.nhr && !s->e.nrun && !s->e.npend) {
            stag_enter(s);
            if (PF(s) && s->stg.state != STG_OFF && !s->stg.lanes) s->dp->ev |= EV_ENTER;   /* (lanes: no crossing) */
        }
        if (s->stg.state != STG_OFF && !s->gpu) stag_lost(s);
        stg = s->stg.state != STG_OFF;
#endif
        if (s->raw_fn && !s->raw_on && !stg && !s->e.nrun && !s->e.npend) s->raw_on = 1;   /* epd_stream_raw */
        if (epd_stream_jit && !ready && pending && exp_evt > 0 && (s->e.nrun + s->e.npend || stg || s->raw_on)) {
            /* jit_size: woken by a submit, it predicts again — a big update
             * arriving during the wait starts at once, not at the small lead */
            for (int first = 1;;) {
                int c = epd_jit_class(jit_rows(&s->e), stg, s->raw_on);
                double lead = epd_jit_lead(&jit, c, epd_stream_jit_size), start = exp_evt - lead - JIT_MARGIN, w = start - now();
                if (first) { s->st.jit_n[c]++; s->st.jit_lead_ms_sum[c] += lead * 1e3; first = 0; PADD(s, DF_LEAD, lead * 1e3); }
                if (w <= 0) break;
                double tj = PF(s) ? now() : 0;
                if (!epd_stream_jit_size) {
                    pthread_mutex_unlock(&s->mu);
                    struct timespec ts = { (time_t)start, (long)((start - (time_t)start) * 1e9) };
                    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
                    pthread_mutex_lock(&s->mu);
                    PADD(s, DF_JIT, (now() - tj) * 1e3);
                    inbox_drain(s);     /* (submitted meanwhile: into this frame's schedule) */
                    break;
                }
                struct timespec ts;
                clock_gettime(CLOCK_REALTIME, &ts);
                long ns = ts.tv_nsec + (long)(w * 1e9);
                ts.tv_sec += ns / 1000000000L;
                ts.tv_nsec = ns % 1000000000L;
                drive_wait(s, &ts);
                PADD(s, DF_JIT, (now() - tj) * 1e3);
                inbox_drain(s);         /* woken by a submit: it is in the engine for the prediction */
            }
        }
        if (!ready && !stg && !s->raw_on && s->link_fails >= LINK_MAX_FAILS && now() < s->link_retry_at &&
            epd_schedule(&s->e)) {
            /* given up on the link (link_kill): nothing can go out until the
             * next try, so don't render — the 4 workers rendered every
             * frame to drop it */
            epd_advance(&s->e, NULL, NULL);
#ifdef EPD_GPU
            s->gpu_full = 1;
#endif
            view_publish(s);
            continue;
        }
#ifdef EPD_GPU
        if (s->gpu) run_ids(s, was);
#endif
        double ts0 = now();
        int sch = 0;
        if (!ready && !stg && !s->raw_on) sch = epd_schedule(&s->e);      /* (as the || below did: only then) */
        double ts1 = now();
        if (PF(s) && !ready) {
            if (!stg && !s->raw_on) s->dp->cur[DF_SCHED] += (float)((ts1 - ts0) * 1e3);
            prof_take_sched(s);
            prof_started(s, ts0, stg);
        }
        if (!ready && (stg || s->raw_on || sch)) {
            int raw = s->raw_on, more = 0, jc = epd_jit_class(jit_rows(&s->e), stg, raw);
            if (PF(s)) {
                struct dprof *d = s->dp;
                d->rendered = 1; d->stg = stg; d->raw = raw; d->cls = jc;
                int heal = !stg && !raw;
                for (int k = 0; heal && k < EPD_RUN_SLOTS; k++) if (s->e.run[k].nframes) heal = 0;
                if (heal) d->ev |= EV_HEAL;
            }
            if (!stg && !raw) {
                s->st.sched_n++;
                acc(&s->st.sched_ms_sum, &s->st.sched_ms_max, (ts1 - ts0) * 1e3);
            }
#ifdef EPD_HOST
            if (epd_stream_host_slow_sched_us > 0 && !stg && !raw) usleep(epd_stream_host_slow_sched_us);   /* (a long claim, under mu) */
#endif
            double tp = PF(s) ? now() : 0;
            view_publish(s);    /* (what started: epd_stream_free_ms sees runs, not pendings, during the render) */
            PADD(s, DF_PUBLISH, (now() - tp) * 1e3);
            set_busy(s, 1);
#ifdef EPD_GPU
            if (stg) {
                double tu = PF(s) ? now() : 0;
                stag_upload(s);
                PADD(s, DF_UPLOAD, (now() - tu) * 1e3);
            }
#endif
            pthread_mutex_unlock(&s->mu);
            double t0 = now(), t1;
            struct scan *b = &s->buf[nb];
            int gpu_done = 0;
#ifdef EPD_GPU
            if (stg) gpu_done = !stag_frame(s, nb);
            else if (!raw) gpu_done = s->gpu && !gpu_frame(s, nb, was);
#endif
            nb = (nb + 1) % NBUF;
            double tc = PF(s) ? now() : 0;
            if (gpu_done) t1 = now();
            else if (raw) {     /* epd_stream_raw (no GPU: refused there) */
                epd_render_begin(&s->e, s->frame, s->fdirty);   /* (the engine is idle: only the last raw frame's lines) */
                more = s->raw_fn(s->raw_arg, s->frame, s->fdirty);
                t1 = now();
                publish(s, b);
            } else {
                render(s);      /* (after a failed GPU frame too: the same frame again) */
                t1 = now();
                publish(s, b);
            }
            if (PF(s) && !gpu_done) s->dp->cur[DF_CPUR] += (float)((now() - tc) * 1e3);
            if (PF(s)) s->dp->gpu_done = gpu_done;
            double rt = (now() - t0) * 1e3, ct = (now() - t1) * 1e3;
            epd_jit_add(&jit, jc, rt / 1e3);
            pthread_mutex_lock(&s->mu);
            s->st.render_ms_sum += rt;
            if (rt > s->st.render_ms_max) s->st.render_ms_max = rt;
            s->st.copy_ms_sum += ct;
            if (ct > s->st.copy_ms_max) s->st.copy_ms_max = ct;
            double ta0 = now();
#ifdef EPD_GPU
            if (gpu_done && s->gpu && gpu_reset_seen(s)) { gpu_reset(s); stg = 0; }
            if (stg && gpu_done) stag_advance(s);
            else if (stg) stag_lost(s);         /* (the CPU rendered the engine's idle frame) */
            else
#endif
            if (raw) { if (!more) { s->raw_fn = NULL; s->raw_on = 0; } }
            else {
#ifdef EPD_GPU
            if (s->gpu && !s->gpe) epd_gpu_finishing(s->gpu, &s->e);   /* nothing packed: epd-gpu.c */
#endif
            epd_advance(&s->e, NULL, NULL);     /* this frame goes out exactly once */
            }
            double ta1 = now();
            acc(&s->st.adv_ms_sum, &s->st.adv_ms_max, (ta1 - ta0) * 1e3);
            if (PF(s)) {
                s->dp->cur[DF_ADV] += (float)((ta1 - ta0) * 1e3);
                prof_take_sched(s);     /* (epd_advance's finish passes) */
#ifdef EPD_GPU
                if (stg && s->stg.state == STG_OFF && !s->stg.lanes) s->dp->ev |= EV_EXIT;
#endif
            }
            view_publish(s);
            if (PF(s)) {
                double tq = now();
                s->dp->cur[DF_PUBLISH] += (float)((tq - ta1) * 1e3);
                if (pending && exp_evt > 0) {
                    s->dp->cur[DF_SLACK] += (float)((exp_evt - tq) * 1e3);
                    if (exp_evt < tq) s->dp->ev |= EV_LATE;
                }
            }
            ready = b;
        }
        if (pending) {
            pthread_mutex_unlock(&s->mu);
            if (s->hv && param_get("hv_on") != 1) drop = 1;
            double t, tw0 = PF(s) ? now() : 0;
            int wf = wait_flip_by(s, &t, s->flip_t + LINK_DEAD_MS / 1e3);
            PADD(s, DF_FLIPW, (now() - tw0) * 1e3);
            if (wf) {
                if (s->link) link_kill(s, "frame's flip event late");
                ready = NULL;           /* not sent: the panel state is unknown now */
            } else if (s->evt_age > LINK_SLOW_MS && !s->evt_queued && ++s->slow_run >= LINK_SLOW_RUN) {
                /* (a frame read late because this thread was busy says nothing
                 * about the link: A2/2 scrolls tripped this 17 times
                 * a minute with the link fine; a real stall finds us waiting) */
                char why[64];
                snprintf(why, sizeof why, "%d frames in a row took > %d ms (last %.1f)", s->slow_run, LINK_SLOW_MS, s->evt_age);
                if (s->link) link_kill(s, why);
                ready = NULL;
            } else {
                if (s->evt_age <= LINK_SLOW_MS) s->slow_run = 0;
                else if (s->evt_queued) s->st.slow_reads++;
                else s->st.slow_frames++;
                prev_t = t;
                cont = ready != NULL;       /* the next frame waits: it should flip at once */
                /* a driven frame really went out: keep the rails up (no 100 ms
                 * link cycle every 25 s of writing; step 1) */
                if (s->link && s->kick_fd >= 0 && (t - s->kick_at) * 1000 > KICK_MS) hv_kick(s, t);
            }
            pending = 0;
            pthread_mutex_lock(&s->mu);
            last_active = now();
        }
        if (ready) {
            pthread_mutex_unlock(&s->mu);
            double tf0 = PF(s) ? now() : 0;
            if (s->link && drop) {
                fprintf(stderr, "epd: rails dropped mid-stream: link cycle\n");
                s->st.rail_drops++;
                link_down(s);
            } else if (s->link && (now() - s->link_since) * 1000 > s->cycle_ms)
                link_down(s);           /* before the driver's hv_max_ms backstop (hv_cap_ms with kicks) */
            drop = 0;
            if (!s->link) {
                for (int tries = 0; link_up(s) && ++tries < 3;) usleep(200000);
                prev_t = 0;
            }
            if (!s->link)
                fprintf(stderr, "epd: cannot bring the link up: frame dropped\n");
            else if (flip(s, ready->fb)) {
                fprintf(stderr, "epd: frame dropped\n");
                link_down(s);
            } else {
                /* its flip event: the first vblank after now */
                double ti = now();
                /* late = a frame that was due right after the last flip event but
                 * flipped too long after it (> ~0.7 ms: it waits a whole period).
                 * Pauses with nothing to send are not late (the old gap count
                 * called every one of them late) */
                if (cont && prev_t > 0) {
                    double lag = (ti - prev_t) * 1e3;
                    PADD(s, DF_LAG, lag);
                    if (lag > s->st.lag_ms_max) s->st.lag_ms_max = lag;
                    if (lag > 0.7) {
                        s->st.late++;
                        if (epd_stream_verbose)
                            fprintf(stderr, "epd: late frame %d of this link: flipped %.2f ms after the event\n",
                                    s->linkframes, lag);
                    }
                }
                cont = 0;
                exp_evt = prev_t > 0 ? prev_t + period : ti + period;
                while (exp_evt < ti + 0.5e-3) exp_evt += period;
                pending = 1;
                s->st.frames++;
                s->linkframes++;
                if (PF(s)) prof_flipped(s, ti);
            }
            ready = NULL;
            if (PF(s)) {
                s->dp->cur[DF_FLIP] += (float)((now() - tf0) * 1e3);
                prof_report(s, 0);      /* (unlocked: the summary's stderr lines, the CSV flush, a dump) */
            }
            pthread_mutex_lock(&s->mu);
            continue;
        }
        /* a frame dropped by a link kill (above) with updates still running:
         * they go on, else this sleeps untimed with busy set (a reexec
         * waiting for idle deadlocked here) */
        if (s->e.nrun || stg || s->raw_on) continue;
        /* nothing on the wire, nothing to send */
        /* parked (a suspend follows): busy until the link is fully down, so
         * glassine-powerd's wakeup_count handshake waits for it */
        set_busy(s, s->e.npend > 0 || (s->park && (s->link || s->warm)));
        if (s->quit && !s->e.npend && !inbox_pending(s)) break;
        long long hold_ns = __atomic_load_n(&s->hold_ns, __ATOMIC_ACQUIRE);
        if (!s->link && hold_ns > now_ns() && !s->quit && !s->park) {  /* keepalive: warm up now */
            pthread_mutex_unlock(&s->mu);
            link_up(s);
            last_active = now();
            pthread_mutex_lock(&s->mu);
            continue;
        }
        if (s->link && (s->park || (now() - last_active >= LINGER_MS / 1000.0 && (now_ns() >= hold_ns || s->quit)))) {
            int warm_ms = __atomic_load_n(&epd_stream_link_warm_ms, __ATOMIC_RELAXED);
            int warm = warm_ms > 0 && !s->park && !s->quit && !s->no_rails;
            pthread_mutex_unlock(&s->mu);
            if (warm) link_warm(s); else link_down(s);
            pthread_mutex_lock(&s->mu);
            continue;
        }
        if (s->warm) {
            int warm_ms = __atomic_load_n(&epd_stream_link_warm_ms, __ATOMIC_RELAXED);
            const char *why = s->park ? "parked" : s->quit ? "closing" : now() - s->warm_since >= warm_ms / 1000.0 ? "timeout" : NULL;
            /* belt and braces: a suspend in warm idle (the resume restores the
             * CRTC: enable raises the rails), or rails up behind our back */
            if (!why && slept() - s->slept_s > 0.5) why = "a suspend since";
            if (!why && param_get("hv_on") == 1) why = "rails up while warm";
            if (why) {
                pthread_mutex_unlock(&s->mu);
                warm_down(s, why);
                pthread_mutex_lock(&s->mu);
                continue;
            }
        }
        if (PF(s) && (s->dp->dump || now() - s->dp->tab.t0 >= s->dp->every)) {
            pthread_mutex_unlock(&s->mu);
            prof_report(s, 0);
            pthread_mutex_lock(&s->mu);
            continue;           /* (anything submitted meanwhile: looked at first) */
        }
        double tw = PF(s) ? now() : 0;
        if (s->link || s->warm) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_nsec += (s->link ? 50 : 1000) * 1000000L;     /* warm: its checks once a second */
            if (ts.tv_nsec >= 1000000000L) { ts.tv_sec += ts.tv_nsec / 1000000000L; ts.tv_nsec %= 1000000000L; }
            drive_wait(s, &ts);
        } else
            drive_wait(s, NULL);
        PADD(s, DF_WAIT, (now() - tw) * 1e3);
    }
    if (PF(s)) prof_close(s);
    if (s->link || s->warm) {
        pthread_mutex_unlock(&s->mu);
        if (s->link) link_down(s); else warm_down(s, "closing");
        pthread_mutex_lock(&s->mu);
    }
#ifdef EPD_GPU
    if (s->gpe) {               /* the state stays the CPU's (save_levels after close) */
        gt_drop(s);
        if (s->gpe == 1) gpe_down(s);
        epd_set_pixel_ops(&s->e, NULL);
        s->gpe = 0;
        pthread_cond_broadcast(&s->gpe_cv);
    }
    if (s->stg.state != STG_OFF && s->gpu) stag_exit(s);    /* (quit drains it first: stag_advance) */
    epd_stag_free(&s->stg.S);
    free(s->stg.hold);
    if (s->gpu) { epd_gpu_close(s->gpu); s->gpu = NULL; }
    free(s->gpu_words); s->gpu_words = NULL;
    free(s->gpe_chk); s->gpe_chk = NULL;
#endif
    pthread_mutex_unlock(&s->mu);
    return NULL;
}

epd_stream *epd_stream_open(const char *wbf_path, int celsius, int hv)
{
    epd_stream *s = calloc(1, sizeof *s);
    if (!s) return NULL;
    int f = open(wbf_path, O_RDONLY | O_CLOEXEC);
    struct stat st;
    if (f < 0 || fstat(f, &st)) { perror(wbf_path); free(s); return NULL; }
    s->wbf = malloc(st.st_size);
    if (!s->wbf || read(f, s->wbf, st.st_size) != st.st_size) { perror(wbf_path); close(f); free(s); return NULL; }
    close(f);
    if (epd_init(&s->e, s->wbf, st.st_size, celsius)) { fprintf(stderr, "epd: bad waveform file\n"); free(s); return NULL; }
    log_temp_range(s, celsius);
    if (s->e.cst)                       /* (GLASSINE_CLAIM_STATE) */
        fprintf(stderr, "epd: claim_state=%s (W: %s)\n", epd_claim_state_name(&s->e), s->e.pk->wsrc);
    s->frame = malloc(sizeof(uint32_t) * EPD_WORDS);
    if (!s->frame) { epd_free(&s->e); free(s->wbf); free(s); return NULL; }
    memcpy(s->frame, s->e.tpl, sizeof(uint32_t) * EPD_WORDS);
    s->fdirty[0] = s->fdirty[1] = 0;
    s->hv = hv;
    s->hv_want = -1;
    s->qos = -1;
    s->hv_max_ms_was = param_get("hv_max_ms");
    if (param_set("hv", hv ? "1" : "0") || param_set("hv_max_ms", "30000")) {
        fprintf(stderr, "epd: panel-onyx-na1-link parameters not writable\n");
        free(s->frame); free(s); return NULL;
    }
    if (setup_drm(s)) { fprintf(stderr, "epd: DRM setup failed\n"); free(s->frame); free(s); return NULL; }
    /* scanout buffers start as the template: whole frame copied once */
    for (int i = 0; i < NBUF; i++) {
        for (int y = 0; y < EPD_LINES; y++)
            memcpy(s->buf[i].map + (size_t)y * s->buf[i].pitch, s->frame + (size_t)y * EPD_LINE, EPD_LINE * 4);
        s->buf[i].dirty[0] = s->buf[i].dirty[1] = 0;
    }
    pthread_mutex_init(&s->wmu, NULL);
    pthread_mutex_init(&s->umu, NULL);
    pthread_cond_init(&s->ucv, NULL);
    for (int i = 0; i < NWORK; i++) pthread_cond_init(&s->wgo[i], NULL);
    pthread_cond_init(&s->wdone, NULL);
    const char *wr = getenv("GLASSINE_WORKER_RT");          /* (as the ctl's worker_rt=nice; tests) */
    if (wr) epd_stream_worker_rt = !strcmp(wr, "nice");
    for (int i = 1; i < NWORK; i++)
        if (pthread_create(&s->work[i], NULL, worker, s)) { fprintf(stderr, "epd: worker threads\n"); return NULL; }
    pthread_mutex_lock(&s->wmu);
    while (s->wleft < NWORK - 1) pthread_cond_wait(&s->wdone, &s->wmu);
    s->wleft = 0;
    pthread_mutex_unlock(&s->wmu);
    const char *ov = getenv("GLASSINE_DRIVE_OVERLAP");     /* (as the ctl's drive_overlap=on; tests) */
    if (ov) epd_stream_drive_overlap = !strcmp(ov, "on") || atoi(ov) > 0;
    s->bops = (epd_band_ops){ band_run, band_start, band_wait, band_done, s };
    epd_set_band_ops(&s->e, &s->bops);  /* (used only with claim_bands > 1 or drive_overlap) */
    pthread_mutex_init(&s->mu, NULL);
    pthread_cond_init(&s->wake, NULL);
    pthread_cond_init(&s->idle, NULL);
#ifdef EPD_GPU
    pthread_cond_init(&s->gpe_cv, NULL);
#endif
    pthread_mutex_init(&s->imu, NULL);
    pthread_cond_init(&s->ispace, NULL);
    s->in = calloc(INBOX_N, sizeof *s->in);
    s->view = calloc(1, sizeof *s->view);
    if (!s->in || !s->view) { free(s->in); free(s->view); free(s->frame); free(s); return NULL; }
    s->sub_id = 1;
    view_publish(s);
    s->wl_fd = open("/sys/power/wake_lock", O_WRONLY | O_CLOEXEC);
    s->wul_fd = open("/sys/power/wake_unlock", O_WRONLY | O_CLOEXEC);
    if (s->wl_fd < 0) fprintf(stderr, "epd: no /sys/power/wake_lock (CONFIG_PM_WAKELOCKS): a suspend may freeze an update\n");
    /* hv_kick: the link stays up while frames go out, cycled only just before
     * the kernel's hard cap (hv_cap_ms, clamped there to 1..3600 s) */
    s->kick_fd = open(PARAMS "hv_kick", O_WRONLY | O_CLOEXEC);
    s->cycle_ms = CYCLE_MS;
    if (s->kick_fd >= 0) {
        int cap = param_get("hv_cap_ms");
        if (cap > 3600000) cap = 3600000;
        if (cap - 5000 > CYCLE_MS) s->cycle_ms = cap - 5000;
    }
    fprintf(stderr, "epd: link cycle every %d s (%s)\n", s->cycle_ms / 1000,
            s->kick_fd >= 0 ? "hv_kick" : "no hv_kick in this kernel");
    /* drive_prof from the start (the harness): GLASSINE_DRIVE_PROF=1 or =<csv path>;
     * applied here, before the thread runs (no message: the inbox counts stay the test's) */
    const char *pe = getenv("GLASSINE_DRIVE_PROF");
    if (pe && *pe && strcmp(pe, "0") && (s->dp = calloc(1, sizeof *s->dp))) {
        const char *csv = strcmp(pe, "1") ? pe : NULL;
        fq_set(1, csv);
        struct inmsg m = { .kind = IN_PROF, .a = 1, .str = csv ? strdup(csv) : NULL };
        prof_apply(s, &m);
    }
    if (pthread_create(&s->th, NULL, thread, s)) { free(s->frame); free(s); return NULL; }
    return s;
}

uint32_t epd_stream_submit(epd_stream *s, epd_rect r, int mode, int full, const uint8_t *grey, int stride)
{
    /* the copy outside the lock: the stream thread renders, schedules and
     * finishes under it (part 2: the hand-off held it ~8 ms per scroll step) */
    double t0 = now();
    epd_upd_runner run = __atomic_load_n(&s->urun, __ATOMIC_ACQUIRE);
    uint8_t *t = run && (long)r.w * r.h >= EPD_UPD_LEVELS_PX                 /* (the levels in parts) */
                 ? epd_target_make_par(&r, grey, stride, run, __atomic_load_n(&s->uctx, __ATOMIC_RELAXED), EPD_UPD_LEVELS_PARTS)
                 : epd_target_make(&r, grey, stride);
    if (!t) return 0;
    struct inmsg m = { .kind = IN_SUBMIT, .r = r, .mode = mode, .full = full, .t = t, .grey = grey, .stride = stride };
    memcpy(m.org, origin_tl, sizeof m.org);
    m.make_ms = (now() - t0) * 1e3;
    if (epd_stream_inbox) { inbox_put(s, &m); return m.id; }     /* (the stream's id; the engine's stays inside) */
    double t1 = now();
    m.tapp = t1;
    pthread_mutex_lock(&s->mu);
    m.lock_ms = (now() - t1) * 1e3;
    pthread_mutex_lock(&s->imu);
    s->in_make = m.make_ms; s->in_lock = m.lock_ms;
    pthread_mutex_unlock(&s->imu);
    uint32_t id = apply_msg(s, &m);
    pthread_cond_signal(&s->wake);
    pthread_mutex_unlock(&s->mu);
    return id;
}

void epd_stream_set_gt(epd_stream *s, const epd_gt_provider *pv)
{
    pthread_mutex_lock(&s->mu);
#ifdef EPD_GPU
    s->gt_pv = pv;
    s->gt_req = 1;
    pthread_cond_signal(&s->wake);
#else
    (void)pv;
#endif
    pthread_mutex_unlock(&s->mu);
}

uint32_t epd_stream_submit_gt(epd_stream *s, epd_rect r, int mode, int full, int gt, const uint8_t *grey, int stride)
{
#ifdef EPD_GPU
    struct inmsg m = { .kind = IN_SUBMIT_GT, .r = r, .mode = mode, .full = full, .gt = gt, .grey = grey, .stride = stride };
    memcpy(m.org, origin_tl, sizeof m.org);
    m.tapp = now();
    if (epd_stream_inbox) {
        /* taken on the published flag (the drive thread's last view): a change
         * since makes apply_msg fall back to the CPU copy itself */
        if (!__atomic_load_n(&s->gt_ok, __ATOMIC_ACQUIRE)) return 0;      /* refused: still the caller's */
        m.pv = __atomic_load_n(&s->gt_pv, __ATOMIC_ACQUIRE);
        inbox_put(s, &m);
        return m.id;
    }
    double t1 = now();
    pthread_mutex_lock(&s->mu);
    if (s->gt_on && s->stg.state == STG_OFF) {
        m.lock_ms = (now() - t1) * 1e3;
        pthread_mutex_lock(&s->imu);
        s->in_make = 0; s->in_lock = m.lock_ms;
        pthread_mutex_unlock(&s->imu);
        uint32_t id = apply_msg(s, &m);
        pthread_cond_signal(&s->wake);
        pthread_mutex_unlock(&s->mu);
        return id;
    }
    pthread_mutex_unlock(&s->mu);
#else
    (void)s; (void)r; (void)mode; (void)full; (void)grey; (void)stride;
#endif
    (void)gt;                   /* refused: still the caller's */
    return 0;
}

void epd_stream_last_submit(epd_stream *s, double *make_ms, double *lock_ms, double *tgt_ms)
{
    pthread_mutex_lock(&s->imu);
    *make_ms = s->in_make; *lock_ms = s->in_lock;
    pthread_mutex_unlock(&s->imu);
    *tgt_ms = __atomic_load_n(&s->in_tgt_ns, __ATOMIC_RELAXED) / 1e6;
}

void epd_stream_s4(epd_stream *s, char *b, size_t n)
{
    pthread_mutex_lock(&s->mu);
    const epd_engine *e = &s->e;
    snprintf(b, n, "claim_bands=%d drive_overlap=%s: %llu banded claims; %llu jobs, late %llu periods, slipped %llu, "
             "joined %llu, queued finishes %llu; apply %.3f ms avg (max %.2f)", __atomic_load_n(&epd_claim_bands, __ATOMIC_RELAXED),
             s->overlap_on ? "on" : "off", (unsigned long long)e->banded, (unsigned long long)e->jobs,
             (unsigned long long)e->job_late, (unsigned long long)e->job_slip, (unsigned long long)e->job_joins,
             (unsigned long long)e->job_qfin, e->jobs ? e->job_apply_s * 1e3 / (double)e->jobs : 0, e->job_apply_max * 1e3);
    pthread_mutex_unlock(&s->mu);
}

void epd_stream_dc(epd_stream *s, int limit, int *max_abs, long *over)
{
    pthread_mutex_lock(&s->mu);
    gpe_sync(s, GPE_DOWN);
    pk_quiet(s);
    epd_dc_stats(&s->e, limit, max_abs, over);
    pthread_mutex_unlock(&s->mu);
}

void epd_stream_dc_map(epd_stream *s, int16_t *dc, uint8_t *level, uint8_t *op, int *r)
{
    pthread_mutex_lock(&s->mu);
    gpe_sync(s, GPE_DOWN);
    pk_quiet(s);
    epd_state_get(&s->e, level, NULL, NULL, op, dc, NULL);
    if (r) *r = s->e.op_r;
    pthread_mutex_unlock(&s->mu);
}

/* the engine knobs: in order with the submits (an update submitted after a
 * knob change gets it; one before does not), see knob */
void epd_stream_set_matched(epd_stream *s, int on) { knob(s, IN_MATCHED, on, 0); }
void epd_stream_set_matched_rule(epd_stream *s, int from, int cap) { knob(s, IN_MATCHED_RULE, from, cap); }
void epd_stream_set_nonblock(epd_stream *s, int on) { knob(s, IN_NONBLOCK, on, 0); }
void epd_stream_set_model(epd_stream *s, int optical, int track) { knob(s, IN_MODEL, optical, track); }
void epd_stream_set_heal(epd_stream *s, int min, int from) { knob(s, IN_HEAL, min, from); }
void epd_stream_set_heal_len(epd_stream *s, int n) { knob(s, IN_HEAL_LEN, n, 0); }
void epd_stream_set_snap(epd_stream *s, int on) { knob(s, IN_SNAP, on, 0); }

unsigned long long epd_stream_snapped(epd_stream *s)
{
    pthread_mutex_lock(&s->mu);
    unsigned long long n = s->e.snapped_px;
    pthread_mutex_unlock(&s->mu);
    return n;
}

unsigned long long epd_stream_healed(epd_stream *s)
{
    pthread_mutex_lock(&s->mu);
    unsigned long long n = s->e.healed_px;
    pthread_mutex_unlock(&s->mu);
    return n;
}

void epd_stream_shortfall(epd_stream *s, int limit, double *mean, long *over)
{
    pthread_mutex_lock(&s->mu);
    gpe_sync(s, GPE_DOWN);
    epd_op_stats(&s->e, limit, mean, over);
    pthread_mutex_unlock(&s->mu);
}

unsigned long long epd_stream_split_starts(epd_stream *s)
{
    pthread_mutex_lock(&s->mu);
    unsigned long long n = s->e.split_starts;
    pthread_mutex_unlock(&s->mu);
    return n;
}

unsigned long long epd_stream_preempted(epd_stream *s)
{
    pthread_mutex_lock(&s->mu);
    unsigned long long n = s->e.preempted;
    pthread_mutex_unlock(&s->mu);
    return n;
}

unsigned long long epd_stream_flash_skipped(epd_stream *s)
{
    pthread_mutex_lock(&s->mu);
    unsigned long long n = s->e.flash_skipped;
    pthread_mutex_unlock(&s->mu);
    return n;
}

unsigned long long epd_stream_absorbed(epd_stream *s)
{
    pthread_mutex_lock(&s->mu);
    unsigned long long n = s->e.absorbed;
    pthread_mutex_unlock(&s->mu);
    return n;
}

unsigned long long epd_stream_matched_px(epd_stream *s)
{
    pthread_mutex_lock(&s->mu);
    unsigned long long n = s->e.matched_px;
    pthread_mutex_unlock(&s->mu);
    return n;
}

int epd_stream_free_ms(epd_stream *s, epd_rect r)
{
    if (epd_stream_inbox) {
        /* the policy's every loop iteration: the drive thread's published view
         * + the submits it has not applied yet (as pendings), no stream lock */
        static __thread struct eng_view v;      /* (6.5 KB: not on the stack of a RT thread) */
        struct inmsg in[INBOX_N];
        pthread_mutex_lock(&s->imu);           /* the ring first, then the view (see inbox_drain) */
        int nin = s->icount;
        for (int i = 0; i < nin; i++) in[i] = s->in[(s->ihead + i) % INBOX_N];
        pthread_mutex_unlock(&s->imu);
        view_read(s, &v);
        int n = view_frames_free(&v, r, in, nin);
        return (n * 1000 + v.hz - 1) / v.hz;
    }
    pthread_mutex_lock(&s->mu);
    int n = epd_frames_until_free(&s->e, r);
#ifdef EPD_GPU
    if (s->stg.state == STG_ON) n = 0;      /* (the stagger takes a new target every frame) */
#endif
    int hz = s->mode.vrefresh ? s->mode.vrefresh : 85;
    pthread_mutex_unlock(&s->mu);
    return (n * 1000 + hz - 1) / hz;
}

void epd_stream_assume(epd_stream *s, int level5)
{
    pthread_mutex_lock(&s->mu);
    while (s->busy || inbox_pending(s)) pthread_cond_wait(&s->idle, &s->mu);
    epd_job_join(&s->e);            /* (none out when idle; belt and braces) */
    gpe_sync(s, GPE_DOWN);
    epd_assume(&s->e, level5);
    gpe_sync(s, GPE_UP);
#ifdef EPD_GPU
    s->gpu_full = 1;
#endif
    pthread_mutex_unlock(&s->mu);
}

/* waveform lengths grow steeply in the cold (GC16: 38 frames >= 24 C, 55 at
 * 18-21, 77 at 12-15, 131 below 3): log each range change, with the length */
static void log_temp_range(epd_stream *s, int c)
{
    const epd_wbf *w = &s->e.wbf;
    int t = epd_wbf_temp_index(w, c);
    uint8_t *o = NULL;
    int gc = epd_wbf_frames(w, EPD_GC16, t, &o);
    free(o);
    o = NULL;
    int du = epd_wbf_frames(w, EPD_DU, t, &o);
    free(o);
    fprintf(stderr, "epd: waveform range %d..%d C (battery %d C): GC16 %d frames, DU %d\n",
            w->ranges[t], w->ranges[t + 1], c, gc, du);
}

void epd_stream_set_temp(epd_stream *s, int c)
{
    knob(s, IN_TEMP, c, 0);
}

/* sleep / pause (and close): no warm idle, keepalives ignored, the link goes
 * fully down as soon as the running updates end (busy until then: the
 * wakeup_count handshake waits); 0 = back to normal */
void epd_stream_park(epd_stream *s, int on)
{
    pthread_mutex_lock(&s->mu);
    if (on != s->park) {
        s->park = on;
        if (on) {
            __atomic_store_n(&s->hold_ns, 0, __ATOMIC_RELEASE);
            if (s->link || s->warm) set_busy(s, 1);
        }
        if (LINK_LOG) LINK_TS();
        if (LINK_LOG) fprintf(stderr, "epd: %s (link %d, warm %d)\n", on ? "parked" : "unparked", s->link, s->warm);
        pthread_cond_signal(&s->wake);
    }
    pthread_mutex_unlock(&s->mu);
}

/* (pen/touch input: no stream lock, the drive thread may be in a frame) */
void epd_stream_keepalive(epd_stream *s, int ms)
{
    if (ms > KEEPALIVE_MAX_MS) ms = KEEPALIVE_MAX_MS;
    struct inmsg m = { .kind = IN_KEEPALIVE, .ns = now_ns() + (long long)ms * 1000000 };
    if (epd_stream_inbox) { inbox_put(s, &m); return; }
    pthread_mutex_lock(&s->mu);
    apply_msg(s, &m);
    pthread_cond_signal(&s->wake);
    pthread_mutex_unlock(&s->mu);
}

void epd_stream_set_cut(epd_stream *s, int cut)
{
    knob(s, IN_CUT, cut, 0);    /* DU updates submitted from now on */
}

/* the engine's per-pixel levels = what the panel shows; kept across a
 * restart of the daemon (a flash from wrong "old" levels drives the wrong
 * waveform) */
int epd_stream_save_levels(epd_stream *s, const char *path)
{
    epd_stream_wait_idle(s);
    int f = open(path, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
    if (f < 0) return -1;
    pthread_mutex_lock(&s->mu);
    gpe_sync(s, GPE_DOWN);
    pk_quiet(s);
    static uint8_t lv[EPD_NPX];
    epd_state_get(&s->e, lv, NULL, NULL, NULL, NULL, NULL);
    int r = write(f, lv, sizeof lv) == (ssize_t)sizeof lv ? 0 : -1;
    pthread_mutex_unlock(&s->mu);
    return close(f) || r ? -1 : 0;
}

int epd_stream_load_levels(epd_stream *s, const char *path)
{
    static uint8_t b[EPD_NPX];
    int f = open(path, O_RDONLY | O_CLOEXEC);
    if (f < 0) return -1;
    int r = read(f, b, sizeof b) == (ssize_t)sizeof b ? 0 : -1;
    close(f);
    if (r) return -1;
    epd_stream_wait_idle(s);
    pthread_mutex_lock(&s->mu);
    epd_job_join(&s->e);
    gpe_sync(s, GPE_DOWN);
    for (size_t i = 0; i < sizeof b; i++) b[i] &= 31;
    epd_state_set(&s->e, b, NULL, NULL, NULL, NULL, NULL);
    gpe_sync(s, GPE_UP);
#ifdef EPD_GPU
    s->gpu_full = 1;
#endif
    pthread_mutex_unlock(&s->mu);
    return 0;
}

int epd_stream_levels_lost(epd_stream *s)
{
    pthread_mutex_lock(&s->mu);
    int r = s->levels_lost;
    s->levels_lost = 0;
    pthread_mutex_unlock(&s->mu);
    return r;
}

int epd_stream_gpu_dead(epd_stream *s)
{
#ifdef EPD_GPU
    return s->gpu_dead;
#else
    (void)s;
    return 0;
#endif
}

int epd_stream_raw(epd_stream *s, int (*fn)(void *arg, uint32_t *words, int dirty[2]), void *arg)
{
    pthread_mutex_lock(&s->mu);
    int ok = fn && !s->raw_fn;
#ifdef EPD_GPU
    ok = ok && !s->gpu;
#endif
    if (ok) {
        s->raw_fn = fn; s->raw_arg = arg;
        set_busy(s, 1);
        pthread_cond_signal(&s->wake);
    }
    pthread_mutex_unlock(&s->mu);
    return ok ? 0 : -1;
}

void epd_stream_wait_idle(epd_stream *s)
{
    pthread_mutex_lock(&s->mu);
    while (s->busy || inbox_pending(s)) pthread_cond_wait(&s->idle, &s->mu);   /* (a submit still in the inbox counts) */
    pthread_mutex_unlock(&s->mu);
}

void epd_stream_stats(epd_stream *s, epd_stats *o)
{
    pthread_mutex_lock(&s->mu);
    *o = s->st;
    pthread_mutex_lock(&s->imu);
    o->inbox_full = s->ifull;
    pthread_mutex_unlock(&s->imu);
    o->merged = s->e.merged; o->superseded = s->e.superseded;
    o->wait_order = s->e.wait_order; o->wait_pixel = s->e.wait_pixel; o->wait_slot = s->e.wait_slot;
    o->nrun_max = s->e.nrun_max; o->npend_max = s->e.npend_max;
    o->gpe_deferred = s->e.deferred; o->gpe_settle_waits = s->e.settle_waits;
    pthread_mutex_unlock(&s->mu);
}

void epd_stream_stats_reset(epd_stream *s)
{
    pthread_mutex_lock(&s->mu);
    memset(&s->st, 0, sizeof s->st);
    pthread_mutex_lock(&s->imu);
    s->ifull = 0;
    pthread_mutex_unlock(&s->imu);
    s->e.merged = s->e.superseded = s->e.wait_order = s->e.wait_pixel = s->e.wait_slot = 0;
    s->e.nrun_max = s->e.npend_max = 0;
    s->e.deferred = s->e.settle_waits = 0;
    pthread_mutex_unlock(&s->mu);
}

static double thread_cpu_s(pthread_t t)
{
    clockid_t id;
    struct timespec ts;
    if (pthread_getcpuclockid(t, &id) || clock_gettime(id, &ts)) return 0;
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

void epd_stream_cpu(epd_stream *s, double *drive_s, double *workers_s)
{
    *drive_s = thread_cpu_s(s->th);
    *workers_s = 0;
    for (int i = 1; i < NWORK; i++) *workers_s += thread_cpu_s(s->work[i]);
}

void epd_stream_close(epd_stream *s)
{
    pthread_mutex_lock(&s->mu);
    s->quit = 1;
    pthread_cond_signal(&s->wake);
    pthread_mutex_unlock(&s->mu);
    pthread_join(s->th, NULL);
    if (s->dp) {                /* drive_prof: the last period, the files closed */
        if (s->prof_on) prof_report(s, 1);
        fq_set(0, "");
        if (s->dp->csv) fclose(s->dp->csv);
        free(s->dp->dump);
        free(s->dp);
        s->dp = NULL;
        s->prof_on = 0;
        epd_sched_prof = 0;
    }
    pthread_mutex_lock(&s->wmu);
    s->wquit = 1;
    for (int i = 1; i < NWORK; i++) pthread_cond_signal(&s->wgo[i]);
    pthread_mutex_unlock(&s->wmu);
    for (int i = 1; i < NWORK; i++) pthread_join(s->work[i], NULL);
    param_set("hv", "0");
    if (s->hv_max_ms_was > 0) {
        char v[16];
        snprintf(v, sizeof v, "%d", s->hv_max_ms_was);
        param_set("hv_max_ms", v);
    }
    if (s->busy) set_busy(s, 0);
    if (s->wl_fd >= 0) close(s->wl_fd);
    if (s->wul_fd >= 0) close(s->wul_fd);
    if (s->kick_fd >= 0) close(s->kick_fd);
    epd_free(&s->e);
#ifdef EPD_HOST
    host_close();
#else
    close(s->fd);
#endif
    for (int i = 0; i < s->icount; i++) {          /* (submitted after the close began: dropped) */
        struct inmsg *m = &s->in[(s->ihead + i) % INBOX_N];
        if (m->kind == IN_SUBMIT) epd_target_free(m->t);
        if (m->kind == IN_SUBMIT_GT && m->pv && m->pv->release) m->pv->release(m->pv->ctx, m->gt);
    }
    free(s->in);
    free(s->view);
    free(s->frame);
    free(s->wbf);
    free(s);
}

/* dry run on/off without a reexec (which a hung GPU forbids): from the next link up */
void epd_stream_set_hv(epd_stream *s, int hv)
{
    __atomic_store_n(&s->hv_want, !!hv, __ATOMIC_RELEASE);
}
