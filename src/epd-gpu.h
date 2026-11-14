/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-gpu — the engine's drive frames rendered on the Adreno (step 5).
 *
 * epd_render_rows as a fragment shader: one fragment per wire word, 8 pixels
 * each; per pixel phase = LUT[run frame][target << 5 | level] & 3 of the run
 * that owns it (busy = slot + 1), OR'd into the control template. The CPU keeps
 * scheduling (epd_submit / epd_schedule / epd_advance); the GPU gets the
 * per-pixel state where it changed (epd_gpu_state after a claim or a finish)
 * and each running update's current LUT frame (epd_gpu_render).
 *
 * The output is an RGBA8 texture of EPD_LINE x EPD_LINES texels whose bytes are
 * the frame's words in memory order (R = bits 0..7): the layout of the DRM
 * XRGB8888 buffer the link sends, so it can later be scanned out directly. */
#ifndef EPD_GPU_H
#define EPD_GPU_H

#include <stddef.h>
#include <stdint.h>
#include "epd-engine.h"
#include "epd-gtarget.h"
#include "epd-stagger.h"

typedef struct epd_gpu epd_gpu;
#define EPD_GPU_TARGETS 8

/* surfaceless EGL (Mesa) + GLES 3; NULL (and a message on stderr) on failure */
epd_gpu *epd_gpu_open(const epd_engine *e);
void epd_gpu_close(epd_gpu *g);
const char *epd_gpu_renderer(epd_gpu *g);
/* upload level/target/busy of panel rect r (after epd_schedule claimed or
 * epd_advance finished pixels there) */
void epd_gpu_state(epd_gpu *g, const epd_engine *e, epd_rect r);
extern double epd_gpu_prof_pack;    /* (epd_sched_prof) s spent packing in epd_gpu_state; the reader zeroes it */
/* Packs around the runs: epd_gpu_finishing before epd_advance (what ends
 * this frame leaves stale codes, nothing is packed), epd_gpu_starts after
 * epd_schedule (was[] = each slot's run id before it, 0 = idle): the rects
 * to epd_gpu_state for the runs that started (and a reused slot's stale
 * codes), merged, at most 2 * EPD_MAX_UPD */
void epd_gpu_finishing(epd_gpu *g, const epd_engine *e);
int epd_gpu_starts(epd_gpu *g, const epd_engine *e, const uint32_t *was, epd_rect *out);
/* its CPU half: r's per-pixel state codes into s (rows of whole texels: r is
 * clipped and widened to multiples of 8 in x). 0 = nothing (empty rect).
 * Separate so the host tests check it without a GPU */
int epd_gpu_pack_state(uint16_t *s, const epd_engine *e, epd_rect *r);
/* drive_state= : where the drive shader's per-pixel codes
 * live. 0 tex = the RGBA32UI texture st, packed and uploaded (default); 1
 * heap = W, a dma-buf the CPU writes in place and the shader samples (the
 * system dma-heap, else a udmabuf: the host's stand-in), no upload (stale
 * codes as with tex; GLASSINE_DRIVE_STATE_FIN=idle: epd_gpu_finishing writes IDLE
 * instead, a measurement). -1: GLASSINE_DRIVE_STATE=heap|tex. Read
 * at epd_gpu_open; heap falls back to tex when the import fails. The same
 * frames either way. GLASSINE_DRIVE_STATE_FMT=8888: W as ABGR8888 (4 fetches a
 * word, the R2 fallback) instead of ABGR16161616 (2) */
const char *epd_gpu_drive_state_of(const epd_gpu *g);   /* "tex", or W's format + source */
/* Frames go into targets: 0 = a texture of the GPU's own (epd_gpu_open), more
 * from dma-bufs of EPD_LINE x EPD_LINES 32-bit words, rows pitch bytes apart
 * (the DRM scanout buffers: the frame lands where the link reads it, row
 * padding untouched). The GPU sees such a buffer flat, 4096 bytes per row, so
 * size must reach the next multiple of 4096 past its last line (allocate
 * EPD_GPU_EXTRA_LINES more lines than the display shows). -1 on failure. */
#define EPD_GPU_EXTRA_LINES 4
int epd_gpu_target_dmabuf(epd_gpu *g, int fd, int pitch, size_t size);
/* the next drive frame of every running update (as epd_render) into target;
 * queued, not waited for. Each target is only redrawn where updates run now
 * or ran when it was drawn last, so a target must receive every frame it is
 * meant to show (as epd_render's buffer). */
void epd_gpu_render(epd_gpu *g, const epd_engine *e, int target);
/* waits for the queued frames, at most timeout_ms: 0 done, -1 the GPU did not
 * finish (hung: a CP fault left no fence ever signalling; after -1 the
 * context must be abandoned, never closed: freeing its buffers waits for the
 * dead fences in the kernel, uninterruptibly) */
int epd_gpu_finish(epd_gpu *g, int timeout_ms);
/* after a -1 (a hung frame or claim): 1 if the GPU since finished everything
 * of ours (the timed-out fence signalled: the context can be used again, its
 * targets are drawn whole next time, the per-pixel state must go up again);
 * 0: still in flight (look again later). Never blocks */
int epd_gpu_revive(epd_gpu *g);
int epd_gpu_error(epd_gpu *g);        /* GL error since the last call (0 = none) */
/* target's frame, EPD_WORDS words (waits; a dma-buf target is read through
 * the GPU, never through its write-combined CPU mapping) */
void epd_gpu_read(epd_gpu *g, int target, uint32_t *words);

/* the engine's per-pixel state on the GPU (gpe): level, target,
 * busy, panel model, DC record and stale kept in textures; a claim's pixel
 * pass (epd_claim_pixels) and the finish of runs done there. Taken from e at
 * open; epd_gpu_gpe_upload puts a rect of e there again. 0 / -1 */
int epd_gpu_gpe_open(epd_gpu *g, const epd_engine *e);
void epd_gpu_gpe_upload(epd_gpu *g, const epd_engine *e, epd_rect r);
/* the whole state from e into new S and D textures (waits for the GPU): no
 * upload into a texture the passes have stored into (tests: the recovery
 * after a difference). 0 / -1 */
int epd_gpu_gpe_reseed(epd_gpu *g, const epd_engine *e);
/* the pixel pass of plan pl; out as epd_claim_pixels (a lazy plan first gets
 * its slots from a probe pass: epd_claim_take_slots). Waits for the GPU, at
 * most timeout_ms (-2: hung, see epd_gpu_finish). -1: not done */
int epd_gpu_gpe_claim(epd_gpu *g, const epd_engine *e, epd_claim_plan *pl, epd_claim_out *out, int timeout_ms);
/* the same in two (the deferred claims, epd_set_defer): submit dispatches
 * the passes with a fence and returns (a lazy plan's probe still waits);
 * collect waits for them, at most timeout_ms (0: only looks, 1 = not done
 * yet), and gives the report (-2 hung, -1 failed). One claim at a time;
 * cancel forgets one never to be collected (the GPU dropped) */
int epd_gpu_gpe_submit(epd_gpu *g, const epd_engine *e, epd_claim_plan *pl, int timeout_ms);
int epd_gpu_gpe_collect(epd_gpu *g, epd_claim_out *out, int timeout_ms);
void epd_gpu_gpe_cancel(epd_gpu *g);
/* the runs in slots (bit s = run[s], their rects in e) finish: their pixels
 * reach their target */
void epd_gpu_gpe_finish(epd_gpu *g, const epd_engine *e, uint64_t slots);
/* epd_pend_query of pending p on the GPU's state (waits; -2 hung) */
int epd_gpu_gpe_query(epd_gpu *g, const epd_pend *p, uint64_t *mask, int *nonbw, int timeout_ms);
/* drive frames (epd_gpu_render) from the engine state directly: no
 * epd_gpu_state packs needed (1), or from the packed codes (0) */
void epd_gpu_gpe_drive(epd_gpu *g, int on);
/* per-pass profile (tests): with epd_gpu_prof_on the GPU is finished after
 * each pass and its seconds added to epd_gpu_prof[pass] */
enum { EPD_GPU_PROF_UPLOAD, EPD_GPU_PROF_PROBE, EPD_GPU_PROF_CLAIM, EPD_GPU_PROF_REDUCE, EPD_GPU_PROF_WAIT,
       EPD_GPU_PROF_READ, EPD_GPU_PROF_FIN, EPD_GPU_PROF_QUERY, EPD_GPU_PROF_N };
/* epd_gpu_gpe_debug: every gpe pass finished + GL errors printed, the limits at open */
extern void (*epd_gpu_gpe_debug_hook)(const char *what);   /* with it: called after each finished pass */
/* the gpe's compute shape, chosen at epd_gpu_gpe_open from the environment:
 * GLASSINE_GPE_RPT (pixels per invocation, 1..32), GLASSINE_GPE_GATOM (1: the claim and
 * query passes aggregate with global atomics, no partials + reduce),
 * GLASSINE_GPE_TEXREAD (1, default: state reads through the texture path; 0 =
 * imageLoad, which misses earlier dispatches' writes on the A509),
 * GLASSINE_GPE_PMAP (1, default: the results mapped once, GL_EXT_buffer_storage
 * where the driver has it; 0 = a map per read), GLASSINE_GPE_WY (the claim/query
 * workgroup's rows, 1..6: 32 x WY invocations, default 4; rpt * WY = the
 * rows a workgroup covers); "rpt .., .." (", wy N" when not 4, ", results
 * mapped per read" without the persistent map) */
const char *epd_gpu_gpe_shape(const epd_gpu *g);
int epd_gpu_gpe_selftest(epd_gpu *g);    /* the reduce pass on known partials (device check), 0 ok */
/* epd_gpu_gpe_noatom: timing experiment: the claim's per-pixel shared atomics off (wrong results) */
extern double epd_gpu_prof[EPD_GPU_PROF_N];
/* the whole state (tests; waits) */
void epd_gpu_gpe_read(epd_gpu *g, uint8_t *level, uint8_t *target, uint8_t *busy, uint8_t *op, int16_t *dc, uint8_t *stale);
/* stage 5: update targets drawn on another display (epd-ingest's
 * layers, epd-gtarget.h): every layer imported now; from then on a claim or
 * query of a pending with a GPU target (gt) copies it from its layer into T
 * on the GPU (after the producer's fence) instead of uploading. NULL: off.
 * 0, or -1 (an import failed: off) */
int epd_gpu_gpe_provider(epd_gpu *g, const epd_gt_provider *pv);
/* a GPU target's levels over r into out (r.w per row; the CPU's fallback),
 * read through the GPU. 0, -1 failed, -2 the GPU did not finish in timeout_ms */
int epd_gpu_gpe_target_read(epd_gpu *g, int gt, epd_rect r, uint8_t *out, int timeout_ms);
/* Staggered cut steps (epd-stagger.h) on the GPU: the state in textures, one
 * compute pass per frame (one invocation per wire word) + the drive shader's
 * template pass; bit-exact with epd_stag_rows. open: GLES 3.1 (0 / -1);
 * enter: every pixel parked on the GPU, s->tgt uploaded whole (after
 * epd_stag_enter; s->f is the frame counter from here on); load: the pixels
 * of r that epd_stag_load loaded since (ld 2 -> 1) merged in on the GPU
 * (their cold state up, the hot bits made there with p's R, ctl, pay);
 * target: s->tgt over r uploaded (the newest content, after epd_stag_target);
 * frame: frame f's words into target, the GPU's state stepped (queued, as
 * epd_gpu_render: epd_gpu_finish waits); read: the cold state over r into
 * s->cold (whole words: the pixels of r's words), hot: also all of s->hot
 * (tests) (waits). All queued in order: no waits but read's */
int epd_gpu_stag_open(epd_gpu *g);   /* 0, -1 no GLES 3.1, -2 GLES 3.1 but a setup failed (a shader) */
void epd_gpu_stag_enter(epd_gpu *g, const epd_stag *s);
void epd_gpu_stag_load(epd_gpu *g, epd_stag *s, const epd_stag_par *p, epd_rect r);
void epd_gpu_stag_target(epd_gpu *g, const epd_stag *s, epd_rect r);
/* (stagger=check) after a load of r: how many of the pixels ld marks 2 (the
 * ones it took) have their flagged value in the upload texture (the upload
 * landed; waits; -1: no framebuffer) */
long epd_gpu_stag_load_probe(epd_gpu *g, epd_rect r, const uint8_t *ld);
int epd_gpu_stag_load_err(epd_gpu *g);  /* the last load upload's glGetError (0: none) */
/* the tile skip: a workgroup tile with no work for G frames, no target or
 * state change and the same params skips the pass (its no-drive output
 * only); exact (the same state as without). 1 on, 0 off, -1: GLASSINE_STAG_TSKIP */
/* LEAD : each stagger compute pass one leading workgroup row
 * into scratch rows (the A509's lost wave lands there). 1 on, 0 off, -1:
 * GLASSINE_STAG_LEAD (default off); read at epd_gpu_stag_open (the scratch rows) */
/* cells (epd_stag_par.cellw) with SWY 64: a tile's cells run in
 * rank order (a wave one group). 1 on, 0 in place (timing), -1: GLASSINE_STAG_CREMAP
 * (default on); the same results either way */
/* frames straight into the target's image (1, GLASSINE_STAG_DIRECT default) or
 * through the words texture and an emit pass into its framebuffer (0) */
void epd_gpu_stag_set_direct(epd_gpu *g, int on);
void epd_gpu_stag_frame(epd_gpu *g, const epd_stag_par *p, uint32_t f, int target);
void epd_gpu_stag_read(epd_gpu *g, epd_stag *s, epd_rect r, int hot);
/* stagger_state=lanes : the pass on the engine's D and T
 * (epd_stagger_state 1 at the open, claim_state packed with D/T dma-bufs;
 * the frames need p->lanes; any group layout, the blue noise too). lanes: 1 if
 * the open took them. Under lanes enter and load do nothing, target only
 * notes the rect (the CPU writes T and syncs it: epd_dmabuf_sync), read
 * takes the cold state from D's map (hot: the acks into s->ack too).
 * lanes_park: H all parked, every ack 1 (the open does it; after a loss).
 * acks: rows [y0, y1) of the acks (a byte a word, EPD_W / 8 a row, 1 | tog
 * << 1 = parked with that tog), after the frames' fence; 0 / -1 (not lanes) */
int epd_gpu_stag_lanes(const epd_gpu *g);
void epd_gpu_stag_lanes_park(epd_gpu *g);
int epd_gpu_stag_acks(epd_gpu *g, uint8_t *ack, int y0, int y1);
/* stagger_ctl 4 (epd-bp4.h; lanes only): the planes' textures and programs
 * (0; -1 not available: a ctl 4 frame runs ctl 3); the planes as RGBA32UI
 * texels, BP_NK x 8 EPD_H (u: BP_NK * 8 * EPD_H * 4 words; tests, check);
 * with epd_gpu_stag_timer the GPU ms of the pack, the visit, the drive, and
 * the frames ([3]) */
int epd_gpu_bp4_open(epd_gpu *g);
void epd_gpu_bp4_read(epd_gpu *g, uint32_t *u);
extern double epd_gpu_bp4_ms[4];
/* (tests) with epd_gpu_stag_timer, each frame waits for its GPU timer queries
 * (GL_EXT_disjoint_timer_query) and adds the pass's and the template pass's
 * GPU ms to epd_gpu_stag_ms[0], [1]; [2] counts the frames. A frame goes
 * straight into its target (image stores, GL_EXT_EGL_image_storage) once the
 * target was drawn whole; GLASSINE_STAG_DIRECT=0: through the template pass */
extern double epd_gpu_stag_ms[3];

#endif
