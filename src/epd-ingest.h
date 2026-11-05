/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-ingest — , pass A: sway's frames (dma-bufs from a GLES sway,
 * turned into panel grey on the GPU.
 *
 * The CPU convert (glassine-epdd-desktop wl_rect, epd-grey.h: XRGB -> the grey value q
 * (luma or luminance) -> gamma LUT, rotated into the panel-order grey image)
 * as a fragment shader with the same integer math, so the result is
 * byte-identical. The grey goes into an RGBA8
 * texture of 4 panel pixels per texel; for now only the damaged part is read
 * back (glReadPixels: the sampled path, never a write-combined CPU mapping)
 * into the caller's grey image, and today's CPU chain goes on from there.
 *
 * Its own EGL display (platform device, the render node) and context, used
 * from one thread (glassine-epdd-desktop's main loop): independent of epd-gpu (the drive
 * frames in the stream thread, a surfaceless display that it may terminate).
 * A frame that does not finish within the timeout marks the context dead: from
 * then on every call fails and nothing of it is freed (freeing waits on the
 * dead fences, uninterruptibly: ) -- until epd_ingest_revive finds that
 * fence signaled after all (the GPU was slow, not hung: a 952 ms
 * frame under the first A2 grey mode left ingest on the CPU for good). */
#ifndef EPD_INGEST_H
#define EPD_INGEST_H
#include <stdint.h>
#include "epd-gtarget.h"

typedef struct epd_ingest epd_ingest;
typedef struct { int x, y, w, h; } epd_ingest_rect;     /* portrait (buffer) pixels */

/* pw x ph: the panel grey image (landscape: pw = the buffer's height), lut:
 * the gamma table. NULL (and a message) on failure */
epd_ingest *epd_ingest_open(int pw, int ph, const uint8_t lut[256]);
/* frees everything, unless the context is dead (then only the struct is kept) */
void epd_ingest_close(epd_ingest *g);
const char *epd_ingest_renderer(epd_ingest *g);
int epd_ingest_dead(const epd_ingest *g);
/* dead by a timeout whose fence has signaled since: alive again (1; the
 * mirror's tiles all invalid), else 0. Never blocks; the ingest's thread */
int epd_ingest_revive(epd_ingest *g);
/* the XRGB frames' gamma LUT (indexed by q), and their q: lum = luminance,
 * 0 = luma (the default; epd-grey.h). The LUT must match the mode */
void epd_ingest_lut(epd_ingest *g, const uint8_t lut[256]);
void epd_ingest_mode(epd_ingest *g, int lum);
/* the R8 frames' LUT (sway's grey output: q by the formula sway announced);
 * open sets it to lut */
void epd_ingest_lut8(epd_ingest *g, const uint8_t lut[256]);
/* a single-plane dma-buf of w x h (portrait: w = ph, h = pw); fourcc
 * XRGB8888/ARGB8888, modifier (DRM_FORMAT_MOD_INVALID: implicit). A handle
 * >= 0, or -1 */
int epd_ingest_import(epd_ingest *g, int fd, int w, int h, int stride, int offset, uint32_t fourcc, uint64_t modifier);
void epd_ingest_drop(epd_ingest *g, int buf);
/* rects of buffer buf (clipped to it) into grey (pw x ph, panel order: buffer
 * pixel (x, y) -> grey[x * pw + pw - 1 - y]); columns are widened to multiples
 * of 4 (the same frame's pixels: harmless). 0, or -1: not done (dead, or the GPU
 * did not finish within timeout_ms: then dead) */
int epd_ingest_frame(epd_ingest *g, int buf, const epd_ingest_rect *r, int n, uint8_t *grey, int timeout_ms);
/* the same, with where the time went (seconds; t may be NULL): gpu = the draw
 * calls until their fence signalled (our shader, plus whatever the GPU's one
 * ring ran before it, plus any wait for the buffer's producer the caller did
 * not do first), read = glReadPixels */
typedef struct { double gpu, read, sync; } epd_ingest_times;
int epd_ingest_frame_t(epd_ingest *g, int buf, const epd_ingest_rect *r, int n, uint8_t *grey, int timeout_ms,
                       epd_ingest_times *t);
/* the same frame by the CPU (no GPU job): buffer buf's mapping
 * (epd_ingest_map: mmap of the import's own dup of the fd, at the first call,
 * until drop), read under DMA_BUF_IOCTL_SYNC START|READ .. END|READ, the same
 * bytes as epd_ingest_frame_t into grey (widened columns too; the rects
 * epd_ingest_stale). t: sync = the START ioctl (the producer's fences), read
 * = the convert, gpu 0. Needs no GL (a dead context too). 0, or -1: no mapping */
int epd_ingest_frame_cpu(epd_ingest *g, int buf, const epd_ingest_rect *r, int n, uint8_t *grey, epd_ingest_times *t);
/* (epd-ingest-cpu.h): how epd_ingest_frame_cpu
 * converts; all three off = 's code path, unchanged. Read at every CPU
 * frame (live); unset = GLASSINE_INGEST_KERNEL / _BANDS / _SYNC at the first one.
 * kernel 0 row (default: today's bounce + epd_grey_lut / epd_grey_rect_mode,
 * the WC-safe way), 1 tile (R8: LUT rows straight from the mapping,
 * the table in registers; XRGB: 64 x 64 tiles, q + LUT, then a
 * 16 x 16 transpose; for cached exporters, wlroots 0006), 2 scalar
 * (the tile loop in generic C: the device's A/B)
 * bands 1..4 (default 1): frames of >= 0.3 Mpx in N parts of whole panel
 * rows, part 0 on the caller, the rest on 's own pool (pinned to
 * GLASSINE_INGEST_CPUS, default 1-4 = the Gold cores; the caller's
 * scheduling), primed before START
 * sync -1 off (default), N: START|READ on a helper thread pinned to CPU N
 * (waited for; a Silver core's invalidate is ~1/3 of a Gold one's),
 * END|READ posted to it, not waited for. t->sync = the wait, read =
 * the convert (+ END when inline) */
extern int epd_ingest_kernel, epd_ingest_bands, epd_ingest_sync;
/* ingest_partial: 1 (default) the syncs cover only the rows read
 * (DMA_BUF_IOCTL_SYNC_PARTIAL; the whole buffer on a kernel without it),
 * 0 the whole buffer; -1 unset (GLASSINE_INGEST_PARTIAL=on|off at the first frame) */
extern int epd_ingest_partial;
int epd_ingest_kernel_of(const char *s);        /* "row" / "tile" / "scalar" -> 0/1/2 */
int epd_ingest_sync_of(const char *s);          /* "off" -> -1, "on" -> 0 (cpu 0: a Silver core), "N" -> N */
void epd_ingest_cpu_switches(int *kernel, int *bands, int *sync);
/* totals since open: frames, banded, waits, the helper's own ioctl ms */
void epd_ingest_cpu_stats(epd_ingest *g, char *s, int n);
/* buffer buf's pixels at (0, 0) (stride: the import's), NULL: none */
const uint8_t *epd_ingest_map(epd_ingest *g, int buf);
/* the zero-copy view : R8 buffer buf read in place. on 1: map it,
 * START|READ over the whole buffer (t->sync: that ioctl, the producer's
 * fences), *base / *stride its rows; on 0: END|READ (the caller is done with
 * it). 0, or -1: not an R8 import / no mapping */
int epd_ingest_view(epd_ingest *g, int buf, int on, const uint8_t **base, int *stride, epd_ingest_times *t);
/* 1: p lies in one of the imports' CPU mappings (a view) */
int epd_ingest_owns(const epd_ingest *g, const void *p);
/* the panel columns a rect touches after the widening, [*c0, *c1) */
void epd_ingest_cols(const epd_ingest *g, epd_ingest_rect r, int *c0, int *c1);

/* ---- dizzy on the GPU ----
 * epd-dizzy.h's layered dizzy of a panel rect: grey (w x h panel pixels, gs
 * apart, at column x0, row y0) through lv (grey -> level 0..30 | EPD_DZ_FIXED:
 * glassine-epdd-desktop's dz_lv) into out (w bytes a row, os apart): 0 or 30 (fixed pixels:
 * 0, the caller keeps them). Equal to epd_dizzy over the rect as a whole
 * (not cut into bands). Submit, then collect (waits, timeout_ms: then dead);
 * a second submit collects the first. -1: not done (no ES 3.1 compute, dead,
 * a GL error): the CPU's */
int epd_ingest_dizzy_submit(epd_ingest *g, const uint8_t *grey, int gs, int x0, int y0, int w, int h, const uint8_t lv[256]);
int epd_ingest_dizzy_collect(epd_ingest *g, uint8_t *out, int os, int timeout_ms);
int epd_ingest_dizzy(epd_ingest *g, const uint8_t *grey, int gs, int x0, int y0, int w, int h, const uint8_t lv[256],
                     uint8_t *out, int os, int timeout_ms);
/* Two forms : EPD_GDZ_BLOCK (the default of the GPU
 * path: epd-dizzy.h's block dizzy, no shared memory, no barriers, =
 * epd_dizzy_block over the rect with the default tables, NOT the layered
 * form's dots) and EPD_GDZ_LAYERED (the shader above, = epd_dizzy).
 * epd_ingest_dizzy_form sets it (a bad value: returns the current one). The
 * _v calls take `view`: grey is a view of this ingest's output (the caller's
 * grey image the frames fill); where the rect was drawn by the GPU since
 * the caller last wrote it (epd_ingest_stale), the block form reads the
 * output texture itself and skips the upload (epd_ingest_dizzy_src: 1 when
 * the last job did). Tests: GLASSINE_GDZ_FORM=layered|block, and _seed puts a grey
 * image into the output as if drawn (x0, w multiples of 4) */
enum { EPD_GDZ_BLOCK, EPD_GDZ_LAYERED };
int epd_ingest_dizzy_form(epd_ingest *g, int form);
int epd_ingest_dizzy_submit_v(epd_ingest *g, const uint8_t *grey, int gs, int x0, int y0, int w, int h, const uint8_t lv[256], int view);
int epd_ingest_dizzy_v(epd_ingest *g, const uint8_t *grey, int gs, int x0, int y0, int w, int h, const uint8_t lv[256],
                       uint8_t *out, int os, int timeout_ms, int view);
int epd_ingest_dizzy_src(const epd_ingest *g);
int epd_ingest_dizzy_seed(epd_ingest *g, const uint8_t *grey, int gs, int x0, int y0, int w, int h);
/* diagnostics: the greys of a rect (x0, w multiples of 4) as the block shader
 * fetches them from the output (how 0: a compute program with its tx), or as
 * glReadPixels reads the output (how 1); into grey, gs apart */
int epd_ingest_dizzy_peek(epd_ingest *g, int x0, int y0, int w, int h, uint8_t *grey, int gs, int how);

/* ---- stage 5: the shadow mirror and GPU targets ----
 * The caller's panel image `shadow` (pw x ph, what the panel shows) kept on
 * the GPU, so update targets are drawn there instead of copied by the CPU
 * (epd-gtarget.h). The caller keeps it equal, under the lock that guards
 * shadow: a commit of grey from this ingest goes through
 * epd_ingest_mirror_commit (the same per-pixel functions as glassine-epdd-desktop's
 * commit, on the GPU from the ingest's own grey), every other write of
 * shadow through epd_ingest_mirror_upload (before the next commit or
 * target). Rects here are panel pixels: x = column, y = row (pr). */
enum { EPD_INGEST_C_ALL, EPD_INGEST_C_BW, EPD_INGEST_C_DITHER, EPD_INGEST_C_THRESH, EPD_INGEST_C_Q4 };   /* = epd-policy.h C_* */
/* on: the mirror := shadow (all of it), the layers allocated (once; up to
 * EPD_GT_MAX). 0, or -1 (and a message): not available */
int epd_ingest_mirror(epd_ingest *g, const uint8_t *shadow, int layers);
void epd_ingest_mirror_upload(epd_ingest *g, epd_ingest_rect pr, const uint8_t *shadow);
/* glassine-epdd-desktop commit(how, all) of the grey this ingest last put into the caller's
 * grey image over pr. -1: not done (the image is not the ingest's there: the
 * CPU wrote it, epd_ingest_stale; or no mirror): upload instead */
int epd_ingest_mirror_commit(epd_ingest *g, epd_ingest_rect pr, int all, int how);
/* the commit's dither (C_DITHER, C_Q4): EPD_DITHER_* of epd-dither.h (default
 * blue), from the next commit; the caller's CPU commit must use the same */
void epd_ingest_dither(epd_ingest *g, int pattern);
/* the caller wrote its grey image itself (buffer rect r, as epd_ingest_frame) */
void epd_ingest_stale(epd_ingest *g, epd_ingest_rect r);
/* the levels ((shadow & 0xF0) >> 3) of the mirror over pr (clipped) into a
 * free layer: its handle (> 0), or 0 (none free, or no mirror). Fenced: the
 * provider's fence (or, without native fences, waited for here) */
int epd_ingest_target(epd_ingest *g, epd_ingest_rect pr);
const epd_gt_provider *epd_ingest_provider(epd_ingest *g);
/* tests and check modes (wait for the GPU): the whole mirror (pw x ph), and a
 * layer's levels over pr (pr.w per row). 0 / -1 */
int epd_ingest_mirror_read(epd_ingest *g, uint8_t *out);
int epd_ingest_target_read(epd_ingest *g, int gt, epd_ingest_rect pr, uint8_t *out, int timeout_ms);

#endif
