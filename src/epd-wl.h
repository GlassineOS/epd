/* epd-wl.h — glassine-epdd-desktop's Wayland frame source (*: the other end of sway's glassine-epd output
 * backend (glassine-epd-output-proto.h). Knows the socket and the buffers, not
 * the panel: each frame's damaged pixels go to a callback, which converts
 * them (glassine-epdd-desktop: into its grey source image). No X, no wlroots. */
#ifndef EPD_WL_H
#define EPD_WL_H
#include <stdint.h>

typedef struct epd_wl epd_wl;

/* one damaged rect of a frame, clipped to the buffer: XRGB8888, px points at
 * the rect's top-left pixel. Rects (here, the damage and the dma-buf ops) are
 * always portrait, whatever the buffer's layout */
typedef void (*epd_wl_rect_fn)(void *ctx, const uint32_t *px, int stride_px, int x, int y, int w, int h);
/* the same for a grey buffer (sway's grey output, R8 in panel
 * order, luma before the gamma table): g points at panel pixel (x, y), a
 * panel rect (the portrait rect turned: x = h - 1 - (portrait y + h' - 1),
 * y = portrait x, w = portrait h, h = portrait w) */
typedef void (*epd_wl_grey_fn)(void *ctx, const uint8_t *g, int stride, int x, int y, int w, int h);
/* without one, grey buffers' CPU-way rects are skipped */

/* dma-buf buffers (a GLES sway). With ops set, each one is
 * imported once (import: b->handle, -1 = refused) and each frame's damage goes
 * to ops->frame (0: read; 1: read, and read in place from now on (epd_wl_unkeep);
 * -1: not, then the CPU reads it through the buffer's
 * mapping into the rect callback — slow, write-combined on msm: a fallback).
 * Without ops every dma-buf frame goes the CPU way. */
typedef struct { int x, y, w, h; } epd_wl_rect;
typedef struct {
    uint32_t id;
    int fd, w, h, stride, offset;
    uint32_t format;
    uint64_t modifier;
    int handle;                             /* the importer's (-1: none) */
} epd_wl_dmabuf;
typedef struct {
    int (*import)(void *ctx, epd_wl_dmabuf *b);
    void (*drop)(void *ctx, epd_wl_dmabuf *b);
    int (*frame)(void *ctx, epd_wl_dmabuf *b, const epd_wl_rect *r, int n);
} epd_wl_dmabuf_ops;
void epd_wl_set_dmabuf(epd_wl *s, const epd_wl_dmabuf_ops *ops, void *ctx);
/* the CPU way for grey buffers (with the open's ctx) */
void epd_wl_set_grey(epd_wl *s, epd_wl_grey_fn fn);
/* the CPU way for rects of dma-buf b, into fn (check mode); 0, or -1: no mapping */
int epd_wl_cpu_rects(epd_wl *s, const epd_wl_dmabuf *b, const epd_wl_rect *r, int n, epd_wl_rect_fn fn,
                     epd_wl_grey_fn gfn, void *ctx);
/* since the last call: dma-buf frames read by ops->frame, and the CPU way; then reset */
void epd_wl_dmabuf_stats(epd_wl *s, unsigned long *gpu, unsigned long *cpu);

/* damage at a frame's arrival : with this set, the frame's rects
 * go here when it arrives, and its pixels are converted (the rect callback /
 * ops->frame) only by epd_wl_flush, epd_wl_done, or a newer frame's arrival
 * when the backend can't hold buffers (no NEXT); a frame replaced unread is
 * never converted (its damage then goes with the newer frame's convert) */
typedef void (*epd_wl_damage_fn)(void *ctx, const epd_wl_rect *r, int n);
void epd_wl_set_damage(epd_wl *s, epd_wl_damage_fn fn, void *ctx);
/* convert the newest frame now, if it isn't yet (before reading the grey) */
void epd_wl_flush(epd_wl *s);
/* sway may draw the next frame now; this one stays readable until its DONE
 * (a backend without NEXT: epd_wl_done) */
void epd_wl_next(epd_wl *s);
/* zero-copy : ops->frame returning 1 = the daemon reads that
 * buffer in place from now on. Its frame is answered NEXT (needs
 * GLASSINE_EPD_CAP_NEXT), its DONE goes when the next frame is converted, or here
 * (the daemon copied the picture out and stopped reading it) */
void epd_wl_unkeep(epd_wl *s);
/* the connected backend's HELLO (GLASSINE_EPD_CAP_*; 0: none / not connected) */
uint32_t epd_wl_caps(const epd_wl *s);
/* the gamma tables (luma's, luminance's: the backend's HELLO picks one), for a
 * backend that applies them on the GPU (GLASSINE_EPD_CAP_GAMMA): sent on HELLO and
 * on every change. Set before the first dispatch */
void epd_wl_set_luts(epd_wl *s, const uint8_t luma[256], const uint8_t lum[256]);
/* the dead band for a backend that can (GLASSINE_EPD_CAP_BAND): T (0: off)
 * and the quiet gap before its true frame, sent on HELLO and on every change.
 * fn gets a true frame's rects at its arrival: no damage (see glassine-epd-output-proto.h),
 * its pixels are the newest frame's as usual */
void epd_wl_set_band(epd_wl *s, int t, int quiet_ms, epd_wl_damage_fn fn, void *ctx);
/* 1 while a frame whose buffer already went through the table is converted
 * (the callbacks then copy: an identity table) */
int epd_wl_prelut(const epd_wl *s);
/* frames dropped since the last call (they carried another table's tag);
 * *pre: frames read that went through the table on the backend's GPU */
unsigned long epd_wl_stale(epd_wl *s, unsigned long *pre);
/* since the last call: frames answered without being converted; then reset */
unsigned long epd_wl_skipped(epd_wl *s);

/* the pointer (CURSOR_IMG / CURSOR_POS): called at the end of a
 * dispatch that changed it. on 0: no image (hidden). r: where the sprite sits,
 * portrait, NOT clipped to the screen (the same buffer -> portrait turn as
 * the frames' rects); px: its ARGB8888 pixels (premultiplied, as wlroots
 * renders them) in portrait layout, r.w x r.h, row by row, valid until the
 * next call; img: the sprite changed; moved: the place did (CURSOR_POS: the
 * user moved it); panel: the backend's frames are the grey output's (R8,
 * panel order: their LUT applies) */
typedef struct { int on, img; epd_wl_rect r; const uint32_t *px; int moved, panel; } epd_wl_cursor;
typedef void (*epd_wl_cursor_fn)(void *ctx, const epd_wl_cursor *c);
void epd_wl_set_cursor(epd_wl *s, epd_wl_cursor_fn fn, void *ctx);

/* listen on path (NULL: $GLASSINE_EPD_SOCK, else GLASSINE_EPD_SOCK); frames of any other size than w x h
 * are refused (logged). NULL on failure. */
epd_wl *epd_wl_open(const char *path, int w, int h, epd_wl_rect_fn fn, void *ctx);
void epd_wl_close(epd_wl *s);
/* the fds to poll for input: the listening socket, and the backend's
 * connection if there is one; returns how many (1 or 2) */
int epd_wl_fds(const epd_wl *s, int fd[2]);
/* read what arrived, without blocking; returns how many damaged rects went to
 * the callback. A frame whose rects were all handed over waits for epd_wl_done. */
int epd_wl_dispatch(epd_wl *s);
/* a frame is waiting for DONE (sway renders the next one only after it) */
int epd_wl_pending(const epd_wl *s);
/* DONE for the latest frame (converted first, if it isn't): sway may draw the next */
void epd_wl_done(epd_wl *s);
/* since the last call: frames, damaged pixels; then reset */
void epd_wl_stats(epd_wl *s, unsigned long *frames, unsigned long long *px);
/* DONE -> the next FRAME (sway + the apps drawing it), since the last call; then reset */
void epd_wl_chain(epd_wl *s, unsigned long *n, double *avg_ms, double *max_ms);
/* the pct-th percentile (0..100) of the last EPD_WL_CHAIN_N DONE -> FRAME
 * times in ms, or `none` with no samples yet: how long before a read sway must
 * be answered for its next frame to be there in time */
#define EPD_WL_CHAIN_N 16
double epd_wl_chain_pct(const epd_wl *s, int pct, double none);
/* ask a running glassine-swayd for its app list ("apps?"): it only sends changes,
 * so after our (re)start we would have none */
void epd_wl_ask_swayd(void);

#endif
