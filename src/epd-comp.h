/* epd-comp.h — glassine-epdd-desktop as the X compositing manager : the windows
 * composited on the CPU (default) or the GPU (opt-in). See epd-comp.c. */
#ifndef EPD_COMP_H
#define EPD_COMP_H
#include <stdint.h>
#include <X11/Xlib.h>
#include "epd-policy.h"

typedef struct comp comp;

/* The backend asked for: GLASSINE_EPDD_COMP=cpu (default) | gpu | off
 * (GLASSINE_EPDD_NOCOMP set: off). */
const char *comp_mode(void);

/* Take over compositing of root's children (portrait W x H screen, panel PW x
 * PH landscape). NULL when off, or when X has no Composite (the stock fbdev X
 * server) or another compositing manager runs: the caller keeps reading the
 * root. gpu falls back to cpu when the GPU path is missing or failed before. */
comp *comp_open(Display *dpy, Window root, int W, int H, int PW, int PH);

/* Give the windows back to X and end GPU use cleanly (the GPU marker goes). */
void comp_close(comp *c);

/* An X event: window structure changes and window damage. Screen areas that
 * need repainting are added to *dirty. Returns 1 when it was the compositor's
 * (a window damage event), 0 otherwise (structure events are also the
 * caller's: return 0). */
int comp_event(comp *c, const XEvent *ev, boxlist *dirty);

/* The grey curve (index: 8-bit luma), as the CPU path's lut[]. */
void comp_set_lut(comp *c, const uint8_t lut[256]);

/* Alpha below this is left out (GTK's soft shadows, anti-aliased edges of
 * rounded corners); at or above, the pixel is opaque. 0..1, default 0.5. */
void comp_set_alpha_cut(comp *c, float cut);

/* Composite portrait rect b and read it back as panel-order grey:
 * g[(px - b.x0) * w + (b.y1 - 1 - py)], w = b.y1 - b.y0 — i.e. the panel rect
 * x = PW - b.y1 .. PW - b.y0, y = b.x0 .. b.x1, row-major. 0 on success. */
int comp_read(comp *c, box b, uint8_t *g);
void comp_skip(comp *c, Window w);

/* the CPU path's pieces, for pixels that don't come from X (glassine-epdd-desktop's
 * impostors): one row of x8r8g8b8 to grey (luma, then lut; alpha ignored),
 * and portrait grey (bw x bh) to the panel order of comp_read */
void comp_grey_row(const uint8_t lut[256], const uint32_t *src, uint8_t *d, int n);
void comp_rotate(const uint8_t *pg, uint8_t *g, int bw, int bh);

/* renderer name ("cpu" or the GL renderer) and counters for the log */
const char *comp_renderer(comp *c);
void comp_stats(comp *c, char *buf, int len);

#endif
