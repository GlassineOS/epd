/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-comp.c — glassine-epdd-desktop as the X compositing manager.
 *
 * The first pipeline was built for the stock 4.4 kernel, which has
 * no GPU driver: X drew into RAM (fbdev / dummy), glassine-epdd-desktop watched the root with
 * XDamage and copied the damaged tiles out with (Shm)GetImage, converting to
 * grey and turning them to the landscape panel on the CPU. Tiles could be
 * caught half-drawn, uncovering a window made its app redraw, and ARGB windows
 * (rounded corners, GTK4 popovers) had no compositor to draw them.
 *
 * Here glassine-epdd-desktop is the compositing manager itself:
 * - it redirects root's children (manual; it does not claim _NET_WM_CM_S0,
 * see comp_open): X keeps
 * every top-level window (xfwm4 frames, override-redirect popups/menus,
 * the desktop) in its own pixmap and draws nothing on screen;
 * - comp_read composites a rect of the screen, bottom to top, into panel
 * grey: luma + the gamma LUT, white where no window, in the frame layout
 * the engine takes (landscape);
 * - alpha: X's ARGB is premultiplied; below alpha_cut a pixel is left out
 * (GTK's grey soft shadows, the fringe of rounded corners: every one would
 * be a grey e-ink update), at or above it counts as opaque (un-premultiplied);
 * - the bounding shape: only the rectangles inside it are composited;
 * - damage per window (XDamage on each redirected window) plus the old/new
 * area of every map/unmap/move/resize/restack gives the dirty rects; the
 * caller's policy decides when and how they go to the panel, as before.
 * Stacking order comes from XQueryTree whenever the structure changed (at
 * most once per read): simpler than following every restack event, and a
 * round trip per burst is nothing next to an e-ink update.
 *
 * Two backends compose the pixels, the same result either way:
 * - cpu (default): each visible part of each window is fetched from its
 * pixmap with XShmGetImage (a round trip: X has finished drawing it, on the
 * GPU or not) and blended here. glassine-epdd-desktop submits nothing to the GPU. Works
 * on the dummy X server (no GPU at all) and on glamor. Windows fully under
 * an opaque window that covers the rect are skipped.
 * - gpu (opt-in, GLASSINE_EPDD_COMP=gpu): each window's pixmap is a GL texture
 * through EGL_KHR_image_pixmap (DRI3: the same buffer, no copy), drawn into
 * an RGBA framebuffer that is already the panel (rotated in the vertex
 * shader, luma + LUT in the fragment shader), one glReadPixels.
 * this hung the Adreno ("hangcheck recover! offending task:
 * glassine-epdd-desktop", from a 900E crash dump; an earlier freeze was likely the
 * same), and the kernel's GPU recovery then froze the whole SoC; text was
 * also seen with missing glyphs and popups with black bands. Hardened
 * since then: an X Sync fence before sampling (X's GPU rendering finished, not
 * just queued: the likely cause of the missing glyphs); the standard RGBA
 * target and readback (not R8/GL_RED); glFinish before every readback and
 * before any texture/image is dropped (never GPU work of ours in flight
 * while X reuses a buffer); a robust context where the driver has one (a
 * reset switches to cpu); repeated GL errors switch to cpu; and a marker
 * file: GPU compositing that ends without comp_close (a crash, a hang, a
 * reboot from it) is not tried again until the marker is deleted
 * (GPU_MARKER).
 */
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>

/* GLES through eglGetProcAddress (EGL 1.5 / KHR_get_all_proc_addresses: core
 * functions too), not linked: libGLESv2 (mesa-gles) need not be installed, and
 * the same binary runs on the stock kernel, where X has no GPU anyway */
#define GL_FNS \
    F(PFNGLACTIVETEXTUREPROC, glActiveTexture) \
    F(PFNGLATTACHSHADERPROC, glAttachShader) \
    F(PFNGLBINDBUFFERPROC, glBindBuffer) \
    F(PFNGLBINDFRAMEBUFFERPROC, glBindFramebuffer) \
    F(PFNGLBINDTEXTUREPROC, glBindTexture) \
    F(PFNGLBUFFERDATAPROC, glBufferData) \
    F(PFNGLCHECKFRAMEBUFFERSTATUSPROC, glCheckFramebufferStatus) \
    F(PFNGLCLEARPROC, glClear) \
    F(PFNGLCLEARCOLORPROC, glClearColor) \
    F(PFNGLCOMPILESHADERPROC, glCompileShader) \
    F(PFNGLCREATEPROGRAMPROC, glCreateProgram) \
    F(PFNGLCREATESHADERPROC, glCreateShader) \
    F(PFNGLDELETETEXTURESPROC, glDeleteTextures) \
    F(PFNGLDISABLEPROC, glDisable) \
    F(PFNGLDRAWARRAYSPROC, glDrawArrays) \
    F(PFNGLENABLEPROC, glEnable) \
    F(PFNGLENABLEVERTEXATTRIBARRAYPROC, glEnableVertexAttribArray) \
    F(PFNGLFINISHPROC, glFinish) \
    F(PFNGLFRAMEBUFFERTEXTURE2DPROC, glFramebufferTexture2D) \
    F(PFNGLGENBUFFERSPROC, glGenBuffers) \
    F(PFNGLGENFRAMEBUFFERSPROC, glGenFramebuffers) \
    F(PFNGLGENTEXTURESPROC, glGenTextures) \
    F(PFNGLGETATTRIBLOCATIONPROC, glGetAttribLocation) \
    F(PFNGLGETERRORPROC, glGetError) \
    F(PFNGLGETINTEGERVPROC, glGetIntegerv) \
    F(PFNGLGETPROGRAMIVPROC, glGetProgramiv) \
    F(PFNGLGETSHADERINFOLOGPROC, glGetShaderInfoLog) \
    F(PFNGLGETSHADERIVPROC, glGetShaderiv) \
    F(PFNGLGETSTRINGPROC, glGetString) \
    F(PFNGLGETUNIFORMLOCATIONPROC, glGetUniformLocation) \
    F(PFNGLLINKPROGRAMPROC, glLinkProgram) \
    F(PFNGLPIXELSTOREIPROC, glPixelStorei) \
    F(PFNGLREADPIXELSPROC, glReadPixels) \
    F(PFNGLSCISSORPROC, glScissor) \
    F(PFNGLSHADERSOURCEPROC, glShaderSource) \
    F(PFNGLTEXIMAGE2DPROC, glTexImage2D) \
    F(PFNGLTEXPARAMETERIPROC, glTexParameteri) \
    F(PFNGLUNIFORM1FPROC, glUniform1f) \
    F(PFNGLUNIFORM1IPROC, glUniform1i) \
    F(PFNGLUNIFORM2FPROC, glUniform2f) \
    F(PFNGLUSEPROGRAMPROC, glUseProgram) \
    F(PFNGLVERTEXATTRIBPOINTERPROC, glVertexAttribPointer) \
    F(PFNGLVIEWPORTPROC, glViewport)
#define F(t, n) static t p_##n;
GL_FNS
#undef F
#define F(t, n) if (!(p_##n = (t)eglGetProcAddress(#n))) return -1;
static int gl_load(void) { GL_FNS return 0; }
#undef F
#define glActiveTexture p_glActiveTexture
#define glAttachShader p_glAttachShader
#define glBindBuffer p_glBindBuffer
#define glBindFramebuffer p_glBindFramebuffer
#define glBindTexture p_glBindTexture
#define glBufferData p_glBufferData
#define glCheckFramebufferStatus p_glCheckFramebufferStatus
#define glClear p_glClear
#define glClearColor p_glClearColor
#define glCompileShader p_glCompileShader
#define glCreateProgram p_glCreateProgram
#define glCreateShader p_glCreateShader
#define glDeleteTextures p_glDeleteTextures
#define glDisable p_glDisable
#define glDrawArrays p_glDrawArrays
#define glEnable p_glEnable
#define glEnableVertexAttribArray p_glEnableVertexAttribArray
#define glFinish p_glFinish
#define glFramebufferTexture2D p_glFramebufferTexture2D
#define glGenBuffers p_glGenBuffers
#define glGenFramebuffers p_glGenFramebuffers
#define glGenTextures p_glGenTextures
#define glGetAttribLocation p_glGetAttribLocation
#define glGetError p_glGetError
#define glGetIntegerv p_glGetIntegerv
#define glGetProgramiv p_glGetProgramiv
#define glGetShaderInfoLog p_glGetShaderInfoLog
#define glGetShaderiv p_glGetShaderiv
#define glGetString p_glGetString
#define glGetUniformLocation p_glGetUniformLocation
#define glLinkProgram p_glLinkProgram
#define glPixelStorei p_glPixelStorei
#define glReadPixels p_glReadPixels
#define glScissor p_glScissor
#define glShaderSource p_glShaderSource
#define glTexImage2D p_glTexImage2D
#define glTexParameteri p_glTexParameteri
#define glUniform1f p_glUniform1f
#define glUniform1i p_glUniform1i
#define glUniform2f p_glUniform2f
#define glUseProgram p_glUseProgram
#define glVertexAttribPointer p_glVertexAttribPointer
#define glViewport p_glViewport
#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/Xutil.h>
#include <X11/extensions/XShm.h>
#include <X11/extensions/sync.h>
#include <X11/extensions/Xcomposite.h>
#include <X11/extensions/Xdamage.h>
#include <X11/extensions/Xfixes.h>
#include <X11/extensions/shape.h>
#include "epd-comp.h"

#ifndef EGL_PLATFORM_X11_KHR
#define EGL_PLATFORM_X11_KHR 0x31D5
#endif
#ifndef EGL_NO_CONFIG_KHR
#define EGL_NO_CONFIG_KHR ((EGLConfig)0)
#endif
#ifndef EGL_CONTEXT_OPENGL_ROBUST_ACCESS_EXT
#define EGL_CONTEXT_OPENGL_ROBUST_ACCESS_EXT 0x30BF
#endif
#ifndef EGL_CONTEXT_OPENGL_RESET_NOTIFICATION_STRATEGY_EXT
#define EGL_CONTEXT_OPENGL_RESET_NOTIFICATION_STRATEGY_EXT 0x3138
#endif
#ifndef EGL_LOSE_CONTEXT_ON_RESET_EXT
#define EGL_LOSE_CONTEXT_ON_RESET_EXT 0x31BF
#endif
typedef GLenum (*reset_status_fn)(void);

/* GPU compositing that did not end in comp_close */
#ifndef GPU_MARKER
#define GPU_MARKER "/var/lib/glassine/comp-gpu.inuse"
#endif

#define MAXWIN 512

typedef struct {
    Window w;
    int x, y, wd, ht, bw;       /* outer geometry: position incl. border */
    int depth, input_only, viewable;
    Visual *vis;
    Damage dmg;
    XRectangle *shp;            /* bounding shape, window-relative (NULL: the whole box) */
    int nshp;
    Pixmap pix;                 /* named window pixmap (0: stale, name on use) */
    EGLImageKHR img;            /* gpu */
    GLuint tex;
} cwin;

struct comp {
    Display *dpy;
    Window root, cm_win;
    int W, H, PW, PH;
    int dmg_ev, shape_ev;       /* shape_ev -1: no SHAPE extension */
    int gpu;                    /* backend: 1 gpu, 0 cpu */
    float cut;
    uint8_t lut[256];
    uint8_t white;              /* lut[255]: the background */
    cwin win[MAXWIN];
    int nwin;
    Window order[MAXWIN];       /* bottom to top (XQueryTree) */
    int norder, order_stale;
    Window skip;                /* left out of every read (comp_skip) */
    XserverRegion region;
    unsigned long reads, draws, names, fetches, skipped;
    /* where a read's time goes (stats): X copy, blend, rotation; gpu: the whole
     * read, split into X sync (t_x), draw (t_blend), readback (t_rot) */
    double t_x, t_blend, t_rot, t_gpu, t_start;
    unsigned long long px;
    char renderer[96];
    /* cpu */
    XShmSegmentInfo shm;
    int shm_ok;
    uint8_t *pg;                /* portrait grey scratch */
    size_t pgcap;
    /* gpu */
    float *quads;               /* vertex scratch for shaped windows */
    int nquads;
    EGLDisplay edpy;
    EGLContext ctx;
    PFNEGLCREATEIMAGEKHRPROC create_image;
    PFNEGLDESTROYIMAGEKHRPROC destroy_image;
    PFNGLEGLIMAGETARGETTEXTURE2DOESPROC image_target;
    reset_status_fn reset_status;
    XSyncFence fence;           /* X finishes its (glamor) rendering before we sample */
    GLuint fbo, fbtex, prog, lut_tex, vbo;
    GLint u_tex, u_lut, u_alpha, u_cut, u_scale, a_pos, a_tc;
    uint8_t *rgba;              /* RGBA readback buffer */
    int gl_errors;
};

static int xerr_hit;
static int ignore_xerr(Display *d, XErrorEvent *e) { (void)d; (void)e; xerr_hit = 1; return 0; }

static box win_box(const cwin *w) { return (box){ w->x, w->y, w->x + w->wd + 2 * w->bw, w->y + w->ht + 2 * w->bw }; }

static void add_dirty(comp *c, boxlist *d, box b)
{
    b = box_clip(b, (box){ 0, 0, c->W, c->H });
    if (!box_empty(b) && d) list_add(d, b);
}

static void unname(comp *c, cwin *w)
{
    if (c->gpu && (w->tex || w->img)) {
        glFinish();                             /* nothing of ours in flight on the old buffer */
        if (w->tex) { glDeleteTextures(1, &w->tex); w->tex = 0; }
        if (w->img) { c->destroy_image(c->edpy, w->img); w->img = EGL_NO_IMAGE_KHR; }
    }
    if (w->pix) { XFreePixmap(c->dpy, w->pix); w->pix = 0; }
}

/* the bounding shape: X clips a shaped window's drawing to it on screen, but
 * its pixmap still holds the pixels outside (glassine-shell zeroes them: black in
 * a 24-bit window) — only the rectangles inside may be composited */
static void get_shape(comp *c, cwin *w)
{
    free(w->shp);
    w->shp = NULL;
    w->nshp = 0;
    if (c->shape_ev < 0 || w->input_only) return;
    Bool bs = False, cs = False;
    int bx, by, cx, cy;
    unsigned int bw_, bh_, cw_, ch_;
    if (!XShapeQueryExtents(c->dpy, w->w, &bs, &bx, &by, &bw_, &bh_, &cs, &cx, &cy, &cw_, &ch_) || !bs) return;
    int n = 0, ord;
    XRectangle *r = XShapeGetRectangles(c->dpy, w->w, ShapeBounding, &n, &ord);
    /* one rectangle covering the whole window is no shape: xfwm4 shapes its
     * frames like that, which kept every framed window (Firefox) from hiding
     * what is under it (the wallpaper was read under it every time) */
    if (n == 1 && r && r[0].x <= -w->bw && r[0].y <= -w->bw &&
        r[0].x + (int)r[0].width >= w->wd + w->bw && r[0].y + (int)r[0].height >= w->ht + w->bw) {
        XFree(r);
        return;
    }
    /* shaped to nothing: no rectangles, and Xlib returns NULL for them — not
     * "unshaped" (that painted the whole pixmap, black where the app zeroed it) */
    w->shp = malloc(sizeof *w->shp * (n ? n : 1));
    if (w->shp && n && r) memcpy(w->shp, r, sizeof *r * n);
    w->nshp = w->shp && r ? n : 0;
    if (r) XFree(r);
}

static cwin *find(comp *c, Window x)
{
    for (int i = 0; i < c->nwin; i++) if (c->win[i].w == x) return &c->win[i];
    return NULL;
}

static cwin *track(comp *c, Window x)
{
    cwin *w = find(c, x);
    if (w) return w;
    XWindowAttributes wa;
    if (x == c->cm_win || !XGetWindowAttributes(c->dpy, x, &wa)) return NULL;
    if (c->nwin == MAXWIN) return NULL;
    w = &c->win[c->nwin++];
    memset(w, 0, sizeof *w);
    w->w = x;
    w->x = wa.x; w->y = wa.y; w->wd = wa.width; w->ht = wa.height; w->bw = wa.border_width;
    w->depth = wa.depth;
    w->vis = wa.visual;
    w->input_only = wa.class == InputOnly;
    w->viewable = wa.map_state == IsViewable;
    if (!w->input_only) w->dmg = XDamageCreate(c->dpy, x, XDamageReportNonEmpty);
    if (c->shape_ev >= 0 && !w->input_only) {
        XShapeSelectInput(c->dpy, x, ShapeNotifyMask);
        get_shape(c, w);
    }
    return w;
}

static void forget(comp *c, Window x)
{
    cwin *w = find(c, x);
    if (!w) return;
    unname(c, w);
    if (w->dmg) XDamageDestroy(c->dpy, w->dmg);   /* BadDamage if the window is gone: ignored */
    free(w->shp);
    *w = c->win[--c->nwin];
}

int comp_event(comp *c, const XEvent *ev, boxlist *d)
{
    if (c->shape_ev >= 0 && ev->type == c->shape_ev + ShapeNotify) {
        const XShapeEvent *se = (const XShapeEvent *)ev;
        cwin *w = find(c, se->window);
        if (w && se->kind == ShapeBounding) {
            get_shape(c, w);
            if (w->viewable) add_dirty(c, d, win_box(w));
        }
        return 1;
    }
    if (ev->type == c->dmg_ev + XDamageNotify) {
        const XDamageNotifyEvent *de = (const XDamageNotifyEvent *)ev;
        cwin *w = find(c, de->drawable);
        XDamageSubtract(c->dpy, de->damage, None, c->region);
        if (!w || !w->viewable) return 1;
        int n = 0;
        XRectangle *r = XFixesFetchRegion(c->dpy, c->region, &n);
        for (int i = 0; i < n; i++) {           /* window-relative, inside the border */
            int x = w->x + w->bw + r[i].x, y = w->y + w->bw + r[i].y;
            add_dirty(c, d, (box){ x, y, x + r[i].width, y + r[i].height });
        }
        if (r) XFree(r);
        return 1;
    }
    switch (ev->type) {
    case CreateNotify:
        if (ev->xcreatewindow.parent != c->root) return 0;
        track(c, ev->xcreatewindow.window);
        c->order_stale = 1;
        return 0;
    case DestroyNotify: {
        cwin *w = find(c, ev->xdestroywindow.window);
        if (w && w->viewable) add_dirty(c, d, win_box(w));
        forget(c, ev->xdestroywindow.window);
        c->order_stale = 1;
        return 0;
    }
    case ReparentNotify:
        if (ev->xreparent.parent == c->root) track(c, ev->xreparent.window);
        else {
            cwin *w = find(c, ev->xreparent.window);
            if (w && w->viewable) add_dirty(c, d, win_box(w));
            forget(c, ev->xreparent.window);
        }
        c->order_stale = 1;
        return 0;
    case MapNotify: {
        if (ev->xmap.event != c->root) return 0;
        cwin *w = track(c, ev->xmap.window);
        if (!w) return 0;
        w->viewable = 1;
        unname(c, w);                           /* new backing pixmap */
        add_dirty(c, d, win_box(w));
        c->order_stale = 1;
        return 0;
    }
    case UnmapNotify: {
        if (ev->xunmap.event != c->root) return 0;
        cwin *w = find(c, ev->xunmap.window);
        if (!w) return 0;
        if (w->viewable) add_dirty(c, d, win_box(w));
        w->viewable = 0;
        unname(c, w);
        return 0;
    }
    case ConfigureNotify: {
        const XConfigureEvent *ce = &ev->xconfigure;
        if (ce->event != c->root) return 0;
        cwin *w = find(c, ce->window);
        if (!w) return 0;
        box ob = win_box(w);
        int resized = ce->width != w->wd || ce->height != w->ht || ce->border_width != w->bw;
        w->x = ce->x; w->y = ce->y; w->wd = ce->width; w->ht = ce->height; w->bw = ce->border_width;
        if (resized) unname(c, w);
        if (w->viewable) { add_dirty(c, d, ob); add_dirty(c, d, win_box(w)); }
        c->order_stale = 1;                     /* "above" changed, maybe */
        return 0;
    }
    case CirculateNotify: {
        cwin *w = find(c, ev->xcirculate.window);
        if (w && w->viewable) add_dirty(c, d, win_box(w));
        c->order_stale = 1;
        return 0;
    }
    }
    return 0;
}

static void restack(comp *c)
{
    Window r, p, *kids = NULL;
    unsigned int n = 0;
    c->norder = 0;
    if (XQueryTree(c->dpy, c->root, &r, &p, &kids, &n))
        for (unsigned int i = 0; i < n && c->norder < MAXWIN; i++)
            if (kids[i] != c->cm_win) c->order[c->norder++] = kids[i];
    if (kids) XFree(kids);
    c->order_stale = 0;
}

/* the window's pixmap (named on first use after map/resize) */
static int name_pix(comp *c, cwin *w)
{
    if (w->pix) return 0;
    xerr_hit = 0;
    w->pix = XCompositeNameWindowPixmap(c->dpy, w->w);
    return w->pix ? 0 : -1;
}

/* the rectangles of window w to composite (its box, or each rectangle of its
 * bounding shape; relative to the inside-border origin), clipped to wb and b;
 * returns the count written to out (room for max(1, nshp)) */
static int win_rects(const cwin *w, box wb, box b, box *out)
{
    int nr = w->shp ? w->nshp : 1, n = 0;
    for (int k = 0; k < nr; k++) {
        box r = wb;
        if (w->shp) {
            int ox = w->x + w->bw, oy = w->y + w->bw;
            r = box_clip((box){ ox + w->shp[k].x, oy + w->shp[k].y,
                                ox + w->shp[k].x + w->shp[k].width, oy + w->shp[k].y + w->shp[k].height }, wb);
        }
        r = box_clip(r, b);
        if (!box_empty(r)) out[n++] = r;
    }
    return n;
}

/* ---- cpu backend ------------------------------------------------------------ */

static void cpu_open(comp *c)
{
    if (!XShmQueryExtension(c->dpy)) return;
    c->shm.shmid = shmget(IPC_PRIVATE, (size_t)c->W * c->H * 4, IPC_CREAT | 0600);
    if (c->shm.shmid < 0) return;                /* the stock kernel: no SysV IPC */
    c->shm.shmaddr = shmat(c->shm.shmid, NULL, 0);
    c->shm.readOnly = False;
    if (c->shm.shmaddr == (char *)-1) { shmctl(c->shm.shmid, IPC_RMID, NULL); return; }
    XErrorHandler old = XSetErrorHandler(ignore_xerr);
    xerr_hit = 0;
    Status st = XShmAttach(c->dpy, &c->shm);
    XSync(c->dpy, False);
    XSetErrorHandler(old);
    shmctl(c->shm.shmid, IPC_RMID, NULL);        /* goes once X and we detach */
    if (!st || xerr_hit) { shmdt(c->shm.shmaddr); return; }
    c->shm_ok = 1;
}

static int shift_of(unsigned long mask)
{
    int s = 0;
    if (!mask) return 0;
    while (!(mask & 1)) { mask >>= 1; s++; }
    return s;
}

static double tsec(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec / 1e9;
}

/* rect r (screen coordinates, inside window w's box wb) of w's pixmap */
static XImage *fetch(comp *c, cwin *w, box wb, box r)
{
    int rw = r.x1 - r.x0, rh = r.y1 - r.y0, sx = r.x0 - wb.x0, sy = r.y0 - wb.y0;
    XImage *img = NULL;
    xerr_hit = 0;
    if (c->shm_ok) {
        img = XShmCreateImage(c->dpy, w->vis, w->depth, ZPixmap, c->shm.shmaddr, &c->shm, rw, rh);
        if (img && XShmGetImage(c->dpy, w->pix, img, sx, sy, AllPlanes) && !xerr_hit) return img;
        if (img) { img->data = NULL; XDestroyImage(img); }
        return NULL;                            /* the window went away meanwhile */
    }
    img = XGetImage(c->dpy, w->pix, sx, sy, rw, rh, AllPlanes, ZPixmap);
    return xerr_hit && img ? (XDestroyImage(img), NULL) : img;
}

static void drop(comp *c, XImage *img)
{
    if (c->shm_ok && img->data == c->shm.shmaddr) img->data = NULL;
    XDestroyImage(img);
}

/* One row of opaque x8r8g8b8 / a8r8g8b8 pixels (red 16, green 8, blue 0) to
 * LUT(luma), NEON: luma 16 px at a time, the 256-byte LUT as four vqtbl4q
 * tables. Returns the row's smallest alpha (argb) so the caller can redo a
 * translucent row the slow way. 2.5 ns/px vs 9 for the scalar loop on
 * the gold cores, bit-identical (scratch blendbench). */
#ifdef __aarch64__
#include <arm_neon.h>
static unsigned row_neon(const uint8_t lut[256], const uint32_t *src, uint8_t *d, int n)
{
    const uint8x16x4_t t0 = vld1q_u8_x4(lut), t1 = vld1q_u8_x4(lut + 64),
                       t2 = vld1q_u8_x4(lut + 128), t3 = vld1q_u8_x4(lut + 192);
    const uint8x16_t k64 = vdupq_n_u8(64);
    const uint8_t *s = (const uint8_t *)src;
    uint8x16_t amin = vdupq_n_u8(255);
    int x = 0;
    for (; x + 16 <= n; x += 16) {
        uint8x16x4_t p = vld4q_u8(s + 4 * x);          /* b, g, r, a planes */
        amin = vminq_u8(amin, p.val[3]);
        uint16x8_t lo = vmull_u8(vget_low_u8(p.val[2]), vdup_n_u8(77));
        lo = vmlal_u8(lo, vget_low_u8(p.val[1]), vdup_n_u8(150));
        lo = vmlal_u8(lo, vget_low_u8(p.val[0]), vdup_n_u8(29));
        uint16x8_t hi = vmull_u8(vget_high_u8(p.val[2]), vdup_n_u8(77));
        hi = vmlal_u8(hi, vget_high_u8(p.val[1]), vdup_n_u8(150));
        hi = vmlal_u8(hi, vget_high_u8(p.val[0]), vdup_n_u8(29));
        uint8x16_t i = vcombine_u8(vshrn_n_u16(lo, 8), vshrn_n_u16(hi, 8));
        uint8x16_t r = vqtbl4q_u8(t0, i);
        i = vsubq_u8(i, k64); r = vorrq_u8(r, vqtbl4q_u8(t1, i));
        i = vsubq_u8(i, k64); r = vorrq_u8(r, vqtbl4q_u8(t2, i));
        i = vsubq_u8(i, k64); r = vorrq_u8(r, vqtbl4q_u8(t3, i));
        vst1q_u8(d + x, r);
    }
    unsigned a = vminvq_u8(amin);
    for (; x < n; x++) {
        uint32_t p = src[x];
        if (p >> 24 < a) a = p >> 24;
        d[x] = lut[((p >> 16 & 0xff) * 77 + (p >> 8 & 0xff) * 150 + (p & 0xff) * 29) >> 8];
    }
    return a;
}
#endif

/* is every pixel of img (a8r8g8b8) opaque? */
static int img_opaque(const XImage *img)
{
    for (int y = 0; y < img->height; y++) {
        const uint32_t *r = (const uint32_t *)((const uint8_t *)img->data + (size_t)y * img->bytes_per_line);
        int x = 0;
#ifdef __aarch64__
        uint8x16_t amin = vdupq_n_u8(255);
        for (; x + 16 <= img->width; x += 16) amin = vminq_u8(amin, vld4q_u8((const uint8_t *)(r + x)).val[3]);
        if (vminvq_u8(amin) != 255) return 0;
#endif
        uint32_t all = 0xff000000u;
        for (; x < img->width; x++) all &= r[x];
        if (all != 0xff000000u) return 0;
    }
    return 1;
}

void comp_grey_row(const uint8_t lut[256], const uint32_t *src, uint8_t *d, int n)
{
#ifdef __aarch64__
    row_neon(lut, src, d, n);
#else
    for (int x = 0; x < n; x++) {
        uint32_t p = src[x];
        d[x] = lut[((p >> 16 & 0xff) * 77 + (p >> 8 & 0xff) * 150 + (p & 0xff) * 29) >> 8];
    }
#endif
}

/* blend rect r of img (which holds rect ib) into the portrait grey scratch of rect b */
static void blend(comp *c, const cwin *w, const XImage *img, box ib, box r, box b)
{
    int bw = b.x1 - b.x0, argb = w->depth == 32;
    int rs = shift_of(w->vis->red_mask), gs = shift_of(w->vis->green_mask), bs = shift_of(w->vis->blue_mask);
    /* alpha a/255 < cut is left out: the smallest a kept */
    unsigned amin = 0;
    while (amin < 256 && amin < c->cut * 255.0f) amin++;
    const uint8_t *lut = c->lut;
    for (int y = r.y0; y < r.y1; y++) {
        const uint32_t *s = (const uint32_t *)((const uint8_t *)img->data + (size_t)(y - ib.y0) * img->bytes_per_line) + (r.x0 - ib.x0);
        uint8_t *d = c->pg + (size_t)(y - b.y0) * bw + (r.x0 - b.x0);
        int n = r.x1 - r.x0;
#ifdef __aarch64__
        if (rs == 16 && gs == 8 && bs == 0) {
            if (!argb) { row_neon(c->lut, s, d, n); continue; }
            /* argb: into a scratch row first; a translucent row goes the slow
             * way below, which keeps the lower window where alpha < cut */
            uint8_t tmp[2048];
            if (n <= (int)sizeof tmp && row_neon(c->lut, s, tmp, n) == 255) { memcpy(d, tmp, n); continue; }
        }
#endif
        /* the common case, an opaque row: no per-pixel alpha logic (the loop
         * vectorises but for the LUT); 35 ns/px before, most of a read */
        uint32_t all = 0xff000000u;
        if (argb) for (int x = 0; x < n; x++) all &= s[x];
        if (all == 0xff000000u) {
            for (int x = 0; x < n; x++) {
                uint32_t p = s[x];
                d[x] = lut[(((p >> rs) & 0xff) * 77 + ((p >> gs) & 0xff) * 150 + ((p >> bs) & 0xff) * 29) >> 8];
            }
            continue;
        }
        for (int x = 0; x < n; x++) {
            uint32_t p = s[x];
            unsigned R = (p >> rs) & 0xff, G = (p >> gs) & 0xff, B = (p >> bs) & 0xff;
            if (argb) {
                unsigned a = p >> 24;           /* the ARGB visual: alpha on top */
                if (a < amin) continue;
                if (a < 255) {                  /* un-premultiply, rounded (as the gpu shader) */
                    R = (R * 510 + a) / (2 * a); if (R > 255) R = 255;
                    G = (G * 510 + a) / (2 * a); if (G > 255) G = 255;
                    B = (B * 510 + a) / (2 * a); if (B > 255) B = 255;
                }
            }
            d[x] = c->lut[(R * 77 + G * 150 + B * 29) >> 8];
        }
    }
}

#ifdef __aarch64__
static inline uint8x16_t rev16(uint8x16_t v) { v = vrev64q_u8(v); return vextq_u8(v, v, 8); }
#endif

/* portrait grey pg (bw x bh) -> panel order g[x * bh + (bh - 1 - y)]. NEON:
 * 16x16 tiles transposed in registers (trn at 8/16/32/64 bits), each column
 * stored reversed; the ragged edges and other CPUs: 32x32 blocks (a plain
 * column-order loop misses the cache on every pixel). rotbench on the
 * device: 1.5 ns/px vs 2.1 blocked, bit-identical. */
void comp_rotate(const uint8_t *pg, uint8_t *g, int bw, int bh)
{
    int bw16 = 0, bh16 = 0;
#ifdef __aarch64__
    bw16 = bw & ~15;
    bh16 = bh & ~15;
    for (int y0 = 0; y0 < bh16; y0 += 16)
        for (int x0 = 0; x0 < bw16; x0 += 16) {
            uint8x16_t r[16], t[16];
            for (int i = 0; i < 16; i++) r[i] = vld1q_u8(pg + (size_t)(y0 + i) * bw + x0);
            for (int i = 0; i < 16; i += 2) { t[i] = vtrn1q_u8(r[i], r[i + 1]); t[i + 1] = vtrn2q_u8(r[i], r[i + 1]); }
            for (int i = 0; i < 16; i += 4)
                for (int k = 0; k < 2; k++) {
                    uint16x8_t a = vreinterpretq_u16_u8(t[i + k]), e = vreinterpretq_u16_u8(t[i + k + 2]);
                    r[i + k] = vreinterpretq_u8_u16(vtrn1q_u16(a, e));
                    r[i + k + 2] = vreinterpretq_u8_u16(vtrn2q_u16(a, e));
                }
            for (int i = 0; i < 16; i += 8)
                for (int k = 0; k < 4; k++) {
                    uint32x4_t a = vreinterpretq_u32_u8(r[i + k]), e = vreinterpretq_u32_u8(r[i + k + 4]);
                    t[i + k] = vreinterpretq_u8_u32(vtrn1q_u32(a, e));
                    t[i + k + 4] = vreinterpretq_u8_u32(vtrn2q_u32(a, e));
                }
            for (int k = 0; k < 8; k++) {
                uint64x2_t a = vreinterpretq_u64_u8(t[k]), e = vreinterpretq_u64_u8(t[k + 8]);
                r[k] = vreinterpretq_u8_u64(vtrn1q_u64(a, e));
                r[k + 8] = vreinterpretq_u8_u64(vtrn2q_u64(a, e));
            }
            /* r[j]: column x0 + j, rows y0 .. y0 + 15 */
            for (int j = 0; j < 16; j++) vst1q_u8(g + (size_t)(x0 + j) * bh + (bh - 16 - y0), rev16(r[j]));
        }
#endif
    /* what the tiles left: all of it (no NEON), else the right strip of the
     * tiled rows and the bottom rows */
    for (int y0 = 0; y0 < bh; y0 += 32)
        for (int x0 = 0; x0 < bw; x0 += 32) {
            int y1 = y0 + 32 < bh ? y0 + 32 : bh, x1 = x0 + 32 < bw ? x0 + 32 : bw;
            if (y1 <= bh16 && x1 <= bw16) continue;
            for (int x = x0; x < x1; x++) {
                uint8_t *d = g + (size_t)x * bh + (bh - 1);
                for (int y = y0; y < y1; y++)
                    if (y >= bh16 || x >= bw16) d[-y] = pg[(size_t)y * bw + x];
            }
        }
}

static int cpu_read(comp *c, box b, uint8_t *g)
{
    int bw = b.x1 - b.x0, bh = b.y1 - b.y0;
    size_t n = (size_t)bw * bh;
    if (n > c->pgcap) {
        uint8_t *p = realloc(c->pg, n);
        if (!p) return -1;
        c->pg = p;
        c->pgcap = n;
    }
    memset(c->pg, c->white, n);
    /* the topmost opaque, unshaped window covering all of b hides everything
     * under it. An ARGB one (Firefox, GTK4 apps) counts when every pixel of b is
     * opaque, which only its pixels tell: it is fetched here and that image
     * reused below (the wallpaper under Firefox was fetched and blended
     * on every read, half the time of a scroll read). */
    int start = 0, pre_i = -1;
    XImage *pre = NULL;
    for (int i = c->norder - 1; i >= 0; i--) {
        cwin *w = c->order[i] == c->skip ? NULL : find(c, c->order[i]);
        if (!w || !w->viewable || w->input_only) continue;
        box wb = win_box(w);
        if (w->shp || wb.x0 > b.x0 || wb.y0 > b.y0 || wb.x1 < b.x1 || wb.y1 < b.y1) continue;
        if (w->depth == 24) { start = i; break; }
        if (w->depth != 32 || name_pix(c, w)) continue;
        double t0 = tsec();
        XImage *img = fetch(c, w, wb, b);
        c->t_x += tsec() - t0;
        if (!img) { unname(c, w); continue; }
        if (img->bits_per_pixel == 32 && img_opaque(img)) { start = i; pre = img; pre_i = i; break; }
        drop(c, img);           /* translucent somewhere: look further down (the shm
                                 * buffer is shared, so no second image is kept) */
    }
    c->skipped += start;
    box rs_[1];
    for (int i = start; i < c->norder; i++) {
        cwin *w = c->order[i] == c->skip ? NULL : find(c, c->order[i]);
        if (!w || !w->viewable || w->input_only) continue;
        box wb = win_box(w);
        if (box_empty(box_clip(wb, b))) continue;
        if (w->depth != 24 && w->depth != 32) continue;
        int nr = w->shp ? w->nshp : 1;
        if (!nr) continue;
        box *rects = nr > 1 ? malloc(sizeof *rects * nr) : rs_;
        if (!rects) continue;
        nr = win_rects(w, wb, b, rects);
        if (i == pre_i && pre) {
            double t1 = tsec();
            if (pre->bits_per_pixel == 32) blend(c, w, pre, b, b, b);
            c->t_blend += tsec() - t1;
            drop(c, pre);
            pre = NULL;
            c->fetches++;
            c->draws++;
        } else if (nr && name_pix(c, w) == 0) {
            /* one fetch of all the rectangles' bounds (a rounded popup's mask
             * is dozens of them), each blended out of it */
            box u = rects[0];
            for (int k = 1; k < nr; k++) u = box_union(u, rects[k]);
            double t0 = tsec();
            XImage *img = fetch(c, w, wb, u);
            double t1 = tsec();
            c->t_x += t1 - t0;
            if (!img) unname(c, w);             /* gone meanwhile: renamed next time */
            else {
                if (img->bits_per_pixel == 32)
                    for (int k = 0; k < nr; k++) blend(c, w, img, u, rects[k], b);
                c->t_blend += tsec() - t1;
                drop(c, img);
                c->fetches++;
                c->draws++;
            }
        }
        if (rects != rs_) free(rects);
    }
    /* portrait -> panel order: g[(px - b.x0) * bh + (b.y1 - 1 - py)] */
    double t0 = tsec();
    comp_rotate(c->pg, g, bw, bh);
    c->t_rot += tsec() - t0;
    return 0;
}

/* ---- gpu backend ------------------------------------------------------------ */

/* the window's pixmap as a texture */
static int name_tex(comp *c, cwin *w)
{
    if (w->tex) return 0;
    if (name_pix(c, w) < 0) return -1;
    const EGLint at[] = { EGL_IMAGE_PRESERVED_KHR, EGL_TRUE, EGL_NONE };
    w->img = c->create_image(c->edpy, EGL_NO_CONTEXT, EGL_NATIVE_PIXMAP_KHR, (EGLClientBuffer)(uintptr_t)w->pix, at);
    if (w->img == EGL_NO_IMAGE_KHR) {           /* e.g. the window went away meanwhile */
        XFreePixmap(c->dpy, w->pix);
        w->pix = 0;
        return -1;
    }
    glGenTextures(1, &w->tex);
    glBindTexture(GL_TEXTURE_2D, w->tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    c->image_target(GL_TEXTURE_2D, (GLeglImageOES)w->img);
    c->names++;
    return 0;
}

/* portrait (x, y) -> panel (PW - y, x) -> clip space; texcoords straight
 * (pixmap row 0 = texture t 0) */
static const char *VS =
    "attribute vec2 pos; attribute vec2 tc; uniform vec2 scale; varying vec2 v;\n"
    "void main() { v = tc; gl_Position = vec4((scale.x - pos.y) / scale.x * 2.0 - 1.0,\n"
    "                                         pos.x / scale.y * 2.0 - 1.0, 0.0, 1.0); }\n";
static const char *FS =
    "precision highp float;\n"
    "uniform sampler2D tex; uniform sampler2D lut; uniform float alpha; uniform float cut;\n"
    "varying vec2 v;\n"
    "void main() {\n"
    "  vec4 p = texture2D(tex, v);\n"
    "  float a = alpha > 0.5 ? p.a : 1.0;\n"
    "  if (a < cut) discard;\n"
    "  vec3 c = alpha > 0.5 ? p.rgb / max(a, 0.004) : p.rgb;\n"
    "  float l = floor(dot(floor(c * 255.0 + 0.5), vec3(77.0, 150.0, 29.0)) / 256.0);\n"
    "  gl_FragColor = vec4(texture2D(lut, vec2((l + 0.5) / 256.0, 0.5)).r, 0.0, 0.0, 1.0);\n"
    "}\n";

static GLuint shader(GLenum t, const char *src)
{
    GLuint s = glCreateShader(t);
    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(s, sizeof log, NULL, log);
        fprintf(stderr, "glassine-epdd-desktop: comp: shader: %s\n", log);
        return 0;
    }
    return s;
}

static int gl_setup(comp *c)
{
    GLuint vs = shader(GL_VERTEX_SHADER, VS), fs = shader(GL_FRAGMENT_SHADER, FS);
    if (!vs || !fs) return -1;
    c->prog = glCreateProgram();
    glAttachShader(c->prog, vs);
    glAttachShader(c->prog, fs);
    glLinkProgram(c->prog);
    GLint ok = 0;
    glGetProgramiv(c->prog, GL_LINK_STATUS, &ok);
    if (!ok) { fprintf(stderr, "glassine-epdd-desktop: comp: link failed\n"); return -1; }
    c->a_pos = glGetAttribLocation(c->prog, "pos");
    c->a_tc = glGetAttribLocation(c->prog, "tc");
    c->u_tex = glGetUniformLocation(c->prog, "tex");
    c->u_lut = glGetUniformLocation(c->prog, "lut");
    c->u_alpha = glGetUniformLocation(c->prog, "alpha");
    c->u_cut = glGetUniformLocation(c->prog, "cut");
    c->u_scale = glGetUniformLocation(c->prog, "scale");

    /* the panel: RGBA8 and an RGBA/UNSIGNED_BYTE readback, the one format pair
     * every GLES2 driver must support (R8 + GL_RED is gone) */
    glGenTextures(1, &c->fbtex);
    glBindTexture(GL_TEXTURE_2D, c->fbtex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, c->PW, c->PH, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glGenFramebuffers(1, &c->fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, c->fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, c->fbtex, 0);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        fprintf(stderr, "glassine-epdd-desktop: comp: no framebuffer\n");
        return -1;
    }
    if (!(c->rgba = malloc((size_t)c->PW * c->PH * 4))) return -1;
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    glGenTextures(1, &c->lut_tex);
    glBindTexture(GL_TEXTURE_2D, c->lut_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, 256, 1, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, c->lut);

    glGenBuffers(1, &c->vbo);
    glViewport(0, 0, c->PW, c->PH);
    glDisable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_SCISSOR_TEST);
    glFinish();
    return glGetError() == GL_NO_ERROR ? 0 : -1;
}

static int has_ext(const char *list, const char *e)
{
    size_t n = strlen(e);
    for (const char *p = list; p && (p = strstr(p, e)); p += n)
        if ((p == list || p[-1] == ' ') && (p[n] == ' ' || !p[n])) return 1;
    return 0;
}

/* marker: created (and synced to disk) before the first GPU submit, deleted by
 * comp_close; still there at start = the last GPU compositing never ended
 * cleanly */
static int marker_set(void)
{
    char dir[128];
    snprintf(dir, sizeof dir, "%s", GPU_MARKER);
    char *sl = strrchr(dir, '/');
    if (sl) { *sl = 0; mkdir(dir, 0755); }
    int fd = open(GPU_MARKER, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) return -1;
    dprintf(fd, "glassine-epdd-desktop pid %d: GPU compositing; left behind = it did not end cleanly (delete to allow it again)\n", (int)getpid());
    fsync(fd);
    close(fd);
    if (sl && (fd = open(dir, O_RDONLY | O_DIRECTORY)) >= 0) { fsync(fd); close(fd); }
    return 0;
}

static void gpu_close(comp *c)
{
    if (c->fence) { XSyncDestroyFence(c->dpy, c->fence); c->fence = 0; }
    if (c->edpy == EGL_NO_DISPLAY) return;
    if (c->ctx != EGL_NO_CONTEXT) {
        glFinish();
        for (int i = 0; i < c->nwin; i++) {
            cwin *w = &c->win[i];
            if (w->tex) { glDeleteTextures(1, &w->tex); w->tex = 0; }
            if (w->img) { c->destroy_image(c->edpy, w->img); w->img = EGL_NO_IMAGE_KHR; }
        }
        eglMakeCurrent(c->edpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        eglDestroyContext(c->edpy, c->ctx);
        c->ctx = EGL_NO_CONTEXT;
    }
    eglTerminate(c->edpy);
    c->edpy = EGL_NO_DISPLAY;
    free(c->rgba); c->rgba = NULL;
    free(c->quads); c->quads = NULL; c->nquads = 0;
}

/* leave the gpu for good (this process): cpu from the next read on */
static void to_cpu(comp *c, const char *why)
{
    fprintf(stderr, "glassine-epdd-desktop: comp: %s: compositing on the CPU from now on\n", why);
    gpu_close(c);
    c->gpu = 0;
    cpu_open(c);
    unlink(GPU_MARKER);
}

static int gpu_open(comp *c)
{
    if (access(GPU_MARKER, F_OK) == 0) {
        fprintf(stderr, "glassine-epdd-desktop: comp: %s exists: the last GPU compositing did not end cleanly "
                "(hang? crash?), not again until it is deleted\n", GPU_MARKER);
        return -1;
    }
    PFNEGLGETPLATFORMDISPLAYEXTPROC gpd = (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");
    c->edpy = gpd ? gpd(EGL_PLATFORM_X11_KHR, c->dpy, NULL) : eglGetDisplay((EGLNativeDisplayType)c->dpy);
    EGLint ema, emi;
    if (c->edpy == EGL_NO_DISPLAY || !eglInitialize(c->edpy, &ema, &emi)) {
        fprintf(stderr, "glassine-epdd-desktop: comp: no EGL on X\n");
        c->edpy = EGL_NO_DISPLAY;
        return -1;
    }
    c->ctx = EGL_NO_CONTEXT;
    const char *ex = eglQueryString(c->edpy, EGL_EXTENSIONS);
    if (!has_ext(ex, "EGL_KHR_image_pixmap") || !has_ext(ex, "EGL_KHR_surfaceless_context") ||
        !has_ext(ex, "EGL_KHR_no_config_context")) {
        fprintf(stderr, "glassine-epdd-desktop: comp: EGL lacks image_pixmap / surfaceless / no_config\n");
        gpu_close(c);
        return -1;
    }
    eglBindAPI(EGL_OPENGL_ES_API);
    int robust = has_ext(ex, "EGL_EXT_create_context_robustness");
    const EGLint car[] = { EGL_CONTEXT_CLIENT_VERSION, 2,
                           EGL_CONTEXT_OPENGL_RESET_NOTIFICATION_STRATEGY_EXT, EGL_LOSE_CONTEXT_ON_RESET_EXT, EGL_NONE };
    const EGLint ca[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };
    if (robust) c->ctx = eglCreateContext(c->edpy, EGL_NO_CONFIG_KHR, EGL_NO_CONTEXT, car);
    if (c->ctx == EGL_NO_CONTEXT) { robust = 0; c->ctx = eglCreateContext(c->edpy, EGL_NO_CONFIG_KHR, EGL_NO_CONTEXT, ca); }
    if (c->ctx == EGL_NO_CONTEXT || !eglMakeCurrent(c->edpy, EGL_NO_SURFACE, EGL_NO_SURFACE, c->ctx)) {
        fprintf(stderr, "glassine-epdd-desktop: comp: no GLES context\n");
        gpu_close(c);
        return -1;
    }
    if (gl_load() < 0) { fprintf(stderr, "glassine-epdd-desktop: comp: GLES functions missing\n"); gpu_close(c); return -1; }
    snprintf(c->renderer, sizeof c->renderer, "%s", (const char *)glGetString(GL_RENDERER));
    /* a software GL (the dummy X has no DRI3: EGL falls back to llvmpipe)
     * would be slower than the cpu backend */
    if ((strstr(c->renderer, "llvmpipe") || strstr(c->renderer, "softpipe")) && !getenv("GLASSINE_EPDD_COMP_SW")) {
        fprintf(stderr, "glassine-epdd-desktop: comp: renderer %s is software\n", c->renderer);
        gpu_close(c);
        return -1;
    }
    if (robust && has_ext((const char *)glGetString(GL_EXTENSIONS), "GL_EXT_robustness"))
        c->reset_status = (reset_status_fn)eglGetProcAddress("glGetGraphicsResetStatusEXT");
    c->create_image = (PFNEGLCREATEIMAGEKHRPROC)eglGetProcAddress("eglCreateImageKHR");
    c->destroy_image = (PFNEGLDESTROYIMAGEKHRPROC)eglGetProcAddress("eglDestroyImageKHR");
    c->image_target = (PFNGLEGLIMAGETARGETTEXTURE2DOESPROC)eglGetProcAddress("glEGLImageTargetTexture2DOES");
    if (!c->create_image || !c->destroy_image || !c->image_target) { gpu_close(c); return -1; }
    if (marker_set() < 0) {
        fprintf(stderr, "glassine-epdd-desktop: comp: cannot write %s (%s)\n", GPU_MARKER, strerror(errno));
        gpu_close(c);
        return -1;
    }
    if (gl_setup(c) < 0) { gpu_close(c); unlink(GPU_MARKER); return -1; }
    int sev, ser, smaj, smin;
    if (XSyncQueryExtension(c->dpy, &sev, &ser) && XSyncInitialize(c->dpy, &smaj, &smin) && (smaj > 3 || (smaj == 3 && smin >= 1)))
        c->fence = XSyncCreateFence(c->dpy, c->root, False);
    fprintf(stderr, "glassine-epdd-desktop: comp: GPU %s, EGL %d.%d, %s, %s\n", c->renderer, ema, emi,
            c->reset_status ? "reset notification" : "no reset notification",
            c->fence ? "X fence sync" : "NO X fence (XSync only)");
    return 0;
}

static int gpu_read(comp *c, box b, uint8_t *g)
{
    int bx0 = c->PW - b.y1, bw = b.y1 - b.y0, by0 = b.x0, bh = b.x1 - b.x0;
    /* XSync alone = X has *processed* the drawing requests, not that the GPU
     * has executed them: sampled then, a window could come without the glyphs
     * just drawn (missing letters, stale popups). Triggering an X fence
     * makes glamor finish its rendering first (as picom's xrender-sync-fence) */
    if (c->fence) {
        XSyncTriggerFence(c->dpy, c->fence);
        XSyncAwaitFence(c->dpy, &c->fence, 1);
        XSyncResetFence(c->dpy, c->fence);
    }
    XSync(c->dpy, False);
    double t0 = tsec();
    c->t_x += t0 - c->t_start;
    glBindFramebuffer(GL_FRAMEBUFFER, c->fbo);
    glScissor(bx0, by0, bw, bh);
    glClearColor(c->white / 255.0f, 0, 0, 1);   /* no window: lut(255), as the root is white */
    glClear(GL_COLOR_BUFFER_BIT);
    glUseProgram(c->prog);
    glUniform1i(c->u_tex, 0);
    glUniform1i(c->u_lut, 1);
    glUniform1f(c->u_cut, c->cut);
    glUniform2f(c->u_scale, (float)c->PW, (float)c->PH);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, c->lut_tex);
    glActiveTexture(GL_TEXTURE0);
    glBindBuffer(GL_ARRAY_BUFFER, c->vbo);
    glEnableVertexAttribArray(c->a_pos);
    glEnableVertexAttribArray(c->a_tc);
    glVertexAttribPointer(c->a_pos, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)0);
    glVertexAttribPointer(c->a_tc, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)(2 * sizeof(float)));
    for (int i = 0; i < c->norder; i++) {
        cwin *w = c->order[i] == c->skip ? NULL : find(c, c->order[i]);
        if (!w || !w->viewable || w->input_only) continue;
        box wb = win_box(w);
        if (box_empty(box_clip(wb, b))) continue;
        int nr = w->shp ? w->nshp : 1;
        if (!nr) continue;
        if (nr > c->nquads) {
            float *q = realloc(c->quads, sizeof(float) * 24 * nr);
            if (!q) continue;
            c->quads = q;
            c->nquads = nr;
        }
        box *rects = malloc(sizeof *rects * nr);
        if (!rects) continue;
        nr = win_rects(w, wb, b, rects);
        if (!nr || name_tex(c, w) < 0) { free(rects); continue; }
        /* two triangles per rectangle */
        float fw = wb.x1 - wb.x0, fh = wb.y1 - wb.y0, *q = c->quads;
        for (int k = 0; k < nr; k++) {
            float x0 = rects[k].x0, y0 = rects[k].y0, x1 = rects[k].x1, y1 = rects[k].y1;
            float s0 = (x0 - wb.x0) / fw, t0 = (y0 - wb.y0) / fh, s1 = (x1 - wb.x0) / fw, t1 = (y1 - wb.y0) / fh;
            const float v[24] = { x0, y0, s0, t0,  x1, y0, s1, t0,  x0, y1, s0, t1,
                                  x1, y0, s1, t0,  x1, y1, s1, t1,  x0, y1, s0, t1 };
            memcpy(q + k * 24, v, sizeof v);
        }
        free(rects);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 24 * nr, q, GL_STREAM_DRAW);
        glBindTexture(GL_TEXTURE_2D, w->tex);
        glUniform1f(c->u_alpha, w->depth == 32 ? 1.0f : 0.0f);
        glDrawArrays(GL_TRIANGLES, 0, 6 * nr);
        c->draws++;
    }
    glFinish();                                 /* ours done before the readback */
    double t1 = tsec();
    c->t_blend += t1 - t0;
    glReadPixels(bx0, by0, bw, bh, GL_RGBA, GL_UNSIGNED_BYTE, c->rgba);
    glFinish();                                 /* nothing of ours in flight between reads */
    for (long i = 0, n = (long)bw * bh; i < n; i++) g[i] = c->rgba[i * 4];
    c->t_rot += tsec() - t1;
    GLenum err = glGetError();
    if (c->reset_status && c->reset_status() != GL_NO_ERROR) {
        to_cpu(c, "GPU reset");
        return cpu_read(c, b, g);
    }
    if (err != GL_NO_ERROR) {
        char why[48];
        snprintf(why, sizeof why, "GL error 0x%x", err);
        if (++c->gl_errors >= 3) { to_cpu(c, why); return cpu_read(c, b, g); }
        fprintf(stderr, "glassine-epdd-desktop: comp: %s\n", why);
        return -1;
    }
    c->gl_errors = 0;
    return 0;
}

/* ---- the interface ------------------------------------------------------------ */

void comp_set_lut(comp *c, const uint8_t lut[256])
{
    memcpy(c->lut, lut, 256);
    c->white = lut[255];
    if (c->gpu) {
        glBindTexture(GL_TEXTURE_2D, c->lut_tex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, 256, 1, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, lut);
    }
}

void comp_set_alpha_cut(comp *c, float cut) { c->cut = cut < 0 ? 0 : cut > 1 ? 1 : cut; }

const char *comp_renderer(comp *c) { return c->gpu ? c->renderer : "cpu"; }

void comp_stats(comp *c, char *buf, int len)
{
    double mp = c->px / 1e6;
    const char *what = c->gpu ? "X sync %.1f ms, draw %.1f ms, readback" : "X copy %.1f ms, blend %.1f ms, rotate";
    char fmt[256];
    snprintf(fmt, sizeof fmt, "comp: %%d windows, %%lu reads, %%lu window draws, %%lu fetches, %%lu skipped (covered), %%lu textures named; "
             "%%.2f Mpx read: %s %%.1f ms, gpu %%.1f ms (per Mpx: %%.1f / %%.1f / %%.1f / %%.1f ms)", what);
    snprintf(buf, len, fmt,
             c->nwin, c->reads, c->draws, c->fetches, c->skipped, c->names, mp,
             c->t_x * 1e3, c->t_blend * 1e3, c->t_rot * 1e3, c->t_gpu * 1e3,
             mp ? c->t_x * 1e3 / mp : 0, mp ? c->t_blend * 1e3 / mp : 0, mp ? c->t_rot * 1e3 / mp : 0, mp ? c->t_gpu * 1e3 / mp : 0);
    c->reads = c->draws = c->fetches = c->skipped = c->names = 0;
    c->t_x = c->t_blend = c->t_rot = c->t_gpu = 0;
    c->px = 0;
}

/* a window every read leaves out (0: none): glassine-epdd-desktop paints a dragged tile
 * handle itself, and what X still has at the handle's old place is what lies
 * under it */
void comp_skip(comp *c, Window w) { c->skip = w; }

int comp_read(comp *c, box b, uint8_t *g)
{
    b = box_clip(b, (box){ 0, 0, c->W, c->H });
    if (box_empty(b)) return -1;
    if (c->order_stale) restack(c);
    XErrorHandler old = XSetErrorHandler(ignore_xerr);
    double t0 = c->t_start = tsec();
    int r = c->gpu ? gpu_read(c, b, g) : cpu_read(c, b, g);
    if (c->gpu) c->t_gpu += tsec() - t0;
    XSetErrorHandler(old);
    c->reads++;
    c->px += (unsigned long long)(b.x1 - b.x0) * (b.y1 - b.y0);
    return r;
}

const char *comp_mode(void)
{
    const char *m = getenv("GLASSINE_EPDD_COMP");
    if (getenv("GLASSINE_EPDD_NOCOMP")) return "off";
    return m && *m ? m : "cpu";
}

comp *comp_open(Display *dpy, Window root, int W, int H, int PW, int PH)
{
    int ev, er, maj = 0, min = 0;
    const char *mode = comp_mode();
    if (!strcmp(mode, "off")) return NULL;
    if (!XCompositeQueryExtension(dpy, &ev, &er) || !XCompositeQueryVersion(dpy, &maj, &min) ||
        (maj == 0 && min < 2)) {
        fprintf(stderr, "glassine-epdd-desktop: comp: no Composite 0.2 in X: reading the root\n");
        return NULL;
    }
    comp *c = calloc(1, sizeof *c);
    if (!c) return NULL;
    c->dpy = dpy; c->root = root; c->W = W; c->H = H; c->PW = PW; c->PH = PH; c->cut = 0.5f; c->white = 255;
    c->edpy = EGL_NO_DISPLAY;
    c->ctx = EGL_NO_CONTEXT;
    c->shm.shmid = -1;
    for (int i = 0; i < 256; i++) c->lut[i] = i;
    if (!XDamageQueryExtension(dpy, &c->dmg_ev, &er)) { free(c); return NULL; }
    if (!XShapeQueryExtension(dpy, &c->shape_ev, &er)) c->shape_ev = -1;

    /* manual redirection; refused while another compositor (xfwm4's) owns the
     * CM selection. The selection itself is NOT claimed : it tells the
     * toolkits a compositor runs, and they answer with ARGB windows, GTK4 with
     * a transparent shadow/resize margin around every CSD window (>= 12 px
     * whatever the CSS says, gtkwindow.c get_shadow_width) that showed the
     * wallpaper, rounded corners, translucent popovers. Without it every app
     * draws opaque, square, margin-free windows; ARGB windows that exist anyway
     * are still blended. GLASSINE_EPDD_CM=1 claims it as before. */
    char sel[32];
    snprintf(sel, sizeof sel, "_NET_WM_CM_S%d", DefaultScreen(dpy));
    Atom cm = XInternAtom(dpy, sel, False);
    if (XGetSelectionOwner(dpy, cm) != None) {
        fprintf(stderr, "glassine-epdd-desktop: comp: another compositing manager owns %s (xfwm4 use_compositing?): reading the root\n", sel);
        free(c);
        return NULL;
    }
    if (!strcmp(mode, "gpu") && gpu_open(c) == 0) c->gpu = 1;
    else if (!strcmp(mode, "gpu")) fprintf(stderr, "glassine-epdd-desktop: comp: no GPU compositing: the CPU composites\n");
    if (!c->gpu) cpu_open(c);

    c->cm_win = XCreateSimpleWindow(dpy, root, -10, -10, 1, 1, 0, 0, 0);
    XStoreName(dpy, c->cm_win, "glassine-epdd-desktop compositor");
    if (getenv("GLASSINE_EPDD_CM") && atoi(getenv("GLASSINE_EPDD_CM"))) XSetSelectionOwner(dpy, cm, c->cm_win, CurrentTime);
    XCompositeRedirectSubwindows(dpy, root, CompositeRedirectManual);
    XSync(dpy, False);
    c->region = XFixesCreateRegion(dpy, NULL, 0);
    {
        Window r, p, *kids = NULL;
        unsigned int n = 0;
        if (XQueryTree(dpy, root, &r, &p, &kids, &n))
            for (unsigned int i = 0; i < n; i++) track(c, kids[i]);
        if (kids) XFree(kids);
    }
    c->order_stale = 1;
    fprintf(stderr, "glassine-epdd-desktop: compositing on the %s (%s), %d windows\n",
            c->gpu ? "GPU" : "CPU", c->gpu ? c->renderer : c->shm_ok ? "MIT-SHM" : "XGetImage", c->nwin);
    return c;
}

void comp_close(comp *c)
{
    if (!c) return;
    if (c->gpu) {
        gpu_close(c);
        unlink(GPU_MARKER);                     /* ended cleanly */
    }
    if (c->shm_ok) { XShmDetach(c->dpy, &c->shm); shmdt(c->shm.shmaddr); c->shm_ok = 0; }
    for (int i = 0; i < c->nwin; i++) {
        if (c->win[i].pix) XFreePixmap(c->dpy, c->win[i].pix);
        free(c->win[i].shp);
    }
    XCompositeUnredirectSubwindows(c->dpy, c->root, CompositeRedirectManual);
    XDestroyWindow(c->dpy, c->cm_win);
    XFixesDestroyRegion(c->dpy, c->region);
    XSync(c->dpy, False);
    free(c->pg);
    free(c);
}
