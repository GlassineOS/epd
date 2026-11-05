/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-ingest — see epd-ingest.h */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE                     /* ('s pool: pthread_setaffinity_np) */
#endif
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl31.h>
#include <GLES2/gl2ext.h>
#include "epd-ingest.h"
#include "epd-grey.h"
#include "epd-dither.h"
#include "epd-dizzy.h"
#include "epd-gldebug.h"
#include "epd-ingest-cpu.h"

/* GLES through eglGetProcAddress (as epd-gpu.c: libGLESv2 need not be installed) */
#define GL_FNS \
    F(PFNGLACTIVETEXTUREPROC, glActiveTexture) \
    F(PFNGLATTACHSHADERPROC, glAttachShader) \
    F(PFNGLBINDFRAMEBUFFERPROC, glBindFramebuffer) \
    F(PFNGLBLITFRAMEBUFFERPROC, glBlitFramebuffer) \
    F(PFNGLFINISHPROC, glFinish) \
    F(PFNGLFLUSHPROC, glFlush) \
    F(PFNGLUNIFORM1UIVPROC, glUniform1uiv) \
    F(PFNGLUNIFORM4IPROC, glUniform4i) \
    F(PFNGLUNIFORM2IPROC, glUniform2i) \
    F(PFNGLBINDTEXTUREPROC, glBindTexture) \
    F(PFNGLBINDVERTEXARRAYPROC, glBindVertexArray) \
    F(PFNGLCHECKFRAMEBUFFERSTATUSPROC, glCheckFramebufferStatus) \
    F(PFNGLCLIENTWAITSYNCPROC, glClientWaitSync) \
    F(PFNGLCOMPILESHADERPROC, glCompileShader) \
    F(PFNGLCREATEPROGRAMPROC, glCreateProgram) \
    F(PFNGLCREATESHADERPROC, glCreateShader) \
    F(PFNGLDELETEPROGRAMPROC, glDeleteProgram) \
    F(PFNGLDELETESHADERPROC, glDeleteShader) \
    F(PFNGLDELETESYNCPROC, glDeleteSync) \
    F(PFNGLDELETETEXTURESPROC, glDeleteTextures) \
    F(PFNGLDISABLEPROC, glDisable) \
    F(PFNGLDRAWARRAYSPROC, glDrawArrays) \
    F(PFNGLENABLEPROC, glEnable) \
    F(PFNGLFENCESYNCPROC, glFenceSync) \
    F(PFNGLFRAMEBUFFERTEXTURE2DPROC, glFramebufferTexture2D) \
    F(PFNGLGENFRAMEBUFFERSPROC, glGenFramebuffers) \
    F(PFNGLGENTEXTURESPROC, glGenTextures) \
    F(PFNGLGENVERTEXARRAYSPROC, glGenVertexArrays) \
    F(PFNGLGETERRORPROC, glGetError) \
    F(PFNGLGETPROGRAMINFOLOGPROC, glGetProgramInfoLog) \
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
    F(PFNGLTEXPARAMETERIPROC, glTexParameteri) \
    F(PFNGLTEXSTORAGE2DPROC, glTexStorage2D) \
    F(PFNGLTEXSUBIMAGE2DPROC, glTexSubImage2D) \
    F(PFNGLUNIFORM1IPROC, glUniform1i) \
    F(PFNGLUSEPROGRAMPROC, glUseProgram) \
    F(PFNGLVIEWPORTPROC, glViewport)

#define MAXIMG 16

struct epd_ingest {
    EGLDisplay dpy;
    EGLContext ctx;
#define F(t, n) t n;
    GL_FNS
#undef F
    PFNGLEGLIMAGETARGETTEXTURE2DOESPROC glEGLImageTargetTexture2DOES;
    PFNEGLCREATEIMAGEKHRPROC eglCreateImageKHR;
    PFNEGLDESTROYIMAGEKHRPROC eglDestroyImageKHR;
    int mods;                           /* EGL_EXT_image_dma_buf_import_modifiers */
    int pw, ph, tw;                     /* panel grey; target texels per row (pw / 4) */
    GLuint prog, lut, out, fbo, vao;
    /* the target in cached memory (a system dma-heap buffer, rendered into
     * through an EGLImage): the CPU reads it at memory speed. Without, the
     * target is a GL texture and glReadPixels copies out of the driver's
     * write-combined staging (~150 MB/s on the A509: 12-25 ms per full frame,
     * measured). hfd < 0: that way */
    int hfd, hstride;
    size_t hsize;
    uint8_t *hmap;
    EGLImageKHR himg;
    /* fd (our dup), stride, off, len, map: the CPU ingest's view of the same
     * buffer (epd_ingest_map: mapped at its first CPU frame) */
    struct { EGLImageKHR img; GLuint tex; int w, h, r8, fd, stride, off; size_t len; uint8_t *map; } im[MAXIMG];
    uint8_t clut[256], clut8[256];      /* the LUTs' copies (lut, lut8) for the CPU ingest */
    int copy8;                          /* lut8 is the identity: R8 frames are copied (sway applied the table) */
    uint8_t *bounce;                    /* its cached rows (epd_ingest_frame_cpu) */
    /* (epd-ingest-cpu.h): the bands' pool, the sync helper, the
     * row way's bounce per part; totals for epd_ingest_cpu_stats */
    icpu_pool pool;
    icpu_sync sy;
    uint8_t *pbnc[ICPU_BANDS_MAX];
    struct { unsigned long frames, banded, ranged; double wait, read, rmax, rbytes; } s3c;
    GLuint prog8;                       /* R8 frames (sway's grey output): the LUT only, no turn */
    GLuint progl;                       /* XRGB frames, the luminance q (epd-grey.h) */
    GLuint sqt, lut8;                   /* epd_grey_sq (unit 6), the R8 frames' LUT (unit 7) */
    int lum;                            /* XRGB frames: progl, else prog (luma) */
    int dead;
    GLsync dead_f;                      /* the fence a timeout abandoned (epd_ingest_revive) */
    char name[128];
    /* stage 5 (epd_ingest_mirror). valid: per TILE x TILE panel
     * tile, 1 = the target above (out) equals the caller's grey image there
     * (a frame of ours covered it whole since the caller last wrote it) */
    int ttw, tth;
    uint8_t *valid;
    int mir;                            /* the mirror is on (gsh = shadow) */
    GLuint gsh[2], fgsh[2];             /* the mirror (RGBA8 like out) + the commit pass's scratch */
    GLuint pcommit, pmake;
    GLint uc_box, uc_how, uc_all;
    GLuint dtex;                        /* the dither's thresholds (epd-dither.h, 64 x 64 R8UI, unit 5) */
    int dpat, dpat_up;                  /* the pattern asked for / in dtex (-1: none yet) */
    int nfence;                         /* EGL_ANDROID_native_fence_sync (else the producer waits) */
    PFNEGLCREATESYNCKHRPROC eglCreateSyncKHR;
    PFNEGLDESTROYSYNCKHRPROC eglDestroySyncKHR;
    PFNEGLDUPNATIVEFENCEFDANDROIDPROC eglDupNativeFenceFDANDROID;
    struct layer {
        GLuint tex, fbo;
        EGLImageKHR img;
        epd_gt_buf b;                   /* its dma-buf (exported) */
        int fence;                      /* sync_file of the last draw, -1 none */
        int used;                       /* a handle out (lmu) */
    } lay[EPD_GT_MAX];
    int nlay;
    pthread_mutex_t lmu;                /* lay[].used, .fence: the engine releases from its thread */
    struct gdz {                        /* dizzy on the GPU (epd_ingest_dizzy): 0 = not tried, 1 = up, -1 = none */
        int st;
        PFNGLDISPATCHCOMPUTEPROC glDispatchCompute;
        PFNGLMEMORYBARRIERPROC glMemoryBarrier;
        PFNGLGENBUFFERSPROC glGenBuffers;
        PFNGLBINDBUFFERPROC glBindBuffer;
        PFNGLBINDBUFFERBASEPROC glBindBufferBase;
        PFNGLBUFFERDATAPROC glBufferData;
        PFNGLMAPBUFFERRANGEPROC glMapBufferRange;
        PFNGLUNMAPBUFFERPROC glUnmapBuffer;
        GLuint prog, plan, bits, gtex, ltex;
        GLint u_rect, u_t0, u_ows;
        size_t bcap;
        uint8_t lv[256];
        int lvt;                        /* lv[] is not the canonical table: the shaders read ltex (else ALU) */
        /* a submitted job (epd_ingest_dizzy_submit), collected by _collect */
        GLsync f;
        int on, x0, y0, w, h, tx0, ty0, ows;
        /* the block form (epd-dizzy.h epd_dizzy_block; ):
         * a program per base order, the pass fixed in it; the mirrors are
         * runtime. bst: 0 not set up, 1 up, -1 failed */
        int form, bst, blsz;
        epd_dz_btabs *bt;
        GLuint bprog[EPD_DZB_MAXORD], btex, inc;
        struct { GLint rect, blk, tl, lim, total; } bu[EPD_DZB_MAXORD];
        size_t icap;
        uint8_t *brow;                  /* the upload's row scratch (whole texels) */
        size_t brcap;
        int bsrc;                       /* the job's grey: 1 = out (the view), 0 = uploaded */
        int bx0, by0, nbxr, nbyr, wpb;  /* the job's blocks: first, count, words a block */
        unsigned long nview, nup;       /* (stats: jobs read from out / uploaded) */
        int bstage;                     /* GLASSINE_GDZ_BSTAGE (timing only): 1 = fetch + levels, no chain */
        GLuint pprog, pbuf;             /* epd_ingest_dizzy_peek: the greys as the block shader fetches them */
        GLint pu_rect;
        size_t pcap;
    } dz;
    /* out's texels the GPU drew since the caller last wrote the grey there
     * (epd_ingest_stale): drawn[row * tw + texel]; the block dizzy reads out
     * where its rect is all drawn (no upload) */
    uint8_t *drawn;
    epd_gt_provider pv;
};
#define TILE 32

#ifndef EGL_PLATFORM_DEVICE_EXT
#define EGL_PLATFORM_DEVICE_EXT 0x313F
#endif
#ifndef EGL_DRM_RENDER_NODE_FILE_EXT
#define EGL_DRM_RENDER_NODE_FILE_EXT 0x3377
#endif
#ifndef EGL_NO_CONFIG_KHR
#define EGL_NO_CONFIG_KHR ((EGLConfig)0)
#endif

/* one fragment per 4 panel pixels of a row: texel (tx, row) = panel columns
 * 4 tx .. 4 tx + 3 of panel row `row` = buffer pixels (row, pw - 1 - col).
 * The integer math of epd-grey.h: the grey value q (luma, or luminance from
 * the S table with the exact integer square root), then the gamma LUT */
static const char *vs =
    "#version 300 es\n"
    "void main() {\n"
    "  vec2 p = vec2(gl_VertexID & 1, gl_VertexID >> 1) * 4.0 - 1.0;\n"
    "  gl_Position = vec4(p, 0.0, 1.0);\n"
    "}\n";
#define FS_HEAD \
    "#version 300 es\n" \
    "precision highp float; precision highp int; precision highp usampler2D;\n" \
    "uniform sampler2D src;    /* the frame, w x h (portrait) */\n" \
    "uniform usampler2D lut;   /* R8UI 256 x 1 */\n" \
    "uniform int pw;\n" \
    "out vec4 o;\n"
#define FS_TAIL \
    "uint grey(int col, int row) {\n" \
    "  uvec3 v = uvec3(round(texelFetch(src, ivec2(row, pw - 1 - col), 0).rgb * 255.0));\n" \
    "  return texelFetch(lut, ivec2(int(q(v)), 0), 0).r;\n" \
    "}\n" \
    "void main() {\n" \
    "  ivec2 c = ivec2(gl_FragCoord.xy);\n" \
    "  int x = c.x * 4;\n" \
    "  o = vec4(grey(x, c.y), grey(x + 1, c.y), grey(x + 2, c.y), grey(x + 3, c.y)) / 255.0;\n" \
    "}\n"
static const char *fs = FS_HEAD
    "uint q(uvec3 v) { return (v.r * 77u + v.g * 150u + v.b * 29u) >> 8; }\n"
    FS_TAIL;
/* epd_grey_q_lum: S from the table, Y = (W. S^2) >> 15, the rounded square
 * root (a float start, then exact: q^2 - q < Y <= q^2 + q) */
static const char *fsl = FS_HEAD
    "uniform usampler2D sq;    /* R8UI 256 x 1: epd_grey_sq */\n"
    "uint q(uvec3 v) {\n"
    "  uint r = texelFetch(sq, ivec2(int(v.r), 0), 0).r, g = texelFetch(sq, ivec2(int(v.g), 0), 0).r,\n"
    "       b = texelFetch(sq, ivec2(int(v.b), 0), 0).r;\n"
    "  uint y = (6966u * r * r + 23436u * g * g + 2366u * b * b) >> 15;\n"
    "  uint k = uint(sqrt(float(y)) + 0.5);\n"
    "  if (y > k * k + k) k++; else if (k > 0u && y <= k * k - k) k--;\n"
    "  return k;\n"
    "}\n"
    FS_TAIL;

/* sway's grey output : an R8 frame already in panel order, its
 * luma before the gamma LUT: texel (tx, row) = LUT of frame pixels (4 tx .. 4 tx + 3, row) */
static const char *fs8 =
    "#version 300 es\n"
    "precision highp float; precision highp int; precision highp usampler2D;\n"
    "uniform sampler2D src;    /* the frame, pw x ph (panel order), R8 */\n"
    "uniform usampler2D lut;   /* R8UI 256 x 1 */\n"
    "out vec4 o;\n"
    "uint grey(int col, int row) {\n"
    "  return texelFetch(lut, ivec2(int(round(texelFetch(src, ivec2(col, row), 0).r * 255.0)), 0), 0).r;\n"
    "}\n"
    "void main() {\n"
    "  ivec2 c = ivec2(gl_FragCoord.xy);\n"
    "  int x = c.x * 4;\n"
    "  o = vec4(grey(x, c.y), grey(x + 1, c.y), grey(x + 2, c.y), grey(x + 3, c.y)) / 255.0;\n"
    "}\n";

static GLuint link_fs(epd_ingest *g, const char *fsrc);
static GLuint shader(epd_ingest *g, GLenum type, const char *src)
{
    GLuint s = g->glCreateShader(type);
    g->glShaderSource(s, 1, &src, NULL);
    g->glCompileShader(s);
    GLint ok = 0;
    g->glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        g->glGetShaderInfoLog(s, sizeof log, NULL, log);
        fprintf(stderr, "epd-ingest: shader: %s\n", log);
        return 0;
    }
    return s;
}

static int has_ext(const char *list, const char *e)
{
    size_t n = strlen(e);
    for (const char *p = list; p && (p = strstr(p, e)); p += n)
        if ((p == list || p[-1] == ' ') && (p[n] == ' ' || !p[n])) return 1;
    return 0;
}

/* a display of its own: the first EGL device with a render node (the GPU, not
 * a software one) */
static EGLDisplay open_display(char *node, size_t len)
{
    PFNEGLQUERYDEVICESEXTPROC qd = (void *)eglGetProcAddress("eglQueryDevicesEXT");
    PFNEGLQUERYDEVICESTRINGEXTPROC qs = (void *)eglGetProcAddress("eglQueryDeviceStringEXT");
    PFNEGLGETPLATFORMDISPLAYEXTPROC gpd = (void *)eglGetProcAddress("eglGetPlatformDisplayEXT");
    if (!qd || !qs || !gpd) return EGL_NO_DISPLAY;
    EGLDeviceEXT dev[8];
    EGLint n = 0;
    if (!qd(8, dev, &n)) return EGL_NO_DISPLAY;
    for (int i = 0; i < n; i++) {
        const char *ex = qs(dev[i], EGL_EXTENSIONS);
        const char *rn = has_ext(ex, "EGL_EXT_device_drm_render_node") ? qs(dev[i], EGL_DRM_RENDER_NODE_FILE_EXT) : NULL;
        if (!rn) continue;
        snprintf(node, len, "%s", rn);
        return gpd(EGL_PLATFORM_DEVICE_EXT, dev[i], NULL);
    }
    return EGL_NO_DISPLAY;
}

static void heap_free(epd_ingest *g)
{
    if (g->hmap) munmap(g->hmap, g->hsize);
    if (g->hfd >= 0) close(g->hfd);
    g->hmap = NULL; g->hfd = -1;
}

/* the target as a system dma-heap buffer (cached pages; the GPU writes them
 * through its IOMMU, DMA_BUF_IOCTL_SYNC invalidates the CPU's lines before a
 * read), rows of tw RGBA8 texels at a pitch the GPU takes. Quietly the
 * texture way on any failure */
static void heap_target(epd_ingest *g)
{
    int h = open("/dev/dma_heap/system", O_RDONLY | O_CLOEXEC);
    if (h < 0) return;
    static const int aligns[] = { 64, 256 };
    for (unsigned a = 0; a < sizeof aligns / sizeof *aligns && g->hfd < 0; a++) {
        int stride = (g->tw * 4 + aligns[a] - 1) / aligns[a] * aligns[a];
        struct dma_heap_allocation_data al = { .len = (size_t)stride * g->ph, .fd_flags = O_RDWR | O_CLOEXEC };
        if (ioctl(h, DMA_HEAP_IOCTL_ALLOC, &al) < 0) break;
        g->hfd = (int)al.fd; g->hsize = al.len; g->hstride = stride;
        EGLint at[] = {
            EGL_WIDTH, g->tw, EGL_HEIGHT, g->ph,
            EGL_LINUX_DRM_FOURCC_EXT, 0x34324241,          /* DRM_FORMAT_ABGR8888: bytes R G B A */
            EGL_DMA_BUF_PLANE0_FD_EXT, g->hfd, EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0,
            EGL_DMA_BUF_PLANE0_PITCH_EXT, stride,
            g->mods ? EGL_DMA_BUF_PLANE0_MODIFIER_LO_EXT : EGL_NONE, 0,
            EGL_DMA_BUF_PLANE0_MODIFIER_HI_EXT, 0,         /* DRM_FORMAT_MOD_LINEAR */
            EGL_NONE };
        g->himg = g->eglCreateImageKHR(g->dpy, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, NULL, at);
        if (g->himg != EGL_NO_IMAGE_KHR) {
            g->glGenTextures(1, &g->out);
            g->glBindTexture(GL_TEXTURE_2D, g->out);
            /* an EGLImage texture has one level and is not immutable: with the
             * default GL_NEAREST_MIPMAP_LINEAR it is mipmap-incomplete, and a
             * shader sampling it gets Mesa's 1 x 1 fallback texture (0, 0, 0, 1)
             * instead -- rendering into it and glReadPixels are unaffected, so
             * the frames were right while the block dizzy's view of out (and
             * fs_commit's G) read 0 (A509, every texel 0, alpha 255
             * at texel 0,0). NEAREST makes it complete; the glTexStorage2D way
             * below is immutable and complete with any filter */
            g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            g->glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, g->himg);
            g->glBindFramebuffer(GL_FRAMEBUFFER, g->fbo);
            g->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, g->out, 0);
            if (g->glGetError() == GL_NO_ERROR && g->glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE &&
                (g->hmap = mmap(NULL, g->hsize, PROT_READ, MAP_SHARED, g->hfd, 0)) != MAP_FAILED)
                break;                  /* done */
            g->hmap = NULL;
            g->glBindFramebuffer(GL_FRAMEBUFFER, 0);
            g->glDeleteTextures(1, &g->out);
            g->out = 0;
            g->eglDestroyImageKHR(g->dpy, g->himg);
        }
        g->himg = NULL;
        while (g->glGetError() != GL_NO_ERROR) ;
        heap_free(g);
    }
    close(h);
}

/* a 256 x 1 R8UI table on the active unit */
static GLuint lut_tex(epd_ingest *g, const uint8_t t[256])
{
    GLuint x;
    g->glGenTextures(1, &x);
    g->glBindTexture(GL_TEXTURE_2D, x);
    g->glTexStorage2D(GL_TEXTURE_2D, 1, GL_R8UI, 256, 1);
    g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    g->glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 256, 1, GL_RED_INTEGER, GL_UNSIGNED_BYTE, t);
    return x;
}

epd_ingest *epd_ingest_open(int pw, int ph, const uint8_t lut[256])
{
    if (pw % 4) { fprintf(stderr, "epd-ingest: panel width %d not a multiple of 4\n", pw); return NULL; }
    epd_ingest *g = calloc(1, sizeof *g);
    if (!g) return NULL;
    g->pw = pw; g->ph = ph; g->tw = pw / 4;
    g->hfd = -1;
    g->ttw = (pw + TILE - 1) / TILE; g->tth = (ph + TILE - 1) / TILE;
    pthread_mutex_init(&g->lmu, NULL);
    for (int i = 0; i < EPD_GT_MAX; i++) g->lay[i].fence = -1;
    if (!(g->valid = calloc((size_t)g->ttw * g->tth, 1))) { free(g); return NULL; }
    if (!(g->drawn = calloc((size_t)g->tw * ph, 1))) { free(g->valid); free(g); return NULL; }
    char node[64] = "";
    g->dpy = open_display(node, sizeof node);
    if (g->dpy == EGL_NO_DISPLAY || !eglInitialize(g->dpy, NULL, NULL)) {
        fprintf(stderr, "epd-ingest: no EGL device with a render node\n");
        g->dpy = EGL_NO_DISPLAY;
        goto fail;
    }
    const char *ex = eglQueryString(g->dpy, EGL_EXTENSIONS);
    if (!has_ext(ex, "EGL_EXT_image_dma_buf_import") || !has_ext(ex, "EGL_KHR_surfaceless_context")) {
        fprintf(stderr, "epd-ingest: EGL lacks dma-buf import / surfaceless\n");
        goto fail;
    }
    g->mods = has_ext(ex, "EGL_EXT_image_dma_buf_import_modifiers");
    g->nfence = has_ext(ex, "EGL_ANDROID_native_fence_sync") && has_ext(ex, "EGL_KHR_fence_sync");
    eglBindAPI(EGL_OPENGL_ES_API);
    /* GLASSINE_EPD_INGEST_PRIO=1 or /etc/glassine/gpu-prio-ingest: a high priority context
     * (msm ring 0, beside sway and the drive frames) instead of the apps' ring 2,
     * where an app frame's ingest waited ~19 ms behind the app's own jobs for
     * ~3 ms of GPU work (trace). Default off */
    const char *pe = getenv("GLASSINE_EPD_INGEST_PRIO");
    FILE *pf = fopen("/etc/glassine/gpu-prio-ingest", "r");
    if (pf) { fclose(pf); pe = "1"; }
    int prio = pe && atoi(pe) > 0 && has_ext(ex, "EGL_IMG_context_priority");
    const EGLint ca[] = { EGL_CONTEXT_MAJOR_VERSION, 3, EGL_NONE };
    const EGLint cp[] = { EGL_CONTEXT_MAJOR_VERSION, 3, 0x3100 /* EGL_CONTEXT_PRIORITY_LEVEL_IMG */, 0x3101 /* HIGH */, EGL_NONE };
    g->ctx = eglCreateContext(g->dpy, EGL_NO_CONFIG_KHR, EGL_NO_CONTEXT, prio ? cp : ca);
    if (prio) fprintf(stderr, "epd-ingest: context priority high\n");
    if (g->ctx == EGL_NO_CONTEXT || !eglMakeCurrent(g->dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, g->ctx)) {
        fprintf(stderr, "epd-ingest: no GLES 3 context\n");
        goto fail;
    }
    epd_gldebug_on("epd-ingest");
#define F(t, n) if (!(g->n = (t)eglGetProcAddress(#n))) { fprintf(stderr, "epd-ingest: missing %s\n", #n); goto fail; }
    GL_FNS
#undef F
    g->glEGLImageTargetTexture2DOES = (void *)eglGetProcAddress("glEGLImageTargetTexture2DOES");
    g->eglCreateImageKHR = (void *)eglGetProcAddress("eglCreateImageKHR");
    g->eglDestroyImageKHR = (void *)eglGetProcAddress("eglDestroyImageKHR");
    if (!g->glEGLImageTargetTexture2DOES || !g->eglCreateImageKHR || !g->eglDestroyImageKHR) {
        fprintf(stderr, "epd-ingest: no EGLImage functions\n");
        goto fail;
    }
    GLuint v = shader(g, GL_VERTEX_SHADER, vs), f = shader(g, GL_FRAGMENT_SHADER, fs);
    if (!v || !f) goto fail;
    g->prog = g->glCreateProgram();
    g->glAttachShader(g->prog, v);
    g->glAttachShader(g->prog, f);
    g->glLinkProgram(g->prog);
    GLint ok = 0;
    g->glGetProgramiv(g->prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024];
        g->glGetProgramInfoLog(g->prog, sizeof log, NULL, log);
        fprintf(stderr, "epd-ingest: link: %s\n", log);
        goto fail;
    }
    g->glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    g->glPixelStorei(GL_PACK_ALIGNMENT, 4);
    g->glActiveTexture(GL_TEXTURE1);
    g->lut = lut_tex(g, lut);
    g->glActiveTexture(GL_TEXTURE2);            /* the target: off units 0 (frames) and 1 */
    g->glGenFramebuffers(1, &g->fbo);
    if (!getenv("GLASSINE_INGEST_NO_HEAP")) heap_target(g);
    if (g->hfd < 0) {
        g->glGenTextures(1, &g->out);
        g->glBindTexture(GL_TEXTURE_2D, g->out);
        g->glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, g->tw, ph);
        g->glBindFramebuffer(GL_FRAMEBUFFER, g->fbo);
        g->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, g->out, 0);
        if (g->glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            fprintf(stderr, "epd-ingest: framebuffer incomplete\n");
            goto fail;
        }
    }
    g->glUseProgram(g->prog);
    g->glUniform1i(g->glGetUniformLocation(g->prog, "src"), 0);
    g->glUniform1i(g->glGetUniformLocation(g->prog, "lut"), 1);
    g->glUniform1i(g->glGetUniformLocation(g->prog, "pw"), pw);
    g->glActiveTexture(GL_TEXTURE6);            /* the S table, the R8 LUT: units 6, 7 (stay bound) */
    g->sqt = lut_tex(g, epd_grey_sq);
    g->glActiveTexture(GL_TEXTURE7);
    g->lut8 = lut_tex(g, lut);
    memcpy(g->clut, lut, 256);
    memcpy(g->clut8, lut, 256);
    if (!(g->progl = link_fs(g, fsl))) goto fail;
    g->glUseProgram(g->progl);
    g->glUniform1i(g->glGetUniformLocation(g->progl, "src"), 0);
    g->glUniform1i(g->glGetUniformLocation(g->progl, "lut"), 1);
    g->glUniform1i(g->glGetUniformLocation(g->progl, "sq"), 6);
    g->glUniform1i(g->glGetUniformLocation(g->progl, "pw"), pw);
    if ((g->prog8 = link_fs(g, fs8))) {     /* without: R8 frames go the CPU way */
        g->glUseProgram(g->prog8);
        g->glUniform1i(g->glGetUniformLocation(g->prog8, "src"), 0);
        g->glUniform1i(g->glGetUniformLocation(g->prog8, "lut"), 7);
    }
    g->glUseProgram(g->prog);
    g->glGenVertexArrays(1, &g->vao);
    g->glBindVertexArray(g->vao);
    g->glDisable(GL_DITHER);
    g->glEnable(GL_SCISSOR_TEST);
    g->glViewport(0, 0, g->tw, ph);
    g->glPixelStorei(GL_PACK_ROW_LENGTH, g->tw);
    if (g->glGetError() != GL_NO_ERROR) { fprintf(stderr, "epd-ingest: GL error in setup\n"); goto fail; }
    snprintf(g->name, sizeof g->name, "%s (%s)%s", (const char *)g->glGetString(GL_RENDERER), node,
             g->hfd >= 0 ? ", cached target" : "");
    return g;
fail:
    epd_ingest_close(g);
    return NULL;
}

void epd_ingest_close(epd_ingest *g)
{
    if (!g) return;
    if (g->dead) return;                /* abandoned: see epd-ingest.h (the struct leaks, once) */
    if (g->dpy != EGL_NO_DISPLAY) {
        if (g->ctx != EGL_NO_CONTEXT && g->glDeleteTextures) {
            eglMakeCurrent(g->dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, g->ctx);
            for (int i = 0; i < MAXIMG; i++) epd_ingest_drop(g, i);
        }
        for (int i = 0; i < g->nlay; i++) {      /* (the engine must be done with them: exit) */
            struct layer *l = &g->lay[i];
            if (l->img) g->eglDestroyImageKHR(g->dpy, l->img);
            for (int k = 0; k < l->b.planes; k++) close(l->b.fd[k]);
            if (l->fence >= 0) close(l->fence);
        }
        if (g->himg && g->eglDestroyImageKHR) g->eglDestroyImageKHR(g->dpy, g->himg);
        eglMakeCurrent(g->dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (g->ctx != EGL_NO_CONTEXT) eglDestroyContext(g->dpy, g->ctx);
        eglTerminate(g->dpy);           /* our own display (platform device) */
    }
    heap_free(g);
    icpu_sync_stop(&g->sy);             /* (after the drops: they flushed it) */
    icpu_pool_stop(&g->pool);
    for (int i = 0; i < ICPU_BANDS_MAX; i++) free(g->pbnc[i]);
    free(g->valid);
    free(g->drawn);
    free(g->bounce);
    free(g->dz.bt);
    free(g->dz.brow);
    free(g);
}

const char *epd_ingest_renderer(epd_ingest *g) { return g->name; }
int epd_ingest_dead(const epd_ingest *g) { return g->dead; }

/* a timeout: the context dead, its fence kept (deleting it may wait) */
static void ing_abandon(epd_ingest *g, GLsync f)
{
    g->dead = 1;
    if (f && !g->dead_f) g->dead_f = f;
}

int epd_ingest_revive(epd_ingest *g)
{
    if (!g || !g->dead || !g->dead_f) return 0;
    if (eglGetCurrentContext() != g->ctx && !eglMakeCurrent(g->dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, g->ctx)) return 0;
    GLenum w = g->glClientWaitSync(g->dead_f, 0, 0);     /* (timeout 0: never blocks) */
    if (w != GL_ALREADY_SIGNALED && w != GL_CONDITION_SATISFIED) return 0;
    g->glDeleteSync(g->dead_f);
    g->dead_f = NULL;
    g->dead = 0;
    memset(g->valid, 0, (size_t)g->ttw * g->tth);         /* (the mirror: nothing trusted) */
    memset(g->drawn, 0, (size_t)g->tw * g->ph);
    while (g->glGetError() != GL_NO_ERROR) {}
    return 1;
}

static int current(epd_ingest *g)
{
    return !g->dead && eglGetCurrentContext() == g->ctx ? 1 :
           !g->dead && eglMakeCurrent(g->dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, g->ctx);
}

static void lut_set(epd_ingest *g, GLenum unit, GLuint tex, const uint8_t lut[256])
{
    if (!current(g)) return;
    g->glActiveTexture(unit);
    g->glBindTexture(GL_TEXTURE_2D, tex);
    g->glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 256, 1, GL_RED_INTEGER, GL_UNSIGNED_BYTE, lut);
}

void epd_ingest_lut(epd_ingest *g, const uint8_t lut[256]) { memcpy(g->clut, lut, 256); lut_set(g, GL_TEXTURE1, g->lut, lut); }
void epd_ingest_lut8(epd_ingest *g, const uint8_t lut[256])
{
    memcpy(g->clut8, lut, 256);
    g->copy8 = 1;
    for (int i = 0; i < 256 && g->copy8; i++) g->copy8 = lut[i] == i;
    lut_set(g, GL_TEXTURE7, g->lut8, lut);
}
void epd_ingest_mode(epd_ingest *g, int lum) { g->lum = lum != 0; }

int epd_ingest_import(epd_ingest *g, int fd, int w, int h, int stride, int offset, uint32_t fourcc, uint64_t modifier)
{
    int r8 = fourcc == 0x20203852u;     /* DRM_FORMAT_R8: panel order (pw x ph) */
    if (!current(g) || (r8 ? w != g->pw || h != g->ph || !g->prog8 : w != g->ph || h != g->pw)) return -1;
    int i;
    for (i = 0; i < MAXIMG && g->im[i].img; i++) ;
    if (i == MAXIMG) { fprintf(stderr, "epd-ingest: more than %d buffers\n", MAXIMG); return -1; }
    EGLint a[32], k = 0;
    a[k++] = EGL_WIDTH; a[k++] = w;
    a[k++] = EGL_HEIGHT; a[k++] = h;
    a[k++] = EGL_LINUX_DRM_FOURCC_EXT; a[k++] = (EGLint)fourcc;
    a[k++] = EGL_DMA_BUF_PLANE0_FD_EXT; a[k++] = fd;
    a[k++] = EGL_DMA_BUF_PLANE0_OFFSET_EXT; a[k++] = offset;
    a[k++] = EGL_DMA_BUF_PLANE0_PITCH_EXT; a[k++] = stride;
    if (g->mods && modifier != 0x00ffffffffffffffull) {        /* DRM_FORMAT_MOD_INVALID: implicit */
        a[k++] = EGL_DMA_BUF_PLANE0_MODIFIER_LO_EXT; a[k++] = (EGLint)(modifier & 0xffffffffu);
        a[k++] = EGL_DMA_BUF_PLANE0_MODIFIER_HI_EXT; a[k++] = (EGLint)(modifier >> 32);
    }
    a[k++] = EGL_NONE;
    EGLImageKHR img = g->eglCreateImageKHR(g->dpy, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, NULL, a);
    if (img == EGL_NO_IMAGE_KHR) {
        fprintf(stderr, "epd-ingest: dma-buf import failed (0x%x; %dx%d stride %d format %.4s modifier 0x%llx)\n",
                eglGetError(), w, h, stride, (const char *)&fourcc, (unsigned long long)modifier);
        return -1;
    }
    GLuint t;
    g->glActiveTexture(GL_TEXTURE0);
    g->glGenTextures(1, &t);
    g->glBindTexture(GL_TEXTURE_2D, t);
    g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    g->glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, img);
    if (g->glGetError() != GL_NO_ERROR) {
        fprintf(stderr, "epd-ingest: dma-buf not usable as a 2D texture\n");
        g->glDeleteTextures(1, &t);
        g->eglDestroyImageKHR(g->dpy, img);
        return -1;
    }
    g->im[i].img = img; g->im[i].tex = t; g->im[i].w = w; g->im[i].h = h; g->im[i].r8 = r8;
    /* our own fd: the CPU ingest maps it, whoever closes theirs */
    g->im[i].fd = fcntl(fd, F_DUPFD_CLOEXEC, 0);
    g->im[i].stride = stride; g->im[i].off = offset;
    g->im[i].len = (size_t)offset + (size_t)stride * h;
    g->im[i].map = NULL;
    return i;
}

const uint8_t *epd_ingest_map(epd_ingest *g, int i)
{
    if (i < 0 || i >= MAXIMG || !g->im[i].img || g->im[i].fd < 0) return NULL;
    if (!g->im[i].map) {
        void *p = mmap(NULL, g->im[i].len, PROT_READ, MAP_SHARED, g->im[i].fd, 0);
        if (p == MAP_FAILED) { perror("epd-ingest: dma-buf mmap"); close(g->im[i].fd); g->im[i].fd = -1; return NULL; }
        g->im[i].map = p;
    }
    return g->im[i].map + g->im[i].off;
}

void epd_ingest_drop(epd_ingest *g, int i)
{
    if (i < 0 || i >= MAXIMG || !g->im[i].img) return;
    icpu_sync_flush(&g->sy);            /* (an END on this fd may still be queued) */
    if (g->im[i].map) munmap(g->im[i].map, g->im[i].len);      /* (the CPU side: also with a dead context) */
    if (g->im[i].fd >= 0) close(g->im[i].fd);
    g->im[i].map = NULL; g->im[i].fd = -1;
    if (!current(g)) return;
    g->glDeleteTextures(1, &g->im[i].tex);
    g->eglDestroyImageKHR(g->dpy, g->im[i].img);
    g->im[i].img = NULL;
}

/* the zero-copy view : an R8 buffer read in place for as long as
 * the caller holds it. START|READ over the whole buffer (the backend's target
 * also carries what its damage ring redrew since we last read it, so a range
 * over this frame's rects is not enough), END|READ when the caller lets go */
static double ing_now(void);

int epd_ingest_view(epd_ingest *g, int i, int on, const uint8_t **base, int *stride, epd_ingest_times *tm)
{
    if (tm) tm->gpu = tm->read = tm->sync = 0;
    if (i < 0 || i >= MAXIMG || !g->im[i].img || !g->im[i].r8) return -1;
    if (!on) {
        if (g->im[i].fd >= 0 && g->im[i].map) icpu_sync_ioctl(g->im[i].fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ, 0, 0);
        return 0;
    }
    const uint8_t *m = epd_ingest_map(g, i);
    if (!m) return -1;
    icpu_sync_flush(&g->sy);            /* (an END on this fd may still be queued) */
    double ta = ing_now();
    icpu_sync_ioctl(g->im[i].fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ, 0, 0);
    if (tm) tm->sync = ing_now() - ta;
    *base = m;
    *stride = g->im[i].stride;
    return 0;
}

int epd_ingest_owns(const epd_ingest *g, const void *p)
{
    const uint8_t *q = p;
    for (int i = 0; g && i < MAXIMG; i++)
        if (g->im[i].map && q >= g->im[i].map && q < g->im[i].map + g->im[i].len) return 1;
    return 0;
}

/* a buffer rect -> target texels [t0, t1) x panel rows [r0, r1); 0: empty */
static int texels(const epd_ingest *g, epd_ingest_rect r, int *t0, int *t1, int *r0, int *r1)
{
    int x0 = r.x < 0 ? 0 : r.x, y0 = r.y < 0 ? 0 : r.y;
    int x1 = r.x + r.w > g->ph ? g->ph : r.x + r.w, y1 = r.y + r.h > g->pw ? g->pw : r.y + r.h;
    if (x1 <= x0 || y1 <= y0) return 0;
    *t0 = (g->pw - y1) / 4; *t1 = (g->pw - y0 + 3) / 4;
    *r0 = x0; *r1 = x1;
    return 1;
}

void epd_ingest_cols(const epd_ingest *g, epd_ingest_rect r, int *c0, int *c1)
{
    int t0, t1, r0, r1;
    if (!texels(g, r, &t0, &t1, &r0, &r1)) { *c0 = *c1 = 0; return; }
    *c0 = t0 * 4; *c1 = t1 * 4;
}

/* tiles wholly inside panel cols [c0, c1) x rows [r0, r1): out = the caller's grey */
static void tiles_valid(epd_ingest *g, int c0, int c1, int r0, int r1)
{
    for (int ty = (r0 + TILE - 1) / TILE; ty < g->tth; ty++) {
        int y0 = ty * TILE, y1 = y0 + TILE < g->ph ? y0 + TILE : g->ph;
        if (y1 > r1 || y0 < r0) break;
        for (int tx = (c0 + TILE - 1) / TILE; tx < g->ttw; tx++) {
            int x0 = tx * TILE, x1 = x0 + TILE < g->pw ? x0 + TILE : g->pw;
            if (x1 > c1) break;
            g->valid[ty * g->ttw + tx] = 1;
        }
    }
}

static double ing_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

int epd_ingest_frame(epd_ingest *g, int buf, const epd_ingest_rect *r, int n, uint8_t *grey, int timeout_ms)
{
    return epd_ingest_frame_t(g, buf, r, n, grey, timeout_ms, NULL);
}

int epd_ingest_frame_t(epd_ingest *g, int buf, const epd_ingest_rect *r, int n, uint8_t *grey, int timeout_ms,
                       epd_ingest_times *tm)
{
    if (tm) tm->gpu = tm->read = tm->sync = 0;
    if (buf < 0 || buf >= MAXIMG || !g->im[buf].img || !current(g)) return -1;
    double ta = ing_now();
    g->glActiveTexture(GL_TEXTURE0);
    g->glBindTexture(GL_TEXTURE_2D, g->im[buf].tex);
    g->glBindFramebuffer(GL_FRAMEBUFFER, g->fbo);
    g->glUseProgram(g->im[buf].r8 ? g->prog8 : g->lum ? g->progl : g->prog);
    int t0, t1, r0, r1, any = 0;
    for (int i = 0; i < n; i++) {
        if (!texels(g, r[i], &t0, &t1, &r0, &r1)) continue;
        g->glScissor(t0, r0, t1 - t0, r1 - r0);
        g->glDrawArrays(GL_TRIANGLES, 0, 3);
        any = 1;
    }
    if (!any) return 0;
    /* a timed wait first: a hung GPU must not block glReadPixels forever */
    GLsync f = g->glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    if (f) {
        GLenum w = g->glClientWaitSync(f, GL_SYNC_FLUSH_COMMANDS_BIT, (GLuint64)timeout_ms * 1000000);
        if (w == GL_TIMEOUT_EXPIRED || w == GL_WAIT_FAILED) {
            fprintf(stderr, "epd-ingest: the GPU did not finish a frame in %d ms: ingest abandoned\n", timeout_ms);
            ing_abandon(g, f);          /* f stays: deleting it may wait too (epd_ingest_revive) */
            return -1;
        }
        g->glDeleteSync(f);
    }
    double tb = ing_now();
    if (g->hmap) {                      /* the cached target: invalidate, then plain copies */
        struct dma_buf_sync sy = { DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ };
        ioctl(g->hfd, DMA_BUF_IOCTL_SYNC, &sy);
        for (int i = 0; i < n; i++) {
            if (!texels(g, r[i], &t0, &t1, &r0, &r1)) continue;
            for (int row = r0; row < r1; row++)
                memcpy(grey + (size_t)row * g->pw + (size_t)t0 * 4, g->hmap + (size_t)row * g->hstride + (size_t)t0 * 4,
                       (size_t)(t1 - t0) * 4);
        }
        sy.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
        ioctl(g->hfd, DMA_BUF_IOCTL_SYNC, &sy);
    } else for (int i = 0; i < n; i++) {
        if (!texels(g, r[i], &t0, &t1, &r0, &r1)) continue;
        g->glReadPixels(t0, r0, t1 - t0, r1 - r0, GL_RGBA, GL_UNSIGNED_BYTE, grey + (size_t)r0 * g->pw + (size_t)t0 * 4);
    }
    if (tm) { tm->gpu = tb - ta; tm->read = ing_now() - tb; }
    if (g->glGetError() != GL_NO_ERROR) { fprintf(stderr, "epd-ingest: GL error in a frame\n"); return -1; }
    for (int i = 0; i < n; i++) {               /* the caller's image = out where a rect covered a tile whole */
        if (!texels(g, r[i], &t0, &t1, &r0, &r1)) continue;
        tiles_valid(g, t0 * 4, t1 * 4, r0, r1);
        for (int row = r0; row < r1; row++) memset(g->drawn + (size_t)row * g->tw + t0, 1, (size_t)(t1 - t0));
    }
    return 0;
}

/* the same rects by the CPU, through the buffer's
 * mapping: no draw, no fence, nothing on the GPU's one ring. The columns are
 * widened as the GPU widens them, so the bytes written are the GPU path's
 * (R8: epd_grey_lut of the LUT8; XRGB: epd_grey_rect_mode, the shaders'
 * integer q; both NEON on arm64, test/epd-ingest-test). START|READ before
 * the reads (it also waits for the producer's write fences: implicit sync;
 * and invalidates where the exporter's pages are cached), END|READ after;
 * the grey is the caller's now, not out's (epd_ingest_stale). No GL: works
 * with a dead context too */
static int frame_cpu_s3c(epd_ingest *g, int buf, const uint8_t *m, const epd_ingest_rect *r, int n, uint8_t *grey,
                         epd_ingest_times *tm, int kern, int bands, int sync);

int epd_ingest_frame_cpu(epd_ingest *g, int buf, const epd_ingest_rect *r, int n, uint8_t *grey, epd_ingest_times *tm)
{
    if (tm) tm->gpu = tm->read = tm->sync = 0;
    const uint8_t *m = epd_ingest_map(g, buf);
    if (!m) return -1;
    {                                   /* any switch on -> its way; all off: today's below */
        int kern, bands, sync;
        epd_ingest_cpu_switches(&kern, &bands, &sync);
        if (kern || bands > 1 || sync >= 0) return frame_cpu_s3c(g, buf, m, r, n, grey, tm, kern, bands, sync);
        icpu_sync_flush(&g->sy);        /* (switched off: the helper's END done first) */
    }
    int fd = g->im[buf].fd, st = g->im[buf].stride, r8 = g->im[buf].r8;
    double ta = ing_now();
    struct dma_buf_sync sy = { DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ };
    ioctl(fd, DMA_BUF_IOCTL_SYNC, &sy);
    double tb = ing_now();
    /* rows go through a cached bounce (8 source rows, <= 45 KB) by memcpy:
     * wide loads from the mapping, the convert's narrow ones from L1. A
     * write-combined / uncached exporter (msm GBM; a VRAM BO on the host:
     * byte loads ~1 MB/s) reads at its streaming rate instead of a load per
     * byte; a cached one pays an L1 copy */
    if (!g->bounce && !(g->bounce = malloc((size_t)8 * 4 * (g->ph > g->pw ? g->ph : g->pw)))) return -1;
    int t0, t1, r0, r1;
    for (int i = 0; i < n; i++) {
        if (!texels(g, r[i], &t0, &t1, &r0, &r1)) continue;
        int c0 = t0 * 4, cw = (t1 - t0) * 4;
        if (r8) for (int row = r0; row < r1; row++) {
            if (g->copy8) { memcpy(grey + (size_t)row * g->pw + c0, m + (size_t)row * st + c0, (size_t)cw); continue; }
            memcpy(g->bounce, m + (size_t)row * st + c0, (size_t)cw);
            epd_grey_lut(g->clut8, grey + (size_t)row * g->pw + c0, g->bounce, cw);
        } else {                        /* the portrait rect of the widened columns: y = pw - c1 .. pw - c0 */
            int y = g->pw - c0 - cw, w = r1 - r0;
            for (int yb = y; yb < y + cw; yb += 8) {
                int bh = y + cw - yb < 8 ? y + cw - yb : 8;
                for (int k = 0; k < bh; k++)
                    memcpy(g->bounce + (size_t)k * w * 4, m + (size_t)(yb + k) * st + (size_t)r0 * 4, (size_t)w * 4);
                epd_grey_rect_mode(g->clut, g->lum, grey, g->pw, (const uint32_t *)g->bounce, w, r0, yb, w, bh);
            }
        }
        epd_ingest_stale(g, (epd_ingest_rect){ r0, g->pw - c0 - cw, r1 - r0, cw });
    }
    sy.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
    ioctl(fd, DMA_BUF_IOCTL_SYNC, &sy);
    if (tm) { tm->sync = tb - ta; tm->read = ing_now() - tb; }
    return 0;
}

/* the switches (epd-ingest.h), unset (< -1 /
 * -1) until the first CPU frame reads GLASSINE_INGEST_KERNEL / _BANDS / _SYNC */
int epd_ingest_kernel = -1, epd_ingest_bands = -1, epd_ingest_sync = -2, epd_ingest_partial = -1;

int epd_ingest_kernel_of(const char *s)
{
    return !strncmp(s, "tile", 4) ? 1 : !strncmp(s, "scalar", 6) ? 2 : 0;
}

int epd_ingest_sync_of(const char *s)
{
    return !strncmp(s, "off", 3) ? -1 : !strncmp(s, "on", 2) ? 0 : (*s >= '0' && *s <= '9') ? atoi(s) : -1;
}

void epd_ingest_cpu_switches(int *kern, int *bands, int *sync)
{
    const char *e;
    if (__atomic_load_n(&epd_ingest_kernel, __ATOMIC_RELAXED) < 0)
        __atomic_store_n(&epd_ingest_kernel, (e = getenv("GLASSINE_INGEST_KERNEL")) ? epd_ingest_kernel_of(e) : 0, __ATOMIC_RELAXED);
    if (__atomic_load_n(&epd_ingest_bands, __ATOMIC_RELAXED) < 0)
        __atomic_store_n(&epd_ingest_bands, (e = getenv("GLASSINE_INGEST_BANDS")) ? atoi(e) : 1, __ATOMIC_RELAXED);
    if (__atomic_load_n(&epd_ingest_sync, __ATOMIC_RELAXED) < -1)
        __atomic_store_n(&epd_ingest_sync, (e = getenv("GLASSINE_INGEST_SYNC")) ? epd_ingest_sync_of(e) : -1, __ATOMIC_RELAXED);
    int b = __atomic_load_n(&epd_ingest_bands, __ATOMIC_RELAXED);
    *kern = __atomic_load_n(&epd_ingest_kernel, __ATOMIC_RELAXED);
    *bands = b < 1 ? 1 : b > ICPU_BANDS_MAX ? ICPU_BANDS_MAX : b;
    *sync = __atomic_load_n(&epd_ingest_sync, __ATOMIC_RELAXED);
    if (__atomic_load_n(&epd_ingest_partial, __ATOMIC_RELAXED) < 0)
        __atomic_store_n(&epd_ingest_partial, (e = getenv("GLASSINE_INGEST_PARTIAL")) ? strncmp(e, "off", 3) != 0 : 1, __ATOMIC_RELAXED);
}

/* the CPU frame with 's switches: the kernel (row = today's convert, tile,
 * scalar = tile in generic C), the rows in parts on the pool (>= 0.3 Mpx;
 * primed before START, so their wake-up runs under it), the sync ioctls on
 * the helper (START waited for, END posted). The same bytes as today's way
 * (test/epd-ingest-bench, epd-ingest-test, heap-ingest-probe) */
static int frame_cpu_s3c(epd_ingest *g, int buf, const uint8_t *m, const epd_ingest_rect *r, int n, uint8_t *grey,
                         epd_ingest_times *tm, int kern, int bands, int sync)
{
    int fd = g->im[buf].fd, st = g->im[buf].stride, r8 = g->im[buf].r8;
    icpu_box bx[16], *b = n <= 16 ? bx : malloc(sizeof *b * (size_t)n);
    if (!b) return -1;
    int nb = 0, t0, t1, r0, r1;
    long px = 0;
    for (int i = 0; i < n; i++) {
        if (!texels(g, r[i], &t0, &t1, &r0, &r1)) continue;
        b[nb++] = (icpu_box){ t0 * 4, (t1 - t0) * 4, r0, r1 };
        px += (long)(t1 - t0) * 4 * (r1 - r0);
    }
    /* ingest_partial (default on): the syncs cover only the bytes the boxes
     * read, one span from the first to the last (the system heap's range
     * sync: ~0.06 ms for 64 rows vs ~1.4 ms for the whole 5 MB on a Gold
     * core); off, or a kernel without the ioctl: the whole buffer */
    unsigned soff = 0, slen = 0;
    if (nb && __atomic_load_n(&epd_ingest_partial, __ATOMIC_RELAXED) > 0) {
        size_t lo = SIZE_MAX, hi = 0;
        for (int k = 0; k < nb; k++) {
            size_t a, z;
            if (r8) { a = (size_t)b[k].r0 * st + b[k].c0; z = (size_t)(b[k].r1 - 1) * st + b[k].c0 + b[k].cw; }
            else {                      /* the portrait rect: buffer rows pw - c0 - cw .. pw - c0, columns r0 .. r1 */
                size_t y = (size_t)(g->pw - b[k].c0 - b[k].cw);
                a = y * st + (size_t)b[k].r0 * 4; z = (y + b[k].cw - 1) * st + (size_t)b[k].r1 * 4;
            }
            if (a < lo) lo = a;
            if (z > hi) hi = z;
        }
        if (hi > lo && hi - lo <= UINT32_MAX) { soff = (unsigned)lo; slen = (unsigned)(hi - lo); }
    }
    int parts = bands > 1 && px >= ICPU_BAND_MIN_PX && !(r8 && g->copy8) ? bands : 1;   /* (a copy: one core is enough) */
    if (parts > 1) {
        const char *c = getenv("GLASSINE_INGEST_CPUS");
        int w = icpu_pool_start(&g->pool, parts - 1, c ? c : "1-4");
        if (parts > w + 1) parts = w + 1;
        icpu_pool_prime(&g->pool);
    }
    for (int k = 0; !kern && k < parts; k++)
        if (!g->pbnc[k] && !(g->pbnc[k] = malloc(ICPU_BOUNCE(g->pw, g->ph)))) { if (b != bx) free(b); return -1; }
    int hs = sync >= 0 && icpu_sync_start(&g->sy, sync) == 0;
    double ta = ing_now();
    struct dma_buf_sync sy = { DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ };
    if (hs) icpu_sync_op(&g->sy, fd, sy.flags, soff, slen, 1);
    else { icpu_sync_flush(&g->sy); icpu_sync_ioctl(fd, sy.flags, soff, slen); }
    double tb = ing_now();
    icpu_job J = { .kernel = kern, .lum = g->lum, .r8 = r8, .st = st, .pw = g->pw, .n = nb, .parts = parts,
                   .lut = g->clut, .lut8 = r8 && g->copy8 ? NULL : g->clut8, .m = m, .grey = grey, .b = b };
    for (int k = 0; k < ICPU_BANDS_MAX; k++) J.bounce[k] = g->pbnc[k];
    icpu_pool_run(&g->pool, parts, icpu_job_part, &J);
    for (int k = 0; k < nb; k++)
        epd_ingest_stale(g, (epd_ingest_rect){ b[k].r0, g->pw - b[k].c0 - b[k].cw, b[k].r1 - b[k].r0, b[k].cw });
    sy.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
    if (hs) icpu_sync_op(&g->sy, fd, sy.flags, soff, slen, 0);   /* (not waited for: no dirty line of ours; a later START queues behind) */
    else icpu_sync_ioctl(fd, sy.flags, soff, slen);
    double tc = ing_now();
    if (tm) { tm->sync = tb - ta; tm->read = tc - tb; }
    g->s3c.frames++;
    g->s3c.banded += parts > 1;
    if (slen) { g->s3c.ranged++; g->s3c.rbytes += slen; }
    g->s3c.wait += tb - ta;
    g->s3c.read += tc - tb;
    if (tc - tb > g->s3c.rmax) g->s3c.rmax = tc - tb;
    if (b != bx) free(b);
    return 0;
}

void epd_ingest_cpu_stats(epd_ingest *g, char *s, int n)
{
    int kern, bands, sync;
    epd_ingest_cpu_switches(&kern, &bands, &sync);
    double st = 0, sx = 0, en = 0, ex = 0;
    unsigned long sn = 0, nn = 0;
    if (g->sy.up) {
        pthread_mutex_lock(&g->sy.mu);
        st = g->sy.st_s; sx = g->sy.st_max; sn = g->sy.st_n; en = g->sy.en_s; ex = g->sy.en_max; nn = g->sy.en_n;
        pthread_mutex_unlock(&g->sy.mu);
    }
    unsigned long f = g->s3c.frames;
    snprintf(s, (size_t)n, "ingest_kernel=%s ingest_bands=%d ingest_sync=%d: %lu frames (%lu banded, %lu parts found "
             "their job spinning), START wait %.2f ms avg, convert %.2f avg (max %.2f); helper START %.2f ms avg "
             "(max %.2f, %lu), END %.2f avg (max %.2f, %lu); ingest_partial=%s: %lu frames ranged, %.0f KiB avg",
             kern == 2 ? "scalar" : kern ? "tile" : "row", bands, sync, f, g->s3c.banded,
             __atomic_load_n(&g->pool.spun, __ATOMIC_RELAXED), f ? g->s3c.wait * 1e3 / f : 0, f ? g->s3c.read * 1e3 / f : 0,
             g->s3c.rmax * 1e3, sn ? st * 1e3 / sn : 0, sx * 1e3, sn, nn ? en * 1e3 / nn : 0, ex * 1e3, nn,
             __atomic_load_n(&epd_ingest_partial, __ATOMIC_RELAXED) <= 0 ? "off"
             : __atomic_load_n(&icpu_partial, __ATOMIC_RELAXED) < 0 ? "on (no kernel ioctl: whole buffer)" : "on",
             g->s3c.ranged, g->s3c.ranged ? g->s3c.rbytes / 1024 / g->s3c.ranged : 0);
}

/* ---- dizzy on the GPU (epd-ingest.h epd_ingest_dizzy) ----
 * epd-dizzy.h's layered form as a compute shader: a workgroup per 64 x 64
 * panel tile (tiles on the panel's 64 grid, like the CPU's), the tile plus 8
 * pixels around it in shared memory (2 pixels a word, 80 x 80 = 12.8 KB): a
 * pixel's dot depends on shares from at most 7 pixels away and their later
 * neighbours' blocked state (8), so the tile's own 64 x 64 come out exactly as
 * epd_dizzy over the whole rect does (the CPU's 128-row bands cut the error,
 * the GPU doesn't: not byte for byte against glassine-epdd-desktop, but against epd_dizzy
 * on the rect, test/epd-ingest-test). Gather, not scatter: a pixel of layer k
 * sums the shares its earlier-layer neighbours left (a blocked one -- fixed,
 * outside the rect -- left 0), snaps, and leaves its own share for the later
 * ones. The plan: per layer the words with a pixel of it (an invocation writes
 * a whole word: no atomics; a word's two pixels in the same layer don't
 * trade), and each pixel's later / earlier neighbour masks -- the same for
 * every tile, the window being 64-periodic in the layer table. Output: the
 * dots as bits (an SSBO: msm buffers are write-combined, so 1 bit a pixel). */
#define GDZ_W 40                        /* words per window row (80 pixels) */
static const char *cs_dizzy =
    "#version 310 es\n"
    "precision highp int; precision highp usampler2D;\n"
    "layout(local_size_x = LSZ) in;\n"
    "layout(std430, binding = 0) readonly buffer Plan { uint off[9]; uint ent[]; };\n"
    "layout(std430, binding = 1) writeonly buffer Bits { uint bits[]; };\n"
    "uniform usampler2D grey;  /* R8UI, panel pixels */\n"
    "uniform usampler2D lvt;   /* R8UI 256 x 1: grey -> level | 0x80 fixed (glassine-epdd-desktop dz_lv) */\n"
    "uniform ivec4 rect;       /* x0 (column), y0 (row), w, h */\n"
    "uniform ivec2 t0;         /* the first tile */\n"
    "uniform int ows;          /* bits words a row */\n"
    "shared uint S[SSZ];      /* a pixel's 16 bits: before its layer level (0..30) | 0x20 blocked; after it\n"
    "                           * its share (14 bits, signed) | 0x8000 the dot */\n"
    "int share_of(uint h) { return int((h & 0x3fffu) << 18u) >> 18; }\n"
    "uint hw(uint i, int x) { return (S[i % uint(SSZ)] >> (16u * uint(x & 1))) & 0xffffu; }\n"
    "void layer(int k, uint lid) {\n"
    "    for (uint j = off[k] + lid; j < off[k + 1]; j += uint(LSZ)) {\n"
    "      uint en = ent[j], wi = en & 0xfffu, old = S[wi], nw = old;\n"
    "      int wy = int(wi / 40u), wx0 = int(wi % 40u) * 2;\n"
    "      for (int h = 0; h < 2; h++) {\n"
    "        if (((en >> uint(12 + h)) & 1u) == 0u) continue;\n"
    "        uint me = (old >> uint(16 * h)) & 0xffffu, res = 0u;\n"
    "        if ((me & 0x20u) == 0u) {\n"
    "          int wx = wx0 + h, acc = int(me & 31u) * 256, n = 0;\n"
    "          uint lm = (en >> uint(14 + 8 * h)) & 15u, em = (en >> uint(18 + 8 * h)) & 15u;\n"
    "          for (int q = 0; q < 4; q++) {   /* R L D U (no local array: a5xx scratch) */\n"
    "            uint b = 1u << uint(q);\n"
    "            if (((lm | em) & b) == 0u) continue;\n"
    "            int x = wx + (q == 0 ? 1 : q == 1 ? -1 : 0), y = wy + (q == 2 ? 1 : q == 3 ? -1 : 0);\n"
    "            if (x < 0 || x >= 80 || y < 0 || y >= 80) continue;\n"
    "            uint nh = hw(uint(y * 40 + (x >> 1)), x);\n"
    "            if ((lm & b) != 0u) n += (nh & 0x20u) == 0u ? 1 : 0;\n"
    "            else acc += share_of(nh);\n"
    "          }\n"
    "          acc = clamp(acc, 0, 7680);\n"
    "          bool hi = acc >= 3840;\n"
    "          int err = acc - (hi ? 7680 : 0);\n"
    "          int dl = n == 1 ? 32768 : n == 2 ? 16384 : n == 3 ? 10923 : n == 4 ? 8192 : 0;\n"
    "          res = (uint((err * dl) >> 15) & 0x3fffu) | (hi ? 0x8000u : 0u);\n"
    "        }\n"
    "        nw = (nw & ~(0xffffu << uint(16 * h))) | (res << uint(16 * h));\n"
    "      }\n"
    "      S[wi] = nw;\n"
    "    }\n"
    "}\n"
    "void main() {\n"
    "  ivec2 tile = ivec2(gl_WorkGroupID.xy) + t0, org = tile * 64 - 8;\n"
    "  uint lid = gl_LocalInvocationIndex;\n"
    "#if STAGE == 4\n"
    "  for (uint t = lid; t < 128u; t += uint(LSZ))\n"
    "    bits[((tile.y - t0.y) * 64 + int(t >> 1)) * ows + (tile.x - t0.x) * 2 + int(t & 1u)] = 0x55555555u;\n"
    "  return;\n"
    "#endif\n"
    "  for (uint i = lid; i < uint(SSZ) + NU; i += uint(LSZ)) {\n"
    "    int wy = int(i / 40u), wx = int(i % 40u) * 2;\n"
    "    uint v = 0u;\n"
    "    for (int h = 0; h < 2; h++) {\n"
    "      ivec2 p = org + ivec2(wx + h, wy);\n"
    "      uint e = 0x20u;\n"
    "      if (all(greaterThanEqual(p, rect.xy)) && all(lessThan(p, rect.xy + rect.zw))) {\n"
    "#if STAGE == 2\n"
    "        uint l = uint(p.x + p.y) & 31u;\n"
    "#else\n"
    "        uint l = texelFetch(lvt, ivec2(int(texelFetch(grey, p, 0).r), 0), 0).r;\n"
    "#endif\n"
    "        e = (l & 0x80u) != 0u ? 0x20u : (l & 31u);\n"
    "      }\n"
    "      v |= e << uint(16 * h);\n"
    "    }\n"
    "    S[i] = v;\n"
    "  }\n"
    "  BAR();\n"
    "#if STAGE == 0\n"
    "  layer(0, lid); BAR();\n"
    "  layer(1, lid); BAR();\n"
    "  layer(2, lid); BAR();\n"
    "  layer(3, lid); BAR();\n"
    "  layer(4, lid); BAR();\n"
    "  layer(5, lid); BAR();\n"
    "  layer(6, lid); BAR();\n"
    "  layer(7, lid); BAR();\n"
    "#endif\n"
    "  for (uint t = lid; t < 128u; t += uint(LSZ)) {   /* 64 rows x 2 words of the tile */\n"
    "    int r = int(t >> 1), c = int(t & 1u), wy = 8 + r;\n"
    "    uint o = 0u;\n"
    "    for (int b = 0; b < 32 + int(NU); b++) {\n"
    "      int wx = 8 + 32 * c + b;\n"
    "      uint hv = hw(uint(wy * 40 + (wx >> 1)), wx);\n"
    "#if STAGE >= 1\n"
    "      hv = (hv & 0x20u) == 0u && (hv & 31u) >= 15u ? 0x8000u : 0u;\n"
    "#endif\n"
    "      o |= ((hv >> 15) & 1u) << uint(b);\n"
    "    }\n"
    "#if STAGE != 3\n"
    "    bits[((tile.y - t0.y) * 64 + r) * ows + (tile.x - t0.x) * 2 + c] = o;\n"
    "#endif\n"
    "  }\n"
    "}\n";

/* the plan: off[9], then per layer its words (word | which halves << 12 |
 * low half's later / earlier masks << 14 / << 18 | high half's << 22 / << 26) */
static int gdz_plan(uint32_t *pl, int cap)
{
    const uint8_t (*L)[64] = epd_dizzy_layers();
    int n = 9;
    for (int k = 0; k < 8; k++) {
        pl[k] = (uint32_t)(n - 9);
        for (int wi = 0; wi < 80 * GDZ_W; wi++) {
            int wy = wi / GDZ_W, wx0 = wi % GDZ_W * 2;
            uint32_t e = (uint32_t)wi;
            for (int h = 0; h < 2; h++) {
                int x = (wx0 + h - 8) & 63, y = (wy - 8) & 63, l = L[y][x];
                if (l != k) continue;
                int nb[4] = { L[y][(x + 1) & 63], L[y][(x - 1) & 63], L[(y + 1) & 63][x], L[(y - 1) & 63][x] };
                uint32_t lm = 0, em = 0;
                for (int q = 0; q < 4; q++) { lm |= (uint32_t)(nb[q] > l) << q; em |= (uint32_t)(nb[q] < l) << q; }
                e |= 1u << (12 + h) | lm << (14 + 8 * h) | em << (18 + 8 * h);
            }
            if (!(e >> 12 & 3)) continue;
            if (n >= cap) return -1;
            pl[n++] = e;
        }
    }
    pl[8] = (uint32_t)(n - 9);
    return n;
}

static int gdz_setup(epd_ingest *g)
{
    struct gdz *z = &g->dz;
#define G(n) if (!z->n && !(z->n = (void *)eglGetProcAddress(#n))) return -1;
    G(glDispatchCompute) G(glMemoryBarrier) G(glGenBuffers) G(glBindBuffer) G(glBindBufferBase) G(glBufferData)
    G(glMapBufferRange) G(glUnmapBuffer)
#undef G
    const char *ver = (const char *)g->glGetString(GL_VERSION);
    if (!ver || strstr(ver, "OpenGL ES 3.0")) return -1;           /* (compute: ES 3.1) */
    /* GLASSINE_GDZ_LSZ: the workgroup size (default 32: one wave; with more the A509 faults at the barrier) */
    const char *ls = getenv("GLASSINE_GDZ_LSZ");
    int lsz = ls && atoi(ls) >= 32 && atoi(ls) <= 1024 ? atoi(ls) : 32;
    char *full = malloc(strlen(cs_dizzy) + 224);
    if (!full) return -1;
    const char *nl = strchr(cs_dizzy, '\n') + 1;   /* (after "#version 310 es") */
    const char *st = getenv("GLASSINE_GDZ_STAGE");      /* (1: no layers, the plain threshold: a test) */
    const char *sz = getenv("GLASSINE_GDZ_SSZ");        /* (the shared array, < 3200: a test, wrong dots) */
    const char *bar = getenv("GLASSINE_GDZ_BAR");        /* (0: no barriers, 1: barrier only: a test) */
    int bm = bar ? atoi(bar) : 2;
    /* GLASSINE_GDZ_NOUNROLL: loop bounds the compiler can't fold (a test: fewer live booleans, half regs) */
    const char *nu = getenv("GLASSINE_GDZ_NOUNROLL") ? "uint(min(rect.z, 0))" : "0u";
    sprintf(full, "%.*s#define LSZ %d\n#define STAGE %d\n#define SSZ %d\n#define BAR() %s\n#define NU %s\n%s", (int)(nl - cs_dizzy),
            cs_dizzy, lsz, st ? atoi(st) : 0, sz && atoi(sz) > 0 && atoi(sz) < 3200 ? atoi(sz) : 3200,
            bm == 0 ? "" : bm == 1 ? "barrier()" : "memoryBarrierShared(); barrier()", nu, nl);
    if (getenv("GLASSINE_GDZ_DEBUG")) {
        PFNGLGETINTEGERVPROC gi = (void *)eglGetProcAddress("glGetIntegerv");
        GLint v[3] = { 0 };
        if (gi) {
            gi(GL_MAX_COMPUTE_SHARED_MEMORY_SIZE, v);
            fprintf(stderr, "epd-ingest: dizzy: %s, shared %d", ver, v[0]);
            gi(GL_MAX_COMPUTE_WORK_GROUP_INVOCATIONS, v);
            fprintf(stderr, ", invocations %d, local size %d\n", v[0], lsz);
        }
    }
    GLuint c = shader(g, GL_COMPUTE_SHADER, full);
    free(full);
    if (!c) return -1;
    z->prog = g->glCreateProgram();
    g->glAttachShader(z->prog, c);
    g->glLinkProgram(z->prog);
    GLint ok = 0;
    g->glGetProgramiv(z->prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024];
        g->glGetProgramInfoLog(z->prog, sizeof log, NULL, log);
        fprintf(stderr, "epd-ingest: dizzy: link: %s\n", log);
        return -1;
    }
    static uint32_t pl[9 + 8 * 80 * GDZ_W];
    int n = gdz_plan(pl, (int)(sizeof pl / sizeof *pl));
    if (n < 0) return -1;
    z->glGenBuffers(1, &z->plan);
    z->glBindBuffer(GL_SHADER_STORAGE_BUFFER, z->plan);
    z->glBufferData(GL_SHADER_STORAGE_BUFFER, (GLsizeiptr)n * 4, pl, GL_STATIC_DRAW);
    if (!z->bits) z->glGenBuffers(1, &z->bits);
    g->glActiveTexture(GL_TEXTURE8);                /* the greys, the level table: units 8, 9 */
    g->glGenTextures(1, &z->gtex);
    g->glBindTexture(GL_TEXTURE_2D, z->gtex);
    g->glTexStorage2D(GL_TEXTURE_2D, 1, GL_R8UI, g->pw, g->ph);
    g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    g->glActiveTexture(GL_TEXTURE9);
    if (!z->ltex) z->ltex = lut_tex(g, z->lv);
    g->glUseProgram(z->prog);
    g->glUniform1i(g->glGetUniformLocation(z->prog, "grey"), 8);
    g->glUniform1i(g->glGetUniformLocation(z->prog, "lvt"), 9);
    z->u_rect = g->glGetUniformLocation(z->prog, "rect");
    z->u_t0 = g->glGetUniformLocation(z->prog, "t0");
    z->u_ows = g->glGetUniformLocation(z->prog, "ows");
    g->glUseProgram(g->prog);
    if (g->glGetError() != GL_NO_ERROR) { fprintf(stderr, "epd-ingest: dizzy: GL error in setup\n"); return -1; }
    return 0;
}

/* the layered form's job: the greys uploaded into gtex (R8UI), one dispatch */
static int gdz_submit_layered(epd_ingest *g, const uint8_t *grey, int gs, int x0, int y0, int w, int h)
{
    struct gdz *z = &g->dz;
    if (!z->st) {
        if (gdz_setup(g) < 0) { z->st = -1; fprintf(stderr, "epd-ingest: no dizzy on the GPU (compute)\n"); return -1; }
        z->st = 1;
        if (getenv("GLASSINE_GDZ_COMPILE_ONLY")) return -1;   /* (a test: FD_MESA_DEBUG=shaderdb prints the stats) */
    }
    int tx0 = x0 >> 6, tx1 = (x0 + w + 63) >> 6, ty0 = y0 >> 6, ty1 = (y0 + h + 63) >> 6, ows = (tx1 - tx0) * 2;
    size_t need = (size_t)(ty1 - ty0) * 64 * ows * 4;
    z->glBindBuffer(GL_SHADER_STORAGE_BUFFER, z->bits);
    if (need > z->bcap) { z->glBufferData(GL_SHADER_STORAGE_BUFFER, (GLsizeiptr)need, NULL, GL_STREAM_READ); z->bcap = need; }
    g->glActiveTexture(GL_TEXTURE8);
    g->glBindTexture(GL_TEXTURE_2D, z->gtex);
    g->glPixelStorei(GL_UNPACK_ROW_LENGTH, gs);
    g->glTexSubImage2D(GL_TEXTURE_2D, 0, x0, y0, w, h, GL_RED_INTEGER, GL_UNSIGNED_BYTE, grey);
    g->glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    g->glUseProgram(z->prog);
    g->glUniform4i(z->u_rect, x0, y0, w, h);
    g->glUniform2i(z->u_t0, tx0, ty0);
    g->glUniform1i(z->u_ows, ows);
    z->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, z->plan);
    z->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, z->bits);
    z->glDispatchCompute((GLuint)(tx1 - tx0), (GLuint)(ty1 - ty0), 1);
    z->tx0 = tx0; z->ty0 = ty0; z->ows = ows;
    return 0;
}

/* the layered job's bits (words of 32 pixels along a row) into out */
static void gdz_collect_layered(epd_ingest *g, const uint32_t *b, uint8_t *out, int os)
{
    struct gdz *z = &g->dz;
    int bx = z->x0 - z->tx0 * 64, by = z->y0 - z->ty0 * 64;
    static uint32_t *row;                   /* (the bits a row: one read of the write-combined map) */
    static int rcap;
    if (z->ows > rcap) { free(row); row = malloc((size_t)z->ows * 4); rcap = row ? z->ows : 0; }
    for (int y = 0; y < z->h; y++) {
        uint8_t *o = out + (size_t)y * os;
        const uint32_t *br = b + (size_t)(by + y) * z->ows;
        if (row) { memcpy(row, br, (size_t)z->ows * 4); br = row; }
        for (int x = 0; x < z->w; x++) {
            int c = bx + x;
            o[x] = (uint8_t)(-(int)(br[c >> 5] >> (c & 31) & 1) & 30);
        }
    }
}

/* ---- the block form ----
 * epd-dizzy.h's block dizzy: an invocation per 8 x BH panel block, the block's
 * error diffusion as straight-line code over named accumulators a0..a63 (no
 * local array: a5xx scratch; no shared memory, no barrier), so one wave's
 * worth of blocks runs with nothing to wait for but its texel fetches. A
 * program per base order of epd_dz_btabs (the chain is the order), its pass
 * fixed too (an aligned table puts a class in one pass: c % passes), so
 * which sides give and which receive are compile-time. The mirror symmetries
 * are runtime: the rows / texels are fetched in the virtual order (a
 * reversed row index, the texel pair swapped and swizzled), the dots are
 * byte / bit reversed at the end, the seam slots and the sides' indices are
 * address arithmetic. Shares across seams go through the Inc SSBO (32 ints a
 * block: 4 sides x 8), written by the giving pixel as it is visited, read by
 * the receiver's pass after a glMemoryBarrier. The greys: out itself
 * (sampler2D on unit 10: RGBA8, 4 px a texel, 16 fetches a block; the
 * fallback upload goes into btex, the same shape) -- a rect the GPU drew since
 * the caller last wrote it there (drawn[]) needs no upload (2.6 MB, ~9 ms on
 * the A509). Dots: a block's BH rows as bytes, block-major. */
static const char *gdz_sides = "RLDU";
static int gdz_side_px(int s, int i, int bh) { return s == 0 ? i * 8 + 7 : s == 1 ? i * 8 : s == 2 ? (bh - 1) * 8 + i : i; }
static void gdz_pass_sides(const epd_dz_btabs *T, int pass, int *gives, int *recv)
{
    int bx = T->passes == 4 ? pass & 1 : pass, by = T->passes == 4 ? pass >> 1 : 0;   /* (a block of that pass) */
    *gives = epd_dzb_gives(T, bx, by);
    *recv = T->passes > 1 ? ~*gives & 15 : 0;   /* (every neighbour is earlier or later) */
}
/* the program of base order b: the chain unrolled from T->ord[b] */
static char *gdz_block_src(const epd_dz_btabs *T, int b, int lvt, int lsz, int stage)
{
    size_t cap = 1 << 18;
    char *src = malloc(cap);
    if (!src) return NULL;
    size_t n = 0;
#define E(...) do { if (n + 512 > cap) { cap *= 2; char *ns = realloc(src, cap); if (!ns) { free(src); return NULL; } src = ns; } \
                    n += (size_t)snprintf(src + n, cap - n, __VA_ARGS__); } while (0)
    int bh = T->bh, npx = T->npx, nbase = T->n / T->nsym, pass = b % T->passes, gives, recv;
    gdz_pass_sides(T, pass, &gives, &recv);
    const uint8_t *O = T->ord[b];
    int kof[EPD_DZB_MAXPX];                     /* position -> its step */
    for (int k = 0; k < npx; k++) kof[O[k]] = k;
    E("#version 310 es\nprecision highp int; precision highp float; precision highp sampler2D; precision highp usampler2D;\n"
      "layout(local_size_x = %d) in;\n"
      "layout(std430, binding = 1) writeonly buffer Bits { uint bits[]; };\n"
      "layout(std430, binding = 2) buffer Inc { int inc[]; };\n"
      "uniform sampler2D grey;    /* out or btex: RGBA8, 4 panel pixels a texel */\n"
      "uniform usampler2D lvt;    /* R8UI 256 x 1: grey -> level | 0x80 fixed (used when LVT) */\n"
      "uniform ivec4 rect;        /* x0, y0, x1, y1 (panel) */\n"
      "uniform ivec4 blk;         /* bx0, by0, bx1, by1 (the rect's blocks) */\n"
      "uniform ivec4 tl;          /* tx0, ty0, ntx, ntiles (64 x 64 tiles) */\n"
      "uniform ivec2 lim;         /* tw - 1, ph - 1 */\n"
      "uniform int total;         /* invocations */\n"
      "uvec4 tx(int x, int y) { return uvec4(round(texelFetch(grey, clamp(ivec2(x, y), ivec2(0), lim), 0) * 255.0)); }\n"
      "int lv(uint g, bool i) {\n", lsz);
    if (lvt) E("  uint e = texelFetch(lvt, ivec2(int(g), 0), 0).r;\n  return i && (e & 0x80u) == 0u ? int(e & 31u) * 256 + 65536 : 0;\n}\n");
    else E("  return i && g >= 16u && g < 240u ? int((g * 30u + 127u) * 32897u >> 23) * 256 + 65536 : 0;\n}\n");
    E("uint bswap(uint v) { return (v << 24) | ((v & 0xff00u) << 8) | ((v >> 8) & 0xff00u) | (v >> 24); }\n"
      "void main() {\n"
      "  int id = int(gl_GlobalInvocationID.x);\n  if (id >= total) return;\n"
      "  int per = %d * tl.w, sym = id / per, r = id - sym * per, s = r / tl.w, t = r - s * tl.w;\n"
      "  int tx_ = t %% tl.z + tl.x, ty_ = t / tl.z + tl.y;\n  int pos = ", T->slots);
    for (int sy = 0; sy < T->nsym; sy++) {     /* the block of (symmetry, slot): a tree of constants */
        if (sy + 1 < T->nsym) E("sym == %d ? (", sy);
        for (int sl = 0; sl < T->slots; sl++) {
            int c = sy * nbase + b;
            if (sl + 1 < T->slots) E("s == %d ? %d : ", sl, T->cpos[c][sl]);
            else E("%d", T->cpos[c][sl]);
        }
        E(sy + 1 < T->nsym ? ") : " : ";\n");
    }
    E("  int bx = tx_ * %d + pos %% %d, by = ty_ * %d + pos / %d;\n"
      "  if (bx < blk.x || bx >= blk.z || by < blk.y || by >= blk.w) return;\n"
      "  int px0 = bx * 8, py0 = by * %d, nbxr = blk.z - blk.x, bi = (by - blk.y) * nbxr + (bx - blk.x), txl = px0 >> 2;\n"
      "  bool mx = (sym & 1) != 0, my = (sym & 2) != 0;\n"
      "  int xlo = rect.x - px0, xhi = rect.z - px0, ylo = rect.y - py0, yhi = rect.w - py0;\n"
      "  int vxlo = mx ? 8 - xhi : xlo, vxhi = mx ? 8 - xlo : xhi, vylo = my ? %d - yhi : ylo, vyhi = my ? %d - ylo : yhi;\n"
      "  int tA = mx ? txl + 1 : txl, tB = mx ? txl : txl + 1;\n",
      T->nbx, T->nbx, T->nby, T->nbx, bh, bh, bh);
    /* the block's rows in the virtual order, the pixels as accumulators */
    for (int vy = 0; vy < bh; vy++) {
        E("  { int py = py0 + (my ? %d : %d); bool yin = %d >= vylo && %d < vyhi;\n"
          "    uvec4 qa = tx(tA, py), qb = tx(tB, py); if (mx) { qa = qa.wzyx; qb = qb.wzyx; }\n", bh - 1 - vy, vy, vy, vy);
        for (int vx = 0; vx < 8; vx++)
            E("    a%d = lv(%s.%c, yin && %d >= vxlo && %d < vxhi);\n", vy * 8 + vx, vx < 4 ? "qa" : "qb", "xyzw"[vx & 3], vx, vx);
        E("  }\n");
    }
    /* which neighbours exist, the sides' physical names, the seam slots */
    E("  int spR = mx ? 1 : 0, spL = 1 - spR, spD = my ? 3 : 2, spU = 5 - spD;\n"
      "  int rbR = mx ? bi - 1 : bi + 1, rbL = mx ? bi + 1 : bi - 1, rbD = my ? bi - nbxr : bi + nbxr, rbU = my ? bi + nbxr : bi - nbxr;\n"
      "  bool hasR = mx ? bx > blk.x : bx + 1 < blk.z, hasL = mx ? bx + 1 < blk.z : bx > blk.x;\n"
      "  bool hasD = my ? by > blk.y : by + 1 < blk.w, hasU = my ? by + 1 < blk.w : by > blk.y;\n");
    if (stage == 1) {                           /* (timing: the fetches and levels alone, a plain threshold) */
        E("  uint d0 = 0u, d1 = 0u;\n");
        for (int p = 0; p < npx; p++) E("  d%d |= uint(a%d >= 32768 && a%d - 65536 >= 3840) << %du;\n", p / 32, p, p, p % 32);
        recv = gives = 0;
    }
    /* what the earlier passes gave (the receiving sides) */
    for (int s = 0; s < 4; s++) {
        if (!(recv & 1 << s)) continue;
        int len = s < 2 ? bh : 8;
        for (int i = 0; i < len; i++) {
            int v = gdz_side_px(s, i, bh);
            if (s < 2) E("  a%d += a%d >= 32768 && has%c ? inc[bi * 32 + sp%c * 8 + (my ? %d : %d)] : 0;\n", v, v, gdz_sides[s], gdz_sides[s], bh - 1 - i, i);
            else E("  a%d += a%d >= 32768 && has%c ? inc[bi * 32 + sp%c * 8 + (mx ? %d : %d)] : 0;\n", v, v, gdz_sides[s], gdz_sides[s], 7 - i, i);
        }
    }
    /* the pixels across the giving sides: free or not */
    if (gives & 1 << 0 || gives & 1 << 1) {
        E("  int pxR = mx ? px0 - 1 : px0 + 8, pxL = mx ? px0 + 8 : px0 - 1;\n");
        for (int s = 0; s < 2; s++) {
            if (!(gives & 1 << s)) continue;
            for (int i = 0; i < bh; i++)
                E("  bool o%c%d; { int py = py0 + (my ? %d : %d); uvec4 q = tx(px%c >> 2, py);\n"
                  "    o%c%d = lv(%s ? q.w : q.x, px%c >= rect.x && px%c < rect.z && py >= rect.y && py < rect.w) != 0; }\n",
                  gdz_sides[s], i, bh - 1 - i, i, gdz_sides[s], gdz_sides[s], i, s == 0 ? "mx" : "!mx", gdz_sides[s], gdz_sides[s]);
        }
    }
    for (int s = 2; s < 4; s++) {
        if (!(gives & 1 << s)) continue;
        E("  bool ");
        for (int i = 0; i < 8; i++) E("o%c%d%s", gdz_sides[s], i, i < 7 ? ", " : ";\n");
        E("  { int py = my ? py0 + %d : py0 + %d; bool yin = py >= rect.y && py < rect.w;\n"
          "    uvec4 qa = tx(tA, py), qb = tx(tB, py); if (mx) { qa = qa.wzyx; qb = qb.wzyx; }\n",
          s == 2 ? -1 : bh, s == 2 ? bh : -1);
        for (int i = 0; i < 8; i++)
            E("    o%c%d = lv(%s.%c, yin && %d >= vxlo && %d < vxhi) != 0;\n", gdz_sides[s], i, i < 4 ? "qa" : "qb", "xyzw"[i & 3], i, i);
        E("  }\n");
    }
    /* the chain */
    if (stage != 1) E("  uint d0 = 0u, d1 = 0u;\n");
    for (int k = 0; k < (stage == 1 ? 0 : npx); k++) {
        int p = O[k], x = p % 8, y = p / 8;
        E("  { bool f = a%d >= 32768; int v = clamp(a%d - 65536, 0, 7680); bool hi = v >= 3840; int e = v - (hi ? 7680 : 0);\n"
          "    d%d |= uint(f && hi) << %du;\n", p, p, y >> 2, (y & 3) * 8 + x);
        int q[4], nq = 0, side[2], ns = 0;
        if (x + 1 < 8 && kof[p + 1] > k) q[nq++] = p + 1;
        if (x > 0 && kof[p - 1] > k) q[nq++] = p - 1;
        if (y + 1 < bh && kof[p + 8] > k) q[nq++] = p + 8;
        if (y > 0 && kof[p - 8] > k) q[nq++] = p - 8;
        if (x == 7 && gives & 1 << 0) side[ns++] = 0;
        if (x == 0 && gives & 1 << 1) side[ns++] = 1;
        if (y == bh - 1 && gives & 1 << 2) side[ns++] = 2;
        if (y == 0 && gives & 1 << 3) side[ns++] = 3;
        E("    int n = 0");
        for (int i = 0; i < nq; i++) E(" + int(a%d >= 32768)", q[i]);
        for (int i = 0; i < ns; i++) E(" + int(o%c%d)", gdz_sides[side[i]], side[i] < 2 ? y : x);
        E(";\n    int sh = (e * (n == 1 ? 32768 : n == 2 ? 16384 : n == 3 ? 10923 : n == 4 ? 8192 : 0)) >> 15;\n");
        for (int i = 0; i < nq; i++) E("    a%d += f && a%d >= 32768 ? sh : 0;\n", q[i], q[i]);
        for (int i = 0; i < ns; i++) {
            int s = side[i], idx = s < 2 ? y : x;
            char c = gdz_sides[s];
            if (s < 2) E("    if (has%c) inc[rb%c * 32 + (sp%c ^ 1) * 8 + (my ? %d : %d)] = f && o%c%d ? sh : 0;\n", c, c, c, bh - 1 - idx, idx, c, idx);
            else E("    if (has%c) inc[rb%c * 32 + (sp%c ^ 1) * 8 + (mx ? %d : %d)] = f && o%c%d ? sh : 0;\n", c, c, c, 7 - idx, idx, c, idx);
        }
        if (k + 1 < npx) E("    bool f1 = a%d >= 32768; a%d += f && n == 0 && f1 ? e : 0;\n", O[k + 1], O[k + 1]);
        if (k + 2 < npx) E("    a%d += f && n == 0 && !f1 && a%d >= 32768 ? e : 0;\n", O[k + 2], O[k + 2]);
        E("  }\n");
    }
    /* the dots into the physical order, out */
    E("  if (mx) { d0 = bswap(bitfieldReverse(d0)); d1 = bswap(bitfieldReverse(d1)); }\n");
    if (bh == 8) E("  if (my) { uint t2 = bswap(d0); d0 = bswap(d1); d1 = t2; }\n  bits[bi * 2] = d0; bits[bi * 2 + 1] = d1;\n}\n");
    else E("  if (my) d0 = bswap(d0);\n  bits[bi] = d0;\n}\n");
    /* the accumulators, declared at the top of main (after the fact: a second header) */
    char *decl = malloc(16 * (size_t)npx + 64);
    if (!decl) { free(src); return NULL; }
    size_t dn = (size_t)sprintf(decl, "  int ");
    for (int i = 0; i < npx; i++) dn += (size_t)sprintf(decl + dn, "a%d%s", i, i + 1 < npx ? ", " : ";\n");
    const char *at = strstr(src, "  int id = int(gl_GlobalInvocationID.x);");
    size_t off = (size_t)(at - src);
    char *full = malloc(n + dn + 1);
    if (!full) { free(src); free(decl); return NULL; }
    memcpy(full, src, off);
    memcpy(full + off, decl, dn);
    memcpy(full + off + dn, src + off, n - off + 1);
    free(src); free(decl);
    return full;
#undef E
}

/* the canonical level table (glassine-epdd-desktop's dz_lv): the shader's ALU form */
static int gdz_lv_canonical(const uint8_t lv[256])
{
    for (int v = 0; v < 256; v++)
        if (lv[v] != (uint8_t)((v * 30 + 127) / 255 | ((v >> 4) == 0 || (v >> 4) == 15 ? EPD_DZ_FIXED : 0))) return 0;
    return 1;
}

static int gdz_block_setup(epd_ingest *g)
{
    struct gdz *z = &g->dz;
#define G(n) if (!z->n && !(z->n = (void *)eglGetProcAddress(#n))) return -1;
    G(glDispatchCompute) G(glMemoryBarrier) G(glGenBuffers) G(glBindBuffer) G(glBindBufferBase) G(glBufferData)
    G(glMapBufferRange) G(glUnmapBuffer)
#undef G
    const char *ver = (const char *)g->glGetString(GL_VERSION);
    if (!ver || strstr(ver, "OpenGL ES 3.0")) return -1;           /* (compute: ES 3.1) */
    /* the knobs: GLASSINE_GDZ_BLOCK=8x8|8x4, GLASSINE_GDZ_ORDERS (16), GLASSINE_GDZ_SYMS (4: the mirrors; 1, 2), GLASSINE_GDZ_PASSES (4), GLASSINE_GDZ_LSZ (64) */
    const char *e;
    int bh = (e = getenv("GLASSINE_GDZ_BLOCK")) && !strcmp(e, "8x4") ? 4 : 8;
    int nbase = (e = getenv("GLASSINE_GDZ_ORDERS")) ? atoi(e) : 16;
    int nsym = (e = getenv("GLASSINE_GDZ_SYMS")) ? atoi(e) : 4;
    int passes = (e = getenv("GLASSINE_GDZ_PASSES")) ? atoi(e) : 4;
    z->blsz = (e = getenv("GLASSINE_GDZ_LSZ")) && atoi(e) >= 32 && atoi(e) <= 1024 ? atoi(e) : 64;
    z->bstage = (e = getenv("GLASSINE_GDZ_BSTAGE")) ? atoi(e) : 0;   /* (timing only: 1 = fetch + levels, dots by threshold, no chain, no seams) */
    if (nsym != 1 && nsym != 2 && nsym != 4) return -1;
    if (!z->bt && !(z->bt = malloc(sizeof *z->bt))) return -1;
    if (epd_dz_btabs_init(z->bt, 8, bh, nbase * nsym, passes, 0, nsym) < 0 || nbase % passes) {
        fprintf(stderr, "epd-ingest: dizzy: no block tables for 8x%d, %d orders x %d, %d passes\n", bh, nbase, nsym, passes);
        return -1;
    }
    z->lvt = !gdz_lv_canonical(z->lv);
    if (!z->ltex) { g->glActiveTexture(GL_TEXTURE9); z->ltex = lut_tex(g, z->lv); }
    for (int b = 0; b < nbase; b++) {
        char *src = gdz_block_src(z->bt, b, z->lvt, z->blsz, z->bstage);
        if (!src) return -1;
        if (getenv("GLASSINE_GDZ_DUMP") && !b) fputs(src, stderr);
        GLuint c = shader(g, GL_COMPUTE_SHADER, src);
        free(src);
        if (!c) return -1;
        if (z->bprog[b]) g->glDeleteProgram(z->bprog[b]);  /* (a rebuild: the level table turned non-canonical) */
        GLuint p = g->glCreateProgram();
        g->glAttachShader(p, c);
        g->glLinkProgram(p);
        g->glDeleteShader(c);
        GLint ok = 0;
        g->glGetProgramiv(p, GL_LINK_STATUS, &ok);
        if (!ok) {
            char log[1024];
            g->glGetProgramInfoLog(p, sizeof log, NULL, log);
            fprintf(stderr, "epd-ingest: dizzy block %d: link: %s\n", b, log);
            return -1;
        }
        z->bprog[b] = p;
        g->glUseProgram(p);
        g->glUniform1i(g->glGetUniformLocation(p, "grey"), 10);
        g->glUniform1i(g->glGetUniformLocation(p, "lvt"), 9);
        z->bu[b].rect = g->glGetUniformLocation(p, "rect");
        z->bu[b].blk = g->glGetUniformLocation(p, "blk");
        z->bu[b].tl = g->glGetUniformLocation(p, "tl");
        z->bu[b].lim = g->glGetUniformLocation(p, "lim");
        z->bu[b].total = g->glGetUniformLocation(p, "total");
    }
    g->glUseProgram(g->prog);
    if (!z->bits) z->glGenBuffers(1, &z->bits);
    if (!z->inc) z->glGenBuffers(1, &z->inc);
    if (!z->btex) {                             /* the upload's texture (out's shape); unit 10 = the job's grey */
        g->glActiveTexture(GL_TEXTURE10);
        g->glGenTextures(1, &z->btex);
        g->glBindTexture(GL_TEXTURE_2D, z->btex);
        g->glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, g->tw, g->ph);
        g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    }
    if (g->glGetError() != GL_NO_ERROR) { fprintf(stderr, "epd-ingest: dizzy block: GL error in setup\n"); return -1; }
    if (getenv("GLASSINE_GDZ_DEBUG"))
        fprintf(stderr, "epd-ingest: dizzy block: 8x%d blocks, %d orders x %d symmetries, %d passes, local size %d, %s levels\n",
                bh, nbase, nsym, passes, z->blsz, z->lvt ? "LUT" : "ALU");
    return 0;
}

/* the rect's texels all drawn by the GPU since the caller last wrote them */
static int gdz_drawn(const epd_ingest *g, int x0, int y0, int w, int h)
{
    int t0 = x0 / 4, t1 = (x0 + w + 3) / 4;
    for (int row = y0; row < y0 + h; row++) {
        const uint8_t *d = g->drawn + (size_t)row * g->tw;
        for (int t = t0; t < t1; t++) if (!d[t]) return 0;
    }
    return 1;
}

/* the block form's job: the greys from out (view: the rect drawn there) or
 * uploaded into btex, a dispatch per base order, pass by pass */
static int gdz_submit_block(epd_ingest *g, const uint8_t *grey, int gs, int x0, int y0, int w, int h, int view)
{
    struct gdz *z = &g->dz;
    if (!z->bst) {
        if (gdz_block_setup(g) < 0) { z->bst = -1; fprintf(stderr, "epd-ingest: no block dizzy on the GPU\n"); return -1; }
        z->bst = 1;
        if (getenv("GLASSINE_GDZ_COMPILE_ONLY")) return -1;
    }
    const epd_dz_btabs *T = z->bt;
    int bh = T->bh, bx0 = x0 / 8, bx1 = (x0 + w + 7) / 8, by0 = y0 / bh, by1 = (y0 + h + bh - 1) / bh;
    int nbxr = bx1 - bx0, nbyr = by1 - by0, wpb = T->npx / 32;
    int tx0 = bx0 / T->nbx, tx1 = (bx1 - 1) / T->nbx + 1, ty0 = by0 / T->nby, ty1 = (by1 - 1) / T->nby + 1;
    int ntx = tx1 - tx0, ntiles = ntx * (ty1 - ty0);
    g->glActiveTexture(GL_TEXTURE10);
    if (view && gdz_drawn(g, x0, y0, w, h)) {
        g->glBindTexture(GL_TEXTURE_2D, g->out);
        z->bsrc = 1; z->nview++;
    } else {                                    /* whole texels: the rect's bytes, the edges' others 0 (fixed) */
        int t0 = x0 / 4, t1 = (x0 + w + 3) / 4;
        size_t rb = (size_t)(t1 - t0) * 4;
        if (rb > z->brcap) { free(z->brow); z->brow = malloc(rb); z->brcap = z->brow ? rb : 0; if (!z->brow) return -1; }
        g->glBindTexture(GL_TEXTURE_2D, z->btex);
        for (int y = 0; y < h; y++) {
            memset(z->brow, 0, rb);
            memcpy(z->brow + (x0 & 3), grey + (size_t)y * gs, (size_t)w);
            g->glTexSubImage2D(GL_TEXTURE_2D, 0, t0, y0 + y, t1 - t0, 1, GL_RGBA, GL_UNSIGNED_BYTE, z->brow);
        }
        z->bsrc = 0; z->nup++;
    }
    size_t nb = (size_t)nbxr * nbyr, need = nb * wpb * 4, ineed = nb * 32 * 4;
    z->glBindBuffer(GL_SHADER_STORAGE_BUFFER, z->bits);
    if (need > z->bcap) { z->glBufferData(GL_SHADER_STORAGE_BUFFER, (GLsizeiptr)need, NULL, GL_STREAM_READ); z->bcap = need; }
    z->glBindBuffer(GL_SHADER_STORAGE_BUFFER, z->inc);
    if (ineed > z->icap) { z->glBufferData(GL_SHADER_STORAGE_BUFFER, (GLsizeiptr)ineed, NULL, GL_STATIC_COPY); z->icap = ineed; }
    z->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, z->bits);
    z->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, z->inc);
    int nbase = T->n / T->nsym, total = T->nsym * T->slots * ntiles;
    for (int pass = 0; pass < T->passes; pass++) {
        if (pass) z->glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);   /* (the seam shares of the pass before) */
        for (int b = 0; b < nbase; b++) {
            if (b % T->passes != pass) continue;
            g->glUseProgram(z->bprog[b]);
            g->glUniform4i(z->bu[b].rect, x0, y0, x0 + w, y0 + h);
            g->glUniform4i(z->bu[b].blk, bx0, by0, bx1, by1);
            g->glUniform4i(z->bu[b].tl, tx0, ty0, ntx, ntiles);
            g->glUniform2i(z->bu[b].lim, g->tw - 1, g->ph - 1);
            g->glUniform1i(z->bu[b].total, total);
            z->glDispatchCompute((GLuint)((total + z->blsz - 1) / z->blsz), 1, 1);
        }
    }
    z->bx0 = bx0; z->by0 = by0; z->nbxr = nbxr; z->nbyr = nbyr; z->wpb = wpb;
    return 0;
}

/* the block job's bits (a block's rows as bytes, block-major) into out */
static void gdz_collect_block(epd_ingest *g, const uint32_t *b, uint8_t *out, int os)
{
    struct gdz *z = &g->dz;
    static uint64_t lut8[256];                  /* a byte of dots -> 8 pixels of 0 / 30 */
    static uint32_t *row;
    static size_t rcap;
    if (!lut8[255]) for (int v = 0; v < 256; v++) for (int i = 0; i < 8; i++) lut8[v] |= (uint64_t)((v >> i & 1) * 30) << (8 * i);
    int bh = z->bt->bh, wpb = z->wpb, x1 = z->x0 + z->w, y1 = z->y0 + z->h;
    size_t rw = (size_t)z->nbxr * wpb;
    if (rw > rcap) { free(row); row = malloc(rw * 4); rcap = row ? rw : 0; }
    for (int by = 0; by < z->nbyr; by++) {
        const uint32_t *br = b + (size_t)by * rw;
        if (row) { memcpy(row, br, rw * 4); br = row; }     /* (one read of the write-combined map) */
        for (int bx = 0; bx < z->nbxr; bx++) {
            int px = (z->bx0 + bx) * 8, py = (z->by0 + by) * bh;
            const uint32_t *wd = br + (size_t)bx * wpb;
            int whole = px >= z->x0 && px + 8 <= x1;
            for (int r = 0; r < bh; r++) {
                int y = py + r;
                if (y < z->y0 || y >= y1) continue;
                uint64_t v = lut8[wd[r >> 2] >> (8 * (r & 3)) & 255];
                uint8_t *o = out + (size_t)(y - z->y0) * os + (px - z->x0);
                if (whole) memcpy(o, &v, 8);
                else for (int i = 0; i < 8; i++) if (px + i >= z->x0 && px + i < x1) o[i] = (uint8_t)(v >> (8 * i));
            }
        }
    }
}

int epd_ingest_dizzy_form(epd_ingest *g, int form)
{
    if (!g) return -1;
    if (form != EPD_GDZ_LAYERED && form != EPD_GDZ_BLOCK) return g->dz.form;
    g->dz.form = form;
    return form;
}

int epd_ingest_dizzy_submit_v(epd_ingest *g, const uint8_t *grey, int gs, int x0, int y0, int w, int h, const uint8_t lv[256], int view)
{
    struct gdz *z = &g->dz;
    if (!g || !current(g) || w <= 0 || h <= 0 || x0 < 0 || y0 < 0 || x0 + w > g->pw || y0 + h > g->ph) return -1;
    if (z->on) epd_ingest_dizzy_collect(g, NULL, 0, 0);     /* (one job at a time) */
    if (!z->st && !z->bst) {
        memcpy(z->lv, lv, 256);
        const char *e = getenv("GLASSINE_GDZ_FORM");         /* (the tests; glassine-epdd-desktop: epd_ingest_dizzy_form) */
        if (e) z->form = !strcmp(e, "layered") ? EPD_GDZ_LAYERED : EPD_GDZ_BLOCK;
    }
    if (memcmp(z->lv, lv, 256)) {
        memcpy(z->lv, lv, 256);
        if (z->ltex) lut_set(g, GL_TEXTURE9, z->ltex, lv);
        if (z->bst > 0 && !z->lvt && !gdz_lv_canonical(lv)) { z->lvt = 1; z->bst = 0; }   /* (the ALU programs: rebuild) */
    }
    int rc = z->form == EPD_GDZ_BLOCK ? (z->bst < 0 ? -1 : gdz_submit_block(g, grey, gs, x0, y0, w, h, view))
                                      : (z->st < 0 ? -1 : gdz_submit_layered(g, grey, gs, x0, y0, w, h));
    if (rc < 0) { g->glUseProgram(g->prog); return -1; }
    z->glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);
    z->f = g->glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    g->glFlush();
    g->glUseProgram(g->prog);
    z->on = 1; z->x0 = x0; z->y0 = y0; z->w = w; z->h = h; z->bsrc = z->form == EPD_GDZ_BLOCK ? z->bsrc : 0;
    return z->f ? 0 : -1;
}

int epd_ingest_dizzy_submit(epd_ingest *g, const uint8_t *grey, int gs, int x0, int y0, int w, int h, const uint8_t lv[256])
{
    return epd_ingest_dizzy_submit_v(g, grey, gs, x0, y0, w, h, lv, 0);
}

int epd_ingest_dizzy_collect(epd_ingest *g, uint8_t *out, int os, int timeout_ms)
{
    struct gdz *z = &g->dz;
    if (!z->on || !current(g)) { z->on = 0; return -1; }
    z->on = 0;
    GLenum wt = g->glClientWaitSync(z->f, GL_SYNC_FLUSH_COMMANDS_BIT, (GLuint64)(timeout_ms > 0 ? timeout_ms : 2000) * 1000000);
    if (wt == GL_TIMEOUT_EXPIRED || wt == GL_WAIT_FAILED) {
        fprintf(stderr, "epd-ingest: the GPU did not finish dizzy in %d ms: ingest abandoned\n", timeout_ms);
        ing_abandon(g, z->f);
        return -1;
    }
    g->glDeleteSync(z->f);
    if (!out) return 0;
    int block = z->form == EPD_GDZ_BLOCK;
    z->glBindBuffer(GL_SHADER_STORAGE_BUFFER, z->bits);
    size_t sz = block ? (size_t)z->nbxr * z->nbyr * z->wpb * 4 : (size_t)((z->y0 + z->h + 63) / 64 - z->ty0) * 64 * z->ows * 4;
    const uint32_t *b = z->glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, (GLsizeiptr)sz, GL_MAP_READ_BIT);
    if (!b) return -1;
    if (block) gdz_collect_block(g, b, out, os);
    else gdz_collect_layered(g, b, out, os);
    z->glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    return g->glGetError() == GL_NO_ERROR ? 0 : -1;
}

int epd_ingest_dizzy_src(const epd_ingest *g) { return g ? g->dz.bsrc : 0; }

int epd_ingest_dizzy(epd_ingest *g, const uint8_t *grey, int gs, int x0, int y0, int w, int h, const uint8_t lv[256],
                     uint8_t *out, int os, int timeout_ms)
{
    if (epd_ingest_dizzy_submit_v(g, grey, gs, x0, y0, w, h, lv, 0) < 0) return -1;
    return epd_ingest_dizzy_collect(g, out, os, timeout_ms);
}

int epd_ingest_dizzy_v(epd_ingest *g, const uint8_t *grey, int gs, int x0, int y0, int w, int h, const uint8_t lv[256],
                       uint8_t *out, int os, int timeout_ms, int view)
{
    if (epd_ingest_dizzy_submit_v(g, grey, gs, x0, y0, w, h, lv, view) < 0) return -1;
    return epd_ingest_dizzy_collect(g, out, os, timeout_ms);
}

/* (tests, gdz-probe: a grey image into out as if the GPU had drawn it, so
 * the view path runs without a dma-buf; whole texels: x0 and w multiples of 4) */
int epd_ingest_dizzy_seed(epd_ingest *g, const uint8_t *grey, int gs, int x0, int y0, int w, int h)
{
    if (!g || x0 % 4 || w % 4 || x0 < 0 || y0 < 0 || x0 + w > g->pw || y0 + h > g->ph || !current(g)) return -1;
    g->glActiveTexture(GL_TEXTURE2);
    g->glBindTexture(GL_TEXTURE_2D, g->out);
    g->glPixelStorei(GL_UNPACK_ROW_LENGTH, gs / 4);
    g->glTexSubImage2D(GL_TEXTURE_2D, 0, x0 / 4, y0, w / 4, h, GL_RGBA, GL_UNSIGNED_BYTE, grey);
    g->glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    g->glFinish();                              /* (A509: a staged upload into a busy texture can vanish; land it) */
    if (g->hmap) {
        /* the cached dma-heap target (heap_target): the driver's CPU copy into the
         * imported pages sits in the CPU's cache; the GPU reads memory. Nothing
         * in GL flushes it (the frames never write out from the CPU), so this
         * does: END + WRITE cleans the buffer's lines to memory (the
         * device's view runs differed on ~20 % of the pixels with the upload
         * exact; the seeded greys, not the shader) */
        struct dma_buf_sync sy = { DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE };
        ioctl(g->hfd, DMA_BUF_IOCTL_SYNC, &sy);
        sy.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE;
        ioctl(g->hfd, DMA_BUF_IOCTL_SYNC, &sy);
    }
    for (int row = y0; row < y0 + h; row++) memset(g->drawn + (size_t)row * g->tw + x0 / 4, 1, (size_t)w / 4);
    return g->glGetError() == GL_NO_ERROR ? 0 : -1;
}

/* the greys of a rect (x0 a multiple of 4, w too) as the block shader would
 * fetch them: how 0 = a compute program with the shader's own tx on the
 * texture the next view job would read (unit 10, out); how 1 = glReadPixels
 * of out (the frames' sampled readback path). Into grey (gs apart). Tells a
 * wrong texel apart from a wrong chain. -1: not done */
int epd_ingest_dizzy_peek(epd_ingest *g, int x0, int y0, int w, int h, uint8_t *grey, int gs, int how)
{
    if (!g || x0 % 4 || w % 4 || w <= 0 || h <= 0 || x0 < 0 || y0 < 0 || x0 + w > g->pw || y0 + h > g->ph || !current(g)) return -1;
    struct gdz *z = &g->dz;
    if (how == 1) {
        g->glBindFramebuffer(GL_FRAMEBUFFER, g->fbo);
        g->glFinish();
        for (int row = 0; row < h; row++)
            g->glReadPixels(x0 / 4, y0 + row, w / 4, 1, GL_RGBA, GL_UNSIGNED_BYTE, grey + (size_t)row * gs);
        return g->glGetError() == GL_NO_ERROR ? 0 : -1;
    }
    if (z->bst <= 0 && (gdz_block_setup(g) < 0)) { z->bst = -1; return -1; }
    z->bst = 1;
    if (!z->pprog) {
        static const char *src =
            "#version 310 es\nprecision highp int; precision highp float; precision highp sampler2D;\n"
            "layout(local_size_x = 64) in;\n"
            "layout(std430, binding = 1) writeonly buffer Out { uint o[]; };\n"
            "uniform sampler2D grey;\nuniform ivec4 rect;  /* tx0, y0, ntx, ntexels */\nuniform ivec2 lim;\n"
            "uvec4 tx(int x, int y) { return uvec4(round(texelFetch(grey, clamp(ivec2(x, y), ivec2(0), lim), 0) * 255.0)); }\n"
            "void main() {\n  int id = int(gl_GlobalInvocationID.x);\n  if (id >= rect.w) return;\n"
            "  uvec4 q = tx(rect.x + id % rect.z, rect.y + id / rect.z);\n"
            "  o[id] = q.x | (q.y << 8) | (q.z << 16) | (q.w << 24);\n}\n";
        GLuint c = shader(g, GL_COMPUTE_SHADER, src);
        if (!c) return -1;
        GLuint p = g->glCreateProgram();
        g->glAttachShader(p, c);
        g->glLinkProgram(p);
        g->glDeleteShader(c);
        GLint ok = 0;
        g->glGetProgramiv(p, GL_LINK_STATUS, &ok);
        if (!ok) { g->glDeleteProgram(p); return -1; }
        z->pprog = p;
        g->glUseProgram(p);
        g->glUniform1i(g->glGetUniformLocation(p, "grey"), 10);
        g->glUniform2i(g->glGetUniformLocation(p, "lim"), g->tw - 1, g->ph - 1);
        z->pu_rect = g->glGetUniformLocation(p, "rect");
        z->glGenBuffers(1, &z->pbuf);
    }
    int ntx = w / 4, n = ntx * h;
    size_t need = (size_t)n * 4;
    g->glActiveTexture(GL_TEXTURE10);
    g->glBindTexture(GL_TEXTURE_2D, g->out);
    z->glBindBuffer(GL_SHADER_STORAGE_BUFFER, z->pbuf);
    if (need > z->pcap) { z->glBufferData(GL_SHADER_STORAGE_BUFFER, (GLsizeiptr)need, NULL, GL_STREAM_READ); z->pcap = need; }
    z->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, z->pbuf);
    g->glUseProgram(z->pprog);
    g->glUniform4i(z->pu_rect, x0 / 4, y0, ntx, n);
    z->glDispatchCompute((GLuint)((n + 63) / 64), 1, 1);
    z->glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);
    g->glFinish();
    g->glUseProgram(g->prog);
    const uint32_t *b = z->glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, (GLsizeiptr)need, GL_MAP_READ_BIT);
    if (!b) return -1;
    for (int row = 0; row < h; row++) memcpy(grey + (size_t)row * gs, b + (size_t)row * ntx, (size_t)ntx * 4);
    z->glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    return g->glGetError() == GL_NO_ERROR ? 0 : -1;
}

/* ---- stage 5: the shadow mirror and GPU targets ----
 *
 * gsh[0] holds the caller's shadow in out's layout (RGBA8, texel (x / 4, y) =
 * panel pixels x .. x + 3 of row y). A commit pass writes its texels into
 * gsh[1], they are blitted back (a pass can't read what it writes). Units: 2
 * = out (bound at open), 3 = gsh[0], 4 = whatever is being created. */
static const char *fs_commit =
    "#version 300 es\n"
    "precision highp float; precision highp int;\n"
    "uniform sampler2D G;      /* out: the ingest's grey */\n"
    "uniform sampler2D S;      /* gsh[0]: shadow */\n"
    "uniform ivec4 box;        /* panel cols [x, y) x rows [z, w) */\n"
    "uniform int how, all;     /* EPD_INGEST_C_*, commit's `all` */\n"
    "uniform highp usampler2D T; /* the dither's thresholds: epd_dither_table()[row & 63][col & 63] */\n"
    "out vec4 o;\n"
    /* glassine-epdd-desktop commit: commit_bw_row (C_BW), commit_q4_row (C_Q4),
     * commit_bw2_row (C_DITHER, C_THRESH at 127), memcpy (C_ALL); the same
     * integer arithmetic per pixel */
    "uint px(uint v, uint s, int row, int col) {\n"
    "  uint hv = v >> 4;\n"
    "  bool bw = hv == 0u || hv == 15u;\n"
    "  if (how == 1) return ((s ^ v) >> 4) != 0u && !bw ? s : v;\n"
    "  if (how < 2 || how > 4) return v;\n"
    "  uint t = how == 3 ? 127u : texelFetch(T, ivec2(col & 63, row & 63), 0).r, q;\n"
    "  if (how == 4) {\n"
    "    uint t3 = v * 3u;\n"
    "    uint k = uint(t3 >= 255u) + uint(t3 >= 510u) + uint(t3 >= 765u);\n"
    "    k += uint(t3 - k * 255u > t);\n"
    "    q = k * 85u;\n"
    "  } else q = v > t ? 255u : 0u;\n"
    "  return s == v && all == 0 ? s : bw ? v : q;\n"
    "}\n"
    "void main() {\n"
    "  ivec2 c = ivec2(gl_FragCoord.xy);\n"
    "  uvec4 g = uvec4(round(texelFetch(G, c, 0) * 255.0)), s = uvec4(round(texelFetch(S, c, 0) * 255.0)), r = s;\n"
    "  bool rin = c.y >= box.z && c.y < box.w;\n"
    "  int x = c.x * 4;\n"
    "  if (rin && x >= box.x && x < box.y) r.r = px(g.r, s.r, c.y, x);\n"
    "  if (rin && x + 1 >= box.x && x + 1 < box.y) r.g = px(g.g, s.g, c.y, x + 1);\n"
    "  if (rin && x + 2 >= box.x && x + 2 < box.y) r.b = px(g.b, s.b, c.y, x + 2);\n"
    "  if (rin && x + 3 >= box.x && x + 3 < box.y) r.a = px(g.a, s.a, c.y, x + 3);\n"
    "  o = vec4(r) / 255.0;\n"
    "}\n";
/* a target: epd_target_make's levels, (grey & 0xF0) >> 3, 4 per texel */
static const char *fs_make =
    "#version 300 es\n"
    "precision highp float; precision highp int;\n"
    "uniform sampler2D S;\n"
    "out vec4 o;\n"
    "void main() {\n"
    "  uvec4 s = uvec4(round(texelFetch(S, ivec2(gl_FragCoord.xy), 0) * 255.0));\n"
    "  o = vec4((s & 240u) >> 3) / 255.0;\n"
    "}\n";

static GLuint link_fs(epd_ingest *g, const char *fsrc)
{
    GLuint v = shader(g, GL_VERTEX_SHADER, vs), f = shader(g, GL_FRAGMENT_SHADER, fsrc);
    if (!v || !f) return 0;
    GLuint p = g->glCreateProgram();
    g->glAttachShader(p, v);
    g->glAttachShader(p, f);
    g->glLinkProgram(p);
    GLint ok = 0;
    g->glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024];
        g->glGetProgramInfoLog(p, sizeof log, NULL, log);
        fprintf(stderr, "epd-ingest: link: %s\n", log);
        return 0;
    }
    return p;
}

static GLuint fbo_on(epd_ingest *g, GLuint tex)
{
    GLuint f;
    g->glGenFramebuffers(1, &f);
    g->glBindFramebuffer(GL_FRAMEBUFFER, f);
    g->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);
    GLenum st = g->glCheckFramebufferStatus(GL_FRAMEBUFFER);
    g->glBindFramebuffer(GL_FRAMEBUFFER, g->fbo);
    return st == GL_FRAMEBUFFER_COMPLETE ? f : 0;
}

static GLuint rgba_tex(epd_ingest *g)
{
    GLuint t;
    g->glGenTextures(1, &t);
    g->glBindTexture(GL_TEXTURE_2D, t);
    g->glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, g->tw, g->ph);
    g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    return t;
}

/* a layer: a texture of ours exported as a dma-buf (EGL_MESA_image_dma_buf_export:
 * whatever layout the driver picked; the engine's display imports it with the
 * modifier). 0 / -1 */
static int layer_new(epd_ingest *g, struct layer *l)
{
    PFNEGLEXPORTDMABUFIMAGEQUERYMESAPROC eq = (void *)eglGetProcAddress("eglExportDMABUFImageQueryMESA");
    PFNEGLEXPORTDMABUFIMAGEMESAPROC ex = (void *)eglGetProcAddress("eglExportDMABUFImageMESA");
    if (!eq || !ex) return -1;
    g->glActiveTexture(GL_TEXTURE4);
    l->tex = rgba_tex(g);
    if (!(l->fbo = fbo_on(g, l->tex))) return -1;
    const EGLint at[] = { EGL_GL_TEXTURE_LEVEL_KHR, 0, EGL_NONE };
    l->img = g->eglCreateImageKHR(g->dpy, g->ctx, EGL_GL_TEXTURE_2D_KHR, (EGLClientBuffer)(uintptr_t)l->tex, at);
    if (l->img == EGL_NO_IMAGE_KHR) { l->img = NULL; return -1; }
    int fourcc = 0, np = 0;
    EGLuint64KHR mod = 0;
    if (!eq(g->dpy, l->img, &fourcc, &np, &mod) || np < 1 || np > 4) return -1;
    EGLint st[4] = { 0 }, of[4] = { 0 };
    int fd[4] = { -1, -1, -1, -1 };
    if (!ex(g->dpy, l->img, fd, st, of)) return -1;
    l->b = (epd_gt_buf){ .w = g->tw, .h = g->ph, .fourcc = (uint32_t)fourcc, .modifier = mod, .planes = np };
    for (int k = 0; k < np; k++) { l->b.fd[k] = fd[k]; l->b.pitch[k] = st[k]; l->b.offset[k] = of[k]; }
    return 0;
}

static int pv_buf(void *ctx, int gt, epd_gt_buf *b)
{
    epd_ingest *g = ctx;
    if (gt < 1 || gt > g->nlay) return -1;
    *b = g->lay[gt - 1].b;
    return 0;
}

static int pv_fence(void *ctx, int gt)
{
    epd_ingest *g = ctx;
    if (gt < 1 || gt > g->nlay) return -1;
    pthread_mutex_lock(&g->lmu);
    int f = g->lay[gt - 1].fence >= 0 ? dup(g->lay[gt - 1].fence) : -1;
    pthread_mutex_unlock(&g->lmu);
    return f;
}

static void pv_release(void *ctx, int gt)
{
    epd_ingest *g = ctx;
    if (gt < 1 || gt > g->nlay) return;
    pthread_mutex_lock(&g->lmu);
    g->lay[gt - 1].used = 0;
    pthread_mutex_unlock(&g->lmu);
}

const epd_gt_provider *epd_ingest_provider(epd_ingest *g) { return g->nlay ? &g->pv : NULL; }

static void restore(epd_ingest *g)                  /* the frame path's state */
{
    g->glUseProgram(g->prog);
    g->glBindFramebuffer(GL_FRAMEBUFFER, g->fbo);
    g->glViewport(0, 0, g->tw, g->ph);
}

int epd_ingest_mirror(epd_ingest *g, const uint8_t *shadow, int layers)
{
    if (!current(g)) return -1;
    if (!g->pcommit) {                              /* once */
        if (!(g->pcommit = link_fs(g, fs_commit)) || !(g->pmake = link_fs(g, fs_make))) return -1;
        g->glActiveTexture(GL_TEXTURE5);            /* (stays bound there) */
        g->glGenTextures(1, &g->dtex);
        g->glBindTexture(GL_TEXTURE_2D, g->dtex);
        g->glTexStorage2D(GL_TEXTURE_2D, 1, GL_R8UI, 64, 64);
        g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        g->dpat_up = -1;
        g->glUseProgram(g->pcommit);
        g->glUniform1i(g->glGetUniformLocation(g->pcommit, "G"), 2);
        g->glUniform1i(g->glGetUniformLocation(g->pcommit, "S"), 3);
        g->glUniform1i(g->glGetUniformLocation(g->pcommit, "T"), 5);
        g->uc_box = g->glGetUniformLocation(g->pcommit, "box");
        g->uc_how = g->glGetUniformLocation(g->pcommit, "how");
        g->uc_all = g->glGetUniformLocation(g->pcommit, "all");
        g->glUseProgram(g->pmake);
        g->glUniform1i(g->glGetUniformLocation(g->pmake, "S"), 3);
        g->glActiveTexture(GL_TEXTURE3);
        for (int i = 1; i >= 0; i--) {              /* (gsh[0] stays bound on unit 3) */
            g->gsh[i] = rgba_tex(g);
            if (!(g->fgsh[i] = fbo_on(g, g->gsh[i]))) { restore(g); fprintf(stderr, "epd-ingest: mirror framebuffer incomplete\n"); return -1; }
        }
        g->eglCreateSyncKHR = (void *)eglGetProcAddress("eglCreateSyncKHR");
        g->eglDestroySyncKHR = (void *)eglGetProcAddress("eglDestroySyncKHR");
        g->eglDupNativeFenceFDANDROID = (void *)eglGetProcAddress("eglDupNativeFenceFDANDROID");
        if (!g->eglCreateSyncKHR || !g->eglDestroySyncKHR || !g->eglDupNativeFenceFDANDROID) g->nfence = 0;
        g->pv = (epd_gt_provider){ 0, pv_buf, pv_fence, pv_release, g };
    }
    if (layers > EPD_GT_MAX) layers = EPD_GT_MAX;
    while (g->nlay < layers) {
        struct layer *l = &g->lay[g->nlay];
        if (layer_new(g, l)) {
            fprintf(stderr, "epd-ingest: no exportable target layer (%d made)\n", g->nlay);
            if (l->img) g->eglDestroyImageKHR(g->dpy, l->img);
            *l = (struct layer){ .fence = -1 };
            break;
        }
        g->nlay++;
    }
    g->pv.n = g->nlay;
    restore(g);
    while (g->glGetError() != GL_NO_ERROR) ;
    if (!g->nlay) return -1;
    g->mir = 1;
    epd_ingest_mirror_upload(g, (epd_ingest_rect){ 0, 0, g->pw, g->ph }, shadow);
    if (g->glGetError() != GL_NO_ERROR) { g->mir = 0; fprintf(stderr, "epd-ingest: GL error in the mirror setup\n"); return -1; }
    return 0;
}

/* pr clipped to the panel and widened to whole texels: texels [*t0, *t1) x rows [*r0, *r1); 0 = empty */
static int ptexels(const epd_ingest *g, epd_ingest_rect pr, int *t0, int *t1, int *r0, int *r1)
{
    int x0 = pr.x < 0 ? 0 : pr.x, y0 = pr.y < 0 ? 0 : pr.y;
    int x1 = pr.x + pr.w > g->pw ? g->pw : pr.x + pr.w, y1 = pr.y + pr.h > g->ph ? g->ph : pr.y + pr.h;
    if (x1 <= x0 || y1 <= y0) return 0;
    *t0 = x0 / 4; *t1 = (x1 + 3) / 4; *r0 = y0; *r1 = y1;
    return 1;
}

void epd_ingest_mirror_upload(epd_ingest *g, epd_ingest_rect pr, const uint8_t *shadow)
{
    int t0, t1, r0, r1;
    if (!g->mir || !ptexels(g, pr, &t0, &t1, &r0, &r1) || !current(g)) return;
    g->glActiveTexture(GL_TEXTURE3);
    g->glBindTexture(GL_TEXTURE_2D, g->gsh[0]);
    g->glPixelStorei(GL_UNPACK_ROW_LENGTH, g->tw);
    g->glTexSubImage2D(GL_TEXTURE_2D, 0, t0, r0, t1 - t0, r1 - r0, GL_RGBA, GL_UNSIGNED_BYTE, shadow + (size_t)r0 * g->pw + (size_t)t0 * 4);
    g->glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
}

void epd_ingest_stale(epd_ingest *g, epd_ingest_rect r)
{
    int x0 = r.x < 0 ? 0 : r.x, y0 = r.y < 0 ? 0 : r.y;      /* buffer rect -> panel rows x0..x1, cols pw - y1 .. pw - y0 */
    int x1 = r.x + r.w > g->ph ? g->ph : r.x + r.w, y1 = r.y + r.h > g->pw ? g->pw : r.y + r.h;
    if (x1 <= x0 || y1 <= y0) return;
    int c0 = g->pw - y1, c1 = g->pw - y0;
    for (int ty = x0 / TILE; ty <= (x1 - 1) / TILE; ty++)
        for (int tx = c0 / TILE; tx <= (c1 - 1) / TILE; tx++) g->valid[ty * g->ttw + tx] = 0;
    for (int row = x0; row < x1; row++) memset(g->drawn + (size_t)row * g->tw + c0 / 4, 0, (size_t)((c1 + 3) / 4 - c0 / 4));
}

void epd_ingest_dither(epd_ingest *g, int pattern) { g->dpat = pattern; }

/* dtex := the table of g->dpat (glassine-epdd-desktop builds its own from the same header) */
static void dither_up(epd_ingest *g)
{
    uint8_t t[64][64];
    epd_dither_table(t, g->dpat);
    if (g->dpat_up >= 0) g->glFinish();         /* (A509: an upload into a texture still in use can vanish) */
    g->glActiveTexture(GL_TEXTURE5);
    g->glBindTexture(GL_TEXTURE_2D, g->dtex);
    g->glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 64, 64, GL_RED_INTEGER, GL_UNSIGNED_BYTE, t);
    g->dpat_up = g->dpat;
}

int epd_ingest_mirror_commit(epd_ingest *g, epd_ingest_rect pr, int all, int how)
{
    int t0, t1, r0, r1;
    if (!g->mir) return -1;
    if (!ptexels(g, pr, &t0, &t1, &r0, &r1)) return 0;
    int c0 = pr.x < 0 ? 0 : pr.x, c1 = pr.x + pr.w > g->pw ? g->pw : pr.x + pr.w;
    for (int ty = r0 / TILE; ty <= (r1 - 1) / TILE; ty++)
        for (int tx = c0 / TILE; tx <= (c1 - 1) / TILE; tx++) if (!g->valid[ty * g->ttw + tx]) return -1;
    if (!current(g)) return -1;
    if (g->dpat_up != g->dpat) dither_up(g);
    g->glUseProgram(g->pcommit);
    g->glUniform4i(g->uc_box, c0, c1, r0, r1);
    g->glUniform1i(g->uc_how, how);
    g->glUniform1i(g->uc_all, all != 0);
    g->glBindFramebuffer(GL_FRAMEBUFFER, g->fgsh[1]);
    g->glViewport(0, 0, g->tw, g->ph);
    g->glScissor(t0, r0, t1 - t0, r1 - r0);
    g->glDrawArrays(GL_TRIANGLES, 0, 3);
    g->glBindFramebuffer(GL_READ_FRAMEBUFFER, g->fgsh[1]);
    g->glBindFramebuffer(GL_DRAW_FRAMEBUFFER, g->fgsh[0]);
    g->glBlitFramebuffer(t0, r0, t1, r1, t0, r0, t1, r1, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    restore(g);
    return 0;
}

int epd_ingest_target(epd_ingest *g, epd_ingest_rect pr)
{
    int t0, t1, r0, r1;
    if (!g->mir || !ptexels(g, pr, &t0, &t1, &r0, &r1)) return 0;
    pthread_mutex_lock(&g->lmu);
    int k;
    for (k = 0; k < g->nlay && g->lay[k].used; k++) ;
    if (k < g->nlay) g->lay[k].used = 1;
    pthread_mutex_unlock(&g->lmu);
    if (k == g->nlay) return 0;
    if (!current(g)) { pv_release(g, k + 1); return 0; }
    struct layer *l = &g->lay[k];
    g->glUseProgram(g->pmake);
    g->glBindFramebuffer(GL_FRAMEBUFFER, l->fbo);
    g->glViewport(0, 0, g->tw, g->ph);
    g->glScissor(t0, r0, t1 - t0, r1 - r0);
    g->glDrawArrays(GL_TRIANGLES, 0, 3);
    restore(g);
    int fd = -1;
    if (g->nfence) {                                /* the engine waits for it on its GPU */
        const EGLint at[] = { EGL_SYNC_NATIVE_FENCE_FD_ANDROID, EGL_NO_NATIVE_FENCE_FD_ANDROID, EGL_NONE };
        EGLSyncKHR s = g->eglCreateSyncKHR(g->dpy, EGL_SYNC_NATIVE_FENCE_ANDROID, at);
        if (s != EGL_NO_SYNC_KHR) {
            g->glFlush();                           /* (the fence fd exists once flushed) */
            fd = g->eglDupNativeFenceFDANDROID(g->dpy, s);
            g->eglDestroySyncKHR(g->dpy, s);
        }
    }
    if (fd < 0) {                                   /* no native fence: done before it is handed out */
        GLsync f = g->glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
        GLenum w = f ? g->glClientWaitSync(f, GL_SYNC_FLUSH_COMMANDS_BIT, 500 * 1000000ull) : GL_WAIT_FAILED;
        if (w == GL_TIMEOUT_EXPIRED || w == GL_WAIT_FAILED) {
            fprintf(stderr, "epd-ingest: a target not drawn in 500 ms: ingest abandoned\n");
            ing_abandon(g, w == GL_TIMEOUT_EXPIRED ? f : NULL);
            pv_release(g, k + 1);
            return 0;
        }
        g->glDeleteSync(f);
    }
    pthread_mutex_lock(&g->lmu);
    if (l->fence >= 0) close(l->fence);
    l->fence = fd;
    pthread_mutex_unlock(&g->lmu);
    if (g->glGetError() != GL_NO_ERROR) { fprintf(stderr, "epd-ingest: GL error drawing a target\n"); pv_release(g, k + 1); return 0; }
    return k + 1;
}

/* the GPU done with what was queued, at most timeout_ms; 0 / -1 (then dead) */
static int ing_finish(epd_ingest *g, int timeout_ms)
{
    GLsync f = g->glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    GLenum w = f ? g->glClientWaitSync(f, GL_SYNC_FLUSH_COMMANDS_BIT, (GLuint64)timeout_ms * 1000000) : GL_WAIT_FAILED;
    if (w == GL_TIMEOUT_EXPIRED || w == GL_WAIT_FAILED) { ing_abandon(g, w == GL_TIMEOUT_EXPIRED ? f : NULL); return -1; }
    g->glDeleteSync(f);
    return 0;
}

int epd_ingest_mirror_read(epd_ingest *g, uint8_t *out)
{
    if (!g->mir || !current(g) || ing_finish(g, 2000)) return -1;
    g->glBindFramebuffer(GL_FRAMEBUFFER, g->fgsh[0]);
    g->glReadPixels(0, 0, g->tw, g->ph, GL_RGBA, GL_UNSIGNED_BYTE, out);       /* (PACK_ROW_LENGTH = tw) */
    restore(g);
    return g->glGetError() == GL_NO_ERROR ? 0 : -1;
}

int epd_ingest_target_read(epd_ingest *g, int gt, epd_ingest_rect pr, uint8_t *out, int timeout_ms)
{
    int t0, t1, r0, r1;
    if (gt < 1 || gt > g->nlay || !ptexels(g, pr, &t0, &t1, &r0, &r1) || !current(g) || ing_finish(g, timeout_ms)) return -1;
    uint8_t *row = malloc((size_t)(t1 - t0) * 4);
    if (!row) return -1;
    int x0 = pr.x < 0 ? 0 : pr.x, w = (pr.x + pr.w > g->pw ? g->pw : pr.x + pr.w) - x0;
    g->glBindFramebuffer(GL_FRAMEBUFFER, g->lay[gt - 1].fbo);
    g->glPixelStorei(GL_PACK_ROW_LENGTH, 0);
    for (int y = r0; y < r1; y++) {
        g->glReadPixels(t0, y, t1 - t0, 1, GL_RGBA, GL_UNSIGNED_BYTE, row);
        memcpy(out + (size_t)(y - r0) * w, row + (x0 - t0 * 4), (size_t)w);
    }
    g->glPixelStorei(GL_PACK_ROW_LENGTH, g->tw);
    free(row);
    restore(g);
    return g->glGetError() == GL_NO_ERROR ? 0 : -1;
}
