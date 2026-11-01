/* epd-gpu — see epd-gpu.h */
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/memfd.h>
#include <linux/udmabuf.h>
#ifdef __aarch64__
#include <arm_neon.h>
#endif
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl31.h>
#include <GLES2/gl2ext.h>
#include "epd-gpu.h"
#include "epd-bp4.h"
#include "epd-bluenoise.h"
#include "epd-gldebug.h"

#ifndef GL_MAP_PERSISTENT_BIT_EXT               /* (GL_EXT_buffer_storage, older headers) */
#define GL_MAP_PERSISTENT_BIT_EXT 0x0040
#define GL_MAP_COHERENT_BIT_EXT 0x0080
#define GL_DYNAMIC_STORAGE_BIT_EXT 0x0100
typedef void (*PFNGLBUFFERSTORAGEEXTPROC)(GLenum target, GLsizeiptr size, const void *data, GLbitfield flags);
#endif

/* GLES through eglGetProcAddress, not linked (as epd-comp.c: libGLESv2 need
 * not be installed) */
#define GL_FNS \
    F(PFNGLACTIVETEXTUREPROC, glActiveTexture) \
    F(PFNGLATTACHSHADERPROC, glAttachShader) \
    F(PFNGLBINDFRAMEBUFFERPROC, glBindFramebuffer) \
    F(PFNGLBLITFRAMEBUFFERPROC, glBlitFramebuffer) \
    F(PFNGLDRAWBUFFERSPROC, glDrawBuffers) \
    F(PFNGLREADBUFFERPROC, glReadBuffer) \
    F(PFNGLUNIFORM1UIPROC, glUniform1ui) \
    F(PFNGLUNIFORM1UIVPROC, glUniform1uiv) \
    F(PFNGLUNIFORM2UIPROC, glUniform2ui) \
    F(PFNGLUNIFORM4IVPROC, glUniform4iv) \
    F(PFNGLBINDTEXTUREPROC, glBindTexture) \
    F(PFNGLBINDVERTEXARRAYPROC, glBindVertexArray) \
    F(PFNGLCHECKFRAMEBUFFERSTATUSPROC, glCheckFramebufferStatus) \
    F(PFNGLCOMPILESHADERPROC, glCompileShader) \
    F(PFNGLCREATEPROGRAMPROC, glCreateProgram) \
    F(PFNGLCREATESHADERPROC, glCreateShader) \
    F(PFNGLDISABLEPROC, glDisable) \
    F(PFNGLDRAWARRAYSPROC, glDrawArrays) \
    F(PFNGLENABLEPROC, glEnable) \
    F(PFNGLCLIENTWAITSYNCPROC, glClientWaitSync) \
    F(PFNGLDELETESYNCPROC, glDeleteSync) \
    F(PFNGLFENCESYNCPROC, glFenceSync) \
    F(PFNGLFINISHPROC, glFinish) \
    F(PFNGLFRAMEBUFFERTEXTURE2DPROC, glFramebufferTexture2D) \
    F(PFNGLGENFRAMEBUFFERSPROC, glGenFramebuffers) \
    F(PFNGLGENTEXTURESPROC, glGenTextures) \
    F(PFNGLDELETETEXTURESPROC, glDeleteTextures) \
    F(PFNGLDELETEFRAMEBUFFERSPROC, glDeleteFramebuffers) \
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
    F(PFNGLUNIFORM4IPROC, glUniform4i) \
    F(PFNGLUNIFORM3IPROC, glUniform3i) \
    F(PFNGLUSEPROGRAMPROC, glUseProgram) \
    F(PFNGLVIEWPORTPROC, glViewport)
/* GLES 3.1, for the gpe only (loaded by epd_gpu_gpe_open: drive frames work on 3.0) */
#define GPE_FNS \
    F(PFNGLDISPATCHCOMPUTEPROC, glDispatchCompute) \
    F(PFNGLMEMORYBARRIERPROC, glMemoryBarrier) \
    F(PFNGLBINDIMAGETEXTUREPROC, glBindImageTexture) \
    F(PFNGLBINDBUFFERBASEPROC, glBindBufferBase) \
    F(PFNGLBINDBUFFERRANGEPROC, glBindBufferRange) \
    F(PFNGLFLUSHPROC, glFlush) \
    F(PFNGLBINDBUFFERPROC, glBindBuffer) \
    F(PFNGLBUFFERDATAPROC, glBufferData) \
    F(PFNGLGENBUFFERSPROC, glGenBuffers) \
    F(PFNGLMAPBUFFERRANGEPROC, glMapBufferRange) \
    F(PFNGLUNMAPBUFFERPROC, glUnmapBuffer) \
    F(PFNGLGETINTEGERVPROC, glGetIntegerv) \
    F(PFNGLBUFFERSUBDATAPROC, glBufferSubData)

struct epd_gpu {
    const epd_engine *eng;              /* (epd_gpu_open's: the stagger's lanes import its D/T) */
    EGLDisplay dpy;
    EGLContext ctx;
    GLsync lost_f;                      /* the fence a timeout left, never deleted (epd_gpu_revive) */
#define F(t, n) t n;
    GL_FNS
    GPE_FNS
#undef F
    PFNGLEGLIMAGETARGETTEXTURE2DOESPROC glEGLImageTargetTexture2DOES;
    PFNEGLCREATEIMAGEKHRPROC eglCreateImageKHR;
    PFNEGLDESTROYIMAGEKHRPROC eglDestroyImageKHR;
    GLuint prog, tpl, st, lut, vao;
    GLint u_pw, u_box;
    uint8_t lutrows[EPD_MAX_UPD * 1024];
    int rows_hi;                        /* LUT rows uploaded so far (idle ones zero) */
    /* per slot: where a finished run left stale codes (slot, old level) that
     * no pack has replaced since (w = 0: none), see epd_gpu_finishing */
    epd_rect stale[EPD_MAX_UPD];
    uint16_t *stage;                    /* state rect, one code per pixel (see fs) */
    /* drive_state=heap : the codes in shared memory W, one
     * uint16 per pixel, rows hstride codes apart, sampled by the fs (HEAPW:
     * hfmt 16 = ABGR16161616 EPD_W/4 x EPD_H, 8 = ABGR8888 EPD_W/2) through
     * Wt; hdirty: written since its last clean (one END|WRITE per frame, in
     * epd_gpu_render). hmap NULL: st */
    uint16_t *hmap;
    int hfd, hstride, hfmt, hdirty, hfin;    /* (hfin: GLASSINE_DRIVE_STATE_FIN=idle, epd_gpu_finishing) */
    /* (claim_state packed): hmap is the engine's W
     * (e->pk), imported from its dma-buf (hfd: a dup); the engine writes it,
     * epd_gpu_state / _finishing have nothing to do, the clean is the
     * engine's (epd_pk_w_sync) */
    int hext;
    size_t hsize;
    EGLImageKHR himg;
    GLuint Wt;
    char hwhat[64];
    uint32_t *flat;                     /* epd_gpu_read of a flat target */
    struct target {
        GLuint fbo, tex;
        EGLImageKHR img;
        int pw, w, h;                   /* flat view: words per wire line (0 = the 2D
                                         * texture EPD_LINE x EPD_LINES), texels w x h */
        int prev[4];                    /* words [x0, x1) x lines [y0, y1) drawn into it last */
        GLuint itex;                    /* the same memory as an immutable texture (image
                                         * stores: the staggered steps' direct frames), 0 none */
        int drawn;                      /* drawn whole once: the template is in it */
    } t[EPD_GPU_TARGETS];
    int nt;
    /* the engine's per-pixel state on the GPU (epd_gpu_gpe_open),
     * updated in place by compute passes: S R32UI = level | target << 8 |
     * busy << 16 | stale << 23 | op << 24, D R32I = the DC record; part: the
     * workgroups' partials + the reduced result (GPE_PART) */
    struct {
        int on;
        GLuint S, D, T, N, B, part, resb, fbs, fbd;
        int rpt, th, gatom, texread, wy;        /* the shape (GLASSINE_GPE_RPT, _GATOM, _TEXREAD, _WY) */
        GLuint pclaim, pred, pfin, pop, pquery;
        GLuint sprog;                           /* the drive shader on S (GPE) */
        GLint su_pw, su_box;
        int drive;                              /* epd_gpu_gpe_drive */
        const void *best;                       /* the best-cut table uploaded */
        uint8_t *stage;                         /* uploads: EPD_W * EPD_H * 4 */
        int8_t nets[2 + EPD_LUT_CUTS][1024];
        uint32_t res[64];                       /* the last reduced result */
        /* resb: two results, rstride bytes apart (region 1 a claim's, which
         * may wait for collect; 0 queries and probes); reg = the one the
         * passes write now. resm: resb mapped for good (GL_EXT_buffer_storage,
         * GLASSINE_GPE_PMAP=0 off), NULL = mapped per read */
        int reg, rstride;
        const uint32_t *resm;
        /* a claim submitted (epd_gpu_gpe_submit), its result not collected:
         * the fence after its passes, its matched slots */
        GLsync cf;
        int cpend;
        uint8_t cmown[EPD_LUT_CUTS];
        /* stage 5: update targets on another display (epd_gpu_gpe_provider) */
        const epd_gt_provider *pv;
        struct { GLuint tex, fbo; EGLImageKHR img; } lay[EPD_GT_MAX];
        PFNEGLCREATESYNCKHRPROC create_sync;    /* NULL: no GPU-side wait (poll the fence) */
        PFNEGLWAITSYNCKHRPROC wait_sync;
        PFNEGLDESTROYSYNCKHRPROC destroy_sync;
    } x;
    /* staggered cut steps (epd_gpu_stag_*): hot state H (RGBA32UI EPD_W/8 x
     * EPD_H, 8 x 16 bit), cold C (R32UI 8 planes of CPLANE x EPD_H, one per
     * octant: cplane), targets T (RG32UI EPD_W/8 x EPD_H, 8 x 8 bit), per G and
     * frame the group's pixels and the dither thresholds (GM RG32UI 8 x 64 rows
     * per (G, frame)), the octant of a word's pixels (OC R32UI 8 x 64), the
     * frame's data words (WD R32UI EPD_W/8 x EPD_H, the template pass only);
     * one program per mode (pcs) */
    struct {
        int on, wy, swy;                        /* wy: workgroup rows (GLASSINE_STAG_WY, 1 2 4 8); swy: the stripe engine's (GLASSINE_STAG_SWY) */
        int coldlin;                            /* (timing/test only: GLASSINE_STAG_DEFS has COLDLIN: C linear, EPD_W x EPD_H) */
        int heapstate, hfd[2], tpitch;          /* (timing/test only, R4: GLASSINE_STAG_DEFS has HEAPSTATE (+ COLDLIN):
                                                 * C and T in cached heap dma-bufs, D ABGR8888 and T ABGR16161616; stag_heap) */
        uint8_t *tmap;
        size_t tlen;
        /* lanes (stagger_state=lanes, ; epd_stagger_state at the
         * open, claim_state packed with D/T dma-bufs): C = the engine's D
         * (ABGR8888 EPD_W x (EPD_H + lr), pitch EPD_PK_S * 4), T = its T folded
         * (ABGR16161616 960 x EPD_H / 4: 4 rows of 240 texels a texel row,
         * pitch EPD_PK_S * 4), A the acks (ABGR8888 GW x (EPD_H + lr), pitch
         * apitch, mapped: amap); heapstate and coldlin set with it. ldfd, ltfd:
         * dups of the engine's fds */
        int lanes, ldfd, ltfd, afd, apitch;
        uint8_t *amap;
        size_t alen;
        GLuint A;
        EGLImageKHR limg[3];
        GLuint H, C, T, WD, OC, GM, fbh, fbc, pcs[128 * 512], pemit;
        GLuint CR;                              /* the cells' table (RG32UI 64 x 64: epd_stag_cells), of shape crk (ord << 16 | w << 8 | h) */
        int crk;
        /* the tile skip (epd_gpu_stag_tskip): AT (R32UI, a texel a workgroup
         * tile of 32 words x wy rows) = the frame id a tile last had work
         * (a state change or a phase running), read through a texture
         * (ATS), written as an image by the pass and by the mark pass
         * (pmark: the rects whose targets / state changed since the last
         * frame, rbx; the params' signature changed: mall, the whole panel) */
        GLuint AT, pmark;
        unsigned dgen;                  /* (the dither tables built from epd_stag_dqt of this generation: stag_dtabs) */
        GLint m_box, m_fid;
        uint32_t fid, psig, tbig;       /* (tbig: no tile skip before this frame id: a big change lately) */
        int mall;                       /* (the params changed: every tile stamped; not a change of content) */
        struct { int x0, x1, y0, y1; uint32_t id; } rbx[16];
        int nrbx;
        /* the cold state linear (R32UI EPD_W x EPD_H): LCU the loads' (bit 12
         * = take it; only ever uploaded), LC the exit's readback (only image
         * stores: the A509 lost uploads into a texture its last store pass
         * wrote); their programs: load (+ clear), store */
        GLuint LCU, LC, fbl, fblu, pload, pstore;   /* (fblu: on LCU, stagger=check's probe) */
        GLint l_box, l_rcp, l_clr, s_box;
        /* LEAD (epd_gpu_stag_lead): lr scratch rows below H, C, WD, AT, LC
         * (0: none, LEAD off at open), the load / store / mark programs with
         * LEAD defined (lp: load, store, mark) and their uniforms */
        int lr, lwarn;
        GLuint lp[3];
        GLint ll_box, ll_rcp, ll_clr, ls_box, lm_box, lm_fid;
        uint32_t *cbuf;                         /* staging (loads) */
        GLenum lerr;                            /* the last load upload's GL error (stagger=check) */
        GLint e_pw, e_box;
        int direct;                             /* frames written straight into the target (GLASSINE_STAG_DIRECT, default 1) */
        /* timer queries (epd_gpu_stag_timer): the pass, the template pass */
        GLuint tq[2];
        void (*gen_q)(GLsizei, GLuint *), (*begin_q)(GLenum, GLuint), (*end_q)(GLenum);
        void (*get_q)(GLuint, GLenum, GLuint64 *);
        /* stagger_ctl 4 (epd-bp4.h; b4_open): S the planes, Q the requests, GM
         * the group masks (layout mkey), fbS S's readback; programs park, pack
         * (pp), visit (pv), drive (pd[direct]); on 0 not tried, 1 ready, -1
         * not available (ctl 3 runs); park: S parked at the next frame */
        struct {
            int on, wx, wy, dlead, park, mkey, kid, uber;
            GLuint S, Q, GM, fbS, ppk, pp, pv, pl, pd[2], pu[2], tq[4];
            GLint u_box, u_vr, u_vpar, u_l[2], u_d[2][6], u_u[2][3];   /* (pu: R, pass, g0) */
        } b4;
        /* px (GLASSINE_STAG_PX=1): one pixel per fragment, the state
         * linear and in place: PH R16UI (epd_stag.hot), PC R32UI (.cold), EPD_W x
         * EPD_H, both render targets of fbpx and read at the fragment's own
         * pixel; RK R32UI 64 x 64: the blue-noise rank | 2 dq << 16; the frame's
         * words packed into WD (fbwd), then the template pass. ppx: the step
         * programs (key without direct, nj, tile skip) */
        int px;
        GLuint PH, PC, RK, fbpx, fbwd, pxld, pxpk, ppx[64 * 512];
        /* pp (GLASSINE_STAG_PXPP): ping-pong, no feedback loop (a texture read while
         * it is the target is undefined; on the A509 beside another process's
         * GPU work an 8 x 8 block came out wrong): two pairs, PH/PC
         * (fbpx) and PH2/PC2 (fbpx2); a step reads one, writes the other, every
         * drawn pixel (a skip copies). pm: bit g = group g's pixels live in pair
         * 2 (stripes: per group; no stripes: 0 or all ones), for G pmG, stripe
         * pmW; the pack reads per word the live one; a read first resolves (the
         * pair-2 stripes copied to pair 1: pm 0). Load and enter write both
         * pairs (fbld, 4 targets) */
        int pp, pmG, pmW;
        uint32_t pm;
        GLuint PH2, PC2, fbpx2, fbld, pxcp;
        GLint xk_pm, xk_stw, xk_gG;
        GLint xl_box, xl_rcp, xl_clr, xk_f6;
    } sg;
};

#define GW (EPD_W / 8)                  /* state texels per row: 8 pixels each */
#define DATA_LINE0 (EPD_DATA0 / EPD_LINE)
#define DATA_COL0 (EPD_DATA0 % EPD_LINE)
#define FLAT_W 1024                     /* texels per row of a flat view (4096 bytes) */
#define IDLE 1023                       /* code of a pixel no update drives: LUT entry
                                         * target 31 / level 31 (levels are even, 0..30),
                                         * zero in every row */

#ifndef EGL_PLATFORM_SURFACELESS_MESA
#define EGL_PLATFORM_SURFACELESS_MESA 0x31DD
#endif
#ifndef EGL_NO_CONFIG_KHR
#define EGL_NO_CONFIG_KHR ((EGLConfig)0)
#endif

/* one fragment per wire word. On the own texture gl_FragCoord = (word, line);
 * a scanout buffer is seen flat, FLAT_W words per texel row whatever its pitch
 * (freedreno renders only into 256-byte aligned rows and the display fails on
 * padded ones): texel index = line * pw + word, and fragments outside the
 * box (words x0..x1, lines y0..y1) or in a row's padding write nothing. Data words: panel
 * row y on line DATA_LINE0 + EPD_H - 1 - y (the row mirror), panel
 * columns 8 * (word - DATA_COL0) .. + 7, the first in the top bits.
 * Per pixel state: a 16-bit code, slot << 10 | target << 5 | level (IDLE for
 * a pixel no update drives), two per 32-bit component, the 8 pixels of a word
 * in one RGBA32UI texel: one fetch per word, then one LUT fetch per pixel. */
#define STR_(x) #x
#define STR(x) STR_(x)
static const char *vs =
    "#version 300 es\n"
    "void main() {\n"
    "  vec2 p = vec2(gl_VertexID & 1, gl_VertexID >> 1) * 4.0 - 1.0;\n"
    "  gl_Position = vec4(p, 0.0, 1.0);\n"
    "}\n";
static const char *fs =
    "#version 300 es\n"
    "precision highp float; precision highp int; precision highp usampler2D;\n"
    "uniform usampler2D tpl;   /* R32UI EPD_LINE x EPD_LINES: control template */\n"
    "#ifndef HEAPW\n"
    "uniform usampler2D st;    /* RGBA32UI EPD_W/8 x EPD_H: pixel codes */\n"
    "#else\n"
    "uniform highp sampler2D W; /* drive_state=heap: the codes in shared memory, unorm (HEAPW 16:\n"
    "                              ABGR16161616 EPD_W/4 x EPD_H, 4 px a texel; 8: ABGR8888 EPD_W/2, 2) */\n"
    "#if HEAPW == 16\n"
    "uvec4 cd(int t, int y) { return uvec4(texelFetch(W, ivec2(t, y), 0) * 65535.0 + 0.5); }\n"
    "#else\n"
    "uvec4 cd(int t, int y) {\n"
    "  uvec4 a = uvec4(texelFetch(W, ivec2(2 * t, y), 0) * 255.0 + 0.5), b = uvec4(texelFetch(W, ivec2(2 * t + 1, y), 0) * 255.0 + 0.5);\n"
    "  return uvec4(a.x | a.y << 8, a.z | a.w << 8, b.x | b.y << 8, b.z | b.w << 8);\n"
    "}\n"
    "#endif\n"
    "#endif\n"
    "uniform usampler2D lut;   /* R8UI 1024 x EPD_MAX_UPD: each run's current frame */\n"
    "uniform int pw;           /* flat view: words per wire line; 0 = 2D */\n"
    "uniform ivec4 box;        /* words [x, y) x lines [z, w) to draw */\n"
    "out vec4 o;\n"
    "uint ph(uint v) { return texelFetch(lut, ivec2(int(v & 1023u), int(v >> 10)), 0).r & 3u; }\n"
    "#ifdef STAG\n"
    "uniform usampler2D WD;    /* staggered steps: the frame's data words (EPD_W/8 x EPD_H) */\n"
    "#endif\n"
    "#ifdef GPE\n"
    "uniform usampler2D S;     /* gpe: level, target, busy, op per panel pixel */\n"
    "uint cd(int x, int y) { uint v = texelFetch(S, ivec2(x, y), 0).r, b = (v >> 16) & 127u; return b != 0u ? (b - 1u) << 10 | ((v >> 8) & 255u) << 5 | (v & 255u) : " STR(IDLE) "u; }\n"
    "#endif\n"
    "void main() {\n"
    "  ivec2 c = ivec2(gl_FragCoord.xy);\n"
    "  if (pw > 0) { int i = c.y * " STR(FLAT_W) " + c.x; c = ivec2(i % pw, i / pw); }\n"
    "  if (c.x < box.x || c.x >= box.y || c.y < box.z || c.y >= box.w) discard;\n"
    "  uint w = texelFetch(tpl, c, 0).r;\n"
    "  int y = " STR(EPD_H) " - 1 - (c.y - " STR(EPD_DATA0) " / " STR(EPD_LINE) ");\n"
    "  int gx = c.x - " STR(EPD_DATA0) " % " STR(EPD_LINE) ";\n"
    "  if (y >= 0 && y < " STR(EPD_H) " && gx >= 0 && gx < " STR(EPD_W) " / 8) {\n"
    "#if defined(STAG)\n"
    "    w |= texelFetch(WD, ivec2(gx, y), 0).r;\n"
    "#elif defined(GPE)\n"
    "    int x = gx * 8;\n"
    "    w |= ph(cd(x, y)) << 14 | ph(cd(x + 1, y)) << 12 | ph(cd(x + 2, y)) << 10 | ph(cd(x + 3, y)) << 8 |\n"
    "         ph(cd(x + 4, y)) << 6 | ph(cd(x + 5, y)) << 4 | ph(cd(x + 6, y)) << 2 | ph(cd(x + 7, y));\n"
    "#elif defined(HEAPW)\n"
    "    uvec4 a = cd(gx * 2, y), b = cd(gx * 2 + 1, y);\n"
    "    w |= ph(a.x) << 14 | ph(a.y) << 12 | ph(a.z) << 10 | ph(a.w) << 8 |\n"
    "         ph(b.x) << 6 | ph(b.y) << 4 | ph(b.z) << 2 | ph(b.w);\n"
    "#else\n"
    "    uvec4 s = texelFetch(st, ivec2(gx, y), 0);\n"
    "    w |= ph(s.x & 65535u) << 14 | ph(s.x >> 16) << 12 | ph(s.y & 65535u) << 10 | ph(s.y >> 16) << 8 |\n"
    "         ph(s.z & 65535u) << 6 | ph(s.z >> 16) << 4 | ph(s.w & 65535u) << 2 | ph(s.w >> 16);\n"
    "#endif\n"
    "  }\n"
    "  o = vec4(uvec4(w, w >> 8, w >> 16, w >> 24) & 255u) / 255.0;\n"
    "}\n";

static GLuint shader(epd_gpu *g, GLenum type, const char *src)
{
    GLuint s = g->glCreateShader(type);
    g->glShaderSource(s, 1, &src, NULL);
    g->glCompileShader(s);
    GLint ok = 0;
    g->glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        g->glGetShaderInfoLog(s, sizeof log, NULL, log);
        fprintf(stderr, "epd-gpu: shader: %s\n", log);
        return 0;
    }
    return s;
}

static GLuint tex(epd_gpu *g, GLenum fmt, int w, int h)
{
    GLuint t;
    g->glGenTextures(1, &t);
    g->glBindTexture(GL_TEXTURE_2D, t);
    g->glTexStorage2D(GL_TEXTURE_2D, 1, fmt, w, h);
    g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    return t;
}

/* an FBO on texture tx (bound on unit 3); its first frame is drawn whole */
static int add_target(epd_gpu *g, GLuint tx, EGLImageKHR img, int pw, int w, int h)
{
    if (g->nt >= EPD_GPU_TARGETS) return -1;
    struct target *t = &g->t[g->nt];
    t->tex = tx; t->img = img; t->pw = pw; t->w = w; t->h = h;
    g->glGenFramebuffers(1, &t->fbo);
    g->glBindFramebuffer(GL_FRAMEBUFFER, t->fbo);
    g->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tx, 0);
    if (g->glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        fprintf(stderr, "epd-gpu: target %d: framebuffer incomplete\n", g->nt);
        return -1;
    }
    t->prev[0] = 0; t->prev[1] = EPD_LINE; t->prev[2] = 0; t->prev[3] = EPD_LINES;
    t->itex = img ? 0 : tx;             /* (the own texture: immutable storage) */
    t->drawn = 0;
    return g->nt++;
}

int epd_gpu_target_dmabuf(epd_gpu *g, int fd, int pitch, size_t size)
{
    if (!g->glEGLImageTargetTexture2DOES || !g->eglCreateImageKHR) {
        fprintf(stderr, "epd-gpu: no EGL dma-buf import\n");
        return -1;
    }
    int pw = pitch / 4, h = (int)(((size_t)pw * EPD_LINES + FLAT_W - 1) / FLAT_W);
    if (pitch % 4 || pw < EPD_LINE || (size_t)h * FLAT_W * 4 > size) {
        fprintf(stderr, "epd-gpu: dma-buf of %zu bytes, pitch %d: no flat view of %d rows\n", size, pitch, h);
        return -1;
    }
    /* ABGR8888 = bytes R, G, B, A = the word's bits 0..7, 8..15, 16..23, 24..31;
     * the scanout buffers are XBGR8888, the same bytes */
    const EGLint a[] = {
        EGL_WIDTH, FLAT_W, EGL_HEIGHT, h,
        EGL_LINUX_DRM_FOURCC_EXT, 0x34324241 /* DRM_FORMAT_ABGR8888 */,
        EGL_DMA_BUF_PLANE0_FD_EXT, fd, EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0,
        EGL_DMA_BUF_PLANE0_PITCH_EXT, FLAT_W * 4, EGL_NONE };
    EGLImageKHR img = g->eglCreateImageKHR(g->dpy, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, NULL, a);
    if (img == EGL_NO_IMAGE_KHR) { fprintf(stderr, "epd-gpu: dma-buf import failed (0x%x)\n", eglGetError()); return -1; }
    GLuint tx;
    g->glActiveTexture(GL_TEXTURE3);
    g->glGenTextures(1, &tx);
    g->glBindTexture(GL_TEXTURE_2D, tx);
    g->glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, img);
    int i = add_target(g, tx, img, pw, FLAT_W, h);
    if (i < 0) { g->eglDestroyImageKHR(g->dpy, img); return i; }
    {                           /* image stores need immutable storage: GL_EXT_EGL_image_storage */
        typedef void (*stor_fn)(GLenum, GLeglImageOES, const GLint *);
        stor_fn ts = (stor_fn)eglGetProcAddress("glEGLImageTargetTexStorageEXT");
        const char *ex = (const char *)g->glGetString(GL_EXTENSIONS);
        if (ts && ex && strstr(ex, "GL_EXT_EGL_image_storage")) {
            GLuint it;
            g->glGenTextures(1, &it);
            g->glBindTexture(GL_TEXTURE_2D, it);
            ts(GL_TEXTURE_2D, (GLeglImageOES)img, NULL);
            if (g->glGetError() == GL_NO_ERROR) g->t[i].itex = it;
        }
    }
    return i;
}

/* drive_state=heap: a dma-buf of n bytes, the system dma-heap's (cached pages:
 * the device, root), else a udmabuf over a memfd (the host's stand-in: the
 * same import, mmap and sync ioctls, no root); -1 neither */
int epd_gpu_drive_state = -1;
int close(int fd);                      /* (not unistd.h: its link is taken here) */
int ftruncate(int fd, off_t n);
int memfd_create(const char *name, unsigned flags);
#ifndef F_ADD_SEALS
#define F_ADD_SEALS 1033
#define F_SEAL_SHRINK 0x0002
#endif
static int heap_fd(size_t n, const char **src)
{
    int h = open("/dev/dma_heap/system", O_RDONLY | O_CLOEXEC);
    if (h >= 0) {
        struct dma_heap_allocation_data al = { .len = n, .fd_flags = O_RDWR | O_CLOEXEC };
        int r = ioctl(h, DMA_HEAP_IOCTL_ALLOC, &al);
        close(h);
        if (r == 0) { *src = "dma-heap"; return (int)al.fd; }
    }
    int m = memfd_create("glassine-epd-w", MFD_ALLOW_SEALING | MFD_CLOEXEC), u = -1, fd = -1;
    if (m < 0) return -1;
    if (!ftruncate(m, (off_t)n) && !fcntl(m, F_ADD_SEALS, F_SEAL_SHRINK) && (u = open("/dev/udmabuf", O_RDWR | O_CLOEXEC)) >= 0) {
        struct udmabuf_create c = { .memfd = (unsigned)m, .flags = UDMABUF_FLAGS_CLOEXEC, .offset = 0, .size = n };
        fd = ioctl(u, UDMABUF_CREATE, &c);
        close(u);
    }
    close(m);
    *src = "udmabuf";
    return fd;
}

static void heap_sync(epd_gpu *g, unsigned flags)
{
    struct dma_buf_sync sy = { flags };
    ioctl(g->hfd, DMA_BUF_IOCTL_SYNC, &sy);
}

/* W imported for sampling on unit 1 (the fs's W, never an upload or an
 * image store: CPU-written, GPU-read only, so the A509's upload-after-store
 * loss cannot meet it); ABGR16161616 first, ABGR8888 if that does not import
 * (or GLASSINE_DRIVE_STATE_FMT=8888); the row pitch 64- then 256-byte aligned
 * (radeonsi takes only 256). 0 = done; -1: st */
int dup(int fd);
static int heap_open(epd_gpu *g, const epd_engine *e)
{
    const char *fv = getenv("GLASSINE_DRIVE_STATE_FMT"), *fi = getenv("GLASSINE_DRIVE_STATE_FIN"), *ex = eglQueryString(g->dpy, EGL_EXTENSIONS), *src = "";
    int mods = ex && strstr(ex, "EGL_EXT_image_dma_buf_import_modifiers");
    static const int aligns[] = { 64, 256 };
    /* (packed: the engine's W, its pitch EPD_PK_S codes) */
    const int ext = e->cst == 1 && e->pk && e->pk->wfd >= 0;
    if (!g->glEGLImageTargetTexture2DOES || !g->eglCreateImageKHR) return -1;
    if (e->cst == 1 && !ext) return -1;        /* (W malloc'ed: tex, st packed from W) */
    for (int f = fv && atoi(fv) == 8888 ? 8 : 16; f >= 8; f -= 8)
        for (unsigned a = 0; a < (ext ? 1 : sizeof aligns / sizeof *aligns); a++) {
            int pitch = ext ? EPD_PK_S * 2 : (EPD_W * 2 + aligns[a] - 1) / aligns[a] * aligns[a];
            size_t n = ext ? e->pk->wsize : ((size_t)pitch * EPD_H + 4095) & ~(size_t)4095;
            int fd = ext ? dup(e->pk->wfd) : heap_fd(n, &src);
            if (fd < 0) return -1;
            if (ext) src = e->pk->wsrc;
            const EGLint at[] = {
                EGL_WIDTH, f == 16 ? EPD_W / 4 : EPD_W / 2, EGL_HEIGHT, EPD_H,
                EGL_LINUX_DRM_FOURCC_EXT, f == 16 ? 0x38344241 /* DRM_FORMAT_ABGR16161616 */ : 0x34324241 /* ABGR8888 */,
                EGL_DMA_BUF_PLANE0_FD_EXT, fd, EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0,
                EGL_DMA_BUF_PLANE0_PITCH_EXT, pitch,
                mods ? EGL_DMA_BUF_PLANE0_MODIFIER_LO_EXT : EGL_NONE, 0,
                EGL_DMA_BUF_PLANE0_MODIFIER_HI_EXT, 0,             /* DRM_FORMAT_MOD_LINEAR */
                EGL_NONE };
            EGLImageKHR img = g->eglCreateImageKHR(g->dpy, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, NULL, at);
            void *m = img == EGL_NO_IMAGE_KHR ? MAP_FAILED : ext ? (void *)e->pk->W : mmap(NULL, n, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
            if (m != MAP_FAILED) {
                GLuint t;
                g->glActiveTexture(GL_TEXTURE1);
                g->glGenTextures(1, &t);
                g->glBindTexture(GL_TEXTURE_2D, t);
                /* (NEAREST: an EGLImage texture is not mipmap-complete with the default
                 * filter, and the shader would read Mesa's 1 x 1 fallback, epd-ingest.c) */
                g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
                g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
                g->glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, img);
                if (g->glGetError() == GL_NO_ERROR) {
                    g->hmap = m; g->hfd = fd; g->hsize = n; g->hstride = pitch / 2; g->hfmt = f;
                    g->himg = img; g->Wt = t; g->hfin = !ext && fi && !strcmp(fi, "idle"); g->hext = ext;
                    snprintf(g->hwhat, sizeof g->hwhat, "heap (%s, %s%s, pitch %d)", f == 16 ? "ABGR16161616" : "ABGR8888",
                             ext ? "the engine's W, " : "", src, pitch);
                    return 0;
                }
                g->glDeleteTextures(1, &t);
                if (!ext) munmap(m, n);
            }
            if (img != EGL_NO_IMAGE_KHR) g->eglDestroyImageKHR(g->dpy, img);
            close(fd);
            while (g->glGetError() != GL_NO_ERROR) ;
        }
    return -1;
}

const char *epd_gpu_drive_state_of(const epd_gpu *g) { return g->hmap ? g->hwhat : "tex"; }

epd_gpu *epd_gpu_open(const epd_engine *e)
{
    epd_gpu *g = calloc(1, sizeof *g);
    if (g) { g->hfd = -1; g->eng = e; g->sg.ldfd = g->sg.ltfd = g->sg.afd = -1; }
    if (!g || !(g->stage = malloc((size_t)EPD_W * EPD_H * 2))) goto fail;
    PFNEGLGETPLATFORMDISPLAYEXTPROC gpd = (void *)eglGetProcAddress("eglGetPlatformDisplayEXT");
    g->dpy = gpd ? gpd(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, NULL) : EGL_NO_DISPLAY;
    if (g->dpy == EGL_NO_DISPLAY || !eglInitialize(g->dpy, NULL, NULL)) { fprintf(stderr, "epd-gpu: no surfaceless EGL\n"); goto fail; }
    eglBindAPI(EGL_OPENGL_ES_API);
    /* GLASSINE_EPD_GPU_PRIO=1 or /etc/glassine/gpu-prio: a high priority context (EGL_IMG_context_priority;
     * msm: ring 0, the high-priority ring of four (0042 + linux-firmware's pfp)),
     * so the drive frames wait only for the job on the GPU, not for the apps'
     * queued ones (stagger frames 5.8 ms GPU, 17.8 ms wall behind
     * GNOME Software). Default off */
    const char *pe = getenv("GLASSINE_EPD_GPU_PRIO"), *ex = eglQueryString(g->dpy, EGL_EXTENSIONS);
    FILE *pf = fopen("/etc/glassine/gpu-prio", "r");     /* (the flag: also across reexecs) */
    if (pf) { fclose(pf); if (!pe) pe = "1"; }      /* (GLASSINE_EPD_GPU_PRIO=0 beats it: tests) */
    int prio = pe && atoi(pe) > 0 && ex && strstr(ex, "EGL_IMG_context_priority");
    if (pe && atoi(pe) > 0 && !prio) fprintf(stderr, "epd-gpu: no EGL_IMG_context_priority: normal priority\n");
    const EGLint ca[] = { EGL_CONTEXT_MAJOR_VERSION, 3, EGL_NONE };
    const EGLint cp[] = { EGL_CONTEXT_MAJOR_VERSION, 3, 0x3100 /* EGL_CONTEXT_PRIORITY_LEVEL_IMG */, 0x3101 /* HIGH */, EGL_NONE };
    g->ctx = eglCreateContext(g->dpy, EGL_NO_CONFIG_KHR, EGL_NO_CONTEXT, prio ? cp : ca);
    if (g->ctx == EGL_NO_CONTEXT || !eglMakeCurrent(g->dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, g->ctx)) {
        fprintf(stderr, "epd-gpu: no GLES 3 context\n");
        goto fail;
    }
    if (prio) {
        EGLint got = 0;
        eglQueryContext(g->dpy, g->ctx, 0x3100, &got);
        fprintf(stderr, "epd-gpu: context priority %s\n", got == 0x3101 ? "high" : got == 0x3102 ? "medium" : got == 0x3103 ? "low" : "?");
    }
    epd_gldebug_on("epd-gpu");
#define F(t, n) if (!(g->n = (t)eglGetProcAddress(#n))) { fprintf(stderr, "epd-gpu: missing %s\n", #n); goto fail; }
    GL_FNS
#undef F
    g->glEGLImageTargetTexture2DOES = (void *)eglGetProcAddress("glEGLImageTargetTexture2DOES");
    g->eglCreateImageKHR = (void *)eglGetProcAddress("eglCreateImageKHR");
    g->eglDestroyImageKHR = (void *)eglGetProcAddress("eglDestroyImageKHR");
    if (epd_gpu_drive_state < 0) {
        const char *ds = getenv("GLASSINE_DRIVE_STATE");
        epd_gpu_drive_state = ds && !strcmp(ds, "heap");
    }
    /* (claim_state packed forces heap: the engine's W imported; without
     * a dma-buf of it, tex: st packed from W) */
    if ((epd_gpu_drive_state > 0 || e->cst == 1) && heap_open(g, e))
        fprintf(stderr, "epd-gpu: drive_state=heap: no dma-buf W (import or allocation failed): the st texture\n");
    char *hsrc = NULL;
    if (g->hmap) {                              /* the fs with HEAPW defined (else fs as it is) */
        static const char ver3[] = "#version 300 es\n";
        size_t n = strlen(fs) + 32;
        if (!(hsrc = malloc(n))) goto fail;
        snprintf(hsrc, n, "%s#define HEAPW %d\n%s", ver3, g->hfmt, fs + strlen(ver3));
    }
    GLuint v = shader(g, GL_VERTEX_SHADER, vs), f = shader(g, GL_FRAGMENT_SHADER, hsrc ? hsrc : fs);
    free(hsrc);
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
        fprintf(stderr, "epd-gpu: link: %s\n", log);
        goto fail;
    }
    g->glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    g->glPixelStorei(GL_PACK_ALIGNMENT, 4);
    g->glActiveTexture(GL_TEXTURE0);
    g->tpl = tex(g, GL_R32UI, EPD_LINE, EPD_LINES);
    g->glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, EPD_LINE, EPD_LINES, GL_RED_INTEGER, GL_UNSIGNED_INT, e->tpl);
    g->glActiveTexture(GL_TEXTURE1);
    if (!g->hmap) g->st = tex(g, GL_RGBA32UI, GW, EPD_H);
    g->glActiveTexture(GL_TEXTURE2);
    g->lut = tex(g, GL_R8UI, 1024, EPD_MAX_UPD);
    g->glActiveTexture(GL_TEXTURE3);            /* targets: not on 0-2 (tex binds) */
    if (add_target(g, tex(g, GL_RGBA8, EPD_LINE, EPD_LINES), NULL, 0, EPD_LINE, EPD_LINES) != 0) goto fail;
    g->glUseProgram(g->prog);
    g->glUniform1i(g->glGetUniformLocation(g->prog, "tpl"), 0);
    g->glUniform1i(g->glGetUniformLocation(g->prog, g->hmap ? "W" : "st"), 1);
    g->glUniform1i(g->glGetUniformLocation(g->prog, "lut"), 2);
    g->u_pw = g->glGetUniformLocation(g->prog, "pw");
    g->u_box = g->glGetUniformLocation(g->prog, "box");
    g->glGenVertexArrays(1, &g->vao);
    g->glBindVertexArray(g->vao);
    g->glDisable(GL_DITHER);
    g->glEnable(GL_SCISSOR_TEST);
    epd_gpu_state(g, e, (epd_rect){ 0, 0, EPD_W, EPD_H });
    if (g->glGetError() != GL_NO_ERROR) { fprintf(stderr, "epd-gpu: GL error in setup\n"); goto fail; }
    return g;
fail:
    epd_gpu_close(g);
    return NULL;
}

void epd_gpu_close(epd_gpu *g)
{
    if (!g) return;
    if (g->dpy != EGL_NO_DISPLAY && g->dpy) {
        for (int i = 0; i < g->nt; i++) if (g->t[i].img) g->eglDestroyImageKHR(g->dpy, g->t[i].img);
        for (int i = 0; i < EPD_GT_MAX; i++) if (g->x.lay[i].img) g->eglDestroyImageKHR(g->dpy, g->x.lay[i].img);
        if (g->himg) g->eglDestroyImageKHR(g->dpy, g->himg);
        for (int i = 0; i < 3; i++) if (g->sg.limg[i]) g->eglDestroyImageKHR(g->dpy, g->sg.limg[i]);
        eglMakeCurrent(g->dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (g->ctx) eglDestroyContext(g->dpy, g->ctx);
        eglTerminate(g->dpy);
    }
    if (g->hmap && !g->hext) munmap(g->hmap, g->hsize);
    if (g->hfd >= 0) close(g->hfd);
    if (g->sg.amap) munmap(g->sg.amap, g->sg.alen);
    if (g->sg.afd >= 0) close(g->sg.afd);
    if (g->sg.ldfd >= 0) close(g->sg.ldfd);
    if (g->sg.ltfd >= 0) close(g->sg.ltfd);
    free(g->stage);
    free(g->flat);
    free(g);
}

const char *epd_gpu_renderer(epd_gpu *g)
{
    return (const char *)g->glGetString(GL_RENDERER);
}

/* one row of state codes: slot << 10 | target << 5 | level, IDLE where no
 * update drives (busy 0) — branch-free, so the compiler vectorizes it */
__attribute__((optimize("O3")))
static void state_row(uint16_t *restrict s, const uint8_t *restrict b, const uint8_t *restrict t,
                      const uint8_t *restrict l, int n)
{
    for (int x = 0; x < n; x++) {
        uint16_t v = (uint16_t)((b[x] - 1) << 10 | t[x] << 5 | l[x]);
        s[x] = b[x] ? v : (uint16_t)IDLE;
    }
}

/* r clipped to the panel and widened to whole words (8 px); 0 = empty */
static int clip_words(epd_rect *r)
{
    if (r->x < 0) { r->w += r->x; r->x = 0; }
    if (r->y < 0) { r->h += r->y; r->y = 0; }
    if (r->x + r->w > EPD_W) r->w = EPD_W - r->x;
    if (r->y + r->h > EPD_H) r->h = EPD_H - r->y;
    if (r->w <= 0 || r->h <= 0) return 0;
    int x0 = r->x & ~7, x1 = (r->x + r->w + 7) & ~7;  /* whole texels */
    r->w = x1 - x0; r->x = x0;
    return 1;
}

int epd_gpu_pack_state(uint16_t *s, const epd_engine *e, epd_rect *r)
{
    if (!clip_words(r)) return 0;
    if (!e->busy) {                     /* (packed: W holds the codes) */
        for (int y = r->y; y < r->y + r->h; y++, s += r->w)
            memcpy(s, e->pk->W + (size_t)y * EPD_PK_S + r->x, (size_t)r->w * 2);
        return 1;
    }
    for (int y = r->y; y < r->y + r->h; y++, s += r->w) {
        size_t o = (size_t)y * EPD_W + r->x;
        state_row(s, e->busy + o, e->target + o, e->level + o, r->w);
    }
    return 1;
}

static int inside(epd_rect a, epd_rect b)       /* a within b */
{
    return a.x >= b.x && a.y >= b.y && a.x + a.w <= b.x + b.w && a.y + a.h <= b.y + b.h;
}

static void bind(epd_gpu *g, int unit, GLuint t)
{
    g->glActiveTexture(GL_TEXTURE0 + unit);
    g->glBindTexture(GL_TEXTURE_2D, t);
}

double epd_gpu_prof_pack;           /* (epd_sched_prof) s in epd_gpu_state's CPU pack (state_row), epd-drm.c reads + zeroes */
void epd_gpu_state(epd_gpu *g, const epd_engine *e, epd_rect r)
{
    struct timespec p0, p1;
    if (g->hext) return;                /* (W is the engine's, written there) */
    if (epd_sched_prof) clock_gettime(CLOCK_MONOTONIC, &p0);
    if (g->hmap) {                      /* drive_state=heap: packed in place, nothing uploaded */
        if (!clip_words(&r)) return;
        /* (START|WRITE: the formal start of a CPU write, a no-op class on the system heap;
         * the caller's last frame is fenced, so no pass of ours reads W now) */
        if (!g->hdirty) { heap_sync(g, DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE); g->hdirty = 1; }
        for (int y = r.y; y < r.y + r.h; y++) {
            size_t o = (size_t)y * EPD_W + r.x;
            state_row(g->hmap + (size_t)y * g->hstride + r.x, e->busy + o, e->target + o, e->level + o, r.w);
        }
        if (epd_sched_prof) {
            clock_gettime(CLOCK_MONOTONIC, &p1);
            epd_gpu_prof_pack += (p1.tv_sec - p0.tv_sec) + (p1.tv_nsec - p0.tv_nsec) / 1e9;
        }
    } else {
        if (!epd_gpu_pack_state(g->stage, e, &r)) return;
        if (epd_sched_prof) {
            clock_gettime(CLOCK_MONOTONIC, &p1);
            epd_gpu_prof_pack += (p1.tv_sec - p0.tv_sec) + (p1.tv_nsec - p0.tv_nsec) / 1e9;
        }
        int x0 = r.x, x1 = r.x + r.w;
        bind(g, 1, g->st);
        g->glTexSubImage2D(GL_TEXTURE_2D, 0, x0 / 8, r.y, (x1 - x0) / 8, r.h, GL_RGBA_INTEGER, GL_UNSIGNED_INT, g->stage);
    }
    for (int s = 0; s < EPD_MAX_UPD; s++)       /* replaced there: no longer stale */
        if (g->stale[s].w && inside(g->stale[s], r)) g->stale[s].w = 0;
}

/* drive_state=heap: IDLE where busy == k (a compare the compiler vectorizes) */
__attribute__((optimize("O3")))
static void idle_row(uint16_t *restrict s, const uint8_t *restrict b, uint8_t k, int n)
{
    for (int x = 0; x < n; x++) s[x] = b[x] == k ? (uint16_t)IDLE : s[x];
}

/* A finished run's pixels keep their codes (its slot, the old level) instead
 * of being packed at the end (matched cuts end up to 9 runs per
 * step on different frames, each a near full-screen pack): an idle slot's LUT
 * row is all zero (epd_gpu_render), so they draw nothing. Only a slot used
 * again must not find them: its stale rect is packed when it starts again,
 * unless a pack covered it since — a scroll step's full-screen start pack
 * covers every stale rect, so in a scroll nothing extra is packed. */
void epd_gpu_finishing(epd_gpu *g, const epd_engine *e)
{
    if (g->hext) return;                /* (the engine's finish writes W's idle codes) */
    if (g->hfin) {
        /* GLASSINE_DRIVE_STATE_FIN=idle (heap only, a measurement for):
         * 's "LUT finish" writes W's idle code where the run's pixels are
         * (busy still theirs: before epd_advance), no stale rects. As a pass of
         * its own it scans every ending run's rect: 1.76 Mpx a frame on
         * epd-gpu-test's scene, the cost the stale codes avoid; fused
         * into 's finish_row it is free. Default: stale codes, as tex (their
         * repack at a slot's restart is then an in-place write, no upload) */
        for (int s = 0; s < EPD_MAX_UPD; s++) {     /* (each over its own rect: vectorized passes beat one table pass over the union) */
            const epd_run *u = &e->run[s];
            if (!u->nframes || u->frame + 1 < u->nframes) continue;
            epd_rect r = u->r;
            if (!clip_words(&r)) continue;
            if (!g->hdirty) { heap_sync(g, DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE); g->hdirty = 1; }
            for (int y = r.y; y < r.y + r.h; y++)
                idle_row(g->hmap + (size_t)y * g->hstride + r.x, e->busy + (size_t)y * EPD_W + r.x, (uint8_t)(s + 1), r.w);
        }
        return;
    }
    for (int s = 0; s < EPD_MAX_UPD; s++) {
        const epd_run *r = &e->run[s];
        if (!r->nframes || r->frame + 1 < r->nframes) continue;
        epd_rect *q = &g->stale[s];
        if (!q->w) { *q = r->r; continue; }
        int x0 = q->x < r->r.x ? q->x : r->r.x, y0 = q->y < r->r.y ? q->y : r->r.y;
        int x1 = q->x + q->w > r->r.x + r->r.w ? q->x + q->w : r->r.x + r->r.w;
        int y1 = q->y + q->h > r->r.y + r->r.h ? q->y + q->h : r->r.y + r->r.h;
        *q = (epd_rect){ x0, y0, x1 - x0, y1 - y0 };
    }
}

/* rects to pack: two that overlap become their bounding box when that is no
 * bigger than both together */
static long area(epd_rect r) { return (long)r.w * r.h; }
static int merge_rects(epd_rect *r, int n)
{
    for (int again = 1; again;) {
        again = 0;
        for (int i = 0; i < n; i++)
            for (int j = i + 1; j < n; j++) {
                int x0 = r[i].x < r[j].x ? r[i].x : r[j].x, y0 = r[i].y < r[j].y ? r[i].y : r[j].y;
                int x1 = r[i].x + r[i].w > r[j].x + r[j].w ? r[i].x + r[i].w : r[j].x + r[j].w;
                int y1 = r[i].y + r[i].h > r[j].y + r[j].h ? r[i].y + r[i].h : r[j].y + r[j].h;
                epd_rect u = { x0, y0, x1 - x0, y1 - y0 };
                if (area(u) > area(r[i]) + area(r[j])) continue;
                r[i] = u;
                r[j--] = r[--n];
                again = 1;
            }
    }
    return n;
}

int epd_gpu_starts(epd_gpu *g, const epd_engine *e, const uint32_t *was, epd_rect *out)
{
    int n = 0;
    for (int s = 0; s < EPD_MAX_UPD; s++) {
        const epd_run *r = &e->run[s];
        if (!r->nframes || r->id == was[s]) continue;
        if (g->stale[s].w && !inside(g->stale[s], r->r)) out[n++] = g->stale[s];
        g->stale[s].w = 0;              /* (inside: its own start pack replaces them) */
        out[n++] = r->r;
    }
    return merge_rects(out, n);
}

void epd_gpu_render(epd_gpu *g, const epd_engine *e, int target)
{
    struct target *tg = &g->t[target];
    int n = 0, b[4] = { EPD_LINE, 0, EPD_LINES, 0 };   /* words of the running updates */
    for (int s = 0; s < EPD_MAX_UPD; s++) {
        const epd_run *r = &e->run[s];
        if (!r->nframes) {              /* idle: zero (stale codes, epd_gpu_finishing) */
            if (s < g->rows_hi) memset(g->lutrows + (size_t)s * 1024, 0, 1024);
            continue;
        }
        memcpy(g->lutrows + (size_t)s * 1024, r->lut + (size_t)r->frame * 1024, 1024);
        memset(g->lutrows + (size_t)s * 1024 + (31 << 5), 0, 32);   /* (target 31: IDLE and the engine's FIN) */
        n = s + 1;
        int w0 = DATA_COL0 + r->r.x / 8, w1 = DATA_COL0 + (r->r.x + r->r.w + 7) / 8;
        int l0 = DATA_LINE0 + EPD_H - r->r.y - r->r.h, l1 = DATA_LINE0 + EPD_H - r->r.y;
        if (w0 < b[0]) b[0] = w0;
        if (w1 > b[1]) b[1] = w1;
        if (l0 < b[2]) b[2] = l0;
        if (l1 > b[3]) b[3] = l1;
    }
    if (g->rows_hi > n) n = g->rows_hi;   /* rows that ran before: now zero */
    g->rows_hi = n;
    if (n) {
        bind(g, 2, g->lut);     /* (its own: the stagger's units overlap 0-2, AT on 2) */
        g->glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 1024, n, GL_RED_INTEGER, GL_UNSIGNED_BYTE, g->lutrows);
    }
    /* draw what runs now and what this target got last time (its data bits go) */
    int *pv = tg->prev;
    int u[4] = { b[0] < pv[0] ? b[0] : pv[0], b[1] > pv[1] ? b[1] : pv[1],
                 b[2] < pv[2] ? b[2] : pv[2], b[3] > pv[3] ? b[3] : pv[3] };
    memcpy(pv, b, sizeof b);
    if (u[0] >= u[1] || u[2] >= u[3]) return;
    g->glBindFramebuffer(GL_FRAMEBUFFER, tg->fbo);
    g->glViewport(0, 0, tg->w, tg->h);
    if (g->x.drive) {                           /* gpe: the codes straight from the engine state */
        g->glActiveTexture(GL_TEXTURE0 + 4);    /* (GU_S) */
        g->glBindTexture(GL_TEXTURE_2D, g->x.S);
        g->glUseProgram(g->x.sprog);
        g->glUniform1i(g->x.su_pw, tg->pw);
        g->glUniform4i(g->x.su_box, u[0], u[1], u[2], u[3]);
    } else {
        /* the main shader's units bound each draw: the stagger binds its own
         * textures there (AT on 2, the template on 0), and a draw after a run
         * read them (the LUT upload hit AT, GL_INVALID_OPERATION,
         * the GPU dropped after every tile-skip run) */
        bind(g, 0, g->tpl);
        if (g->hmap) {                  /* drive_state=heap: W cleaned to memory, then sampled */
            if (g->hext) epd_pk_w_sync(e);      /* (the engine's W) */
            else if (g->hdirty) { heap_sync(g, DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE); g->hdirty = 0; }
            bind(g, 1, g->Wt);
        } else
            bind(g, 1, g->st);
        bind(g, 2, g->lut);
        g->glUseProgram(g->prog);
        g->glUniform1i(g->u_pw, tg->pw);
        g->glUniform4i(g->u_box, u[0], u[1], u[2], u[3]);
    }
    if (tg->pw) {               /* flat: the texel rows that hold the box's lines */
        int r0 = (u[2] * tg->pw + u[0]) / FLAT_W, r1 = ((u[3] - 1) * tg->pw + u[1] + FLAT_W - 1) / FLAT_W;
        g->glScissor(0, r0, FLAT_W, r1 - r0);
    } else
        g->glScissor(u[0], u[2], u[1] - u[0], u[3] - u[2]);
    g->glDrawArrays(GL_TRIANGLES, 0, 3);
    tg->drawn = 1;
}

int epd_gpu_finish(epd_gpu *g, int timeout_ms)
{
    GLsync f = g->glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    if (!f) { g->glFinish(); return 0; }
    GLenum r = g->glClientWaitSync(f, GL_SYNC_FLUSH_COMMANDS_BIT, (GLuint64)timeout_ms * 1000000);
    if (r == GL_TIMEOUT_EXPIRED) {                  /* f stays: deleting it may wait too */
        if (!g->lost_f) g->lost_f = f;              /* (epd_gpu_revive looks at it) */
        return -1;
    }
    g->glDeleteSync(f);
    return r == GL_WAIT_FAILED ? -1 : 0;
}

/* after a timeout: the last fence signalled late (a slow or starved GPU, a
 * kernel recover that signalled it) = nothing of ours is in flight any more,
 * the context works again. Never blocks. Every target is drawn whole at its
 * next frame, the caller uploads the per-pixel state again (gpu_full) */
int epd_gpu_revive(epd_gpu *g)
{
    if (!g->lost_f && !g->x.cf) return 0;          /* (nothing to look at: stays lost) */
    if (eglGetCurrentContext() != g->ctx && !eglMakeCurrent(g->dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, g->ctx)) return 0;
    GLsync f[2] = { g->lost_f, g->x.cf };
    for (int i = 0; i < 2; i++) {
        if (!f[i]) continue;
        GLenum w = g->glClientWaitSync(f[i], 0, 0);
        if (w != GL_ALREADY_SIGNALED && w != GL_CONDITION_SATISFIED) return 0;
    }
    for (int i = 0; i < 2; i++) if (f[i]) g->glDeleteSync(f[i]);
    g->lost_f = 0;
    g->x.cf = 0;
    g->x.cpend = 0;
    while (g->glGetError() != GL_NO_ERROR) {}
    for (int i = 0; i < g->nt; i++) {
        struct target *t = &g->t[i];
        t->prev[0] = 0; t->prev[1] = EPD_LINE; t->prev[2] = 0; t->prev[3] = EPD_LINES;
        t->drawn = 0;
    }
    memset(g->stale, 0, sizeof g->stale);
    return 1;
}

int epd_gpu_error(epd_gpu *g)
{
    return (int)g->glGetError();
}

void epd_gpu_read(epd_gpu *g, int target, uint32_t *words)
{
    struct target *t = &g->t[target];
    g->glBindFramebuffer(GL_FRAMEBUFFER, t->fbo);
    if (!t->pw) {
        g->glReadPixels(0, 0, EPD_LINE, EPD_LINES, GL_RGBA, GL_UNSIGNED_BYTE, words);
        return;
    }
    /* flat: through the GPU into cached memory (the scanout mapping is
     * write-combined: reading it costs ~450 ms a frame), then unpadded */
    if (!g->flat && !(g->flat = malloc((size_t)FLAT_W * t->h * 4))) return;
    g->glReadPixels(0, 0, FLAT_W, t->h, GL_RGBA, GL_UNSIGNED_BYTE, g->flat);
    for (int l = 0; l < EPD_LINES; l++)
        memcpy(words + (size_t)l * EPD_LINE, g->flat + (size_t)l * t->pw, EPD_LINE * 4);
}

/* ---- the engine's per-pixel state on the GPU (gpe) ----
 *
 * Compute shaders (GLES 3.1; the A509 under Mesa 25.1 has it): the state is
 * updated in place, S = R32UI level | target << 8 | busy << 16 | stale << 23 |
 * op << 24 (busy <= 64: 7 bits; stale 0/1) and D = R32I dc (GLES 3.1 images
 * are read-write only as r32*). S is what every pixel needs; D only where the
 * DC record changes (preempted, claimed, healed pixels): an idle unchanged
 * pixel costs 4 bytes + its target, not 8 (the A509's claim is memory-bound:
 * test/gpe-cs-bench). The
 * fragment version (branch gpu-engine up to b4088d8) needed a
 * ping-pong copy and four reduction passes: every full-panel pass costs the
 * A509 1.5-3 ms, the claim came to 15.8 ms vs the A53's 9.5 (epd-gpe-test
 * GPE_PROF on the device). Now: one claim dispatch (workgroups reduce their
 * run extents and counters in shared memory into one partial each) + one
 * single-workgroup pass over the partials; ~250 bytes read back.
 *
 * The claim kernel is epd_claim_pixels (epd-engine.c) per pixel, in the same
 * order: preempt -> snap -> claim (or claim while a heal run is in flight) ->
 * heal -> stale -> the gather (matched length, DC record, panel model). The
 * arithmetic is the C code's, including its 8-bit (op, level) and 16-bit (dc)
 * wraps. */
#define GPE_A 1u                        /* taken by the step's run (oa) */
#define GPE_D 2u                        /* by its DU part (od) */
#define GPE_H 4u                        /* healed */
#define GPE_M0 8u                       /* << c: moved to matched cut c (bits 3..11) */
#define GPE_PRE 4096u                   /* preempted */
#define GPE_SNAP 8192u
#define GPE_FSKIP 16384u                /* a flash left it alone */
/* Workgroups with a barrier stay at 128 invocations: on the A509 (Mesa 25.1
 * freedreno) a 256-invocation workgroup with a loop before its barrier
 * writes nothing and can hang the GPU, 64 and 128 work (test/gpe-cs-probe2).
 * Wider is faster there (gpe-cs-bench: 32x8 12 ms, 32x2 18 ms a full pass) */
#define GPE_WX 32                       /* claim / query workgroup: 32 x 4 invocations */
#define GPE_WY 4                        /* (default; GLASSINE_GPE_WY 1..6 = 32 x 1..6, at most 192
                                         * invocations with barrier: ) */
#define GPE_WYS "@WY@"                  /* the rows in the claim/query text: link_cs writes
                                         * x.wy there (GPE_WY: the text as with STR(GPE_WY)) */
#define GPE_RPT 16                      /* default pixels per invocation (rows lq.y, lq.y + WY, ..;
                                         * GLASSINE_GPE_RPT 1..32, the shaders' RPT / TH):
                                         * a workgroup covers 32 x 64 (fewer partials) */
#define GPE_FY 8                        /* finish / op (no barrier): 32 x 8, one pixel each */
#define GPE_PART 64                     /* uints per workgroup partial (and the result):
                                         * [4k..4k+3] owner k: min x, max x + 1, min y, max y + 1
                                         * (rect-relative; min ~0 = none), [48..52] counters
                                         * preempted, snapped, flash skipped, matched, healed,
                                         * [53..61] probe: first wanting pixel per cut (y * EPD_W + x) */
#define GPE_MAXWG(th) (((EPD_W + GPE_WX - 1) / GPE_WX) * ((EPD_H + (th) - 1) / (th)))

#define GPE_HEAD \
    "#version 310 es\n" \
    "precision highp float; precision highp int; precision highp usampler2D; precision highp isampler2D;\n" \
    "precision highp uimage2D; precision highp iimage2D;\n"
#define GPE_SD \
    "layout(r32ui, binding = 0) uniform uimage2D S;\n" \
    "layout(r32i, binding = 1) uniform iimage2D D;\n" \
    "#if TEXREAD\n            /* GLASSINE_GPE_TEXREAD: reads through the texture path (each\n" \
    "                             invocation reads its own pixels before writing them) */\n" \
    "uniform usampler2D SS; uniform isampler2D DS;\n" \
    "#define LOADS(p) texelFetch(SS, (p), 0).r\n" \
    "#define LOADD(p) texelFetch(DS, (p), 0).r\n" \
    "#else\n" \
    "#define LOADS(p) imageLoad(S, (p)).r\n" \
    "#define LOADD(p) imageLoad(D, (p)).r\n" \
    "#endif\n"
#define GPE_PARTS \
    "layout(std430, binding = 0) buffer Part { uint part[]; };\n"
#define GPE_RES \
    "layout(std430, binding = 1) buffer Res { uint res[]; };\n" \
    "#if GATOM\n" \
    "#define PUT_MIN(i, v) if ((v) != 0xffffffffu) atomicMin(res[i], (v))\n" \
    "#define PUT_MAX(i, v) if ((v) != 0u) atomicMax(res[i], (v))\n" \
    "#define PUT_SUM(i, v) if ((v) != 0u) atomicAdd(res[i], (v))\n" \
    "#define PUT_OR(i, v) if ((v) != 0u) atomicOr(res[i], (v))\n" \
    "#else\n" \
    "#define PUT_MIN(i, v) part[o + (i)] = (v)\n" \
    "#define PUT_MAX(i, v) part[o + (i)] = (v)\n" \
    "#define PUT_SUM(i, v) part[o + (i)] = (v)\n" \
    "#define PUT_OR(i, v) part[o + (i)] = (v)\n" \
    "#endif\n"

static const char *gpe_cs_claim =
    GPE_HEAD
    "layout(local_size_x = " STR(GPE_WX) ", local_size_y = " GPE_WYS ") in;\n"
    GPE_SD GPE_PARTS GPE_RES
    "uniform usampler2D T;     /* the update's target levels at (x, y) - rc.xy */\n"
    "uniform sampler2D L;      /* stage 5: the update's target layer (epd-gtarget.h) */\n"
    "uniform int tl;           /* 1: the target from L, 0: from T */\n"
    "uint tg(ivec2 q, ivec2 p) {   /* the target level at rect-relative q = panel p */\n"
    "  if (tl == 0) return texelFetch(T, q, 0).r;\n"
    "  vec4 v = texelFetch(L, ivec2(p.x >> 2, p.y), 0);\n"
    "  int k = p.x & 3;\n"
    "  return uint(round((k == 0 ? v.r : k == 1 ? v.g : k == 2 ? v.b : v.a) * 255.0));\n"
    "}\n"
    "uniform isampler2D N;     /* net drive per transition: row 0 oa, 1 od, 2 + c matched cut c */\n"
    "uniform usampler2D B;     /* best cut: (d + MT_D, transition) */\n"
    "uniform ivec4 rc;\n"
    "uniform int fm, split, opm, R, aclamp, snp, hact, mt, cs, cap, model, npre, hmin, hmax;\n"
    "uniform uint oa, od, va, vd, hown, hf0, hf1;\n"
    "uniform int noat;         /* epd_gpu_gpe_noatom (timing only: no extents/counters) */\n"
    "uniform int probe;        /* 1: every cut as if it had a slot (which cuts are wanted, and\n"
    "                             where first; nothing stored) */\n"
    "uniform uint mown[" STR(EPD_LUT_CUTS) "];\n"
    "uniform uint rail[32];\n"
    "uniform uint pre[" STR(EPD_MAX_UPD) " + 1];\n"
    "uniform ivec4 rn[" STR(EPD_RUN_SLOTS) " + 1];\n"
    "shared uint rowm[TH], colm[" STR(GPE_WX) "], rowc[TH];\n"
    "shared uint rowf[TH * " STR(EPD_LUT_CUTS) "];\n"
    "bool rl(uint v) { return v == 0u || v == 30u; }\n"
    "#define DCL if (!hd) { dv = LOADD(p); dc = dv; hd = true; }   /* D where the DC record is used */\n"
    "uint px(ivec2 q) {         /* rect-relative pixel q: claimed, its GPE_* bits */\n"
    "  uint cl = 0u;\n"
    "  ivec2 p = rc.xy + q;\n"
    "  uint sv = LOADS(p);\n"
    "  int dv = 0, dc = 0;\n"
    "  bool hd = false;\n"
    "  uint l = sv & 255u, et = (sv >> 8) & 255u, b = (sv >> 16) & 127u, st = (sv >> 23) & 1u;\n"
    "  int op = int(sv >> 24);\n"
    "  uint t = tg(q, p);\n"
    /* preempt_row */
    "  if (npre > 0 && b != 0u && pre[b] != 0u && t != et && rl(t) && rl(et) && rl(l)) {\n"
    "    DCL\n"
    "    int r = rn[pre[b]][(et != 0u ? 2 : 0) | (l != 0u ? 1 : 0)];\n"
    "    if (opm != 0) op = clamp(op - r, 0, R);\n"
    "    dc = clamp(dc - r, -30000, 30000);\n"
    "    l = et; b = 0u; cl |= " STR(GPE_PRE) ";\n"
    "  }\n"
    /* snap_row */
    "  if (snp != 0 && b == 0u) {\n"
    "    if (l == 0u && t == 30u && 2 * op > R) { l = 30u; cl |= " STR(GPE_SNAP) "; }\n"
    "    else if (l == 30u && t == 0u && 2 * op < R) { l = 0u; cl |= " STR(GPE_SNAP) "; }\n"
    "  }\n"
    /* claim_row / claim_row_heal */
    "  bool fmv = fm == 2 ? st != 0u : fm == 1;\n"
    "  bool h = hact != 0 && b >= uint(" STR(EPD_HEAL_B) ");\n"
    "  uint lr = h ? et : l;\n"
    "  bool chg = (b == 0u || h) && (fmv || t != lr), bw = rl(t) && rl(lr);\n"
    "  bool ta = chg && (bw || split == 0), td = chg && split != 0 && !bw && od != 0u;\n"
    "  uint own = ta ? oa : td ? od : 0u;\n"
    "  if (hact != 0) {\n"
    "    bool pr = h && own != 0u;\n"
    "    uint k = (l + 1u) >> 1, done = b == uint(" STR(EPD_HEAL_B) ") ? hf0 : hf1;\n"
    "    int rem = k > done ? int(k - done) : 0;\n"
    "    int ru = pr && lr == 30u ? rem : 0, rd = pr && lr == 0u ? rem : 0;\n"
    "    op = (op - ru + rd) & 255;\n"
    "    if (ru != 0 || rd != 0) { DCL dc = dc - ru + rd; }\n"
    "    if (pr) l = lr;\n"
    "  }\n"
    "  if (own != 0u) { et = t; b = own; }\n"
    "  cl |= (ta ? " STR(GPE_A) " : 0u) | (td ? " STR(GPE_D) " : 0u);\n"
    /* heal_row */
    "  if (hown != 0u) {\n"
    "    bool w = l == 30u, bl = l == 0u;\n"
    "    int def = w ? (R - op) & 255 : op;\n"
    "    if (b == 0u && t == l && (w || bl) && def >= hmin) {\n"
    "      DCL\n"
    "      int k = min(def, hmax), ku = w ? k : 0, kd = bl ? k : 0;\n"
    "      op = (op + ku - kd) & 255;\n"
    "      dc = dc + ku - kd;\n"
    "      b = hown; et = l; l = uint(2 * k - 1); cl |= " STR(GPE_H) ";\n"
    "    }\n"
    "  }\n"
    /* stale_row, flash_skips */
    "  if (b == oa) st = va; else if (od != 0u && b == od) st = vd; else if (hown != 0u && b == hown) st = 1u;\n"
    "  if (fm == 2 && st == 0u && t == l && b == 0u) cl |= " STR(GPE_FSKIP) ";\n"
    /* the gather */
    "  if (b == oa || (od != 0u && b == od)) {\n"
    "    DCL\n"
    "    uint wo = b;\n"
    "    int i = int(t << 5 | l);\n"
    "    int n = texelFetch(N, ivec2(i, wo == oa ? 0 : 1), 0).r;\n"
    "    if (mt != 0 && wo == oa && n != 0) {\n"
    "      bool want = false; int d = 0;\n"
    "      if (model != 0) {\n"
    "        if (rl(t) && rl(l)) {\n"
    "          int need = t == 30u ? R - op : op;\n"
    "          if (need > abs(n)) { want = true; d = n > 0 ? -need : need; }\n"
    "          if (model == 2 && dc != 0 && (dc < 0) != (n < 0) && (!want || abs(dc) > need)) { want = true; d = dc; }\n"
    "        }\n"
    "      } else if (dc != 0 && (dc < 0) != (n < 0)) { want = true; d = dc; }\n"
    "      if (want) {\n"
    "        d = clamp(d, -" STR(EPD_MT_D) ", " STR(EPD_MT_D) ");\n"
    "        int c = int(texelFetch(B, ivec2(d + " STR(EPD_MT_D) ", i), 0).r);\n"
    "        if (model != 0 && c > cs) c = cs;\n"
    "        if (cap != 0 && c < cs - cap) c = cs - cap;\n"
    "        if (c != cs && (probe != 0 || mown[c] != 0u)) {\n"
    "          b = mown[c]; n = texelFetch(N, ivec2(i, 2 + c), 0).r; cl |= " STR(GPE_M0) " << uint(c);\n"
    "        }\n"
    "      }\n"
    "    }\n"
    "    dc = clamp(dc + n, -30000, 30000);\n"
    "    if (opm != 0) op = wo == oa && aclamp != 0 && rl(t) && rl(l) ? clamp(op + n, 0, R) : int(rail[t]);\n"
    "  }\n"
    "  if (probe == 0) {          /* (the C arrays' widths: level/op uint8, dc int16) */\n"
    "    uint ns = (l & 255u) | (et & 255u) << 8 | (b & 127u) << 16 | (st & 1u) << 23 | uint(op & 255) << 24;\n"
    "    if (ns != sv) imageStore(S, p, uvec4(ns));   /* (most of a rect stays as it was:\n"
    "                                                   only what changed is written) */\n"
    "    if (hd && (dc << 16 >> 16) != dv) imageStore(D, p, ivec4(dc << 16 >> 16));\n"
    "  }\n"
    "  return cl;\n"
    "}\n"
    "void main() {\n"
    "  uint li = gl_LocalInvocationIndex;\n"
    "  uvec2 lq = gl_LocalInvocationID.xy;\n"
    "  if (li < uint(TH)) { rowm[li] = 0u; rowc[li] = 0u; }\n"
    "  if (li < uint(" STR(GPE_WX) ")) colm[li] = 0u;\n"
    "  for (uint k = li; k < uint(TH * " STR(EPD_LUT_CUTS) "); k += uint(" STR(GPE_WX) " * " GPE_WYS ")) rowf[k] = 0xffffffffu;\n"
    "  memoryBarrierShared(); barrier();\n"
    "  ivec2 q0 = ivec2(gl_WorkGroupID.xy) * ivec2(" STR(GPE_WX) ", TH);\n"
    "  uint cm = 0u;\n"
    "  for (int j = 0; j < RPT; j++) {\n"
    "    int lr = j * " GPE_WYS " + int(lq.y);      /* the row within the workgroup's tile */\n"
    "    ivec2 q = q0 + ivec2(lq.x, lr);               /* rect-relative */\n"
    "    if (q.x >= rc.z || q.y >= rc.w) continue;\n"
    "    uint cl = px(q);\n"
    "    if (cl == 0u || noat != 0) continue;\n"
    /* this workgroup's extents and counters: per row and column of its tile first */
    "    cm |= cl;\n"
    "    atomicOr(rowm[lr], cl & 4095u);\n"
    "    atomicAdd(rowc[lr], ((cl >> 12) & 1u) | ((cl >> 13) & 1u) << 6 | ((cl >> 14) & 1u) << 12 |\n"
    "                        ((cl & 4088u) != 0u ? 1u : 0u) << 18 | ((cl >> 2) & 1u) << 24);\n"
    "    if (probe != 0)\n"
    "      for (int c = 0; c < " STR(EPD_LUT_CUTS) "; c++)\n"
    "        if ((cl & (" STR(GPE_M0) " << uint(c))) != 0u) atomicMin(rowf[lr * " STR(EPD_LUT_CUTS) " + c], uint(q.x));\n"
    "  }\n"
    "  if (cm != 0u) atomicOr(colm[lq.x], cm & 4095u);\n"
    "  memoryBarrierShared(); barrier();\n"
    "  uint wg = gl_WorkGroupID.y * gl_NumWorkGroups.x + gl_WorkGroupID.x, o = wg * " STR(GPE_PART) "u;\n"
    "  if (li < 12u) {             /* owner li */\n"
    "    uint bit = 1u << li, x0 = 0xffffffffu, x1 = 0u, y0 = 0xffffffffu, y1 = 0u;\n"
    "    for (int k = 0; k < " STR(GPE_WX) "; k++) if ((colm[k] & bit) != 0u) { x0 = min(x0, uint(q0.x + k)); x1 = uint(q0.x + k + 1); }\n"
    "    for (int k = 0; k < TH; k++) if ((rowm[k] & bit) != 0u) { y0 = min(y0, uint(q0.y + k)); y1 = uint(q0.y + k + 1); }\n"
    "    PUT_MIN(li * 4u, x0); PUT_MAX(li * 4u + 1u, x1); PUT_MIN(li * 4u + 2u, y0); PUT_MAX(li * 4u + 3u, y1);\n"
    "  } else if (li < 17u) {      /* counter li - 12 */\n"
    "    uint s = 0u, sh = (li - 12u) * 6u;\n"
    "    for (int k = 0; k < TH; k++) s += (rowc[k] >> sh) & 63u;\n"
    "    PUT_SUM(36u + li, s);\n"
    "  } else if (li < 17u + uint(" STR(EPD_LUT_CUTS) ")) {    /* probe: first pixel wanting cut li - 17 */\n"
    "    uint c = li - 17u, f = 0xffffffffu;\n"
    "    for (int k = 0; k < TH && f == 0xffffffffu; k++)\n"
    "      if (rowf[k * " STR(EPD_LUT_CUTS) " + int(c)] != 0xffffffffu) f = uint(q0.y + k) * " STR(EPD_W) "u + rowf[k * " STR(EPD_LUT_CUTS) " + int(c)];\n"
    "    PUT_MIN(36u + li, f);\n"
    "  }\n"
    "}\n";

/* the partials of n workgroups into res[0..GPE_PART): mode 0 a claim's (min /
 * max / sum per field, see GPE_PART), 1 a blocked test's (OR of [0..2]). One
 * workgroup */
static const char *gpe_cs_reduce =
    GPE_HEAD
    "layout(local_size_x = " STR(GPE_PART) ") in;\n"
    GPE_PARTS
    "layout(std430, binding = 1) buffer Res { uint res[]; };\n"
    "uniform int n, mode;\n"
    "bool ismin(uint f) { return f < 48u ? (f & 1u) == 0u : f >= 53u; }\n"
    "bool issum(uint f) { return f >= 48u && f < 53u; }\n"
    "void main() {             /* field f, one invocation each (no barrier: see GPE_WX) */\n"
    "  uint f = gl_LocalInvocationIndex, nf = mode == 0 ? 62u : 3u;\n"
    "  uint a = mode == 0 && ismin(f) ? 0xffffffffu : 0u;\n"
    "  if (f < nf)\n"
    "    for (int w = 0; w < n; w++) {\n"
    "      uint v = part[uint(w) * " STR(GPE_PART) "u + f];\n"
    "      if (mode != 0) a |= v; else if (ismin(f)) a = min(a, v); else if (issum(f)) a += v; else a = max(a, v);\n"
    "    }\n"
    "  res[f] = a;\n"
    "}\n";

/* a run's pixels reach their target (finish_row) for every slot set in fin */
static const char *gpe_cs_fin =
    GPE_HEAD
    "layout(local_size_x = " STR(GPE_WX) ", local_size_y = " STR(GPE_FY) ") in;\n"
    GPE_SD
    "uniform ivec4 rc;\n"
    "uniform uvec2 fin;        /* slots finishing: bit busy - 1 */\n"
    "void main() {\n"
    "  ivec2 q = ivec2(gl_GlobalInvocationID.xy);\n"
    "  if (q.x >= rc.z || q.y >= rc.w) return;\n"
    "  ivec2 p = rc.xy + q;\n"
    "  uint v = LOADS(p), b = (v >> 16) & 127u;\n"
    "  if (b == 0u) return;\n"
    "  uint i = b - 1u;\n"
    "  if (((i < 32u ? fin.x >> i : fin.y >> (i - 32u)) & 1u) != 0u)\n"
    "    imageStore(S, p, uvec4((v & 0xff80ff00u) | (v >> 8) & 255u));    /* level = target, busy 0 (stale kept) */\n"
    "}\n";

/* op_prep on the whole panel: the model from the levels (mode 1) or rescaled
 * from op_r old to R (2) */
static const char *gpe_cs_op =
    GPE_HEAD
    "layout(local_size_x = " STR(GPE_WX) ", local_size_y = " STR(GPE_FY) ") in;\n"
    GPE_SD
    "uniform int mode, R, old;\n"
    "void main() {\n"
    "  ivec2 p = ivec2(gl_GlobalInvocationID.xy);\n"
    "  if (p.x >= " STR(EPD_W) " || p.y >= " STR(EPD_H) ") return;\n"
    "  uint v = LOADS(p), l = v & 255u;\n"
    "  int op = mode == 1 ? (l >= 30u ? R : R * int(l) / 30) : int(v >> 24) * R / old;\n"
    "  imageStore(S, p, uvec4((v & 0xffffffu) | uint(op & 255) << 24));\n"
    "}\n";

/* epd_pend_query: the slots in flight elsewhere than the target (busy <
 * EPD_HEAL_B) as a 64-bit mask, and whether an idle pixel that changes (any,
 * if full) is not black/white -> black/white; per workgroup into its partial */
static const char *gpe_cs_query =
    GPE_HEAD
    "layout(local_size_x = " STR(GPE_WX) ", local_size_y = " GPE_WYS ") in;\n"
    GPE_SD GPE_PARTS GPE_RES
    "uniform usampler2D T;\n"
    "uniform sampler2D L;      /* stage 5: the update's target layer (epd-gtarget.h) */\n"
    "uniform int tl;           /* 1: the target from L, 0: from T */\n"
    "uint tg(ivec2 q, ivec2 p) {   /* the target level at rect-relative q = panel p */\n"
    "  if (tl == 0) return texelFetch(T, q, 0).r;\n"
    "  vec4 v = texelFetch(L, ivec2(p.x >> 2, p.y), 0);\n"
    "  int k = p.x & 3;\n"
    "  return uint(round((k == 0 ? v.r : k == 1 ? v.g : k == 2 ? v.b : v.a) * 255.0));\n"
    "}\n"
    "uniform ivec4 rc;\n"
    "uniform int full;\n"
    "shared uint rq[3];\n"
    "bool rl(uint v) { return v == 0u || v == 30u; }\n"
    "void main() {\n"
    "  uint li = gl_LocalInvocationIndex;\n"
    "  uvec2 lq = gl_LocalInvocationID.xy;\n"
    "  if (li < 3u) rq[li] = 0u;\n"
    "  memoryBarrierShared(); barrier();\n"
    "  ivec2 q0 = ivec2(gl_WorkGroupID.xy) * ivec2(" STR(GPE_WX) ", TH);\n"
    "  uint m0 = 0u, m1 = 0u, nb = 0u;\n"
    "  for (int j = 0; j < RPT; j++) {\n"
    "    ivec2 q = q0 + ivec2(lq.x, j * " GPE_WYS " + int(lq.y));\n"
    "    if (q.x >= rc.z || q.y >= rc.w) continue;\n"
    "    uint v = LOADS(rc.xy + q), l = v & 255u, et = (v >> 8) & 255u, b = (v >> 16) & 127u;\n"
    "    uint t = tg(q, rc.xy + q);\n"
    "    if (b != 0u && b < uint(" STR(EPD_HEAL_B) ") && et != t) { uint i = b - 1u; if (i < 32u) m0 |= 1u << i; else m1 |= 1u << (i - 32u); }\n"
    "    if (b == 0u && (full != 0 || t != l) && !(rl(t) && rl(l))) nb = 1u;\n"
    "  }\n"
    "  if (m0 != 0u) atomicOr(rq[0], m0);\n"
    "  if (m1 != 0u) atomicOr(rq[1], m1);\n"
    "  if (nb != 0u) atomicOr(rq[2], 1u);\n"
    "  memoryBarrierShared(); barrier();\n"
    "  uint o = (gl_WorkGroupID.y * gl_NumWorkGroups.x + gl_WorkGroupID.x) * " STR(GPE_PART) "u;\n"
    "  if (li < 3u) PUT_OR(li, rq[li]);\n"
    "}\n";

/* texture units of the gpe passes (the drive frames use 0..3) */
enum { GU_S = 4, GU_D, GU_T, GU_N, GU_B, GU_L };

/* per-pass profile (epd_gpu_prof_on): the GPU finished after each pass, the
 * time since the previous mark added to its slot; slows everything down */
int epd_gpu_prof_on;
int epd_gpu_gpe_debug;
int epd_gpu_gpe_noatom;
void (*epd_gpu_gpe_debug_hook)(const char *what);
/* epd_gpu_gpe_debug: the GPU finished after pass `what`, a GL error printed */
static void gl_check(epd_gpu *g, const char *what)
{
    if (!epd_gpu_gpe_debug) return;
    g->glFinish();
    GLenum e = g->glGetError();
    if (e != GL_NO_ERROR) fprintf(stderr, "epd-gpu: gpe %s: GL error 0x%x\n", what, e);
    if (epd_gpu_gpe_debug_hook) epd_gpu_gpe_debug_hook(what);
}
double epd_gpu_prof[EPD_GPU_PROF_N];
static double prof_t;
static double tnow(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}
static void prof(epd_gpu *g, int k)
{
    if (!epd_gpu_prof_on) return;
    g->glFinish();
    double t = tnow();
    if (k >= 0) epd_gpu_prof[k] += t - prof_t;
    prof_t = t;
}

static GLuint link_cs(epd_gpu *g, const char *src)
{
    char defs[128], *full;
    snprintf(defs, sizeof defs, "#define RPT %d\n#define TH %d\n#define GATOM %d\n#define TEXREAD %d\n", g->x.rpt, g->x.th,
             g->x.gatom, g->x.texread);
    const char *nl = strchr(src, '\n') + 1;   /* (after "#version 310 es") */
    if (!(full = malloc(strlen(src) + sizeof defs))) return 0;
    snprintf(full, strlen(src) + sizeof defs, "%.*s%s%s", (int)(nl - src), src, defs, nl);
    for (char *w; (w = strstr(full, GPE_WYS)); ) {      /* the workgroup rows (x.wy <= 6: one digit) */
        *w = (char)('0' + g->x.wy);
        memmove(w + 1, w + strlen(GPE_WYS), strlen(w + strlen(GPE_WYS)) + 1);
    }
    GLuint c = shader(g, GL_COMPUTE_SHADER, full);
    free(full);
    if (!c) return 0;
    GLuint p = g->glCreateProgram();
    g->glAttachShader(p, c);
    g->glLinkProgram(p);
    GLint ok = 0;
    g->glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024];
        g->glGetProgramInfoLog(p, sizeof log, NULL, log);
        fprintf(stderr, "epd-gpu: gpe link: %s\n", log);
        return 0;
    }
    g->glUseProgram(p);
    static const struct { const char *n; int u; } m[] = { { "T", GU_T }, { "N", GU_N }, { "B", GU_B }, { "L", GU_L }, { "SS", GU_S }, { "DS", GU_D } };
    for (unsigned i = 0; i < sizeof m / sizeof *m; i++) {
        GLint l = g->glGetUniformLocation(p, m[i].n);
        if (l >= 0) g->glUniform1i(l, m[i].u);
    }
    return p;
}

static GLuint fbo_of(epd_gpu *g, GLuint tx)
{
    GLuint f;
    g->glGenFramebuffers(1, &f);
    g->glBindFramebuffer(GL_FRAMEBUFFER, f);
    g->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tx, 0);
    if (g->glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        fprintf(stderr, "epd-gpu: gpe framebuffer incomplete\n");
        return 0;
    }
    return f;
}

#define U1I(p, n, v) g->glUniform1i(g->glGetUniformLocation(p, n), (v))
#define U1U(p, n, v) g->glUniform1ui(g->glGetUniformLocation(p, n), (v))

static int t_set(epd_gpu *g, GLuint p, epd_rect r, const uint8_t *tg, int gt);

/* program p over rect r (rect-relative invocations), S and D as images; then
 * every later use sees its writes */
static void dispatch(epd_gpu *g, GLuint p, epd_rect r, int th)
{
    g->glUseProgram(p);
    GLint l = g->glGetUniformLocation(p, "rc");
    if (l >= 0) g->glUniform4i(l, r.x, r.y, r.w, r.h);
    g->glBindImageTexture(0, g->x.S, 0, GL_FALSE, 0, GL_READ_WRITE, GL_R32UI);
    g->glBindImageTexture(1, g->x.D, 0, GL_FALSE, 0, GL_READ_WRITE, GL_R32I);
    g->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, g->x.part);
    g->glBindBufferRange(GL_SHADER_STORAGE_BUFFER, 1, g->x.resb, (GLintptr)g->x.reg * g->x.rstride, GPE_PART * 4);
    g->glDispatchCompute((GLuint)(r.w + GPE_WX - 1) / GPE_WX, (GLuint)(r.h + th - 1) / th, 1);
    g->glMemoryBarrier(GL_ALL_BARRIER_BITS);
    gl_check(g, p == g->x.pclaim ? "claim" : p == g->x.pfin ? "finish" : p == g->x.pop ? "op" : "query");
}

/* before a claim (mode 0) or query (1) dispatch with GATOM: the result's
 * identities (min fields ~0, the rest 0), the passes' atomics land on them */
static void res_init(epd_gpu *g, int mode)
{
    if (!g->x.gatom) return;
    uint32_t id[GPE_PART];
    for (int f = 0; f < GPE_PART; f++) id[f] = mode == 0 && (f < 48 ? (f & 1) == 0 : f >= 53) ? 0xffffffffu : 0;
    g->glBindBuffer(GL_SHADER_STORAGE_BUFFER, g->x.resb);
    g->glBufferSubData(GL_SHADER_STORAGE_BUFFER, (GLintptr)g->x.reg * g->x.rstride, sizeof id, id);
}

/* the partials of the last dispatch over r reduced (mode: gpe_cs_reduce);
 * nothing with GATOM (the dispatch's atomics made the result) */
static void reduce(epd_gpu *g, epd_rect r, int th, int mode)
{
    if (g->x.gatom && th != 1) return;          /* (th 1: the selftest's own partials) */
    g->glUseProgram(g->x.pred);
    U1I(g->x.pred, "n", ((r.w + GPE_WX - 1) / GPE_WX) * ((r.h + th - 1) / th));
    U1I(g->x.pred, "mode", mode);
    g->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, g->x.part);
    g->glBindBufferRange(GL_SHADER_STORAGE_BUFFER, 1, g->x.resb, (GLintptr)g->x.reg * g->x.rstride, GPE_PART * 4);
    g->glDispatchCompute(1, 1, 1);
    g->glMemoryBarrier(GL_ALL_BARRIER_BITS);
    gl_check(g, "reduce");
}

/* result region reg into res, the GPU done with it (NULL: not mapped) */
static const uint32_t *res_read(epd_gpu *g, int reg)
{
    if (g->x.resm) {                            /* (coherent: the passes' barrier made it visible) */
        memcpy(g->x.res, (const uint8_t *)g->x.resm + (size_t)reg * g->x.rstride, sizeof g->x.res);
        return g->x.res;
    }
    g->glBindBuffer(GL_SHADER_STORAGE_BUFFER, g->x.resb);
    const uint32_t *m = g->glMapBufferRange(GL_SHADER_STORAGE_BUFFER, (GLintptr)reg * g->x.rstride, GPE_PART * 4, GL_MAP_READ_BIT);
    if (m) memcpy(g->x.res, m, sizeof g->x.res);
    g->glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    return m ? g->x.res : NULL;
}

/* the reduced result in region reg (waits; NULL: hung) — valid until the next
 * call */
static const uint32_t *result(epd_gpu *g, int timeout_ms)
{
    if (epd_gpu_finish(g, timeout_ms)) return NULL;
    return res_read(g, g->x.reg);
}

/* the reduce pass on known partials vs the same on the CPU (a device check:
 * the gpe's results were all zero on the A509); 0 ok */
int epd_gpu_gpe_selftest(epd_gpu *g)
{
    if (!g->x.on) return -1;
    enum { NW = 10 };
    static uint32_t pt[NW * GPE_PART];
    uint32_t want[GPE_PART];
    int bad = 0;
    srand(7);
    for (int mode = 0; mode < 2; mode++) {
        for (int i = 0; i < NW * GPE_PART; i++) pt[i] = (uint32_t)rand() % 100000;
        g->glBindBuffer(GL_SHADER_STORAGE_BUFFER, g->x.part);
        g->glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof pt, pt);
        int nf = mode == 0 ? 62 : 3;
        for (int f = 0; f < nf; f++) {
            int mn = f < 48 ? (f & 1) == 0 : f >= 53, sm = f >= 48 && f < 53;
            uint32_t a = mode == 0 && mn ? 0xffffffffu : 0;
            for (int w = 0; w < NW; w++) {
                uint32_t v = pt[w * GPE_PART + f];
                if (mode) a |= v; else if (mn) a = v < a ? v : a; else if (sm) a += v; else a = v > a ? v : a;
            }
            want[f] = a;
        }
        reduce(g, (epd_rect){ 0, 0, NW * GPE_WX, 1 }, 1, mode);
        const uint32_t *res = result(g, 2000);
        if (!res) { fprintf(stderr, "epd-gpu: selftest reduce %d: no result\n", mode); bad++; continue; }
        int nb = 0, first = -1;
        for (int f = 0; f < nf; f++) if (res[f] != want[f]) { nb++; if (first < 0) first = f; }
        fprintf(stderr, "epd-gpu: selftest reduce mode %d: %s", mode, nb ? "FAIL" : "ok");
        if (nb) fprintf(stderr, " (%d fields wrong, first %d: gpu %u want %u; gpu [0..3] %u %u %u %u)", nb, first, res[first], want[first],
                        res[0], res[1], res[2], res[3]);
        fprintf(stderr, "\n");
        bad += nb != 0;
        /* the partials still there? (read back at offset 0) */
        g->glBindBuffer(GL_SHADER_STORAGE_BUFFER, g->x.part);
        const uint32_t *m = g->glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, 16, GL_MAP_READ_BIT);
        if (m) fprintf(stderr, "epd-gpu: selftest partials [0..3] %u %u %u %u (uploaded %u %u %u %u)\n", m[0], m[1], m[2], m[3], pt[0], pt[1], pt[2], pt[3]);
        g->glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    }
    return bad;
}

void epd_gpu_gpe_upload(epd_gpu *g, const epd_engine *e, epd_rect r)
{
    if (!g->x.on || !epd_gpu_pack_state((uint16_t *)g->x.stage, e, &r)) return;   /* (clips r; the pack is overwritten) */
    uint32_t *s4 = (uint32_t *)g->x.stage;
    for (int y = 0; y < r.h; y++)
        for (int x = 0; x < r.w; x++) {
            size_t i = (size_t)(r.y + y) * EPD_W + r.x + x;
            s4[(size_t)y * r.w + x] = e->level[i] | (uint32_t)e->target[i] << 8 | (uint32_t)e->busy[i] << 16 |
                                      (uint32_t)(e->stale[i] & 1) << 23 | (uint32_t)e->op[i] << 24;
        }
    bind(g, GU_S, g->x.S);
    g->glTexSubImage2D(GL_TEXTURE_2D, 0, r.x, r.y, r.w, r.h, GL_RED_INTEGER, GL_UNSIGNED_INT, s4);
    int32_t *d4 = (int32_t *)g->x.stage;
    for (int y = 0; y < r.h; y++)
        for (int x = 0; x < r.w; x++) {
            size_t i = (size_t)(r.y + y) * EPD_W + r.x + x;
            d4[(size_t)y * r.w + x] = e->dc[i];
        }
    bind(g, GU_D, g->x.D);
    g->glTexSubImage2D(GL_TEXTURE_2D, 0, r.x, r.y, r.w, r.h, GL_RED_INTEGER, GL_INT, d4);
}

/* S and D made anew and filled from e (the GPU idle first): the state put
 * back without a CPU upload into a texture that compute passes have
 * image-stored into (on the A509 such an upload can be lost); the
 * fresh textures are idle and never stored, as at open */
int epd_gpu_gpe_reseed(epd_gpu *g, const epd_engine *e)
{
    if (!g->x.on) return -1;
    g->glFinish();
    GLuint old[2] = { g->x.fbs, g->x.fbd }, ot[2] = { g->x.S, g->x.D };
    g->glDeleteFramebuffers(2, old);
    g->glDeleteTextures(2, ot);
    g->glActiveTexture(GL_TEXTURE0 + GU_S);
    g->x.S = tex(g, GL_R32UI, EPD_W, EPD_H);
    g->glActiveTexture(GL_TEXTURE0 + GU_D);
    g->x.D = tex(g, GL_R32I, EPD_W, EPD_H);
    int bad = !(g->x.fbs = fbo_of(g, g->x.S)) || !(g->x.fbd = fbo_of(g, g->x.D));
    g->glBindFramebuffer(GL_FRAMEBUFFER, 0);
    epd_gpu_gpe_upload(g, e, (epd_rect){ 0, 0, EPD_W, EPD_H });
    return bad || g->glGetError() != GL_NO_ERROR ? -1 : 0;
}

const char *epd_gpu_gpe_shape(const epd_gpu *g)
{
    static char b[128];                 /* (all three parts fit: no truncation) */
    int n = snprintf(b, sizeof b, "rpt %d, gatom %d, texread %d", g->x.rpt, g->x.gatom, g->x.texread);
    if (g->x.wy != GPE_WY) n += snprintf(b + n, sizeof b - n, ", wy %d (32 x %d)", g->x.wy, g->x.wy);
    if (!g->x.resm) snprintf(b + n, sizeof b - n, ", results mapped per read");     /* (GLASSINE_GPE_PMAP=0, or no GL_EXT_buffer_storage) */
    return b;
}

int epd_gpu_gpe_open(epd_gpu *g, const epd_engine *e)
{
    if (g->x.on) return 0;
    if (e->cst) {                       /* (the gpe reads and writes the six arrays) */
        fprintf(stderr, "epd-gpu: gpe needs claim_state=arrays (%s)\n", epd_claim_state_name(e));
        return -1;
    }
    const char *ver = (const char *)g->glGetString(GL_VERSION);
    int maj = 0, min = 0;
    if (!ver || sscanf(ver, "OpenGL ES %d.%d", &maj, &min) != 2 || maj * 10 + min < 31) {
        fprintf(stderr, "epd-gpu: gpe needs GLES 3.1 (%s)\n", ver ? ver : "?");
        return -1;
    }
#define F(t, n) if (!(g->n = (t)eglGetProcAddress(#n))) { fprintf(stderr, "epd-gpu: missing %s\n", #n); return -1; }
    GPE_FNS
#undef F
    if (!(g->x.stage = malloc((size_t)EPD_W * EPD_H * 4))) return -1;
    {
        const char *v = getenv("GLASSINE_GPE_RPT"), *a = getenv("GLASSINE_GPE_GATOM");
        const char *y = getenv("GLASSINE_GPE_WY");
        /* the workgroup rows (2 = the 10-04 sweep's 32 x 2, 4 the
         * code, 6 = 192): a loop before barrier at more than one wave writes
         * nothing at 256 invocations on the A509, so never above 192 */
        g->x.wy = y ? atoi(y) : GPE_WY;
        if (g->x.wy < 1 || g->x.wy > 6) {
            fprintf(stderr, "epd-gpu: GLASSINE_GPE_WY=%s: 1..6 (32 x WY <= 192 invocations), %d taken\n", y, GPE_WY);
            g->x.wy = GPE_WY;
        }
        g->x.rpt = v ? atoi(v) : GPE_RPT;
        /* (TH <= the invocations, 32 x WY: the shared init, one row each;
         * the old cap 64 / WY was 32 x 2's) */
        if (g->x.rpt < 1 || g->x.rpt > GPE_WX) g->x.rpt = GPE_RPT;
        g->x.th = g->x.wy * g->x.rpt;
        g->x.gatom = a ? atoi(a) != 0 : 0;
        /* texread 1 by default: on the A509 (Mesa 25.1.9 freedreno) an imageLoad
         * in a dispatch can miss the previous dispatch's imageStores despite
         * glMemoryBarrier(ALL) (device sweep texread 0 diverged in
         * every shape, 0 differences with a glFinish per pass; texread 1 exact
         * in all 10 shapes, and the fastest) */
        g->x.texread = (v = getenv("GLASSINE_GPE_TEXREAD")) ? atoi(v) != 0 : 1;
    }
    g->glActiveTexture(GL_TEXTURE0 + GU_T);     /* (tex binds on the active unit) */
    g->x.S = tex(g, GL_R32UI, EPD_W, EPD_H);
    g->x.D = tex(g, GL_R32I, EPD_W, EPD_H);
    g->x.T = tex(g, GL_R8UI, EPD_W, EPD_H);
    g->x.N = tex(g, GL_R8I, 1024, 2 + EPD_LUT_CUTS);
    g->x.B = tex(g, GL_R8UI, 2 * EPD_MT_D + 1, 1024);
    g->glGenBuffers(1, &g->x.part);
    g->glBindBuffer(GL_SHADER_STORAGE_BUFFER, g->x.part);
    g->glBufferData(GL_SHADER_STORAGE_BUFFER, g->x.gatom ? 16 * GPE_PART * 4 : (GLsizeiptr)GPE_MAXWG(g->x.th) * GPE_PART * 4, NULL, GL_DYNAMIC_COPY);
    {                                           /* the reduced results: two regions (x.reg) */
        GLint al = 0;
        g->glGetIntegerv(GL_SHADER_STORAGE_BUFFER_OFFSET_ALIGNMENT, &al);
        int st = GPE_PART * 4;
        if (al > 0) st = (st + al - 1) / al * al;
        g->x.rstride = st;
        g->x.reg = 0;
        g->glGenBuffers(1, &g->x.resb);
        g->glBindBuffer(GL_SHADER_STORAGE_BUFFER, g->x.resb);
        /* mapped once (part 1 of the deferred claims, 15 : a map per claim
         * was 1.8 ms on the A509), else per read */
        const char *v = getenv("GLASSINE_GPE_PMAP"), *ex = (const char *)g->glGetString(GL_EXTENSIONS);
        PFNGLBUFFERSTORAGEEXTPROC bs = (!v || atoi(v)) && ex && strstr(ex, "GL_EXT_buffer_storage") ?
                                       (PFNGLBUFFERSTORAGEEXTPROC)eglGetProcAddress("glBufferStorageEXT") : NULL;
        const GLbitfield mf = GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT_EXT | GL_MAP_COHERENT_BIT_EXT;
        g->x.resm = NULL;
        if (bs) {
            bs(GL_SHADER_STORAGE_BUFFER, 2 * st, NULL, mf | GL_DYNAMIC_STORAGE_BIT_EXT);
            if (g->glGetError() == GL_NO_ERROR) g->x.resm = g->glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, 2 * st, mf);
            if (!g->x.resm) {                   /* (a fresh name: immutable storage can't be redone) */
                while (g->glGetError() != GL_NO_ERROR) {}
                g->glGenBuffers(1, &g->x.resb);
                g->glBindBuffer(GL_SHADER_STORAGE_BUFFER, g->x.resb);
                bs = NULL;
            }
        }
        if (!bs) g->glBufferData(GL_SHADER_STORAGE_BUFFER, 2 * st, NULL, GL_DYNAMIC_READ);
        if (epd_gpu_gpe_debug) fprintf(stderr, "epd-gpu: gpe results %s, %d bytes apart\n", g->x.resm ? "mapped for good" : "mapped per read", st);
    }
    if (!(g->x.fbs = fbo_of(g, g->x.S)) || !(g->x.fbd = fbo_of(g, g->x.D))) return -1;
    if (!(g->x.pclaim = link_cs(g, gpe_cs_claim)) || !(g->x.pred = link_cs(g, gpe_cs_reduce)) ||
        !(g->x.pfin = link_cs(g, gpe_cs_fin)) || !(g->x.pop = link_cs(g, gpe_cs_op)) || !(g->x.pquery = link_cs(g, gpe_cs_query)))
        return -1;
    {                                           /* the drive shader on S: fs with GPE defined */
        static const char ver3[] = "#version 300 es\n";
        size_t n = strlen(fs) + 32;
        char *src = malloc(n);
        if (!src) return -1;
        snprintf(src, n, "%s#define GPE 1\n%s", ver3, fs + strlen(ver3));
        GLuint v = shader(g, GL_VERTEX_SHADER, vs), f = shader(g, GL_FRAGMENT_SHADER, src);
        free(src);
        if (!v || !f) return -1;
        GLuint p = g->glCreateProgram();
        g->glAttachShader(p, v);
        g->glAttachShader(p, f);
        g->glLinkProgram(p);
        GLint ok = 0;
        g->glGetProgramiv(p, GL_LINK_STATUS, &ok);
        if (!ok) { fprintf(stderr, "epd-gpu: gpe drive shader: link failed\n"); return -1; }
        g->x.sprog = p;
        g->glUseProgram(p);
        g->glUniform1i(g->glGetUniformLocation(p, "tpl"), 0);
        g->glUniform1i(g->glGetUniformLocation(p, "lut"), 2);
        g->glUniform1i(g->glGetUniformLocation(p, "S"), GU_S);
        g->x.su_pw = g->glGetUniformLocation(p, "pw");
        g->x.su_box = g->glGetUniformLocation(p, "box");
    }
    if (epd_gpu_gpe_debug) {
        GLint v[3];
        g->glGetIntegerv(GL_MAX_COMPUTE_WORK_GROUP_INVOCATIONS, v);
        fprintf(stderr, "epd-gpu: gpe on %s: max invocations %d", g->glGetString(GL_RENDERER), v[0]);
        g->glGetIntegerv(GL_MAX_COMPUTE_SHARED_MEMORY_SIZE, v);
        fprintf(stderr, ", shared %d", v[0]);
        g->glGetIntegerv(GL_MAX_COMPUTE_UNIFORM_COMPONENTS, v);
        fprintf(stderr, ", uniform components %d", v[0]);
        g->glGetIntegerv(GL_MAX_COMPUTE_IMAGE_UNIFORMS, v);
        fprintf(stderr, ", images %d", v[0]);
        g->glGetIntegerv(GL_MAX_COMPUTE_SHADER_STORAGE_BLOCKS, v);
        fprintf(stderr, ", ssbos %d\n", v[0]);
        static const char *nm[] = { "claim", "reduce", "finish", "op", "query" };
        GLuint ps[] = { g->x.pclaim, g->x.pred, g->x.pfin, g->x.pop, g->x.pquery };
        for (int i = 0; i < 5; i++) {
            g->glGetProgramiv(ps[i], GL_COMPUTE_WORK_GROUP_SIZE, v);
            GLint nu = 0;
            g->glGetProgramiv(ps[i], GL_ACTIVE_UNIFORMS, &nu);
            fprintf(stderr, "epd-gpu: gpe %s: work group %dx%dx%d, %d uniforms\n", nm[i], v[0], v[1], v[2], nu);
        }
        gl_check(g, "setup");
    }
    g->glUseProgram(g->prog);
    g->x.on = 1;
    epd_gpu_gpe_upload(g, e, (epd_rect){ 0, 0, EPD_W, EPD_H });
    g->glBindFramebuffer(GL_FRAMEBUFFER, 0);
    if (g->glGetError() != GL_NO_ERROR) { fprintf(stderr, "epd-gpu: GL error in the gpe setup\n"); g->x.on = 0; return -1; }
    return 0;
}

static void claim_uniforms(epd_gpu *g, const epd_claim_plan *pl, int probe)
{
    GLuint p = g->x.pclaim;
    g->glUseProgram(p);
    U1I(p, "noat", epd_gpu_gpe_noatom);
    U1I(p, "fm", pl->fm); U1I(p, "split", pl->split); U1I(p, "opm", pl->opm); U1I(p, "R", pl->R);
    U1I(p, "aclamp", pl->aclamp); U1I(p, "snp", pl->snp); U1I(p, "hact", pl->hact); U1I(p, "mt", pl->mt);
    U1I(p, "cs", pl->cs); U1I(p, "cap", pl->cap); U1I(p, "model", pl->model); U1I(p, "npre", pl->npre);
    U1I(p, "hmin", pl->hmin); U1I(p, "hmax", pl->hmax);
    U1U(p, "oa", pl->oa); U1U(p, "od", pl->od); U1U(p, "va", pl->va); U1U(p, "vd", pl->vd);
    U1U(p, "hown", pl->hown); U1U(p, "hf0", pl->hf0); U1U(p, "hf1", pl->hf1);
    GLuint u[EPD_MAX_UPD + 1];
    for (int i = 0; i < EPD_LUT_CUTS; i++) u[i] = pl->mown[i];
    g->glUniform1uiv(g->glGetUniformLocation(p, "mown"), EPD_LUT_CUTS, u);
    for (int i = 0; i < 32; i++) u[i] = pl->rail[i];
    g->glUniform1uiv(g->glGetUniformLocation(p, "rail"), 32, u);
    if (pl->npre) {
        for (int i = 0; i <= EPD_MAX_UPD; i++) u[i] = pl->pre[i];
        g->glUniform1uiv(g->glGetUniformLocation(p, "pre"), EPD_MAX_UPD + 1, u);
        GLint rn[(EPD_RUN_SLOTS + 1) * 4];
        for (int i = 0; i <= pl->npre; i++) for (int k = 0; k < 4; k++) rn[i * 4 + k] = pl->rn[i][k];
        g->glUniform4iv(g->glGetUniformLocation(p, "rn"), pl->npre + 1, rn);
    }
    U1I(p, "probe", probe);
}

/* a lazy plan (too few free slots for every cut): a probe pass finds which
 * cuts the pixels want and where first (raster order); the engine hands out
 * the slots in that order (epd_claim_take_slots), as its own pass would */
static int probe_slots(epd_gpu *g, const epd_engine *e, epd_claim_plan *pl, int timeout_ms)
{
    claim_uniforms(g, pl, 1);
    res_init(g, 0);
    dispatch(g, g->x.pclaim, pl->r, g->x.th);
    reduce(g, pl->r, g->x.th, 0);
    const uint32_t *res = result(g, timeout_ms);
    if (!res) return -2;
    int cuts[EPD_LUT_CUTS], n = 0;
    for (int c = 0; c < EPD_LUT_CUTS; c++) if (res[53 + c] != 0xffffffffu) cuts[n++] = c;
    for (int i = 1; i < n; i++)                 /* by where they come first */
        for (int j = i; j > 0 && res[53 + cuts[j]] < res[53 + cuts[j - 1]]; j--) { int t = cuts[j]; cuts[j] = cuts[j - 1]; cuts[j - 1] = t; }
    epd_claim_take_slots(e, pl, cuts, n);
    return 0;
}

int epd_gpu_gpe_submit(epd_gpu *g, const epd_engine *e, epd_claim_plan *pl, int timeout_ms)
{
    if (!g->x.on || g->x.cpend) return -1;     /* (one claim at a time: the engine collects first) */
    const epd_rect r = pl->r;
    prof(g, -1);
    if (pl->op_init) {                          /* op_prep: the whole panel first */
        g->glUseProgram(g->x.pop);
        U1I(g->x.pop, "mode", pl->op_init); U1I(g->x.pop, "R", pl->R); U1I(g->x.pop, "old", pl->op_old);
        dispatch(g, g->x.pop, (epd_rect){ 0, 0, EPD_W, EPD_H }, GPE_FY);
    }
    /* the update's target and the tables */
    if (t_set(g, g->x.pclaim, r, pl->tg, pl->gt)) return -1;
    memcpy(g->x.nets[0], pl->neta, 1024);
    memcpy(g->x.nets[1], pl->netd, 1024);
    if (pl->mt) memcpy(g->x.nets[2], pl->mnet, (size_t)EPD_LUT_CUTS * 1024);
    bind(g, GU_N, g->x.N);
    g->glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 1024, pl->mt ? 2 + EPD_LUT_CUTS : 2, GL_RED_INTEGER, GL_BYTE, g->x.nets);
    if (pl->mt && pl->best != g->x.best) {
        bind(g, GU_B, g->x.B);
        g->glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 2 * EPD_MT_D + 1, 1024, GL_RED_INTEGER, GL_UNSIGNED_BYTE, pl->best);
        g->x.best = pl->best;
    }
    prof(g, EPD_GPU_PROF_UPLOAD);
    if (pl->lazy) {
        int pr = probe_slots(g, e, pl, timeout_ms);
        if (pr) return pr;
        prof(g, EPD_GPU_PROF_PROBE);
    }
    claim_uniforms(g, pl, 0);
    g->x.reg = 1;                               /* (its own result region: queries may come before collect) */
    res_init(g, 0);
    dispatch(g, g->x.pclaim, r, g->x.th);
    prof(g, EPD_GPU_PROF_CLAIM);
    reduce(g, r, g->x.th, 0);
    g->x.reg = 0;
    prof(g, EPD_GPU_PROF_REDUCE);
    g->glUseProgram(g->prog);
    /* (after the profile's finish: a fresh fence's round trip is the wait, as
     * the per-claim readback measured it before) */
    if (g->x.cf) g->glDeleteSync(g->x.cf);
    g->x.cf = g->glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    g->glFlush();                               /* (started now, not at the next wait) */
    memcpy(g->x.cmown, pl->mown, sizeof g->x.cmown);
    g->x.cpend = 1;
    return g->glGetError() == GL_NO_ERROR ? 0 : -1;
}

/* mark: the profile starts here (an asynchronous collect; the synchronous
 * claim's goes on from its submit) */
static int collect(epd_gpu *g, epd_claim_out *out, int timeout_ms, int mark)
{
    if (!g->x.on || !g->x.cpend) return -1;
    if (mark && epd_gpu_prof_on) prof_t = tnow();       /* (no finish: the wait is what is measured) */
    if (g->x.cf) {
        GLenum w = g->glClientWaitSync(g->x.cf, GL_SYNC_FLUSH_COMMANDS_BIT,
                                       timeout_ms > 0 ? (GLuint64)timeout_ms * 1000000 : 0);
        if (w == GL_TIMEOUT_EXPIRED) return timeout_ms > 0 ? -2 : 1;   /* (hung: cf stays, see epd_gpu_finish) */
        g->glDeleteSync(g->x.cf);
        g->x.cf = 0;
        if (w == GL_WAIT_FAILED) { g->x.cpend = 0; return -1; }
    } else if (epd_gpu_finish(g, timeout_ms > 0 ? timeout_ms : 2000)) return -2;     /* (no fence made) */
    prof(g, EPD_GPU_PROF_WAIT);
    g->x.cpend = 0;
    const uint32_t *res = res_read(g, 1);
    if (!res) return -1;
    prof(g, EPD_GPU_PROF_READ);
    /* the extents as epd_claim_pixels reports them */
    memset(out, 0, sizeof *out);
    for (int c = 0; c < EPD_LUT_CUTS; c++) out->mslot[c] = g->x.cmown[c] ? g->x.cmown[c] - 1 : -1;
    for (int k = 0; k < EPD_CL_OWNERS; k++) {
        const uint32_t *q = res + k * 4;
        if (q[2] == 0xffffffffu) { out->x0[k] = 0; out->x1[k] = -1; out->y0[k] = out->y1[k] = -1; continue; }
        out->x0[k] = (int)q[0]; out->x1[k] = (int)q[1] - 1; out->y0[k] = (int)q[2]; out->y1[k] = (int)q[3] - 1;
    }
    out->preempted = res[48]; out->snapped = res[49]; out->flash_skipped = res[50]; out->matched = res[51]; out->healed = res[52];
    return g->glGetError() == GL_NO_ERROR ? 0 : -1;
}

int epd_gpu_gpe_collect(epd_gpu *g, epd_claim_out *out, int timeout_ms)
{
    return collect(g, out, timeout_ms, 1);
}

void epd_gpu_gpe_cancel(epd_gpu *g)
{
    g->x.cpend = 0;                             /* (cf stays: deleting a hung fence may wait) */
}

int epd_gpu_gpe_claim(epd_gpu *g, const epd_engine *e, epd_claim_plan *pl, epd_claim_out *out, int timeout_ms)
{
    int r = epd_gpu_gpe_submit(g, e, pl, timeout_ms);
    if (r) { g->x.cpend = 0; return r; }
    if ((r = collect(g, out, timeout_ms > 0 ? timeout_ms : 1, 0))) g->x.cpend = 0;
    return r;
}

void epd_gpu_gpe_finish(epd_gpu *g, const epd_engine *e, uint64_t slots)
{
    if (!g->x.on || !slots) return;
    int x0 = EPD_W, y0 = EPD_H, x1 = 0, y1 = 0;
    for (int s = 0; s < EPD_MAX_UPD; s++) {
        const epd_rect *r = &e->run[s].r;
        if (!(slots >> s & 1) || r->w <= 0) continue;
        if (r->x < x0) x0 = r->x;
        if (r->y < y0) y0 = r->y;
        if (r->x + r->w > x1) x1 = r->x + r->w;
        if (r->y + r->h > y1) y1 = r->y + r->h;
    }
    if (x1 <= x0 || y1 <= y0) return;
    prof(g, -1);
    g->glUseProgram(g->x.pfin);
    g->glUniform2ui(g->glGetUniformLocation(g->x.pfin, "fin"), (GLuint)slots, (GLuint)(slots >> 32));
    dispatch(g, g->x.pfin, (epd_rect){ x0, y0, x1 - x0, y1 - y0 }, GPE_FY);
    g->glUseProgram(g->prog);
    prof(g, EPD_GPU_PROF_FIN);
}

void epd_gpu_gpe_read(epd_gpu *g, uint8_t *level, uint8_t *target, uint8_t *busy, uint8_t *op, int16_t *dc, uint8_t *stale)
{
    size_t n = (size_t)EPD_W * EPD_H;
    uint32_t *q = malloc(n * 16);
    int32_t *d = (int32_t *)q;
    if (!q) return;
    g->glBindFramebuffer(GL_FRAMEBUFFER, g->x.fbs);
    g->glReadPixels(0, 0, EPD_W, EPD_H, GL_RGBA_INTEGER, GL_UNSIGNED_INT, q);
    for (size_t i = 0; i < n; i++) {
        uint32_t v = q[i * 4];
        level[i] = (uint8_t)v; target[i] = (uint8_t)(v >> 8); busy[i] = (uint8_t)((v >> 16) & 127); op[i] = (uint8_t)(v >> 24);
        stale[i] = (uint8_t)((v >> 23) & 1);
    }
    g->glBindFramebuffer(GL_FRAMEBUFFER, g->x.fbd);
    g->glReadPixels(0, 0, EPD_W, EPD_H, GL_RGBA_INTEGER, GL_INT, d);
    for (size_t i = 0; i < n; i++) dc[i] = (int16_t)d[i * 4];
    g->glBindFramebuffer(GL_FRAMEBUFFER, 0);
    free(q);
}

void epd_gpu_gpe_drive(epd_gpu *g, int on)
{
    g->x.drive = g->x.on && on;
}

int epd_gpu_gpe_query(epd_gpu *g, const epd_pend *p, uint64_t *mask, int *nonbw, int timeout_ms)
{
    *mask = 0; *nonbw = 0;
    if (!g->x.on) return -1;
    const epd_rect r = p->r;
    prof(g, -1);
    if (t_set(g, g->x.pquery, r, p->target, p->gt)) return -1;
    g->glUseProgram(g->x.pquery);
    U1I(g->x.pquery, "full", p->full);
    res_init(g, 1);
    dispatch(g, g->x.pquery, r, g->x.th);
    reduce(g, r, g->x.th, 1);
    g->glUseProgram(g->prog);
    const uint32_t *res = result(g, timeout_ms);
    if (!res) return -2;
    *mask = res[0] | (uint64_t)res[1] << 32;
    *nonbw = res[2] != 0;
    prof(g, EPD_GPU_PROF_QUERY);
    return g->glGetError() == GL_NO_ERROR ? 0 : -1;
}

/* ---- stage 5: update targets drawn on another display ----
 * epd-ingest draws a target into a layer (panel-sized RGBA8, 4 levels per
 * texel, epd-gtarget.h) on its own display; here the layer's dma-buf is a
 * texture the claim and query passes read directly (tg in the shaders),
 * instead of an upload of the CPU's copy into T (a full-panel R8 upload per
 * claim and per query before; 2.3-2.6 ms on the A509). */
int close(int fd);                      /* (not unistd.h: its link is taken here) */

static int has_egl_ext(EGLDisplay d, const char *e)
{
    const char *l = eglQueryString(d, EGL_EXTENSIONS);
    size_t n = strlen(e);
    for (const char *p = l; p && (p = strstr(p, e)); p += n)
        if ((p == l || p[-1] == ' ') && (p[n] == ' ' || !p[n])) return 1;
    return 0;
}

static int layer_import(epd_gpu *g, int i, const epd_gt_buf *b)
{
    static const EGLint pa[4][5] = {
        { EGL_DMA_BUF_PLANE0_FD_EXT, EGL_DMA_BUF_PLANE0_OFFSET_EXT, EGL_DMA_BUF_PLANE0_PITCH_EXT,
          EGL_DMA_BUF_PLANE0_MODIFIER_LO_EXT, EGL_DMA_BUF_PLANE0_MODIFIER_HI_EXT },
        { EGL_DMA_BUF_PLANE1_FD_EXT, EGL_DMA_BUF_PLANE1_OFFSET_EXT, EGL_DMA_BUF_PLANE1_PITCH_EXT,
          EGL_DMA_BUF_PLANE1_MODIFIER_LO_EXT, EGL_DMA_BUF_PLANE1_MODIFIER_HI_EXT },
        { EGL_DMA_BUF_PLANE2_FD_EXT, EGL_DMA_BUF_PLANE2_OFFSET_EXT, EGL_DMA_BUF_PLANE2_PITCH_EXT,
          EGL_DMA_BUF_PLANE2_MODIFIER_LO_EXT, EGL_DMA_BUF_PLANE2_MODIFIER_HI_EXT },
        { EGL_DMA_BUF_PLANE3_FD_EXT, EGL_DMA_BUF_PLANE3_OFFSET_EXT, EGL_DMA_BUF_PLANE3_PITCH_EXT,
          EGL_DMA_BUF_PLANE3_MODIFIER_LO_EXT, EGL_DMA_BUF_PLANE3_MODIFIER_HI_EXT } };
    EGLint a[64], k = 0;
    a[k++] = EGL_WIDTH; a[k++] = b->w;
    a[k++] = EGL_HEIGHT; a[k++] = b->h;
    a[k++] = EGL_LINUX_DRM_FOURCC_EXT; a[k++] = (EGLint)b->fourcc;
    for (int p = 0; p < b->planes && p < 4; p++) {
        a[k++] = pa[p][0]; a[k++] = b->fd[p];
        a[k++] = pa[p][1]; a[k++] = b->offset[p];
        a[k++] = pa[p][2]; a[k++] = b->pitch[p];
        if (b->modifier != 0x00ffffffffffffffull) {             /* DRM_FORMAT_MOD_INVALID: implicit */
            a[k++] = pa[p][3]; a[k++] = (EGLint)(b->modifier & 0xffffffffu);
            a[k++] = pa[p][4]; a[k++] = (EGLint)(b->modifier >> 32);
        }
    }
    a[k++] = EGL_NONE;
    EGLImageKHR img = g->eglCreateImageKHR(g->dpy, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, NULL, a);
    if (img == EGL_NO_IMAGE_KHR) {
        fprintf(stderr, "epd-gpu: target layer %d: import failed (0x%x; %.4s modifier 0x%llx, %d planes)\n", i + 1,
                eglGetError(), (const char *)&b->fourcc, (unsigned long long)b->modifier, b->planes);
        return -1;
    }
    GLuint t;
    g->glActiveTexture(GL_TEXTURE0 + GU_L);
    g->glGenTextures(1, &t);
    g->glBindTexture(GL_TEXTURE_2D, t);
    g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    g->glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, img);
    g->x.lay[i].img = img; g->x.lay[i].tex = t;
    if (g->glGetError() != GL_NO_ERROR || !(g->x.lay[i].fbo = fbo_of(g, t))) {
        fprintf(stderr, "epd-gpu: target layer %d: not usable as a texture\n", i + 1);
        return -1;
    }
    return 0;
}

int epd_gpu_gpe_provider(epd_gpu *g, const epd_gt_provider *pv)
{
    g->x.pv = NULL;
    if (!pv) return 0;
    if (!g->x.on || pv->n < 1 || pv->n > EPD_GT_MAX || !g->eglCreateImageKHR || !g->glEGLImageTargetTexture2DOES) return -1;
    if (!g->x.create_sync && !g->x.wait_sync) {
        if (has_egl_ext(g->dpy, "EGL_ANDROID_native_fence_sync") && has_egl_ext(g->dpy, "EGL_KHR_wait_sync")) {
            g->x.create_sync = (void *)eglGetProcAddress("eglCreateSyncKHR");
            g->x.wait_sync = (void *)eglGetProcAddress("eglWaitSyncKHR");
            g->x.destroy_sync = (void *)eglGetProcAddress("eglDestroySyncKHR");
            if (!g->x.wait_sync || !g->x.destroy_sync) g->x.create_sync = NULL;
        }
    }
    for (int i = 0; i < pv->n; i++) {
        epd_gt_buf b;
        if (g->x.lay[i].img) continue;                  /* (imported before: same layers) */
        if (pv->buf(pv->ctx, i + 1, &b) || layer_import(g, i, &b)) {
            g->glBindFramebuffer(GL_FRAMEBUFFER, 0);
            while (g->glGetError() != GL_NO_ERROR) ;
            return -1;
        }
    }
    g->glBindFramebuffer(GL_FRAMEBUFFER, 0);
    g->x.pv = pv;
    return 0;
}

/* the producer's fence of layer gt: waited for on our GPU (queued, the CPU
 * goes on), or on the CPU without native fences here; -1 not signalled */
static int gt_wait(epd_gpu *g, int gt)
{
    int fd = g->x.pv->fence(g->x.pv->ctx, gt);
    if (fd < 0) return 0;
    if (g->x.create_sync) {
        const EGLint at[] = { EGL_SYNC_NATIVE_FENCE_FD_ANDROID, fd, EGL_NONE };
        EGLSyncKHR s = g->x.create_sync(g->dpy, EGL_SYNC_NATIVE_FENCE_ANDROID, at);   /* (owns fd from here) */
        if (s != EGL_NO_SYNC_KHR) {
            g->x.wait_sync(g->dpy, s, 0);
            g->x.destroy_sync(g->dpy, s);
            return 0;
        }
    }
    struct pollfd pf = { fd, POLLIN, 0 };
    int r = poll(&pf, 1, 2000);
    close(fd);
    return r == 1 ? 0 : -1;
}

static int gt_ok(const epd_gpu *g, int gt) { return g->x.pv && gt >= 1 && gt <= g->x.pv->n && g->x.lay[gt - 1].tex; }

/* the target of a claim / query run by program p: the CPU's copy uploaded
 * into T, or (gt) the layer bound after the producer's fence */
static int t_set(epd_gpu *g, GLuint p, epd_rect r, const uint8_t *tg, int gt)
{
    g->glUseProgram(p);
    if (!gt_ok(g, gt)) {                                /* the CPU's copy */
        if (!tg) return -1;
        bind(g, GU_T, g->x.T);
        g->glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, r.w, r.h, GL_RED_INTEGER, GL_UNSIGNED_BYTE, tg);
        U1I(p, "tl", 0);
        return 0;
    }
    if (gt_wait(g, gt)) return -1;
    bind(g, GU_L, g->x.lay[gt - 1].tex);
    U1I(p, "tl", 1);
    return 0;
}

int epd_gpu_gpe_target_read(epd_gpu *g, int gt, epd_rect r, uint8_t *out, int timeout_ms)
{
    if (!gt_ok(g, gt) || r.w <= 0 || r.h <= 0 || r.x < 0 || r.y < 0 || r.x + r.w > EPD_W || r.y + r.h > EPD_H) return -1;
    if (gt_wait(g, gt)) return -1;
    if (epd_gpu_finish(g, timeout_ms)) return -2;
    int t0 = r.x / 4, t1 = (r.x + r.w + 3) / 4;
    uint8_t *row = malloc((size_t)(t1 - t0) * 4 * r.h);
    if (!row) return -1;
    g->glBindFramebuffer(GL_FRAMEBUFFER, g->x.lay[gt - 1].fbo);
    g->glReadPixels(t0, r.y, t1 - t0, r.h, GL_RGBA, GL_UNSIGNED_BYTE, row);
    g->glBindFramebuffer(GL_FRAMEBUFFER, 0);
    for (int y = 0; y < r.h; y++) memcpy(out + (size_t)y * r.w, row + (size_t)y * (t1 - t0) * 4 + (r.x - t0 * 4), (size_t)r.w);
    free(row);
    return g->glGetError() == GL_NO_ERROR ? 0 : -1;
}

/* ---- staggered cut steps (epd-stagger.h) -------------------------------
 * One invocation per wire word (8 pixels, 32 x WY per workgroup, no barrier).
 * Every frame: the word's hot texel (8 x 16 bit: the step's phase, length,
 * start; level class, settled, payback due), tested two pixels a uint (16-bit
 * lanes) for "driving" and "step done". The pixels of this frame's group come
 * from a mask table (GM, per G and frame); with the word-stratified blue noise
 * (epd-bluenoise.h) every word has the same number of them (8 / G), so the
 * loops over them cost the same in every lane. Such a pixel reads its target
 * byte; only one with something to do (its hot bits: a change, the model short
 * of the rail, payback due) reads and writes its cold state, 4 bytes in the
 * plane of its octant (cplane). One program per mode (ctl, pay on, dither,
 * drain, direct: compile-time; a uniform branch per pixel cost the A509 ~3x).
 * The frame goes straight into the target (image stores, the template ORed
 * in) once the target was drawn whole; else the words go to WD and the
 * template pass (STAG) ORs them in. Same integer steps as epd-stagger.c px.
 * A509 GPU time, a scrolled full page : A2/2 ~12 ms, A2/4 ~8.4,
 * A2/8 ~6.7, a still page ~4.3. Unit 16 and up are not there for a compute
 * stage (GLES 3.1 has 16): GM sits on 3, a unit the main shaders leave free. */
enum { GS_H = 10, GS_C, GS_T, GS_WD, GS_OC, GS_LC, GS_GM = 3, GS_AT = 2 };
#define CPLANE_SH 8
#define CPLANE (1 << CPLANE_SH)                /* the cold planes' stride: >= GW, a power of two */
_Static_assert(GW <= CPLANE, "a cold plane holds a row of words");
int epd_gpu_stag_timer;
/* the tile skip (-1: GLASSINE_STAG_TSKIP, default 1; 0 off): a workgroup tile idle for
 * its G frames since its last work, its targets and state unchanged since
 * G frames, and the pass's params the same, writes its no-drive output and
 * nothing else (its state as it is: every pixel there would skip) */
int epd_gpu_stag_tskip = -1;
/* LEAD (-1: GLASSINE_STAG_LEAD, default 0; 1 on, 0 off; a workaround, opt-in):
 * the A509 loses one wave's stores in the first workgroup row of a compute
 * dispatch after a program switch across a CPU wait under foreign GPU load
 * . Every stagger compute pass gets one leading
 * workgroup row (gl_WorkGroupID.y 0) that runs the first real row's code
 * and stores into scratch rows below the state (EPD_H + its local row);
 * the real rows follow one workgroup row down. The scratch rows exist only
 * when it was on at epd_gpu_stag_open (sg.lr); a later "on" waits for the
 * next open */
int epd_gpu_stag_lead = -1;
/* cells in rank order (-1: GLASSINE_STAG_CREMAP, default 1; 0: in place, a timing
 * comparison): with SWY 64 the word pass runs a tile's cells sorted by their
 * rank, so a wave holds one group; the same results */
int epd_gpu_stag_cremap = -1;
/* lanes with per-pixel groups (blue noise): the NOHOIST, NJ 8 shape (stag_prog;
 * GLASSINE_STAG_LPX=0: the hoisted one, a timing comparison); the same results */
static int epd_gpu_stag_lpx(void)
{
    static int on = -1;
    if (on < 0) {
        const char *v = getenv("GLASSINE_STAG_LPX");
        on = v ? atoi(v) > 0 : 1;
    }
    return on;
}
double epd_gpu_stag_ms[3];
/* (the passes' stores: ST(image, p, v); LEAD: the leading row's into its
 * scratch row, in every program of the stagger's compute path) */
#define STAG_ST \
    "#ifdef LEAD\n" \
    "#define ST(I, p, v) imageStore(I, lead ? ivec2((p).x, " STR(EPD_H) " + int(gl_LocalInvocationID.y)) : (p), v)\n" \
    "#else\n" \
    "#define ST(I, p, v) imageStore(I, p, v)\n" \
    "#endif\n"
/* (HEAPSTATE, timing/test only (R4): C and T are unorm views of heap
 * dma-bufs; a texel back to its bytes, exact for every value (heap-import-probe) */
#define STAG_HEAP \
    "#ifdef HEAPSTATE\n" \
    "uint heap_u32(highp vec4 v) { uvec4 u = uvec4(v * 255.0 + 0.5); return u.x | u.y << 8 | u.z << 16 | u.w << 24; }\n" \
    "uvec2 heap_u64(highp vec4 v) { uvec4 u = uvec4(v * 65535.0 + 0.5); return uvec2(u.x | u.y << 16, u.z | u.w << 16); }\n" \
    "#endif\n"
static const char *stag_cs =
    "#version 310 es\n"
    "precision highp int; precision highp uimage2D; precision highp usampler2D; precision highp image2D;\n"
    "layout(local_size_x = WX, local_size_y = WY) in;\n"
    "#ifdef HEAPSTATE\n"
    "uniform usampler2D HS, OC, GM;\nuniform highp sampler2D CS, TT;\n"
    "#else\n"
    "uniform usampler2D HS, CS, TT, OC, GM;\n"
    "#endif\n"
    "uniform int f6, n, pay, R, gb, f62, jlo, jhi, gG, gK, pcap, fm;\n"
    "#ifdef G1C\n"             /* (G 1 = n 1 at compile time: A2/1's steps folded) */
    "#define gG 1\n#define n 1\n#define jlo 0\n#define jhi 7\n"
    "#endif\n"
    "#ifdef G2C\n"             /* (stripes at G 2, n 1 compiled in: a timing probe) */
    "#define gG 2\n#define n 1\n#define jlo 0\n#define jhi 7\n"
    "#endif\n"
    "#if HTOP\n"
    "uniform int htop;\n"
    "#endif\n"
    "#ifdef CELLS\n"          /* (the cells' shape: log2 of a cell's width, height in pixels; GM = their table) */
    "uniform int cwl, chl;\n"
    "#endif\n"
    "#if DIRECT\n"
    "uniform usampler2D TP;   /* the template (EPD_LINE x EPD_LINES words) */\n"
    "uniform int pw;          /* the target seen flat: words per wire line; 0 = 2D */\n"
    "layout(rgba8, binding = 3) writeonly uniform image2D F;\n"
    "#endif\n"
    "layout(rgba32ui, binding = 0) writeonly uniform uimage2D H;\n"
    "#ifdef HEAPSTATE\n"
    "layout(rgba8, binding = 1) writeonly uniform highp image2D C;\n"
    "#else\n"
    "layout(r32ui, binding = 1) writeonly uniform uimage2D C;\n"
    "#endif\n"
    "layout(r32ui, binding = 2) writeonly uniform uimage2D O;\n"
    "#ifdef LANES\n"
    "layout(rgba8, binding = 5) writeonly uniform highp image2D A;\n"
    "#endif\n"
    "#if TSKIP\n"
    "layout(r32ui, binding = 4) writeonly uniform highp uimage2D AT;\n"
    "uniform highp usampler2D ATS;\n"
    "uniform int fid;\n"
    "#endif\n"
    "#ifdef COUNT\n"
    "layout(std430, binding = 0) buffer CNT { uint cn[6]; };\n"
    "#endif\n"
    "#ifdef XLIST\n"             /* (X*: timing: a per-tile list's appends, GLASSINE_STAG_XLIST) */
    "layout(std430, binding = 1) buffer XL { uint xl[]; };\n"
    "#endif\n"
    STAG_ST
    STAG_HEAP
    /* a pixel on its group's frame with something to do: the next step (tc:
     * the target white); its phase bits in the word (0: none) */
    "#ifdef XTRIV\n"      /* (X*: timing: the bare -1|0|+1 rail step, K idle; no payback) */
    "uint stp(uint i, inout uint h, inout uint cv, uint tc) {\n"
    "  int op = int((cv >> 5) & 63u), k = int((cv >> 12) & 15u), dc = int(cv) >> 16;\n"
    "  int need = tc != 0u ? R - op : op;\n"
    "  bool pl = (h & 3u) != 0u;\n"
    "  int nn = need > 0 && !(pl && k >= gK) ? 1 : 0;\n"
    "  int sn = tc != 0u ? nn : -nn;\n"
    "  op += sn; dc += sn;\n"
    "  k = nn != 0 ? (pl ? k + 1 : 1) : 0;\n"
    "  h = (nn != 0 ? (1u + tc) | 4u | uint(f6) << 6 : 0u) | tc << 12 | (need - nn == 0 ? 1u : 0u) << 14;\n"
    "  cv = (tc != 0u ? 30u : 0u) | uint(op) << 5 | 2048u | uint(k) << 12 | (uint(dc) & 65535u) << 16;\n"
    "  return nn != 0 ? (1u + tc) << (14u - 2u * i) : 0u;\n"
    "}\n"
    "#else\n"
    "uint stp(uint i, inout uint h, inout uint cv, uint tc) {\n"
    "  int l = int(cv & 31u), op = int((cv >> 5) & 63u), dc = int(cv) >> 16, nn;\n"
    "  uint kk = 0u;\n"
    "#if PRULE\n"
    /* payrule (epd-stagger.h): an odd level = a rail with b payback frames
     * to come; an arrival starting from a rail sets b */
    "  int b = 0;\n"
    "  if ((l & 1) != 0) { if (l < 15) { b = (l + 1) >> 1; l = 0; } else { b = (31 - l) >> 1; l = 30; } }\n"
    "  if (l != (tc != 0u ? 30 : 0)) {\n"
    "    int x = (tc != 0u ? -dc : dc) - (tc != 0u ? R - op : op);\n"
    "    b = (l != 0 && l != 30) || x <= 0 ? 0 : min(x, pcap);\n"
    "#if HTOP\n"
    "    if ((l == 0 || l == 30) && (tc != 0u ? R - op : op) < R) b = max(b, min(htop, pcap));\n"
    "#endif\n"   /* (the top-up: a reversal) */
    "  }\n"
    "#endif\n"
    "  bool kc = false;\n"
    "#if CTL == 3\n"
    /* the whole cycle (up to G frames, 15: the n field), k: the pulses
     * since the last idle frame (cold bits 12..15), at most gK */
    "  int need = tc != 0u ? R - op : op, gc = min(gG, 15);\n"
    "#if PRULE\n"
    "  nn = min(need + b, gc);\n"
    "#elif PAY\n"
    "  int owe = tc != 0u ? -dc : dc;\n"
    "  nn = min(need > 0 ? need : owe >= pay ? owe : 0, gc);\n"
    "#else\n"
    "  nn = min(max(need, 0), gc);\n"
    "#endif\n"
    "  uint ph0 = h & 3u;\n"
    "  int kin = ph0 != 0u && ((uint(f6) - (h >> 6)) & 63u) == ((h >> 2) & 15u) ? int((cv >> 12) & 15u) : 0;\n"
    "  kc = nn == gG && kin + nn > gK;\n"   /* (the K idle: G 1 cuts nn to 0) */
    "  if (kc) nn = gG - 1;\n"
    "  kk = nn == gG ? uint(kin + nn) : 0u;\n"
    "#elif CTL == 0\n"
    "  nn = l != (tc != 0u ? 30 : 0) ? n : 0;\n"
    "#else\n"
    "  int need = tc != 0u ? R - op : op;\n"
    "#if PRULE\n"
    "  nn = min(need + b, need > 0 || CTL != 2 ? n : min(gG, 15));\n"
    "#elif PAY\n"
    "  int owe = tc != 0u ? -dc : dc;\n"
    "  nn = need > 0 ? min(need, n) : owe >= pay ? min(owe, CTL == 2 ? min(gG, 15) : n) : 0;\n"
    "#else\n"
    "  nn = need > 0 ? min(need, n) : 0;\n"
    "#endif\n"
    "#endif\n"
    "#if CTL == 2\n"
    /* right after the last step's drive frames (no idle yet): a reversal
     * waits for the next group frame, state unchanged */
    "  uint ph0 = h & 3u;\n"
    "  if (ph0 != 0u && ((uint(f6) - (h >> 6)) & 63u) == ((h >> 2) & 15u) && nn != 0 && ph0 != 1u + tc) return 0u;\n"
    "#endif\n"
    "  uint st = nn != 0 || l != (tc != 0u ? 30 : 0) ? 1u : (cv >> 11) & 1u;\n"
    "#if PRULE\n"
    "  b -= max(nn - (tc != 0u ? R - op : op), 0);\n"
    "  uint lc = b > 0 ? 2u : tc, le = b <= 0 ? (tc != 0u ? 30u : 0u) : tc != 0u ? uint(31 - 2 * b) : uint(2 * b - 1);\n"
    "#else\n"
    "  uint lc = tc, le = tc != 0u ? 30u : 0u;\n"
    "#endif\n"
    "  int sn = tc != 0u ? nn : -nn;\n"
    "  op = clamp(op + sn, 0, R);\n"
    "  dc = clamp(dc + sn, -" STR(EPD_STAG_DCMAX) ", " STR(EPD_STAG_DCMAX) ");\n"
    "  uint set = lc == tc && (tc != 0u ? op == R : op == 0) ? 1u : 0u;\n"
    "#if CTL && PAY\n"
    "  uint due = (tc != 0u ? -dc : dc) >= pay ? 1u : 0u;\n"
    "#else\n"
    "  uint due = 0u;\n"
    "#endif\n"
    "  h = (nn != 0 ? (1u + tc) | uint(nn) << 2 | uint(f6) << 6 : 0u) | lc << 12 | set << 14 | due << 15;\n"
    "  cv = le | uint(op) << 5 | st << 11 | kk << 12 | (uint(dc) & 65535u) << 16;\n"
    "  return nn != 0 ? (1u + tc) << (14u - 2u * i) : 0u;\n"
    "}\n"
    "#endif\n"
    "void main() {\n"
    "#ifdef LEAD\n"               /* (the leading row: the first real row's words, every lane one) */
    "  bool lead = gl_WorkGroupID.y == 0u;\n"
    "  ivec2 c = ivec2(gl_GlobalInvocationID.xy) - ivec2(0, lead ? 0 : WY);\n"
    "  if (lead) c.x = min(c.x, " STR(GW) " - 1);\n"
    "#else\n"
    "  ivec2 c = ivec2(gl_GlobalInvocationID.xy);\n"
    "#endif\n"
    /* CELLR (cells, SWY 64: a workgroup = one 64 x 64 tile): the tile's cells
     * in the order of their ranks (epd_stag_cells' inverse, GM .g), so the
     * invocations of a group are consecutive (N / G cells each): a wave one
     * group; place cs_ = the invocation's cell's rank, its words then rows
     * inside the cell. The leading row (LEAD) as it was (scratch stores) */
    "#ifdef CELLR\n"
    "  uint cs_ = gl_LocalInvocationIndex >> uint(cwl - 3 + chl);\n"
    "#ifdef LEAD\n  if (!lead)\n#endif\n"
    "  {\n"
    "    int ww = cwl - 3, tx = 6 - cwl;\n"
    "    uint wi = gl_LocalInvocationIndex & ((1u << uint(ww + chl)) - 1u);\n"
    "    ivec2 wg = c - ivec2(gl_LocalInvocationID.xy);\n"
    "    int ci = int(texelFetch(GM, ivec2(((wg.x >> 3) & ((1 << cwl) - 1)) << tx | int(cs_ & ((1u << uint(tx)) - 1u)),\n"
    "                                      ((wg.y >> 6) & ((1 << chl) - 1)) << (6 - chl) | int(cs_ >> uint(tx))), 0).g);\n"
    "    c = wg + ivec2((ci & ((1 << tx) - 1)) << ww | int(wi & ((1u << uint(ww)) - 1u)), (ci >> tx) << chl | int(wi >> uint(ww)));\n"
    "  }\n"
    "#endif\n"
    "  if (c.x >= " STR(GW) " || c.y >= " STR(EPD_H) ") return;\n"
    "  uint w = 0u;\n"
    "#if DIRECT && !defined(NOHOIST)\n"
    "  uint tp_ = texelFetch(TP, ivec2(" STR(DATA_COL0) " + c.x, " STR(DATA_LINE0) " + " STR(EPD_H) " - 1 - c.y), 0).r;\n"
    "#endif\n"
    "#if TSKIP\n"
    /* the whole workgroup alike (its tile): idle = no work in the last G
     * frames (every group's pixels skipped) and no change (the mark pass
     * stamps a changed tile as if it had work) */
    "#ifdef LEAD\n"
    "  ivec2 tl = ivec2(gl_WorkGroupID.x, lead ? 0u : gl_WorkGroupID.y - 1u);\n"
    "#else\n"
    "  ivec2 tl = ivec2(gl_WorkGroupID.xy);\n"
    "#endif\n"
    "#ifdef TS_NOIDLE\n"
    "  bool idle = false;\n"
    "#else\n"
    "  bool idle = int(uint(fid) - texelFetch(ATS, tl, 0).r) > gG;\n"
    "#endif\n"
    "  bool busy = false;\n"
    "#ifdef TS_NEST\n"
    "  if (!idle) {\n"
    "#else\n"
    "  if (idle) {\n"
    "#if DIRECT\n"
    "    int line = " STR(DATA_LINE0) " + " STR(EPD_H) " - 1 - c.y, col = " STR(DATA_COL0) " + c.x, k = line * pw + col;\n"
    "#ifdef NOHOIST\n"
    "    uint v = texelFetch(TP, ivec2(col, line), 0).r;\n"
    "#else\n"
    "    uint v = tp_;\n"
    "#endif\n"
    "#ifdef LEAD\n"               /* (F, the scanout, has no scratch rows: the leading row's word into O's) */
    "    if (lead) ST(O, c, uvec4(v)); else\n"
    "#endif\n"
    "    imageStore(F, pw > 0 ? ivec2(k % " STR(FLAT_W) ", k / " STR(FLAT_W) ") : ivec2(col, line), unpackUnorm4x8(v));\n"
    "#else\n"
    "    ST(O, c, uvec4(0u));\n"
    "#endif\n"
    "    return;\n"
    "  }\n"
    "  {\n"
    "#endif\n"
    "#else\n"
    "  {\n"
    "#endif\n"
    /* STRIPE (64 px column stripes, epd_stag_srank): a word's 8 pixels in one
     * group, its stripe's; a workgroup one stripe wide (WX 8): the waves
     * of the other groups' stripes skip the cold path whole */
    /* BLOCKS (stripes cut into 64 x 64 blocks, epd_stag_brank: the 16 x 16
     * Bayer index of the block, a = bx ^ by, b = by): a workgroup (8 words x
     * SWY rows, SWY dividing 64) inside one block, still one group */
    /* CELLS (epd_stag_cells): a word's cell's rank, from its
     * place (CELLR) or the table (GM .r) */
    "#if STRIPE && defined(CELLS)\n"
    "#ifdef CELLR\n"
    "  uint crk = cs_ << uint(cwl + chl);\n"
    "#else\n"
    "  uint crk = texelFetch(GM, ivec2((c.x >> (cwl - 3)) & 63, (c.y >> chl) & 63), 0).r;\n"
    "#endif\n"
    "  uvec2 gm = uvec2(int((crk * uint(gG)) >> 12) == fm ? 0xFFu : 0u, 0u);\n"
    "#elif STRIPE && defined(BLOCKS)\n"
    "  uint ba = uint((c.x >> 3) ^ (c.y >> 6)) & 15u, bb = uint(c.y >> 6) & 15u;\n"
    "  uint bi = (ba & 1u) << 7 | (bb & 1u) << 6 | (ba & 2u) << 4 | (bb & 2u) << 3 | (ba & 4u) << 1 | (bb & 4u) | (ba & 8u) >> 2 | (bb & 8u) >> 3;\n"
    "  uvec2 gm = uvec2(int(((bi << 4) * uint(gG)) >> 12) == fm ? 0xFFu : 0u, 0u);\n"
    "#elif STRIPE\n"
    "  uvec2 gm = uvec2(int((((bitfieldReverse(uint(c.x >> 3) & 31u) >> 27) << 7) * uint(gG)) >> 12) == fm ? 0xFFu : 0u, 0u);\n"
    "#else\n"
    "  uvec2 gm = texelFetch(GM, ivec2(c.x & 7, gb + (c.y & 63)), 0).rg;\n"
    "#endif\n"
    "#ifdef XHALF\n"            /* (X*: timing: the other groups' stripes do nothing, Fable round 5 L1) */
    "  if (gm.x == 0u) return;\n"
    "#endif\n"
    "  uvec4 q0 = texelFetch(HS, c, 0);\n"
    "  uint q[4] = uint[4](q0.x, q0.y, q0.z, q0.w);\n"
    /* every fetch at once, up front (the cold states speculatively: the A509
     * pass is latency-bound, not bandwidth-bound: each dependent round of
     * fetches cost ~1 ms a full panel); NOHOIST: where they are used */
    "#ifndef NOHOIST\n"
    "#ifndef LANES\n"
    "#ifdef HEAPSTATE\n"
    "  uvec2 t2_ = heap_u64(texelFetch(TT, c, 0));\n"
    "#else\n"
    "  uvec2 t2_ = texelFetch(TT, c, 0).rg;\n"
    "#endif\n"
    "#endif\n"
    "  uvec4 oc4_ = texelFetch(OC, ivec2(c.x & 7, c.y & 63), 0);\n"
    /* COLDLIN (timing/test only): the cold state linear, a
     * pixel a texel (x = word * 8 + pixel), all 8 of a word read (one line):
     * the one-state design's D. Not with SBIT; the default unchanged */
    "#ifdef COLDLIN\n"
    "  uint cv_[8];\n"
    "#ifdef HEAPSTATE\n"
    "  for (int k = 0; k < 8; k++) cv_[k] = heap_u32(texelFetch(CS, ivec2(c.x * 8 + k, c.y), 0));\n"
    "#else\n"
    "  for (int k = 0; k < 8; k++) cv_[k] = texelFetch(CS, ivec2(c.x * 8 + k, c.y), 0).r;\n"
    "#endif\n"
    "#else\n"
    "  uint cv_[NJ];\n"
    "  for (int k = 0; k < NJ; k++) cv_[k] = texelFetch(CS, ivec2(min(jlo + k, 7) << " STR(CPLANE_SH) " | c.x, c.y), 0).r;\n"
    "#endif\n"
    "#endif\n"

    /* two pixels a uint: age = f6 - start (mod 64), driving = ph && age < n,
     * done = !ph || age > n (bias bits 6 and 7 keep the lanes apart) */
    "  uint sm = 0u, f2 = uint(f62) | 0x00400040u;\n"
    "  for (int j = 0; j < 4; j++) {\n"
    "    uint x = q[j], ph = x & 0x00030003u, k = (x >> 2) & 0x000F000Fu;\n"
    "    uint age = (f2 - ((x >> 6) & 0x003F003Fu)) & 0x003F003Fu;\n"
    "    uint lt = ~((age | 0x00800080u) - k) & 0x00800080u;\n"
    "    uint gt = ~((k | 0x00800080u) - age) & 0x00800080u;\n"
    "    uint nz = (ph | (ph >> 1)) & 0x00010001u;\n"
    "    uint act = lt & (nz << 7), pa = ph & ((act >> 7) | (act >> 6));\n"
    "    w |= (pa & 3u) << (14 - 4 * j) | (pa >> 16) << (12 - 4 * j);\n"
    "#if CTL >= 2\n"
    "    uint st = (~nz & 0x00010001u) | ((~lt & 0x00800080u) >> 7);   /* age >= n */\n"
    "#else\n"
    "    uint st = (~nz & 0x00010001u) | (gt >> 7);\n"
    "#endif\n"
    "    sm |= ((st & 1u) | ((st >> 15) & 2u)) << (2 * j);\n"
    "  }\n"
    "  sm &= gm.x;\n"
    "#ifdef NOSM\n  sm = 0u;\n#endif\n"
    "  if (sm != 0u) {\n"
    /* LANES (stagger_state=lanes, ; epd-stagger.c px_lane): T
     * is the engine's (folded: 4 rows a texel row), its bytes target 5 |
     * tog | arm | evict; a parked lane (level class 3) of the group's frame
     * with arm (no evict) takes D's state (its hot bits as stag_load_cs makes
     * them) and decides with the rest; a lane done with evict parks, its
     * final D (payrule's budget dropped); parked lanes decide nothing; a word
     * all parked with an evict acks (A: 1 | tog << 1) */
    "#ifdef LANES\n"
    "    uvec2 t2r = heap_u64(texelFetch(TT, ivec2((c.y & 3) * 240 + c.x, c.y >> 2), 0));\n"
    "    uint arm8 = 0u, ev8 = 0u, pkm = 0u;\n"
    "    for (int j = 0; j < 4; j++) {\n"
    "      uint pc = (q[j] >> 12) & (q[j] >> 13) & 0x00010001u;\n"
    "      pkm |= (pc & 1u) << (2 * j) | (pc >> 16) << (2 * j + 1);\n"
    "    }\n"
    "    for (uint i = 0u; i < 8u; i++) {\n"
    "      uint tb = ((i < 4u ? t2r.x : t2r.y) >> (8u * (i & 3u))) & 255u;\n"
    "      arm8 |= ((tb >> 6) & 1u) << i;\n"
    "      ev8 |= ((tb >> 7) & 1u) << i;\n"
    "    }\n"
    "    uint lm = sm & pkm & arm8 & ~ev8, em = sm & ~pkm & ev8;\n"
    "    for (uint m = lm | em; m != 0u; m &= m - 1u) {\n"
    "      uint i = uint(findLSB(m)), sh = (i & 1u) << 4, hn;\n"
    "      uint cv = heap_u32(texelFetch(CS, ivec2(c.x * 8 + int(i), c.y), 0));\n"
    "      if (((lm >> i) & 1u) != 0u) {\n"
    "        int l = int(cv & 31u), op = int((cv >> 5) & 63u), dc = int(cv) >> 16;\n"
    "        uint lv = l == 0 ? 0u : l == 30 ? 1u : 2u;\n"
    "        uint set = (lv == 0u ? op == 0 : lv == 1u ? op == R : false) ? 1u : 0u;\n"
    "#if PAY\n"
    "        uint due = lv < 2u && (lv == 1u ? -dc : dc) >= pay ? 1u : 0u;\n"
    "#else\n"
    "        uint due = 0u;\n"
    "#endif\n"
    "        hn = lv << 12 | set << 14 | due << 15;\n"
    "      } else {\n"
    "        uint l = cv & 31u;\n"
    "        if ((l & 1u) != 0u) l = l < 15u ? 0u : 30u;\n"
    "        ST(C, ivec2(c.x * 8 + int(i), c.y), unpackUnorm4x8((cv & ~31u) | l));\n"
    "        hn = 0x3000u;\n"
    "      }\n"
    "      q[i >> 1u] = (q[i >> 1u] & ~(65535u << sh)) | hn << sh;\n"
    "    }\n"
    "    pkm = (pkm & ~lm) | em;\n"
    "    if (pkm == 0xFFu && ev8 != 0u) {\n"
    "      uint e0 = uint(findLSB(ev8)), tb0 = ((e0 < 4u ? t2r.x : t2r.y) >> (8u * (e0 & 3u))) & 255u;\n"
    "      ST(A, c, vec4(float(1u | ((tb0 >> 5) & 1u) << 1) / 255.0, 0.0, 0.0, 0.0));\n"
    "    }\n"
    "    sm &= ~pkm;\n"
    "#endif\n"
    "#if DRAIN\n"
    "    for (uint m = sm; m != 0u; m &= m - 1u) { uint i = uint(findLSB(m)); q[i >> 1] &= ~(3u << ((i & 1u) << 4)); }\n"
    "#else\n"
    "#ifdef LANES\n"
    "    uvec2 t2 = t2r & uvec2(0x1F1F1F1Fu);\n"
    "#else\n"
    "#ifdef NOHOIST\n"
    "#ifdef HEAPSTATE\n"
    "    uvec2 t2 = heap_u64(texelFetch(TT, c, 0));\n"
    "#else\n"
    "    uvec2 t2 = texelFetch(TT, c, 0).rg;\n"
    "#endif\n"
    "#else\n"
    "    uvec2 t2 = t2_;\n"
    "#endif\n"
    "#endif\n"
    "    uint cm = 0u, tw = 0u;\n"
    "#ifndef SMLOOP\n"
    /* all 8 pixels at once: tc (target white) per byte, t >= the threshold
     * (bit 7 kept apart: t may be >= 128); then per 16-bit lane: class == tc,
     * and (ctl) settled and not due. Group pixels done with that: their phase
     * cleared; the rest of the done ones (cm) take the cold path */
    "#ifdef NOHOIST\n"
    "    uvec4 oc4 = texelFetch(OC, ivec2(c.x & 7, c.y & 63), 0);\n"
    "#else\n"
    "    uvec4 oc4 = oc4_;\n"
    "#endif\n"
    "    uvec2 th = DITH != 0 ? oc4.zw : uvec2(0x10101010u);\n"
    "    uvec2 tb = ((((t2 & 0x7F7F7F7Fu) | 0x80808080u) - th) | t2) & 0x80808080u;\n"
    "    uvec2 tn = (tb >> 7) & 1u | (tb >> 14) & 2u | (tb >> 21) & 4u | (tb >> 28) & 8u;\n"
    "    tw = tn.x | tn.y << 4;\n"
    "    uint sk = 0u;\n"
    "    for (int j = 0; j < 4; j++) {\n"
    "      uint x = q[j], tl = (tw >> (2 * j)) & 1u | ((tw >> (2 * j + 1)) & 1u) << 16;\n"
    "      uint d = ((x >> 12) & 0x00030003u) ^ tl;\n"
    "      uint ok = ~(d | (d >> 1)) & 0x00010001u;\n"
    "#if CTL\n"
    "      ok &= x >> 14;\n"
    "#if PAY\n"
    "      ok &= ~(x >> 15);\n"
    "#endif\n"
    "#endif\n"
    "      ok &= 0x00010001u;\n"
    "      sk |= (ok & 1u) << (2 * j) | (ok >> 16) << (2 * j + 1);\n"
    "    }\n"
    "    cm = sm & ~sk;\n"
    "    uint cl = sm & sk;\n"
    "    for (int j = 0; j < 4; j++)\n"
    "      q[j] &= ~(((cl >> (2 * j)) & 1u) * 3u | ((cl >> (2 * j + 1)) & 1u) * 0x30000u);\n"
    "#endif\n"
    "#ifdef SMLOOP\n"
    "    for (uint m = sm; m != 0u; m &= m - 1u) {\n"
    "      uint i = uint(findLSB(m)), sh = (i & 1u) << 4, h = q[i >> 1] >> sh;\n"
    "      uint t = ((i < 4u ? t2.x : t2.y) >> ((i & 3u) << 3)) & 255u;\n"
    "      uint tc = DITH != 0 ? (t >= 2u * ((gm.y >> (4u * i)) & 15u) ? 1u : 0u) : (t >= 16u ? 1u : 0u);\n"
    "      tw |= tc << i;\n"
    "      if (((h >> 12) & 3u) == tc && (CTL == 0 || (((h >> 14) & 1u) != 0u && !(PAY != 0 && ((h >> 15) & 1u) != 0u))))\n"
    "        q[i >> 1] &= ~(3u << sh);\n"
    "      else cm |= 1u << i;\n"
    "    }\n"
    "#endif\n"
    "#ifdef NOCM\n    cm = 0u;\n#endif\n"
    "#ifdef XALLCM\n    cm = sm;\n#endif\n"     /* (X*: timing: every group pixel stepped) */

    "#ifdef COUNT\n"
    "#ifdef LEAD\n    if (!lead)\n#endif\n"    /* (LEAD: the leading row never counts) */
    "    atomicAdd(cn[0], 1u);\n"
    "#ifdef LEAD\n    if (!lead)\n#endif\n"
    "    if (cm != 0u) { atomicAdd(cn[1], 1u); atomicAdd(cn[2], uint(bitCount(cm))); }\n"
    "#endif\n"
    "    if (cm != 0u) {\n"
    "#ifdef XLIST\n"
    "#ifdef LEAD\n      if (!lead)\n#endif\n"
    "      {\n"
    "        uint tile = gl_WorkGroupID.y * gl_NumWorkGroups.x + gl_WorkGroupID.x;\n"
    "        uint kb = atomicAdd(xl[tile], uint(bitCount(cm)));\n"
    "        for (uint m = cm; m != 0u; m &= m - 1u, kb++)\n"
    "          xl[2048u + tile * 2048u + (kb & 2047u)] = uint(c.x) | uint(c.y) << 8 | uint(findLSB(m)) << 19;\n"
    "      }\n"
    "#endif\n"
    /* by octant: plane j in every lane at once (the lanes' accesses side by
     * side), the group's octants jlo..jhi, NJ of them at most (compile time);
     * OC .g: the pixel of each octant. All the cold reads first, then the
     * steps and the stores: a loop reading and storing by turns gave wrong
     * state on the A509 under GPU load (epd-stagger-test beside
     * a GPU load, deterministic; not a cache-invalidate race: an idle batch
     * between the passes did not help) */
    "#ifdef NOHOIST\n"
    "#ifndef COLDLIN\n"
    "      uint po = texelFetch(OC, ivec2(c.x & 7, c.y & 63), 0).g;\n"
    "#endif\n"
    /* NJ 8 (G 1): the 8 reads in registers, shifted through the loop (no
     * indexed array) */
    "#if NJ == 8\n"
    "#ifdef XNOCVF\n"
    "#define CVF(j) (uint(c.x) * uint(j + 1) & 0x7FFu)\n"
    "#elif defined(HEAPSTATE)\n"
    "#define CVF(j) heap_u32(texelFetch(CS, ivec2(c.x * 8 + (j), c.y), 0))\n"
    "#elif defined(COLDLIN)\n"
    "#define CVF(j) texelFetch(CS, ivec2(c.x * 8 + (j), c.y), 0).r\n"
    "#else\n"
    "#define CVF(j) texelFetch(CS, ivec2((j) << " STR(CPLANE_SH) " | c.x, c.y), 0).r\n"
    "#endif\n"
    "      uint c0 = CVF(0), c1 = CVF(1), c2 = CVF(2), c3 = CVF(3), c4 = CVF(4), c5 = CVF(5), c6 = CVF(6), c7 = CVF(7);\n"
    "#elif defined(COLDLIN)\n"
    "      uint cvs[8];\n"
    "#ifdef HEAPSTATE\n"
    "      for (int k = 0; k < 8; k++) cvs[k] = heap_u32(texelFetch(CS, ivec2(c.x * 8 + k, c.y), 0));\n"
    "#else\n"
    "      for (int k = 0; k < 8; k++) cvs[k] = texelFetch(CS, ivec2(c.x * 8 + k, c.y), 0).r;\n"
    "#endif\n"
    "#else\n"
    "      uint cvs[NJ];\n"
    "      for (int k = 0; k < NJ; k++) {\n"
    "        int j = min(jlo + k, 7);\n"
    "        cvs[k] = texelFetch(CS, ivec2(j << " STR(CPLANE_SH) " | c.x, c.y), 0).r;\n"
    "      }\n"
    "#endif\n"
    "#elif defined(COLDLIN)\n"
    "      uint cvs[8] = cv_;\n"
    "#else\n"
    "      uint po = oc4_.y;\n"
    "      uint cvs[NJ] = cv_;\n"
    "#endif\n"
    /* SBIT (stripes, NJ 8, NOHOIST; opt-in, GLASSINE_STAG_DEFS): by the set bits
     * of cm, not by octant: a wave pays its busiest lane's pixels, not 8
     * steps; ip: OC .g inverted (a pixel's octant), the 8 cold reads
     * (already in registers) indexed (A509 idle, SWY 64, ctl 1
     * static 6.73 -> 6.23 ms, window 5.59 -> 5.26, ctl 3 static 5.47 -> 5.10,
     * scroll equal; its load gate is owed) */
    "#if defined(SBIT) && STRIPE && NJ == 8 && defined(NOHOIST) && !defined(COLDLIN)\n"
    "      uint cva[8] = uint[8](c0, c1, c2, c3, c4, c5, c6, c7), ip = 0u;\n"
    "      for (int jj = 0; jj < 8; jj++) ip |= uint(jj) << (3u * ((po >> (3u * uint(jj))) & 7u));\n"
    "      for (uint mb = cm; mb != 0u; mb &= mb - 1u) {\n"
    "        uint i = uint(findLSB(mb));\n"
    "        int j = int((ip >> (3u * i)) & 7u);\n"
    "#ifdef XNOIDX\n"           /* (X*: timing: no dynamic index in the loop, wrong results; Fable round 5) */
    "        uint cvk = c0;\n"
    "#else\n"
    "        uint cvk = cva[j];\n"
    "#endif\n"
    "#define CVK cvk\n"
    "#else\n"
    "#ifdef COLDLIN\n"           /* (by pixel: k = the pixel; the group's pixels are cm's) */
    "      for (int k = 0; k < 8; k++) {\n"
    "#else\n"
    "      for (int k = 0; k < NJ; k++) {\n"
    "        int j = jlo + k;\n"
    "        if (j > jhi) break;\n"
    "#endif\n"
    "#if NJ == 8 && defined(NOHOIST)\n"   /* (every turn, before the continue: octant k's in c0) */
    "        uint cvk = c0;\n"
    "        c0 = c1; c1 = c2; c2 = c3; c3 = c4; c4 = c5; c5 = c6; c6 = c7;\n"
    "#define CVK cvk\n"
    "#else\n"
    "#define CVK cvs[k]\n"
    "#endif\n"
    "#ifdef COLDLIN\n"
    "        uint i = uint(k);\n"
    "#else\n"
    "        uint i = (po >> (3u * uint(j))) & 7u;\n"
    "#endif\n"
    "        if (((cm >> i) & 1u) == 0u) continue;\n"
    "#endif\n"
    "#ifdef XNOIDX\n"
    "#define QI 0\n"
    "#else\n"
    "#define QI (i >> 1)\n"
    "#endif\n"
    "        uint sh = (i & 1u) << 4, h = (q[QI] >> sh) & 65535u, cv = CVK;\n"
    "#ifdef XNOSTP\n"            /* (X*: timing experiments only, wrong results) */
    "        cv ^= 1u; h ^= 1u;\n"
    "#else\n"
    "#ifdef COUNT\n"
    "#ifdef LEAD\n        if (!lead)\n#endif\n"
    "        { uint pz = h & 3u; if (pz == 0u) atomicAdd(cn[5], 1u); else if (pz == 1u + ((tw >> i) & 1u)) atomicAdd(cn[4], 1u); }\n"
    "#endif\n"
    "        w |= stp(i, h, cv, (tw >> i) & 1u);\n"
    "#endif\n"
    "        q[QI] = (q[QI] & ~(65535u << sh)) | h << sh;\n"
    "#ifdef COLDLIN\n"
    "#ifdef HEAPSTATE\n"
    "        ST(C, ivec2(c.x * 8 + int(i), c.y), unpackUnorm4x8(cv));\n"
    "#else\n"
    "        ST(C, ivec2(c.x * 8 + int(i), c.y), uvec4(cv));\n"
    "#endif\n"
    "#else\n"
    "        ST(C, ivec2(j << " STR(CPLANE_SH) " | c.x, c.y), uvec4(cv));\n"
    "#endif\n"
    "#if TSKIP\n"
    "        busy = busy || cv != CVK;\n"
    "#endif\n"
    "      }\n"
    "    }\n"
    "#endif\n"
    "    uvec4 q2 = uvec4(q[0], q[1], q[2], q[3]);\n"
    "    if (q2 != q0) ST(H, c, q2);\n"
    "#if TSKIP\n"
    "    busy = busy || q2 != q0;\n"
    "#endif\n"
    "  }\n"
    "#if TSKIP\n"                 /* (work this frame or a phase still running: the tile's frame id) */
    "#ifndef TS_NOSTORE\n"
    "  if (busy || ((q[0] | q[1] | q[2] | q[3]) & 0x00030003u) != 0u) ST(AT, tl, uvec4(uint(fid)));\n"
    "#endif\n"
    "#endif\n"
    "  }\n"
    "#ifdef COUNT\n"
    "#ifdef LEAD\n  if (!lead)\n#endif\n"
    "  if (w != 0u) atomicAdd(cn[3], 1u);\n"
    "#endif\n"
    "#if DIRECT\n"
    "  int line = " STR(DATA_LINE0) " + " STR(EPD_H) " - 1 - c.y, col = " STR(DATA_COL0) " + c.x, k = line * pw + col;\n"
    "#ifdef NOHOIST\n"
    "  uint v = texelFetch(TP, ivec2(col, line), 0).r | w;\n"
    "#else\n"
    "  uint v = tp_ | w;\n"
    "#endif\n"
    "#ifdef LEAD\n"
    "  if (lead) ST(O, c, uvec4(v)); else\n"
    "#endif\n"
    "  imageStore(F, pw > 0 ? ivec2(k % " STR(FLAT_W) ", k / " STR(FLAT_W) ") : ivec2(col, line),\n"
    "             unpackUnorm4x8(v));\n"
    "#else\n"
    "  ST(O, c, uvec4(w));\n"
    "#endif\n"
    "}\n";

/* GLASSINE_STAG_COUNT or /etc/glassine/stag-count (epdd: across reexecs) */
static int stag_count_on(void)
{
    static int on = -1;
    if (on < 0) {
        FILE *f = fopen("/etc/glassine/stag-count", "r");
        on = getenv("GLASSINE_STAG_COUNT") || f;
        if (f) fclose(f);
    }
    return on;
}

/* the program of mode k = ctl | pay on << 1 | dither << 2 | drain << 3 | direct << 4,
 * + 32 x nj (the octants a group spans: the cold reads of a word; 8 as 0), + 256 x
 * (ctl - 1) for ctl 2 and 3, + 1024 for payrule 1 (ctl >= 1; pay then off)
 * + 2048 LEAD (epd_gpu_stag_lead), + 4096 blocks (with stripes), + 8192 tile skip,
 * + 16384 htop, + 32768 stripes; with stripes (nj 8, 0 in the key) + 32 cells,
 * + 64 cells in rank order (CELLR); lanes without stripes, G >= 2: nj 7 (lpx) */
static GLuint stag_prog(epd_gpu *g, int k)
{
    if (g->sg.pcs[k]) return g->sg.pcs[k];
    int nj = k >> 15 & 1 ? 8 : k >> 5 & 7 ? k >> 5 & 7 : 8;   /* (8, G 1: 0 in the key; stripes: 8, the bits the cells') */
    const char *defs = getenv("GLASSINE_STAG_DEFS");     /* (timing experiments) */
    if (stag_count_on()) defs = g->sg.lanes ? "#define COUNT 1" : g->sg.heapstate ? "#define COUNT 1\n#define COLDLIN 1\n#define HEAPSTATE 1" : g->sg.coldlin ? "#define COUNT 1\n#define COLDLIN 1" : "#define COUNT 1";   /* (the work census, epd_gpu_stag_frame; COLDLIN: C's layout) */
    size_t sn = strlen(stag_cs) + (defs ? strlen(defs) : 0) + 512;    /* (the defines above it) */
    char *src = malloc(sn);
    if (!src) return 0;
    /* G 1 (NJ 8, A2/1): every word takes the cold path; G 1 and n 1 compiled
     * in (G1C), the octants fetched where used (NOHOIST) (A509,
     * dither scroll 75 -> 21.5 ms) */
    int stw = k >> 15 & 1;
    /* lanes, per-pixel groups (blue noise, G >= 2; nj 7 in the key, a value
     * the octants never take): the cold is linear, so no octants; the 8 reads
     * where used, in registers (NOHOIST, NJ 8), G and n stay uniforms */
    int lpx = g->sg.lanes && !stw && nj == 7;
    if (lpx) nj = 8;
    snprintf(src, sn, "#version 310 es\n%s%s%s%s%s%s#define STRIPE %d\n#define WX %d\n#define WY %d\n#define CTL %d\n#define PAY %d\n#define DITH %d\n#define DRAIN %d\n#define DIRECT %d\n#define NJ %d\n#define PRULE %d\n#define TSKIP %d\n#define HTOP %d\n%s\n%s",
             g->sg.lanes ? "#define COLDLIN 1\n#define HEAPSTATE 1\n#define LANES 1\n" : "",
             k >> 11 & 1 ? "#define LEAD 1\n" : "", stw && k >> 12 & 1 ? "#define BLOCKS 1\n" : "",
             stw && k >> 5 & 1 ? "#define CELLS 1\n" : "", stw && k >> 6 & 1 ? "#define CELLR 1\n" : "",
             stw || lpx ? "#define NOHOIST 1\n" : nj == 8 ? "#define G1C 1\n#define NOHOIST 1\n" : "", stw, stw ? 8 : 32, stw ? g->sg.swy : g->sg.wy, (k & 1) + (k >> 8 & 3), k >> 1 & 1, k >> 2 & 1, k >> 3 & 1, k >> 4 & 1, nj, k >> 10 & 1, k >> 13 & 1, k >> 14 & 1, defs ? defs : "", stag_cs + strlen("#version 310 es\n"));
    GLuint c = shader(g, GL_COMPUTE_SHADER, src);
    free(src);
    if (!c) return 0;
    GLuint p = g->glCreateProgram();
    g->glAttachShader(p, c);
    g->glLinkProgram(p);
    GLint ok = 0;
    g->glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) { fprintf(stderr, "epd-gpu: staggered steps: link failed\n"); return 0; }
    g->glUseProgram(p);
    static const struct { const char *n; int u; } m[] = { { "HS", GS_H }, { "CS", GS_C }, { "TT", GS_T }, { "OC", GS_OC }, { "GM", GS_GM }, { "TP", 0 }, { "ATS", GS_AT } };
    for (unsigned i = 0; i < sizeof m / sizeof *m; i++) g->glUniform1i(g->glGetUniformLocation(p, m[i].n), m[i].u);
    return g->sg.pcs[k] = p;
}

/* load: the pixels of the box (words x0 x1, rows y0 y1) whose LC value has
 * bit 12 (loaded on the CPU since the last load) into their cold plane and
 * hot half, the hot bits made from the cold state (epd_stag_hotbits; rcp =
 * R, ctl, pay); clr: the box parked (enter). store: the box's cold state
 * into LC, linear (the exit's readback) */
/* the tile skip's mark pass: the tiles of a changed rect (tiles x0..x1, y0..y1,
 * exclusive) get the coming frame's id, as if they had work */
static const char *stag_mark_cs =
    "#version 310 es\n"
    "layout(local_size_x = 8, local_size_y = 8) in;\n"
    "layout(r32ui, binding = 4) writeonly uniform highp uimage2D AT;\n"
    "uniform ivec4 box;\n"
    "uniform uint fid;\n"
    STAG_ST
    "void main() {\n"
    "#ifdef LEAD\n"               /* (the leading row: the box's first tiles, clamped into it: every lane stores) */
    "  bool lead = gl_WorkGroupID.y == 0u;\n"
    "  ivec2 t = lead ? min(box.xz + ivec2(gl_GlobalInvocationID.xy), box.yw - 1) : box.xz + ivec2(gl_GlobalInvocationID.xy) - ivec2(0, 8);\n"
    "#else\n"
    "  ivec2 t = box.xz + ivec2(gl_GlobalInvocationID.xy);\n"
    "#endif\n"
    "  if (t.x < box.y && t.y < box.w) ST(AT, t, uvec4(fid));\n"
    "}\n";

static const char *stag_load_cs =
    "#version 310 es\n"
    "precision highp int; precision highp uimage2D; precision highp usampler2D;\n"
    "layout(local_size_x = 32, local_size_y = 8) in;\n"
    "uniform usampler2D HS, LS, OC;\n"
    "uniform ivec4 box;\n"
    "uniform ivec3 rcp;\n"
    "uniform int clr;\n"
    "layout(rgba32ui, binding = 0) writeonly uniform uimage2D H;\n"
    "#ifdef HEAPSTATE\n"
    "layout(rgba8, binding = 1) writeonly uniform highp image2D C;\n"
    "#else\n"
    "layout(r32ui, binding = 1) writeonly uniform uimage2D C;\n"
    "#endif\n"
    STAG_ST
    "void main() {\n"
    "#ifdef LEAD\n"               /* (the leading row: the box's first words, clamped into it: every lane works) */
    "  bool lead = gl_WorkGroupID.y == 0u;\n"
    "  ivec2 c = lead ? min(ivec2(gl_GlobalInvocationID.xy) + box.xz, box.yw - 1) : ivec2(gl_GlobalInvocationID.xy) - ivec2(0, 8) + box.xz;\n"
    "#else\n"
    "  ivec2 c = ivec2(gl_GlobalInvocationID.xy) + box.xz;\n"
    "#endif\n"
    "  if (c.x >= box.y || c.y >= box.w) return;\n"
    "  uint oq = texelFetch(OC, ivec2(c.x & 7, c.y & 63), 0).r;\n"
    "  if (clr != 0) {\n"
    "    uint pk = 0x40004000u;   /* EPD_STAG_PARKED, two pixels */\n"
    "    ST(H, c, uvec4(pk));\n"
    "#ifdef COLDLIN\n"
    "#ifdef HEAPSTATE\n"
    "    for (uint i = 0u; i < 8u; i++) ST(C, ivec2(c.x * 8 + int(i), c.y), vec4(0.0));\n"
    "#else\n"
    "    for (uint i = 0u; i < 8u; i++) ST(C, ivec2(c.x * 8 + int(i), c.y), uvec4(0u));\n"
    "#endif\n"
    "#else\n"
    "    for (uint i = 0u; i < 8u; i++) ST(C, ivec2(int(((oq >> (4u * i)) & 15u) << " STR(CPLANE_SH) ") | c.x, c.y), uvec4(0u));\n"
    "#endif\n"
    "    return;\n"
    "  }\n"
    "  uvec4 q0 = texelFetch(HS, c, 0);\n"
    "  uint q[4] = uint[4](q0.x, q0.y, q0.z, q0.w);\n"
    "  bool ch = false;\n"
    "  for (uint i = 0u; i < 8u; i++) {\n"
    "    uint v = texelFetch(LS, ivec2(c.x * 8 + int(i), c.y), 0).r;\n"
    "    if ((v & 4096u) == 0u) continue;\n"
    "    v &= ~4096u;\n"
    "#ifdef COLDLIN\n"
    "#ifdef HEAPSTATE\n"
    "    ST(C, ivec2(c.x * 8 + int(i), c.y), unpackUnorm4x8(v));\n"
    "#else\n"
    "    ST(C, ivec2(c.x * 8 + int(i), c.y), uvec4(v));\n"
    "#endif\n"
    "#else\n"
    "    ST(C, ivec2(int(((oq >> (4u * i)) & 15u) << " STR(CPLANE_SH) ") | c.x, c.y), uvec4(v));\n"
    "#endif\n"
    "    int l = int(v & 31u), op = int((v >> 5) & 63u), dc = int(v) >> 16;\n"
    "    uint lv = l == 0 ? 0u : l == 30 ? 1u : 2u;\n"
    "    uint set = (lv == 0u ? op == 0 : lv == 1u ? op == rcp.x : false) ? 1u : 0u;\n"
    "    int owe = lv == 1u ? -dc : dc;\n"
    "    uint due = lv < 2u && rcp.y != 0 && rcp.z > 0 && owe >= rcp.z ? 1u : 0u;\n"
    "    uint sh = (i & 1u) << 4;\n"
    "    q[i >> 1u] = (q[i >> 1u] & ~(65535u << sh)) | (lv << 12 | set << 14 | due << 15) << sh;\n"
    "    ch = true;\n"
    "  }\n"
    "#ifdef LEAD\n"
    "  ch = ch || lead;\n"     /* (a leading lane with nothing fresh stores too) */
    "#endif\n"
    "  if (ch) ST(H, c, uvec4(q[0], q[1], q[2], q[3]));\n"
    "}\n";
static const char *stag_store_cs =
    "#version 310 es\n"
    "precision highp int; precision highp uimage2D; precision highp usampler2D;\n"
    "layout(local_size_x = 32, local_size_y = 8) in;\n"
    "#ifdef HEAPSTATE\n"
    "uniform usampler2D OC;\nuniform highp sampler2D CS;\n"
    "#else\n"
    "uniform usampler2D CS, OC;\n"
    "#endif\n"
    "uniform ivec4 box;\n"
    "layout(r32ui, binding = 0) writeonly uniform uimage2D L;\n"
    STAG_ST
    STAG_HEAP
    "void main() {\n"
    "#ifdef LEAD\n"               /* (as the load's) */
    "  bool lead = gl_WorkGroupID.y == 0u;\n"
    "  ivec2 c = lead ? min(ivec2(gl_GlobalInvocationID.xy) + box.xz, box.yw - 1) : ivec2(gl_GlobalInvocationID.xy) - ivec2(0, 8) + box.xz;\n"
    "#else\n"
    "  ivec2 c = ivec2(gl_GlobalInvocationID.xy) + box.xz;\n"
    "#endif\n"
    "  if (c.x >= box.y || c.y >= box.w) return;\n"
    "  uint oq = texelFetch(OC, ivec2(c.x & 7, c.y & 63), 0).r;\n"
    "  for (uint i = 0u; i < 8u; i++)\n"
    "#ifdef COLDLIN\n"
    "#ifdef HEAPSTATE\n"
    "    ST(L, ivec2(c.x * 8 + int(i), c.y), uvec4(heap_u32(texelFetch(CS, ivec2(c.x * 8 + int(i), c.y), 0))));\n"
    "#else\n"
    "    ST(L, ivec2(c.x * 8 + int(i), c.y), uvec4(texelFetch(CS, ivec2(c.x * 8 + int(i), c.y), 0).r));\n"
    "#endif\n"
    "#else\n"
    "    ST(L, ivec2(c.x * 8 + int(i), c.y),\n"
    "               uvec4(texelFetch(CS, ivec2(int(((oq >> (4u * i)) & 15u) << " STR(CPLANE_SH) ") | c.x, c.y), 0).r));\n"
    "#endif\n"
    "}\n";

static GLuint cs_prog(epd_gpu *g, const char *src, const char *what)
{
    GLuint c = shader(g, GL_COMPUTE_SHADER, src);
    if (!c) return 0;
    GLuint p = g->glCreateProgram();
    g->glAttachShader(p, c);
    g->glLinkProgram(p);
    GLint ok = 0;
    g->glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) { fprintf(stderr, "epd-gpu: staggered steps' %s pass: link failed\n", what); return 0; }
    return p;
}


/* px (GLASSINE_STAG_PX): the step as a fragment pass, one pixel a
 * fragment, the state read at the fragment's own pixel and written in place
 * (A509, an R32UI target read so 200 frames, beside GPU load,
 * bit for bit). The same decisions as stag_cs, per pixel: a pixel of the
 * frame's group, its phase done, skips (phase cleared) as there or steps
 * (stp, the same text); the rest discard */
static const char *stag_px_fs =
    "layout(location = 0) out uint oh;\n"
    "layout(location = 1) out uint oc;\n"
    "void main() {\n"
    "  ivec2 p = ivec2(gl_FragCoord.xy);\n"
    /* (every fetch up front, before any branch: the A509 gave wrong state for
     * one 8 x 8 block beside another process's GPU work with fetches after the
     * discards, as the word pass did) */
    "  uint h = texelFetch(PHS, p, 0).r, cv = texelFetch(PCS, p, 0).r;\n"
    "  uint rk = texelFetch(RK, p & 63, 0).r;\n"
    "  uvec2 t2 = texelFetch(TT, ivec2(p.x >> 3, p.y), 0).rg;\n"
    "#if PP\n"
    "#define SKIP { oh = h; oc = cv; return; }\n"
    "#else\n"
    "#define SKIP discard;\n"
    "#endif\n"
    "  uint grk = stw > 0 ? (bitfieldReverse(uint(p.x / stw) & 31u) >> 27) << 7 : rk & 65535u;\n"   /* (epd_stag_srank) */
    "  if (int((grk * uint(gG)) >> 12) != fm) SKIP\n"

    "  uint ph = h & 3u, k = (h >> 2) & 15u, age = (uint(f6) - (h >> 6)) & 63u;\n"
    "#if CTL >= 2\n"
    "  if (ph != 0u && age < k) SKIP\n"
    "#else\n"
    "  if (ph != 0u && age <= k) SKIP\n"
    "#endif\n"

    "#if DRAIN\n"
    "  h &= ~3u;\n"
    "#else\n"
    "  uint t = (((p.x & 7) < 4 ? t2.x : t2.y) >> (8u * uint(p.x & 3))) & 255u;\n"
    "  uint lv = (h >> 12) & 3u, set = (h >> 14) & 1u, due = (h >> 15) & 1u, tc;\n"
    "  bool ok;\n"
    "  tc = t >= (DITH != 0 ? rk >> 16 : 16u) ? 1u : 0u;\n"
    "  ok = lv == tc;\n"
    "#if CTL\n"
    "  ok = ok && set != 0u;\n"
    "#if PAY\n"
    "  ok = ok && due == 0u;\n"
    "#endif\n"
    "#endif\n"
    "  if (ok) h &= ~3u;\n"
    "  else stp(0u, h, cv, tc);\n"
    "#endif\n"
    /* (each output written once, at the end: no early return) */
    "  oh = h; oc = cv;\n"
    "}\n";

/* px: the load (as stag_load_cs: the pixels of LS with bit 12, their hot bits
 * made from the cold state) and the enter's clear (clr: parked, 0) over the
 * scissor box; the pack: the frame's drive bits per word into WD (a pixel
 * drives while its phase runs, age < n: the step pass's result) */
static const char *stag_px_load_fs =
    "#version 310 es\n"
    "precision highp int; precision highp usampler2D;\n"
    "uniform usampler2D LS;\n"
    "uniform ivec3 rcp;\n"
    "uniform int clr;\n"
    "layout(location = 0) out uint oh;\n"
    "layout(location = 1) out uint oc;\n"
    "layout(location = 2) out uint oh2;\n"     /* (pp: the other pair, fbld) */
    "layout(location = 3) out uint oc2;\n"
    "void main() {\n"
    "  uint v = clr != 0 ? 4096u : texelFetch(LS, ivec2(gl_FragCoord.xy), 0).r;\n"
    "  if ((v & 4096u) == 0u) discard;\n"
    "  v &= ~4096u;\n"
    "  int l = int(v & 31u), op = int((v >> 5) & 63u), dc = int(v) >> 16;\n"
    "  uint lv = l == 0 ? 0u : l == 30 ? 1u : 2u;\n"
    "  uint set = (lv == 0u ? op == 0 : lv == 1u ? op == rcp.x : false) ? 1u : 0u;\n"
    "  int owe = lv == 1u ? -dc : dc;\n"
    "  uint due = lv < 2u && rcp.y != 0 && rcp.z > 0 && owe >= rcp.z ? 1u : 0u;\n"
    "  oh = clr != 0 ? 16384u : lv << 12 | set << 14 | due << 15;\n"   /* (clr: EPD_STAG_PARKED, 0) */
    "  oc = v;\n"
    "  oh2 = oh; oc2 = oc;\n"
    "}\n";
static const char *stag_px_copy_fs =
    "#version 310 es\n"
    "precision highp int; precision highp usampler2D;\n"
    "uniform usampler2D PHS2, PCS2;\n"
    "layout(location = 0) out uint oh;\n"
    "layout(location = 1) out uint oc;\n"
    "void main() {\n"
    "  ivec2 p = ivec2(gl_FragCoord.xy);\n"
    "  oh = texelFetch(PHS2, p, 0).r; oc = texelFetch(PCS2, p, 0).r;\n"
    "}\n";
static const char *stag_px_pack_fs =
    "#version 310 es\n"
    "precision highp int; precision highp usampler2D;\n"
    "uniform usampler2D PHS, PHS2;\n"
    "uniform int f6, stw, gG;\n"
    "uniform uint pm;\n"
    "layout(location = 0) out uint ow;\n"
    "void main() {\n"
    "  ivec2 c = ivec2(gl_FragCoord.xy);\n"
    "  uint gi = stw > 0 ? (((bitfieldReverse(uint(c.x * 8 / stw) & 31u) >> 27) << 7) * uint(gG)) >> 12 : 0u;\n"
    "  bool b = ((pm >> gi) & 1u) != 0u;\n"      /* (pp: the word's group lives in pair 2) */
    "  uint w = 0u;\n"
    "  for (int i = 0; i < 8; i++) {\n"
    "    ivec2 q = ivec2(c.x * 8 + i, c.y);\n"
    "    uint h = b ? texelFetch(PHS2, q, 0).r : texelFetch(PHS, q, 0).r, ph = h & 3u;\n"
    "    if (ph != 0u && ((uint(f6) - (h >> 6)) & 63u) < ((h >> 2) & 15u)) w |= ph << uint(14 - 2 * i);\n"
    "  }\n"
    "  ow = w;\n"
    "}\n";

static GLuint fs_prog(epd_gpu *g, const char *src, const char *what)
{
    static const char vs31[] =          /* (vs as GLSL ES 3.10: a program's stages share the version) */
        "#version 310 es\n"
        "void main() {\n"
        "  vec2 p = vec2(gl_VertexID & 1, gl_VertexID >> 1) * 4.0 - 1.0;\n"
        "  gl_Position = vec4(p, 0.0, 1.0);\n"
        "}\n";
    GLuint v = shader(g, GL_VERTEX_SHADER, vs31), f = shader(g, GL_FRAGMENT_SHADER, src);
    if (!v || !f) return 0;
    GLuint p = g->glCreateProgram();
    g->glAttachShader(p, v);
    g->glAttachShader(p, f);
    g->glLinkProgram(p);
    GLint ok = 0;
    g->glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) { fprintf(stderr, "epd-gpu: staggered steps' %s pass: link failed\n", what); return 0; }
    return p;
}

/* the px step program of mode k (stag_prog's key; direct, nj, tile skip ignored):
 * stag_cs's tables and stp, then stag_px_fs */
enum { GS_PH = GS_H, GS_PC = GS_C, GS_RK = GS_OC, GS_PH2 = 8, GS_PC2 = 9 };
static GLuint stag_px_prog(epd_gpu *g, int k)
{
    if (g->sg.ppx[k]) return g->sg.ppx[k];
    const char *a = strstr(stag_cs, "#if HTOP\n"), *b = strstr(stag_cs, "#if DIRECT\n");
    const char *c = strstr(stag_cs, "#ifdef XTRIV\n"), *d = strstr(stag_cs, "void main() {");
    if (!a || !b || !c || !d || b < a || d < c) return 0;
    const char *defs = getenv("GLASSINE_STAG_DEFS");
    size_t sn = (size_t)(b - a) + (size_t)(d - c) + strlen(stag_px_fs) + (defs ? strlen(defs) : 0) + 640;
    char *src = malloc(sn);
    if (!src) return 0;
    snprintf(src, sn, "#version 310 es\nprecision highp int; precision highp usampler2D;\n"
             "#define PP %d\n#define CTL %d\n#define PAY %d\n#define DITH %d\n#define DRAIN %d\n#define DIRECT 0\n#define NJ %d\n#define PRULE %d\n#define TSKIP 0\n#define STRIPE 0\n#define HTOP %d\n%s\n"
             "uniform usampler2D PHS, PCS, TT, RK;\nuniform int f6, n, pay, R, gG, gK, pcap, fm, stw;\n%.*s%.*s%s",
             g->sg.pp, (k & 1) + (k >> 8 & 3), k >> 1 & 1, k >> 2 & 1, k >> 3 & 1, k >> 5 & 1 ? 4 : 8, k >> 10 & 1, k >> 14 & 1, defs ? defs : "",
             (int)(b - a), a, (int)(d - c), c, stag_px_fs);
    GLuint p = fs_prog(g, src, "px step");
    free(src);
    if (!p) return 0;
    g->glUseProgram(p);
    static const struct { const char *n; int u; } m[] = { { "PHS", GS_PH }, { "PCS", GS_PC }, { "TT", GS_T }, { "RK", GS_RK } };
    for (unsigned i = 0; i < sizeof m / sizeof *m; i++) g->glUniform1i(g->glGetUniformLocation(p, m[i].n), m[i].u);
    return g->sg.ppx[k] = p;
}

static int stag_px_open(epd_gpu *g)
{
    g->glActiveTexture(GL_TEXTURE0 + GS_PH);
    g->sg.PH = tex(g, GL_R16UI, EPD_W, EPD_H);
    g->sg.PC = tex(g, GL_R32UI, EPD_W, EPD_H);
    {
        uint32_t rk[64 * 64];
        epd_stag_dither_need();
        for (int y = 0; y < 64; y++)
            for (int x = 0; x < 64; x++) rk[y * 64 + x] = (uint32_t)epd_bn_rank[y][x] | (uint32_t)(2 * epd_stag_dqt[y][x]) << 16;
        g->sg.RK = tex(g, GL_R32UI, 64, 64);
        g->glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 64, 64, GL_RED_INTEGER, GL_UNSIGNED_INT, rk);
    }
    g->glGenFramebuffers(1, &g->sg.fbpx);
    g->glBindFramebuffer(GL_FRAMEBUFFER, g->sg.fbpx);
    g->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, g->sg.PH, 0);
    g->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, g->sg.PC, 0);
    static const GLenum db[2] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1 };
    g->glDrawBuffers(2, db);
    if (g->glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) { fprintf(stderr, "epd-gpu: px framebuffer incomplete\n"); return -1; }
    if (g->sg.pp) {
        g->glActiveTexture(GL_TEXTURE0 + GS_PH2);
        g->sg.PH2 = tex(g, GL_R16UI, EPD_W, EPD_H);
        g->sg.PC2 = tex(g, GL_R32UI, EPD_W, EPD_H);
        g->glGenFramebuffers(1, &g->sg.fbpx2);
        g->glBindFramebuffer(GL_FRAMEBUFFER, g->sg.fbpx2);
        g->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, g->sg.PH2, 0);
        g->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, g->sg.PC2, 0);
        g->glDrawBuffers(2, db);
        GLenum s2 = g->glCheckFramebufferStatus(GL_FRAMEBUFFER);
        g->glGenFramebuffers(1, &g->sg.fbld);
        g->glBindFramebuffer(GL_FRAMEBUFFER, g->sg.fbld);
        g->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, g->sg.PH, 0);
        g->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, g->sg.PC, 0);
        g->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, g->sg.PH2, 0);
        g->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT3, GL_TEXTURE_2D, g->sg.PC2, 0);
        static const GLenum d4[4] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2, GL_COLOR_ATTACHMENT3 };
        g->glDrawBuffers(4, d4);
        if (s2 != GL_FRAMEBUFFER_COMPLETE || g->glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            fprintf(stderr, "epd-gpu: px ping-pong framebuffers incomplete\n");
            return -1;
        }
    }
    if (!(g->sg.fbwd = fbo_of(g, g->sg.WD))) return -1;
    if (!(g->sg.pxld = fs_prog(g, stag_px_load_fs, "px load")) || !(g->sg.pxpk = fs_prog(g, stag_px_pack_fs, "px pack"))) return -1;
    g->glUseProgram(g->sg.pxld);
    g->glUniform1i(g->glGetUniformLocation(g->sg.pxld, "LS"), GS_LC);
    g->sg.xl_rcp = g->glGetUniformLocation(g->sg.pxld, "rcp");
    g->sg.xl_clr = g->glGetUniformLocation(g->sg.pxld, "clr");
    g->glUseProgram(g->sg.pxpk);
    g->glUniform1i(g->glGetUniformLocation(g->sg.pxpk, "PHS"), GS_PH);
    g->sg.xk_f6 = g->glGetUniformLocation(g->sg.pxpk, "f6");
    g->glUniform1i(g->glGetUniformLocation(g->sg.pxpk, "PHS2"), GS_PH2);
    g->sg.xk_pm = g->glGetUniformLocation(g->sg.pxpk, "pm");
    g->sg.xk_stw = g->glGetUniformLocation(g->sg.pxpk, "stw");
    g->sg.xk_gG = g->glGetUniformLocation(g->sg.pxpk, "gG");
    if (g->sg.pp) {
        if (!(g->sg.pxcp = fs_prog(g, stag_px_copy_fs, "px copy"))) return -1;
        g->glUseProgram(g->sg.pxcp);
        g->glUniform1i(g->glGetUniformLocation(g->sg.pxcp, "PHS2"), GS_PH2);
        g->glUniform1i(g->glGetUniformLocation(g->sg.pxcp, "PCS2"), GS_PC2);
    }
    if (!stag_px_prog(g, 1 | 4 | 1024 | 2 << 8)) return -1;     /* (ctl 3 payrule dither: checks the source) */
    g->glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return 0;
}

/* px: a pass of prog into fbpx over pixels [x0, x1) x rows [y0, y1) */
static void stag_px_drawto(epd_gpu *g, GLuint fb, int x0, int x1, int y0, int y1)
{
    g->glBindFramebuffer(GL_FRAMEBUFFER, fb);
    g->glViewport(0, 0, EPD_W, EPD_H);
    g->glScissor(x0, y0, x1 - x0, y1 - y0);
    g->glDrawArrays(GL_TRIANGLES, 0, 3);
}

/* px: the enter's and load's pass (pp: into both pairs) */
static void stag_px_draw(epd_gpu *g, int x0, int x1, int y0, int y1)
{
    stag_px_drawto(g, g->sg.pp ? g->sg.fbld : g->sg.fbpx, x0, x1, y0, y1);
}

/* px pp: the pixels living in pair 2 copied to pair 1 (pm 0 after) */
static void stag_px_resolve(epd_gpu *g)
{
    if (!g->sg.pp || !g->sg.pm) return;
    g->glUseProgram(g->sg.pxcp);
    bind(g, GS_PH2, g->sg.PH2);
    bind(g, GS_PC2, g->sg.PC2);
    if (g->sg.pmW <= 0)
        stag_px_drawto(g, g->sg.fbpx, 0, EPD_W, 0, EPD_H);
    else
        for (int x = 0; x < EPD_W; x += g->sg.pmW)
            if (g->sg.pm >> (epd_stag_srank(x / g->sg.pmW) * g->sg.pmG >> 12) & 1)
                stag_px_drawto(g, g->sg.fbpx, x, x + g->sg.pmW > EPD_W ? EPD_W : x + g->sg.pmW, 0, EPD_H);
    g->glFlush();
    g->sg.pm = 0;
}

/* the tables by pixel position: OC, GM (and the px engine's RK); their
 * dither thresholds from epd_stag_dqt (the engine's pattern), rebuilt when it
 * changes (epd_stag_dgen) */
static int stag_dtabs(epd_gpu *g)
{
    epd_stag_dither_need();
    /* .r the octant of each pixel of a word (4 bits each), .g the pixel of
     * each octant (3 bits each; word-stratified: one pixel per octant) */
    uint32_t oc[64 * 8 * 4] = { 0 };       /* (.b .a: each pixel's dither threshold 2 dq, a byte) */
    for (int y = 0; y < 64; y++)
        for (int x = 0; x < 8; x++)
            for (int i = 0; i < 8; i++) {
                int rk = epd_bn_rank[y][x * 8 + i], o = rk >> 9;
                oc[(y * 8 + x) * 4] |= (uint32_t)o << (4 * i);
                oc[(y * 8 + x) * 4 + 1] |= (uint32_t)i << (3 * o);
                oc[(y * 8 + x) * 4 + 2 + i / 4] |= (uint32_t)(2 * epd_stag_dqt[y][x * 8 + i]) << (8 * (i & 3));
            }
    g->glActiveTexture(GL_TEXTURE0 + GS_OC);
    if (!g->sg.OC) g->sg.OC = tex(g, GL_RGBA32UI, 8, 64);
    else g->glBindTexture(GL_TEXTURE_2D, g->sg.OC);
    g->glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 8, 64, GL_RGBA_INTEGER, GL_UNSIGNED_INT, oc);
    /* per G and frame fm (row (G (G - 1) / 2 + fm) * 64 + y): .r the pixels of
     * group fm (bit i), .g the dither thresholds (4 bits each) */
    enum { NGM = EPD_STAG_GMAX * (EPD_STAG_GMAX + 1) / 2 * 64 };
    uint32_t *gm = calloc(NGM * 8 * 2, 4);
    if (!gm) return -2;
    for (int G = 1; G <= EPD_STAG_GMAX; G++)
        for (int fm = 0; fm < G; fm++)
            for (int y = 0; y < 64; y++)
                for (int x = 0; x < 8; x++) {
                    uint32_t *e = gm + (((G * (G - 1) / 2 + fm) * 64 + y) * 8 + x) * 2;
                    for (int i = 0; i < 8; i++) {
                        int rk = epd_bn_rank[y][x * 8 + i];
                        e[0] |= (uint32_t)((rk * G >> 12) == fm) << i;
                        e[1] |= (uint32_t)epd_stag_dqt[y][x * 8 + i] << (4 * i);
                    }
                }
    g->glActiveTexture(GL_TEXTURE0 + GS_GM);
    if (!g->sg.GM) g->sg.GM = tex(g, GL_RG32UI, 8, NGM);
    else g->glBindTexture(GL_TEXTURE_2D, g->sg.GM);
    g->glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 8, NGM, GL_RG_INTEGER, GL_UNSIGNED_INT, gm);
    free(gm);
    if (g->sg.RK) {
        uint32_t rk[64 * 64];
        for (int y = 0; y < 64; y++)
            for (int x = 0; x < 64; x++) rk[y * 64 + x] = (uint32_t)epd_bn_rank[y][x] | (uint32_t)(2 * epd_stag_dqt[y][x]) << 16;
        g->glBindTexture(GL_TEXTURE_2D, g->sg.RK);
        g->glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 64, 64, GL_RED_INTEGER, GL_UNSIGNED_INT, rk);
    }
    g->sg.dgen = epd_stag_dgen;
    return 0;
}

static int stag_heap(epd_gpu *g);
static int stag_lanes(epd_gpu *g);
static void stag_lanes_park(epd_gpu *g);
int epd_gpu_stag_open(epd_gpu *g)
{
    if (g->sg.on) return 0;
    const char *ver = (const char *)g->glGetString(GL_VERSION);
    int maj = 0, min = 0;
    if (!ver || sscanf(ver, "OpenGL ES %d.%d", &maj, &min) != 2 || maj * 10 + min < 31) {
        fprintf(stderr, "epd-gpu: staggered steps need GLES 3.1 (%s)\n", ver ? ver : "?");
        return -1;
    }
#define F(t, n) if (!(g->n = (t)eglGetProcAddress(#n))) { fprintf(stderr, "epd-gpu: missing %s\n", #n); return -2; }
    GPE_FNS
#undef F
    {
        const char *v = getenv("GLASSINE_STAG_WY");
        g->sg.wy = v ? atoi(v) : 8;            /* (A509: 8 beat 4 by 5-15 %) */
        if (g->sg.wy != 1 && g->sg.wy != 2 && g->sg.wy != 4 && g->sg.wy != 8 && g->sg.wy != 16 && g->sg.wy != 32) g->sg.wy = 8;
        /* the stripe engine's (WX 8): a workgroup as many invocations as the
         * plain one's (A509, the pass costs by workgroups, 8 x 8
         * took 1.5x the 32 x 8 time, 8 x 4 2.7x; 64 rows: 15.92 ms
         * a full scroll vs 16.52 at 32, measured in the since removed grey mode) */
        v = getenv("GLASSINE_STAG_SWY");
        g->sg.swy = v ? atoi(v) : 64;
        if (g->sg.swy < 1 || g->sg.swy > 128) g->sg.swy = 64;
    }
    if (epd_gpu_stag_lead < 0) {        /* (LEAD: the scratch rows, a workgroup row of the tallest pass) */
        const char *v = getenv("GLASSINE_STAG_LEAD");
        epd_gpu_stag_lead = v && atoi(v) > 0;
    }
    g->sg.lr = 0;
    if (epd_gpu_stag_lead > 0) {
        g->sg.lr = g->sg.wy > g->sg.swy ? g->sg.wy : g->sg.swy;
        if (g->sg.lr < 8) g->sg.lr = 8;
        fprintf(stderr, "epd-gpu: staggered steps: LEAD (a leading workgroup row per pass, %d scratch rows; the A509 lost wave)\n", g->sg.lr);
    }
    g->glActiveTexture(GL_TEXTURE0 + GS_H);
    g->sg.H = tex(g, GL_RGBA32UI, GW, EPD_H + g->sg.lr);
    {                                   /* AT: frame id 0 everywhere (idle; the first frames see a whole-panel rect);
                                         * a tile a workgroup: 32 words x WY, stripes 8 x SWY */
        GLuint z[(GW + 7) / 8];
        memset(z, 0, sizeof z);
        g->sg.AT = tex(g, GL_R32UI, (GW + 7) / 8, EPD_H + g->sg.lr);
        for (int y = 0; y < EPD_H; y++) g->glTexSubImage2D(GL_TEXTURE_2D, 0, 0, y, (GW + 7) / 8, 1, GL_RED_INTEGER, GL_UNSIGNED_INT, z);
    }
    {                                   /* (lanes: the engine's D and T) */
        const epd_engine *e = g->eng;
        int px = getenv("GLASSINE_STAG_PX") && atoi(getenv("GLASSINE_STAG_PX")) > 0;
        g->sg.lanes = epd_stagger_state == 1 && e && e->cst == 1 && e->pk && e->pk->dfd >= 0 && !px;
        if (epd_stagger_state == 1 && !g->sg.lanes)
            fprintf(stderr, "epd-gpu: staggered steps: stagger_state=lanes needs claim_state=packed with D/T dma-bufs and the compute path: runs\n");
    }
    if (g->sg.lanes)
        g->sg.coldlin = g->sg.heapstate = 1;
    else {                              /* (COLDLIN, a timing/test define in GLASSINE_STAG_DEFS: the cold state linear) */
        const char *dv = getenv("GLASSINE_STAG_DEFS");
        g->sg.coldlin = dv && strstr(dv, "COLDLIN") != NULL;
        if (g->sg.coldlin) fprintf(stderr, "epd-gpu: staggered steps: COLDLIN (cold state linear; timing/test only)\n");
        g->sg.heapstate = dv && strstr(dv, "HEAPSTATE") != NULL;
        if (g->sg.heapstate && (!g->sg.coldlin || (getenv("GLASSINE_STAG_PX") && atoi(getenv("GLASSINE_STAG_PX")) > 0))) {
            fprintf(stderr, "epd-gpu: staggered steps: HEAPSTATE needs COLDLIN and the compute path\n");
            return -2;
        }
    }
    if (g->sg.lanes) {
        if (stag_lanes(g)) return -2;
    } else if (g->sg.heapstate) {       /* (HEAPSTATE: C and T in heap dma-bufs) */
        if (stag_heap(g)) return -2;
    } else if (g->sg.coldlin)           /* (linear: x = word * 8 + pixel; written only by the passes, never uploaded) */
        g->sg.C = tex(g, GL_R32UI, EPD_W, EPD_H + g->sg.lr);
    else
        g->sg.C = tex(g, GL_R32UI, 8 * CPLANE, EPD_H + g->sg.lr);  /* planes: x = octant * CPLANE + word (cplane) */
    if (!g->sg.heapstate) g->sg.T = tex(g, GL_RG32UI, GW, EPD_H);
    g->sg.WD = tex(g, GL_R32UI, GW, EPD_H + g->sg.lr);
    if (stag_dtabs(g)) return -2;
    g->glActiveTexture(GL_TEXTURE0 + GS_LC);
    g->sg.LC = tex(g, GL_R32UI, EPD_W, EPD_H + g->sg.lr);
    g->sg.LCU = tex(g, GL_R32UI, EPD_W, EPD_H);
    if (!(g->sg.cbuf = malloc((size_t)EPD_W * EPD_H * 4))) return -2;
    if (!(g->sg.fbh = fbo_of(g, g->sg.H)) || !(g->sg.fbc = fbo_of(g, g->sg.C)) || !(g->sg.fbl = fbo_of(g, g->sg.LC))) return -2;
    if (g->sg.coldlin) {                /* (the load and store passes on the linear layout: the define after #version) */
        const char *pre = g->sg.heapstate ? "#version 310 es\n#define COLDLIN 1\n#define HEAPSTATE 1\n" : "#version 310 es\n#define COLDLIN 1\n";
        size_t vn = strlen("#version 310 es\n"), ln = strlen(stag_load_cs) + strlen(pre), sn = strlen(stag_store_cs) + strlen(pre);
        char *ls = malloc(ln + 1), *ss = malloc(sn + 1);
        if (!ls || !ss) { free(ls); free(ss); return -2; }
        snprintf(ls, ln + 1, "%s%s", pre, stag_load_cs + vn);
        snprintf(ss, sn + 1, "%s%s", pre, stag_store_cs + vn);
        g->sg.pload = cs_prog(g, ls, "load");
        g->sg.pstore = cs_prog(g, ss, "store");
        free(ls);
        free(ss);
        if (!g->sg.pload || !g->sg.pstore) return -2;
    } else if (!(g->sg.pload = cs_prog(g, stag_load_cs, "load")) || !(g->sg.pstore = cs_prog(g, stag_store_cs, "store"))) return -2;
    if (!(g->sg.pmark = cs_prog(g, stag_mark_cs, "mark"))) return -2;
    g->sg.m_box = g->glGetUniformLocation(g->sg.pmark, "box");
    g->sg.m_fid = g->glGetUniformLocation(g->sg.pmark, "fid");
    g->glUseProgram(g->sg.pload);
    g->glUniform1i(g->glGetUniformLocation(g->sg.pload, "HS"), GS_H);
    g->glUniform1i(g->glGetUniformLocation(g->sg.pload, "LS"), GS_LC);
    g->glUniform1i(g->glGetUniformLocation(g->sg.pload, "OC"), GS_OC);
    g->sg.l_box = g->glGetUniformLocation(g->sg.pload, "box");
    g->sg.l_rcp = g->glGetUniformLocation(g->sg.pload, "rcp");
    g->sg.l_clr = g->glGetUniformLocation(g->sg.pload, "clr");
    g->glUseProgram(g->sg.pstore);
    g->glUniform1i(g->glGetUniformLocation(g->sg.pstore, "CS"), GS_C);
    g->glUniform1i(g->glGetUniformLocation(g->sg.pstore, "OC"), GS_OC);
    g->sg.s_box = g->glGetUniformLocation(g->sg.pstore, "box");
    if (g->sg.lr) {                     /* LEAD: the load, store and mark passes with it defined */
        const char *pre = g->sg.heapstate ? "#version 310 es\n#define COLDLIN 1\n#define HEAPSTATE 1\n#define LEAD 1\n" :
                          g->sg.coldlin ? "#version 310 es\n#define COLDLIN 1\n#define LEAD 1\n" : "#version 310 es\n#define LEAD 1\n";
        const char *srcs[3] = { stag_load_cs, stag_store_cs, stag_mark_cs }, *what[3] = { "load (LEAD)", "store (LEAD)", "mark (LEAD)" };
        for (int i = 0; i < 3; i++) {
            size_t vn = strlen("#version 310 es\n"), n = strlen(pre) + strlen(srcs[i]) + 1;
            char *s = malloc(n);
            if (!s) return -2;
            snprintf(s, n, "%s%s", pre, srcs[i] + vn);
            g->sg.lp[i] = cs_prog(g, s, what[i]);
            free(s);
            if (!g->sg.lp[i]) return -2;
        }
        g->glUseProgram(g->sg.lp[0]);
        g->glUniform1i(g->glGetUniformLocation(g->sg.lp[0], "HS"), GS_H);
        g->glUniform1i(g->glGetUniformLocation(g->sg.lp[0], "LS"), GS_LC);
        g->glUniform1i(g->glGetUniformLocation(g->sg.lp[0], "OC"), GS_OC);
        g->sg.ll_box = g->glGetUniformLocation(g->sg.lp[0], "box");
        g->sg.ll_rcp = g->glGetUniformLocation(g->sg.lp[0], "rcp");
        g->sg.ll_clr = g->glGetUniformLocation(g->sg.lp[0], "clr");
        g->glUseProgram(g->sg.lp[1]);
        g->glUniform1i(g->glGetUniformLocation(g->sg.lp[1], "CS"), GS_C);
        g->glUniform1i(g->glGetUniformLocation(g->sg.lp[1], "OC"), GS_OC);
        g->sg.ls_box = g->glGetUniformLocation(g->sg.lp[1], "box");
        g->sg.lm_box = g->glGetUniformLocation(g->sg.lp[2], "box");
        g->sg.lm_fid = g->glGetUniformLocation(g->sg.lp[2], "fid");
    }
    g->glBindFramebuffer(GL_FRAMEBUFFER, 0);
    if (!stag_prog(g, 4 << 5)) return -2;      /* (G 2: the scroll's; checks the source) */
    if (g->sg.lr && !stag_prog(g, 4 << 5 | 2048)) return -2;   /* (and its LEAD form) */
    {
        const char *v = getenv("GLASSINE_STAG_DIRECT");
        g->sg.direct = v ? atoi(v) != 0 : 1;
        g->sg.gen_q = (void *)eglGetProcAddress("glGenQueriesEXT");
        g->sg.begin_q = (void *)eglGetProcAddress("glBeginQueryEXT");
        g->sg.end_q = (void *)eglGetProcAddress("glEndQueryEXT");
        g->sg.get_q = (void *)eglGetProcAddress("glGetQueryObjectui64vEXT");
        const char *ex = (const char *)g->glGetString(GL_EXTENSIONS);
        if (g->sg.gen_q && g->sg.begin_q && g->sg.end_q && g->sg.get_q && ex && strstr(ex, "GL_EXT_disjoint_timer_query"))
            g->sg.gen_q(2, g->sg.tq);
    }
    {                                           /* the template pass: fs with STAG defined */
        static const char ver3[] = "#version 300 es\n";
        size_t n = strlen(fs) + 32;
        char *src = malloc(n);
        if (!src) return -2;
        snprintf(src, n, "%s#define STAG 1\n%s", ver3, fs + strlen(ver3));
        GLuint v = shader(g, GL_VERTEX_SHADER, vs), f = shader(g, GL_FRAGMENT_SHADER, src);
        free(src);
        if (!v || !f) return -2;
        GLuint p = g->glCreateProgram();
        g->glAttachShader(p, v);
        g->glAttachShader(p, f);
        g->glLinkProgram(p);
        GLint ok = 0;
        g->glGetProgramiv(p, GL_LINK_STATUS, &ok);
        if (!ok) { fprintf(stderr, "epd-gpu: staggered steps' template pass: link failed\n"); return -2; }
        g->sg.pemit = p;
        g->glUseProgram(p);
        g->glUniform1i(g->glGetUniformLocation(p, "tpl"), 0);
        g->glUniform1i(g->glGetUniformLocation(p, "WD"), GS_WD);
        g->sg.e_pw = g->glGetUniformLocation(p, "pw");
        g->sg.e_box = g->glGetUniformLocation(p, "box");
    }
    {
        const char *v = getenv("GLASSINE_STAG_PX");
        g->sg.px = v && atoi(v) > 0;
        v = getenv("GLASSINE_STAG_PXPP");
        g->sg.pp = g->sg.px && v && atoi(v) > 0;
        if (g->sg.px && stag_px_open(g) < 0) return -2;
    }
    if (g->sg.lanes) stag_lanes_park(g);
    g->glUseProgram(g->prog);
    if (g->glGetError() != GL_NO_ERROR) { fprintf(stderr, "epd-gpu: staggered steps: GL error in setup\n"); return -2; }
    g->sg.on = 1;
    return 0;
}

/* (pixel (x, y)'s cold state sits in column (rank >> 9) * CPLANE + x / 8 of
 * C: a word's pixels in the plane of their octant, word-stratified blue noise
 * = one pixel of each per word, so a pixel on its group's frame reads and
 * writes its own 4 bytes; the load and store passes map it) */

/* a compute pass of prog over the words [x0, x1) x rows [y0, y1) */
/* a rect of words x0..x1, rows y0..y1 changed (targets or state): the tile
 * skip sees it from the next frame on (16 kept: then their bounding box) */
static void stag_note(epd_gpu *g, int x0, int x1, int y0, int y1)
{
    if (g->sg.nrbx == 16) {
        for (int i = 1; i < 16; i++) {
            if (g->sg.rbx[i].x0 < g->sg.rbx[0].x0) g->sg.rbx[0].x0 = g->sg.rbx[i].x0;
            if (g->sg.rbx[i].x1 > g->sg.rbx[0].x1) g->sg.rbx[0].x1 = g->sg.rbx[i].x1;
            if (g->sg.rbx[i].y0 < g->sg.rbx[0].y0) g->sg.rbx[0].y0 = g->sg.rbx[i].y0;
            if (g->sg.rbx[i].y1 > g->sg.rbx[0].y1) g->sg.rbx[0].y1 = g->sg.rbx[i].y1;
        }
        g->sg.rbx[0].id = g->sg.fid;    /* (the merged box: as young as its youngest) */
        g->sg.nrbx = 1;
    }
    g->sg.rbx[g->sg.nrbx++] = (typeof(g->sg.rbx[0])){ x0, x1, y0, y1, g->sg.fid };
}

/* LEAD this pass (epd_gpu_stag_lead, with the scratch rows; not px, whose
 * passes are draws): taken once per call, the program and its dispatch agree */
static int stag_lead(epd_gpu *g)
{
    if (epd_gpu_stag_lead > 0 && !g->sg.lr && !g->sg.lwarn) {
        g->sg.lwarn = 1;
        fprintf(stderr, "epd-gpu: staggered steps: LEAD on after the open: no scratch rows, off until the next open\n");
    }
    return epd_gpu_stag_lead > 0 && g->sg.lr && !g->sg.px;
}

/* lead: one leading workgroup row (the program's LEAD form) */
static void stag_box(epd_gpu *g, GLint loc, int lead, int x0, int x1, int y0, int y1)
{
    stag_note(g, x0, x1, y0, y1);
    g->glUniform4i(loc, x0, x1, y0, y1);
    g->glDispatchCompute((GLuint)(x1 - x0 + 31) / 32, (GLuint)(y1 - y0 + 7) / 8 + (lead != 0), 1);
    g->glMemoryBarrier(GL_ALL_BARRIER_BITS);
}

/* HEAPSTATE (timing/test only, R4): a cached dma-buf (the system heap; else
 * a memfd through /dev/udmabuf: the host's stand-in) */
static int stag_heap_buf(size_t len)
{
    int h = open("/dev/dma_heap/system", O_RDONLY | O_CLOEXEC);
    if (h >= 0) {
        struct dma_heap_allocation_data al = { .len = len, .fd_flags = O_RDWR | O_CLOEXEC };
        int r = ioctl(h, DMA_HEAP_IOCTL_ALLOC, &al);
        close(h);
        if (r == 0) return (int)al.fd;
    }
    if (getenv("HS_SRC") && !strcmp(getenv("HS_SRC"), "heap")) return -1;   /* (the test scripts: the dma-heap only) */
    struct { uint32_t memfd, flags; uint64_t offset, size; } uc = { 0 };
    int u = open("/dev/udmabuf", O_RDWR | O_CLOEXEC), m = (int)syscall(SYS_memfd_create, "epd-heapstate", 2u /* MFD_ALLOW_SEALING */), fd = -1;
    if (u >= 0 && m >= 0 && ftruncate(m, (off_t)len) == 0 && fcntl(m, 1033 /* F_ADD_SEALS */, 2 /* F_SEAL_SHRINK */) == 0) {
        uc.memfd = (uint32_t)m; uc.flags = 1; uc.size = len;
        fd = ioctl(u, _IOW('u', 0x42, typeof(uc)) /* UDMABUF_CREATE */, &uc);
        if (fd >= 0) fprintf(stderr, "epd-gpu: staggered steps: HEAPSTATE on a udmabuf (no dma-heap: the host's stand-in)\n");
    }
    if (m >= 0) close(m);
    if (u >= 0) close(u);
    return fd;
}

/* HEAPSTATE: C = D (ABGR8888 EPD_W x (EPD_H + lr), a pixel a texel: sampled and image-stored
 * through one immutable-storage texture), T (ABGR16161616 GW x EPD_H: 8 targets a texel,
 * the RG32UI bytes; written by the CPU through its map); pitches 256-B aligned */
static int stag_heap(epd_gpu *g)
{
    typedef void (*stor_fn)(GLenum, GLeglImageOES, const GLint *);
    stor_fn ts = (stor_fn)eglGetProcAddress("glEGLImageTargetTexStorageEXT");
    const char *ex = (const char *)g->glGetString(GL_EXTENSIONS);
    if (!ts || !ex || !strstr(ex, "GL_EXT_EGL_image_storage") || !g->eglCreateImageKHR) {
        fprintf(stderr, "epd-gpu: HEAPSTATE: no GL_EXT_EGL_image_storage / dma-buf import\n");
        return -1;
    }
    int dp = (EPD_W * 4 + 255) & ~255, dh = EPD_H + g->sg.lr;
    g->sg.tpitch = (GW * 8 + 255) & ~255;
    g->sg.tlen = ((size_t)g->sg.tpitch * EPD_H + 4095) & ~(size_t)4095;
    g->sg.hfd[0] = stag_heap_buf(((size_t)dp * (size_t)dh + 4095) & ~(size_t)4095);
    g->sg.hfd[1] = stag_heap_buf(g->sg.tlen);
    if (g->sg.hfd[0] < 0 || g->sg.hfd[1] < 0) { fprintf(stderr, "epd-gpu: HEAPSTATE: no heap\n"); return -1; }
    void *tm = mmap(NULL, g->sg.tlen, PROT_READ | PROT_WRITE, MAP_SHARED, g->sg.hfd[1], 0);
    if (tm == MAP_FAILED) { fprintf(stderr, "epd-gpu: HEAPSTATE: no map of T\n"); return -1; }
    g->sg.tmap = tm;
    const EGLint ad[] = { EGL_WIDTH, EPD_W, EGL_HEIGHT, dh, EGL_LINUX_DRM_FOURCC_EXT, 0x34324241 /* ABGR8888 */,
                          EGL_DMA_BUF_PLANE0_FD_EXT, g->sg.hfd[0], EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0, EGL_DMA_BUF_PLANE0_PITCH_EXT, dp, EGL_NONE };
    const EGLint at[] = { EGL_WIDTH, GW, EGL_HEIGHT, EPD_H, EGL_LINUX_DRM_FOURCC_EXT, 0x38344241 /* ABGR16161616 */,
                          EGL_DMA_BUF_PLANE0_FD_EXT, g->sg.hfd[1], EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0, EGL_DMA_BUF_PLANE0_PITCH_EXT, g->sg.tpitch, EGL_NONE };
    EGLImageKHR di = g->eglCreateImageKHR(g->dpy, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, NULL, ad);
    EGLImageKHR ti = g->eglCreateImageKHR(g->dpy, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, NULL, at);
    if (di == EGL_NO_IMAGE_KHR || ti == EGL_NO_IMAGE_KHR) { fprintf(stderr, "epd-gpu: HEAPSTATE: import refused (0x%x)\n", eglGetError()); return -1; }
    while (g->glGetError() != GL_NO_ERROR) {}
    g->glGenTextures(1, &g->sg.C);
    g->glBindTexture(GL_TEXTURE_2D, g->sg.C);
    ts(GL_TEXTURE_2D, (GLeglImageOES)di, NULL);
    g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    g->glGenTextures(1, &g->sg.T);
    g->glBindTexture(GL_TEXTURE_2D, g->sg.T);
    g->glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, (GLeglImageOES)ti);
    g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
    if (g->glGetError() != GL_NO_ERROR) { fprintf(stderr, "epd-gpu: HEAPSTATE: texture setup failed\n"); return -1; }
    fprintf(stderr, "epd-gpu: staggered steps: HEAPSTATE (C = heap D ABGR8888 pitch %d, T = heap ABGR16161616 pitch %d; timing/test only)\n", dp, g->sg.tpitch);
    return 0;
}

/* lanes : the engine's D (C: sampled as CS, stored as image 1)
 * and T (folded) imported, the acks A allocated; storage-EXT textures as
 * stag_heap's. -1: lanes impossible (the open fails: the caller runs runs) */
static int stag_lanes(epd_gpu *g)
{
    typedef void (*stor_fn)(GLenum, GLeglImageOES, const GLint *);
    stor_fn ts = (stor_fn)eglGetProcAddress("glEGLImageTargetTexStorageEXT");
    const char *ex = (const char *)g->glGetString(GL_EXTENSIONS);
    const epd_pk *k = g->eng->pk;
    if (!ts || !ex || !strstr(ex, "GL_EXT_EGL_image_storage") || !g->eglCreateImageKHR) {
        fprintf(stderr, "epd-gpu: lanes: no GL_EXT_EGL_image_storage / dma-buf import\n");
        return -1;
    }
    if (g->sg.lr > EPD_PK_DPAD) { fprintf(stderr, "epd-gpu: lanes: %d scratch rows, D has %d\n", g->sg.lr, EPD_PK_DPAD); return -1; }
    GLint mi = 0;
    g->glGetIntegerv(0x90CF /* GL_MAX_COMPUTE_IMAGE_UNIFORMS */, &mi);
    if (mi < 6) { fprintf(stderr, "epd-gpu: lanes: %d compute image units (6 needed)\n", mi); return -1; }
    int dh = EPD_H + g->sg.lr, dp = EPD_PK_S * 4;
    g->sg.apitch = (GW * 4 + 255) & ~255;
    g->sg.alen = ((size_t)g->sg.apitch * (size_t)dh + 4095) & ~(size_t)4095;
    g->sg.ldfd = dup(k->dfd);
    g->sg.ltfd = dup(k->tfd);
    g->sg.afd = stag_heap_buf(g->sg.alen);
    if (g->sg.ldfd < 0 || g->sg.ltfd < 0 || g->sg.afd < 0) { fprintf(stderr, "epd-gpu: lanes: no fds / no heap for A\n"); return -1; }
    void *am = mmap(NULL, g->sg.alen, PROT_READ | PROT_WRITE, MAP_SHARED, g->sg.afd, 0);
    if (am == MAP_FAILED) { fprintf(stderr, "epd-gpu: lanes: no map of A\n"); return -1; }
    g->sg.amap = am;
    const EGLint ad[] = { EGL_WIDTH, EPD_W, EGL_HEIGHT, dh, EGL_LINUX_DRM_FOURCC_EXT, 0x34324241 /* ABGR8888 */,
                          EGL_DMA_BUF_PLANE0_FD_EXT, g->sg.ldfd, EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0, EGL_DMA_BUF_PLANE0_PITCH_EXT, dp, EGL_NONE };
    const EGLint at[] = { EGL_WIDTH, 960, EGL_HEIGHT, EPD_H / 4, EGL_LINUX_DRM_FOURCC_EXT, 0x38344241 /* ABGR16161616 */,
                          EGL_DMA_BUF_PLANE0_FD_EXT, g->sg.ltfd, EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0, EGL_DMA_BUF_PLANE0_PITCH_EXT, EPD_PK_S * 4, EGL_NONE };
    const EGLint aa[] = { EGL_WIDTH, GW, EGL_HEIGHT, dh, EGL_LINUX_DRM_FOURCC_EXT, 0x34324241 /* ABGR8888 */,
                          EGL_DMA_BUF_PLANE0_FD_EXT, g->sg.afd, EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0, EGL_DMA_BUF_PLANE0_PITCH_EXT, g->sg.apitch, EGL_NONE };
    const EGLint *ats[3] = { ad, at, aa };
    for (int i = 0; i < 3; i++) {
        g->sg.limg[i] = g->eglCreateImageKHR(g->dpy, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, NULL, ats[i]);
        if (g->sg.limg[i] == EGL_NO_IMAGE_KHR) {
            g->sg.limg[i] = 0;
            fprintf(stderr, "epd-gpu: lanes: the %s import refused (0x%x)\n", i == 0 ? "D" : i == 1 ? "T" : "A", eglGetError());
            return -1;
        }
    }
    while (g->glGetError() != GL_NO_ERROR) {}
    GLuint *tx[3] = { &g->sg.C, &g->sg.T, &g->sg.A };
    for (int i = 0; i < 3; i++) {
        g->glGenTextures(1, tx[i]);
        g->glBindTexture(GL_TEXTURE_2D, *tx[i]);
        ts(GL_TEXTURE_2D, (GLeglImageOES)g->sg.limg[i], NULL);
        g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        g->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    }
    if (g->glGetError() != GL_NO_ERROR) { fprintf(stderr, "epd-gpu: lanes: texture setup failed\n"); return -1; }
    fprintf(stderr, "epd-gpu: staggered steps: lanes (C = the engine's D ABGR8888 pitch %d, T = its T folded ABGR16161616 960 x %d, "
            "A ABGR8888 pitch %d; %s; %d compute image units, %d scratch rows)\n", dp, EPD_H / 4, g->sg.apitch, k->dtsrc, mi, g->sg.lr);
    return 0;
}

/* lanes: H all parked (EPD_STAG_PARKL), A all 1 (the open; after a reset) */
static void stag_lanes_park(epd_gpu *g)
{
    int h = EPD_H + g->sg.lr;
    uint32_t *pk = malloc((size_t)GW * 4 * 4);
    if (!pk) return;
    for (int i = 0; i < GW * 4; i++) pk[i] = 0x30003000u;
    g->glBindTexture(GL_TEXTURE_2D, g->sg.H);
    for (int y = 0; y < h; y++) g->glTexSubImage2D(GL_TEXTURE_2D, 0, 0, y, GW, 1, GL_RGBA_INTEGER, GL_UNSIGNED_INT, pk);
    free(pk);
    g->glFinish();
    epd_dmabuf_sync(g->sg.afd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE, 0, 0);
    for (int y = 0; y < h; y++) memset(g->sg.amap + (size_t)y * g->sg.apitch, 0, (size_t)g->sg.apitch);
    for (int y = 0; y < h; y++)
        for (int x = 0; x < GW; x++) g->sg.amap[(size_t)y * g->sg.apitch + (size_t)x * 4] = 1;
    epd_dmabuf_sync(g->sg.afd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE, 0, 0);
    g->sg.mall = 1;
    g->sg.b4.park = 1;                  /* (ctl 4: its planes too) */
}

int epd_gpu_stag_lanes(const epd_gpu *g) { return g->sg.on && g->sg.lanes; }

void epd_gpu_stag_lanes_park(epd_gpu *g) { if (g->sg.lanes) stag_lanes_park(g); }

int epd_gpu_stag_acks(epd_gpu *g, uint8_t *ack, int y0, int y1)
{
    if (!g->sg.lanes) return -1;
    if (y0 < 0) y0 = 0;
    if (y1 > EPD_H) y1 = EPD_H;
    if (y1 <= y0) return 0;
    size_t off = (size_t)y0 * g->sg.apitch, len = (size_t)(y1 - y0) * g->sg.apitch;
    epd_dmabuf_sync(g->sg.afd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ, off, len);
    for (int y = y0; y < y1; y++) {
        const uint8_t *a = g->sg.amap + (size_t)y * g->sg.apitch;
        for (int x = 0; x < GW; x++) ack[(size_t)y * GW + x] = a[(size_t)x * 4];
    }
    epd_dmabuf_sync(g->sg.afd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ, off, len);
    return 0;
}


/* the targets of rows [y, y + h), texels [x0, x1), into T */
static void stag_tup(epd_gpu *g, const epd_stag *s, int x0, int x1, int y, int h)
{
    const uint8_t *src = s->tgt + (size_t)y * EPD_W + (size_t)x0 * 8;
    stag_note(g, x0, x1, y, y + h);
    if (g->sg.heapstate) {              /* (HEAPSTATE: the rows into the heap T; the frame that read it finished first) */
        struct dma_buf_sync sy = { DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE };
        g->glFinish();
        ioctl(g->sg.hfd[1], DMA_BUF_IOCTL_SYNC, &sy);
        for (int r = 0; r < h; r++)
            memcpy(g->sg.tmap + (size_t)(y + r) * (size_t)g->sg.tpitch + (size_t)x0 * 8, src + (size_t)r * EPD_W, (size_t)(x1 - x0) * 8);
        sy.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE;
        ioctl(g->sg.hfd[1], DMA_BUF_IOCTL_SYNC, &sy);
        return;
    }
    bind(g, GS_T, g->sg.T);
    g->glPixelStorei(GL_UNPACK_ROW_LENGTH, GW);
    g->glTexSubImage2D(GL_TEXTURE_2D, 0, x0, y, x1 - x0, h, GL_RG_INTEGER, GL_UNSIGNED_INT, src);
    g->glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
}

void epd_gpu_stag_enter(epd_gpu *g, const epd_stag *s)
{
    if (g->sg.lanes) { g->sg.mall = 1; return; }   /* (lanes: H stays parked between runs, nothing to load) */
    if (g->sg.px) {
        g->glUseProgram(g->sg.pxld);
        g->glUniform1i(g->sg.xl_clr, 1);
        stag_px_draw(g, 0, EPD_W, 0, EPD_H);
        g->sg.pm = 0;                   /* (pp: both pairs written) */
        /* (the next pass into fbpx reads what this one wrote: a draw into the
         * same framebuffer joins this batch (freedreno) and reads its textures
         * stale; the flush ends it. Host radeonsi too: 3k words a frame differed) */
        g->glFlush();
        stag_note(g, 0, GW, 0, EPD_H);
        stag_tup(g, s, 0, GW, 0, EPD_H);
        g->glBindFramebuffer(GL_FRAMEBUFFER, 0);
        g->glUseProgram(g->prog);
        return;
    }
    g->glMemoryBarrier(GL_ALL_BARRIER_BITS);    /* (a frame pass / store of the last run) */
    bind(g, GS_OC, g->sg.OC);
    int ld = stag_lead(g);
    g->glUseProgram(ld ? g->sg.lp[0] : g->sg.pload);
    g->glUniform1i(ld ? g->sg.ll_clr : g->sg.l_clr, 1);
    g->glBindImageTexture(0, g->sg.H, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32UI);
    g->glBindImageTexture(1, g->sg.C, 0, GL_FALSE, 0, GL_WRITE_ONLY, g->sg.heapstate ? GL_RGBA8 : GL_R32UI);
    stag_box(g, ld ? g->sg.ll_box : g->sg.l_box, ld, 0, GW, 0, EPD_H);
    stag_tup(g, s, 0, GW, 0, EPD_H);
    g->glUseProgram(g->prog);
}

void epd_gpu_stag_load(epd_gpu *g, epd_stag *s, const epd_stag_par *p, epd_rect r)
{
    if (r.x < 0) { r.w += r.x; r.x = 0; }
    if (r.y < 0) { r.h += r.y; r.y = 0; }
    if (r.x + r.w > EPD_W) r.w = EPD_W - r.x;
    if (r.y + r.h > EPD_H) r.h = EPD_H - r.y;
    if (r.w <= 0 || r.h <= 0 || g->sg.lanes) return;   /* (lanes: the arm bits in T) */
    int x0 = r.x / 8, x1 = (r.x + r.w + 7) / 8, w = (x1 - x0) * 8;   /* whole words */
    int y0 = -1, y1 = 0;
    for (int y = r.y; y < r.y + r.h; y++) {     /* the rows with fresh pixels, staged */
        size_t o = (size_t)y * EPD_W + (size_t)x0 * 8;
        uint32_t *d = g->sg.cbuf + (size_t)(y - r.y) * w;
        if (!memchr(s->ld + o, 2, (size_t)w)) { memset(d, 0, (size_t)w * 4); continue; }
        for (int i = 0; i < w; i++) {
            int f = s->ld[o + i] == 2;
            d[i] = f ? s->cold[o + i] | 4096u : 0;
            if (f) s->ld[o + i] = 1;
        }
        if (y0 < 0) y0 = y;
        y1 = y + 1;
    }
    if (y0 < 0) return;
    /* (an upload into LCU while the GPU still uses it goes through freedreno's
     * staging blit: the A509's 2D engine writes nothing there, so the entry's
     * loads were lost and a glFinish before the upload was the
     * stopgap; our Mesa patch 0003 takes the 3D path on the A509) */
    while (g->glGetError() != GL_NO_ERROR) {}
    bind(g, GS_LC, g->sg.LCU);
    g->glTexSubImage2D(GL_TEXTURE_2D, 0, x0 * 8, y0, w, y1 - y0, GL_RED_INTEGER, GL_UNSIGNED_INT,
                       g->sg.cbuf + (size_t)(y0 - r.y) * w);
    g->sg.lerr = g->glGetError();
    if (g->sg.px) {
        g->glUseProgram(g->sg.pxld);
        g->glUniform1i(g->sg.xl_clr, 0);
        g->glUniform3i(g->sg.xl_rcp, p->R, p->ctl, p->payrule ? 0 : p->pay);
        stag_px_draw(g, x0 * 8, x1 * 8, y0, y1);
        g->glFlush();                   /* (as the enter's) */
        stag_note(g, x0, x1, y0, y1);
        g->glBindFramebuffer(GL_FRAMEBUFFER, 0);
        g->glUseProgram(g->prog);
        return;
    }
    bind(g, GS_H, g->sg.H);
    bind(g, GS_OC, g->sg.OC);
    int ld = stag_lead(g);
    g->glUseProgram(ld ? g->sg.lp[0] : g->sg.pload);
    g->glUniform1i(ld ? g->sg.ll_clr : g->sg.l_clr, 0);
    g->glUniform3i(ld ? g->sg.ll_rcp : g->sg.l_rcp, p->R, p->ctl, p->payrule ? 0 : p->pay);
    g->glBindImageTexture(0, g->sg.H, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32UI);
    g->glBindImageTexture(1, g->sg.C, 0, GL_FALSE, 0, GL_WRITE_ONLY, g->sg.heapstate ? GL_RGBA8 : GL_R32UI);
    stag_box(g, ld ? g->sg.ll_box : g->sg.l_box, ld, x0, x1, y0, y1);
    g->glUseProgram(g->prog);
}

int epd_gpu_stag_load_err(epd_gpu *g) { return (int)g->sg.lerr; }

long epd_gpu_stag_load_probe(epd_gpu *g, epd_rect r, const uint8_t *ld)
{
    if (r.x < 0) { r.w += r.x; r.x = 0; }
    if (r.y < 0) { r.h += r.y; r.y = 0; }
    if (r.x + r.w > EPD_W) r.w = EPD_W - r.x;
    if (r.y + r.h > EPD_H) r.h = EPD_H - r.y;
    if (r.w <= 0 || r.h <= 0) return 0;
    int x0 = r.x / 8, x1 = (r.x + r.w + 7) / 8, w = (x1 - x0) * 8;
    if (!g->sg.fblu && !(g->sg.fblu = fbo_of(g, g->sg.LCU))) return -1;
    g->glMemoryBarrier(GL_ALL_BARRIER_BITS);
    g->glBindFramebuffer(GL_FRAMEBUFFER, g->sg.fblu);
    g->glReadPixels(x0 * 8, r.y, w, r.h, GL_RED_INTEGER, GL_UNSIGNED_INT, g->sg.cbuf);
    g->glBindFramebuffer(GL_FRAMEBUFFER, 0);
    long n = 0;
    for (int y = 0; y < r.h; y++)
        for (int i = 0; i < w; i++)
            n += ld[(size_t)(r.y + y) * EPD_W + (size_t)x0 * 8 + i] == 2 && (g->sg.cbuf[(size_t)y * w + i] & 4096u);
    return n;
}

void epd_gpu_stag_target(epd_gpu *g, const epd_stag *s, epd_rect r)
{
    if (r.x < 0) { r.w += r.x; r.x = 0; }
    if (r.y < 0) { r.h += r.y; r.y = 0; }
    if (r.x + r.w > EPD_W) r.w = EPD_W - r.x;
    if (r.y + r.h > EPD_H) r.h = EPD_H - r.y;
    if (r.w <= 0 || r.h <= 0) return;
    int x0 = r.x / 8, x1 = (r.x + r.w + 7) / 8;  /* whole texels */
    if (g->sg.lanes) { stag_note(g, x0, x1, r.y, r.y + r.h); return; }   /* (lanes: T is the engine's, written by the CPU) */
    stag_tup(g, s, x0, x1, r.y, r.h);
}

/* ---- stagger_ctl 4: ctl 3 on bit planes (epd-bp4.h; and
 *. Lanes only. The planes are the
 * GPU-owned lanes' state (S, RGBA32UI BP_NK x 7 EPD_H + 64 LEAD rows, layer L
 * at rows L EPD_H: LW P OW0 OW1 | -3 | AC0-3 | AC4 PD RV H8 | DC0-3 |
 * DC4-7 | ST PARK). On its group's frame a parked lane with ARM takes its D,
 * a loaded one with EVICT writes its final D and parks, a byte all parked
 * with an evict acks (A): the protocol of stag_cs's LANES block. Per frame:
 * the pack (the rects whose T changed: TB ARM EV TOG into Q, RGBA32UI BP_NK
 * x EPD_H + 64), the visit (even frames: rows vr + M j), the drive (arm /
 * evict / ack, the decisions, the words straight into the target or WD).
 * LEAD on every program, >= 8 px rows (the A509's lost early wave,
 * : a 4-px lead let a 32 x 1 drive lose words). The twin:
 * bp4_lane_rows; the gate: epd-stagger-test STAG_BP4_GPU */
double epd_gpu_bp4_ms[4];
#define B4_HEAD \
    "#version 310 es\n" \
    "precision highp int; precision highp usampler2D; precision highp uimage2D; precision highp image2D; precision highp sampler2D;\n" \
    "#define LH " STR(EPD_H) "\n#define NK " STR(BP_NK) "\n" \
    "uint heap_u32(highp vec4 v) { uvec4 u = uvec4(v * 255.0 + 0.5); return u.x | u.y << 8 | u.z << 16 | u.w << 24; }\n" \
    "uvec2 heap_u64(highp vec4 v) { uvec4 u = uvec4(v * 65535.0 + 0.5); return uvec2(u.x | u.y << 16, u.z | u.w << 16); }\n"
/* every lane parked, the rest of S 0 (no upload into a texture stores write: the A509) */
static const char *b4_park_cs =
    B4_HEAD
    "layout(local_size_x = 32, local_size_y = 4) in;\n"
    "layout(rgba32ui, binding = 0) writeonly uniform highp uimage2D S;\n"
    "void main() {\n"
    "  ivec2 c = ivec2(gl_GlobalInvocationID.xy) - ivec2(0, LEADY);\n"
    "  if (c.y < 0) { imageStore(S, ivec2(min(c.x, NK - 1), 8 * LH + LEADY + c.y), uvec4(uint(c.x))); return; }\n"
    "  if (c.x >= NK || c.y >= 8 * LH) return;\n"
    "  uint pk = c.y >= 6 * LH && c.y < 7 * LH ? (c.x == NK - 1 ? 0xFFFF0000u : 0xFFFFFFFFu) : 0u;\n"
    "  imageStore(S, c, uvec4(0u, pk, 0u, 0u));\n"
    "}\n";
/* T (the engine's, folded: 4 rows a texel row of 240) over box (plane words
 * k0..k1, rows y0..y1) into Q: TB (level >= 16), ARM, EV, TOG (bp4_lane_pack) */
static const char *b4_pack_cs =
    B4_HEAD
    "layout(local_size_x = 32, local_size_y = 4) in;\n"
    "uniform highp sampler2D TT;\n"
    "uniform ivec4 box;\n"
    "layout(rgba32ui, binding = 4) writeonly uniform highp uimage2D Q;\n"
    "void main() {\n"
    "  ivec2 g = ivec2(gl_GlobalInvocationID.xy) - ivec2(0, LEADY);\n"
    "  if (g.y < 0) { imageStore(Q, ivec2(min(g.x + box.x, NK - 1), LH + LEADY + g.y), uvec4(uint(g.x))); return; }\n"
    "  ivec2 c = g + box.xz;\n"
    "  if (c.x >= box.y || c.y >= box.w) return;\n"
    "  int tx = (c.y & 3) * 240 + 4 * c.x, ty = c.y >> 2;\n"   /* (the last word's 2 texels past the panel: the row's pad, masked) */
    "  uvec2 tt[4] = uvec2[4](heap_u64(texelFetch(TT, ivec2(tx, ty), 0)), heap_u64(texelFetch(TT, ivec2(tx + 1, ty), 0)),\n"
    "                         heap_u64(texelFetch(TT, ivec2(tx + 2, ty), 0)), heap_u64(texelFetch(TT, ivec2(tx + 3, ty), 0)));\n"
    "  uvec4 r = uvec4(0u);\n"
    "  for (int i = 0; i < 4; i++)\n"
    "    for (int b = 0; b < 8; b++) {\n"
    "      uint by = ((b < 4 ? tt[i].x : tt[i].y) >> (8u * uint(b & 3))) & 255u, bit = 1u << uint(31 - 8 * i - b);\n"
    "      r |= uvec4((by & 31u) >= 16u ? bit : 0u, (by & 64u) != 0u ? bit : 0u, (by & 128u) != 0u ? bit : 0u, (by & 32u) != 0u ? bit : 0u);\n"
    "    }\n"
    "  if (c.x == NK - 1) r &= uvec4(0xFFFF0000u);\n"
    "  imageStore(Q, c, r);\n"
    "}\n";
/* the visit: rows vr, vr + M, ... (bp4_visit_word, word for word; a parked
 * lane has AC and PD 0: nothing to do) */
static const char *b4_visit_cs =
    B4_HEAD
    "layout(local_size_x = 32, local_size_y = 4) in;\n"
    "uniform usampler2D SS;\n"
    "uniform int vr;\n"
    "uniform ivec4 par;\n"
    "layout(rgba32ui, binding = 0) writeonly uniform highp uimage2D S;\n"
    "#define N4 4\n"
    "void vadd(in uint a[N4], in uint b[N4], out uint r[N4]) { uint c = 0u; for (int i = 0; i < N4; i++) { uint x = a[i] ^ b[i]; r[i] = x ^ c; c = (a[i] & b[i]) | (x & c); } }\n"
    "void vsub(in uint a[N4], in uint b[N4], out uint r[N4]) { uint c = 0xFFFFFFFFu; for (int i = 0; i < N4; i++) { uint nb = ~b[i], x = a[i] ^ nb; r[i] = x ^ c; c = (a[i] & nb) | (x & c); } }\n"
    "void vsel(uint m, in uint a[N4], in uint b[N4], out uint r[N4]) { for (int i = 0; i < N4; i++) r[i] = (a[i] & m) | (b[i] & ~m); }\n"
    "void vc(out uint r[N4], int c) { for (int i = 0; i < N4; i++) r[i] = ((c >> i) & 1) != 0 ? 0xFFFFFFFFu : 0u; }\n"
    "uint vnz(in uint a[N4]) { return a[0] | a[1] | a[2] | a[3]; }\n"
    "uint vlt(in uint a[N4], in uint b[N4]) { uint d[N4]; vsub(a, b, d); return d[N4 - 1]; }\n"
    "void main() {\n"
    "  ivec2 g = ivec2(gl_GlobalInvocationID.xy) - ivec2(0, LEADY);\n"
    "  if (g.y < 0) { imageStore(S, ivec2(min(g.x, NK - 1), 8 * LH + LEADY + g.y), uvec4(uint(g.x))); return; }\n"
    "  int y = vr + par.w * g.y;\n"
    "  if (g.x >= NK || y >= LH) return;\n"
    "  ivec2 c = ivec2(g.x, y);\n"
    "  uvec4 h = texelFetch(SS, c, 0), q = texelFetch(SS, c + ivec2(0, LH), 0);\n"
    "  uvec4 a = texelFetch(SS, c + ivec2(0, 2 * LH), 0), e = texelFetch(SS, c + ivec2(0, 3 * LH), 0);\n"
    "  uint anz = a.x | a.y | a.z | a.w | e.x, pd = e.y;\n"
    "  if (anz == 0u && pd == 0u) return;\n"
    "  uvec4 d0 = texelFetch(SS, c + ivec2(0, 4 * LH), 0), d1 = texelFetch(SS, c + ivec2(0, 5 * LH), 0);\n"
    "  uint d[8] = uint[8](d0.x, d0.y, d0.z, d0.w, d1.x, d1.y, d1.z, d1.w);\n"
    "  if (anz != 0u) {\n"                  /* d += sext(acc), saturating at +-127 */
    "    uint av[5] = uint[5](a.x, a.y, a.z, a.w, e.x), cy = 0u, sm[8];\n"
    "    for (int i = 0; i < 8; i++) { uint bb = av[min(i, 4)], x = d[i] ^ bb; sm[i] = x ^ cy; cy = (d[i] & bb) | (x & cy); }\n"
    "    uint ovf = ~(d[7] ^ av[4]) & (sm[7] ^ d[7]);\n"
    "    uint m128 = sm[7] & ~(sm[0] | sm[1] | sm[2] | sm[3] | sm[4] | sm[5] | sm[6]) & ~ovf;\n"
    "    uint ng = d[7];\n"
    "    d[0] = sm[0] | ovf | m128;\n"
    "    for (int i = 1; i < 7; i++) d[i] = (sm[i] & ~ovf) | (~ng & ovf);\n"
    "    d[7] = (sm[7] & ~ovf) | (ng & ovf);\n"
    "    imageStore(S, c + ivec2(0, 4 * LH), uvec4(d[0], d[1], d[2], d[3]));\n"
    "    imageStore(S, c + ivec2(0, 5 * LH), uvec4(d[4], d[5], d[6], d[7]));\n"
    "    a = uvec4(0u); e.x = 0u;\n"
    "  }\n"
    "  uint go = pd & ~(q.x | q.y | q.z | q.w);\n"
    "  if (go != 0u) {\n"
    "    uint lw = h.x;\n"
    "    uint ge3 = ~d[7] & (d[6] | d[5] | d[4] | d[3] | d[2] | (d[1] & d[0]));\n"
    "    uint le3 = d[7] & ~(d[6] & d[5] & d[4] & d[3] & d[2] & d[1]);\n"
    "    uint big = (lw & le3) | (~lw & ge3), small = (lw & ge3) | (~lw & le3);\n"
    "    uint n0 = d[0], n1 = d[1] ^ d[0], n2 = d[2] ^ (d[1] | d[0]);\n"
    "    uint x0 = (lw & n0) | (~lw & d[0]), x1 = (lw & n1) | (~lw & d[1]), x2 = (lw & n2) | (~lw & d[2]);\n"
    "    x0 = (x0 & ~big & ~small) | big | small;\n"
    "    x1 = (x1 & ~big & ~small) | big;\n"
    "    x2 = (x2 & ~big & ~small) | small;\n"
    "    uint t[N4] = uint[N4](x0, x1, x2, x2), ow[N4] = uint[N4](h.z, h.w, 0u, 0u), z[N4], pc[N4], hcv[N4], paid[N4], u[N4], bv[N4];\n"
    "    vc(z, 0); vc(pc, par.y); vc(hcv, par.z);\n"
    "    vsub(hcv, ow, u); vsel(e.w, u, z, paid);\n"
    "    vadd(t, paid, u);\n"
    "    vsel(vlt(u, pc), u, pc, t); vsel(u[3] | ~vnz(u), z, t, bv);\n"
    "    if (par.z > 0) { vsel(e.z & vlt(bv, hcv), hcv, bv, u); bv = u; }\n"
    "    vsub(bv, paid, u); vsel(u[3], z, u, bv);\n"
    "    h.z = (bv[0] & go) | (h.z & ~go); h.w = (bv[1] & go) | (h.w & ~go);\n"
    "    e.y &= ~go; e.z &= ~go; e.w &= ~go;\n"
    "    imageStore(S, c, h);\n"
    "  }\n"
    "  imageStore(S, c + ivec2(0, 2 * LH), a);\n"
    "  imageStore(S, c + ivec2(0, 3 * LH), e);\n"
    "}\n";
/* the lanes (out of the drive, their own pass before it): per
 * plane word the arm (D in), the evict (D out, parked) and the acks
 * (bp4_lane_arm, _evict, _ack), on the words whose group frame it is and
 * whose requests (Q: ARM, EV) are set; every other invocation reads Q and
 * the mask and returns. Compiled into the drive, these blocks held the
 * registers of every invocation (the drive 4.5 ms on a static
 * frame); the drive after it reads the lanes' state as its own */
static const char *b4_lane_cs =
    B4_HEAD
    "layout(local_size_x = WX, local_size_y = WY) in;\n"
    "uniform usampler2D SS, QS, GMT;\n"
    "uniform highp sampler2D CS;\n"
    "uniform int g0, R;\n"
    "layout(rgba32ui, binding = 0) writeonly uniform highp uimage2D S;\n"
    "layout(rgba8, binding = 1) writeonly uniform highp image2D C;\n"
    "layout(rgba8, binding = 5) writeonly uniform highp image2D A;\n"
    "void main() {\n"
    "  ivec2 c = ivec2(gl_GlobalInvocationID.xy) - ivec2(0, LEADY);\n"
    "  if (c.y < 0) { imageStore(S, ivec2(min(c.x, NK - 1), 8 * LH + LEADY + c.y), uvec4(uint(c.x))); return; }\n"
    "  if (c.x >= NK || c.y >= LH) return;\n"
    "  uvec4 rq = texelFetch(QS, c, 0);\n"
    "  uint m0 = texelFetch(GMT, ivec2(c.x, g0 + (c.y & 63)), 0).r;\n"
    "  if (m0 == 0u || (rq.y | rq.z) == 0u) return;\n"
    "  uvec4 l6 = texelFetch(SS, c + ivec2(0, 6 * LH), 0);\n"
    "  uint ev = rq.z, st = l6.x, pk = l6.y;\n"
    "  uint am = m0 & pk & rq.y & ~ev, em = m0 & ~pk & ev;\n"
    "  if ((am | em) != 0u) {\n"
    "    uvec4 h = texelFetch(SS, c, 0), q = texelFetch(SS, c + ivec2(0, LH), 0);\n"
    "    uvec4 a = texelFetch(SS, c + ivec2(0, 2 * LH), 0), e = texelFetch(SS, c + ivec2(0, 3 * LH), 0);\n"
    "    uvec4 d0 = texelFetch(SS, c + ivec2(0, 4 * LH), 0), d1 = texelFetch(SS, c + ivec2(0, 5 * LH), 0);\n"
    "    uint lw = h.x, P = h.y, b0 = h.z, b1 = h.w;\n"
    /* (the armed bits cleared once, then each one OR'd in: the
     * arm frame's loop ~1,350 ops a full word) */
    "    uint na = ~am;\n"
    "    lw &= na; st &= na; q &= uvec4(na); d0 &= uvec4(na); d1 &= uvec4(na);\n"
    "    for (uint m = am; m != 0u; m &= m - 1u) {\n"
    "      uint b = uint(findLSB(m));\n"
    "      uint cv = heap_u32(texelFetch(CS, ivec2(32 * c.x + 31 - int(b), c.y), 0));\n"
    "      int l = int(cv & 31u), op = min(int((cv >> 5) & 63u), R), dc = clamp(int(cv) >> 16, -127, 127);\n"
    "      uint qv = uint(l >= 15 ? R - op : op), dv = uint(dc) & 255u;\n"   /* (level >= 15: white; an odd payrule level: its rail) */
    "      lw |= uint(l >= 15) << b;\n"
    "      st |= ((cv >> 11) & 1u) << b;\n"
    "      q |= ((uvec4(qv) >> uvec4(0u, 1u, 2u, 3u)) & 1u) << b;\n"
    "      d0 |= ((uvec4(dv) >> uvec4(0u, 1u, 2u, 3u)) & 1u) << b;\n"
    "      d1 |= ((uvec4(dv) >> uvec4(4u, 5u, 6u, 7u)) & 1u) << b;\n"
    "    }\n"
    "    if (em != 0u) {\n"                /* the evicted lanes' accumulator into dc (the visit's add, saturating at +-127) */
    "      uint av[5] = uint[5](a.x, a.y, a.z, a.w, e.x), d[8] = uint[8](d0.x, d0.y, d0.z, d0.w, d1.x, d1.y, d1.z, d1.w), cy = 0u, sm[8], n[8];\n"
    "      for (int i = 0; i < 8; i++) { uint bb = av[min(i, 4)], x = d[i] ^ bb; sm[i] = x ^ cy; cy = (d[i] & bb) | (x & cy); }\n"
    "      uint ovf = ~(d[7] ^ av[4]) & (sm[7] ^ d[7]);\n"
    "      uint m128 = sm[7] & ~(sm[0] | sm[1] | sm[2] | sm[3] | sm[4] | sm[5] | sm[6]) & ~ovf;\n"
    "      n[0] = sm[0] | ovf | m128;\n"
    "      for (int i = 1; i < 7; i++) n[i] = (sm[i] & ~ovf) | (~d[7] & ovf);\n"
    "      n[7] = (sm[7] & ~ovf) | (d[7] & ovf);\n"
    "      for (int i = 0; i < 8; i++) d[i] = (n[i] & em) | (d[i] & ~em);\n"
    "      d0 = uvec4(d[0], d[1], d[2], d[3]); d1 = uvec4(d[4], d[5], d[6], d[7]);\n"
    "    }\n"
    "    uint cl = am | em;\n"             /* (both: P, OW, AC, PD, RV, H8 cleared) */
    "    P &= ~cl; b0 &= ~cl; b1 &= ~cl; a &= uvec4(~cl); e &= uvec4(~cl);\n"
    "    pk = (pk & ~am) | em;\n"
    "    imageStore(S, c, uvec4(lw, P, b0, b1));\n"
    "    imageStore(S, c + ivec2(0, LH), q);\n"
    "    imageStore(S, c + ivec2(0, 2 * LH), a);\n"
    "    imageStore(S, c + ivec2(0, 3 * LH), e);\n"
    "    imageStore(S, c + ivec2(0, 4 * LH), d0);\n"
    "    imageStore(S, c + ivec2(0, 5 * LH), d1);\n"
    "    imageStore(S, c + ivec2(0, 6 * LH), uvec4(st, pk, l6.z, l6.w));\n"
    "#if KIDLE\n"
    "    uvec4 l7 = texelFetch(SS, c + ivec2(0, 7 * LH), 0);\n"
    "    imageStore(S, c + ivec2(0, 7 * LH), uvec4(l7.xyz & uvec3(~cl), l7.w));\n"
    "#endif\n"
    "    for (uint m = em; m != 0u; m &= m - 1u) {\n"     /* the evicted lanes' final D (k 0; the drive's decisions skip parked bits) */
    "      uint b = uint(findLSB(m)), w = (lw >> b) & 1u;\n"
    "      uint qv = ((q.x >> b) & 1u) | ((q.y >> b) & 1u) << 1 | ((q.z >> b) & 1u) << 2 | ((q.w >> b) & 1u) << 3;\n"
    "      uint dv = ((d0.x >> b) & 1u) | ((d0.y >> b) & 1u) << 1 | ((d0.z >> b) & 1u) << 2 | ((d0.w >> b) & 1u) << 3 |\n"
    "                ((d1.x >> b) & 1u) << 4 | ((d1.y >> b) & 1u) << 5 | ((d1.z >> b) & 1u) << 6 | ((d1.w >> b) & 1u) << 7;\n"
    "      uint op = w != 0u ? uint(R) - qv : qv, dc = uint(int(dv << 24) >> 24) & 65535u;\n"
    "      imageStore(C, ivec2(32 * c.x + 31 - int(b), c.y), unpackUnorm4x8((w != 0u ? 30u : 0u) | op << 5 | ((st >> b) & 1u) << 11 | dc << 16));\n"
    "    }\n"
    "  }\n"
    "  if (ev != 0u)\n"                    /* the acks: a byte M0 hits, all 8 parked, an evict among them (its first pixel's tog) */
    "    for (int k = 0; k < 4; k++) {\n"
    "      uint sh = uint(24 - 8 * k), eb = (ev >> sh) & 255u;\n"
    "      if (((m0 >> sh) & 255u) != 0u && ((pk >> sh) & 255u) == 255u && eb != 0u && 4 * c.x + k < " STR(GW) ")\n"
    "        imageStore(A, ivec2(4 * c.x + k, c.y), vec4(float(1u | ((rq.w >> (sh + uint(findMSB(eb)))) & 1u) << 1) / 255.0, 0.0, 0.0, 0.0));\n"
    "    }\n"
    "}\n";
/* the drive: per plane word (WX x WY workgroups) the decisions of the loaded
 * lanes (bp4_word), then the stores (all fetches first, the math, the stores
 * last: the A509); the lanes' arm / evict / ack ran before it (b4_lane_cs) */
static const char *b4_drive_cs =
    B4_HEAD
    "layout(local_size_x = WX, local_size_y = WY) in;\n"
    "uniform usampler2D SS, QS, GMT, TP;\n"
    "uniform int g0, gi, kt, pw, cyc;\n"
    "uniform ivec4 par;\n"              /* (R, pcap, min(htop, pcap) or 0, M) */
    "layout(rgba32ui, binding = 0) writeonly uniform highp uimage2D S;\n"
    "#if DIRECT\n"
    "layout(rgba8, binding = 3) writeonly uniform highp image2D F;\n"
    "#else\n"
    "layout(r32ui, binding = 2) writeonly uniform highp uimage2D O;\n"
    "#endif\n"
    "uint wire(uint wh, uint bk, uint sh) {\n"
    "  uint v = ((wh >> sh) & 255u) | ((bk >> sh) & 255u) << 16;\n"
    "  v = (v | v << 4) & 0x0F0F0F0Fu; v = (v | v << 2) & 0x33333333u; v = (v | v << 1) & 0x55555555u;\n"
    "  return (v & 0xFFFFu) << 1 | v >> 16;\n"
    "}\n"
    "void main() {\n"
    "  ivec2 c = ivec2(gl_GlobalInvocationID.xy) - ivec2(0, LEADY);\n"
    "  if (c.y < 0) { imageStore(S, ivec2(min(c.x, NK - 1), 8 * LH + LEADY + c.y), uvec4(uint(c.x))); return; }\n"
    "  if (c.x >= NK || c.y >= LH) return;\n"
    "  int line = " STR(DATA_LINE0) " + LH - 1 - c.y, col = " STR(DATA_COL0) " + 4 * c.x;\n"
    "  bool full = c.x < NK - 1;\n"     /* (the last plane word: 2 wire words) */
    "  uvec4 h = texelFetch(SS, c, 0), q0 = texelFetch(SS, c + ivec2(0, LH), 0), l6 = texelFetch(SS, c + ivec2(0, 6 * LH), 0);\n"
    "  uint tb = texelFetch(QS, c, 0).x;\n"
    "#if KIDLE\n"
    "  uvec4 l7 = texelFetch(SS, c + ivec2(0, 7 * LH), 0);\n"
    "#else\n"
    "  uvec4 l7 = uvec4(0u);\n"
    "#endif\n"
    "  uint m0 = texelFetch(GMT, ivec2(c.x, g0 + (c.y & 63)), 0).r, mi = texelFetch(GMT, ivec2(c.x, gi + (c.y & 63)), 0).r;\n"
    "#if DIRECT\n"
    "  uint t0 = texelFetch(TP, ivec2(col, line), 0).r, t1 = texelFetch(TP, ivec2(col + 1, line), 0).r;\n"
    "  uint t2 = texelFetch(TP, ivec2(col + 2, line), 0).r, t3 = texelFetch(TP, ivec2(col + 3, line), 0).r;\n"
    "#endif\n"
    "  uint lw = h.x, P = h.y, b0 = h.z, b1 = h.w, st = l6.x, pk = l6.y;\n"
    "  uvec4 q = q0, a0 = uvec4(0u), e0 = uvec4(0u), a = a0, e = e0;\n"
    "  uint kc[3] = uint[3](l7.x, l7.y, l7.z);\n"
    "  uint M0 = m0 & ~pk, drv = mi & P;\n"
    "  uint qnz = q.x | q.y | q.z | q.w, arrive = M0 & (lw ^ tb);\n"
    "  bool heavy = (M0 & ((lw ^ tb) | b0 | b1 | qnz)) != 0u;\n"
    "  if (heavy) {\n"
    "    a0 = texelFetch(SS, c + ivec2(0, 2 * LH), 0); e0 = texelFetch(SS, c + ivec2(0, 3 * LH), 0);\n"
    "    a = a0; e = e0;\n"
    "  }\n"
    "  if (M0 != 0u && !heavy) { P &= ~M0; if (kt >= 0) for (int i = 0; i < 3; i++) kc[i] &= ~M0; }\n"  /* (light: nothing to do) */
    "  else if (heavy) {\n"
    "    uint idle;\n"                     /* kt < 0: the shared word-diagonal idle (stagger_idle=shared) */
    "    if (kt < 0) idle = (uint(cyc) + uint((c.x + c.y) % 5)) % 5u == 4u ? 0xFFFFFFFFu : 0u;\n"
    "    else {\n"                          /* ctl 3's K: a full cycle idles when kc >= K / 2 (kt) */
    "      uint kb = 0u;\n"
    "      for (int i = 0; i < 3; i++) { uint b = ((kt >> i) & 1) != 0 ? 0xFFFFFFFFu : 0u; kb = (~kc[i] & b) | (~(kc[i] ^ b) & kb); }\n"
    "      idle = ~kb;\n"
    "    }\n"
    "    uint qv[4] = uint[4](q.x, q.y, q.z, q.w), n[4], br = 0u;\n"
    "    for (int i = 0; i < 4; i++) {\n"       /* need: q, or R - q at an arrival */
    "      uint r = ((par.x >> i) & 1) != 0 ? 0xFFFFFFFFu : 0u, d = r ^ qv[i] ^ br;\n"
    "      br = (~r & (qv[i] | br)) | (qv[i] & br);\n"
    "      n[i] = (d & arrive) | (qv[i] & ~arrive);\n"
    "    }\n"
    "    uint n1 = n[0] | n[1] | n[2] | n[3], n2 = n[1] | n[2] | n[3];\n"
    "    uint h8 = par.z > 0 ? arrive & qnz : 0u;\n"
    "    uint c0 = (par.z & 1) != 0 ? h8 : 0u, c1 = (par.z & 2) != 0 ? h8 : 0u;\n"
    "    b0 = (c0 & arrive) | (b0 & ~arrive); b1 = (c1 & arrive) | (b1 & ~arrive);\n"
    "    uint bnz = b0 | b1, act = M0 & (arrive | n1 | bnz);\n"
    "    uint ge1 = n1 | bnz, ge2 = n2 | b1 | (n1 & bnz);\n"
    "    uint nn2 = ge2 & ~idle, nn1 = ge1 & ~nn2;\n"
    "    uint p1 = (~n1 & nn1) | (n1 & ~n2 & nn2), p2 = ~n1 & nn2;\n"
    "    uint r0 = b0 ^ p1, bw = ~b0 & p1, r1 = b1 ^ p2 ^ bw;\n"
    "    uint qn[4], bo = 0u, sb[4] = uint[4](nn1, nn2, 0u, 0u);\n"
    "    for (int i = 0; i < 4; i++) { uint d = n[i] ^ sb[i] ^ bo; bo = (~n[i] & (sb[i] | bo)) | (sb[i] & bo); qn[i] = d; }\n"
    "    uint nnz = nn1 | nn2, ntb = ~tb & nnz;\n"
    "    uint v[5] = uint[5](nn1, nn2 | (~tb & nn1), ntb, ntb, ntb), av[5] = uint[5](a.x, a.y, a.z, a.w, e.x), cy = 0u;\n"
    "    for (int i = 0; i < 5; i++) { uint x = av[i] ^ v[i], s = x ^ cy; cy = (av[i] & v[i]) | (x & cy); av[i] = (s & act) | (av[i] & ~act); }\n"
    "    q = uvec4(((qn[0] & ~bo) & act) | (q.x & ~act), ((qn[1] & ~bo) & act) | (q.y & ~act),\n"
    "              ((qn[2] & ~bo) & act) | (q.z & ~act), ((qn[3] & ~bo) & act) | (q.w & ~act));\n"
    "    a = uvec4(av[0], av[1], av[2], av[3]);\n"
    "    e = uvec4(av[4], e.y | arrive, (e.z & ~arrive) | (arrive & qnz), (e.w & ~arrive) | h8);\n"
    "    st |= act & (nnz | arrive);\n"
    "    P = (P & ~M0) | (act & nn2);\n"
    "    b0 = (r0 & act) | (b0 & ~act); b1 = (r1 & act) | (b1 & ~act);\n"
    "    lw = (lw & ~act) | (tb & act);\n"
    "    drv |= act & ge1;\n"
    "    if (kt >= 0) {\n"                  /* kc: + 1 on a full cycle, else 0 */
    "      uint inc = act & nn2, ci = inc;\n"
    "      for (int i = 0; i < 3; i++) { uint sm = kc[i] ^ ci; ci = kc[i] & ci; kc[i] = (sm & inc) | (kc[i] & ~M0); }\n"
    "    }\n"
    "  }\n"
    /* the stores */
    "  uvec4 h2 = uvec4(lw, P, b0, b1), l62 = uvec4(st, pk, l6.z, l6.w);\n"
    "  if (h2 != h) imageStore(S, c, h2);\n"
    "  if (q != q0) imageStore(S, c + ivec2(0, LH), q);\n"
    "  if (heavy && a != a0) imageStore(S, c + ivec2(0, 2 * LH), a);\n"
    "  if (heavy && e != e0) imageStore(S, c + ivec2(0, 3 * LH), e);\n"
    "  if (l62 != l6) imageStore(S, c + ivec2(0, 6 * LH), l62);\n"
    "#if KIDLE\n"
    "  uvec4 l72 = uvec4(kc[0], kc[1], kc[2], l7.w);\n"
    "  if (l72 != l7) imageStore(S, c + ivec2(0, 7 * LH), l72);\n"
    "#endif\n"
    "  uint wh = drv & lw, bk = drv & ~lw;\n"
    "#if DIRECT\n"
    "  uint tv[4] = uint[4](t0, t1, t2, t3);\n"
    "  for (int k = 0; k < (full ? 4 : 2); k++) {\n"
    "    int i = line * pw + col + k;\n"
    "    imageStore(F, pw > 0 ? ivec2(i % " STR(FLAT_W) ", i / " STR(FLAT_W) ") : ivec2(col + k, line), unpackUnorm4x8(tv[k] | wire(wh, bk, uint(24 - 8 * k))));\n"
    "  }\n"
    "#else\n"
    "  for (int k = 0; k < (full ? 4 : 2); k++) imageStore(O, ivec2(4 * c.x + k, c.y), uvec4(wire(wh, bk, uint(24 - 8 * k))));\n"
    "#endif\n"
    "}\n";

/* lanes + drive as ONE program, a uniform pass picking the body, still two
 * dispatches (the program switch between the lanes and the
 * drive dispatch is what hangs the A509 under foreign load; one program 0
 * hangs in 23,234 frames, separate 15-16 per 30 s; same branches, registers,
 * constlen). Built from the two sources: their mains renamed, the union of
 * their declarations in front. Every image it declares is bound before the
 * first dispatch (Mesa segfaults otherwise) */
static char *b4_uber_src(void)
{
    const char *lb = strstr(b4_lane_cs, "void main() {"), *db = strstr(b4_drive_cs, "uint wire(");
    const char *dm = db ? strstr(db, "void main() {") : NULL;
    if (!lb || !dm) return NULL;
    static const char *decl =
        "layout(local_size_x = WX, local_size_y = WY) in;\n"
        "uniform usampler2D SS, QS, GMT, TP;\n"
        "uniform highp sampler2D CS;\n"
        "uniform int g0, R, gi, kt, pw, cyc, pass;\n"
        "uniform ivec4 par;\n"
        "layout(rgba32ui, binding = 0) writeonly uniform highp uimage2D S;\n"
        "layout(rgba8, binding = 1) writeonly uniform highp image2D C;\n"
        "layout(rgba8, binding = 5) writeonly uniform highp image2D A;\n"
        "#if DIRECT\n"
        "layout(rgba8, binding = 3) writeonly uniform highp image2D F;\n"
        "#else\n"
        "layout(r32ui, binding = 2) writeonly uniform highp uimage2D O;\n"
        "#endif\n";
    static const char *tail = "void main() { if (pass == 0) lanes(); else drive(); }\n";
    const size_t k = strlen("void main() {");
    size_t n = strlen(B4_HEAD) + strlen(decl) + strlen(lb) + strlen(db) + strlen(tail) + 64;
    char *s = malloc(n);
    if (s) snprintf(s, n, "%s%svoid lanes() {%s%.*svoid drive() {%s%s", B4_HEAD, decl, lb + k, (int)(dm - db), db, dm + k, tail);
    return s;
}

static GLuint b4_prog(epd_gpu *g, const char *src, const char *defs, const char *what)
{
    size_t n = strlen(src) + strlen(defs) + 2;
    char *s = malloc(n);
    if (!s) return 0;
    const char *nl = strchr(src, '\n');         /* (the defines after #version) */
    snprintf(s, n, "%.*s%s%s", (int)(nl - src + 1), src, defs, nl + 1);
    GLuint p = cs_prog(g, s, what);
    free(s);
    return p;
}

/* the textures and programs (once; -1: ctl 4 not available, ctl 3 runs) */
static int b4_open(epd_gpu *g)
{
    if (g->sg.b4.on) return g->sg.b4.on > 0 ? 0 : -1;
    g->sg.b4.on = -1;
    if (!g->sg.lanes) return -1;
#ifndef BP4_WX_DEF                               /* (59 x 4: the exact grid, BP_NK x EPD_H; see the lanes below) */
#define BP4_WX_DEF 59
#define BP4_WY_DEF 4
#endif
    int wx = BP4_WX_DEF, wy = BP4_WY_DEF;
    const char *v = getenv("GLASSINE_BP4_SHAPE");      /* (tests: the drive's and the lanes' workgroup, WxH) */
    if (v && (sscanf(v, "%dx%d", &wx, &wy) != 2 || wx < 1 || wy < 1 || wx * wy > 256 || wy > 64)) { wx = BP4_WX_DEF; wy = BP4_WY_DEF; }
    g->sg.b4.wx = wx; g->sg.b4.wy = wy;
    g->sg.b4.dlead = (8 + wy - 1) / wy;           /* (the drive's lead: >= 8 px rows) */
    char d1[64], d2[96], d3[96], d4[96];
    /* GLASSINE_BP4_KIDLE=0 (a hang bisect): no per-pixel K idle in the programs (the shared
     * idle, layer 7 untouched; the twin no longer matches) */
#ifndef BP4_KIDLE_DEF
#define BP4_KIDLE_DEF 1
#endif
    int kid = getenv("GLASSINE_BP4_KIDLE") ? strcmp(getenv("GLASSINE_BP4_KIDLE"), "0") != 0 : BP4_KIDLE_DEF;
    g->sg.b4.kid = kid;
    snprintf(d1, sizeof d1, "#define LEADY 8\n");   /* (the 32 x 4 programs: 2 rows) */
    snprintf(d2, sizeof d2, "#define LEADY %d\n#define WX %d\n#define WY %d\n#define DIRECT 1\n#define KIDLE %d\n", g->sg.b4.dlead * wy, wx, wy, kid);
    /* the lanes: the drive's shape and lead. The lanes -> drive pair hangs the A509 under foreign GPU
     * load (a lost wave that ends in a hang): each cure alone held
     * there, a full lead row of real stores on both dispatches or no bounds early return (an exact
     * grid); GLASSINE_BP4_SHAPE=59x4 gives both (59 words x 1404 rows: no invocation out of bounds) */
    snprintf(d4, sizeof d4, "#define LEADY %d\n#define WX %d\n#define WY %d\n#define KIDLE %d\n", g->sg.b4.dlead * wy, wx, wy, kid);
    snprintf(d3, sizeof d3, "#define LEADY %d\n#define WX %d\n#define WY %d\n#define DIRECT 0\n#define KIDLE %d\n", g->sg.b4.dlead * wy, wx, wy, kid);
    if (!(g->sg.b4.ppk = b4_prog(g, b4_park_cs, d1, "ctl 4 park")) || !(g->sg.b4.pp = b4_prog(g, b4_pack_cs, d1, "ctl 4 pack")) ||
        !(g->sg.b4.pv = b4_prog(g, b4_visit_cs, d1, "ctl 4 visit")) || !(g->sg.b4.pl = b4_prog(g, b4_lane_cs, d4, "ctl 4 lanes")) || !(g->sg.b4.pd[1] = b4_prog(g, b4_drive_cs, d2, "ctl 4 drive")) ||
        !(g->sg.b4.pd[0] = b4_prog(g, b4_drive_cs, d3, "ctl 4 drive (WD)"))) {
        fprintf(stderr, "epd-gpu: stagger_ctl 4: a program failed: ctl 3 runs\n");
        return -1;
    }
    GLuint p = g->sg.b4.pp;
    g->glUseProgram(p);
    U1I(p, "TT", GS_T);
    g->sg.b4.u_box = g->glGetUniformLocation(p, "box");
    p = g->sg.b4.pv;
    g->glUseProgram(p);
    U1I(p, "SS", GS_H);
    g->sg.b4.u_vr = g->glGetUniformLocation(p, "vr");
    g->sg.b4.u_vpar = g->glGetUniformLocation(p, "par");
    p = g->sg.b4.pl;
    g->glUseProgram(p);
    U1I(p, "SS", GS_H); U1I(p, "QS", GS_OC); U1I(p, "GMT", GS_GM); U1I(p, "CS", GS_C);
    g->sg.b4.u_l[0] = g->glGetUniformLocation(p, "g0");
    g->sg.b4.u_l[1] = g->glGetUniformLocation(p, "R");
    /* the lanes and the drive as two programs by default (3.08 vs 4.05 ms a frame on a real
     * scroll); needs a kernel fix (a5xx CP_RB_WPTR under the ring's preempt_lock): before
     * it two programs hung ~4 times in 10 min under foreign GPU load, after it 0 in 45,617 frames.
     * GLASSINE_BP4_UBER=1: the one program (the older kernels' guard) */
    g->sg.b4.uber = getenv("GLASSINE_BP4_UBER") && atoi(getenv("GLASSINE_BP4_UBER")) != 0;
    if (g->sg.b4.uber) {
        char *us = b4_uber_src();
        if (!us || !(g->sg.b4.pu[1] = b4_prog(g, us, d2, "ctl 4 lanes + drive")) || !(g->sg.b4.pu[0] = b4_prog(g, us, d3, "ctl 4 lanes + drive (WD)"))) {
            free(us);
            fprintf(stderr, "epd-gpu: stagger_ctl 4: the one-program lanes + drive failed: ctl 3 runs\n");
            return -1;
        }
        free(us);
        for (int i = 0; i < 2; i++) g->sg.b4.pd[i] = g->sg.b4.pu[i];
    }
    for (int i = 0; i < 2; i++) {
        p = g->sg.b4.pd[i];
        g->glUseProgram(p);
        U1I(p, "SS", GS_H); U1I(p, "QS", GS_OC); U1I(p, "GMT", GS_GM); U1I(p, "TP", 0);
        static const char *un[6] = { "g0", "gi", "kt", "pw", "par", "cyc" };
        for (int j = 0; j < 6; j++) g->sg.b4.u_d[i][j] = g->glGetUniformLocation(p, un[j]);
        if (g->sg.b4.uber) {
            U1I(p, "CS", GS_C);
            static const char *uu[3] = { "R", "pass", "g0" };
            for (int j = 0; j < 3; j++) g->sg.b4.u_u[i][j] = g->glGetUniformLocation(p, uu[j]);
        }
    }
    g->glUseProgram(g->prog);
    while (g->glGetError() != GL_NO_ERROR) {}
    g->sg.b4.S = tex(g, GL_RGBA32UI, BP_NK, 8 * EPD_H + 64);
    g->sg.b4.Q = tex(g, GL_RGBA32UI, BP_NK, EPD_H + 64);
    g->sg.b4.GM = tex(g, GL_R32UI, BP_NK, 128);
    if (!(g->sg.b4.fbS = fbo_of(g, g->sg.b4.S)) || g->glGetError() != GL_NO_ERROR) { fprintf(stderr, "epd-gpu: stagger_ctl 4: texture setup failed: ctl 3 runs\n"); return -1; }
    if (g->sg.gen_q) g->sg.gen_q(4, g->sg.b4.tq);
    g->sg.b4.mkey = -1;
    g->sg.b4.park = 1;
    g->sg.b4.on = 1;
    fprintf(stderr, "epd-gpu: stagger_ctl 4: bit planes (%.1f MB), drive %dx%d, LEAD %d px, lanes + drive %s\n",
            (double)BP_NK * (9 * EPD_H + 128) * 16 / 1e6 + BP_NK * 128 * 4 / 1e6, wx, wy, g->sg.b4.dlead * wy, g->sg.b4.uber ? "one program" : "two programs");
    return 0;
}

int epd_gpu_bp4_open(epd_gpu *g) { return b4_open(g); }

/* the planes as RGBA32UI texels, BP_NK x 7 EPD_H (tests, stagger=check) */
void epd_gpu_bp4_read(epd_gpu *g, uint32_t *u)
{
    if (g->sg.b4.on <= 0) return;
    g->glMemoryBarrier(GL_ALL_BARRIER_BITS);
    g->glFinish();
    g->glBindFramebuffer(GL_FRAMEBUFFER, g->sg.b4.fbS);
    g->glReadPixels(0, 0, BP_NK, 8 * EPD_H, GL_RGBA_INTEGER, GL_UNSIGNED_INT, u);
    g->glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

/* frame f of ctl 4: the masks (a new layout), the park (open, a reset), the
 * pack over the noted rects, the visit, the drive (direct: into the target) */
static void b4_frame(epd_gpu *g, const epd_stag_par *p, uint32_t f, struct target *tg, int direct, int tm)
{
    int mk = p->stripe << 8 ^ p->cellw << 4 ^ p->cellh << 20 ^ p->cellord << 28 ^ p->blocks << 2;
    if (mk != g->sg.b4.mkey) {
        bp_masks m;
        epd_stag_par q = *p;
        q.G = 2;
        if (bp_masks_make(&m, &q)) return;
        bind(g, GS_GM, g->sg.b4.GM);
        g->glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, BP_NK, 128, GL_RED_INTEGER, GL_UNSIGNED_INT, m.gm);
        bp_masks_free(&m);
        g->sg.b4.mkey = mk;
    }
    g->glMemoryBarrier(GL_ALL_BARRIER_BITS);
    double ms[3] = { 0, 0, 0 };
    if (g->sg.b4.park) {
        g->glUseProgram(g->sg.b4.ppk);
        g->glBindImageTexture(0, g->sg.b4.S, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32UI);
        g->glDispatchCompute((BP_NK + 31) / 32, (8 * EPD_H + 3) / 4 + 2, 1);
        g->glMemoryBarrier(GL_ALL_BARRIER_BITS);
        g->sg.b4.park = 0;
        g->sg.mall = 1;
    }
    int k0 = BP_NK, k1 = 0, y0 = EPD_H, y1 = 0;     /* the pack: the noted rects' bounds (GW texels -> plane words) */
    if (g->sg.mall) { k0 = 0; k1 = BP_NK; y0 = 0; y1 = EPD_H; }
    for (int i = 0; i < g->sg.nrbx; i++) {
        int a = g->sg.rbx[i].x0 / 4, b = (g->sg.rbx[i].x1 + 3) / 4;
        if (a < k0) k0 = a;
        if (b > k1) k1 = b;
        if (g->sg.rbx[i].y0 < y0) y0 = g->sg.rbx[i].y0;
        if (g->sg.rbx[i].y1 > y1) y1 = g->sg.rbx[i].y1;
    }
    g->sg.nrbx = 0;
    g->sg.mall = 0;
    if (k1 > BP_NK) k1 = BP_NK;
    if (k1 > k0 && y1 > y0) {
        g->glUseProgram(g->sg.b4.pp);
        bind(g, GS_T, g->sg.T);
        g->glBindImageTexture(4, g->sg.b4.Q, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32UI);
        g->glUniform4i(g->sg.b4.u_box, k0, k1, y0, y1);
        if (tm) g->sg.begin_q(0x88BF, g->sg.b4.tq[0]);
        g->glDispatchCompute((GLuint)(k1 - k0 + 31) / 32, (GLuint)(y1 - y0 + 3) / 4 + 2, 1);
        if (tm) g->sg.end_q(0x88BF);
        g->glMemoryBarrier(GL_ALL_BARRIER_BITS);
        ms[0] = -1;
    }
    int M = bp4_vm(p), hc = p->htop > 0 ? (p->htop < p->pcap ? p->htop : p->pcap) : 0;
    if (getenv("GLASSINE_BP4_BREAK")) hc = 0;           /* (tests: the negative control, the GPU's hammer off) */
    if (f % 2 == 0) {
        int vr = (int)((f / 2) % (uint32_t)M), rows = (EPD_H - vr + M - 1) / M;
        g->glUseProgram(g->sg.b4.pv);
        g->glUniform1i(g->sg.b4.u_vr, vr);
        g->glUniform4i(g->sg.b4.u_vpar, p->R, p->pcap, hc, M);
        bind(g, GS_H, g->sg.b4.S);
        g->glBindImageTexture(0, g->sg.b4.S, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32UI);
        if (tm) g->sg.begin_q(0x88BF, g->sg.b4.tq[1]);
        g->glDispatchCompute((BP_NK + 31) / 32, (GLuint)(rows + 3) / 4 + 2, 1);
        if (tm) g->sg.end_q(0x88BF);
        g->glMemoryBarrier(GL_ALL_BARRIER_BITS);
        ms[1] = -1;
    }
    int fm = (int)(f & 1);
    /* the lanes first (arm, evict, acks; their own timer, added to the drive's: one
     * query over both dispatches hung the A509, 4 hangcheck recovers in 10 min) */
    if (g->sg.b4.uber) {                /* (one program, pass 0; every binding of both bodies set first) */
        GLint *uu = g->sg.b4.u_u[direct];
        g->glUseProgram(g->sg.b4.pd[direct]);
        g->glUniform1i(uu[2], fm * 64);
        g->glUniform1i(uu[0], p->R);
        g->glUniform1i(uu[1], 0);
        if (direct) {
            bind(g, 0, g->tpl);
            g->glBindImageTexture(3, tg->itex, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA8);
        } else
            g->glBindImageTexture(2, g->sg.WD, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R32UI);
    } else {
        g->glUseProgram(g->sg.b4.pl);
        g->glUniform1i(g->sg.b4.u_l[0], fm * 64);
        g->glUniform1i(g->sg.b4.u_l[1], p->R);
    }
    bind(g, GS_H, g->sg.b4.S);
    bind(g, GS_OC, g->sg.b4.Q);
    bind(g, GS_GM, g->sg.b4.GM);
    bind(g, GS_C, g->sg.C);
    g->glBindImageTexture(0, g->sg.b4.S, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32UI);
    g->glBindImageTexture(1, g->sg.C, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA8);
    g->glBindImageTexture(5, g->sg.A, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA8);
    if (tm) g->sg.begin_q(0x88BF, g->sg.b4.tq[3]);
    g->glDispatchCompute((GLuint)(BP_NK + g->sg.b4.wx - 1) / (GLuint)g->sg.b4.wx, (GLuint)(EPD_H + g->sg.b4.wy - 1) / (GLuint)g->sg.b4.wy + (GLuint)g->sg.b4.dlead, 1);
    if (tm) g->sg.end_q(0x88BF);
    g->glMemoryBarrier(GL_ALL_BARRIER_BITS);
    GLuint dp = g->sg.b4.pd[direct];
    GLint *u = g->sg.b4.u_d[direct];
    g->glUseProgram(dp);
    if (g->sg.b4.uber) g->glUniform1i(g->sg.b4.u_u[direct][1], 1);     /* (the same program: pass 1, the drive) */
    g->glUniform1i(u[0], fm * 64);
    g->glUniform1i(u[1], (fm ^ 1) * 64);
    g->glUniform1i(u[2], p->sidle || !g->sg.b4.kid ? -1 : (p->K < 0 ? 0 : p->K > 15 ? 15 : p->K) / 2);
    g->glUniform1i(u[5], (int)(f / 2));
    g->glUniform4i(u[4], p->R, p->pcap, hc, M);
    bind(g, GS_H, g->sg.b4.S);
    bind(g, GS_OC, g->sg.b4.Q);
    bind(g, GS_GM, g->sg.b4.GM);
    g->glBindImageTexture(0, g->sg.b4.S, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32UI);
    if (direct) {
        bind(g, 0, g->tpl);
        g->glUniform1i(u[3], tg->pw);
        g->glBindImageTexture(3, tg->itex, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA8);
    } else
        g->glBindImageTexture(2, g->sg.WD, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R32UI);
    if (tm) g->sg.begin_q(0x88BF, g->sg.b4.tq[2]);
    g->glDispatchCompute((GLuint)(BP_NK + g->sg.b4.wx - 1) / (GLuint)g->sg.b4.wx, (GLuint)(EPD_H + g->sg.b4.wy - 1) / (GLuint)g->sg.b4.wy + (GLuint)g->sg.b4.dlead, 1);
    if (tm) g->sg.end_q(0x88BF);
    g->glMemoryBarrier(GL_ALL_BARRIER_BITS);
    if (tm) {
        for (int i = 0; i < 3; i++) {
            GLuint64 ns = 0;
            if (i == 2 || ms[i] < 0) { g->sg.get_q(g->sg.b4.tq[i], 0x8866, &ns); ms[i] = ns / 1e6; }
            if (i == 2) { g->sg.get_q(g->sg.b4.tq[3], 0x8866, &ns); ms[2] += ns / 1e6; }
            epd_gpu_bp4_ms[i] += ms[i];
            epd_gpu_stag_ms[0] += ms[i];
        }
        epd_gpu_bp4_ms[3]++;
        epd_gpu_stag_ms[2]++;
    }
}

void epd_gpu_stag_set_direct(epd_gpu *g, int on) { g->sg.direct = on != 0; }

void epd_gpu_stag_frame(epd_gpu *g, const epd_stag_par *p, uint32_t f, int target)
{
    struct target *tg = &g->t[target];
    int direct = !g->sg.px && g->sg.direct && tg->itex && tg->drawn;
    int b4 = 0, tm = epd_gpu_stag_timer && g->sg.tq[0];
    epd_stag_par p3;
    if (p->ctl == 4) {                  /* (stagger_ctl 4: the bit planes; not available: ctl 3) */
        if (g->sg.lanes && !g->sg.px && !b4_open(g)) {
            /* (no GPU timer queries in ctl 4 unless a bench asks, GLASSINE_BP4_TIMER=1: with the
             * lane pass they hung the A509, 4 hangcheck recovers per 10 min soak, with one
             * query over both passes and with one per pass; 0 with the timer off;
             * see freedreno/msm) */
            static int bt = -1;
            if (bt < 0) bt = getenv("GLASSINE_BP4_TIMER") && atoi(getenv("GLASSINE_BP4_TIMER")) > 0;
            b4_frame(g, p, f, tg, direct, tm && bt);
            b4 = 1;
            goto emit;
        }
        p3 = *p;
        p3.ctl = 3;
        p = &p3;
    }
    int nj = 1;                         /* the most octants a group of G spans (G >= 2: <= 5; G 1, A2/1: all 8) */
    for (int fm = 0; fm < p->G; fm++) {
        int lo = (4096 * fm + p->G - 1) / p->G, hi = (4096 * (fm + 1) + p->G - 1) / p->G - 1;
        if ((hi >> 9) - (lo >> 9) + 1 > nj) nj = (hi >> 9) - (lo >> 9) + 1;
    }
    if (nj > 5 && nj != 8) return;
    int stw = !g->sg.px && p->stripe > 0 && p->G > 1;  /* (64 px stripes: a word's pixels all in its stripe's group) */
    const uint16_t *ctab = stw && p->cellw ? epd_stag_cells(p->cellw, p->cellh, p->cellord) : NULL;  /* (cells, their tables) */
    int cel = ctab != NULL;
    int blk = stw && p->blocks && !cel; /* (the stripes cut into 64 x 64 blocks, epd_stag_brank) */
    if (epd_gpu_stag_cremap < 0) {
        const char *v = getenv("GLASSINE_STAG_CREMAP");
        epd_gpu_stag_cremap = v ? atoi(v) > 0 : 1;
    }
    int crm = cel && g->sg.swy == 64 && epd_gpu_stag_cremap;   /* (a workgroup a tile: its cells in rank order) */
    if (g->sg.dgen != epd_stag_dgen) {  /* (a new dither pattern: the tables, the whole panel) */
        if (stag_dtabs(g)) return;
        g->sg.mall = 1;
    }
    if (stw) nj = 8;
    else if (g->sg.lanes && p->G > 1 && epd_gpu_stag_lpx()) nj = 7;  /* (lanes' per-pixel shape, stag_prog) */
    int pr = p->ctl && p->payrule;
    int key = (p->ctl != 0) | (p->ctl && p->pay > 0 && !pr) << 1 | (p->dither != 0) << 2 | (p->drain != 0) << 3 | direct << 4 | (nj & 7) << 5 |
              (p->ctl >= 2 ? p->ctl - 1 : 0) << 8 | pr << 10 | (pr && p->htop > 0) << 14 | stw << 15 | blk << 12 | cel << 5 | crm << 6;
    if (epd_gpu_stag_tskip < 0) {
        const char *v = getenv("GLASSINE_STAG_TSKIP");
        epd_gpu_stag_tskip = v ? (atoi(v) > 0 ? atoi(v) : 0) : 1;
    }
    {                                   /* a big change (> 40 % of the panel) in the last 8 frames: the plain pass */
        long a = 0;
        for (int i = 0; i < g->sg.nrbx; i++)
            a += (long)(g->sg.rbx[i].x1 - g->sg.rbx[i].x0) * (g->sg.rbx[i].y1 - g->sg.rbx[i].y0);
        if (a * 5 > (long)GW * EPD_H * 2) g->sg.tbig = g->sg.fid + 8;
    }
    /* the tile skip pays on part of a panel only: a whole panel in motion
     * has no idle tile and the test costs it (A509: +1.1 ms a scroll frame,
     * measured); a switch changes the key, so the params' signature notes
     * the whole panel (stale stamps never count) */
    int tsk = epd_gpu_stag_tskip && !p->drain && !g->sg.px && (int)(g->sg.fid - g->sg.tbig) >= 0;
    key |= tsk << 13;
    int ld = stag_lead(g);              /* (LEAD: the mark and the frame pass, a leading workgroup row each) */
    key |= ld << 11;
    {                                   /* the params' signature: a change counts as the whole panel's */
        uint32_t sg = (uint32_t)key * 2654435761u;
        int pv[] = { p->G, p->n, p->pay, p->K, p->R, p->pcap, p->dither, p->htop, p->stripe, p->blocks, p->cellw, p->cellh, p->cellord };
        for (unsigned i = 0; i < sizeof pv / sizeof *pv; i++) sg = (sg ^ (uint32_t)pv[i]) * 16777619u;
        if (sg != g->sg.psig) g->sg.mall = 1;
        g->sg.psig = sg;
    }
    if (tsk && g->sg.mall) {            /* (the whole panel: one rect) */
        g->sg.nrbx = 1;
        g->sg.rbx[0] = (typeof(g->sg.rbx[0])){ 0, GW, 0, EPD_H, g->sg.fid };
    }
    g->sg.mall = 0;
    if (tsk && g->sg.nrbx) {            /* the changes since the last frame: their tiles stamped */
        int wx = stw ? 8 : 32, wy = stw ? g->sg.swy : g->sg.wy;   /* (a workgroup's tile; a switch: a new key, the whole panel) */
        g->glUseProgram(ld ? g->sg.lp[2] : g->sg.pmark);
        g->glBindImageTexture(4, g->sg.AT, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R32UI);
        g->glUniform1ui(ld ? g->sg.lm_fid : g->sg.m_fid, g->sg.fid);
        for (int i = 0; i < g->sg.nrbx; i++) {
            int tx0 = g->sg.rbx[i].x0 / wx, tx1 = (g->sg.rbx[i].x1 + wx - 1) / wx;
            int ty0 = g->sg.rbx[i].y0 / wy, ty1 = (g->sg.rbx[i].y1 + wy - 1) / wy;
            if (tx1 <= tx0 || ty1 <= ty0) continue;
            g->glUniform4i(ld ? g->sg.lm_box : g->sg.m_box, tx0, tx1, ty0, ty1);
            g->glDispatchCompute((GLuint)(tx1 - tx0 + 7) / 8, (GLuint)(ty1 - ty0 + 7) / 8 + ld, 1);
        }
        g->glMemoryBarrier(GL_ALL_BARRIER_BITS);
    }
    g->sg.nrbx = 0;
    if (g->sg.px) {
        int kx = (key & ~(1 << 4 | 7 << 5 | 1 << 12 | 1 << 13 | 1 << 15)) | (nj != 8) << 5;   /* (G 1 or not) */
        static int bwarn;
        if ((p->blocks || p->cellw) && p->stripe > 0 && !bwarn++)
            fprintf(stderr, "epd-gpu: stagger: px draws %s as column stripes (not the CPU reference's groups)\n", p->cellw ? "cells" : "blocks");
        GLuint ps = stag_px_prog(g, kx);
        if (!ps) return;
        prof(g, -1);
        g->glUseProgram(ps);
        U1I(ps, "f6", (int)(f & 63));
        U1I(ps, "fm", (int)(f % (unsigned)p->G));
        U1I(ps, "n", p->n);
        U1I(ps, "pay", p->pay);
        U1I(ps, "gG", p->G);
        U1I(ps, "gK", p->K);
        U1I(ps, "R", p->R);
        if (pr) U1I(ps, "pcap", p->pcap);
        if (pr && p->htop > 0) U1I(ps, "htop", p->htop);
        int sw = p->stripe > 0 && p->G > 1 ? p->stripe : 0, fm = (int)(f % (unsigned)p->G), src = 0;
        U1I(ps, "stw", sw);
        if (g->sg.pp) {                 /* (the group's live pair; a changed grouping first resolved) */
            if (sw ? g->sg.pm && (g->sg.pmG != p->G || g->sg.pmW != sw) : g->sg.pm != 0 && g->sg.pm != ~0u) {
                stag_px_resolve(g);
                g->glUseProgram(ps);
            }
            g->sg.pmG = p->G;
            g->sg.pmW = sw;
            src = (int)(sw ? g->sg.pm >> fm & 1 : g->sg.pm & 1);
        }
        GLuint dst = g->sg.pp && !src ? g->sg.fbpx2 : g->sg.fbpx;
        bind(g, GS_PH, src ? g->sg.PH2 : g->sg.PH);
        bind(g, GS_PC, src ? g->sg.PC2 : g->sg.PC);
        bind(g, GS_T, g->sg.T);
        bind(g, GS_RK, g->sg.RK);
        if (tm) g->sg.begin_q(0x88BF /* GL_TIME_ELAPSED_EXT */, g->sg.tq[0]);
        if (sw) {                       /* only the group's stripes drawn: the others cost nothing */
            for (int x = 0; x < EPD_W; x += sw)
                if ((epd_stag_srank(x / sw) * p->G >> 12) == fm)
                    stag_px_drawto(g, dst, x, x + sw > EPD_W ? EPD_W : x + sw, 0, EPD_H);
        } else
            stag_px_drawto(g, dst, 0, EPD_W, 0, EPD_H);
        if (g->sg.pp) g->sg.pm = sw ? g->sg.pm ^ 1u << fm : src ? 0 : ~0u;
        bind(g, GS_PH, g->sg.PH);       /* (the pack: both pairs) */
        bind(g, GS_PH2, g->sg.pp ? g->sg.PH2 : g->sg.PH);
        g->glUseProgram(g->sg.pxpk);
        g->glUniform1i(g->sg.xk_f6, (int)(f & 63));
        g->glUniform1ui(g->sg.xk_pm, g->sg.pm);
        g->glUniform1i(g->sg.xk_stw, sw);
        g->glUniform1i(g->sg.xk_gG, p->G);
        g->glBindFramebuffer(GL_FRAMEBUFFER, g->sg.fbwd);
        g->glViewport(0, 0, GW, EPD_H);
        g->glScissor(0, 0, GW, EPD_H);
        g->glDrawArrays(GL_TRIANGLES, 0, 3);
        if (tm) g->sg.end_q(0x88BF);
        g->sg.fid++;
        prof(g, EPD_GPU_PROF_CLAIM);
        goto emit;
    }
    GLuint cs = stag_prog(g, key);
    if (!cs) return;
    prof(g, -1);
    g->glUseProgram(cs);
    U1I(cs, "f6", (int)(f & 63));
    U1I(cs, "gb", (p->G * (p->G - 1) / 2 + (int)(f % (unsigned)p->G)) * 64);
    U1I(cs, "f62", (int)((f & 63) * 0x10001u));
    U1I(cs, "n", p->n);
    U1I(cs, "pay", p->pay);
    U1I(cs, "gG", p->G);
    U1I(cs, "gK", p->K);
    U1I(cs, "R", p->R);
    if (pr) U1I(cs, "pcap", p->pcap);
    if (pr && p->htop > 0) U1I(cs, "htop", p->htop);
    if (tsk) {
        U1I(cs, "fid", (int)g->sg.fid);
        bind(g, GS_AT, g->sg.AT);
        g->glBindImageTexture(4, g->sg.AT, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R32UI);
    }
    {                                   /* the octants of group fm: ranks ceil(4096 fm / G) .. */
        int fm = (int)(f % (unsigned)p->G), lo = (4096 * fm + p->G - 1) / p->G, hi = (4096 * (fm + 1) + p->G - 1) / p->G - 1;
        U1I(cs, "jlo", stw ? 0 : lo >> 9);
        U1I(cs, "jhi", stw ? 7 : hi >> 9);
        U1I(cs, "fm", fm);
    }
    bind(g, GS_H, g->sg.H);
    bind(g, GS_C, g->sg.C);
    bind(g, GS_T, g->sg.T);
    bind(g, GS_OC, g->sg.OC);
    if (cel) {                          /* (cells: their table on GM's unit, rebuilt for a new shape) */
        int ck = p->cellord << 16 | p->cellw << 8 | p->cellh;
        g->glActiveTexture(GL_TEXTURE0 + GS_GM);
        if (!g->sg.CR) g->sg.CR = tex(g, GL_RG32UI, 64, 64);
        else g->glBindTexture(GL_TEXTURE_2D, g->sg.CR);
        if (g->sg.crk != ck) {
            static uint32_t ct2[64 * 64 * 2];
            for (int i = 0; i < 64 * 64; i++) ct2[2 * i] = ctab[i], ct2[2 * i + 1] = ctab[4096 + i];
            g->glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 64, 64, GL_RG_INTEGER, GL_UNSIGNED_INT, ct2);
            g->sg.crk = ck;
        }
        U1I(cs, "cwl", __builtin_ctz((unsigned)p->cellw));
        U1I(cs, "chl", __builtin_ctz((unsigned)p->cellh));
    }
    bind(g, GS_GM, cel ? g->sg.CR : g->sg.GM);
    g->glBindImageTexture(0, g->sg.H, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32UI);
    g->glBindImageTexture(1, g->sg.C, 0, GL_FALSE, 0, GL_WRITE_ONLY, g->sg.heapstate ? GL_RGBA8 : GL_R32UI);
    g->glBindImageTexture(2, g->sg.WD, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R32UI);
    if (g->sg.lanes) g->glBindImageTexture(5, g->sg.A, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA8);
    if (direct) {
        bind(g, 0, g->tpl);
        U1I(cs, "pw", tg->pw);
        g->glBindImageTexture(3, tg->itex, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA8);
    }
    /* GLASSINE_STAG_COUNT=1: a census of the pass's work (words with group pixels
     * to look at, words / pixels on the cold path, words driven), every 85
     * frames to stderr. Atomics: a measurement, not exact */
    static GLuint cnt;
    static long cnf;
    if (stag_count_on()) {
        if (!cnt) {
            g->glGenBuffers(1, &cnt);
            g->glBindBuffer(GL_SHADER_STORAGE_BUFFER, cnt);
            g->glBufferData(GL_SHADER_STORAGE_BUFFER, 24, (uint32_t[6]){ 0 }, GL_DYNAMIC_COPY);
        }
        g->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, cnt);
    }
    static GLuint xlb, xdp;             /* (timing experiments: GLASSINE_STAG_XLIST (defs XLIST), GLASSINE_STAG_XDISP=N) */
    if (getenv("GLASSINE_STAG_XLIST")) {
        if (!xlb) {
            g->glGenBuffers(1, &xlb);
            g->glBindBuffer(GL_SHADER_STORAGE_BUFFER, xlb);
            g->glBufferData(GL_SHADER_STORAGE_BUFFER, (GLsizeiptr)(2048 + 1408 * 2048) * 4, NULL, GL_DYNAMIC_COPY);
        }
        g->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, xlb);
    }
    int xdn = getenv("GLASSINE_STAG_XDISP") ? atoi(getenv("GLASSINE_STAG_XDISP")) : 0;
    if (xdn > 0 && !xdp) {
        GLuint c = shader(g, GL_COMPUTE_SHADER, "#version 310 es\nlayout(local_size_x = 32, local_size_y = 8) in;\nvoid main() {}\n");
        if (c) { xdp = g->glCreateProgram(); g->glAttachShader(xdp, c); g->glLinkProgram(xdp); }
    }
    if (tm) g->sg.begin_q(0x88BF /* GL_TIME_ELAPSED_EXT */, g->sg.tq[0]);
    g->glDispatchCompute(stw ? (GW + 7) / 8 : (GW + 31) / 32, (stw ? (EPD_H + g->sg.swy - 1) / g->sg.swy : (EPD_H + g->sg.wy - 1) / g->sg.wy) + ld, 1);
    for (int i = 0; i < xdn && xdp; i++) {
        g->glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_SHADER_STORAGE_BARRIER_BIT);
        g->glUseProgram(xdp);
        g->glDispatchCompute((GW + 31) / 32, (EPD_H + 7) / 8, 1);
        g->glUseProgram(cs);
    }
    g->sg.fid++;                        /* (the tile skip's frame id: changes noted from here show next frame) */
    static int cnw;
    if (!cnw) { const char *v = getenv("GLASSINE_STAG_COUNT_N"); cnw = v && atoi(v) > 0 ? atoi(v) : 85; }
    if (cnt && ++cnf % cnw == 0) {
        g->glMemoryBarrier(GL_ALL_BARRIER_BITS);
        g->glBindBuffer(GL_SHADER_STORAGE_BUFFER, cnt);
        const uint32_t *m = g->glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, 24, GL_MAP_READ_BIT);
        if (m) {
            double wf = (double)cnw * GW * EPD_H;
            fprintf(stderr, "epd-gpu: stagger census (%d frames): words with group work %.1f %%, on the cold path %.1f %% (%.2f px each), driven %.1f %%\n",
                    cnw, 100 * m[0] / wf, 100 * m[1] / wf, m[1] ? (double)m[2] / m[1] : 0, 100 * m[3] / wf);
            if (m[2])           /* (the steps' cause: continuing a run toward the same target, from rest, a reversal) */
                fprintf(stderr, "epd-gpu: stagger census: steps continuing %.1f %%, from rest %.1f %%, reversing %.1f %%\n",
                        100.0 * m[4] / m[2], 100.0 * m[5] / m[2], 100.0 * ((double)m[2] - m[4] - m[5]) / m[2]);
            g->glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
        }
        g->glBufferData(GL_SHADER_STORAGE_BUFFER, 24, (uint32_t[6]){ 0 }, GL_DYNAMIC_COPY);
    }
    if (tm) g->sg.end_q(0x88BF);
    g->glMemoryBarrier(GL_ALL_BARRIER_BITS);
    prof(g, EPD_GPU_PROF_CLAIM);
emit:;
    int b[4] = { DATA_COL0, DATA_COL0 + GW, DATA_LINE0, DATA_LINE0 + EPD_H }, *pv = tg->prev;
    if (direct) {                       /* (written whole: a slot frame after it redraws the data area) */
        memcpy(pv, b, sizeof b);
        g->glUseProgram(g->prog);
        if (tm && !b4) {
            GLuint64 ns = 0;
            g->sg.get_q(g->sg.tq[0], 0x8866 /* GL_QUERY_RESULT_EXT */, &ns);
            epd_gpu_stag_ms[0] += ns / 1e6;
            epd_gpu_stag_ms[2]++;
        }
        return;
    }
    /* the words into the target: the whole data area (+ what it got last) */
    int u[4] = { b[0] < pv[0] ? b[0] : pv[0], b[1] > pv[1] ? b[1] : pv[1],
                 b[2] < pv[2] ? b[2] : pv[2], b[3] > pv[3] ? b[3] : pv[3] };
    memcpy(pv, b, sizeof b);
    bind(g, GS_WD, g->sg.WD);
    g->glBindFramebuffer(GL_FRAMEBUFFER, tg->fbo);
    g->glViewport(0, 0, tg->w, tg->h);
    g->glUseProgram(g->sg.pemit);
    g->glUniform1i(g->sg.e_pw, tg->pw);
    g->glUniform4i(g->sg.e_box, u[0], u[1], u[2], u[3]);
    if (tg->pw) {
        int r0 = (u[2] * tg->pw + u[0]) / FLAT_W, r1 = ((u[3] - 1) * tg->pw + u[1] + FLAT_W - 1) / FLAT_W;
        g->glScissor(0, r0, FLAT_W, r1 - r0);
    } else
        g->glScissor(u[0], u[2], u[1] - u[0], u[3] - u[2]);
    if (tm) g->sg.begin_q(0x88BF, g->sg.tq[1]);
    g->glDrawArrays(GL_TRIANGLES, 0, 3);
    if (tm) g->sg.end_q(0x88BF);
    tg->drawn = 1;
    g->glUseProgram(g->prog);
    prof(g, EPD_GPU_PROF_FIN);
    if (tm) {
        GLuint64 a = 0, e = 0;
        if (!b4) g->sg.get_q(g->sg.tq[0], 0x8866, &a);
        g->sg.get_q(g->sg.tq[1], 0x8866, &e);
        epd_gpu_stag_ms[0] += a / 1e6;
        epd_gpu_stag_ms[1] += e / 1e6;
        if (!b4) epd_gpu_stag_ms[2]++;
    }
}

/* (8012e9b direct check: 1 hand-back of 17 read 73k pixels as 0
 * = LC's never-stored content: the readback ahead of the store pass, or the
 * store pass ahead of the frames' stores. Both waited for here: a read waits
 * anyway, the finish costs the queue's tail only) */
void epd_gpu_stag_read(epd_gpu *g, epd_stag *s, epd_rect r, int hot)
{
    if (g->sg.px) {                     /* (linear: straight readbacks; R16UI as RGBA_INTEGER, the one read format GLES promises) */
        stag_px_resolve(g);
        g->glFinish();
        g->glBindFramebuffer(GL_FRAMEBUFFER, g->sg.fbpx);
        if (hot) {
            g->glReadBuffer(GL_COLOR_ATTACHMENT0);
            const int rows = EPD_H / 4;         /* (cbuf: EPD_W x EPD_H x 4 bytes = EPD_H / 4 rows of RGBA32UI) */
            for (int y = 0; y < EPD_H; y += rows) {
                int h = y + rows > EPD_H ? EPD_H - y : rows;
                g->glReadPixels(0, y, EPD_W, h, GL_RGBA_INTEGER, GL_UNSIGNED_INT, g->sg.cbuf);
                for (size_t i = 0; i < (size_t)EPD_W * h; i++) s->hot[(size_t)y * EPD_W + i] = (uint16_t)g->sg.cbuf[4 * i];
            }
        }
        if (r.x < 0) { r.w += r.x; r.x = 0; }
        if (r.y < 0) { r.h += r.y; r.y = 0; }
        if (r.x + r.w > EPD_W) r.w = EPD_W - r.x;
        if (r.y + r.h > EPD_H) r.h = EPD_H - r.y;
        if (r.w > 0 && r.h > 0) {
            g->glReadBuffer(GL_COLOR_ATTACHMENT1);
            g->glPixelStorei(GL_PACK_ROW_LENGTH, EPD_W);
            g->glReadPixels(r.x, r.y, r.w, r.h, GL_RED_INTEGER, GL_UNSIGNED_INT, s->cold + (size_t)r.y * EPD_W + r.x);
            g->glPixelStorei(GL_PACK_ROW_LENGTH, 0);
        }
        g->glReadBuffer(GL_COLOR_ATTACHMENT0);
        g->glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return;
    }
    g->glMemoryBarrier(GL_ALL_BARRIER_BITS);
    g->glFinish();
    if (hot) {
        g->glBindFramebuffer(GL_FRAMEBUFFER, g->sg.fbh);
        g->glReadPixels(0, 0, GW, EPD_H, GL_RGBA_INTEGER, GL_UNSIGNED_INT, s->hot);
    }
    if (r.x < 0) { r.w += r.x; r.x = 0; }
    if (r.y < 0) { r.h += r.y; r.y = 0; }
    if (r.x + r.w > EPD_W) r.w = EPD_W - r.x;
    if (r.y + r.h > EPD_H) r.h = EPD_H - r.y;
    if (g->sg.lanes) {                  /* (lanes: the cold state is the engine's D, read through its map) */
        if (r.w > 0 && r.h > 0) {
            const uint32_t *D = g->eng->pk->D;
            size_t off = (size_t)r.y * EPD_PK_S * 4, len = (size_t)r.h * EPD_PK_S * 4;
            epd_dmabuf_sync(g->sg.ldfd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ, off, len);
            for (int y = r.y; y < r.y + r.h; y++)
                memcpy(s->cold + (size_t)y * EPD_W + r.x, D + (size_t)y * EPD_PK_S + r.x, (size_t)r.w * 4);
            epd_dmabuf_sync(g->sg.ldfd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ, off, len);
        }
        if (hot && s->ack) epd_gpu_stag_acks(g, s->ack, 0, EPD_H);
        g->glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return;
    }
    if (r.w > 0 && r.h > 0) {
        int x0 = r.x / 8, x1 = (r.x + r.w + 7) / 8;
        bind(g, GS_C, g->sg.C);
        bind(g, GS_OC, g->sg.OC);
        int ld = stag_lead(g);
        g->glUseProgram(ld ? g->sg.lp[1] : g->sg.pstore);
        g->glBindImageTexture(0, g->sg.LC, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R32UI);
        stag_box(g, ld ? g->sg.ls_box : g->sg.s_box, ld, x0, x1, r.y, r.y + r.h);
        g->glFinish();
        g->glUseProgram(g->prog);
        g->glBindFramebuffer(GL_FRAMEBUFFER, g->sg.fbl);
        g->glPixelStorei(GL_PACK_ROW_LENGTH, EPD_W);
        g->glReadPixels(x0 * 8, r.y, (x1 - x0) * 8, r.h, GL_RED_INTEGER, GL_UNSIGNED_INT,
                        s->cold + (size_t)r.y * EPD_W + (size_t)x0 * 8);
        g->glPixelStorei(GL_PACK_ROW_LENGTH, 0);
    }
    g->glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
