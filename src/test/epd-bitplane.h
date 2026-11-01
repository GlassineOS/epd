/* epd-bitplane.h — a bit-plane layout of the stagger's A2 drive pass, for
 * timing only (idea:, lesson 2).
 * Test code: included by epd-stagger-test.c alone, nothing in the daemon uses it.
 *
 * Semantics: exactly the CPU reference's (epd-stagger.c px) for ctl 0, G >= 2
 * groups, n = G - 1 drive frames, dither 0 (a target is white from level 16),
 * no payback, no drain, runs (lanes: their steady state, every lane armed, no
 * evict). Then a pixel's step always starts on its group's frame and is done
 * by the next one (age G > n), so its state is three bits:
 * LW level white (the step's direction too: a step drives toward its level)
 * LG level grey (not a rail yet: the first group frame always steps it)
 * A a step started on its last group frame (ph != 0)
 * and one frame is pure bitwise, 32 pixels a uint (M0: the frame's group, MI:
 * the group on its idle frame, pos G - 1; TB: the target white):
 * chg = M0 & (LG | (LW ^ TB)); A' = (A & ~M0) | chg
 * LW' = (LW & ~M0) | (TB & M0); LG' = LG & ~M0
 * drive = A' & ~MI (G 2: A' & M0); white = drive & LW', black = drive & ~LW'
 * The op model, the DC record, stale and k (today's cold state) do not decide
 * anything at ctl 0 and are not kept: a lower bound on a bit-plane design.
 *
 * Layout: plane word k of row y = pixels 32 k .. 32 k + 31, pixel 32 k + j at
 * bit 31 - j (MSB first, as the wire word: pixel i of a wire word at shift
 * 14 - 2 i); BP_NK = 59 words a row, the last one holds 16 pixels (its low
 * half is never in a group: inert). Wire word 4 k + q = byte 3 - q of a plane
 * word, spread to 2 bits a pixel.
 * GPU: the state S is one RGBA32UI texel a plane word (.x LW, .y LG, .z A),
 * TB an R32UI plane made by a pack pass from the targets' bytes (T8, upload
 * only) when they change, the group masks a table (GMT: per group and row mod
 * 64; stripes 64, word Bayer 8 x 1 and the blue noise all repeat every 64
 * rows); the frame goes into the target words F (rgba8, the template ORed in),
 * as today's direct pass. One invocation a plane word; the workgroup shape is
 * the variable (BP shapes "WXxWY": 32x1 a row run, 8x8 a tile).
 * A509 rules kept: texture/image units 0-5 only; every fetch first, then the
 * math, the stores last (no fetch/store loop); textures either upload-only
 * (T8, TP, the staging U, GMT) or store-only (S, TB, F; S starts from a load
 * pass out of U, never from an upload). */
#ifndef EPD_BITPLANE_H
#define EPD_BITPLANE_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "../epd-engine.h"
#include "../epd-stagger.h"
#include "../epd-bluenoise.h"
#define EPD_BP4_TV 1                            /* (bp4_rows times its visits: bp4_tv) */
#include "../epd-bp4.h"                 /* (BP_NK, bp_masks, bp_wire: the planes' layout, shared) */

typedef struct { uint32_t *lw, *lg, *a, *tb; } bp_state;

static inline int bp_alloc(bp_state *s)
{
    s->lw = calloc(BP_NW, 4); s->lg = calloc(BP_NW, 4); s->a = calloc(BP_NW, 4); s->tb = calloc(BP_NW, 4);
    return s->lw && s->lg && s->a && s->tb ? 0 : -1;
}
static inline void bp_free(bp_state *s) { free(s->lw); free(s->lg); free(s->a); free(s->tb); memset(s, 0, sizeof *s); }

/* the layouts the masks take: a pixel's group as the CPU reference makes it
 * (epd_stag_grank), and the same every 64 rows */
static inline int bp_layout_ok(const epd_stag_par *p)
{
    if (p->G < 2 || p->G > EPD_STAG_GMAX || p->n != p->G - 1 || p->ctl || p->dither || p->drain || p->lanes > 1) return 0;
    if (p->stripe <= 0) return 1;                               /* the blue noise */
    if (p->cellw) return p->cellh == 1;                         /* cells 8 x 1 (word Bayer), 16 x 1 ... */
    return !p->blocks;                                          /* column stripes */
}

/* pixel x of row y as the CPU reference's level 0..30 (odd: payrule's budget,
 * not at ctl 0) */
static inline void bp_set_level(bp_state *s, int x, int y, int l, int active)
{
    size_t o = (size_t)y * BP_NK + x / 32;
    uint32_t b = 1u << (31 - x % 32);
    s->lw[o] = l == 30 ? s->lw[o] | b : s->lw[o] & ~b;
    s->lg[o] = l != 0 && l != 30 ? s->lg[o] | b : s->lg[o] & ~b;
    s->a[o] = active ? s->a[o] | b : s->a[o] & ~b;
}
/* the targets of r (levels, EPD_W apart: tgt[y * EPD_W + x]) into TB (dither 0) */
static inline void bp_targets(bp_state *s, const uint8_t *tgt, epd_rect r)
{
    for (int y = r.y; y < r.y + r.h; y++)
        for (int k = r.x / 32; k <= (r.x + r.w - 1) / 32; k++) {
            uint32_t v = 0;
            for (int j = 0; j < 32 && 32 * k + j < EPD_W; j++) v |= (uint32_t)((tgt[(size_t)y * EPD_W + 32 * k + j] & 31) >= 16) << (31 - j);
            s->tb[(size_t)y * BP_NK + k] = v;
        }
}

/* the gate's negative controls (STAG_BP_BREAK): 1 the CPU twin, 2 the shader
 * drive only the frame's own group (both wrong at G > 2 only: a step's
 * middle frames lost) */
static int bp_break;


/* frame f's words of rows [y0, y1) (template | phases), the state stepped:
 * the CPU twin of bp_cs */
static void bp_rows(bp_state *s, const bp_masks *m, uint32_t f, const uint32_t *tpl, uint32_t *words, int y0, int y1)
{
    int G = m->G, fm = (int)(f % (unsigned)G), fi = (fm + 1) % G;
    for (int y = y0; y < y1; y++) {
        const uint32_t *m0 = m->gm + ((size_t)fm * 64 + (y & 63)) * BP_NK, *mi = m->gm + ((size_t)fi * 64 + (y & 63)) * BP_NK;
        size_t o = (size_t)y * BP_NK, wo = (size_t)(EPD_DATA0 / EPD_LINE + EPD_H - 1 - y) * EPD_LINE + EPD_DATA0 % EPD_LINE;
        for (int k = 0; k < BP_NK; k++) {
            uint32_t lw = s->lw[o + k], lg = s->lg[o + k], a = s->a[o + k], tb = s->tb[o + k], M0 = m0[k];
            uint32_t chg = M0 & (lg | (lw ^ tb));
            a = (a & ~M0) | chg;
            lw = (lw & ~M0) | (tb & M0);
            lg &= ~M0;
            uint32_t drv = G == 2 || bp_break == 1 ? a & M0 : a & ~mi[k];   /* (bp_break 1: the gate's negative control, wrong at G > 2) */
            s->lw[o + k] = lw; s->lg[o + k] = lg; s->a[o + k] = a;
            for (int q = 0; q < 4 && 4 * k + q < BP_GW; q++)
                words[wo + 4 * k + q] = tpl[wo + 4 * k + q] | bp_wire(drv & lw, drv & ~lw, q);
        }
    }
}

/* the CPU reference's state (runs: hot, cold) as bit planes: the differing
 * pixels (*first: the first one, y * EPD_W + x) */
static long bp_cmp_ref(const bp_state *s, const epd_stag *ref, size_t *first)
{
    long d = 0;
    *first = (size_t)-1;
    for (int y = 0; y < EPD_H; y++)
        for (int x = 0; x < EPD_W; x++) {
            size_t i = (size_t)y * EPD_W + x, o = (size_t)y * BP_NK + x / 32;
            unsigned b = 31 - x % 32, h = ref->hot[i], lv = EPD_STAG_LV(h);
            int l = EPD_STAG_LEVEL(ref->cold[i]);
            unsigned lw = (s->lw[o] >> b) & 1, lg = (s->lg[o] >> b) & 1, a = (s->a[o] >> b) & 1;
            /* (the class in hot and the level in cold agree; the step's direction is its level) */
            int bad = lw != (lv == 1) || lg != (lv == 2) || a != ((h & 3) != 0) || lw != (l == 30) || lg != (l != 0 && l != 30) ||
                      ((h & 3) && (h & 3) != (lw ? 2u : 1u));
            if (bad) { if (!d) *first = i; d++; }
        }
    return d;
}

#ifdef EPD_GPU
#include <stdio.h>
#include <EGL/egl.h>
#include <GLES3/gl31.h>

#define BP_STR_(x) #x
#define BP_STR(x) BP_STR_(x)

/* GLES through eglGetProcAddress (the test links EGL only) */
#define BP_GL \
    F(PFNGLACTIVETEXTUREPROC, glActiveTexture) F(PFNGLATTACHSHADERPROC, glAttachShader) \
    F(PFNGLBINDFRAMEBUFFERPROC, glBindFramebuffer) F(PFNGLBINDIMAGETEXTUREPROC, glBindImageTexture) \
    F(PFNGLBINDTEXTUREPROC, glBindTexture) F(PFNGLCHECKFRAMEBUFFERSTATUSPROC, glCheckFramebufferStatus) \
    F(PFNGLCOMPILESHADERPROC, glCompileShader) F(PFNGLCREATEPROGRAMPROC, glCreateProgram) \
    F(PFNGLCREATESHADERPROC, glCreateShader) F(PFNGLDELETEFRAMEBUFFERSPROC, glDeleteFramebuffers) \
    F(PFNGLDELETEPROGRAMPROC, glDeleteProgram) F(PFNGLDELETESHADERPROC, glDeleteShader) \
    F(PFNGLDELETETEXTURESPROC, glDeleteTextures) F(PFNGLDISPATCHCOMPUTEPROC, glDispatchCompute) \
    F(PFNGLFINISHPROC, glFinish) F(PFNGLFLUSHPROC, glFlush) F(PFNGLFRAMEBUFFERTEXTURE2DPROC, glFramebufferTexture2D) \
    F(PFNGLGENFRAMEBUFFERSPROC, glGenFramebuffers) F(PFNGLGENTEXTURESPROC, glGenTextures) \
    F(PFNGLGETERRORPROC, glGetError) F(PFNGLGETPROGRAMINFOLOGPROC, glGetProgramInfoLog) \
    F(PFNGLGETPROGRAMIVPROC, glGetProgramiv) F(PFNGLGETSHADERINFOLOGPROC, glGetShaderInfoLog) \
    F(PFNGLGETSHADERIVPROC, glGetShaderiv) F(PFNGLGETUNIFORMLOCATIONPROC, glGetUniformLocation) \
    F(PFNGLLINKPROGRAMPROC, glLinkProgram) F(PFNGLMEMORYBARRIERPROC, glMemoryBarrier) \
    F(PFNGLPIXELSTOREIPROC, glPixelStorei) F(PFNGLREADPIXELSPROC, glReadPixels) \
    F(PFNGLSHADERSOURCEPROC, glShaderSource) F(PFNGLTEXPARAMETERIPROC, glTexParameteri) \
    F(PFNGLTEXSTORAGE2DPROC, glTexStorage2D) F(PFNGLTEXSUBIMAGE2DPROC, glTexSubImage2D) \
    F(PFNGLUNIFORM1IPROC, glUniform1i) F(PFNGLUNIFORM4IPROC, glUniform4i) F(PFNGLUSEPROGRAMPROC, glUseProgram)
static struct {
#define F(t, n) t n;
    BP_GL
#undef F
    void (*genq)(GLsizei, GLuint *);
    void (*beginq)(GLenum, GLuint);
    void (*endq)(GLenum);
    void (*getq)(GLuint, GLenum, uint64_t *);
    int ok;
} bpgl;
static int bp_gl_load(void)
{
    if (bpgl.ok) return 0;
#define F(t, n) if (!(bpgl.n = (t)eglGetProcAddress(#n))) { fprintf(stderr, "bitplane: missing %s\n", #n); return -1; }
    BP_GL
#undef F
    const char *ex = (const char *)((const GLubyte *(*)(GLenum))eglGetProcAddress("glGetString"))(GL_EXTENSIONS);
    if (ex && strstr(ex, "GL_EXT_disjoint_timer_query")) {
        bpgl.genq = (void *)eglGetProcAddress("glGenQueriesEXT");
        bpgl.beginq = (void *)eglGetProcAddress("glBeginQueryEXT");
        bpgl.endq = (void *)eglGetProcAddress("glEndQueryEXT");
        bpgl.getq = (void *)eglGetProcAddress("glGetQueryObjectui64vEXT");
        if (!bpgl.genq || !bpgl.beginq || !bpgl.endq || !bpgl.getq) bpgl.genq = NULL;
    }
    bpgl.ok = 1;
    return 0;
}

/* the drive pass: one invocation a plane word. Every fetch first (state,
 * target plane, masks, the 4 template words), then the bitwise step, the
 * stores last */
static const char *bp_cs =
    "precision highp int; precision highp usampler2D; precision highp uimage2D; precision highp image2D;\n"
    "layout(local_size_x = WX, local_size_y = WY) in;\n"
    "uniform usampler2D SS, TBS, GMT, TP;\n"
    "uniform int g0, gi;\n"            /* (GMT rows: the frame's group, the idle group's; 64 each) */
    "layout(rgba32ui, binding = 0) writeonly uniform uimage2D S;\n"
    "layout(rgba8, binding = 2) writeonly uniform highp image2D F;\n"
    "uint wire(uint wh, uint bk, uint sh) {\n"
    "  uint v = ((wh >> sh) & 255u) | ((bk >> sh) & 255u) << 16;\n"
    "  v = (v | v << 4) & 0x0F0F0F0Fu; v = (v | v << 2) & 0x33333333u; v = (v | v << 1) & 0x55555555u;\n"
    "  return (v & 0xFFFFu) << 1 | v >> 16;\n"
    "}\n"
    "void main() {\n"
    "  ivec2 c = ivec2(gl_GlobalInvocationID.xy);\n"
    "  if (c.x >= " BP_STR(BP_NK) " || c.y >= " BP_STR(EPD_H) ") return;\n"
    "  int line = " BP_STR(EPD_DATA0) " / " BP_STR(EPD_LINE) " + " BP_STR(EPD_H) " - 1 - c.y, col = " BP_STR(EPD_DATA0) " % " BP_STR(EPD_LINE) " + 4 * c.x;\n"
    "  bool full = c.x < " BP_STR(BP_NK) " - 1;\n"   /* (the last plane word: 2 wire words) */
    "  uvec4 s = texelFetch(SS, c, 0);\n"
    "  uint tb = texelFetch(TBS, c, 0).r;\n"
    "  uint m0 = texelFetch(GMT, ivec2(c.x, g0 + (c.y & 63)), 0).r;\n"
    "#if BPG > 2\n"
    "  uint mi = texelFetch(GMT, ivec2(c.x, gi + (c.y & 63)), 0).r;\n"
    "#endif\n"
    "  uint t0 = texelFetch(TP, ivec2(col, line), 0).r, t1 = texelFetch(TP, ivec2(col + 1, line), 0).r;\n"
    "  uint t2 = texelFetch(TP, ivec2(col + 2, line), 0).r, t3 = texelFetch(TP, ivec2(col + 3, line), 0).r;\n"
    "  uint chg = m0 & (s.y | (s.x ^ tb));\n"
    "  uint a = (s.z & ~m0) | chg, lw = (s.x & ~m0) | (tb & m0), lg = s.y & ~m0;\n"
    "#if BPG > 2\n"
    "  uint drv = a & ~mi;\n"
    "#else\n"
    "  uint drv = a & m0;\n"
    "#endif\n"
    "  uint wh = drv & lw, bk = drv & ~lw;\n"
    "  uint w0 = t0 | wire(wh, bk, 24u), w1 = t1 | wire(wh, bk, 16u), w2 = t2 | wire(wh, bk, 8u), w3 = t3 | wire(wh, bk, 0u);\n"
    "  uvec4 s2 = uvec4(lw, lg, a, 0u);\n"
    "  if (s2 != s) imageStore(S, c, s2);\n"
    "  imageStore(F, ivec2(col, line), unpackUnorm4x8(w0));\n"
    "  imageStore(F, ivec2(col + 1, line), unpackUnorm4x8(w1));\n"
    "  if (full) {\n"
    "    imageStore(F, ivec2(col + 2, line), unpackUnorm4x8(w2));\n"
    "    imageStore(F, ivec2(col + 3, line), unpackUnorm4x8(w3));\n"
    "  }\n"
    "}\n";

/* the pack: the targets' bytes (T8: RGBA32UI, 16 pixels a texel, EPD_W / 16 + 1
 * texels a row) of a box of plane words into TB (dither 0: bit 4 of the level) */
static const char *bp_pack_cs =
    "precision highp int; precision highp usampler2D; precision highp uimage2D;\n"
    "layout(local_size_x = 32, local_size_y = 4) in;\n"
    "uniform usampler2D T8;\n"
    "uniform ivec4 box;\n"            /* (words k0, k1, rows y0, y1) */
    "layout(r32ui, binding = 1) writeonly uniform uimage2D TB;\n"
    "uint nib(uint u) { u = (u >> 4) & 0x01010101u; return (u | u >> 7 | u >> 14 | u >> 21) & 15u; }\n"   /* (byte b's bit 4 -> bit b) */
    "void main() {\n"
    "  ivec2 c = ivec2(gl_GlobalInvocationID.xy) + box.xz;\n"
    "  if (c.x >= box.y || c.y >= box.w) return;\n"
    "  uvec4 a = texelFetch(T8, ivec2(2 * c.x, c.y), 0), b = texelFetch(T8, ivec2(2 * c.x + 1, c.y), 0);\n"
    "  uint l = nib(a.x) | nib(a.y) << 4 | nib(a.z) << 8 | nib(a.w) << 12 | nib(b.x) << 16 | nib(b.y) << 20 | nib(b.z) << 24 | nib(b.w) << 28;\n"
    "  if (c.x == " BP_STR(BP_NK) " - 1) l &= 0xFFFFu;\n"   /* (past the panel) */
    "  imageStore(TB, c, uvec4(bitfieldReverse(l)));\n"
    "}\n";

/* the load: S from the staging texture U (upload-only) */
static const char *bp_load_cs =
    "precision highp int; precision highp usampler2D; precision highp uimage2D;\n"
    "layout(local_size_x = 32, local_size_y = 4) in;\n"
    "uniform usampler2D U;\n"
    "layout(rgba32ui, binding = 0) writeonly uniform uimage2D S;\n"
    "void main() {\n"
    "  ivec2 c = ivec2(gl_GlobalInvocationID.xy);\n"
    "  if (c.x >= " BP_STR(BP_NK) " || c.y >= " BP_STR(EPD_H) ") return;\n"
    "  imageStore(S, c, texelFetch(U, c, 0));\n"
    "}\n";

enum { BPU_SS = 0, BPU_TB = 1, BPU_GM = 2, BPU_TP = 3, BPU_T8 = 4, BPU_U = 5 };   /* texture units (0-15 only on the A509) */
#define BP_T8W (EPD_W / 16 + 1)                 /* (a pad texel: the last plane word's second half) */

typedef struct {
    int wx, wy, G;
    GLuint pdrv, ppack, pload, S, TB, GMT, TP, T8, U, F, fbS, fbF, tq[2];
    GLint u_g0, u_gi, u_box;
    int pk[4], npk;                             /* the pack box pending (plane words, rows) */
    double ms_drive, ms_pack, ms_max;           /* (timer sums; ms_max: drive + pack, the frame's) */
    int nf, nup;
} bp_gpu;

static GLuint bp_prog(const char *body, const char *defs, const char *what)
{
    char *src = malloc(strlen(body) + strlen(defs) + 64);
    if (!src) return 0;
    sprintf(src, "#version 310 es\n%s%s", defs, body);
    GLuint c = bpgl.glCreateShader(GL_COMPUTE_SHADER);
    const char *ss = src;
    bpgl.glShaderSource(c, 1, &ss, NULL);
    bpgl.glCompileShader(c);
    free(src);
    GLint ok = 0;
    bpgl.glGetShaderiv(c, GL_COMPILE_STATUS, &ok);
    if (!ok) { char lg[2048]; bpgl.glGetShaderInfoLog(c, sizeof lg, NULL, lg); fprintf(stderr, "bitplane: %s: compile failed: %s\n", what, lg); return 0; }
    GLuint p = bpgl.glCreateProgram();
    bpgl.glAttachShader(p, c);
    bpgl.glLinkProgram(p);
    bpgl.glDeleteShader(c);
    bpgl.glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) { char lg[2048]; bpgl.glGetProgramInfoLog(p, sizeof lg, NULL, lg); fprintf(stderr, "bitplane: %s: link failed: %s\n", what, lg); return 0; }
    return p;
}
static GLuint bp_tex(GLenum fmt, int w, int h)
{
    GLuint t;
    bpgl.glGenTextures(1, &t);
    bpgl.glBindTexture(GL_TEXTURE_2D, t);
    bpgl.glTexStorage2D(GL_TEXTURE_2D, 1, fmt, w, h);
    bpgl.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    bpgl.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    return t;
}
static void bp_bind(GLuint unit, GLuint t) { bpgl.glActiveTexture(GL_TEXTURE0 + unit); bpgl.glBindTexture(GL_TEXTURE_2D, t); }
static GLuint bp_fbo(GLuint t)
{
    GLuint fb;
    bpgl.glGenFramebuffers(1, &fb);
    bpgl.glBindFramebuffer(GL_FRAMEBUFFER, fb);
    bpgl.glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, t, 0);
    GLenum st = bpgl.glCheckFramebufferStatus(GL_FRAMEBUFFER);
    bpgl.glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return st == GL_FRAMEBUFFER_COMPLETE ? fb : 0;
}

/* "WXxWY" (WX * WY <= 1024) */
static int bp_shape_parse(const char *s, int *wx, int *wy)
{
    char *e;
    long a = strtol(s, &e, 10);
    if (e == s || *e != 'x') return -1;
    long b = strtol(e + 1, &e, 10);
    if (a < 1 || b < 1 || a * b > 1024 || a > 1024 || b > 1024) return -1;
    *wx = (int)a, *wy = (int)b;
    return 0;
}

/* the programs and textures of shape wx x wy for the masks m (G), the
 * template tpl (EPD_WORDS); 0 / -1 */
static int bp_gpu_open(bp_gpu *b, int wx, int wy, const bp_masks *m, const uint32_t *tpl)
{
    memset(b, 0, sizeof *b);
    if (bp_gl_load()) return -1;
    b->wx = wx, b->wy = wy, b->G = m->G;
    char defs[128];
    snprintf(defs, sizeof defs, "#define WX %d\n#define WY %d\n#define BPG %d\n", wx, wy, bp_break == 2 ? 2 : m->G);
    if (!(b->pdrv = bp_prog(bp_cs, defs, "drive")) || !(b->ppack = bp_prog(bp_pack_cs, "", "pack")) || !(b->pload = bp_prog(bp_load_cs, "", "load"))) return -1;
    bpgl.glUseProgram(b->pdrv);
    bpgl.glUniform1i(bpgl.glGetUniformLocation(b->pdrv, "SS"), BPU_SS);
    bpgl.glUniform1i(bpgl.glGetUniformLocation(b->pdrv, "TBS"), BPU_TB);
    bpgl.glUniform1i(bpgl.glGetUniformLocation(b->pdrv, "GMT"), BPU_GM);
    bpgl.glUniform1i(bpgl.glGetUniformLocation(b->pdrv, "TP"), BPU_TP);
    b->u_g0 = bpgl.glGetUniformLocation(b->pdrv, "g0");
    b->u_gi = bpgl.glGetUniformLocation(b->pdrv, "gi");
    bpgl.glUseProgram(b->ppack);
    bpgl.glUniform1i(bpgl.glGetUniformLocation(b->ppack, "T8"), BPU_T8);
    b->u_box = bpgl.glGetUniformLocation(b->ppack, "box");
    bpgl.glUseProgram(b->pload);
    bpgl.glUniform1i(bpgl.glGetUniformLocation(b->pload, "U"), BPU_U);
    b->S = bp_tex(GL_RGBA32UI, BP_NK, EPD_H);
    b->TB = bp_tex(GL_R32UI, BP_NK, EPD_H + 64);   /* (+64: scratch rows for bp4's pack LEAD) */
    b->U = bp_tex(GL_RGBA32UI, BP_NK, EPD_H);
    b->T8 = bp_tex(GL_RGBA32UI, BP_T8W, EPD_H);
    b->F = bp_tex(GL_RGBA8, EPD_LINE, EPD_LINES);
    b->GMT = bp_tex(GL_R32UI, BP_NK, 64 * m->G);
    bpgl.glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, BP_NK, 64 * m->G, GL_RED_INTEGER, GL_UNSIGNED_INT, m->gm);
    b->TP = bp_tex(GL_R32UI, EPD_LINE, EPD_LINES);
    bpgl.glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, EPD_LINE, EPD_LINES, GL_RED_INTEGER, GL_UNSIGNED_INT, tpl);
    {                                           /* (T8's pad column: zeros, once) */
        static uint32_t z[4 * EPD_H];
        bpgl.glBindTexture(GL_TEXTURE_2D, b->T8);
        bpgl.glTexSubImage2D(GL_TEXTURE_2D, 0, BP_T8W - 1, 0, 1, EPD_H, GL_RGBA_INTEGER, GL_UNSIGNED_INT, z);
    }
    if (!(b->fbS = bp_fbo(b->S)) || !(b->fbF = bp_fbo(b->F))) { fprintf(stderr, "bitplane: no readback framebuffer\n"); return -1; }
    if (bpgl.genq) bpgl.genq(2, b->tq);
    GLenum e = bpgl.glGetError();
    if (e != GL_NO_ERROR) { fprintf(stderr, "bitplane: GL error 0x%x in setup\n", e); return -1; }
    return 0;
}
static void bp_gpu_close(bp_gpu *b)
{
    GLuint t[] = { b->S, b->TB, b->GMT, b->TP, b->T8, b->U, b->F }, fb[] = { b->fbS, b->fbF };
    bpgl.glDeleteTextures(7, t);
    bpgl.glDeleteFramebuffers(2, fb);
    bpgl.glDeleteProgram(b->pdrv); bpgl.glDeleteProgram(b->ppack); bpgl.glDeleteProgram(b->pload);
    memset(b, 0, sizeof *b);
}
static void bp_note(bp_gpu *b, int k0, int k1, int y0, int y1)
{
    if (!b->npk) { b->pk[0] = k0, b->pk[1] = k1, b->pk[2] = y0, b->pk[3] = y1; b->npk = 1; return; }
    if (k0 < b->pk[0]) b->pk[0] = k0;
    if (k1 > b->pk[1]) b->pk[1] = k1;
    if (y0 < b->pk[2]) b->pk[2] = y0;
    if (y1 > b->pk[3]) b->pk[3] = y1;
}
/* the targets of r (tgt: EPD_W apart) into T8 (an upload), the pack noted
 * for the next frame (its plane words) */
static void bp_gpu_target(bp_gpu *b, const uint8_t *tgt, epd_rect r)
{
    int x0 = r.x / 16, x1 = (r.x + r.w + 15) / 16;
    bp_bind(BPU_T8, b->T8);
    bpgl.glPixelStorei(GL_UNPACK_ROW_LENGTH, EPD_W / 16);
    bpgl.glTexSubImage2D(GL_TEXTURE_2D, 0, x0, r.y, x1 - x0, r.h, GL_RGBA_INTEGER, GL_UNSIGNED_INT, tgt + (size_t)r.y * EPD_W + (size_t)x0 * 16);
    bpgl.glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    bp_note(b, r.x / 32, (r.x + r.w + 31) / 32, r.y, r.y + r.h);
}
/* S := s (the load pass from U); TB from T8 at the next frame (the caller
 * uploads the targets: bp_gpu_target over the panel) */
static void bp_gpu_load(bp_gpu *b, const bp_state *s)
{
    uint32_t *u = malloc((size_t)BP_NW * 16);
    if (!u) return;
    for (size_t i = 0; i < BP_NW; i++) u[4 * i] = s->lw[i], u[4 * i + 1] = s->lg[i], u[4 * i + 2] = s->a[i], u[4 * i + 3] = 0;
    bp_bind(BPU_U, b->U);
    bpgl.glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, BP_NK, EPD_H, GL_RGBA_INTEGER, GL_UNSIGNED_INT, u);
    free(u);
    bpgl.glUseProgram(b->pload);
    bpgl.glBindImageTexture(0, b->S, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32UI);
    bpgl.glDispatchCompute((BP_NK + 31) / 32, (EPD_H + 3) / 4, 1);
    bpgl.glMemoryBarrier(GL_ALL_BARRIER_BITS);
}
/* frame f: the pending pack, then the drive pass (each timed with the
 * timer: *pms, *dms; -1: no timer); the words in F */
static void bp_gpu_frame(bp_gpu *b, uint32_t f, double *pms, double *dms)
{
    int tm = bpgl.genq != NULL, pk = b->npk;
    *pms = 0; *dms = -1;
    if (pk) {
        bpgl.glUseProgram(b->ppack);
        bp_bind(BPU_T8, b->T8);
        bpgl.glBindImageTexture(1, b->TB, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R32UI);
        bpgl.glUniform4i(b->u_box, b->pk[0], b->pk[1], b->pk[2], b->pk[3]);
        if (tm) bpgl.beginq(0x88BF /* GL_TIME_ELAPSED_EXT */, b->tq[0]);
        bpgl.glDispatchCompute((GLuint)(b->pk[1] - b->pk[0] + 31) / 32, (GLuint)(b->pk[3] - b->pk[2] + 3) / 4, 1);
        if (tm) bpgl.endq(0x88BF);
        bpgl.glMemoryBarrier(GL_ALL_BARRIER_BITS);
        b->npk = 0;
    }
    int fm = (int)(f % (unsigned)b->G);
    bpgl.glUseProgram(b->pdrv);
    bpgl.glUniform1i(b->u_g0, fm * 64);
    bpgl.glUniform1i(b->u_gi, (fm + 1) % b->G * 64);
    bp_bind(BPU_SS, b->S);
    bp_bind(BPU_TB, b->TB);
    bp_bind(BPU_GM, b->GMT);
    bp_bind(BPU_TP, b->TP);
    bpgl.glBindImageTexture(0, b->S, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32UI);
    bpgl.glBindImageTexture(2, b->F, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA8);
    if (tm) bpgl.beginq(0x88BF, b->tq[1]);
    bpgl.glDispatchCompute((GLuint)(BP_NK + b->wx - 1) / (GLuint)b->wx, (GLuint)(EPD_H + b->wy - 1) / (GLuint)b->wy, 1);
    if (tm) bpgl.endq(0x88BF);
    bpgl.glMemoryBarrier(GL_ALL_BARRIER_BITS);
    if (tm) {
        uint64_t ns = 0;
        if (pk) { bpgl.getq(b->tq[0], 0x8866 /* GL_QUERY_RESULT_EXT */, &ns); *pms = ns / 1e6; }
        ns = 0;
        bpgl.getq(b->tq[1], 0x8866, &ns);
        *dms = ns / 1e6;
    }
}
/* the frame's words (F) into words' data area only (the rest untouched) */
static void bp_gpu_words(bp_gpu *b, uint32_t *words)
{
    static uint32_t fw[EPD_WORDS];
    bpgl.glFinish();
    bpgl.glBindFramebuffer(GL_FRAMEBUFFER, b->fbF);
    bpgl.glReadPixels(0, 0, EPD_LINE, EPD_LINES, GL_RGBA, GL_UNSIGNED_BYTE, fw);
    bpgl.glBindFramebuffer(GL_FRAMEBUFFER, 0);
    for (int y = 0; y < EPD_H; y++) {
        size_t wo = (size_t)(EPD_DATA0 / EPD_LINE + EPD_H - 1 - y) * EPD_LINE + EPD_DATA0 % EPD_LINE;
        memcpy(words + wo, fw + wo, BP_GW * 4);
    }
}
/* S into s's planes (tb untouched) */
static void bp_gpu_state(bp_gpu *b, bp_state *s)
{
    uint32_t *u = malloc((size_t)BP_NW * 16);
    if (!u) return;
    bpgl.glFinish();
    bpgl.glBindFramebuffer(GL_FRAMEBUFFER, b->fbS);
    bpgl.glReadPixels(0, 0, BP_NK, EPD_H, GL_RGBA_INTEGER, GL_UNSIGNED_INT, u);
    bpgl.glBindFramebuffer(GL_FRAMEBUFFER, 0);
    for (size_t i = 0; i < BP_NW; i++) s->lw[i] = u[4 * i], s->lg[i] = u[4 * i + 1], s->a[i] = u[4 * i + 2];
    free(u);
}
#endif
#endif
