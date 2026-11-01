/* epd-bp4g.h — the two-tier ctl 3 (epd-bp4.h) on the GPU: the visit pass
 * (the cycle's rows y % M == cycle % M, before the frame) and the drive
 * pass. Test code, included by epd-stagger-test.c after epd-bp4.h, under
 * EPD_GPU. The state: one RGBA32UI texture BP_NK x 6 EPD_H, layer L at rows
 * L EPD_H:
 * 0 LW P OW0 OW1 (every word)
 * 1 (every word: the light test)
 * 2 AC0 AC1 AC2 AC3 (a word with work, and the visit)
 * 3 AC4 PD RV H8
 * 4 DC0 DC1 DC2 DC3 (the visit only)
 * 5 DC4 DC5 DC6 DC7
 * Gated against bp4_rows (STAG_BP4_GATE with STAG_BITPLANE shapes). */
#ifndef EPD_BP4G_H
#define EPD_BP4G_H

static const char *bp4_cs =
    "precision highp int; precision highp usampler2D; precision highp uimage2D; precision highp image2D;\n"
    "layout(local_size_x = WX, local_size_y = WY) in;\n"
    "uniform usampler2D SS, TBS, GMT, TP;\n"
    "uniform int g0, gi;\n"
    "uniform int cyc;\n"
    "uniform ivec4 par;\n"              /* (R, pcap, min(htop, pcap) or 0, M) */
    "layout(rgba32ui, binding = 0) writeonly uniform uimage2D S;\n"
    "layout(rgba8, binding = 2) writeonly uniform highp image2D F;\n"
    "#define LH " BP_STR(EPD_H) "\n"
    "uint wire(uint wh, uint bk, uint sh) {\n"
    "  uint v = ((wh >> sh) & 255u) | ((bk >> sh) & 255u) << 16;\n"
    "  v = (v | v << 4) & 0x0F0F0F0Fu; v = (v | v << 2) & 0x33333333u; v = (v | v << 1) & 0x55555555u;\n"
    "  return (v & 0xFFFFu) << 1 | v >> 16;\n"
    "}\n"
    "void main() {\n"
    "  ivec2 c = ivec2(gl_GlobalInvocationID.xy) - ivec2(0, LEADY);\n"
    "#if LEADY > 0\n"                           /* (LEAD: the A509's lost early wave lands in this scratch row) */
    "  if (c.y < 0) { imageStore(S, ivec2(min(c.x, " BP_STR(BP_NK) " - 1), 6 * LH + LEADY + c.y), uvec4(uint(c.x))); return; }\n"
    "#endif\n"
    "  if (c.x >= " BP_STR(BP_NK) " || c.y >= LH) return;\n"
    "  int line = " BP_STR(EPD_DATA0) " / " BP_STR(EPD_LINE) " + LH - 1 - c.y, col = " BP_STR(EPD_DATA0) " % " BP_STR(EPD_LINE) " + 4 * c.x;\n"
    "  bool full = c.x < " BP_STR(BP_NK) " - 1;\n"
    "  uvec4 h = texelFetch(SS, c, 0), q = texelFetch(SS, c + ivec2(0, LH), 0);\n"
    "  uint tb = texelFetch(TBS, c, 0).r;\n"
    "  uint m0 = texelFetch(GMT, ivec2(c.x, g0 + (c.y & 63)), 0).r, mi = texelFetch(GMT, ivec2(c.x, gi + (c.y & 63)), 0).r;\n"
    "  uint t0 = texelFetch(TP, ivec2(col, line), 0).r, t1 = texelFetch(TP, ivec2(col + 1, line), 0).r;\n"
    "  uint t2 = texelFetch(TP, ivec2(col + 2, line), 0).r, t3 = texelFetch(TP, ivec2(col + 3, line), 0).r;\n"
    "  uint lw = h.x, P = h.y, b0 = h.z, b1 = h.w;\n"
    "  uint qnz = q.x | q.y | q.z | q.w, arrive = m0 & (lw ^ tb);\n"
    "  uint drv = mi & P;\n"
    "  if (m0 != 0u && (m0 & ((lw ^ tb) | b0 | b1 | qnz)) == 0u) {\n"     /* (light: nothing to do) */
    "    uvec4 h2 = uvec4(lw, P & ~m0, b0, b1);\n"
    "    if (h2 != h) imageStore(S, c, h2);\n"
    "  } else if (m0 != 0u) {\n"
    "    uvec4 a = texelFetch(SS, c + ivec2(0, 2 * LH), 0), e = texelFetch(SS, c + ivec2(0, 3 * LH), 0);\n"
    "    uint idle = (uint(cyc) + uint((c.x + c.y) % 5)) % 5u == 4u ? 0xFFFFFFFFu : 0u;\n"
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
    "    uint bnz = b0 | b1, act = m0 & (arrive | n1 | bnz);\n"
    "    uint ge1 = n1 | bnz, ge2 = n2 | b1 | (n1 & bnz);\n"
    "    uint nn2 = ge2 & ~idle, nn1 = ge1 & ~nn2;\n"
    "    uint p1 = (~n1 & nn1) | (n1 & ~n2 & nn2), p2 = ~n1 & nn2;\n"
    "    uint r0 = b0 ^ p1, bw = ~b0 & p1, r1 = b1 ^ p2 ^ bw;\n"
    "    uint qn[4], bo = 0u, sb[4] = uint[4](nn1, nn2, 0u, 0u);\n"
    "    for (int i = 0; i < 4; i++) { uint d = n[i] ^ sb[i] ^ bo; bo = (~n[i] & (sb[i] | bo)) | (sb[i] & bo); qn[i] = d; }\n"
    "    uint nnz = nn1 | nn2, ntb = ~tb & nnz;\n"
    "    uint v[5] = uint[5](nn1, nn2 | (~tb & nn1), ntb, ntb, ntb), av[5] = uint[5](a.x, a.y, a.z, a.w, e.x), cy = 0u;\n"
    "    for (int i = 0; i < 5; i++) { uint x = av[i] ^ v[i], s = x ^ cy; cy = (av[i] & v[i]) | (x & cy); av[i] = (s & act) | (av[i] & ~act); }\n"
    "    uint lw2 = (lw & ~act) | (tb & act);\n"
    "    uvec4 h2 = uvec4(lw2, (P & ~m0) | (act & nn2), (r0 & act) | (b0 & ~act), (r1 & act) | (b1 & ~act));\n"
    "    uvec4 q2 = uvec4(((qn[0] & ~bo) & act) | (q.x & ~act), ((qn[1] & ~bo) & act) | (q.y & ~act),\n"
    "                     ((qn[2] & ~bo) & act) | (q.z & ~act), ((qn[3] & ~bo) & act) | (q.w & ~act));\n"
    "    uvec4 a2 = uvec4(av[0], av[1], av[2], av[3]);\n"
    "    uvec4 e2 = uvec4(av[4], e.y | arrive, (e.z & ~arrive) | (arrive & qnz), (e.w & ~arrive) | h8);\n"
    "    drv |= act & ge1;\n"
    "    lw = lw2;\n"
    "    if (h2 != h) imageStore(S, c, h2);\n"
    "    if (q2 != q) imageStore(S, c + ivec2(0, LH), q2);\n"
    "    if (a2 != a) imageStore(S, c + ivec2(0, 2 * LH), a2);\n"
    "    if (e2 != e) imageStore(S, c + ivec2(0, 3 * LH), e2);\n"
    "  }\n"
    "  uint wh = drv & lw, bk = drv & ~lw;\n"
    "  imageStore(F, ivec2(col, line), unpackUnorm4x8(t0 | wire(wh, bk, 24u)));\n"
    "  imageStore(F, ivec2(col + 1, line), unpackUnorm4x8(t1 | wire(wh, bk, 16u)));\n"
    "  if (full) {\n"
    "    imageStore(F, ivec2(col + 2, line), unpackUnorm4x8(t2 | wire(wh, bk, 8u)));\n"
    "    imageStore(F, ivec2(col + 3, line), unpackUnorm4x8(t3 | wire(wh, bk, 0u)));\n"
    "  }\n"
    "}\n";

/* the visit: rows vr, vr + M, ... (bp4_visit_word, word for word) */
static const char *bp4_visit_cs =
    "precision highp int; precision highp usampler2D; precision highp uimage2D;\n"
    "layout(local_size_x = 32, local_size_y = 4) in;\n"
    "uniform usampler2D SS;\n"
    "uniform int vr;\n"
    "uniform ivec4 par;\n"
    "layout(rgba32ui, binding = 0) writeonly uniform uimage2D S;\n"
    "#define LH " BP_STR(EPD_H) "\n"
    "#define N4 4\n"
    "void vadd(in uint a[N4], in uint b[N4], out uint r[N4]) { uint c = 0u; for (int i = 0; i < N4; i++) { uint x = a[i] ^ b[i]; r[i] = x ^ c; c = (a[i] & b[i]) | (x & c); } }\n"
    "void vsub(in uint a[N4], in uint b[N4], out uint r[N4]) { uint c = 0xFFFFFFFFu; for (int i = 0; i < N4; i++) { uint nb = ~b[i], x = a[i] ^ nb; r[i] = x ^ c; c = (a[i] & nb) | (x & c); } }\n"
    "void vsel(uint m, in uint a[N4], in uint b[N4], out uint r[N4]) { for (int i = 0; i < N4; i++) r[i] = (a[i] & m) | (b[i] & ~m); }\n"
    "void vc(out uint r[N4], int c) { for (int i = 0; i < N4; i++) r[i] = ((c >> i) & 1) != 0 ? 0xFFFFFFFFu : 0u; }\n"
    "uint vnz(in uint a[N4]) { return a[0] | a[1] | a[2] | a[3]; }\n"
    "uint vlt(in uint a[N4], in uint b[N4]) { uint d[N4]; vsub(a, b, d); return d[N4 - 1]; }\n"
    "void main() {\n"
    "  ivec2 g = ivec2(gl_GlobalInvocationID.xy) - ivec2(0, LEADY);\n"
    "#if LEADY > 0\n"
    "  if (g.y < 0) { imageStore(S, ivec2(min(g.x, " BP_STR(BP_NK) " - 1), 6 * LH + LEADY + g.y), uvec4(uint(g.x))); return; }\n"
    "#endif\n"
    "  int y = vr + par.w * g.y;\n"
    "  if (g.x >= " BP_STR(BP_NK) " || y >= LH) return;\n"
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

/* the uber program (STAG_BP4_UBER=1, default off: it segfaults in Mesa on the A509): load, pack, visit and drive
 * in ONE program, the pass a uniform, one workgroup shape. The A509 loses an
 * early wave of the next DIFFERENT compute program after a CPU wait under
 * foreign GPU load; the same program across a wait is clean.
 * Built from the four sources above: their declarations dropped (declared once
 * here), each main renamed, so the passes' code stays word for word. */
static const char *bp4_uber_head =
    "precision highp int; precision highp usampler2D; precision highp uimage2D; precision highp image2D;\n"
    "layout(local_size_x = WX, local_size_y = WY) in;\n"
    "uniform usampler2D SS, TBS, GMT, TP, T8, U;\n"
    "uniform int g0, gi, cyc, vr, pass;\n"
    "uniform ivec4 par, box;\n"
    "layout(rgba32ui, binding = 0) writeonly uniform uimage2D S;\n"
    "layout(r32ui, binding = 1) writeonly uniform uimage2D TB;\n"
    "layout(rgba8, binding = 2) writeonly uniform highp image2D F;\n"
    "#define LH " BP_STR(EPD_H) "\n";
/* src's lines without its declarations, its main renamed to fn */
static size_t bp4_uber_part(char *o, const char *src, const char *fn)
{
    size_t n = 0;
    for (const char *l = src; *l; ) {
        const char *e = strchr(l, '\n');
        size_t len = e ? (size_t)(e - l + 1) : strlen(l);
        if (strncmp(l, "precision", 9) && strncmp(l, "layout(", 7) && strncmp(l, "uniform ", 8) && strncmp(l, "#define LH ", 11)) {
            if (!strncmp(l, "void main()", 11)) n += (size_t)sprintf(o + n, "void %s()%.*s", fn, (int)(len - 11), l + 11);
            else { memcpy(o + n, l, len); n += len; }
        }
        l += len;
    }
    o[n] = 0;
    return n;
}
static char *bp4_uber_src(void)
{
    size_t cap = strlen(bp4_uber_head) + strlen(bp4_cs) + strlen(bp4_visit_cs) + strlen(bp_pack_cs) + strlen(bp3_load_cs) + 512;
    char *o = malloc(cap);
    if (!o) return NULL;
    size_t n = (size_t)sprintf(o, "%s", bp4_uber_head);
    n += bp4_uber_part(o + n, bp3_load_cs, "p_load");
    n += bp4_uber_part(o + n, bp_pack_cs, "p_pack");
    n += bp4_uber_part(o + n, bp4_visit_cs, "p_visit");
    n += bp4_uber_part(o + n, bp4_cs, "p_drive");
    sprintf(o + n, "void main() { if (pass == 3) p_drive(); else if (pass == 2) p_visit(); else if (pass == 1) p_pack(); else p_load(); }\n");
    return o;
}

/* the pack with LEAD rows : on a frame with new targets the pack is
 * the first different program after the CPU wait, so the A509's lost early wave
 * hits its TB stores (an undriven pixel in the scroll's new top lines). Built from
 * bp_pack_cs; its scratch rows: TB's rows EPD_H.. (bp_gpu_open gives TB +64). */
static char *bp4_pack_src(void)
{
    static const char *old = "  ivec2 c = ivec2(gl_GlobalInvocationID.xy) + box.xz;\n";
    static const char *lead =
        "  ivec2 g = ivec2(gl_GlobalInvocationID.xy) - ivec2(0, LEADY);\n"
        "#if LEADY > 0\n"
        "  if (g.y < 0) { imageStore(TB, ivec2(min(g.x + box.x, " BP_STR(BP_NK) " - 1), " BP_STR(EPD_H) " + LEADY + g.y), uvec4(uint(g.x))); return; }\n"
        "#endif\n"
        "  ivec2 c = g + box.xz;\n";
    const char *at = strstr(bp_pack_cs, old);
    char *o = at ? malloc(strlen(bp_pack_cs) + strlen(lead) + 1) : NULL;
    if (!o) return NULL;
    sprintf(o, "%.*s%s%s", (int)(at - bp_pack_cs), bp_pack_cs, lead, at + strlen(old));
    return o;
}

typedef struct {
    bp_gpu b;                                   /* (its TB, T8, GMT, TP, F, pack) */
    GLuint pdrv, pvis, pload, S, U, fbS, vq;   /* (vq: the visit's timer query) */
    GLint u_g0, u_gi, u_cyc, u_par, v_vr, v_par;
    GLuint ppack, puber;                       /* (ppack: the pack with LEAD rows) */
    GLint k_box;                              /* (STAG_BP4_UBER: all passes in one program; 0 = the separate ones) */
    GLint x_pass, x_g0, x_gi, x_cyc, x_vr, x_par, x_box;
    int par[4], lead, dlead;                   /* (lead: the pack's and the visit's leading scratch workgroup rows (4 px each), 0 = off;
                                                   dlead: the drive's, at least STAG_BP4_LEADPX pixel rows) */
} bp4_gpu;

static int bp4_gpu_open(bp4_gpu *g, int wx, int wy, const bp_masks *m, const epd_stag_par *p, int M, const uint32_t *tpl)
{
    memset(g, 0, sizeof *g);
    if (bp_gpu_open(&g->b, wx, wy, m, tpl)) return -1;
    char defs[96], vdefs[32];
    g->lead = getenv("STAG_BP4_LEAD") ? atoi(getenv("STAG_BP4_LEAD")) : 1;  /* (STAG_BP4_LEAD=0: the control without it; n: n rows) */
    {   /* (10-30: a 32x1 lead row is 1 px high and the lost wave reached the first 7 real rows; 8x8 (8 px) was clean) */
        int px = getenv("STAG_BP4_LEADPX") ? atoi(getenv("STAG_BP4_LEADPX")) : 8, d = g->lead * wy;
        if (g->lead && d < px) d = px;
        g->dlead = (d + wy - 1) / wy;
    }
    if (g->lead < 0 || g->dlead * wy > 64 || g->lead * 4 > 64) { fprintf(stderr, "bitplane two-tier: STAG_BP4_LEAD %d: over the 64 scratch rows\n", g->lead); return -1; }
    snprintf(defs, sizeof defs, "#define WX %d\n#define WY %d\n#define LEADY %d\n", wx, wy, g->dlead * wy);
    snprintf(vdefs, sizeof vdefs, "#define LEADY %d\n", g->lead * 4);
    if (!(g->pdrv = bp_prog(bp4_cs, defs, "two-tier drive")) || !(g->pvis = bp_prog(bp4_visit_cs, vdefs, "two-tier visit")) ||
        !(g->pload = bp_prog(bp3_load_cs, "", "two-tier load"))) return -1;
    {   char *ps = bp4_pack_src(), pd[32];
        snprintf(pd, sizeof pd, "#define LEADY %d\n", g->lead * 4);
        if (!ps || !(g->ppack = bp_prog(ps, pd, "two-tier pack"))) { free(ps); return -1; }
        free(ps);
        bpgl.glUseProgram(g->ppack);
        bpgl.glUniform1i(bpgl.glGetUniformLocation(g->ppack, "T8"), BPU_T8);
        g->k_box = bpgl.glGetUniformLocation(g->ppack, "box");
    }
    if (getenv("STAG_BP4_UBER") && atoi(getenv("STAG_BP4_UBER"))) {      /* (default off: Mesa 25.1.9 segfaults compiling it on the A509) */
        char *us = bp4_uber_src();
        if (!us || !(g->puber = bp_prog(us, defs, "two-tier uber"))) { free(us); return -1; }
        free(us);
        GLuint q = g->puber;
        bpgl.glUseProgram(q);
        static const struct { const char *n; int u; } su[] = { { "SS", BPU_SS }, { "TBS", BPU_TB }, { "GMT", BPU_GM }, { "TP", BPU_TP }, { "T8", BPU_T8 }, { "U", BPU_U } };
        for (size_t i = 0; i < sizeof su / sizeof su[0]; i++) bpgl.glUniform1i(bpgl.glGetUniformLocation(q, su[i].n), su[i].u);
        g->x_pass = bpgl.glGetUniformLocation(q, "pass"); g->x_g0 = bpgl.glGetUniformLocation(q, "g0"); g->x_gi = bpgl.glGetUniformLocation(q, "gi");
        g->x_cyc = bpgl.glGetUniformLocation(q, "cyc"); g->x_vr = bpgl.glGetUniformLocation(q, "vr");
        g->x_par = bpgl.glGetUniformLocation(q, "par"); g->x_box = bpgl.glGetUniformLocation(q, "box");
    }
    bpgl.glUseProgram(g->pdrv);
    bpgl.glUniform1i(bpgl.glGetUniformLocation(g->pdrv, "SS"), BPU_SS);
    bpgl.glUniform1i(bpgl.glGetUniformLocation(g->pdrv, "TBS"), BPU_TB);
    bpgl.glUniform1i(bpgl.glGetUniformLocation(g->pdrv, "GMT"), BPU_GM);
    bpgl.glUniform1i(bpgl.glGetUniformLocation(g->pdrv, "TP"), BPU_TP);
    g->u_g0 = bpgl.glGetUniformLocation(g->pdrv, "g0");
    g->u_gi = bpgl.glGetUniformLocation(g->pdrv, "gi");
    g->u_cyc = bpgl.glGetUniformLocation(g->pdrv, "cyc");
    g->u_par = bpgl.glGetUniformLocation(g->pdrv, "par");
    bpgl.glUseProgram(g->pvis);
    bpgl.glUniform1i(bpgl.glGetUniformLocation(g->pvis, "SS"), BPU_SS);
    g->v_vr = bpgl.glGetUniformLocation(g->pvis, "vr");
    g->v_par = bpgl.glGetUniformLocation(g->pvis, "par");
    int hc = p->htop > 0 ? (p->htop < p->pcap ? p->htop : p->pcap) : 0;
    if (getenv("STAG_BP4_BREAK")) hc = 0;      /* (the gate's negative control: the GPU's hammer off, the twin's on) */
    g->par[0] = p->R; g->par[1] = p->pcap; g->par[2] = hc; g->par[3] = M;
    if (bpgl.genq) bpgl.genq(1, &g->vq);
    bpgl.glUseProgram(g->pload);
    bpgl.glUniform1i(bpgl.glGetUniformLocation(g->pload, "U"), BPU_U);
    g->S = bp_tex(GL_RGBA32UI, BP_NK, 6 * EPD_H + 64);    /* (+64: LEAD's scratch rows) */
    g->U = bp_tex(GL_RGBA32UI, BP_NK, 6 * EPD_H);
    if (!(g->fbS = bp_fbo(g->S))) { fprintf(stderr, "bitplane two-tier: no readback framebuffer\n"); return -1; }
    GLenum e = bpgl.glGetError();
    if (e != GL_NO_ERROR) { fprintf(stderr, "bitplane two-tier: GL error 0x%x in setup\n", e); return -1; }
    return 0;
}
static void bp4_gpu_close(bp4_gpu *g)
{
    bpgl.glDeleteTextures(1, &g->S); bpgl.glDeleteTextures(1, &g->U);
    bpgl.glDeleteFramebuffers(1, &g->fbS);
    bpgl.glDeleteProgram(g->pdrv); bpgl.glDeleteProgram(g->pvis); bpgl.glDeleteProgram(g->pload);
    if (g->puber) bpgl.glDeleteProgram(g->puber);
    if (g->ppack) bpgl.glDeleteProgram(g->ppack);
    bp_gpu_close(&g->b);
    memset(g, 0, sizeof *g);
}
static void bp4_texels(const bp4_state *s, uint32_t *u)
{
    const size_t L = (size_t)4 * BP_NW;
    for (size_t i = 0; i < BP_NW; i++) {
        uint32_t *t = u + 4 * i;
        t[0] = s->lw[i]; t[1] = s->p[i]; t[2] = s->ow[0][i]; t[3] = s->ow[1][i];
        for (int j = 0; j < 4; j++) t[L + j] = s->op[j][i];
        for (int j = 0; j < 4; j++) t[2 * L + j] = s->ac[j][i];
        t[3 * L + 0] = s->ac[4][i]; t[3 * L + 1] = s->pd[i]; t[3 * L + 2] = s->rv[i]; t[3 * L + 3] = s->h8[i];
        for (int j = 0; j < 4; j++) t[4 * L + j] = s->dc[j][i];
        for (int j = 0; j < 4; j++) t[5 * L + j] = s->dc[4 + j][i];
    }
}
static void bp4_gpu_load(bp4_gpu *g, const bp4_state *s)
{
    uint32_t *u = malloc((size_t)BP_NW * 16 * 6);
    if (!u) return;
    bp4_texels(s, u);
    bp_bind(BPU_U, g->U);
    bpgl.glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, BP_NK, 6 * EPD_H, GL_RGBA_INTEGER, GL_UNSIGNED_INT, u);
    free(u);
    bpgl.glBindImageTexture(0, g->S, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32UI);
    if (g->puber) {
        bpgl.glUseProgram(g->puber);
        bpgl.glUniform1i(g->x_pass, 0);
        bp_bind(BPU_U, g->U);
        bpgl.glDispatchCompute((GLuint)(BP_NK + g->b.wx - 1) / (GLuint)g->b.wx, (GLuint)(6 * EPD_H + g->b.wy - 1) / (GLuint)g->b.wy, 1);
    } else {
        bpgl.glUseProgram(g->pload);
        bpgl.glDispatchCompute((BP_NK + 31) / 32, (6 * EPD_H + 3) / 4, 1);
    }
    bpgl.glMemoryBarrier(GL_ALL_BARRIER_BITS);
}
/* frame f: the pending pack, the visit (M > 0, the cycle's first frame), the
 * drive pass; GPU times: *pms the pack, *vms the visit, *dms the drive (-1: no timer) */
static void bp4_gpu_frame(bp4_gpu *g, uint32_t f, double *pms, double *vms, double *dms)
{
    bp_gpu *b = &g->b;
    int tm = bpgl.genq != NULL, pk = b->npk, M = g->par[3], vis = M > 0 && f % 2 == 0;
    uint32_t c = f / 2;
    *pms = 0; *vms = 0; *dms = -1;
    GLuint ux = g->puber;
    if (pk) {
        bpgl.glUseProgram(ux ? ux : g->ppack);
        bp_bind(BPU_T8, b->T8);
        bpgl.glBindImageTexture(1, b->TB, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R32UI);
        if (ux) bpgl.glUniform1i(g->x_pass, 1);
        bpgl.glUniform4i(ux ? g->x_box : g->k_box, b->pk[0], b->pk[1], b->pk[2], b->pk[3]);
        int wx = ux ? b->wx : 32, wy = ux ? b->wy : 4, pl = ux ? 0 : g->lead;
        if (tm) bpgl.beginq(0x88BF, b->tq[0]);
        bpgl.glDispatchCompute((GLuint)(b->pk[1] - b->pk[0] + wx - 1) / (GLuint)wx, (GLuint)(b->pk[3] - b->pk[2] + wy - 1) / (GLuint)wy + (GLuint)pl, 1);
        if (tm) bpgl.endq(0x88BF);
        bpgl.glMemoryBarrier(GL_ALL_BARRIER_BITS);
        b->npk = 0;
    }
    if (vis) {
        int vr = (int)(c % (uint32_t)M), rows = (EPD_H - vr + M - 1) / M;
        bpgl.glUseProgram(ux ? ux : g->pvis);
        if (ux) bpgl.glUniform1i(g->x_pass, 2);
        bpgl.glUniform1i(ux ? g->x_vr : g->v_vr, vr);
        bpgl.glUniform4i(ux ? g->x_par : g->v_par, g->par[0], g->par[1], g->par[2], g->par[3]);
        bp_bind(BPU_SS, g->S);
        bpgl.glBindImageTexture(0, g->S, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32UI);
        int wx = ux ? b->wx : 32, wy = ux ? b->wy : 4;
        if (tm) bpgl.beginq(0x88BF, g->vq);
        bpgl.glDispatchCompute((GLuint)(BP_NK + wx - 1) / (GLuint)wx, (GLuint)(rows + wy - 1) / (GLuint)wy + (GLuint)(ux ? g->dlead : g->lead), 1);
        if (tm) bpgl.endq(0x88BF);
        bpgl.glMemoryBarrier(GL_ALL_BARRIER_BITS);
        {   static int sy = -1;                 /* (STAG_BP4_SYNC 1: glFinish, 2: glFlush between the visit and the drive; the A509 under load) */
            if (sy < 0) sy = getenv("STAG_BP4_SYNC") ? atoi(getenv("STAG_BP4_SYNC")) : 0;
            if (sy == 1) bpgl.glFinish(); else if (sy == 2) bpgl.glFlush();
        }
    }
    int fm = (int)(f & 1);
    bpgl.glUseProgram(ux ? ux : g->pdrv);
    if (ux) bpgl.glUniform1i(g->x_pass, 3);
    bpgl.glUniform1i(ux ? g->x_g0 : g->u_g0, fm * 64);
    bpgl.glUniform1i(ux ? g->x_gi : g->u_gi, (fm ^ 1) * 64);
    bpgl.glUniform1i(ux ? g->x_cyc : g->u_cyc, (int)c);
    bpgl.glUniform4i(ux ? g->x_par : g->u_par, g->par[0], g->par[1], g->par[2], g->par[3]);
    bp_bind(BPU_SS, g->S);
    bp_bind(BPU_TB, b->TB);
    bp_bind(BPU_GM, b->GMT);
    bp_bind(BPU_TP, b->TP);
    bpgl.glBindImageTexture(0, g->S, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32UI);
    bpgl.glBindImageTexture(2, b->F, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA8);
    if (tm) bpgl.beginq(0x88BF, b->tq[1]);
    bpgl.glDispatchCompute((GLuint)(BP_NK + b->wx - 1) / (GLuint)b->wx, (GLuint)(EPD_H + b->wy - 1) / (GLuint)b->wy + (GLuint)g->dlead, 1);
    if (tm) bpgl.endq(0x88BF);
    bpgl.glMemoryBarrier(GL_ALL_BARRIER_BITS);
    if (tm) {
        uint64_t ns = 0;
        if (pk) { bpgl.getq(b->tq[0], 0x8866, &ns); *pms = ns / 1e6; }
        if (vis) { ns = 0; bpgl.getq(g->vq, 0x8866, &ns); *vms = ns / 1e6; }
        ns = 0;
        bpgl.getq(b->tq[1], 0x8866, &ns);
        *dms = ns / 1e6;
    }
}
/* S against the CPU twin's planes: the differing plane words */
static long bp4_gpu_cmp(bp4_gpu *g, const bp4_state *s)
{
    size_t n = (size_t)BP_NW * 4 * 6;
    uint32_t *u = malloc(n * 4), *v = malloc(n * 4);
    if (!u || !v) { free(u); free(v); return -1; }
    bpgl.glFinish();
    bpgl.glBindFramebuffer(GL_FRAMEBUFFER, g->fbS);
    bpgl.glReadPixels(0, 0, BP_NK, 6 * EPD_H, GL_RGBA_INTEGER, GL_UNSIGNED_INT, u);
    bpgl.glBindFramebuffer(GL_FRAMEBUFFER, 0);
    bp4_texels(s, v);
    long d = 0;
    for (size_t i = 0; i < n; i++) d += u[i] != v[i];
    free(u); free(v);
    return d;
}
#endif
