/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* test/epd-bp4.h — the gate of stagger_ctl 4's twin (../epd-bp4.h, the two-tier
 * ctl 3 on bit planes) against the tier model (epd-tier.h, variant
 * m<M>is) and, with STAG_BITPLANE shapes, of the GPU pass (epd-bp4g.h) against
 * the twin. Test code, included by epd-stagger-test.c after epd-tier.h. */
#ifndef EPD_BP4_TEST_H
#define EPD_BP4_TEST_H

/* the twin itself: ../epd-bp4.h (included by epd-bitplane.h, EPD_BP4_TV set) */

#ifdef EPD_GPU
#include "epd-bp4g.h"                    /* (the GPU pass, gated below with STAG_BITPLANE shapes) */
#endif

/* the gate: the twin == the tier model (m<M>is) on the gate's frames, every
 * frame's drive per pixel and the state every 8th frame; then the time a
 * frame against bp3_rows (today's ctl 3 on planes), one thread, this host */
static int bp4_gate(void)
{
    const char *ls = getenv("STAG_BP_LAYOUTS"), *rs = getenv("STAG_BP3_RS"), *ms = getenv("STAG_TIER_M");
    const int ph = 1404 * 4, M = ms ? atoi(ms) : 4;
    uint8_t *pg = page(ph), *tg = malloc(EPD_NPX);
    bp4_state b4;
    bp3_state b3;
    bp_masks m;
    tier_v v;
    char vn[16];
    static uint32_t W4[EPD_WORDS], W3[EPD_WORDS];
    long nbad = 0, nfr = 0;
    if (!tg || bp4_alloc(&b4) || bp3_alloc(&b3)) return 2;
#ifdef EPD_BP4G_H
    int sw[BP_SHMAX], sh[BP_SHMAX], ns = 0;     /* (STAG_BITPLANE: the GPU pass's shapes, gated and timed) */
    {   const char *shs = getenv("STAG_BITPLANE");
        char buf[256];
        snprintf(buf, sizeof buf, "%s", shs && *shs ? shs : "none");
        for (char *t = strtok(buf, " ,"); t && ns < BP_SHMAX; t = strtok(NULL, " ,")) {
            if (!strcmp(t, "none")) continue;
            if (bp_shape_parse(t, &sw[ns], &sh[ns])) { printf("BP4: bad shape %s\n", t); return 2; }
            ns++;
        }
        if (ns && !epd_gpu_open(&E)) { printf("BP4 gate: no GPU (EGL): GPU shapes skipped\n"); ns = 0; }
    }
    static uint32_t W5[EPD_WORDS];
#endif
    char lb[128], rb[64], *lsv, *rsv;
    snprintf(lb, sizeof lb, "%s", ls && *ls ? ls : "64 wbayer 0");
    snprintf(rb, sizeof rb, "%s", rs && *rs ? rs : "9 14");
    snprintf(vn, sizeof vn, "m%dis", M);
    for (char *lt = strtok_r(lb, " ,", &lsv); lt; lt = strtok_r(NULL, " ,", &lsv)) {
        char rb2[64];
        snprintf(rb2, sizeof rb2, "%s", rb);
        for (char *rt = strtok_r(rb2, " ,", &rsv); rt; rt = strtok_r(NULL, " ,", &rsv))
        for (int wide = 0; wide < 2; wide++) {
            int R = atoi(rt);
            epd_stag_par p = { .G = 2, .n = 1, .ctl = 3, .K = 8, .payrule = 1, .pcap = 3, .htop = 2, .dither = 0, .R = R };
            if (bp_layout(&p, lt) || !bp3_par_ok(&p) || bp_masks_make(&m, &p)) { printf("BP4: layout %s R %d: not taken\n", lt, R); return 2; }
            if (tier_parse(&v, vn, R) || tier_alloc(&v)) return 2;
            memcpy(tg, pg, EPD_NPX);
            for (int y = 0; y < EPD_H; y++)
                for (int x = 0; x < EPD_W; x++) {
                    size_t i = (size_t)y * EPD_W + x;
                    int l = tg[i] >= 16, op = l ? R : 0, dc = wide ? (int)((i * 37) % 201) - 100 : (int)(i % 7) - 3;
                    if (dc > 127) dc = 127;
                    if (dc < -127) dc = -127;
                    v.lvl[i] = (uint8_t)l; v.opv[i] = (uint8_t)op; v.dc[i] = (int16_t)dc;
                    size_t o = (size_t)y * BP_NK + x / 32;
                    uint32_t bit = 1u << (31 - x % 32);
#define BP4_SET(pl, val) ((pl)[o] = (val) ? (pl)[o] | bit : (pl)[o] & ~bit)
                    BP4_SET(b4.lw, l); BP4_SET(b4.p, 0); BP4_SET(b4.pd, 0); BP4_SET(b4.rv, 0); BP4_SET(b4.h8, 0);
                    for (int j = 0; j < 4; j++) BP4_SET(b4.op[j], 0);     /* (q: at its rail) */
                    for (int j = 0; j < 2; j++) BP4_SET(b4.ow[j], 0);
                    for (int j = 0; j < 5; j++) BP4_SET(b4.ac[j], 0);
                    for (int j = 0; j < 8; j++) BP4_SET(b4.dc[j], (dc >> j) & 1);
#undef BP4_SET
                    bp3_put(&b3, x, y, l ? 30 : 0, 0, op, 0, 0, dc, 0);
                }
            {   bp_state t = { .tb = b4.tb }; bp_targets(&t, tg, ALL); memcpy(b3.tb, b4.tb, BP_NW * 4); }
#ifdef EPD_BP4G_H
            bp4_gpu gp[BP_SHMAX];
            double gd[BP_SHMAX][3], gv[BP_SHMAX][3], gpk[BP_SHMAX], gmax[BP_SHMAX];
            long gbad[BP_SHMAX];
            int gnp[BP_SHMAX], gtm[BP_SHMAX];
            for (int k = 0; k < ns; k++) {
                if (bp4_gpu_open(&gp[k], sw[k], sh[k], &m, &p, M, E.tpl)) { printf("BP4 gate: shape %dx%d: the GPU setup failed\n", sw[k], sh[k]); return 2; }
                bp4_gpu_load(&gp[k], &b4);
                bp_gpu_target(&gp[k].b, tg, ALL);
                memset(gd[k], 0, sizeof gd[k]); memset(gv[k], 0, sizeof gv[k]); gpk[k] = 0; gmax[k] = 0; gbad[k] = 0; gnp[k] = 0; gtm[k] = 1;
            }
#endif
            int y0 = 0;
            long lbad = 0;
            double t4[3] = { 0, 0, 0 }, t3[3] = { 0, 0, 0 }, tw[3] = { 0, 0, 0 }, tv[3] = { 0, 0, 0 };
            int nf[3] = { 0, 0, 0 };
            uint32_t fc = 61;
            for (int f = 0; f < 160 && lbad < 5; f++, fc++) {
                epd_rect r = { 0, 0, 0, 0 };
                int stretch = f < 64 ? 0 : f < 96 ? 1 : 2;
                if (stretch == 0) { if (f % 2 == 0) { y0 = (y0 + 17) % (ph - EPD_H); r = ALL; } }
                else if (stretch == 2 && f % 2 == 0) {
                    y0 = (y0 + 17) % (ph - EPD_H);
                    r = (f / 2) % 2 ? (epd_rect){ 300, 400, 500, 380 } : (epd_rect){ 37, 101, 1500, 1203 };
                }
                if (r.w) {
                    for (int y = r.y; y < r.y + r.h; y++)
                        memcpy(tg + (size_t)y * EPD_W + r.x, pg + (size_t)(y0 + y) * EPD_W + r.x, (size_t)r.w);
                    bp_state t = { .tb = b4.tb };
                    bp_targets(&t, tg, r);
                    memcpy(b3.tb, b4.tb, BP_NW * 4);
#ifdef EPD_BP4G_H
                    for (int k = 0; k < ns; k++) bp_gpu_target(&gp[k].b, tg, r);
#endif
                }
                static int8_t rd[EPD_NPX];
                tier_job j = { &v, &p, &m, tg, rd, fc };
                tier_run(&j);                   /* (the model; its agree counter is unused here) */
                memcpy(W4, E.tpl, sizeof W4);
                double a = now();
                bp4_tv = &tv[stretch];
                bp4_rows(&b4, &m, &p, M, fc, E.tpl, W4, 0, EPD_H);
                bp4_tv = NULL;
                t4[stretch] += now() - a;
                memcpy(W3, E.tpl, sizeof W3);
                a = now();
                bp3_rows(&b3, &m, &p, fc, E.tpl, W3, 0, EPD_H, NULL);
                t3[stretch] += now() - a;
                memcpy(W3, E.tpl, sizeof W3);
                a = now();                      /* (the wire words alone, the part both twins share) */
                for (int y = 0; y < EPD_H; y++) {
                    size_t row = (size_t)y * BP_NK, wo = (size_t)(EPD_DATA0 / EPD_LINE + EPD_H - 1 - y) * EPD_LINE + EPD_DATA0 % EPD_LINE;
                    for (int k = 0; k < BP_NK; k++) {
                        uint32_t dv = b4.p[row + k], lw = b4.lw[row + k];
                        for (int q = 0; q < 4 && 4 * k + q < BP_GW; q++)
                            W3[wo + 4 * k + q] = E.tpl[wo + 4 * k + q] | bp_wire(dv & lw, dv & ~lw, q);
                    }
                }
                tw[stretch] += now() - a;
                nf[stretch]++;
                /* the twin's drive == the model's: the model's own phys arrays moved by its drive;
                 * compare the direction per pixel from against the model's lvl / P after the frame */
                long d = 0;
                size_t fi = 0;
                for (int y = 0; y < EPD_H && d < 1000; y++)
                    for (int x = 0; x < EPD_W; x++) {
                        size_t i = (size_t)y * EPD_W + x;
                        int ph4 = phase_at(W4, x, y), d4 = ph4 == 2 ? 1 : ph4 == 1 ? -1 : 0;
                        if (d4 != v.lastdir[i]) { if (!d) fi = i; d++; }
                    }
                if (d) { printf("BP4 gate: %s R %d %s frame %d: %ld px drive differently, first (%zu, %zu): twin %d model %d\n",
                                bp_lname(&p), R, wide ? "wide" : "small", f, d, fi % EPD_W, fi / EPD_W,
                                phase_at(W4, (int)(fi % EPD_W), (int)(fi / EPD_W)), v.lastdir[fi]); lbad++; }
#ifdef EPD_BP4G_H
                for (int k = 0; k < ns; k++) {
                    double pm, vm, dm;
                    int pk = gp[k].b.npk;
                    bp4_gpu_frame(&gp[k], fc, &pm, &vm, &dm);
                    if (dm < 0) gtm[k] = 0;
                    gd[k][stretch] += dm; gv[k][stretch] += vm;
                    if (pk) gpk[k] += pm, gnp[k]++;
                    if (dm + vm > gmax[k]) gmax[k] = dm + vm;
                    memcpy(W5, E.tpl, sizeof W5);
                    bp_gpu_words(&gp[k].b, W5);
                    size_t fw;
                    long wd = bp_wdiff(W4, W5, &fw);
                    if (wd) { printf("BP4 gate: %s R %d shape %dx%d frame %d: GPU: %ld words differ, first line %zu word %zu: cpu %08x gpu %08x\n",
                                     bp_lname(&p), R, sw[k], sh[k], f, wd, fw / EPD_LINE, fw % EPD_LINE, W4[fw], W5[fw]); gbad[k]++; }
                    if (f % 8 == 7 || f == 159) {
                        long sd = bp4_gpu_cmp(&gp[k], &b4);
                        if (sd) { printf("BP4 gate: %s R %d shape %dx%d frame %d: GPU state: %ld plane words differ\n", bp_lname(&p), R, sw[k], sh[k], f, sd); gbad[k]++; }
                    }
                    GLenum e = bpgl.glGetError();
                    if (e != GL_NO_ERROR) { printf("BP4 gate: shape %dx%d: GL error 0x%x\n", sw[k], sh[k], e); gbad[k]++; }
                }
#endif
                if (f % 8 == 7 || f == 159) {
                    long sd = 0;
                    for (size_t i = 0; i < EPD_NPX; i++) {
                        size_t o = (i / EPD_W) * BP_NK + (i % EPD_W) / 32;
                        unsigned sh = 31 - (unsigned)(i % EPD_W) % 32;
                        int op = 0, ow = 0, ac = 0, dc = 0;
                        for (int q = 0; q < 4; q++) op |= (int)((b4.op[q][o] >> sh) & 1) << q;
                        for (int q = 0; q < 2; q++) ow |= (int)((b4.ow[q][o] >> sh) & 1) << q;
                        for (int q = 0; q < 5; q++) ac |= (int)((b4.ac[q][o] >> sh) & 1) << q;
                        for (int q = 0; q < 8; q++) dc |= (int)((b4.dc[q][o] >> sh) & 1) << q;
                        ac = ac >= 16 ? ac - 32 : ac; dc = (int)(int8_t)dc;
                        int lw = (int)((b4.lw[o] >> sh) & 1), pp = (int)((b4.p[o] >> sh) & 1);
                        op = lw ? R - op : op;          /* (the planes keep q = frames to the level's rail) */
                        int fl = pp | (int)((b4.pd[o] >> sh) & 1) << 1 | (int)((b4.rv[o] >> sh) & 1) << 2 | (int)((b4.h8[o] >> sh) & 1) << 3;
                        sd += op != v.opv[i] || ow != v.owed[i] || ac != v.acc[i] || dc != v.dc[i] || lw != v.lvl[i] || fl != v.fl[i];
                    }
                    if (sd) { printf("BP4 gate: %s R %d %s frame %d: state: %ld px differ\n", bp_lname(&p), R, wide ? "wide" : "small", f, sd); lbad++; }
                }
                nfr++;
            }
            printf("BP4 gate: %-9s R %2d %-5s: %s; ms a frame, one thread, this host: scroll %.2f vs bp3 %.2f, still %.2f vs %.2f, window %.2f vs %.2f; the wire words alone %.2f; the visits of it: %.2f / %.2f / %.2f\n",
                   bp_lname(&p), R, wide ? "wide" : "small", lbad ? "DIFFERS" : "equal",
                   t4[0] / nf[0] * 1e3, t3[0] / nf[0] * 1e3, t4[1] / nf[1] * 1e3, t3[1] / nf[1] * 1e3, t4[2] / nf[2] * 1e3, t3[2] / nf[2] * 1e3, tw[0] / nf[0] * 1e3, tv[0] / nf[0] * 1e3, tv[1] / nf[1] * 1e3, tv[2] / nf[2] * 1e3);
            nbad += lbad;
#ifdef EPD_BP4G_H
            for (int k = 0; k < ns; k++) {
                printf("BP4 GPU:  %-9s R %2d %-5s shape %dx%d: %s; drive ms a frame scroll %.2f, still %.2f, window %.2f; visit %.2f / %.2f / %.2f; max %.2f; pack %.2f an upload (%s)\n",
                       bp_lname(&p), R, wide ? "wide" : "small", sw[k], sh[k], gbad[k] ? "DIFFERS" : "equal",
                       gd[k][0] / nf[0], gd[k][1] / nf[1], gd[k][2] / nf[2], gv[k][0] / nf[0], gv[k][1] / nf[1], gv[k][2] / nf[2], gmax[k],
                       gnp[k] ? gpk[k] / gnp[k] : 0.0, gtm[k] ? "GPU timer" : "no timer");
                nbad += gbad[k];
                bp4_gpu_close(&gp[k]);
            }
#endif
            tier_free(&v);
            bp_masks_free(&m);
        }
    }
    printf("BP4 gate: %ld frames, %ld bad (M %d)\n", nfr, nbad, M);
    bp4_free(&b4); bp3_free(&b3);
    free(pg); free(tg);
    return nbad ? 1 : 0;
}

/* STAG_BP4_LANES: the twin's lanes (../epd-bp4.h bp4_lane_rows), host only.
 * (a) every lane armed from D at frame 0 == the plain twin (bp4_rows), words
 * every frame and all planes every 8th, through a scroll then windows; (b) a
 * popup evict over (300, 400, 500, 380), padded to 16 px, targets frozen for
 * a cycle first: every byte acks 1 | tog << 1 within 3 frames, D over the
 * rect = the target's rail, dc in +-127, k 0, D elsewhere untouched, the
 * parked lanes never drive; (c) the rect re-armed from its D: no lane parked
 * after 2 frames. STAG_BP4L_BREAK=1 (one D word's dc off by one) and 2 (the
 * evict written without its tog, 3 expected) must fail. */
static int bp4_lane_gate(void)
{
    const char *ms = getenv("STAG_TIER_M"), *ls = getenv("STAG_BP_LAYOUTS");
    const int ph = 1404 * 4, M = ms ? atoi(ms) : 4, brk = getenv("STAG_BP4L_BREAK") ? atoi(getenv("STAG_BP4L_BREAK")) : 0;
    const size_t nd = (size_t)EPD_PK_S * (EPD_H + EPD_PK_DPAD);
    uint8_t *pg = page(ph), *tg = malloc(EPD_NPX), *T = calloc((size_t)EPD_PK_S * EPD_H, 1), *ack = calloc((size_t)BP_GW * EPD_H, 1);
    uint32_t *D = calloc(nd, 4), *D0 = malloc(nd * 4);
    static uint32_t Wp[EPD_WORDS], Wl[EPD_WORDS];
    bp4_state bp, bl;
    long nbad = 0, nfr = 0;
    if (!pg || !tg || !T || !ack || !D || !D0 || bp4_alloc(&bp) || bp4_alloc(&bl)) return 2;
    char lb[128], *lsv;
    snprintf(lb, sizeof lb, "%s", ls && *ls ? ls : "64 wbayer 0");
    for (char *lt = strtok_r(lb, " ,", &lsv); lt; lt = strtok_r(NULL, " ,", &lsv))
    for (int wide = 0; wide < 2; wide++) {
        const int R = 9;
        const char *wn = wide ? "wide" : "small";
        epd_stag_par p = { .G = 2, .n = 1, .ctl = 3, .K = 8, .payrule = 1, .pcap = 3, .htop = 2, .dither = 0, .R = R };
        bp_masks m;
        if (bp_layout(&p, lt) || !bp3_par_ok(&p) || bp_masks_make(&m, &p)) { printf("BP4L: layout %s: not taken\n", lt); return 2; }
        long lbad = 0;
        memcpy(tg, pg, EPD_NPX);
        for (int i = 0; i < BP4_NALL; i++) memset(*bp4_slot(&bp, i), 0, BP_NW * 4);
        bp4_lane_park(&bl);
        memset(ack, 1, (size_t)BP_GW * EPD_H);
        for (int y = 0; y < EPD_H; y++)
            for (int x = 0; x < EPD_W; x++) {
                size_t i = (size_t)y * EPD_W + x, o = (size_t)y * BP_NK + x / 32;
                uint32_t bit = 1u << (31 - x % 32);
                int l = tg[i] >= 16, dc = wide ? (int)((i * 37) % 201) - 100 : (int)(i % 7) - 3;
                bp4_setbit(bp.lw, o, bit, l); bp4_setbit(bp.tb, o, bit, l);
                for (int j = 0; j < 8; j++) bp4_setbit(bp.dc[j], o, bit, (dc >> j) & 1);
                D[(size_t)y * EPD_PK_S + x] = epd_stag_cold(l ? 30 : 0, l ? R : 0, 0, dc + (brk == 1 && i == 777777));
                T[(size_t)y * EPD_PK_S + x] = (uint8_t)(tg[i] | EPD_PK_ARM);
            }
        bp4_lane_pack(&bl, T, ALL);
        int y0 = 0;
        uint32_t fc = 61;
        for (int f = 0; f < 120 && lbad < 5; f++, fc++, nfr++) {     /* (a) */
            if (f < 64 ? f % 2 == 0 : f % 4 == 0) {
                y0 = (y0 + 17) % (ph - EPD_H);
                epd_rect r = f < 64 ? ALL : (f / 4) % 2 ? (epd_rect){ 300, 400, 500, 380 } : (epd_rect){ 37, 101, 1500, 1203 };
                for (int y = r.y; y < r.y + r.h; y++)
                    for (int x = r.x; x < r.x + r.w; x++) {
                        tg[(size_t)y * EPD_W + x] = pg[(size_t)(y0 + y) * EPD_W + x];
                        T[(size_t)y * EPD_PK_S + x] = (uint8_t)(tg[(size_t)y * EPD_W + x] | EPD_PK_ARM);
                    }
                bp_state t = { .tb = bp.tb };
                bp_targets(&t, tg, r);
                bp4_lane_pack(&bl, T, r);
            }
            memcpy(Wp, E.tpl, sizeof Wp); memcpy(Wl, E.tpl, sizeof Wl);
            bp4_rows(&bp, &m, &p, M, fc, E.tpl, Wp, 0, EPD_H);
            bp4_lane_rows(&bl, &m, &p, M, fc, E.tpl, Wl, 0, EPD_H, D, D, ack);
            size_t fw;
            long wd = bp_wdiff(Wp, Wl, &fw);
            if (wd) {
                printf("BP4L: %s %s frame %d: %ld words differ, first line %zu word %zu\n", bp_lname(&p), wn, f, wd, fw / EPD_LINE, fw % EPD_LINE);
                if (getenv("STAG_BP4L_DBG")) {
                    int y = EPD_DATA0 / EPD_LINE + EPD_H - 1 - (int)(fw / EPD_LINE), k = ((int)(fw % EPD_LINE) - EPD_DATA0 % EPD_LINE) / 4;
                    size_t o = (size_t)y * BP_NK + k;
                    printf("  y %d k %d: plain lw %08x tb %08x p %08x q %08x %08x st %08x | lanes lw %08x tb %08x p %08x q %08x %08x st %08x park %08x arm %08x ev %08x | W %08x %08x\n",
                           y, k, bp.lw[o], bp.tb[o], bp.p[o], bp.op[0][o], bp.op[1][o], bp.st[o], bl.lw[o], bl.tb[o], bl.p[o], bl.op[0][o], bl.op[1][o], bl.st[o], bl.park[o], bl.arm[o], bl.ev[o], Wp[fw], Wl[fw]);
                }
                lbad++;
            }
            if (f >= 2 && (f % 8 == 7 || f == 119)) {
                long sd = 0;
                for (int i = 0; i < BP4_NPL + 1; i++)   /* (the controller's planes and ST) */
                    for (size_t o = 0; o < BP_NW; o++) sd += (*bp4_slot(&bp, i))[o] != (*bp4_slot(&bl, i))[o];
                for (size_t o = 0; o < BP_NW; o++) sd += bl.park[o] != 0;
                if (sd) { printf("BP4L: %s %s frame %d: %ld plane words differ\n", bp_lname(&p), wn, f, sd); lbad++; }
            }
        }
        for (int f = 0; f < 2; f++, fc++, nfr++) {    /* (b): the targets frozen a cycle, then the evict */
            memcpy(Wl, E.tpl, sizeof Wl);
            bp4_lane_rows(&bl, &m, &p, M, fc, E.tpl, Wl, 0, EPD_H, D, D, ack);
        }
        epd_rect er = epd_stag_lane_pad((epd_rect){ 300, 400, 500, 380 });
        memcpy(D0, D, nd * 4);
        for (int y = er.y; y < er.y + er.h; y++)
            for (int x = er.x; x < er.x + er.w; x++) T[(size_t)y * EPD_PK_S + x] = (uint8_t)(tg[(size_t)y * EPD_W + x] | EPD_PK_EVICT | (brk == 2 ? 0 : EPD_PK_TOG));
        bp4_lane_pack(&bl, T, er);
        int want = 3, f = 0;
        long un = 1;
        for (; f < 3 && un; f++, fc++, nfr++) {
            memcpy(Wl, E.tpl, sizeof Wl);
            bp4_lane_rows(&bl, &m, &p, M, fc, E.tpl, Wl, 0, EPD_H, D, D, ack);
            un = 0;
            for (int y = er.y; y < er.y + er.h; y++)
                for (int g = er.x / 8; g < (er.x + er.w) / 8; g++) un += ack[(size_t)y * BP_GW + g] != want;
        }
        if (un) { printf("BP4L: %s %s: evict: %ld bytes not acked after %d frames\n", bp_lname(&p), wn, un, f); lbad++; }
        long dl = 0, dn = 0, dv = 0;
        for (int y = 0; y < EPD_H; y++)
            for (int x = 0; x < EPD_W; x++) {
                size_t i = (size_t)y * EPD_PK_S + x;
                if (!(x >= er.x && x < er.x + er.w && y >= er.y && y < er.y + er.h)) { dn += D[i] != D0[i]; continue; }
                dl += EPD_STAG_LEVEL(D[i]) != (tg[(size_t)y * EPD_W + x] >= 16 ? 30 : 0);
                dv += EPD_STAG_DC(D[i]) > 127 || EPD_STAG_DC(D[i]) < -127 || EPD_STAG_K(D[i]) || EPD_STAG_OP(D[i]) > R;
            }
        if (dl || dn || dv) { printf("BP4L: %s %s: evict: D level != rail %ld, D outside changed %ld, dc/op/k out of range %ld\n", bp_lname(&p), wn, dl, dn, dv); lbad++; }
        long dr = 0;
        for (int k = 0; k < 4; k++, fc++, nfr++) {    /* (parked: no drive in the rect) */
            memcpy(Wl, E.tpl, sizeof Wl);
            bp4_lane_rows(&bl, &m, &p, M, fc, E.tpl, Wl, 0, EPD_H, D, D, ack);
            for (int y = er.y; y < er.y + er.h; y++)
                for (int x = er.x; x < er.x + er.w; x++) dr += phase_at(Wl, x, y) != 0;
        }
        if (dr) { printf("BP4L: %s %s: %ld parked pixel frames driven\n", bp_lname(&p), wn, dr); lbad++; }
        for (int y = er.y; y < er.y + er.h; y++)     /* (c) */
            for (int x = er.x; x < er.x + er.w; x++) T[(size_t)y * EPD_PK_S + x] = (uint8_t)(tg[(size_t)y * EPD_W + x] | EPD_PK_ARM);
        bp4_lane_pack(&bl, T, er);
        for (int k = 0; k < 2; k++, fc++, nfr++) {
            memcpy(Wl, E.tpl, sizeof Wl);
            bp4_lane_rows(&bl, &m, &p, M, fc, E.tpl, Wl, 0, EPD_H, D, D, ack);
        }
        long pk = 0;
        for (size_t o = 0; o < BP_NW; o++) pk += __builtin_popcount(bl.park[o]);
        if (pk) { printf("BP4L: %s %s: re-arm: %ld lanes still parked\n", bp_lname(&p), wn, pk); lbad++; }
        printf("BP4L: %-9s %-5s: %s\n", bp_lname(&p), wn, lbad ? "DIFFERS" : "equal");
        nbad += lbad;
        bp_masks_free(&m);
    }
    printf("BP4L gate: %ld frames, %ld bad (M %d)\n", nfr, nbad, M);
    bp4_free(&bp); bp4_free(&bl);
    free(pg); free(tg); free(T); free(ack); free(D); free(D0);
    return nbad ? 1 : 0;
}
#endif
