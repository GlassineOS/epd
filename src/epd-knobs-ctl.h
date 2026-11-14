/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-knobs-ctl — the control lines' side of epd-knobs.h, for the desktop
 * daemon: the table of the owners it links (the GPU's with -DEPD_GPU), the
 * rows it parses itself (EPD_KN_FN: several globals, or the open stream) and
 * epd_knobs_ctl, which apply_cmd calls for every line (GLASSINE_EPDD, the
 * control FIFO, eink.conf's "@" lines). Header only: glassine-epdd-desktop.c and
 * test/epd-knobs-test.c (against the parser it replaced) include it. */
#ifndef EPD_KNOBS_CTL_H
#define EPD_KNOBS_CTL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "epd-knobs.h"
#include "epd-drm.h"
#include "epd-ingest.h"
#include "epd-stagger.h"

static const epd_knob epd_ctl_knobs[] = {
    EPD_KNOBS_ENGINE(EPD_KNOB_ROW)
    EPD_KNOBS_DRM(EPD_KNOB_ROW)
#ifdef EPD_GPU
    EPD_KNOBS_GPU(EPD_KNOB_ROW)
#endif
    EPD_KNOBS_INGEST(EPD_KNOB_ROW)
};
#define EPD_CTL_KNOBS_N EPD_KNOBS_N(epd_ctl_knobs)

#define EPD_KS(var, x) __atomic_store_n(&(var), (x), __ATOMIC_RELAXED)
/* the EPD_KN_FN rows with a name; v: the text after "name=", ctx: the open
 * stream (NULL: none yet) */
static void epd_ctl_fn(const epd_knob *k, const char *v, void *ctx)
{
    epd_stream *eng = ctx;
    if (k->p == &epd_stream_stagger) {          /* on | off | check (the CPU reference in lockstep) */
        if (!strncmp(v, "on", 2)) EPD_KS(epd_stream_stagger, 1), EPD_KS(epd_stream_stag_check, 0);
        else if (!strncmp(v, "off", 3)) EPD_KS(epd_stream_stagger, 0), EPD_KS(epd_stream_stag_check, 0);
        else if (!strncmp(v, "check", 5)) EPD_KS(epd_stream_stagger, 1), EPD_KS(epd_stream_stag_check, 1);
    } else if (k->p == &epd_stream_stag_pay) {  /* panel (the payback rule) | N (the frames rule) */
        if (!strncmp(v, "panel", 5)) EPD_KS(epd_stream_stag_payrule, 1);
        else EPD_KS(epd_stream_stag_payrule, 0), EPD_KS(epd_stream_stag_pay, atoi(v) > 0 ? atoi(v) : 0);
    } else if (k->p == &epd_stream_stag_stripe) {   /* (the GPU engine: 64 only; blocks, cells: stripe 64 cut up) */
        int cw = 0, ch = 0, wb = !strncmp(v, "wbayer", 6);
        if (!strncmp(v, "cells", 5) && epd_stag_cells_parse(v, &cw, &ch))
            fprintf(stderr, "glassine-epdd-desktop: stagger_stripe: %.16s is no cell shape (cells, cellsN, cellsWxH: W 8 16 32 64, H 1..64, powers of two)\n", v);
        if (wb) cw = 8, ch = 1;
        EPD_KS(epd_stream_stag_cellw, cw), EPD_KS(epd_stream_stag_cellh, ch);
        EPD_KS(epd_stream_stag_cellord, wb ? EPD_STAG_CORD_WBAYER : EPD_STAG_CORD_BN);
        EPD_KS(epd_stream_stag_blocks, !strncmp(v, "blocks", 6));
        EPD_KS(epd_stream_stag_stripe, epd_stream_stag_blocks || wb || !strncmp(v, "cells", 5) || atoi(v) == 64 ? 64 : 0);   /* (a bad shape: stripes) */
    } else if (k->p == &epd_stream_gpu_hang_test) {
        if (*v == '1') {
            EPD_KS(epd_stream_gpu_hang_test, 1);
            fprintf(stderr, "glassine-epdd-desktop: gpu_hang_test: the next GPU frame is taken as hung (abandon, then the revive)\n");
        }
    } else if (k->p == &epd_stream_rt_umin) {   /* N | default: the running threads' too */
        int u = strncmp(v, "default", 7) ? atoi(v) : -1;
        if (eng) epd_stream_set_rt_umin(eng, u); else EPD_KS(epd_stream_rt_umin, u);
    } else if (k->p == &epd_stream_qos) {       /* global | off | cpu:N (us; cpu:0 = none tolerated) */
        if (!strncmp(v, "global", 6)) EPD_KS(epd_stream_qos, -1);
        else if (!strncmp(v, "off", 3)) EPD_KS(epd_stream_qos, -2);
        else if (!strncmp(v, "cpu:", 4)) EPD_KS(epd_stream_qos, atoi(v + 4));
    } else if (k->p == &epd_ingest_sync) {      /* read by every ingest=cpu frame (epd-ingest.h) */
        EPD_KS(epd_ingest_sync, epd_ingest_sync_of(v));
    }
}
#undef EPD_KS

/* a control line's knobs (only: that name alone, NULL = all); eng: the open
 * stream or NULL. The rows that matched */
static inline int epd_knobs_ctl(const char *cmd, const char *only, epd_stream *eng)
{
    return epd_knobs_apply(epd_ctl_knobs, EPD_CTL_KNOBS_N, cmd, only, epd_ctl_fn, eng);
}

#endif
