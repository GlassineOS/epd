/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-knobs-test — the knob table (epd-knobs.h, epd-knobs-ctl.h) against what
 * it replaced:
 *   1. control lines: epd_knobs_ctl against a frozen copy of apply_cmd's (and
 *      upd_parse's) lines for these knobs before the table (ref_apply below,
 *      glassine-epdd-desktop.c at cb27c8b), every global compared after each
 *      line: every name with every value of a corpus, and random lines of
 *      several knobs among other commands, without and with an open stream
 *   2. a knob named twice in a line: the first counts (GLASSINE_EPDD's own
 *      before glassine-ddr's: glassine-session-sway)
 *   3. no control name is the tail of another ("xjit=" would hide "jit="):
 *      the table's names and those of NAMES (a file of "name=" lines: run.sh
 *      takes them from the daemon and the policy)
 *   4. the engine's environment: epd_knob_env on its rows against the lazy
 *      reads it replaced (epd-engine.c at cb27c8b), and epd_init applying them
 *      once, a knob already set kept
 *   epd-knobs-test WBF [NAMES]   exit 0 = all equal */
#define EPD_GPU 1                       /* (the GPU's rows in the table too) */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../epd-knobs-ctl.h"

/* the owners not linked here (the engine and the stagger are) */
EPD_KNOBS_DRM(EPD_KNOB_DEFINE)
EPD_KNOBS_GPU(EPD_KNOB_DEFINE)
EPD_KNOBS_INGEST(EPD_KNOB_DEFINE)

/* epd-ingest.c's, as they were at cb27c8b (epd-ingest.c needs EGL): the
 * reference calls both, the table epd_ingest_sync_of (ingest_kernel is a
 * word list now) */
int epd_ingest_kernel_of(const char *s) { return !strncmp(s, "tile", 4) ? 1 : !strncmp(s, "scalar", 6) ? 2 : 0; }
int epd_ingest_sync_of(const char *s) { return !strncmp(s, "off", 3) ? -1 : !strncmp(s, "on", 2) ? 0 : (*s >= '0' && *s <= '9') ? atoi(s) : -1; }
/* the open stream's rt_umin (epd-drm.c): recorded */
static int umin_calls, umin_last;
void epd_stream_set_rt_umin(epd_stream *s, int v) { (void)s; umin_calls++; umin_last = v; }

/* ---- the reference: glassine-epdd-desktop.c's lines at cb27c8b, as they were ---- */
static void ref_apply(const char *cmd, epd_stream *eng)
{
    const char *v;
    if ((v = strstr(cmd, "target_pool="))) epd_target_pool = atoi(v + 12) < 0 ? 0 : atoi(v + 12) > 8 ? 8 : atoi(v + 12);
    /* read by every ingest=cpu frame (epd-ingest.h) */
    if ((v = strstr(cmd, "ingest_kernel="))) __atomic_store_n(&epd_ingest_kernel, epd_ingest_kernel_of(v + 14), __ATOMIC_RELAXED);
    if ((v = strstr(cmd, "ingest_bands="))) {
        int n = atoi(v + 13);
        __atomic_store_n(&epd_ingest_bands, n < 1 ? 1 : n > 4 ? 4 : n, __ATOMIC_RELAXED);
    }
    if ((v = strstr(cmd, "ingest_sync="))) __atomic_store_n(&epd_ingest_sync, epd_ingest_sync_of(v + 12), __ATOMIC_RELAXED);
    if ((v = strstr(cmd, "ingest_partial="))) __atomic_store_n(&epd_ingest_partial, strncmp(v + 15, "off", 3) != 0, __ATOMIC_RELAXED);
    if ((v = strstr(cmd, "link_warm="))) {
        int ms = atoi(v + 10);
        __atomic_store_n(&epd_stream_link_warm_ms, ms < 0 ? 0 : ms, __ATOMIC_RELAXED);
    }
    if (strstr(cmd, "link_log=on")) epd_stream_link_log = 1;
    if (strstr(cmd, "link_log=off")) epd_stream_link_log = 0;
    /* read by every claim (epd-engine.h epd_claim_kernel) */
    if (strstr(cmd, "claim_kernel=soa")) __atomic_store_n(&epd_claim_kernel, 0, __ATOMIC_RELAXED);
    if (strstr(cmd, "claim_kernel=packed")) __atomic_store_n(&epd_claim_kernel, 1, __ATOMIC_RELAXED);
    if (strstr(cmd, "claim_kernel=check")) __atomic_store_n(&epd_claim_kernel, 2, __ATOMIC_RELAXED);
    /* banded claims (every claim), the job (the stream thread's loop top) */
    if ((v = strstr(cmd, "claim_bands="))) {
        int n = atoi(v + 12);
        __atomic_store_n(&epd_claim_bands, n < 1 ? 1 : n > EPD_CLAIM_BANDS_MAX ? EPD_CLAIM_BANDS_MAX : n, __ATOMIC_RELAXED);
    }
    if (strstr(cmd, "drive_overlap=on")) __atomic_store_n(&epd_stream_drive_overlap, 1, __ATOMIC_RELAXED);
    if (strstr(cmd, "drive_overlap=off")) __atomic_store_n(&epd_stream_drive_overlap, 0, __ATOMIC_RELAXED);
    /* (upd_parse's) */
    if (strstr(cmd, "worker_rt=fifo")) __atomic_store_n(&epd_stream_worker_rt, 0, __ATOMIC_RELAXED);
    if (strstr(cmd, "worker_rt=nice")) __atomic_store_n(&epd_stream_worker_rt, 1, __ATOMIC_RELAXED);
    /* staggered cut steps (epd-drm.c epd_stream_stagger), live */
    if (strstr(cmd, "stagger=on")) epd_stream_stagger = 1, epd_stream_stag_check = 0;
    if (strstr(cmd, "stagger=off")) epd_stream_stagger = 0, epd_stream_stag_check = 0;
    if (strstr(cmd, "stagger=check")) epd_stream_stagger = 1, epd_stream_stag_check = 1;
    if ((v = strstr(cmd, "stagger_direct="))) epd_stream_stag_direct = atoi(v + 15) != 0;
#ifdef EPD_GPU
    if (strstr(cmd, "stagger_tskip=on")) epd_gpu_stag_tskip = 1;
    if (strstr(cmd, "stagger_tskip=off")) epd_gpu_stag_tskip = 0;
    if (strstr(cmd, "stagger_lead=on")) epd_gpu_stag_lead = 1;
    if (strstr(cmd, "stagger_lead=off")) epd_gpu_stag_lead = 0;
    if (strstr(cmd, "drive_state=heap")) epd_gpu_drive_state = 1;
    if (strstr(cmd, "drive_state=tex")) epd_gpu_drive_state = 0;
#endif
    if ((v = strstr(cmd, "stagger_ctl="))) epd_stream_stag_ctl = atoi(v + 12) < 0 ? 0 : atoi(v + 12) > 4 ? 4 : atoi(v + 12);
    if (strstr(cmd, "stagger_a21=on")) epd_stream_stag_a21 = 1;
    if (strstr(cmd, "stagger_a21=off")) epd_stream_stag_a21 = 0;
    if ((v = strstr(cmd, "stagger_idle="))) epd_stream_stag_sidle = !strncmp(v + 13, "shared", 6);
    if ((v = strstr(cmd, "stagger_k="))) epd_stream_stag_k = atoi(v + 10) < 0 ? 0 : atoi(v + 10) > 15 ? 15 : atoi(v + 10);
    if ((v = strstr(cmd, "stagger_pay="))) {
        if (!strncmp(v + 12, "panel", 5)) epd_stream_stag_payrule = 1;
        else epd_stream_stag_payrule = 0, epd_stream_stag_pay = atoi(v + 12) > 0 ? atoi(v + 12) : 0;
    }
    if ((v = strstr(cmd, "stagger_stripe="))) {     /* (the GPU engine: 64 only; blocks, cells: stripe 64 cut up) */
        int cw = 0, ch = 0, wb = !strncmp(v + 15, "wbayer", 6);
        if (!strncmp(v + 15, "cells", 5) && epd_stag_cells_parse(v + 15, &cw, &ch))
            fprintf(stderr, "glassine-epdd-desktop: stagger_stripe: %.16s is no cell shape (cells, cellsN, cellsWxH: W 8 16 32 64, H 1..64, powers of two)\n", v + 15);
        if (wb) cw = 8, ch = 1;
        epd_stream_stag_cellw = cw, epd_stream_stag_cellh = ch;
        epd_stream_stag_cellord = wb ? EPD_STAG_CORD_WBAYER : EPD_STAG_CORD_BN;
        epd_stream_stag_blocks = !strncmp(v + 15, "blocks", 6);
        epd_stream_stag_stripe = epd_stream_stag_blocks || wb || !strncmp(v + 15, "cells", 5) || atoi(v + 15) == 64 ? 64 : 0;   /* (a bad shape: stripes) */
    }
    if ((v = strstr(cmd, "stagger_cap="))) epd_stream_stag_cap = atoi(v + 12) < 0 ? -1 : atoi(v + 12);
    if ((v = strstr(cmd, "stagger_linger="))) epd_stream_stag_linger = atoi(v + 15);
    if ((v = strstr(cmd, "stagger_warm="))) epd_stream_stag_warm = strncmp(v + 13, "off", 3) != 0;
    if ((v = strstr(cmd, "stagger_entry="))) epd_stream_stag_entry = !strncmp(v + 14, "first", 5);
    if (strstr(cmd, "gpu_hang_test=1")) {
        epd_stream_gpu_hang_test = 1;
        fprintf(stderr, "glassine-epdd-desktop: gpu_hang_test: the next GPU frame is taken as hung (abandon, then the revive)\n");
    }
    if ((v = strstr(cmd, "hammer_top="))) epd_stream_stag_htop = atoi(v + 11) < 0 ? 0 : atoi(v + 11) > 7 ? 7 : atoi(v + 11);
#ifdef EPD_GPU
    if ((v = strstr(cmd, "stagger_timer="))) epd_gpu_stag_timer = atoi(v + 14) != 0;
#endif
    if (strstr(cmd, "jit=on")) epd_stream_jit = 1;
    if (strstr(cmd, "jit=off")) epd_stream_jit = 0;
    if (strstr(cmd, "jit_size=on")) epd_stream_jit_size = 1;
    if (strstr(cmd, "jit_size=off")) epd_stream_jit_size = 0;
    /* the hand-off inbox (a device A/B: lock = the old path) */
    if (strstr(cmd, "handoff=inbox")) epd_stream_inbox = 1;
    if (strstr(cmd, "handoff=lock")) epd_stream_inbox = 0;
    /* (engine only): threads per frame, their uclamp.min, CPU wakeup latency hold */
    if ((v = strstr(cmd, "nwork="))) epd_stream_nwork = atoi(v + 6);
    if ((v = strstr(cmd, "rt_umin="))) {
        int u = strncmp(v + 8, "default", 7) ? atoi(v + 8) : -1;
        if (eng) epd_stream_set_rt_umin(eng, u); else epd_stream_rt_umin = u;
    }
    if ((v = strstr(cmd, "qos="))) {        /* global | off | cpu:N (us; cpu:0 = none tolerated) */
        if (!strncmp(v + 4, "global", 6)) epd_stream_qos = -1;
        else if (!strncmp(v + 4, "off", 3)) epd_stream_qos = -2;
        else if (!strncmp(v + 4, "cpu:", 4)) epd_stream_qos = atoi(v + 8);
    }
}

/* ---- the engine's lazy env reads at cb27c8b (epd-engine.c), by variable ---- */
static int ref_env(const char *var)
{
    const char *v = getenv(var);
    int n;
    if (!strcmp(var, "GLASSINE_CLAIM_STATE") || !strcmp(var, "GLASSINE_CLAIM_KERNEL"))
        return !v ? 0 : !strcmp(v, "packed") ? 1 : !strcmp(v, "check") ? 2 : 0;
    if (!strcmp(var, "GLASSINE_STAGGER_STATE")) return v && !strcmp(v, "lanes");
    if (!strcmp(var, "GLASSINE_CLAIM_SKIP") || !strcmp(var, "GLASSINE_CLAIM_PREFETCH")) return v && !strcmp(v, "1");
    if (!strcmp(var, "GLASSINE_CLAIM_TAIL16") || !strcmp(var, "GLASSINE_FINISH_MASK")) return !(v && !strcmp(v, "0"));
    if (!strcmp(var, "GLASSINE_CLAIM_BANDS")) {
        n = v ? atoi(v) : 4;
        return n < 1 ? 1 : n > EPD_CLAIM_BANDS_MAX ? EPD_CLAIM_BANDS_MAX : n;
    }
    return -999;
}

#define NK EPD_CTL_KNOBS_N
static int fails;
#define FAIL(...) do { printf("epd-knobs: FAIL: " __VA_ARGS__); printf("\n"); fails++; } while (0)
static unsigned rs = 12345;
static unsigned rnd(void) { rs = rs * 1103515245u + 12345u; return rs >> 8; }

static void knobs_set(const int *v) { for (int i = 0; i < NK; i++) *epd_ctl_knobs[i].p = v[i]; }
static void knobs_get(int *v) { for (int i = 0; i < NK; i++) v[i] = *epd_ctl_knobs[i].p; }

static long lines;
/* one line through both, from the same start; 0 = the same globals and calls */
static int compare(const char *cmd, epd_stream *eng)
{
    int s0[NK], a[NK], b[NK];
    for (int i = 0; i < NK; i++) s0[i] = (int)(rnd() % 80) - 5;
    knobs_set(s0); umin_calls = 0; umin_last = -77;
    ref_apply(cmd, eng);
    knobs_get(a);
    int ca = umin_calls, la = umin_last;
    knobs_set(s0); umin_calls = 0; umin_last = -77;
    epd_knobs_ctl(cmd, NULL, eng);
    knobs_get(b);
    lines++;
    int bad = ca != umin_calls || la != umin_last;
    for (int i = 0; i < NK; i++)
        if (a[i] != b[i]) {
            if (!bad) FAIL("\"%s\"%s: %s %d (was %d), the table %d", cmd, eng ? " (stream open)" : "", epd_ctl_knobs[i].var, a[i], s0[i], b[i]);
            bad = 1;
        }
    if (bad && ca != umin_calls) FAIL("\"%s\": set_rt_umin calls %d / %d", cmd, ca, umin_calls);
    return bad;
}

static const char *const vals[] = {
    "on", "off", "check", "panel", "blocks", "cells", "cells16", "cells8x4", "cells64x64", "cells9", "cellsx", "wbayer",
    "0", "1", "-1", "2", "3", "4", "5", "6", "7", "8", "9", "15", "16", "64", "99", "-5", "250", "30000", "100000", "-100000",
    "default", "global", "cpu:0", "cpu:600", "cpu:", "fifo", "nice", "tex", "heap", "row", "tile", "scalar", "shared", "pixel",
    "first", "engine", "inbox", "lock", "soa", "packed", "", "x", "onx", "offf", "o", "1x", "01", " 3",
};
#define NV (int)(sizeof vals / sizeof *vals)
static const char *const noise[] = { "stats", "verbose", "a2=on", "idle_ms=250", "payback=on", "heal=2", "full", "mark", "boost=150", "ink=apps" };

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: epd-knobs-test WBF [NAMES]\n"); return 2; }
    FILE *err = stderr;
    stderr = fopen("/dev/null", "w");          /* (both parsers' notes: the same lines) */
    if (!stderr) stderr = err;
    static int fake;
    epd_stream *open_s = (epd_stream *)&fake;
    /* 1a. every name with every value, alone and between other commands */
    int named = 0;
    for (int i = 0; i < NK; i++) {
        const epd_knob *k = &epd_ctl_knobs[i];
        if (!*k->name) continue;
        named++;
        for (int j = 0; j < NV; j++) {
            char c[160];
            for (int e = 0; e < 2; e++) {
                snprintf(c, sizeof c, "%s=%s", k->name, vals[j]);
                compare(c, e ? open_s : NULL);
                snprintf(c, sizeof c, "%s %s=%s %s", noise[j % 10], k->name, vals[j], noise[(j + 3) % 10]);
                compare(c, e ? open_s : NULL);
            }
        }
    }
    /* 1b. random lines of 2..6 different knobs */
    for (int n = 0; n < 20000; n++) {
        char c[600] = "";
        size_t o = 0;
        int used[NK] = { 0 }, m = 2 + (int)(rnd() % 5);
        for (int t = 0; t < m; t++) {
            int i = (int)(rnd() % NK);
            if (!*epd_ctl_knobs[i].name || used[i]) continue;
            used[i] = 1;
            if (rnd() % 3 == 0) o += (size_t)snprintf(c + o, sizeof c - o, "%s ", noise[rnd() % 10]);
            o += (size_t)snprintf(c + o, sizeof c - o, "%s=%s ", epd_ctl_knobs[i].name, vals[rnd() % NV]);
        }
        compare(c, n & 1 ? open_s : NULL);
    }
    /* 2. named twice: the first counts */
    static const struct { const char *cmd; int *p; int want; } dup[] = {
        { "jit=off jit=on", &epd_stream_jit, 0 }, { "jit=on jit=off", &epd_stream_jit, 1 },
        { "stagger_k=3 stagger_k=5", &epd_stream_stag_k, 3 }, { "claim_kernel=check claim_kernel=soa", &epd_claim_kernel, 2 },
        { "handoff=lock handoff=inbox", &epd_stream_inbox, 0 },
    };
    for (unsigned d = 0; d < sizeof dup / sizeof *dup; d++) {
        *dup[d].p = -9;
        epd_knobs_ctl(dup[d].cmd, NULL, NULL);
        if (*dup[d].p != dup[d].want) FAIL("\"%s\": %d, not the first's %d", dup[d].cmd, *dup[d].p, dup[d].want);
    }
    /* 3. no name is another's tail */
    char names[512][64];
    int nn = 0;
    for (int i = 0; i < NK; i++) if (*epd_ctl_knobs[i].name) snprintf(names[nn++], 64, "%s=", epd_ctl_knobs[i].name);
    int ntable = nn;
    FILE *nf = argc > 2 ? fopen(argv[2], "r") : NULL;
    char l[64];
    while (nf && nn < 512 && fgets(l, sizeof l, nf)) {
        l[strcspn(l, "\n")] = 0;
        if (*l) snprintf(names[nn++], 64, "%s", l);
    }
    if (nf) fclose(nf);
    for (int i = 0; i < ntable; i++)
        for (int j = 0; j < nn; j++) {
            size_t a = strlen(names[i]), b = strlen(names[j]);
            if (b > a && !strcmp(names[j] + b - a, names[i])) FAIL("\"%s\" ends with the knob \"%s\": the knob's search would find it", names[j], names[i]);
        }
    /* 4a. the engine's env rules */
    static const epd_knob ek[] = { EPD_KNOBS_ENGINE(EPD_KNOB_ROW) };
    static const char *const ev[] = { NULL, "", "0", "1", "2", "4", "5", "-3", "packed", "check", "arrays", "soa", "lanes", "runs", "packedx", "abc", " 1" };
    int nenv = 0;
    for (unsigned i = 0; i < sizeof ek / sizeof *ek; i++) {
        if (!*ek[i].env || ek[i].kind == EPD_KN_FN) continue;
        nenv++;
        for (unsigned j = 0; j < sizeof ev / sizeof *ev; j++) {
            if (ev[j]) setenv(ek[i].env, ev[j], 1); else unsetenv(ek[i].env);
            int r = ref_env(ek[i].env), t = epd_knob_env(&ek[i]);
            if (r != t) FAIL("%s=%s: was %d, the table %d", ek[i].env, ev[j] ? ev[j] : "(unset)", r, t);
        }
        unsetenv(ek[i].env);
    }
    /* 4b. epd_init: the env once, a knob already set kept */
    FILE *f = fopen(argv[1], "rb");
    if (!f) { perror(argv[1]); return 2; }
    fseek(f, 0, SEEK_END);
    size_t wn = (size_t)ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *wbf = malloc(wn);
    if (!wbf || fread(wbf, 1, wn, f) != wn) return 2;
    fclose(f);
    setenv("GLASSINE_CLAIM_BANDS", "2", 1); setenv("GLASSINE_CLAIM_KERNEL", "packed", 1); setenv("GLASSINE_CLAIM_SKIP", "1", 1);
    setenv("GLASSINE_FINISH_MASK", "0", 1); setenv("GLASSINE_CLAIM_PREFETCH", "1", 1); setenv("GLASSINE_DMABUF_PARTIAL", "0", 1);
    epd_claim_bands = epd_claim_kernel = epd_claim_skip = epd_finish_mask = epd_claim_tail16 = epd_dmabuf_partial = -1;
    epd_claim_prefetch = 0;                     /* (set before: kept) */
    static epd_engine E;
    if (epd_init(&E, wbf, wn, 24)) { FAIL("epd_init"); return 1; }
    if (epd_claim_bands != 2 || epd_claim_kernel != 1 || epd_claim_skip != 1 || epd_finish_mask != 0 || epd_claim_tail16 != 1 ||
        epd_dmabuf_partial != 0 || epd_claim_prefetch != 0)
        FAIL("epd_init: bands %d kernel %d skip %d finish_mask %d tail16 %d dmabuf_partial %d prefetch %d",
             epd_claim_bands, epd_claim_kernel, epd_claim_skip, epd_finish_mask, epd_claim_tail16, epd_dmabuf_partial, epd_claim_prefetch);
    setenv("GLASSINE_CLAIM_BANDS", "3", 1);
    epd_init(&E, wbf, wn, 24);                  /* (resolved: a second open keeps it) */
    if (epd_claim_bands != 2) FAIL("a second epd_init: claim_bands %d", epd_claim_bands);
    /* the stats line */
    char b[2048];
    epd_stream_jit = 1; epd_stream_inbox = 0; epd_stream_stag_k = 8;
    epd_knobs_line(epd_ctl_knobs, NK, b, sizeof b);
    if (!strstr(b, "jit=on") || !strstr(b, "handoff=lock") || !strstr(b, "stagger_k=8")) FAIL("stats line: %s", b);
    stderr = err;
    if (fails) { printf("epd-knobs: %d failures\n", fails); return 1; }
    printf("epd-knobs: ok (%d control names, %ld lines equal to the old parser, %d engine env rules, %d names without a clash)\n",
           named, lines, nenv, nn);
    return 0;
}
