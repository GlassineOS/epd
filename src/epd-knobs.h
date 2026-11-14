/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-knobs — the one list of the runtime knobs the engine (epd-engine.c), the
 * drive (epd-drm.c), the GPU (epd-gpu.c) and the ingest (epd-ingest.c) expose
 * as process globals. Everything else is made from it:
 *   - the extern declarations (below; the owners' headers include this file),
 *   - the definitions with their first values (EPD_KNOBS_<OWNER>(EPD_KNOB_DEFINE)
 *     in the owner, and in test/epd-stream-stub.c for the drive's),
 *   - the control lines' parser: glassine-epdd-desktop's apply_cmd (GLASSINE_EPDD,
 *     the control FIFO, eink.conf's "@" lines) goes through epd_knobs_apply over
 *     a table of these rows, and "stats" prints the named ones,
 *   - the engine's environment: epd_init reads its rows' variables once
 *     (epd_knobs_env); the other owners still read theirs where they did.
 * Adding a knob is a row here (its owner reads the variable; the long
 * description goes to the owner's header and the FIFO help at the top of
 * glassine-epdd-desktop.c). The control names and GLASSINE_* variables are a
 * runtime interface (README "Runtime interfaces"): keep them as they are.
 *
 * A row: X(var, init, def, name, kind, lo, hi, words, env, when)
 *   var    the int global
 *   init   its value before anything set it (-1 and the like: "not set yet",
 *          resolved from env at its owner's open)
 *   def    what env resolves to when the variable is unset or not understood
 *   name   the control name ("name=value"), "" = none (env or code only)
 *   kind   how a value is read:
 *          EPD_KN_INT   atoi, clamped to [lo, hi]
 *          EPD_KN_WORD  words "a|b|c" -> 0, 1, 2. A control value counts when
 *                       it starts with the word; a value that starts with none
 *                       gives lo (EPD_KN_KEEP: the knob is left as it is). An
 *                       env value counts when it is the whole word, else def
 *          EPD_KN_NZ    atoi != 0
 *          EPD_KN_FN    the daemon parses it (several globals, or the open
 *                       stream; glassine-epdd-desktop.c knob_fn); env: the owner
 *   env    the GLASSINE_* variable that sets it ("" = none). The engine's are
 *          read by epd_init from this table; the others by their owner, at
 *          its open or first use, or by the daemon (noted per row)
 *   when   EPD_KN_LIVE: read where it is used, a change applies from then on;
 *          EPD_KN_OPEN: taken when its owner opens (the engine, the stream,
 *          the GPU): a later change waits for the next open
 * A line is searched as before: the first "name=" anywhere in it.
 */
#ifndef EPD_KNOBS_H
#define EPD_KNOBS_H

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { EPD_KN_INT, EPD_KN_WORD, EPD_KN_NZ, EPD_KN_FN };
enum { EPD_KN_LIVE, EPD_KN_OPEN };
#define EPD_KN_KEEP INT_MIN

/* epd-engine.c */
#define EPD_KNOBS_ENGINE(X) \
    X(epd_claim_kernel,   -1, 0, "claim_kernel", EPD_KN_WORD, EPD_KN_KEEP, 0, "soa|packed|check", "GLASSINE_CLAIM_KERNEL", EPD_KN_LIVE) \
    X(epd_claim_bands,    -1, 4, "claim_bands", EPD_KN_INT, 1, EPD_CLAIM_BANDS_MAX, "", "GLASSINE_CLAIM_BANDS", EPD_KN_LIVE) \
    X(epd_claim_state,    -1, 0, "", EPD_KN_WORD, EPD_KN_KEEP, 0, "arrays|packed|check", "GLASSINE_CLAIM_STATE", EPD_KN_OPEN) /* (eink.conf @claim_state: the daemon sets the env) */ \
    X(epd_stagger_state,  -1, 0, "", EPD_KN_WORD, EPD_KN_KEEP, 0, "runs|lanes", "GLASSINE_STAGGER_STATE", EPD_KN_OPEN) /* (as @claim_state) */ \
    X(epd_claim_prefetch, -1, 0, "", EPD_KN_WORD, EPD_KN_KEEP, 0, "0|1", "GLASSINE_CLAIM_PREFETCH", EPD_KN_OPEN) \
    X(epd_claim_skip,     -1, 0, "", EPD_KN_WORD, EPD_KN_KEEP, 0, "0|1", "GLASSINE_CLAIM_SKIP", EPD_KN_OPEN) \
    X(epd_claim_tail16,   -1, 1, "", EPD_KN_WORD, EPD_KN_KEEP, 0, "0|1", "GLASSINE_CLAIM_TAIL16", EPD_KN_OPEN) \
    X(epd_finish_mask,    -1, 1, "", EPD_KN_WORD, EPD_KN_KEEP, 0, "0|1", "GLASSINE_FINISH_MASK", EPD_KN_OPEN) \
    X(epd_dmabuf_partial, -1, -1, "", EPD_KN_FN, 0, 0, "", "GLASSINE_DMABUF_PARTIAL", EPD_KN_OPEN) /* (-1 probed at the first sync; env 0: never) */ \
    X(epd_target_pool,     4, 4, "target_pool", EPD_KN_INT, 0, 8, "", "", EPD_KN_LIVE) \
    X(epd_tile_stat_on,    0, 0, "", EPD_KN_INT, 0, 1, "", "", EPD_KN_LIVE) /* (epd-drm.c drive_prof) */ \
    X(epd_sched_prof,      0, 0, "", EPD_KN_INT, 0, 1, "", "", EPD_KN_LIVE) /* (epd-drm.c drive_prof) */

/* epd-drm.c (and test/epd-stream-stub.c) */
#define EPD_KNOBS_DRM(X) \
    X(epd_stream_verbose,      0, 0, "", EPD_KN_INT, 0, INT_MAX, "", "", EPD_KN_LIVE) \
    X(epd_stream_gpu,          0, 0, "", EPD_KN_FN, 0, 0, "", "GLASSINE_EPDD_GPU", EPD_KN_OPEN) /* (gpu= reexecs; the daemon reads the env) */ \
    X(epd_stream_gpe,          0, 0, "", EPD_KN_FN, 0, 0, "", "GLASSINE_EPDD_GPE", EPD_KN_OPEN) /* (gpe=, as gpu=) */ \
    X(epd_stream_stagger,      0, 0, "stagger", EPD_KN_FN, 0, 0, "", "", EPD_KN_LIVE) /* (on, off, check: and stag_check) */ \
    X(epd_stream_stag_check,   0, 0, "", EPD_KN_INT, 0, 1, "", "", EPD_KN_LIVE) /* (stagger=check) */ \
    X(epd_stream_stag_direct, -1, -1, "stagger_direct", EPD_KN_NZ, 0, 0, "", "GLASSINE_STAG_DIRECT", EPD_KN_LIVE) /* (-1: epd-gpu.c reads the env) */ \
    X(epd_stream_stag_ctl,     3, 3, "stagger_ctl", EPD_KN_INT, 0, 4, "", "", EPD_KN_LIVE) \
    X(epd_stream_stag_k,       8, 8, "stagger_k", EPD_KN_INT, 0, 15, "", "", EPD_KN_LIVE) \
    X(epd_stream_stag_sidle,   0, 0, "stagger_idle", EPD_KN_WORD, 0, 0, "pixel|shared", "", EPD_KN_LIVE) \
    X(epd_stream_stag_pay,     0, 0, "stagger_pay", EPD_KN_FN, 0, 0, "", "", EPD_KN_LIVE) /* (N, or panel: stag_payrule) */ \
    X(epd_stream_stag_payrule, 0, 0, "", EPD_KN_INT, 0, 1, "", "", EPD_KN_LIVE) /* (stagger_pay=panel) */ \
    X(epd_stream_stag_cap,    -1, -1, "stagger_cap", EPD_KN_INT, -1, INT_MAX, "", "", EPD_KN_LIVE) \
    X(epd_stream_stag_stripe,  0, 0, "stagger_stripe", EPD_KN_FN, 0, 0, "", "", EPD_KN_LIVE) /* (and the four below) */ \
    X(epd_stream_stag_blocks,  0, 0, "", EPD_KN_INT, 0, 1, "", "", EPD_KN_LIVE) /* (stagger_stripe=blocks) */ \
    X(epd_stream_stag_cellw,   0, 0, "", EPD_KN_INT, 0, 64, "", "", EPD_KN_LIVE) /* (stagger_stripe=cells..., wbayer) */ \
    X(epd_stream_stag_cellh,   0, 0, "", EPD_KN_INT, 0, 64, "", "", EPD_KN_LIVE) \
    X(epd_stream_stag_cellord, 0, 0, "", EPD_KN_INT, 0, 1, "", "", EPD_KN_LIVE) /* (stagger_stripe=wbayer) */ \
    X(epd_stream_stag_htop,    2, 2, "hammer_top", EPD_KN_INT, 0, 7, "", "", EPD_KN_LIVE) \
    X(epd_stream_gpu_hang_test, 0, 0, "gpu_hang_test", EPD_KN_FN, 0, 0, "", "", EPD_KN_LIVE) /* (=1 only) */ \
    X(epd_stream_stag_a21,     0, 0, "stagger_a21", EPD_KN_WORD, EPD_KN_KEEP, 0, "off|on", "", EPD_KN_LIVE) \
    X(epd_stream_stag_linger, 250, 250, "stagger_linger", EPD_KN_INT, INT_MIN, INT_MAX, "", "", EPD_KN_LIVE) \
    X(epd_stream_stag_warm,    1, 1, "stagger_warm", EPD_KN_WORD, 1, 0, "off|on", "", EPD_KN_LIVE) \
    X(epd_stream_stag_entry,   1, 1, "stagger_entry", EPD_KN_WORD, 0, 0, "engine|first", "", EPD_KN_LIVE) \
    X(epd_stream_jit,          1, 1, "jit", EPD_KN_WORD, EPD_KN_KEEP, 0, "off|on", "", EPD_KN_LIVE) \
    X(epd_stream_jit_size,     1, 1, "jit_size", EPD_KN_WORD, EPD_KN_KEEP, 0, "off|on", "", EPD_KN_LIVE) \
    X(epd_stream_inbox,        1, 1, "handoff", EPD_KN_WORD, EPD_KN_KEEP, 0, "lock|inbox", "", EPD_KN_LIVE) \
    X(epd_stream_nwork,        4, 4, "nwork", EPD_KN_INT, INT_MIN, INT_MAX, "", "", EPD_KN_LIVE) /* (NREND; clamped 1..NREND where read) */ \
    X(epd_stream_rt_umin,     -1, -1, "rt_umin", EPD_KN_FN, 0, 0, "", "", EPD_KN_LIVE) /* (N or default; the open stream's threads) */ \
    X(epd_stream_link_log,     0, 0, "link_log", EPD_KN_WORD, EPD_KN_KEEP, 0, "off|on", "", EPD_KN_LIVE) \
    X(epd_stream_link_warm_ms, 30000, 30000, "link_warm", EPD_KN_INT, 0, INT_MAX, "", "", EPD_KN_LIVE) \
    X(epd_stream_qos,         -1, -1, "qos", EPD_KN_FN, 0, 0, "", "", EPD_KN_LIVE) /* (global, off, cpu:N; from the next link-up) */ \
    X(epd_stream_drive_overlap, 0, 0, "drive_overlap", EPD_KN_WORD, EPD_KN_KEEP, 0, "off|on", "GLASSINE_DRIVE_OVERLAP", EPD_KN_LIVE) /* (env: epd-drm.c at open) */ \
    X(epd_stream_upd_fifo,     0, 0, "", EPD_KN_INT, 0, 1, "", "", EPD_KN_LIVE) /* (the daemon: update_job + update_rt) */ \
    X(epd_stream_worker_rt,    0, 0, "worker_rt", EPD_KN_WORD, EPD_KN_KEEP, 0, "fifo|nice", "GLASSINE_WORKER_RT", EPD_KN_LIVE) /* (env: epd-drm.c at open, and the daemon) */

/* epd-gpu.c (-DEPD_GPU builds) */
#define EPD_KNOBS_GPU(X) \
    X(epd_gpu_drive_state, -1, 0, "drive_state", EPD_KN_WORD, EPD_KN_KEEP, 0, "tex|heap", "GLASSINE_DRIVE_STATE", EPD_KN_OPEN) /* (env: epd-gpu.c at open) */ \
    X(epd_gpu_stag_tskip,  -1, 1, "stagger_tskip", EPD_KN_WORD, EPD_KN_KEEP, 0, "off|on", "GLASSINE_STAG_TSKIP", EPD_KN_LIVE) /* (env: epd-gpu.c) */ \
    X(epd_gpu_stag_lead,   -1, 0, "stagger_lead", EPD_KN_WORD, EPD_KN_KEEP, 0, "off|on", "GLASSINE_STAG_LEAD", EPD_KN_OPEN) /* (env: epd-gpu.c at the stagger's open) */ \
    X(epd_gpu_stag_cremap, -1, 1, "", EPD_KN_INT, 0, 1, "", "GLASSINE_STAG_CREMAP", EPD_KN_OPEN) /* (env: epd-gpu.c) */ \
    X(epd_gpu_stag_timer,   0, 0, "stagger_timer", EPD_KN_NZ, 0, 0, "", "", EPD_KN_LIVE) \
    X(epd_gpu_prof_on,      0, 0, "", EPD_KN_INT, 0, 1, "", "", EPD_KN_LIVE) /* (tests: per-pass profile) */ \
    X(epd_gpu_gpe_debug,    0, 0, "", EPD_KN_INT, 0, 1, "", "", EPD_KN_LIVE) /* (tests: every gpe pass finished, GL errors) */ \
    X(epd_gpu_gpe_noatom,   0, 0, "", EPD_KN_INT, 0, 1, "", "", EPD_KN_LIVE) /* (a timing experiment: wrong results) */

/* epd-ingest.c */
#define EPD_KNOBS_INGEST(X) \
    X(epd_ingest_kernel,  -1, 0, "ingest_kernel", EPD_KN_WORD, 0, 0, "row|tile|scalar", "GLASSINE_INGEST_KERNEL", EPD_KN_LIVE) /* (env: epd-ingest.c, first frame) */ \
    X(epd_ingest_bands,   -1, 1, "ingest_bands", EPD_KN_INT, 1, 4, "", "GLASSINE_INGEST_BANDS", EPD_KN_LIVE) /* (env: as kernel) */ \
    X(epd_ingest_sync,    -2, -1, "ingest_sync", EPD_KN_FN, 0, 0, "", "GLASSINE_INGEST_SYNC", EPD_KN_LIVE) /* (off, on, N: epd_ingest_sync_of; env: as kernel) */ \
    X(epd_ingest_partial, -1, 1, "ingest_partial", EPD_KN_WORD, 1, 0, "off|on", "GLASSINE_INGEST_PARTIAL", EPD_KN_LIVE) /* (env: as kernel) */

#define EPD_KNOB_DECL(var, ...) extern int var;
EPD_KNOBS_ENGINE(EPD_KNOB_DECL)
EPD_KNOBS_DRM(EPD_KNOB_DECL)
EPD_KNOBS_GPU(EPD_KNOB_DECL)
EPD_KNOBS_INGEST(EPD_KNOB_DECL)
#undef EPD_KNOB_DECL
#define EPD_KNOB_DEFINE(var, init, ...) int var = init;

typedef struct epd_knob {
    int *p;
    const char *var, *name;
    int init, def, kind, lo, hi;
    const char *words, *env;
    int when;
} epd_knob;
#define EPD_KNOB_ROW(var, init, def, name, kind, lo, hi, words, env, when) \
    { &var, #var, name, init, def, kind, lo, hi, words, env, when },
#define EPD_KNOBS_N(t) ((int)(sizeof (t) / sizeof *(t)))

/* the index of the word of k's list that v starts with (exact: is), -1 none */
static inline int epd_knob_word(const epd_knob *k, const char *v, int exact)
{
    const char *w = k->words;
    for (int i = 0; *w; i++) {
        size_t n = strcspn(w, "|");
        if (n && !strncmp(v, w, n) && (!exact || !v[n])) return i;
        w += n + (w[n] == '|');
    }
    return -1;
}

/* k's control value at v (the text after "name="): 1 and *out, or 0 = not
 * a value of k (left as it is). EPD_KN_FN: 0 (the caller's) */
static inline int epd_knob_value(const epd_knob *k, const char *v, int *out)
{
    long n;
    int i;
    switch (k->kind) {
    case EPD_KN_INT:
        n = atoi(v);
        *out = n < k->lo ? k->lo : n > k->hi ? k->hi : (int)n;
        return 1;
    case EPD_KN_NZ:
        *out = atoi(v) != 0;
        return 1;
    case EPD_KN_WORD:
        if ((i = epd_knob_word(k, v, 0)) >= 0) { *out = i; return 1; }
        if (k->lo == EPD_KN_KEEP) return 0;
        *out = k->lo;
        return 1;
    }
    return 0;
}

/* the value k's env variable gives (unset or not understood: def) */
static inline int epd_knob_env(const epd_knob *k)
{
    const char *v = *k->env ? getenv(k->env) : NULL;
    int i;
    if (!v) return k->def;
    switch (k->kind) {
    case EPD_KN_INT: {
        long n = atoi(v);
        return n < k->lo ? k->lo : n > k->hi ? k->hi : (int)n;
    }
    case EPD_KN_NZ: return atoi(v) != 0;
    case EPD_KN_WORD: return (i = epd_knob_word(k, v, 1)) >= 0 ? i : k->def;
    }
    return k->def;
}

/* every row of t with an env variable and not set yet (< 0, EPD_KN_FN
 * rows aside: their owner's): from the env (once; epd_init) */
static inline void epd_knobs_env(const epd_knob *t, int n)
{
    for (int i = 0; i < n; i++)
        if (*t[i].env && t[i].kind != EPD_KN_FN && __atomic_load_n(t[i].p, __ATOMIC_RELAXED) < 0)
            __atomic_store_n(t[i].p, epd_knob_env(&t[i]), __ATOMIC_RELAXED);
}

/* a control line: for every named row of t (only: that name alone, NULL =
 * all) whose "name=" is in cmd, its value stored (atomically: the stream
 * thread and the workers read them per frame); EPD_KN_FN rows go to fn
 * (NULL: skipped). The number of rows that matched */
static inline int epd_knobs_apply(const epd_knob *t, int n, const char *cmd, const char *only,
                                  void (*fn)(const epd_knob *k, const char *v, void *ctx), void *ctx)
{
    int hit = 0;
    for (int i = 0; i < n; i++) {
        const epd_knob *k = &t[i];
        char key[40];
        if (!*k->name || (only && strcmp(only, k->name))) continue;
        snprintf(key, sizeof key, "%s=", k->name);
        const char *v = strstr(cmd, key);
        if (!v) continue;
        v += strlen(key);
        hit++;
        int x;
        if (k->kind == EPD_KN_FN) { if (fn) fn(k, v, ctx); }
        else if (epd_knob_value(k, v, &x)) __atomic_store_n(k->p, x, __ATOMIC_RELAXED);
    }
    return hit;
}

/* the named rows as "name=value ..." into b (words as words) */
static inline void epd_knobs_line(const epd_knob *t, int n, char *b, size_t bn)
{
    size_t o = 0;
    if (bn) b[0] = 0;
    for (int i = 0; i < n && o < bn; i++) {
        const epd_knob *k = &t[i];
        if (!*k->name) continue;
        int x = __atomic_load_n(k->p, __ATOMIC_RELAXED);
        const char *w = k->kind == EPD_KN_WORD ? k->words : "";
        int j = 0;
        while (*w && j < x) { w += strcspn(w, "|"); w += *w == '|'; j++; }
        size_t wl = strcspn(w, "|");
        if (*w && x >= 0 && wl) o += (size_t)snprintf(b + o, bn - o, "%s%s=%.*s", o ? " " : "", k->name, (int)wl, w);
        else o += (size_t)snprintf(b + o, bn - o, "%s%s=%d", o ? " " : "", k->name, x);
    }
}

#endif
