/* epd-stream-stub — epd-drm.c's API without a panel, for host runs of the
 * whole glassine-epdd-desktop (the sway host test, ../../sway/host-test.sh EPDD=1): every
 * submitted update is logged ("SUBMIT id mode x,y wxh full") and copied into
 * a panel image; $STUB_OUT/panel.pgm is rewritten after each (1872x1404
 * panel order, as the "shot" FIFO command writes it). Updates finish at once. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "../epd-drm.h"

#define PW 1872
#define PH 1404
struct epd_stream { uint32_t id; uint8_t *img; epd_stats st; };
int epd_stream_stag_check;
int epd_stream_stag_direct = -1;
int epd_stream_stagger, epd_stream_stag_ctl, epd_stream_stag_pay, epd_stream_stag_linger = 250;
int epd_stream_stag_payrule, epd_stream_stag_cap = -1;
int epd_stream_stag_stripe;
int epd_stream_stag_blocks;
int epd_stream_stag_cellw, epd_stream_stag_cellh;
int epd_stream_stag_htop;
int epd_stream_stag_a21;
int epd_stream_stag_k = 8;
int epd_stream_verbose, epd_stream_gpu, epd_stream_gpe, epd_stream_jit = 1, epd_stream_nwork = 4, epd_stream_rt_umin = -1, epd_stream_qos = -1;

static long long ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1000LL + t.tv_nsec / 1000000;
}

epd_stream *epd_stream_open(const char *wbf_path, int celsius, int hv)
{
    epd_stream *s = calloc(1, sizeof *s);
    s->img = malloc((size_t)PW * PH);
    memset(s->img, 0xff, (size_t)PW * PH);
    fprintf(stderr, "stub: engine open (%s, %d C, hv %d)\n", wbf_path, celsius, hv);
    return s;
}

uint32_t epd_stream_submit(epd_stream *s, epd_rect r, int mode, int full, const uint8_t *grey, int stride)
{
    for (int y = r.y; y < r.y + r.h; y++) memcpy(s->img + (size_t)y * PW + r.x, grey + (size_t)y * stride + r.x, r.w);
    s->st.updates++;
    fprintf(stderr, "%6lld SUBMIT %u mode %d %d,%d %dx%d%s\n", ms() % 1000000, ++s->id, mode, r.x, r.y, r.w, r.h, full ? " full" : "");
    static FILE *dump;                  /* STUB_DUMP=FILE: the panel image after each update, raw (sway/band-dots.py) */
    if (!dump && getenv("STUB_DUMP")) dump = fopen(getenv("STUB_DUMP"), "w");
    if (dump) { fwrite(s->img, 1, (size_t)PW * PH, dump); fflush(dump); fprintf(stderr, "DUMP %u mode %d\n", s->id, mode); }
    const char *d = getenv("STUB_OUT");
    if (d) {
        char p[512], q[520];
        snprintf(p, sizeof p, "%s/panel.pgm.tmp", d);
        snprintf(q, sizeof q, "%s/panel.pgm", d);
        FILE *f = fopen(p, "w");
        if (f) {
            fprintf(f, "P5 %d %d 255\n", PW, PH);
            fwrite(s->img, 1, (size_t)PW * PH, f);
            fclose(f);
            rename(p, q);
        }
    }
    return s->id;
}

void epd_stream_cpu(epd_stream *s, double *drive_s, double *workers_s) { (void)s; *drive_s = *workers_s = 0; }
void epd_stream_dc(epd_stream *s, int limit, int *max_abs, long *over) { (void)s; (void)limit; *max_abs = 0; *over = 0; }
void epd_stream_dc_map(epd_stream *s, int16_t *dc, uint8_t *level, uint8_t *op, int *r) { (void)s; (void)dc; (void)level; (void)op; (void)r; }
void epd_stream_set_matched(epd_stream *s, int on) { (void)s; (void)on; }
void epd_stream_set_matched_rule(epd_stream *s, int from, int cap) { (void)s; (void)from; (void)cap; }
void epd_stream_set_nonblock(epd_stream *s, int on) { (void)s; (void)on; }
unsigned long long epd_stream_split_starts(epd_stream *s) { (void)s; return 0; }
unsigned long long epd_stream_absorbed(epd_stream *s) { (void)s; return 0; }
unsigned long long epd_stream_preempted(epd_stream *s) { (void)s; return 0; }
unsigned long long epd_stream_flash_skipped(epd_stream *s) { (void)s; return 0; }
void epd_stream_set_model(epd_stream *s, int optical, int track) { (void)s; (void)optical; (void)track; }
void epd_stream_set_heal(epd_stream *s, int min, int from) { (void)s; (void)min; (void)from; }
void epd_stream_set_heal_len(epd_stream *s, int n) { (void)s; (void)n; }
unsigned long long epd_stream_healed(epd_stream *s) { (void)s; return 0; }
void epd_stream_set_snap(epd_stream *s, int on) { (void)s; (void)on; }
unsigned long long epd_stream_snapped(epd_stream *s) { (void)s; return 0; }
void epd_stream_shortfall(epd_stream *s, int limit, double *mean, long *over) { (void)s; (void)limit; *mean = 0; *over = 0; }
unsigned long long epd_stream_matched_px(epd_stream *s) { (void)s; return 0; }
int epd_stream_free_ms(epd_stream *s, epd_rect r) { (void)s; (void)r; return 0; }   /* updates finish at once */
void epd_stream_last_submit(epd_stream *s, double *m, double *l, double *t) { (void)s; *m = *l = *t = 0; }
void epd_stream_assume(epd_stream *s, int level5) { (void)s; (void)level5; }
void epd_stream_set_temp(epd_stream *s, int celsius) { (void)s; (void)celsius; }
void epd_stream_set_cut(epd_stream *s, int cut) { (void)s; (void)cut; }
void epd_stream_keepalive(epd_stream *s, int ms_) { (void)s; (void)ms_; }
void epd_stream_wait_idle(epd_stream *s) { (void)s; }
int epd_stream_save_levels(epd_stream *s, const char *path) { (void)s; (void)path; return -1; }
int epd_stream_load_levels(epd_stream *s, const char *path) { (void)s; (void)path; return -1; }
void epd_stream_stats(epd_stream *s, epd_stats *out) { *out = s->st; }
void epd_stream_stats_reset(epd_stream *s) { memset(&s->st, 0, sizeof s->st); }
void epd_stream_close(epd_stream *s) { if (s) { free(s->img); free(s); } }
void epd_stream_set_hv(epd_stream *s, int hv) { (void)s; (void)hv; }
int epd_stream_levels_lost(epd_stream *s) { (void)s; return 0; }
int epd_stream_gpu_dead(epd_stream *s) { (void)s; return 0; }
void epd_stream_set_rt_umin(epd_stream *s, int v) { (void)s; (void)v; }
void epd_stream_set_gt(epd_stream *s, const struct epd_gt_provider *pv) { (void)s; (void)pv; }
uint32_t epd_stream_submit_gt(epd_stream *s, epd_rect r, int mode, int full, int gt, const uint8_t *grey, int stride)
{ (void)s; (void)r; (void)mode; (void)full; (void)gt; (void)grey; (void)stride; return 0; }
/* the stall fixes / GPU revive knobs (15): none of them do anything here */
int epd_stream_gpu_hang_test, epd_stream_jit_size = 1, epd_stream_link_log, epd_stream_link_warm_ms, epd_stream_inbox = 1;
void epd_stream_park(epd_stream *s, int on) { (void)s; (void)on; }
/* drive_prof and the stagger cells : nothing to profile or shape here */
void epd_stream_prof(epd_stream *s, int on, int every_s, const char *csv) { (void)s; (void)on; (void)every_s; (void)csv; }
void epd_stream_prof_dump(epd_stream *s, const char *path) { (void)s; (void)path; }
void epd_stream_set_origin(const double *org) { (void)org; }
int epd_stag_cells_parse(const char *s, int *w, int *h) { (void)s; (void)w; (void)h; return -1; }
/* (the update job, / switches and the worker policy, as no-ops) */
int epd_stream_stag_sidle, epd_stream_stag_cellord, epd_stream_drive_overlap, epd_stream_upd_fifo, epd_stream_worker_rt;
void epd_stream_s4(epd_stream *s, char *b, size_t n) { (void)s; if (n) b[0] = 0; }
void epd_stream_upd_run(epd_stream *s, void (*fn)(void *, int), void *arg, int n) { (void)s; for (int k = 0; k < n; k++) fn(arg, k); }
void epd_stream_set_upd_runner(epd_stream *s, epd_upd_runner run, void *ctx) { (void)s; (void)run; (void)ctx; }
void epd_stream_upd_stats(epd_stream *s, unsigned long long *jobs, unsigned long long *wparts,
                          unsigned long long *cparts, unsigned long long *serial)
{
    (void)s;
    *jobs = *wparts = *cparts = *serial = 0;
}
