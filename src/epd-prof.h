/* epd-prof.h - opt-in per-record timing tables (default off; ):
 * the drive thread's frame split (epd-drm.c, drive_prof=on) and the main
 * thread's update split (glassine-epdd-desktop.c, main_prof=on). A record is one loop
 * iteration (drive) or one burst (main): up to EPD_PROF_F fields (ms, or a
 * count where the field's name says so) under a record type. Per type and
 * field: sum and max exact, p90 over the newest EPD_PROF_RING records of the
 * period (a sorted copy at print time; at 85 Hz a 5 s period is 425
 * records). Allocate it (calloc: ~600 KB), not on a stack; nothing in here allocates. The caller times with CLOCK_MONOTONIC and owns the
 * table (one thread writes it, the same thread prints it).
 *
 * The summary, one line per type with records ("who" = drive|main):
 * epd-prof <who> <type>: <n> in <s> s [<extra>]; <field> <avg>/<p50>/<p90>/<max> ...
 * fields that were 0 in every record of the period are left out. */
#ifndef EPD_PROF_H
#define EPD_PROF_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define EPD_PROF_T 10
#define EPD_PROF_F 28
#define EPD_PROF_RING 512

typedef struct {
    int nt, nf;
    const char *const *tname;           /* [nt] */
    const char *const *fname;           /* [nf] */
    unsigned long n[EPD_PROF_T];        /* records this period */
    unsigned long total[EPD_PROF_T];    /* since enabled */
    double sum[EPD_PROF_T][EPD_PROF_F], max[EPD_PROF_T][EPD_PROF_F];
    float ring[EPD_PROF_T][EPD_PROF_RING][EPD_PROF_F];
    double t0;                          /* s: the period's start */
} epd_prof;

static inline double epd_prof_now(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec / 1e9;
}

/* a new period */
static inline void epd_prof_reset(epd_prof *p)
{
    for (int t = 0; t < p->nt; t++) {
        p->n[t] = 0;
        for (int k = 0; k < p->nf; k++) { p->sum[t][k] = 0; p->max[t][k] = -1e30; }
    }
    p->t0 = epd_prof_now();
}

static inline void epd_prof_init(epd_prof *p, int nt, const char *const *tname, int nf, const char *const *fname)
{
    memset(p, 0, sizeof *p);
    p->nt = nt < EPD_PROF_T ? nt : EPD_PROF_T;
    p->nf = nf < EPD_PROF_F ? nf : EPD_PROF_F;
    p->tname = tname;
    p->fname = fname;
    epd_prof_reset(p);
}

/* one record of type t: v[nf] */
static inline void epd_prof_add(epd_prof *p, int t, const float *v)
{
    if (t < 0 || t >= p->nt) return;
    unsigned long k = p->n[t]++ % EPD_PROF_RING;
    p->total[t]++;
    for (int f = 0; f < p->nf; f++) {
        p->sum[t][f] += v[f];
        if (v[f] > p->max[t][f]) p->max[t][f] = v[f];
        p->ring[t][k][f] = v[f];
    }
}

static inline int epd_prof_cmpf(const void *a, const void *b)
{
    float x = *(const float *)a, y = *(const float *)b;
    return x < y ? -1 : x > y;
}

/* the q-quantile (0..1) of field f of type t over the ring (nearest rank) */
static inline double epd_prof_quantile(const epd_prof *p, int t, int f, double q)
{
    static float tmp[EPD_PROF_RING];
    unsigned long n = p->n[t] < EPD_PROF_RING ? p->n[t] : EPD_PROF_RING;
    if (!n) return 0;
    for (unsigned long i = 0; i < n; i++) tmp[i] = p->ring[t][i][f];
    qsort(tmp, n, sizeof tmp[0], epd_prof_cmpf);
    unsigned long r = (unsigned long)(q * n + 0.999999);       /* ceil(q n), 1-based */
    if (r < 1) r = 1;
    if (r > n) r = n;
    return tmp[r - 1];
}

/* the summary into f (one line per type with records, extra[t]: NULL or a
 * per-type note), then a new period */
static inline void epd_prof_print(epd_prof *p, FILE *f, const char *who, const char *const *extra)
{
    double now = epd_prof_now(), secs = now - p->t0;
    for (int t = 0; t < p->nt; t++) {
        if (!p->n[t]) continue;
        char line[2048];
        int o = snprintf(line, sizeof line, "epd-prof %s %s: %lu in %.1f s%s%s;", who, p->tname[t], p->n[t], secs,
                         extra && extra[t] ? " " : "", extra && extra[t] ? extra[t] : "");
        for (int k = 0; k < p->nf && o < (int)sizeof line - 64; k++) {
            if (p->max[t][k] == 0 && p->sum[t][k] == 0) continue;     /* (all zero: left out) */
            o += snprintf(line + o, sizeof line - o, " %s %.2f/%.2f/%.2f/%.2f", p->fname[k], p->sum[t][k] / p->n[t],
                          epd_prof_quantile(p, t, k, 0.5), epd_prof_quantile(p, t, k, 0.9), p->max[t][k]);
        }
        fprintf(f, "%s\n", line);
    }
    epd_prof_reset(p);
}

/* the CSV header: t_s,type,<extra columns>,<fields> */
static inline void epd_prof_csv_head(const epd_prof *p, FILE *f, const char *extra_cols)
{
    fprintf(f, "t_s,type%s%s", extra_cols && *extra_cols ? "," : "", extra_cols ? extra_cols : "");
    for (int k = 0; k < p->nf; k++) fprintf(f, ",%s", p->fname[k]);
    fprintf(f, "\n");
}

static inline void epd_prof_csv_row(const epd_prof *p, FILE *f, double t, int type, const char *extra_vals, const float *v)
{
    fprintf(f, "%.6f,%s%s%s", t, p->tname[type], extra_vals && *extra_vals ? "," : "", extra_vals ? extra_vals : "");
    for (int k = 0; k < p->nf; k++) fprintf(f, ",%.3f", v[k]);
    fprintf(f, "\n");
}

/* the CPU's current frequency in kHz (scaling_cur_freq), -1 unknown */
static inline long epd_prof_cpu_khz(int cpu)
{
    char path[96], b[32];
    snprintf(path, sizeof path, "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq", cpu);
    FILE *f = fopen(path, "re");
    if (!f) return -1;
    long v = fgets(b, sizeof b, f) ? atol(b) : -1;
    fclose(f);
    return v;
}

#endif
