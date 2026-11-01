/* epd-upd.h — : the update job's own helper pool.
 *
 * glassine-epdd-desktop's update job (scan, commit, the levels of a burst) runs as parts
 * fn(arg, k), k < n, taken one at a time from a counter by the caller and by
 * the helpers; back when every part ran. Two places it can run:
 * - the stream's frame workers (epd_stream_upd_run, epd-drm.c): Gold cores,
 * SCHED_FIFO 50, shared with the claim bands (a worker serves band and
 * render parts first, between update parts);
 * - this pool: UPD_HELPERS threads of their own, pinned to a CPU list
 * (update_cpus=, none = unpinned) at SCHED_FIFO UPD_FIFO (39) or SCHED_OTHER
 * nice UPD_NICE (update_rt=). Live: a helper re-applies the pinning and the
 * policy at its next job (cfg moved). UPD_FIFO is below the pen thread
 * (FIFO 40, unpinned) and the stream's threads (50/51): the pool runs on
 * Silver or any LIST, the pen thread's cores too, so a FIFO helper never
 * delays ink or a band part (the pen guard). A FIFO helper
 * still yields between parts (same-priority helpers on one CPU).
 * One job at a time: a second caller (or n <= 1) runs every part itself. The
 * parts write disjoint memory, so the bytes never depend on who ran what.
 * Header-only (static inline) so the tests take the same code. */
#ifndef EPD_UPD_H
#define EPD_UPD_H
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

#define UPD_HELPERS 4
#define UPD_NICE (-10)
#define UPD_FIFO 39                     /* below the pen thread (FIFO 40) */
#define UPD_ACCT_N 4

typedef struct upd_pool {
    int nth;
    pthread_t th[UPD_HELPERS];
    pthread_mutex_t mu;                 /* the wake (go) and the caller's wait (done) */
    pthread_cond_t go, done;
    int quit;
    /* the job: fn/arg set before on (release); the rest atomic */
    void (*fn)(void *, int);
    void *arg;
    int n, next, left, help, on, busy;
    unsigned jgen;                      /* under mu: a new job (the helpers' wake) */
    /* the placement: cpus (pin: any set), rt 0 FIFO UPD_FIFO / 1 nice; cfg moves on a change (under mu) */
    cpu_set_t cpus;
    int pin, rt;
    unsigned cfg;
    unsigned long long jobs, hparts, cparts, serial;    /* (atomic) */
    /* step 3: per job function, its parts' thread CPU time and
     * the caller's wall time of the job (upd_acct on; the stats print it) */
    struct upd_acct { void (*fn)(void *, int); unsigned long long jobs, parts; double cpu_ms, wall_ms; } acct[UPD_ACCT_N];
} upd_pool;

static int upd_acct;                    /* (atomic) 1: account (two clock reads a part) */
static inline double upd_ms(clockid_t c)
{
    struct timespec t;
    clock_gettime(c, &t);
    return t.tv_sec * 1e3 + t.tv_nsec / 1e6;
}
/* fn's slot (first come; the last slot takes the rest) */
static inline struct upd_acct *upd_slot(upd_pool *P, void (*fn)(void *, int))
{
    for (int i = 0; i < UPD_ACCT_N - 1; i++) {
        void (*f)(void *, int) = __atomic_load_n(&P->acct[i].fn, __ATOMIC_ACQUIRE);
        if (f == fn) return &P->acct[i];
        if (!f) {
            void (*z)(void *, int) = NULL;
            if (__atomic_compare_exchange_n(&P->acct[i].fn, &z, fn, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE) || z == fn) return &P->acct[i];
        }
    }
    return &P->acct[UPD_ACCT_N - 1];
}
static inline void upd_add(double *d, double v)
{
    double o, n;
    __atomic_load(d, &o, __ATOMIC_RELAXED);
    do n = o + v; while (!__atomic_compare_exchange(d, &o, &n, 1, __ATOMIC_RELAXED, __ATOMIC_RELAXED));
}
/* fn(arg, k), accounted when on */
static inline void upd_call(upd_pool *P, void (*fn)(void *, int), void *arg, int k, int acct)
{
    if (!acct) { fn(arg, k); return; }
    double t0 = upd_ms(CLOCK_THREAD_CPUTIME_ID);
    fn(arg, k);
    struct upd_acct *a = upd_slot(P, fn);
    upd_add(&a->cpu_ms, upd_ms(CLOCK_THREAD_CPUTIME_ID) - t0);
    __atomic_add_fetch(&a->parts, 1, __ATOMIC_RELAXED);
}
/* the caller's side of a job: its wall time */
static inline void upd_job_done(upd_pool *P, void (*fn)(void *, int), double w0)
{
    struct upd_acct *a = upd_slot(P, fn);
    __atomic_add_fetch(&a->jobs, 1, __ATOMIC_RELAXED);
    upd_add(&a->wall_ms, upd_ms(CLOCK_MONOTONIC) - w0);
}

static inline void upd_place(upd_pool *P, cpu_set_t *cpus, int pin, int rt)
{
    if (pin) pthread_setaffinity_np(pthread_self(), sizeof *cpus, cpus);
    else {
        cpu_set_t all;
        CPU_ZERO(&all);
        for (int c = 0; c < CPU_SETSIZE && c < 64; c++) CPU_SET(c, &all);
        pthread_setaffinity_np(pthread_self(), sizeof all, &all);
    }
    if (!rt) {
        struct sched_param sp = { .sched_priority = UPD_FIFO };
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    } else {
        struct sched_param sp = { .sched_priority = 0 };
        pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp);
        setpriority(PRIO_PROCESS, (id_t)syscall(SYS_gettid), UPD_NICE);
    }
    (void)P;
}

static inline void upd_note(upd_pool *P)
{
    pthread_mutex_lock(&P->mu);
    pthread_cond_broadcast(&P->done);
    pthread_mutex_unlock(&P->mu);
}

static inline void *upd_helper(void *arg)
{
    upd_pool *P = arg;
    pthread_mutex_lock(&P->mu);
    unsigned seen = P->jgen, cfg = P->cfg - 1;  /* (applied at the first job) */
    for (;;) {
        while (P->jgen == seen && !P->quit) pthread_cond_wait(&P->go, &P->mu);
        if (P->quit) break;
        seen = P->jgen;
        if (cfg != P->cfg) {
            cfg = P->cfg;
            cpu_set_t c = P->cpus;
            int pin = P->pin, rt = P->rt;
            pthread_mutex_unlock(&P->mu);
            upd_place(P, &c, pin, rt);
            pthread_mutex_lock(&P->mu);
        }
        int rt = P->rt;
        pthread_mutex_unlock(&P->mu);
        __atomic_add_fetch(&P->help, 1, __ATOMIC_SEQ_CST);
        if (__atomic_load_n(&P->on, __ATOMIC_SEQ_CST)) {
            void (*fn)(void *, int) = P->fn;
            void *a = P->arg;
            int n = __atomic_load_n(&P->n, __ATOMIC_RELAXED), ac = __atomic_load_n(&upd_acct, __ATOMIC_RELAXED);
            for (;;) {
                int k = __atomic_fetch_add(&P->next, 1, __ATOMIC_RELAXED);
                if (k >= n) break;
                upd_call(P, fn, a, k, ac);
                __atomic_add_fetch(&P->hparts, 1, __ATOMIC_RELAXED);
                if (__atomic_sub_fetch(&P->left, 1, __ATOMIC_ACQ_REL) == 0) upd_note(P);
                if (!rt) sched_yield();         /* (a woken band worker of the same priority first) */
            }
        }
        if (__atomic_sub_fetch(&P->help, 1, __ATOMIC_SEQ_CST) == 0) upd_note(P);
        pthread_mutex_lock(&P->mu);
    }
    pthread_mutex_unlock(&P->mu);
    return NULL;
}

/* the placement (live: at each helper's next job); list: "1-4,6" style, or NULL / "" for none */
static inline void upd_pool_config(upd_pool *P, const cpu_set_t *cpus, int pin, int rt)
{
    if (!P->nth && !P->quit) {
        pthread_mutex_init(&P->mu, NULL);
        pthread_cond_init(&P->go, NULL);
        pthread_cond_init(&P->done, NULL);
        P->nth = -1;                    /* (initialized, no helpers yet) */
    }
    pthread_mutex_lock(&P->mu);
    if (cpus) P->cpus = *cpus; else CPU_ZERO(&P->cpus);
    P->pin = pin && cpus;
    P->rt = rt;
    P->cfg++;
    pthread_mutex_unlock(&P->mu);
}

static inline void upd_pool_run(upd_pool *P, void (*fn)(void *, int), void *arg, int n)
{
    const int ac = __atomic_load_n(&upd_acct, __ATOMIC_RELAXED);
    const double w0 = ac ? upd_ms(CLOCK_MONOTONIC) : 0;
    if (n <= 1 || !P->nth || __atomic_exchange_n(&P->busy, 1, __ATOMIC_ACQUIRE)) {
        if (n > 1) __atomic_add_fetch(&P->serial, 1, __ATOMIC_RELAXED);
        for (int k = 0; k < n; k++) upd_call(P, fn, arg, k, ac);
        if (ac) upd_job_done(P, fn, w0);
        return;
    }
    if (P->nth < 0) {                   /* (the helpers, at the first job) */
        P->nth = 0;
        for (int i = 0; i < UPD_HELPERS; i++)
            if (!pthread_create(&P->th[P->nth], NULL, upd_helper, P)) P->nth++;
        if (!P->nth) P->nth = -1;
    }
    P->fn = fn; P->arg = arg;
    __atomic_store_n(&P->n, n, __ATOMIC_RELAXED);
    __atomic_store_n(&P->next, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&P->left, n, __ATOMIC_RELAXED);
    __atomic_store_n(&P->on, 1, __ATOMIC_SEQ_CST);
    __atomic_add_fetch(&P->jobs, 1, __ATOMIC_RELAXED);
    pthread_mutex_lock(&P->mu);
    P->jgen++;
    pthread_cond_broadcast(&P->go);
    pthread_mutex_unlock(&P->mu);
    for (;;) {
        int k = __atomic_fetch_add(&P->next, 1, __ATOMIC_RELAXED);
        if (k >= n) break;
        upd_call(P, fn, arg, k, ac);
        __atomic_add_fetch(&P->cparts, 1, __ATOMIC_RELAXED);
        __atomic_sub_fetch(&P->left, 1, __ATOMIC_ACQ_REL);
    }
    pthread_mutex_lock(&P->mu);
    while (__atomic_load_n(&P->left, __ATOMIC_ACQUIRE)) pthread_cond_wait(&P->done, &P->mu);
    __atomic_store_n(&P->on, 0, __ATOMIC_SEQ_CST);
    while (__atomic_load_n(&P->help, __ATOMIC_SEQ_CST)) pthread_cond_wait(&P->done, &P->mu);
    pthread_mutex_unlock(&P->mu);
    __atomic_store_n(&P->busy, 0, __ATOMIC_RELEASE);
    if (ac) upd_job_done(P, fn, w0);
}

/* the runner shape (epd_upd_runner) */
static inline void upd_pool_runner(void *ctx, void (*fn)(void *, int), void *arg, int n) { upd_pool_run(ctx, fn, arg, n); }

static inline void upd_pool_stop(upd_pool *P)
{
    if (P->nth <= 0) return;
    pthread_mutex_lock(&P->mu);
    P->quit = 1;
    pthread_cond_broadcast(&P->go);
    pthread_mutex_unlock(&P->mu);
    for (int i = 0; i < P->nth; i++) pthread_join(P->th[i], NULL);
    P->nth = 0;
}
#endif
