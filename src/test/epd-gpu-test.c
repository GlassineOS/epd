/* epd-gpu-test — the GPU drive frames (epd-gpu.c) against the CPU engine
 * (epd_render), word for word, and how long the GPU takes per frame. Runs on
 * the device (Adreno, surfaceless EGL); touches neither the panel nor X.
 *
 * epd-gpu-test <eink_waveform.wbf> [seed] [dumb]
 *
 * dumb: render into three DRM dumb buffers (the scanout buffers' kind, from
 * /dev/dri/card0, imported as dma-bufs, used in turn like epd-drm's) and
 * compare what the link would read there (their CPU mapping, at their pitch);
 * no mode set, no flip.
 *
 * Scenarios: a full-screen GC16 flash (the heaviest frame), random updates of
 * every mode (partial/full, DU with frame cuts, GL16, A2-or-DU, overlapping
 * and waiting, many running at once), a pen stroke (small A2-or-DU pieces,
 * several per frame). Exit 0 = every frame identical. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <libdrm/drm.h>
#include <libdrm/drm_mode.h>
#include "../epd-engine.h"
#include "../epd-gpu.h"

static epd_engine E;
static uint32_t cpu[EPD_WORDS], gpu[EPD_WORDS];
static uint8_t grey[EPD_W * EPD_H];
static epd_gpu *G;
static long frames, bad;
static double t_render, t_render_max, t_state, t_read;
static int ndumb;                       /* dumb targets 1..ndumb, used in turn */
static uint8_t *dmap[3];
static uint32_t dpitch[3];
static size_t dsize[3];

static int dumb_open(void)
{
    int fd = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
    if (fd < 0) { perror("/dev/dri/card0"); return -1; }
    for (int i = 0; i < 3; i++) {
        /* the scanout buffers' own pitch, EPD_GPU_EXTRA_LINES more lines for the
         * GPU's flat view (epd-gpu.h); the link reads EPD_LINE words of each row */
        struct drm_mode_create_dumb c = { .width = EPD_LINE, .height = EPD_LINES + EPD_GPU_EXTRA_LINES, .bpp = 32 };
        if (ioctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &c)) { perror("CREATE_DUMB"); return -1; }
        struct drm_mode_map_dumb m = { .handle = c.handle };
        if (ioctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &m)) { perror("MAP_DUMB"); return -1; }
        dmap[i] = mmap(0, c.size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, m.offset);
        if (dmap[i] == MAP_FAILED) { perror("mmap"); return -1; }
        memset(dmap[i], 0xa5, c.size);  /* garbage: the first frame must write it all */
        dpitch[i] = c.pitch;
        dsize[i] = c.size;
        struct drm_prime_handle p = { .handle = c.handle, .flags = DRM_CLOEXEC | DRM_RDWR };
        if (ioctl(fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &p)) { perror("PRIME_HANDLE_TO_FD"); return -1; }
        if (epd_gpu_target_dmabuf(G, p.fd, (int)c.pitch, (size_t)c.size) != i + 1) return -1;
    }
    printf("dumb targets: pitch %u bytes\n", dpitch[0]);
    ndumb = 3;
    return 0;
}

static double now(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec / 1e9;
}

static uint32_t rng = 1;
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }
static int rr(int a, int b) { return a + (int)(rnd() % (uint32_t)(b - a + 1)); }

static void state(epd_rect r)
{
    double t0 = now();
    epd_gpu_state(G, &E, r);
    t_state += now() - t0;
}

/* one drive frame on both; 0 when nothing runs */
static int step(const char *what)
{
    uint32_t prev[EPD_MAX_UPD];
    for (int s = 0; s < EPD_MAX_UPD; s++) prev[s] = E.run[s].nframes ? E.run[s].id : 0;
    static int dirty[2] = { EPD_LINES, 0 };
    static int init;
    if (!init) { memcpy(cpu, E.tpl, sizeof cpu); init = 1; }
    if (!epd_schedule(&E)) return 0;
    epd_rect st[2 * EPD_MAX_UPD];               /* claimed: busy/target changed there (as epd-drm) */
    int nst = epd_gpu_starts(G, &E, prev, st);
    for (int k = 0; k < nst; k++) state(st[k]);
    epd_render(&E, cpu, dirty);
    int tg = ndumb ? 1 + (int)(frames % ndumb) : 0;
    double t0 = now();
    epd_gpu_render(G, &E, tg);
    if (epd_gpu_finish(G, 2000)) { fprintf(stderr, "GPU hung\n"); exit(3); }
    double dt = now() - t0;
    t_render += dt;
    if (dt > t_render_max) t_render_max = dt;
    t0 = now();
    if (tg)                                     /* what the link would read */
        for (int l = 0; l < EPD_LINES; l++)
            memcpy(gpu + (size_t)l * EPD_LINE, dmap[tg - 1] + (size_t)l * dpitch[tg - 1], EPD_LINE * 4);
    else
        epd_gpu_read(G, 0, gpu);
    t_read += now() - t0;
    if (tg && frames % 16 == 0) {               /* epd_gpu_read of a flat target (epdd's check) */
        static uint32_t back[EPD_WORDS];
        epd_gpu_read(G, tg, back);
        if (memcmp(back, gpu, sizeof back)) { if (bad++ < 5) printf("%s: epd_gpu_read differs from the mapping\n", what); }
    }
    frames++;
    if (memcmp(cpu, gpu, sizeof cpu)) {
        long n = 0, first = -1;
        for (long i = 0; i < EPD_WORDS; i++) if (cpu[i] != gpu[i]) { n++; if (first < 0) first = i; }
        if (bad++ < 5)
            printf("MISMATCH %s frame %ld: %ld words, first at line %ld word %ld: cpu %08x gpu %08x\n",
                   what, frames, n, first / EPD_LINE, first % EPD_LINE, cpu[first], gpu[first]);
    }
    epd_gpu_finishing(G, &E);       /* finished: nothing packed (stale codes, epd-gpu.c) */
    epd_advance(&E, NULL, NULL);
    return 1;
}

static void drain(const char *what) { while (step(what)) ; }

/* random content in r: mostly black/white, some greys, some flat areas */
static void paint(epd_rect r)
{
    int kind = rr(0, 3), flat = rr(0, 255);
    for (int y = r.y < 0 ? 0 : r.y; y < r.y + r.h && y < EPD_H; y++)        /* rects may start off-screen */
        for (int x = r.x < 0 ? 0 : r.x; x < r.x + r.w && x < EPD_W; x++) {
            uint8_t v;
            switch (kind) {
            case 0: v = rnd() & 1 ? 255 : 0; break;
            case 1: v = (uint8_t)rnd(); break;
            case 2: v = (uint8_t)flat; break;
            default: v = (uint8_t)((x * 7 + y * 3) & 255); break;
            }
            grey[(size_t)y * EPD_W + x] = v;
        }
}

static const int modes[] = { EPD_DU, EPD_GC16, EPD_GL16, EPD_A2_OR_DU, EPD_A2 };

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s eink_waveform.wbf [seed]\n", argv[0]); return 2; }
    if (argc > 2) rng = (uint32_t)strtoul(argv[2], NULL, 0) | 1;
    FILE *f = fopen(argv[1], "rb");
    if (!f) { perror(argv[1]); return 2; }
    static uint8_t wbf[1 << 20];
    size_t n = fread(wbf, 1, sizeof wbf, f);
    fclose(f);
    if (epd_init(&E, wbf, n, 24)) { fprintf(stderr, "bad waveform file\n"); return 2; }
    if (!(G = epd_gpu_open(&E))) return 1;
    printf("drive_state %s, claim_state %s\n", epd_gpu_drive_state_of(G), epd_claim_state_name(&E));     /* (packed imports the engine's W) */
    printf("renderer: %s\n", epd_gpu_renderer(G));
    if (argc > 3 && !strcmp(argv[3], "dumb") && dumb_open()) return 1;

    /* 1. the heaviest frame: a full-screen GC16 flash, every pixel driven */
    paint((epd_rect){ 0, 0, EPD_W, EPD_H });
    epd_submit(&E, (epd_rect){ 0, 0, EPD_W, EPD_H }, EPD_GC16, 1, grey, EPD_W);
    long f0 = frames; t_render = t_render_max = 0;
    drain("full GC16");
    printf("full-screen GC16: %ld frames, GPU %.2f ms/frame (max %.2f)\n",
           frames - f0, 1e3 * t_render / (frames - f0), 1e3 * t_render_max);

    /* 2. random updates, submitted while others run */
    for (int round = 0; round < 40; round++) {
        epd_set_cut(&E, rr(0, 5));
        int k = rr(1, 12);
        for (int i = 0; i < k; i++) {
            epd_rect r = { rr(-50, EPD_W - 20), rr(-50, EPD_H - 20), rr(8, 700), rr(8, 700) };
            paint(r);
            epd_submit(&E, r, modes[rr(0, 4)], rr(0, 3) == 0, grey, EPD_W);
        }
        for (int i = rr(0, 25); i > 0 && step("random"); i--) ;
    }
    drain("random");

    /* 2b. stale codes: a big update finishes (its pixels keep their old codes,
     * nothing is packed at the end), then its slot is reused by a small one
     * while another small one far away widens the drawn box over the stale
     * pixels — they must stay idle (the shader's slot rects) */
    for (int round = 0; round < 6; round++) {
        epd_rect a = { 0, 0, 600, 600 };
        paint(a);
        epd_submit(&E, a, round & 1 ? EPD_DU : EPD_A2_OR_DU, 0, grey, EPD_W);
        drain("stale: big");
        epd_rect b = { 8, 8, 48, 48 }, c = { 1000, 900, 48, 48 };
        paint(b); paint(c);
        /* the big one's pixels again "in flight" in its old transition: give
         * them changed content so their stale index is a real transition */
        epd_submit(&E, b, EPD_DU, 0, grey, EPD_W);
        epd_submit(&E, c, EPD_DU, 0, grey, EPD_W);
        drain("stale: reuse");
    }

    /* 2c. a scroll with matched cuts and non-blocking payback: cut-A2 steps
     * of a shifting text page, the next one when the area is free (as
     * glassine-epdd-desktop paces sway): payback runs end on their own frames, steps start
     * around them, stragglers are claimed later — nothing packed at the ends */
    epd_set_matched(&E, 1);
    epd_set_matched_rule(&E, 2, 0);
    epd_set_nonblock(&E, 1);
    for (int st = 0; st < 120; st++) {
        epd_rect a = { 0, 0, EPD_W, EPD_H };
        for (int y = 0; y < EPD_H; y++)
            for (int x = 0; x < EPD_W; x++)
                grey[(size_t)y * EPD_W + x] = (((y + st * 37) / 6) % 7 == 0 && ((x / 5 + (y + st * 37) / 40) % 3)) ? 0 : 255;
        static const int lens[] = { 2, 2, 4, 6, 8, 2, 4 };
        epd_submit(&E, a, EPD_MODE_A2LEN(EPD_A2_OR_DU, lens[st % 7]), 0, grey, EPD_W);
        for (int i = 0; i < 400 && (epd_frames_until_free(&E, a) || E.npend); i++) {
            if (!step("matched scroll") && !E.npend) break;
            if (!epd_frames_until_free(&E, a)) break;
        }
    }
    drain("matched scroll");
    printf("matched scroll: %llu px matched, %llu split starts\n", (unsigned long long)E.matched_px, (unsigned long long)E.split_starts);

    /* 2d. the same with healing (epd_set_heal): heal runs drive pixels that
     * stay black/white (LUT diagonal entries, zero in every other table), in
     * every step (from 2), uncut ones too */
    epd_set_heal(&E, 1, 2);
    for (int st = 0; st < 80; st++) {
        epd_rect a = { 0, 0, EPD_W, EPD_H };
        for (int y = 0; y < EPD_H; y++)
            for (int x = 0; x < EPD_W; x++)
                grey[(size_t)y * EPD_W + x] = (((y + st * 29) / 6) % 7 == 0 && ((x / 5 + (y + st * 29) / 40) % 3)) ? 0 : 255;
        static const int lens[] = { 2, 4, 6, 8, 0, 2, 0 };
        epd_submit(&E, a, EPD_MODE_A2LEN(EPD_A2_OR_DU, lens[st % 7]), 0, grey, EPD_W);
        for (int i = 0; i < 400 && (epd_frames_until_free(&E, a) || E.npend); i++) {
            if (!step("heal scroll") && !E.npend) break;
            if (!epd_frames_until_free(&E, a)) break;
        }
    }
    drain("heal scroll");
    printf("heal scroll: %llu px healed\n", (unsigned long long)E.healed_px);
    if (!E.healed_px) { printf("heal scroll: nothing healed\n"); bad++; }

    /* 2e. panel-model lengths with preemption (epd_set_nonblock 2): paying
     * pixels change hands mid-run (busy/level rewritten under running runs) */
    epd_set_model(&E, 2, 1);           /* (with the DC record's extra frames) */
    epd_set_heal(&E, 2, 2);
    epd_set_nonblock(&E, 2);
    unsigned long long pre0 = E.preempted;
    for (int st = 0; st < 80; st++) {
        epd_rect a = { 0, 0, EPD_W, EPD_H };
        for (int y = 0; y < EPD_H; y++)
            for (int x = 0; x < EPD_W; x++)
                grey[(size_t)y * EPD_W + x] = (((y + st * 41) / 6) % 7 == 0 && ((x / 5 + (y + st * 41) / 40) % 3)) ? 0 : 255;
        static const int lens[] = { 2, 2, 2, 4, 2, 6, 0 };
        epd_submit(&E, a, EPD_MODE_A2LEN(EPD_A2_OR_DU, lens[st % 7]), 0, grey, EPD_W);
        for (int i = 0; i < 400 && (epd_frames_until_free(&E, a) || E.npend); i++) {
            if (!step("preempt scroll") && !E.npend) break;
            if (!epd_frames_until_free(&E, a)) break;
        }
    }
    drain("preempt scroll");
    printf("preempt scroll: %llu px preempted\n", E.preempted - pre0);
    if (E.preempted == pre0) { printf("preempt scroll: nothing preempted\n"); bad++; }
    epd_set_model(&E, 0, 0);
    epd_set_heal(&E, 0, 2);
    epd_set_matched(&E, 0);
    epd_set_nonblock(&E, 0);

    /* 3. a pen stroke: 16x16 A2-or-DU pieces, three per frame */
    long p0 = frames; double pr = t_render; t_render_max = 0;
    int x = 200, y = 300;
    for (int i = 0; i < 300; i++) {
        for (int j = 0; j < 3; j++) {
            x += rr(-6, 12); y += rr(-6, 6);
            if (x > EPD_W - 20) x = 20;
            if (y < 10 || y > EPD_H - 30) y = 300;
            epd_rect r = { x, y, 16, 16 };
            for (int yy = 0; yy < 16; yy++)
                for (int xx = 0; xx < 16; xx++)
                    grey[(size_t)(y + yy) * EPD_W + x + xx] = (xx - 8) * (xx - 8) + (yy - 8) * (yy - 8) < 20 ? 0 : grey[(size_t)(y + yy) * EPD_W + x + xx];
            epd_submit(&E, r, EPD_A2_OR_DU, 0, grey, EPD_W);
        }
        step("pen");
    }
    drain("pen");
    printf("pen stroke: %ld frames, GPU %.2f ms/frame (max %.2f)\n",
           frames - p0, 1e3 * (t_render - pr) / (frames - p0), 1e3 * t_render_max);

    /* the GPU must leave what the link does not read alone: row padding, extra lines */
    for (int i = 0; i < ndumb; i++) {
        long n = 0;
        for (size_t o = 0; o < dsize[i]; o++) {
            size_t l = o / dpitch[i], w = o % dpitch[i];
            if ((l >= EPD_LINES || w >= EPD_LINE * 4) && dmap[i][o] != 0xa5) n++;
        }
        printf("dumb %d: %ld padding bytes written\n", i, n);
        if (n) bad++;
    }

    printf("%ld frames, %ld mismatched; GPU render %.2f ms/frame avg (max %.2f); state uploads %.1f ms total; "
           "readback (test only) %.2f ms/frame; peak running updates %d\n",
           frames, bad, 1e3 * t_render / frames, 1e3 * t_render_max, 1e3 * t_state, 1e3 * t_read / frames, E.nrun_max);
    epd_gpu_close(G);
    return bad ? 1 : 0;
}
