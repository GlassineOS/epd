/* epd-defer-test — , deferred claims (epd_set_defer): a deferred
 * engine against a synchronous one, CPU only. Both get the same submits (the
 * pacing from the synchronous one); every frame their drive frames (the wire
 * words) and the finished runs' ids (in order), every 4th their whole state
 * (level, busy, stale, model, DC record, target where busy) must be equal;
 * at the end the counters and nrun_max. Scenarios as epd-gpe-test (a paced
 * scroll, a flood, random updates of every mode, scarce slots, 8 C), with
 * glassine-epdd-desktop's payback defaults, then nonblock 1 without snap, then nonblock 0
 * (matched plans not deferred there).
 *
 * epd-defer-test <eink_waveform.wbf> [frames per scenario]
 *
 * Exit 0 = equal throughout. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../epd-engine.h"

static epd_engine A, B;                 /* synchronous, deferred */
static long frames, bad, words_cmp, over_pace;
static uint32_t wa[EPD_WORDS], wb[EPD_WORDS];
static int da[2], db[2];
static uint32_t ida[4 * EPD_MAX_UPD], idb[4 * EPD_MAX_UPD];
static int nida, nidb;

static uint8_t *slurp(const char *p, size_t *n)
{
    FILE *f = fopen(p, "rb");
    if (!f) { perror(p); exit(2); }
    fseek(f, 0, SEEK_END); *n = (size_t)ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t *d = malloc(*n);
    if (fread(d, 1, *n, f) != *n) exit(2);
    fclose(f);
    return d;
}

static void fail(const char *what)
{
    printf("frame %ld: %s\n", frames, what);
    if (++bad > 5) { printf("epd-defer: too many differences\n"); exit(1); }
}

static void done_a(uint32_t id, void *arg) { (void)arg; if (nida < 4 * EPD_MAX_UPD) ida[nida++] = id; }
static void done_b(uint32_t id, void *arg) { (void)arg; if (nidb < 4 * EPD_MAX_UPD) idb[nidb++] = id; }

static void state_cmp(void)
{
    struct { const char *n; const void *a, *b; size_t sz; } f[] = {
        { "level", A.level, B.level, 1 }, { "busy", A.busy, B.busy, 1 }, { "stale", A.stale, B.stale, 1 },
        { "dc", A.dc, B.dc, 2 }, { "op", A.op, B.op, 1 },
    };
    for (unsigned k = 0; k < sizeof f / sizeof *f; k++)
        if (memcmp(f[k].a, f[k].b, (size_t)EPD_W * EPD_H * f[k].sz)) { char w[48]; snprintf(w, sizeof w, "%s differs", f[k].n); fail(w); }
    for (size_t i = 0; i < (size_t)EPD_W * EPD_H; i++)
        if (A.busy[i] && A.target[i] != B.target[i]) { fail("target differs where busy"); break; }
    if (A.npend != B.npend) fail("pending count differs");
}

static void frame(void)
{
    epd_schedule(&A);
    epd_schedule(&B);
    epd_render(&A, wa, da);
    epd_render(&B, wb, db);
    words_cmp++;
    if (memcmp(wa, wb, sizeof wa)) {
        int k = 0;
        while (wa[k] == wb[k]) k++;
        char w[80];
        snprintf(w, sizeof w, "drive frame differs, first word %d: sync %08x deferred %08x", k, wa[k], wb[k]);
        fail(w);
    }
    nida = nidb = 0;
    epd_advance(&A, done_a, NULL);
    epd_advance(&B, done_b, NULL);
    if (nida != nidb || memcmp(ida, idb, sizeof *ida * (size_t)nida)) fail("finished runs differ");
    if (frames % 4 == 0) state_cmp();          /* (the drive frames every frame) */
    frames++;
}

static void submit(epd_rect r, int mode, int full, const uint8_t *g, int stride)
{
    epd_submit(&A, r, mode, full, g, stride);
    epd_submit(&B, r, mode, full, g, stride);
}

#define PAGE_H 20000
static uint8_t *page;

static void make_page(void)
{
    page = malloc((size_t)EPD_W * PAGE_H);
    memset(page, 255, (size_t)EPD_W * PAGE_H);
    srand(5);
    for (int line = 0; line < PAGE_H / 40; line++)
        for (int x = 40; x < EPD_W - 40; x += 14 + rand() % 6) {
            if (rand() % 7 == 0) { x += 20; continue; }
            int h = 14 + rand() % 6, y0 = line * 40 + 10;
            for (int y = y0; y < y0 + h && y < PAGE_H; y++)
                for (int k = 0; k < 9; k++) {
                    int v = rand() % 4;
                    page[(size_t)y * EPD_W + x + k] = v == 0 ? 0 : v == 1 ? 96 : v == 2 ? 192 : 255;
                }
        }
}

static void scroll(int nf, int flood)
{
    static const int cuts[] = { 8, 6, 4, 2 };
    epd_rect win = { 0, 60, EPD_W, 1300 };
    int off = 0, n = 0;
    for (int f = 0; f < nf; f++) {
        int fa = epd_frames_until_free(&A, win), fb = epd_frames_until_free(&B, win);
        if (fb < fa) fail("pacing below the synchronous engine's");
        over_pace += fb != fa;
        int want = flood ? f % 4 == 0 : fa <= 1 && A.npend == 0;
        if (want) {
            off = (off + 36) % (PAGE_H - win.h);
            submit(win, EPD_MODE_A2LEN(EPD_A2_OR_DU, cuts[n++ % 4]), 0, page + (size_t)off * EPD_W - (size_t)win.y * EPD_W + win.x, EPD_W);
        }
        frame();
    }
}

static void randoms(int nf, int seed)
{
    static uint8_t img[EPD_W * EPD_H];
    srand(seed);
    static const int modes[] = { EPD_GC16, EPD_GL16, EPD_DU, EPD_DU4, EPD_A2, EPD_A2_OR_DU };
    for (int f = 0; f < nf; f++) {
        if (rand() % 3 == 0) {
            int off = rand() % (PAGE_H - EPD_H);
            memcpy(img, page + (size_t)off * EPD_W, sizeof img);
            for (int k = rand() % 4; k > 0; k--) {
                int x = rand() % (EPD_W - 200), y = rand() % (EPD_H - 200), v = rand() % 256;
                for (int yy = y; yy < y + 150; yy++) memset(img + (size_t)yy * EPD_W + x, v, 150);
            }
            int w = 16 + rand() % (EPD_W - 16), h = 16 + rand() % (EPD_H - 16);
            epd_rect r = { rand() % (EPD_W - w + 1), rand() % (EPD_H - h + 1), w, h };
            int m = modes[rand() % 6], full = rand() % 8 == 0 ? 1 + rand() % 2 : 0;
            if ((m == EPD_A2 || m == EPD_A2_OR_DU) && rand() % 2) m = EPD_MODE_A2LEN(m, 2 + rand() % 7);
            submit(r, m, full, img, EPD_W);
        }
        frame();
    }
}

static void scarce(int rounds)
{
    static uint8_t img[EPD_W * EPD_H];
    static const int cuts[] = { 8, 6, 4 };
    epd_rect win = { 0, 60, EPD_W, 1300 };
    while (!epd_idle(&A)) frame();
    for (int k = 0; k < rounds; k++) {
        memcpy(img, page + (size_t)(k * 97 % (PAGE_H - EPD_H)) * EPD_W, sizeof img);
        for (int i = 0; i < 58; i++)
            for (int y = 20; y < 40; y++) memset(img + (size_t)y * EPD_W + i * 30, (k & 1) ? 96 : 192, 18);
        for (int i = 0; i < 58; i++) submit((epd_rect){ i * 30, 20, 18, 20 }, EPD_DU, 0, img, EPD_W);
        frame();
        submit(win, EPD_MODE_A2LEN(EPD_A2_OR_DU, cuts[k % 3]), 0, img, EPD_W);
        for (int f = 0; f < 12; f++) frame();
    }
}

static void setup(epd_engine *e, const uint8_t *wbf, size_t wn, int nonblock, int snap, int defer)
{
    if (epd_init(e, wbf, wn, 24)) { fprintf(stderr, "wbf\n"); exit(1); }
    epd_assume(e, 30);
    epd_set_matched(e, 1);
    epd_set_matched_rule(e, 2, 3);
    epd_set_nonblock(e, nonblock);
    epd_set_model(e, 2, 0);
    epd_set_heal(e, 2, 2);
    epd_set_snap(e, snap);
    epd_set_defer(e, defer);
}

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s <wbf> [frames]\n", argv[0]); return 2; }
    int nf = argc > 2 ? atoi(argv[2]) : 300;
    size_t wn;
    uint8_t *wbf = slurp(argv[1], &wn);
    make_page();
    static const struct { int nonblock, snap; } cfg[] = { { 2, 1 }, { 1, 0 }, { 0, 1 } };
    for (unsigned c = 0; c < sizeof cfg / sizeof *cfg; c++) {
        setup(&A, wbf, wn, cfg[c].nonblock, cfg[c].snap, 0);
        setup(&B, wbf, wn, cfg[c].nonblock, cfg[c].snap, 1);
        long f0 = frames;
        scroll(nf, 0);
        scroll(nf, 1);
        randoms(nf, 11 + (int)c);
        scarce(nf / 20);
        A.celsius = B.celsius = 8;
        scroll(nf / 2, 0);
        randoms(nf / 2, 23 + (int)c);
        while (!epd_idle(&A) || !epd_idle(&B)) frame();
        epd_claim_settle(&B, 1);
        if (A.preempted != B.preempted || A.snapped_px != B.snapped_px || A.matched_px != B.matched_px ||
            A.healed_px != B.healed_px || A.flash_skipped != B.flash_skipped) fail("counters differ");
        if (A.nrun_max != B.nrun_max) {
            char w[64];
            snprintf(w, sizeof w, "nrun_max differs: sync %d deferred %d", A.nrun_max, B.nrun_max);
            fail(w);
        }
        printf("epd-defer: nonblock %d snap %d: %ld frames, %llu of the claims deferred, %ld frames paced on the over-estimate\n",
               cfg[c].nonblock, cfg[c].snap, frames - f0, (unsigned long long)B.deferred, over_pace);
        over_pace = 0;                          /* (the tables leak: the next config's engines start fresh) */
    }
    printf("epd-defer: %ld frames, %ld drive frames compared, %ld differences\n", frames, words_cmp, bad);
    if (bad) return 1;
    printf("epd-defer: ok\n");
    return 0;
}
