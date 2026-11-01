/* glassine-epdd [-c CONFIG] [-s SOCKET]
 * The daemon that owns the e-ink panel, first for the recovery (*. Clients draw into a memfd and hand
 * their frames over with the glassine-epd frame protocol (glassine-epd-proto.h);
 * TWRP's minuitwrp epd backend is one. The panel is driven by glassine-epdd-desktop's
 * userspace engine (epd-engine.c + epd-drm.c), CPU only.
 *
 * Generic core + device adapter: everything about the device comes from the
 * adapter config (/etc/glassine/epd.conf, shipped by the device tree):
 * panel_width, panel_height the panel's native size (the engine's, so far)
 * rotation the portrait surface -> the panel: ccw, cw or none
 * waveform the waveform file (glassine-epd-start fetches it)
 * temperature sysfs files tried in turn, value / temperature_scale = degrees C
 * frontlight brightness files, off while asleep (each kept as it was)
 * sleep_text the asleep bar's text (capitals, epd-glyphs.h; default
 * PRESS POWER TO WAKE)
 * Unknown keys are ignored (glassine-epd-start reads its own).
 *
 * At start the panel is flashed white (INIT). Each frame drives only the
 * pixels that changed (GC16 partial); a frame that changes more than half the
 * panel (a page change) is a full GC16, which also clears ghosting.
 * SLEEP (the client's screen timeout) puts a bar across the bottom of the UI
 * saying how to wake up, turns the light off and parks the panel link; WAKE
 * undoes that, and the client's next frame paints over the bar.
 * Static, no X/Wayland/GPU: built like glassine-epd-show. */
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#include "epd-engine.h"
#include "epd-drm.h"
#include "epd-glyphs.h"
#include "glassine-epd-proto.h"

#define W EPD_W                         /* the panel, as the engine is built */
#define H EPD_H
#define NBUF 4
#define NTEMP 4
#define NLIGHT 4
#define BAR_H 72                        /* the asleep bar, as glassine-epd-show's */

#define FOURCC(a, b, c, d) ((uint32_t)(a) | (uint32_t)(b) << 8 | (uint32_t)(c) << 16 | (uint32_t)(d) << 24)
#define FMT_XBGR FOURCC('X', 'B', '2', '4')     /* bytes R G B X (TWRP's RGBX) */
#define FMT_ABGR FOURCC('A', 'B', '2', '4')
#define FMT_XRGB FOURCC('X', 'R', '2', '4')     /* bytes B G R X */
#define FMT_ARGB FOURCC('A', 'R', '2', '4')

enum { ROT_NONE, ROT_CCW, ROT_CW };

static struct {                         /* the device adapter */
    int width, height, rot, temp_scale;
    char waveform[256], temp[NTEMP][256], light[NLIGHT][256], sleep_text[64];
} ad = { W, H, ROT_CCW, 10, "/tmp/eink_waveform.wbf", { "" }, { "" }, "PRESS POWER TO WAKE" };

static int asleep;
static char light_was[NLIGHT][16];      /* the light's levels before SLEEP */

struct buf { int live; uint8_t *map; size_t len; uint32_t w, h, stride, off, bgr; };

static struct buf bufs[NBUF];
static uint8_t grey[W * H];             /* what was last submitted, panel order */

static int load_adapter(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) { perror(path); return -1; }
    char l[512];
    int nt = 0, nl = 0;
    while (fgets(l, sizeof l, f)) {
        char *v = strchr(l, '=');
        if (*l == '#' || !v) continue;
        *v++ = 0;
        v[strcspn(v, "\r\n#")] = 0;          /* (sleep_text keeps its spaces) */
        for (char *e = v + strlen(v); e > v && (e[-1] == ' ' || e[-1] == '\t');) *--e = 0;
        if (!strcmp(l, "panel_width")) ad.width = atoi(v);
        else if (!strcmp(l, "panel_height")) ad.height = atoi(v);
        else if (!strcmp(l, "rotation")) ad.rot = !strcmp(v, "cw") ? ROT_CW : !strcmp(v, "none") ? ROT_NONE : ROT_CCW;
        else if (!strcmp(l, "waveform")) snprintf(ad.waveform, sizeof ad.waveform, "%s", v);
        else if (!strcmp(l, "temperature") && nt < NTEMP) snprintf(ad.temp[nt++], sizeof ad.temp[0], "%s", v);
        else if (!strcmp(l, "temperature_scale")) ad.temp_scale = atoi(v) > 0 ? atoi(v) : 1;
        else if (!strcmp(l, "frontlight") && nl < NLIGHT) snprintf(ad.light[nl++], sizeof ad.light[0], "%s", v);
        else if (!strcmp(l, "sleep_text")) snprintf(ad.sleep_text, sizeof ad.sleep_text, "%s", v);
    }
    fclose(f);
    if (ad.width != W || ad.height != H) {
        fprintf(stderr, "glassine-epdd: %s: panel %dx%d, the engine drives %dx%d only\n", path, ad.width, ad.height, W, H);
        return -1;
    }
    return 0;
}

static int celsius(void)
{
    for (int i = 0; i < NTEMP && *ad.temp[i]; i++) {
        FILE *f = fopen(ad.temp[i], "r");
        char b[32];
        if (!f) continue;
        int ok = fgets(b, sizeof b, f) != NULL;
        fclose(f);
        if (ok) return atoi(b) / ad.temp_scale;
    }
    return 24;
}

/* the surface the clients draw: portrait unless the panel is used as it is */
static uint32_t surf_w(void) { return ad.rot == ROT_NONE ? W : H; }
static uint32_t surf_h(void) { return ad.rot == ROT_NONE ? H : W; }

static void drop(uint32_t id)
{
    if (id >= NBUF || !bufs[id].live) return;
    munmap(bufs[id].map, bufs[id].len);
    bufs[id].live = 0;
}

static void take_buf(const struct glassine_epd_msg *m, int fd)
{
    if (fd < 0) return;
    int bgr = m->format == FMT_XBGR || m->format == FMT_ABGR;
    if (m->id >= NBUF || !(bgr || m->format == FMT_XRGB || m->format == FMT_ARGB)
        || m->width != surf_w() || m->height != surf_h() || m->stride < m->width * 4) {
        fprintf(stderr, "glassine-epdd: BUF %u refused (%ux%u stride %u format %08x)\n",
                m->id, m->width, m->height, m->stride, m->format);
        close(fd);
        return;
    }
    drop(m->id);
    size_t len = (size_t)m->offset + (size_t)m->stride * m->height;
    void *p = mmap(NULL, len, PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    if (p == MAP_FAILED) { perror("glassine-epdd: mmap BUF"); return; }
    bufs[m->id] = (struct buf){ 1, p, len, m->width, m->height, m->stride, m->offset, bgr };
}

/* surface (u, v) is panel (W - 1 - v, u) for ccw, (v, H - 1 - u) for cw */
static inline void to_panel(int u, int v, int *x, int *y)
{
    *x = ad.rot == ROT_CCW ? W - 1 - v : ad.rot == ROT_CW ? v : u;
    *y = ad.rot == ROT_CCW ? u : ad.rot == ROT_CW ? H - 1 - u : v;
}

/* the surface's rect into the panel's grey; the bounding box of the panel
 * pixels that changed in *ch (w 0: none) */
static void ingest(const struct buf *b, struct glassine_epd_rect r, epd_rect *ch)
{
    int x0 = W, y0 = H, x1 = -1, y1 = -1;
    for (int v = r.y; v < r.y + r.h; v++) {
        const uint8_t *row = b->map + b->off + (size_t)v * b->stride;
        for (int u = r.x; u < r.x + r.w; u++) {
            const uint8_t *p = row + (size_t)u * 4;
            int rr = b->bgr ? p[0] : p[2], bb = b->bgr ? p[2] : p[0];
            uint8_t g = (uint8_t)((77 * rr + 150 * p[1] + 29 * bb) >> 8);
            int x, y;
            to_panel(u, v, &x, &y);
            uint8_t *d = &grey[(size_t)y * W + x];
            if (*d == g) continue;
            *d = g;
            if (x < x0) x0 = x;
            if (x > x1) x1 = x;
            if (y < y0) y0 = y;
            if (y > y1) y1 = y;
        }
    }
    *ch = x1 < 0 ? (epd_rect){ 0, 0, 0, 0 } : (epd_rect){ x0, y0, x1 - x0 + 1, y1 - y0 + 1 };
}

static void clip(struct glassine_epd_rect *r)
{
    int sw = (int)surf_w(), sh = (int)surf_h();
    if (r->x < 0) { r->w += r->x; r->x = 0; }
    if (r->y < 0) { r->h += r->y; r->y = 0; }
    if (r->x + r->w > sw) r->w = sw - r->x;
    if (r->y + r->h > sh) r->h = sh - r->y;
    if (r->w < 0) r->w = 0;
    if (r->h < 0) r->h = 0;
}

static void frame(epd_stream *s, int c, const struct glassine_epd_msg *m)
{
    struct glassine_epd_msg done = { .type = GLASSINE_EPD_DONE, .id = m->id, .seq = m->seq };
    if (!asleep && m->id < NBUF && bufs[m->id].live) {
        struct glassine_epd_rect all = { 0, 0, (int)surf_w(), (int)surf_h() };
        uint32_t n = m->nrect > GLASSINE_EPD_MAXRECT ? GLASSINE_EPD_MAXRECT : m->nrect;
        int x0 = W, y0 = H, x1 = -1, y1 = -1;
        epd_stream_wait_idle(s);        /* a running update may still read grey */
        for (uint32_t i = 0; i < (n ? n : 1); i++) {
            struct glassine_epd_rect r = n ? m->rect[i] : all;
            epd_rect ch;
            clip(&r);
            ingest(&bufs[m->id], r, &ch);
            if (!ch.w) continue;
            if (ch.x < x0) x0 = ch.x;
            if (ch.y < y0) y0 = ch.y;
            if (ch.x + ch.w - 1 > x1) x1 = ch.x + ch.w - 1;
            if (ch.y + ch.h - 1 > y1) y1 = ch.y + ch.h - 1;
        }
        if (x1 >= 0) {
            epd_rect r = { x0, y0, x1 - x0 + 1, y1 - y0 + 1 };
            int page = (long)r.w * r.h * 2 > (long)W * H;
            if (page) r = (epd_rect){ 0, 0, W, H };
            epd_stream_submit(s, r, EPD_GC16, page, grey + (size_t)r.y * W + r.x, W);
        }
    }
    send(c, &done, sizeof done, MSG_NOSIGNAL);
}

/* the light's level now ("" if unreadable) */
static void light_level(int i, char out[16])
{
    FILE *f = fopen(ad.light[i], "r");
    *out = 0;
    if (f && fgets(out, 16, f)) out[strcspn(out, "\n")] = 0;
    if (f) fclose(f);
}

static void light(int i, const char *level)
{
    int fd = open(ad.light[i], O_WRONLY | O_CLOEXEC);
    if (fd < 0) return;
    if (write(fd, level, strlen(level)) < 0) perror(ad.light[i]);
    close(fd);
}

/* the bar across the bottom of the UI as seen (white capitals on black), the
 * light off, the link parked */
static void sleep_panel(epd_stream *s)
{
    if (asleep) return;
    const char *t = ad.sleep_text;
    int len = (int)strlen(t), sw = (int)surf_w(), sh = (int)surf_h();
    int x0 = W, y0 = H, x1 = -1, y1 = -1;
    epd_stream_wait_idle(s);
    for (int v = sh - BAR_H; v < sh; v++)
        for (int u = 0; u < sw; u++) {
            int x, y;
            to_panel(u, v, &x, &y);
            grey[(size_t)y * W + x] = epd_bar_ink(t, len, 6, sw, BAR_H, u, v - (sh - BAR_H)) ? 0xff : 0x00;
            if (x < x0) x0 = x;
            if (x > x1) x1 = x;
            if (y < y0) y0 = y;
            if (y > y1) y1 = y;
        }
    epd_rect r = { x0, y0, x1 - x0 + 1, y1 - y0 + 1 };
    epd_stream_submit(s, r, EPD_GC16, 0, grey + (size_t)r.y * W + r.x, W);
    epd_stream_wait_idle(s);
    /* only a light that is on is switched off and remembered: a client that
     * dims the light itself (TWRP sets 0 before it blanks) keeps that job, and
     * WAKE never overwrites the level it sets again */
    for (int i = 0; i < NLIGHT && *ad.light[i]; i++) {
        light_level(i, light_was[i]);
        if (!strcmp(light_was[i], "0")) *light_was[i] = 0;
        if (*light_was[i]) light(i, "0");
    }
    epd_stream_park(s, 1);
    asleep = 1;
}

static void wake_panel(epd_stream *s)
{
    if (!asleep) return;
    char now[16];
    for (int i = 0; i < NLIGHT && *ad.light[i]; i++) {
        light_level(i, now);
        if (*light_was[i] && !strcmp(now, "0")) light(i, light_was[i]);   /* still off as we left it */
        *light_was[i] = 0;
    }
    epd_stream_park(s, 0);
    asleep = 0;
}

/* one client until it goes; 0: it spoke a version we do not */
static int serve(epd_stream *s, int c)
{
    for (;;) {
        struct glassine_epd_msg m;
        char cbuf[CMSG_SPACE(sizeof(int))];
        struct iovec iov = { &m, sizeof m };
        struct msghdr h = { .msg_iov = &iov, .msg_iovlen = 1, .msg_control = cbuf, .msg_controllen = sizeof cbuf };
        memset(&m, 0, sizeof m);
        ssize_t n = recvmsg(c, &h, MSG_CMSG_CLOEXEC);
        if (n <= 0) { if (n < 0 && errno == EINTR) continue; return 1; }
        int fd = -1;
        for (struct cmsghdr *cm = CMSG_FIRSTHDR(&h); cm; cm = CMSG_NXTHDR(&h, cm))
            if (cm->cmsg_level == SOL_SOCKET && cm->cmsg_type == SCM_RIGHTS) memcpy(&fd, CMSG_DATA(cm), sizeof fd);
        switch (m.type) {
        case GLASSINE_EPD_HELLO: {
            struct glassine_epd_msg r = { .type = GLASSINE_EPD_HELLO, .id = GLASSINE_EPD_PROTO_VERSION };
            send(c, &r, sizeof r, MSG_NOSIGNAL);
            if (m.id != GLASSINE_EPD_PROTO_VERSION) {
                fprintf(stderr, "glassine-epdd: client speaks protocol %u, not %u\n", m.id, GLASSINE_EPD_PROTO_VERSION);
                return 0;
            }
            break;
        }
        case GLASSINE_EPD_BUF: take_buf(&m, fd); fd = -1; break;
        case GLASSINE_EPD_FRAME: frame(s, c, &m); break;
        case GLASSINE_EPD_DROP: drop(m.id); break;
        case GLASSINE_EPD_SLEEP: sleep_panel(s); break;
        case GLASSINE_EPD_WAKE: wake_panel(s); break;
        default: break;                 /* newer, compatible messages */
        }
        if (fd >= 0) close(fd);
    }
}

int main(int argc, char **argv)
{
    const char *conf = "/etc/glassine/epd.conf", *sock = "/dev/socket/glassine-epd";
    for (int o; (o = getopt(argc, argv, "c:s:")) != -1;) {
        if (o == 'c') conf = optarg;
        else if (o == 's') sock = optarg;
        else { fprintf(stderr, "usage: %s [-c CONFIG] [-s SOCKET]\n", argv[0]); return 2; }
    }
    if (load_adapter(conf)) return 1;
    signal(SIGPIPE, SIG_IGN);
    epd_stream *s = epd_stream_open(ad.waveform, celsius(), 1);
    if (!s) { fprintf(stderr, "glassine-epdd: no panel (waveform %s)\n", ad.waveform); return 1; }
    memset(grey, 0xff, sizeof grey);
    epd_stream_submit(s, (epd_rect){ 0, 0, W, H }, EPD_INIT, 1, grey, W);

    int l = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
    struct sockaddr_un a = { .sun_family = AF_UNIX };
    snprintf(a.sun_path, sizeof a.sun_path, "%s", sock);
    unlink(sock);
    if (l < 0 || bind(l, (struct sockaddr *)&a, sizeof a) || listen(l, 1)) {
        perror("glassine-epdd: socket");
        return 1;
    }
    chmod(sock, 0660);
    for (;;) {
        int c = accept4(l, NULL, NULL, SOCK_CLOEXEC);
        if (c < 0) { if (errno == EINTR) continue; perror("glassine-epdd: accept"); return 1; }
        serve(s, c);
        close(c);
        wake_panel(s);                  /* the next client finds the panel up */
        for (uint32_t i = 0; i < NBUF; i++) drop(i);
        epd_stream_set_temp(s, celsius());
    }
}
