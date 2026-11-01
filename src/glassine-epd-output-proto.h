/* glassine-epd-output-proto.h — sway's glassine-epd output backend <-> glassine-epdd-desktop (*.
 *
 * The panel is a wlroots output. sway renders into shm buffers (pixman
 * renderer, memfd); the backend hands each buffer's fd to glassine-epdd-desktop ONCE, then
 * per frame only "buffer N is the new frame, these rects changed". Zero copies.
 * glassine-epdd-desktop answers DONE when it has read what it needs from that frame; only
 * then does the backend give the buffer back to sway's swapchain and let sway
 * render the next frame (wlr_output_send_frame): apps draw at the panel's pace.
 *
 * Socket: GLASSINE_EPD_SOCK, AF_UNIX SOCK_SEQPACKET (one message per packet).
 * glassine-epdd-desktop listens (it owns the panel and lives longer); the backend connects
 * and retries while none is listening. On (re)connect the backend sends BUF for
 * every live buffer, then a full FRAME. No consumer: frames are paced by a
 * timer and dropped.
 *
 * backend -> glassine-epdd-desktop:
 * BUF id, width, height, stride, format (DRM fourcc), offset; the fd via SCM_RIGHTS
 * FRAME id, seq, nrect, rect[] (buffer coordinates; nrect == 0: the whole buffer);
 * format (GLASSINE_EPD_CAP_GAMMA) = the tag of the gamma table
 * the whole buffer went through, 0 = none (the raw grey q, as before);
 * offset (GLASSINE_EPD_CAP_BAND) = GLASSINE_EPD_FRAME_TRUE: the dead
 * band's true frame, no new content: BAND below
 * DMABUF as BUF, for a GPU buffer (GLES sway): a single-plane
 * dma-buf; the modifier in seq (low 32 bits) and nrect (high), so the
 * message layout stays as it was (an older glassine-epdd-desktop ignores it and
 * answers that buffer's frames unread)
 * DROP id: that buffer is gone (close its fd, unmap)
 * HELLO first on a connection: seq = what this backend
 * understands (GLASSINE_EPD_CAP_*); an older one sends none.
 * GLASSINE_EPD_CAP_LUM : its R8 grey output carries the
 * luminance q (epd-grey.h), not luma
 * GLASSINE_EPD_CAP_GAMMA : the grey output applies
 * glassine-epdd-desktop's gamma table on the GPU (LUT below): a final pass over
 * each frame's damage into the backend's own R8 buffers
 * GLASSINE_EPD_CAP_BAND : that pass can hold greys in a dead
 * band (BAND below)
 * CURSOR_IMG the pointer's sprite changed: width, height,
 * stride, offset = where its ARGB8888 pixels sit in the cursor memfd
 * (the backend's, two slots used in turn: the one named stays put until
 * the IMG after next), format = the layout (GLASSINE_EPD_CUR_*); the
 * hotspot in rect[0].x/y; width 0 = no image (hidden). The memfd goes
 * with the first IMG on a connection (SCM_RIGHTS). Everything is in
 * buffer coordinates, as FRAME rects: wlroots renders the sprite
 * already turned by the output transform and hands the hotspot over
 * turned the same way
 * CURSOR_POS the pointer's hotspot moved to rect[0].x/y (buffer coordinates;
 * the sprite's top-left = this - the hotspot). An older glassine-epdd-desktop ignores
 * both (the memfd is closed unread): no pointer on the panel
 * glassine-epdd-desktop -> backend:
 * LUT id = the table's tag (nonzero: glassine_epd_lut_tag), its 256 bytes in
 * rect[] (GLASSINE_EPD_CAP_GAMMA only; after HELLO and on every change).
 * The backend applies it from the next frame and sends the whole
 * picture again through it; frames until then carry the old tag
 * BAND seq = T (0..16, 0: off), width = the quiet gap in ms
 * (GLASSINE_EPD_CAP_BAND only; after HELLO and on every change). A pixel
 * keeps its last accepted grey while |new - accepted| < T, after the
 * table. The damage since the last true picture is remembered; after
 * the gap without a frame the backend redraws it without the band and
 * sends it as one FRAME marked GLASSINE_EPD_FRAME_TRUE (T changing too):
 * the same picture as far as the apps go, so no damage for the policy
 * DONE id, seq: frame seq's buffer is no longer read; the next frame may come
 * NEXT id, seq (GLASSINE_EPD_CAP_NEXT only): the next frame may come, but frame
 * seq's buffer stays locked until its DONE (glassine-epdd-desktop reads a frame
 * only when it needs one: a frame replaced before that is answered
 * DONE unread, its damage rides on the newer one, whose buffer holds
 * the whole picture)
 */
#ifndef GLASSINE_EPD_PROTO_H
#define GLASSINE_EPD_PROTO_H
#include <stddef.h>
#include <stdint.h>

#define GLASSINE_EPD_SOCK "/run/glassine-epd.sock"
#define GLASSINE_EPD_MAXRECT 32                  /* more damage rects: sent as their bounding box */

enum { GLASSINE_EPD_BUF = 1, GLASSINE_EPD_FRAME = 2, GLASSINE_EPD_DROP = 3, GLASSINE_EPD_DONE = 4, GLASSINE_EPD_DMABUF = 5,
       GLASSINE_EPD_HELLO = 6, GLASSINE_EPD_NEXT = 7, GLASSINE_EPD_CURSOR_IMG = 8, GLASSINE_EPD_CURSOR_POS = 9,
       GLASSINE_EPD_LUT = 12, GLASSINE_EPD_BAND = 13 };
#define GLASSINE_EPD_CAP_NEXT 1u
#define GLASSINE_EPD_CAP_LUM 2u
#define GLASSINE_EPD_CAP_GAMMA 4u
#define GLASSINE_EPD_CAP_BAND 8u
#define GLASSINE_EPD_FRAME_TRUE 1u               /* FRAME's offset: the dead band's true frame */
#define GLASSINE_EPD_BAND_MAX 16
/* CURSOR_IMG's format: the frames' layout the sprite and positions are in */
#define GLASSINE_EPD_CUR_PORTRAIT 0u             /* XRGB frames: portrait */
#define GLASSINE_EPD_CUR_PANEL 1u                /* the grey output: R8 frames in panel order */
#define GLASSINE_EPD_CUR_SLOT 65536u             /* one memfd slot: up to 128 x 128 ARGB */
#define GLASSINE_EPD_CUR_MAX 128

struct glassine_epd_rect { int32_t x, y, w, h; };

struct glassine_epd_msg {
    uint32_t type, id;
    uint32_t width, height, stride, format, offset;    /* BUF */
    uint32_t seq, nrect;                                /* FRAME, DONE */
    struct glassine_epd_rect rect[GLASSINE_EPD_MAXRECT];
};

/* a gamma table's tag: FNV-1a of its 256 bytes, never 0 */
static inline uint32_t glassine_epd_lut_tag(const uint8_t lut[256])
{
    uint32_t h = 2166136261u;
    for (int i = 0; i < 256; i++) h = (h ^ lut[i]) * 16777619u;
    return h | 1u;
}

#endif
