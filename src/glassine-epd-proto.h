/* glassine-epd-proto.h — the glassine-epd frame protocol: a client (TWRP's
 * minuitwrp epd backend, later the compositor) <-> glassine-epdd, the daemon
 * that owns the e-ink panel.
 *
 * Socket: AF_UNIX SOCK_SEQPACKET, one message per packet; the path is the
 * client's setting (the recovery: /dev/socket/glassine-epd). The daemon
 * listens; a client connects and retries while none is listening.
 *
 * The client draws into a memfd and hands its fd over ONCE (BUF); per frame it
 * only says "buffer N is the new frame, these rects changed" (FRAME). The
 * daemon answers DONE once it has read what it needs; only then may the client
 * draw into that buffer again.
 *
 * Version: the first message on a connection is HELLO with id = the client's
 * GLASSINE_EPD_PROTO_VERSION. The daemon answers HELLO with its own version
 * and closes the connection when it does not speak the client's; a client that
 * gets a version it does not speak disconnects. Compatible additions (new
 * message types, which a peer ignores) keep the version; anything else bumps
 * it, so the recovery and the daemon can be updated apart.
 *
 * client -> daemon:
 * HELLO id = version
 * BUF id, width, height, stride, format (DRM fourcc), offset; the fd via SCM_RIGHTS
 * FRAME id, seq, nrect, rect[] (buffer coordinates; nrect == 0: the whole buffer)
 * DROP id: that buffer is gone
 * SLEEP the client's screen timeout: the daemon shows that the power key
 * wakes the device (a bar across the bottom), turns the light off and
 * powers the panel link down. Frames until WAKE are answered unread.
 * WAKE the light and the link come back; the client then sends a frame
 * daemon -> client:
 * HELLO id = version
 * DONE id, seq: frame seq's buffer is no longer read; the next frame may come
 *
 * The layout and the type numbers are those of glassine-epd-output-proto.h
 * (sway <-> glassine-epdd-desktop), so one daemon can serve both while they converge.
 */
#ifndef GLASSINE_EPD_PROTO_H
#define GLASSINE_EPD_PROTO_H
#include <stdint.h>

#define GLASSINE_EPD_PROTO_VERSION 1
#define GLASSINE_EPD_MAXRECT 32         /* more damage rects: send their bounding box */

enum { GLASSINE_EPD_BUF = 1, GLASSINE_EPD_FRAME = 2, GLASSINE_EPD_DROP = 3, GLASSINE_EPD_DONE = 4,
       GLASSINE_EPD_HELLO = 6, GLASSINE_EPD_SLEEP = 10, GLASSINE_EPD_WAKE = 11 };

struct glassine_epd_rect { int32_t x, y, w, h; };

struct glassine_epd_msg {
    uint32_t type, id;
    uint32_t width, height, stride, format, offset;    /* BUF */
    uint32_t seq, nrect;                                /* FRAME, DONE */
    struct glassine_epd_rect rect[GLASSINE_EPD_MAXRECT];
};

#endif
