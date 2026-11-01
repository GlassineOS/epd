/* epd-gtarget — stage 5: update targets made on the GPU.
 *
 * The producer (epd-ingest, glassine-epdd-desktop's main thread, its own EGL display)
 * draws an update's target levels into a layer: a panel-sized RGBA8 image, 4
 * panel pixels per texel (texel (x / 4, y), component x % 4 = the level of
 * panel pixel (x, y), as the byte value). The engine gets the layer's handle
 * (epd_submit_gt); its GPU side (epd-gpu, the stream thread, another display)
 * imports the layer's dma-buf and reads it after the producer's fence. A
 * layer is drawn once per handle: from the submit until release nothing
 * writes it. */
#ifndef EPD_GTARGET_H
#define EPD_GTARGET_H
#include <stddef.h>
#include <stdint.h>

#define EPD_GT_MAX 16                   /* handles 1 .. EPD_GT_MAX */

typedef struct {
    int w, h;                           /* texels */
    uint32_t fourcc;                    /* DRM fourcc of the image */
    uint64_t modifier;                  /* DRM_FORMAT_MOD_INVALID (0x00ffffffffffffff): implicit */
    int planes;
    int fd[4], pitch[4], offset[4];     /* the provider's fds: dup them to keep */
} epd_gt_buf;

typedef struct epd_gt_provider {
    int n;                                          /* layers: handles 1 .. n */
    int (*buf)(void *ctx, int gt, epd_gt_buf *b);   /* 0 / -1 */
    /* a sync_file fd to wait on before reading layer gt (the caller closes
     * it), or -1: nothing to wait for. Any thread */
    int (*fence)(void *ctx, int gt);
    void (*release)(void *ctx, int gt);             /* free for the next draw; any thread */
    void *ctx;
} epd_gt_provider;

#endif
