/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* heap-shim.h: the cached dma-buf the probes run on (; tests only).
 * hs_alloc: /dev/dma_heap/system (the device: root) first, else a memfd turned into a
 * dma-buf by /dev/udmabuf (the host stand-in: a real dma-buf, DMA_BUF_IOCTL_SYNC and the
 * EGL import work on it, but its numbers are not the A509's), else "no heap" (-1, the
 * reason in why). HS_SRC=heap|udmabuf picks one; unset: heap, then udmabuf.
 * The system heap's begin/end_cpu_access sync only the attachments that are mapped
 * (drivers/dma-buf/heaps/system_heap.c): a buffer no device imported syncs nothing, so a
 * timing of the ioctl imports it first (hs_import). Exit code 78 = no heap (77 = no GPU).
 * With HS_EGL: a surfaceless GLES 3.1 context and the dma-buf import. */
#ifndef HEAP_SHIM_H
#define HEAP_SHIM_H
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#if __has_include(<linux/udmabuf.h>)
#include <linux/udmabuf.h>
#else
struct udmabuf_create { uint32_t memfd, flags; uint64_t offset, size; };
#define UDMABUF_FLAGS_CLOEXEC 0x01
#define UDMABUF_CREATE _IOW('u', 0x42, struct udmabuf_create)
#endif
#ifndef F_ADD_SEALS
#define F_ADD_SEALS 1033
#endif
#ifndef F_SEAL_SHRINK
#define F_SEAL_SHRINK 0x0002
#endif
#ifndef MFD_ALLOW_SEALING
#define MFD_ALLOW_SEALING 0x0002U
#endif
#define HS_NOHEAP 78

typedef struct { int fd; size_t len; uint8_t *map; const char *src; } hs_buf;

static const char *hs_last __attribute__((unused)) = "none";   /* (the last good alloc's source: the probes' label) */

__attribute__((unused)) static int hs_heap(size_t len, char *why, size_t wn)
{
    int h = open("/dev/dma_heap/system", O_RDONLY | O_CLOEXEC);
    if (h < 0) { snprintf(why, wn, "/dev/dma_heap/system: %s", strerror(errno)); return -1; }
    struct dma_heap_allocation_data al = { .len = len, .fd_flags = O_RDWR | O_CLOEXEC };
    int r = ioctl(h, DMA_HEAP_IOCTL_ALLOC, &al), e = errno;
    close(h);
    if (r < 0) { snprintf(why, wn, "DMA_HEAP_IOCTL_ALLOC %zu: %s", len, strerror(e)); return -1; }
    return (int)al.fd;
}

__attribute__((unused)) static int hs_udmabuf(size_t len, char *why, size_t wn)
{
    size_t n = strlen(why);
    int u = open("/dev/udmabuf", O_RDWR | O_CLOEXEC);
    if (u < 0) { snprintf(why + n, wn - n, "%s/dev/udmabuf: %s", n ? "; " : "", strerror(errno)); return -1; }
    int m = memfd_create("heap-shim", MFD_ALLOW_SEALING);
    struct udmabuf_create c = { 0 };
    int fd = -1;
    if (m >= 0 && ftruncate(m, (off_t)len) == 0 && fcntl(m, F_ADD_SEALS, F_SEAL_SHRINK) == 0) {
        c.memfd = (uint32_t)m; c.flags = UDMABUF_FLAGS_CLOEXEC; c.offset = 0; c.size = len;
        fd = ioctl(u, UDMABUF_CREATE, &c);
    }
    if (fd < 0) snprintf(why + n, wn - n, "%sudmabuf %zu: %s", n ? "; " : "", len, strerror(errno));
    if (m >= 0) close(m);
    close(u);
    return fd;
}

/* len rounded up to whole pages; the map is read/write, shared (the CPU's cached view) */
__attribute__((unused)) static int hs_alloc(hs_buf *b, size_t len, char *why, size_t wn)
{
    const char *s = getenv("HS_SRC");
    long pg = sysconf(_SC_PAGESIZE);
    len = (len + (size_t)pg - 1) / (size_t)pg * (size_t)pg;
    why[0] = 0;
    b->fd = -1; b->map = NULL; b->len = len; b->src = NULL;
    if (!s || !strcmp(s, "heap")) {
        b->fd = hs_heap(len, why, wn);
        if (b->fd >= 0) b->src = "dma-heap system";
    }
    if (b->fd < 0 && (!s || !strcmp(s, "udmabuf"))) {
        b->fd = hs_udmabuf(len, why, wn);
        if (b->fd >= 0) b->src = "udmabuf stand-in";
    }
    if (b->fd < 0) { if (!why[0]) snprintf(why, wn, "HS_SRC=%s: unknown", s); return -1; }
    void *p = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, b->fd, 0);
    if (p == MAP_FAILED) { snprintf(why, wn, "mmap of the %s: %s", b->src, strerror(errno)); close(b->fd); b->fd = -1; return -1; }
    b->map = p;
    hs_last = b->src;
    return 0;
}

__attribute__((unused)) static void hs_free(hs_buf *b)
{
    if (b->map) munmap(b->map, b->len);
    if (b->fd >= 0) close(b->fd);
    b->map = NULL; b->fd = -1;
}

/* DMA_BUF_IOCTL_SYNC (flags: DMA_BUF_SYNC_START|END | READ|WRITE|RW); EINTR/EAGAIN retried */
__attribute__((unused)) static int hs_sync(int fd, unsigned flags)
{
    struct dma_buf_sync s = { flags };
    int r;
    do r = ioctl(fd, DMA_BUF_IOCTL_SYNC, &s); while (r < 0 && (errno == EINTR || errno == EAGAIN));
    return r;
}

#ifdef HS_EGL
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl31.h>
#include <GLES2/gl2ext.h>
#ifndef EGL_LINUX_DMA_BUF_EXT
#define EGL_LINUX_DMA_BUF_EXT 0x3270
#endif
static PFNEGLCREATEIMAGEKHRPROC hs_eglCreateImageKHR;
static PFNEGLDESTROYIMAGEKHRPROC hs_eglDestroyImageKHR;
static PFNGLEGLIMAGETARGETTEXTURE2DOESPROC hs_glEGLImageTargetTexture2DOES;
typedef void (GL_APIENTRY *hs_texstor_fn)(GLenum, GLeglImageOES, const GLint *);
static hs_texstor_fn hs_glEGLImageTargetTexStorageEXT;  /* (NULL: no GL_EXT_EGL_image_storage) */

/* the import's entry points (an existing context current); 0 ok */
__attribute__((unused)) static int hs_egl_fns(void)
{
    const char *ex = (const char *)glGetString(GL_EXTENSIONS);
    hs_eglCreateImageKHR = (PFNEGLCREATEIMAGEKHRPROC)eglGetProcAddress("eglCreateImageKHR");
    hs_eglDestroyImageKHR = (PFNEGLDESTROYIMAGEKHRPROC)eglGetProcAddress("eglDestroyImageKHR");
    hs_glEGLImageTargetTexture2DOES = (PFNGLEGLIMAGETARGETTEXTURE2DOESPROC)eglGetProcAddress("glEGLImageTargetTexture2DOES");
    if (ex && strstr(ex, "GL_EXT_EGL_image_storage"))
        hs_glEGLImageTargetTexStorageEXT = (hs_texstor_fn)eglGetProcAddress("glEGLImageTargetTexStorageEXT");
    return hs_eglCreateImageKHR && hs_eglDestroyImageKHR && hs_glEGLImageTargetTexture2DOES ? 0 : -1;
}

/* a surfaceless GLES 3.1 context (current); 0 ok, the display in *dp */
__attribute__((unused)) static int hs_egl_open(EGLDisplay *dp)
{
    PFNEGLGETPLATFORMDISPLAYEXTPROC gpd = (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");
    EGLDisplay d = gpd ? gpd(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, NULL) : EGL_NO_DISPLAY;
    if (d == EGL_NO_DISPLAY || !eglInitialize(d, NULL, NULL)) return -1;
    eglBindAPI(EGL_OPENGL_ES_API);
    static const EGLint ca[] = { EGL_CONTEXT_MAJOR_VERSION, 3, EGL_CONTEXT_MINOR_VERSION, 1, EGL_NONE };
    EGLContext c = eglCreateContext(d, EGL_NO_CONFIG_KHR, EGL_NO_CONTEXT, ca);
    if (c == EGL_NO_CONTEXT || !eglMakeCurrent(d, EGL_NO_SURFACE, EGL_NO_SURFACE, c)) return -1;
    *dp = d;
    return hs_egl_fns();
}

/* the dma-buf as an EGLImage of fourcc, w x h texels, pitch bytes per row at offset off
 * (EGL_NO_IMAGE_KHR: refused, eglGetError in *err) */
__attribute__((unused)) static EGLImageKHR hs_import(EGLDisplay d, int fd, uint32_t fourcc, int w, int h, int pitch, int off, EGLint *err)
{
    const EGLint a[] = {
        EGL_WIDTH, w, EGL_HEIGHT, h, EGL_LINUX_DRM_FOURCC_EXT, (EGLint)fourcc,
        EGL_DMA_BUF_PLANE0_FD_EXT, fd, EGL_DMA_BUF_PLANE0_OFFSET_EXT, off,
        EGL_DMA_BUF_PLANE0_PITCH_EXT, pitch, EGL_NONE };
    EGLImageKHR im = hs_eglCreateImageKHR(d, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, NULL, a);
    if (err) *err = im == EGL_NO_IMAGE_KHR ? eglGetError() : EGL_SUCCESS;
    return im;
}

/* a sampler texture of the image (GL_NEAREST, one level: 's incomplete-texture trap) */
__attribute__((unused)) static GLuint hs_tex_sample(EGLImageKHR im)
{
    GLuint t;
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    hs_glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, (GLeglImageOES)im);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
    return t;
}

/* an immutable-storage texture of the image (image stores; a fresh texture object, as
 * epd_gpu_target_dmabuf); 0 if the extension is missing or the call raised an error */
__attribute__((unused)) static GLuint hs_tex_storage(EGLImageKHR im, GLenum *glerr)
{
    if (glerr) *glerr = GL_NO_ERROR;
    if (!hs_glEGLImageTargetTexStorageEXT) return 0;
    while (glGetError() != GL_NO_ERROR) {}
    GLuint t;
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    hs_glEGLImageTargetTexStorageEXT(GL_TEXTURE_2D, (GLeglImageOES)im, NULL);
    GLenum e = glGetError();
    if (e != GL_NO_ERROR) { if (glerr) *glerr = e; glDeleteTextures(1, &t); return 0; }
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    return t;
}
#endif
#endif
