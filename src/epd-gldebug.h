/* epd-gldebug.h: the GL's own words for an error (KHR_debug), in the current
 * context: the first errors go to stderr with the context's name and the
 * thread, so "GL error 0x502" names its call. Errors only, at most 20 a
 * context (GPU drive frames went OFF on a GL error, from nowhere) */
#ifndef EPD_GLDEBUG_H
#define EPD_GLDEBUG_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <EGL/egl.h>
#include <GLES2/gl2ext.h>

static void GL_APIENTRY epd_gldebug_cb(GLenum src, GLenum type, GLuint id, GLenum sev, GLsizei len,
                                       const GLchar *msg, const void *user)
{
    static int n, db = -1;              /* (all contexts: a storm stays short) */
    (void)src; (void)id; (void)sev; (void)len;
    /* freedreno's FD_MESA_DEBUG=shaderdb stats ("... shader: N inst, ... full, ...
     * waves") come through this callback too (type OTHER), and nowhere else once
     * a callback is installed: with that variable set they pass, uncapped
     * (IR3_SHADER_DEBUG=shaderdb is not the switch, FD_MESA_DEBUG is) */
    if (db < 0) db = getenv("FD_MESA_DEBUG") != NULL;
    if (db && type != GL_DEBUG_TYPE_ERROR_KHR && strstr(msg, "shader:")) {
        fprintf(stderr, "%s: %s\n", (const char *)user, msg);
        return;
    }
    if (type != GL_DEBUG_TYPE_ERROR_KHR || n >= 40) return;
    n++;
    fprintf(stderr, "%s: GL says (thread %ld): %s\n", (const char *)user, (long)syscall(SYS_gettid), msg);
}

/* in the context just made current; what: its name ("epd-gpu", "epd-ingest") */
static inline void epd_gldebug_on(const char *what)
{
    PFNGLDEBUGMESSAGECALLBACKKHRPROC cb = (void *)eglGetProcAddress("glDebugMessageCallbackKHR");
    void (GL_APIENTRY *en)(GLenum) = (void *)eglGetProcAddress("glEnable");
    if (!cb || !en) return;
    en(GL_DEBUG_OUTPUT_KHR);
    cb(epd_gldebug_cb, what);
    /* the shaderdb stats above are NOTIFICATION severity (Mesa: SHADER_INFO ->
     * OTHER / NOTIFICATION, on by default in 25.x; host-checked with radeonsi's
     * "Shader Stats"): with FD_MESA_DEBUG set, make sure that severity passes */
    if (getenv("FD_MESA_DEBUG")) {
        PFNGLDEBUGMESSAGECONTROLKHRPROC ctl = (void *)eglGetProcAddress("glDebugMessageControlKHR");
        if (ctl) ctl(GL_DONT_CARE, GL_DONT_CARE, GL_DEBUG_SEVERITY_NOTIFICATION_KHR, 0, NULL, GL_TRUE);
    }
}

#endif
