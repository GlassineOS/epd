/* epd-group: the one group whose members may drive the desktop daemon: the
 * control FIFO, the profile directory (epd-profpath.h), eink.conf and the
 * compositor socket (epd-wl.c) belong to it, and the socket also accepts its
 * members. Named once, here: $GLASSINE_EPD_GROUP, else "video". Set but empty,
 * or a name the system doesn't know: no group (those files stay root's, the
 * socket takes root and the daemon's own user only).
 *
 * Header only: glassine-epdd-desktop.c, epd-wl.c and the host test
 * (src/test/epd-group-test.c) include it. */
#ifndef EPD_GROUP_H
#define EPD_GROUP_H
#include <grp.h>
#include <stdlib.h>
#include <sys/types.h>

#define EPD_GROUP_ENV "GLASSINE_EPD_GROUP"
#define EPD_GROUP_DEFAULT "video"

/* the group's name ("" : none) */
static inline const char *epd_group_name(void)
{
    const char *n = getenv(EPD_GROUP_ENV);
    return n ? n : EPD_GROUP_DEFAULT;
}

/* the group's gid, or `none` when there is no such group */
static inline gid_t epd_group(gid_t none)
{
    const char *n = epd_group_name();
    struct group *g = *n ? getgrnam(n) : NULL;
    return g ? g->gr_gid : none;
}
#endif
