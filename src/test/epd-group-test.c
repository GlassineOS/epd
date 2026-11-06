/* epd-group-test — the one group of the desktop daemon's FIFO, profile directory,
 * eink.conf and compositor socket (epd-group.h): $GLASSINE_EPD_GROUP, else video;
 * empty or unknown: none. exit 0 = pass */
#include "../epd-group.h"
#include <stdio.h>
#include <string.h>

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; fprintf(stderr, "FAIL: " __VA_ARGS__); fputc('\n', stderr); } } while (0)

int main(void)
{
    unsetenv(EPD_GROUP_ENV);
    CHECK(!strcmp(epd_group_name(), "video"), "default name '%s' (want video)", epd_group_name());
    struct group *v = getgrnam("video");
    CHECK(epd_group(77) == (v ? v->gr_gid : 77), "default gid is not video's (%s)", v ? "present" : "absent");

    struct group *r = getgrgid(0);          /* a group every system has: gid 0 */
    if (r) {
        setenv(EPD_GROUP_ENV, r->gr_name, 1);
        CHECK(!strcmp(epd_group_name(), r->gr_name) && epd_group(77) == 0, "named '%s': gid %u (want 0)", r->gr_name, (unsigned)epd_group(77));
    }
    setenv(EPD_GROUP_ENV, "glassine-no-such-group", 1);
    CHECK(epd_group(77) == 77 && epd_group((gid_t)-1) == (gid_t)-1, "an unknown group gave a gid");
    setenv(EPD_GROUP_ENV, "", 1);
    CHECK(epd_group(77) == 77, "an empty name gave a gid (video's, by the default?)");

    if (fails) { fprintf(stderr, "epd-group: %d failures\n", fails); return 1; }
    printf("epd-group: ok\n");
    return 0;
}
