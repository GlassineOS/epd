/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-waveform-test: the waveform lookup of glassine-epdd-desktop and
 * glassine-epd-show (src/epd-waveform.h): environment, then the adapter's
 * waveform=, then the old default path. Scratch files stand in for
 * /etc/glassine/epd.conf and (built with -DEPD_WAVEFORM_DEFAULT=...) for the
 * old path. */
#include "../epd-waveform.h"
#include <unistd.h>

static int fails;
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)

static char conf[300];

static void put(const char *s) { FILE *f = fopen(conf, "w"); if (f) { fputs(s, f); fclose(f); } }

static const char *look(void)
{
    static char p[256];
    return epd_waveform_path(conf, p, sizeof p);
}

int main(void)
{
    char dir[256];
    snprintf(dir, sizeof dir, "%s/epd-waveform-XXXXXX", getenv("TMPDIR") ? getenv("TMPDIR") : "/tmp");
    if (!mkdtemp(dir)) { perror("mkdtemp"); return 1; }
    snprintf(conf, sizeof conf, "%s/epd.conf", dir);
    unsetenv("GLASSINE_EPDD_WBF");

    /* no adapter file: the old path */
    CHECK(!strcmp(look(), EPD_WAVEFORM_DEFAULT));

    /* the adapter's key, with a comment, blanks and other keys around it */
    put("# panel\npanel_width=1872\nwaveform=/run/glassine/device/waveform.wbf  # seeded\nrotation=ccw\n");
    CHECK(!strcmp(look(), "/run/glassine/device/waveform.wbf"));

    /* CRLF line ends; the last waveform= wins */
    put("waveform=/a.wbf\r\nwaveform=/b.wbf\r\n");
    CHECK(!strcmp(look(), "/b.wbf"));

    /* a commented-out key, a key that only starts the same, an empty value: not set */
    put("#waveform=/x.wbf\nwaveform_source=system_a:/w.wbf\n");
    CHECK(!strcmp(look(), EPD_WAVEFORM_DEFAULT));
    put("waveform=\n");
    CHECK(!strcmp(look(), EPD_WAVEFORM_DEFAULT));
    put("waveform=/a.wbf\nwaveform=   \n");
    CHECK(!strcmp(look(), EPD_WAVEFORM_DEFAULT));

    /* too long for the caller's buffer: not used */
    {
        char small[8];
        put("waveform=/run/glassine/device/waveform.wbf\n");
        CHECK(!epd_adapter_waveform(conf, small, sizeof small));
    }

    /* the adapter's file missing but the old one there: the old one; both
     * missing: the adapter's (the open then fails with that name) */
    {
        FILE *f = fopen(EPD_WAVEFORM_DEFAULT, "w");
        CHECK(f != NULL);
        if (f) fclose(f);
        put("waveform=/nowhere/waveform.wbf\n");
        CHECK(!strcmp(look(), EPD_WAVEFORM_DEFAULT));
        put("waveform=" EPD_WAVEFORM_DEFAULT "\n");                       /* (readable: as named) */
        CHECK(!strcmp(look(), EPD_WAVEFORM_DEFAULT));
        unlink(EPD_WAVEFORM_DEFAULT);
        put("waveform=/nowhere/waveform.wbf\n");
        CHECK(!strcmp(look(), "/nowhere/waveform.wbf"));
    }

    /* the environment wins over the adapter; empty counts as not set */
    put("waveform=/conf.wbf\n");
    setenv("GLASSINE_EPDD_WBF", "/env.wbf", 1);
    CHECK(!strcmp(look(), "/env.wbf"));
    setenv("GLASSINE_EPDD_WBF", "", 1);
    CHECK(!strcmp(look(), "/conf.wbf"));
    unsetenv("GLASSINE_EPDD_WBF");

    unlink(conf);
    rmdir(dir);
    if (fails) return 1;
    printf("epd-waveform: ok\n");
    return 0;
}
