/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-waveform: which waveform file the panel tools open.
 *
 * The device adapter (/etc/glassine/epd.conf, shipped by the device package)
 * names it with waveform=, as glassine-epdd reads it. glassine-epdd-desktop and
 * glassine-epd-show use the same key through this header, in this order:
 *   1. GLASSINE_EPDD_WBF in the environment (tests, bring-up);
 *   2. waveform= in the adapter file;
 *   3. EPD_WAVEFORM_DEFAULT, the path older images put the file at; also when
 *      the adapter names a file that can't be read but this one can (a device
 *      seed that failed on an image that still has the file: the panel keeps
 *      working, the health check still reports the missing seed).
 * The adapter line is parsed like glassine-epdd's: "key=value", "#" starts a
 * comment, trailing blanks and the line end are dropped; the last waveform=
 * wins. An empty value counts as not set.
 *
 * Header only: glassine-epdd-desktop.c, glassine-epd-show.c and the host test
 * (src/test/epd-waveform-test.c) include it. The lookup takes the adapter path
 * as an argument so the test can use a scratch file. */
#ifndef EPD_WAVEFORM_H
#define EPD_WAVEFORM_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define EPD_ADAPTER_CONF "/etc/glassine/epd.conf"
#ifndef EPD_WAVEFORM_DEFAULT          /* (a test build names a scratch file) */
#define EPD_WAVEFORM_DEFAULT "/waveform/eink_waveform.wbf"
#endif

/* waveform= from the adapter file `conf` into out (n bytes). 1 if found and
 * not empty, 0 otherwise (no file, no key, empty value, too long for out). */
static inline int epd_adapter_waveform(const char *conf, char *out, size_t n)
{
    FILE *f = fopen(conf, "r");
    if (!f) return 0;
    char l[512];
    int found = 0;
    while (fgets(l, sizeof l, f)) {
        char *v = strchr(l, '=');
        if (*l == '#' || !v) continue;
        *v++ = 0;
        if (strcmp(l, "waveform")) continue;
        v[strcspn(v, "\r\n#")] = 0;
        for (char *e = v + strlen(v); e > v && (e[-1] == ' ' || e[-1] == '\t');) *--e = 0;
        size_t k = strlen(v);
        if (k && k < n) { memcpy(out, v, k + 1); found = 1; }
        else found = 0;
    }
    fclose(f);
    return found;
}

/* the waveform path by the order above, into out (n bytes); returns out */
static inline const char *epd_waveform_path(const char *conf, char *out, size_t n)
{
    const char *e = getenv("GLASSINE_EPDD_WBF");
    if (e && *e && strlen(e) < n) { memcpy(out, e, strlen(e) + 1); return out; }
    if (epd_adapter_waveform(conf, out, n)) {
        if (!access(out, R_OK) || access(EPD_WAVEFORM_DEFAULT, R_OK)) return out;
        fprintf(stderr, "epd: no waveform at %s (%s), using %s\n", out, conf, EPD_WAVEFORM_DEFAULT);
    }
    snprintf(out, n, "%s", EPD_WAVEFORM_DEFAULT);
    return out;
}
#endif
