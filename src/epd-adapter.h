/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-adapter: the device adapter, /etc/glassine/epd.conf, which the device
 * package ships. What the panel programs know about the device comes from
 * it: glassine-epdd, glassine-epdd-desktop, glassine-epd-show and the drive
 * test (src/test/epd-stream-test.c) read it through this header, and the
 * temperature through epd_adapter_celsius.
 *
 * Format: "key=value" lines; "#" starts a comment, also after a value;
 * trailing blanks and the line end are dropped (a value keeps its inner
 * blanks). Unknown keys are ignored: other tools read their own from the
 * same file. The keys:
 *   panel_width, panel_height  the panel's native size (the engine's, so far)
 *   rotation      the portrait surface -> the panel: ccw (default), cw or none
 *   waveform      the waveform file; the last one wins, and an empty one (or
 *                 one too long to keep) is the default. glassine-epdd-desktop
 *                 and glassine-epd-show take GLASSINE_EPDD_WBF before it
 *                 (epd_adapter_waveform)
 *   temperature   files tried in turn; the first that reads gives value /
 *                 temperature_scale degrees C. None reads: 24 C
 *   temperature_scale  the files' unit (default 10: tenths of a degree)
 *   frontlight    brightness files, off while glassine-epdd sleeps
 *   sleep_text    glassine-epdd's asleep bar (capitals, epd-glyphs.h)
 *   pen           the pen digitizer: a part of its evdev name, then " invert"
 *                 if its axes are turned 180 degrees from the panel's. Only
 *                 its node with pressure counts. glassine-penfix's "Glassine
 *                 Pen" is a pen without it
 *   touch         the touchscreen: a part of its evdev name; the first device
 *                 that matches. glassine-touchd's "Glassine Touch" clone is
 *                 taken before it when it exists
 * The list keys (temperature, frontlight, pen, touch) hold up to EPD_AD_N
 * entries, in the file's order: the key's first line replaces its default
 * list, and a line with an empty value empties it ("pen=": no digitizer but
 * glassine-penfix's). An entry past EPD_AD_N or too long to keep is left out.
 *
 * A key the file doesn't name keeps its default below: the Note Air's, the
 * device these programs were written on, so its epd.conf may leave out what
 * it doesn't change (and glassine-epdd-desktop and glassine-epd-show run as
 * before without the file). Another device is its own epd.conf.
 *
 * Header only, like the other helpers the daemons share: other repositories'
 * host tests build glassine-epdd-desktop from a fixed list of sources.
 * epd_adapter_load takes the path, so the host test (src/test/epd-adapter-test.c)
 * uses scratch files. */
#ifndef EPD_ADAPTER_H
#define EPD_ADAPTER_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define EPD_ADAPTER_CONF "/etc/glassine/epd.conf"
#define EPD_AD_N 4                      /* entries of a list key */
#define EPD_AD_CELSIUS 24               /* when no temperature file reads */

enum { EPD_ROT_NONE, EPD_ROT_CCW, EPD_ROT_CW };

typedef struct {
    int width, height, rot, temp_scale;
    char waveform[256];
    char temp[EPD_AD_N][256];           /* a list ends at its first empty entry */
    char light[EPD_AD_N][256];
    char sleep_text[64];
    char pen[EPD_AD_N][64];
    int pen_invert[EPD_AD_N];
    char touch[EPD_AD_N][64];
} epd_adapter;

#define EPD_AD_WAVEFORM "/waveform/eink_waveform.wbf"

/* the defaults: the Onyx Boox Note Air's */
static inline void epd_adapter_defaults(epd_adapter *a)
{
    memset(a, 0, sizeof *a);
    a->width = 1872;
    a->height = 1404;
    a->rot = EPD_ROT_CCW;
    snprintf(a->waveform, sizeof a->waveform, "%s", EPD_AD_WAVEFORM);
    /* no sensor at the panel (the MAX17135 has no thermistor): the battery's, in tenths */
    snprintf(a->temp[0], sizeof a->temp[0], "%s", "/sys/class/power_supply/qcom-battery/temp");
    snprintf(a->temp[1], sizeof a->temp[1], "%s", "/sys/class/power_supply/battery/temp");
    a->temp_scale = 10;
    snprintf(a->sleep_text, sizeof a->sleep_text, "%s", "PRESS POWER TO WAKE");
    /* the mainline i2c-hid Wacom digitizer: the stock kernel's axes turned 180 degrees
     * (top-left tap = raw X~560/Y~15170) */
    snprintf(a->pen[0], sizeof a->pen[0], "%s", "2D1F:012B");
    a->pen_invert[0] = 1;
    snprintf(a->touch[0], sizeof a->touch[0], "%s", "cyttsp5");
}

/* a list key's line into its list (n entries of size sz at list; *k: entries
 * so far, -1 before the key's first line): the slot it went to, or -1 */
static inline int epd_ad_entry(char *list, size_t sz, int *k, const char *v)
{
    if (*k < 0 || !*v) { memset(list, 0, sz * EPD_AD_N); *k = 0; }
    if (!*v || *k >= EPD_AD_N || strlen(v) >= sz) return -1;
    memcpy(list + sz * *k, v, strlen(v) + 1);
    return (*k)++;
}

/* the lines of f over what a holds (epd_adapter_defaults first) */
static inline void epd_adapter_parse(epd_adapter *a, FILE *f)
{
    char l[512];
    int nt = -1, nl = -1, np = -1, nc = -1;
    while (fgets(l, sizeof l, f)) {
        char *v = strchr(l, '=');
        if (*l == '#' || !v) continue;
        *v++ = 0;
        v[strcspn(v, "\r\n#")] = 0;
        for (char *e = v + strlen(v); e > v && (e[-1] == ' ' || e[-1] == '\t');) *--e = 0;
        if (!strcmp(l, "panel_width")) a->width = atoi(v);
        else if (!strcmp(l, "panel_height")) a->height = atoi(v);
        else if (!strcmp(l, "rotation")) a->rot = !strcmp(v, "cw") ? EPD_ROT_CW : !strcmp(v, "none") ? EPD_ROT_NONE : EPD_ROT_CCW;
        else if (!strcmp(l, "waveform"))
            snprintf(a->waveform, sizeof a->waveform, "%s", *v && strlen(v) < sizeof a->waveform ? v : EPD_AD_WAVEFORM);
        else if (!strcmp(l, "temperature")) epd_ad_entry(a->temp[0], sizeof a->temp[0], &nt, v);
        else if (!strcmp(l, "temperature_scale")) a->temp_scale = atoi(v) > 0 ? atoi(v) : 1;
        else if (!strcmp(l, "frontlight")) epd_ad_entry(a->light[0], sizeof a->light[0], &nl, v);
        else if (!strcmp(l, "sleep_text")) snprintf(a->sleep_text, sizeof a->sleep_text, "%s", v);
        else if (!strcmp(l, "touch")) epd_ad_entry(a->touch[0], sizeof a->touch[0], &nc, v);
        else if (!strcmp(l, "pen")) {
            size_t k = strlen(v);
            int inv = k > 7 && !strcmp(v + k - 6, "invert") && (v[k - 7] == ' ' || v[k - 7] == '\t');
            if (inv)
                for (char *e = v + k - 6; e > v && (e[-1] == ' ' || e[-1] == '\t');) *--e = 0;
            if (np < 0 || !*v) memset(a->pen_invert, 0, sizeof a->pen_invert);
            int i = epd_ad_entry(a->pen[0], sizeof a->pen[0], &np, v);
            if (i >= 0) a->pen_invert[i] = inv;
        }
    }
}

/* the defaults, then the file at path over them. 0, or -1 if it can't be
 * opened (a holds the defaults) */
static inline int epd_adapter_load(epd_adapter *a, const char *path)
{
    epd_adapter_defaults(a);
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    epd_adapter_parse(a, f);
    fclose(f);
    return 0;
}

/* the temperature in C: the first temperature file that reads, divided by
 * temperature_scale; EPD_AD_CELSIUS if none does. *from (may be NULL): the
 * file it came from, or NULL */
static inline int epd_adapter_celsius(const epd_adapter *a, const char **from)
{
    if (from) *from = NULL;
    for (int i = 0; i < EPD_AD_N && *a->temp[i]; i++) {
        FILE *f = fopen(a->temp[i], "r");
        char b[32];
        if (!f) continue;
        int ok = fgets(b, sizeof b, f) != NULL;
        fclose(f);
        if (ok) {
            if (from) *from = a->temp[i];
            return atoi(b) / (a->temp_scale > 0 ? a->temp_scale : 1);
        }
    }
    return EPD_AD_CELSIUS;
}

/* the waveform file of glassine-epdd-desktop and glassine-epd-show:
 * GLASSINE_EPDD_WBF if set and not empty (tests, bring-up), else the
 * adapter's. Into out (n bytes); returns out */
static inline const char *epd_adapter_waveform(const epd_adapter *a, char *out, size_t n)
{
    const char *e = getenv("GLASSINE_EPDD_WBF");
    snprintf(out, n, "%s", e && *e && strlen(e) < n ? e : a->waveform);
    return out;
}

/* 1 if the evdev name is the adapter's pen (it holds a pen= entry);
 * *invert (may be NULL): the first matching entry's flag, else 0 */
static inline int epd_adapter_pen(const epd_adapter *a, const char *name, int *invert)
{
    if (invert) *invert = 0;
    for (int i = 0; i < EPD_AD_N && *a->pen[i]; i++)
        if (strstr(name, a->pen[i])) { if (invert) *invert = a->pen_invert[i]; return 1; }
    return 0;
}

/* 1 if the evdev name is the adapter's touchscreen (it holds a touch= entry) */
static inline int epd_adapter_touch(const epd_adapter *a, const char *name)
{
    for (int i = 0; i < EPD_AD_N && *a->touch[i]; i++)
        if (strstr(name, a->touch[i])) return 1;
    return 0;
}
#endif
