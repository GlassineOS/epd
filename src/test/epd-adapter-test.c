/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-adapter-test: the device adapter (src/epd-adapter.h) against what it
 * replaced, with scratch files for /etc/glassine/epd.conf and the sysfs files:
 *   1. the waveform path: environment, then waveform=, then the default
 *      (the checks of epd-waveform-test, which tested src/epd-waveform.h)
 *   2. random files: the fields glassine-epdd parsed == a frozen copy of its
 *      load_adapter (at 1ea7dab, ref_load below), for every key the file names;
 *      the others are the defaults
 *   3. the temperature == the desktop daemon's and the splash's battery_celsius
 *      (frozen, with its two files as arguments) for the defaults, and
 *      glassine-epdd's rules for a file's own list and scale
 *   4. pen and touch: the defaults match evdev names as the literals they
 *      replaced did (2D1F:012B, turned; cyttsp5); a file's lists and "invert"
 *   5. the sample conf (argument 1, conf/epd.conf.onyx-noteair) == the defaults
 *      but for its waveform
 * epd-adapter-test [SAMPLE.conf] */
#include "../epd-adapter.h"
#include <sys/stat.h>
#include <unistd.h>

static int fails;
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)

static char dir[200], conf[240];

static void put(const char *s) { FILE *f = fopen(conf, "w"); if (f) { fputs(s, f); fclose(f); } }

static epd_adapter ad;
static const epd_adapter *load(void) { epd_adapter_load(&ad, conf); return &ad; }

static const char *look(void)
{
    static char p[256];
    epd_adapter_load(&ad, conf);
    return epd_adapter_waveform(&ad, p, sizeof p);
}

/* ---- frozen: glassine-epdd.c's adapter at 1ea7dab ---- */
enum { R_NONE, R_CCW, R_CW };
static struct {
    int width, height, rot, temp_scale;
    char waveform[256], temp[4][256], light[4][256], sleep_text[64];
} ref;
static int ref_load(const char *path)
{
    memset(&ref, 0, sizeof ref);
    ref.width = 1872; ref.height = 1404; ref.rot = R_CCW; ref.temp_scale = 10;
    snprintf(ref.waveform, sizeof ref.waveform, "%s", "/tmp/eink_waveform.wbf");
    snprintf(ref.sleep_text, sizeof ref.sleep_text, "%s", "PRESS POWER TO WAKE");
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    char l[512];
    int nt = 0, nl = 0;
    while (fgets(l, sizeof l, f)) {
        char *v = strchr(l, '=');
        if (*l == '#' || !v) continue;
        *v++ = 0;
        v[strcspn(v, "\r\n#")] = 0;
        for (char *e = v + strlen(v); e > v && (e[-1] == ' ' || e[-1] == '\t');) *--e = 0;
        if (!strcmp(l, "panel_width")) ref.width = atoi(v);
        else if (!strcmp(l, "panel_height")) ref.height = atoi(v);
        else if (!strcmp(l, "rotation")) ref.rot = !strcmp(v, "cw") ? R_CW : !strcmp(v, "none") ? R_NONE : R_CCW;
        else if (!strcmp(l, "waveform")) snprintf(ref.waveform, sizeof ref.waveform, "%s", v);
        else if (!strcmp(l, "temperature") && nt < 4) snprintf(ref.temp[nt++], sizeof ref.temp[0], "%s", v);
        else if (!strcmp(l, "temperature_scale")) ref.temp_scale = atoi(v) > 0 ? atoi(v) : 1;
        else if (!strcmp(l, "frontlight") && nl < 4) snprintf(ref.light[nl++], sizeof ref.light[0], "%s", v);
        else if (!strcmp(l, "sleep_text")) snprintf(ref.sleep_text, sizeof ref.sleep_text, "%s", v);
    }
    fclose(f);
    return 0;
}
static int ref_celsius(void)
{
    for (int i = 0; i < 4 && *ref.temp[i]; i++) {
        FILE *f = fopen(ref.temp[i], "r");
        char b[32];
        if (!f) continue;
        int ok = fgets(b, sizeof b, f) != NULL;
        fclose(f);
        if (ok) return atoi(b) / ref.temp_scale;
    }
    return 24;
}

/* ---- frozen: glassine-epdd-desktop.c / glassine-epd-show.c battery_celsius at
 * 1ea7dab, its two /sys/class/power_supply files as arguments ---- */
static int ref_battery_celsius(const char *p0, const char *p1)
{
    const char *p[] = { p0, p1 };
    for (unsigned i = 0; i < sizeof p / sizeof *p; i++) {
        FILE *f = fopen(p[i], "r");
        char b[32];
        if (!f) continue;
        int ok = fgets(b, sizeof b, f) != NULL;
        fclose(f);
        if (ok) return atoi(b) / 10;
    }
    return 24;
}

static void file(const char *path, const char *s)
{
    if (!s) { unlink(path); return; }
    FILE *f = fopen(path, "w");
    if (f) { fputs(s, f); fclose(f); }
}

static unsigned rnd = 12345;
static unsigned rn(unsigned n) { rnd = rnd * 1103515245u + 12345u; return (rnd >> 16) % n; }

static void waveform_path(void)
{
    unsetenv("GLASSINE_EPDD_WBF");
    /* no adapter file: the default */
    unlink(conf);
    CHECK(!strcmp(look(), EPD_AD_WAVEFORM));
    CHECK(!strcmp(look(), "/waveform/eink_waveform.wbf"));      /* the path older images put it at */

    /* the adapter's key, with a comment, blanks and other keys around it */
    put("# panel\npanel_width=1872\nwaveform=/run/glassine/device/waveform.wbf  # seeded\nrotation=ccw\n");
    CHECK(!strcmp(look(), "/run/glassine/device/waveform.wbf"));

    /* CRLF line ends; the last waveform= wins */
    put("waveform=/a.wbf\r\nwaveform=/b.wbf\r\n");
    CHECK(!strcmp(look(), "/b.wbf"));

    /* a commented-out key, a key that only starts the same, an empty value: not set */
    put("#waveform=/x.wbf\nwaveform_source=system_a:/w.wbf\n");
    CHECK(!strcmp(look(), EPD_AD_WAVEFORM));
    put("waveform=\n");
    CHECK(!strcmp(look(), EPD_AD_WAVEFORM));
    put("waveform=/a.wbf\nwaveform=   \n");
    CHECK(!strcmp(look(), EPD_AD_WAVEFORM));

    /* too long to keep: the default; too long for the caller's buffer: cut */
    {
        char l[400] = "waveform=/", small[8];
        memset(l + 10, 'w', 300);
        strcpy(l + 310, "\n");
        put(l);
        CHECK(!strcmp(look(), EPD_AD_WAVEFORM));
        put("waveform=/run/glassine/device/waveform.wbf\n");
        load();
        CHECK(!strcmp(epd_adapter_waveform(&ad, small, sizeof small), "/run/gl"));
    }

    /* the environment wins over the adapter; empty counts as not set */
    put("waveform=/conf.wbf\n");
    setenv("GLASSINE_EPDD_WBF", "/env.wbf", 1);
    CHECK(!strcmp(look(), "/env.wbf"));
    setenv("GLASSINE_EPDD_WBF", "", 1);
    CHECK(!strcmp(look(), "/conf.wbf"));
    unsetenv("GLASSINE_EPDD_WBF");
}

/* glassine-epdd's fields over random files, against its old parser. Values of
 * list keys and waveform= are never empty here: an empty one now empties the
 * list / means the default (checked in lists()) */
static void vs_epdd(int rounds)
{
    static const char *keys[] = { "panel_width", "panel_height", "rotation", "waveform", "temperature",
                                  "temperature_scale", "frontlight", "sleep_text", "pen", "touch",
                                  "waveform_source", "#waveform", "rotation ", "Temperature", "" };
    static const char *vals[] = { "1872", "1404", "0", "-3", "abc", "ccw", "cw", "none", "CW", "/a/b", "/x y",
                                  "/sys/t  ", "7 # c", "PRESS POWER", "10\r", "2D1F:012B invert", "x\t" };
    static const char *ends[] = { "\n", "\r\n", "  \n", " # note\n" };
    int compared = 0;
    for (int r = 0; r < rounds; r++) {
        char text[4096] = "";
        int has[16] = { 0 }, n = rn(12);
        for (int i = 0; i < n; i++) {
            int k = rn(sizeof keys / sizeof *keys);
            char line[256];
            if (rn(10) == 0) snprintf(line, sizeof line, "# a comment %d\n", i);
            else if (rn(10) == 0) snprintf(line, sizeof line, "no equals sign\n");
            else {
                snprintf(line, sizeof line, "%s=%s%s", keys[k], vals[rn(sizeof vals / sizeof *vals)], ends[rn(4)]);
                has[k] = 1;
            }
            strncat(text, line, sizeof text - strlen(text) - 1);
        }
        put(text);
        CHECK(!ref_load(conf));
        load();
        int f0 = fails;
        CHECK(ad.width == ref.width && ad.height == ref.height && ad.rot == ref.rot);
        CHECK(ad.temp_scale == ref.temp_scale);
        CHECK(!strcmp(ad.sleep_text, ref.sleep_text));
        if (has[3]) CHECK(!strcmp(ad.waveform, ref.waveform));
        else CHECK(!strcmp(ad.waveform, EPD_AD_WAVEFORM));
        if (has[4]) CHECK(!memcmp(ad.temp, ref.temp, sizeof ad.temp));
        else CHECK(!strcmp(ad.temp[0], "/sys/class/power_supply/qcom-battery/temp"));
        CHECK(!memcmp(ad.light, ref.light, sizeof ad.light));
        if (has[4]) CHECK(epd_adapter_celsius(&ad, NULL) == ref_celsius());
        if (fails != f0) { fprintf(stderr, "  the file:\n%s", text); return; }
        compared++;
    }
    printf("epd-adapter: %d random files: glassine-epdd's fields as its old parser read them\n", compared);
}

static void temperature(void)
{
    char t0[240], t1[240], t2[240], text[1024];
    snprintf(t0, sizeof t0, "%s/qcom-battery-temp", dir);
    snprintf(t1, sizeof t1, "%s/battery-temp", dir);
    snprintf(t2, sizeof t2, "%s/thermal", dir);
    /* the defaults: the battery's two files, tenths: as battery_celsius read them */
    static const char *v[] = { NULL, "", "235\n", "-75\n", "9\n", "301", "abc\n", "1000000\n" };
    int n = sizeof v / sizeof *v;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            file(t0, v[i]);
            file(t1, v[j]);
            epd_adapter_defaults(&ad);
            snprintf(ad.temp[0], sizeof ad.temp[0], "%s", t0);      /* (the sysfs files, here) */
            snprintf(ad.temp[1], sizeof ad.temp[1], "%s", t1);
            const char *from;
            int c = epd_adapter_celsius(&ad, &from);
            CHECK(c == ref_battery_celsius(t0, t1));
            CHECK(from == (v[i] && *v[i] ? ad.temp[0] : v[j] && *v[j] ? ad.temp[1] : NULL));
        }
    /* a file's own: replaces the battery's, in order, with its scale */
    file(t0, "40\n"); file(t1, "50\n"); file(t2, "31250\n");
    snprintf(text, sizeof text, "temperature=%s\ntemperature_scale=1000\ntemperature=%s\n", t2, t0);
    put(text);
    load();
    CHECK(!strcmp(ad.temp[0], t2) && !strcmp(ad.temp[1], t0) && !*ad.temp[2]);
    CHECK(epd_adapter_celsius(&ad, NULL) == 31);
    file(t2, NULL);
    CHECK(epd_adapter_celsius(&ad, NULL) == 0);                 /* 40 / 1000 */
    /* scale 0 or not a number: 1, as glassine-epdd had it */
    snprintf(text, sizeof text, "temperature=%s\ntemperature_scale=0\n", t0);
    put(text);
    CHECK(epd_adapter_celsius(load(), NULL) == 40);
    /* nothing reads: 24 */
    snprintf(text, sizeof text, "temperature=%s/none\n", dir);
    put(text);
    const char *from = "x";
    CHECK(epd_adapter_celsius(load(), &from) == 24 && !from);
    put("temperature=\n");
    CHECK(!*load()->temp[0] && epd_adapter_celsius(&ad, NULL) == EPD_AD_CELSIUS);
    unlink(t0); unlink(t1);
}

static void lists(void)
{
    /* evdev names the daemon met, and what the literals said of them */
    static const struct { const char *name; int pen, flip, touch; } names[] = {
        { "WACF2200:00 2D1F:012B", 1, 1, 0 },
        { "WACF2200:00 2D1F:012B Stylus", 1, 1, 0 },
        { "Glassine Pen", 0, 0, 0 },
        { "cyttsp5_mt", 0, 0, 1 },
        { "Glassine Touch", 0, 0, 0 },
        { "gpio-keys", 0, 0, 0 },
        { "2D1F:012C", 0, 0, 0 },
        { "", 0, 0, 0 },
    };
    unlink(conf);
    load();
    for (unsigned i = 0; i < sizeof names / sizeof *names; i++) {
        int inv = -1;
        CHECK(epd_adapter_pen(&ad, names[i].name, &inv) == (strstr(names[i].name, "2D1F:012B") != NULL));
        CHECK(epd_adapter_pen(&ad, names[i].name, &inv) == names[i].pen && inv == names[i].flip);
        CHECK(epd_adapter_touch(&ad, names[i].name) == (strstr(names[i].name, "cyttsp5") != NULL));
        CHECK(epd_adapter_touch(&ad, names[i].name) == names[i].touch);
    }
    CHECK(epd_adapter_pen(&ad, "x", NULL) == 0);

    /* a file's pens: replace the default, in order; " invert" after blanks, comments */
    put("pen=Wacom I2C Digitizer\npen=ELAN 2514:0001   invert  # turned\npen=invert\npen=a\tinvert\npen=X\n");
    load();
    CHECK(!strcmp(ad.pen[0], "Wacom I2C Digitizer") && !ad.pen_invert[0]);
    CHECK(!strcmp(ad.pen[1], "ELAN 2514:0001") && ad.pen_invert[1]);
    CHECK(!strcmp(ad.pen[2], "invert") && !ad.pen_invert[2]);   /* only a name */
    CHECK(!strcmp(ad.pen[3], "a") && ad.pen_invert[3]);
    put("pen=Reinvert\n");                                     /* "invert" is a word of its own */
    CHECK(!strcmp(load()->pen[0], "Reinvert") && !ad.pen_invert[0]);
    put("pen=Wacom I2C Digitizer\npen=ELAN 2514:0001 invert\n");
    load();
    int inv = -1;
    CHECK(!epd_adapter_pen(&ad, "WACF2200:00 2D1F:012B", &inv) && inv == 0);
    CHECK(epd_adapter_pen(&ad, "ELAN 2514:0001 Stylus", &inv) && inv == 1);
    CHECK(epd_adapter_pen(&ad, "Wacom I2C Digitizer Pen", &inv) && inv == 0);
    CHECK(!strcmp(ad.touch[0], "cyttsp5"));                     /* (touch= not named: its default) */

    /* more than EPD_AD_N: the first ones; too long: left out */
    put("touch=A\ntouch=B\ntouch=C\ntouch=D\ntouch=E\n"
        "pen=0123456789012345678901234567890123456789012345678901234567890123456789\npen=p\n");
    load();
    CHECK(!strcmp(ad.touch[3], "D") && epd_adapter_touch(&ad, "xDx") && !epd_adapter_touch(&ad, "E"));
    CHECK(!strcmp(ad.pen[0], "p") && !*ad.pen[1] && !epd_adapter_touch(&ad, "cyttsp5_mt"));

    /* an empty value empties the list: no adapter pen or touchscreen */
    put("pen=\ntouch=\nfrontlight=\n");
    load();
    CHECK(!*ad.pen[0] && !*ad.touch[0] && !*ad.light[0]);
    CHECK(!epd_adapter_pen(&ad, "WACF2200:00 2D1F:012B", NULL) && !epd_adapter_touch(&ad, "cyttsp5_mt"));
    /* and the lines after it fill it again */
    put("pen=x invert\npen=\npen=y\ntouch=t\ntouch=\n");
    load();
    CHECK(!strcmp(ad.pen[0], "y") && !ad.pen_invert[0] && !*ad.pen[1] && !*ad.touch[0]);
}

int main(int argc, char **argv)
{
    snprintf(dir, sizeof dir, "%s/epd-adapter-XXXXXX", getenv("TMPDIR") ? getenv("TMPDIR") : "/tmp");
    if (!mkdtemp(dir)) { perror("mkdtemp"); return 1; }
    snprintf(conf, sizeof conf, "%s/epd.conf", dir);

    waveform_path();
    vs_epdd(20000);
    temperature();
    lists();

    /* no file: -1 and the defaults, which are the Note Air's */
    unlink(conf);
    epd_adapter d;
    epd_adapter_defaults(&d);
    CHECK(epd_adapter_load(&ad, conf) == -1 && !memcmp(&ad, &d, sizeof d));
    CHECK(d.width == 1872 && d.height == 1404 && d.rot == EPD_ROT_CCW && d.temp_scale == 10);

    /* the sample: every key written out, the defaults but for the waveform */
    if (argc > 1) {
        CHECK(!epd_adapter_load(&ad, argv[1]));
        CHECK(!strcmp(ad.waveform, "/run/glassine/device/waveform.wbf"));
        memcpy(ad.waveform, d.waveform, sizeof d.waveform);
        CHECK(!memcmp(ad.light[0], "/sys/class/backlight/frontlight-a/brightness", 45));
        memset(ad.light, 0, sizeof ad.light);
        CHECK(!memcmp(&ad, &d, sizeof d));
    }

    rmdir(dir);
    if (fails) return 1;
    printf("epd-adapter: ok\n");
    return 0;
}
