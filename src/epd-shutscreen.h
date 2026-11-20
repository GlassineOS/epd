/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-shutscreen — the shutdown screen's inputs for glassine-epdd-desktop, pure (no
 * panel, no policy) so the host test reads them as the daemon does:
 * - the control line "shutdown-screen [TEXT]": TEXT, the fallback bar's line, is
 * used only if it is capitals and blanks ([A-Z ], at least one letter, at most
 * EPD_SHUT_TEXT_MAX after the blanks around it are taken off); anything else: none
 * - the picture EPD_SHUT_PATH: a binary PGM (P5), maxval 255, exactly the daemon's
 * logical screen (portrait W x H, turned as the person sees it: the frame of the
 * sleep pill's "# at X Y"), rows top to bottom. Comments in the header are allowed;
 * bytes past the picture are ignored.
 * Header only: glassine-epdd-desktop.c and test/epd-shutscreen-test.c include it. */
#ifndef EPD_SHUTSCREEN_H
#define EPD_SHUTSCREEN_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define EPD_SHUT_PATH "/run/glassine-shutdown-screen.pgm"
#define EPD_SHUT_WORD "shutdown-screen"
#define EPD_SHUT_TEXT_MAX 63

/* the TEXT after the first whole-word "shutdown-screen" in cmd, up to the line's
 * end, into text (n bytes). 1 = usable, 0 = none or not [A-Z ] (text is then "") */
static inline int epd_shut_text(const char *cmd, char *text, size_t n)
{
    const size_t wl = sizeof EPD_SHUT_WORD - 1;
    const char *c = cmd, *t = NULL;
    if (n) text[0] = 0;
    for (; (c = strstr(c, EPD_SHUT_WORD)); c += wl)
        if ((c == cmd || c[-1] == ' ' || c[-1] == '\n') && (!c[wl] || c[wl] == ' ' || c[wl] == '\n')) { t = c + wl; break; }
    if (!t || *t != ' ' || n < 2) return 0;
    while (*t == ' ') t++;
    size_t len = strcspn(t, "\n");
    while (len && t[len - 1] == ' ') len--;
    if (!len || len > EPD_SHUT_TEXT_MAX || len >= n) return 0;
    int letters = 0;
    for (size_t i = 0; i < len; i++) {
        if (t[i] >= 'A' && t[i] <= 'Z') letters++;
        else if (t[i] != ' ') return 0;
    }
    if (!letters) return 0;
    memcpy(text, t, len);
    text[len] = 0;
    return 1;
}

enum { EPD_SHUT_OK = 0, EPD_SHUT_UNREADABLE = -1, EPD_SHUT_NOT_P5 = -2, EPD_SHUT_SIZE = -3, EPD_SHUT_SHORT = -4 };
static inline const char *epd_shut_why(int r)
{
    return r == EPD_SHUT_OK ? "ok" : r == EPD_SHUT_UNREADABLE ? "unreadable" : r == EPD_SHUT_NOT_P5 ? "not a P5 of maxval 255" :
           r == EPD_SHUT_SIZE ? "not the screen's size" : "short";
}

/* a header number of a PGM (blanks and # comments before it); -1: none */
static inline long epd_shut_num(FILE *f)
{
    int ch;
    for (;;) {
        ch = getc(f);
        if (ch == '#') { while ((ch = getc(f)) != EOF && ch != '\n') {} continue; }
        if (ch == ' ' || ch == '\t' || ch == '\n' || ch == '\r' || ch == '\v' || ch == '\f') continue;
        break;
    }
    if (ch < '0' || ch > '9') return -1;
    long v = 0;
    while (ch >= '0' && ch <= '9') {
        if (v > 1000000) return -1;
        v = v * 10 + (ch - '0');
        ch = getc(f);
    }
    /* exactly one blank after the number (after maxval: the data starts) */
    if (!(ch == ' ' || ch == '\t' || ch == '\n' || ch == '\r' || ch == '\v' || ch == '\f')) return -1;
    return v;
}

/* the picture at path into px (w * h bytes, rows top to bottom): EPD_SHUT_OK, or why not
 * (px is then undefined) */
static inline int epd_shut_load(const char *path, int w, int h, uint8_t *px)
{
    FILE *f = fopen(path, "rb");
    if (!f) return EPD_SHUT_UNREADABLE;
    int r = EPD_SHUT_OK;
    long fw, fh, mv;
    if (getc(f) != 'P' || getc(f) != '5') r = EPD_SHUT_NOT_P5;
    else if ((fw = epd_shut_num(f)) < 0 || (fh = epd_shut_num(f)) < 0 || (mv = epd_shut_num(f)) < 0 || mv != 255) r = EPD_SHUT_NOT_P5;
    else if (fw != w || fh != h) r = EPD_SHUT_SIZE;
    else if (fread(px, 1, (size_t)w * h, f) != (size_t)w * h) r = EPD_SHUT_SHORT;
    fclose(f);
    return r;
}

#endif
