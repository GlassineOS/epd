/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* the shutdown screen's inputs as glassine-epdd-desktop reads them (epd-shutscreen.h):
 * the fallback TEXT of "shutdown-screen [TEXT]", the picture (P5 of the logical
 * screen: good, missing, wrong magic, maxval, size, short), and a glyph for every
 * capital the fallback bar may show (epd-glyphs.h).
 *   epd-shutscreen-test DIR   (DIR: where the test pictures are written) */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../epd-shutscreen.h"
#include "../epd-glyphs.h"

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

#define SW 36
#define SH 48

static void put(const char *path, const char *hdr, long bytes)
{
    FILE *f = fopen(path, "wb");
    if (!f) { perror(path); exit(2); }
    fputs(hdr, f);
    for (long i = 0; i < bytes; i++) fputc((int)(i * 7 % 256), f);
    fclose(f);
}

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: epd-shutscreen-test DIR\n"); return 2; }
    char path[4096];
    snprintf(path, sizeof path, "%s/shut.pgm", argv[1]);

    /* TEXT */
    static const struct { const char *cmd, *want; } tx[] = {
        { "shutdown-screen", "" },
        { "shutdown-screen\n", "" },
        { "shutdown-screen ", "" },
        { "shutdown-screen RESTARTING", "RESTARTING" },
        { "shutdown-screen RESTARTING\n", "RESTARTING" },
        { "shutdown-screen   POWERED OFF  \nfull\n", "POWERED OFF" },
        { "win 1 2\nshutdown-screen STARTING RECOVERY\nwake", "STARTING RECOVERY" },
        { "shutdown-screen Restarting", "" },                 /* not capitals */
        { "shutdown-screen RESTARTING\xe2\x80\xa6", "" },     /* (an ellipsis) */
        { "shutdown-screen GO 2", "" },
        { "shutdown-screen    ", "" },
        { "shutdown-screens RESTARTING", "" },                /* not the word */
        { "xshutdown-screen RESTARTING", "" },
        { "shutdown-screen=RESTARTING", "" },
        { "shutdown-screen ABCDEFGHIJKLMNOPQRSTUVWXYZ ABCDEFGHIJKLMNOPQRSTUVWXYZ ABCDEFGHI", "ABCDEFGHIJKLMNOPQRSTUVWXYZ ABCDEFGHIJKLMNOPQRSTUVWXYZ ABCDEFGHI" },     /* 63 */
        { "shutdown-screen ABCDEFGHIJKLMNOPQRSTUVWXYZ ABCDEFGHIJKLMNOPQRSTUVWXYZ ABCDEFGHIJ", "" },    /* 64: too long */
    };
    for (unsigned i = 0; i < sizeof tx / sizeof *tx; i++) {
        char t[EPD_SHUT_TEXT_MAX + 1] = "junk";
        int r = epd_shut_text(tx[i].cmd, t, sizeof t);
        CHECK(r == (tx[i].want[0] != 0) && !strcmp(t, tx[i].want), "text of \"%s\": %d \"%s\", want \"%s\"", tx[i].cmd, r, t, tx[i].want);
    }
    {   /* a short buffer: never overrun */
        char t[4] = "x";
        CHECK(epd_shut_text("shutdown-screen RESTARTING", t, sizeof t) == 0 && !t[0], "text into a short buffer");
    }

    /* the picture */
    uint8_t *px = malloc(SW * SH);
    char hdr[128];
    snprintf(hdr, sizeof hdr, "P5\n%d %d\n255\n", SW, SH);
    put(path, hdr, SW * SH);
    CHECK(epd_shut_load(path, SW, SH, px) == EPD_SHUT_OK && px[0] == 0 && px[SW * SH - 1] == (uint8_t)((SW * SH - 1) * 7 % 256),
          "a good picture");
    snprintf(hdr, sizeof hdr, "P5\n# glassine-sleep-pill -m screen\n%d  %d\n# x\n255\n", SW, SH);
    put(path, hdr, SW * SH + 10);               /* (comments, blanks; extra bytes are ignored) */
    CHECK(epd_shut_load(path, SW, SH, px) == EPD_SHUT_OK && px[1] == 7, "a picture with comments");
    snprintf(hdr, sizeof hdr, "P5 %d %d 255 ", SW, SH);
    put(path, hdr, SW * SH);
    CHECK(epd_shut_load(path, SW, SH, px) == EPD_SHUT_OK && px[1] == 7, "a one-line header");
    snprintf(hdr, sizeof hdr, "P5\n%d %d\n255\n", SW, SH);
    put(path, hdr, SW * SH - 1);
    CHECK(epd_shut_load(path, SW, SH, px) == EPD_SHUT_SHORT, "a short picture");
    snprintf(hdr, sizeof hdr, "P5\n%d %d\n255\n", SH, SW);
    put(path, hdr, SW * SH);
    CHECK(epd_shut_load(path, SW, SH, px) == EPD_SHUT_SIZE, "a turned size");
    snprintf(hdr, sizeof hdr, "P5\n%d %d\n255\n", SW + 1, SH);
    put(path, hdr, (SW + 1) * SH);
    CHECK(epd_shut_load(path, SW, SH, px) == EPD_SHUT_SIZE, "another width");
    snprintf(hdr, sizeof hdr, "P2\n%d %d\n255\n", SW, SH);
    put(path, hdr, SW * SH);
    CHECK(epd_shut_load(path, SW, SH, px) == EPD_SHUT_NOT_P5, "a P2");
    snprintf(hdr, sizeof hdr, "P7\nWIDTH %d\nHEIGHT %d\nDEPTH 1\nMAXVAL 255\nENDHDR\n", SW, SH);
    put(path, hdr, SW * SH);
    CHECK(epd_shut_load(path, SW, SH, px) == EPD_SHUT_NOT_P5, "a PAM");
    snprintf(hdr, sizeof hdr, "P5\n%d %d\n65535\n", SW, SH);
    put(path, hdr, SW * SH * 2);
    CHECK(epd_shut_load(path, SW, SH, px) == EPD_SHUT_NOT_P5, "maxval 65535");
    snprintf(hdr, sizeof hdr, "P5\n%d %d\n15\n", SW, SH);
    put(path, hdr, SW * SH);
    CHECK(epd_shut_load(path, SW, SH, px) == EPD_SHUT_NOT_P5, "maxval 15");
    put(path, "P5\n", 0);
    CHECK(epd_shut_load(path, SW, SH, px) == EPD_SHUT_NOT_P5, "a header only");
    put(path, "", 0);
    CHECK(epd_shut_load(path, SW, SH, px) == EPD_SHUT_NOT_P5, "an empty file");
    remove(path);
    CHECK(epd_shut_load(path, SW, SH, px) == EPD_SHUT_UNREADABLE, "no file");
    for (int r = 0; r >= -4; r--) CHECK(epd_shut_why(r) && *epd_shut_why(r), "why %d", r);
    free(px);

    /* every capital has a glyph, and every glyph has ink */
    for (char c = 'A'; c <= 'Z'; c++) {
        int found = 0, ink = 0;
        for (unsigned k = 0; k < sizeof epd_glyphs / sizeof *epd_glyphs; k++)
            if (epd_glyphs[k].c == c) { found++; for (int y = 0; y < 7; y++) ink |= epd_glyphs[k].row[y]; }
        CHECK(found == 1 && ink, "glyph %c: %d entries", c, found);
    }

    if (!fails) printf("epd-shutscreen: ok (the fallback text, the picture's checks, A-Z glyphs)\n");
    return fails != 0;
}
