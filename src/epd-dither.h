/* epd-dither.h - the ordered dither's thresholds (glassine-epdd-desktop's C_DITHER / C_Q4
 * commit, and its GPU twin in epd-ingest.c: both read this one table).
 * t[row & 63][col & 63], panel rows and columns; a grey g becomes white where
 * g > t (C_Q4: the step up where the remainder g * 3 % 255 > t).
 * EPD_DITHER_BLUE (default): the stagger's 64 x 64 blue-noise ranks,
 * t = rank * 255 >> 12 (0..254: a grey g gets g / 255 white), read
 * transposed and shifted by 32 so it is independent of the stagger's own
 * groups (rank * G >> 12): a 50 % grey's black pixels split 0.497/0.503 over
 * G 2, 0.246..0.253 over G 4 (the untransformed tile: 0/1, all in one group)
 * EPD_DITHER_BAYER: the old 8 x 8 Bayer, bayer8 * 4 + 2 */
#ifndef EPD_DITHER_H
#define EPD_DITHER_H
#include <stdint.h>
#include <string.h>
#include "epd-bluenoise.h"

enum { EPD_DITHER_BLUE, EPD_DITHER_BAYER, EPD_DITHER_DIZZY };     /* (dizzy: epd-dizzy.h, not a table: here it reads as blue) */

static inline int epd_dither_parse(const char *v)
{
    return !strncmp(v, "bayer", 5) ? EPD_DITHER_BAYER : !strncmp(v, "dizzy", 5) ? EPD_DITHER_DIZZY : EPD_DITHER_BLUE;
}
static inline const char *epd_dither_name(int p) { return p == EPD_DITHER_BAYER ? "bayer" : p == EPD_DITHER_DIZZY ? "dizzy" : "blue"; }

/* a pixel's dither rank (0..4095: it goes white above rank / 4096 of white),
 * the one source for the engine's table below and the stagger's (epd_stag_dqt) */
static inline int epd_dither_rank(int x, int y, int pattern)
{
    static const uint8_t b8[8][8] = {
        { 0, 32,  8, 40,  2, 34, 10, 42 }, { 48, 16, 56, 24, 50, 18, 58, 26 },
        { 12, 44,  4, 36, 14, 46,  6, 38 }, { 60, 28, 52, 20, 62, 30, 54, 22 },
        { 3, 35, 11, 43,  1, 33,  9, 41 }, { 51, 19, 59, 27, 49, 17, 57, 25 },
        { 15, 47,  7, 39, 13, 45,  5, 37 }, { 63, 31, 55, 23, 61, 29, 53, 21 } };
    return pattern == EPD_DITHER_BAYER ? b8[y & 7][x & 7] * 64 + 32 : epd_bn_rank[(x + 32) & 63][(y + 32) & 63];
}

static inline void epd_dither_table(uint8_t t[64][64], int pattern)
{
    static const uint8_t bayer8[8][8] = {
        { 0, 32,  8, 40,  2, 34, 10, 42 }, { 48, 16, 56, 24, 50, 18, 58, 26 },
        { 12, 44,  4, 36, 14, 46,  6, 38 }, { 60, 28, 52, 20, 62, 30, 54, 22 },
        { 3, 35, 11, 43,  1, 33,  9, 41 }, { 51, 19, 59, 27, 49, 17, 57, 25 },
        { 15, 47,  7, 39, 13, 45,  5, 37 }, { 63, 31, 55, 23, 61, 29, 53, 21 } };
    for (int y = 0; y < 64; y++)
        for (int x = 0; x < 64; x++)
            t[y][x] = pattern == EPD_DITHER_BAYER ? (uint8_t)(bayer8[y & 7][x & 7] * 4 + 2)
                                                  : (uint8_t)(epd_bn_rank[(x + 32) & 63][(y + 32) & 63] * 255 >> 12);
}
#endif
