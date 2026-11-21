/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-inkarea — the pen's ink areas as the compositor sends them, and whose stroke a
 * contact is. glassine-epdd-desktop's pen thread draws raw ink inside the areas; the
 * compositor gives each pen stroke, whole, to the surface it starts on (the tablet's
 * implicit grab; a writing region takes the strokes that start in it, wherever they
 * go). So the preview decides per stroke too: where the pen lands decides whether the
 * stroke inks and whose it is, and every later report inks only on that owner's part
 * of the panel:
 *   - started in a writing region (S 2): wherever the owner's window is visible (its
 *     "ink vis" rects), also past the region's edge, never over what covers the window;
 *   - started in any other ink area (S 1 or a per-app ink=on window): only in that
 *     owner's areas (an app's canvas draws the stroke clipped to the canvas);
 *   - started outside every ink area: no ink at all, also where it crosses into one.
 * Lines (portrait px; see sway's glassine-ink-area.c), one block per change:
 *   ink begin
 *   ink win KEY                  the owner of the lines up to the next win (a stable token)
 *   ink vis X Y W H              the owner's visible region (sent for owners with regions)
 *   ink r K S X Y W H APP        an ink area with brush K (S: 0 per-app, 1 the app's own, 2 a region)
 *   ink S X Y W H APP            the same without a brush
 *   ink end
 * A block without win lines (an older compositor): one owner per APP, no regions.
 * Header only: the daemon and its host test (test/epd-inkarea-test.c) use the same code. */
#ifndef EPD_INKAREA_H
#define EPD_INKAREA_H
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "epd-policy.h"         /* box */

#define IA_C 4                  /* the masks' cell: 4x4 px, in when its centre is */
#define IA_MAXR 256             /* rect lines per block */
#define IA_MAXO 127             /* owners per block (the mask byte's low 7 bits) */
#define IA_REGION 0x80          /* owner mask: the cell is a writing region's */

typedef struct { box b; int sdk, br, own, vis; char app[128]; } ia_rect;     /* own: 1..no */
typedef struct {
    ia_rect r[IA_MAXR];
    int n, no, cur;             /* rects, owners, the owner of the lines now (0: by APP) */
    uint64_t key[IA_MAXO + 1];  /* key[o]: owner o's token, hashed (key[0] unused) */
} ia_block;

/* a stroke's owner: its key and whether it follows the window (started in a region) */
typedef struct { uint64_t key; int follow; } ia_owner;

static inline uint64_t ia_hash(const char *s, size_t n)
{
    uint64_t h = 1469598103934665603ull;            /* FNV-1a */
    for (size_t i = 0; i < n; i++) { h ^= (unsigned char)s[i]; h *= 1099511628211ull; }
    return h | 1;                                   /* never 0 */
}

static inline int ia_owner_of(ia_block *k, uint64_t key)
{
    for (int o = 1; o <= k->no; o++)
        if (k->key[o] == key) return o;
    if (k->no == IA_MAXO) return IA_MAXO;           /* (never: the compositor sends <= 32) */
    k->key[++k->no] = key;
    return k->no;
}

static inline void ia_begin(ia_block *k) { k->n = 0; k->no = 0; k->cur = 0; }

/* one line of a block (not begin/end/brush): 1 when it was an area line. clip: the panel */
static inline int ia_line(ia_block *k, const char *l, box clip)
{
    int x, y, w, h, s = 0, br = 0, nc = 0, vis = 0;
    if (!strncmp(l, "ink win ", 8)) {
        const char *t = l + 8;
        k->cur = ia_owner_of(k, ia_hash(t, strcspn(t, " \n")));
        return 1;
    }
    if (!strncmp(l, "ink vis ", 8)) {
        if (sscanf(l, "ink vis %d %d %d %d", &x, &y, &w, &h) != 4 || !k->cur) return 0;
        vis = 1;
    } else if (!strncmp(l, "ink r ", 6)) {
        if (sscanf(l, "ink r %d %d %d %d %d %d %n", &br, &s, &x, &y, &w, &h, &nc) != 6 || !nc) return 0;
        if (br < 0 || br > 32) br = 0;
    } else if (sscanf(l, "ink %d %d %d %d %d %n", &s, &x, &y, &w, &h, &nc) != 5 || !nc) {
        return 0;
    }
    if (k->n >= IA_MAXR) return 0;
    ia_rect *r = &k->r[k->n++];
    r->b = box_clip((box){ x, y, x + w, y + h }, clip);
    r->sdk = vis ? 0 : s < 0 ? 0 : s > 2 ? 1 : s;
    r->br = br;
    r->vis = vis;
    snprintf(r->app, sizeof r->app, "%s", vis ? "" : l + nc);
    if (k->cur) r->own = k->cur;
    else {                                          /* no win lines: one owner per APP */
        char t[160];
        int n = snprintf(t, sizeof t, "app:%s", r->app);
        r->own = ia_owner_of(k, ia_hash(t, n < (int)sizeof t ? (size_t)n : sizeof t - 1));
    }
    return 1;
}

/* the cells of b (centre inside) in a cw x ch mask set to v; returns how many */
static inline int ia_fill(uint8_t *m, int cw, int ch, box b, uint8_t v)
{
    int cells = 0;
    for (int cy = (b.y0 + IA_C / 2) / IA_C; cy < ch && cy * IA_C + IA_C / 2 < b.y1; cy++)
        for (int cx = (b.x0 + IA_C / 2) / IA_C; cx < cw && cx * IA_C + IA_C / 2 < b.x1; cx++) {
            if (cy * IA_C + IA_C / 2 < b.y0 || cx * IA_C + IA_C / 2 < b.x0) continue;
            m[cy * cw + cx] = v;
            cells++;
        }
    return cells;
}

/* the owner mask: each owner's visible region (owners with regions), then the areas
 * that ink (inks(rect) != 0; a per-app area of an app without ink=on doesn't), each
 * cell its owner, IA_REGION on a region's */
static inline void ia_owners(const ia_block *k, uint8_t *own, int cw, int ch,
                             int (*inks)(const ia_rect *, void *), void *data)
{
    memset(own, 0, (size_t)cw * ch);
    for (int i = 0; i < k->n; i++)
        if (k->r[i].vis) ia_fill(own, cw, ch, k->r[i].b, (uint8_t)k->r[i].own);
    for (int i = 0; i < k->n; i++)
        if (!k->r[i].vis && inks(&k->r[i], data))
            ia_fill(own, cw, ch, k->r[i].b, (uint8_t)(k->r[i].own | (k->r[i].sdk == 2 ? IA_REGION : 0)));
}

/* the mask cell of portrait (px,py), -1 off the panel */
static inline long ia_cell(int px, int py, int cw, int ch)
{
    if (px < 0 || py < 0 || px >= cw * IA_C || py >= ch * IA_C) return -1;
    return (long)(py / IA_C) * cw + px / IA_C;
}

/* a contact's first report on cell (slot: its ink area's brush, 0 none; own: the
 * owner mask): 1 = the stroke inks, *ow its owner */
static inline int ia_start(uint8_t slot, uint8_t own, const uint64_t *keys, ia_owner *ow)
{
    if (!slot || !(own & ~IA_REGION)) return 0;
    ow->key = keys[own & ~IA_REGION];
    ow->follow = (own & IA_REGION) != 0;
    return 1;
}

/* a later report of a stroke that inks: still on its owner's part of the panel? */
static inline int ia_cont(uint8_t slot, uint8_t own, const uint64_t *keys, const ia_owner *ow)
{
    int o = own & ~IA_REGION;
    return o && keys[o] == ow->key && (ow->follow || slot);
}
#endif
