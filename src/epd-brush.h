/* epd-brush: declarative pen-preview brushes
 * "Brushes"). The contract between sway (glassine_ink_v1 v3 brush objects ->
 * brush files), glassine-epdd-desktop (loads them, stamps them on the pen thread) and the
 * host tests.
 *
 * Transport: sway writes each selected brush as a brush file
 * /run/glassine-ink/<id>.brush id = 16 lower-case hex = the first 8 bytes
 * of the file's SHA-256 (content-addressed)
 * (write to a temp name in the same dir, then rename) and names it on the
 * existing FIFO brush line, after the 9 v2 numbers:
 * ink brush K TKIND TMIN TMAX TEXP RKIND RMIN RMAX REXP [tip=<id>] [rub=<id>]
 * TMIN/TMAX/RMIN/RMAX stay the diameter range in panel px (sway already
 * multiplies by the output scale); the file holds the scale-free rest. An
 * older glassine-epdd-desktop reads the 9 numbers and ignores the rest. With a file the
 * KIND is 2 (pen) or 3 (erase) by the brush's ink kind, 1 for ink kind none.
 * /run/glassine-ink is the session user's (created by the session start), 0755;
 * glassine-epdd-desktop (root) only reads files named by a valid id there, re-validates
 * every field (the FIFO is world-writable) and caches by id.
 *
 * Threads: epdb_load/epdb_unref run on epdd's main thread only (at "ink
 * end"); the pen thread only reads prepared brushes (immutable after load)
 * and its own stroke state. */
#ifndef EPD_BRUSH_H
#define EPD_BRUSH_H
#include <stddef.h>
#include <stdint.h>

/* ---- the brush file (little-endian, packed by hand: no struct padding) ----
 * off size
 * 0 4 magic "EPDB"
 * 4 2 version (1)
 * 6 1 ink kind NB_INK / NB_ERASE / NB_NONE
 * 7 1 blend NB_DARKEN / NB_LIGHTEN / NB_REPLACE
 * 8 1 grey 0..255 (0 black)
 * 9 1 opacity 0..255
 * 10 1 angle mode NB_ANG_FIXED / NB_ANG_DIRECTION / NB_ANG_TILT
 * 11 1 scatter count 0..8
 * 12 2 hardness x 1/65535 (0..1)
 * 14 2 spacing x 1/1024 diameters (0.02..4)
 * 16 2 elongation x 1/65535 (0.05..1)
 * 18 2 angle int16, x 1/64 degree
 * 20 2 scatter position jitter x 1/1024 diameters (0..2)
 * 22 2 scatter size jitter x 1/65535 (0..1)
 * 24 2 scatter opacity jitter x 1/65535 (0..1)
 * 26 1 curves present: bit c = curve c (NB_CURVE_*) below is valid
 * 27 1 reserved 0
 * 28 64 curves[4][16] uint8 (see glassine_ink_v1 set_curve)
 * 92 1 tip size 0..64 (0 = the disc)
 * 93 1 tip variants 1..16 (1 for the disc)
 * 94 2 reserved 0
 * 96 - tip masks: size*size*variants bytes, 255 = full ink
 * total = 96 + size*size*variants (<= 96 + 64 KiB) */
#define NB_MAGIC "EPDB"
#define NB_VERSION 1
#define NB_HDR 96
#define NB_MAXTIP 64
#define NB_MAXVAR 16
#define NB_DIR "/run/glassine-ink"
enum { NB_INK, NB_ERASE, NB_NONE };
enum { NB_DARKEN, NB_LIGHTEN, NB_REPLACE };
enum { NB_ANG_FIXED, NB_ANG_DIRECTION, NB_ANG_TILT };
enum { NB_CURVE_P_SIZE, NB_CURVE_P_OPACITY, NB_CURVE_V_SIZE, NB_CURVE_T_SIZE, NB_NCURVE };

/* the decoded settings (also what sway fills before epdb_encode) */
typedef struct {
    uint8_t kind, blend, grey, opacity, angle_mode, scatter;
    float hardness, spacing, elongation, angle_deg;
    float pos_jitter, size_jitter, op_jitter;
    uint8_t curves_present;
    uint8_t curve[NB_NCURVE][16];
    uint8_t tip_size, tip_variants;
    const uint8_t *tip;                 /* tip_size^2 * tip_variants bytes (NULL: disc) */
} epdb_desc;

/* the default: a hard black disc, spacing 0.25 (= today's brush) */
void epdb_desc_default(epdb_desc *d);
/* the file bytes for d (clamped to the ranges above); returns the length,
 * 0 on error; out must hold NB_HDR + 64*64*16 bytes */
size_t epdb_encode(const epdb_desc *d, uint8_t *out);
/* the 16-hex id of file bytes (SHA-256's first 8 bytes) */
void epdb_id(const uint8_t *file, size_t len, char id[17]);

/* ---- prepared brushes (epdd) ------------------------------------------- */
typedef struct epd_brush epd_brush;     /* immutable once loaded, refcounted */
/* parse + validate + prepare file bytes (main thread); NULL + err on bad input */
epd_brush *epdb_load(const uint8_t *file, size_t len, char *err, size_t errlen);
/* the built-in disc of a v2 brush line (no file): hardness 1, spacing 0.25 at
 * <= 8 px diameter (today's ri/2 steps), black, darken (erase: white, replace) */
epd_brush *epdb_disc(int erase);
epd_brush *epdb_ref(epd_brush *b);
void epdb_unref(epd_brush *b);
int epdb_kind(const epd_brush *b);     /* NB_INK / NB_ERASE / NB_NONE */

/* ---- drawing (pen thread) ---------------------------------------------- */
/* one pen report, in panel buffer px (the frame ink_disc writes) */
typedef struct {
    float x, y;
    float pressure;                     /* 0..1 */
    float tilt_x, tilt_y;               /* degrees, -90..90 (0, 0: upright / unknown) */
    int has_tilt;
    double t;                           /* seconds, monotonic (event timestamp) */
} epdb_point;

/* per-stroke state: zero it, then epdb_stroke_begin */
typedef struct {
    uint32_t seed;                      /* xorshift32 state, never 0 */
    float lx, ly;                       /* last point */
    double lt;
    float dist;                         /* distance walked since the last dab */
    float vel;                          /* smoothed speed, panel px/s (curve range 0..4000 = ~0..450 mm/s) */
    float dir;                          /* stroke direction, radians */
    int n;                              /* points so far */
} epdb_stroke;

/* where dabs go. grey: the shadow (what the panel shows, 0..255, stride px);
 * xrgb: ebuf (may be NULL; same geometry); both are written together.
 * halo (may be NULL): dab pixels with coverage < 50 % go there instead
 * of grey/xrgb: halo byte 0 = nothing pending, else the pixel's target level
 * + 1 (levels 0..254; a white 255 is stored as 255 = level 254, close
 * enough); epdb_halo_flush writes them later. NULL = everything at once.
 * dither: the engine's threshold map (C_DITHER, panel-position keyed:
 * thresh[(y % th) * tw + x % tw]); NULL = greys written as greys.
 * With dither, every written level is 0 or 255 (the pen goes out as DU). */
typedef struct {
    uint8_t *grey;
    uint32_t *xrgb;
    int w, h, stride;
    uint8_t *halo;
    const uint8_t *thresh;
    int tw, th;
} epdb_target;

typedef struct { int x0, y0, x1, y1; } epdb_box;    /* panel px, half-open; empty: x0 >= x1 */

/* the dab budget per call: past it the spacing grows (a fast stroke stays
 * <= ~80 us on the A53) */
#define NB_MAXDABS 8

void epdb_stroke_begin(epdb_stroke *s, const epdb_point *p, uint32_t seed);
/* stamp from the stroke's last point to p. dmin/dmax: the diameter range in
 * panel px (the brush line's); pexp: the line's pressure exponent (used when
 * the brush has no pressure_size curve). Returns the panel box written
 * (grey/xrgb, plus the halo's in *halo_box when t->halo). */
epdb_box epdb_segment(const epd_brush *b, float dmin, float dmax, float pexp,
                      epdb_stroke *s, const epdb_point *p, const epdb_target *t,
                      epdb_box *halo_box);
/* move pending halo pixels inside box into grey/xrgb (dithered as
 * above), clear them; returns the box written */
epdb_box epdb_halo_flush(const epdb_target *t, epdb_box box);

#endif
