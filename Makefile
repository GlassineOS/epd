# glassine-epdd: the recovery's panel daemon (CPU engine, static)
# glassine-epd-show: shows a PGM on the panel (boot splash, static)
# glassine-epdd-desktop: the desktop session's panel daemon (GPU engine, X11 + Wayland frontends)
CC ?= cc
CFLAGS ?= -O2
WARN = -Wall -Wno-format-truncation
DRM_CFLAGS = $(shell pkg-config --cflags libdrm 2>/dev/null || echo -I/usr/include/libdrm)
PREFIX ?= /usr
S = src

all: glassine-epdd glassine-epd-show glassine-epdd-desktop

glassine-epdd: $(S)/glassine-epdd.c $(S)/epd-engine.c $(S)/epd-drm.c
	$(CC) -static $(CFLAGS) $(WARN) $(DRM_CFLAGS) -o $@ $^ -lm -lpthread

glassine-epd-show: $(S)/glassine-epd-show.c $(S)/epd-engine.c $(S)/epd-drm.c
	$(CC) -static $(CFLAGS) $(WARN) $(DRM_CFLAGS) -o $@ $^ -lm -lpthread

DESKTOP_SRC = $(S)/glassine-epdd-desktop.c $(S)/epd-wl.c $(S)/epd-ingest.c $(S)/epd-policy.c \
	$(S)/epd-motion.c $(S)/epd-engine.c $(S)/epd-drm.c $(S)/epd-gpu.c $(S)/epd-stagger.c \
	$(S)/epd-comp.c $(S)/epd-brush.c
glassine-epdd-desktop: $(DESKTOP_SRC)
	$(CC) $(CFLAGS) -fvect-cost-model=cheap -Wall -DEPD_GPU $(DRM_CFLAGS) -o $@ $(DESKTOP_SRC) \
		-lX11 -lXext -lXdamage -lXfixes -lXi -lXcomposite -lEGL -lm -lpthread -ldrm

check:
	sh $(S)/test/run.sh

install: all
	install -Dm755 glassine-epdd $(DESTDIR)$(PREFIX)/sbin/glassine-epdd
	install -Dm755 glassine-epd-show $(DESTDIR)$(PREFIX)/sbin/glassine-epd-show
	install -Dm755 glassine-epdd-desktop $(DESTDIR)$(PREFIX)/sbin/glassine-epdd-desktop
	install -Dm644 $(S)/glassine-epd-proto.h $(DESTDIR)$(PREFIX)/include/glassine/glassine-epd-proto.h
	install -Dm644 $(S)/glassine-epd-output-proto.h $(DESTDIR)$(PREFIX)/include/glassine/glassine-epd-output-proto.h

clean:
	rm -f glassine-epdd glassine-epd-show glassine-epdd-desktop

.PHONY: all check install clean
