# Daemon options

The command line, the environment and the exit codes of the three programs this
repository builds.

**From source** untested on hardware
{ .gl-untested }

Read from `src/glassine-epdd.c`, `src/glassine-epdd-desktop.c` and
`src/glassine-epd-show.c` at commit `1302819` (2026-11-15). Nothing on this page was run,
on a device or on a host.

## Purpose

To say what each program takes, so that bringing one up by hand, or on a second device,
doesn't need a read of `main`.

## Design

The three programs share the engine (`src/epd-engine.c`, `src/epd-drm.c`) and the device
adapter (`src/epd-adapter.h`, see [Device adapter](device-adapter.md)). Only
`glassine-epdd` has options for the adapter's path and its socket.

## Interfaces

### `glassine-epdd [-c CONFIG] [-s SOCKET]`

The recovery's panel daemon: it serves the frame protocol
(`src/glassine-epd-proto.h`) on a Unix socket.

| Option | Meaning | Default |
|---|---|---|
| `-c PATH` | the adapter config to read | `/etc/glassine/epd.conf` |
| `-s PATH` | the socket to listen on (a `SOCK_SEQPACKET` Unix socket) | `/dev/socket/glassine-epd` |

- Any other option prints `usage: … [-c CONFIG] [-s SOCKET]` and exits with **2**.
- It exits with **1** if the config can't be opened (the error names the path), if its
  `panel_width` × `panel_height` aren't 1872 × 1404 (the engine is built for that
  geometry: `src/epd-engine.h`), if the engine finds no panel (the message names the
  waveform path), or if the socket can't be bound or `accept` fails.
- It does not start without the config file: unlike the two programs below, it doesn't
  fall back to the defaults.
- Before it binds, it removes whatever file is at the socket path (`unlink`), and
  afterwards sets the socket to mode 0660. Give `-s` a path you don't mind losing.
- It doesn't read `GLASSINE_EPDD_WBF`: its waveform is the config's `waveform` key.
- To point it at the sample config before installing it:
  `glassine-epdd -c conf/epd.conf.onyx-noteair -s /tmp/test.sock`. The order in `main`
  is: read the config, open the panel through the engine, flash it white, then bind the
  socket. So on a machine with no panel it stops at "no panel" (exit 1) before the
  socket exists, and the waveform path in the sample is a device path
  (`/run/glassine/device/waveform.wbf`). That command line was not run.

### `glassine-epdd-desktop [-v|-vv]`

The desktop session's panel daemon.

- Its only arguments are `-v` and `-vv` (verbose, more verbose), and only as the first
  argument. It has no `-c` or `-s`: the adapter is always `/etc/glassine/epd.conf`, and a
  missing file means the adapter's defaults (the Note Air's), noted in the log only when
  verbose. The sockets and the FIFO are fixed (README, "Runtime interfaces").
- The comment above `main` documents it as `DISPLAY=:0 glassine-epdd-desktop [-v]`; the
  frame source is X, unless `GLASSINE_EPDD_SRC=wl` selects the compositor's e-ink
  output backend.
- Environment read at start, in `main`: `GLASSINE_EPDD` (control lines, see
  [Control FIFO](control-fifo.md)), `GLASSINE_EPDD_WBF` (the waveform file, before the
  config's), `GLASSINE_EPDD_GPU`, `GLASSINE_EPDD_GPE`, `GLASSINE_EPDD_GTARGET`
  (GPU modes: `0`/`1`/`2`/`check`; `gpu` also `pad`), `GLASSINE_EPDD_HV` (`0`: no high
  voltage, a dry run), `GLASSINE_EPDD_SRC`, and the knobs' `GLASSINE_*` variables (see
  [Runtime knobs](knobs.md)). `GLASSINE_EPDD_REEXEC`, `GLASSINE_EPDD_LIVE` and
  `GLASSINE_EPDD_EXECPROBE` are set by the daemon for its own re-exec; don't set them.
- Which of the two backends runs depends on `/dev/ebc`: if it can be opened, the stock
  kernel's driver; if it doesn't exist (`ENOENT`), the userspace engine.

### `glassine-epd-show IMAGE.pgm [waveform_mode [BAR TEXT]]`

Shows a PGM on the panel (the boot splash). No argument: usage, exit **2**.

- The image must be 1872 × 1404 or 1404 × 1872, else exit **1**.
- `waveform_mode` defaults to 2. `GLASSINE_EPD_ROT=cw` turns the image the other way
  (default counter-clockwise). `GLASSINE_EPDD_WBF` and `GLASSINE_EPDD_HV` apply as for
  the desktop daemon. No config file: the defaults.

## Open questions

- None of the above was run. The exit codes are the `return` values in `main` and the
  functions it calls.
- Whether `glassine-epdd -c` works on a second device depends on the engine's fixed
  geometry; making it a runtime setting is open work (README, "Device adapter").
