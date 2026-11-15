# GlassineOS/epd

The e-ink panel engine and the daemons that own the panel. On a device whose display
controller is a software TCON (the kernel only scans out what it's given), every
refresh is computed here: the engine turns grey frames into the panel's drive
frames using the panel's waveform file, schedules overlapping updates, and keeps
track of what the panel shows.

## Contents

| Path | What |
|---|---|
| `src/epd-engine.c` | the engine: waveform file, drive frames, the update scheduler, the panel model |
| `src/epd-drm.c` | streams drive frames to the panel through DRM (mainline kernel) |
| `src/epd-gpu.c`, `src/epd-stagger.c` | the engine's passes on the GPU (EGL / GLES compute) and the staggered fast steps |
| `src/epd-policy.c`, `src/epd-motion.c` | the refresh policy: which mode, when to clean up, motion |
| `src/epd-ingest.c`, `src/epd-wl.c`, `src/epd-comp.c` | frame ingest: from the Wayland compositor's e-ink output backend, or as the X compositing manager |
| `src/epd-brush.c` | pen preview brushes |
| `src/epd-knobs.h`, `src/epd-knobs-ctl.h` | the one list of the engine's, drive's, GPU's and ingest's runtime knobs (their globals, first values, control names, `GLASSINE_*` variables) and the daemon's parser over it |
| `src/epd-adapter.h` | the device adapter (`/etc/glassine/epd.conf`): its parser and defaults, the temperature reader, the pen and touch names; `conf/epd.conf.onyx-noteair` is the Note Air's |
| `src/glassine-epdd.c` | **glassine-epdd**: the recovery's panel daemon (CPU engine, static, no GPU) |
| `src/glassine-epdd-desktop.c` | **glassine-epdd-desktop**: the desktop session's panel daemon (GPU engine) |
| `src/glassine-epd-show.c` | **glassine-epd-show**: shows a PGM on the panel (boot splash) |
| `src/glassine-epd-proto.h` | the frame protocol: a client (the recovery's UI) ↔ glassine-epdd |
| `src/glassine-epd-output-proto.h` | the output protocol: the compositor's e-ink output backend ↔ glassine-epdd-desktop |
| `src/test/` | host tests (`run.sh`) and a synthetic waveform generator |
| `docs/` | reference pages for the runtime interfaces: the control FIFO, the daemon options, the knobs, the device adapter ([index](docs/index.md)) |

No waveform file is in this repository. A waveform belongs to the panel; the device
package fetches it from the device at runtime (`waveform` in the adapter config).

## Device adapter

The programs read what they know about the device from an adapter config shipped by
the device package (`/etc/glassine/epd.conf`), all through one parser,
`src/epd-adapter.h`, whose head lists the keys: panel size, rotation, waveform path,
temperature files and their scale, frontlight files, the asleep bar's text, and the pen
digitizer's and touchscreen's evdev names (`pen=NAME [invert]`, `touch=NAME`). A key
the file leaves out keeps its default, the Note Air's; `conf/epd.conf.onyx-noteair`
writes every key out and is the sample for another device. `glassine-epdd-desktop`
and `glassine-epd-show` let `GLASSINE_EPDD_WBF` override the waveform and run on the
defaults without the file; `glassine-epdd` needs the file (`-c PATH` names another one,
default `/etc/glassine/epd.conf`; `-s PATH` the socket, default `/dev/socket/glassine-epd`;
it exits with 1 if the file can't be loaded: [`docs/daemon-options.md`](docs/daemon-options.md)). The engine itself is still
built for one panel geometry (1872 × 1404, `src/epd-engine.h`); making that a runtime
setting is open work.

## Runtime interfaces

Other components depend on these names; changing one is a coordinated change.

| Name | Used by |
|---|---|
| `/dev/socket/glassine-epd` (default) | the recovery UI (frame protocol) |
| `/run/glassine-epd.sock` | sway's glassine-epd output backend (`src/glassine-epd-output-proto.h`); root:video 0660 (the panel group; 0600 if there is no such group or the chown to it fails), and only root, the daemon's own user or a member of that group is accepted (SO_PEERCRED; "member" by the group database: the peer's primary group, or a group listing its user, since SO_PEERCRED carries no supplementary groups). Buffer and cursor memfds must be sealable against shrinking (`F_SEAL_SHRINK`, added by the daemon if the sender allowed sealing) and at least as large as the buffer, or the buffer is refused |
| `/run/glassine-epdd-desktop.ctl` | control FIFO of the desktop daemon (settings, shell, tests); root:video 0660 (the panel group), so its writers must be in it. The commands are in [`docs/control-fifo.md`](docs/control-fifo.md) and, in full, in the help comment at the top of `src/glassine-epdd-desktop.c` (from "FIFO commands"); the knobs' control names are in `src/epd-knobs.h` |
| `/run/glassine-epdd-desktop.levels`, `.classes`, `-frames.csv` | status files of the desktop daemon |
| `/run/glassine-epdd-prof/` | the only place a control line (`main_prof_csv=`, `drive_prof_csv=`) may write to, and where `shot`, `dcmap` and `srccheck` put their PGMs: root-owned, 0750 the panel group, files 0640. A path outside it is refused |
| `/etc/glassine/eink.conf` | per-application refresh profiles |
| `/etc/glassine/epd.conf` | the device adapter |
| `GLASSINE_EPD_GROUP` | the panel group named in the rows above (default `video`): the group of the socket, the control FIFO, the profile directory and `eink.conf`, and the members the socket accepts. Empty or unknown: root only (`src/epd-group.h`) |
| `GLASSINE_*` environment variables | tuning switches of the engine, the drive, the GPU, the ingest, the daemons and the tests. `src/epd-knobs.h` (the rows' `env` column) is the list of the knobs' variables; `GLASSINE_EPDD_WBF` (the waveform file) is in `src/epd-adapter.h`; see [`docs/knobs.md`](docs/knobs.md) |

## Building and testing

```sh
make                      # the three programs (C compiler, libdrm; the desktop daemon also X11 + EGL)
make check                # host tests; GPU tests skip without a render node
WBF=panel.wbf make check  # also the tests that need a real panel's waveform
```

`make check` needs `python3`: `src/test/run.sh` makes the synthetic waveform with
`src/test/gen-test-wbf.py` (`PYTHON=... make check` names another interpreter), and CI
installs it. The `WBF=panel.wbf` example has a known problem with a relative name,
tracked in [#10](https://github.com/GlassineOS/epd/issues/10).

CI builds on Alpine arm64 (as packaged), runs the host tests with the synthetic
waveform, and checks that no development-notebook references or waveform files are committed.

## License

GPL-2.0-or-later (`LICENSE`); every source file carries an `SPDX-License-Identifier` line.
The two installed wire headers, `glassine-epd-proto.h` and `glassine-epd-output-proto.h`, are
MIT, so that MIT-licensed clients can include them.
