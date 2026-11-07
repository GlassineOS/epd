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
| `src/glassine-epdd.c` | **glassine-epdd**: the recovery's panel daemon (CPU engine, static, no GPU) |
| `src/glassine-epdd-desktop.c` | **glassine-epdd-desktop**: the desktop session's panel daemon (GPU engine) |
| `src/glassine-epd-show.c` | **glassine-epd-show**: shows a PGM on the panel (boot splash) |
| `src/glassine-epd-proto.h` | the frame protocol: a client (the recovery's UI) ↔ glassine-epdd |
| `src/glassine-epd-output-proto.h` | the output protocol: the compositor's e-ink output backend ↔ glassine-epdd-desktop |
| `src/test/` | host tests (`run.sh`) and a synthetic waveform generator |

No waveform file is in this repository. A waveform belongs to the panel; the device
package fetches it from the device at runtime (`waveform` in the adapter config).

## Device adapter

The daemons read everything about the device from an adapter config shipped by the
device package (`/etc/glassine/epd.conf`: panel size, rotation, waveform path,
temperature and frontlight files; see the head of `src/glassine-epdd.c`). All three
programs take the waveform from its `waveform=` line; `glassine-epdd-desktop` and
`glassine-epd-show` let `GLASSINE_EPDD_WBF` override it and fall back to
`/waveform/eink_waveform.wbf` without either (`src/epd-waveform.h`). The engine
itself is still built for one panel geometry (1872 × 1404, `src/epd-engine.h`); making
that a runtime setting is open work.

## Runtime interfaces

Other components depend on these names; changing one is a coordinated change.

| Name | Used by |
|---|---|
| `/dev/socket/glassine-epd` (default) | the recovery UI (frame protocol) |
| `/run/glassine-epd.sock` | sway's glassine-epd output backend (`src/glassine-epd-output-proto.h`); root:video 0660 (the panel group; 0600 if there is no such group or the chown to it fails), and only root, the daemon's own user or a member of that group is accepted (SO_PEERCRED; "member" by the group database: the peer's primary group, or a group listing its user, since SO_PEERCRED carries no supplementary groups). Buffer and cursor memfds must be sealable against shrinking (`F_SEAL_SHRINK`, added by the daemon if the sender allowed sealing) and at least as large as the buffer, or the buffer is refused |
| `/run/glassine-epdd-desktop.ctl` | control FIFO of the desktop daemon (settings, shell, tests); root:video 0660 (the panel group), so its writers must be in it |
| `/run/glassine-epdd-desktop.levels`, `.classes`, `-frames.csv` | status files of the desktop daemon |
| `/run/glassine-epdd-prof/` | the only place a control line (`main_prof_csv=`, `drive_prof_csv=`) may write to, and where `shot`, `dcmap` and `srccheck` put their PGMs: root-owned, 0750 the panel group, files 0640. A path outside it is refused |
| `/etc/glassine/eink.conf` | per-application refresh profiles |
| `/etc/glassine/epd.conf` | the device adapter |
| `GLASSINE_EPD_GROUP` | the panel group named in the rows above (default `video`): the group of the socket, the control FIFO, the profile directory and `eink.conf`, and the members the socket accepts. Empty or unknown: root only (`src/epd-group.h`) |
| `GLASSINE_*` environment variables | tuning switches of the desktop daemon and the tests |

## Building and testing

```sh
make                      # the three programs (C compiler, libdrm; the desktop daemon also X11 + EGL)
make check                # host tests; GPU tests skip without a render node
WBF=panel.wbf make check  # also the tests that need a real panel's waveform
```

CI builds on Alpine arm64 (as packaged), runs the host tests with the synthetic
waveform, and checks that no development-notebook references or waveform files are committed.

## License

GPL-2.0-or-later (`LICENSE`); every source file carries an `SPDX-License-Identifier` line.
The two installed wire headers, `glassine-epd-proto.h` and `glassine-epd-output-proto.h`, are
MIT, so that MIT-licensed clients can include them.
