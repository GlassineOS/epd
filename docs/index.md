# GlassineOS/epd reference

Reference pages for the runtime interfaces of the e-ink panel engine and its daemons.
They sit in the repository so that they change in the same commit as the code
(`src/`); the README lists every name and path in "Runtime interfaces" and is the
reference for those.

| Page | What it covers |
|---|---|
| [Control FIFO](control-fifo.md) | what can be written to `/run/glassine-epdd-desktop.ctl`, and how a line is read |
| [Daemon options](daemon-options.md) | the command line and the exit codes of `glassine-epdd`, `glassine-epdd-desktop` and `glassine-epd-show` |
| [Runtime knobs](knobs.md) | where the list of knobs is, and the three ways to set one |
| [Device adapter](device-adapter.md) | the keys of `/etc/glassine/epd.conf` and what a key left out does |

**From source** untested on hardware
{ .gl-untested }

Every statement about what a program does in these pages was read from the source at
commit `1302819` (2026-11-15). Nobody ran it for these pages. Where a statement is a
claim about the panel or the device, not about the code, it says so.

No vendor material is in this repository or in these pages: no waveform file, firmware,
dump or decompiler output. The examples are made up.
