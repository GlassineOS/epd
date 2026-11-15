# Runtime knobs

Where the list of the daemon's runtime knobs is, and how one is set.

**From source** untested on hardware
{ .gl-untested }

Read from `src/epd-knobs.h`, `src/epd-knobs-ctl.h` and `src/glassine-epdd-desktop.c` at
commit `1302819` (2026-11-15). No knob was set for this page.

## Purpose

The engine, the drive, the GPU and the ingest expose their tuning as process globals.
`src/epd-knobs.h` is the one list of them. This page doesn't repeat that list: a copy
would be out of date at the next knob. It says how to read the list and how a knob
gets its value.

## Design

`src/epd-knobs.h` is a table of rows. The head comment of that file defines the row
format (`var, init, def, name, kind, lo, hi, words, env, when`) and what each column
means; read it there. From one table the code makes the declarations, the definitions
with their first values, the parser of control lines (`epd_knobs_ctl`, in
`src/epd-knobs-ctl.h`, called by `apply_cmd`), the `stats` output of the named ones, and
the engine's environment reading.

Rows are grouped by the file that owns the knob: `EPD_KNOBS_ENGINE`, `EPD_KNOBS_DRM`,
`EPD_KNOBS_GPU` (only when built with `-DEPD_GPU`) and `EPD_KNOBS_INGEST`.

## Interfaces

**The three ways to set a knob.** A row's `name` column is its control name; its `env`
column is its `GLASSINE_*` variable. Either may be empty: `""` in `name` means the knob
is set by the environment or by code only.

| Way | Form | When it is read |
|---|---|---|
| Control line | `name=value`, on the [control FIFO](control-fifo.md), in `GLASSINE_EPDD` at start, or as an `@name=value` line in `/etc/glassine/eink.conf` | on every line |
| Environment | the row's `GLASSINE_*` variable | the engine's rows once, by `epd_init`; the others by their owner when it opens or first uses it, or by the daemon (noted per row) |

**When a change takes effect.** The row's `when` column: `EPD_KN_LIVE` is read where it
is used, so a change applies from then on; `EPD_KN_OPEN` is taken when its owner opens
(the engine, the stream, the GPU), so a later change waits for the next open, which for
the daemon means a re-exec or a restart.

**How a value is read** (the `kind` column): `EPD_KN_INT` is `atoi`, clamped to
`lo`..`hi`; `EPD_KN_WORD` matches words like `a|b|c` (a control value counts when it
starts with the word; an environment value only when it is the whole word, otherwise
the row's `def`); `EPD_KN_NZ` is `atoi` ≠ 0; `EPD_KN_FN` is parsed by the daemon
(`knob_fn` in `src/glassine-epdd-desktop.c`).

**The names are an interface.** The control names and the `GLASSINE_*` variables are
listed under "Runtime interfaces" in the README as used by other components. The
`epd-knobs.h` head comment says to keep them as they are; renaming one needs its
consumers changed in the same release (`CLAUDE.md`).

**Not every control word is a knob row.** The refresh policy's words (`du_max=`,
`idle_ms=` and the like) are parsed in `src/epd-policy.c` (`pol_config`), and the
daemon parses some itself in `apply_cmd`. The help comment at the top of
`src/glassine-epdd-desktop.c` lists all control words with their values and defaults;
the table in `src/epd-knobs.h` lists the knobs' variables.

`GLASSINE_EPDD_WBF` (the waveform file) and `GLASSINE_EPD_GROUP` (the panel group) are
not rows of the table: see `src/epd-adapter.h` and `src/epd-group.h`.

## Open questions

- The table was not walked row by row for this page; the statements above come from the
  head comment of `src/epd-knobs.h` and from the code that reads the table. The
  repository's host test (`src/test/epd-knobs-test.c`, run by `make check`) compares the
  parser against the one it replaced; it is the check on the table's behaviour, not
  this page.
- Which knobs have a measured effect on a panel isn't recorded here. The help comment
  quotes some measurements; this page didn't verify them.
