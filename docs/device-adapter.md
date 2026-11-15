# Device adapter

The file `/etc/glassine/epd.conf`, which the device package ships, and the one parser
the panel programs read it with.

**From source** untested on hardware
{ .gl-untested }

Read from `src/epd-adapter.h` and `conf/epd.conf.onyx-noteair` at commit `1302819`
(2026-11-15). The sample's device paths and names were not checked on a device for this
page; they are the sample's own claims.

## Purpose

What the programs know about the device comes from this file, so that another device is
a file and not a code change. The engine itself is still built for one panel geometry
(1872 × 1404, `src/epd-engine.h`); making that a runtime setting is open work (README).

## Design

- One parser, header only: `src/epd-adapter.h`. `glassine-epdd`,
  `glassine-epdd-desktop`, `glassine-epd-show` and the drive test
  (`src/test/epd-stream-test.c`) include it.
- The defaults are the Onyx Boox Note Air's. The file is read over them, so a key the
  file leaves out keeps the default.
- A missing file: `glassine-epdd` doesn't start (see [Daemon options](daemon-options.md));
  `glassine-epdd-desktop` and `glassine-epd-show` run on the defaults.
- `glassine-epdd` takes another path with `-c`; the other two read only
  `/etc/glassine/epd.conf`.

## Interfaces

Format: `key=value` lines. `#` starts a comment, also after a value. Trailing blanks and
the line end are dropped; a value keeps its inner blanks. Unknown keys are ignored,
because other tools read their own keys from the same file. The last line of a scalar
key wins.

| Key | Meaning | Default (the parser's) |
|---|---|---|
| `panel_width`, `panel_height` | the panel's native size | 1872, 1404 |
| `rotation` | portrait surface → panel: `ccw`, `cw` or `none` (anything else: `ccw`) | `ccw` |
| `waveform` | the waveform file; empty or too long (256 bytes or more) gives the default | `/waveform/eink_waveform.wbf` |
| `temperature` | files tried in turn; the first that reads gives value ÷ `temperature_scale` °C; none reads: 24 °C | the battery's two sysfs files |
| `temperature_scale` | the files' unit (a value below 1 counts as 1) | 10 (tenths of a degree) |
| `frontlight` | brightness files, switched off while `glassine-epdd` sleeps | none |
| `sleep_text` | the asleep bar's text (capitals; `src/epd-glyphs.h`) | `PRESS POWER TO WAKE` |
| `pen` | the pen digitizer: part of its evdev name, then ` invert` if its axes are turned 180° from the panel's | the Note Air's digitizer, inverted |
| `touch` | the touchscreen: part of its evdev name; the first device that matches | the Note Air's touchscreen |

**List keys** (`temperature`, `frontlight`, `pen`, `touch`) hold up to four entries
(`EPD_AD_N`), in the file's order. The key's first line replaces its default list;
a line with an empty value empties it (`pen=`: no digitizer but `glassine-penfix`'s).
An entry past the fourth, or too long to keep, is left out.

**The waveform override.** `GLASSINE_EPDD_WBF`, when set and not empty, replaces the
`waveform` key in `glassine-epdd-desktop` and `glassine-epd-show` (tests, bring-up).
`glassine-epdd` doesn't read it.

**The sample.** `conf/epd.conf.onyx-noteair` writes every key out and is the sample for
another device's file. Its comments say that the device package installs its own copy,
which may leave out a key whose default is the same. Its `waveform` points at the device
seed's copy under `/run/glassine/device/`: the waveform belongs to the panel and isn't in
this repository.

## Open questions

- Only the Note Air's file exists. What another panel needs beyond these keys isn't
  known to this page.
- The `pen` and `touch` names, the frontlight and temperature paths are the Note Air's
  as the source states them; none was checked on a device for this page.
