# Control FIFO

The control FIFO of `glassine-epdd-desktop`: settings, actions and test hooks, one line
at a time. Its path and owner are in the README's "Runtime interfaces".

**From source** untested on hardware
{ .gl-untested }

Read from `src/glassine-epdd-desktop.c` and `src/epd-policy.c` at commit `1302819`
(2026-11-15). No command below was run for this page.

## Purpose

To change a setting of the running daemon, or to ask it to do one thing (a full refresh,
a statistics dump), without restarting it and so without ending the desktop session.

## Design

- **Where.** `/run/glassine-epdd-desktop.ctl`. The daemon creates it at start: made with
  mode 0600, then given to the panel group (`GLASSINE_EPD_GROUP`, default `video`;
  `src/epd-group.h`), then set to 0660. With no such group it stays root's. A writer
  has to be root or in that group.
- **What a write is.** One line of words separated by blanks. The daemon opens the FIFO
  for reading and writing, so a write doesn't block when nothing else is reading.
- **How a line is read.** Each line goes through `apply_cmd`, and the daemon's
  main loop looks for the command words in it. Both look at the whole line with a
  substring search, not at a first word. Two things follow:
  - Several settings can share a line (`du_max=0.05 idle_ms=300`), and a setting is
    found wherever it stands in the line; the first `name=` wins.
  - A value must not contain the name of another command (`stats`, `sleep`, `mark`,
    `wake`, `shot` and the like): it would trigger that command too. The help comment
    says this about profile file names. `full` and `shutdown-screen` are matched as
    whole words (`pol_cmd_word`, `src/epd-policy.c`), so `clean=full` doesn't start a
    full refresh; this page didn't check whether any other word is. The host test
    `src/test/epd-cmdwords-test.py` fails if any word the sources look for could fire on
    a `shutdown-screen [TEXT]` line.
- **How much is read at once.** Under X a read takes at most 127 bytes; under the
  Wayland compositor backend at most 4095 (`src/glassine-epdd-desktop.c`, the read of
  `ctl`). Keep a line short; the source comment says writes of up to `PIPE_BUF` are
  atomic, which several writers rely on (the compositor's window hints).
- **The same words elsewhere.** The words and settings are also read from the
  environment variable `GLASSINE_EPDD` at start, from the `@` lines of
  `/etc/glassine/eink.conf` (without the `@`), and, after a re-exec, from
  `GLASSINE_EPDD_LIVE`, which the daemon sets itself.
- **Writing safely from a shell.** Don't read the FIFO (`cat`, `grep -r` on `/run`): a
  read blocks and takes the commands from the daemon.

## Interfaces

A selection, each checked against the code for its existence and its effect as the
code states it. The full list, with every setting's values and its default, is the
help comment at the top of `src/glassine-epdd-desktop.c`, from the line
"FIFO commands (also env GLASSINE_EPDD=... at start)". It isn't copied here, so it can't
drift from the code. For the settings that are knobs (`src/epd-knobs.h`) see
[Runtime knobs](knobs.md).

| Line | Effect |
|---|---|
| `full` | a full-screen flashing GC16 refresh at the next opportunity (`want_full`) |
| `verbose` | toggles the update log (stderr) |
| `reload` | re-reads `/etc/glassine/eink.conf` now (it is also re-read when it changes) |
| `reexec` | the daemon executes its own binary again with the same PID; refused with a message if the GPU is abandoned or the binary doesn't pass an exec probe |
| `stats` | prints the counters since the last `stats` to the log (stderr), then resets them |
| `mark` | logs a timestamp (milliseconds, the verbose log's clock) |
| `sleep`, `wake` | around suspend: a banner is painted and updates stop; the banner is replaced by the screen and updates resume |
| `pause`, `resume` | stop touching the panel; `resume` repaints with a full GC16 |
| `shutdown-screen [TEXT]` | a restart or power-off began (GlassineOS/shell's `glassine-shutdown-screen`): the picture `/run/glassine-shutdown-screen.pgm` (README, "Runtime interfaces") replaces the whole page in one full flashing GC16, from any state, asleep and paused too; then the panel is never written again until the daemon exits (state `gone`), and every later line is ignored, `wake`, `resume`, `full`, `reexec` and another `shutdown-screen` included. A picture that can't be read (missing, not a P5 of maxval 255 at the logical screen's size, short) gives a white page, with TEXT as a black bar at the bottom when it is capitals and blanks only (`[A-Z ]`, at most 63). The word is matched whole and taken alone: the rest of its line and of the same read doesn't apply. The log says which it drew (`picture` or `fallback`) |
| `shot` | saves the panel buffer and a fresh read as panel-order PGMs, only into `/run/glassine-epdd-prof/` (README) |
| `du_max=F` | largest screen fraction sent as DU (default 0.03) |
| `flash_min=F` | smallest screen fraction sent as a flashing GC16 (default 0.35) |
| `idle_ms=N` | milliseconds without damage before the GC16 cleanup (default 250; values below 50 are raised to 50) |
| `gpu=off\|on\|check`, `gpe=…`, `gtarget=…`, `dry=on\|off` | set an environment variable and re-exec the daemon (`dry=` is live when the userspace engine is open) |
| `drag …`, `impostor …` | pen/tile helpers, X only; their forms are in `drag_cmd` and `impostor` |

The log goes to the daemon's stderr; where that ends up depends on how the session
starts the daemon, which isn't in this repository.

## Open questions

- None of this was run. `from source, untested` applies to every row, including the
  defaults, which are read from `src/epd-policy.c` (`du_max`, `flash_min`, `IDLE_GC_MS`).
- The 127-byte read under X means a long line may be cut in the middle of a word. What
  the daemon does with the rest was not checked for this page.
- The help comment and the code can differ; the code wins. When they do, the comment
  is what to fix.
