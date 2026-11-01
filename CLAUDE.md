# CLAUDE.md — GlassineOS/epd

The e-ink panel engine and its daemons (see `README.md`). Every change: run
`make check` and extend the tests for new behaviour; most tests compare two code paths
bit for bit, so a new path gets a comparison against the one it replaces.

- Nothing vendor-owned in this repository: no waveform files, firmware, dumps or
  decompiler output. Tests use the synthetic waveform (`src/test/gen-test-wbf.py`).
- The names in README's "Runtime interfaces" are used by other repositories: renaming
  one needs the consumers changed in the same release.
- Commits carry an `Assisted-by:` or `Co-Authored-By:` trailer. Never add
  `Signed-off-by`. Run `.glassine/org/scripts/safety-scan.sh --staged` before committing.
- Organization guidance for coding agents, including the project knowledge search
  (run `git submodule update --init` if the import below is missing):

@.glassine/org/AGENTS.md
