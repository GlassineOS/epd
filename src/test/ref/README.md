# The frozen policy (parity2's oracle)

`epd-policy.c` / `epd-policy.h` here are copies of `../../epd-policy.{c,h}` at the last
approved behaviour. `wrap.c` renames their symbols and translates to `ref-api.h`'s plain
types; `../epd-policy-parity2.c` compares the live policy against them.

Rules:
- A refactor must stay bit-exact: parity2 shows 0 divergences. Never edit these copies to
  make it pass.
- A deliberate behaviour change comes with a scenario that fails on the frozen copy and
  passes on the new code, a check that parity2's divergences are confined to what changed,
  and then a re-freeze: copy the new files here (through the same renames) in the same
  commit.
