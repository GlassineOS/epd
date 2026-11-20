# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright GlassineOS contributors
"""The control FIFO's words against "shutdown-screen [TEXT]".

The daemon and the policy find most commands by a substring search over the whole
line (docs/control-fifo.md). A word that occurs inside "shutdown-screen", or that a
"shutdown-screen" line with its fallback capitals ([A-Z ]) could contain, would fire
along with the shutdown screen. This test reads every word the sources look for
(strstr / strncmp on a control line, pol_cmd_word, the knob names as "name=") and
fails on any such word, so a new command can't collide.

  epd-cmdwords-test.py SRC...   (the .c / .h files that read control lines)
"""
import re
import sys

LINE = "shutdown-screen"
TEXT = re.compile(r"[A-Z ]*")

PATTERNS = [
    re.compile(r'\bstrstr\(\s*(?:cmd|c|l|line)\s*,\s*"((?:[^"\\]|\\.)*)"'),
    re.compile(r'\bstrncmp\(\s*(?:cmd|c|l|line)\s*,\s*"((?:[^"\\]|\\.)*)"'),
    re.compile(r'\b(?:pol_)?cmd_word\(\s*\w+\s*,\s*"((?:[^"\\]|\\.)*)"'),
]
# (strncmp on a line `l` also reads file headers: these are not control words)
NOT_CONTROL = {"ENDHDR"}           # pill_load: the sleep pill's PAM header
KNOB = re.compile(r'^\s*X\(\s*\w+\s*,[^,]*,[^,]*,\s*"([a-z0-9_]+)"', re.M)


def words(paths):
    found = {}
    for p in paths:
        src = open(p, encoding="utf-8").read()
        for pat in PATTERNS:
            for m in pat.finditer(src):
                w = m.group(1).encode().decode("unicode_escape")
                if w not in NOT_CONTROL:
                    found.setdefault(w, p)
        for m in KNOB.finditer(src):
            found.setdefault(m.group(1) + "=", p)
    return found


def collides(w):
    """Could w occur in a line "shutdown-screen" or "shutdown-screen TEXT", TEXT in [A-Z ]*?"""
    if w == LINE:
        return False                    # the word itself
    if not w:
        return True
    if w in LINE:
        return True
    if TEXT.fullmatch(w):               # inside the capitals
        return True
    # starts inside the word and runs on into " TEXT"
    for i in range(len(LINE)):
        head = LINE[i:]
        if w.startswith(head) and len(w) > len(head):
            rest = w[len(head):]
            if rest[0] == " " and TEXT.fullmatch(rest):
                return True
    return False


def main():
    found = words(sys.argv[1:])
    fails = 0
    # the scan is not vacuous: it sees the commands it must see
    for must in ("sleep", "wake", "pause", "resume", "full", "reexec", "rotate", "impostor ", "drag ", "shutdown-screen", "idle_ms="):
        if must not in found:
            print(f"epd-cmdwords: FAIL: the scan did not find \"{must}\"")
            fails += 1
    # and would flag a colliding one
    for planted in ("screen", "down", "shut", "n-s", "screen R", "OFF", "OK "):
        if not collides(planted):
            print(f"epd-cmdwords: FAIL: a planted \"{planted}\" is not flagged")
            fails += 1
    for w, p in sorted(found.items()):
        if collides(w):
            print(f"epd-cmdwords: FAIL: \"{w}\" ({p}) fires on a \"{LINE} [TEXT]\" line")
            fails += 1
    if fails:
        sys.exit(1)
    print(f"epd-cmdwords: ok ({len(found)} control words, none inside a \"{LINE} [TEXT]\" line)")


main()
