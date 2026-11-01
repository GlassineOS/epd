#!/bin/sh
# glassine-epdd-desktop.c's scan + commit and their helpers (changed_row ... the end of
# commit_cpu) for scan-commit-test.c; the stock-kernel commit_ebuf is left
# out, and commit is the CPU's alone (glassine-epdd-desktop's also keeps the GPU mirror)
awk '
/^\/\* visibly changed \(the panel/ { on = 1 }
/^static long commit_ebuf\(/ { skip = 1 }
on && !skip { print }
skip && /^}/ { skip = 0; next }
on && /^static long commit_cpu\(const frame/ { last = 1 }
last && /^}/ { exit }
' "$1"
echo 'static long commit(const frame *f, int all, int how) { return commit_cpu(f, all, how); }'
