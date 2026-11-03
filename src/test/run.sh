#!/bin/sh
# Host tests of the engine, the refresh policy and the daemons' helpers.
# Most compare two code paths bit for bit (CPU vs GPU, banded vs serial, the
# hand-off inbox vs a direct submit); GPU tests exit 77 without a render node
# and count as skipped. WBF: a waveform file; by default a synthetic one
# (gen-test-wbf.py), which is enough for comparisons, not for a panel.
# Tests marked "real" check the engine's model against a real panel's
# waveform (settling, DC balance, cut lengths); they run only with WBF set.
#   run.sh            exit 0 = every test passed or was skipped
#   WBF=FILE run.sh   also the "real" tests, on that waveform
set -e
cd "$(dirname "$0")"
REAL=${WBF:+1}
B=${TMPDIR:-/tmp}/glassine-epd-test; mkdir -p "$B"
V=$(mktemp -d); trap 'rm -rf "$V"' EXIT
if [ -z "${WBF:-}" ]; then WBF=$B/test.wbf; ${PYTHON:-python3} gen-test-wbf.py "$WBF"; fi
CC=${CC:-cc}
c() { $CC -O2 -Wall -Wextra "$@"; }
skip77() { r=$?; tail -20 "$1"; [ $r = 77 ]; }
# real NAME: skip the next test unless a real waveform was given
real() { [ -n "$REAL" ] && return 0; echo "$1: skipped (needs a real waveform: WBF=FILE)"; return 1; }
have_gl=0; printf '#include <EGL/egl.h>\n#include <gbm.h>\nint main(void){return 0;}\n' > "$V/t.c" &&
    $CC -o "$V/t" "$V/t.c" -lEGL -lgbm 2>/dev/null && have_gl=1

# the refresh policy: scenarios, and parity with the frozen copy in ref/ over the whole API
c -o $B/epd-policy-test ../epd-policy.c epd-policy-test.c -lm
$B/epd-policy-test
c -o $B/epd-policy-parity2 ../epd-policy.c ref/wrap.c epd-policy-parity2.c -lm
$B/epd-policy-parity2 100 5000
PARITY2_SELFTEST=1 $B/epd-policy-parity2 1 3000 | tail -1
PARITY2_SELFTEST=2 $B/epd-policy-parity2 1 3000 | tail -1
c -o $B/epd-motion-test ../epd-motion.c epd-motion-test.c -lm
$B/epd-motion-test

# the files a control line may name: only a file name in the daemon's directory, 0640, no symlinks
c -o $B/epd-profpath-test epd-profpath-test.c
$B/epd-profpath-test

# the group of the FIFO, the profile directory, eink.conf and the socket: named once
c -o $B/epd-group-test epd-group-test.c
$B/epd-group-test

# the engine on the CPU
if real epd-matched; then  # the matched-cut model's settling and DC figures
    c -o $B/epd-matched-test ../epd-engine.c epd-matched-test.c
    $B/epd-matched-test "$WBF" 40
fi
c -I.. -o $B/epd-flash-test ../epd-engine.c epd-flash-test.c
$B/epd-flash-test "$WBF" >/dev/null && echo "epd-flash: ok"
c -I.. -o $B/epd-greystart-test ../epd-engine.c epd-greystart-test.c
$B/epd-greystart-test "$WBF"
c -o $B/epd-defer-test ../epd-engine.c epd-defer-test.c
$B/epd-defer-test "$WBF" 40 > "$V/defer.log" && tail -2 "$V/defer.log" || { tail -20 "$V/defer.log"; false; }
c -o $B/epd-tpool-test ../epd-engine.c epd-tpool-test.c -lpthread
MUSL=1 $B/epd-tpool-test "$WBF" 3000 2>/dev/null
c -o $B/epd-stagger-test ../epd-engine.c ../epd-stagger.c epd-stagger-test.c -lpthread
$B/epd-stagger-test "$WBF" > "$V/stagger-cpu.log" && tail -1 "$V/stagger-cpu.log" || { tail -20 "$V/stagger-cpu.log"; false; }
c -o $B/epd-handover-test ../epd-engine.c ../epd-stagger.c epd-handover-test.c
$B/epd-handover-test "$WBF" > "$V/handover.log" && tail -1 "$V/handover.log" || { tail -20 "$V/handover.log"; false; }
c -o $B/epd-jit-test epd-jit-test.c
$B/epd-jit-test

# the drive loop against epd-drm.c's fake DRM device (-DEPD_HOST)
c -DEPD_HOST -I/usr/include/libdrm -o $B/epd-stream-test ../epd-drm.c ../epd-engine.c epd-stream-test.c -lpthread
$B/epd-stream-test "$WBF" 1 host > "$V/stream.log" 2>&1 && grep "epd-stream host" "$V/stream.log" || { tail -30 "$V/stream.log"; false; }

# the claim pass: banded, packed and in-place forms == the serial pass, bit for bit
c -o $B/epd-claim-bench epd-claim-bench.c -lpthread -lm
$B/epd-claim-bench "$WBF" -t 3 -n 300 > "$V/claim.log" 2>&1 && tail -1 "$V/claim.log" || { tail -20 "$V/claim.log"; false; }
KERNEL=packed $B/epd-claim-bench "$WBF" -t 3 -n 300 > "$V/claim-packed.log" 2>&1 && tail -1 "$V/claim-packed.log" || { tail -20 "$V/claim-packed.log"; false; }
PROD=1 GLASSINE_CLAIM_KERNEL=soa $B/epd-claim-bench "$WBF" -t 4 -n 300 > "$V/claim-prod.log" 2>&1 && tail -1 "$V/claim-prod.log" || { tail -20 "$V/claim-prod.log"; false; }
PROD=1 GLASSINE_CLAIM_KERNEL=packed $B/epd-claim-bench "$WBF" -t 4 -n 300 > "$V/claim-prod-packed.log" 2>&1 && tail -1 "$V/claim-prod-packed.log" || { tail -20 "$V/claim-prod-packed.log"; false; }
S8=1 $B/epd-claim-bench "$WBF" -t 3 -n 300 > "$V/claim-s8.log" 2>&1 && tail -1 "$V/claim-s8.log" || { tail -20 "$V/claim-s8.log"; false; }
GLASSINE_CLAIM_SKIP=1 GLASSINE_CLAIM_PREFETCH=1 S8=1 $B/epd-claim-bench "$WBF" -t 3 -n 300 > "$V/claim-s8-skip.log" 2>&1 && tail -1 "$V/claim-s8-skip.log" || { tail -20 "$V/claim-s8-skip.log"; false; }

# the scheduler: every variant hashes the same as the serial run
c -o $B/epd-sched-bench ../epd-engine.c epd-sched-bench.c -lpthread
sb() { env "$@" HASH=1 $B/epd-sched-bench "$WBF" paced payback - | grep '^hash'; }
h0=$(sb X=0)
h1=$(sb GLASSINE_CLAIM_BANDS=4 THREADS=4)
h2=$(HASH=1 $B/epd-sched-bench "$WBF" paced payback - --effective 2 | grep '^hash')
h3=$(GLASSINE_CLAIM_BANDS=4 THREADS=4 HASH=1 $B/epd-sched-bench "$WBF" paced payback - --effective 2 | grep '^hash')
h4=$(sb GLASSINE_CLAIM_STATE=packed)
h7=$(sb GLASSINE_CLAIM_SKIP=1 GLASSINE_CLAIM_PREFETCH=1 GLASSINE_CLAIM_STATE=packed)
d6=$(GLASSINE_CLAIM_STATE=check HASH=1 $B/epd-sched-bench "$WBF" flood payback mix | grep '^claim_state' | grep -o '[0-9]* differ$')
[ -n "$h0" ] && [ "$h1" = "$h0" ] && [ -n "$h2" ] && [ "$h3" = "$h2" ] && [ "$h4" = "$h0" ] && [ "$h7" = "$h0" ] && [ "$d6" = "0 differ" ] &&
    echo "epd-sched-bench: ok (bands, packed, skip+prefetch == serial; check 0 differ)" ||
    { echo "epd-sched-bench: FAIL ($h0 / $h1 / $h2 / $h3 / $h4 / $h7 / $d6)"; false; }

# ingest, dithering, grey values, brushes, the Wayland frame path
c -fvect-cost-model=cheap -o $B/epd-ingest-bench epd-ingest-bench.c -lpthread -lm
QUICK=1 $B/epd-ingest-bench > "$V/ingest-bench.log" 2>&1 && grep "bytes:" "$V/ingest-bench.log" || { tail -20 "$V/ingest-bench.log"; false; }
c -o $B/epd-dizzy-test epd-dizzy-test.c
$B/epd-dizzy-test - - 3
c -Wno-missing-field-initializers -o $B/epd-dzblock-test epd-dzblock-test.c -lm
$B/epd-dzblock-test
c -o $B/epd-grey-test epd-grey-test.c -lm
$B/epd-grey-test | tail -1
c -fvect-cost-model=cheap -o $B/epd-brush-test ../epd-brush.c epd-brush-test.c -lm
$B/epd-brush-test | tail -1
c -o $B/epd-wl-test ../epd-wl.c epd-wl-test.c
$B/epd-wl-test 2>/dev/null

# the desktop daemon's vectorized scan + commit vs the per-pixel reference
sh extract-scan-commit.sh ../glassine-epdd-desktop.c > "$V/scan-commit.inc"
c -DLIVE='"'"$V/scan-commit.inc"'"' -o $B/scan-commit-test scan-commit-test.c ../epd-policy.c -lm -lpthread
$B/scan-commit-test
c -DEPD_HOST -I/usr/include/libdrm -DLIVE='"'"$V/scan-commit.inc"'"' -o $B/epd-stream-test-scan ../epd-drm.c ../epd-engine.c ../epd-policy.c epd-stream-test.c -lm -lpthread
GLASSINE_UPD_STEPS=12 $B/epd-stream-test-scan "$WBF" 1 host > "$V/stream-scan.log" 2>&1 && grep "epd-stream host: update: .*: \|epd-stream host: ok" "$V/stream-scan.log" || { tail -30 "$V/stream-scan.log"; false; }

# on the GPU (77 = no render node: skipped)
if [ $have_gl = 0 ]; then echo "GPU tests: skipped (no EGL/gbm)"; exit 0; fi
c -DEPD_GPU -I/usr/include/libdrm -o $B/epd-pack-test ../epd-gpu.c ../epd-engine.c ../epd-stagger.c epd-pack-test.c -lEGL -lpthread
$B/epd-pack-test || [ $? = 77 ]
c -DEPD_GPU -I/usr/include/libdrm -o $B/epd-gpe-test ../epd-gpu.c ../epd-engine.c ../epd-stagger.c epd-gpe-test.c -lEGL -lpthread
for e in X=0 GPE_ASYNC=1 GPE_PEN=1 "GPE_PEN=1 GPE_ASYNC=1"; do
    env $e $B/epd-gpe-test "$WBF" 200 > "$V/gpe.log" && tail -2 "$V/gpe.log" || skip77 "$V/gpe.log"
done
c -DEPD_GPU -I/usr/include/libdrm -o $B/epd-stagger-gpu ../epd-gpu.c ../epd-engine.c ../epd-stagger.c epd-stagger-test.c -lEGL -lpthread
$B/epd-stagger-gpu "$WBF" 120 > "$V/stagger.log" && tail -2 "$V/stagger.log" || skip77 "$V/stagger.log"
c -DEPD_GPU -I/usr/include/libdrm -DLIVE='"'"$V/scan-commit.inc"'"' -o $B/epd-gtarget-test epd-gtarget-test.c ../epd-ingest.c ../epd-gpu.c ../epd-engine.c ../epd-stagger.c ../epd-policy.c -lEGL -lgbm -lm -lpthread
$B/epd-gtarget-test "$WBF" 60 > "$V/gtarget.log" && tail -2 "$V/gtarget.log" || skip77 "$V/gtarget.log"
c -o $B/epd-ingest-test ../epd-ingest.c epd-ingest-test.c -lEGL -lgbm -lm
$B/epd-ingest-test || [ $? = 77 ]
