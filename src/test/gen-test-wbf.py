# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright GlassineOS contributors
"""gen-test-wbf.py OUT: a synthetic waveform file for the host tests.

It has the layout epd_wbf_load / epd_wbf_frames read (epd-engine.c) and
nothing else: 8 modes, 4 temperature ranges, RLE frame data. Each frame is a
32 x 32 table of 2-bit drive values (0 = hold, 1 = to black, 2 = to white)
indexed by new grey level << 5 | old; a transition drives for a number of frames
proportional to the level distance. The values are made up: the tests compare
the engine against itself (CPU vs GPU, banded vs serial, old vs new code
paths), not against a panel. A real panel needs its own waveform file."""
import struct
import sys

MODES = [30, 10, 24, 24, 24, 24, 6, 12]   # INIT DU GC16 GL16 4 5 A2 DU4
TEMPS = [0, 10, 20, 30, 50]                # 4 ranges, degrees C
END, SWITCH = 0xFF, 0xFC                   # never a data byte (value 3 is unused)
FRAME_RATE = 85


def frame(f, n):
    """frame f of n: 1024 entries -> 256 bytes, 4 entries per byte (low bits first)"""
    vals = []
    for i in range(1024):
        new, old = i >> 5, i & 31
        d = abs(new - old)
        steps = 0 if d == 0 else max(1, d * n // 32)
        vals.append(0 if f >= steps else (1 if new < old else 2))
    return bytes(sum(vals[b + k] << (2 * k) for k in range(4)) for b in range(0, 1024, 4))


def rle(data):
    """the RLE form decode() reads: byte, count-1 (RLE on from the start)"""
    out, i = bytearray(), 0
    while i < len(data):
        j = i
        while j + 1 < len(data) and data[j + 1] == data[i] and j - i < 255:
            j += 1
        out += bytes([data[i], j - i])
        i = j + 1
    return bytes(out + bytes([END]))


def main():
    temps = len(TEMPS) - 1
    head = bytearray(0x100)
    head[0x17] = FRAME_RATE
    head[0x25] = len(MODES) - 1
    head[0x26] = temps - 1
    head[0x28], head[0x29] = END, SWITCH
    head[0x30:0x30 + len(TEMPS)] = bytes(TEMPS)
    body = bytearray()
    base = len(head)
    mode_table = base
    body += bytes(4 * len(MODES))
    temp_tables = []
    for m in MODES:
        temp_tables.append(base + len(body))
        body += bytes(4 * temps)
    data_off = {}
    for mi, n in enumerate(MODES):
        for t in range(temps):
            data_off[mi, t] = base + len(body)
            body += rle(b''.join(frame(f, n + 2 * t) for f in range(n + 2 * t)))

    def put24(off, v):
        body[off - base:off - base + 3] = struct.pack('<I', v)[:3]
    for mi in range(len(MODES)):
        put24(mode_table + 4 * mi, temp_tables[mi])
        for t in range(temps):
            put24(temp_tables[mi] + 4 * t, data_off[mi, t])
    head[0x20:0x23] = struct.pack('<I', mode_table)[:3]
    with open(sys.argv[1], 'wb') as f:
        f.write(head + body)


if __name__ == '__main__':
    main()
