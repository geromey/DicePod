#!/usr/bin/env python3
"""Split a RunPod firmware image into bootloader write frames.

Usage: python3 -I frame_plan.py FIRMWARE.mp

The bootloader polls the UART and uses no frame checksum. A dropped byte
can shift the fields enough for payload bytes to become the command or the
16-bit write address (0x2100 + OFF + IDX).

Choose the frame length, IDX, and SEQ so that dropping any one byte and
receiving a trailing byte results in a write within [0x2100, 0xEE00),
a command that leaves flash intact, or a rejected frame. The 0xEE00–0xEFFF
segment overlaps the bootloader and is excluded.

Erased flash already contains 0xFF, so skip those bytes where possible.
"""
import struct
import sys

BASE = 0x2100
LIMIT = 0xEE00
MAX_DATA = 12          # AB LEN SEQ CMD OFF(2) IDX(2) + 12 = 20 bytes (one ATT write).
WRITE_COMMANDS = (0x04, 0x08)
ERASE_COMMANDS = (0x06, 0x07)


def parse_effects(stream):
    """Model the bootloader parser (f_f67a) and its flash commands.

    Return the effects as ("W", address) or ("E", address)."""
    effects, i, n = [], 0, len(stream)
    while i < n:
        if stream[i] != 0xAB:
            i += 1
            continue
        if i + 1 >= n:
            break
        frame_length = stream[i + 1]
        i += 2
        if frame_length >= 0x17:
            continue                       # Invalid length. Look for the next 0xAB.
        if i + frame_length > n:
            break                          # Incomplete frames are not executed.
        body = stream[i:i + frame_length]
        i += frame_length
        pos = [0]

        def read(k):
            # f_f338 returns zero and consumes nothing if bytes are missing.
            if len(body) - pos[0] < k:
                return 0
            v = int.from_bytes(body[pos[0]:pos[0] + k], "little")
            pos[0] += k
            return v
        read(1)                            # SEQ
        cmd = read(1)
        if cmd in WRITE_COMMANDS:
            off, idx = read(2), read(2)
            while pos[0] < len(body):
                effects.append(("W", (BASE + off + idx) & 0xFFFF))
                idx = (idx + 1) & 0xFFFF
                pos[0] += 1
        elif cmd in ERASE_COMMANDS:
            start, size = read(2), read(2)
            end = (start + size) & 0xFFFF
            o, iterations = start, 0
            while o < end and iterations < 200:
                effects.append(("E", (BASE + o) & 0xFFFF))
                o = (o + 0x200) & 0xFFFF
                iterations += 1
    return effects


def safe_effects(effects, allow_erase=False):
    """Allow writes only within [BASE, LIMIT) and reject unexpected erases."""
    for t, a in effects:
        if t == "E" and not allow_erase:
            return False
        if not (BASE <= a < LIMIT) or (t == "E" and (a & ~0x1FF) + 0x200 > LIMIT):
            return False
    return True


def write_frame(seq, address, idx, data):
    off = (address - BASE - idx) & 0xFFFF
    body = bytes([seq, 0x04]) + struct.pack("<HH", off, idx) + data
    return bytes([0xAB, len(body)]) + body


def safe_frame(t):
    """Return whether the frame remains safe after any single byte loss."""
    if not safe_effects(parse_effects(t)):
        return False
    if 0xAB in t[1:-1]:                    # Allow 0xAB only as the last byte.
        return False
    for i in range(len(t)):
        remaining = t[:i] + t[i + 1:]
        for trailing_byte in (0x00, 0x7F, 0xAB, 0xFF):
            if not safe_effects(parse_effects(remaining + bytes([trailing_byte]) * 2)):
                return False
    return True


# Try small IDX values first. The address 0x2100 + OFF + IDX wraps at
# 16 bits, so IDX influences where a shifted frame writes.
IDX_CANDIDATES = list(range(0x40)) + [(h << 8) | b for h in range(0, 0x100, 8) for b in range(0, 0x100, 8)]


def sequences():
    s = 0x17
    while True:
        if s not in (0xAB, 0xBA):
            yield s
        s = s + 1 if s < 0xFE else 0x17


def plan_writes(image, start=BASE + 4, base=BASE):
    """Return (address, frame) pairs covering non-0xFF bytes from the start address."""
    end = base + len(image)
    seqs = sequences()
    seq = next(seqs)
    plan, a = [], start
    while a < end:
        if image[a - base] == 0xFF:
            a += 1
            continue
        chosen = None
        for n in range(MAX_DATA, 0, -1):
            data = image[a - base:a - base + n]
            if len(data) < n or 0xFF in data[-1:]:
                continue
            for idx in IDX_CANDIDATES:
                t = write_frame(seq, a, idx, data)
                if safe_frame(t):
                    chosen = t
                    break
            if chosen:
                break
        if chosen is None:
            raise ValueError(f"cannot build a safe frame at 0x{a:04x}")
        plan.append((a, chosen))
        a += len(chosen) - 8
        seq = next(seqs)
    return plan


def main():
    image = open(sys.argv[1], "rb").read()
    length = struct.unpack_from("<H", image)[0]
    image = image[:4 + length]
    plan = plan_writes(image)
    total = sum(len(t) - 8 for _, t in plan)
    non_ff_bytes = sum(1 for b in image[4:] if b != 0xFF)
    print(f"{len(plan)} frames, {total} bytes written ({non_ff_bytes} non-0xFF bytes in the body)")
    sizes = [len(t) - 8 for _, t in plan]
    print("Frame size distribution:", {n: sizes.count(n) for n in sorted(set(sizes))})
    header = plan_writes(image[:4], start=BASE)
    print("Header:", [t.hex() for _, t in header])


if __name__ == "__main__":
    main()
