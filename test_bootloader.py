#!/usr/bin/env python3
"""Test the RunPod bootloader offline in the emulator.

Usage: python3 -I test_bootloader.py [--long]

--long adds the 60 s inactivity test.
"""
import copy
import os
import random
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import emulator  # noqa: E402
from emulator import Pod, EmulationStopped, MCLK  # noqa: E402
from frame_plan import plan_writes  # noqa: E402

FLASH = os.path.join(HERE, "backups", "flash_2100_ffff.bin")
INFO = os.path.join(HERE, "backups", "info_1000_10ff.bin")
MP = os.path.join(HERE, "backups", "original.mp")
APPLICATION_LIMIT = 0xEE00          # First protected address, including the segment at 0xEE00–0xEFFF.
BYTES_PER_FRAME = 12          # AB LEN SEQ 04 OFF IDX + 12 = 20 bytes in one ATT write.
failures = []


def check(cond, text):
    print(("  ok   " if cond else "  FAIL ") + text)
    if not cond:
        failures.append(text)


def fletcher(data):
    s1 = s2 = 0xFF
    i = 0
    while i < len(data):
        for b in data[i:i + 20]:
            s1 += b
            s2 += s1
        s1 = (s1 & 0xFF) + (s1 >> 8)
        s2 = (s2 & 0xFF) + (s2 >> 8)
        i += 20
    s1 = (s1 & 0xFF) + (s1 >> 8)
    s2 = (s2 & 0xFF) + (s2 >> 8)
    return ((s2 & 0xFF) << 8) | (s1 & 0xFF)


def bootloader_pod():
    p = Pod(FLASH, INFO)
    p.reset(watchdog=True)
    p.run(MCLK // 2)
    return p


_ready = None


def ready_pod():
    """Copy a bootloader state after 5.5 s. Its first write needs 5 s."""
    global _ready
    if _ready is None:
        _ready = bootloader_pod()
        _ready.run(5 * MCLK)
    return copy.deepcopy(_ready)


def read(p, address):
    r = p.command(0x05, struct.pack("<H", (address - 0x2100) & 0xFFFF))
    assert r and r[3] == 0x05 and r[4] == 0, r
    return r[7:]


def read_region(p, start, end):
    out = bytearray()
    a = start
    while a < end:
        out += read(p, a)
        a += 13
    return bytes(out[:end - start])


def erase_segment(p, address):
    assert 0x2100 <= address < APPLICATION_LIMIT
    r = p.command(0x06, struct.pack("<HH", address - 0x2100, 1))
    assert r and r[3] == 0x06 and r[4] == 0, r



def send_frame(p, t, cycles_max=8 * MCLK):
    p.tx.clear()
    p.send(t)
    p.run(cycles_max, lambda: p.received_frame() is not None)
    r = p.received_frame()
    off, idx = struct.unpack_from("<HH", t, 4)
    assert r and r[2] == t[2] and r[3] == 0x04 and r[4] == 0, r
    assert struct.unpack_from("<HH", r, 5) == (off, (idx + len(t) - 8) & 0xFFFF), r


def install(p, image, interrupt_after=None):
    """Erase, write and verify the body, then write the header last."""
    end = 0x2100 + len(image)
    for seg in range(0x2000, end, 0x200):      # Aligned 512-byte segments.
        erase_segment(p, max(seg, 0x2100))
    for n, (_, t) in enumerate(plan_writes(image), 1):
        send_frame(p, t)
        if interrupt_after and n >= interrupt_after:
            return False
    if read_region(p, 0x2104, end) != image[4:]:
        return False
    for _, t in plan_writes(image[:4], start=0x2100):
        send_frame(p, t)
    return read_region(p, 0x2100, 0x2104) == image[:4]


def drop_one_byte(t, following_frame):
    """Replay t with each byte removed in turn, followed by the next frame."""
    unsafe_losses = []
    for i in range(len(t)):
        p = ready_pod()
        p.flash_log.clear()
        p.send(t[:i] + t[i + 1:] + following_frame)
        p.run(MCLK // 4)
        outside = [a for _, a in p.flash_log if not (0x2100 <= a < APPLICATION_LIMIT)]
        if outside or p.outside_flash or p.violations:
            unsafe_losses.append((i, [f"{a:04x}" for a in (outside + p.outside_flash)[:2]]))
    return unsafe_losses


def main():
    emulator.UART_BYTE_CYCLES = 600   # Space bytes 0.6 ms apart.
    image = open(MP, "rb").read()
    length = struct.unpack_from("<H", image)[0]
    image = image[:4 + length]

    print("1. Power on with a valid image")
    p = Pod(FLASH, INFO)
    try:
        p.run(MCLK)
        check(False, "the application should have started")
    except EmulationStopped as e:
        check(p.application_started, f"{e}, after {p.cycles / MCLK:.2f} s")

    print("2. Watchdog reset leaves the bootloader active")
    p = bootloader_pod()
    r = p.command(0x01)
    check(r is not None and r[4:] == bytes.fromhex("0009000400") + b"\x29\x55",
             f"command 01 -> {r.hex() if r else r}")
    check(read(p, 0x2100)[:4] == image[:4], "command 05 at 0x2100 matches the .mp header")
    check(read(p, 0xFFF0)[:13] == open(FLASH, "rb").read()[0xFFF0 - 0x2100:][:13],
             "command 05 at 0xFFF0 matches the backup")
    r = p.command(0x03)
    check(r is not None and r[4] == 0x01, f"unknown command 03 -> status 01 ({r.hex() if r else r})")

    print("3. Command 02: P = 1 stays in the bootloader, then P = 0 starts the application")
    p = bootloader_pod()
    r = p.command(0x02, b"\x01\x00", cycles_max=MCLK)
    check(r is None and p.reset_count >= 2 and p.per[0x02] & 1 == 0,
             "P = 1: no reply, watchdog reset, bootloader restarted")
    try:
        p.command(0x02, b"\x00\x00", cycles_max=2 * MCLK)
        check(False, "P = 0 should have started the application")
    except EmulationStopped as e:
        check(p.application_started, f"P = 0: {e}")

    print("4. Reinstall the entire stock firmware")
    p = ready_pod()
    ok = install(p, image)
    check(ok, "body and header match on readback")
    erased = sorted({a for t, a in p.flash_log if t == "E"})
    check(erased[0] == 0x2000 and erased[-1] == 0x7200,
             f"erased segments 0x{erased[0]:04x}–0x{erased[-1]:04x} ({len(erased)})")
    check(not p.violations and not p.outside_flash,
             "no writes or erases in 0xEF00–0xFFFF or outside flash")
    check(p.mem[0x2100:0x2100 + len(image)] == image, "emulated memory matches the .mp image")
    check(fletcher(p.mem[0x2104:0x2104 + length]) == struct.unpack_from("<H", image, 2)[0],
             "Fletcher-16 computed from emulated memory matches the header")
    print(f"     emulated time: {p.cycles / MCLK:.0f} s, UART overruns: {p.overruns}")
    try:
        p.command(0x02, b"\x00\x00", cycles_max=2 * MCLK)
        check(False, "the application should have started")
    except EmulationStopped:
        check(True, "command 02 P = 0 starts the application with a valid header")

    print("5. Power fails partway through a write")
    p = bootloader_pod()
    install(p, image, interrupt_after=500)
    p.reset(watchdog=False)       # Power back on with WDTIFG = 0.
    try:
        p.run(2 * MCLK)
        r = p.command(0x01)
        check(r is not None and r[5] == 0x09, "the erased header keeps the bootloader active after power returns")
    except EmulationStopped as e:
        check(False, f"the application should not start: {e}")
    check(install(p, image), "a full reinstall succeeds after power returns")

    print("6. Read frames with dropped bytes on a fast UART link")
    random.seed(1)
    emulator.UART_BYTE_CYCLES = 140   # Cause frequent UART overruns.
    p = bootloader_pod()
    for i in range(300):
        seq = random.randrange(0x17, 0xA0)          # Exclude 04, 06, 07, 08, AB, and BA.
        p.command(0x01, seq=seq, cycles_max=MCLK // 20)
        p.command(0x05, bytes([0x00, random.choice([0x00, 0x01, 0x02, 0x20, 0x40])]),
                   seq=seq, cycles_max=MCLK // 20)
    check(p.overruns > 0 and not p.flash_log,
             f"{p.overruns} dropped bytes, no flash writes or erases")

    print("7. Drop one byte from a write frame")
    emulator.UART_BYTE_CYCLES = 600
    unprotected = bytes.fromhex("ab12300404530000f0") + b"\x11" * 11
    unsafe_losses = drop_one_byte(unprotected, unprotected)
    check(bool(unsafe_losses), f"unprotected frame (OFF=0x5304, first byte 0xF0): {len(unsafe_losses)} unsafe losses {unsafe_losses[:3]}")
    plan = plan_writes(image)
    plan += plan_writes(image[:4], start=0x2100)
    sample = list(range(0, len(plan), max(1, len(plan) // 12))) + [len(plan) - 1]
    unsafe = []
    for k in sample:
        following_frame = plan[k + 1][1] if k + 1 < len(plan) else plan[0][1]
        unsafe += drop_one_byte(plan[k][1], following_frame)
    check(not unsafe, f"{len(sample)} planned frames with each byte removed in turn: "
                            f"no writes outside 0x2100–0xEDFF ({unsafe[:3]})")

    if "--long" in sys.argv:
        print("8. Bootloader returns to the application after 60 s of inactivity")
        emulator.UART_BYTE_CYCLES = 600
        p = bootloader_pod()
        try:
            p.run(62 * MCLK)
            check(False, "the application did not restart within 62 s")
        except EmulationStopped:
            check(55 < p.cycles / MCLK < 62, f"the application restarted after {p.cycles / MCLK:.1f} s")

    print("\nResult:", "PASS" if not failures else f"{len(failures)} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
