#!/usr/bin/env python3
"""Test installer failure and recovery paths offline in the emulator.

Usage: python3 -I test_installer.py
"""
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import runpod_installer as I  # noqa: E402

MP = os.path.join(HERE, "backups", "original.mp")
FLASH = os.path.join(HERE, "backups", "flash_2100_ffff.bin")
INFO = os.path.join(HERE, "backups", "info_1000_10ff.bin")
failures = []


def check(cond, text):
    print(("  ok   " if cond else "  FAIL ") + text)
    if not cond:
        failures.append(text)


def run_install(transport, image, plan):
    try:
        state, _ = I.install(I.Session(transport), image, plan, report=lambda *_: None)
        return state, None
    except I.InstallError as e:
        return None, str(e)


def matches_image(transport, image):
    p = transport.pod
    return p.mem[0x2100:0x2100 + len(image)] == image and not p.violations and not p.outside_flash


def main():
    image = I.read_image(MP)
    plan = I.prepare_plan(image)

    print("1. Reject disallowed frames")
    rejected = {
        "erase segment 0xEE00": I.erase_frame(0x30, 0xEE00),
        "erase at 0xF000": I.erase_frame(0x30, 0xF000),
        "command 07": bytes.fromhex("ab08300700000100 0000".replace(" ", "")),
        "erase without padding": bytes.fromhex("ab063006000001 00".replace(" ", "")),
        "write at 0xEF00": I._frame(0x30, 0x04, struct.pack("<HH", 0xEF00 - 0x2100, 0) + b"\x11"),
        "unprotected write (OFF=0x5304, 0xF0)": bytes.fromhex("ab12300404530000f0") + b"\x11" * 11,
        "reboot with parameter 0002": I._frame(0x30, 0x02, b"\x02\x00"),
        "SEQ 0x05": I.version_frame(0x05),
        "command 0A": I._frame(0x30, 0x0A, b"\x00\x00"),
    }
    for name, t in rejected.items():
        try:
            I.validate_frame(t)
            check(False, f"should have rejected {name}")
        except I.InstallError:
            check(True, f"rejected: {name}")

    print("2. Reject a corrupted image")
    raw = bytearray(open(MP, "rb").read())
    raw[100] ^= 1
    path = os.path.join(HERE, "logs", "test_corrupt_image.mp")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    open(path, "wb").write(raw)
    try:
        I.read_image(path)
        check(False, "accepted a corrupted image")
    except I.InstallError as e:
        check("Fletcher" in str(e), f"rejected: {e}")
    os.remove(path)

    print("3. Drop a byte from write 300, then retry")
    tr = I.EmulatedTransport(FLASH, INFO)
    counter = {"n": 0}

    def drop_byte(frame):
        if frame[3] == 0x04:
            counter["n"] += 1
            if counter["n"] == 300:
                return 6                             # The low byte of IDX is the highest-risk case.
        return None
    tr.drop_byte = drop_byte
    state, err = run_install(tr, image, plan)
    check(err is not None and not tr.pod.violations and not tr.pod.outside_flash,
             f"stopped with {err!r}, without writing to the bootloader or outside flash")
    tr.drop_byte = None
    state, err = run_install(tr, image, plan)
    check(state == "stock" and matches_image(tr, image), f"retry: {state or err}, memory matches image")

    print("4. Simulate power loss during a write, then retry")
    tr = I.EmulatedTransport(FLASH, INFO)
    counter = {"n": 0}

    class PowerLoss(Exception):
        pass
    original_exchange = tr.exchange

    def exchange(t, timeout):
        if t[3] == 0x04:
            counter["n"] += 1
            if counter["n"] == 900:
                raise PowerLoss()
        return original_exchange(t, timeout)
    tr.exchange = exchange
    try:
        run_install(tr, image, plan)
    except PowerLoss:
        pass
    tr.exchange = original_exchange
    tr.pod.reset(watchdog=False)              # Power back on.
    tr.stock_running = False
    tr.wait(1)
    check(not tr.stock_running, "the erased header keeps the bootloader active after power returns")
    state, err = run_install(tr, image, plan)
    check(state == "stock" and matches_image(tr, image), f"retry: {state or err}, memory matches image")

    print("5. Leave the header erased after a readback mismatch")
    tr = I.EmulatedTransport(FLASH, INFO)
    modified = bytearray(image)
    modified[5000] ^= 0xFF                               # Write the original image but verify against the modified one.
    modified_plan = dict(plan)
    state, err = run_install(tr, bytes(modified), modified_plan)
    check(err is not None and "mismatched" in err and tr.pod.mem[0x2100:0x2102] == b"\xff\xff",
             f"stopped with {err!r}, header still erased")

    print("\nResult:", "PASS" if not failures else f"{len(failures)} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
