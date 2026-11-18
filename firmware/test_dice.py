#!/usr/bin/env python3
"""Test DicePod in the emulator with the original bootloader.

Usage: python3 -I firmware/test_dice.py [dice_v3.mp]
"""
import math
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import emulator  # noqa: E402
from emulator import Pod, MCLK  # noqa: E402

FLASH = os.environ.get("DICEPOD_TEST_FLASH", os.path.join(HERE, "..", "backups", "flash_2100_ffff.bin"))
INFO = os.environ.get("DICEPOD_TEST_INFO", os.path.join(HERE, "..", "backups", "info_1000_10ff.bin"))
READY = 0x30
ACK = 0x31
failures = []


def check(cond, text):
    print(("  ok   " if cond else "  FAIL ") + text)
    if not cond:
        failures.append(text)


def frames(tx):
    """Split the transmitted bytes into BA frames."""
    result, i = [], 0
    while True:
        i = tx.find(0xBA, i)
        if i < 0 or i + 2 > len(tx) or i + 2 + tx[i + 1] > len(tx):
            return result
        result.append(bytes(tx[i:i + 2 + tx[i + 1]]))
        i += 2 + tx[i + 1]


def measurements(tx):
    return [(t[7], struct.unpack_from("<I", t, 10)[0]) for t in frames(tx) if t[3] == 0x84 and len(t) == 14]


def dice_pod(image):
    p = Pod(FLASH, INFO)
    p.mem[0x2100:0x2100 + len(image)] = image
    p.run_application = True
    p.flow_control = True      # The Bluetooth chip waits for a P1.3 toggle after each byte.
    p.reset(watchdog=False)
    p.set_p1_input(0x04, high=True)  # The accelerometer interrupt idles high.
    return p


def command(p, cmd, params=b"", seq=0x30, duration=0.5):
    p.tx.clear()
    body = bytes([seq, cmd]) + params
    p.send(bytes([0xAB, len(body)]) + body)
    p.run(int(duration * MCLK))
    return [t for t in frames(p.tx) if t[2] == seq and t[3] == cmd]


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "dice_v3.mp")
    mp = open(path, "rb").read()
    image = mp[:4 + struct.unpack_from("<H", mp)[0]]
    emulator.UART_BYTE_CYCLES = MCLK * 10 // 115_200      # Send bytes back-to-back.
    sys.path.insert(0, HERE)
    import asm430
    _, symbols, _ = asm430.assemble(open(os.path.join(HERE, "dice.s"), encoding="utf-8").read())
    MAIN_LOOP = symbols["main_loop"]

    print("1. Bootloader starts DicePod, resting with +Z up")
    p = dice_pod(image)
    p.run(3 * MCLK)
    m = measurements(p.tx)
    check(p.application_started and not m, "no face notification before subscribing")
    check(command(p, READY, seq=0x23) == [bytes.fromhex("ba03233000")], "client subscribes")
    check(p.mem[symbols["V_FACE"]] == 1 and not measurements(p.tx),
          "record the initial face without triggering an action")
    check(p.mma.regs[0x2A] == 0x23 and p.mma.regs[0x0E] == 0, "accelerometer active at 50 Hz, ±2 g")
    check(p.watchdog_resets == 0, "no watchdog reset")

    print("2. Roll for 1 s, then rest on -X")
    t0 = p.cycles / MCLK

    def orientation(t):
        dt = t - t0
        if dt < 1.0:                        # Gravity changes direction rapidly while rolling.
            a, b = 9 * dt, 13 * dt
            return (math.sin(a) * math.cos(b), math.sin(b), math.cos(a) * math.cos(b), 0.15)
        return (-1.0, 0.0, 0.0, 0.01)
    p.mma.orientation = orientation
    p.tx.clear()
    p.run(3 * MCLK)
    m = measurements(p.tx)
    check(m == [(5, 1)], f"resting on -X after movement: one change ({m})")
    p.tx.clear()
    p.mma.orientation = lambda t: (0.6, 0.6, 0.5, 0.01)
    p.run(MCLK)
    p.mma.orientation = lambda t: (-1.0, 0.0, 0.0, 0.01)
    p.run(2 * MCLK)
    check(not measurements(p.tx), "lifting the die and returning to face 5 produces no notification")

    print("3. Six orientations")
    expected = {(1, 0, 0): 2, (-1, 0, 0): 5, (0, 1, 0): 3, (0, -1, 0): 4, (0, 0, 1): 1, (0, 0, -1): 6}
    ok = True
    for g, face in expected.items():
        t1 = p.cycles / MCLK
        p.mma.orientation = (lambda g, t1: lambda t: ((0.7, 0.7, 0.0, 0.2) if t - t1 < 0.3 else (*g, 0.01)))(g, t1)
        p.tx.clear()
        p.run(int(1.5 * MCLK))
        m = measurements(p.tx)
        if not m or m[-1][0] != face:
            ok = False
            print(f"     {g} -> {m[-1] if m else None}, expected {face}")
    check(ok, "all six faces detected")
    p.mma.orientation = lambda t: (0.6, 0.6, 0.5, 0.01)        # Resting on an edge.
    p.tx.clear()
    p.run(int(1.5 * MCLK))
    m = measurements(p.tx)
    check(not m, f"no notification on an edge ({m})")
    p.mma.orientation = lambda t: (0.0, 0.0, 1.0, 0.01)
    p.run(MCLK)

    print("4. Bluetooth commands with back-to-back bytes")
    r = command(p, 0x01)
    check(r and r[0] == bytes.fromhex("ba09300100d10304022955"), f"01 -> {r[0].hex(' ') if r else None}")
    r = command(p, 0x05, struct.pack("<H", (0x1100 - 0x2100) & 0xFFFF), seq=0x31)
    check(r and r[0][7] == 1 and r[0][1] == 0x12, f"05 at 0x1100 (face) -> {r[0].hex(' ') if r else None}")
    before = len(p.flash_log)
    for c in (0x04, 0x06, 0x07, 0x08, 0x0A):
        r = command(p, c, b"\x00\x00\x01\x00\x00\x00\x11", seq=0x40 + c)
        check(r and r[0][4] == 0x01, f"command {c:02x} rejected (status 01)")
    check(len(p.flash_log) == before and not p.violations, "no flash operations")

    print("5. Command 02 enters the bootloader and restarts DicePod")
    emulator.UART_BYTE_CYCLES = 600   # Space the bytes out for the polling bootloader.
    command(p, 0x02, b"\x01\x00", seq=0x50, duration=1.0)
    r = command(p, 0x01, seq=0x51)
    check(r and r[0][5] == 0x09, f"02 01 00 -> bootloader ({r[0].hex(' ') if r else None})")
    command(p, 0x02, b"\x00\x00", seq=0x52, duration=1.0)
    r = command(p, 0x01, seq=0x53, duration=1.0)
    check(r and r[0][5] == 0xD1, f"02 00 00 -> dice ({r[0].hex(' ') if r else None})")

    emulator.UART_BYTE_CYCLES = MCLK * 10 // 115_200
    print("6. Watchdog returns to the bootloader after a firmware hang")
    loop = MAIN_LOOP                                             # Address of the first instruction of the main loop.
    p.mem[loop:loop + 4] = bytes.fromhex("0343fe3f")        # NOP followed by a jump back to the NOP.
    p.run(2 * MCLK)
    emulator.UART_BYTE_CYCLES = 600
    r = command(p, 0x01, seq=0x60)
    emulator.UART_BYTE_CYCLES = MCLK * 10 // 115_200
    check(p.watchdog_resets >= 1 and r and r[0][5] == 0x09,
             f"{p.watchdog_resets} watchdog reset(s), bootloader active ({r[0].hex(' ') if r else None})")

    print("7. Firmware keeps running without an accelerometer")
    p = dice_pod(image)

    class MissingAccelerometer:
        def start_transaction(self, e): pass
        def write(self, o): pass
        def read(self): return 0
    p.i2c.device = MissingAccelerometer()
    p.run(3 * MCLK)
    m = measurements(p.tx)
    r = command(p, 0x01, seq=0x70)
    check(not m and r and p.watchdog_resets == 0,
          f"no face, version reply {'received' if r else 'missing'}, no reset")

    print("7b. Recover from an accelerometer holding SDA low")
    p = dice_pod(image)
    p.run(2 * MCLK)
    command(p, READY)
    p.tx.clear()
    p.i2c.sda_hold_clocks = 9
    p.mma.orientation = lambda t: (1.0, 0.0, 0.0, 0.01)
    p.run(2 * MCLK)
    check(p.mem[symbols["V_ERR_I2C"]] == 1 and p.i2c.sda_hold_clocks == 0,
          "nine GPIO clock pulses release SDA after an I2C timeout")
    check(measurements(p.tx) == [(2, 1)] and p.watchdog_resets == 0,
          "report the new face after bus recovery without restarting the pod")

    print("8. Two minutes without movement requests disconnect and puts the device to sleep")
    src = open(os.path.join(HERE, "dice.s"), encoding="utf-8").read()
    p = dice_pod(image)
    p.run(3 * MCLK)
    command(p, READY, seq=0x71)
    original_resets = p.reset_count
    p.run(110 * MCLK)
    check(not p.mem[symbols["V_SLEEPING"]], "still awake before 120 seconds")
    command(p, 0x01, seq=0x72)
    p.run(15 * MCLK)
    check(p.mem[symbols["V_SLEEPING"]] and bytes.fromhex("ba03713002") in p.tx,
          "disconnect requested after 120 seconds despite recent command traffic")
    for _ in range(2000):
        if p.r[2] & 0xD0 == 0xD0:
            break
        p.step()
    check(p.mma.regs[0x2A] == 0x33 and p.mma.regs[0x2B] == 3 and p.r[2] & 0xD0 == 0xD0,
          "accelerometer at 6.25 Hz in low-power mode, MSP430 in LPM3")
    check(p.per[0x61] & 1 and p.mem[symbols["V_FACE"]] == 1, "UART stopped, last face retained")
    p.tx.clear()
    p.run(480 * MCLK)
    check(p.mem[symbols["V_SLEEPING"]] and p.reset_count == original_resets and not p.tx,
          "no bootloader fallback or periodic traffic after ten minutes")

    print("9. A slow rotation wakes the sleeping device and holds the new face")
    t0 = p.cycles / MCLK

    def slow_turn(t):
        angle = math.pi * min((t - t0) / 4, 1) / 2
        return math.sin(angle), 0.0, math.cos(angle), 0.0

    p.mma.orientation = slow_turn
    p.run(5 * MCLK)
    check(not p.mem[symbols["V_SLEEPING"]] and p.mem[symbols["V_FACE"]] == 2,
          "slow rotation wakes the MSP430 and stabilizes on face 2")
    check(not measurements(p.tx), "no notification while waiting for the subscriber")
    command(p, READY, seq=0x73)
    check(measurements(p.tx) == [(2, 1)], "the subscriber receives the latest stable face")
    stale_ack = command(p, ACK, struct.pack("<BI", 1, 0))
    check(stale_ack and stale_ack[0][4] == 1, "an older acknowledgment cannot discard the latest face")
    command(p, READY, seq=0x74)
    check(measurements(p.tx) == [(2, 1)], "reconnection retries the unacknowledged face once")
    good_ack = command(p, ACK, struct.pack("<BI", 2, 1))
    check(good_ack and good_ack[0][4] == 0, "matching acknowledgment accepted")
    command(p, READY, seq=0x75)
    check(not measurements(p.tx), "reconnection does not repeat a confirmed face")

    print("10. Accelerometer interrupt wakes the device without a new face")
    short_idle = src.replace(".equ IDLE_TICKS, 3000", ".equ IDLE_TICKS, 50")
    mem, _, _ = asm430.assemble(short_idle)
    p = dice_pod(asm430.image_mp(mem))
    p.run(5 * MCLK)
    p.set_p1_input(0x04, high=False)
    p.run(MCLK)
    check(p.interrupts.get(2) == 1 and not p.mem[symbols["V_SLEEPING"]],
          "Port 1 interrupt wakes the MSP430")
    p.set_p1_input(0x04, high=True)
    command(p, READY)
    check(not measurements(p.tx), "no face notification from waking on the same face")

    print("\nResult:", "PASS" if not failures else f"{len(failures)} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
