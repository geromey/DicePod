#!/usr/bin/env python3
"""Install a .mp firmware image on a Zwift RunPod through its BLE bootloader.

Usage:
  python3 -I runpod_installer.py IMAGE.mp --plan
  python3 -I runpod_installer.py IMAGE.mp --emulate [--run]
  python3 -I runpod_installer.py IMAGE.mp ADDRESS --confirm [--force-bootloader]

--plan validates the image and write plan offline. In --emulate mode,
--run starts the installed application after flashing. For a real install,
wake the pod nearby. If BlueZ has yet to discover it, the tool scans for
up to 30 seconds. The log is saved to logs/installation_*.log.

The installer validates the image length, Fletcher-16, and vectors, then
prepares and checks every frame before connecting. It switches to the
bootloader (command 02, argument 0001), erases the required segments, writes
and verifies the body, then writes and verifies the header. Reboot command
02 with argument 0000 starts the application.

An invalid or missing reply stops the installation. While the header is
erased, the bootloader stays available even after a power loss. Run the
installer again to recover. With --force-bootloader the first frame is the
bootloader entry command, for known firmware that cannot reply to version
requests. The bootloader must still reply before writes begin.

Writes and erases are limited to 0x2100–0xEDFF. Each frame is validated
again before transmission, including a check for loss of any one byte.
See technical.md for more details.
"""
import os
import struct
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from frame_plan import (BASE, LIMIT, parse_effects, safe_effects, plan_writes,  # noqa: E402
                         sequences, safe_frame)

SEGMENT = 0x200
VERSIONS = {0x34: "stock", 0x09: "bootloader", 0xD1: "dice"}     # First byte after the status in command 01 replies.
APPLICATIONS = ("stock", "dice")
TRAILING_BYTES = (0x00, 0x7F, 0xAB, 0xFF)


class InstallError(Exception):
    pass


# --------------------------------------------------------------- image
def fletcher(data):
    s1 = s2 = 0xFF
    for i in range(0, len(data), 20):
        for b in data[i:i + 20]:
            s1 += b
            s2 += s1
        s1 = (s1 & 0xFF) + (s1 >> 8)
        s2 = (s2 & 0xFF) + (s2 >> 8)
    s1 = (s1 & 0xFF) + (s1 >> 8)
    s2 = (s2 & 0xFF) + (s2 >> 8)
    return ((s2 & 0xFF) << 8) | (s1 & 0xFF)


def read_image(path):
    """Validate the image and return the bytes to write at 0x2100, including its header."""
    raw = open(path, "rb").read()
    length, checksum = struct.unpack_from("<HH", raw)
    image = raw[:4 + length]
    if len(image) != 4 + length:
        raise InstallError(f"truncated file: {len(raw)} bytes for a declared body length of {length}")
    if any(b != 0xFF for b in raw[4 + length:]):
        raise InstallError("unexpected data past the declared body length")
    if fletcher(image[4:]) != checksum:
        raise InstallError(f"Fletcher-16 mismatch: computed 0x{fletcher(image[4:]):04x}, header 0x{checksum:04x}")
    end = BASE + len(image)
    if end > LIMIT:
        raise InstallError(f"image too large: end 0x{end:04x}, limit 0x{LIMIT:04x}")
    vectors = struct.unpack_from("<16H", image, 4)
    for i, v in enumerate(vectors):
        unused = i < 15 and v in (0x0000, 0xFFFF)     # Unused interrupt vector.
        if not unused and (not (BASE + 36 <= v < end) or v & 1):
            raise InstallError(f"vector {i} points outside the image code at 0x{v:04x}")
    return image


# Frames
def _frame(seq, cmd, params):
    body = bytes([seq, cmd]) + params
    return bytes([0xAB, len(body)]) + body


def version_frame(seq):
    return _frame(seq, 0x01, b"")


def read_frame(seq, address):
    return _frame(seq, 0x05, struct.pack("<H", (address - BASE) & 0xFFFF))


def reboot_frame(seq, bootloader):
    return _frame(seq, 0x02, b"\x01\x00" if bootloader else b"\x00\x00")


def erase_frame(seq, address):
    """Erase the segment containing the address. Use size 1 and two zero padding
    bytes so that if a header byte drops, the shifted size reads as zero."""
    return _frame(seq, 0x06, struct.pack("<HH", address - BASE, 1) + b"\x00\x00")


def segment_start(a):
    return a & ~(SEGMENT - 1)


def validate_frame(t):
    """Validate a frame immediately before sending it."""
    if len(t) < 4 or t[0] != 0xAB or t[1] != len(t) - 2 or t[1] > 18:
        raise InstallError(f"malformed frame: {t.hex(' ')}")
    seq, cmd, params = t[2], t[3], t[4:]
    if not (0x17 <= seq <= 0xFE) or seq in (0xAB, 0xBA):
        raise InstallError(f"disallowed sequence number: {t.hex(' ')}")
    if 0xAB in t[1:-1]:
        raise InstallError(f"0xAB in frame body: {t.hex(' ')}")
    effects = parse_effects(t)
    if cmd == 0x01:
        ok = not params
    elif cmd == 0x05:
        ok = len(params) == 2
    elif cmd == 0x02:
        ok = params in (b"\x00\x00", b"\x01\x00")
    elif cmd == 0x04:
        ok = len(params) >= 5 and safe_frame(t) and safe_effects(effects) and len(effects) == len(params) - 4
    elif cmd == 0x06:
        start, size = struct.unpack_from("<HH", params)
        a = BASE + start
        ok = (len(params) == 6 and params[4:] == b"\x00\x00" and size == 1
              and BASE <= a and segment_start(a) + SEGMENT <= LIMIT and effects == [("E", a)])
    else:
        ok = False
    if not ok:
        raise InstallError(f"disallowed frame: {t.hex(' ')}")
    # Check every possible single-byte loss.
    intended = set(effects) if cmd == 0x06 else set()
    for i in range(len(t)):
        remaining = t[:i] + t[i + 1:]
        for c in TRAILING_BYTES:
            e = parse_effects(remaining + bytes([c]) * 2)
            if not safe_effects(e, allow_erase=True):
                raise InstallError(f"dropping byte {i} would make this frame unsafe: {t.hex(' ')}")
            if {x for x in e if x[0] == "E"} - intended:
                raise InstallError(f"dropping byte {i} would cause an unintended erase: {t.hex(' ')}")
    return t


def read_addresses(start, end):
    """Return starts of 13-byte reads covering [start, end), excluding 0xAB in offsets."""
    result, a = [], start
    while a < end:
        s = a
        while 0xAB in struct.pack("<H", (s - BASE) & 0xFFFF):
            s -= 1
        result.append(s)
        a = s + 13
    return result


def prepare_plan(image):
    """Prepare and validate all write and erase frames."""
    end = BASE + len(image)
    segments = [max(s, BASE) for s in range(segment_start(BASE), end, SEGMENT)]
    seqs = sequences()
    effacements = [validate_frame(erase_frame(next(seqs), s)) for s in segments]
    body = [validate_frame(t) for _, t in plan_writes(image)]
    header = [validate_frame(t) for _, t in plan_writes(image[:4], start=BASE)]
    for t in body + header:
        for _, a in parse_effects(t):
            if not (BASE <= a < end):
                raise InstallError(f"write outside the image: {t.hex(' ')}")
    return {"segments": segments, "erases": effacements, "body": body, "header": header,
            "reads": read_addresses(BASE + 4, end)}


# --------------------------------------------------------------- session
class Session:
    """Send frames individually and match replies by sequence number."""

    def __init__(self, transport, log=None):
        self.t = transport
        self.log = log
        self.seqs = sequences()
        self.nb = 0

    def _log(self, text):
        if self.log:
            self.log.write(f"{time.strftime('%H:%M:%S')} {text}\n")

    def next_seq(self):
        return next(self.seqs)

    def exchange(self, t, timeout=3.0, expect_reply=True):
        """Validate and send t, then return the matching reply or None."""
        validate_frame(t)
        self._log(f"→ {t.hex(' ')}")
        self.nb += 1
        replies = self.t.exchange(t, timeout if expect_reply else 0.5)
        matching_reply = None
        for r in replies:
            self._log(f"← {r.hex(' ')}")
            if len(r) > 4 and r[0] == 0xBA and r[1] == len(r) - 2 and r[2] == t[2] and r[3] == t[3]:
                matching_reply = r
            elif expect_reply:
                self._log("  (ignored an unrelated reply)")
        return matching_reply

    def require_reply(self, t, timeout=3.0):
        r = self.exchange(t, timeout)
        if r is None:
            raise InstallError(f"no reply to {t.hex(' ')}")
        if r[4] != 0:
            raise InstallError(f"status {r[4]:02x} for {t.hex(' ')}: {r.hex(' ')}")
        return r

    def version(self, attempts=4):
        for _ in range(attempts):
            r = self.exchange(version_frame(self.next_seq()), timeout=4)
            if r and len(r) >= 11 and r[4] == 0:
                return VERSIONS.get(r[5], "unknown"), r
            self.t.wait(1)
        return None, None

    def read(self, address):
        t = read_frame(self.next_seq(), address)
        r = self.require_reply(t)
        if r[1] != 0x12 or struct.unpack_from("<H", r, 5)[0] != (address - BASE) & 0xFFFF:
            raise InstallError(f"unexpected read reply at 0x{address:04x}: {r.hex(' ')}")
        return r[7:20]


def install(session, image, plan, report=print, force_bootloader=False):
    t = session.t
    end = BASE + len(image)

    if force_bootloader:
        # Send 02 01 00 before requesting the version.
        # Stock firmware, DicePod, and the bootloader all support this command.
        # Use it if the installed firmware cannot reply, as in DicePod revision 1.
        state = "stock"
    else:
        state, r = session.version()
        report(f"Pod: {state} ({r.hex(' ') if r else 'no reply'})")
    if state in APPLICATIONS:
        report("Entering the bootloader (02 01 00)")
        session.exchange(reboot_frame(session.next_seq(), True), expect_reply=False)
        t.wait(2)
        state, r = session.version(attempts=6)
        report(f"Pod: {state} ({r.hex(' ') if r else 'no reply'})")
    if state != "bootloader":
        raise InstallError("bootloader did not reply. Flash was not written")
    entered_at = t.now()

    report(f"Erasing {len(plan['erases'])} segments (0x{BASE:04x}–0x{segment_start(end - 1) + SEGMENT - 1:04x})")
    for tr in plan["erases"]:
        r = session.require_reply(tr)
        if r[1] != 7 or r[5:7] != tr[4:6]:
            raise InstallError(f"unexpected erase reply: {r.hex(' ')}")
        t.wait(0.03)
    for a in (BASE, BASE + 13):
        if session.read(a) != b"\xff" * 13:
            raise InstallError(f"0x{a:04x} was not erased")
    report("  verified that 0x2100–0x2119 is erased")

    # The bootloader waits five seconds after startup before its first write (f_f744).
    remaining = 6.5 - (t.now() - entered_at)
    if remaining > 0:
        t.wait(remaining)

    report(f"Writing the body: {len(plan['body'])} frames")
    for i, tr in enumerate(plan["body"], 1):
        r = session.require_reply(tr, timeout=8 if i == 1 else 3)
        off, idx = struct.unpack_from("<HH", tr, 4)
        if r[1] != 7 or struct.unpack_from("<HH", r, 5) != (off, (idx + len(tr) - 8) & 0xFFFF):
            raise InstallError(f"unexpected write reply {r.hex(' ')} for {tr.hex(' ')}")
        if i % 200 == 0:
            report(f"  {i}/{len(plan['body'])}")

    report(f"Verifying the body: {len(plan['reads'])} reads")
    lu = bytearray(len(image))
    for i, a in enumerate(plan["reads"], 1):
        block = session.read(a)
        lu[a - BASE:a - BASE + 13] = block[:max(0, end - a)]
        if i % 400 == 0:
            report(f"  {i}/{len(plan['reads'])}")
    mismatches = [BASE + i for i in range(4, len(image)) if lu[i] != image[i]]
    if mismatches:
        raise InstallError(f"{len(mismatches)} mismatched bytes, first at 0x{mismatches[0]:04x}. "
                       "The header was not written and the bootloader remains active")
    report("  body matches the image")

    report("Writing the header last")
    for tr in plan["header"]:
        session.require_reply(tr)
    header = session.read(BASE)[:4]
    if header != image[:4]:
        raise InstallError(f"header read back as {header.hex(' ')}, expected {image[:4].hex(' ')}")
    report(f"  header verified: {header.hex(' ')}")

    report("Rebooting (02 00 00)")
    session.exchange(reboot_frame(session.next_seq(), False), expect_reply=False)
    t.wait(2)
    state, r = session.version(attempts=6)
    report(f"Pod: {state} ({r.hex(' ') if r else 'no reply'})")
    report(f"Done. Sent {session.nb} frames.")
    return state, r


# --------------------------------------------------------------- transports
class TransportBLE:
    def __init__(self, address):
        import dbus
        from gi.repository import GLib
        import runpod_cmd
        self.dbus, self.GLib = dbus, GLib
        self.pod = runpod_cmd.Pod(address)

    def exchange(self, frame, timeout):
        p, GLib = self.pod, self.GLib
        ctx = GLib.MainContext.default()
        while ctx.pending():
            ctx.iteration(False)
        replies = list(p.received)
        p.received.clear()
        p.expected = frame[2]
        try:
            p.fff2.WriteValue(self.dbus.Array(frame, signature="y"), {"type": "request"})
        except self.dbus.exceptions.DBusException as e:
            raise InstallError(f"Bluetooth link lost ({e.get_dbus_name()}). "
                           "Find the pod again and rerun the installer")
        p.source = GLib.timeout_add(int(timeout * 1000), p._expire)
        p.loop.run()
        if p.source is not None:
            GLib.source_remove(p.source)
            p.source = None
        replies += p.received
        p.received.clear()
        return replies

    def wait(self, s):
        time.sleep(s)

    def now(self):
        return time.monotonic()

    def close(self):
        self.pod.close()


class EmulatedTransport:
    """Run the real bootloader and simulate the stock application."""

    def __init__(self, flash_bin, info_bin=None, drop_byte=None, run_application=False, image=None):
        """With run_application, run the newly flashed application.
        If image is supplied, run it from startup.
        """
        import emulator
        self.em = emulator
        emulator.UART_BYTE_CYCLES = 600
        self.pod = emulator.Pod(flash_bin, info_bin)
        self.pod.flow_control = True     # Require P1.3 acknowledgment as on the physical device.
        self.pod.run_application = run_application or image is not None
        self.stock_running = image is None    # Simulated stock application.
        if image is not None:
            self.pod.mem[BASE:BASE + len(image)] = image
            self.pod.reset(watchdog=False)
            self._run(self.em.MCLK)
        self.drop_byte = drop_byte            # A function returning the dropped byte index or None.
        self.n = 0

    def now(self):
        return self.pod.cycles / self.em.MCLK

    def wait(self, s):
        if not self.stock_running:
            self._run(int(s * self.em.MCLK))

    def _run(self, cycles, cond=None):
        try:
            self.pod.run(cycles, cond)
        except self.em.EmulationStopped:
            self.stock_running = True         # The bootloader has started the application.

    def exchange(self, frame, timeout):
        self.n += 1
        if self.stock_running:
            if frame[3] == 0x01:
                return [bytes([0xBA, 9, frame[2], 1, 0, 0x34, 0, 4, 1, 0x29, 0x55])]
            if frame[3] == 0x02 and frame[4:6] == b"\x01\x00":
                self.stock_running = False
                self.pod.reset(watchdog=True)
            return []
        outgoing = frame
        i = self.drop_byte(frame) if self.drop_byte else None
        if i is not None:
            outgoing = frame[:i] + frame[i + 1:]
        self.pod.tx.clear()
        self.pod.send(outgoing)
        expected_reply = lambda: any(t[2] == frame[2] and t[3] == frame[3] for t in self._frames())
        self._run(int(timeout * self.em.MCLK), expected_reply)
        return self._frames()

    def _frames(self):
        """Return all complete BA frames, including unsolicited notifications."""
        tx, result, i = self.pod.tx, [], 0
        while True:
            i = tx.find(0xBA, i)
            if i < 0 or i + 2 > len(tx) or i + 2 + tx[i + 1] > len(tx):
                return result
            result.append(bytes(tx[i:i + 2 + tx[i + 1]]))
            i += 2 + tx[i + 1]

    def close(self):
        pass


# --------------------------------------------------------------- main
def main():
    args = sys.argv[1:]
    if "--help" in args or "-h" in args:
        print(__doc__)
        return 0
    if not args:
        print(__doc__)
        return 2
    allowed = {"--plan", "--emulate", "--run", "--confirm", "--force-bootloader"}
    unknown = [arg for arg in args if arg.startswith("--") and arg not in allowed]
    if unknown:
        print(f"Unknown option: {unknown[0]}", file=sys.stderr)
        return 2
    image = read_image(args[0])
    print(f"Image: {len(image) - 4} bytes, Fletcher-16 0x{struct.unpack_from('<H', image, 2)[0]:04x}, "
          f"end 0x{BASE + len(image) - 1:04x}, reset vector 0x{struct.unpack_from('<H', image, 34)[0]:04x}")
    plan = prepare_plan(image)
    print(f"Frames validated: {len(plan['erases'])} erases, {len(plan['body'])} body writes,"
          f" {len(plan['header'])} header writes, {len(plan['reads'])} reads")
    if "--plan" in args:
        return 0
    if "--emulate" in args:
        transport = EmulatedTransport(os.path.join(HERE, "backups", "flash_2100_ffff.bin"),
                                       os.path.join(HERE, "backups", "info_1000_10ff.bin"),
                                       run_application="--run" in args)
    elif len(args) >= 2 and "--confirm" in args:
        transport = TransportBLE(args[1])
    else:
        print("To flash a pod, supply ADDRESS and --confirm.")
        return 2
    name = time.strftime("installation_%Y-%m-%d_%H-%M-%S.log")
    os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
    with open(os.path.join(HERE, "logs", name), "w") as log:
        session = Session(transport, log)
        try:
            state, _ = install(session, image, plan, force_bootloader="--force-bootloader" in args)
        except InstallError as e:
            print(f"Stopped: {e}")
            print(f"Log: logs/{name}")
            return 1
        finally:
            transport.close()
    if "--emulate" in args:
        mem = transport.pod.mem[BASE:BASE + len(image)]
        print("Emulated memory matches the image:", mem == image,
              "| Bootloader violations:", transport.pod.violations,
              "| Writes outside flash:", transport.pod.outside_flash)
    print(f"Log: logs/{name}")
    return 0 if state in APPLICATIONS else 1


if __name__ == "__main__":
    sys.exit(main())
