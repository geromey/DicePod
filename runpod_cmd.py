#!/usr/bin/env python3
"""Send AB commands to a Zwift RunPod and display BA replies.

Usage:
  python3 -I runpod_cmd.py ADDRESS "01" "05 00 00" ...
  python3 -I runpod_cmd.py ADDRESS --read START END FILE.bin
  python3 -I runpod_cmd.py ADDRESS --test-bootloader COUNT backups/flash_2100_ffff.bin
  python3 -I runpod_cmd.py ADDRESS --axes [SECONDS]

Enter each raw command in hex, starting with the command code.
The tool adds the AB marker, length, and sequence number.

Raw commands are restricted to the read-only allowlist. The bootloader
diagnostic also uses command 02 to reboot with 0001 (enter bootloader) or
0000 (start the application). Flash write and erase commands are rejected.

The bootloader polls the UART and may drop bytes. The tool uses sequence
numbers 0x17–0xAA, excludes 0xAB from the frame body, and filters diagnostic
read offsets. See technical.md for bootloader details.

Protocol
  Host to pod, characteristic fff2: AB LEN SEQ CMD parameters
  Pod to host, notification fff1: BA LEN SEQ CMD STATUS data
Command 05 reads from 0x2100 plus its 16-bit offset.
"""
import sys
import time

import dbus
import dbus.mainloop.glib
from gi.repository import GLib

BLUEZ = "org.bluez"
CHAR_IF = "org.bluez.GattCharacteristic1"
PROPS_IF = "org.freedesktop.DBus.Properties"
FFF1 = "0000fff1-0000-1000-8000-00805f9b34fb"
FFF2 = "0000fff2-0000-1000-8000-00805f9b34fb"
READ_ONLY = {0x01, 0x05, 0x0A, 0x10}  # Version, two memory read commands, and value 0x10.
BASE = 0x2100
SAFE_SEQUENCES = range(0x17, 0xAB)
BLOCKED_BYTES = {0x02, 0x04, 0x06, 0x07, 0x08, 0xAB, 0xBA}
REBOOTS = {b"\x01\x00": "bootloader", b"\x00\x00": "normal"}


class Pod:
    def __init__(self, address):
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        self.bus = dbus.SystemBus()
        self.path = "/org/bluez/hci0/dev_" + address.upper().replace(":", "_")
        self.received = []
        self.seq = SAFE_SEQUENCES[0]
        self.loop = GLib.MainLoop()
        self.expected = None
        self.source = None
        self.late_replies = []
        self._connect()

    def _objects(self):
        return dbus.Interface(self.bus.get_object(BLUEZ, "/"),
                              "org.freedesktop.DBus.ObjectManager").GetManagedObjects()

    def _discover(self, timeout=30):
        """Scan until BlueZ discovers the pod. Shake the pod to wake it up."""
        if self.path in self._objects():
            return
        print(f"BlueZ has not discovered the pod. Shake it during the Bluetooth scan (up to {timeout} s).", flush=True)
        adapter = dbus.Interface(self.bus.get_object(BLUEZ, "/org/bluez/hci0"), "org.bluez.Adapter1")
        try:
            adapter.StartDiscovery()
        except dbus.DBusException as e:
            if "InProgress" not in e.get_dbus_name():
                raise
        try:
            end = time.time() + timeout
            while time.time() < end:
                if self.path in self._objects():
                    print("  Found the pod.", flush=True)
                    return
                time.sleep(0.5)
            raise SystemExit("Pod not found. Shake it near the computer and try again.")
        finally:
            try:
                adapter.StopDiscovery()
            except dbus.DBusException:
                pass

    def _connect(self):
        self._discover()
        dev = self.bus.get_object(BLUEZ, self.path)
        self.dev = dbus.Interface(dev, "org.bluez.Device1")
        props = dbus.Interface(dev, PROPS_IF)
        if not props.Get("org.bluez.Device1", "Connected"):
            self.dev.Connect()
        for _ in range(80):                     # Allow up to 20 s for service discovery.
            if props.Get("org.bluez.Device1", "ServicesResolved"):
                break
            time.sleep(0.25)
        uuids = {}
        for _ in range(20):
            uuids = {str(i[CHAR_IF]["UUID"]).lower(): p for p, i in self._objects().items()
                     if p.startswith(self.path) and CHAR_IF in i}
            if FFF1 in uuids and FFF2 in uuids:
                break
            time.sleep(0.5)
        else:
            raise SystemExit("RunPod GATT services not found. Wake the pod nearby and try again.")
        self.fff1 = dbus.Interface(self.bus.get_object(BLUEZ, uuids[FFF1]), CHAR_IF)
        self.fff2 = dbus.Interface(self.bus.get_object(BLUEZ, uuids[FFF2]), CHAR_IF)
        if getattr(self, "_subscribed_path", None) != uuids[FFF1]:
            self.bus.add_signal_receiver(self._notif, "PropertiesChanged", PROPS_IF, path=uuids[FFF1])
            self._subscribed_path = uuids[FFF1]
        self.fff1.StartNotify()

    def _notif(self, _iface, changed, _inv):
        if "Value" in changed:
            v = bytes(changed["Value"])
            self.received.append(v)
            if len(v) > 2 and v[0] == 0xBA and v[2] == self.expected:
                self.loop.quit()

    def _expire(self):
        self.source = None
        self.loop.quit()
        return False

    def command(self, cmd, params=b"", timeout=3):
        if cmd not in READ_ONLY:
            raise SystemExit(f"Command 0x{cmd:02x} rejected. Only read-only commands are allowed.")
        return self._send(cmd, params, timeout)

    def reboot(self, params):
        """Send reboot command 02 with 0001 (bootloader) or 0000 (normal)."""
        if bytes(params) not in REBOOTS:
            raise SystemExit("Command 02 requires parameter 0000 or 0001.")
        return self._send(0x02, bytes(params), timeout=1)

    def _send(self, cmd, params, timeout):
        i = SAFE_SEQUENCES.index(self.seq) if self.seq in SAFE_SEQUENCES else -1
        self.seq = SAFE_SEQUENCES[(i + 1) % len(SAFE_SEQUENCES)]
        body = bytes([self.seq, cmd]) + params
        if 0xAB in body:
            raise SystemExit("Frame rejected because the body contains 0xAB.")
        frame = bytes([0xAB, len(body)]) + body
        # Process notifications already received before sending another frame.
        # A delayed reply may belong to a previous request.
        ctx = GLib.MainContext.default()
        while ctx.pending():
            ctx.iteration(False)
        self.late_replies = list(self.received)
        self.received.clear()
        self.expected = self.seq
        try:
            self.fff2.WriteValue(dbus.Array(frame, signature="y"), {"type": "request"})
        except dbus.exceptions.DBusException as e:
            # Reconnect after a dropped link without retransmitting the frame.
            print(f"\n  Bluetooth disconnected ({e.get_dbus_name()}). Reconnecting…")
            try:
                time.sleep(1)
                self._connect()
            except Exception as e2:
                print(f"  Reconnection failed: {e2}")
            return frame, None
        self.source = GLib.timeout_add(int(timeout * 1000), self._expire)
        self.loop.run()
        if self.source is not None:
            GLib.source_remove(self.source)
            self.source = None
        matches = [r for r in self.received if len(r) > 2 and r[0] == 0xBA and r[2] == self.seq]
        return frame, (matches[0] if matches else None)

    def close(self):
        try:
            self.fff1.StopNotify()
        finally:
            self.dev.Disconnect()


def read_memory(pod, start, end, path, attempts=5):
    """Back up memory in [start, end) using absolute MSP430 addresses.

    The file is appended to as data arrives. Rerun the same command to
    resume from the last saved byte.
    """
    import os
    already_read = os.path.getsize(path) if os.path.exists(path) else 0
    a = start + already_read
    if already_read:
        print(f"Resuming at 0x{a:04x} ({already_read} bytes already read)")
    with open(path, "ab") as f:
        while a < end:
            # Offsets containing 0xAB are rejected. Read from an earlier
            # address and discard the leading bytes.
            read_start = a
            while 0xAB in ((read_start - BASE) & 0xFFFF).to_bytes(2, "little"):
                read_start -= 1
            off = (read_start - BASE) & 0xFFFF
            for attempt in range(attempts):
                _, reply = pod.command(0x05, off.to_bytes(2, "little"), timeout=2)
                if (reply and reply[0] == 0xBA and len(reply) > 7 and reply[4] == 0
                        and int.from_bytes(reply[5:7], "little") == off):
                    break
                print(f"\n  No valid reply at 0x{a:04x}, attempt {attempt + 1}/{attempts}")
            else:
                raise SystemExit(f"Stopped at 0x{a:04x}. Run the same command to resume.")
            block = reply[7:][a - read_start:][:end - a]
            f.write(block)
            f.flush()
            a += len(block)
            print(f"\r0x{a:04x} / 0x{end:04x}", end="", flush=True)
    print(f"\nSaved {end - start} bytes to {path}.")


def version(pod):
    """Return the application or bootloader state and the version reply."""
    _, reply = pod.command(0x01, timeout=4)
    if reply and len(reply) >= 11 and reply[0] == 0xBA and reply[3] == 0x01 and reply[4] == 0:
        return {0x34: "stock", 0x09: "bootloader", 0xD1: "dice"}.get(reply[5], "unknown"), reply
    return None, reply


def safe_read_offset(rng, regions):
    while True:
        start, end = rng.choice(regions)
        a = rng.randrange(start, end - 13)
        off = a - BASE
        if not {off & 0xFF, off >> 8} & BLOCKED_BYTES:
            return a, off


def test_bootloader(pod, n, backup):
    """Enter the bootloader, compare n reads with the backup, and reboot.

    This diagnostic sends only read and reboot commands.
    """
    import random
    ref = open(backup, "rb").read()
    state, reply = version(pod)
    print(f"Before: {state} ({reply.hex(' ') if reply else 'no reply'})")
    if state not in ("stock", "dice"):
        raise SystemExit("The application did not return the expected version.")
    entered = False
    try:
        print("→ 02 01 00: enter bootloader")
        pod.reboot(b"\x01\x00")
        entered = True
        for attempt in range(5):
            time.sleep(1.5)
            state, reply = version(pod)
            print(f"  version: {state} ({reply.hex(' ') if reply else 'no reply'})")
            if state:
                break
        if state != "bootloader":
            raise SystemExit("The bootloader did not reply.")
        rng = random.Random(18)
        regions = [(0x2100, 0x7300), (0xEF00, 0x10000)]   # Stable application and bootloader regions.
        # Match each reply by sequence number. A final version request
        # allows time for the last read reply.
        id_off = (0x0FF0 - BASE) & 0xFFFF
        _, reply = pod.command(0x05, id_off.to_bytes(2, "little"), timeout=2)
        print(f"  Chip ID at 0x0FF0: {reply[7:9].hex(' ') if reply else 'no reply'}")
        pending_reads = {}
        ok = incorrect = consecutive_misses = 0
        t0 = time.time()
        for i in range(n + 1):
            if i < n:
                a, off = safe_read_offset(rng, regions)
                _, reply = pod.command(0x05, off.to_bytes(2, "little"), timeout=2)
                pending_reads[pod.seq] = (a, off)
            else:
                _, reply = pod.command(0x01, timeout=2)
            if not reply:
                consecutive_misses += 1
                print(f"  No reply to frame {i + 1}.")
                if consecutive_misses >= 3:
                    print("  Stopping after three consecutive missing replies.")
                    break
                continue
            consecutive_misses = 0
            request = pending_reads.pop(reply[2], None)
            if request is None and reply[3] == 0x01:
                continue                                  # Reply to the final version request.
            if request is None:
                print(f"  Reply without a pending read request: {reply.hex(' ')}")
                continue
            a, off = request
            expected = ref[a - BASE:a - BASE + 13]
            if (reply[0] == 0xBA and reply[3] == 0x05 and reply[4] == 0
                    and int.from_bytes(reply[5:7], "little") == off and reply[7:20] == expected):
                ok += 1
            else:
                incorrect += 1
                print(f"  Incorrect reply at 0x{a:04x}: {reply.hex(' ')}")
            if (i + 1) % 50 == 0:
                print(f"  {i + 1}/{n}: {ok} correct, {incorrect} incorrect", flush=True)
        duration = time.time() - t0
        print(f"Reads: {ok} correct, {incorrect} incorrect, {len(pending_reads)} unanswered out of {n}"
              f" in {duration:.1f} s ({1000 * duration / max(1, n):.0f} ms per frame)")
    finally:
        if entered:
            print("→ 02 00 00: restart the application")
            pod.reboot(b"\x00\x00")
            for attempt in range(5):
                time.sleep(1.5)
                state, reply = version(pod)
                print(f"  version: {state} ({reply.hex(' ') if reply else 'no reply'})")
                if state:
                    break


def monitor_axes(pod, duration):
    """Read DicePod's RAM at 0x1100 twice per second with command 05."""
    state, reply = version(pod)
    if state != "dice":
        raise SystemExit(f"The pod did not identify itself as DicePod ({state}).")
    counter_label = "changes" if reply[6] == 3 else "rolls"
    names = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"]
    end = time.time() + duration
    print("Rest the pod on each side. A raw reading of 64 is 1 g. Press Ctrl+C to stop.")
    while time.time() < end:
        _, reply = pod.command(0x05, ((0x1100 - BASE) & 0xFFFF).to_bytes(2, "little"), timeout=2)
        if reply and len(reply) >= 20 and reply[4] == 0:
            d = reply[7:]
            x, y, z = [(v - 256 if v > 127 else v) for v in d[4:7]]
            ax = max(range(3), key=lambda i: abs((x, y, z)[i]))
            axis = names[2 * ax + ((x, y, z)[ax] < 0)]
            count = int.from_bytes(d[8:12], "little")
            print(f"  X {x / 64:+.2f} g  Y {y / 64:+.2f} g  Z {z / 64:+.2f} g   axis pointing up: {axis}"
                  f"   reported face: {d[0] or '-'}   {counter_label}: {count}   I2C errors: {d[7]}", flush=True)
        time.sleep(0.5)


def main():
    args = sys.argv[1:]
    if "--help" in args or "-h" in args:
        print(__doc__)
        return
    if len(args) < 2:
        raise SystemExit(__doc__)
    if args[1].startswith("--") and args[1] not in ("--read", "--test-bootloader", "--axes"):
        raise SystemExit(f"Unknown option: {args[1]}")
    address = args[0]
    pod = Pod(address)
    try:
        if sys.argv[2] == "--read":
            read_memory(pod, int(sys.argv[3], 16), int(sys.argv[4], 16), sys.argv[5])
            return
        if sys.argv[2] == "--axes":
            monitor_axes(pod, float(sys.argv[3]) if len(sys.argv) > 3 else 3600)
            return
        if sys.argv[2] == "--test-bootloader":
            test_bootloader(pod, int(sys.argv[3]), sys.argv[4])
            return
        for arg in sys.argv[2:]:
            payload = bytes.fromhex(arg)
            frame, reply = pod.command(payload[0], payload[1:])
            print(f"→ {frame.hex(' ')}")
            print(f"← {reply.hex(' ') if reply else '(no reply)'}")
    finally:
        pod.close()


if __name__ == "__main__":
    main()
