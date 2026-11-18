#!/usr/bin/env python3
"""Read a Zwift RunPod over BLE through BlueZ D-Bus.

Usage: python3 runpod_read.py ADDRESS [listen_seconds, 0 = unlimited] [--dice]
Read the available GATT characteristics, then display notifications
for the requested duration.

--dice: decode RSC notifications and acknowledge stable faces on revision 3.
Disconnect when the device goes to sleep and reconnect when it is reachable.
On earlier revisions, send a version request once a minute to keep them active.
"""
import struct
import sys
import time

import dbus
import dbus.mainloop.glib
from gi.repository import GLib

BLUEZ = "org.bluez"
CHAR_IF = "org.bluez.GattCharacteristic1"
DESC_IF = "org.bluez.GattDescriptor1"
PROPS_IF = "org.freedesktop.DBus.Properties"
DEVICE_IF = "org.bluez.Device1"
CMD_READY = 0x30
CMD_ACK = 0x31
STATUS_IDLE = 0x02

LABELS = {
    "2a00": "Device name", "2a01": "Appearance", "2a04": "Connection parameters",
    "2a19": "Battery (%)", "2a23": "System ID", "2a24": "Model", "2a25": "Serial number",
    "2a26": "Firmware", "2a27": "Hardware revision", "2a28": "Software revision",
    "2a29": "Manufacturer", "2a2a": "Regulatory certification", "2a50": "PnP ID",
    "2a53": "RSC measurement (running)", "2a54": "RSC features", "2a5d": "Sensor location",
    "2a55": "SC control point", "2a05": "Service Changed", "2aa6": "Central Address Resolution",
}
POSITIONS = {0: "other", 1: "top of shoe", 2: "in shoe", 3: "hip",
             4: "front wheel", 5: "left crank", 6: "right crank", 7: "left pedal",
             8: "right pedal", 9: "front hub", 10: "rear", 11: "rear hub",
             12: "chest", 13: "spider", 14: "chain ring"}


def short_uuid(uuid):
    u = str(uuid).lower()
    return u[4:8] if u.endswith("-0000-1000-8000-00805f9b34fb") else u


def decode(code, b):
    b = bytes(b)
    if code == "2a19":
        return f"{b[0]} %"
    if code == "2a50" and len(b) >= 7:
        src, vid, pid, ver = struct.unpack("<BHHH", b[:7])
        return f"source={src} vendor=0x{vid:04x} product=0x{pid:04x} version=0x{ver:04x}"
    if code == "2a54" and len(b) >= 2:
        f = struct.unpack("<H", b[:2])[0]
        names = ["stride length", "total distance", "walking/running", "calibration",
                "multiple sensor locations"]
        return ", ".join(n for i, n in enumerate(names) if f >> i & 1) or "none"
    if code == "2a5d":
        return POSITIONS.get(b[0], f"code {b[0]}")
    if code == "2a53":
        return decode_rsc(b)
    if code == "fff1" and len(b) > 4 and b[0] == 0xBA:
        if DICE_MODE and b[3] == 0x84 and len(b) >= 14:
            return "frame 0x84 (fff1): " + decode_dice(b[4:])
        if b[3] == 0x01 and len(b) >= 11:
            name = {0x34: "stock application", 0x09: "bootloader", 0xD1: "dice"}.get(b[5], "unknown")
            return f"version: {name}"
    if b and all(32 <= c < 127 for c in b):
        return repr(bytes(b).decode())
    return None


DICE_MODE = "--dice" in sys.argv


def dice_measurement(b):
    """Decode the face and unscaled counter from an RSC notification."""
    b = bytes(b)
    if len(b) < 4:
        return None
    face, i = b[3], 4 + (2 if b[0] & 1 else 0)
    count = struct.unpack_from("<I", b, i)[0] if b[0] & 2 and len(b) >= i + 4 else None
    return face, count


def decode_dice(b):
    measurement = dice_measurement(b)
    if measurement is None:
        return None
    face, rolls = measurement
    return f"FACE {face if face else '— (moving or on an edge)'}, rolls: {rolls}"


def decode_rsc(b):
    b = bytes(b)
    if DICE_MODE:
        return decode_dice(b)
    flags = b[0]
    speed = struct.unpack("<H", b[1:3])[0] / 256  # m/s
    cadence = b[3]
    i = 4
    txt = f"speed {speed:.2f} m/s ({speed * 3.6:.1f} km/h), cadence {cadence} steps/min"
    if flags & 1:
        txt += f", stride length {struct.unpack('<H', b[i:i + 2])[0] / 100:.2f} m"
        i += 2
    if flags & 2:
        txt += f", distance {struct.unpack('<I', b[i:i + 4])[0] / 10:.1f} m"
    txt += ", activity: " + ("running" if flags & 4 else "walking")
    return txt


def discover(bus, device_path, timeout=30):
    """Scan until BlueZ discovers the pod. Shake the pod to wake it up."""
    def is_known():
        objects = dbus.Interface(bus.get_object(BLUEZ, "/"),
                                "org.freedesktop.DBus.ObjectManager").GetManagedObjects()
        return device_path in objects
    if is_known():
        return True
    print(f"BlueZ has not discovered the pod. Shake it during the Bluetooth scan (up to {timeout} s).", flush=True)
    adapter = dbus.Interface(bus.get_object(BLUEZ, "/org/bluez/hci0"), "org.bluez.Adapter1")
    try:
        adapter.StartDiscovery()
    except dbus.DBusException as e:
        if "InProgress" not in e.get_dbus_name():
            raise
    try:
        end = time.time() + timeout
        while time.time() < end:
            if is_known():
                print("  Found the pod.", flush=True)
                return True
            time.sleep(0.5)
        return False
    finally:
        try:
            adapter.StopDiscovery()
        except dbus.DBusException:
            pass


def main():
    args = sys.argv[1:]
    if "--help" in args or "-h" in args:
        print(__doc__)
        return
    unknown = [arg for arg in args if arg.startswith("--") and arg != "--dice"]
    if unknown:
        raise SystemExit(f"Unknown option: {unknown[0]}")
    args = [arg for arg in args if arg != "--dice"]
    if not args:
        raise SystemExit(__doc__)
    address = args[0].upper()
    duration = int(args[1]) if len(args) > 1 else 30
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()
    device_path = "/org/bluez/hci0/dev_" + address.replace(":", "_")
    deadline = time.monotonic() + duration if duration > 0 else None
    dice = {"revision": None, "last_face": None, "seq": 0x16}
    try:
        while deadline is None or time.monotonic() < deadline:
            if not listen(bus, device_path, deadline, dice):
                return
            time.sleep(2 if deadline is None else min(2, max(0, deadline - time.monotonic())))
    except KeyboardInterrupt:
        pass


def listen(bus, device_path, deadline, dice):
    scan_time = 30 if deadline is None else min(30, max(0, deadline - time.monotonic()))
    if not discover(bus, device_path, timeout=scan_time):
        if DICE_MODE:
            return True
        raise SystemExit("Pod not found. Shake it near the computer and try again.")
    dev = bus.get_object(BLUEZ, device_path)
    props = dbus.Interface(dev, PROPS_IF)
    device = dbus.Interface(dev, DEVICE_IF)
    connected = False
    try:
        if not props.Get(DEVICE_IF, "Connected"):
            device.Connect()
        connected = True
        return listen_connected(bus, device_path, props, deadline, dice)
    except dbus.DBusException:
        if not DICE_MODE or dice["revision"] not in (None, 3):
            raise
        return True
    finally:
        if connected:
            try:
                device.Disconnect()
            except dbus.DBusException:
                pass


def listen_connected(bus, device_path, props, deadline, dice):
    for _ in range(40):                         # Allow up to 20 s for service discovery.
        if props.Get(DEVICE_IF, "ServicesResolved"):
            break
        time.sleep(0.5)
    time.sleep(0.5)                             # Give BlueZ time to publish the characteristics.

    objects = dbus.Interface(bus.get_object(BLUEZ, "/"),
                            "org.freedesktop.DBus.ObjectManager").GetManagedObjects()
    chars = sorted((p, i[CHAR_IF]) for p, i in objects.items()
                   if p.startswith(device_path) and CHAR_IF in i)
    descs = {p: i[DESC_IF] for p, i in objects.items() if p.startswith(device_path) and DESC_IF in i}

    print("\n=== Characteristics ===")
    subscriptions = []
    for path, c in chars:
        code = short_uuid(c["UUID"])
        flags = [str(f) for f in c["Flags"]]
        # Read the device-provided description (descriptor 0x2901).
        desc_txt = ""
        for dp, d in descs.items():
            if dp.startswith(path + "/") and short_uuid(d["UUID"]) == "2901":
                try:
                    v = dbus.Interface(bus.get_object(BLUEZ, dp), DESC_IF).ReadValue({})
                    desc_txt = bytes(v).decode(errors="replace").strip("\x00")
                except dbus.DBusException:
                    pass
        print(f"\n[{code}] {LABELS.get(code, '')} {repr(desc_txt) if desc_txt else ''}")
        print(f"   properties: {', '.join(flags)}")
        if "read" in flags:
            try:
                v = list(dbus.Interface(bus.get_object(BLUEZ, path), CHAR_IF).ReadValue({}))
                print(f"   raw: {bytes(v).hex(' ') or '(empty)'}")
                d = decode(code, v)
                if d:
                    print(f"   value: {d}")
            except dbus.DBusException as e:
                print(f"   read failed: {e.get_dbus_message()}")
        if "notify" in flags or "indicate" in flags:
            subscriptions.append((path, code))

    fff2 = next((p for p, c in chars if short_uuid(c["UUID"]) == "fff2"), None)
    return listen_notifications(bus, device_path, subscriptions, fff2, deadline, dice)


def listen_notifications(bus, device_path, subscriptions, fff2, deadline, dice):
    print("\n=== Notifications. Move the pod. ===")
    t0 = time.monotonic()
    control = dbus.Interface(bus.get_object(BLUEZ, fff2), CHAR_IF) if DICE_MODE and fff2 else None
    loop = GLib.MainLoop()
    signal_matches = []
    timeouts = []
    subscribed = set()
    reconnect = False
    version_seq = ready_seq = None
    handshake_complete = False

    def stop_for_reconnect():
        nonlocal reconnect
        reconnect = True
        loop.quit()

    def send_command(command, params=b""):
        seq = dice["seq"] + 1 if dice["seq"] < 0xAA else 0x17
        dice["seq"] = seq
        frame = bytes([0xAB, 2 + len(params), seq, command]) + params
        control.WriteValue(dbus.Array(frame, signature="y"), {"type": "request"})
        return seq

    def keep_alive():
        try:
            send_command(0x01)
        except dbus.DBusException as e:
            print(f"   version request (01) failed: {e.get_dbus_message()}")
        return True

    def print_notification(code, data, label):
        print(f"{time.monotonic() - t0:6.1f}s [{code}] {data.hex(' ')}  {label}", flush=True)

    def receive_control(data):
        nonlocal ready_seq, handshake_complete
        if len(data) < 5 or data[0] != 0xBA:
            return False
        seq, command, status = data[2:5]
        if command == 0x01 and seq == version_seq and status == 0:
            if len(data) >= 11 and data[5:7] == b"\xd1\x03":
                dice["revision"] = 3
                if "2a53" in subscribed:
                    ready_seq = send_command(CMD_READY)
            else:
                dice["revision"] = 2
                handshake_complete = True
                timeouts.append(GLib.timeout_add_seconds(60, keep_alive))
            return False
        if dice["revision"] != 3:
            return False
        if command == CMD_READY and seq == ready_seq:
            if status == STATUS_IDLE:
                print("DicePod is going to sleep. Waiting for movement.", flush=True)
                stop_for_reconnect()
            elif status == 0:
                handshake_complete = True
            return True
        return command in (CMD_ACK, 0x84)

    def receive_face(data):
        if ready_seq is None:
            return
        measurement = dice_measurement(data)
        if measurement is None:
            return
        face, count = measurement
        if not 1 <= face <= 6 or count is None:
            return
        if measurement != dice["last_face"]:
            print_notification("2a53", data, f"FACE {face}, changes: {count}")
            dice["last_face"] = measurement
        send_command(CMD_ACK, struct.pack("<BI", face, count))

    def received(code):
        def cb(_interface, changed, _invalid):
            if "Value" not in changed:
                return
            data = bytes(changed["Value"])
            try:
                if control and code == "fff1" and receive_control(data):
                    return
                if dice["revision"] == 3 and code == "2a53":
                    receive_face(data)
                    return
            except dbus.DBusException:
                stop_for_reconnect()
                return
            print_notification(code, data, decode(code, data) or "")
        return cb

    def connection_changed(interface, changed, _invalid):
        if interface == DEVICE_IF and "Connected" in changed and not changed["Connected"]:
            if DICE_MODE and dice["revision"] in (None, 3):
                stop_for_reconnect()
            else:
                loop.quit()

    def check_handshake():
        if not handshake_complete:
            stop_for_reconnect()
        return False

    try:
        for path, code in subscriptions:
            obj = bus.get_object(BLUEZ, path)
            match = bus.add_signal_receiver(received(code), "PropertiesChanged", PROPS_IF, path=path)
            signal_matches.append(match)
            try:
                dbus.Interface(obj, CHAR_IF).StartNotify()
                subscribed.add(code)
                print(f"   subscribed to [{code}]")
            except dbus.DBusException as e:
                print(f"   [{code}] subscription failed: {e.get_dbus_name()}")
        signal_matches.append(bus.add_signal_receiver(connection_changed, "PropertiesChanged", PROPS_IF,
                                                      path=device_path))
        if control and "fff1" in subscribed:
            version_seq = send_command(0x01)
            timeouts.append(GLib.timeout_add_seconds(4, check_handshake))
        elif DICE_MODE and dice["revision"] == 3:
            return True
        if deadline is not None:
            remaining_ms = max(1, int((deadline - time.monotonic()) * 1000))
            timeouts.append(GLib.timeout_add(remaining_ms, loop.quit))
        loop.run()
    finally:
        for match in signal_matches:
            match.remove()
        context = GLib.MainContext.default()
        for source in timeouts:
            if context.find_source_by_id(source) is not None:
                GLib.source_remove(source)
    return reconnect


if __name__ == "__main__":
    main()
