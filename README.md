# DicePod

DicePod turns a **Zwift RunPod** into a six-sided Bluetooth remote. Choose an action by resting the die on the corresponding face. Install the firmware over Bluetooth Low Energy from Linux without opening the sensor.

Revision **D1.3** was installed and verified by readback on a Zwift RunPod with an MSP430G2955 (chip ID **29 55**). The Linux reader disconnected after two minutes without movement. The RunPod remained visible in Bluetooth LE scans during standby. Wake on motion and delivery after reconnection have been tested in the emulator. A hardware test of that full sequence remains pending.

## 1. Set up Linux and find your RunPod

You need a Bluetooth LE adapter managed by BlueZ. The tools use the first adapter (`hci0`). On Debian or MX Linux, install the system dependencies and clone the repository.

```bash
sudo apt install git bluez python3 python3-dbus python3-gi gir1.2-glib-2.0 curl
git clone https://github.com/geromey/DicePod.git
cd DicePod
bluetoothctl power on
bluetoothctl --timeout 20 scan on
```

Shake the RunPod during the scan. Look for its `Zwift RunPod` entry and copy its Bluetooth address. Replace the example address below and use the same terminal for the remaining steps.

```bash
POD='AA:BB:CC:DD:EE:FF'
python3 -I runpod_cmd.py "$POD" 01
```

The tested stock firmware replies with `ba 09 … 01 00 34 00 04 01 29 55`. If you cannot find the pod, bring it closer, shake it, and close other apps that might connect to it, such as Zwift on your phone. The tools scan for up to 30 seconds if BlueZ has yet to discover the pod.

## 2. Back up the RunPod before flashing

These commands read flash and information memory. The end address is exclusive. If a read is interrupted, run the same command again with the same pod to resume.

```bash
mkdir -p backups
python3 -I runpod_cmd.py "$POD" --read 2100 10000 backups/flash_2100_ffff.bin
python3 -I runpod_cmd.py "$POD" --read 1000 1100 backups/info_1000_10ff.bin
wc -c backups/flash_2100_ffff.bin backups/info_1000_10ff.bin
```

The expected sizes are **57,088** and **256** bytes. Keep these files in a safe place. They may contain the pod's activity log and calibration data. Git ignores the `backups/` directory.

Download the stock application from Zwift and verify its checksum.

```bash
curl --fail --location --output backups/original.mp \
  https://cdn.zwift.com/public/fw/runpod/a5c72f97f21838a9f6a2159b330eb570_msp430_v4.a43.mp
printf '%s  %s\n' '7aa20c7c32832f46f8bad04df81900255283f4f27dcc2aea9a4946c71cda0174' \
  backups/original.mp | sha256sum -c -
```

If the download is unavailable, [extract the stock application from your backup](technical.md#extract-the-stock-application-from-a-backup).

## 3. Install DicePod

The first command validates the image and write plan offline. The second replaces the running firmware. Prepare the recovery image in step 2 before flashing. Use a good battery, keep the pod near the computer, and close any other RunPod readers before starting.

```bash
python3 -I runpod_installer.py firmware/dice_v3.mp --plan
python3 -I runpod_installer.py firmware/dice_v3.mp "$POD" --confirm
```

The installer reads back the data to verify it, then restarts the pod. On success, the output shows `Pod: dice` and a version reply containing `d1 03`. The installation log is in `logs/`.

## 4. Read and orient the die

```bash
python3 runpod_read.py "$POD" 0 --dice
```

Move the die before connecting if it has been idle. The first stable face at power-up sets the starting position. Rest the die on a different face to display a line such as `FACE 6, changes: 5`. The counter records stable face changes since startup. After two minutes without movement, the die enters standby and the reader disconnects. Press Ctrl+C to stop.

A Home Assistant integration can subscribe, receive and acknowledge face changes using the [Bluetooth protocol](technical.md#from-the-sensor-to-the-notification).

To make a physical die, secure the RunPod inside a cube with access to the battery. The default mapping is **+Z = 1**, **−Z = 6**, **+X = 2**, **−X = 5**, **+Y = 3**, and **−Y = 4**. Rest the cube on each side and run the following command to identify the axis pointing up. Use that mapping to label the faces, or edit `TABLE_FACES` if the cube is already numbered.

```bash
python3 -I runpod_cmd.py "$POD" --axes
```

## Restore the stock firmware

Stop the reader, wake the pod, and install the verified stock image from step 2.

```bash
python3 -I runpod_installer.py backups/original.mp --plan
python3 -I runpod_installer.py backups/original.mp "$POD" --confirm
python3 -I runpod_cmd.py "$POD" 01
```

The version reply should again contain `34 00 04 01 29 55`. If installation is interrupted while writing the image body, the bootloader normally stays active and the installer detects it on the next run. See the [technical guide](technical.md) for diagnostics and recovery.

To modify the firmware, follow the [build and test guide](firmware/README.md).
