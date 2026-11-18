# RunPod firmware and Bluetooth

DicePod is written in MSP430 assembly. Its design draws on reverse engineering of the original Zwift RunPod firmware and bootloader. This analysis informed the hardware initialization and the communication between the MSP430 and the Bluetooth chip.

## From the sensor to the notification

The MSP430 reads the NXP MMA845xQ accelerometer over I²C at address `0x1C`. The active sensor runs at 50 Hz with a ±2 g range. DicePod samples at 25 Hz. Eight steady samples on the same face confirm the orientation. At startup the first stable face sets the baseline. Each later stable face different from the previous one increments the 32-bit counter. Motion, ambiguous tilt and returning to the previous face produce no notification.

The corrected D1.3 build reads all three axes in one I²C burst at 40 kHz, below the limit for repeated starts in MSP430G2955 erratum USCI35. After a timeout, the firmware resets the controller. If the accelerometer holds SDA low, the firmware pulses SCL up to nine times, then generates STOP once SDA releases. [MSP430G2955 errata](https://www.ti.com/lit/pdf/SLAZ441) · [I²C bus specification](https://www.nxp.com/docs/en/user-guide/UM10204.pdf)

The MSP430 sends a `0x84` UART frame for a pending face when a client is ready. The Bluetooth chip publishes the measurement on GATT characteristic **Running Speed and Cadence Measurement (`2A53`)**. The cadence byte represents the face and the 32-bit distance field holds the unscaled face-change counter in little-endian order. Speed and stride length are zero.

1. Subscribe to `FFF1` for command replies and `2A53` for measurements. Identify D1.3 by sending the version command `AB 02 SEQ 01` to `FFF2`. The reply is `BA 09 SEQ 01 00 D1 03 04 02 29 55`.
2. Write `AB 02 SEQ 30` to `FFF2` after both subscriptions succeed. The success reply is `BA 03 SEQ 30 00`. The firmware sends the pending measurement once on this subscription. With multiple offline changes it keeps the latest stable face.
3. After accepting a measurement, write `AB 07 SEQ 31 FACE C0 C1 C2 C3` to `FFF2`. Use the face and four counter bytes from the notification. Acknowledging that pair clears the pending result. A new subscription can receive an unconfirmed result again, so the client should recognize duplicates by face and counter. The counter and face are stored in RAM.
4. On `BA 03 SEQ 30 02`, with the sequence from step 2, disconnect. Discover the device again and repeat the subscription on reconnect. The Linux reader implements these exchanges in `--dice` mode.

After 120 seconds without motion, D1.3 asks the subscribed client to disconnect, stops the UART and enters MSP430 LPM3. The accelerometer runs at 6.25 Hz in low-power mode. A transient interrupt on P1.2 wakes the MSP430. Sampling at 4 Hz during standby also detects gradual orientation changes. On wake, DicePod uses the original Bluetooth controller's wake handshake.

The Bluetooth radio runs separate firmware. On the tested RunPod, the Linux reader received the standby request and closed the link. A subsequent Bluetooth LE scan still received advertisements from the RunPod. Standby current and face delivery after waking and reconnecting remain unverified on hardware.

## Installation and recovery

The stock bootloader lives at `0xEF00–0xFFFF`. It checks the application header at `0x2100` before starting the application. The header contains two little-endian 16-bit values, the body length and its Fletcher-16 checksum. The body starts at `0x2104` with the interrupt vector table.

The installer limits writes and erases to the application area through `0xEDFF`. It checks the write plan before connecting and validates each frame before sending it. It erases the required segments, writes and verifies the body, then writes and verifies the header. Frame validation models the loss of one byte. Losing more than one byte can disrupt installation.

If installation stops, check the last step in `logs/installation_*.log`. During body writes, the application header is invalid. The bootloader waits for a new installation, even after a power failure. Once Bluetooth is available again, retry the installation or the stock firmware restore.

D1.3 enters the bootloader through the reboot command or a watchdog reset. Wake a sleeping RunPod before sending the reboot command. For D1.2, the reader's `--dice` mode still sends a version request every minute to avoid the 10-minute bootloader fallback. Use the installer's `--force-bootloader` option if the running firmware receives the reboot command but cannot reply to a version request. The installer waits for a bootloader reply before writing flash.

## Extract the stock application from a backup

Use a full flash backup made **before installing DicePod**. The extraction uses the length stored in the header. The useful part of the reference firmware, version 4.0.52.21801, is 20,762 bytes. Zwift's download adds 512 bytes of `ff` padding.

From the repository root, create a recovery image from your backup. Opening the output in `xb` mode protects an existing recovery file.

```bash
python3 - <<'PY'
from pathlib import Path

flash = Path("backups/flash_2100_ffff.bin").read_bytes()
if len(flash) != 57_088:
    raise SystemExit("Incomplete flash backup")
end = 4 + int.from_bytes(flash[:2], "little")
if not 36 < end <= 0xEE00 - 0x2100:
    raise SystemExit("Invalid application length")
with Path("backups/stock-extracted.mp").open("xb") as output:
    output.write(flash[:end])
PY
python3 -I runpod_installer.py backups/stock-extracted.mp --plan
```

For 4.0.52.21801, the plan should report a **20,758-byte** body and Fletcher-16 **`0x9c2a`**. To confirm the extracted bytes match the reference image, also verify the SHA-256.

```bash
printf '%s  %s\n' '4f2673508bd621c641460dc0395f96852fd8f56e587595f0e20efe03878bf6a5' \
  backups/stock-extracted.mp | sha256sum -c -
```

After verification, use `backups/stock-extracted.mp` in the stock firmware restore command.
