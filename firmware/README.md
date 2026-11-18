# Firmware development

[`dice.s`](dice.s) is the DicePod firmware in MSP430 assembly. [`asm430.py`](asm430.py) assembles it and generates the application header. [`dice_v3.mp`](dice_v3.mp) is the current D1.3 image. [`dice_v2.mp`](dice_v2.mp) is the D1.2 image verified on the RunPod. You only need Python 3 to rebuild D1.3.

## Build and validate the image

Run these commands from the repository root.

```bash
mkdir -p logs
python3 -I firmware/asm430.py firmware/dice.s logs/dice.mp --list logs/dice.lst
cmp firmware/dice_v3.mp logs/dice.mp
python3 -I runpod_installer.py logs/dice.mp --plan
```

To change the face mapping, edit `TABLE_FACES` in the order `+X, −X, +Y, −Y, +Z, −Z`. Update the expected faces in `firmware/test_dice.py`, then rebuild and test the modified image before installing it.

## Test without Bluetooth

The emulator runs the original bootloader from a backup and models the accelerometer, UART, and watchdog. Set up `backups/flash_2100_ffff.bin`, `backups/info_1000_10ff.bin`, and `backups/original.mp` by following the [installation guide](../README.md#2-back-up-the-runpod-before-flashing). The bootloader tests require a flash backup that contains the reference stock application.

```bash
python3 -I firmware/test_dice.py logs/dice.mp
python3 -I runpod_installer.py logs/dice.mp --emulate --run
python3 -I test_bootloader.py
python3 -I test_installer.py
```

The die tests cover all six orientations, face changes, standby, wake on movement, notification acknowledgment, UART communication, bootloader entry and watchdog recovery. The emulator models the MSP430 side of the Bluetooth exchange. The last two suites include recovery from an interrupted installation and readback verification. See the [technical guide](../technical.md) for communication and recovery details.
