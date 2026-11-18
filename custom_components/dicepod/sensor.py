"""Last stable face reported by a DicePod running firmware D1.3."""

import asyncio
from contextlib import suppress
import logging

from bleak import BleakError
from bleak.backends.device import BLEDevice
from bleak_retry_connector import BleakClientWithServiceCache, establish_connection

from homeassistant.components import bluetooth
from homeassistant.components.sensor import RestoreSensor
from homeassistant.config_entries import ConfigEntry
from homeassistant.const import CONF_ADDRESS
from homeassistant.core import HomeAssistant, callback
from homeassistant.helpers import device_registry as dr
from homeassistant.helpers.entity_platform import AddConfigEntryEntitiesCallback

DOMAIN = "dicepod"
_LOGGER = logging.getLogger(__name__)
REPLY_CHAR = "0000fff1-0000-1000-8000-00805f9b34fb"
WRITE_CHAR = "0000fff2-0000-1000-8000-00805f9b34fb"
FACE_CHAR = "00002a53-0000-1000-8000-00805f9b34fb"
VERSION = 0x01
READY = 0x30
ACK = 0x31
STANDBY = 0x02
INACTIVITY_TIMEOUT = 120


class ProtocolError(Exception):
    """Unexpected response from the die."""


def command_frame(sequence: int, command: int, payload: bytes = b"") -> bytes:
    """Build a DicePod command."""
    return bytes((0xAB, 2 + len(payload), sequence, command)) + payload


def face_payload(data: bytes) -> bytes | None:
    """Extract the face and four original counter bytes from an RSC frame."""
    if len(data) < 8 or not data[0] & 0x02 or not 1 <= data[3] <= 6:
        return None
    offset = 6 if data[0] & 0x01 else 4
    if len(data) != offset + 4:
        return None
    return data[3:4] + data[offset:]


async def async_setup_entry(
    hass: HomeAssistant, entry: ConfigEntry, async_add_entities: AddConfigEntryEntitiesCallback
) -> None:
    """Add a sensor to the existing iBeacon device when present."""
    address = entry.data[CONF_ADDRESS]
    identifiers = {(DOMAIN, address)}
    for device in dr.async_get(hass).devices.values():
        for domain, identifier in device.identifiers:
            if domain == "ibeacon" and identifier.endswith(f"_{address}"):
                identifiers.add((domain, identifier))

    async_add_entities([DicePodFace(entry, address, identifiers)])


class DicePodFace(RestoreSensor):
    """Listen for stable faces and keep the last reading during standby."""

    _attr_has_entity_name = True
    _attr_translation_key = "face"
    _attr_icon = "mdi:dice-6"
    _attr_should_poll = False
    _attr_native_value = None

    def __init__(self, entry: ConfigEntry, address: str, identifiers: set[tuple[str, str]]) -> None:
        self._entry = entry
        self._address = address
        self._task = None
        self._attr_unique_id = address
        self._attr_device_info = dr.DeviceInfo(
            identifiers=identifiers,
            connections={(dr.CONNECTION_BLUETOOTH, address)},
            default_name="DicePod",
        )

    async def async_added_to_hass(self) -> None:
        """Restore the known face and start the Bluetooth listener."""
        await super().async_added_to_hass()
        if (last := await self.async_get_last_sensor_data()) and last.native_value in range(1, 7):
            self._attr_native_value = last.native_value
        self._task = self._entry.async_create_background_task(
            self.hass, self._listen(), f"DicePod {self._address}"
        )

    async def async_will_remove_from_hass(self) -> None:
        """Stop listening and release the Bluetooth connection."""
        if self._task:
            self._task.cancel()
            with suppress(asyncio.CancelledError):
                await self._task
        await super().async_will_remove_from_hass()

    async def _listen(self) -> None:
        retry_delay = 5
        while True:
            device = await self._wait_for_device()
            try:
                await self._session(device)
            except (BleakError, ConnectionError, ProtocolError, TimeoutError) as err:
                _LOGGER.debug("DicePod %s: %s", self._address, err)
                await asyncio.sleep(retry_delay)
                retry_delay = min(30, retry_delay * 2)
            else:
                retry_delay = 5

    async def _wait_for_device(self) -> BLEDevice:
        """Wait for a fresh advertisement, even if its payload is unchanged."""
        found: asyncio.Future[BLEDevice] = asyncio.get_running_loop().create_future()

        @callback
        def device_seen(info: bluetooth.BluetoothServiceInfoBleak) -> None:
            if info.connectable and not found.done():
                found.set_result(info.device)

        unregister = bluetooth.async_register_advertisement_callback(
            self.hass, device_seen, self._address
        )
        try:
            return await found
        finally:
            unregister()

    async def _session(self, device: BLEDevice) -> None:
        messages: asyncio.Queue[tuple[str, bytes] | None] = asyncio.Queue()
        loop = asyncio.get_running_loop()
        inactivity: asyncio.Timeout | None = None

        def notify(source: str, data: bytearray) -> None:
            if inactivity is not None and not inactivity.expired():
                inactivity.reschedule(loop.time() + INACTIVITY_TIMEOUT)
            messages.put_nowait((source, bytes(data)))

        client = await establish_connection(
            BleakClientWithServiceCache,
            device,
            f"DicePod {self._address}",
            disconnected_callback=lambda _: messages.put_nowait(None),
            max_attempts=1,
        )
        sequence = 0x16

        async def send(command: int, payload: bytes = b"") -> int:
            nonlocal sequence
            sequence = sequence + 1 if sequence < 0xAA else 0x17
            async with asyncio.timeout(6):
                await client.write_gatt_char(
                    WRITE_CHAR, command_frame(sequence, command, payload), response=True
                )
            return sequence

        try:
            async with asyncio.timeout(12):
                await client.start_notify(
                    REPLY_CHAR, lambda _, data: notify(REPLY_CHAR, data)
                )
                await client.start_notify(
                    FACE_CHAR, lambda _, data: notify(FACE_CHAR, data)
                )
                version_sequence = await send(VERSION)
                while (message := await messages.get()) is not None:
                    source, data = message
                    if source == REPLY_CHAR and data[:4] == bytes((0xBA, 9, version_sequence, VERSION)):
                        if len(data) != 11 or data[4:7] != bytes((0, 0xD1, 0x03)):
                            raise ProtocolError(f"Firmware D1.3 expected, got {data.hex(' ')}")
                        break
                else:
                    raise ConnectionError("Disconnected before firmware identification")

                ready_sequence = await send(READY)
                while (message := await messages.get()) is not None:
                    source, data = message
                    if source == REPLY_CHAR and data[:4] == bytes((0xBA, 3, ready_sequence, READY)):
                        if data[4:] != b"\x00":
                            raise ProtocolError(f"Subscription rejected: {data.hex(' ')}")
                        break
                else:
                    raise ConnectionError("Disconnected before subscription confirmation")

            standby = bytes((0xBA, 3, ready_sequence, READY, STANDBY))
            try:
                async with asyncio.timeout(INACTIVITY_TIMEOUT) as inactivity:
                    while (message := await messages.get()) is not None:
                        source, data = message
                        if source == REPLY_CHAR and data == standby:
                            return
                        if source == FACE_CHAR and (payload := face_payload(data)) is not None:
                            if self._attr_native_value != payload[0]:
                                self._attr_native_value = payload[0]
                                self.async_write_ha_state()
                            await send(ACK, payload)
            except TimeoutError:
                if not inactivity.expired():
                    raise
                _LOGGER.debug(
                    "DicePod %s: no notifications for %s seconds",
                    self._address,
                    INACTIVITY_TIMEOUT,
                )
        finally:
            inactivity = None
            if client.is_connected:
                with suppress(BleakError, TimeoutError):
                    async with asyncio.timeout(8):
                        await client.disconnect()
