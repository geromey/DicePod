"""Configure a DicePod by its Bluetooth address."""

import re

import voluptuous as vol

from homeassistant import config_entries
from homeassistant.const import CONF_ADDRESS

DOMAIN = "dicepod"
ADDRESS = re.compile(r"(?:[0-9A-F]{2}:){5}[0-9A-F]{2}")


class DicePodConfigFlow(config_entries.ConfigFlow, domain=DOMAIN):
    """Add one configuration entry per die."""

    async def async_step_user(self, user_input=None):
        """Ask for the die's Bluetooth address."""
        errors = {}
        if user_input is not None:
            address = user_input[CONF_ADDRESS].strip().upper()
            if ADDRESS.fullmatch(address):
                await self.async_set_unique_id(address)
                self._abort_if_unique_id_configured()
                return self.async_create_entry(title="DicePod", data={CONF_ADDRESS: address})
            errors[CONF_ADDRESS] = "invalid_address"

        return self.async_show_form(
            step_id="user",
            data_schema=vol.Schema({vol.Required(CONF_ADDRESS): str}),
            errors=errors,
        )
