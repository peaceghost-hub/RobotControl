"""Optional cloud integrations (ThingSpeak & Blynk)"""

import logging
from typing import Dict, Any

import requests

logger = logging.getLogger('cloud_uploader')


class CloudUploader:
    def __init__(self, config: Dict[str, Any]):
        self.thingspeak = config.get('thingspeak', {}) if config else {}
        self.blynk = config.get('blynk', {}) if config else {}

    # ------------------------------------------------------------------
    def publish_sensor_data(self, data: Dict[str, Any]) -> None:
        if self.thingspeak.get('enabled'):
            self._send_thingspeak(data)
        if self.blynk.get('enabled'):
            self._send_blynk(data)

    # ------------------------------------------------------------------
    def _send_thingspeak(self, data: Dict[str, Any]) -> None:
        api_key = self.thingspeak.get('api_key')
        if not api_key:
            logger.warning("ThingSpeak enabled without API key")
            return

        field_map = self.thingspeak.get('field_map', {})
        payload = {'api_key': api_key}

        for name, field_num in field_map.items():
            value = data.get(name)
            if value is not None:
                payload[f'field{field_num}'] = value

        try:
            resp = requests.post('https://api.thingspeak.com/update', data=payload, timeout=5)
            if resp.status_code != 200:
                logger.warning("ThingSpeak update failed: %s - %s", resp.status_code, resp.text)
        except Exception as exc:
            logger.error("ThingSpeak update error: %s", exc)

    def _send_blynk(self, data: Dict[str, Any]) -> None:
        token = self.blynk.get('auth_token')
        pins = self.blynk.get('virtual_pins', {})
        if not token or not pins:
            logger.warning("Blynk enabled without auth token or pin mapping")
            return

        updates = {}
        for name, pin in pins.items():
            value = data.get(name)
            if value is not None:
                updates[pin] = value

        if not updates:
            return

        try:
            params = {'token': token}
            for pin, value in updates.items():
                params[pin] = value
            resp = requests.get('https://blynk.cloud/external/api/batch/update', params=params, timeout=5)
            if resp.status_code != 200:
                logger.warning("Blynk update failed: %s - %s", resp.status_code, resp.text)
        except Exception as exc:
            logger.error("Blynk update error: %s", exc)
