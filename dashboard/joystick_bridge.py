"""USB serial bridge for ESP8266 joystick control frames."""

from __future__ import annotations

import json
import logging
import re
import threading
import time
from typing import Callable, Optional

try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover - optional dependency in dev envs
    serial = None

logger = logging.getLogger(__name__)
THR_STR_RE = re.compile(r"Thr:\s*(-?\d+)\s+Str:\s*(-?\d+)", re.IGNORECASE)
LEFT_RIGHT_RE = re.compile(r"Left:\s*(-?\d+)\s*\|\s*Right:\s*(-?\d+)", re.IGNORECASE)


class JoystickBridge:
    """Read joystick packets from a USB serial device in the background."""

    SERIAL_AVAILABLE = serial is not None

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        read_timeout: float = 0.15,
        device_id: Optional[str] = None,
        on_message: Optional[Callable[[dict], None]] = None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.read_timeout = max(0.05, float(read_timeout))
        self.device_id = device_id or "esp8266_joystick"
        self.on_message = on_message

        self._serial = None
        self._reader_thread: Optional[threading.Thread] = None
        self._running = threading.Event()

    @property
    def ready(self) -> bool:
        return self._serial is not None and self._serial.is_open

    @property
    def serial_available(self) -> bool:
        return self.SERIAL_AVAILABLE

    def start(self) -> bool:
        """Open the serial device and launch the reader thread."""
        if serial is None:
            logger.warning("pyserial is not installed; joystick bridge disabled")
            return False

        if self.ready:
            return True

        try:
            self._serial = serial.Serial(self.port, self.baudrate, timeout=self.read_timeout)
            try:
                # Keep the USB serial bridge passive; some ESP8266 boards react
                # badly when desktop apps toggle modem-control lines.
                self._serial.setDTR(False)
                self._serial.setRTS(False)
            except Exception:
                pass
            try:
                self._serial.reset_input_buffer()
            except Exception:
                pass
            self._running.set()
            self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._reader_thread.start()
            logger.info("Joystick bridge connected on %s", self.port)
            return True
        except Exception as exc:  # pragma: no cover - hardware dependent
            logger.error("Failed to open joystick port %s: %s", self.port, exc)
            self._serial = None
            return False

    def stop(self) -> None:
        """Stop the reader thread and close the serial device."""
        self._running.clear()
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=2)
        self._reader_thread = None

        if self._serial:
            try:
                self._serial.close()
            except Exception as exc:  # pragma: no cover - hardware dependent
                logger.debug("Error closing joystick serial port: %s", exc)
        self._serial = None

    def _read_loop(self) -> None:  # pragma: no cover - hardware dependent
        transient_errors = 0
        try:
            while self._running.is_set() and self._serial:
                try:
                    raw = self._serial.readline()
                except Exception as exc:
                    if self._is_transient_read_error(exc):
                        transient_errors += 1
                        if transient_errors <= 3 or transient_errors % 10 == 0:
                            logger.warning("Transient joystick serial read glitch on %s (%d): %s",
                                           self.port, transient_errors, exc)
                        time.sleep(0.1)
                        continue
                    raise

                if not raw:
                    transient_errors = 0
                    continue

                transient_errors = 0

                text = raw.decode("utf-8", errors="ignore").strip()
                if not text:
                    continue

                message = None
                if text.startswith("{"):
                    try:
                        message = json.loads(text)
                    except json.JSONDecodeError:
                        message = None

                if message is None:
                    message = self._parse_legacy_frame(text)

                if message is None:
                    logger.debug("Discarding malformed joystick frame: %r", raw)
                    continue

                if "device_id" not in message:
                    message["device_id"] = self.device_id
                if "type" not in message:
                    message["type"] = "joystick"

                if self.on_message:
                    try:
                        self.on_message(message)
                    except Exception as exc:
                        logger.error("Joystick message handler error: %s", exc)
        except Exception as exc:
            logger.error("Joystick read loop error: %s", exc)
        finally:
            self._running.clear()
            if self._serial:
                try:
                    self._serial.close()
                except Exception:
                    pass
            self._serial = None
            logger.info("Joystick bridge reader stopped")

    @staticmethod
    def _is_transient_read_error(exc: Exception) -> bool:
        """Return True for CH340/USB serial glitches that are worth retrying."""
        text = str(exc).lower()
        return (
            'returned no data' in text
            or 'device disconnected or multiple access on port' in text
            or 'input/output error' in text
            or 'resource temporarily unavailable' in text
        )

    def _parse_legacy_frame(self, text: str) -> Optional[dict]:
        """Parse simple CSV joystick frames into a dict payload."""
        debug_match = THR_STR_RE.search(text)
        if debug_match:
            return {
                "type": "joystick",
                "y": float(debug_match.group(1)),
                "x": float(debug_match.group(2)),
            }

        tank_match = LEFT_RIGHT_RE.search(text)
        if tank_match:
            return {
                "type": "joystick",
                "left_speed": float(tank_match.group(1)),
                "right_speed": float(tank_match.group(2)),
            }

        parts = [segment.strip() for segment in text.split(",")]
        if not parts:
            return None

        keyword = parts[0].upper()
        try:
            if keyword == "J" and len(parts) >= 5:
                return {
                    "type": "joystick",
                    "y": float(parts[1]),
                    "x": float(parts[2]),
                    "enable": int(parts[3]),
                    "seq": int(parts[4]),
                }

            if keyword == "JOYSTICK" and len(parts) >= 2:
                payload: dict = {"type": "joystick"}
                direction = parts[1].lower()
                if direction:
                    payload["direction"] = direction
                if len(parts) > 2 and parts[2]:
                    payload["speed"] = float(parts[2])
                if len(parts) > 3 and parts[3]:
                    payload["enable"] = int(parts[3])
                if len(parts) > 4 and parts[4]:
                    payload["seq"] = int(parts[4])
                return payload
        except ValueError as exc:
            logger.debug("Failed to parse joystick frame '%s': %s", text, exc)
            return None

        return None

    def __del__(self) -> None:  # pragma: no cover - best effort cleanup
        try:
            self.stop()
        except Exception:
            pass
