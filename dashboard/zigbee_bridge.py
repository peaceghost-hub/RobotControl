"""ZigBee bridge utilities for the dashboard backup control path."""

from __future__ import annotations

import json
import logging
import threading
from typing import Callable, Optional

try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover - serial optional in dev envs
    serial = None

logger = logging.getLogger(__name__)


class ZigbeeBridge:
    """Simple serial bridge to communicate with a ZigBee coordinator."""

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        device_id: Optional[str] = None,
        on_message: Optional[Callable[[dict], None]] = None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.device_id = device_id or "robot_backup"
        self.on_message = on_message

        self._serial = None
        self._reader_thread: Optional[threading.Thread] = None
        self._running = threading.Event()
        self._write_lock = threading.Lock()

    @property
    def ready(self) -> bool:
        """Return True when the underlying serial interface is open."""
        return self._serial is not None and self._serial.is_open

    def start(self) -> bool:
        """Open the serial connection and launch the background reader."""
        if serial is None:
            logger.warning("pyserial is not installed; ZigBee bridge disabled")
            return False

        if self.ready:
            return True

        try:
            self._serial = serial.Serial(self.port, self.baudrate, timeout=1)
            self._running.set()
            self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._reader_thread.start()
            logger.info("ZigBee bridge connected on %s", self.port)
            return True
        except Exception as exc:  # pragma: no cover - hardware dependent
            logger.error("Failed to open ZigBee port %s: %s", self.port, exc)
            self._serial = None
            return False

    def stop(self) -> None:
        """Shut down the reader thread and close the serial connection."""
        self._running.clear()
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=2)
        self._reader_thread = None

        if self._serial:
            try:
                self._serial.close()
            except Exception as exc:  # pragma: no cover - hardware dependent
                logger.debug("Error closing ZigBee serial port: %s", exc)
        self._serial = None

    def send_control(self, command: str, payload: Optional[dict] = None) -> None:
        """Send a control command payload to the ZigBee coordinator."""
        packet = {
            "type": "control",
            "command": command,
            "payload": payload or {},
            "device_id": self.device_id,
        }
        self._write_packet(packet)

    def send_packet(self, packet: dict) -> None:
        """Send an arbitrary JSON packet via ZigBee."""
        if "device_id" not in packet:
            packet["device_id"] = self.device_id
        self._write_packet(packet)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _write_packet(self, packet: dict) -> None:
        if not self.ready:
            raise RuntimeError("ZigBee bridge is not connected")

        encoded = json.dumps(packet, separators=(",", ":")) + "\n"
        with self._write_lock:
            self._serial.write(encoded.encode("utf-8"))
            self._serial.flush()
        logger.debug("Sent ZigBee packet: %s", packet)

    def _read_loop(self) -> None:  # pragma: no cover - hardware dependent
        while self._running.is_set() and self._serial:
            try:
                raw = self._serial.readline()
                if not raw:
                    continue
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
                        logger.debug("Discarding malformed ZigBee frame: %r", raw)
                        continue

                logger.debug("Received ZigBee frame: %s", message)
                if self.on_message:
                    try:
                        self.on_message(message)
                    except Exception as exc:
                        logger.error("ZigBee message handler error: %s", exc)
            except Exception as exc:
                logger.error("ZigBee read loop error: %s", exc)
                break

        self._running.clear()
        logger.info("ZigBee bridge reader stopped")

    # ------------------------------------------------------------------
    # Legacy frame parsing helpers
    # ------------------------------------------------------------------

    def _parse_legacy_frame(self, text: str) -> Optional[dict]:
        """Convert CSV/keyword frames (Mega/UNO firmware) into dict payloads."""
        parts = [segment.strip() for segment in text.split(',')]
        if not parts:
            return None

        keyword = parts[0].upper()
        try:
            if keyword == 'GPS' and len(parts) >= 3:
                latitude = float(parts[1]) if parts[1] else 0.0
                longitude = float(parts[2]) if parts[2] else 0.0
                speed = float(parts[3]) if len(parts) > 3 and parts[3] else 0.0
                heading = float(parts[4]) if len(parts) > 4 and parts[4] else 0.0
                satellites = int(parts[5]) if len(parts) > 5 and parts[5] else 0
                timestamp = parts[6] if len(parts) > 6 and parts[6] else None
                return {
                    'type': 'gps',
                    'latitude': latitude,
                    'longitude': longitude,
                    'speed': speed,
                    'heading': heading,
                    'satellites': satellites,
                    'timestamp': timestamp,
                    'device_id': self.device_id,
                    'source': 'backup'
                }

            if keyword == 'MODE' and len(parts) >= 2:
                return {
                    'type': 'status',
                    'mode': parts[1],
                    'navigation': parts[2] if len(parts) > 2 else None,
                    'waypoints': int(parts[3]) if len(parts) > 3 and parts[3].isdigit() else None,
                    'device_id': self.device_id
                }

            if keyword == 'HELLO':
                return {
                    'type': 'status',
                    'state': 'hello',
                    'device_id': parts[1] if len(parts) > 1 else self.device_id
                }

            if keyword == 'JOYSTICK' and len(parts) >= 2:
                return {
                    'type': 'joystick',
                    'direction': parts[1].lower(),
                    'speed': int(parts[2]) if len(parts) > 2 and parts[2].isdigit() else None,
                    'device_id': self.device_id
                }

        except ValueError as exc:
            logger.debug("Failed to parse legacy frame '%s': %s", text, exc)
            return None

        return None

    def __del__(self) -> None:  # pragma: no cover - best effort cleanup
        try:
            self.stop()
        except Exception:
            pass