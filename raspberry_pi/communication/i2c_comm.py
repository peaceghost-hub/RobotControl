"""I2C communication helper between Raspberry Pi and Arduino Mega"""

from __future__ import annotations

import logging
import struct
import time
from typing import Dict, Any, List, Optional

logger = logging.getLogger('i2c_comm')

try:
    from smbus2 import SMBus, i2c_msg
    I2C_AVAILABLE = True
except ImportError:
    I2C_AVAILABLE = False
    SMBus = None
    i2c_msg = None
    logger.warning("smbus2 library not available; I2C communication disabled")


class I2CComm:
    """High level I2C protocol wrapper for Mega navigation board"""

    # Command opcodes sent to Mega
    CMD_PING = ord('P')
    CMD_NAV_START = ord('S')
    CMD_NAV_STOP = ord('T')
    CMD_NAV_PAUSE = ord('A')
    CMD_NAV_RESUME = ord('R')
    CMD_WAYPOINT_CLEAR = ord('C')
    CMD_WAYPOINT_PACKET = ord('W')
    CMD_WAYPOINT_COMMIT = ord('M')
    CMD_REQUEST_GPS = ord('G')
    CMD_REQUEST_STATUS = ord('U')
    CMD_REQUEST_OBSTACLE = ord('O')   # New: Request obstacle flag/distance
    CMD_HEARTBEAT = ord('H')
    CMD_SEND_GPS = ord('F')          # New: Send GPS from Pi to Mega (for fallback/broadcast)
    CMD_SEND_HEADING = ord('D')      # New: Send heading from Pi to Mega
    CMD_RETURN_TO_START = ord('B')   # New: Return to starting position
    CMD_MANUAL_OVERRIDE = ord('V')   # New: Manual control signal with joystick data
    CMD_EMERGENCY_STOP = ord('E')    # New: Emergency stop
    CMD_WIRELESS_BROADCAST = ord('Z') # New: Broadcast position via wireless
    CMD_FOLLOW_LINE = ord('L')        # New: Enable/disable line follower

    # Response opcodes returned by Mega
    RESP_ACK = 0x80
    RESP_GPS = 0x81
    RESP_STATUS = 0x82
    RESP_ERROR = 0xFF

    GPS_PAYLOAD_LEN = 1 + (4 * 4) + 1  # valid flag + floats + satellites
    STATUS_PAYLOAD_LEN = 6  # mode, navActive, manualOverride, waypointCount, battery, signal

    def __init__(self, config: Dict[str, Any]):
        self.address = config.get('mega_address', 0x08)
        self.bus_id = config.get('bus', 1)
        self.retry_attempts = config.get('retry_attempts', 3)
        self.request_timeout = config.get('request_timeout_ms', 200) / 1000.0

        if not I2C_AVAILABLE:
            logger.warning("I2C communication unavailable. Install smbus2 to enable Mega link.")
            self.bus = None
        else:
            try:
                self.bus = SMBus(self.bus_id)
                logger.info("I2C bus %s opened, Mega address 0x%02X", self.bus_id, self.address)
            except Exception as exc:
                logger.error("Failed to open I2C bus: %s", exc)
                self.bus = None

    # ------------------------------------------------------------------
    # Low-level helpers
    # ------------------------------------------------------------------
    def _write(self, payload: bytes) -> bool:
        if not self.bus:
            return False
        try:
            msg = i2c_msg.write(self.address, payload)
            self.bus.i2c_rdwr(msg)
            return True
        except Exception as exc:
            logger.error("I2C write failed: %s", exc)
            return False

    def _read(self, length: int) -> Optional[bytes]:
        if not self.bus:
            return None
        try:
            msg = i2c_msg.read(self.address, length)
            self.bus.i2c_rdwr(msg)
            return bytes(msg)
        except Exception as exc:
            logger.error("I2C read failed: %s", exc)
            return None

    def _exchange(self, command: int, data: bytes = b"", expect: int = 0) -> Optional[bytes]:
        payload = bytes([command]) + data
        attempts = max(1, int(self.retry_attempts))

        for attempt in range(1, attempts + 1):
            if not self._write(payload):
                if attempt < attempts:
                    time.sleep(min(0.05, self.request_timeout))
                    continue
                return None

            if expect == 0:
                return b""

            time.sleep(self.request_timeout)
            resp = self._read(expect)
            if resp is not None:
                return resp

            if attempt < attempts:
                time.sleep(min(0.05, self.request_timeout))

        return None

    # ------------------------------------------------------------------
    # Public operations
    # ------------------------------------------------------------------
    def ping(self) -> bool:
        resp = self._exchange(self.CMD_PING, expect=2)
        return self._is_ack(resp)

    def start_navigation(self) -> bool:
        return self._is_ack(self._exchange(self.CMD_NAV_START, expect=2))

    def stop_navigation(self) -> bool:
        return self._is_ack(self._exchange(self.CMD_NAV_STOP, expect=2))

    def pause_navigation(self) -> bool:
        return self._is_ack(self._exchange(self.CMD_NAV_PAUSE, expect=2))

    def resume_navigation(self) -> bool:
        return self._is_ack(self._exchange(self.CMD_NAV_RESUME, expect=2))

    def clear_waypoints(self) -> bool:
        return self._is_ack(self._exchange(self.CMD_WAYPOINT_CLEAR, expect=2))

    def send_waypoints(self, waypoints: List[Dict[str, Any]]) -> bool:
        if not waypoints:
            return True

        if not self.clear_waypoints():
            logger.error("Failed to clear existing waypoints")
            return False

        for wp in waypoints:
            payload = self._encode_waypoint_packet(wp)
            resp = self._exchange(self.CMD_WAYPOINT_PACKET, payload, expect=2)
            if not self._is_ack(resp):
                logger.error("Failed to send waypoint %s", wp.get('id'))
                return False

        return self._is_ack(self._exchange(self.CMD_WAYPOINT_COMMIT, expect=2))

    def request_gps_data(self) -> Optional[Dict[str, Any]]:
        resp = self._exchange(self.CMD_REQUEST_GPS, expect=1 + self.GPS_PAYLOAD_LEN)
        if not resp:
            return None
        if resp[0] == self.RESP_GPS and len(resp) >= 1 + self.GPS_PAYLOAD_LEN:
            return self._decode_gps(resp[1:])
        self._log_unexpected(resp, "GPS")
        return None

    def request_status(self) -> Optional[Dict[str, Any]]:
        resp = self._exchange(self.CMD_REQUEST_STATUS, expect=1 + self.STATUS_PAYLOAD_LEN)
        if not resp:
            return None
        if resp[0] == self.RESP_STATUS and len(resp) >= 1 + self.STATUS_PAYLOAD_LEN:
            return self._decode_status(resp[1:])
        self._log_unexpected(resp, "STATUS")
        return None

    def send_heartbeat(self) -> bool:
        resp = self._exchange(self.CMD_HEARTBEAT, expect=2)
        return self._is_ack(resp)
    
    def send_gps_data(self, gps_data: Dict[str, Any]) -> bool:
        """
        Send GPS data from Pi to Mega (as fallback or broadcast source)
        
        Args:
            gps_data: Dict with latitude, longitude, altitude, speed, heading, satellites
            
        Returns:
            bool: True if accepted
        """
        payload = self._encode_gps_payload(gps_data)
        resp = self._exchange(self.CMD_SEND_GPS, payload, expect=2)
        return self._is_ack(resp)
    
    def command_return_to_start(self) -> bool:
        """
        Instruct Mega to navigate back to starting position
        
        Returns:
            bool: True if command accepted
        """
        resp = self._exchange(self.CMD_RETURN_TO_START, expect=2)
        return self._is_ack(resp)
    
    def send_manual_control(self, left_motor: int, right_motor: int, joystick_active: bool = False) -> bool:
        """
        Send manual motor control with override flag
        
        Args:
            left_motor: -255 to 255
            right_motor: -255 to 255
            joystick_active: If True, immediately override autonomous nav
            
        Returns:
            bool: True if accepted
        """
        payload = struct.pack('<bbb', 
                            int(left_motor) & 0xFF,
                            int(right_motor) & 0xFF,
                            1 if joystick_active else 0)
        resp = self._exchange(self.CMD_MANUAL_OVERRIDE, payload, expect=2)
        return self._is_ack(resp)
    
    def emergency_stop(self) -> bool:
        """Send emergency stop command"""
        resp = self._exchange(self.CMD_EMERGENCY_STOP, expect=2)
        return self._is_ack(resp)
    
    def broadcast_position_wireless(self, latitude: float, longitude: float, 
                                   altitude: float = 0, speed: float = 0) -> bool:
        """
        Tell Mega to broadcast current position via wireless module
        
        Args:
            latitude, longitude: Position
            altitude, speed: Additional data
            
        Returns:
            bool: True if sent
        """
        payload = struct.pack('<ffff', latitude, longitude, altitude, speed)
        resp = self._exchange(self.CMD_WIRELESS_BROADCAST, payload, expect=2)
        return self._is_ack(resp)

    def set_line_follow(self, enabled: bool) -> bool:
        """Enable or disable line follower mode on Mega."""
        payload = bytes([1 if enabled else 0])
        resp = self._exchange(self.CMD_FOLLOW_LINE, payload, expect=2)
        return self._is_ack(resp)

    def request_obstacle_status(self) -> Optional[Dict[str, Any]]:
        """Request obstacle detection flag and distance from Mega."""
        resp = self._exchange(self.CMD_REQUEST_OBSTACLE, expect=4)
        if not resp:
            return None
        if resp[0] == 0x83 and len(resp) >= 4:  # RESP_OBSTACLE
            flag = bool(resp[1])
            distance = (resp[2] << 8) | resp[3]
            return {"obstacle": flag, "distance_cm": distance}
        self._log_unexpected(resp, "OBSTACLE")
        return None

    # ------------------------------------------------------------------
    # Encoding helpers
    # ------------------------------------------------------------------
    def _encode_gps_payload(self, gps_data: Dict[str, Any]) -> bytes:
        """Encode GPS data for transmission to Mega"""
        lat = float(gps_data.get('latitude', 0.0))
        lon = float(gps_data.get('longitude', 0.0))
        speed = float(gps_data.get('speed', 0.0))
        heading = float(gps_data.get('heading', 0.0))
        return struct.pack('<ffff', lat, lon, speed, heading)
    
    def _encode_waypoint_packet(self, waypoint: Dict[str, Any]) -> bytes:
        wp_id = int(waypoint.get('id', 0)) & 0xFFFF
        seq = int(waypoint.get('sequence', 0)) & 0xFF
        lat = float(waypoint.get('latitude', 0.0))
        lon = float(waypoint.get('longitude', 0.0))
        return struct.pack('<HBff', wp_id, seq, lat, lon)

    def _decode_gps(self, payload: bytes) -> Dict[str, Any]:
        valid = bool(payload[0])
        if not valid:
            return {"valid": False}
        lat, lon, speed, heading = struct.unpack('<ffff', payload[1:17])
        satellites = payload[17]
        return {
            "valid": True,
            "latitude": lat,
            "longitude": lon,
            "speed": speed,
            "heading": heading,
            "satellites": satellites
        }

    def _decode_status(self, payload: bytes) -> Dict[str, Any]:
        mode = payload[0]
        nav_active = bool(payload[1])
        manual_override = bool(payload[2])
        waypoint_count = payload[3]
        battery = payload[4]
        signal = payload[5]
        return {
            "mode": "AUTO" if mode == 0 else "MANUAL",
            "navigation_active": nav_active,
            "manual_override": manual_override,
            "waypoint_count": waypoint_count,
            "battery_percent": battery,
            "signal_quality": signal
        }

    def _is_ack(self, resp: Optional[bytes]) -> bool:
        return bool(resp) and resp[0] == self.RESP_ACK

    def _log_unexpected(self, resp: bytes, label: str) -> None:
        if not resp:
            logger.warning("No response when expecting %s", label)
            return
        if resp[0] == self.RESP_ERROR and len(resp) > 1:
            logger.error("Mega reported error for %s: code %s", label, resp[1])
        else:
            logger.warning("Unexpected %s response: %s", label, resp.hex())

    def close(self):
        if self.bus:
            try:
                self.bus.close()
            except Exception:
                pass
            self.bus = None

    # Context manager helpers -------------------------------------------------
    def __enter__(self) -> "I2CComm":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()