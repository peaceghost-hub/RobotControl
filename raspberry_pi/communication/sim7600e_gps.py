"""
SIM7600E GSM Module with Integrated GPS
Supports LTE + GPS antenna
"""

import logging
import serial
import time
import threading
import glob
import os
from typing import Optional, Dict, Tuple
from datetime import datetime

logger = logging.getLogger('sim7600e_gps')


class SIM7600EGPS:
    """SIM7600E module with built-in GPS support"""
    
    def __init__(self, config: Dict):
        """
        Initialize SIM7600E module
        
        Args:
            config: Configuration dict with keys:
                - port: Serial port (e.g., '/dev/ttyUSB0')
                - baudrate: Usually 115200
                - apn, apn_user, apn_password: APN settings
                - gps_enabled: Whether to enable GPS (default True)
        """
        # Allow "auto" port selection (USB or UART) by probing common device paths.
        self.port = config.get('port', 'auto')
        self.baudrate = config.get('baudrate', 115200)
        self.apn = config.get('apn', 'internet')
        self.apn_user = config.get('apn_user', '')
        self.apn_password = config.get('apn_password', '')
        self.gps_enabled = config.get('gps_enabled', True)
        self.data_enabled = config.get('data_enabled', True)
        
        self.serial = None
        self.connected = False
        self.gps_lock = False
        self.last_gps_data = None
        self._stop_event = threading.Event()
        self._gps_thread = None
        
        logger.info(f"SIM7600E module initialized (port={self.port})")

    @staticmethod
    def _candidate_ports() -> list[str]:
        # Prefer stable symlinks if present
        candidates: list[str] = []
        for fixed in ('/dev/serial0', '/dev/ttyAMA0'):
            candidates.append(fixed)
        # USB serial adapters / CDC ACM
        candidates.extend(sorted(glob.glob('/dev/ttyUSB*')))
        candidates.extend(sorted(glob.glob('/dev/ttyACM*')))
        # De-duplicate while preserving order
        seen = set()
        out: list[str] = []
        for p in candidates:
            if p not in seen:
                seen.add(p)
                out.append(p)
        return out

    def _resolve_port(self) -> Optional[str]:
        """Return a working serial port path by probing for AT response."""
        preferred = str(self.port).strip() if self.port is not None else 'auto'

        def can_try(path: str) -> bool:
            return bool(path) and os.path.exists(path)

        ports: list[str] = []
        if preferred and preferred.lower() != 'auto':
            ports.append(preferred)
        ports.extend(self._candidate_ports())

        saw_permission_error = False
        for path in ports:
            if not can_try(path):
                continue
            try:
                test_ser = serial.Serial(
                    port=path,
                    baudrate=self.baudrate,
                    timeout=1,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                )
                time.sleep(0.3)
                test_ser.reset_input_buffer()
                test_ser.write(b'AT\r\n')
                test_ser.flush()
                start = time.time()
                buf = ''
                while (time.time() - start) < 1.5:
                    if test_ser.in_waiting:
                        buf += test_ser.read(test_ser.in_waiting).decode(errors='ignore')
                        if 'OK' in buf:
                            test_ser.close()
                            return path
                    time.sleep(0.05)
                test_ser.close()
            except Exception as exc:
                # If device exists but isn't accessible, hint about permissions.
                try:
                    import errno
                    if getattr(exc, 'errno', None) == errno.EACCES:
                        saw_permission_error = True
                except Exception:
                    pass
                continue

        if saw_permission_error:
            logger.error("SIM7600E serial device found but permission denied (add user to 'dialout' or run as root)")
        return None
    
    def connect(self) -> bool:
        """Connect and initialize module"""
        try:
            resolved = self._resolve_port()
            if not resolved:
                logger.error("SIM7600E serial port not found. Set config port or connect via USB/UART.")
                return False

            if resolved != self.port:
                logger.info("SIM7600E auto-selected port %s", resolved)
            self.port = resolved

            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=2,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            time.sleep(2)
            
            # Test AT command
            if not self._send_at_command('AT', 'OK'):
                logger.error("SIM7600E not responding to AT")
                return False
            
            # Power on module
            self._send_at_command('AT+CFUN=1', 'OK')
            time.sleep(1)

            # Basic SIM readiness
            if not self._send_at_command('AT+CPIN?', 'READY'):
                logger.warning("SIM card not ready or requires PIN")
            else:
                logger.info("SIM card ready")

            # Bring up data session if enabled
            if self.data_enabled:
                try:
                    if self._setup_data_connection():
                        logger.info("SIM7600E data session ready")
                    else:
                        logger.warning("SIM7600E data session not ready")
                except Exception as exc:
                    logger.warning("SIM7600E data setup failed: %s", exc)
            
            # Enable GPS if configured
            if self.gps_enabled:
                if self._enable_gps():
                    logger.info("GPS enabled on SIM7600E")
                    # Start GPS polling thread
                    self._stop_event.clear()
                    self._gps_thread = threading.Thread(target=self._gps_poll_loop, daemon=True)
                    self._gps_thread.start()
            
            self.connected = True
            logger.info("SIM7600E connected successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect SIM7600E: {e}")
            return False

    def _setup_data_connection(self) -> bool:
        """Attempt to bring up a PDP context / data session."""
        if not self.serial or not self.serial.is_open:
            return False

        # Prefer LTE registration query when available
        self._send_at_command('AT+CEREG?', 'OK')
        self._send_at_command('AT+CREG?', 'OK')
        self._send_at_command('AT+CGREG?', 'OK')

        # Set APN
        apn = (self.apn or '').strip()
        if not apn:
            return False
        if not self._send_at_command(f'AT+CGDCONT=1,"IP","{apn}"', 'OK'):
            return False

        # Attach to packet service
        self._send_at_command('AT+CGATT=1', 'OK', timeout=10)

        # Activate PDP context
        if not self._send_at_command('AT+CGACT=1,1', 'OK', timeout=10):
            return False

        # Open network (SIMCom stack)
        # NETOPEN returns OK and then +NETOPEN: 0 on success.
        if not self._send_at_command('AT+NETOPEN', 'OK', timeout=10):
            # Some firmwares require closing first
            self._send_at_command('AT+NETCLOSE', 'OK', timeout=5)
            if not self._send_at_command('AT+NETOPEN', 'OK', timeout=10):
                return False
        
        time.sleep(2)  # Allow network to stabilize

        # Query IP address
        self._send_at_command('AT+IPADDR', '+IPADDR:', timeout=5)

        # Final sanity: registration check used as "internet ready" proxy
        return self.check_internet()
    
    def disconnect(self):
        """Disconnect module"""
        self._stop_event.set()
        if self._gps_thread:
            self._gps_thread.join(timeout=5)
        
        if self.serial and self.serial.is_open:
            self._send_at_command('AT+CGPS=0', 'OK')
            self.serial.close()
            self.connected = False
            logger.info("SIM7600E disconnected")
    
    def _send_at_command(self, cmd: str, wait_response: str = 'OK', timeout: float = 5.0) -> bool:
        """Send AT command and check response"""
        if not self.serial or not self.serial.is_open:
            return False
        
        try:
            self.serial.write((cmd + '\r\n').encode())
            self.serial.flush()
            
            start = time.time()
            response = ''
            
            while (time.time() - start) < timeout:
                if self.serial.in_waiting:
                    response += self.serial.read(self.serial.in_waiting).decode(errors='ignore')
                    if wait_response in response:
                        logger.debug(f"AT: {cmd} → OK")
                        return True
                time.sleep(0.05)
            
            logger.warning(f"AT timeout: {cmd} (expected '{wait_response}')")
            return False
            
        except Exception as e:
            logger.error(f"AT error: {e}")
            return False
    
    def _enable_gps(self) -> bool:
        """Enable GPS on SIM7600E"""
        # Power on GPS GNSS
        if not self._send_at_command('AT+CGNSPWR=1', 'OK'):
            return False
        
        time.sleep(3)  # Allow GPS to initialize
        
        # Enable GNSS positioning
        if not self._send_at_command('AT+CGPS=1', 'OK'):
            return False
        
        return True
    
    def get_gps_data(self) -> Optional[Dict]:
        """
        Get current GPS data
        
        Returns:
            Dict with keys: latitude, longitude, altitude, speed, heading, 
                           satellites, timestamp, valid, accuracy
            or None if no fix
        """
        return self.last_gps_data.copy() if self.last_gps_data else None
    
    def _gps_poll_loop(self):
        """Background thread to poll GPS data"""
        logger.info("GPS polling thread started")
        
        while not self._stop_event.is_set():
            try:
                self._update_gps_data()
                time.sleep(2)  # Poll every 2 seconds
            except Exception as e:
                logger.error(f"GPS poll error: {e}")
                time.sleep(5)
    
    def _update_gps_data(self):
        """Query GPS data from module"""
        if not self.connected or not self.gps_enabled:
            return
        
        try:
            # Query GNSS position: +CGNSINF: <gnss_run>,<fix_stat>,<utc_time>,<lat>,<lon>,<altitude>,
            #                                  <speed>,<course>,<fix_accur>,<hdop>,<pdop>,<vdop>
            cmd = 'AT+CGNSINF'
            if not self.serial or not self.serial.is_open:
                return
            
            self.serial.write((cmd + '\r\n').encode())
            self.serial.flush()
            
            response = ''
            start = time.time()
            
            while (time.time() - start) < 3.0:
                if self.serial.in_waiting:
                    response += self.serial.read(self.serial.in_waiting).decode(errors='ignore')
                    if 'OK' in response or '+CGNSINF:' in response:
                        break
                time.sleep(0.05)
            
            # Parse response
            if '+CGNSINF:' in response:
                # Extract the data line
                for line in response.split('\n'):
                    if '+CGNSINF:' in line:
                        self._parse_gps_response(line)
                        break
            
        except Exception as e:
            logger.debug(f"GPS update error: {e}")
    
    def _parse_gps_response(self, line: str):
        """Parse GPS data from AT+CGNSINF response"""
        try:
            # Format: +CGNSINF: <gnss_run>,<fix_stat>,<utc_time>,<lat>,<lon>,<altitude>,
            #                    <speed>,<course>,<fix_accur>,<hdop>,<pdop>,<vdop>
            
            # Remove command prefix
            if '+CGNSINF:' in line:
                data = line.split('+CGNSINF:')[1].strip()
            else:
                return
            
            parts = data.split(',')
            
            if len(parts) < 6:
                return
            
            try:
                gnss_run = int(parts[0])
                fix_stat = int(parts[1])
                utc_time = parts[2].strip()
                latitude = float(parts[3]) if parts[3].strip() else 0
                longitude = float(parts[4]) if parts[4].strip() else 0
                altitude = float(parts[5]) if parts[5].strip() else 0
                speed = float(parts[6]) if len(parts) > 6 and parts[6].strip() else 0
                course = float(parts[7]) if len(parts) > 7 and parts[7].strip() else 0
                fix_accuracy = float(parts[8]) if len(parts) > 8 and parts[8].strip() else 0
                satellites = int(parts[10]) if len(parts) > 10 and parts[10].strip() else 0
            except (ValueError, IndexError) as e:
                logger.debug(f"Parse error: {e}")
                return
            
            # Update only if we have a fix
            if fix_stat > 0 and latitude != 0 and longitude != 0:
                self.last_gps_data = {
                    'valid': True,
                    'latitude': latitude,
                    'longitude': longitude,
                    'altitude': altitude,
                    'speed': speed,
                    'heading': course,
                    'satellites': satellites,
                    'accuracy': fix_accuracy,
                    'timestamp': datetime.utcnow().isoformat(),
                    'source': 'SIM7600E'
                }
                self.gps_lock = True
                logger.debug(f"GPS fix: {latitude:.6f}, {longitude:.6f} ({satellites} sats)")
            else:
                self.gps_lock = False
        
        except Exception as e:
            logger.error(f"GPS parse error: {e}")
    
    def check_internet(self) -> bool:
        """Check if internet connection is available"""
        if not self.connected:
            return False
        
        # Check if network is open
        if not self._send_at_command('AT+NETOPEN?', '+NETOPEN: 0'):
            return False
        
        # Check network registration
        response = ''
        try:
            self.serial.write(b'AT+CREG?\r\n')
            self.serial.flush()
            
            start = time.time()
            while (time.time() - start) < 2.0:
                if self.serial.in_waiting:
                    response += self.serial.read(self.serial.in_waiting).decode(errors='ignore')
                    if 'OK' in response:
                        break
                time.sleep(0.05)
            
            # Parse response: look for +CREG: line
            for line in response.split('\n'):
                line = line.strip()
                if '+CREG:' in line:
                    # Format: +CREG: <n>,<stat>[,<lac>,<ci>,<AcT>]
                    parts = line.split(':')[1].split(',')
                    if len(parts) >= 2:
                        stat_str = parts[1].strip()
                        # Remove non-numeric characters
                        stat_str = ''.join(c for c in stat_str if c.isdigit() or c == '-')
                        try:
                            stat = int(stat_str)
                            return stat in [1, 5]  # 1=registered home, 5=registered roaming
                        except ValueError:
                            logger.error(f"Invalid stat value: '{stat_str}' from response: {response}")
                            return False
        except Exception as e:
            logger.error(f"Error checking internet: {e}")
        
        return False
                        except ValueError:
                            logger.warning(f"Failed to parse CREG stat: '{stat_str}'")
                            return False
            return False
        except Exception as e:
            logger.error(f"Internet check error: {e}")
            return False
    
    def get_signal_strength(self) -> int:
        """Get signal strength in dBm (-100 to -30 typical range)"""
        if not self.connected:
            return -110
        
        try:
            response = ''
            self.serial.write(b'AT+CSQ\r\n')
            self.serial.flush()
            
            start = time.time()
            while (time.time() - start) < 2.0:
                if self.serial.in_waiting:
                    response += self.serial.read(self.serial.in_waiting).decode(errors='ignore')
                    if 'OK' in response:
                        break
                time.sleep(0.05)
            
            # Parse: +CSQ: <rssi>,<ber>
            if '+CSQ:' in response:
                parts = response.split('+CSQ:')[1].split(',')
                rssi = int(parts[0].strip())
                # Convert RSSI to dBm: -120 + (rssi * 2)
                dbm = -120 + (rssi * 2) if rssi < 32 else -110
                return dbm
            
            return -110
        except Exception as e:
            logger.error(f"Signal check error: {e}")
            return -110
