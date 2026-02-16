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
        """Enable GPS on SIM7600E using AT+CGPS (not AT+CGNS which is SIM800 series)"""
        # AT+CGPS=1,1  →  1=enable, 1=standalone mode
        resp_ok = self._send_at_command('AT+CGPS=1,1', 'OK')
        if not resp_ok:
            # GPS may already be running — "GPS has started" is not an error
            logger.info("AT+CGPS=1,1 did not return OK — GPS may already be running")
        
        time.sleep(2)  # Allow GPS engine to initialise
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
        """Query GPS data from SIM7600E using AT+CGPSINFO (not AT+CGNSINF which is SIM800)"""
        if not self.connected or not self.gps_enabled:
            return
        
        try:
            if not self.serial or not self.serial.is_open:
                return
            
            self.serial.reset_input_buffer()
            self.serial.write(b'AT+CGPSINFO\r\n')
            self.serial.flush()
            
            response = ''
            start = time.time()
            
            while (time.time() - start) < 3.0:
                if self.serial.in_waiting:
                    response += self.serial.read(self.serial.in_waiting).decode(errors='ignore')
                    if 'OK' in response or 'ERROR' in response:
                        break
                time.sleep(0.05)
            
            # Parse +CGPSINFO: response
            if '+CGPSINFO:' in response:
                for line in response.split('\n'):
                    if '+CGPSINFO:' in line:
                        self._parse_cgpsinfo_response(line.strip())
                        break
            
        except Exception as e:
            logger.debug(f"GPS update error: {e}")
    
    @staticmethod
    def _ddmm_to_decimal(raw: str, hemisphere: str) -> float:
        """Convert DDMM.MMMM (or DDDMM.MMMM) to decimal degrees.

        SIM7600E AT+CGPSINFO returns lat as DDMM.MMMM and lon as DDDMM.MMMM.
        """
        try:
            val = float(raw)
        except (ValueError, TypeError):
            return 0.0
        degrees = int(val / 100)
        minutes = val - degrees * 100
        dec = degrees + minutes / 60.0
        if hemisphere in ('S', 'W'):
            dec = -dec
        return dec

    def _parse_cgpsinfo_response(self, line: str):
        """Parse GPS data from SIM7600E AT+CGPSINFO response.

        Format:  +CGPSINFO: lat,N/S,lon,E/W,date,UTC,alt,speed,course
        No fix:  +CGPSINFO: ,,,,,,,,
        """
        try:
            if '+CGPSINFO:' not in line:
                return
            data = line.split('+CGPSINFO:')[1].strip()

            # No fix — all fields empty
            if not data or data.replace(',', '').strip() == '':
                self.gps_lock = False
                return

            parts = data.split(',')
            if len(parts) < 9:
                return

            lat_raw   = parts[0].strip()
            lat_hemi  = parts[1].strip()
            lon_raw   = parts[2].strip()
            lon_hemi  = parts[3].strip()
            # parts[4] = date (DDMMYY), parts[5] = UTC time (HHMMSS.S)
            alt_str   = parts[6].strip()
            spd_str   = parts[7].strip()  # knots
            crs_str   = parts[8].strip()  # course/heading

            if not lat_raw or not lon_raw:
                self.gps_lock = False
                return

            latitude  = self._ddmm_to_decimal(lat_raw, lat_hemi)
            longitude = self._ddmm_to_decimal(lon_raw, lon_hemi)
            altitude  = float(alt_str) if alt_str else 0.0
            speed_kn  = float(spd_str) if spd_str else 0.0
            speed_mps = speed_kn * 0.514444   # knots → m/s
            course    = float(crs_str) if crs_str else 0.0

            if latitude == 0.0 and longitude == 0.0:
                self.gps_lock = False
                return

            self.last_gps_data = {
                'valid': True,
                'latitude': latitude,
                'longitude': longitude,
                'altitude': altitude,
                'speed': speed_mps,
                'heading': course,
                'satellites': 0,       # AT+CGPSINFO doesn't report sat count
                'accuracy': 0.0,
                'timestamp': datetime.utcnow().isoformat(),
                'source': 'SIM7600E'
            }
            self.gps_lock = True
            logger.debug(f"GPS fix: {latitude:.6f}, {longitude:.6f}")

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
