"""
SIM7600E GSM Module with Integrated GPS
Supports LTE + GPS antenna
"""

import logging
import serial
import time
import threading
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
        self.port = config.get('port', '/dev/ttyUSB0')
        self.baudrate = config.get('baudrate', 115200)
        self.apn = config.get('apn', 'internet')
        self.apn_user = config.get('apn_user', '')
        self.apn_password = config.get('apn_password', '')
        self.gps_enabled = config.get('gps_enabled', True)
        
        self.serial = None
        self.connected = False
        self.gps_lock = False
        self.last_gps_data = None
        self._stop_event = threading.Event()
        self._gps_thread = None
        
        logger.info(f"SIM7600E module initialized on {self.port}")
    
    def connect(self) -> bool:
        """Connect and initialize module"""
        try:
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
        
        time.sleep(2)  # Allow GPS to initialize
        
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
                        # Parse: +CREG: <n>,<stat> where stat=1 (registered) or 5 (roaming)
                        if '+CREG:' in response:
                            parts = response.split('+CREG:')[1].split(',')
                            stat = int(parts[1].strip())
                            return stat in [1, 5]
                time.sleep(0.05)
            
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
