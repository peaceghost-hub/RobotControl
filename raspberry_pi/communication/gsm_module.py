"""
GSM Module Communication
Supports SIM800L, SIM900, and similar modules
"""

import logging
import serial
import time
from typing import Optional

logger = logging.getLogger('gsm_module')


class GSMModule:
    """GSM/GPRS Module for internet connectivity"""
    
    def __init__(self, config):
        """
        Initialize GSM module
        
        Args:
            config: GSM configuration dictionary
        """
        # Validate required configuration keys early to avoid opaque KeyErrors.
        self.port = config.get('port')
        self.baudrate = config.get('baudrate')
        self.apn = config.get('apn')

        missing = [k for k, v in (('port', self.port), ('baudrate', self.baudrate), ('apn', self.apn)) if not v]
        if missing:
            raise ValueError(f"Missing GSM config keys: {', '.join(missing)}")
        self.apn_user = config.get('apn_user', '')
        self.apn_password = config.get('apn_password', '')
        self.module_type = config.get('module_type', 'SIM800L')
        
        self.serial = None
        self.connected = False
        
        logger.info(f"GSM Module initialized: {self.module_type}")
    
    def connect(self) -> bool:
        """
        Connect to GSM module
        
        Returns:
            bool: True if successful
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            time.sleep(2)
            
            # Test AT command
            if not self._send_at_command('AT'):
                logger.error("GSM module not responding")
                return False
            
            # Setup GPRS
            if not self._setup_gprs():
                logger.error("Failed to setup GPRS")
                return False
            
            self.connected = True
            logger.info("GSM module connected")
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect GSM module: {e}")
            return False
    
    def disconnect(self):
        """Disconnect GSM module"""
        if self.serial and self.serial.is_open:
            self._send_at_command('AT+CIPSHUT')
            self.serial.close()
            self.connected = False
            logger.info("GSM module disconnected")
    
    def _send_at_command(self, command: str, wait_response: str = 'OK', timeout: float = 5.0) -> bool:
        """
        Send AT command and wait for response
        
        Args:
            command: AT command string
            wait_response: Expected response
            timeout: Timeout in seconds
            
        Returns:
            bool: True if expected response received
        """
        if not self.serial or not self.serial.is_open:
            return False
        
        try:
            self.serial.write((command + '\r\n').encode())
            self.serial.flush()
            
            start_time = time.time()
            response = ''
            
            while (time.time() - start_time) < timeout:
                if self.serial.in_waiting:
                    response += self.serial.read(self.serial.in_waiting).decode()
                    if wait_response in response:
                        logger.debug(f"AT: {command} -> OK")
                        return True
                time.sleep(0.1)
            
            logger.warning(f"AT command timeout: {command}")
            return False
            
        except Exception as e:
            logger.error(f"Error sending AT command: {e}")
            return False
    
    def _setup_gprs(self) -> bool:
        """
        Setup GPRS connection
        
        Returns:
            bool: True if successful
        """
        logger.info("Setting up GPRS...")
        
        # Set APN
        if not self._send_at_command(f'AT+CSTT="{self.apn}","{self.apn_user}","{self.apn_password}"'):
            return False
        
        # Bring up wireless connection
        if not self._send_at_command('AT+CIICR', timeout=10):
            return False
        
        # Get local IP address
        if not self._send_at_command('AT+CIFSR'):
            return False
        
        logger.info("GPRS setup complete")
        return True
    
    def check_internet(self) -> bool:
        """
        Check if internet connection is available
        
        Returns:
            bool: True if connected
        """
        if not self.connected:
            return False
        
        # Check GPRS status
        return self._send_at_command('AT+CIPSTATUS', 'IP', timeout=3)
    
    def get_signal_strength(self) -> int:
        """
        Get signal strength
        
        Returns:
            int: Signal strength in dBm (negative value)
        """
        if not self.serial or not self.serial.is_open:
            return -999
        
        try:
            self.serial.write(b'AT+CSQ\r\n')
            self.serial.flush()
            time.sleep(0.5)
            
            response = self.serial.read(self.serial.in_waiting).decode()
            
            # Parse response: +CSQ: <rssi>,<ber>
            if '+CSQ:' in response:
                rssi = int(response.split(':')[1].split(',')[0].strip())
                # Convert to dBm (approximate)
                if rssi == 99:
                    return -999
                dbm = -113 + (rssi * 2)
                return dbm
                
        except Exception as e:
            logger.error(f"Error getting signal strength: {e}")
        
        return -999
