"""
Serial Communication with Arduino Mega
"""

import logging
import serial
import json
import time
from typing import Dict, Any, List, Optional

logger = logging.getLogger('arduino_comm')


class ArduinoComm:
    """Serial communication with Arduino Mega 2560"""
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize Arduino communication
        
        Args:
            config: Arduino configuration dictionary
        """
        self.port = config['port']
        self.baudrate = config['baudrate']
        self.timeout = config['timeout']
        self.serial = None
        self.running = False
        
        self._connect()
    
    def _connect(self):
        """Establish serial connection"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            time.sleep(2)  # Wait for Arduino to reset
            logger.info(f"Connected to Arduino on {self.port}")
        except Exception as e:
            logger.error(f"Failed to connect to Arduino: {e}")
            self.serial = None
    
    def start(self):
        """Start communication"""
        if not self.serial or not self.serial.is_open:
            self._connect()
        self.running = True
        logger.info("Arduino communication started")
    
    def stop(self):
        """Stop communication and close port"""
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()
            logger.info("Arduino connection closed")
    
    def send_command(self, command: str, data: Any = None) -> bool:
        """
        Send command to Arduino
        
        Args:
            command: Command string
            data: Optional data payload
            
        Returns:
            bool: True if successful
        """
        if not self.serial or not self.serial.is_open:
            logger.error("Serial port not open")
            return False
        
        try:
            message = {
                'cmd': command,
                'data': data
            }
            json_str = json.dumps(message) + '\n'
            self.serial.write(json_str.encode())
            self.serial.flush()
            logger.debug(f"Sent: {json_str.strip()}")
            return True
        except Exception as e:
            logger.error(f"Error sending command: {e}")
            return False
    
    def receive_response(self, timeout: float = 2.0) -> Optional[Dict]:
        """
        Receive response from Arduino
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            Response dictionary or None
        """
        if not self.serial or not self.serial.is_open:
            return None
        
        try:
            self.serial.timeout = timeout
            line = self.serial.readline().decode().strip()
            
            if line:
                response = json.loads(line)
                logger.debug(f"Received: {line}")
                return response
        except json.JSONDecodeError as e:
            logger.warning(f"Invalid JSON received: {e}")
        except Exception as e:
            logger.error(f"Error receiving response: {e}")
        
        return None
    
    def request_gps_data(self) -> Optional[Dict]:
        """
        Request GPS data from Arduino
        
        Returns:
            GPS data dictionary or None
        """
        if self.send_command('GET_GPS'):
            response = self.receive_response()
            if response and response.get('type') == 'gps':
                return response.get('data')
        return None
    
    def send_waypoints(self, waypoints: List[Dict]) -> bool:
        """
        Send waypoints to Arduino
        
        Args:
            waypoints: List of waypoint dictionaries
            
        Returns:
            bool: True if successful
        """
        # Convert waypoints to format Arduino expects
        waypoint_data = [
            {
                'id': wp['id'],
                'lat': wp['latitude'],
                'lon': wp['longitude'],
                'seq': wp['sequence']
            }
            for wp in waypoints
        ]
        
        if self.send_command('SET_WAYPOINTS', waypoint_data):
            response = self.receive_response()
            return response and response.get('status') == 'ok'
        return False
    
    def send_navigation_command(self, command: str) -> bool:
        """
        Send navigation command to Arduino
        
        Args:
            command: 'START', 'STOP', 'PAUSE', 'RESUME'
            
        Returns:
            bool: True if successful
        """
        if self.send_command('NAV_CONTROL', {'action': command}):
            response = self.receive_response()
            return response and response.get('status') == 'ok'
        return False
