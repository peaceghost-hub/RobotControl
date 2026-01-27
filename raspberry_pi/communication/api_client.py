"""
Dashboard API Client
Handles communication with the web dashboard server
"""

import logging
import requests
import json
from typing import Dict, Any, List, Optional

logger = logging.getLogger('api_client')


class DashboardAPI:
    """Client for communicating with dashboard API"""
    
    def __init__(self, config: Dict[str, Any], device_id: str):
        """
        Initialize API client
        
        Args:
            config: API configuration dictionary
            device_id: Unique device identifier
        """
        self.base_url = config['base_url']
        self.endpoints = config['endpoints']
        self.timeout = config.get('timeout', 10)
        self.retry_attempts = config.get('retry_attempts', 3)
        self.device_id = device_id
        
        self.session = requests.Session()
        self.session.headers.update({
            'Content-Type': 'application/json',
            'User-Agent': f'RobotClient/{device_id}'
        })
        
        logger.info(f"Dashboard API client initialized: {self.base_url}")
    
    def _make_request(self, method: str, endpoint: str, data: Optional[Dict] = None) -> Optional[Dict]:
        """
        Make HTTP request with retry logic
        
        Args:
            method: HTTP method ('GET' or 'POST')
            endpoint: API endpoint path
            data: Request data dictionary
            
        Returns:
            Response data or None on failure
        """
        url = f"{self.base_url}{endpoint}"
        
        for attempt in range(self.retry_attempts):
            try:
                if method == 'GET':
                    response = self.session.get(url, timeout=self.timeout)
                elif method == 'POST':
                    response = self.session.post(url, json=data, timeout=self.timeout)
                else:
                    logger.error(f"Unsupported HTTP method: {method}")
                    return None
                
                response.raise_for_status()
                return response.json()
                
            except requests.exceptions.Timeout:
                logger.warning(f"Request timeout (attempt {attempt + 1}/{self.retry_attempts})")
            except requests.exceptions.ConnectionError:
                logger.warning(f"Connection error (attempt {attempt + 1}/{self.retry_attempts})")
            except requests.exceptions.HTTPError as e:
                logger.error(f"HTTP error: {e}")
                return None
            except Exception as e:
                logger.error(f"Unexpected error: {e}")
                return None
        
        logger.error(f"Failed to {method} {endpoint} after {self.retry_attempts} attempts")
        return None
    
    def send_sensor_data(self, data: Dict[str, Any]) -> bool:
        """
        Send sensor data to dashboard
        
        Args:
            data: Sensor readings dictionary
            
        Returns:
            bool: True if successful
        """
        response = self._make_request('POST', self.endpoints['sensor_data'], data)
        return response is not None and response.get('status') == 'success'
    
    def send_gps_data(self, data: Dict[str, Any]) -> bool:
        """
        Send GPS data to dashboard
        
        Args:
            data: GPS data dictionary
            
        Returns:
            bool: True if successful
        """
        response = self._make_request('POST', self.endpoints['gps_data'], data)
        return response is not None and response.get('status') == 'success'
    
    def send_status(self, data: Dict[str, Any]) -> bool:
        """
        Send robot status to dashboard
        
        Args:
            data: Status data dictionary
            
        Returns:
            bool: True if successful
        """
        response = self._make_request('POST', self.endpoints['status'], data)
        return response is not None and response.get('status') == 'success'

    def send_event(self, data: Dict[str, Any]) -> bool:
        """Send robot event (e.g., obstacle detected) to dashboard."""
        endpoint = self.endpoints.get('event', '/api/event')
        response = self._make_request('POST', endpoint, data)
        return response is not None and response.get('status') == 'success'
    
    def send_camera_frame(self, data: Dict[str, Any]) -> bool:
        """
        Send camera frame to dashboard
        
        Args:
            data: Frame data with base64 encoded image
            
        Returns:
            bool: True if successful
        """
        response = self._make_request('POST', self.endpoints['camera_frame'], data)
        return response is not None and response.get('status') == 'success'
    
    def get_waypoints(self) -> Optional[List[Dict]]:
        """
        Get waypoints from dashboard
        
        Returns:
            List of waypoint dictionaries or None
        """
        params = f"?device_id={self.device_id}"
        response = self._make_request('GET', self.endpoints['waypoints'] + params)
        
        if response and response.get('status') == 'success':
            return response.get('data', [])
        return None
    
    def fetch_pending_commands(self) -> Optional[List[Dict]]:
        """
        Fetch pending commands from dashboard
        
        Returns:
            List of command dictionaries or None
        """
        params = f"?device_id={self.device_id}"
        # Use the commands/pending endpoint
        endpoint = self.endpoints.get('commands', '/api/commands/pending')
        response = self._make_request('GET', endpoint + params)
        
        if response and response.get('status') == 'success':
            return response.get('commands', [])
        return []
    
    def test_connection(self) -> bool:
        """
        Test connection to dashboard
        
        Returns:
            bool: True if connected
        """
        try:
            response = requests.get(f"{self.base_url}/api/health", timeout=5)
            return response.status_code == 200
        except:
            return False
