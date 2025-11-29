"""
Data Formatter Utility
"""

from datetime import datetime
from typing import Dict, Any


class DataFormatter:
    """Format data for transmission"""
    
    @staticmethod
    def format_sensor_data(data: Dict[str, Any], device_id: str) -> Dict[str, Any]:
        """Format sensor data for API"""
        return {
            'timestamp': datetime.now().isoformat(),
            'device_id': device_id,
            **data
        }
    
    @staticmethod
    def format_gps_data(data: Dict[str, Any], device_id: str) -> Dict[str, Any]:
        """Format GPS data for API"""
        return {
            'timestamp': datetime.now().isoformat(),
            'device_id': device_id,
            **data
        }
