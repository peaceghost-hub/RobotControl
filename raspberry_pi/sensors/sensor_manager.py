"""
Sensor Manager - Coordinates all environmental sensors
"""

import logging
from .dht_sensor import DHTSensor
from .mq_sensors import MQSensors

logger = logging.getLogger('sensor_manager')


class SensorManager:
    """Manages all environmental sensors"""
    
    def __init__(self, config):
        """
        Initialize sensor manager
        
        Args:
            config: Sensor configuration dictionary
        """
        logger.info("Initializing Sensor Manager...")
        
        self.config = config
        self.sensors = {}
        
        # Initialize DHT sensor (Temperature & Humidity)
        try:
            self.sensors['dht'] = DHTSensor(
                pin=config['dht']['pin'],
                sensor_type=config['dht']['type']
            )
            logger.info("DHT sensor initialized")
        except Exception as e:
            logger.error(f"Failed to initialize DHT sensor: {e}")
            self.sensors['dht'] = None
        
        # Initialize MQ gas sensors
        try:
            self.sensors['mq'] = MQSensors(config['mq_sensors'], config['adc'])
            logger.info("MQ sensors initialized")
        except Exception as e:
            logger.error(f"Failed to initialize MQ sensors: {e}")
            self.sensors['mq'] = None
    
    def read_all(self):
        """
        Read all sensors
        
        Returns:
            dict: Dictionary containing all sensor readings
        """
        data = {}
        
        # Read DHT sensor
        if self.sensors['dht']:
            try:
                dht_data = self.sensors['dht'].read()
                data['temperature'] = dht_data.get('temperature')
                data['humidity'] = dht_data.get('humidity')
            except Exception as e:
                logger.error(f"Error reading DHT sensor: {e}")
                data['temperature'] = None
                data['humidity'] = None
        
        # Read MQ sensors
        if self.sensors['mq']:
            try:
                mq_data = self.sensors['mq'].read_all()
                data.update(mq_data)
            except Exception as e:
                logger.error(f"Error reading MQ sensors: {e}")
        
        return data
    
    def calibrate_sensors(self):
        """Calibrate all sensors (if applicable)"""
        logger.info("Calibrating sensors...")
        
        if self.sensors['mq']:
            try:
                self.sensors['mq'].calibrate()
                logger.info("MQ sensors calibrated")
            except Exception as e:
                logger.error(f"Error calibrating MQ sensors: {e}")
    
    def get_sensor_status(self):
        """
        Get status of all sensors
        
        Returns:
            dict: Status of each sensor
        """
        status = {
            'dht': self.sensors['dht'] is not None,
            'mq': self.sensors['mq'] is not None
        }
        return status
