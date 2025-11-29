"""
DHT Sensor Module - Temperature and Humidity
Supports DHT11 and DHT22 sensors
"""

import logging
import time

try:
    import Adafruit_DHT
    DHT_AVAILABLE = True
except ImportError:
    DHT_AVAILABLE = False
    logger = logging.getLogger('dht_sensor')
    logger.warning("Adafruit_DHT library not available")

logger = logging.getLogger('dht_sensor')


class DHTSensor:
    """DHT11/DHT22 Temperature and Humidity Sensor"""
    
    if DHT_AVAILABLE:
        SENSOR_TYPES = {
            'DHT11': Adafruit_DHT.DHT11,
            'DHT22': Adafruit_DHT.DHT22,
            'AM2302': Adafruit_DHT.AM2302
        }
    else:
        SENSOR_TYPES = {}
    
    def __init__(self, pin, sensor_type='DHT22'):
        """
        Initialize DHT sensor
        
        Args:
            pin (int): GPIO pin number (BCM mode)
            sensor_type (str): 'DHT11', 'DHT22', or 'AM2302'
        """
        self.pin = pin
        
        if sensor_type not in self.SENSOR_TYPES:
            raise ValueError(f"Unsupported sensor type: {sensor_type}")
        
        self.sensor = self.SENSOR_TYPES[sensor_type]
        self.last_reading = None
        self.last_read_time = 0
        self.min_read_interval = 2  # seconds
        
        logger.info(f"DHT sensor initialized on pin {pin}, type: {sensor_type}")
    
    def read(self):
        """
        Read temperature and humidity
        
        Returns:
            dict: {'temperature': float, 'humidity': float}
        """
        # Rate limiting
        current_time = time.time()
        if current_time - self.last_read_time < self.min_read_interval:
            if self.last_reading:
                return self.last_reading
        
        if not DHT_AVAILABLE:
            logger.warning("Adafruit_DHT not available, returning dummy data")
            return {'temperature': 25.0, 'humidity': 60.0}
        
        try:
            humidity, temperature = Adafruit_DHT.read_retry(
                self.sensor,
                self.pin,
                retries=3,
                delay_seconds=2
            )
            
            if humidity is not None and temperature is not None:
                self.last_reading = {
                    'temperature': round(temperature, 2),
                    'humidity': round(humidity, 2)
                }
                self.last_read_time = current_time
                logger.debug(f"DHT reading: {self.last_reading}")
                return self.last_reading
            else:
                logger.warning("Failed to read DHT sensor")
                return {'temperature': None, 'humidity': None}
                
        except Exception as e:
            logger.error(f"Error reading DHT sensor: {e}")
            return {'temperature': None, 'humidity': None}
    
    def get_temperature(self):
        """Get temperature only"""
        data = self.read()
        return data.get('temperature')
    
    def get_humidity(self):
        """Get humidity only"""
        data = self.read()
        return data.get('humidity')
