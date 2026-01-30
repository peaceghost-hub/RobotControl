"""
Compass Sensor (HMC5883L) for Raspberry Pi
Connected to I2C bus for heading data
"""

import smbus2
import math
import time
import logging

logger = logging.getLogger('compass')

class Compass:
    """HMC5883L magnetometer for heading"""
    
    def __init__(self, bus=1, address=0x0C):
        self.bus = smbus2.SMBus(bus)
        self.address = address
        self.declination = 0  # Magnetic declination in radians
        
        try:
            # Initialize HMC5883L
            self.bus.write_byte_data(self.address, 0x00, 0x70)  # 8-average, 15Hz, normal
            self.bus.write_byte_data(self.address, 0x01, 0xA0)  # Gain=5
            self.bus.write_byte_data(self.address, 0x02, 0x00)  # Continuous mode
            logger.info(f"Compass initialized at 0x{address:02X}")
        except Exception as e:
            logger.error(f"Failed to initialize compass: {e}")
            raise
    
    def read_raw(self):
        """Read raw magnetometer data"""
        try:
            data = self.bus.read_i2c_block_data(self.address, 0x03, 6)
            x = (data[0] << 8) | data[1]
            z = (data[2] << 8) | data[3]
            y = (data[4] << 8) | data[5]
            
            # Convert to signed
            if x > 32767: x -= 65536
            if y > 32767: y -= 65536
            if z > 32767: z -= 65536
            
            return x, y, z
        except Exception as e:
            logger.error(f"Error reading compass: {e}")
            return 0, 0, 0
    
    def read_heading(self):
        """Read compass heading in degrees"""
        x, y, z = self.read_raw()
        
        if x == 0 and y == 0:
            return 0.0
        
        # Calculate heading
        heading = math.atan2(y, x) * 180 / math.pi
        
        # Adjust for declination
        heading += self.declination * 180 / math.pi
        
        # Normalize to 0-360
        if heading < 0:
            heading += 360
        elif heading > 360:
            heading -= 360
        
        return heading
    
    def calibrate(self):
        """Simple calibration - rotate 360 degrees"""
        logger.info("Starting compass calibration - rotate sensor 360 degrees")
        
        min_x = min_y = 9999
        max_x = max_y = -9999
        
        start_time = time.time()
        while time.time() - start_time < 30:  # 30 seconds
            x, y, z = self.read_raw()
            min_x = min(min_x, x)
            max_x = max(max_x, x)
            min_y = min(min_y, y)
            max_y = max(max_y, y)
            time.sleep(0.1)
        
        # Calculate offset
        offset_x = (max_x + min_x) / 2
        offset_y = (max_y + min_y) / 2
        
        logger.info(f"Calibration complete. Offsets: X={offset_x}, Y={offset_y}")
        # Note: For full calibration, you'd store and apply these offsets
        return offset_x, offset_y