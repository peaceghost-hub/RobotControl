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
    """Magnetometer for heading.

    Supports:
    - HMC5883L (classic register map, often at 0x1E)
    - QMC5883L (common replacement, often at 0x0D, sometimes at 0x2C/0x0C)
    """
    
    def __init__(self, bus=1):
        self.bus = smbus2.SMBus(bus)
        self.address = None
        self.declination = 0  # Magnetic declination in radians
        self.chip = None  # 'HMC5883L' | 'QMC5883L'
        
        # Try likely addresses (some modules vary / use different breakouts)
        possible_addresses = [0x2C, 0x0D, 0x0C, 0x1E]
        
        for addr in possible_addresses:
            try:
                # Test if device responds at this address
                self.bus.read_byte_data(addr, 0x00)
                self.address = addr
                logger.info(f"Compass found at address 0x{addr:02X}")
                break
            except Exception:
                continue
        
        if self.address is None:
            raise Exception("Compass not found on I2C (tried 0x0C, 0x2C, 0x1E, 0x0D)")

        # Identify chip type (best-effort)
        self.chip = self._detect_chip_type(self.address)

        try:
            if self.chip == 'QMC5883L':
                # QMC5883L basic setup (continuous mode)
                # 0x0B: Set/Reset period
                self.bus.write_byte_data(self.address, 0x0B, 0x01)
                # 0x09: Control (OSR=512, RNG=2G, ODR=100Hz, MODE=Continuous)
                self.bus.write_byte_data(self.address, 0x09, 0x1D)
            else:
                # HMC5883L setup
                self.bus.write_byte_data(self.address, 0x00, 0x70)  # 8-average, 15Hz, normal
                self.bus.write_byte_data(self.address, 0x01, 0x20)  # Gain 1.3 (matches Arduino handler)
                self.bus.write_byte_data(self.address, 0x02, 0x00)  # Continuous mode

            logger.info(f"Compass initialized at 0x{self.address:02X} as {self.chip}")
        except Exception as e:
            logger.error(f"Failed to initialize compass at 0x{self.address:02X}: {e}")
            raise

    def _detect_chip_type(self, addr: int) -> str:
        """Attempt to detect HMC5883L vs QMC5883L.

        HMC5883L has ID registers 0x0A..0x0C that typically read 'H','4','3'.
        Many QMC5883L breakouts do not provide those IDs and use a different register map.
        """
        try:
            ida = self.bus.read_byte_data(addr, 0x0A)
            idb = self.bus.read_byte_data(addr, 0x0B)
            idc = self.bus.read_byte_data(addr, 0x0C)
            if bytes([ida, idb, idc]) == b'H43':
                return 'HMC5883L'
        except Exception:
            pass

        # Heuristic: your address 0x2C is treated as QMC in the Arduino firmware.
        if addr in (0x2C, 0x0D, 0x0C):
            return 'QMC5883L'
        return 'HMC5883L'
    
    def read_raw(self):
        """Read raw magnetometer data"""
        try:
            if self.chip == 'QMC5883L':
                # QMC5883L data starts at 0x00: X_L, X_H, Y_L, Y_H, Z_L, Z_H
                data = self.bus.read_i2c_block_data(self.address, 0x00, 6)
            else:
                # HMC5883L data starts at 0x03: X_MSB, X_LSB, Z_MSB, Z_LSB, Y_MSB, Y_LSB
                data = self.bus.read_i2c_block_data(self.address, 0x03, 6)

            # If the bus returned all 0x00 or all 0xFF, treat as invalid read.
            if data == [0, 0, 0, 0, 0, 0] or data == [255, 255, 255, 255, 255, 255]:
                logger.warning(
                    "Compass returned invalid raw bytes at 0x%02X: %s",
                    self.address,
                    ' '.join(f"{b:02X}" for b in data),
                )
                return 0, 0, 0

            if self.chip == 'QMC5883L':
                x = (data[1] << 8) | data[0]
                y = (data[3] << 8) | data[2]
                z = (data[5] << 8) | data[4]
            else:
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