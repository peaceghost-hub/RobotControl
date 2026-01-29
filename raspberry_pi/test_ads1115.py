#!/usr/bin/env python3
"""
ADS1115 I2C Detection and Test Script
Tests the 16-bit ADC for analog sensor readings
"""

import sys
import time

try:
    import smbus2
except ImportError:
    print("ERROR: smbus2 not installed")
    print("Install: pip3 install smbus2")
    sys.exit(1)

I2C_BUS = 1

# ADS1115 possible addresses (based on ADDR pin connection)
ADS1115_ADDRESSES = {
    0x48: "ADDR to GND (default)",
    0x49: "ADDR to VDD",
    0x4A: "ADDR to SDA",
    0x4B: "ADDR to SCL"
}

# ADS1115 registers
ADS1115_REG_CONVERSION = 0x00
ADS1115_REG_CONFIG = 0x01

def scan_for_ads1115():
    """Scan I2C bus for ADS1115"""
    print("=" * 60)
    print("Scanning for ADS1115 ADC...")
    print("=" * 60)
    
    try:
        bus = smbus2.SMBus(I2C_BUS)
    except Exception as e:
        print(f"✗ Failed to open I2C bus: {e}")
        return None
    
    found_addresses = []
    
    for addr, desc in ADS1115_ADDRESSES.items():
        try:
            # Try to read config register
            data = bus.read_word_data(addr, ADS1115_REG_CONFIG)
            print(f"✓ Found ADS1115 at 0x{addr:02X} ({desc})")
            print(f"  Config register: 0x{data:04X}")
            found_addresses.append(addr)
        except OSError:
            print(f"✗ No device at 0x{addr:02X} ({desc})")
    
    bus.close()
    
    if not found_addresses:
        print("\n❌ No ADS1115 detected!")
        print("\nCheck:")
        print("1. ADS1115 VDD connected to 3.3V or 5V")
        print("2. ADS1115 GND connected to Pi GND")
        print("3. ADS1115 SDA connected to Pi Pin 3 (GPIO 2)")
        print("4. ADS1115 SCL connected to Pi Pin 5 (GPIO 3)")
        print("5. ADDR pin connection (default: GND for 0x48)")
        return None
    
    return found_addresses[0]

def test_ads1115_basic(address):
    """Test basic ADS1115 functionality"""
    print("\n" + "=" * 60)
    print(f"Testing ADS1115 at 0x{address:02X}")
    print("=" * 60)
    
    try:
        bus = smbus2.SMBus(I2C_BUS)
    except Exception as e:
        print(f"✗ Failed to open I2C bus: {e}")
        return False
    
    try:
        # Read current config
        config = bus.read_word_data(address, ADS1115_REG_CONFIG)
        # Swap bytes (ADS1115 is big-endian, but read_word_data is little-endian)
        config = ((config & 0xFF) << 8) | ((config >> 8) & 0xFF)
        print(f"✓ Current config: 0x{config:04X}")
        
        # Configure for single-ended reading on A0
        # Config: Start single conversion, AIN0, ±4.096V, single-shot, 128 SPS
        config_value = 0xC383  # Binary: 1100 0011 1000 0011
        
        # Write config (swap bytes back)
        config_swapped = ((config_value & 0xFF) << 8) | ((config_value >> 8) & 0xFF)
        bus.write_word_data(address, ADS1115_REG_CONFIG, config_swapped)
        print("✓ Wrote conversion config")
        
        # Wait for conversion (max 8ms at 128 SPS)
        time.sleep(0.01)
        
        # Read conversion result
        result = bus.read_word_data(address, ADS1115_REG_CONVERSION)
        # Swap bytes
        result = ((result & 0xFF) << 8) | ((result >> 8) & 0xFF)
        
        # Convert to signed 16-bit
        if result & 0x8000:
            result -= 65536
        
        # Convert to voltage (±4.096V range, 16-bit)
        voltage = result * 4.096 / 32768.0
        
        print(f"✓ Raw ADC value: {result}")
        print(f"✓ Voltage on A0: {voltage:.4f}V")
        
        bus.close()
        return True
        
    except Exception as e:
        print(f"✗ Error testing ADS1115: {e}")
        bus.close()
        return False

def test_all_channels(address):
    """Test all 4 analog input channels"""
    print("\n" + "=" * 60)
    print("Reading All ADS1115 Channels")
    print("=" * 60)
    
    try:
        bus = smbus2.SMBus(I2C_BUS)
    except Exception as e:
        print(f"✗ Failed to open I2C bus: {e}")
        return
    
    channel_configs = {
        0: 0xC383,  # A0 vs GND
        1: 0xD383,  # A1 vs GND
        2: 0xE383,  # A2 vs GND
        3: 0xF383,  # A3 vs GND
    }
    
    print("\nChannel | Raw Value | Voltage  | Expected Sensor")
    print("--------|-----------|----------|------------------")
    
    for channel, config in channel_configs.items():
        try:
            # Write config (swap bytes)
            config_swapped = ((config & 0xFF) << 8) | ((config >> 8) & 0xFF)
            bus.write_word_data(address, ADS1115_REG_CONFIG, config_swapped)
            time.sleep(0.01)
            
            # Read result
            result = bus.read_word_data(address, ADS1115_REG_CONVERSION)
            result = ((result & 0xFF) << 8) | ((result >> 8) & 0xFF)
            
            if result & 0x8000:
                result -= 65536
            
            voltage = result * 4.096 / 32768.0
            
            # Map to expected sensors from config.json
            sensor_map = {
                0: "MQ-2 (Smoke/LPG)",
                1: "MQ-135 (Air Quality)",
                2: "MQ-7 (CO)",
                3: "Reserved/Unused"
            }
            
            print(f"  A{channel}    | {result:9d} | {voltage:7.4f}V | {sensor_map[channel]}")
            
        except Exception as e:
            print(f"  A{channel}    | ERROR: {e}")
    
    bus.close()

def verify_wiring():
    """Display wiring information"""
    print("\n" + "=" * 60)
    print("ADS1115 Wiring Verification")
    print("=" * 60)
    print("""
ADS1115 Module          Raspberry Pi
──────────────          ────────────
VDD                 →   Pin 2 (5V) or Pin 1 (3.3V)
GND                 →   Pin 6 (GND)
SCL                 →   Pin 5 (GPIO 3, SCL)
SDA                 →   Pin 3 (GPIO 2, SDA)
ADDR                →   GND (for address 0x48)

Analog Inputs (from config.json):
A0                  ←   MQ-2 AOUT (Smoke/LPG)
A1                  ←   MQ-135 AOUT (Air Quality)
A2                  ←   MQ-7 AOUT (Carbon Monoxide)
A3                  ←   Reserved

Notes:
- ADS1115 works with both 3.3V and 5V supply
- Analog sensors (MQ-x) need 5V power
- ADDR pin determines I2C address:
    GND  → 0x48 (default)
    VDD  → 0x49
    SDA  → 0x4A
    SCL  → 0x4B
""")

def main():
    print("ADS1115 16-bit ADC Detection and Test")
    print()
    
    # Scan for ADS1115
    address = scan_for_ads1115()
    
    if not address:
        verify_wiring()
        print("\nRun I2C scan to see all devices:")
        print("  sudo i2cdetect -y 1")
        sys.exit(1)
    
    # Test basic functionality
    if not test_ads1115_basic(address):
        sys.exit(1)
    
    # Test all channels
    test_all_channels(address)
    
    # Show wiring info
    verify_wiring()
    
    print("\n" + "=" * 60)
    print("Test Complete!")
    print("=" * 60)
    print("\nIf ADS1115 is working:")
    print("- Analog sensors should show voltage readings")
    print("- MQ sensors may show ~1-4V depending on gas concentration")
    print("- Your Pi controller will use these readings for air quality monitoring")

if __name__ == '__main__':
    main()
