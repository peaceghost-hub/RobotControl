#!/usr/bin/env python3
"""
I2C Communication Test - Raspberry Pi to Arduino Mega
Tests basic I2C read/write at address 0x08
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
MEGA_ADDRESS = 0x08

def test_i2c_detection():
    """Check if device responds at 0x08"""
    print("=" * 60)
    print("I2C Detection Test")
    print("=" * 60)
    
    try:
        bus = smbus2.SMBus(I2C_BUS)
        print(f"✓ Opened I2C bus {I2C_BUS}")
    except Exception as e:
        print(f"✗ Failed to open I2C bus: {e}")
        print("\nRun diagnostic first: ./raspberry_pi/diagnose_i2c.sh")
        return False
    
    print(f"Testing address 0x{MEGA_ADDRESS:02X}...")
    
    try:
        # Try to read a byte (this will trigger Mega's requestEvent)
        data = bus.read_byte(MEGA_ADDRESS)
        print(f"✓ Device responded! Received: 0x{data:02X} ({data})")
        bus.close()
        return True
    except OSError as e:
        if e.errno == 121:  # Remote I/O error
            print(f"✗ No response from 0x{MEGA_ADDRESS:02X}")
            print("\nPossible causes:")
            print("1. Mega not running I2C sketch")
            print("2. Wrong I2C address in Mega code")
            print("3. SDA/SCL not connected")
            print("4. Missing pull-up resistors")
            print("5. No common ground")
        else:
            print(f"✗ I2C error: {e}")
        bus.close()
        return False

def test_i2c_communication():
    """Test multiple read/write operations"""
    print("\n" + "=" * 60)
    print("I2C Communication Test")
    print("=" * 60)
    
    try:
        bus = smbus2.SMBus(I2C_BUS)
    except Exception as e:
        print(f"✗ Failed to open I2C bus: {e}")
        return
    
    print("Performing 5 read operations...")
    print("(Check Mega Serial Monitor for received messages)")
    print()
    
    success_count = 0
    for i in range(5):
        try:
            data = bus.read_byte(MEGA_ADDRESS)
            print(f"  Read #{i+1}: 0x{data:02X} ({data})")
            success_count += 1
            time.sleep(0.5)
        except Exception as e:
            print(f"  Read #{i+1}: FAILED - {e}")
    
    print()
    print(f"Success rate: {success_count}/5")
    
    # Test write
    print("\nTesting write operation...")
    try:
        bus.write_byte(MEGA_ADDRESS, 0x42)
        print("✓ Write successful (sent 0x42)")
    except Exception as e:
        print(f"✗ Write failed: {e}")
    
    bus.close()

def test_advanced():
    """Test block read/write (like the actual robot code)"""
    print("\n" + "=" * 60)
    print("Advanced I2C Test (Block Operations)")
    print("=" * 60)
    
    try:
        bus = smbus2.SMBus(I2C_BUS)
    except Exception as e:
        print(f"✗ Failed to open I2C bus: {e}")
        return
    
    # Test writing a command byte
    print("Sending command 0x01 (STATUS request)...")
    try:
        bus.write_byte(MEGA_ADDRESS, 0x01)
        time.sleep(0.1)
        
        # Try to read response
        data = bus.read_i2c_block_data(MEGA_ADDRESS, 0x01, 16)
        print(f"✓ Received {len(data)} bytes: {[hex(b) for b in data[:8]]}...")
    except Exception as e:
        print(f"⚠ Command/response test: {e}")
        print("  (This is expected if Mega is running simple test sketch)")
    
    bus.close()

def main():
    print("Raspberry Pi → Arduino Mega I2C Test")
    print("Make sure:")
    print("1. I2C is enabled on Pi (sudo raspi-config)")
    print("2. Pull-up resistors installed (4.7kΩ on SDA and SCL)")
    print("3. Common ground connected")
    print("4. Mega is running i2c_slave_test.ino sketch")
    print()
    
    # Run detection test
    if not test_i2c_detection():
        print("\n" + "!" * 60)
        print("DETECTION FAILED - Fix hardware before proceeding")
        print("!" * 60)
        print("\nRun diagnostic: ./raspberry_pi/diagnose_i2c.sh")
        sys.exit(1)
    
    # Run communication test
    test_i2c_communication()
    
    # Advanced test
    test_advanced()
    
    print("\n" + "=" * 60)
    print("Test Complete!")
    print("=" * 60)
    print("\nIf all tests passed:")
    print("- Hardware is working correctly")
    print("- Upload robot_navigation.ino to Mega")
    print("- Run: python3 raspberry_pi/main.py")

if __name__ == '__main__':
    main()
