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

# Robot/Mega protocol commands (ASCII)
CMD_PING = ord('P')
CMD_REQUEST_GPS = ord('G')
CMD_REQUEST_STATUS = ord('U')
CMD_REQUEST_OBSTACLE = ord('O')

RESP_ACK = 0x80
RESP_GPS = 0x81
RESP_STATUS = 0x82
RESP_OBSTACLE = 0x83

GPS_RESPONSE_LEN = 19      # 1 opcode + 1 valid + 16 floats + 1 satellites
STATUS_RESPONSE_LEN = 9    # 1 opcode + 8 payload
OBSTACLE_RESPONSE_LEN = 4  # 1 opcode + 1 flag + 2 distance


def _write_then_read(bus: "smbus2.SMBus", command: int, read_len: int) -> bytes:
    """Write a single-byte command, then read raw bytes back.

    Important: we use i2c_msg.read/write (not SMBus block ops) to avoid sending
    an extra 'register' byte (0x00) which confuses the Mega.
    """
    msg_w = smbus2.i2c_msg.write(MEGA_ADDRESS, bytes([command]))
    bus.i2c_rdwr(msg_w)
    time.sleep(0.1)
    msg_r = smbus2.i2c_msg.read(MEGA_ADDRESS, read_len)
    bus.i2c_rdwr(msg_r)
    return bytes(msg_r)

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
    
    print(f"Testing address 0x{MEGA_ADDRESS:02X} with protocol PING...")

    try:
        resp = _write_then_read(bus, CMD_PING, 2)
        ok = len(resp) == 2 and resp[0] == RESP_ACK
        if ok:
            print(f"✓ Device responded! ACK: {resp.hex()}")
            bus.close()
            return True
        print(f"✗ Unexpected ping response: {resp.hex()}")
        bus.close()
        return False
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
    
    print("Performing 5 PING operations...")
    print("(Check Mega Serial Monitor for received messages: should see 'P')")
    print()
    
    success_count = 0
    for i in range(5):
        try:
            resp = _write_then_read(bus, CMD_PING, 2)
            ok = len(resp) == 2 and resp[0] == RESP_ACK
            print(f"  Ping #{i+1}: {resp.hex()} {'OK' if ok else 'BAD'}")
            if ok:
                success_count += 1
            time.sleep(0.5)
        except Exception as e:
            print(f"  Ping #{i+1}: FAILED - {e}")
    
    print()
    print(f"Success rate: {success_count}/5")
    
    print("\nRequesting STATUS (U)...")
    try:
        resp = _write_then_read(bus, CMD_REQUEST_STATUS, STATUS_RESPONSE_LEN)
        print(f"  Status raw: {resp.hex()}")
        if len(resp) == STATUS_RESPONSE_LEN and resp[0] == RESP_STATUS:
            print("  ✓ STATUS opcode OK")
        else:
            print("  ⚠ Unexpected STATUS response")
    except Exception as e:
        print(f"  ⚠ STATUS request failed: {e}")

    print("\nRequesting OBSTACLE (O)...")
    try:
        resp = _write_then_read(bus, CMD_REQUEST_OBSTACLE, OBSTACLE_RESPONSE_LEN)
        print(f"  Obstacle raw: {resp.hex()}")
        if len(resp) == OBSTACLE_RESPONSE_LEN and resp[0] == RESP_OBSTACLE:
            detected = bool(resp[1])
            dist_cm = (resp[2] << 8) | resp[3]
            print(f"  ✓ OBSTACLE opcode OK (detected={detected}, dist_cm={dist_cm})")
        else:
            print("  ⚠ Unexpected OBSTACLE response")
    except Exception as e:
        print(f"  ⚠ OBSTACLE request failed: {e}")
    
    bus.close()

def test_advanced():
    """Test block read/write (like the actual robot code)"""
    print("\n" + "=" * 60)
    print("Advanced I2C Test (Protocol Requests)")
    print("=" * 60)
    
    try:
        bus = smbus2.SMBus(I2C_BUS)
    except Exception as e:
        print(f"✗ Failed to open I2C bus: {e}")
        return
    
    print("Requesting GPS (G)...")
    try:
        resp = _write_then_read(bus, CMD_REQUEST_GPS, GPS_RESPONSE_LEN)
        print(f"  GPS raw: {resp.hex()}")
        if len(resp) == GPS_RESPONSE_LEN and resp[0] == RESP_GPS:
            valid = bool(resp[1])
            if not valid:
                print("  ⚠ GPS valid flag=0 (Neo-6M has no fix yet)")
            else:
                print("  ✓ GPS valid flag=1")
        else:
            print("  ⚠ Unexpected GPS response")
    except Exception as e:
        print(f"⚠ GPS request failed: {e}")
    
    bus.close()

def main():
    print("Raspberry Pi → Arduino Mega I2C Test")
    print("Make sure:")
    print("1. I2C is enabled on Pi (sudo raspi-config)")
    print("2. Pull-up resistors installed (4.7kΩ on SDA and SCL)")
    print("3. Common ground connected")
    print("4. Mega is running robot_navigation.ino (or compatible protocol sketch)")
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
