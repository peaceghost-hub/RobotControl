#!/usr/bin/env python3
"""
SIM7600E UART Test Script
Tests serial UART connection (/dev/ttyAMA0) and internet setup
"""

import serial
import time
import sys
import os

def send_at(ser, cmd, wait_time=1.0):
    """Send AT command and return response"""
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    try:
        ser.write(f'{cmd}\r\n'.encode())
        ser.flush()
    except Exception as e:
        print(f"Error writing to serial: {e}")
        return ""
    
    time.sleep(wait_time)
    response = ""
    
    try:
        while ser.in_waiting:
            chunk = ser.read(ser.in_waiting)
            response += chunk.decode('utf-8', errors='ignore')
            time.sleep(0.05)
    except Exception as e:
        print(f"Error reading from serial: {e}")
    
    return response

def test_basic_connection():
    """Test basic AT communication"""
    print("\n=== Testing UART Connection ===")
    
    try:
        ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=2)
        time.sleep(0.5)
    except PermissionError:
        print("❌ Permission denied on /dev/ttyAMA0")
        print("\nFix with:")
        print("  sudo usermod -aG dialout $USER")
        print("  newgrp dialout")
        print("Then restart the script")
        return None
    except FileNotFoundError:
        print("❌ /dev/ttyAMA0 not found!")
        print("\nMake sure:")
        print("1. UART is enabled: sudo raspi-config")
        print("2. Go to: Interface Options → Serial Port")
        print("3. Enable serial port hardware: YES")
        print("4. Disable login shell: NO")
        print("5. Reboot: sudo reboot")
        return None
    except Exception as e:
        print(f"❌ Error opening /dev/ttyAMA0: {e}")
        return None
    
    print("✓ /dev/ttyAMA0 opened successfully")
    
    # Test AT command
    print("Sending AT command...")
    response = send_at(ser, 'AT', 0.5)
    
    if 'OK' in response:
        print("✓ SIM7600E responding to AT commands!")
        return ser
    else:
        print(f"✗ No OK response. Got: {repr(response[:100])}")
        ser.close()
        return None

def get_module_info(ser):
    """Get module information"""
    print("\n=== Module Information ===")
    
    # Manufacturer
    resp = send_at(ser, 'AT+CGMI', 0.5)
    mfr = resp.split('\n')[0].strip() if resp else "Unknown"
    print(f"Manufacturer: {mfr}")
    
    # Model
    resp = send_at(ser, 'AT+CGMM', 0.5)
    model = resp.split('\n')[0].strip() if resp else "Unknown"
    print(f"Model: {model}")
    
    # Firmware
    resp = send_at(ser, 'AT+CGMR', 0.5)
    fw = resp.split('\n')[0].strip() if resp else "Unknown"
    print(f"Firmware: {fw}")
    
    # SIM status
    resp = send_at(ser, 'AT+CPIN?', 0.5)
    if 'READY' in resp:
        print("SIM Card: ✓ Ready")
    else:
        print(f"SIM Card Status: {resp.strip()[:60]}")
    
    # Signal strength
    resp = send_at(ser, 'AT+CSQ', 0.5)
    print(f"Signal Strength: {resp.strip()[:60]}")
    
    # Network registration
    resp = send_at(ser, 'AT+CREG?', 0.5)
    print(f"Network Reg: {resp.strip()[:80]}")
    
    resp = send_at(ser, 'AT+CGREG?', 0.5)
    print(f"GPRS Reg: {resp.strip()[:80]}")

def setup_data_connection(ser, apn):
    """Setup mobile data connection"""
    print(f"\n=== Setting up Data Connection (APN: {apn}) ===")
    
    # Configure PDP context
    print("1. Configuring PDP context...")
    cmd = f'AT+CGDCONT=1,"IP","{apn}"'
    resp = send_at(ser, cmd, 1.0)
    if 'OK' in resp:
        print("   ✓ PDP context set")
    else:
        print(f"   ⚠ Response: {resp.strip()[:60]}")
    
    # Attach to GPRS
    print("2. Attaching to GPRS network...")
    resp = send_at(ser, 'AT+CGATT=1', 3.0)
    if 'OK' in resp or 'ERROR' not in resp:
        print("   ✓ GPRS attached")
    else:
        print(f"   ⚠ Response: {resp.strip()[:60]}")
    
    # Check if network already open
    resp = send_at(ser, 'AT+NETOPEN?', 1.0)
    if 'opened' in resp.lower():
        print("3. Network already open")
        return True
    
    # Open network
    print("3. Opening network connection (this may take 10-20 seconds)...")
    resp = send_at(ser, 'AT+NETOPEN', 30.0)
    
    if 'OK' in resp or '+NETOPEN: 0' in resp:
        print("   ✓ Network opened successfully!")
        return True
    elif '+NETOPEN: 1' in resp:
        print("   ⚠ Already connected")
        return True
    else:
        print(f"   ⚠ Response: {resp.strip()[:60]}")
        # Verify anyway
        time.sleep(2)
        resp = send_at(ser, 'AT+NETOPEN?', 1.0)
        if 'opened' in resp.lower():
            print("   ✓ Network is open")
            return True
        return False

def test_internet(ser):
    """Test internet connectivity"""
    print("\n=== Testing Internet Connectivity ===")
    
    print("Checking PDP context...")
    resp = send_at(ser, 'AT+CGACT?', 1.0)
    print(f"PDP Status: {resp.strip()[:80]}")
    
    # Get IP address
    resp = send_at(ser, 'AT+CIFSR', 2.0)
    print(f"IP Address: {resp.strip()[:80]}")
    
    print("\nPinging 8.8.8.8...")
    resp = send_at(ser, 'AT+CPING="8.8.8.8",1,4,64,1000,10000', 20.0)
    
    if 'reachable' in resp.lower() or '+CPING' in resp:
        print("✓ Internet connectivity confirmed!")
        print(f"Response: {resp.strip()[:100]}")
        return True
    else:
        print(f"⚠ Ping response: {resp.strip()[:100]}")
        return False

def main():
    print("=" * 60)
    print("SIM7600E UART Connection Test")
    print("=" * 60)
    
    # Test basic connection
    ser = test_basic_connection()
    if not ser:
        sys.exit(1)
    
    try:
        # Get module info
        get_module_info(ser)
        
        # Get APN
        print("\n" + "=" * 60)
        apn = input("Enter your mobile carrier APN (or press Enter for 'internet'): ").strip()
        if not apn:
            apn = 'internet'
        
        # Setup data connection
        if setup_data_connection(ser, apn):
            time.sleep(3)
            test_internet(ser)
        else:
            print("\n⚠ Failed to open data connection")
            print("Troubleshooting:")
            print("1. Check signal strength (CSQ should be >5)")
            print("2. Verify APN is correct for your carrier")
            print("3. Check if roaming is needed")
            print("4. Try manual commands in minicom/screen")
        
        print("\n" + "=" * 60)
        print("Test complete!")
        print("\nIf successful, update config.json with:")
        print(f'  "port": "/dev/ttyAMA0",')
        print(f'  "apn": "{apn}"')
        print("\nThen start your robot with:")
        print("  python3 raspberry_pi/main.py")
        
    finally:
        ser.close()

if __name__ == '__main__':
    main()
