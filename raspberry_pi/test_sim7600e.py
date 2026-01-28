#!/usr/bin/env python3
"""
SIM7600E Connectivity Test Script
Tests USB/UART connection and internet setup
"""

import serial
import time
import glob
import os
import sys

def find_sim7600e():
    """Try to find SIM7600E on common ports"""
    candidates = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyUSB3',
                  '/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2',
                  '/dev/serial0', '/dev/ttyAMA0']
    
    print("Scanning for SIM7600E (this may take a minute)...\n")
    permission_error_ports = []
    found_ports = []
    
    for port in candidates:
        if not os.path.exists(port):
            continue
        print(f"  Trying {port}...", end=' ', flush=True)
        try:
            # Short timeout to fail fast on unresponsive ports
            ser = serial.Serial(port, 115200, timeout=0.5, write_timeout=0.5)
            time.sleep(0.1)
            
            # Clear buffers
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            
            # Send AT command with timeout
            try:
                ser.write(b'AT\r\n')
                ser.flush()
            except:
                ser.close()
                print("✗ Write timeout")
                continue
            
            # Wait for response with timeout
            start = time.time()
            response = ''
            while time.time() - start < 1.0:
                if ser.in_waiting:
                    response += ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                    if 'OK' in response:
                        break
                time.sleep(0.05)
            
            ser.close()
            
            if 'OK' in response:
                print("✓ FOUND!")
                found_ports.append(port)
            else:
                print("✗ No response")
        except PermissionError:
            print("✗ Permission denied")
            permission_error_ports.append(port)
        except Exception as e:
            print(f"✗ {type(e).__name__}")
    
    if found_ports:
        print(f"\n✓ Found {len(found_ports)} responsive port(s)")
        return found_ports[0]
    
    print("\n❌ SIM7600E not found on any port!")
    
    if permission_error_ports:
        print(f"\n⚠️  Permission denied on: {', '.join(permission_error_ports)}")
        print("\nFix permissions with:")
        print(f"  sudo usermod -aG dialout $USER")
        print(f"  sudo chmod 666 {' '.join(permission_error_ports)}")
        print("Then restart or run: newgrp dialout")
        return None
    
    print("\nTroubleshooting:")
    print("1. Check USB cable is connected to SIM7600E")
    print("2. Run: ls -l /dev/tty* | grep -E 'USB|ACM'")
    print("3. Check power LED on SIM7600E module (should be blinking)")
    print("4. Try: dmesg | tail -20 (look for new USB device)")
    print("5. Verify user is in dialout group: groups $USER")
    return None

def send_at(ser, cmd, timeout=2):
    """Send AT command and return response"""
    ser.reset_input_buffer()
    ser.write(f'{cmd}\r\n'.encode())
    time.sleep(timeout)
    response = ser.read_all().decode('utf-8', errors='ignore')
    return response

def test_module_info(ser):
    """Get module information"""
    print("\n=== Module Information ===")
    
    # Manufacturer
    resp = send_at(ser, 'AT+CGMI', 1)
    print(f"Manufacturer: {resp.strip()}")
    
    # Model
    resp = send_at(ser, 'AT+CGMM', 1)
    print(f"Model: {resp.strip()}")
    
    # Firmware
    resp = send_at(ser, 'AT+CGMR', 1)
    print(f"Firmware: {resp.strip()}")
    
    # SIM status
    resp = send_at(ser, 'AT+CPIN?', 1)
    if 'READY' in resp:
        print("SIM Card: ✓ Ready")
    else:
        print(f"SIM Card: ✗ {resp.strip()}")
    
    # Signal strength
    resp = send_at(ser, 'AT+CSQ', 1)
    print(f"Signal: {resp.strip()}")
    
    # Network registration
    resp = send_at(ser, 'AT+CREG?', 1)
    print(f"Network Registration: {resp.strip()}")
    
    resp = send_at(ser, 'AT+CGREG?', 1)
    print(f"GPRS Registration: {resp.strip()}")

def setup_data_connection(ser, apn):
    """Setup mobile data connection"""
    print(f"\n=== Setting up data connection (APN: {apn}) ===")
    
    # Configure PDP context
    print("1. Configuring PDP context...")
    resp = send_at(ser, f'AT+CGDCONT=1,"IP","{apn}"', 2)
    if 'OK' in resp:
        print("   ✓ PDP context set")
    else:
        print(f"   ✗ Failed: {resp}")
        return False
    
    # Attach to GPRS
    print("2. Attaching to GPRS network...")
    resp = send_at(ser, 'AT+CGATT=1', 5)
    if 'OK' in resp or 'ERROR' not in resp:
        print("   ✓ GPRS attached")
    else:
        print(f"   ✗ Failed: {resp}")
    
    # Check network open status
    resp = send_at(ser, 'AT+NETOPEN?', 1)
    if 'Network opened' in resp:
        print("3. Network already open")
        return True
    
    # Open network
    print("3. Opening network connection...")
    resp = send_at(ser, 'AT+NETOPEN', 15)
    if 'OK' in resp or '+NETOPEN: 0' in resp:
        print("   ✓ Network opened")
        return True
    else:
        print(f"   ⚠ Response: {resp}")
        # Check if already open
        resp = send_at(ser, 'AT+NETOPEN?', 1)
        if 'opened' in resp:
            print("   ✓ Network is open")
            return True
        return False

def test_internet(ser):
    """Test internet connectivity with HTTP request"""
    print("\n=== Testing Internet Connectivity ===")
    
    # Ping Google DNS
    print("Pinging 8.8.8.8...")
    resp = send_at(ser, 'AT+CPING="8.8.8.8",1,4,64,1000,10000', 15)
    print(resp)
    
    if '+CPING:' in resp and 'reachable' in resp.lower():
        print("✓ Internet connectivity confirmed!")
        return True
    else:
        print("✗ Ping failed - check APN settings")
        return False

def main():
    print("SIM7600E Connectivity Test")
    print("=" * 50)
    
    # Find module
    port = find_sim7600e()
    if not port:
        sys.exit(1)
    
    # Open serial connection
    print(f"\nOpening {port}...")
    ser = serial.Serial(port, 115200, timeout=2)
    time.sleep(1)
    
    try:
        # Test basic AT
        print("\nTesting AT commands...")
        resp = send_at(ser, 'AT', 1)
        if 'OK' not in resp:
            print("✗ Module not responding to AT commands")
            sys.exit(1)
        print("✓ Module responding")
        
        # Get info
        test_module_info(ser)
        
        # Get APN from user
        print("\n" + "=" * 50)
        apn = input("Enter your mobile carrier APN (or press Enter for 'internet'): ").strip()
        if not apn:
            apn = 'internet'
        
        # Setup data
        if setup_data_connection(ser, apn):
            time.sleep(2)
            test_internet(ser)
        
        print("\n" + "=" * 50)
        print("Test complete! Update config.json with correct APN:")
        print(f'  "apn": "{apn}"')
        
    finally:
        ser.close()

if __name__ == '__main__':
    main()
