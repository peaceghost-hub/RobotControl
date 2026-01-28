#!/usr/bin/env python3
"""SIM7600E UART Test Script.

Tests serial UART connection (Pi UART) and data setup.

Notes:
- Many SIM7600E/SIM7x00 *module* UART pins are 1.8V logic (per SIMCom docs).
    If your board exposes raw UART pins, do NOT connect Pi TX (3.3V) directly to
    module RX (1.8V). Use a level shifter.
"""

import argparse
import serial
import time
import sys
import os


DEFAULT_PORTS = ["/dev/serial0", "/dev/ttyAMA0", "/dev/ttyS0"]

def send_at(ser: serial.Serial, cmd: str, deadline_s: float = 1.0, tries: int = 2) -> str:
    """Send an AT command and return whatever response arrives before deadline.

    Uses short, non-blocking reads and never blocks indefinitely on write/flush.
    """
    payload = f"{cmd}\r\n".encode()
    response = ""

    for _ in range(max(1, tries)):
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass

        try:
            ser.write(payload)
        except serial.SerialTimeoutException:
            return "__WRITE_TIMEOUT__"
        except Exception as e:
            return f"__WRITE_ERROR__:{type(e).__name__}:{e}"

        start = time.time()
        buf = ""
        while (time.time() - start) < max(0.1, deadline_s):
            try:
                waiting = ser.in_waiting
                if waiting:
                    buf += ser.read(waiting).decode("utf-8", errors="ignore")
                    if "OK" in buf or "ERROR" in buf:
                        break
            except Exception:
                break
            time.sleep(0.05)

        response += buf
        if "OK" in response or "ERROR" in response:
            break

    return response


def _open_serial(port: str, baudrate: int) -> serial.Serial:
    return serial.Serial(
        port=port,
        baudrate=baudrate,
        timeout=0.2,
        write_timeout=0.5,
        rtscts=False,
        dsrdtr=False,
        exclusive=True,
    )

def test_basic_connection(ports: list[str], baudrate: int) -> serial.Serial | None:
    """Test basic AT communication on one or more ports."""
    print("\n=== Testing UART Connection ===")

    for port in ports:
        if not port:
            continue
        if not os.path.exists(port):
            continue

        print(f"\nOpening {port} @ {baudrate}...")
        try:
            ser = _open_serial(port, baudrate)
            time.sleep(0.2)
        except PermissionError:
            print(f"❌ Permission denied on {port}")
            continue
        except FileNotFoundError:
            continue
        except Exception as e:
            print(f"❌ Error opening {port}: {type(e).__name__}: {e}")
            continue

        print("Sending AT (autobaud-style: may need a few tries)...")
        response = send_at(ser, "AT", deadline_s=0.8, tries=4)

        if response == "__WRITE_TIMEOUT__":
            print("❌ Write timed out. Common causes:")
            print("- Wrong UART device (console/BT using it)")
            print("- Miswired TX/RX or missing GND")
            print("- Board requires RTS/CTS (rare)")
            ser.close()
            continue

        if "OK" in response:
            print("✓ SIM7600E responding to AT commands!")
            return ser

        print(f"✗ No OK. First bytes: {repr(response[:120])}")
        ser.close()

    print("\n❌ Could not get AT response on any port.")
    print("\nMost common causes on Raspberry Pi:")
    print("1) Wrong UART device: try /dev/serial0 (recommended), /dev/ttyS0, /dev/ttyAMA0")
    print("2) Serial console/Bluetooth is using the UART")
    print("   - In raspi-config: Interface Options → Serial Port")
    print("     Disable login shell over serial: YES")
    print("     Enable serial port hardware: YES")
    print("   - Then reboot")
    print("3) Wiring: TX/RX swapped and common GND required")
    print("4) Logic levels: many SIM7x00 UART pins are 1.8V; Pi TX is 3.3V")
    print("   - Use a level shifter if your board exposes raw module UART pins")
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

    parser = argparse.ArgumentParser(add_help=True)
    parser.add_argument(
        "--port",
        default="auto",
        help="Serial port to use (default: auto -> /dev/serial0,/dev/ttyAMA0,/dev/ttyS0)",
    )
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    args = parser.parse_args()

    ports = DEFAULT_PORTS if str(args.port).lower() == "auto" else [args.port]

    # Test basic connection
    ser = test_basic_connection(ports=ports, baudrate=args.baud)
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
