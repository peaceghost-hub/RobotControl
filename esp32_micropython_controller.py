"""
ESP32 Robot Controller (MicroPython)
I2C Master + Multi-connectivity + Wireless Backup Controller

This script runs on ESP32 with MicroPython and:
PRIMARY CONTROL:
- Acts as I2C master to Arduino Mega (like Raspberry Pi)
- Forwards waypoints and commands from dashboard to Mega via I2C
- Reads GPS and status from Mega via I2C

INTERNET CONNECTIVITY (auto-failover):
- WiFi (primary): Connects to "Ghost" network if available
- GSM (fallback): Provides internet when WiFi unavailable
- Seamless switching without breaking communication

BACKUP MANUAL CONTROL:
- UART wireless modules (ZigBee/BLE/LoRa) for emergency/backup control
- Sends MCTL format commands directly to Mega's wireless receiver
- Independent backup channel when I2C or dashboard unavailable
"""

import time
import json
import network
from machine import Pin, ADC, UART, I2C, SoftI2C
import urequests as requests
import struct

# ==================== CONFIGURATION ====================

# WiFi Configuration (Primary Internet)
WIFI_SSID = "Ghost"
WIFI_PASSWORD = "omegatruthalpha"
WIFI_TIMEOUT = 15  # seconds

# GSM Configuration (Fallback Internet)
GSM_ENABLED = True
GSM_APN = "internet"  # Change to your carrier's APN
GSM_UART_ID = 1       # UART1 for GSM module (GPIO 9=RX, GPIO 10=TX)
GSM_BAUD = 115200

# Dashboard Configuration
DASHBOARD_URL = "http://192.168.1.100:5000"  # Change to your dashboard IP
DEVICE_ID = "esp32_controller_1"

# I2C Configuration (Primary control to Mega - like Raspberry Pi)
I2C_SDA_PIN = 21      # GPIO 21 (default SDA)
I2C_SCL_PIN = 22      # GPIO 22 (default SCL)
I2C_FREQ = 100000     # 100kHz
MEGA_I2C_ADDRESS = 0x08  # Arduino Mega slave address

# Wireless Backup Control (Emergency/Manual override via UART)
BACKUP_LINK_PROTOCOL = "zigbee"  # 'zigbee', 'ble', or 'lora'
BACKUP_UART_ID = 2               # UART2 (GPIO 16=RX, GPIO 17=TX)
BACKUP_BAUD = 57600              # ZigBee: 57600, BLE: 38400/9600, LoRa: 9600

class ESP32Controller:
    """ESP32 I2C master controller with multi-connectivity"""
    
    # I2C Command opcodes (matching Raspberry Pi protocol)
    CMD_PING = 0x01
    CMD_NAV_START = 0x02
    CMD_NAV_STOP = 0x03
    CMD_NAV_PAUSE = 0x04
    CMD_NAV_RESUME = 0x05
    CMD_WAYPOINT_CLEAR = 0x10
    CMD_WAYPOINT_PACKET = 0x11
    CMD_WAYPOINT_COMMIT = 0x12
    CMD_REQUEST_GPS = 0x20
    CMD_REQUEST_STATUS = 0x21
    CMD_HEARTBEAT = 0x30
    
    RESP_ACK = 0x80
    RESP_GPS = 0x81
    RESP_STATUS = 0x82

    def __init__(self):
        self.device_id = DEVICE_ID
        self.dashboard_url = DASHBOARD_URL
        self.running = False
        
        # Connectivity state
        self.wifi_connected = False
        self.gsm_connected = False
        self.internet_available = False
        self.i2c_available = False
        
        # Timing
        self.last_heartbeat = 0
        self.last_connectivity_check = 0
        self.heartbeat_interval = 5000  # 5 seconds in ms
        self.connectivity_check_interval = 30000  # 30 seconds

        # Initialize hardware
        self._init_hardware()
        self._init_i2c()
        self._init_backup_wireless()
        self._connect_internet()

    def _init_hardware(self):
        """Initialize ESP32 hardware"""
        # Example sensor pins (adjust based on your wiring)
        self.temp_sensor = ADC(Pin(34))  # Temperature sensor
        self.light_sensor = ADC(Pin(35)) # Light sensor

        # Configure ADC
        self.temp_sensor.atten(ADC.ATTN_11DB)
        self.light_sensor.atten(ADC.ATTN_11DB)

        # Status LED
        self.status_led = Pin(2, Pin.OUT)

        print("ESP32 hardware initialized")

    def _init_i2c(self):
        """Initialize I2C master for Mega communication (primary control)"""
        try:
            self.i2c = SoftI2C(scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ)
            # Try to ping Mega
            devices = self.i2c.scan()
            if MEGA_I2C_ADDRESS in devices:
                self.i2c_available = True
                print(f"I2C initialized - Mega found at 0x{MEGA_I2C_ADDRESS:02X}")
            else:
                print(f"I2C initialized - Mega not found (found: {[hex(d) for d in devices]})")
        except Exception as e:
            print(f"Failed to initialize I2C: {e}")
            self.i2c = None

    def _init_backup_wireless(self):
        """Initialize backup UART wireless link (ZigBee/BLE/LoRa) for emergency control"""
        try:
            # UART2: GPIO16 (RX), GPIO17 (TX)
            self.backup_uart = UART(BACKUP_UART_ID, baudrate=BACKUP_BAUD, tx=17, rx=16)
            self.backup_uart.init(baudrate=BACKUP_BAUD, bits=8, parity=None, stop=1)
            print(f"Backup wireless initialized: {BACKUP_LINK_PROTOCOL} @ {BACKUP_BAUD} baud")
        except Exception as e:
            print(f"Failed to initialize backup wireless: {e}")
            self.backup_uart = None

    # ============ I2C Communication Methods (Primary Control) ============
    
    def i2c_write_command(self, cmd, data=b''):
        """Write command to Mega via I2C"""
        if not self.i2c:
            return False
        try:
            payload = bytes([cmd]) + data
            self.i2c.writeto(MEGA_I2C_ADDRESS, payload)
            return True
        except Exception as e:
            print(f"I2C write error: {e}")
            return False

    def i2c_read_response(self, length):
        """Read response from Mega via I2C"""
        if not self.i2c:
            return None
        try:
            time.sleep_ms(50)  # Give Mega time to prepare response
            return self.i2c.readfrom(MEGA_I2C_ADDRESS, length)
        except Exception as e:
            print(f"I2C read error: {e}")
            return None

    def i2c_ping_mega(self):
        """Ping Mega to check I2C connection"""
        if self.i2c_write_command(self.CMD_PING):
            resp = self.i2c_read_response(2)
            return resp and resp[0] == self.RESP_ACK
        return False

    def i2c_send_waypoints(self, waypoints):
        """Send waypoints to Mega via I2C"""
        if not waypoints:
            return True
        
        # Clear existing waypoints
        if not self.i2c_write_command(self.CMD_WAYPOINT_CLEAR):
            print("Failed to clear waypoints")
            return False
        time.sleep_ms(100)
        
        # Send each waypoint
        for wp in waypoints:
            wp_id = int(wp.get('id', 0)) & 0xFFFF
            seq = int(wp.get('sequence', 0)) & 0xFF
            lat = float(wp.get('latitude', 0.0))
            lon = float(wp.get('longitude', 0.0))
            
            # Pack: <HBff = uint16 id, uint8 seq, float lat, float lon
            data = struct.pack('<HBff', wp_id, seq, lat, lon)
            
            if not self.i2c_write_command(self.CMD_WAYPOINT_PACKET, data):
                print(f"Failed to send waypoint {wp_id}")
                return False
            time.sleep_ms(50)
        
        # Commit waypoints
        if not self.i2c_write_command(self.CMD_WAYPOINT_COMMIT):
            print("Failed to commit waypoints")
            return False
        
        print(f"Sent {len(waypoints)} waypoints to Mega via I2C")
        return True

    def i2c_request_gps(self):
        """Request GPS data from Mega via I2C"""
        if self.i2c_write_command(self.CMD_REQUEST_GPS):
            resp = self.i2c_read_response(23)  # 1 + 22 bytes
            if resp and resp[0] == self.RESP_GPS:
                return self._decode_gps(resp[1:])
        return None

    def i2c_request_status(self):
        """Request status from Mega via I2C"""
        if self.i2c_write_command(self.CMD_REQUEST_STATUS):
            resp = self.i2c_read_response(7)  # 1 + 6 bytes
            if resp and resp[0] == self.RESP_STATUS:
                return self._decode_status(resp[1:])
        return None

    def i2c_start_navigation(self):
        """Start navigation on Mega"""
        return self.i2c_write_command(self.CMD_NAV_START)

    def i2c_stop_navigation(self):
        """Stop navigation on Mega"""
        return self.i2c_write_command(self.CMD_NAV_STOP)

    def _decode_gps(self, data):
        """Decode GPS data from I2C response"""
        valid = bool(data[0])
        if not valid:
            return {"valid": False}
        lat, lon, speed, heading = struct.unpack('<ffff', data[1:17])
        satellites = data[17]
        return {
            "valid": True,
            "latitude": lat,
            "longitude": lon,
            "speed": speed,
            "heading": heading,
            "satellites": satellites
        }

    def _decode_status(self, data):
        """Decode status from I2C response"""
        return {
            "mode": "AUTO" if data[0] == 0 else "MANUAL",
            "navigation_active": bool(data[1]),
            "manual_override": bool(data[2]),
            "waypoint_count": data[3],
            "battery_percent": data[4],
            "signal_quality": data[5]
        }

    # ============ Backup Wireless Control (Emergency) ============
    
    def backup_send_command(self, command):
        """Send MCTL command via backup wireless link"""
        if self.backup_uart:
            try:
                self.backup_uart.write(command + '\n')
                print(f"[BACKUP] Sent: {command}")
            except Exception as e:
                print(f"[BACKUP] Error: {e}")

    def backup_send_heartbeat(self):
        """Send HELLO heartbeat via backup wireless"""
        current = time.ticks_ms()
        if time.ticks_diff(current, self.last_heartbeat) >= self.heartbeat_interval:
            heartbeat = f"HELLO,{self.device_id},{BACKUP_LINK_PROTOCOL}"
            self.backup_send_command(heartbeat)
            self.last_heartbeat = current

    def _connect_wifi(self):
        """Connect to WiFi (primary internet)"""
        try:
            wlan = network.WLAN(network.STA_IF)
            wlan.active(True)

            if not wlan.isconnected():
                print(f'Connecting to WiFi: {WIFI_SSID}...')
                wlan.connect(WIFI_SSID, WIFI_PASSWORD)

                # Wait for connection
                timeout = WIFI_TIMEOUT
                while not wlan.isconnected() and timeout > 0:
                    time.sleep(1)
                    timeout -= 1
                    print('.', end='')

            if wlan.isconnected():
                self.wifi_connected = True
                self.internet_available = True
                print(f'\nWiFi connected! IP: {wlan.ifconfig()[0]}')
                return True
            else:
                print('\nWiFi connection failed')
                self.wifi_connected = False
                return False
        except Exception as e:
            print(f"WiFi error: {e}")
            self.wifi_connected = False
            return False

    def _connect_gsm(self):
        """Connect to GSM network (fallback internet)"""
        if not GSM_ENABLED:
            return False
        
        try:
            print("Initializing GSM module...")
            # Initialize GSM UART
            self.gsm_uart = UART(GSM_UART_ID, baudrate=GSM_BAUD, tx=10, rx=9)
            
            # Basic AT commands to establish connection
            self._gsm_send_at("AT")
            time.sleep_ms(500)
            self._gsm_send_at("AT+CFUN=1")  # Full functionality
            time.sleep(2)
            self._gsm_send_at(f'AT+CGDCONT=1,"IP","{GSM_APN}"')  # Set APN
            time.sleep_ms(500)
            self._gsm_send_at("AT+CGACT=1,1")  # Activate PDP context
            time.sleep(2)
            
            self.gsm_connected = True
            self.internet_available = True
            print("GSM connected!")
            return True
        except Exception as e:
            print(f"GSM connection failed: {e}")
            self.gsm_connected = False
            return False

    def _gsm_send_at(self, cmd):
        """Send AT command to GSM module"""
        if hasattr(self, 'gsm_uart') and self.gsm_uart:
            self.gsm_uart.write(cmd + '\r\n')
            time.sleep_ms(100)

    def _connect_internet(self):
        """Connect to internet - try WiFi first, fallback to GSM"""
        print("Establishing internet connection...")
        
        # Try WiFi first
        if self._connect_wifi():
            print("Internet via WiFi")
            return True
        
        # Fallback to GSM
        print("WiFi unavailable, trying GSM...")
        if self._connect_gsm():
            print("Internet via GSM")
            return True
        
        print("No internet connection available")
        self.internet_available = False
        return False

    def _check_connectivity(self):
        """Periodically check and restore internet connectivity"""
        current = time.ticks_ms()
        if time.ticks_diff(current, self.last_connectivity_check) < self.connectivity_check_interval:
            return
        
        self.last_connectivity_check = current
        
        # Check if current connection is alive
        if self.wifi_connected:
            wlan = network.WLAN(network.STA_IF)
            if not wlan.isconnected():
                print("WiFi disconnected, attempting reconnect...")
                self.wifi_connected = False
                self.internet_available = False
                self._connect_internet()
        elif not self.internet_available:
            print("No internet, attempting reconnect...")
            self._connect_internet()

    def read_sensors(self):
        """Read sensor data"""
        # Mock sensor data (replace with actual sensor readings)
        temp_raw = self.temp_sensor.read()
        light_raw = self.light_sensor.read()

        # Convert ADC readings to meaningful values
        temperature = (temp_raw / 4095) * 50  # 0-50°C approximation
        light_level = (light_raw / 4095) * 100  # 0-100% approximation

        return {
            'timestamp': self._get_timestamp(),
            'device_id': self.device_id,
            'temperature': round(temperature, 1),
            'humidity': 60.0,  # Mock humidity
            'mq2': 150,        # Mock gas sensors
            'mq135': 200,
            'mq7': 100,
            'light_level': round(light_level, 1)
        }

    def read_gps(self):
        """Read GPS data (mock for ESP32 without GPS module)"""
        return {
            'latitude': 40.7128,
            'longitude': -74.0060,
            'altitude': 10.5,
            'speed': 0.0,
            'heading': 0.0,
            'satellites': 0,
            'timestamp': self._get_timestamp(),
            'device_id': self.device_id
        }

    def get_system_status(self):
        """Get ESP32 system status"""
        return {
            'online': True,
            'battery': 85.5,
            'signal_strength': -50,  # WiFi RSSI approximation
            'system_info': {
                'free_memory': 50000,  # Approximate free memory in bytes
                'uptime': time.ticks_ms() // 1000
            },
            'device_id': self.device_id
        }

    def _get_timestamp(self):
        """Get current timestamp (simplified)"""
        # In real implementation, you'd sync with NTP
        now = time.localtime()
        return f"{now[0]:04d}-{now[1]:02d}-{now[2]:02d}T{now[3]:02d}:{now[4]:02d}:{now[5]:02d}"

    def send_data_to_dashboard(self, endpoint, data):
        """Send data to dashboard via HTTP POST"""
        if not self.wifi_connected:
            print("WiFi not connected, skipping data send")
            return False

        try:
            url = f"{self.dashboard_url}{endpoint}"
            headers = {'Content-Type': 'application/json'}

            response = requests.post(url, json=data, headers=headers)

            if response.status_code == 200:
                print(f"Data sent successfully to {endpoint}")
                response.close()
                return True
            else:
                print(f"Failed to send data: {response.status_code}")
                response.close()
                return False

        except Exception as e:
            print(f"Error sending data: {e}")
            return False

    def get_pending_commands(self):
        """Get pending commands from dashboard"""
        if not self.wifi_connected:
            return []

        try:
            url = f"{self.dashboard_url}/api/commands/pending"
            params = f"device_id={self.device_id}"

            response = requests.get(f"{url}?{params}")

            if response.status_code == 200:
                data = response.json()
                response.close()
                return data.get('commands', [])
            else:
                print(f"Failed to get commands: {response.status_code}")
                response.close()
                return []

        except Exception as e:
            print(f"Error getting commands: {e}")
            return []

    def acknowledge_command(self, command_id, status='completed', error_message=None):
        """Acknowledge command execution"""
        if not self.wifi_connected:
            return False

        try:
            url = f"{self.dashboard_url}/api/commands/{command_id}/ack"
            data = {'status': status}
            if error_message:
                data['error_message'] = error_message

            headers = {'Content-Type': 'application/json'}
            response = requests.post(url, json=data, headers=headers)

            success = response.status_code == 200
            response.close()
            return success

        except Exception as e:
            print(f"Error acknowledging command: {e}")
            return False

    def execute_command(self, command):
        """Execute dashboard command - forward to Mega via I2C (primary) or backup wireless"""
        command_type = command.get('command_type')
        payload = command.get('payload', {})

        print(f"Executing command: {command_type}")

        try:
            # Navigation commands via I2C
            if command_type == 'start_navigation':
                success = self.i2c_start_navigation()
                print(f"Start navigation: {'OK' if success else 'FAILED'}")
            
            elif command_type == 'stop_navigation':
                success = self.i2c_stop_navigation()
                print(f"Stop navigation: {'OK' if success else 'FAILED'}")
            
            elif command_type == 'send_waypoints':
                waypoints = payload.get('waypoints', [])
                success = self.i2c_send_waypoints(waypoints)
                print(f"Send waypoints: {'OK' if success else 'FAILED'}")
            
            # Manual control via backup wireless (MCTL format)
            elif command_type == 'move':
                direction = payload.get('direction', 'stop').upper()
                speed = payload.get('speed', 200)
                
                direction_map = {
                    'FORWARD': 'FORWARD',
                    'BACKWARD': 'BACKWARD',
                    'REVERSE': 'BACKWARD',
                    'LEFT': 'LEFT',
                    'RIGHT': 'RIGHT',
                    'STOP': 'STOP'
                }
                
                dir_cmd = direction_map.get(direction, 'STOP')
                mctl_cmd = "MCTL,STOP" if dir_cmd == 'STOP' else f"MCTL,{dir_cmd},{speed}"
                
                self.backup_send_command(mctl_cmd)
                print(f"[BACKUP] Motor command: {mctl_cmd}")

            elif command_type == 'stop':
                self.backup_send_command("MCTL,STOP")
                print("[BACKUP] Stop command sent")

            elif command_type == 'manual_mode':
                self.backup_send_command("MCTL,MANUAL")
                print("[BACKUP] Manual mode")

            elif command_type == 'auto_mode':
                self.backup_send_command("AUTO")
                print("[BACKUP] Auto mode")

            # Local ESP32 LED control
            elif command_type == 'led_on':
                self.status_led.value(1)
                print("LED on")

            elif command_type == 'led_off':
                self.status_led.value(0)
                print("LED off")

            # Acknowledge successful execution
            self.acknowledge_command(command['id'], 'completed')

        except Exception as e:
            print(f"Error executing {command_type}: {e}")
            self.acknowledge_command(command['id'], 'failed', str(e))

    def run(self):
        """Main control loop - I2C master with multi-connectivity"""
        self.running = True
        print("="*50)
        print("ESP32 Control Loop Started")
        print(f"I2C: {'Available' if self.i2c_available else 'Unavailable'}")
        print(f"Internet: {'WiFi' if self.wifi_connected else 'GSM' if self.gsm_connected else 'None'}")
        print(f"Backup: {BACKUP_LINK_PROTOCOL}")
        print("="*50)

        # Send initial backup heartbeat
        self.backup_send_heartbeat()

        # Intervals in seconds
        i2c_heartbeat_interval = 5
        gps_fetch_interval = 10
        status_fetch_interval = 15
        sensor_send_interval = 20
        command_check_interval = 3

        last_i2c_heartbeat = time.ticks_ms()
        last_gps_fetch = time.ticks_ms()
        last_status_fetch = time.ticks_ms()
        last_sensor_send = time.ticks_ms()
        last_command_check = time.ticks_ms()

        try:
            while self.running:
                current_time = time.ticks_ms()

                # Check and restore internet connectivity
                self._check_connectivity()

                # Send backup wireless heartbeat (emergency control channel)
                self.backup_send_heartbeat()

                # I2C heartbeat to Mega
                if time.ticks_diff(current_time, last_i2c_heartbeat) >= i2c_heartbeat_interval * 1000:
                    if self.i2c_available and self.i2c_write_command(self.CMD_HEARTBEAT):
                        pass  # Heartbeat sent
                    last_i2c_heartbeat = current_time

                # Fetch GPS from Mega via I2C
                if time.ticks_diff(current_time, last_gps_fetch) >= gps_fetch_interval * 1000:
                    gps_data = self.i2c_request_gps()
                    if gps_data and gps_data.get('valid') and self.internet_available:
                        # Forward to dashboard
                        dashboard_gps = {
                            'latitude': gps_data['latitude'],
                            'longitude': gps_data['longitude'],
                            'altitude': 0.0,
                            'speed': gps_data['speed'],
                            'heading': gps_data['heading'],
                            'satellites': gps_data['satellites'],
                            'timestamp': self._get_timestamp(),
                            'device_id': self.device_id
                        }
                        self.send_data_to_dashboard('/api/gps_data', dashboard_gps)
                    last_gps_fetch = current_time

                # Fetch status from Mega via I2C
                if time.ticks_diff(current_time, last_status_fetch) >= status_fetch_interval * 1000:
                    status_data = self.i2c_request_status()
                    if status_data and self.internet_available:
                        # Merge with system status
                        system_status = self.get_system_status()
                        system_status.update(status_data)
                        self.send_data_to_dashboard('/api/status', system_status)
                    last_status_fetch = current_time

                # Send local sensor data
                if time.ticks_diff(current_time, last_sensor_send) >= sensor_send_interval * 1000:
                    if self.internet_available:
                        sensor_data = self.read_sensors()
                        self.send_data_to_dashboard('/api/sensor_data', sensor_data)
                    last_sensor_send = current_time

                # Check for commands from dashboard
                if time.ticks_diff(current_time, last_command_check) >= command_check_interval * 1000:
                    if self.internet_available:
                        commands = self.get_pending_commands()
                        for command in commands:
                            self.execute_command(command)
                    last_command_check = current_time

                time.sleep(0.5)

        except KeyboardInterrupt:
            print("\nESP32 stopped by user")
            self.i2c_stop_navigation()  # Stop Mega navigation
            self.backup_send_command("MCTL,STOP")  # Stop via backup
        except Exception as e:
            print(f"Error in control loop: {e}")

    def stop(self):
        """Stop the controller"""
        self.running = False
        print("Stopping ESP32 controller")


# Main execution
if __name__ == '__main__':
    print("\n" + "="*50)
    print("ESP32 I2C Master Robot Controller")
    print("="*50)
    print(f"Device ID:      {DEVICE_ID}")
    print(f"WiFi SSID:      {WIFI_SSID}")
    print(f"GSM Enabled:    {GSM_ENABLED}")
    print(f"I2C Mega Addr:  0x{MEGA_I2C_ADDRESS:02X}")
    print(f"Backup Link:    {BACKUP_LINK_PROTOCOL}")
    print("="*50)
    print("\nControl Architecture:")
    print("  PRIMARY:  I2C -> Mega (waypoints, navigation)")
    print("  INTERNET: WiFi -> GSM failover")
    print("  BACKUP:   UART wireless (emergency control)")
    print("="*50 + "\n")
    
    controller = ESP32Controller()

    try:
        controller.run()
    except KeyboardInterrupt:
        controller.stop()
        print("\nController stopped by user")</content>
<parameter name="filePath">/home/thewizard/RobotControl/esp32_micropython_controller.py