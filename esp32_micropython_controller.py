"""
ESP32 Robot Controller (MicroPython)
Connects to the Environmental Monitoring Dashboard

This script runs on ESP32 with MicroPython and:
- Reads sensor data (if connected)
- Sends mock data to dashboard
- Receives and executes commands from dashboard
"""

import time
import json
import network
from machine import Pin, ADC, UART
import urequests as requests
import _thread
from datetime import datetime

# WiFi Configuration
WIFI_SSID = "YOUR_WIFI_SSID"
WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"

# Dashboard Configuration
DASHBOARD_URL = "http://192.168.1.100:5000"  # Change to your dashboard IP
DEVICE_ID = "esp32_01"

class ESP32Controller:
    """ESP32 robot controller using MicroPython"""

    def __init__(self):
        self.device_id = DEVICE_ID
        self.dashboard_url = DASHBOARD_URL
        self.running = False
        self.wifi_connected = False

        # Initialize hardware
        self._init_hardware()
        self._connect_wifi()

    def _init_hardware(self):
        """Initialize ESP32 hardware"""
        # Example sensor pins (adjust based on your wiring)
        self.temp_sensor = ADC(Pin(34))  # Temperature sensor
        self.light_sensor = ADC(Pin(35)) # Light sensor

        # Configure ADC
        self.temp_sensor.atten(ADC.ATTN_11DB)
        self.light_sensor.atten(ADC.ATTN_11DB)

        print("ESP32 hardware initialized")

    def _connect_wifi(self):
        """Connect to WiFi"""
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)

        if not wlan.isconnected():
            print('Connecting to WiFi...')
            wlan.connect(WIFI_SSID, WIFI_PASSWORD)

            # Wait for connection
            timeout = 10
            while not wlan.isconnected() and timeout > 0:
                time.sleep(1)
                timeout -= 1
                print('.')

        if wlan.isconnected():
            self.wifi_connected = True
            print(f'WiFi connected! IP: {wlan.ifconfig()[0]}')
        else:
            print('WiFi connection failed!')
            self.wifi_connected = False

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
        """Execute a received command"""
        command_type = command.get('command_type')
        payload = command.get('payload', {})

        print(f"Executing command: {command_type}")

        try:
            if command_type == 'move':
                direction = payload.get('direction')
                speed = payload.get('speed', 0)
                # Your motor control code here
                print(f"Moving {direction} at speed {speed}")
                # Control motors via GPIO pins

            elif command_type == 'stop':
                print("Stopping ESP32 robot")
                # Stop motors

            elif command_type == 'led_on':
                # Turn on LED
                led = Pin(2, Pin.OUT)
                led.value(1)
                print("LED turned on")

            elif command_type == 'led_off':
                # Turn off LED
                led = Pin(2, Pin.OUT)
                led.value(0)
                print("LED turned off")

            # Acknowledge successful execution
            self.acknowledge_command(command['id'], 'completed')

        except Exception as e:
            print(f"Error executing command {command_type}: {e}")
            self.acknowledge_command(command['id'], 'failed', str(e))

    def run(self):
        """Main control loop"""
        self.running = True
        print("Starting ESP32 control loop")

        sensor_interval = 10  # Send sensor data every 10 seconds
        gps_interval = 15     # Send GPS data every 15 seconds
        status_interval = 30  # Send status every 30 seconds
        command_interval = 5  # Check for commands every 5 seconds

        last_sensor = time.ticks_ms()
        last_gps = time.ticks_ms()
        last_status = time.ticks_ms()
        last_command = time.ticks_ms()

        try:
            while self.running:
                current_time = time.ticks_ms()

                # Send sensor data
                if time.ticks_diff(current_time, last_sensor) >= sensor_interval * 1000:
                    sensor_data = self.read_sensors()
                    self.send_data_to_dashboard('/api/sensor_data', sensor_data)
                    last_sensor = current_time

                # Send GPS data
                if time.ticks_diff(current_time, last_gps) >= gps_interval * 1000:
                    gps_data = self.read_gps()
                    self.send_data_to_dashboard('/api/gps_data', gps_data)
                    last_gps = current_time

                # Send status
                if time.ticks_diff(current_time, last_status) >= status_interval * 1000:
                    status_data = self.get_system_status()
                    self.send_data_to_dashboard('/api/status', status_data)
                    last_status = current_time

                # Check for commands
                if time.ticks_diff(current_time, last_command) >= command_interval * 1000:
                    commands = self.get_pending_commands()
                    for command in commands:
                        # Execute commands (in MicroPython, threading is limited)
                        self.execute_command(command)
                    last_command = current_time

                time.sleep(1)  # Small delay

        except KeyboardInterrupt:
            print("ESP32 control loop stopped")
        except Exception as e:
            print(f"Error in control loop: {e}")

    def stop(self):
        """Stop the controller"""
        self.running = False
        print("Stopping ESP32 controller")


# Main execution
if __name__ == '__main__':
    controller = ESP32Controller()

    try:
        controller.run()
    except KeyboardInterrupt:
        controller.stop()</content>
<parameter name="filePath">/home/thewizard/RobotControl/esp32_micropython_controller.py