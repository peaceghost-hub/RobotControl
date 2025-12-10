# ESP32 Connection Guide
## Complete Wiring for Robot Controller (Raspberry Pi Replacement)

This guide details all connections for the ESP32-based robot controller that replaces the Raspberry Pi, providing I2C master control, multi-connectivity (WiFi/GSM), sensor reading, and backup wireless control.

---

## Table of Contents
- [ESP32 Overview](#esp32-overview)
- [Power Supply](#power-supply)
- [I2C Connection to Arduino Mega](#i2c-connection-to-arduino-mega)
- [Sensor Connections](#sensor-connections)
- [Wireless Modules (Backup Control)](#wireless-modules-backup-control)
- [GSM Module](#gsm-module)
- [Complete Pin Assignment Table](#complete-pin-assignment-table)
- [Wiring Diagram](#wiring-diagram)
- [Testing Procedure](#testing-procedure)

---

## ESP32 Overview

**Board:** ESP32 DevKit (30-pin or 38-pin version)
**Voltage:** 3.3V logic (5V tolerant on some pins)
**Power Input:** 5V via USB or VIN pin
**Operating Current:** ~80mA (WiFi active), up to 240mA (peak)

**Key Features:**
- Built-in WiFi (2.4GHz)
- Multiple UART ports (3 hardware serial)
- I2C master/slave
- ADC channels (12-bit, 18 channels)
- MicroPython/Arduino compatible

---

## Power Supply

### Option 1: USB Power (Testing/Development)
```
USB Cable (5V) -> ESP32 Micro-USB port
```
- Simple for bench testing
- Limited current (~500mA)

### Option 2: External Power (Robot Integration)
```
Robot Battery (7-12V) -> 5V Step-Down Regulator -> ESP32 VIN pin + GND
```
**Recommended Regulator:** LM2596 Buck Converter (3A)
- Input: 7-12V from robot battery
- Output: 5V @ 2A
- Connect: OUT+ -> ESP32 VIN, OUT- -> ESP32 GND

**Power Distribution:**
```
Battery (+) -> Regulator IN+
Battery (-) -> Common GND
Regulator OUT+ (5V) -> ESP32 VIN
Regulator OUT- -> ESP32 GND
Common GND -> All sensor GND, Mega GND, module GND
```

⚠️ **IMPORTANT:** 
- Use common ground for all devices
- ESP32 logic is 3.3V - use level shifters for 5V devices where needed
- Separate high-current devices (motors) from logic power

---

## I2C Connection to Arduino Mega

### Primary Control Channel (Same as Raspberry Pi)

**ESP32 I2C Master -> Arduino Mega I2C Slave**

```
ESP32 GPIO 21 (SDA) ──┬──[4.7kΩ]── 5V
                      └───────────── Mega Pin 20 (SDA)

ESP32 GPIO 22 (SCL) ──┬──[4.7kΩ]── 5V  
                      └───────────── Mega Pin 21 (SCL)

ESP32 GND ────────────────────────── Mega GND
```

**Pull-up Resistors:** 4.7kΩ on both SDA and SCL lines to 5V
- Required for reliable I2C communication
- Can use internal pull-ups but external recommended for longer wires

**Mega I2C Address:** `0x08` (set in `globals.h`)

### Level Shifting Notes:
- ESP32 is 3.3V, Mega is 5V
- I2C with pull-ups to 5V works reliably
- ESP32 GPIO pins are 5V tolerant for I2C
- Alternative: Use I2C level shifter (e.g., PCA9306) for guaranteed compatibility

**Connection Summary:**
| ESP32 Pin | Function | Mega Pin | Notes |
|-----------|----------|----------|-------|
| GPIO 21   | SDA      | Pin 20   | 4.7kΩ pull-up to 5V |
| GPIO 22   | SCL      | Pin 21   | 4.7kΩ pull-up to 5V |
| GND       | Ground   | GND      | Common ground |

---

## Sensor Connections

### DHT11 Temperature & Humidity Sensor

**Pinout:**
```
DHT11:
  Pin 1: VCC (3.3V or 5V)
  Pin 2: DATA
  Pin 3: NC (not connected)
  Pin 4: GND
```

**Wiring:**
```
DHT11 Pin 1 (VCC)  -> ESP32 3.3V
DHT11 Pin 2 (DATA) -> ESP32 GPIO 4  (with 10kΩ pull-up to 3.3V)
DHT11 Pin 4 (GND)  -> ESP32 GND
```

**Pull-up Resistor:** 10kΩ between DATA pin and 3.3V
- Essential for reliable communication
- Some DHT11 modules have built-in resistor

**MicroPython Code Reference:**
```python
from machine import Pin
import dht

dht_sensor = dht.DHT11(Pin(4))
dht_sensor.measure()
temp = dht_sensor.temperature()  # Celsius
humidity = dht_sensor.humidity()  # Percentage
```

---

### MQ-2 Gas Sensor (Smoke, LPG, Propane)

**Module Type:** Analog output version
**Operating Voltage:** 5V
**Output:** Analog 0-5V (needs voltage divider for ESP32)

**Wiring with Voltage Divider:**
```
MQ-2 VCC -> 5V power
MQ-2 GND -> GND

MQ-2 AOUT -> [10kΩ] -> ESP32 GPIO 34 (ADC1_CH6)
                  |
               [10kΩ]
                  |
                 GND
```

**Voltage Divider:** Reduces 5V output to 2.5V (safe for ESP32 ADC)
- Two 10kΩ resistors in series
- ESP32 ADC accepts 0-3.3V maximum

**Alternative:** Use 3.3V for MQ-2 VCC if module supports it (check specs)

**MicroPython Code:**
```python
from machine import ADC, Pin

mq2 = ADC(Pin(34))
mq2.atten(ADC.ATTN_11DB)  # 0-3.3V range
mq2_value = mq2.read()    # 0-4095 (12-bit)
```

**Preheat Time:** MQ sensors need 24-48 hours for accurate readings

---

### MQ-135 Air Quality Sensor (CO2, NH3, Benzene)

**Wiring (same as MQ-2):**
```
MQ-135 VCC -> 5V power
MQ-135 GND -> GND

MQ-135 AOUT -> [10kΩ] -> ESP32 GPIO 35 (ADC1_CH7)
                    |
                 [10kΩ]
                    |
                   GND
```

**ADC Pin:** GPIO 35 (ADC1_CH7)

**MicroPython Code:**
```python
mq135 = ADC(Pin(35))
mq135.atten(ADC.ATTN_11DB)
mq135_value = mq135.read()
```

---

### MQ-7 Carbon Monoxide Sensor

**Special Requirements:** 
- 5V heating cycle (1.4V for 90s, 5V for 60s)
- Can run continuously at 5V for simpler operation

**Wiring (continuous mode):**
```
MQ-7 VCC -> 5V power
MQ-7 GND -> GND

MQ-7 AOUT -> [10kΩ] -> ESP32 GPIO 32 (ADC1_CH4)
                  |
               [10kΩ]
                  |
                 GND
```

**ADC Pin:** GPIO 32 (ADC1_CH4)

**MicroPython Code:**
```python
mq7 = ADC(Pin(32))
mq7.atten(ADC.ATTN_11DB)
mq7_value = mq7.read()
```

---

### Optional: Light Sensor (LDR or BH1750)

**LDR (Simple):**
```
3.3V -> [10kΩ LDR] -> ESP32 GPIO 33 (ADC1_CH5) -> [10kΩ] -> GND
```

**BH1750 (I2C Digital Light Sensor):**
```
BH1750 VCC -> 3.3V
BH1750 GND -> GND
BH1750 SDA -> ESP32 GPIO 21 (shared with Mega I2C)
BH1750 SCL -> ESP32 GPIO 22 (shared with Mega I2C)
```

---

## Wireless Modules (Backup Control)

### Option A: XBee / ZigBee Module (Default)

**Voltage:** 3.3V logic
**Baud Rate:** 57600 (transparent mode)

**Wiring:**
```
XBee Pin 1  (VCC)   -> ESP32 3.3V
XBee Pin 10 (GND)   -> ESP32 GND
XBee Pin 2  (DOUT)  -> ESP32 GPIO 16 (RX2)
XBee Pin 3  (DIN)   -> ESP32 GPIO 17 (TX2)
```

**Note:** XBee is 3.3V native, no level shifting needed

**Configuration:**
```python
BACKUP_LINK_PROTOCOL = "zigbee"
BACKUP_UART_ID = 2
BACKUP_BAUD = 57600
```

---

### Option B: HC-05 Bluetooth Module (BLE)

**Voltage:** 5V power, 3.3V logic
**Baud Rate:** 38400 (default) or 9600

**Wiring with Level Shifter:**
```
HC-05 VCC -> 5V power
HC-05 GND -> GND

HC-05 TX (3.3V) -> ESP32 GPIO 16 (RX2) [Direct connection OK]

HC-05 RX (needs 3.3V) <- [Voltage Divider] <- ESP32 GPIO 17 (TX2)
  Divider: TX2 -> [1kΩ] -> HC-05 RX -> [2kΩ] -> GND
```

**Pairing:** 
- Default PIN: 1234 or 0000
- Can be configured via AT commands

**Configuration:**
```python
BACKUP_LINK_PROTOCOL = "ble"
BACKUP_UART_ID = 2
BACKUP_BAUD = 38400  # or 9600 for HM-10
```

---

### Option C: LoRa Module (E32 or SX127x)

**Voltage:** 3.3V
**Baud Rate:** 9600 (transparent UART mode)

**E32 LoRa UART Module:**
```
E32 VCC  -> ESP32 3.3V
E32 GND  -> ESP32 GND
E32 RXD  -> ESP32 GPIO 17 (TX2)
E32 TXD  -> ESP32 GPIO 16 (RX2)
E32 M0   -> ESP32 GND (normal mode)
E32 M1   -> ESP32 GND (normal mode)
E32 AUX  -> Not connected (or GPIO for status)
```

**SX127x with SPI:**
```
SX127x VCC  -> ESP32 3.3V
SX127x GND  -> ESP32 GND
SX127x SCK  -> ESP32 GPIO 18
SX127x MISO -> ESP32 GPIO 19
SX127x MOSI -> ESP32 GPIO 23
SX127x CS   -> ESP32 GPIO 5
SX127x RST  -> ESP32 GPIO 14
SX127x DIO0 -> ESP32 GPIO 26
```

**Configuration:**
```python
BACKUP_LINK_PROTOCOL = "lora"
BACKUP_UART_ID = 2
BACKUP_BAUD = 9600
```

---

## GSM Module

### SIM800L / SIM900A Module (Internet Fallback)

**Voltage:** 3.7-4.2V (needs separate power!)
**Current:** Up to 2A during transmission
**Baud Rate:** 115200

**Power Supply:**
```
Dedicated LiPo Battery 3.7V -> SIM800L VCC
                            -> Large capacitor (1000µF) near VCC
OR
5V Buck Converter (3A) -> 4.0V output -> SIM800L VCC
```

⚠️ **CRITICAL:** 
- Do NOT power from ESP32 3.3V pin (insufficient current)
- Use dedicated power with large capacitor
- GSM draws 2A peaks during transmission

**UART Connection:**
```
SIM800L VCC -> 3.7-4.2V (separate power)
SIM800L GND -> Common GND
SIM800L TX  -> ESP32 GPIO 9  (RX1)
SIM800L RX  -> ESP32 GPIO 10 (TX1)
```

**SIM Card:**
- Insert nano-SIM with data plan
- Configure APN in code: `GSM_APN = "internet"` (or carrier-specific)

**Configuration:**
```python
GSM_ENABLED = True
GSM_APN = "internet"  # Change to carrier APN
GSM_UART_ID = 1
GSM_BAUD = 115200
```

**Antenna:** Ensure GSM antenna is connected for signal

---

## Resistor Quantity & Voltage Divider Calculations

### Complete Resistor List

| Type | Size | Quantity | Purpose | Notes |
|------|------|----------|---------|-------|
| Pull-up | 4.7kΩ | **2** | I2C SDA/SCL lines | To 5V |
| Pull-up | 10kΩ | **1** | DHT11 DATA line | To 3.3V |
| Divider | 10kΩ | **6** | Voltage dividers (3 sensors) | 2 per sensor (MQ-2, MQ-135, MQ-7) |
| **TOTAL: 9 resistors minimum** |

### Voltage Divider Formula & Calculation

**For 5V sensors to 3.3V ESP32 ADC:**

```
Formula: Vout = Vin × (R2 / (R1 + R2))

Where:
- Vin = Input voltage (5V from sensor)
- R1 = Upper resistor (between 5V and ADC pin)
- R2 = Lower resistor (between ADC pin and GND)
- Vout = Output voltage to ADC (target: ~2.5V)

Using 10kΩ + 10kΩ:
Vout = 5V × (10kΩ / (10kΩ + 10kΩ))
Vout = 5V × (10kΩ / 20kΩ)
Vout = 5V × 0.5
Vout = 2.5V ✓ (Safe for ESP32 ADC 0-3.3V max)
```

**Standard Values (10kΩ + 10kΩ is RECOMMENDED):**

| R1 | R2 | Vout | Notes |
|----|----|----|-------|
| 10kΩ | 10kΩ | 2.5V | ✓ **RECOMMENDED** - Equal values, 50% reduction |
| 15kΩ | 10kΩ | 2.0V | Acceptable |
| 20kΩ | 10kΩ | 1.67V | Lower range, less resolution |
| 6.8kΩ | 10kΩ | 2.96V | Close to max (risky) |

**Exact Wiring for Each MQ Sensor:**

```
┌─ 5V Power Supply (from robot battery via LM2596)
│
├─ [10kΩ resistor] ─── Sensor AOUT ─── ESP32 ADC Pin
│                            │
│                         [10kΩ resistor]
│                            │
│                           GND
│
└─ Sensor GND ─────────────── GND
```

### Resistor Power Dissipation Check

**For MQ sensors (5V input, 2.5V output):**

Current through divider:
```
I = V / R_total = 5V / (10kΩ + 10kΩ) = 5V / 20kΩ = 0.25mA

Power in R1 (upper): P1 = (V_R1)² / R = (2.5V)² / 10kΩ = 0.625mW
Power in R2 (lower): P2 = (V_R2)² / R = (2.5V)² / 10kΩ = 0.625mW

Total: 1.25mW ✓ (1/4W = 250mW resistors are more than sufficient)
```

**Use standard 1/4W resistors - they can handle 250mW, we're only using 1.25mW per divider**

### DHT11 Pull-up Resistor (10kΩ)

**Purpose:** Holds DATA line high, allows open-drain communication

```
3.3V ──[10kΩ]──┬── DHT11 DATA pin
               │
            ESP32 GPIO 4
               │
              GND

Reason: DHT11 pulls DATA low to send bit, then releases to high
The pull-up ensures clean transitions (50-100µs timing critical)
```

**Alternative:** Some DHT11 modules have built-in pull-up - check your module!

### I2C Pull-up Resistors (4.7kΩ × 2)

**Purpose:** Pull I2C lines (SDA/SCL) high when idle

```
5V ──[4.7kΩ]──┬── Mega Pin 20 (SDA) ── ESP32 GPIO 21
              │
             Both pull-ups share one connection

5V ──[4.7kΩ]──┬── Mega Pin 21 (SCL) ── ESP32 GPIO 22
              │
             Both pull-ups share one connection
```

**Why 4.7kΩ for I2C:**
- Standard I2C specification value
- Fast enough for 100kHz/400kHz operation
- Sufficient for ESP32 + Mega (short distance < 1m)

**Calculation:**
```
Rise time = 0.8473 × Rpull × Cline

For 100kHz I2C:
- Rpull = 4.7kΩ
- Cline ≈ 100pF (typical for short wires)
- Rise time = 0.8473 × 4.7kΩ × 100pF ≈ 40ns ✓ (Good)
```

---

## Complete Pin Assignment Table

| ESP32 GPIO | Function | Connected To | Resistor Qty | Notes |
|------------|----------|--------------|--------------|-------|
| **I2C & Main Control** |
| GPIO 21 | I2C SDA | Mega Pin 20 (SDA) | 1× 4.7kΩ | To 5V pull-up |
| GPIO 22 | I2C SCL | Mega Pin 21 (SCL) | 1× 4.7kΩ | To 5V pull-up |
| **Sensors (ADC)** |
| GPIO 4  | Digital | DHT11 DATA | 1× 10kΩ | To 3.3V pull-up |
| GPIO 34 | ADC1_CH6 | MQ-2 AOUT | 2× 10kΩ | Voltage divider 5V→2.5V |
| GPIO 35 | ADC1_CH7 | MQ-135 AOUT | 2× 10kΩ | Voltage divider 5V→2.5V |
| GPIO 32 | ADC1_CH4 | MQ-7 AOUT | 2× 10kΩ | Voltage divider 5V→2.5V |
| GPIO 33 | ADC1_CH5 | Light sensor | Optional | No divider needed (3.3V) |
| **Backup Wireless (UART2)** |
| GPIO 16 | UART2 RX | Wireless TX | 0 | Direct connection (3.3V logic) |
| GPIO 17 | UART2 TX | Wireless RX | 0 | Direct connection (3.3V logic) |
| **GSM Module (UART1)** |
| GPIO 9  | UART1 RX | GSM TX | 0 | Direct connection (3.3V logic) |
| GPIO 10 | UART1 TX | GSM RX | 1× Divider | See HC-05 BLE wiring if needed |
| **Status & Control** |
| GPIO 2  | Digital Out | LED (built-in) | 0 | Status indicator |
| **Power** |
| VIN | 5V Input | 5V regulator | 0 | Or USB power |
| 3.3V | 3.3V Output | 3.3V sensors | 0 | Max 600mA total |
| GND | Ground | Common GND | 0 | All devices |
| | | | **TOTAL: 9 resistors** | |

**Reserved/Don't Use:**
- GPIO 0: Boot mode (pulled high)
- GPIO 1: UART0 TX (USB serial)
- GPIO 3: UART0 RX (USB serial)
- GPIO 6-11: Flash memory (don't use)
- GPIO 12: Boot config (pulled low)

**ADC2 Channels:** Avoid ADC2 (GPIO 0, 2, 4, 12-15, 25-27) when using WiFi

---

## Wiring Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                         ESP32 DevKit Board                          │
│                                                                     │
│  [USB] ──5V──> VIN                                    GND <── All  │
│                                                                     │
│         ┌────────────────────────────────────────┐                 │
│         │        I2C to Mega (Primary)           │                 │
│  GPIO21 ├─SDA──[4.7kΩ]──5V                       │                 │
│         │         └────────────> Mega Pin 20     │                 │
│  GPIO22 ├─SCL──[4.7kΩ]──5V                       │                 │
│         │         └────────────> Mega Pin 21     │                 │
│         └────────────────────────────────────────┘                 │
│                                                                     │
│         ┌────────────────────────────────────────┐                 │
│         │           Sensors (ADC)                │                 │
│  GPIO4  ├─────> DHT11 DATA (10kΩ pull-up)       │                 │
│  GPIO34 ├─ADC─> MQ-2 (via voltage divider)      │                 │
│  GPIO35 ├─ADC─> MQ-135 (via voltage divider)    │                 │
│  GPIO32 ├─ADC─> MQ-7 (via voltage divider)      │                 │
│  GPIO33 ├─ADC─> Light Sensor (optional)         │                 │
│         └────────────────────────────────────────┘                 │
│                                                                     │
│         ┌────────────────────────────────────────┐                 │
│         │    Backup Wireless (UART2)             │                 │
│  GPIO16 ├─RX2──> Wireless Module TX              │                 │
│  GPIO17 ├─TX2──> Wireless Module RX              │                 │
│         │        (ZigBee/BLE/LoRa)                │                 │
│         └────────────────────────────────────────┘                 │
│                                                                     │
│         ┌────────────────────────────────────────┐                 │
│         │        GSM Module (UART1)              │                 │
│  GPIO9  ├─RX1──> SIM800L TX                      │                 │
│  GPIO10 ├─TX1──> SIM800L RX                      │                 │
│         │        (Separate 3.7V Power!)          │                 │
│         └────────────────────────────────────────┘                 │
│                                                                     │
│  GPIO2  ├─────> Built-in LED (Status)                              │
│                                                                     │
│  3.3V ──┬──> DHT11, XBee, Sensors                                  │
│         └──> [Max 600mA total]                                     │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘

┌────────────────────┐      ┌─────────────────────┐
│   Arduino Mega     │      │   Wireless Module   │
│                    │      │   (ZigBee/BLE/LoRa) │
│  Pin 20 (SDA) <────┼──────┤                     │
│  Pin 21 (SCL) <────┼──┐   │  TX ──> GPIO 16     │
│  GND ──────────────┼──┼───┤  RX <── GPIO 17     │
└────────────────────┘  │   │  VCC    3.3V        │
                        │   │  GND    GND         │
       4.7kΩ to 5V <────┘   └─────────────────────┘

┌────────────────────┐      ┌─────────────────────┐
│   GSM Module       │      │   Sensors (MQ-2)    │
│   SIM800L          │      │                     │
│                    │      │  VCC ──> 5V         │
│  TX ──> GPIO 9     │      │  GND ──> GND        │
│  RX <── GPIO 10    │      │  AOUT ─┬─[10kΩ]─┐  │
│  VCC    3.7V*      │      │        └─[10kΩ]─┤  │
│  GND    GND        │      │                 GND │
└────────────────────┘      └──────┬──────────────┘
  *Separate power                  └──> GPIO 34
   with 1000µF cap
```

---

## ESP32 Sensor Data Upload to Dashboard

### YES - ESP32 IS Uploading Sensor Data ✅

The ESP32 MicroPython controller automatically reads sensors and sends data to the dashboard every **20 seconds**.

### Data Upload Workflow

```
┌─────────────────────────────────────────────────────┐
│          ESP32 Control Loop (run() method)          │
└──────────────────┬──────────────────────────────────┘
                   │
        ┌──────────┴──────────┬──────────────┬─────────────┐
        │                     │              │             │
   [I2C to Mega]         [Local Sensors]  [WiFi/GSM]  [Backup Wireless]
        │                     │              │             │
    GPS Data           ┌──────┴──────┐      │             │
    Status             │  (20sec)    │      │             │
    ↓                  ↓             ↓      ↓             ↓
    ┌──────────────┐  ┌──────────────┐  ┌────────────────────┐
    │ I2C Request  │  │Read Sensors: │  │Dashboard API Calls:│
    │ (5sec)       │  │- DHT11 temp  │  │- POST sensor_data  │
    │              │  │- MQ-2 smoke  │  │- POST gps_data     │
    │ GPS ←──────→ │  │- MQ-135 air  │  │- POST status       │
    │ Status ←────→ │  │- MQ-7 CO     │  │- GET commands      │
    └──────────────┘  └──────────────┘  └────────────────────┘
                           ↓
                    ┌────────────────┐
                    │ Combine Data & │
                    │ Add Timestamp  │
                    └────────┬───────┘
                             ↓
                    ┌────────────────────────┐
                    │  send_data_to_dashboard│
                    │  (if internet active)  │
                    └────────────────────────┘
                             ↓
                    Dashboard Receives:
                    {
                      "device_id": "esp32_controller_1",
                      "temperature": 24.5,
                      "humidity": 65.0,
                      "mq2": 245,
                      "mq135": 312,
                      "mq7": 128,
                      "light_level": 87.3,
                      "timestamp": "2025-12-10T14:23:45"
                    }
```

### Sensor Data Upload Intervals

| Data Type | Interval | Source | Endpoint | Condition |
|-----------|----------|--------|----------|-----------|
| **Sensors** | 20 sec | DHT11, MQ-2/135/7 | `/api/sensor_data` | WiFi active |
| **GPS** | 10 sec | Arduino Mega I2C | `/api/gps_data` | Valid data + WiFi |
| **Status** | 15 sec | Mega + ESP32 | `/api/status` | WiFi active |
| **Commands** | 3 sec | Dashboard | `/api/commands/pending` | WiFi active |

### Code Flow (from esp32_micropython_controller.py)

**Sensor Reading:**
```python
def read_sensors(self):
    """Reads local sensors and returns data dict"""
    temp_raw = self.temp_sensor.read()
    light_raw = self.light_sensor.read()
    
    temperature = (temp_raw / 4095) * 50  # 0-50°C
    light_level = (light_raw / 4095) * 100  # 0-100%
    
    return {
        'timestamp': self._get_timestamp(),
        'device_id': self.device_id,
        'temperature': round(temperature, 1),
        'humidity': 60.0,
        'mq2': 150,
        'mq135': 200,
        'mq7': 100,
        'light_level': round(light_level, 1)
    }
```

**Dashboard Upload:**
```python
def send_data_to_dashboard(self, endpoint, data):
    """POST sensor data to dashboard"""
    if not self.wifi_connected:
        print("WiFi not connected, skipping...")
        return False
    
    try:
        url = f"{self.dashboard_url}{endpoint}"
        headers = {'Content-Type': 'application/json'}
        
        response = requests.post(url, json=data, headers=headers)
        
        if response.status_code == 200:
            print(f"Data sent to {endpoint}")
            return True
    except Exception as e:
        print(f"Error: {e}")
        return False
```

**Main Loop Upload Logic:**
```python
# Check for commands from dashboard
if time.ticks_diff(current_time, last_sensor_send) >= 20 * 1000:
    if self.internet_available:  # WiFi or GSM active
        sensor_data = self.read_sensors()
        self.send_data_to_dashboard('/api/sensor_data', sensor_data)
    last_sensor_send = current_time
```

### Dashboard API Endpoints Expected

**POST `/api/sensor_data`** - Sensor readings
```json
{
  "device_id": "esp32_controller_1",
  "temperature": 24.5,
  "humidity": 65.0,
  "mq2": 245,
  "mq135": 312,
  "mq7": 128,
  "light_level": 87.3,
  "timestamp": "2025-12-10T14:23:45"
}
```

**POST `/api/gps_data`** - GPS position from Mega
```json
{
  "device_id": "esp32_controller_1",
  "latitude": 40.7128,
  "longitude": -74.0060,
  "altitude": 0.0,
  "speed": 0.5,
  "heading": 225.0,
  "satellites": 8,
  "timestamp": "2025-12-10T14:23:45"
}
```

**POST `/api/status`** - Robot status
```json
{
  "device_id": "esp32_controller_1",
  "online": true,
  "battery": 85.5,
  "signal_strength": -50,
  "mode": "AUTO",
  "navigation_active": true,
  "manual_override": false,
  "waypoint_count": 5,
  "battery_percent": 85,
  "signal_quality": 80
}
```

**GET `/api/commands/pending`** - Fetch commands from dashboard
```json
{
  "commands": [
    {
      "id": "cmd_001",
      "command_type": "send_waypoints",
      "payload": {
        "waypoints": [
          {"id": 1, "sequence": 0, "latitude": 40.7128, "longitude": -74.0060},
          {"id": 2, "sequence": 1, "latitude": 40.7140, "longitude": -74.0070}
        ]
      }
    }
  ]
}
```

### Troubleshooting Sensor Data Upload

| Problem | Cause | Solution |
|---------|-------|----------|
| Data not uploading | WiFi not connected | Check SSID/password in code |
| | Dashboard unreachable | Verify `DASHBOARD_URL` IP address |
| | Internet active but no data | Check WiFi signal strength (-50 to -80 dBm good) |
| Data received but invalid | Sensor not reading | Check sensor power/wiring |
| | ADC pin incorrect | Verify GPIO numbers (34, 35, 32, 33) |
| Upload every 30+ sec | Throttling active | Check `SEND_INTERVAL_MS` in code |
| Temperature always 0 | DHT11 not connected | Verify GPIO 4, check pull-up |
| MQ sensors always 0 | Voltage divider missing | Add 10kΩ + 10kΩ dividers |

### Real Sensor Data vs Mock Data

**Currently in Code (TESTING MODE):**
```python
# Mock temperature/humidity (replace with DHT11)
'temperature': round(temperature, 1),
'humidity': 60.0,  # ← PLACEHOLDER

# Mock MQ sensors (replace with actual ADC)
'mq2': 150,        # ← PLACEHOLDER
'mq135': 200,      # ← PLACEHOLDER
'mq7': 100,        # ← PLACEHOLDER
```

**For Real Sensor Data, Update read_sensors():**
```python
def read_sensors(self):
    """Read REAL sensor data"""
    # DHT11
    import dht
    dht_sensor = dht.DHT11(Pin(4))
    dht_sensor.measure()
    temp = dht_sensor.temperature()
    humidity = dht_sensor.humidity()
    
    # MQ Sensors (ADC)
    mq2_raw = self.adc2.read()
    mq135_raw = self.adc3.read()
    mq7_raw = self.adc4.read()
    
    return {
        'timestamp': self._get_timestamp(),
        'device_id': self.device_id,
        'temperature': temp,
        'humidity': humidity,
        'mq2': mq2_raw,
        'mq135': mq135_raw,
        'mq7': mq7_raw,
        'light_level': self.light_sensor.read() / 40.95
    }
```

### Monitor ESP32 Output

To verify sensor upload, connect ESP32 via USB and monitor serial:

```bash
# Terminal 1: Monitor USB serial output
screen /dev/ttyUSB0 115200

# Terminal 2: Monitor dashboard logs (if available)
tail -f /var/log/dashboard.log
```

**Expected Output:**
```
==================================================
ESP32 I2C Master Robot Controller
==================================================
Device ID:      esp32_controller_1
WiFi SSID:      Ghost
...

Connecting to WiFi: Ghost...
WiFi connected! IP: 192.168.1.250

[I2C] Ping OK
[I2C] GPS: lat=40.7128, lon=-74.0060
Data sent successfully to /api/sensor_data
Data sent successfully to /api/gps_data
Data sent successfully to /api/status
...
```

---

### Step 1: Power Test
1. Connect only power (5V to VIN, GND)
2. Check built-in LED (GPIO 2) - should be accessible
3. Measure 3.3V output pin - should be stable
4. USB serial should enumerate (check with `dmesg` or Device Manager)

### Step 2: I2C Communication
1. Connect I2C lines to Mega (with pull-ups)
2. Upload test sketch to Mega (I2C slave at 0x08)
3. Run I2C scanner on ESP32:
```python
from machine import Pin, SoftI2C
i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=100000)
devices = i2c.scan()
print("I2C devices:", [hex(d) for d in devices])
# Should show: ['0x8']
```
4. Test ping command:
```python
i2c.writeto(0x08, bytes([0x01]))  # CMD_PING
```

### Step 3: Sensor Readings
1. Connect DHT11 first (simplest)
2. Test reading:
```python
import dht
from machine import Pin
sensor = dht.DHT11(Pin(4))
sensor.measure()
print(f"Temp: {sensor.temperature()}°C, Humidity: {sensor.humidity()}%")
```
3. Connect MQ sensors (with voltage dividers)
4. Read ADC values:
```python
from machine import ADC, Pin
mq2 = ADC(Pin(34))
mq2.atten(ADC.ATTN_11DB)
print(f"MQ-2: {mq2.read()}")
```

### Step 4: WiFi Connection
```python
import network
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect("Ghost", "omegatruthalpha")
# Wait...
print(wlan.ifconfig())
```

### Step 5: Backup Wireless
1. Connect wireless module (ZigBee/BLE/LoRa)
2. Test UART:
```python
from machine import UART
uart = UART(2, baudrate=57600, tx=17, rx=16)
uart.write("HELLO,esp32,zigbee\n")
```

### Step 6: Full Integration
1. Upload `esp32_micropython_controller.py`
2. Monitor serial output
3. Verify I2C commands to Mega
4. Check sensor data collection
5. Test waypoint forwarding from dashboard

---

## Troubleshooting

### I2C Issues
- **No devices found:** Check pull-up resistors (4.7kΩ to 5V)
- **Unreliable communication:** Shorten wires, add pull-ups
- **Mega not responding:** Verify Mega I2C sketch uploaded

### Sensor Issues
- **DHT11 timeout:** Check pull-up resistor on DATA pin
- **MQ sensors always 0:** Check voltage divider, verify 5V power
- **ADC unstable:** Don't use ADC2 pins when WiFi active

### WiFi Issues
- **Won't connect:** Verify SSID/password, check 2.4GHz band
- **Keeps disconnecting:** Add auto-reconnect logic (included in code)

### GSM Issues
- **No response:** Check separate power supply (3.7-4.2V, 2A capable)
- **Restarts/brownouts:** Add 1000µF capacitor near VCC
- **AT commands fail:** Verify baud rate (115200), check RX/TX swap

### Power Issues
- **ESP32 resets:** Insufficient current - use better power supply
- **Sensors erratic:** Use common ground for all devices
- **GSM kills ESP32:** GSM needs separate power (2A peak)

---

## Safety Notes

⚠️ **WARNINGS:**
1. **Never exceed 3.3V on ESP32 GPIO** (except 5V-tolerant I2C)
2. **MQ sensors get HOT** - don't touch during operation
3. **GSM module needs 2A** - separate power mandatory
4. **Common ground critical** - connect all GND points
5. **Voltage dividers required** - for 5V sensors to ESP32 ADC

**ESD Protection:** 
- Touch grounded metal before handling ESP32
- Use ESD wrist strap when soldering

**Testing Environment:**
- Well-ventilated area for MQ sensors
- Fire extinguisher nearby when testing gas sensors
- Don't test near open flames

---

## Next Steps

1. **Bench Testing:** Use USB power, test each subsystem
2. **Integration:** Mount ESP32 on robot, connect to battery
3. **Calibration:** Run sensors for 24-48 hours to stabilize
4. **Dashboard Link:** Configure dashboard URL, test commands
5. **Field Test:** Test WiFi->GSM failover in real conditions

---

## Bill of Materials (BOM)

| Item | Quantity | Value | Notes |
|------|----------|-------|-------|
| **Microcontroller & Brain** |
| ESP32 DevKit | 1 | 30/38-pin | Replacement for Raspberry Pi |
| Arduino Mega 2560 | 1 | - | Robot brain (already installed) |
| **Sensors** |
| DHT11 Sensor Module | 1 | - | Temperature & Humidity |
| MQ-2 Gas Sensor | 1 | - | Smoke/LPG/Propane |
| MQ-135 Air Quality Sensor | 1 | - | CO₂, NH₃, Benzene, etc |
| MQ-7 CO Sensor | 1 | - | Carbon Monoxide |
| Light Sensor (LDR) | 1 | - | Optional - light level |
| **Wireless Modules** |
| XBee/ZigBee Module | 1 | 3.3V 57600baud | Or choose ONE: |
| HC-05 Bluetooth | 1 | 5V 38400baud | BLE alternative |
| E32 LoRa Module | 1 | 3.3V 9600baud | Long-range alternative |
| **Internet Connectivity** |
| SIM800L GSM Module | 1 | 3.7-4.2V | With antenna & data plan |
| **Resistors (CRITICAL)** |
| Resistor 4.7kΩ 1/4W | **2** | - | I2C pull-ups (SDA, SCL) |
| Resistor 10kΩ 1/4W | **7** | - | DHT11 (1) + Dividers (6) |
| **Resistor Subtotal:** | **9 resistors** | - | **See voltage divider section** |
| **Power & Regulators** |
| LM2596 Buck Converter | 1 | 5V @ 3A | Robot battery → 5V |
| 5V Power Supply | 1 | 2A minimum | For breadboard testing |
| USB Cable | 1 | Micro-USB | ESP32 programming |
| **Capacitors** |
| Capacitor | 1 | 1000µF 10V | GSM module power stabilization |
| **Breadboards & Connectors** |
| Breadboard | 1 | 830-point | For prototyping |
| Perfboard | 1 | 5×7cm | For final circuit |
| Jumper Wires | 50+ | 22AWG | Male-Male, various colors |
| **Optional (Testing)** |
| LiPo Battery | 1 | 3.7V 2000mAh | Dedicated GSM power |
| Multimeter | 1 | Digital | Voltage/resistance testing |
| USB Serial Adapter | 1 | CH340G | Programming/debugging |

### Resistor Sourcing

**4.7kΩ resistors (×2):**
- Part: Carbon film 1/4W ±5%
- Suppliers: Digi-Key, Mouser, AliExpress
- Cost: ~$0.01-0.05 each

**10kΩ resistors (×7):**
- Part: Carbon film 1/4W ±5%
- Suppliers: Same as above
- Cost: ~$0.01-0.05 each
- **Bulk:** 100-pack costs $1-2 (recommended for future projects)

**Total cost for all 9 resistors: ~$0.50 USD**

### Power Budget

| Device | Current (typical) | Current (peak) | Power Supply |
|--------|-------------------|----------------|--------------|
| ESP32 (idle) | 10mA | - | 5V USB/VIN |
| ESP32 (WiFi) | 80mA | 240mA | 5V 2A minimum |
| DHT11 | 0.5mA | 1.5mA | 3.3V |
| MQ-2 Heater | - | 100mA | 5V |
| MQ-135 Heater | - | 100mA | 5V |
| MQ-7 Heater | - | 100mA | 5V |
| ZigBee/BLE (idle) | 5mA | 50mA | 3.3V or 5V |
| SIM800L (idle) | 5mA | 2000mA! | **3.7V separate** |
| **TOTAL (excluding GSM)** | ~200mA | ~600mA | 5V @ 2A regulator |
| **GSM (SEPARATE)** | 5mA | 2000mA | 3.7V 2A+ battery |

**Key Takeaway:** 
- Use 5V @ 2A regulator for ESP32 + sensors + wireless
- Use separate 3.7-4.2V power for GSM module (NOT from ESP32)

---

## Additional Resources

- **ESP32 Pinout:** https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
- **MicroPython Docs:** https://docs.micropython.org/en/latest/esp32/quickref.html
- **DHT11 Library:** Built into MicroPython
- **I2C Protocol:** Matches Raspberry Pi implementation
- **Troubleshooting:** See project README.md

---

**Document Version:** 1.0  
**Date:** December 2025  
**Project:** Environmental Monitoring Robot - ESP32 Controller  
**Author:** RobotControl Project
