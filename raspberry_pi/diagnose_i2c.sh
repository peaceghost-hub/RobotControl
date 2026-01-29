#!/bin/bash
# I2C Connection Diagnostic for Raspberry Pi ↔ Arduino Mega
# This checks all common I2C issues

echo "=========================================="
echo "I2C Connection Diagnostic"
echo "=========================================="
echo ""

# Check if I2C is enabled
echo "1. I2C Kernel Module Status:"
echo "-----------------------------"
if lsmod | grep -q i2c_bcm2835; then
    echo "✓ I2C kernel module loaded (i2c_bcm2835)"
else
    echo "✗ I2C kernel module NOT loaded"
    echo "  Fix: sudo raspi-config → Interface Options → I2C → Enable"
fi

if lsmod | grep -q i2c_dev; then
    echo "✓ I2C device driver loaded (i2c_dev)"
else
    echo "✗ I2C device driver NOT loaded"
fi

echo ""
echo "2. I2C Device Nodes:"
echo "--------------------"
ls -l /dev/i2c* 2>/dev/null || echo "✗ No /dev/i2c-* devices found!"

echo ""
echo "3. User Permissions:"
echo "--------------------"
if groups | grep -q i2c; then
    echo "✓ Current user is in 'i2c' group"
else
    echo "✗ Current user NOT in 'i2c' group"
    echo "  Fix: sudo usermod -aG i2c $USER && newgrp i2c"
fi

echo ""
echo "4. I2C Bus Scan (looking for Mega at address 0x08):"
echo "---------------------------------------------------"
if command -v i2cdetect &> /dev/null; then
    echo "Scanning I2C bus 1..."
    sudo i2cdetect -y 1
    echo ""
    if sudo i2cdetect -y 1 | grep -q '08'; then
        echo "✓ Device found at 0x08 (Arduino Mega)"
    else
        echo "✗ No device at 0x08"
        echo ""
        echo "Common causes:"
        echo "  - Mega not powered or not running sketch"
        echo "  - Wrong I2C address in Mega code (check globals.h)"
        echo "  - SDA/SCL pins not connected"
        echo "  - Missing or wrong pull-up resistors"
        echo "  - No common ground between Pi and Mega"
    fi
else
    echo "✗ i2c-tools not installed"
    echo "  Install: sudo apt-get install -y i2c-tools"
fi

echo ""
echo "5. GPIO Pin Configuration (SDA=GPIO2, SCL=GPIO3):"
echo "--------------------------------------------------"
if command -v raspi-gpio &> /dev/null; then
    echo "GPIO 2 (SDA):"
    raspi-gpio get 2
    echo "GPIO 3 (SCL):"
    raspi-gpio get 3
    echo ""
    if raspi-gpio get 2 | grep -q "level=1" && raspi-gpio get 3 | grep -q "level=1"; then
        echo "✓ Both pins are HIGH (pull-ups working or idle state)"
    else
        echo "⚠ One or both pins LOW (check pull-ups or bus contention)"
    fi
else
    echo "⚠ raspi-gpio not available"
fi

echo ""
echo "6. I2C Bus Speed:"
echo "-----------------"
if [ -f /sys/module/i2c_bcm2835/parameters/baudrate ]; then
    BAUDRATE=$(cat /sys/module/i2c_bcm2835/parameters/baudrate)
    echo "Current I2C baudrate: $BAUDRATE Hz"
    if [ "$BAUDRATE" -gt 100000 ]; then
        echo "⚠ Fast mode (>100kHz) - may need stronger pull-ups"
        echo "  Try lowering to 100kHz: add 'dtparam=i2c_arm_baudrate=100000' to /boot/config.txt"
    else
        echo "✓ Standard mode (≤100kHz)"
    fi
else
    echo "⚠ Cannot read baudrate"
fi

echo ""
echo "=========================================="
echo "WIRING CHECKLIST:"
echo "=========================================="
echo "Verify these connections:"
echo ""
echo "  Raspberry Pi          Arduino Mega"
echo "  ────────────          ────────────"
echo "  Pin 3 (GPIO 2, SDA) ──→ Pin 20 (SDA)"
echo "  Pin 5 (GPIO 3, SCL) ──→ Pin 21 (SCL)"
echo "  Pin 6 (GND)         ──→ GND"
echo ""
echo "Pull-up resistors (4.7kΩ each):"
echo "  SDA line ──[4.7kΩ]── 3.3V (Pi Pin 1) or 5V (Mega)"
echo "  SCL line ──[4.7kΩ]── 3.3V (Pi Pin 1) or 5V (Mega)"
echo ""
echo "Note: Pull-ups to 3.3V are safer for Pi GPIO."
echo "      Pull-ups to 5V work but push specs (Pi tolerates it)."
echo ""
echo "=========================================="
echo "MEGA CHECKLIST:"
echo "=========================================="
echo "1. Is Mega powered on and running?"
echo "2. Is robot_navigation.ino uploaded to Mega?"
echo "3. Check globals.h has: #define I2C_ADDRESS 0x08"
echo "4. Mega serial monitor shows I2C initialization?"
echo ""
echo "=========================================="
echo "NEXT STEPS:"
echo "=========================================="
echo ""
echo "If i2cdetect shows nothing at 0x08:"
echo "  1. Upload a simple I2C slave test to Mega"
echo "  2. Check continuity with multimeter (SDA/SCL/GND)"
echo "  3. Measure voltages: SDA and SCL should be ~3.3V when idle"
echo ""
echo "If i2cdetect shows 0x08:"
echo "  ✓ Hardware is good! Test with Python:"
echo "  python3 -c 'import smbus2; bus=smbus2.SMBus(1); bus.write_byte(0x08, 0x01)'"
