#!/bin/bash
# Raspberry Pi UART Diagnostic Script

echo "============================================"
echo "Raspberry Pi UART Configuration Check"
echo "============================================"
echo ""

echo "1. UART Device Status:"
echo "----------------------"
ls -la /dev/serial* /dev/ttyAMA* /dev/ttyS* 2>/dev/null

echo ""
echo "2. UART Processes (checking if console/getty is using UART):"
echo "-------------------------------------------------------------"
ps aux | grep -E 'getty|serial|console' | grep -v grep

echo ""
echo "3. Bluetooth Status (may use UART on Pi 3/4):"
echo "----------------------------------------------"
systemctl status bluetooth 2>/dev/null | head -5 || echo "Bluetooth service not found"
hciconfig 2>/dev/null || echo "hciconfig not available"

echo ""
echo "4. Boot Config (checking /boot/config.txt or /boot/firmware/config.txt):"
echo "------------------------------------------------------------------------"
if [ -f /boot/config.txt ]; then
    grep -E 'enable_uart|dtoverlay.*uart|console' /boot/config.txt | grep -v '^#' || echo "No UART settings found"
elif [ -f /boot/firmware/config.txt ]; then
    grep -E 'enable_uart|dtoverlay.*uart|console' /boot/firmware/config.txt | grep -v '^#' || echo "No UART settings found"
else
    echo "Config file not found"
fi

echo ""
echo "5. Kernel Command Line (checking for console on serial):"
echo "---------------------------------------------------------"
cat /proc/cmdline | grep -o 'console=[^ ]*' || echo "No console= parameter"

echo ""
echo "6. Device Tree Info:"
echo "--------------------"
dtoverlay -l 2>/dev/null | grep -i uart || echo "dtoverlay command not available"

echo ""
echo "============================================"
echo "DIAGNOSIS:"
echo "============================================"

if ps aux | grep -q '[g]etty.*tty'; then
    echo "⚠️  WARNING: Getty (login console) is running on serial port"
    echo "   Fix: sudo raspi-config → Interface Options → Serial Port"
    echo "   → Disable login shell (NO), Enable hardware (YES)"
fi

if systemctl is-active bluetooth &>/dev/null; then
    echo "⚠️  WARNING: Bluetooth may be using the UART (common on Pi 3/Zero W/4)"
    echo "   If you need UART for SIM7600E, disable BT or use miniUART:"
    echo "   Add to /boot/config.txt: dtoverlay=disable-bt"
    echo "   Then: sudo systemctl disable hciuart"
fi

echo ""
echo "============================================"
echo "QUICK FIX COMMANDS:"
echo "============================================"
echo "1. Disable serial console + enable UART hardware:"
echo "   sudo raspi-config"
echo "   → 3 Interface Options → I6 Serial Port"
echo "   → Login shell: NO | Hardware: YES"
echo ""
echo "2. If Bluetooth conflicts (Pi 3/4), disable it:"
echo "   echo 'dtoverlay=disable-bt' | sudo tee -a /boot/config.txt"
echo "   sudo systemctl disable hciuart"
echo "   sudo reboot"
echo ""
echo "3. After fixing, reboot and re-run UART test"
