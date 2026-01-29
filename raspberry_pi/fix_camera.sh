#!/bin/bash
# Fix Pi Camera "Out of Memory" Error
# Run this on the Raspberry Pi

echo "========================================"
echo "Pi Camera Diagnostics & Fix"
echo "========================================"
echo ""

echo "1. Checking for processes using camera..."
echo "----------------------------------------"
CAMERA_PROCS=$(sudo fuser /dev/vchiq 2>/dev/null)
if [ -n "$CAMERA_PROCS" ]; then
    echo "⚠️  Found processes using camera: $CAMERA_PROCS"
    ps -p $CAMERA_PROCS -o pid,comm,cmd
    echo ""
    read -p "Kill these processes? (y/n): " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo kill -9 $CAMERA_PROCS
        echo "✓ Killed processes"
    fi
else
    echo "✓ No processes holding camera"
fi
echo ""

echo "2. Checking camera detection..."
echo "----------------------------------------"
vcgencmd get_camera
echo ""

echo "3. Checking GPU memory allocation..."
echo "----------------------------------------"
GPU_MEM=$(vcgencmd get_mem gpu | grep -oP '\d+')
echo "Current GPU memory: ${GPU_MEM}M"
if [ "$GPU_MEM" -lt 128 ]; then
    echo "⚠️  WARNING: GPU memory is low (${GPU_MEM}M)"
    echo "   Camera needs at least 128M GPU memory"
    echo ""
    echo "   To fix, add this to /boot/config.txt:"
    echo "   gpu_mem=256"
    echo ""
    echo "   Then reboot with: sudo reboot"
else
    echo "✓ GPU memory is adequate (${GPU_MEM}M)"
fi
echo ""

echo "4. Checking camera modules loaded..."
echo "----------------------------------------"
lsmod | grep -i bcm2835 || echo "⚠️  bcm2835-v4l2 module not loaded"
echo ""

echo "5. Testing camera capture..."
echo "----------------------------------------"
if command -v raspistill &> /dev/null; then
    echo "Attempting test capture..."
    timeout 5s raspistill -t 1 -o /tmp/camera_test.jpg 2>&1
    if [ $? -eq 0 ]; then
        echo "✓ Camera capture successful!"
        ls -lh /tmp/camera_test.jpg
        rm -f /tmp/camera_test.jpg
    else
        echo "✗ Camera capture failed"
    fi
else
    echo "⚠️  raspistill not available (legacy camera tools)"
fi
echo ""

echo "6. Quick fixes to try:"
echo "----------------------------------------"
echo "Fix 1: Release camera resource"
echo "  sudo modprobe -r bcm2835-v4l2"
echo "  sudo modprobe bcm2835-v4l2"
echo ""
echo "Fix 2: Increase GPU memory (requires reboot)"
echo "  echo 'gpu_mem=256' | sudo tee -a /boot/config.txt"
echo "  sudo reboot"
echo ""
echo "Fix 3: Kill any Python processes holding camera"
echo "  sudo pkill -9 python"
echo ""
echo "========================================"
