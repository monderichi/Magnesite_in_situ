#!/bin/bash
# RealSense D456 USB Diagnostics Script

echo "========================================="
echo "RealSense D456 USB Diagnostics"
echo "========================================="
echo ""

# Check USB devices
echo "[1] USB Devices (RealSense):"
lsusb | grep -i "intel\|real\|8086" || echo "   No Intel/RealSense devices found"
echo ""

# Check USB speed
echo "[2] USB Connection Speed:"
lsusb -t | grep -A2 -B2 "8086" || echo "   Cannot determine USB tree"
echo ""

# Check video devices
echo "[3] Video Devices:"
ls -la /dev/video* 2>/dev/null | head -20 || echo "   No video devices found"
echo ""

# Check USB power management
echo "[4] USB Power Management:"
for device in /sys/bus/usb/devices/*/idVendor; do
    if [ -f "$device" ] && grep -q "8086" "$device" 2>/dev/null; then
        usb_path=$(dirname "$device")
        echo "   Device: $(basename $usb_path)"
        echo "   Power Control: $(cat $usb_path/power/control 2>/dev/null || echo 'N/A')"
        echo "   Power Runtime Status: $(cat $usb_path/power/runtime_status 2>/dev/null || echo 'N/A')"
        cat $usb_path/uevent 2>/dev/null | grep -E "(PRODUCT|DEVTYPE)" | head -2
        echo ""
    fi
done

# Check udev rules
echo "[5] Udev Rules:"
if [ -f /etc/udev/rules.d/99-realsense-libusb.rules ]; then
    echo "   ✓ RealSense udev rules installed"
    cat /etc/udev/rules.d/99-realsense-libusb.rules 2>/dev/null | head -5
else
    echo "   ✗ RealSense udev rules NOT found"
    echo "     Install with: sudo apt install ros-humble-realsense2-camera"
fi
echo ""

# Check USB bandwidth usage
echo "[6] USB Bandwidth (other devices on same bus):"
for bus in $(lsusb | grep "8086" | awk '{print $2}' | sort -u); do
    echo "   Bus $bus devices:"
    lsusb -s $bus: | head -5
done
echo ""

# Test camera with rs-enumerate-devices if available
echo "[7] RealSense SDK Test (if installed):"
if command -v rs-enumerate-devices &> /dev/null; then
    rs-enumerate-devices 2>&1 | head -30
else
    echo "   rs-enumerate-devices not found"
    echo "   Install with: sudo apt install librealsense2-utils"
fi
echo ""

# Check kernel modules
echo "[8] Kernel Modules:"
lsmod | grep -E "uvcvideo|videodev|usbcore" | head -5
echo ""

# Check dmesg for USB errors
echo "[9] Recent USB Errors (dmesg):"
dmesg | grep -iE "usb|real|uvc|video" | tail -20
echo ""

echo "========================================="
echo "Diagnostics Complete"
echo "========================================="
echo ""
echo "Common fixes for VIDIOC_QBUF errors:"
echo "  1. Use a different USB 3.0 port (try all available ports)"
echo "  2. Use a powered USB 3.0 hub between PC and camera"
echo "  3. Replace USB cable with shorter/better quality one"
echo "  4. Disable USB autosuspend:"
echo "     echo 'on' | sudo tee /sys/bus/usb/devices/XXXX/power/control"
echo "  5. Update RealSense firmware using RealSense Viewer"
echo ""
