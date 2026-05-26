#!/usr/bin/env python3
"""
Diagnostic script for myCobot 320 M5 robot
Tests various communication methods to identify connection issues
"""

import sys
import time

print("=" * 60)
print("myCobot 320 M5 Diagnostic Tool")
print("=" * 60)
print()

# Check pymycobot installation
print("[1/6] Checking pymycobot installation...")
try:
    from pymycobot import MyCobot320
    print("✓ pymycobot is installed")
except ImportError as e:
    print(f"✗ pymycobot not found: {e}")
    print("Install with: pip3 install pymycobot")
    sys.exit(1)

# Check serial port
print("\n[2/6] Checking serial port...")
import os
port = '/dev/ttyACM0'
if os.path.exists(port):
    print(f"✓ {port} exists")
    # Check permissions
    import stat
    st = os.stat(port)
    mode = stat.filemode(st.st_mode)
    print(f"  Permissions: {mode}")
    if os.access(port, os.R_OK) and os.access(port, os.W_OK):
        print(f"✓ Read/Write access OK")
    else:
        print(f"✗ No read/write access - run: sudo chmod 666 {port}")
else:
    print(f"✗ {port} not found")
    print("Available ports:")
    os.system("ls -la /dev/tty* 2>/dev/null | grep -E 'ACM|USB'")
    sys.exit(1)

# Try to connect
print("\n[3/6] Attempting to connect to robot...")
try:
    mc = MyCobot320(port, 115200)
    print(f"✓ Serial connection established")
except Exception as e:
    print(f"✗ Failed to connect: {e}")
    sys.exit(1)

# Wait for initialization
print("\n[4/6] Waiting for robot initialization (3 seconds)...")
time.sleep(3)

# Test basic commands
print("\n[5/6] Testing robot communication...")

tests = [
    ("get_system_version", lambda: mc.get_system_version(), "System version"),
    ("is_power_on", lambda: mc.is_power_on(), "Power status"),
    ("get_angles", lambda: mc.get_angles(), "Joint angles"),
    ("get_coords", lambda: mc.get_coords(), "Coordinates"),
]

results = {}
for name, func, description in tests:
    try:
        result = func()
        results[name] = result
        if result == -1 or result is None:
            print(f"  {description:20s}: ✗ Returned: {result} (No response)")
        else:
            print(f"  {description:20s}: ✓ {result}")
    except Exception as e:
        results[name] = None
        print(f"  {description:20s}: ✗ Error: {e}")

# Diagnosis
print("\n[6/6] Diagnosis:")
print("-" * 60)

all_failed = all(v in [-1, None] for v in results.values())
some_failed = any(v in [-1, None] for v in results.values())

if all_failed:
    print("✗ CRITICAL: Robot is not responding to any commands")
    print()
    print("Possible causes:")
    print("  1. Robot is not powered ON")
    print("  2. Robot firmware is not running/crashed")
    print("  3. Wrong serial port or device")
    print("  4. USB cable issue")
    print("  5. Robot requires initialization via M5 screen")
    print()
    print("Troubleshooting steps:")
    print("  1. Check if robot is powered on (M5 screen should be lit)")
    print("  2. Check M5 screen for firmware status/errors")
    print("  3. Try resetting the robot (power cycle)")
    print("  4. Try different USB port or cable")
    print("  5. Check if robot needs 'Atom' firmware flash")
    print("  6. Try different baud rates: 9600, 57600, 115200, 1000000")
elif some_failed:
    print("⚠ WARNING: Partial communication")
    print(f"Some commands work, but others return -1")
    print("This may indicate:")
    print("  - Robot is partially initialized")
    print("  - Some features disabled in firmware")
    print("  - Robot in special mode")
else:
    print("✓ SUCCESS: Robot is responding normally!")
    print()
    print("The robot driver should work correctly.")

print("-" * 60)
print()

# Additional info
if 'get_angles' in results and results['get_angles'] not in [-1, None]:
    print(f"Current joint angles: {results['get_angles']}")
    
print("\nTo fix permission issues permanently:")
print(f"  sudo usermod -a -G dialout $USER")
print(f"  (then log out and log back in)")
print()
print("=" * 60)
