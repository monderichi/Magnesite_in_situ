import time
import glob
from pymycobot import MyCobot320

def test_ports():
    ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    if not ports:
        print("❌ No serial ports found! Is the robot plugged in?")
        return

    print(f"🔍 Found ports: {ports}")
    for port in ports:
        print(f"\n======================================")
        print(f"Testing port: {port}")
        for baud in [115200, 1000000]:
            print(f"  Attempting connection at {baud} baud...")
            try:
                mc = MyCobot320(port, baud)
                time.sleep(1)
                angles = mc.get_angles()
                
                # pymycobot returns -1 if device is silent/timing out
                if angles == -1:
                    print(f"  ✗ Connection opened, but device is SILENT (returned -1).")
                    print(f"    -> If this is the robot, the screen MUST be set to 'Transponder' mode!")
                    print(f"    -> If this is the Spray Valve Arduino, it correctly ignored the myCobot commands.")
                    continue
                    
                if isinstance(angles, list) and len(angles) >= 6:
                    print(f"  ✅ SUCCESS! Robot found on {port} at {baud} baud!")
                    print(f"  ✅ Current Angles: {angles}")
                    return
                else:
                    print(f"  ✗ Connected, but gave weird response: {angles}")
            except Exception as e:
                print(f"  ✗ Exception: {e}")
                
    print("\n======================================")
    print("❌ Could not get angles from any port.")
    print("Please physically verify:")
    print("1. The myCobot USB is actually plugged in into the computer.")
    print("2. The myCobot M5 screen is ON and set to 'Transponder' or 'Mac' -> 'USB UART' mode.")

if __name__ == '__main__':
    test_ports()
