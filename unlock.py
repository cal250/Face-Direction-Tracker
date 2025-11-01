import serial
import time

print("Testing COM10... (will retry 5 times)")

for i in range(5):
    try:
        ser = serial.Serial("COM10", 9600, timeout=1)
        time.sleep(2)
        print("COM10 IS FREE AND WORKING!")
        ser.write(b"P0,T0\n")
        ser.close()
        break
    except Exception as e:
        print(f"Attempt {i+1}: {e}")
        if "Access is denied" in str(e):
            print("   PORT LOCKED! Close EVERYTHING using COM10.")
        time.sleep(2)
else:
    print("\nFAILED AFTER 5 TRIES")
    print("DO THIS NOW:")
    print("   1. Open Resource Monitor")
    print("   2. Search 'COM10'")
    print("   3. Kill any process")
    print("   4. Unplug & replug Arduino")
    input("   Press Enter to exit...")
    exit()