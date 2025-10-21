import serial
import time

# --- Adjust these if needed ---
PORT = 'COM3'          # Check in Device Manager (e.g. COM4, COM5, etc.)
BAUD_RATE = 115200
# ------------------------------

def read_serial():
    try:
        # Connect to Arduino
        arduino = serial.Serial(PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Give Arduino time to reset
        print(f"✅ Connected to {PORT} at {BAUD_RATE} baud\n")

        while True:
            line = arduino.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(line)  # Print all serial output
                if "Pressed" in line:
                    print("🔵 Button detected!")

    except serial.SerialException as e:
        print(f"❌ Serial error: {e}")
    except KeyboardInterrupt:
        print("\n🛑 Exiting...")
    finally:
        if 'arduino' in locals() and arduino.is_open:
            arduino.close()
            print("🔌 Serial port closed.")

if __name__ == "__main__":
    read_serial()
