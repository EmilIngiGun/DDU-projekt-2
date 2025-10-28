import serial, time

PORT = 'COM6'
BAUD = 115200

with serial.Serial(PORT, BAUD, timeout=1) as s:
    time.sleep(2)
    print(f"Connected to {PORT}")
    while True:
        line = s.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(line)
