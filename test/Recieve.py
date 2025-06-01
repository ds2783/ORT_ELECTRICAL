import serial
import time

# Replace with your Pico's serial port:
# Windows: 'COM3', 'COM4', etc.
# macOS/Linux: '/dev/ttyACM0' or '/dev/ttyUSB0'
SERIAL_PORT = '/dev/ttyACM0'  # <-- Change this!
BAUDRATE = 115200

# Open serial port
with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
    time.sleep(2)  # Wait for Pico to reboot and start sending
    print("Reading distance values...\n")

    while True:
        line = ser.readline().decode('utf-8').strip()
        if line:
            try:
                distance = int(line)
                print(f"Distance: {distance} cm")
            except ValueError:
                print(f"Invalid data: {line}")
