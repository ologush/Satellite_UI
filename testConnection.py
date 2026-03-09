import serial, time

try:
    s = serial.Serial('/dev/cu.usbmodem2070316A30311', 115200, timeout=1)
    time.sleep(0.1)
    written = s.write(bytes([0x03]))
    print(f"Wrote {written} bytes to the serial port.")
    s
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit(1)

