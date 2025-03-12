import serial
import time

# Define serial port and baud rate
SERIAL_PORT = "/dev/tty.usbserial-2140"  # Adjust for your OS (Windows: "COM3", Linux/Mac: "/dev/ttyUSB0")
BAUD_RATE = 9600

try:
    # Open the serial port
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    
    # Wait for the port to initialize
    time.sleep(2)  # Give some time for the serial connection to establish

    while True:
        # Data to send
        data = "Hello, Serial!\n"

        # Write to serial port
        ser.write(data.encode())  # Convert string to bytes and send
        print(f"Sent: {data.strip()}")

        # Wait before sending the next message
        time.sleep(1)

except serial.SerialException as e:
    print(f"Error: {e}")

except KeyboardInterrupt:
    print("\nStopping serial communication...")

finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed.")