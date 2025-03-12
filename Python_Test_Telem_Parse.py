import serial
import time
import struct

# Define serial port and baud rate
SERIAL_PORT = "/dev/tty.usbserial-2140"  # Adjust for your OS (Windows: "COM3", Linux/Mac: "/dev/ttyUSB0")
BAUD_RATE = 9600
# Definition of packet structure
PACKET_FORMAT = "BBBBIfff"  # (uint8, uint8, uint8, uint32, float, float, float)

# Example data
frame_start_7F = 0x7F         # 1 byte (uint8)
frame_start_F0 = 0xF0         # 1 byte (uint8)
frame_start_1C = 0x1C         # 1 byte (uint8)
frame_start_AF = 0xAF         # 1 byte (uint8)
packet_count = 0x00000000 # 4 bytes (uint32)
gyro_x_rate = 12.34       # 4 bytes (float)
gyro_y_rate = 56.78       # 4 bytes (float)
gyro_z_rate = 41.20       # 4 bytes (float)

# Pack the data into a 16-byte binary packet
packet = struct.pack(PACKET_FORMAT, frame_start_7F, frame_start_F0, frame_start_1C, frame_start_AF, packet_count, gyro_x_rate, gyro_y_rate, gyro_z_rate)

# Print raw binary packet
print(f"Packed Packet: {packet.hex()}")  # Display as hex

# Unpack the packet to verify
unpacked_data = struct.unpack(PACKET_FORMAT, packet)

# Display unpacked values
print(f"Unpacked Data: {unpacked_data}")

try:
    # Open the serial port
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    
    # Wait for the port to initialize
    time.sleep(1)  # Give some time for the serial connection to establish

    while True:
        # Data to send
        # data = "Hello, Serial!\n"

        # Write to serial port
        # ser.write(data.encode())  # Convert string to bytes and send
        ser.write(packet)  # Convert string to bytes and send
        print(f"Sent: {packet}")

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