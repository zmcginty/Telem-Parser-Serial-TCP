import serial
import time
import struct
import socket
import threading

# Define serial port and baud rate
SERIAL_PORT = "/dev/ttyUSB0"  # Adjust for your OS (Windows: "COM3", Linux/Mac: "/dev/ttyUSB0")
BAUD_RATE = 921600
# Definition of packet structure
PACKET_FORMAT = "BBBBIfff"  # (uint8, uint8, uint8, uint32, float, float, float)
# Broadcast listening port
BROADCAST_PORT = 5000

# packet struct data members
frame_start_7F = 0x7F         # 1 byte (uint8)
frame_start_F0 = 0xF0         # 1 byte (uint8)
frame_start_1C = 0x1C         # 1 byte (uint8)
frame_start_AF = 0xAF         # 1 byte (uint8)
packet_count = 0x00000000 # 4 bytes (uint32)
gyro_x_rate = 01.00       # 4 bytes (float)
gyro_y_rate = 02.00       # 4 bytes (float)
gyro_z_rate = 03.00       # 4 bytes (float)

# Pack the data into a 16-byte binary packet
packet = struct.pack(PACKET_FORMAT, frame_start_7F, frame_start_F0, frame_start_1C, frame_start_AF, packet_count, gyro_x_rate, gyro_y_rate, gyro_z_rate)

# Print raw binary packet
print(f"Packed Packet: {packet.hex()}")  # Display as hex

# Unpack the packet to verify
unpacked_data = struct.unpack(PACKET_FORMAT, packet)

# Display unpacked values
print(f"Unpacked Data: {unpacked_data}")

def send_serial_packet():
    try:
        # Open the serial port
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

        time.sleep(2)
        while True:
            # Write to serial port
            ser.write(packet)  # Convert string to bytes and send
            # print(f"Sent: {packet}")
            time.sleep(0.001)

    except serial.SerialException as e:
        print(f"Error: {e}")

    except KeyboardInterrupt:
        print("\nStopping serial communication...")

    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

# def setup_serial_port():
#     try:
#         # Open the serial port
#         ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
#     except serial.SerialException as e:
#         print(f"Error: {e}")


def listen_for_broadcast():
    while True:
        # Create a UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Allow multiple programs to use the same address/port
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Bind to all interfaces on the given port
        sock.bind(("", BROADCAST_PORT))

        print(f"Listening for UDP broadcasts on port {BROADCAST_PORT}...")

        while True:
            data, addr = sock.recvfrom(1024)  # Receive up to 1024 bytes
            # print(f"Received from {addr}: {data.decode().strip()}")
            print(f"Received from {addr}: {data}")

# Thread for sending serial packets
serial_packet_thread = threading.Thread(target=send_serial_packet, daemon=True)
serial_packet_thread.start()

# Thread for listening for network/UDP packets
listener_thread = threading.Thread(target=listen_for_broadcast, daemon=True)
listener_thread.start()

# Main program continues running while listening in the background
print("[Main] UDP Listener running in the background. Press Enter to exit...")
input()  # Keeps the main thread alive until Enter is pressed

