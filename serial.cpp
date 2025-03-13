#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include <array>
#include <vector>

#define NON_BLOCKING 0  // Change this to match your device
//for non-blocking threaded serial:
#if 0
#include <iostream>
#include <thread>
#include <atomic>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
std::atomic<bool> keepRunning(true); // Atomic flag to control the thread
#endif

#define SERIAL_PORT "/dev/ttyUSB1"
#define DATA_LENGTH 16
#define DATA "123456789asdcxhe"
#define DATA2 "FFFFFFFFFFFFFFFF"

// #if 0
class Serial{
public:
    int serialSetup();
    std::vector<uint8_t> readSerial();
    int writeSerial(std::vector<uint8_t> writeData);
    void closeSerial();

private:
    int serial_fd;
    //Do I want packet_count and num_missed_packets HERE?????
    // uint32_t packet_count;
    // size_t num_missed_packets = 0;
    // Also, do I want to store a serial buffer????
};

int Serial::serialSetup() {
    // std::cout << "before open port " << std::endl;
    serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_SYNC);
    // std::cout << "successfully opened port" << std::endl;
    if (serial_fd < 0) {
        std::cerr << "Error opening serial port: " << strerror(errno) << std::endl;
        return 1;
    }
    // std::cout << "successfully opened serial_port " << std::endl;

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial_fd, &tty) != 0) {
        std::cerr << "Error getting terminal attributes: " << strerror(errno) << std::endl;
        close(serial_fd);
        return 1;
    }
    // std::cout << "successfully got terminal attributes " << std::endl;

    // Configure serial port settings
    cfsetospeed(&tty, B9600);  // Set baud rate to 9600
    cfsetispeed(&tty, B9600);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
    tty.c_iflag &= ~IGNBRK;  // Disable break processing
    tty.c_lflag = 0;         // No signaling characters, no echo, no canonical processing
    tty.c_oflag = 0;         // No remapping, no delays
    tty.c_cc[VMIN]  = DATA_LENGTH; // Read exactly DATA_LENGTH bytes
    tty.c_cc[VTIME] = 10;    // Timeout (in tenths of a second)

    // Apply settings
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting terminal attributes: " << strerror(errno) << std::endl;
        close(serial_fd);
        return 1;
    }
    // std::cout << "successfully set terminal attributes " << std::endl;


    return 0;
}

//MAYBE want to change read function to read one byte at a time..... to look for frame_start byte(s) or read 4 bytes at a time so I can look for all 4 frame_start bytes.
// Function to read from serial (single threaded, blocking... need to make threaded eventually)
std::vector<uint8_t> Serial::readSerial() {
    // std::cout << "readSerial start\n";
    // Read fixed-length data
    std::vector<uint8_t> packet_buffer;
    // char* byte_buffer;
    uint8_t byte_buffer[1];
    // int bytes_read = read(serial_fd, packet_buffer, DATA_LENGTH);
    // How do you consume a whole packet??? One byte at a time.
    int bytes_read = read(serial_fd, byte_buffer, 1);
    // std::cout << "readSerial read first byte\n";
    // I think I'll need a way to check when the next byte is available....?? I'm doing it live here...
    // Here we're looking for the first frame_start byte. Once we get it we'll add it to the packet_buffer which we'll de-serialize
    while(bytes_read > 0 && byte_buffer[0] != 0x7F) {
        // std::cout << "bytes read > 0, and byte != 0x7F\n";
        int bytes_read = read(serial_fd, byte_buffer, 1);   //This SHOULD read the NEXT byte
    }
    packet_buffer.push_back(byte_buffer[0]);
    // If we get here, we SHOULD be at the start of a frame/packet.
    for(size_t i=0; i<DATA_LENGTH-1; i++) {
        int bytes_read = read(serial_fd, byte_buffer, 1);   //This SHOULD read the NEXT byte
        if(bytes_read <= 0) {
            std::cerr << "Error reading from serial port: " << strerror(errno) << std::endl;
        }
        packet_buffer.push_back(byte_buffer[0]);
    }
    return packet_buffer;
    // if (bytes_read > 0) {
    //     // packet_buffer[bytes_read] = '\0'; // Ensure null termination
    //     // std::cout << "Received data: " << packet_buffer << std::endl;
    //     // I KNOW; probably not the most efficient way of doing it... but it works for now.
    //     // std::array<uint8_t, 16> packet;
    //     // std::memcpy(packet.data(), packet_buffer, 16);
    //     return packet;
    // } else {
    //     std::cerr << "Error reading from serial port: " << strerror(errno) << std::endl;
    // }
}

int Serial::writeSerial(std::vector<uint8_t> writeData) {
    int bytes_written = write(serial_fd, writeData.data(), writeData.size());
    if (bytes_written <= 0) {
        std::cerr << "Error writing to serial port: " << strerror(errno) << std::endl;
        return -1;
    } else {
        // std::cout << "Aparently we were successful sending shit " << std::endl;
        return bytes_written;
    }
}

void Serial::closeSerial() {
    close(serial_fd);
}
// #endif

#if 0
// NON-Blocking Threaded SerialRead
void Serial::readSerial() {
    int serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        std::cerr << "Error opening serial port: " << strerror(errno) << std::endl;
        return;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial_fd, &tty) != 0) {
        std::cerr << "Error getting terminal attributes: " << strerror(errno) << std::endl;
        close(serial_fd);
        return;
    }

    // Configure serial settings
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;   // Don't block
    tty.c_cc[VTIME] = 0;   // No timeout

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting terminal attributes: " << strerror(errno) << std::endl;
        close(serial_fd);
        return;
    }

    while (keepRunning) {
        int bytes_available = 0;

        // Check available bytes
        if (ioctl(serial_fd, FIONREAD, &bytes_available) < 0) {
            std::cerr << "Error checking available bytes: " << strerror(errno) << std::endl;
            break;
        }

        if (bytes_available >= DATA_LENGTH) {
            char buffer[DATA_LENGTH + 1] = {0};
            int bytes_read = read(serial_fd, buffer, DATA_LENGTH);

            if (bytes_read > 0) {
                buffer[bytes_read] = '\0'; // Null terminate for safety
                std::cout << "Received: " << buffer << std::endl;
            } else {
                std::cerr << "Error reading from serial port: " << strerror(errno) << std::endl;
            }
        }

        usleep(10000); // Sleep 10ms to prevent CPU overuse
    }

    close(serial_fd);
}
#endif

// Boost ASIO
//////////////////////////////////////////////////////////////////////////////////
// int main() {
//     boost::asio::io_service io;
//     boost::asio::serial_port serial(io, "/dev/ttyUSB0");

//     // Set parameters
//     serial.set_option(boost::asio::serial_port_base::baud_rate(9600));
//     serial.set_option(boost::asio::serial_port_base::character_size(8));
//     serial.set_option(boost::asio::serial_port_base::stop_bits(
//         boost::asio::serial_port_base::stop_bits::one));
//     serial.set_option(boost::asio::serial_port_base::flow_control(
//         boost::asio::serial_port_base::flow_control::none));

//     char data[16];
//     boost::asio::read(serial, boost::asio::buffer(data, 16));
//     std::cout << "Received: " << data << std::endl;

//     return 0;
// }

// LibSerial/SerialPort.h
////////////////////////////////////////////////////////////////////////////////////
// class Serial{
// public:
//     int serialSetup();
//     void serialReceive();
//     void serialClose();

// private:
//     LibSerial::SerialPort serial;
//     char buffer[DATA_LENGTH];
// };

// int Serial::serialSetup() {
//     serial.Open("SERIAL_PORT");
    
//     // Set port settings
//     serial.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
//     serial.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
//     serial.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
//     serial.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    
//     return 0;
// }
    
// void Serial::serialReceive() {
//     serial.Read(buffer, 16, 1000);  // Read 16 bytes with a timeout
//     std::cout << "Received: " << buffer << std::endl;
// }

// void Serial::serialClose() {
//     serial.Close();
// }