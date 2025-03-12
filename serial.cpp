#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

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

#define SERIAL_PORT "/dev/ttyUSB0"  // Change this to match your device
#define DATA_LENGTH 16              // Fixed length of incoming data
#define DATA "123456789asdcxhe"              // Fixed length of incoming data

// #if 0
class Serial{
public:
    int serialSetup();
    void readSerial();
    void writeSerial();
    void closeSerial();

private:
    int serial_fd;
    // char* buffer;
};

int Serial::serialSetup() {
    std::cout << "before open port " << std::endl;
    serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_SYNC);
    std::cout << "successfully opened port" << std::endl;
    if (serial_fd < 0) {
        std::cerr << "Error opening serial port: " << strerror(errno) << std::endl;
        return 1;
    }
    std::cout << "successfully opened serial_port " << std::endl;

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial_fd, &tty) != 0) {
        std::cerr << "Error getting terminal attributes: " << strerror(errno) << std::endl;
        close(serial_fd);
        return 1;
    }
    std::cout << "successfully got terminal attributes " << std::endl;

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
    std::cout << "successfully set terminal attributes " << std::endl;


    return 0;
}
void Serial::readSerial() {
    std::cout << "beginning of readSerial" << std::endl;
    // Read fixed-length data
    char buffer[DATA_LENGTH + 1] = {0}; // +1 for null termination
    std::cout << "readSerial after buffer" << std::endl;
    int bytes_read = read(serial_fd, buffer, DATA_LENGTH);
    if (bytes_read > 0) {
        buffer[bytes_read] = '\0'; // Ensure null termination
        std::cout << "Received data: " << buffer << std::endl;
    } else {
        std::cerr << "Error reading from serial port: " << strerror(errno) << std::endl;
    }
    std::cout << "end of readSerial" << std::endl;
}

void Serial::writeSerial() {
    // Read fixed-length data
    // buffer[DATA_LENGTH + 1] = {0}; // +1 for null termination
    // int bytes_read = read(serial_fd, buffer, DATA_LENGTH);
    // ssize_t	 write(int __fd, const void * __buf, size_t __nbyte) __DARWIN_ALIAS_C(write);
    int bytes_written = write(serial_fd, DATA, DATA_LENGTH);
    if (bytes_written > 0) {
        std::cout << "Aparently we were successful sending shit " << std::endl;
    } else {
        std::cerr << "Error writing to serial port: " << strerror(errno) << std::endl;
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