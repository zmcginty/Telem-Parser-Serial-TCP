#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include <array>
#include <vector>
#include <thread>
#include <atomic>
#include <sys/ioctl.h>
#include <chrono>
#include <queue>
#include <mutex>
#include <condition_variable>

#define SERIAL_PORT "/dev/ttyUSB1"

class Serial{
public:
    Serial() {
        if(serialSetup() == 1){
            //serial_setup failed
        }
        running = false;
    }
    ~Serial() {
        if(serial_fd >=0){
            stop();
            closeSerial();
        }
    }
    void start() {
        running = true;
        std::thread(&Serial::serialThread, this).detach();
    }
    void stop() {
        running = true;
    }
    bool getPacket(std::vector<uint8_t> &packet);

    // std::vector<uint8_t> readSerial();
    int writeSerial(std::vector<uint8_t> writeData);
    void closeSerial();

private:
    size_t packetSize = 20;
    const uint8_t startOfFrame = 0x7F;
    int serialSetup();
    void serialThread();
    int serial_fd;
    std::queue<std::vector<uint8_t>> packetQueue;
    std::mutex queueMutex;
    std::condition_variable queueCondVar;
    std::atomic<bool> running; // for thread control
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
    cfsetospeed(&tty, B921600);  // Set baud rate to 9600
    cfsetispeed(&tty, B921600);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
    tty.c_iflag &= ~IGNBRK;  // Disable break processing
    tty.c_lflag = 0;         // No signaling characters, no echo, no canonical processing
    tty.c_oflag = 0;         // No remapping, no delays
    tty.c_cc[VMIN]  = packetSize; // Read exactly DATA_LENGTH bytes
    tty.c_cc[VTIME] = 10;    // Timeout (in tenths of a second)

    // Apply settings
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting terminal attributes: " << strerror(errno) << std::endl;
        close(serial_fd);
        return 1;
    }
    return 0;
}

void Serial::serialThread() {
    std::vector<uint8_t> buffer;
    uint8_t byte;

    if(read(serial_fd, &byte, 1) > 0) {
        if(byte == startOfFrame) {
            buffer.clear();
            buffer.push_back(byte);

            for(size_t i = 1; i< packetSize; ++i) {
                if(read(serial_fd, &byte, 1) > 0) {
                    buffer.push_back(byte);
                }
                else{
                    break;
                }
            }
            if(buffer.size() == packetSize) {
                std::lock_guard<std::mutex> lock(queueMutex);
                packetQueue.push(buffer);
                queueCondVar.notify_one();
            }
        }
    }
}

bool Serial::getPacket(std::vector<uint8_t> &packet) {
    std::unique_lock<std::mutex> lock(queueMutex);
    queueCondVar.wait(lock, [this] { return !packetQueue.empty(); });
    packet = packetQueue.front();
    packetQueue.pop();
    return true;
}

// std::vector<uint8_t> Serial::readSerial() {
//     std::vector<uint8_t> packet_buffer;
//     uint8_t byte_buffer[1];
//     // How do you consume a whole packet??? One byte at a time.
//     while(running){
//         int bytes_read = read(serial_fd, byte_buffer, 1);
//         // I think I'll need a way to check when the next byte is available....?? I'm doing it live here...
//         // Here we're looking for the first frame_start byte. Once we get it we'll add it to the packet_buffer which we'll de-serialize
//         while(bytes_read > 0 && byte_buffer[0] != 0x7F) {
//             // std::cout << "bytes read > 0, and byte != 0x7F\n";
//             int bytes_read = read(serial_fd, byte_buffer, 1);   //This SHOULD read the NEXT byte
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(1));  //NOTE: Adjust this to achieve 80ms inter-packet timing
//     }
//     // push the first frame_start byte we just checked
//     packet_buffer.push_back(byte_buffer[0]);
//     // If we get here, we SHOULD be at the start of a frame/packet. So we'll loop throug the DATA_LENGTH-1 rest of them
//     for(size_t i=0; i<DATA_LENGTH-1; i++) {
//         int bytes_read = read(serial_fd, byte_buffer, 1);   //This SHOULD read the NEXT byte
//         if(bytes_read <= 0) {
//             std::cerr << "Error reading from serial port: " << strerror(errno) << std::endl;
//         }
//         packet_buffer.push_back(byte_buffer[0]);
//     }
//     return packet_buffer;
// }
    

// function to write out to serial
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

// // Function to read from serial (single threaded, blocking... need to make threaded eventually)
// std::vector<uint8_t> Serial::readSerial() {
//     std::vector<uint8_t> packet_buffer;
//     uint8_t byte_buffer[1];
//     // How do you consume a whole packet??? One byte at a time.
//     int bytes_read = read(serial_fd, byte_buffer, 1);
//     // I think I'll need a way to check when the next byte is available....?? I'm doing it live here...
//     // Here we're looking for the first frame_start byte. Once we get it we'll add it to the packet_buffer which we'll de-serialize
//     while(bytes_read > 0 && byte_buffer[0] != 0x7F) {
//         // std::cout << "bytes read > 0, and byte != 0x7F\n";
//         int bytes_read = read(serial_fd, byte_buffer, 1);   //This SHOULD read the NEXT byte
//     }
//     // push the first frame_start byte we just checked
//     packet_buffer.push_back(byte_buffer[0]);
//     // If we get here, we SHOULD be at the start of a frame/packet. So we'll loop throug the DATA_LENGTH-1 rest of them
//     for(size_t i=0; i<DATA_LENGTH-1; i++) {
//         int bytes_read = read(serial_fd, byte_buffer, 1);   //This SHOULD read the NEXT byte
//         if(bytes_read <= 0) {
//             std::cerr << "Error reading from serial port: " << strerror(errno) << std::endl;
//         }
//         packet_buffer.push_back(byte_buffer[0]);
//     }
//     return packet_buffer;
// }

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