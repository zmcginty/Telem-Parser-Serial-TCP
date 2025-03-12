#include <iostream>
#include <thread>
#include <atomic>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>

#define SERIAL_PORT "/dev/ttyUSB0"  // Adjust to your actual serial port
#define DATA_LENGTH 16              // Minimum bytes to read

std::atomic<bool> keepRunning(true); // Atomic flag to control the thread

void readSerialData() {
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

int main() {
    std::thread serialThread(readSerialData);

    std::cout << "Press Enter to stop...\n";
    std::cin.get();  // Wait for user input

    keepRunning = false; // Stop the serial thread
    serialThread.join(); // Wait for the thread to finish

    std::cout << "Serial reader stopped.\n";
    return 0;
}
