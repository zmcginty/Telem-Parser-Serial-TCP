#include <iostream>
#include "serial.cpp"
#include "network.cpp"

int main() {
    Network networkPort;
    Serial serialPort;

    //Serial-ey stuffs
    std::cout << "made serial and network objs " << std::endl;
    serialPort.serialSetup();
    std::cout << "after serial port setup " << std::endl;
    serialPort.writeSerial(); 
    std::cout << "after serial write " << std::endl;
    serialPort.readSerial();

    //Network-ey stuffs
    // networkPort.networkSetup();
    // networkPort.networkBroadcastMessage();
    // networkPort.networkClose();
    return 0;
}
