#include <iostream>
#include "serial.cpp"
#include "network.cpp"
#include <cstdint>
#include <iomanip>

struct SerialPacketStructure {
    uint8_t frame_start_7F;
    uint8_t frame_start_F0;
    uint8_t frame_start_1C;
    uint8_t frame_start_AF;
    uint32_t packet_count;
    float gyro_x_rate;
    float gyro_y_rate;
    float gyro_z_rate;
};

int main() {
    Network networkPort;
    Serial serialPort;

    uint32_t packet_count;
    size_t num_missed_packets = 0;
    
    //setup network port
    networkPort.networkSetup();
    
    //setup serial port
    serialPort.serialSetup();
   
    while(1){
        //read from serial port
        std::vector<uint8_t> packet = serialPort.readSerial();
        
        //testing network broadcast
        // #define MESSAGE "BOOOOOOOOOOOOP" 
        // networkPort.networkBroadcastMessage(MESSAGE, strlen(MESSAGE));
        
        // print out raw packet data in hex which we received.
        std::cout << "packet_array data = ";
        for(uint8_t byte : packet) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
            // std::cout << std::hex << static_cast<int>(byte) << " ";
        }
        std::cout << std::dec << std::endl;

        // Time to unpack && parse packet
        //      probs want to put it into some kind of buffer/queue or something since the networking stuff might be running in its own thread.
        //      Should I create some deserialize packed function? or just go through an chew bytes until I catch a frame_start???
        SerialPacketStructure serPkt;
        if((packet[0] == 0x7F) && (packet[1] == 0xF0) && (packet[2] == 0x1C) && (packet[3] == 0xAF)){
            //Frame Starts are all correct!
            serPkt.frame_start_7F = packet[0];
            serPkt.frame_start_F0 = packet[1];
            serPkt.frame_start_1C = packet[2];
            serPkt.frame_start_AF = packet[3];
            serPkt.packet_count = (packet[4] & packet[5] & packet[6] & packet[7]);
            serPkt.gyro_x_rate = (packet[8] & packet[9] & packet[10] & packet[11]);
            serPkt.gyro_y_rate = (packet[12] & packet[13] & packet[14] & packet[15]);
            serPkt.gyro_x_rate = (packet[16] & packet[17] & packet[18] & packet[19]);
            //if this is our first packet, lets set our internal packet_count equal to the one we're sent. Then we'll track missed packets from there.
            if(packet_count == 0){
                packet_count = serPkt.packet_count;
                packet_count++;     //This is sus... though I feel like I must do this for some reason...
            }
            // if it's not our first packet, let's increment our internal packet count then see if it's equal to the packet count we're sent.
            else {
                packet_count++;
                if(packet_count != serPkt.packet_count){
                    //we've missed a packet
                    std::cout << "our packet_count = " << packet_count << std::endl;
                    std::cout << "received packet_count = " << serPkt.packet_count << std::endl;
                    //maybe do.... something.... like restart what could have caused it (ie. have power controller cycle IMU... to start extreme)
                }
            }
        }
        //Probs want to have these network broadcasts run in their own thread so they don't hang the parsing.
        networkPort.networkBroadcastMessage(&serPkt.packet_count, 4);
        networkPort.networkBroadcastMessage(&serPkt.gyro_x_rate, 4);
        networkPort.networkBroadcastMessage(&serPkt.gyro_y_rate, 4);
        networkPort.networkBroadcastMessage(&serPkt.gyro_z_rate, 4);
    }
    return 0;
}
