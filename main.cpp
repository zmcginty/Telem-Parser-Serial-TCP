#include <iostream>
#include "serial.cpp"
#include "network.cpp"
// #include "packet.h"
#include "packet_processor.cpp"
#include <cstdint>
#include <iomanip>


int main() {
    Network networkPort;
    Serial serialPort;

    uint32_t packet_count;
    size_t num_missed_packets = 0;
    
    //setup and start serial port
    serialPort.start();
    
    //setup network port
    networkPort.networkSetup();
    
    while(1) {
        std::vector<uint8_t> rawPacket;
        if(serialPort.getPacket(rawPacket)) {
            Packet parsedPacket = PacketProcessor::parse(rawPacket);
            std::cout << "packet_array data = ";
            for(uint8_t byte : rawPacket) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
                // std::cout << std::hex << static_cast<int>(byte) << " ";
            }
            std::cout << std::dec << std::endl;
            // std::cout << "PARSED packet_array data = ";
            // for(uint8_t byte : parsedPacket) {
            //     std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
            //     // std::cout << std::hex << static_cast<int>(byte) << " ";
            // }
            // std::cout << std::dec << std::endl;
            std::cout << "PARSED packet_array data \n";
            std::cout << "packet_count = " << parsedPacket.packet_count << std::endl;
            std::cout << "gyro_x_rate = " << parsedPacket.gyro_x_rate << std::endl;
            std::cout << "gyro_y_rate = " << parsedPacket.gyro_y_rate << std::endl;
            std::cout << "gyro_z_rate = " << parsedPacket.gyro_z_rate << std::endl;

            
            networkPort.networkBroadcastMessage(&parsedPacket.packet_count, sizeof(uint32_t));
            networkPort.networkBroadcastMessage(&parsedPacket.gyro_x_rate, sizeof(float));
            networkPort.networkBroadcastMessage(&parsedPacket.gyro_y_rate, sizeof(float));
            networkPort.networkBroadcastMessage(&parsedPacket.gyro_z_rate, sizeof(float));
        }
    }
    serialPort.stop();
    return 0;
}
   
    // while(1){
    //     //read from serial port
    //     std::vector<uint8_t> packet = serialPort.readSerial();
        
    //     // print out raw packet data in hex which we received.
    //     std::cout << "packet_array data = ";
    //     for(uint8_t byte : packet) {
    //         std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    //         // std::cout << std::hex << static_cast<int>(byte) << " ";
    //     }
    //     std::cout << std::dec << std::endl;

    //     // Time to unpack && parse packet
    //     //      probs want to put it into some kind of buffer/queue or something since the networking stuff might be running in its own thread.
    //     //      Should I create some deserialize packed function? or just go through an chew bytes until I catch a frame_start???
    //     SerialPacketStructure serPkt;
    //     if((packet[0] == 0x7F) && (packet[1] == 0xF0) && (packet[2] == 0x1C) && (packet[3] == 0xAF)){
    //         //Frame Starts are all correct!
    //         // serPkt.frame_start_7F = packet[0];
    //         // serPkt.frame_start_F0 = packet[1];
    //         // serPkt.frame_start_1C = packet[2];
    //         // serPkt.frame_start_AF = packet[3];
    //         serPkt.packet_count = (packet[4] & packet[5] & packet[6] & packet[7]);
    //         serPkt.gyro_x_rate = (packet[8] & packet[9] & packet[10] & packet[11]);
    //         serPkt.gyro_y_rate = (packet[12] & packet[13] & packet[14] & packet[15]);
    //         serPkt.gyro_x_rate = (packet[16] & packet[17] & packet[18] & packet[19]);
    //         //if this is our first packet, lets set our internal packet_count equal to the one we're sent. Then we'll track missed packets from there.
    //         if(packet_count == 0){
    //             packet_count = serPkt.packet_count;
    //             packet_count++;     //This is sus... though I feel like I must do this for some reason...
    //         }
    //         // if it's not our first packet, let's increment our internal packet count then see if it's equal to the packet count we're sent.
    //         else {
    //             packet_count++;
    //             if(packet_count != serPkt.packet_count){
    //                 //we've missed a packet
    //                 std::cout << "our packet_count = " << packet_count << std::endl;
    //                 std::cout << "received packet_count = " << serPkt.packet_count << std::endl;
    //                 //maybe do.... something.... like restart what could have caused it (ie. have power controller cycle IMU... to start extreme)
    //             }
    //         }
    //     }
        //Probs want to have these network broadcasts run in their own thread so they don't hang the parsing.
        // networkPort.networkBroadcastMessage(&serPkt.packet_count, 4);
