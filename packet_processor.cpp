#include "packet.h"
#include <vector>

class PacketProcessor {
public:
    static Packet parse(const std::vector<uint8_t>& rawPacket);

private:
};

Packet PacketProcessor::parse(const std::vector<uint8_t>& rawPacket) {
    Packet pkt;
    if((rawPacket[0] == 0x7F) && (rawPacket[1] == 0xF0) && (rawPacket[2] == 0x1C) && (rawPacket[3] == 0xAF)){
        //Frame Starts are all correct!
        std::memcpy(&pkt.packet_count, &rawPacket[4], sizeof(uint32_t));
        std::memcpy(&pkt.gyro_x_rate, &rawPacket[8], sizeof(float));
        std::memcpy(&pkt.gyro_y_rate, &rawPacket[12], sizeof(float));
        std::memcpy(&pkt.gyro_z_rate, &rawPacket[16], sizeof(float));
        return pkt;        
    }
    //Not sure what to do here....
    return pkt;
}