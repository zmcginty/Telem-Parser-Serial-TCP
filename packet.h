#include <cstdint>
#include <vector>

struct Packet {
    //taking these out of pkt structure
    // uint8_t frame_start_7F;
    // uint8_t frame_start_F0;
    // uint8_t frame_start_1C;
    // uint8_t frame_start_AF;
    uint32_t packet_count;
    float gyro_x_rate;
    float gyro_y_rate;
    float gyro_z_rate;
    //do I need packetsize here??? for network???
    // uint8_t packetSize = 20;    //size with frame_starts
};