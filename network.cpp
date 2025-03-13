#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#define BROADCAST_IP "10.0.0.255" // broadcast address 
#define BROADCAST_PORT 5000            // broadcast port 

class Network{
public:
    int networkSetup();
    int networkBroadcastMessage(const void* buffer, size_t buffer_len);
    // int networkBroadcastMessage();
    void networkClose();

private:
    int sockfd;
    struct sockaddr_in broadcastAddr;
};

int Network::networkSetup() {
    int broadcastEnable = 1;

    // Create UDP socket
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Error creating socket!" << std::endl;
        return 1;
    }

    // Enable broadcast mode
    if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) < 0) {
        std::cerr << "Error setting broadcast option!" << std::endl;
        close(sockfd);
        return 1;
    }

    // Configure broadcast address
    memset(&broadcastAddr, 0, sizeof(broadcastAddr));
    broadcastAddr.sin_family = AF_INET;
    broadcastAddr.sin_port = htons(BROADCAST_PORT);
    broadcastAddr.sin_addr.s_addr = inet_addr(BROADCAST_IP);

    // close(sockfd);
    return 0;
}

int Network::networkBroadcastMessage(const void* buffer, size_t buffer_len) {
    // Send broadcast message
    if (sendto(sockfd, buffer, buffer_len, 0, 
               (struct sockaddr*)&broadcastAddr, sizeof(broadcastAddr)) < 0) {
        std::cerr << "Error sending broadcast message!" << std::endl;
        close(sockfd);
        return 1;
    }
    return 0;
}

void Network::networkClose() {
    close(sockfd);
}