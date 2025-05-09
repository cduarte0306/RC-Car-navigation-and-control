#include "udp_socket.hpp"
#include "sockets.hpp"
#include <functional>
#include <regex>
#include <sys/socket.h>
#include <netinet/in.h>
#include <thread>
#include <unistd.h>

using namespace Network;


UDPSocket::UDPSocket(int port){
    // Create a UDP socket
    socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0) {
        throw(0);
    }

    // Set up the server address
    struct sockaddr_in serverAddress;
    memset((void*)&serverAddress, 0, sizeof(serverAddress));
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = INADDR_ANY;
    serverAddress.sin_port = htons(port);

    // Bind the socket to the server address
    if (bind(socket_, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0) {
        throw("Failed to open socket");
    }

    // Start the data transmission thread
    this->transmissionThread = std::thread(&UDPSocket::transmissionThreadHandler, this);
}


UDPSocket::~UDPSocket() {
    // Close the socket
    close(socket_);

    this->threadCanRun = false;
    this->transmissionThread.join();
}


bool UDPSocket::transmit(uint8_t* pBuf, size_t length) {
    return true;
}


void UDPSocket::transmissionThreadHandler(void) {
    char buffer[1500];  // MTU size

    while( this->threadCanRun ) {
        // Receive data from the socket
        struct sockaddr_in clientAddress;
        socklen_t clientAddressLength = sizeof(clientAddress);
        ssize_t bytesRead = recvfrom(socket_, buffer, sizeof(buffer), 0, (struct sockaddr *)&clientAddress, &clientAddressLength);
        
        if (bytesRead > 0) {
            // Process the received data
            // ...
            
            // Call the reception callback
            if (receptionCallback) {
                receptionCallback();
            }
        }

        // Fire the signal notify data has been received
    }
}