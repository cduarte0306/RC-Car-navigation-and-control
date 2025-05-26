#include "udp_socket.hpp"
#include "sockets.hpp"
#include <functional>
#include <regex>
#include <sys/socket.h>
#include <netinet/in.h>
#include <thread>
#include <unistd.h>

using namespace Network;


UDPSocket::UDPSocket(int sPort, int dPort) {
    // Initialize inherited variables
    sport_ = sPort;
    dport_ = dPort;

    // Create a UDP socket
    socketFD_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketFD_ < 0) {
        throw(0);
    }

    // Set up the server address
    struct sockaddr_in serverAddress;
    memset((void*)&serverAddress, 0, sizeof(serverAddress));
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = INADDR_ANY;
    serverAddress.sin_port = htons(sPort);

    // Bind the socket to the server address
    if (bind(socketFD_, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0) {
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
            // Fire the signal to notify data has been received
            onDataReceived(reinterpret_cast<const uint8_t*>(buffer), static_cast<size_t>(bytesRead));  // Emit the data received signal
        }
    }
}