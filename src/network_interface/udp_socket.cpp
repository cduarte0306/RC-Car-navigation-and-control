#include "udp_socket.hpp"
#include <functional>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

using namespace Network;


UDPSocket::UDPSocket(int port) {
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
        // Handle socket binding error
        // You can throw an exception or handle it in any other way
    }
}



UDPSocket::~UDPSocket() {
    // Close the socket
    close(socket_);
}
