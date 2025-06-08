#include "udp_socket.hpp"
#include "sockets.hpp"
#include <functional>
#include <ostream>
#include <regex>
#include <sys/socket.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <thread>
#include <unistd.h>
#include <ifaddrs.h>
#include <cstring>
#include <netdb.h>
#include <iostream>


using namespace Network;


UDPSocket::UDPSocket(int sPort, int dPort):Sockets() {
    sport_ = sPort;
    dport_ = dPort;

    // Create a UDP socket
    socketFD_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketFD_ < 0) {
        throw std::runtime_error("Failed to create socket");
    }

    // Look for ens33 interface to bind to
    struct ifaddrs* ifaddr;
    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        throw std::runtime_error("Failed to get network interfaces");
    }

    std::cout << "Available interfaces:\n";
    std::string ipAddress;
    for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr) continue;

        if (ifa->ifa_addr->sa_family == AF_INET &&
            std::string(ifa->ifa_name) == "ens33") {
            std::cout << ifa->ifa_name << std::endl;
            char host[NI_MAXHOST];
            int s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in),
                                host, NI_MAXHOST, nullptr, 0, NI_NUMERICHOST);
            if (s == 0) {
                ipAddress = host;
                break;
            }
        }
    }
    freeifaddrs(ifaddr);

    if (ipAddress.empty()) {
        throw std::runtime_error("Could not find IP for interface ens33");
    }

    // Set up the server address
    struct sockaddr_in serverAddress;
    memset(&serverAddress, 0, sizeof(serverAddress));
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(sPort);
    inet_pton(AF_INET, ipAddress.c_str(), &serverAddress.sin_addr);

    // Bind the socket
    if (bind(socketFD_, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0) {
        perror("bind");
        throw std::runtime_error("Failed to bind socket");
    }

    std::cout << "UDP socket bound to " << ipAddress << ":" << sPort << "\n";

    // Start the data transmission thread
    this->threadCanRun = true;
    this->transmissionThread = std::thread(&UDPSocket::transmissionThreadHandler, this);
}


UDPSocket::~UDPSocket() {
    // Close the socket
    close(socketFD_);

    this->threadCanRun = false;
    this->transmissionThread.join();
}


/**
 * @brief Transmit data over the UDP socket
 * 
 * @param pBuf Pointer to the data buffer to transmit
 * @param length Length of the data buffer
 * @return true Transmission successful
 * @return false Transmission failed
 */
bool UDPSocket::transmit(uint8_t* pBuf, size_t length) {
    if (pBuf == nullptr || length == 0 || lastClientAddress.sin_family != AF_INET) {
        return false;
    }

    // Set up the destination address
    struct sockaddr_in destAddress;
    std::memcpy(&destAddress, &lastClientAddress, sizeof(destAddress));

    destAddress.sin_family = AF_INET;
    destAddress.sin_port = htons(dport_);
    destAddress.sin_addr = this->lastClientAddress.sin_addr;
    // Send the data
    ssize_t bytesSent = sendto(socketFD_, pBuf, length, 0, (struct sockaddr *)&destAddress, sizeof(destAddress));
    if (bytesSent < 0) {
        perror("sendto");
        return false;
    }
    
    return true;
}


/**
 * @brief Thread handler for receiving data from the UDP socket
 * 
 * This function runs in a separate thread and listens for incoming UDP packets.
 * When data is received, it emits a signal to notify subscribers.
 */
void UDPSocket::transmissionThreadHandler(void) {
    char buffer[1500];

    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    if (setsockopt(socketFD_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        perror("setsockopt SO_RCVTIMEO");
    }

    while( this->threadCanRun ) {
        // Receive data from the socket
        struct sockaddr_in clientAddress;
        socklen_t clientAddressLength = sizeof(clientAddress);
        ssize_t bytesRead = recvfrom(socket_, buffer, sizeof(buffer), 0, (struct sockaddr *)&clientAddress, &clientAddressLength);
        if (bytesRead < 0) {
            if (errno == EWOULDBLOCK || errno == EAGAIN) {
                // Timeout occurred, just continue the loop
                continue;
            }
        }

        this->lastClientAddress = clientAddress;

        // Fire the signal to notify data has been received
        onDataReceived(reinterpret_cast<const uint8_t*>(buffer), static_cast<size_t>(bytesRead));
    }
}