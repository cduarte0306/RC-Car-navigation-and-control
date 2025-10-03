#include "udp_socket.hpp"
#include "sockets.hpp"
#include <functional>
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
#include "utils/logger.hpp"


using namespace Network;


UDPSocket::UDPSocket(int sPort, int dPort):Sockets() {
    sport_ = sPort;
    Logger* logger = Logger::getLoggerInst();

    // Create a UDP socket
    socketFD_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketFD_ < 0) {
        throw std::runtime_error("Failed to create socket");
    }

    // Look for enP8p1s0 interface to bind to
    struct ifaddrs* ifaddr;
    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        logger->log(Logger::LOG_LVL_ERROR, "Failed to get network interfaces\r\n");
        throw std::runtime_error("");
    }

    std::string ipAddress;
    for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr) continue;

        if (ifa->ifa_addr->sa_family == AF_INET &&
            std::string(ifa->ifa_name) == "wlP1p1s0") {
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
        logger->log(Logger::LOG_LVL_ERROR, "Could not find IP for interface wlP1p1s0\r\n");
        throw std::runtime_error("");
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
        logger->log(Logger::LOG_LVL_ERROR, "Failed to bind socket\r\n");
        throw std::runtime_error("");
    }

    logger->log(Logger::LOG_LVL_INFO, "UDP socket bound to %s:%lu\r\n", ipAddress.c_str(), sPort);

    // Start the data transmission thread
    this->threadCanRun = true;
    this->transmissionThread = std::thread(&UDPSocket::transmissionThreadHandler, this);
}


UDPSocket::UDPSocket(int sPort, char* adapter, bool startThread):Sockets()  {
    sport_ = sPort;
    Logger* logger = Logger::getLoggerInst();

    // Create a UDP socket
    socketFD_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketFD_ < 0) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to create socket\r\n");
        throw std::runtime_error("");
    }

    // Look for enP8p1s0 interface to bind to
    struct ifaddrs* ifaddr;
    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        logger->log(Logger::LOG_LVL_ERROR, "Failed to get network interfaces\r\n");
        throw std::runtime_error("");
    }

    std::string ipAddress;
    for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr) continue;

        if (ifa->ifa_addr->sa_family == AF_INET &&
            std::string(ifa->ifa_name) == adapter) {
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
        std::stringstream errOut;
        errOut << "Could not find IP for interface" << adapter;
        logger->log(Logger::LOG_LVL_ERROR, "Could not find IP for interface %s\r\n", adapter);
        throw std::runtime_error(errOut.str());
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

    logger->log(Logger::LOG_LVL_INFO, "UDP socket bound to %s:%lu\r\n", ipAddress.c_str(), sPort);

    // Start the data transmission thread
    if (!startThread) return;
    
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

    // Send the data
    ssize_t bytesSent = sendto(socketFD_, pBuf, length, 0, (struct sockaddr *)&this->lastClientAddress, sizeof(this->lastClientAddress));
    if (bytesSent < 0) {
        perror("sendto");
        return false;
    }
    
    return true;
}


/**
 * @brief Receive data from the UDP socket
 * 
 * @param pBuf Pointer to the buffer to store received data
 * @param length Length of the buffer
 * @return true Data received successfully
 * @return false Data reception failed
 */
bool UDPSocket::receive(uint8_t* pBuf, size_t length) {
    if (!pBuf) {
        return false;
    }

    ssize_t bytesRead = recvfrom(socketFD_, pBuf, length, 0, nullptr, nullptr);
    if (bytesRead < 0) {
        perror("recvfrom");
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
    struct sockaddr_in clientAddress;
    bool clientDetected = false;

    struct timeval timeout;
    timeout.tv_sec = 60;  // 5 seconds
    timeout.tv_usec = 0; // 0 microseconds

    Logger* logger = Logger::getLoggerInst();

    while( this->threadCanRun ) {
        // Receive data from the socket
        socklen_t clientAddressLength = sizeof(clientAddress);
        ssize_t bytesRead = recvfrom(this->socketFD_, buffer, sizeof(buffer), 0, (struct sockaddr *)&clientAddress, &clientAddressLength);
        if (bytesRead < 0) {
            continue;
        }

        if (!clientDetected) {
            logger->log(Logger::LOG_LVL_INFO, "Client detected: %s:%lu\r\n", inet_ntoa(clientAddress.sin_addr), ntohs(clientAddress.sin_port));
            clientDetected = true;
        }

        this->lastClientAddress = clientAddress;

        // Fire the signal to notify data has been received
        onDataReceived(reinterpret_cast<const uint8_t*>(buffer), static_cast<size_t>(bytesRead));
    }
}