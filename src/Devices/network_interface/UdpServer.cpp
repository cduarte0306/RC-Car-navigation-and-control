#include "UdpServer.hpp"
#include "sockets.hpp"
#include <functional>
#include <regex>
#include <ifaddrs.h>
#include <thread>
#include <unistd.h>
#include <cstring>
#include <netdb.h>
#include <iostream>
#include "utils/logger.hpp"


using namespace Network;


UdpServer::UdpServer(boost::asio::io_context& io_context, std::string adapter, unsigned short port):
    Sockets(io_context, port) {

    sport_ = port;
    Logger* logger = Logger::getLoggerInst();

    auto getAdapter = [&logger](std::string& adapterName) -> std::string {
        std::string ipAddress;

        // Look for enP8p1s0 interface to bind to
        struct ifaddrs* ifaddr;
        if (getifaddrs(&ifaddr) == -1) {
            perror("getifaddrs");
            logger->log(Logger::LOG_LVL_ERROR, "Failed to get network interfaces\r\n");
            throw std::runtime_error("");
        }

        for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
            if (ifa->ifa_addr == nullptr) continue;

            if (ifa->ifa_addr->sa_family == AF_INET &&
                !(std::strcmp(ifa->ifa_name, adapterName.c_str()))) {
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
        return ipAddress;
    };

    std::string ipAddress = getAdapter(adapter);
    if (ipAddress.empty()) {
        ipAddress = getAdapter(adapter);
        if (ipAddress.empty()) {
            logger->log(Logger::LOG_LVL_ERROR, "Could not find IP for interface %s\r\n", adapter.c_str());
            throw std::runtime_error("");
        }
    }

    udp::endpoint listen_endpoint(boost::asio::ip::make_address(ipAddress), port);
    socket_.open(listen_endpoint.protocol());
    socket_.bind(listen_endpoint);
    logger->log(Logger::LOG_LVL_INFO, "Opened UDP socket: %s:%lu\r\n", ipAddress.c_str(), port);
}


/**
 * @brief Start receiving data asynchronously
 * 
 * @param dataReceivedCallback_ Callback function to handle received data
 */
void UdpServer::startReceive(std::function<void(const uint8_t* data, size_t length)> dataReceivedCallback_) {
    dataReceivedCallback = dataReceivedCallback_;

    this->startReceive_();
}


/**
 * @brief Start asynchronous receive operation
 * 
 */
void UdpServer::startReceive_(void) {
    socket_.async_receive_from(
        boost::asio::buffer(recv_buffer_), remote_endpoint_,
        [this](boost::system::error_code ec, std::size_t bytes_recvd) {
            if (!ec && bytes_recvd > 0) {
                // Call the data received callback
                if (dataReceivedCallback) {
                    dataReceivedCallback(reinterpret_cast<const uint8_t*>(recv_buffer_.data()), bytes_recvd);
                }
            }
            // Continue receiving
            this->startReceive_();
        });
}


UdpServer::~UdpServer() {
    // Close the socket
    socket_.close();
}


/**
 * @brief Transmit data over the UDP socket
 * 
 * @param pBuf Pointer to the data buffer to transmit
 * @param length Length of the data buffer
 * @return true Transmission successful
 * @return false Transmission failed
 */
bool UdpServer::transmit(uint8_t* pBuf, size_t length) {
    if (pBuf == nullptr || length == 0 || lastClientAddress.sin_family != AF_INET) {
        return false;
    }
    
    return true;
}

