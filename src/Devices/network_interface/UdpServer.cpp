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


UdpServer::UdpServer(boost::asio::io_context& io_context, std::string adapter, std::string fallbackAdapter, unsigned short port):
    Sockets(io_context, port), io_context_(io_context) {

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
    m_AdapterName = adapter;
    std::string ipAddress = getAdapter(adapter);
    if (ipAddress.empty()) {
        ipAddress = getAdapter(fallbackAdapter);
        m_AdapterName = fallbackAdapter;
        if (ipAddress.empty()) {
            logger->log(Logger::LOG_LVL_ERROR, "Could not find IP for interface %s\r\n", adapter.c_str());
            throw std::runtime_error("");
        }
    }
    boost::system::error_code ec;
    udp::endpoint listen_endpoint(boost::asio::ip::make_address(ipAddress), port);
    m_Socket_.open(listen_endpoint.protocol(), ec);
    m_Socket_.set_option(boost::asio::socket_base::reuse_address(true), ec);
    m_Socket_.bind(listen_endpoint, ec);
    if (ec) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to open UDP socket: %s:%lu\r\n", ipAddress.c_str(), port);
        throw std::runtime_error("");
    }

    logger->log(Logger::LOG_LVL_INFO, "Opened UDP socket: %s:%lu\r\n", ipAddress.c_str(), port);
}


UdpServer::~UdpServer() {
    // Close the socket
    boost::system::error_code ec;
    m_Socket_.cancel(ec);
    m_Socket_.close(ec);
    if (ec) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Failed to Delete UDP socket: %s\r\n", ec.message().c_str());
    }
}


/**
 * @brief Start receiving data asynchronously
 * 
 * @param dataReceivedCallback_ Callback function to handle received data
 */
void UdpServer::startReceive(std::function<void(const uint8_t* data, size_t& length)> dataReceivedCallback_) {
    m_DataReceivedCallback = dataReceivedCallback_;

    this->startReceive_();
}


int UdpServer::connectAdapter(std::string& ipAddress, int port) {
    boost::system::error_code ec;
    m_Socket_.cancel(ec);
    m_Socket_.close(ec);
    m_Socket_ = udp::socket(io_context_);

    m_Socket_.open(udp::v4(), ec);
    if (ec) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Failed to open socket\r\n");
        return -1;
    }

    m_Socket_.set_option(boost::asio::socket_base::reuse_address(true));
    udp::endpoint listen_endpoint(boost::asio::ip::make_address(ipAddress, ec), port);

    m_Socket_.bind(listen_endpoint, ec);
    if (ec) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Failed to bind UDP socket: %s\r\n", ec.message().c_str());
        return -1;
    }

    // Restart async reception
    this->startReceive_();
    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Reconnected UDP socket: %s:%d\r\n",
                ipAddress.c_str(), port);
    return 0;
}


/**
 * @brief Start asynchronous receive operation
 * 
 */
void UdpServer::startReceive_(void) {
    m_Socket_.async_receive_from(
        boost::asio::buffer(recv_buffer_), remote_endpoint_,
        [this](boost::system::error_code ec, std::size_t bytes_recvd) {
            if (!ec && bytes_recvd > 0) {
                // Call the data received callback
                if (m_DataReceivedCallback) {
                    m_DataReceivedCallback(reinterpret_cast<const uint8_t*>(recv_buffer_.data()), bytes_recvd);

                    // Reply with the receive buffer
                    m_Socket_.async_send_to(
                    boost::asio::buffer(recv_buffer_.data(), bytes_recvd), remote_endpoint_,
                    [](const boost::system::error_code& ec, std::size_t bytes_sent) {
                        if (ec) {
                            Logger* logger = Logger::getLoggerInst();
                            logger->log(Logger::LOG_LVL_ERROR, "UDP send error: %s\r\n", ec.message().c_str());
                        }
                    });

                }
            }
            // Continue receiving
            this->startReceive_();
        });
}
