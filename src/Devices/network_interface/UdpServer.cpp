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
#include <sstream>
#include <fstream>
#include "utils/logger.hpp"


using namespace Network;


UdpServer::UdpServer(boost::asio::io_context& io_context, std::string adapter, unsigned short sPort, unsigned short dPort, size_t bufferSize, bool broadcast):
    Sockets(io_context, sPort), m_Broadcast(broadcast) {
    if (!UdpServer::openSocket(adapter, sPort, dPort, bufferSize, broadcast)) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR,
            "UdpServer: failed to open socket on adapter '%s' port %d\r\n",
            adapter.c_str(), sPort);
    }
}


UdpServer::UdpServer(boost::asio::io_context& io_context) : Sockets(io_context) {
    
}


bool UdpServer::openSocket(std::string& adapterName, int sPort, int dPort, size_t bufferSize, bool broadcast) {
    sport_ = sPort;
    dport_ = dPort;
    Logger* logger = Logger::getLoggerInst();

    m_RecvBuffer.resize(bufferSize);

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

    std::string ipAddress = getAdapter(adapterName);
    if (ipAddress.empty()) {
        logger->log(Logger::LOG_LVL_ERROR,
            "UdpServer: adapter '%s' not found — check interface name (available: run 'ip link show')\r\n",
            adapterName.c_str());
        return false;
    }

    udp::endpoint listen_endpoint = m_Broadcast
        ? udp::endpoint(udp::v4(), sPort)
        : udp::endpoint(boost::asio::ip::make_address(ipAddress), sPort);
    socket_.open(listen_endpoint.protocol());
    socket_.set_option(boost::asio::socket_base::reuse_address(true));
    socket_.set_option(boost::asio::socket_base::broadcast(m_Broadcast));
    socket_.bind(listen_endpoint);

    sport_ = socket_.local_endpoint().port();  // Update in case we used port 0
    dport_ = dPort;

    // Fill host field with broadcast version
    if (m_Broadcast) {
        std::string netMask = getNetMask(adapterName);
        std::vector<char> ipOctets;
        std::vector<char> maskOctets;

        ipOctets.reserve(4);
        maskOctets.reserve(4);
        std::istringstream ipStream(ipAddress);
        std::istringstream maskStream(netMask);
        std::string segment;
        while (std::getline(ipStream, segment, '.')) {
            ipOctets.push_back(static_cast<char>(std::stoi(segment)));
        }
        while (std::getline(maskStream, segment, '.')) {
            maskOctets.push_back(static_cast<char>(std::stoi(segment)));
        }

        if (ipOctets.size() == 4 && maskOctets.size() == 4) {
            std::string broadcastIp;
            for (size_t i = 0; i < 4; ++i) {
                char broadcastOctet = ipOctets[i] | (~maskOctets[i]);
                broadcastIp += std::to_string(static_cast<unsigned char>(broadcastOctet));
                if (i < 3) {
                    broadcastIp += ".";
                }
            }
            m_BroadcastIP = broadcastIp;
        }
    }

    logger->log(Logger::LOG_LVL_INFO, "Opened UDP socket: %s:%d\r\n", ipAddress.c_str(), sport_);
    return true;
}


bool UdpServer::setBroadcast(bool broadcast) {
    boost::system::error_code ec;
    socket_.set_option(boost::asio::socket_base::broadcast(broadcast), ec);
    if (ec) {
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_ERROR, "Failed to set broadcast mode: %s\r\n", ec.message().c_str());
        return false;
    }

    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Broadcast mode %s\r\n", broadcast ? "enabled" : "disabled");
    m_Broadcast = broadcast;
    return true;
}


/**
 * @brief Reads MAC address from device
 * 
 * @return std::string 
 */
std::string UdpServer::getNetMask(std::string& iface) {
    ifaddrs* ifaddr = nullptr;
    if (getifaddrs(&ifaddr) != 0)
        return std::string("");

    std::string result("");

    for (auto* ifa = ifaddr; ifa; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr || !ifa->ifa_netmask)
            continue;

        if (iface != ifa->ifa_name)
            continue;

        if (ifa->ifa_addr->sa_family == AF_INET) {
            auto* nm = reinterpret_cast<sockaddr_in*>(ifa->ifa_netmask);
            result = inet_ntoa(nm->sin_addr);  // dotted-decimal
            break;
        }
    }

    freeifaddrs(ifaddr);
    return result;

}


/**
 * @brief Start receiving data asynchronously
 * 
 * @param dataReceivedCallback_ Callback function to handle received data
 */
void UdpServer::startReceive(std::function<void(std::vector<char>&)> dataReceivedCallback_, bool asyncTx) {
    dataReceivedCallback = dataReceivedCallback_;
    m_AsyncTx = asyncTx;
    this->startReceive_();
}


/**
 * @brief Start asynchronous receive operation
 * 
 */
void UdpServer::startReceive_(void) {
    socket_.async_receive_from(
        boost::asio::buffer(m_RecvBuffer), remoteEndpoint,
        [this](boost::system::error_code ec, std::size_t bytes_recvd) {
            if (!ec && bytes_recvd > 0) {
                // Call the data received callback
                if (dataReceivedCallback) {
                    Logger* logger = Logger::getLoggerInst();
                    std::vector<char> dataReceived(m_RecvBuffer.begin(), m_RecvBuffer.begin() + bytes_recvd);
                    m_HostIP = remoteEndpoint.address().to_string();
                    m_Port = remoteEndpoint.port();
                    if (!m_HostFound) {
                        if (dport_ <= 0) {
                            dport_ = remoteEndpoint.port();
                        }
                        logger->log(Logger::LOG_LVL_INFO, "Host found: %s:%d\n", m_HostIP.c_str(), dport_);
                        m_HostFound = true;
                    }

                    dataReceivedCallback(dataReceived);
                    if (dataReceived.size() > m_RecvBuffer.size()) {
                        m_RecvBuffer.resize(dataReceived.size());
                    }
                    std::copy(dataReceived.begin(), dataReceived.end(), m_RecvBuffer.begin());
                    // Reply with the receive buffer
                    if (m_AsyncTx) {
                        socket_.async_send_to(
                            boost::asio::buffer(m_RecvBuffer.data(), dataReceived.size()), remoteEndpoint,
                            [&](const boost::system::error_code& ec, std::size_t bytes_sent) {
                                if (ec) {
                                    logger->log(Logger::LOG_LVL_ERROR, "UDP send error: %s\r\n", ec.message().c_str());
                                }
                        });
                    }
                }
            }
            // Continue receiving
            m_RxBytes += bytes_recvd;
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
bool UdpServer::transmit(const uint8_t* pBuf, size_t length, std::string& ip) {
    if (pBuf == nullptr || length == 0 || ip.length() == 0 || dport_ == 0) {
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_ERROR, "Invalid parameters for UDP transmit: pBuf=%p, length=%zu, ip=%s, dport=%d\r\n", pBuf, length, ip.c_str(), dport_);
        return false;
    }

    if (m_Broadcast) {
        ip = m_BroadcastIP;
    }
    udp::endpoint remoteEndpoint(
        boost::asio::ip::address::from_string(ip),
        dport_
    );

    Logger* logger = Logger::getLoggerInst();
    ssize_t bytes_sent = socket_.send_to(boost::asio::buffer(pBuf, length), remoteEndpoint);
    if (bytes_sent < 0) {
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_ERROR, "UDP send error: %s, message length: %lu\r\n", strerror(errno), length);
        return false;
    }

    m_TxBytes += length;
    return true;
}

