#include "TcpServer.hpp"
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


TcpServer::TcpServer(boost::asio::io_context& io_context, std::string adapter, std::string fallbackAdapter, unsigned short sPort, unsigned short dPort, size_t bufferSize, bool broadcast):
    Sockets(io_context, sPort), acceptor_(io_context), clientSocket_(io_context), m_Broadcast(broadcast) {

    (void) broadcast; // Broadcast is not applicable for TCP, but we keep the parameter for interface consistency with UdpServer
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

    std::string ipAddress = getAdapter(adapter);
    if (ipAddress.empty() && !fallbackAdapter.empty()) {
        ipAddress = getAdapter(fallbackAdapter);
    }
    if (ipAddress.empty()) {
        throw std::runtime_error("Failed to get IP address for adapter: " + adapter);
    }

    boost::asio::ip::tcp::endpoint listen_endpoint(boost::asio::ip::make_address(ipAddress), sport_);
    acceptor_.open(listen_endpoint.protocol());
    acceptor_.bind(listen_endpoint);
    acceptor_.listen();
    logger->log(Logger::LOG_LVL_INFO, "Opened TCP socket: %s:%d\r\n", ipAddress.c_str(), sport_);
    
}


TcpServer::~TcpServer() {
    threadCanRun = false;
    if (clientSocket_.is_open()) {
        boost::system::error_code ec;
        clientSocket_.close(ec);
    }
    acceptor_.close();
}


int TcpServer::acceptConnection() {
    if (clientSocket_.is_open()) {
        boost::system::error_code ec;
        clientSocket_.close(ec);
    }

    boost::system::error_code ec;
    acceptor_.accept(clientSocket_, ec);
    if (ec) {
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_ERROR, "TCP accept error: %s\r\n", ec.message().c_str());
        return -1;
    }

    const auto clientEndpoint = clientSocket_.remote_endpoint(ec);
    if (ec) {
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_ERROR, "TCP endpoint error: %s\r\n", ec.message().c_str());
        return -1;
    }

    m_HostIP = clientEndpoint.address().to_string();
    Logger* logger = Logger::getLoggerInst();
    logger->log(Logger::LOG_LVL_INFO, "Client connected: %s:%d\r\n", m_HostIP.c_str(), clientEndpoint.port());
    return 0;
}


bool TcpServer::transmit(uint8_t* pBuf, size_t length) {
    if (pBuf == nullptr || length == 0) {
        return false;
    }

    if (!clientSocket_.is_open() && acceptConnection() != 0) {
        return false;
    }

    boost::system::error_code ec;
    const size_t bytesSent = boost::asio::write(clientSocket_, boost::asio::buffer(pBuf, length), ec);
    if (ec || bytesSent == 0) {
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_ERROR, "TCP send error: %s\r\n", ec.message().c_str());
        return false;
    }

    m_TxBytes += bytesSent;
    return true;
}


bool TcpServer::receive(uint8_t* pBuf, size_t length) {
    if (pBuf == nullptr || length == 0) {
        return false;
    }

    if (!clientSocket_.is_open() && acceptConnection() != 0) {
        return false;
    }

    boost::system::error_code ec;
    const size_t bytesReceived = clientSocket_.read_some(boost::asio::buffer(pBuf, length), ec);
    if (ec || bytesReceived == 0) {
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_ERROR, "TCP receive error: %s\r\n", ec.message().c_str());
        return false;
    }

    m_RxBytes += bytesReceived;
    return true;
}


void TcpServer::startReceive(std::function<void(std::vector<char>&)> dataReceivedCallback_, bool asyncTx) {
    dataReceivedCallback = std::move(dataReceivedCallback_);
    m_AsyncTx = asyncTx;

    if (!clientSocket_.is_open() && acceptConnection() != 0) {
        return;
    }

    this->startReceive_();
}


void TcpServer::startReceive_(void) {
    if (!clientSocket_.is_open()) {
        return;
    }

    clientSocket_.async_read_some(
        boost::asio::buffer(m_RecvBuffer),
        [this](const boost::system::error_code& ec, std::size_t bytes_recvd) {
            if (!ec && bytes_recvd > 0) {
                m_RxBytes += bytes_recvd;

                if (dataReceivedCallback) {
                    std::vector<char> dataReceived(m_RecvBuffer.begin(), m_RecvBuffer.begin() + bytes_recvd);
                    dataReceivedCallback(dataReceived);

                    if (m_AsyncTx && !dataReceived.empty()) {
                        auto txData = std::make_shared<std::vector<char>>(std::move(dataReceived));
                        boost::asio::async_write(
                            clientSocket_,
                            boost::asio::buffer(*txData),
                            [this, txData](const boost::system::error_code& writeEc, std::size_t bytes_sent) {
                                if (!writeEc) {
                                    m_TxBytes += bytes_sent;
                                    return;
                                }
                                Logger* logger = Logger::getLoggerInst();
                                logger->log(Logger::LOG_LVL_ERROR, "TCP async send error: %s\r\n", writeEc.message().c_str());
                            });
                    }
                }

                this->startReceive_();
                return;
            }

            Logger* logger = Logger::getLoggerInst();
            logger->log(Logger::LOG_LVL_ERROR, "TCP receive loop ended: %s\r\n", ec.message().c_str());
            clientSocket_.close();
            if (acceptConnection() == 0) {
                this->startReceive_();
            }
        });
}


