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
    (void) fallbackAdapter; // fallback adapter is not currently used during reopen
    openSocket(adapter, sPort, dPort, bufferSize, broadcast);
}


bool TcpServer::openSocket(std::string& adapterName, int sPort, int dPort, size_t bufferSize, bool broadcast) {
    (void) broadcast; // Broadcast is not applicable for TCP, but kept for interface consistency.

    sport_ = sPort;
    dport_ = dPort;
    Logger* logger = Logger::getLoggerInst();

    m_RecvBuffer.resize(bufferSize);

    if (clientSocket_.is_open()) {
        boost::system::error_code closeEc;
        clientSocket_.close(closeEc);
    }

    if (acceptor_.is_open()) {
        boost::system::error_code cancelEc;
        acceptor_.cancel(cancelEc);
        boost::system::error_code closeEc;
        acceptor_.close(closeEc);
    }

    auto getAdapter = [&logger](std::string& adapter) -> std::string {
        std::string ipAddress;

        struct ifaddrs* ifaddr;
        if (getifaddrs(&ifaddr) == -1) {
            perror("getifaddrs");
            logger->log(Logger::LOG_LVL_ERROR, "Failed to get network interfaces\r\n");
            return "";
        }

        for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
            if (ifa->ifa_addr == nullptr) {
                continue;
            }

            if (ifa->ifa_addr->sa_family == AF_INET &&
                !(std::strcmp(ifa->ifa_name, adapter.c_str()))) {
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
        logger->log(Logger::LOG_LVL_ERROR, "Failed to get IP address for adapter: %s\r\n", adapterName.c_str());
        return false;
    }

    boost::system::error_code ec;
    boost::asio::ip::tcp::endpoint listenEndpoint(boost::asio::ip::make_address(ipAddress, ec), sport_);
    if (ec) {
        logger->log(Logger::LOG_LVL_ERROR, "Invalid TCP bind address: %s\r\n", ec.message().c_str());
        return false;
    }

    acceptor_.open(listenEndpoint.protocol(), ec);
    if (ec) {
        logger->log(Logger::LOG_LVL_ERROR, "TCP open error: %s\r\n", ec.message().c_str());
        return false;
    }

    acceptor_.set_option(boost::asio::socket_base::reuse_address(true), ec);
    if (ec) {
        logger->log(Logger::LOG_LVL_ERROR, "TCP set_option error: %s\r\n", ec.message().c_str());
        return false;
    }

    acceptor_.bind(listenEndpoint, ec);
    if (ec) {
        logger->log(Logger::LOG_LVL_ERROR, "TCP bind error: %s\r\n", ec.message().c_str());
        return false;
    }

    acceptor_.listen(boost::asio::socket_base::max_listen_connections, ec);
    if (ec) {
        logger->log(Logger::LOG_LVL_ERROR, "TCP listen error: %s\r\n", ec.message().c_str());
        return false;
    }

    logger->log(Logger::LOG_LVL_INFO, "Opened TCP socket: %s:%d\r\n", ipAddress.c_str(), sport_);
    return true;
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

    if (connectionEstablishedCallback_) {
        connectionEstablishedCallback_();
    }

    return 0;
}


void TcpServer::onConnectionEstablished(std::function<void()> callback) {
    connectionEstablishedCallback_ = std::move(callback);
}


bool TcpServer::transmit(const uint8_t* pBuf, size_t length) {
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


