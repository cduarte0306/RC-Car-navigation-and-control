#include "TcpClient.hpp"
#include "sockets.hpp"
#include "utils/logger.hpp"
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>

namespace Network {
TcpClient::TcpClient(boost::asio::io_context& io_context, std::string host, unsigned short sPort, unsigned short dPort, size_t bufferSize, bool broadcast) :
    Sockets(io_context, sPort), tcpSocket_(io_context), reconnectTimer_(io_context) {
    // Create the socket
    dport_ = dPort;
    m_HostIP = host;
    (void) Open();
    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "TCP client socket created -> %s:%d\r\n", tcpSocket_.local_endpoint().address().to_string().c_str(), tcpSocket_.local_endpoint().port());
}

TcpClient::~TcpClient() {
    if (tcpSocket_.is_open()) {
        boost::system::error_code ec;
        tcpSocket_.close(ec);
    }
}

bool TcpClient::receive(uint8_t* pBuf, size_t length) {
    if (!tcpSocket_.is_open()) {
        return false;
    }
    boost::system::error_code ec;
    size_t bytesRead = boost::asio::read(tcpSocket_, boost::asio::buffer(pBuf, length), ec);
    if (ec) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "TCP receive error: %s. Attempting to reconnect\r\n", ec.message().c_str());
        Connect();
    }
    return !ec && bytesRead == length;
}

bool TcpClient::transmit(const uint8_t* pBuf, size_t length) {
    if (!tcpSocket_.is_open()) {
        return false;
    }
    boost::system::error_code ec;
    boost::asio::write(tcpSocket_, boost::asio::buffer(pBuf, length), ec);
    if (ec) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "TCP send error: %s. Attempting to reconnect\r\n", ec.message().c_str());
        Connect();
    }
    return !ec;
}

bool TcpClient::openSocket(std::string& adapterName, int sPort, int dPort, size_t bufferSize, bool broadcast) {
    (void) adapterName;
    (void) sPort;
    (void) dPort;
    (void) bufferSize;
    (void) broadcast;
    return true;
}

void TcpClient::onConnectionEstablished(std::function<void(void)> callback) {
    connectionEstablishedCallback_ = callback;
}

int TcpClient::close() {
    if (tcpSocket_.is_open()) {
        boost::system::error_code ec;
        tcpSocket_.close(ec);
    }
    return 0;
}

int TcpClient::TimerHandler(const boost::system::error_code& ec) {
    if (ec) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Timer error: %s\r\n", ec.message().c_str());
        return -1;
    }
    Connect();
    return 0;
}

int TcpClient::HandleDisconnectEvent(void) {
    connected_ = false;
    boost::system::error_code ec;
    if (tcpSocket_.is_open()) {
        tcpSocket_.close(ec);
    }
    reconnectTimer_.expires_after(std::chrono::seconds(5));
    reconnectTimer_.async_wait([this](const boost::system::error_code& ec) {
        TimerHandler(ec);
    });
    return 0;
}

int TcpClient::Open(void) {
    tcpSocket_.open(boost::asio::ip::tcp::v4());
    tcpSocket_.set_option(boost::asio::ip::tcp::socket::reuse_address(true));
    tcpSocket_.bind(boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 0));
    
    serverEndpoint_ = boost::asio::ip::tcp::endpoint(boost::asio::ip::make_address(m_HostIP), dport_);
    tcpSocket_.connect(serverEndpoint_);
    Connect();
    return 0;
}

int TcpClient::Connect() {
    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Attempting to connect to server -> %s:%d\r\n", m_HostIP.c_str(), dport_);
    tcpSocket_.async_connect(
        boost::asio::ip::tcp::endpoint(boost::asio::ip::make_address(m_HostIP), dport_),
        [this](const boost::system::error_code& ec) {
            if (ec) {
                Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Failed to connect to server\r\n");
            } else {
                // Connection successful
                connected_ = true;
                reconnectTimer_.cancel();
                serverEndpoint_ = tcpSocket_.remote_endpoint();
                if (connectionEstablishedCallback_) {
                    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "TCP connection established -> %s:%d\r\n", serverEndpoint_.address().to_string().c_str(), serverEndpoint_.port());
                    connectionEstablishedCallback_();
                }
            }
        }
    );
    return 0;
}
} // namespace Network