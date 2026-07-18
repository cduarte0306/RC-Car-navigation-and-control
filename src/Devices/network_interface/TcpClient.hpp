#ifndef TCPCLIENT_HPP
#define TCPCLIENT_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <atomic>
#include <boost/asio.hpp>
#include "sockets.hpp"

namespace Network {
class TcpClient : public Sockets {
public:
    TcpClient(boost::asio::io_context& io_context, std::string host, unsigned short sPort, unsigned short dPort, size_t bufferSize=1024, bool broadcast=false);
    ~TcpClient();

    bool receive(std::vector<char>& buffer);

    bool transmit(const uint8_t* pBuf, size_t length);

    bool openSocket(std::string& adapterName, int sPort, int dPort, size_t bufferSize=1024, bool broadcast=false) override;

    void onConnectionEstablished(std::function<void(void)> callback);

    void startReceive(std::function<void(std::vector<char>&)> dataReceivedCallback_) override {(void) dataReceivedCallback_;}

    int close() override;
private:
    std::atomic<bool> connected_{false};
    std::function<void(void)> connectionEstablishedCallback_;
    boost::asio::steady_timer reconnectTimer_;
    boost::asio::ip::tcp::socket tcpSocket_;
    
    boost::asio::ip::tcp::endpoint serverEndpoint_; 

    int Open(void);
    int HandleDisconnectEvent(void);
    int TimerHandler(const boost::system::error_code& ec);

    int Connect();
};
} // namespace Network

#endif