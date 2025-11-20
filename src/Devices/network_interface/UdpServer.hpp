#ifndef UdpServer_HPP
#define UdpServer_HPP

#include <cstddef>
#include <unistd.h>

#include <mutex>

#include "sockets.hpp"
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>

namespace Network {

class UdpServer : public Sockets {
public:
    UdpServer(boost::asio::io_context& io_context, std::string adapter, std::string fallbackAdapter, unsigned short port);
    UdpServer(boost::asio::io_context& io_context, int port, char* adapter, bool startThread=true);
    ~UdpServer();

    virtual void startReceive(std::function<void(const uint8_t* data, size_t& length)> dataReceivedCallback_) override;

    int connectAdapter(std::string& ipAddress, int port);
private:
    void startReceive_(void);

    static constexpr int BUFFER_SIZE = 1024;
    static constexpr int MAX_CONNECTIONS = 10;
    static constexpr int TIMEOUT = 5000;

    bool threadCanRun = true;
    struct sockaddr_in lastClientAddress;

    boost::asio::io_context& io_context_;
};

}

#endif