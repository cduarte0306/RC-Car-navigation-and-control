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
    UdpServer(boost::asio::io_context& io_context, std::string adapter, std::string fallbackAdapter, unsigned short port, size_t bufferSize=1024, bool broadcast=false);
    ~UdpServer();

    bool transmit(uint8_t* pBuf, size_t length, std::string& ip) override;

    virtual void startReceive(std::function<void(std::vector<char>&)> dataReceivedCallback_, bool asyncTx=true) override;
    
private:
    void startReceive_(void);

    std::string getNetMask(std::string& iface);

    static constexpr int BUFFER_SIZE = 1024;
    static constexpr int MAX_CONNECTIONS = 10;
    static constexpr int TIMEOUT = 5000;

    bool threadCanRun = true;

    bool m_Broadcast{false};

    std::string m_BroadcastIP{""};

    bool m_HostFound{false};
};

}

#endif