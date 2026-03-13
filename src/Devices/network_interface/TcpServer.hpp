#ifndef TcpServer_HPP
#define TcpServer_HPP

#include <cstddef>
#include <unistd.h>

#include <mutex>

#include "sockets.hpp"
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>

namespace Network {

class TcpServer : public Sockets {
public:
    TcpServer(boost::asio::io_context& io_context, std::string adapter, std::string fallbackAdapter, unsigned short sPort, unsigned short dPort, size_t bufferSize=1024, bool broadcast=false);
    ~TcpServer();

    bool receive(uint8_t* pBuf, size_t length) override;

    bool transmit(uint8_t* pBuf, size_t length) override;

    virtual void startReceive(std::function<void(std::vector<char>&)> dataReceivedCallback_, bool asyncTx=true) override;

    int acceptConnection();
    void onConnectionEstablished(std::function<void()> callback);

private:
    std::function<void()> connectionEstablishedCallback_;
    void startReceive_(void);
    
    boost::asio::ip::tcp::acceptor acceptor_;
    boost::asio::ip::tcp::socket clientSocket_;

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