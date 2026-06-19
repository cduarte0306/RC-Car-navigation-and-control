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
    UdpServer(boost::asio::io_context& io_context, std::string adapter, unsigned short sPort, unsigned short dPort, size_t bufferSize=1024, bool broadcast=false);
    UdpServer(boost::asio::io_context& io_context);
    ~UdpServer();

    bool transmit(const uint8_t* pBuf, size_t length, std::string& ip) override;

    /** 
     * @brief Set broadcast mode for the socket. If enabled, the socket will send to the broadcast address of the network.
     * 
     * @param broadcast True to enable broadcast mode, false to disable
     * @return true Broadcast mode set successfully
     * @return false Failed to set broadcast mode
     */
    bool setBroadcast(bool broadcast);

    /**
     * @brief Opens UDP socket with provided options
     * 
     * @param adapterName Name of adapter at which the socket will be opened
     * @param sPort Source port
     * @param dPort Destination port
     * @param bufferSize Size of receive buffr
     * @param broadcast Is this a broadcast socket?
     * @return true 
     * @return false 
     */
    bool openSocket(std::string& adapterName, int sPort, int dPort, size_t bufferSize=1024, bool broadcast=false);

    virtual void startReceive(std::function<void(std::vector<char>&)> dataReceivedCallback_) override;
protected:
    void startReceive_(void);

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