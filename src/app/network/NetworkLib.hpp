#pragma once

#include <memory>

#include "Devices/network_interface/sockets.hpp"
#include "Devices/network_interface/UdpServer.hpp"
#include "Devices/network_interface/TcpServer.hpp"
#include "Devices/network_interface/TcpClient.hpp"

namespace NetUtils {
template<typename T>
class NetworkPort;

class PortManager {
public:
    PortManager(boost::asio::io_context& ioContext, int srcPort = 0, int dstPort = 0, size_t bufferSize = 1024) : ioContext_(ioContext), srcPort_(srcPort), dstPort_(dstPort), bufferSize(bufferSize) {}
    virtual ~PortManager() = default;

    virtual int open(int srcPort = 0, int dstPort = 0) = 0;
    virtual int close() = 0;
    virtual int sendEth(std::string& targetIP, const std::vector<char>& data) = 0;
    virtual int sendWlan(std::string& targetIP, const std::vector<char>& data) = 0;
    virtual int setReceiveCallback(std::function<void(std::vector<char>&)> callback) = 0;
    virtual std::string getHostIP() = 0;
protected:
    boost::asio::io_context& ioContext_;
    int srcPort_;
    int dstPort_;
    const size_t bufferSize{1024};
};

template<>
class NetworkPort<Network::UdpServer> : public PortManager {
public:
    NetworkPort(boost::asio::io_context& ioContext, int srcPort = 0, int dstPort = 0, size_t bufferSize_=1024);
    explicit NetworkPort(boost::asio::io_context& ioContext);
    ~NetworkPort();
    NetworkPort(const NetworkPort&) = delete;
    NetworkPort& operator=(const NetworkPort&) = delete;
    NetworkPort(NetworkPort&&) = delete;
    NetworkPort& operator=(NetworkPort&&) = delete;

    Network::UdpServer* eth() const;
    Network::UdpServer* wlan() const;
    Network::UdpServer* preferred() const;

    bool hasEth() const;
    bool hasWlan() const;

    virtual int open(int srcPort = 0, int dstPort = 0);
    virtual int close() override;
    virtual int sendEth(std::string& targetIP, const std::vector<char>& data) override;
    virtual int sendWlan(std::string& targetIP, const std::vector<char>& data) override;
    virtual int setReceiveCallback(std::function<void(std::vector<char>&)> callback) override;
    virtual std::string getHostIP() override;

private:
    std::unique_ptr<Network::UdpServer> m_Eth;
    std::unique_ptr<Network::UdpServer> m_Wlan;
};

template<>
class NetworkPort<Network::TcpServer> : public PortManager {
public:
    NetworkPort(boost::asio::io_context& ioContext, int srcPort = 0, int dstPort = 0, size_t bufferSize_=1024, bool lo=false);
    explicit NetworkPort(boost::asio::io_context& ioContext);
    ~NetworkPort();
    NetworkPort(const NetworkPort&) = delete;
    NetworkPort& operator=(const NetworkPort&) = delete;
    NetworkPort(NetworkPort&&) = delete;
    NetworkPort& operator=(NetworkPort&&) = delete;

    Network::TcpServer* eth() const;
    Network::TcpServer* wlan() const;
    Network::TcpServer* preferred() const;
    bool hasEth() const;
    bool hasWlan() const;
    bool hasLo() const;

    virtual int open(int srcPort = 0, int dstPort = 0);
    virtual int close() override;
    virtual int sendEth(std::string& targetIP, const std::vector<char>& data) override;
    virtual int sendWlan(std::string& targetIP, const std::vector<char>& data) override;
    virtual int setReceiveCallback(std::function<void(std::vector<char>&)> callback) override;
    virtual std::string getHostIP() override;

private:
    bool lo_{false};
    std::unique_ptr<Network::TcpServer> m_Eth  = nullptr;
    std::unique_ptr<Network::TcpServer> m_Wlan = nullptr;
    std::unique_ptr<Network::TcpServer> m_Lo   = nullptr;
};

template<>
class NetworkPort<Network::TcpClient> : public PortManager {
public:
    NetworkPort(boost::asio::io_context& ioContext, int srcPort = 0, int dstPort = 0, size_t bufferSize_=1024, bool lo=false);
    explicit NetworkPort(boost::asio::io_context& ioContext);
    ~NetworkPort();
    NetworkPort(const NetworkPort&) = delete;
    NetworkPort& operator=(const NetworkPort&) = delete;
    NetworkPort(NetworkPort&&) = delete;
    NetworkPort& operator=(NetworkPort&&) = delete;

    Network::TcpClient* eth() const;
    Network::TcpClient* wlan() const;
    Network::TcpClient* preferred() const;
    bool hasEth() const;
    bool hasWlan() const;
    bool hasLo() const;

    int open(std::string& host, unsigned dstPort);
    virtual int open(int srcPort = 0, int dstPort = 0);
    virtual int close() override;
    virtual int sendEth(std::string& targetIP, const std::vector<char>& data) override;
    virtual int sendWlan(std::string& targetIP, const std::vector<char>& data) override;
    virtual int setReceiveCallback(std::function<void(std::vector<char>&)> callback) override;
    virtual std::string getHostIP() override;

private:
    bool lo_{false};
    std::unique_ptr<Network::TcpClient> m_Eth  = nullptr;
    std::unique_ptr<Network::TcpClient> m_Wlan = nullptr;
    std::unique_ptr<Network::TcpClient> m_Lo   = nullptr;
};
}

#pragma endregion