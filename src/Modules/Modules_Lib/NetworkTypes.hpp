#ifndef NETWORK_TYPES_HPP
#define NETWORK_TYPES_HPP

#include <string>
#include <functional>
#include <atomic>
#include <vector>

// Mirrors Adapter::CommsAdapter's nested adapter-type enum (AdapterBase.hpp), duplicated here
// because this header can't include AdapterBase.hpp without creating a circular include.
enum NetworkAdapterType {
    UdpAdapterType       = 1,
    TcpServerAdapterType = 2,
    TcpClientAdapterType = 3
};

class NetworkAdapter {
public:
NetworkAdapter(const std::string& adapter_, int sPort_, int dPort_, size_t bufferSize_=2048);
    ~NetworkAdapter();
    std::function<int(const uint8_t*, size_t)> sendCallbackEth    = nullptr;
    std::function<int(const uint8_t*, size_t)> sendCallbackWlan   = nullptr;
    std::function<int(const uint8_t*, size_t)> sendCallback       = nullptr;
    std::function<int(const uint8_t*, size_t)> sendCallbackTcp    = nullptr;
    std::function<int(void)>                   preferredSrcPortCb = nullptr;
    std::function<int(void)>                   closeSocketCb      = nullptr;
    std::function<bool(void)>                  EthPresent         = nullptr;
    std::function<bool(void)>                  hostPresentCB      = nullptr;
    std::function<void()> onConnected = nullptr;

    int id = -1;
    int typeID = -1;
    int sPort = -1;
    int sPortEth = -1;
    int dPort = -1;
    const size_t bufferSize = 0;
    bool loopback = false;
    bool connected = false;
    std::string adapter;
    std::string parent;
    std::atomic<bool> wlanLinkDetected;
    std::atomic<bool> ethLinkDetected;
    bool broadcast = false;
    int adapterType = -1;
    int type = -1;

    int socketDesc{-1};

    int send(const uint8_t* data, size_t length, std::string destIp="");

    int getPreferredSrcPort() const;

    int closeSocket();

    bool IsEthPresent(void) const;

    bool IsHostPresent(void) const;

    void setParent(const std::string& name);
    
    void OnEthLinkDetected(bool state);

    void OnWlanLinkDetected(bool state);
};

class NetworkTcp : public NetworkAdapter {
public:
    std::function<int(const std::vector<char>&)> receiveCallback = nullptr;

    NetworkTcp(const std::string& adapter_, int sPort_, int dPort_, size_t bufferSize_=2048)
        : NetworkAdapter(adapter_, sPort_, dPort_, bufferSize_) {
        adapterType = TcpClientAdapterType;
    }

    /**
     * @brief Receive data from the TCP network adapter
     * 
     * @param buffer Buffer to store received data
     * @return int Status code
     */
    int receive(std::vector<char>& buffer);
};

#endif // NETWORK_TYPES_HPP