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

class NetworkProxy : public NetworkTcp {
public:

    enum {
        UpdaterRouteAddr = 0x01, /*!< Updater application route address */
        WebAppRouteAddr  = 0x02, /*!< Web application route address */
        MainAppRouteAddr = 0x03  /*!< Main application route address */
    };


    std::function<int(const std::vector<char>&)> OnReceiveWebApp = nullptr;
    std::function<int(const std::vector<char>&)> OnReceiveUpdater = nullptr;

    NetworkProxy(int port, size_t bufferSize_=2048) 
        : NetworkTcp("lo", 0, port, bufferSize_) {
            routeCallbacks.reserve(WebAppRouteAddr);
            routeCallbacks[WebAppRouteAddr ] = webAppCallback;
            routeCallbacks[UpdaterRouteAddr] = updaterCallback;
    }

    /**
     * @brief Dispatch data to the web application
     * 
     * @param data Data to be dispatched
     * @return int Status code
     */
    int dispatchWebApp(const std::vector<char>& data);
    
    /**
     * @brief Dispatch data to the updater
     * 
     * @param data Data to be dispatched
     * @return int Status code
     */
    int dispatchUpdater(const std::vector<char>& data);

    /**
     * @brief Register callbacks for web app and updater
     * 
     * @param webAppCallback Callback for web app messages
     * @param updaterCallback Callback for updater messages
     * @return int Status code
     */
    template <typename T>
    int registerCallbacks(int (T::*webAppCallback)(const std::vector<char>&),
                          int (T::*updaterCallback)(const std::vector<char>&));

    /**
     * @brief Route a message based on its header information
     * 
     * @param data Data to be routed
     * @return int Status code
     */
    int routeMsg(const std::vector<char>& data);

private:
    struct ProxyMsgHdr {
        int srcAddr;   /*!< Source address */
        int destAddr;  /*!< Destination address */
        int len;
    };

    const int UpdaterDestAddr = MainAppRouteAddr;
    const int WebAppDestAddr  = WebAppRouteAddr;

    std::vector<std::function<int(const std::vector<char>&)>> routeCallbacks;

    std::function<int(const std::vector<char>&)> webAppCallback = nullptr;
    std::function<int(const std::vector<char>&)> updaterCallback = nullptr;
    const uint16_t webAppPort = 0;
    const uint16_t updaterPort = 0;

};

#endif // NETWORK_TYPES_HPP