#ifndef SOCKETS_HPP
#define SOCKETS_HPP

#include <sys/socket.h>
#include <netinet/in.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <functional>
#include <vector>
#include <thread>
#include <optional>

#include <boost/signals2.hpp> // Added for boost signals
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <functional>

#define ETHAdapter "enP8p1s0"
#define WLANAdapter "wlP1p1s0"

using boost::asio::ip::udp;
using boost::asio::ip::tcp;

namespace Network {

class Sockets {
public:

    Sockets(boost::asio::io_context& io_context, unsigned short port) : socket_(io_context) {

    }

    explicit Sockets(boost::asio::io_context& io_context)
        : Sockets(io_context, 0) {}

    virtual ~Sockets() {

    }

    virtual bool transmit(const uint8_t* pBuf, size_t length, std::string& ip) {
        return true;
    }

    virtual bool transmit(const uint8_t* pBuf, size_t length) {
        return true;
    }

    virtual bool transmit(const std::vector<uint8_t>& data) {
        return true;
    }

    virtual bool receive(uint8_t* pBuf, size_t length ) {
        return true;
    }

    virtual int close() {
        return 0;
    }

    boost::signals2::signal<void(const uint8_t* data, size_t length)> onDataReceived;

    // Receive callback uses a mutable vector buffer to avoid raw pointer/length pairs.
    virtual void startReceive(std::function<void(std::vector<char>&)> callback) = 0;

    /**
     * @brief Read the host IP
     * 
     * @return std::string 
     */
    std::string getHostIP() const {
        return m_HostIP;
    }

    void setRemoteEndpoint(const std::string& hostIP, int port) {
        if (!hostIP.empty()) {
            m_HostIP = hostIP;
        }
        if (port > 0) {
            m_Port = port;
            dport_ = port;
        }
    }

    /**
     * @brief Get the port number of the remote host
     * 
     * @return int 
     */
    int getPort() const {
        return m_Port;
    }
    

    static std::optional<std::string> findInterface(const char* name) {
        if (!name) {
            return std::nullopt;
        }

        struct ifaddrs* ifaddr = nullptr;
        if (getifaddrs(&ifaddr) == -1) {
            perror("getifaddrs");
            return std::nullopt;
        }

        std::string ipAddress;
        for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
            if (ifa->ifa_addr == nullptr) continue;

            if (ifa->ifa_addr->sa_family == AF_INET &&
                std::string(ifa->ifa_name) == name) {
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

        if (ipAddress.empty()) {
            return std::nullopt;
        }

        return ipAddress;
    }

    std::string& GetAdapterName() { return m_AdapterName; }

    int getSrcPort() const { return sport_; };

    int getDstPort() const { return dport_; };

    size_t GetTxBytes() const { return m_TxBytes; };

    size_t GetRxBytes() const { return m_RxBytes; };

    void resetCounters() {
        m_TxBytes = 0;
        m_RxBytes = 0;
    }

    virtual bool openSocket(std::string& adapterName, int sPort, int dPort, size_t bufferSize=1024, bool broadcast=false) {
        return true;
    }

    /**
     * @brief Reads MAC address from device
     * 
     * @return std::string 
     */
    static std::string getNetMask(std::string& iface) {
        ifaddrs* ifaddr = nullptr;
        if (getifaddrs(&ifaddr) != 0)
            return std::string("");

        std::string result("");

        for (auto* ifa = ifaddr; ifa; ifa = ifa->ifa_next) {
            if (!ifa->ifa_addr || !ifa->ifa_netmask)
                continue;

            if (iface != ifa->ifa_name)
                continue;

            if (ifa->ifa_addr->sa_family == AF_INET) {
                auto* nm = reinterpret_cast<sockaddr_in*>(ifa->ifa_netmask);
                result = inet_ntoa(nm->sin_addr);  // dotted-decimal
                break;
            }
        }

        freeifaddrs(ifaddr);
        return result;
    }

protected:
    bool threadCanRun = true;

    int sport_ = -1;
    int dport_ = -1;

    // Option to let the server known wether it should reply asynchronously
    bool m_AsyncTx{false};

    /**
     * @brief Callback function when data is received
     * 
     */
    std::function<void(std::vector<char>&)> dataReceivedCallback;
    
    std::string m_AdapterName;
    std::string m_HostIP;
    int m_Port = -1;
    udp::socket socket_;
    udp::endpoint remoteEndpoint;

    tcp::endpoint tcpEndpoint_;
    // std::array<char, 32768> m_RecvBuffer;
    std::vector<char> m_RecvBuffer;

    size_t m_TxBytes = 0;
    size_t m_RxBytes = 0;
    
    /** Socket is opened flag */
    bool mSocketOpened{false};
};

} // namespace Network

#endif