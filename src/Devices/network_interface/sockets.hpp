#ifndef SOCKETS_HPP
#define SOCKETS_HPP

#include <sys/socket.h>
#include <netinet/in.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <functional>
#include <thread>
#include <optional>

#include <boost/signals2.hpp> // Added for boost signals
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <functional>

using boost::asio::ip::udp;

class Sockets {
public:
    Sockets(boost::asio::io_context& io_context, unsigned short port) : socket_(io_context) {

    }
    
    virtual ~Sockets() {

    }

    virtual bool transmit(uint8_t* pBuf, size_t length) {
        return true;   
    }

    virtual bool receive(uint8_t* pBuf, size_t length) {
        return true;
    }

    boost::signals2::signal<void(const uint8_t* data, size_t length)> onDataReceived;

    virtual void startReceive(std::function<void(const uint8_t* data, size_t length)>)  = 0;

    std::optional<std::string> findInterface(const char* name) {
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

protected:
    bool threadCanRun = true;

    int sport_ = -1;
    int dport_ = -1;

    /**
     * @brief Callback function when data is received
     * 
     */
    std::function<void(const uint8_t* data, size_t length)> dataReceivedCallback;

    udp::socket socket_;
    udp::endpoint remote_endpoint_;
    std::array<char, 32768> recv_buffer_;
};


#endif