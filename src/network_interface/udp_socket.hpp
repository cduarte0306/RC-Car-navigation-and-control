#ifndef UDP_SOCKET_HPP
#define UDP_SOCKET_HPP

#include <cstddef>
#include <unistd.h>

#include <mutex>

#include "sockets.hpp"


namespace Network {

class UDPSocket : public Sockets {
public:
    UDPSocket(int sPort, int dPort=-1);
    ~UDPSocket();

    bool transmit(uint8_t* pBuf, size_t length) override;
    void transmissionThreadHandler(void) override;

private:
    static constexpr int BUFFER_SIZE = 1024;
    static constexpr int MAX_CONNECTIONS = 10;
    static constexpr int TIMEOUT = 5000;
    int socket_ = -1;

    bool threadCanRun = true;
    struct sockaddr_in lastClientAddress;
private:
};

}

#endif