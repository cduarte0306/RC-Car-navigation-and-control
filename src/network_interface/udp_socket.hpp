#ifndef UDP_SOCKET_HPP
#define UDP_SOCKET_HPP

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>


namespace Network {
class UDPSocket {
public:
    UDPSocket(int port);
    ~UDPSocket();
private:
    static constexpr int BUFFER_SIZE = 1024;
    static constexpr int MAX_CONNECTIONS = 10;
    static constexpr int TIMEOUT = 5000;
    int socket_ = -1;
};;

}


#endif