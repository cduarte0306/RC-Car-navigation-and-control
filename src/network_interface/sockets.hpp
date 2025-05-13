#ifndef SOCKETS_HPP
#define SOCKETS_HPP

#include <sys/socket.h>
#include <netinet/in.h>

#include <functional>
#include <thread>


class Sockets {
public:
    Sockets() {

    }
    
    virtual ~Sockets() {

    }

    virtual bool transmit(uint8_t* pBuf, size_t length) {
        return true;   
    }

    bool setOnReceiveCallback(std::function<void(void)> callback=nullptr) {
        if (!callback) {
            return false;
        }
        
        this->receptionCallback = callback;
        return true;
    }

    virtual void transmissionThreadHandler(void) {
        
    }

protected:
    bool threadCanRun = true;

    int port_ = -1;
    int socketFD_ = -1;

    std::thread transmissionThread;
    std::function<void(void)> receptionCallback = nullptr;   
};


#endif