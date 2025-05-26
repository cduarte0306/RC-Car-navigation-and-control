#ifndef SOCKETS_HPP
#define SOCKETS_HPP

#include <sys/socket.h>
#include <netinet/in.h>

#include <functional>
#include <thread>

#include <boost/signals2.hpp> // Added for boost signals


class Sockets {
public:
    Sockets() {

    }
    
    virtual ~Sockets() {

    }

    virtual bool transmit(uint8_t* pBuf, size_t length) {
        return true;   
    }

    bool setOnReceiveCallback(std::function<void(const uint8_t* data, size_t length)> callback=nullptr) {
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

    int sport_ = -1;
    int dport_ = -1;
    int socketFD_ = -1;

    std::thread transmissionThread;
    std::function<void(const uint8_t* data, size_t length)> receptionCallback = nullptr;
    
    // Signal to notify when data is received
    boost::signals2::signal<void(const uint8_t* data, size_t length)> onDataReceived;
};


#endif