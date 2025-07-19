#ifndef RC_CAR_HPP
#define RC_CAR_HPP


#include "peripheral_driver.hpp"
#include "stdint.h"
#include "types.h"

#include "network_interface/udp_socket.hpp"
#include "gps_interface/gps_interface.hpp"

#include "navigation.hpp"

#include <thread>


class RcCar {
public:
    RcCar(void);
    ~RcCar();
    
    PeripheralCtrl* getPeripheralDev(void) const {
        return this->peripherals;
    }

    GPS::Navigation* getNavObject(void) const {
        return this->navigation;
    }
    
    void doRCMain(void);
    void joinThread(void);
private:
    enum {
        CMD_NOOP,
        CMD_FWD_DIR,
        CMD_STEER,
    };

    typedef struct __attribute__((__packed__))
    {
        uint32_t   sequence_id;
        uint16_t   msg_length;

        struct __attribute__((__packed__))
        {
            uint8_t    command;
            val_type_t data;
        } payload;
        
        uint32_t   crc_32;
    } client_req_t;

    typedef struct __attribute__((__packed__)) {
        val_type_t data;
        uint8_t state;    
    } reply_t;

private:
    
    bool isControllerConnected = false;
    bool threadCanRun          = true;
    PeripheralCtrl* peripherals = nullptr;

    Network::UDPSocket* commandServer = nullptr;
    std::thread configurationInterfaceThread;
    std::thread mainThread;

    GPS::Navigation* navigation = nullptr;

private:
    void transmitTelemetryData(void);
    void configInterfaceProcess(void);
    void rcCarThread(void);
    void processCommand(const uint8_t* pData, size_t length);
};


#endif