#ifndef RC_CAR_HPP
#define RC_CAR_HPP


#include "peripheral_driver.hpp"
#include "stdint.h"
#include "types.h"

#include "network_interface/udp_socket.hpp"

#include <thread>


class RcCar {
public:
    RcCar(void);
    ~RcCar();

    void doRCMain(void);
    void joinThread(void);
private:
    bool isControllerConnected = false;
    bool threadCanRun          = true;
    PeripheralCtrl* peripherals = nullptr;

    Network::UDPSocket* commandServer = nullptr;
    std::thread configurationInterfaceThread;
    std::thread mainThread;

private:
    void transmitTelemetryData(void);
    void configInterfaceProcess(void);
    void rcCarThread(void);
};


#endif