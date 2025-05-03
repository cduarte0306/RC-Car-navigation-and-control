#ifndef RC_CAR_HPP
#define RC_CAR_HPP


#include "peripheral_driver.hpp"
#include "stdint.h"
#include "types.h"

#include "network_interface/udp_socket.hpp"


class RcCar {
public:
    RcCar(void);
    ~RcCar();

    void doRCMain(void);
private:
    bool isControllerConnected = false;
    PeripheralCtrl* peripherals = nullptr;

    Network::UDPSocket* commandServer = nullptr;
};


#endif