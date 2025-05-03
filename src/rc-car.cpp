#include "rc-car.hpp"
#include "network_interface/udp_socket.hpp"
#include "peripheral_driver.hpp"
#include <functional>


RcCar::RcCar( void ) {
    this->peripherals = new PeripheralCtrl();
    this->peripherals->doConfigureDevice();
    this->isControllerConnected = this->peripherals->doDetectDevice();

    this->commandServer = new Network::UDPSocket(65000);

    // Open up the configuration port. This is a LAN port that is used to configure items such as 
    // the wifi access point.
    
}


RcCar::~RcCar() {

}