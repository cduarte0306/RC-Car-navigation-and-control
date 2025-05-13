#include "rc-car.hpp"
#include "network_interface/udp_socket.hpp"
#include "peripheral_driver.hpp"
#include <functional>
#include <thread>


RcCar::RcCar( void ) {
    this->peripherals = new PeripheralCtrl();
    this->peripherals->doConfigureDevice();
    this->isControllerConnected = this->peripherals->doDetectDevice();

    this->commandServer = new Network::UDPSocket(65000);

    // Open up the configuration port. This is a LAN port that is used to configure items such as 
    // the wifi access point.
    this->configurationInterfaceThread = std::thread(&RcCar::configInterfaceProcess, this);
}


RcCar::~RcCar() {

}


void RcCar::joinThread(void) {
    this->configurationInterfaceThread.join();
    this->mainThread.join();
}


void RcCar::configInterfaceProcess(void) {
    while(threadCanRun) {
        
    }
}