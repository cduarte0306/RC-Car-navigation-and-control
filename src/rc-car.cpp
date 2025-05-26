#include "rc-car.hpp"
#include "network_interface/udp_socket.hpp"
#include "peripheral_driver.hpp"
#include <functional>
#include <thread>
#include <chrono>

#include <nlohmann/json.hpp>


RcCar::RcCar( void ) {
    this->peripherals = new PeripheralCtrl();
    this->peripherals->doConfigureDevice();
    this->isControllerConnected = this->peripherals->doDetectDevice();

    this->commandServer = new Network::UDPSocket(65000);
    this->commandServer->setOnReceiveCallback(nullptr);

    // Open up the configuration port. This is a LAN port that is used to configure items such as 
    // the wifi access point.
    this->configurationInterfaceThread = std::thread(&RcCar::configInterfaceProcess, this);
    this->mainThread = std::thread(&RcCar::rcCarThread, this);
    
}


RcCar::~RcCar() {

}


void RcCar::joinThread(void) {
    this->configurationInterfaceThread.join();
    this->mainThread.join();
}


/**
 * @brief Transmit telemetry data to the command server
 * 
 */
void RcCar::transmitTelemetryData(void) {
    if (this->isControllerConnected) {
        PeripheralCtrl::psocDataStruct data;
        this->peripherals->readData(data);

        nlohmann::json telemetryJson;
        telemetryJson["version_major"] = data.version_major.u8;
        telemetryJson["version_minor"] = data.version_minor.u8;
        telemetryJson["version_build"] = data.version_build.u8;
        telemetryJson["speed"        ] = data.speed.u32;
        telemetryJson["frontDistance"] = data.frontDistance.u32;
        telemetryJson["leftDistance" ] = data.leftDistance.u32;
        telemetryJson["rightDistance"] = data.rightDistance.u32;

        std::string jsonString = telemetryJson.dump();
        this->commandServer->transmit((uint8_t*)jsonString.c_str(), jsonString.size());
    }
}


/**
 * @brief Process the configuration interface in a separate thread
 * 
 */
void RcCar::configInterfaceProcess(void) {
    while(threadCanRun) {
        this->transmitTelemetryData();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}


/**
 * @brief Main thread for the RC car operations
 * 
 */
void RcCar::rcCarThread(void) {
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}