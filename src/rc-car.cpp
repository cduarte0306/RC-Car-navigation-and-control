#include "rc-car.hpp"
#include "network_interface/udp_socket.hpp"
#include "peripheral_driver.hpp"
#include <functional>
#include <thread>
#include <chrono>

#include <nlohmann/json.hpp>
#include <iostream>

#include <boost/signals2.hpp> // Added for boost signals
#include <boost/bind/bind.hpp>


#define TELEMETRY_PORT 65000
using namespace boost::placeholders;


RcCar::RcCar( void ) {
    this->peripherals = new PeripheralCtrl();
    this->peripherals->doConfigureDevice();
    
    uint32_t psocVersion = 0;
    this->isControllerConnected = this->peripherals->doDetectDevice(psocVersion);
    if (this->isControllerConnected) {
        std::cout << "PSoC version detedcted: " << ((psocVersion >> 16 ) & 0xFF) << "." 
                                                << ((psocVersion >> 16 ) & 0xFF) << "."
                                                << (psocVersion & 0xFF);
    }

    this->commandServer = new Network::UDPSocket(TELEMETRY_PORT);
    this->commandServer->onDataReceived.connect(
        boost::bind(&RcCar::processCommand, this, _1, _2)
    );

    // Open up the configuration port. This is a LAN port that is used to configure items such as 
    // the wifi access point.
    this->configurationInterfaceThread = std::thread(&RcCar::configInterfaceProcess, this);
    this->mainThread = std::thread(&RcCar::rcCarThread, this);
}


RcCar::~RcCar() {

}


// template<typename PeripheralCtrl> PeripheralCtrl* RcCar::getModule(void) {
//     return peripherals;
// }


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


/**
 * @brief Process the received command from the client
 * 
 * @param pData Pointer to the received data
 * @param length Length of the received data
 */
void RcCar::processCommand(const uint8_t* pData, size_t length) {
    if (pData == nullptr || length == 0 || length < sizeof(RcCar::client_req_t)) {
        return;
    }

    client_req_t* clientData = reinterpret_cast<client_req_t*>(const_cast<uint8_t*>(pData));
    RcCar::reply_t reply;

    switch(clientData->payload.command) {
        case CMD_NOOP:
            // No operation command, do nothing
            reply.state = true; // Success
            break;

        case CMD_FWD_DIR:
            reply.state = true; // Example state for success
            break;

        case CMD_STEER:
            // Handle steering command
            reply.state = true; // Example state for success
            break;

        default:
            // Unknown command, set error state
            reply.state = true; // Error state
            break;
    }

    // Prepare the reply
    this->commandServer->transmit(reinterpret_cast<uint8_t*>(&reply), sizeof(reply));
}