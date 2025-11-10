#include <iostream>
#include <chrono>
#include <thread>
#include <type_traits>

#include "rc-car.hpp"
#include "cli/cli.hpp"

#include "utils/logger.hpp"
#include "version.h"

#include "rc-car-lib/RcMessageLib.hpp"
#include "rc-car-lib/AdapterBase.hpp"
#include "rc-car-lib/RcBase.hpp"
#include "rc-car-lib/RcMotorController.hpp"
#include "rc-car-lib/RcWirelessComms.hpp"


int main(int argc, char* argv[]) {
    Logger* logger = Logger::getLoggerInst();
    logger->log(Logger::LOG_LVL_INFO, "RC Car navigation and control V%u.%u.%u\r\n", VERSION_MAJOR, VERSION_MINOR, VERSION_BUILD);
    
    std::unique_ptr<Device::MotorController> motorController = std::make_unique<Device::MotorController>(Device::MOTOR_CONTROLLER, "MainMotorController");
    std::unique_ptr<Device::WirelessComms>   commsController = std::make_unique<Device::WirelessComms>  (Device::WIRELESS_COMMS, "MainWirelessComms");

    // Create adapters
    commsController->createAdapter<Adapter::MotorAdapter>();

    // Bind modules
    commsController->moduleBind<Adapter::MotorAdapter>(motorController->getInputAdapter());
    
    // Start each module
    motorController->init();
    commsController->init();

    // Connect modules to one another
    Device::Base::joinThreads();

    // RcCar rcCar;
    // AppCLI cli(rcCar);

    Device::Base::joinThreads();
    return 0;
}