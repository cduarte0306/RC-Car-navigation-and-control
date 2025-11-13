#include <iostream>
#include <chrono>
#include <thread>
#include <type_traits>

#include "Modules/cli.hpp"
// 
#include "utils/logger.hpp"
#include "version.h"

#include "Modules/RcMessageLib.hpp"
#include "Modules/AdapterBase.hpp"
#include "Modules/RcBase.hpp"
#include "Modules/RcMotorController.hpp"
#include "Modules/RcWirelessComms.hpp"


int main(int argc, char* argv[]) {
    Logger* logger = Logger::getLoggerInst();
    logger->log(Logger::LOG_LVL_INFO, "RC Car navigation and control V%u.%u.%u\r\n", VERSION_MAJOR, VERSION_MINOR, VERSION_BUILD);
    
    std::unique_ptr<Modules::MotorController> motorController = std::make_unique<Modules::MotorController>(Modules::MOTOR_CONTROLLER, "MainMotorController");
    std::unique_ptr<Modules::WirelessComms>   commsController = std::make_unique<Modules::WirelessComms>  (Modules::WIRELESS_COMMS, "MainWirelessComms");
    std::unique_ptr<Modules::AppCLI>          cli             = std::make_unique<Modules::AppCLI>(Modules::CLI_INTERFACE, "AppCli");

    // Create adapters
    commsController->createAdapter<Adapter::MotorAdapter>();
    cli->createAdapter<Adapter::MotorAdapter>();

    // Bind modules
    commsController->moduleBind<Adapter::MotorAdapter>(motorController->getInputAdapter());
    cli->moduleBind<Adapter::MotorAdapter>(motorController->getInputAdapter());
    
    // Start each module
    motorController->init();
    commsController->init();
    cli->init();

    // Connect modules to one another
    Modules::Base::joinThreads();

    return 0;
}