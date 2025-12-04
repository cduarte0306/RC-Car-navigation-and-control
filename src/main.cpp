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
// #include "Modules/RcCommsController.hpp"
#include "Modules/RcCommandAndControl.hpp"
#include "Modules/RcVisionControl.hpp"


int main(int argc, char* argv[]) {
    Logger* logger = Logger::getLoggerInst();
    logger->log(Logger::LOG_LVL_INFO, "RC Car navigation and control V%u.%u.%u\r\n", VERSION_MAJOR, VERSION_MINOR, VERSION_BUILD);

    std::unique_ptr<Modules::MotorController  > motorController   = std::make_unique<Modules::MotorController>(Modules::MOTOR_CONTROLLER, "MainMotorController");
    std::unique_ptr<Modules::CommandController> commandController = std::make_unique<Modules::CommandController>(Modules::WIRELESS_COMMS, "MainCommsController");
    std::unique_ptr<Modules::AppCLI           > cli               = std::make_unique<Modules::AppCLI>(Modules::CLI_INTERFACE, "AppCli");
    std::unique_ptr<Modules::VisionControls   > rcVision          = std::make_unique<Modules::VisionControls>(Modules::CAMERA_CONTROLLER, "CamController");

    // Create adapters
    commandController->createAdapter<Adapter::MotorAdapter>();
    commandController->createAdapter<Adapter::CameraAdapter>();
    rcVision->createAdapter<Adapter::MotorAdapter>();
    rcVision->createAdapter<Adapter::CommsAdapter>();
    cli->createAdapter<Adapter::MotorAdapter>();

    // Bind modules
    commandController->moduleBind<Adapter::MotorAdapter>(motorController->getInputAdapter());
    commandController->moduleBind<Adapter::CameraAdapter>(rcVision->getInputAdapter());
    cli->moduleBind<Adapter::MotorAdapter>(motorController->getInputAdapter());
    rcVision->moduleBind<Adapter::MotorAdapter>(motorController->getInputAdapter());
    rcVision->moduleBind<Adapter::CommsAdapter>(commandController->getInputAdapter());

    // Start each module
    motorController->trigger();
    commandController->trigger();
    rcVision->trigger();
    cli->trigger();

    // Connect modules to one another
    Modules::Base::joinThreads();

    return 0;
}