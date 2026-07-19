#include <iostream>
#include <chrono>
#include <thread>
#include <type_traits>

#include "Modules/cli.hpp"
// 
#include "utils/logger.hpp"
#include "version.h"

#include "app/ml/TensorRTEngine.hpp"

#include "lib/MessageLib.hpp"
#include "Modules/Modules_Lib/ModulesDefs.hpp"
#include "Modules/Modules_Lib/AdapterBase.hpp"
#include "Modules/Modules_Lib/RcBase.hpp"
#include "Modules/RcMotorController.hpp"
#include "Modules/RcCommsController.hpp"
#include "Modules/RcVisionControl.hpp"
#include "Modules/RcCarTelemetry.hpp"
#include "Modules/RcUpdater.hpp"

#include "utils/Utils.hpp"


int main(int argc, char* argv[]) {
    int ret;

    Utils::ConfigCores();
    Utils::install_crash_handler();

    Logger* logger = Logger::getLoggerInst();
    logger->log(Logger::LOG_LVL_INFO, "RC Car navigation and control V%u.%u.%u\r\n", VERSION_MAJOR, VERSION_MINOR, VERSION_BUILD);

    ret = TensorRTEngine::createEngineFile("/home/models/lanenet/lanenet.onnx", "/data/model-engines/lanenet.engine");
    if (ret != 0) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to create engine file for lanenet.onnx\r\n");
        return -1;
    }

    std::unique_ptr<Modules::MotorController  > motorController   = std::make_unique<Modules::MotorController>  (ModuleDefs::DeviceType::MotorControllerModule, "MainMotorController");
    std::unique_ptr<Modules::NetworkComms     > networkComms      = std::make_unique<Modules::NetworkComms>     (ModuleDefs::DeviceType::CommsModule, "MainNetworkComms");
    std::unique_ptr<Modules::AppCLI           > cli               = std::make_unique<Modules::AppCLI>           (ModuleDefs::DeviceType::CliModule, "AppCli");
    std::unique_ptr<Modules::VisionControls   > rcVision          = std::make_unique<Modules::VisionControls>   (ModuleDefs::DeviceType::CameraControllerModule, "CamController");
    std::unique_ptr<Modules::RcCarTelemetry   > rcTelemetry       = std::make_unique<Modules::RcCarTelemetry>   (ModuleDefs::DeviceType::TelemetryModule, "TelemetryModule");
    std::unique_ptr<Modules::Updater          > rcUpdater         = std::make_unique<Modules::Updater>          (ModuleDefs::DeviceType::UpdaterModule, "UpdaterModule");

    // Create adapters
    motorController->createAdapter<Adapter::TlmAdapter>();

    networkComms->createAdapter<Adapter::MotorAdapter>();
    networkComms->createAdapter<Adapter::CameraAdapter>();
    networkComms->createAdapter<Adapter::UpdateAdapter>();

    rcVision->createAdapter<Adapter::MotorAdapter>();
    rcVision->createAdapter<Adapter::CommsAdapter>();
    rcVision->createAdapter<Adapter::TlmAdapter>();

    cli->createAdapter<Adapter::MotorAdapter>();
    cli->createAdapter<Adapter::CommsAdapter>();
    cli->createAdapter<Adapter::CameraAdapter>();

    rcTelemetry->createAdapter<Adapter::CommsAdapter>();

    rcUpdater->createAdapter<Adapter::MotorAdapter>();
    rcUpdater->createAdapter<Adapter::CommsAdapter>();

    // Bind modules
    networkComms->moduleBind<Adapter::MotorAdapter>(motorController->getInputAdapter());
    networkComms->moduleBind<Adapter::CameraAdapter>(rcVision->getInputAdapter());
    networkComms->moduleBind<Adapter::UpdateAdapter>(rcUpdater->getInputAdapter());
    motorController->moduleBind<Adapter::TlmAdapter>(rcTelemetry->getInputAdapter());
    cli->moduleBind<Adapter::MotorAdapter>(motorController->getInputAdapter());
    cli->moduleBind<Adapter::CommsAdapter>(networkComms->getInputAdapter());
    cli->moduleBind<Adapter::CameraAdapter>(rcVision->getInputAdapter());
    rcVision->moduleBind<Adapter::CommsAdapter>(networkComms->getInputAdapter());
    rcVision->moduleBind<Adapter::MotorAdapter>(motorController->getInputAdapter());
    rcVision->moduleBind<Adapter::TlmAdapter>(rcTelemetry->getInputAdapter());
    rcTelemetry->moduleBind<Adapter::CommsAdapter>(networkComms->getInputAdapter());
    rcUpdater->moduleBind<Adapter::MotorAdapter>(motorController->getInputAdapter());
    rcUpdater->moduleBind<Adapter::CommsAdapter>(networkComms->getInputAdapter());

    // Preliminary initialization
    motorController->init();
    networkComms->init();
    rcVision->init();
    rcTelemetry->init();
    cli->init();
    rcUpdater->init();

    // Start each module
    motorController->trigger();
    networkComms->trigger();
    rcVision->trigger();
    rcTelemetry->trigger();
    cli->trigger();
    rcUpdater->trigger();

    // Connect modules to one another
    Modules::Base::joinThreads();

    return 0;
}