#include "RcMotorController.hpp"

#include <iostream>
#include <chrono>
#include <string>
#include "utils/logger.hpp"
#include <nlohmann/json.hpp>


namespace Modules {
MotorController::MotorController(int moduleID_, std::string name) : Base(moduleID, name), Adapter::MotorAdapter(name) {
    Logger* logger = Logger::getLoggerInst();

    try {
        this->peripheralDriver = std::make_unique<Device::PeripheralCtrl>();
        int ret = this->peripheralDriver->doDetectDevice();
        if (ret == 0) {
            uint8_t major, minor, build;
            this->peripheralDriver->getVers(major, minor, build);
            logger->log(Logger::LOG_LVL_INFO, "PSoC Version detected: %u.%u.%u\r\n", major, minor, build);
            m_isControllerConnected = true;
        }
    } catch (const std::exception& e) {
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_ERROR, "Failed to initialize PeripheralCtrl: %s\r\n", e.what());
    }
}

MotorController::~MotorController() {
    delete this->peripheralDriver.get();
}

/**
 * @brief Serial-like interface for motor controller module
 * 
 * @param pbuf 
 * @param len 
 * @return int 
 */
int MotorController::moduleCommand_(char* pbuf, size_t len) {
    Logger* logger = Logger::getLoggerInst();
    logger->log(Logger::LOG_LVL_INFO, "Module command received\r\n");
    MotorCommand_t* cmd = reinterpret_cast<MotorCommand_t*>(pbuf);
    char* payload = reinterpret_cast<char*>(pbuf + sizeof(MotorCommand_t));
    bool ret = false;
    switch (cmd->command)
    {
    case MOTOR_CMD_SET_SPEED:
        break;

    case MOTOR_CMD_STEER:
        /* code */
        break;

    case MOTOR_CMD_DISABLE:
        /* code */
        break;

    case MOTOR_CMD_READ_DATA:
        {
        std::lock_guard<std::mutex> lock(mtrControllerMutex);
        std::memcpy(payload, &psocData, sizeof(psocData));
        }
        break;

    case MOTOR_CMD_SPI_WRITE:
        logger->log(Logger::LOG_LVL_INFO, "SPI WRITE: Reg: %u Data: %u\r\n", cmd->data_1.u8, cmd->data_2.u32);
        {
            std::lock_guard<std::mutex> lock(mtrControllerMutex);
            ret = this->peripheralDriver->xferSPI(reinterpret_cast<uint8_t*>(&cmd->data_1), sizeof(val_type_t));
        }
        if (!ret) {
            logger->log(Logger::LOG_LVL_ERROR, "SPI WRITE failed\r\n");
            return -1;
        }
        break;

    case MOTOR_CMD_SPI_READ:
        logger->log(Logger::LOG_LVL_INFO, "SPI READ: Reg: %u\r\n", cmd->data_1.u8);
        {
            std::lock_guard<std::mutex> lock(mtrControllerMutex);
            ret = this->peripheralDriver->xferSPI(reinterpret_cast<uint8_t*>(&cmd->data_1), sizeof(val_type_t));
        }
        if (!ret) {
            logger->log(Logger::LOG_LVL_ERROR, "SPI READ failed\r\n");
            return -1;
        }
        break;
    
    default:
        break;
    }

    // Implementation for processing module commands
    return 0;
}


void MotorController::pollTlmData(void) {
    if (!m_isControllerConnected) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mtrControllerMutex);
        this->peripheralDriver->readData(psocData);
    }

    nlohmann::json telemetryJson;
    telemetryJson["version_major"] = psocData.version_major.u8;
    telemetryJson["version_minor"] = psocData.version_minor.u8;
    telemetryJson["version_build"] = psocData.version_build.u8;
    telemetryJson["speed"        ] = psocData.speed.u32;
    telemetryJson["frontDistance"] = psocData.frontDistance.u32;
    telemetryJson["leftDistance" ] = psocData.leftDistance.u32;
    telemetryJson["rightDistance"] = psocData.rightDistance.u32;

    // Transmit over UDP
}


void MotorController::mainProc() {
    // Main processing loop for the motor controller
    while (true) {
        this->pollTlmData();
        
        // Process motor commands
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
}
