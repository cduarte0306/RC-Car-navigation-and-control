#include <filesystem>
#include <fstream>
#include <iostream>
#include "RcUpdater.hpp"

#include "utils/CFile.hpp"
#include "utils/logger.hpp"


static constexpr char* tempFilePath = "/data/firmware/";
static CFile updateFile;

namespace Modules {
Updater::Updater(int moduleID_, std::string name) : Base(moduleID_, name), Adapter::UpdateAdapter(name), m_Buffer(10) {
    Logger* logger = Logger::getLoggerInst();
    logger->log(Logger::LOG_LVL_INFO, "Updater object initialized\r\n");
}


Updater::~Updater() {
}


/**
 * @brief Initialize the updater module
 * 
 * @return int Error code
 */
int Updater::init(void) {
    moduleRegisterCommand(PrepareForUpdate, &Updater::prepareForUpdateHandler);
    moduleRegisterCommand(UploadFirmwareData, &Updater::uploadFirmwareDataHandler);
    moduleRegisterCommand(VerifyFirmware, &Updater::verifyFirmwareHandler);
    moduleRegisterCommand(InstallFirmware, &Updater::installFirmwareHandler);
    return 0;
}


int Updater::prepareForUpdateHandler(const std::vector<char>& payload) {
    // Perform necessary steps to prepare the system for an update, such as stopping motors and closing connections
    if (payload.size() == 0) {
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_ERROR, "PrepareForUpdate command received with empty payload\r\n");
        return -1;
    }

    int ret = 0;

    std::string fileName(payload.begin(), payload.end());
    m_UpdateFileInfo.fileName = fileName;
    
    updateFile.open(std::string(tempFilePath) + fileName, "wb");
    if (!updateFile.isOpen()) {
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_ERROR, "Failed to open update file\r\n");
        return -1;
    }

    // Command motor shut off
    ret = motorAdapter->CommandMotorState(false);
    return ret;
}


int Updater::uploadFirmwareDataHandler(const std::vector<char>& payload) {
    std::vector<uint8_t> firmwareData(payload.begin(), payload.end());
    m_Buffer.push(firmwareData);
    m_DataReceived.notify_one();
    return 0;
}


int Updater::verifyFirmwareHandler(const std::vector<char>& payload) {
    return 0;
}


int Updater::installFirmwareHandler(const std::vector<char>& payload) {
    return 0;
}


/**
 * @brief Handle module commands
 * 
 * @param buffer Vector containing the command data
 * @return int Error code
 */
int Updater::moduleCommand_(std::vector<char>& buffer) {
    UpdaterReqHeader* req = reinterpret_cast<UpdaterReqHeader*>(buffer.data());
    uint64_t payloadLen = req->payloadLen;
    std::vector<char> payload;
    int ret = 0;
    if (payloadLen > 0) {
        payload.insert(payload.end(), buffer.begin() + sizeof(UpdaterReqHeader), buffer.begin() + sizeof(UpdaterReqHeader) + payloadLen);
        if (buffer.size() < sizeof(UpdaterReqHeader) + payloadLen) {
            return -1; // Payload length mismatch
        }
    }

    switch(req->command) {
        case PrepareForUpdate:   ret = prepareForUpdateHandler  (payload); break;
        case UploadFirmwareData: ret = uploadFirmwareDataHandler(payload); break;
        case VerifyFirmware:     ret = verifyFirmwareHandler    (payload); break;
        case InstallFirmware:    ret = installFirmwareHandler   (payload); break;

        default:
            // Unknown command
            return -1;
    }   
    
    // Implementation to handle module commands
    return ret;
}


void Updater::mainProc() {
    // Implementation of the main processing loop for the updater
    static constexpr char* tempFilePath = "/data/rc_updater/";
    
    if (std::filesystem::exists(tempFilePath)) {
        std::filesystem::remove_all(tempFilePath);
    } else if (!std::filesystem::create_directories(tempFilePath)) {
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_ERROR, "Failed to create updater temp directory\r\n");
        return;
    }

    
    while (m_ThreadCanRun) {
        {
            std::unique_lock<std::mutex> lk(m_UpdateSignalMutex);
            m_DataReceived.wait_for(lk, std::chrono::milliseconds(1), [this] {
                return !m_Buffer.isEmpty() || !m_ThreadCanRun;
            });
        }

        if (m_Buffer.isEmpty()) {
            continue; // No data received
        }

        // Data received. Write to file and signal updater server if needed
        std::vector<uint8_t>& data = m_Buffer.getHead();
        if (!updateFile.isOpen()) {
            Logger* logger = Logger::getLoggerInst();
            logger->log(Logger::LOG_LVL_ERROR, "Failed to open update file\r\n");
            continue;
        }

        // Write the received data to the file
        
        m_Buffer.pop();
    }
}
}