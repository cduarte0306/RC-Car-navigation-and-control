#include <filesystem>
#include <fstream>
#include <iostream>
#include "RcUpdater.hpp"

#include "utils/CFile.hpp"
#include "utils/logger.hpp"


static constexpr char* tempFilePath = "/data/firmware/";
static CFile updateFile;

namespace Modules {
Updater::Updater(ModuleDefs::DeviceType moduleID_, std::string name) : Base(moduleID_, name), Adapter::UpdateAdapter(name), m_Buffer(10) {
    Logger* logger = Logger::getLoggerInst();
    logger->log(Logger::LOG_LVL_INFO, "Updater object initialized\r\n");
}


Updater::~Updater() {
    if (updateFile.isOpen()) {
        updateFile.close();
    }
}


/**
 * @brief Initialize the updater module
 * 
 * @return int Error code
 */
int Updater::init(void) {
    int ret = 0;

    ret = Base::moduleRegisterCommand(PrepareForUpdate,   &Updater::prepareForUpdateHandler);
    ret = Base::moduleRegisterCommand(UploadFirmwareData, &Updater::uploadFirmwareDataHandler);
    ret = Base::moduleRegisterCommand(VerifyFirmware,     &Updater::verifyFirmwareHandler);
    ret = Base::moduleRegisterCommand(InstallFirmware,    &Updater::installFirmwareHandler);

    // Define the module payload
    Base::DefinePayloadLoc(sizeof(UpdaterReqHeader));
    return ret;
}


int Updater::prepareForUpdateHandler(val_type_t val, const std::vector<char>& payload) {
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
    ret = motorAdapter->stopCmd();
    
    // TODO: Add command to notify update app that we are starting the update process and it can start sending firmware data
    
    return ret;
}


int Updater::uploadFirmwareDataHandler(val_type_t val, const std::vector<char>& payload) {
    (void) val;
    std::vector<uint8_t> firmwareData(payload.begin(), payload.end());
    m_Buffer.push(firmwareData);
    return 0;
}


int Updater::verifyFirmwareHandler(val_type_t val, const std::vector<char>& payload) {
    (void) val;
    return 0;
}


int Updater::installFirmwareHandler(val_type_t val, const std::vector<char>& payload) {
    (void) val;
    (void) payload;
    return 0;
}


/**
 * @brief Handle module commands
 * 
 * @param buffer Vector containing the command data
 * @return int Error code
 */
int Updater::moduleCommand_(std::vector<char>& buffer) {
    return 0;
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