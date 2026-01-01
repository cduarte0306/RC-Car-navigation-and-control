#include <filesystem>
#include <fstream>
#include <iostream>
#include "RcUpdater.hpp"


namespace Modules {
Updater::Updater(int moduleID_, std::string name) : Base(moduleID_, name), Adapter::CommandAdapter(name) {
}


Updater::~Updater() {
}


/**
 * @brief Initialize the updater module
 * 
 * @return int Error code
 */
int Updater::init(void) {
    // Implementation to initialize the updater
    m_UpdatServerPort = this->CommsAdapter->createNetworkAdapter(6600, "lo", Adapter::CommsAdapter::MaxUDPPacketSize);
    if (!m_UpdatServerPort) {
        return -1;
    }

    if (!std::filesystem::exists(IMAGE_LOCATION)) {
        std::filesystem::create_directories(IMAGE_LOCATION);
    }

    return 0;
}


/**
 * @brief Handle module commands
 * 
 * @param buffer Vector containing the command data
 * @return int Error code
 */
int Updater::moduleCommand_(std::vector<char>& buffer) {
    UpdaterReq_t* req = reinterpret_cast<UpdaterReq_t*>(buffer.data());
    uint64_t payloadLen = req->header.payloadLen;
    std::vector<uint8_t> payload;
    if (payloadLen > 0) {
        payload.insert(payload.end(), buffer.begin() + sizeof(UpdaterReq_t), buffer.begin() + sizeof(UpdaterReq_t) + payloadLen);
        if (buffer.size() < sizeof(UpdaterReq_t) + payloadLen) {
            return -1; // Payload length mismatch
        }
    }
    
    switch(req->header.command) {
        case DownloadFirmwareData: {
            
            break;
        }
        
        case VerifyFirmware:
            // Command updater server to verify firmware
            break;

        case InstallFirmware:
            // Command updater server to install firmware
            break;

        default:
            // Unknown command
            return -1;
    }   
    
    // Implementation to handle module commands
    return 0;
}


void Updater::mainProc() {
    // Implementation of the main processing loop for the updater
    while (m_ThreadCanRun) {
        // Updater main loop processing
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}
}