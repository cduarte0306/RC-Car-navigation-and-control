#pragma once

#include "RcBase.hpp"
#include "Devices/network_interface/UdpServer.hpp"

namespace Modules {
class Updater : public Base, public Adapter::UpdateAdapter {
public:
    Updater(ModuleDefs::DeviceType moduleID_, std::string name);
    ~Updater();

    /**
     * @brief Get the input adapter
     * 
     * @return Adapter::AdapterBase* Pointer to the input adapter
     */
    Adapter::AdapterBase* getInputAdapter() override {
        return static_cast<Adapter::AdapterBase*>(static_cast<Adapter::UpdateAdapter*>(this));
    }

    virtual int stopCmd(void) override {
        return stop();
    }

    /**
     * @brief Initialize the updater module
     * 
     * @return int Error code
     */
    virtual int stop(void) override {
        // Implementation to stop the updater
        return 0;
    }

    /**
     * @brief Initialize the updater module
     * 
     * @return int Error code
     */
    virtual int init(void) override;

    /**
     * @brief Get the input adapter
     * 
     * @return Adapter::AdapterBase* Pointer to the input adapter
     */
    virtual int moduleCommand_(std::vector<char>& buffer);

protected:
    enum {
        PrepareForUpdate = 1,  // Command to prepare the system for an update (e.g., stop motors, close connections)
        UploadFirmwareData,  // Download firmware data command
        VerifyFirmware      ,  // Verify firmware command
        InstallFirmware        // Install firmware command
    };

    struct UpdaterReqHeader {
        uint8_t command;      // Command identifier
        uint64_t payloadLen;  // Length of the payload following the header
    };

    struct FileInfo {
      std::string fileName;
      size_t fileLen;  
    };

    /**
     * @brief Main processing loop for the updater module
     * 
     */
    virtual void mainProc() override;

    /**
     * @brief Handle the prepare for update command, which may involve steps like stopping motors and closing connections to ensure a safe update process
     * 
     * @param payload Command payload containing any necessary information for preparing for the update (e.g., target file name)
     * @return int Error code indicating success or failure of the preparation step
     */
    int prepareForUpdateHandler(val_type_t val, const std::vector<char>& payload);

    /**
    * @brief Handle the upload firmware data command, which involves receiving chunks of firmware data and writing them to a temporary file for later verification and installation
    * 
    * @param payload Command payload containing the chunk of firmware data to be written
    * @return int Error code indicating success or failure of the data upload step
    */
    int uploadFirmwareDataHandler(val_type_t val, const std::vector<char>& payload);

    /**
     * @brief Handle the verify firmware command, which involves checking the integrity and authenticity of the received firmware data (e.g., by comparing hashes) before allowing installation
     * 
     * @param payload Command payload containing any necessary information for verifying the firmware (e.g., expected hash value)
     * @return int Error code indicating success or failure of the verification step
     */
    int verifyFirmwareHandler(val_type_t val, const std::vector<char>& payload);

    /**
     * @brief Handle the install firmware command, which involves replacing the existing firmware with the new verified firmware and performing any necessary cleanup or reboot steps
     * 
     * @param payload Command payload containing any necessary information for installing the firmware (e.g., installation instructions)
     * @return int Error code indicating success or failure of the installation step
     */
    int installFirmwareHandler(val_type_t val, const std::vector<char>& payload);

    static constexpr char* IMAGE_LOCATION = (char*)"/data/rc_updater/";

    /**
     * @brief Update file data buffer
     * 
     */
    Msg::CircularBuffer<std::vector<uint8_t>> m_Buffer;

    /**
     * @brief Update file info struct to hold metadata about the incoming firmware update
     * 
     */
    FileInfo m_UpdateFileInfo;
};
}

#pragma endregion