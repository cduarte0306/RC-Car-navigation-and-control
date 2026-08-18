#pragma once

#include <atomic>
#include <queue>
#include <condition_variable>
#include <mutex>
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
    Adapter::AdapterBase* getInputAdapter() override
    {
        return static_cast<Adapter::AdapterBase*>(static_cast<Adapter::UpdateAdapter*>(this));
    }

    virtual int stopCmd(void) override
    {
        return stop();
    }

    /**
     * @brief Initialize the updater module
     * 
     * @return int Error code
     */
    virtual int stop(void) override
    {
        // Implementation to stop the updater
        return 0;
    }

    /**
     * @brief Initialize the updater module
     * 
     * @return int Error code
     */
    virtual int init(void) override;

protected:
    enum {
        RequestFirmwareRev = 1,  // Command to request the firmware revision from the updater module
        PrepareForUpdate,        // Command to prepare the system for an update (e.g., stop motors, close connections)
        UploadFirmwareData,      // Download firmware data command
        InstallFirmware,         // Install firmware command
        QueryUpdateStatus,       // Query update status command
        UpdaterCleanState,       // Clean state command
        UpdaterFinalize,         // Reboot command
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
     * @brief Timer callback function for the updater module, called periodically to perform time-based tasks
     * 
     */
    virtual void OnTimer(void) override;

    /**
     * @brief Handle the request for firmware revision command, which involves responding with the current firmware version of the updater module
     * 
     * @param payload Command payload containing any necessary information for requesting the firmware revision
     * @return int Error code indicating success or failure of the request
     */
    void reqRevHandler(val_type_t val, const std::vector<char>& payload);

    /**
     * @brief Handle the prepare for update command, which may involve steps like stopping motors and closing connections to ensure a safe update process
     * 
     * @param payload Command payload containing any necessary information for preparing for the update (e.g., target file name)
     * @return int Error code indicating success or failure of the preparation step
     */
    void initUpdateHandler(val_type_t val, const std::vector<char>& payload);

    /**
    * @brief Handle the upload firmware data command, which involves receiving chunks of firmware data and writing them to a temporary file for later verification and installation
    * 
    * @param payload Command payload containing the chunk of firmware data to be written
    * @return int Error code indicating success or failure of the data upload step
    */
    void uploadFirmwareDataHandler(val_type_t val, const std::vector<char>& payload);

    /**
     * @brief Handle the install firmware command, which involves replacing the existing firmware with the new verified firmware and performing any necessary cleanup or reboot steps
     * 
     * @param payload Command payload containing any necessary information for installing the firmware (e.g., installation instructions)
     * @return int Error code indicating success or failure of the installation step
     */
    void installFirmwareHandler(val_type_t val, const std::vector<char>& payload);

    /**
     * @brief Handle the query update status command, which involves responding with the current status of the firmware update process
     * 
     * @param payload Command payload containing any necessary information for querying the update status
     * @return int Error code indicating success or failure of the query step
     */
    void queryUpdateStatusHandler(val_type_t val, const std::vector<char>& payload);

    /**
     * @brief Handle the finalize update command, which involves performing any necessary finalization steps after a successful firmware installation, such as rebooting the system or cleaning up temporary files
     * 
     * @param payload Command payload containing any necessary information for finalizing the update
     * @return int Error code indicating success or failure of the finalization step
     */
    void finalizeUpdateHandler(val_type_t val, const std::vector<char>& payload);

    /**
     * @brief Handle the clean state command from the update server, which involves resetting the updater state and performing any necessary cleanup
     * 
     * @param data Vector containing any necessary information for handling the clean state command
     */
    void OnUpdateServerCleanStateHandler(val_type_t val, const std::vector<char>& payload);

    /**
     * @brief Callback function that is called when a chunk of firmware data is written to the update file
     * 
     * @param data Vector containing the chunk of firmware data that was written
     */
    void OnFileWrite(std::vector<char>& data);

    /**
     * @brief Callback function that is called when the update server receives a doorbell signal
     * 
     * @param data Vector containing the doorbell signal data
     */
    int OnUpdateServerDoorBell(const nlohmann::json& j);

    /**
     * @brief Callback function that is called when the web application receives a doorbell signal
     * 
     * @param data Vector containing the doorbell signal data
     */
    int OnWebAppDoorBell(const nlohmann::json& j);

    /**
     * @brief Perform necessary wind down operations for the updater module, such as stopping motors and closing connections
     * 
     */
    void DoCommandWindDown(void);

    /**
     * @brief Perform necessary wind up operations for the updater module, such as starting motors and re-establishing connections
     * 
     */
    void DoCommandWindUp(void);

    static constexpr char* IMAGE_LOCATION = (char*)"/data/rc_updater/";

    /**
     * @brief Progress of the firmware installation (0-100%)
     * 
     */
    int m_InstallProgress = 0;  // Progress of the firmware installation (0-100%)

    /**
     * @brief State of the firmware installation (true if installation is in progress, false otherwise)
     * 
     */
    bool m_InstallState{true};  // State of the firmware installation (true if installation is in progress, false otherwise)

    /**
     * @brief Flag indicating whether an update is currently in progress
     * 
     */
    std::atomic<bool> m_UpdateInProgress{false};

    std::atomic<bool> m_DoReset{false};  // Flag indicating whether a reset is required after the update process

    /**
     * @brief Condition variable for synchronizing access to the update status buffer
     * 
     */
    std::condition_variable m_StatusCv;  // Condition variable for synchronizing access to the update status buffer

    /**
     * @brief Mutex for synchronizing access to the condition variable
     * 
     */
    std::mutex m_CondMutex;  // Mutex for synchronizing access to the condition variable

    /**
     * @brief Mutex for synchronizing access to the update status buffer
     * 
     */
    std::mutex m_StatusMutex;  // Mutex for synchronizing access to the update status buffer

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
    
    /**
     * @brief Network adapter for handling firmware file transfers
     * 
     */
    std::unique_ptr<NetworkAdapter> m_fwFileAdapter{nullptr};

    /**
     * @brief Network adapter for handling internal updater server
     * 
     */
    std::unique_ptr<NetworkProxy> m_updaterServerAdapter{nullptr};

    /**
     * @brief Update status buffer for storing update status as a pair of success flag and progress percentage
     * 
     */
    Msg::CircularBuffer<std::pair<bool, int>> m_UpdateStatusBuffer;
};
}

#pragma endregion