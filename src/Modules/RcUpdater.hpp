#pragma once

#include "RcBase.hpp"
#include "Devices/network_interface/UdpServer.hpp"


namespace Modules {
class Updater : public Base, public Adapter::CommandAdapter {
public:
    Updater(int moduleID_, std::string name);
    ~Updater();

    /**
     * @brief Get the input adapter
     * 
     * @return Adapter::AdapterBase* Pointer to the input adapter
     */
    Adapter::AdapterBase* getInputAdapter() override {
        return static_cast<Adapter::AdapterBase*>(static_cast<Adapter::UpdaterAdapter*>(this));
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
    virtual int moduleCommand_(std::vector<char>& buffer) override;

protected:
    enum {
        DownloadFirmwareData = 1,  // Download firmware data command
        VerifyFirmware       = 2,  // Verify firmware command
        InstallFirmware      = 3   // Install firmware command
    };

    typedef struct __attribute__((__packed__))
    {
        struct __attribute__((__packed__))
        {
            uint8_t    command;
            val_type_t data;
            uint32_t   payloadLen;
        } header;
        uint8_t payload[];
    } UpdaterReq_t;

    /**
     * @brief Main processing loop for the updater module
     * 
     */
    virtual void mainProc() override;

    static constexpr char* IMAGE_LOCATION = (char*)"/data/rc_updater/";

    // Local Updaterr port interface
    std::unique_ptr<Adapter::CommsAdapter::NetworkAdapter> m_UpdatServerPort{nullptr};
};
}

#pragma endregion