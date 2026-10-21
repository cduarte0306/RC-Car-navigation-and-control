#pragma once
#ifndef ADAPTERUPDATER_HPP
#define ADAPTERUPDATER_HPP

#include "AdapterBase.hpp"

namespace Adapter {
    class UpdateAdapter : public AdapterBase {
    public:

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

        UpdateAdapter(std::string parentName_="");
    protected:
        virtual int bind_(AdapterBase* Adapter) override;

        void bindInterface(UpdateAdapter* adapter);
    };
}

#endif // ADAPTERUPDATER_HPP
