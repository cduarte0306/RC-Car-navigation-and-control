#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include "RcUpdater.hpp"
#include "utils/CFile.hpp"
#include "utils/logger.hpp"

#include "Modules_Lib/ModulesDefs.hpp"


static const char* tempFilePath = "/data/firmware/";
static CFile updateFile;

namespace Modules
{
Updater::Updater(ModuleDefs::DeviceType moduleID_, std::string name) :
Base(moduleID_, name), Adapter::UpdateAdapter(name), m_Buffer(10), m_UpdateStatusBuffer(25)
{
    Logger* logger = Logger::getLoggerInst();
    logger->log(Logger::LOG_LVL_INFO, "Updater object initialized\r\n");

    setInputAdapter(static_cast<Adapter::AdapterBase*>(static_cast<Adapter::UpdateAdapter*>(this)));
}


Updater::~Updater()
{
    if (updateFile.isOpen())
    {
        updateFile.close();
    }
}

/**
 * @brief Initialize the updater module
 * 
 * @return int Error code
 */
int Updater::init(void)
{
    int ret = 0;

    ret = Base::moduleRegisterCommand(PrepareForUpdate,   &Updater::initUpdateHandler              );
    ret = Base::moduleRegisterCommand(UploadFirmwareData, &Updater::uploadFirmwareDataHandler      );
    ret = Base::moduleRegisterCommand(InstallFirmware,    &Updater::installFirmwareHandler         );
    ret = Base::moduleRegisterCommand(UpdaterCleanState,  &Updater::OnUpdateServerCleanStateHandler);
    ret = Base::moduleRegisterCommand(QueryUpdateStatus,  &Updater::queryUpdateStatusHandler       );

    // Define the module payload
    Base::DefinePayloadLoc(sizeof(UpdaterReqHeader));

    // Initialize the network adapter for the internal updater server if needed
    m_updaterServerAdapter = this->CommsAdapter->OpenNetworkAdapter<NetworkProxy>
    (getName(), WebAppIface::MAIN_APP_PROXY_PORT, 0, true);
    if (!m_updaterServerAdapter)
    {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Failed to create network adapter for internal updater server\r\n");
        return -1;
    }

    m_updaterServerAdapter->setParent(this->getName());
    m_updaterServerAdapter->registerCallbacks
    (
        this,
        &Updater::OnWebAppDoorBell,
        &Updater::OnUpdateServerDoorBell
    );
    // m_updaterServerAdapter->start();
    return ret;
}

void Updater::initUpdateHandler(val_type_t val, const std::vector<char>& payload)
{
    (void)val;
    // Perform necessary steps to prepare the system for an update, such as stopping motors and closing connections
    if (payload.size() == 0)
    {
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_ERROR, "PrepareForUpdate command received with empty payload\r\n");
        Base::DoAck(false, {});
        return;
    }

    // If open, close before starting a new update
    if (updateFile.isOpen())
    {
        // Do file cleanup
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_INFO, "Closing previously opened update file\r\n");
        updateFile.close();
    }

    std::string fileName(payload.begin(), payload.end());
    m_UpdateFileInfo.fileName = fileName;

    // Strip filepath
    std::filesystem::path path(fileName);
    fileName = std::string(tempFilePath) + path.filename().string();
    updateFile.open(fileName, "wb");
    if (!updateFile.isOpen())
    {
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_ERROR, "Failed to open update file\r\n");
        Base::DoAck(false, {});
        return;
    }

    // Command motor shut off
    if (motorAdapter && motorAdapter->stopCmd() < 0)
    {
        Base::DoAck(false, {});
        return;
    }

    if (m_fwFileAdapter.get() != nullptr)
    {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Closing existing network adapter for firmware file transfers\r\n");
        m_fwFileAdapter->closeSocket();
        m_fwFileAdapter.reset();
    }

    // Open network adapter for firmware file transfers
    if (!m_fwFileAdapter)
    {
        m_fwFileAdapter = this->CommsAdapter->OpenNetworkAdapter<NetworkTcpServer, Adapter::CommsAdapter::MaxUDPPacketSize>
            (getName(), 0, 0);
        if (!m_fwFileAdapter)
        {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Failed to create network adapter for firmware file transfers\r\n");
            Base::DoAck(false, {});
            return;
        }
        m_fwFileAdapter->onConnected = [this]()
        {
            // Simulate the enter key press to show the invitation prompt
            Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Firmware file transfer network adapter connected\r\n");
        };
        m_fwFileAdapter->setParent(this->getName());
        this->CommsAdapter->startReceive(
            *m_fwFileAdapter,
            std::bind(&Updater::OnFileWrite, this, std::placeholders::_1),
            false);
    }

    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Initializing update file with name: %s\r\n", fileName.c_str());
    Base::DoAck(true, m_fwFileAdapter->getPreferredSrcPort());
}

void Updater::uploadFirmwareDataHandler(val_type_t val, const std::vector<char>& payload)
{
    (void) val;
    Base::DoAck(true, {});
    return;
}

void Updater::installFirmwareHandler(val_type_t val, const std::vector<char>& payload)
{
    (void) val;
    (void) payload;

    using json = nlohmann::json;
    json j =
    {
        {"command",   WebAppIface::INITIATE_UPDATE },
        {"file_path", updateFile.getFilePath() }
    };

    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Firmware installation status: %s\r\n", j.dump().c_str());
    std::vector<char> updateStatusVec(j.dump().begin(), j.dump().end());
    m_updaterServerAdapter->dispatchUpdater(j);
    {
        std::lock_guard<std::mutex> lock(m_CondMutex);
        m_StatusCv.notify_all();
    }
    Base::DoAck(true, {});
}

void Updater::queryUpdateStatusHandler(val_type_t val, const std::vector<char>& payload)
{
    (void) val;
    (void) payload;

    std::lock_guard<std::mutex> lock(m_StatusMutex);
    Base::DoAck(m_InstallState, m_InstallProgress);
}

void Updater::OnUpdateServerCleanStateHandler(val_type_t val, const std::vector<char>& payload)
{
    (void) val;
    (void) payload;
    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Update server received clean state command\r\n");

    m_InstallProgress = 0;  // Reset installation progress
    m_InstallState = false;  // Reset installation state

    // Perform necessary cleanup and reset the updater state
    if (updateFile.isOpen())
    {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Removing update file: %s\r\n", updateFile.getFilePath().c_str());
        updateFile.remove();
    }
    
    if (m_fwFileAdapter)
    {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Closing and removing firmware file adapter\r\n");
        m_fwFileAdapter.reset();
    }
    else
    {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Firmware file adapter is already closed\r\n");
    }
    Base::DoAck(true, {});
}

void Updater::OnFileWrite(std::vector<char>& data)
{
    Logger::getLoggerInst()->log(Logger::LOG_LVL_DEBUG, "Firmware data chunk written of size: %zu\r\n", data.size());
    std::vector<uint8_t> firmwareData(data.begin(), data.end());
    if(updateFile.write(data))
    {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_DEBUG, "Firmware data chunk written successfully\r\n");
    }
    else
    {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Failed to write firmware data chunk\r\n");
    }
}

int Updater::OnUpdateServerDoorBell(const nlohmann::json& j)
{
    try
    {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_DEBUG, "Update server received doorbell signal of size: %zu\r\n", j.size());
        if (j.contains("percentage"))
        {
            std::lock_guard<std::mutex> lock(m_StatusMutex);
            m_InstallProgress = j["percentage"].get<int>();
            m_InstallState    = j["status"].get<bool>();

            Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Update progress: %d%%\r\n", m_InstallProgress);
        }
    }
    catch (const std::exception& e)
    {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Failed to process update server doorbell signal: %s\r\n", e.what());
        return -1;
    }
    return 0;
}

int Updater::OnWebAppDoorBell(const nlohmann::json& data)
{
    Logger::getLoggerInst()->log(Logger::LOG_LVL_DEBUG, "Web app received doorbell signal of size: %zu\r\n", data.size());
    return 0;
}

void Updater::mainProc()
{
    using nlohmann::json;
    Logger* logger = Logger::getLoggerInst();
    logger->log(Logger::LOG_LVL_INFO, "Updater mainProc thread started\r\n");
    while(m_Running.load())
    {
        std::unique_lock<std::mutex> lock(m_CondMutex);
        {
            m_StatusCv.wait(lock);
            logger->log(Logger::LOG_LVL_INFO, "Updater mainProc thread awakened\r\n");
        }

        while(m_InstallProgress < 100 && m_Running.load())
        {
            // Poll the update status buffer for new updates
            json j =
            {
                {"command",   WebAppIface::READ_UPDATE_STATUS }
            };

            m_updaterServerAdapter->dispatchUpdater(j);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }
}
} // namespace Modules