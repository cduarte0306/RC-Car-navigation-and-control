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

namespace Modules {
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

    ret = Base::moduleRegisterCommand(PrepareForUpdate,   &Updater::initUpdateHandler        );
    ret = Base::moduleRegisterCommand(UploadFirmwareData, &Updater::uploadFirmwareDataHandler);
    ret = Base::moduleRegisterCommand(VerifyFirmware,     &Updater::verifyFirmwareHandler    );
    ret = Base::moduleRegisterCommand(InstallFirmware,    &Updater::installFirmwareHandler   );
    ret = Base::moduleRegisterCommand(QueryUpdateStatus,  &Updater::queryUpdateStatusHandler);

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
    std::filesystem::path path(std::string(tempFilePath) + fileName);
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

    if (m_fwFileAdapter != nullptr)
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

void Updater::verifyFirmwareHandler(val_type_t val, const std::vector<char>& payload)
{
    (void) val;
    (void) payload;
    std::vector<char> fileHash;
    int ret = updateFile.GetSha256Hash(fileHash);
    if (ret < 0)
    {
        Base::DoAck(false, {});
    }

    Base::DoAck(true, fileHash);
    return;
}

void Updater::installFirmwareHandler(val_type_t val, const std::vector<char>& payload)
{
    (void) val;
    (void) payload;

    using json = nlohmann::json;
    json j =
    {
        {"command", WebAppIface::INITIATE_UPDATE}
    };

    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Firmware installation status: %s\r\n", j.dump().c_str());
    m_updaterServerAdapter->send(reinterpret_cast<const uint8_t*>(j.dump().c_str()), j.dump().size());
    j = SynchUpdaterReply();
    if (j.is_null())
    {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Failed to get updater reply\r\n");
        Base::DoAck(false, {});
    }
    else
    {
        std::vector<char> replyVec(j.dump().begin(), j.dump().end());
        Base::DoAck(true, replyVec);
    }
}

void Updater::queryUpdateStatusHandler(val_type_t val, const std::vector<char>& payload)
{
    (void) val;
    (void) payload;

    if (!m_UpdateStatusBuffer.isEmpty())
    {
        nlohmann::json updateStatus = m_UpdateStatusBuffer.getHead();
        m_UpdateStatusBuffer.pop();
        std::vector<char> updateStatusVec(updateStatus.dump().begin(), updateStatus.dump().end());
        Base::DoAck(true, updateStatusVec);
    }
    else
    {
        Base::DoAck(false, {});
    }
    return;
}

void Updater::OnFileWrite(std::vector<char>& data)
{
    Logger::getLoggerInst()->log(Logger::LOG_LVL_DEBUG, "Firmware data chunk written of size: %zu\r\n", data.size());
    typedef struct {
        uint64_t chunkID;
        uint64_t fileSize;
        uint64_t chunkSize;
    } updateInfo;

    updateInfo* info = reinterpret_cast<updateInfo*>(const_cast<char*>(data.data()));
    m_LastChunkID = info->chunkID;
    Logger::getLoggerInst()->log(Logger::LOG_LVL_DEBUG, "Received file chunk of size: %d\r\n", data.size());
    std::vector<uint8_t> firmwareData(data.begin(), data.end());
    (void) updateFile.write(data);
}

nlohmann::json Updater::SynchUpdaterReply()
{
    // Implementation for synchronously retrieving the updater reply as a JSON object
    nlohmann::json reply;
    if (!m_UpdateStatusBuffer.isEmpty())
    {
        try
        {
            reply = m_UpdateStatusBuffer.getHead(1);
            m_UpdateStatusBuffer.pop();   
        }
        catch (const std::exception& e)
        {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Failed to get update status: %s\r\n", e.what());
            return nlohmann::json();
        }
    }
    return reply;
}

int Updater::OnUpdateServerDoorBell(const std::vector<char>& data)
{
    using json = nlohmann::json;
    Logger::getLoggerInst()->log(Logger::LOG_LVL_DEBUG, "Update server received doorbell signal of size: %zu\r\n", data.size());

    // Decode as json
    json j;
    try
    {
        j = json::parse(data.begin(), data.end());
        if (j.contains("status") && j["status"] == "doorbell")
        {
            if (j.contains("updateStatus"))
            {
                m_UpdateStatusBuffer.push(j["updateStatus"]);
            }
        }
    }
    catch (const json::parse_error& e)
    {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Failed to parse JSON: %s\r\n", e.what());
        return -1;
    }
    return 0;
}

int Updater::OnWebAppDoorBell(const std::vector<char>& data)
{
    Logger::getLoggerInst()->log(Logger::LOG_LVL_DEBUG, "Web app received doorbell signal of size: %zu\r\n", data.size());
    return 0;
}

void Updater::mainProc() { }
} // namespace Modules