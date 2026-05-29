#include "RcBase.hpp"
#include "lib/RegisterMap.hpp"
#include "utils/logger.hpp"
#include <map>

namespace Modules {
    std::vector<std::thread> Base::workerThreads;
    boost::asio::io_context Base::io_context;

    RcThread::~RcThread() {
        if (internal_thread.joinable()) {
            internal_thread.join();
        }
    }

    RcThread::RcThread(RcThread&& other) noexcept
        : internal_thread(std::move(other.internal_thread)) {
    }

    RcThread& RcThread::operator=(RcThread&& other) noexcept {
        if (this != &other) {
            if (internal_thread.joinable()) {
                internal_thread.join();
            }
            internal_thread = std::move(other.internal_thread);
        }
        return *this;
    }

    void RcThread::detach(void) {
        internal_thread.detach();
    }

    std::thread::native_handle_type RcThread::native_handle() {
        return internal_thread.native_handle();
    }

    bool RcThread::joinable() const {
        return internal_thread.joinable();
    }

    void Base::joinThreads() {
        Base::io_context.run();
        for (auto& worker : workerThreads) {
            if (worker.joinable()) {
                worker.join();
            }
        }

        Lib::Thread::joinAll();
    }

    Base::Base(ModuleDefs::DeviceType moduleID_, const std::string& name, Adapter::AdapterBase* inputAdpt) :
     m_name(name), moduleID(moduleID_), m_MailBoxOut(100), m_MailBoxIn(100), m_InputAdapter(inputAdpt) {
        auto regMap = RegisterMap::getInstance();
        if (regMap) {
            if (auto moduleMap = regMap->get<std::unordered_map<std::string, int>>(RegisterMap::RegisterKeys::ModuleMap)) {
                (*moduleMap)[name] = static_cast<int>(moduleID_);
                regMap->set(RegisterMap::RegisterKeys::ModuleMap, *moduleMap);
            } else {
                std::unordered_map<std::string, int> newModuleMap;
                newModuleMap[name] = static_cast<int>(moduleID_);
                regMap->set(RegisterMap::RegisterKeys::ModuleMap, newModuleMap);
            }
        }
    }

    Base::~Base() {
        this->m_Running.store(false);

        // stop();
        if (thread.joinable()) {
            thread.join();
        }
        for (auto& worker : workerThreads) {
            if (worker.joinable()) {
                worker.join();
            }
        }
    }

    int Base::trigger(void) {
        this->thread = std::thread(&Base::mainProc, this);
        this->m_TimerThread = std::thread(&Base::timerThread, this);

        // Move threads to the static workerThreads vector for centralized management
        workerThreads.emplace_back(std::move(this->thread));
        workerThreads.emplace_back(std::move(this->m_TimerThread));
        return 0;
    }

    void Base::DefinePayloadLoc(size_t offset) {
        m_PayloadOffset = offset;
    }

    const std::string& Base::getName(void) const {
        return m_name;
    }

    Adapter::AdapterBase* Base::getInputAdapter() {
        return m_InputAdapter;
    }

    void Base::setInputAdapter(Adapter::AdapterBase* inputAdpt) {
        m_InputAdapter = inputAdpt;
        if (!m_InputAdapter) {
            return;
        }

        Adapter::AdapterBase* adapter = m_InputAdapter;
        adapter->bindOnModuleMsgReceived([this, adapter](std::vector<char>& buffer) {
            return this->OnModuleMsgReceived_(buffer, adapter->GetParentID());
        });
        adapter->SetModDispatchCallback([this](Msg::MessageCapsule<std::vector<char>>& capsule) {
            return this->OnModuleMsgReceived(capsule);
        });
        adapter->SetReplyHandlerCallback([this](Msg::MessageAck<std::vector<char>>& ack) {
            return this->OnReply(ack);
        });
    }

    int Base::attachAdapter(std::unique_ptr<Adapter::AdapterBase> adapter) {
        if (!adapter) {
            return -1;
        }
        auto registerBoundAdapter = [this](Adapter::AdapterBase* adapterPtr) {
            if (!adapterPtr) {
                return;
            }
            const int parentId = adapterPtr->GetParentID();
            if (parentId < 0) {
                return;
            }
            std::lock_guard<std::mutex> lock(mutex);
            m_BoundAdaptersMap[static_cast<ModuleDefs::DeviceType>(parentId)] = adapterPtr;
        };

        if (auto p = dynamic_cast<Adapter::MotorAdapter*>(adapter.get())) {
            motorAdapter.reset(static_cast<Adapter::MotorAdapter*>(adapter.release()));
            registerBoundAdapter(motorAdapter.get());
            return 0;
        }
        if (auto p = dynamic_cast<Adapter::CameraAdapter*>(adapter.get())) {
            CameraAdapter.reset(static_cast<Adapter::CameraAdapter*>(adapter.release()));
            registerBoundAdapter(CameraAdapter.get());
            return 0;
        }
        if (auto p = dynamic_cast<Adapter::CommsAdapter*>(adapter.get())) {
            CommsAdapter.reset(static_cast<Adapter::CommsAdapter*>(adapter.release()));
            registerBoundAdapter(CommsAdapter.get());
            return 0;
        }
        if (auto p = dynamic_cast<Adapter::CommandAdapter*>(adapter.get())) {
            CommandAdapter.reset(static_cast<Adapter::CommandAdapter*>(adapter.release()));
            registerBoundAdapter(CommandAdapter.get());
            return 0;
        }
        if (auto p = dynamic_cast<Adapter::TlmAdapter*>(adapter.get())) {
            TlmAdapter.reset(static_cast<Adapter::TlmAdapter*>(adapter.release()));
            registerBoundAdapter(TlmAdapter.get());
            return 0;
        }
        if (auto p = dynamic_cast<Adapter::UpdateAdapter*>(adapter.get())) {
            UpdateAdapter.reset(static_cast<Adapter::UpdateAdapter*>(adapter.release()));
            registerBoundAdapter(UpdateAdapter.get());
            return 0;
        }

        baseAdapter = std::move(adapter);
        registerBoundAdapter(baseAdapter.get());
        return 0;
    }

    void Base::OnTimer(void) {
        m_TimerCanRun = false;
    }

    void Base::setPeriod(int period) {
        m_SleepPeriod.store(period);
    }

    void Base::timerThread(void) {
        while (m_ThreadCanRun && m_TimerCanRun) {
            OnTimer();
            std::this_thread::sleep_for(std::chrono::milliseconds(m_SleepPeriod.load()));
        }
    }

    int Base::dispatchToModule(std::vector<char>& msg) {
        if (msg.size() < sizeof(BaseMsgHdr)) {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN, "Received message too small for header (size: %zu)\r\n", msg.size());
            return -1;
        }

        // Encode in the message capsule
        BaseMsgHdr* hdr = reinterpret_cast<BaseMsgHdr*>(msg.data());
        if (hdr->command == static_cast<uint8_t>(ModuleDefs::DeviceType::NullModule)) {
            return 0; // Null mode messages are only for pinging. Drop immediately
        }
        uint16_t seqID = hdr->seqID;

        if (msg.size() < sizeof(BaseMsgHdr) + sizeof(ModMsgHdr)) {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN, "Received message too small for module header (size: %zu)\r\n", msg.size());
            return -1;
        }

        ModMsgHdr* modHdr  = reinterpret_cast<ModMsgHdr*>(msg.data() + sizeof(BaseMsgHdr));
        uint8_t    modCmd  = static_cast<uint8_t>(modHdr->cmd);
        val_type_t modData = modHdr->data;

        std::vector<char> payload(msg.begin() + sizeof(BaseMsgHdr) + sizeof(ModMsgHdr), msg.end());
        Msg::MessageCapsule<std::vector<char>> capsule(seqID, hdr->command, modCmd, modData, std::move(payload), static_cast<int>(moduleID));

        const std::unordered_map<
            ModuleDefs::DeviceType, Adapter::AdapterBase*
        > CmdToAdapter = {
            {ModuleDefs::DeviceType::NullModule,             nullptr},
            {ModuleDefs::DeviceType::CommsModule,            CommsAdapter.get()},
            {ModuleDefs::DeviceType::CliModule,              &CliAdapter},
            {ModuleDefs::DeviceType::TelemetryModule,        TlmAdapter.get()},
            {ModuleDefs::DeviceType::MotorControllerModule,  motorAdapter.get()},
            {ModuleDefs::DeviceType::CameraControllerModule, CameraAdapter.get()},
            {ModuleDefs::DeviceType::UpdaterModule,          UpdateAdapter.get()},
        };

        auto it = CmdToAdapter.find(static_cast<ModuleDefs::DeviceType>(hdr->command));
        if (it == CmdToAdapter.end() || it->second == nullptr) {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN, "Received message with invalid/unbound command ID: %d\r\n", hdr->command);
            return -1;
        }

        // This is a ping message. Reply immediately
        if (it->first == ModuleDefs::DeviceType::NullModule) {
            return 0;
        }

        const std::unordered_map<
            ModuleDefs::DeviceType, std::string
        > CmdToModuleName = {
            {ModuleDefs::DeviceType::NullModule,             "NullModule"},
            {ModuleDefs::DeviceType::CommsModule,            "CommsModule"},
            {ModuleDefs::DeviceType::CliModule,              "CliModule"},
            {ModuleDefs::DeviceType::TelemetryModule,        "TelemetryModule"},
            {ModuleDefs::DeviceType::MotorControllerModule,  "MotorControllerModule"},
            {ModuleDefs::DeviceType::CameraControllerModule, "CameraControllerModule"},
            {ModuleDefs::DeviceType::UpdaterModule,          "UpdaterModule"},
        };

        Logger::getLoggerInst()->log(Logger::LOG_LVL_DEBUG, "Dispatching message to module %s. SeqID: %d, Cmd: %d, Flag: %d, Payload size: %zu\r\n", 
                                    CmdToModuleName.at(static_cast<ModuleDefs::DeviceType>(hdr->command)).c_str(), 
                                    capsule.getSeqID(), capsule.getModCmd(), capsule.getDataField().u32, capsule.getData().size());

        // Forward the message to the appropriate adapter for handling
        int ret = it->second->dispatchCommand(capsule);

        // Notify the local adapter we're now expecting a reply
        return ret;
    }

    int Base::moduleRegisterCommand(const int commandID, std::function<void(val_type_t, std::vector<char>&)> handler) {
        m_CommandHandlers[commandID] = [handler](val_type_t val, std::vector<char>& payload) {
            handler(val, payload);
        };
        return 0; // Success
    }

    int Base::moduleRegisterCommand(const int commandID, std::function<int(val_type_t, std::vector<char>&)> handler) {
        m_CommandHandlers[commandID] = [this, handler](val_type_t val, std::vector<char>& payload) {
            const int status = handler(val, payload);
            if (status < 0) {
                this->DoAck(false, {});
            }
        };
        return 0; // Success
    }

    int Base::GetBaseMsgHdr(std::vector<char>& buffer, BaseMsgHdr& hdr) {
        if (buffer.size() < sizeof(BaseMsgHdr)) {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN, "Buffer too small for BaseMsgHdr (size: %zu)\r\n", buffer.size());
            return -1; // Buffer too small
        }
        std::memcpy(&hdr, buffer.data(), sizeof(BaseMsgHdr));
        return 0; // Success    
    }

    int Base::dispatchCommand(const int commandID, val_type_t val, std::vector<char>& payload) {
        try {
            auto handler = m_CommandHandlers[commandID];
            handler(val, payload);
            return 0;
        } catch (const std::out_of_range& e) {
            // Handle the case where the commandID is not found in the map
            return -1; // Command not found
        }

        return 0; // Success
    }

    int Base::DoAck(bool status, const std::vector<char>& replyData) {
        // For now, we simply log the acknowledgment data. In a real implementation, this could involve more complex processing.
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_INFO, "Acknowledgment status: %s, Reply data size: %zu\r\n", status ? "Success" : "Failure", replyData.size());
        
        // Just cache for the OnModuleMsgReceived to pick it up
        Msg::MessageAck<std::vector<char>> ack(status, replyData); // Assuming commandID and seqID are 0 for now
        m_AckCache.push_back(ack);
        return 0; // Success
    }

    int Base::OnModuleMsgReceived(Msg::MessageCapsule<std::vector<char>>& capsule) {
        auto& payload = capsule.getData();
        using PayloadType = std::remove_reference_t<decltype(capsule.getData())>;
        PayloadType extraPayload;

        const std::unordered_map<
            ModuleDefs::DeviceType, std::string
        > CmdToModuleName = {
            {ModuleDefs::DeviceType::NullModule,             "NullModule"},
            {ModuleDefs::DeviceType::CommsModule,            "CommsModule"},
            {ModuleDefs::DeviceType::CliModule,              "CliModule"},
            {ModuleDefs::DeviceType::TelemetryModule,        "TelemetryModule"},
            {ModuleDefs::DeviceType::MotorControllerModule,  "MotorControllerModule"},
            {ModuleDefs::DeviceType::CameraControllerModule, "CameraControllerModule"},
            {ModuleDefs::DeviceType::UpdaterModule,          "UpdaterModule"},
        };
        Logger::getLoggerInst()->log(Logger::LOG_LVL_DEBUG, "Receveied @module %s. Msg source: %s, SeqID: %d, Cmd: %d, Flag: %d, Payload size: %zu\r\n", 
                                    CmdToModuleName.at(static_cast<ModuleDefs::DeviceType>(moduleID)).c_str(), 
                                    CmdToModuleName.at(static_cast<ModuleDefs::DeviceType>(capsule.getSource())).c_str(),
                                    capsule.getSeqID(), capsule.getModCmd(), capsule.getDataField().u32, payload.size());

        int commandId = static_cast<int>(capsule.getModCmd());
        val_type_t commandData = capsule.getDataField();
        extraPayload = payload;

        auto it = m_CommandHandlers.find(commandId);
        // Compatibility fallback for legacy payloads that still embed ModMsgHdr in data.
        if (it == m_CommandHandlers.end()) {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Unknown command received: %d\r\n", commandId);
            return -1;
        }

        m_AckCache.clear();
        it->second(commandData, extraPayload);
        if (m_AckCache.size() && capsule.isAckRequested()) {
            Msg::MessageAck<std::vector<char>>& ack = m_AckCache.front();
            ack.mCommandID   = commandId;
            ack.mSeqID       = capsule.getSeqID();
            ack.mReplyDestID = capsule.getSource();
            DoReply(ack);
        }

        m_AckCache.clear(); // Clear any pending ack payload after command completion.

        return 0;
    }

    int Base::DoReply(Msg::MessageAck<std::vector<char>>& ack) {
        // In a real implementation, this would involve sending the acknowledgment back to the sender module thread. For now, we simply log the acknowledgment data.
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_INFO, "Submitting reply - Command ID: %d, Seq ID: %d, Status: %s, Reply data size: %zu\r\n", 
                    ack.mCommandID, ack.mSeqID, ack.GetStatus() ? "Success" : "Failure", ack.mReplyData.size());
     
        // Route the Ack to the adapter
        const std::unordered_map<
            ModuleDefs::DeviceType, Adapter::AdapterBase*
        > CmdToAdapter = {
            {ModuleDefs::DeviceType::NullModule,             nullptr},
            {ModuleDefs::DeviceType::CommsModule,            CommsAdapter.get()},
            {ModuleDefs::DeviceType::CliModule,              &CliAdapter},
            {ModuleDefs::DeviceType::TelemetryModule,        TlmAdapter.get()},
            {ModuleDefs::DeviceType::MotorControllerModule,  motorAdapter.get()},
            {ModuleDefs::DeviceType::CameraControllerModule, CameraAdapter.get()},
            {ModuleDefs::DeviceType::UpdaterModule,          UpdateAdapter.get()},
        };
        
        auto it = CmdToAdapter.find(static_cast<ModuleDefs::DeviceType>(ack.mReplyDestID));
        if (it == CmdToAdapter.end() || it->second == nullptr) {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN, "Received message with invalid/unbound command ID: %d\r\n", ack.mReplyDestID);
            return -1;
        }

        return it->second->ConnectModuleReply(ack);
    }

    int Base::OnModuleMsgReceived_(std::vector<char>& buffer, int srcId) {
        // Extract the standard module message header. If payload size, then 
        // we define a payload
        if (buffer.size() < sizeof(BaseMsgHdr)) {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN, "Received message too small for header + payload (size: %zu)\r\n", buffer.size());
            return -1; // Buffer too small for header + payload
        }

        if (buffer.size() < sizeof(BaseMsgHdr) + sizeof(ModMsgHdr)) {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN, "Received message too small for module header + payload (size: %zu)\r\n", buffer.size());
            return -1;
        }

        int ret = 0;
        BaseMsgHdr* hdr = reinterpret_cast<BaseMsgHdr*>(buffer.data());

        ModMsgHdr* modHdr = reinterpret_cast<ModMsgHdr*>(buffer.data() + sizeof(BaseMsgHdr));
        uint8_t modCmd = static_cast<uint8_t>(modHdr->cmd);
        val_type_t modData = modHdr->data;

        std::vector<char> payload(buffer.begin() + sizeof(BaseMsgHdr) + sizeof(ModMsgHdr), buffer.end());

        uint16_t seqId = hdr->seqID;
        Msg::MessageCapsule<std::vector<char>> capsule(seqId, hdr->command, modCmd, modData, std::move(payload), srcId);

         // Extract the payload based on the payload length in the header
        ret = OnModuleMsgReceived(capsule);
        if (capsule.isReplyPresent()) {
            // Clear the buffer, then refill using the data from the reply buffer
            buffer.clear();
            std::vector<char> replyData = capsule.GetAckRaw();
            buffer.insert(buffer.end(), replyData.begin(), replyData.end());
        }


        return ret;
    }
}