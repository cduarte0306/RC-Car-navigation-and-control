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

    void Base::mailBoxThread(void) {
        while (m_ThreadCanRun) {
            Msg::MessageCapsule<char>& capsule = m_MailBoxIn.getHead();
            m_MailBoxIn.pop();
        }
    }

    int Base::doReply(Msg::MessageCapsule<char>& capsule) {
        // This function can be used by command handlers to send a reply back to the adapter after processing a command. The capsule contains the acknowledgment data that the command handler has set, and this function will handle sending that data back to the adapter as part of the reply.
        if (!capsule.isReplyPresent()) {
            return -1; // No reply data to send
        }

        std::vector<char> replyData = capsule.GetAckRaw();
        // Here you would implement the logic to send the replyData back to the adapter using the appropriate communication mechanism (e.g., through a socket, serial port, etc.)
        // This is just a placeholder for demonstration purposes
        Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Sending reply back to adapter (size: %zu)\r\n", replyData.size());
        return 0; // Success
    }

    Adapter::AdapterBase* Base::resolveCommandAdapter(uint8_t commandID) const {
        const Adapter::AdapterBase* CmdToAdapter[] = {
            motorAdapter.get(),
            CameraAdapter.get(),
            CommsAdapter.get(),
            TlmAdapter.get(),
            UpdateAdapter.get()
        };

        if (commandID >= sizeof(CmdToAdapter) / sizeof(CmdToAdapter[0])) {
            return nullptr; // Invalid command ID
        }

        auto adapter = CmdToAdapter[commandID];
        if (adapter != nullptr) {
            return const_cast<Adapter::AdapterBase*>(adapter); // Return the bound adapter for the command ID
        }
        return nullptr; // Command ID not found
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

        std::vector<char> payload(msg.begin() + sizeof(BaseMsgHdr), msg.end());

        uint16_t seqID = hdr->seqID;
        Msg::MessageCapsule<char> capsule(seqID, hdr->command, std::move(payload), static_cast<int>(moduleID));

        // Route the message
        Adapter::AdapterBase* adapter = resolveCommandAdapter(hdr->command);
        if (!adapter) {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN, "Received message with invalid command ID: %d\r\n", hdr->command);
            return -1; // Invalid command ID
        }

        // Forward the message to the appropriate adapter for handling
        int ret = adapter->dispatchCommand(capsule);

        // Notify the local adapter we're now expecting a reply
        m_InputAdapter->NotifyExpectingReply();
        return ret;
    }

    int Base::moduleRegisterCommand(const int commandID, std::function<int(val_type_t, std::vector<char>&)> handler) {
        m_CommandHandlers[commandID] = [handler](val_type_t val, std::vector<char>& payload) {
            return handler(val, payload);
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
            return handler(val, payload);
        } catch (const std::out_of_range& e) {
            // Handle the case where the commandID is not found in the map
            return -1; // Command not found
        }

        return 0; // Success
    }

    int Base::OnModuleMsgReceived(Msg::MessageCapsule<char>& capsule) {
        auto& payload = capsule.getData();
        using PayloadType = std::remove_reference_t<decltype(capsule.getData())>;
        PayloadType extraPayload;

        if (payload.size() < sizeof(ModMsgHdr)) {
            return -1;
        }

        ModMsgHdr* hdr = reinterpret_cast<ModMsgHdr*>(payload.data());
        if (hdr == nullptr) {
            return -1;
        }
        if (payload.size() > sizeof(ModMsgHdr)) {
            extraPayload.assign(payload.begin() + sizeof(ModMsgHdr), payload.end());
        } else {
            extraPayload.clear();
        }

        auto it = m_CommandHandlers.find(hdr->cmd);
        if (it == m_CommandHandlers.end()) {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Unknown command received: %d\r\n", hdr->cmd);
            return -1;
        }

        return it->second(hdr->data, extraPayload);
    }

    int Base::OnModuleMsgReceived_(std::vector<char>& buffer, int srcId) {
        // Extract the standard module message header. If payload size, then 
        // we define a payload
        int ret = 0;
        BaseMsgHdr* hdr = reinterpret_cast<BaseMsgHdr*>(buffer.data());
        if (buffer.size() < sizeof(BaseMsgHdr)) {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN, "Received message too small for header + payload (size: %zu)\r\n", buffer.size());
            return -1; // Buffer too small for header + payload
        }

        std::vector<char> payload(buffer.begin() + sizeof(BaseMsgHdr), buffer.end());

        uint16_t seqId = hdr->seqID;
        Msg::MessageCapsule<char> capsule(seqId, hdr->command, std::move(payload), srcId);

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