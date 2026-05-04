#include "RcBase.hpp"
#include "lib/RegisterMap.hpp"
#include "utils/logger.hpp"
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

    Base::Base(ModuleDefs::DeviceType moduleID_, const std::string& name) : m_name(name), moduleID(moduleID_) {
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
        workerThreads.emplace_back(std::move(this->thread));
        return 0;
    }

    void Base::DefinePayloadLoc(size_t offset) {
        m_PayloadOffset = offset;
    }

    std::vector<char> Base::getPayload(std::vector<char>& buffer) {
        if (buffer.size() <= m_PayloadOffset) {
            return {};
        }
        return std::vector<char>(buffer.begin() + m_PayloadOffset, buffer.end());
    }

    const std::string& Base::getName(void) const {
        return m_name;
    }

    Adapter::AdapterBase* Base::getInputAdapter() {
        return nullptr;
    }

    int Base::attachAdapter(std::unique_ptr<Adapter::AdapterBase> adapter) {
        if (!adapter) {
            return -1;
        }
        if (auto p = dynamic_cast<Adapter::MotorAdapter*>(adapter.get())) {
            motorAdapter.reset(static_cast<Adapter::MotorAdapter*>(adapter.release()));
            return 0;
        }
        if (auto p = dynamic_cast<Adapter::CameraAdapter*>(adapter.get())) {
            CameraAdapter.reset(static_cast<Adapter::CameraAdapter*>(adapter.release()));
            return 0;
        }
        if (auto p = dynamic_cast<Adapter::CommsAdapter*>(adapter.get())) {
            CommsAdapter.reset(static_cast<Adapter::CommsAdapter*>(adapter.release()));
            return 0;
        }
        if (auto p = dynamic_cast<Adapter::CommandAdapter*>(adapter.get())) {
            CommandAdapter.reset(static_cast<Adapter::CommandAdapter*>(adapter.release()));
            return 0;
        }
        if (auto p = dynamic_cast<Adapter::TlmAdapter*>(adapter.get())) {
            TlmAdapter.reset(static_cast<Adapter::TlmAdapter*>(adapter.release()));
            return 0;
        }
        if (auto p = dynamic_cast<Adapter::UpdateAdapter*>(adapter.get())) {
            UpdateAdapter.reset(static_cast<Adapter::UpdateAdapter*>(adapter.release()));
            return 0;
        }

        baseAdapter = std::move(adapter);
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

    int Base::moduleRegisterCommand(const int commandID, std::function<int(val_type_t, std::vector<char>&)> handler) {
        m_CommandHandlers[commandID] = [handler](val_type_t val, std::vector<char>& payload) {
            return handler(val, payload);
        };
        return 0; // Success
    }
    int Base::GetModMsgHdr(std::vector<char>& buffer, ModMsgHdr& hdr) {
        if (buffer.size() < sizeof(ModMsgHdr)) {
            return -1; // Buffer too small
        }
        std::memcpy(&hdr, buffer.data(), sizeof(ModMsgHdr));
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

    int Base::OnModuleMsgReceived_(std::vector<char>& buffer, int srcId) {
        // Extract the standard module message header. If payload size, then 
        // we define a payload
        int ret = 0;
        ModMsgHdr* hdr = reinterpret_cast<ModMsgHdr*>(buffer.data());
        if (buffer.size() < sizeof(ModMsgHdr) + hdr->payloadLen) {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN, "Received message too small for header + payload (size: %zu, need at least %zu)\r\n", buffer.size(), sizeof(ModMsgHdr) + hdr->payloadLen);
            return -1; // Buffer too small for header + payload
        }

        std::unique_ptr<Msg::MessageCapsule<char>> capsule = 
            std::make_unique<Msg::MessageCapsule<char>>(hdr->command, getPayload(buffer), srcId);

         // Extract the payload based on the payload length in the header
        ret = OnModuleMsgReceived(*capsule);
        if (capsule->isReplyPresent()) {
            // Clear the buffer, then refill using the data from the reply buffer
            buffer.clear();
            std::vector<char> replyData = capsule->GetAck();
            buffer.insert(buffer.end(), replyData.begin(), replyData.end());
        }

        return ret;
    }
}