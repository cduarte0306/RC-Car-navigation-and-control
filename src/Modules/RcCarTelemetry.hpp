#ifndef RCCARTELEMETRY_HPP
#define RCCARTELEMETRY_HPP

#include <unordered_set>
#include <string>
#include <mutex>
#include <nlohmann/json.hpp>
#include "Modules/RcBase.hpp"
#include "RcMessageLib.hpp"


namespace Modules {
class RcCarTelemetry : public Modules::Base, public Adapter::TlmAdapter {
public:
    RcCarTelemetry(int moduleID, std::string name);
    ~RcCarTelemetry() {}

    int init(void) override;
    int stop(void) override {
        return 0;
    }

    Adapter::AdapterBase* getInputAdapter() override;
protected:
    void mainProc() override;

    int registerTelemetrySource_(const std::string& sourceName) override;
    int publishTelemetry_(const std::string& sourceName, const uint8_t* data, size_t length) override;

    Msg::CircularBuffer<nlohmann::json> m_TlmBuffer{400};
    std::mutex m_txMutex;
    std::unique_ptr<Adapter::CommsAdapter::NetworkAdapter> m_TxAdapter{nullptr};
    std::unordered_set<std::string> m_registeredSources;
};
}

#endif