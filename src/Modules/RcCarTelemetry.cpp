#include "Modules/RcCarTelemetry.hpp"
#include "Devices/RegisterMap.hpp"

#include "utils/logger.hpp"
#include <nlohmann/json.hpp>

#include <chrono>
#include <thread>
#include <exception>


namespace Modules {
RcCarTelemetry::RcCarTelemetry(int moduleID, std::string name) : Modules::Base(moduleID, name), Adapter::TlmAdapter(name) {
}

int RcCarTelemetry::init(void) {
    // Initialize transmission adapter
    Logger* logger = Logger::getLoggerInst();
    m_TxAdapter = this->CommsAdapter->createNetworkAdapter(6000, "wlP1p1s0");
    if (!m_TxAdapter) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to create telemetry transmission adapter\r\n");
        return -1;
    }

    logger->log(Logger::LOG_LVL_INFO, "Telemetry module initialized\r\n");
    return 0;
}


Adapter::AdapterBase* RcCarTelemetry::getInputAdapter() {
    return static_cast<Adapter::AdapterBase*>(static_cast<Adapter::TlmAdapter*>(this));
}


/**
 * @brief Register a telemetry source
 * 
 * @param sourceName Name of the telemetry source
 * @return int Status code
 */
int RcCarTelemetry::registerTelemetrySource_(const std::string& sourceName) {
    if (sourceName.empty()) {
        return -1;
    }

    Logger* logger = Logger::getLoggerInst();
    logger->log(Logger::LOG_LVL_INFO, "Registered telemetry source: %s\r\n", sourceName.c_str());
    m_registeredSources.insert(sourceName);
    return 0;
}


/**
 * @brief Publish telemetry data
 * 
 * @param sourceName Name of the telemetry source
 * @param data Pointer to data buffer
 * @param length Size of data
 * @return int Status code
 */
int RcCarTelemetry::publishTelemetry_(const std::string& sourceName, const uint8_t* data, size_t length) {
    if (!data || length == 0) {
        return -1;
    }

    // Only allow known sources
    if (m_registeredSources.find(sourceName) == m_registeredSources.end()) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Telemetry publish from unregistered source: %s\r\n", sourceName.c_str());
        return -1;
    }

    if (!m_TxAdapter || !this->CommsAdapter) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Telemetry publish failed: Transmission adapter not initialized\r\n");
        return -1;
    }

    std::lock_guard<std::mutex> lock(m_txMutex);
    nlohmann::json telemetryJson;
    telemetryJson["source"] = sourceName;
    telemetryJson["payload"] = std::string(reinterpret_cast<const char*>(data), length);
    m_TlmBuffer.push(telemetryJson);
    return 0;
}


/**
 * @brief Main processing loop
 * 
 */
void RcCarTelemetry::mainProc() {
    Logger* logger = Logger::getLoggerInst();
    RegisterMap* regMap = RegisterMap::getInstance();
    std::string hostIP;
    while (m_Running.load()) {
        // Resolve host IP if needed
        {
            auto retVal = regMap->get<std::string>(RegisterMap::RegisterKeys::HostIP);
            if (retVal.has_value()) {
                hostIP = *retVal;
            } else {
                hostIP.clear();
            }
        }

        // Poll telemetry data from buffer
        while (!m_TlmBuffer.isEmpty()) {
            std::lock_guard<std::mutex> lock(m_txMutex);
            nlohmann::json tlmData = m_TlmBuffer.getHead();
            m_TlmBuffer.pop();
            std::string payload = tlmData.dump();
            m_TxAdapter->send(hostIP,
                reinterpret_cast<const uint8_t*>(payload.data()),
                payload.size());
        }

        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}
}
