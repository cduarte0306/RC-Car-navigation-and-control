#pragma once
#ifndef ADAPTERTLM_HPP
#define ADAPTERTLM_HPP

#include "AdapterBase.hpp"

namespace Adapter {
    class TlmAdapter : public AdapterBase {
    public:
        TlmAdapter(std::string parentName_="");

        int registerTelemetrySource(const std::string& sourceName);

        int publishTelemetry(const std::string& sourceName, const uint8_t* data, size_t length);

        int publishTelemetry(const std::string& sourceName, const std::string& payload);
        
    protected:
        std::function<int(const std::string&)> registerSourceCommand = nullptr;
        std::function<int(const std::string&, const uint8_t*, size_t)> publishTelemetryCommand = nullptr;

        virtual int bind_(AdapterBase* Adapter) final;

        void bindInterface(TlmAdapter* adapter);

        virtual int registerTelemetrySource_(const std::string& sourceName);

        virtual int publishTelemetry_(const std::string& sourceName, const uint8_t* data, size_t length);
    };
}

#endif // ADAPTERTLM_HPP
