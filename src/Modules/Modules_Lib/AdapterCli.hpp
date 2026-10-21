#pragma once
#ifndef ADAPTERCLI_HPP
#define ADAPTERCLI_HPP

#include "AdapterBase.hpp"

namespace Adapter {
    class CLIAdapter : public AdapterBase {
    public:
        CLIAdapter(std::string parentName_="");

    protected:
        std::function<std::string()> readStats = nullptr;
        std::function<int(bool)>     setStreamingStateCommand = nullptr;

        virtual int bind_(AdapterBase* Adapter) override;

        void bindInterface(CLIAdapter* adapter);

        virtual std::string readModuleStats_(void);
    };
}

#endif // ADAPTERCLI_HPP
