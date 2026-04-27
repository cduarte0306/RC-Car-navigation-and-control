#include "RcBase.hpp"

namespace Modules {
    std::vector<std::thread> Base::workerThreads;
    boost::asio::io_context Base::io_context;
    
    Base::Base(int moduleID_, const std::string& name) : m_name(name), moduleID(moduleID_) {
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

    int Base::moduleRegisterCommand(const int commandID, std::function<int(std::vector<char>&)> handler) {
        commandHandlers[commandID] = handler;
        return 0; // Success
    }

    int Base::dispatchCommand(const int commandID, std::vector<char>& buffer) {
        try {
            auto handler = commandHandlers[commandID];
            return handler(buffer);
        } catch (const std::out_of_range& e) {
            // Handle the case where the commandID is not found in the map
            return -1; // Command not found
        }

        return 0; // Success
    }
}