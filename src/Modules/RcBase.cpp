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
}