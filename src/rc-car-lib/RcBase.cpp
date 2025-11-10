#include "RcBase.hpp"

namespace Device {
    std::vector<std::thread> Base::workerThreads;
    
    Base::Base(int moduleID_, const std::string& name) : m_name(name), moduleID(moduleID_) {
    }

    Base::~Base() {
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