#include <thread>
#include <mutex>
#include <atomic>
#include "RcBase.hpp"


namespace Device {
class WirelessComms : public Base, public Adapter::CommsAdapter {
public:
    WirelessComms(int moduleID, std::string name) : Base(moduleID, name) { }
    ~WirelessComms() { }

    virtual int stop(void) override {
        // Implementation to stop the motor controller
        return 0;
    }

    Adapter::AdapterBase* getInputAdapter() override {
        // WirelessComms may not implement a Motor in-adapter itself. If
        // it has an owned baseAdapter, return that; otherwise return nullptr.
        if (baseAdapter) return baseAdapter.get();
        return nullptr;
    }
protected:
    virtual void mainProc() override {
        // Main processing loop for the motor controller
        while (true) {
            // Process motor commands
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
};
};