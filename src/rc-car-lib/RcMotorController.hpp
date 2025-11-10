#include <thread>
#include <mutex>
#include <atomic>
#include "AdapterBase.hpp"
#include "RcBase.hpp"
#include "RcMessageLib.hpp"


namespace Device {
class MotorController : public Device::Base, public Adapter::MotorAdapter {
public:
    MotorController(int moduleID_, std::string name);
    ~MotorController();

    virtual int stop(void) override {
        // Implementation to stop the motor controller
        return 0;
    }

    Adapter::AdapterBase* getInputAdapter() override {
        return static_cast<Adapter::AdapterBase*>(static_cast<Adapter::MotorAdapter*>(this));
    }


protected:
    virtual void mainProc() override;
};
} // namespace Device