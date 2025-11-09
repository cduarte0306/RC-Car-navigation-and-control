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
protected:
    
};
} // namespace Device