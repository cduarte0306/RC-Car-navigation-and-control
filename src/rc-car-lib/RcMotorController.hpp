#include <thread>
#include <mutex>
#include <atomic>

#include "RcBase.hpp"
#include "RcMessageLib.hpp"


class MotorController : public Device::Base<Device::MotorCommand> {
public:
    MotorController(std::string name) : Base(name) {}
    ~MotorController();
protected:
};