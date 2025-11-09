#include <thread>
#include <mutex>
#include <atomic>
#include "RcBase.hpp"


namespace Device {
class WirelessComms : public Base {
public:
    WirelessComms(int moduleID, std::string name) : Base(moduleID, name) { }
    ~WirelessComms() { }
protected:
};
};