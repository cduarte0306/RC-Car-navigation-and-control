#include <thread>
#include <mutex>
#include <atomic>
#include "RcBase.hpp"


namespace Device {
class WirelessComms : public Base {
public:
    WirelessComms(std::string name) : Base(name) {
        
    }

    ~WirelessComms();
protected:
};
};