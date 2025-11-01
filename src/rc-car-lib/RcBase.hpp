#include <thread>
#include <mutex>
#include <atomic>

#include "RcMessageLib.hpp"


using namespace std;

namespace Device {

enum DeviceType {
    WIRELESS_COMMS,
    MOTOR_CONTROLLER,
    CAMERA_CONTROLLER
};

struct MotorCommand {
    int srcID;
    int speed;
    int direction;
    bool brake;
};

struct CameraCommand {
    int srcID;
    int panAngle;
    int tiltAngle;
    bool captureImage;
};

template<typename T>
class Base {
public:
    enum {
        WIRELESS_COMMS,
        MOTOR_CONTROLLER,
        CAMERA_CONTROLLER   
    }; 
public:
    explicit Base(const std::string& name) : m_name(name){ };

    ~Base() {
        
    };

    int init(void) {
        this->thread = std::thread(Base::mainProc);
    }

    virtual int stop(void) = 0;
    
protected:
    // Module name
    std::string m_name;

    int sendMailbox(char* pbuf, size_t len);
    int recvMailbox(char* pbuf, size_t len);

    virtual void mainProc() = 0;

    // Circular buffer
    Msg::CircularBuffer<std::unique_ptr<Msg::MessageCapsule<T>>> mailboxBuffer{100};
private:
    std::mutex mutex;
    std::thread thread;
};


};