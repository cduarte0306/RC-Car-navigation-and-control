#pragma once

#include <thread>
#include <mutex>
#include <atomic>
#include <map>
#include <memory>
#include <unordered_map>
#include <type_traits>
#include <string>

#include "RcMessageLib.hpp"
#include "AdapterBase.hpp"

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

class Base {
public:
    enum {
        WIRELESS_COMMS,
        MOTOR_CONTROLLER,
        CAMERA_CONTROLLER
    }; 
public:
    explicit Base(int moduleID_, const std::string& name);

    ~Base();

    /**
     * @brief Join all worker threads
     * 
     */
    static void joinThreads() {
        for (auto& worker : workerThreads) {
            if (worker.joinable()) {
                worker.join();
            }
        }
    }

    /**
     * @brief Stop the device's main processing loop
     * 
     * @return int 
     */
    virtual int stop(void) = 0;

    /**
     * @brief Initialize the device and start its main processing loop
     * 
     * @return int 
     */
    int init(void) {
        // start the thread running this instance's mainProc
        this->thread = std::thread(&Base::mainProc, this);
        return 0;
    }

    /**
     * @brief Get the module name
     * 
     * @return const std::string& 
     */
    const std::string& getName(void) const {
        return m_name;
    }

    // Bind a module and transfer ownership. Module U must derive from Base<T>.
    template<typename U>
    int moduleBind(std::unique_ptr<U> module) {
        static_assert(std::is_base_of<Adapter::AdapterBase, U>::value, "moduleBind: U must derive from Base<T>");
        if (!module) return -1;
        const std::string& moduleName = module->getName();
        if (m_boundModules.find(moduleName) != m_boundModules.end()) {
            return -1; // Module already bound
        }

        module->bind();
        m_boundModules[moduleName] = std::move(module);
        return 0;
    }

    template<typename U>
    void createAdapter() {
        static_assert(std::is_base_of<Adapter::AdapterBase, U>::value, "createAdapter: U must derive from AdapterBase");
        if constexpr (std::is_same<U, Adapter::MotorAdapter>::value) {
            motorAdapter = std::make_unique<Adapter::MotorAdapter>();
        } else if constexpr (std::is_same<U, Adapter::CameraAdapter>::value) {
            CameraAdapter = std::make_unique<Adapter::CameraAdapter>();
        } else if constexpr (std::is_same<U, Adapter::CommsAdapter>::value) {
            CommsAdapter = std::make_unique<Adapter::CommsAdapter>();
        } else {
            baseAdapter = std::make_unique<Adapter::AdapterBase>();
        }
    }

protected:
    int sendMailbox(char* pbuf, size_t len);
    int recvMailbox(char* pbuf, size_t len);

    virtual void mainProc() = 0;

    /**
     * @brief Get the module ID
     * 
     */
    const int moduleID = -1;

    /**
     * @brief Module name
     * 
     */
    std::string m_name;

    /**
     * @brief Bound modules map
     * 
     */
    std::unordered_map<std::string, std::unique_ptr<Base>> m_boundModules;
    
    // Adapters for different device types
    std::unique_ptr<Adapter::AdapterBase>  baseAdapter   = nullptr;
    std::unique_ptr<Adapter::MotorAdapter > motorAdapter  = nullptr;
    std::unique_ptr<Adapter::CameraAdapter> CameraAdapter = nullptr;
    std::unique_ptr<Adapter::CommsAdapter > CommsAdapter  = nullptr;
private:
    static std::vector<std::thread> workerThreads;

    std::mutex mutex;
    std::thread thread;
};

} // namespace Device