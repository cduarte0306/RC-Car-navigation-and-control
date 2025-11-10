#pragma once

#include <thread>
#include <mutex>
#include <vector>
#include <atomic>
#include <map>
#include <memory>
#include <unordered_map>
#include <type_traits>
#include <string>
#include <iostream>

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
        // start the thread running this instance's mainProc and move it into the
        // shared workerThreads vector so lifetime is managed centrally.
        this->thread = std::thread(&Base::mainProc, this);
        workerThreads.emplace_back(std::move(this->thread));
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

    // Bind a module and transfer ownership. Module U must derive from Device::Base.
    template<typename U>
    int moduleBind(std::unique_ptr<U> module) {
        static_assert(std::is_base_of<Adapter::AdapterBase, U>::value, "moduleBind: U must derive from Adapter::AdapterBase");
        if (!module) return -1;
        const std::string moduleName = module->getParentName();
        std::lock_guard<std::mutex> lock(mutex);
        if (m_boundAdapters.find(moduleName) != m_boundAdapters.end()) {
            return -1; // Module already bound
        }

        // Store the adapter to keep ownership
        m_boundAdapters[moduleName] = std::move(module);

        // Now call bind on the appropriate owned adapter if present
        if (motorAdapter) {
            std::cout << "Binding MotorAdapter to " << moduleName << "\n";
            motorAdapter->bind(m_boundAdapters[moduleName].get());
        }
        if (CameraAdapter) {
            std::cout << "Binding CameraAdapter to " << moduleName << "\n";
            CameraAdapter->bind(m_boundAdapters[moduleName].get());
        }
        if (CommsAdapter) {
            std::cout << "Binding CommsAdapter to " << moduleName << "\n";
            CommsAdapter->bind(m_boundAdapters[moduleName].get());
        }

        return 0;
    }

    template<typename U>
    void createAdapter() {
        static_assert(std::is_base_of<Adapter::AdapterBase, U>::value, "createAdapter: U must derive from AdapterBase");
        if constexpr (std::is_same<U, Adapter::MotorAdapter>::value) {
            std::cout << "Creating MotorAdapter\n";
            motorAdapter = std::make_unique<Adapter::MotorAdapter>();
        } else if constexpr (std::is_same<U, Adapter::CameraAdapter>::value) {
            std::cout << "Creating CameraAdapter\n";
            CameraAdapter = std::make_unique<Adapter::CameraAdapter>();
        } else if constexpr (std::is_same<U, Adapter::CommsAdapter>::value) {
            std::cout << "Creating CommsAdapter\n";
            CommsAdapter = std::make_unique<Adapter::CommsAdapter>();
        } else {
            static_assert(!std::is_same<U, U>::value, "createAdapter: unsupported adapter type");
        }
    }

    // Return the adapter instance of the requested type U (transfer ownership)
    template<typename U>
    std::unique_ptr<U> getAdapter() {
        static_assert(std::is_base_of<Adapter::AdapterBase, U>::value, "getAdapter: U must derive from AdapterBase");
        if constexpr (std::is_same<U, Adapter::MotorAdapter>::value) {
            return std::move(motorAdapter);
        } else if constexpr (std::is_same<U, Adapter::CameraAdapter>::value) {
            return std::move(CameraAdapter);
        } else if constexpr (std::is_same<U, Adapter::CommsAdapter>::value) {
            return std::move(CommsAdapter);
        } else {
            return nullptr;
        }
    }

    // Return a non-owning pointer to this module's "in" adapter (ancestor interface).
    // Implementations must NOT transfer ownership; they should return a pointer
    // to an adapter object owned by the module (or nullptr if none).
    virtual Adapter::AdapterBase* getInputAdapter() = 0;

    // moduleBind overload to accept non-owning adapter pointers (in-adapters).
    template<typename U>
    int moduleBind(Adapter::AdapterBase* adapter) {
        static_assert(std::is_base_of<Adapter::AdapterBase, U>::value, "getAdapter: U must derive from AdapterBase");
        if (!adapter) return -1;
        const std::string moduleName = adapter->getParentName();
        std::lock_guard<std::mutex> lock(mutex);
        if (m_boundAdapters.find(moduleName) != m_boundAdapters.end() || m_boundAdaptersNonOwning.find(moduleName) != m_boundAdaptersNonOwning.end()) {
            return -1; // already bound
        }
        // store non-owning pointer
        m_boundAdaptersNonOwning[moduleName] = adapter;

        if constexpr (std::is_same<U, Adapter::MotorAdapter>::value) {
            motorAdapter->bind(adapter);
        } else if constexpr (std::is_same<U, Adapter::CameraAdapter>::value) {
            CameraAdapter->bind(adapter);
        } else if constexpr (std::is_same<U, Adapter::CommsAdapter>::value) {
            CommsAdapter->bind(adapter);
        } else {
            return -1;
        }
        return 0;
    }

    /**
     * @brief Attach an existing adapter instance to this module (transfer ownership)
     *
     * This accepts a unique_ptr to AdapterBase and stores it in the appropriate
     * concrete adapter slot if the object is of a known derived type.
     */
    int attachAdapter(std::unique_ptr<Adapter::AdapterBase> adapter) {
        if (!adapter) return -1;
        // try MotorAdapter
        if (auto p = dynamic_cast<Adapter::MotorAdapter*>(adapter.get())) {
            std::cout << "Attaching MotorAdapter\n";
            motorAdapter.reset(static_cast<Adapter::MotorAdapter*>(adapter.release()));
            return 0;
        }
        if (auto p = dynamic_cast<Adapter::CameraAdapter*>(adapter.get())) {
            std::cout << "Attaching CameraAdapter\n";
            CameraAdapter.reset(static_cast<Adapter::CameraAdapter*>(adapter.release()));
            return 0;
        }
        if (auto p = dynamic_cast<Adapter::CommsAdapter*>(adapter.get())) {
            std::cout << "Attaching CommsAdapter\n";
            CommsAdapter.reset(static_cast<Adapter::CommsAdapter*>(adapter.release()));
            return 0;
        }
        // unknown concrete adapter: keep as baseAdapter
        baseAdapter = std::move(adapter);
        return 0;
    }

    // /**
    //  * @brief Get the adapter of this module
    //  */
    // std::unique_ptr<Adapter::AdapterBase> getAdapter() {
    //     return std::move(baseAdapter);
    // }

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
     * @brief Bound adapters map (store adapters attached to this module)
     */
    std::unordered_map<std::string, std::unique_ptr<Adapter::AdapterBase>> m_boundAdapters; // owned adapters
    std::unordered_map<std::string, Adapter::AdapterBase*> m_boundAdaptersNonOwning;      // non-owning adapters (in-adapters)
    
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