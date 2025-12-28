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
#include <chrono>
#include <iostream>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#include "RcMessageLib.hpp"
#include "AdapterBase.hpp"

#include "lib/Thread.hpp"


namespace Modules {

enum DeviceType {
    WIRELESS_COMMS,
    COMMAND_CONTROLLER,
    MOTOR_CONTROLLER,
    TELEMETRY_MODULE,
    CAMERA_CONTROLLER,
    VIDEO_STREAMER,
    CLI_INTERFACE
};

class RcThread {
public:
    // Variadic template constructor that accepts any arguments 
    // that the std::thread constructor would accept.
    template<typename F, typename... Args>
    explicit RcThread(F&& f, Args&&... args) {
        internal_thread = std::thread(std::forward<F>(f), std::forward<Args>(args)...);
    }

    // The destructor automatically joins the thread.
    // This simplifies cleanup and prevents std::terminate being called 
    // if the thread is left joinable when the object is destroyed.
    ~RcThread() {
        if (internal_thread.joinable()) {
            internal_thread.join();
        }
    }

    // Prevent copy construction and assignment for safety, as threads cannot be copied.
    RcThread(const RcThread&) = delete;
    RcThread& operator=(const RcThread&) = delete;

    // Allow moving the wrapper object.
    RcThread(RcThread&& other) noexcept 
        : internal_thread(std::move(other.internal_thread)) {}
    
    RcThread& operator=(RcThread&& other) noexcept {
        if (this != &other) {
            if (internal_thread.joinable()) {
                internal_thread.join(); // Ensure existing thread is handled
            }
            internal_thread = std::move(other.internal_thread);
        }
        return *this;
    }

    // Optional: provide access to the underlying native handle if needed
    std::thread::native_handle_type native_handle() {
        return internal_thread.native_handle();
    }

    bool joinable() const {
        return internal_thread.joinable();
    }

private:
    std::thread internal_thread; // Composition
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
        Base::io_context.run();
        for (auto& worker : workerThreads) {
            if (worker.joinable()) {
                worker.join();
            }
        }

        Lib::Thread::joinAll();
    }

    /**
     * @brief Initialize the device and start its main processing loop
     * 
     * @return int 
     */
    virtual int init(void) = 0;

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
    int trigger(void) {
        this->thread = std::thread(&Base::mainProc, this);
        this->m_TimerThread = std::thread(&Base::timerThread, this);
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

    // Bind a module and transfer ownership. Module U must derive from Modules::Base.
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
            motorAdapter->bind(m_boundAdapters[moduleName].get());
        }
        if (CameraAdapter) {
            CameraAdapter->bind(m_boundAdapters[moduleName].get());
        }
        if (CommandAdapter) {
            CommandAdapter->bind(m_boundAdapters[moduleName].get());
        }
        if (CommsAdapter) {
            CommsAdapter->bind(m_boundAdapters[moduleName].get());
        }
        if (TlmAdapter) {
            TlmAdapter->bind(m_boundAdapters[moduleName].get());
        }

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
        } else if constexpr (std::is_same<U, Adapter::CommandAdapter>::value) {
            CommandAdapter = std::make_unique<Adapter::CommandAdapter>();
        } else if constexpr (std::is_same<U, Adapter::TlmAdapter>::value) {
            TlmAdapter = std::make_unique<Adapter::TlmAdapter>();
        } else {
            throw(std::runtime_error("createAdapter: unsupported adapter type"));
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
        } else if constexpr (std::is_same<U, Adapter::CommandAdapter>::value) {
            return std::move(CommandAdapter);
        } else {
            throw(std::runtime_error("createAdapter: unsupported adapter type"));
            return nullptr;
        }
    }

    // Return a non-owning pointer to this module's "in" adapter (ancestor interface).
    // Implementations must NOT transfer ownership; they should return a pointer
    // to an adapter object owned by the module (or nullptr if none).
    virtual Adapter::AdapterBase* getInputAdapter() {
        return nullptr;
    }

    // moduleBind overload to accept non-owning adapter pointers (in-adapters).
    template<typename U>
    int moduleBind(Adapter::AdapterBase* adapter) {
        static_assert(std::is_base_of<Adapter::AdapterBase, U>::value, "getAdapter: U must derive from AdapterBase");
        if (!adapter) return -1;
        const std::string moduleName = adapter->getParentName();
        std::lock_guard<std::mutex> lock(mutex);
        if (m_boundAdaptersNonOwning.find(moduleName) != m_boundAdaptersNonOwning.end()) {
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
        } else if constexpr (std::is_same<U, Adapter::CommandAdapter>::value) {
            CommandAdapter->bind(adapter);
        } else if constexpr (std::is_same<U, Adapter::TlmAdapter>::value) {
            TlmAdapter->bind(adapter);
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
            motorAdapter.reset(static_cast<Adapter::MotorAdapter*>(adapter.release()));
            return 0;
        }
        if (auto p = dynamic_cast<Adapter::CameraAdapter*>(adapter.get())) {
            CameraAdapter.reset(static_cast<Adapter::CameraAdapter*>(adapter.release()));
            return 0;
        }
        if (auto p = dynamic_cast<Adapter::CommsAdapter*>(adapter.get())) {
            CommsAdapter.reset(static_cast<Adapter::CommsAdapter*>(adapter.release()));
            return 0;
        }
        if (auto p = dynamic_cast<Adapter::CommandAdapter*>(adapter.get())) {
            CommandAdapter.reset(static_cast<Adapter::CommandAdapter*>(adapter.release()));
            return 0;
        }
        if (auto p = dynamic_cast<Adapter::TlmAdapter*>(adapter.get())) {
            TlmAdapter.reset(static_cast<Adapter::TlmAdapter*>(adapter.release()));
            return 0;
        }
        // unknown concrete adapter: keep as baseAdapter
        baseAdapter = std::move(adapter);
        return 0;
    }

protected:
    int sendMailbox(char* pbuf, size_t len);
    int recvMailbox(char* pbuf, size_t len);

    virtual void mainProc() = 0;

    virtual void OnTimer(void) {
        m_TimerCanRun = false; // If this method isn't overwritten, then exit thread
    }

    /**
     * @brief Set the sleep period for the timer
     * 
     * @param period Period in milliseconds
     */
    void setPeriod(int period) {
        m_SleepPeriod.store(period);
    }

    void timerThread(void) {
        while(m_ThreadCanRun && m_TimerCanRun) {
            OnTimer();
            std::this_thread::sleep_for(std::chrono::milliseconds(m_SleepPeriod.load()));
        }
    }

    /**
     * @brief Global thread can run flag
     * 
     */
    bool m_ThreadCanRun = true;

    /**
     * @brief Timer thread keep-alive flag
     * 
     */
    bool m_TimerCanRun{true};

    /**
     * @brief Timer thread sleep period
     * 
     */
    std::atomic<int> m_SleepPeriod{1}; // Default to 1ms

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
    std::unique_ptr<Adapter::AdapterBase    > baseAdapter    = nullptr;
    std::unique_ptr<Adapter::MotorAdapter   > motorAdapter   = nullptr;
    std::unique_ptr<Adapter::CameraAdapter  > CameraAdapter  = nullptr;
    std::unique_ptr<Adapter::CommandAdapter > CommandAdapter = nullptr;
    std::unique_ptr<Adapter::CommsAdapter   > CommsAdapter   = nullptr;
    std::unique_ptr<Adapter::TlmAdapter     > TlmAdapter     = nullptr;

    static boost::asio::io_context io_context;
    std::mutex mutex;
    std::thread thread;
    std::thread m_TimerThread;  // This thread handles time-based events per object
    // boost::thread boostThread;

    std::atomic<bool> m_Running{true};

    static std::vector<std::thread> workerThreads;
};

} // namespace Modules