#ifndef RC_CAR_MODULES_RCBASE_HPP
#define RC_CAR_MODULES_RCBASE_HPP

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
#include <unordered_map>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#include "lib/MessageLib.hpp"
#include "Modules_Lib/AdapterBase.hpp"
#include "ModulesDefs.hpp"

#include "lib/Thread.hpp"
#include "types.h"

namespace Modules {

enum DeviceType {
    
};

class RcThread {
public:
    // Variadic template constructor that accepts any arguments 
    // that the std::thread constructor would accept.
    template<typename F, typename... Args>
    explicit RcThread(F&& f, Args&&... args)
    {
        internal_thread = std::thread(std::forward<F>(f), std::forward<Args>(args)...);
    }

    // The destructor automatically joins the thread.
    // This simplifies cleanup and prevents std::terminate being called 
    // if the thread is left joinable when the object is destroyed.
    ~RcThread();

    // Prevent copy construction and assignment for safety, as threads cannot be copied.
    RcThread(const RcThread&) = delete;
    RcThread& operator=(const RcThread&) = delete;

    // Allow moving the wrapper object.
    RcThread(RcThread&& other) noexcept;
    
    RcThread& operator=(RcThread&& other) noexcept;

    void detach(void);
    

    // Optional: provide access to the underlying native handle if needed
    std::thread::native_handle_type native_handle();

    bool joinable() const;

private:
    std::thread internal_thread; // Composition
};

class Base {
public:
    explicit Base(ModuleDefs::DeviceType moduleID_, const std::string& name, Adapter::AdapterBase* inputAdpt = nullptr);

    ~Base();

    /**
     * @brief Join all worker threads
     * 
     */
    static void joinThreads();

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
    int trigger(void);

    
    /**
     * @brief Set the Payload Offset for the module command. This can be used by adapters that need to handle a specific command format where the actual payload starts at a certain offset within the command buffer. By setting the payload offset, the adapter can ensure that when it forwards commands to the module, it correctly extracts the payload from the command buffer.
     * 
     * @param offset The byte offset within the command buffer where the payload starts
     */
    virtual void DefinePayloadLoc(size_t offset);

    /**
     * @brief Get the module name
     * 
     * @return const std::string& 
     */
    const std::string& getName(void) const;

    // Bind a module and transfer ownership. Module U must derive from Modules::Base.
    template<typename U>
    int moduleBind(std::unique_ptr<U> module)
    {
        static_assert(std::is_base_of<Adapter::AdapterBase, U>::value, "moduleBind: U must derive from Adapter::AdapterBase");
        if (!module)
        {
            return -1;
        }
        const std::string moduleName = module->getParentName();
        std::lock_guard<std::mutex> lock(mutex);
        if (m_boundAdapters.find(moduleName) != m_boundAdapters.end())
        {
            return -1;
        }

        m_boundAdapters[moduleName] = std::move(module);
        
        // Track bound adapter by parent module device type.
        Adapter::AdapterBase* adapterPtr = m_boundAdapters[moduleName].get();
        const int parentId = adapterPtr->GetParentID();
        if (parentId >= 0)
        {
            m_BoundAdaptersMap[static_cast<ModuleDefs::DeviceType>(parentId)] = adapterPtr;
        }
        if constexpr (std::is_same<U, Adapter::MotorAdapter>::value)
        {
            if (!motorAdapter)
            {
                return -1;
            }
            motorAdapter->bind(adapterPtr);
        } else if constexpr (std::is_same<U, Adapter::CameraAdapter>::value) {
            if (!CameraAdapter)
            {
                return -1;
            }
            CameraAdapter->bind(adapterPtr);
        } else if constexpr (std::is_same<U, Adapter::CommsAdapter>::value) {
            if (!CommsAdapter)
            {
                return -1;
            }
            CommsAdapter->bind(adapterPtr);
        } else if constexpr (std::is_same<U, Adapter::TlmAdapter>::value) {
            if (!TlmAdapter)
            {
                return -1;
            }
            TlmAdapter->bind(adapterPtr);
        } else if constexpr (std::is_same<U, Adapter::UpdateAdapter>::value) {
            if (!UpdateAdapter)
            {
                return -1;
            }
            UpdateAdapter->bind(adapterPtr);
        }
        else
        {
            return -1;
        }

        return 0;
    }

    template<typename U>
    void createAdapter()
    {
        static_assert(std::is_base_of<Adapter::AdapterBase, U>::value, "createAdapter: U must derive from AdapterBase");
        if constexpr (std::is_same<U, Adapter::MotorAdapter>::value)
        {
            motorAdapter = std::make_unique<Adapter::MotorAdapter>(m_name);
        } else if constexpr (std::is_same<U, Adapter::CameraAdapter>::value) {
            CameraAdapter = std::make_unique<Adapter::CameraAdapter>(m_name);
        } else if constexpr (std::is_same<U, Adapter::CommsAdapter>::value) {
            CommsAdapter = std::make_unique<Adapter::CommsAdapter>(m_name);
        } else if constexpr (std::is_same<U, Adapter::TlmAdapter>::value) {
            TlmAdapter = std::make_unique<Adapter::TlmAdapter>(m_name);
        } else if constexpr (std::is_same<U, Adapter::UpdateAdapter>::value) {
            UpdateAdapter = std::make_unique<Adapter::UpdateAdapter>(m_name);
        }
        else
        {
            throw(std::runtime_error("createAdapter: unsupported adapter type"));
            static_assert(!std::is_same<U, U>::value, "createAdapter: unsupported adapter type");
        }
    }

    // Return the adapter instance of the requested type U (transfer ownership)
    template<typename U>
    std::unique_ptr<U> getAdapter()
    {
        static_assert(std::is_base_of<Adapter::AdapterBase, U>::value, "getAdapter: U must derive from AdapterBase");
        if constexpr (std::is_same<U, Adapter::MotorAdapter>::value)
        {
            return std::move(motorAdapter);
        } else if constexpr (std::is_same<U, Adapter::CameraAdapter>::value) {
            return std::move(CameraAdapter);
        } else if constexpr (std::is_same<U, Adapter::CommsAdapter>::value) {
            return std::move(CommsAdapter);
        } else if constexpr (std::is_same<U, Adapter::TlmAdapter>::value) {
            return std::move(TlmAdapter);
        } else if constexpr (std::is_same<U, Adapter::UpdateAdapter>::value) {
            return std::move(UpdateAdapter);
        }
        else
        {
            throw(std::runtime_error("createAdapter: unsupported adapter type"));
            return nullptr;
        }
    }

    // Return a non-owning pointer to this module's "in" adapter (ancestor interface).
    // Implementations must NOT transfer ownership; they should return a pointer
    // to an adapter object owned by the module (or nullptr if none).
    virtual Adapter::AdapterBase* getInputAdapter();

    // Set/initialize this module's input adapter pointer after construction.
    // This binds inbound adapter messages to the module receive callback path.
    void setInputAdapter(Adapter::AdapterBase* inputAdpt);

    // moduleBind overload to accept non-owning adapter pointers (in-adapters).
    template<typename U>
    int moduleBind(Adapter::AdapterBase* adapter)
    {
        static_assert(std::is_base_of<Adapter::AdapterBase, U>::value, "getAdapter: U must derive from AdapterBase");
        if (!adapter)
        {
            return -1;
        }
        const std::string moduleName = adapter->getParentName();
        std::lock_guard<std::mutex> lock(mutex);
        if (m_boundAdaptersNonOwning.find(moduleName) != m_boundAdaptersNonOwning.end())
        {
            return -1;
        }
        m_boundAdaptersNonOwning[moduleName] = adapter;
        const int parentId = adapter->GetParentID();
        if (parentId >= 0)
        {
            m_BoundAdaptersMap[static_cast<ModuleDefs::DeviceType>(parentId)] = adapter;
        }

        if constexpr (std::is_same<U, Adapter::MotorAdapter>::value)
        {
            if (!motorAdapter)
            {
                return -1;
            }
            motorAdapter->bind(adapter);
        } else if constexpr (std::is_same<U, Adapter::CameraAdapter>::value) {
            if (!CameraAdapter)
            {
                return -1;
            }
            CameraAdapter->bind(adapter);
        } else if constexpr (std::is_same<U, Adapter::CommsAdapter>::value) {
            if (!CommsAdapter)
            {
                return -1;
            }
            CommsAdapter->bind(adapter);
        } else if constexpr (std::is_same<U, Adapter::TlmAdapter>::value) {
            if (!TlmAdapter)
            {
                return -1;
            }
            TlmAdapter->bind(adapter);
        } else if constexpr (std::is_same<U, Adapter::UpdateAdapter>::value) {
            if (!UpdateAdapter)
            {
                return -1;
            }
            UpdateAdapter->bind(adapter);
        }
        else
        {
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
    int attachAdapter(std::unique_ptr<Adapter::AdapterBase> adapter);

    /**
     * @brief Register a command handler for a specific command ID
     * 
     * @param commandID Command identifier to register the handler for
     * @param handler Function that takes a vector of chars as input and returns an int error code
     * @return int Error code indicating success or failure of the registration process
     */
    int moduleRegisterCommand(const int commandID, std::function<void(val_type_t, std::vector<char>&)> handler);

    int moduleRegisterCommand(const int commandID, std::function<int(val_type_t, std::vector<char>&)> handler);

    template<typename T>
    int moduleRegisterCommand(const int commandID, void (T::*handler)(val_type_t, const std::vector<char>&))
    {
        static_assert(std::is_base_of<Base, T>::value, "moduleRegisterCommand: T must derive from Base");
        T* instance = static_cast<T*>(this);
        m_CommandHandlers[commandID] = [instance, handler](val_type_t val, std::vector<char>& payload)
        {
            (instance->*handler)(val, payload);
        };
        return 0;
    }

    template<typename T>
    int moduleRegisterCommand(const int commandID, void (T::*handler)(std::vector<char>&))
    {
        static_assert(std::is_base_of<Base, T>::value, "moduleRegisterCommand: T must derive from Base");
        T* instance = static_cast<T*>(this);
        m_CommandHandlers[commandID] = [instance, handler](val_type_t val, std::vector<char>& payload)
        {
            (void)val;
            (instance->*handler)(payload);
        };
        return 0;
    }

    template<typename T>
    int moduleRegisterCommand(const int commandID, int (T::*handler)(val_type_t, const std::vector<char>&))
    {
        static_assert(std::is_base_of<Base, T>::value, "moduleRegisterCommand: T must derive from Base");
        T* instance = static_cast<T*>(this);
        m_CommandHandlers[commandID] = [instance, handler](val_type_t val, std::vector<char>& payload)
        {
            const int status = (instance->*handler)(val, payload);
            if (status < 0)
            {
                instance->DoAck(false, {});
            }
        };
        return 0;
    }

    template<typename T>
    int moduleRegisterCommand(const int commandID, int (T::*handler)(std::vector<char>&))
    {
        static_assert(std::is_base_of<Base, T>::value, "moduleRegisterCommand: T must derive from Base");
        T* instance = static_cast<T*>(this);
        m_CommandHandlers[commandID] = [instance, handler](val_type_t val, std::vector<char>& payload)
        {
            (void)val;
            const int status = (instance->*handler)(payload);
            if (status < 0)
            {
                instance->DoAck(false, {});
            }
        };
        return 0;
    }

    /**
     * @brief Dispatch an incoming command to the appropriate registered handler based on the command ID
     * 
     * @param commandID Command identifier to dispatch
     * @param val Value associated with the command (can be used for additional command parameters or flags)
     * @param payload Vector containing the command data to be passed to the handler
     * @return int Error code indicating success or failure of the command dispatch process
     */
    int dispatchCommand(const int commandID, val_type_t val, std::vector<char>& payload);

protected:
    /**
     * @brief Standard module interface command handler. This is the main entry point for commands sent to the module. It should parse the command buffer, extract the command ID and payload, and then dispatch to the appropriate handler based on the command ID.
     * 
     */
    struct BaseMsgHdr {
        uint8_t  command;  /**< Module ID */
        uint16_t seqID;    /**< Sequence ID */
        uint8_t  ack;      /**< Acknowledgment flag (0 for no ack, 1 for ack requested) */
    } __attribute__((__packed__));

    /**
     * @brief Module command 
     * 
     */
    struct ModMsgHdr  {
        uint8_t cmd;    /**< Module command  */
        val_type_t data;      /**< Module data */
        uint32_t payloadLen;
    } __attribute__((__packed__));

    virtual void mainProc() = 0;

    virtual void OnTimer(void);

    /**
     * @brief Set the sleep period for the timer
     * 
     * @param period Period in milliseconds
     */
    void SetTimerPeriod(int period);

    /**
     * @brief Thread function for handling time-based events. This thread continuously runs and calls the OnTimer function at regular intervals defined by m_SleepPeriod. It can be used by derived classes to implement any periodic tasks or time-based logic that needs to run in the background.
     * 
     */
    void timerThread(void);

    /**
     * @brief Wrapper for handling an incoming command message received from an adapter. This function is called after the command ID and payload have been extracted from the command buffer, and it is responsible for dispatching the command to the appropriate handler based on the command ID. It may also perform any necessary preprocessing or validation of the command before dispatching.
     * 
     * @param buffer Vector containing the raw command data received from an adapter
     * @param srcId The source identifier of the message (if applicable)
     * @return int Error code indicating success or failure of the message handling
     */
    int OnModuleMsgReceived_(std::vector<char>& buffer, int srcId);

    /**
     * @brief Deserialize the standard module message header from the incoming command buffer. This is typically called by adapters when they receive a command that needs to be forwarded to the module, allowing them to extract the command ID and payload from the raw command buffer before dispatching to the module's command handler. 
     * 
     * @param buffer Vector containing the raw command data received from an adapter
     * @return BaseMsgHdr Deserialized module message header
     */
    int GetBaseMsgHdr(std::vector<char>& buffer, BaseMsgHdr& hdr);

    /**
     * @brief Handle reception of command message and route it to the appropriate module.
     * 
     * @param msg Vector containing the raw command data received from an adapter
     * @return int Error code indicating success or failure of the message handling process
     */
    int dispatchToModule(std::vector<char>& msg);

    /**
     * @brief Handle an acknowledgment message received from a command handler. This function is called when a command handler sets acknowledgment data to be sent back to the adapter as part of the reply, allowing the module to process the acknowledgment data and prepare it for transmission back to the adapter.
     * 
     * @param ack Reference to the MessageAck object containing the acknowledgment data set by the command handler
     * @return int Error code indicating success or failure of the acknowledgment handling process
     */
    int DoAck(bool status, const std::vector<char>& replyData);

    template<typename T>
    int DoAck(bool status, const T& replyData)
    {
        return DoAck(status, Msg::PayloadCodec::serialize(replyData));
    }

    template<typename T>
    static bool DecodePayload(const std::vector<char>& payload, T& out)
    {
        return Msg::PayloadCodec::deserialize(payload, out);
    }

    /**
     * @brief Submit the acknowledgement back to the sender module thread
     * 
     * @param ack Reference to the MessageAck object containing the acknowledgment data to be sent back to the sender module thread
     * @return int Error code indicating success or failure of the acknowledgment submission process
     */
    int DoReply(Msg::MessageAck<std::vector<char>>& ack);

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
    const ModuleDefs::DeviceType moduleID = static_cast<ModuleDefs::DeviceType>(-1);

    /**
     * @brief Module name
     * 
     */
    std::string m_name;

    /**
     * @brief Mailbox buffers for sending and receiving messages to/from adapters
     * 
     */
    Msg::CircularBuffer<Msg::MessageCapsule<std::vector<char>>> m_MailBoxOut; // Buffer for outgoing messages to adapters

    /**
     * @brief Mailbox in circular buffer
     * 
     */
    Msg::CircularBuffer<Msg::MessageCapsule<std::vector<char>>> m_MailBoxIn;  // Buffer for incoming messages from adapters

    /**
     * @brief Bound adapters map (store adapters attached to this module)
     */
    std::unordered_map<std::string, std::unique_ptr<Adapter::AdapterBase>> m_boundAdapters; // owned adapters
    std::unordered_map<std::string, Adapter::AdapterBase*> m_boundAdaptersNonOwning;      // non-owning adapters (in-adapters)
    std::unordered_map<ModuleDefs::DeviceType, Adapter::AdapterBase*> m_BoundAdaptersMap; /**< Map of device type to adapter pointer */

    // Adapters for different device types
    std::unique_ptr<Adapter::AdapterBase   > baseAdapter    = nullptr;
    std::unique_ptr<Adapter::MotorAdapter  > motorAdapter   = nullptr;
    std::unique_ptr<Adapter::CameraAdapter > CameraAdapter  = nullptr;
    std::unique_ptr<Adapter::CommsAdapter  > CommsAdapter   = nullptr;
    std::unique_ptr<Adapter::TlmAdapter    > TlmAdapter     = nullptr;
    std::unique_ptr<Adapter::UpdateAdapter > UpdateAdapter  = nullptr;

    // Default CLI adapter on every module
    Adapter::CLIAdapter CliAdapter;

    static boost::asio::io_context io_context;
    std::mutex mutex;
    // boost::thread boostThread;

    /**
     * @brief Map of command handlers for different command IDs. Each handler is a function that takes a vector of chars 
     * as input and returns an int error code.
     * 
     */
    std::unordered_map<int, std::function<void(val_type_t, std::vector<char>&)>> m_CommandHandlers;

    std::atomic<bool> m_Running{true};  // Flag to control the main processing loop and allow for graceful shutdown

    static std::vector<std::thread> workerThreads;
private:
    std::thread thread;                                              // Main thread for the module's main processing loop
    std::thread m_TimerThread;                                       // This thread handles time-based events per object
    std::vector<Msg::MessageAck<std::vector<char>>> m_AckCache;      // Cache for command acknowledgments

    size_t m_PayloadOffset = 0; // Default payload offset is 0, can be set by derived classes if needed

    char* m_SerializedAck = nullptr;

    Adapter::AdapterBase* m_InputAdapter = nullptr; // Pointer to the input adapter (ancestor interface) for this module, if applicable    
protected:
    /**
     * @brief Handle an incoming command message received from an adapter. This function is called after the command ID and payload have been extracted from the command buffer, and it is responsible for dispatching the command to the appropriate handler based on the command ID. It may also perform any necessary preprocessing or validation of the command before dispatching.
     * 
     * @param buffer 
     */
    virtual int OnModuleMsgReceived(Msg::MessageCapsule<std::vector<char>>& capsule);

    /**
     * @brief Handle an acknowledgment message received from a command handler. This function is called when a command handler sets acknowledgment data to be sent back to the adapter as part of the reply, allowing the module to process the acknowledgment data and prepare it for transmission back to the adapter.
     * 
     * @param ack Reference to the MessageAck object containing the acknowledgment data set by the command handler
     * @return int Error code indicating success or failure of the acknowledgment handling process
     */
    virtual int OnReply(Msg::MessageAck<std::vector<char>>& ack) { return 0; }
};

} // namespace Modules

#endif // RC_CAR_MODULES_RCBASE_HPP
