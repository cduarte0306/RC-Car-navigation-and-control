#pragma once
#ifndef ADAPTERBASE_HPP
#define ADAPTERBASE_HPP

#include <map>
#include <string>
#include <unordered_map>
#include <list>
#include <utility>
#include <atomic>
#include <functional>
#include <iostream>
#include <memory>
#include <thread>
#include <condition_variable>

#include "ModulesDefs.hpp"

#include "lib/MessageLib.hpp"
#include "NetworkTypes.hpp"

namespace Adapter {
    class AdapterBase {
    public:    
        AdapterBase(ModuleDefs::AdapterId id, std::string parentName_="");
        virtual ~AdapterBase();

        /**
         * @brief Bind this adapter to a module adapter. This allows the module and adapter to communicate by forwarding calls between them. The module will call the adapter's moduleCommand, which will forward to the bound module adapter's implementation, and the module adapter can call this adapter's moduleCommand to forward commands back to the module.
         * 
         * @param Adapter Pointer to the adapter to bind
         * @return int Status code (0 for success, -1 for failure)
         */
        int bind(AdapterBase* Adapter);

        /**
         * @brief Bind a command dispatch function to this adapter. This allows the adapter to forward received commands to the module's dispatchCommand implementation.
         * 
         * @param dispatchFunc The dispatch function to bind, which should match the signature of dispatchCommand (taking a MessageCapsule and returning an int)
         * @return int Status code (0 for success, -1 for failure)
         */
        int bindCommandDispatch(std::function<int(Msg::MessageCapsule<std::vector<char>>& capsule)> dispatchFunc);

        /**
         * @brief Bind module message reception callback. This allows the adapter to forward 
         * received messages to the module's OnModuleMsgReceived implementation.
         * 
         * @return std::string Stats information as a string
         */
        int bindOnModuleMsgReceived(std::function<int(std::vector<char>& buffer)> onMsgReceivedFunc);

        /**
         * @brief Default implementation of readStats. Adapters that need to provide custom handling should override this.
         * 
         * @return std::string Stats information as a string
         */
        virtual std::string readStats();

        /**
         * @brief Bind an adapter to this adapter. Must be implemented by derived classes.
         * 
         * @param Adapter Pointer to the adapter to bind
         * @return int Status code
         */
        virtual int bind_(AdapterBase* Adapter)= 0;

        /**
         * @brief Default implementation of moduleCommand. Adapters that need to
         * provide custom handling should override this.
         */
        virtual int stopCmd(void);

        /**
         * @brief Default implementation of startCmd. Adapters that need to
         * provide custom handling should override this.
         */
        virtual int startCmd(void);

        /**
         * @brief Default implementation of moduleCommand. Modules or adapters that need to
         * provide custom handling should override this.
         */
        virtual int dispatchCommand(Msg::MessageCapsule<std::vector<char>>& capsule);

        /**
         * Default handler for moduleCommand. Modules or adapters that need to
         * provide custom handling should override this. Providing a non-pure
         * implementation here allows concrete adapter types to be instantiated
         * without requiring every adapter to implement moduleCommand.
         */
        virtual int moduleCommand(char* pbuf, size_t len);

        /**
         * @brief Default handler for moduleCommand with vector input. Modules or adapters that need to
         * provide custom handling should override this.
         */
        virtual int moduleCommand(std::vector<char>& buffer);

        virtual int cliCommand(std::vector<std::string>& buffer);

        /**
         * @brief Get the parent name
         * 
         * @return std::string parent module name 
         */
        std::string getParentName() const;
        
        /**
         * @brief Get the device type associated with this adapter
         * 
         * @return ModuleDefs::AdapterId The adapter's device type identifier
         */
        const ModuleDefs::AdapterId getDeviceType() const
        {
            return adapterId;
        }

        /**
         * @brief Acknowledge a received message by sending a reply back to the adapter. This can be used by command 
         * handlers to provide any necessary response data back to the adapter after processing a command.
         * 
         * @param capsule The message capsule containing the original command and data
         * @param reply Buffer containing any response data to be sent back to the adapter
         * @param len Length of the reply buffer
         * @return int Error code indicating success or failure of the acknowledgment process
         */
        virtual int AckMsg(Msg::MessageCapsule<std::vector<char>>& capsule, char* reply, int len);

        /**
         * @brief Get the parent module's ID
         * 
         * @return int Parent module ID, or -1 if not found
         */
        int GetParentID() const
        {
            return m_ModuleID;
        }

        /**
         * @brief Set the Mod Dispatch Callback for adapter-module comms
         * 
         * @param modDispatchCB Callback for module command dispatching
         * @return int 
         */
        int SetModDispatchCallback(std::function<int(Msg::MessageCapsule<std::vector<char>>&)> modDispatchCB);

        /**
         * @brief Set the reply handler callback for processing module acknowledgments
         * 
         * @param replyHandlerCB Callback for handling module acknowledgments
         * @return int 
         */
        int SetReplyHandlerCallback(std::function<int(Msg::MessageAck<std::vector<char>>&)> replyHandlerCB);

        /**
         * @brief Forward a module acknowledgment through the bound reply callback chain.
         *
         * This is typically called by module-side code to route a reply back to
         * the adapter that originated the command.
         */
        int ConnectedModuleReply(Msg::MessageAck<std::vector<char>>& ack);
    protected:
        std::thread m_ReplyProcThread;
        std::thread m_InputProcThread;
        std::string parentName;
        std::unordered_map<std::string, AdapterBase*> adapterMap;
        std::function< int(char* pbuf, size_t len) > moduleWriteCmd = nullptr;
        std::function< int(std::vector<std::string>&) > moduleCliCmd = nullptr;
        std::function< int(std::vector<char>&)     > moduleWriteCmdVector = nullptr;
        std::function<std::string(void)> readStatsCommand = nullptr;
        std::function<int(Msg::MessageCapsule<std::vector<char>>&)> dispatchCommandFunc = nullptr;
        std::function<int(std::vector<char>& buffer)> OnModuleMsgReceivedFunc = nullptr;
        std::function<int(Msg::MessageCapsule<std::vector<char>>&)> moduleDispatchCmd = nullptr;
        std::function<int(Msg::MessageAck<std::vector<char>>&)> moduleReplyCmd = nullptr;
        std::function<int(Msg::MessageAck<std::vector<char>>&)> replyHandlerFunc = nullptr;
        Msg::CircularBuffer<Msg::MessageAck<std::vector<char>>> m_ReplyMailBox;
        Msg::CircularBuffer<Msg::MessageCapsule<std::vector<char>>> m_InputMailBox;
        
        int m_ModuleID = -1;
        bool m_ReplyThreadRunning{true};

        const ModuleDefs::AdapterId adapterId;

        virtual int moduleCommand_(char* pbuf, size_t len);

        virtual int moduleCommand_(std::vector<char>& buffer);

        virtual int moduleCliCmd_(std::vector<std::string>& buffer);

        void procReplyThread(void);

        void procInputThread(void);

        /**
         * @brief Submit a message capsule to the adapter's input mailbox for processing by the module. This can be used by command handlers or other adapter logic to forward messages to the module for handling.
         * 
         * @param capsule The message capsule containing the command and data to be submitted to the module
         * @return int Error code indicating success or failure of the submission process
         */
        int SubmitMailBox(Msg::MessageCapsule<std::vector<char>>& capsule)
        {
            if (m_InputMailBox.isFull())
            {
                return -1;
            }

            m_InputMailBox.push(capsule);
            return 0; 
        }

        /**
         * @brief Submit an acknowledgment to the adapter's reply mailbox for processing by the reply thread. This can be used by command handlers to send acknowledgments back to the adapter, which can then be forwarded to the appropriate destination (e.g. network adapter) for transmission back to the original sender.
         * 
         * @param ack The acknowledgment object containing the acknowledgment data to be sent back to the sender module thread
         * @return int Error code indicating success or failure of the submission process
         */
        int SubmitReplyMailBox(Msg::MessageAck<std::vector<char>>& ack)
        {
            if (m_ReplyMailBox.isFull())
            {
                return -1;
            }

            m_ReplyMailBox.push(ack);
            return 0; 
        }
    };
}

#pragma endregion

#endif // ADAPTERBASE_HPP