#pragma once

#include <mutex>
#include <vector>
#include <stdexcept> // For std::out_of_range
#include <condition_variable>
#include <cstddef>
#include <utility>

#include "types.h"

namespace Msg {

    template<typename T>
    class MessageAck;  // Forward declaration of MessageAck for use in MessageCapsule
    
    template<typename T>
    class MessageCapsule {
    public:
        MessageCapsule() = default;
        MessageCapsule(uint16_t seqID, uint8_t cmd, const std::vector<T>& rawData, int source);
        MessageCapsule(uint16_t seqID, uint8_t cmd, std::vector<T>&& rawData, int source);
        MessageCapsule(const MessageCapsule&) = default;
        MessageCapsule(MessageCapsule&&) noexcept = default;
        MessageCapsule& operator=(const MessageCapsule& other) = default;
        MessageCapsule& operator=(MessageCapsule&&) noexcept = default;
        ~MessageCapsule() = default;

        // Getter for data
        std::vector<T>& getData();
        const std::vector<T>& getData() const;

        /**
         * @brief Get the Payload 
         * 
         * @return size_t Payload size in bytes
         */
        size_t GetPayloadSize() const {
            return data.size();
        }

        /**
         * @brief Get the acknowledgment data that was set by the command handler using SendAck. This can be used by adapters to retrieve any response data that the command handler has set to be sent back to the adapter as part of the reply.
         * 
         * @return std::vector<T> A vector containing the acknowledgment data set by the command handler
         */
        std::vector<T>& GetAckRaw();

        /**
         * @brief Get the acknowledgment object associated with this message. This can be used by command handlers to set acknowledgment data that will be sent back to the adapter as part of the reply when SendAck is called.
         * 
         * @return MessageAck<T>& Reference to the acknowledgment object for this message
         */
        MessageAck<T>& GetAck() {
            return m_MessageAck;
        }
        
        /**
         * @brief Set the data object
         * 
         * @param d 
         */
        void setData(const std::vector<T>& d);
        
        /**
         * @brief Get the Source object
         * 
         * @return int 
         */
        int getSource() const;

        /**
         * @brief Set the Source object
         * 
         * @param src 
         */
        void setSource(int src);

        /**
         * @brief Get the command extra data flag
         * 
         * @return val_type_t 
         */
        val_type_t getFlag() const {
            return wrtData;
        }

        /**
         * @brief Get the Command object
         * 
         * @return uint16_t 
         */
        uint16_t getSeqID() const;

        /**
         * @brief Send a reply message back to the adapter that sent the original command. This can be used by command handlers to provide any necessary response data back to the adapter after processing a command.
         * 
         * @param seqID Sequence identifier for which the reply is being sent
         * @param reply Buffer containing any response data to be sent back to the adapter
         * @param len Length of the reply buffer
         * @return int Error code indicating success or failure of the reply sending process
         */
        int SendAck(uint16_t seqID, char* reply, int len);

        /**
         * @brief Check if a reply has already been sent for this message. This can be used by command handlers to ensure that they do not send multiple replies for the same command, which could lead to confusion or errors on the adapter side.
         * 
         * @return true If a reply has already been sent for this message
         * @return false If no reply has been sent yet for this message
         */
        bool isReplyPresent() const {
            return m_ReplyPresent;
        }

        /**
         * @brief Get the Command object
         * 
         * @return uint8_t 
         */
        uint8_t getCommand() const {
            return command;
        }
    private:
        /**
         * @brief Sequence Identifier for this message
         * 
         */
        uint16_t seqID = 0;

        /**
         * @brief Command identifier for this message
         * 
         */
        uint8_t command = 0;

        /**
         * @brief Data field as a union type for extra command actions (write values, read values, etc.). 
         * This can be used by adapters to store additional information related to the command, 
         * such as parameters or flags, without needing to modify the MessageCapsule structure. 
         * Adapters can set and read this field as needed for their specific command handling logic.
         *
         */
        val_type_t wrtData;

        /**
         * @brief Raw data buffer for the message
         * 
         */
        std::vector<T> rawData;

        /**
         * @brief Source identifier for the message
         * 
         */
        int source = -1;
        std::vector<T> data;

        /**
         * @brief Sequence ID for which the reply is being sent
         * 
         */
        uint16_t m_ReplySeqID = 0;

        /**
         * @brief Optional reply field
         * 
         */
        std::vector<T> m_ReplyData;

        /**
         * @brief Optional acknowledgment object for this message
         * 
         */
        MessageAck<T> m_MessageAck;

        /**
         * @brief Flag to indicate if a reply has already been sent for this message
         * 
         */
        bool m_ReplyPresent = false; // Flag to indicate if a reply has already been sent for this message
    };

    template<typename T>
    class MessageAck {
    public:
        MessageAck() = default;
        MessageAck(int commandID, const std::vector<T>& replyData);
        ~MessageAck();
        MessageAck& operator=(const MessageAck& other);

        /**
         * @brief Set the reply data for this acknowledgment. This can be used by command handlers to set the response data that will be sent back to the adapter as part of the reply when SendAck is called on the associated MessageCapsule.
         * 
         * @param replyData Vector containing the reply data to be sent back to the adapter
         * @return int Error code indicating success or failure of setting the reply data
         */
        int SetReplyPayload(const std::vector<T>& replyData) {
            m_ReplyData = replyData;
            return 0; // Success
        }

        /**
         * @brief Get the Command ID for which this acknowledgment is being sent
         * 
         * @return int Command ID
         */        
        int getCommandID() const;

        /**
         * @brief Get the reply data associated with this acknowledgment
         * 
         * @return std::vector<T> Reply data as a vector of type T
         */
        int GetReplyData(T* buffer, size_t bufferSize) const;

        /**
         * @brief Get the size of the reply data associated with this acknowledgment
         * 
         * @return int Size of the reply data in bytes
         */
        int GetReplyDataSize() const;
    
    private:
        int m_CommandID;           // Command ID for which this acknowledgment is being sent
        std::vector<T> m_ReplyData; // Optional reply data to be sent back to the adapter
    };

    template <typename T>
    class CircularBuffer {
    public:
        // Constructor: Initializes the buffer with a given capacity
        explicit CircularBuffer(size_t capacity);

        CircularBuffer(const CircularBuffer&) = delete;
        CircularBuffer& operator=(const CircularBuffer&) = delete;
        CircularBuffer(CircularBuffer&&) = delete;
        CircularBuffer& operator=(CircularBuffer&&) = delete;
    
        ~CircularBuffer() = default;

        void killProcess();

        // Adds an element to the buffer (overwrites oldest if full)
        void push(const T& item);

        // Removes and returns the oldest element from the buffer
        void pop();

        // Clears the buffer completely
        void flush();

        // Returns a reference to the element at a specific index relative to the head
        // (0 is the oldest element, size-1 is the newest)
        T& operator[](size_t index);

        // Const version of operator[]
        const T& operator[](size_t index) const;

        /**
         * @brief Peek at an element at a specific index without removing it from the buffer
         * 
         * @param index 
         * @return T& 
         */
        T& peek(size_t index);

        /**
         * @brief If the buffer is empty, this will block until an item is added, then return a reference to the newest item (head)
         * 
         * @param timeout Timeout in milliseconds (-1 for infinite)
         * @return T& 
         */
        T& getHead(int timeout=-1);

        // Const version of getHead
        const T& getHead() const;

        const std::vector<T>& getBuffer() const;

        // Checks if the buffer is empty
        bool isEmpty() const;

        // Checks if the buffer is full
        bool isFull() const;

        // Returns the current number of elements in the buffer
        size_t size() const;

        // Returns the maximum capacity of the buffer
        size_t capacity() const;

    private:
        std::vector<T> buffer_;             // Underlying storage for the buffer
        size_t head_ = 0;                   // Index of the next available slot for writing
        size_t tail_ = 0;                   // Index of the oldest element (next to be read)
        size_t size_ = 0;                   // Current number of elements in the buffer
        size_t count = 0;                   // Number of items in buffer
        size_t capacity_ = 0;               // Maximum capacity of the buffer
        std::condition_variable m_BufferCv; // Condition variable for synchronization
        std::mutex bufferMutex;             // Circular buffer mutex
    };
};

#define MESSAGE_LIB_IMPL
#include "MessageLib.cpp"
#undef MESSAGE_LIB_IMPL
