#pragma once

#include <mutex>
#include <vector>
#include <stdexcept> // For std::out_of_range
#include <condition_variable>
#include <cstddef>

#include "types.h"

namespace Msg {
    template<typename T>
    class MessageCapsule {
    public:
        MessageCapsule();
        ~MessageCapsule();

        MessageCapsule(int command, const std::vector<T>& rawData, int source);

        MessageCapsule& operator=(const MessageCapsule& other);

        // Getter for data
        std::vector<T>& getData();
        const std::vector<T>& getData() const;

        std::vector<T>& GetAck();
        
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
         * @return int 
         */
        int getCommand() const;

        /**
         * @brief Send a reply message back to the adapter that sent the original command. This can be used by command handlers to provide any necessary response data back to the adapter after processing a command.
         * 
         * @param commandID Command identifier for which the reply is being sent
         * @param reply Buffer containing any response data to be sent back to the adapter
         * @param len Length of the reply buffer
         * @return int Error code indicating success or failure of the reply sending process
         */
        int SendAck(int commandID, char* reply, int len);

        /**
         * @brief Check if a reply has already been sent for this message. This can be used by command handlers to ensure that they do not send multiple replies for the same command, which could lead to confusion or errors on the adapter side.
         * 
         * @return true If a reply has already been sent for this message
         * @return false If no reply has been sent yet for this message
         */
        bool isReplyPresent() const {
            return m_ReplyPresent;
        }
    private:
        /**
         * @brief Command field
         * 
         */
        int cmd = -1;

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
         * @brief Command ID for which the reply is being sent
         * 
         */
        int m_ReplyCommandID = -1;

        /**
         * @brief Optional reply field
         * 
         */
        std::vector<T> m_ReplyData;

        /**
         * @brief Flag to indicate if a reply has already been sent for this message
         * 
         */
        bool m_ReplyPresent = false; // Flag to indicate if a reply has already been sent for this message
    };

    template <typename T>
    class CircularBuffer {
    public:
        // Constructor: Initializes the buffer with a given capacity
        explicit CircularBuffer(size_t capacity);
    
        ~CircularBuffer();

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
