#pragma once

#include <mutex>
#include <vector>
#include <stdexcept> // For std::out_of_range
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <utility>
#include <type_traits>

#include "types.h"

namespace Msg {

    template<typename>
    struct dependent_false : std::false_type {};

    template<typename T>
    using DefaultCapsuleStorage = std::conditional_t<std::is_same<T, char>::value, std::vector<T>, T>;

    template<typename U>
    struct is_std_vector : std::false_type {};

    template<typename U, typename Alloc>
    struct is_std_vector<std::vector<U, Alloc>> : std::true_type {};

    template<typename U, typename = void>
    struct has_size_method : std::false_type {};

    template<typename U>
    struct has_size_method<U, std::void_t<decltype(std::declval<const U&>().size())>> : std::true_type {};

    template<typename T>
    class MessageAck;  // Forward declaration of MessageAck for use in MessageCapsule

    template<typename T, typename Enable = void>
    struct PayloadSerializer {
        static std::vector<char> serialize(const T&) {
            static_assert(dependent_false<T>::value, "PayloadSerializer: unsupported type. Provide a specialization.");
            return {};
        }

        static bool deserialize(const char*, size_t, T&) {
            static_assert(dependent_false<T>::value, "PayloadSerializer: unsupported type. Provide a specialization.");
            return false;
        }

        static bool deserialize(const std::vector<char>& payload, T& out) {
            return deserialize(payload.data(), payload.size(), out);
        }
    };

    template<>
    struct PayloadSerializer<std::vector<char>, void> {
        static std::vector<char> serialize(const std::vector<char>& value) {
            return value;
        }

        static bool deserialize(const char* data, size_t size, std::vector<char>& out) {
            if (size == 0) {
                out.clear();
                return true;
            }

            if (!data) {
                return false;
            }

            out.assign(data, data + size);
            return true;
        }

        static bool deserialize(const std::vector<char>& payload, std::vector<char>& out) {
            out = payload;
            return true;
        }
    };

    template<>
    struct PayloadSerializer<std::string, void> {
        static std::vector<char> serialize(const std::string& value) {
            return std::vector<char>(value.begin(), value.end());
        }

        static bool deserialize(const char* data, size_t size, std::string& out) {
            if (size == 0) {
                out.clear();
                return true;
            }

            if (!data) {
                return false;
            }

            out.assign(data, data + size);
            return true;
        }

        static bool deserialize(const std::vector<char>& payload, std::string& out) {
            out.assign(payload.begin(), payload.end());
            return true;
        }
    };

    template<typename T>
    struct PayloadSerializer<T, std::enable_if_t<std::is_trivially_copyable<T>::value>> {
        static std::vector<char> serialize(const T& value) {
            std::vector<char> bytes(sizeof(T));
            std::memcpy(bytes.data(), &value, sizeof(T));
            return bytes;
        }

        static bool deserialize(const char* data, size_t size, T& out) {
            if (size != sizeof(T) || !data) {
                return false;
            }

            std::memcpy(&out, data, sizeof(T));
            return true;
        }

        static bool deserialize(const std::vector<char>& payload, T& out) {
            return deserialize(payload.data(), payload.size(), out);
        }
    };

    template<typename U>
    struct PayloadSerializer<std::vector<U>, std::enable_if_t<std::is_trivially_copyable<U>::value && !std::is_same<U, char>::value>> {
        static std::vector<char> serialize(const std::vector<U>& value) {
            std::vector<char> bytes(value.size() * sizeof(U));
            if (!value.empty()) {
                std::memcpy(bytes.data(), value.data(), bytes.size());
            }
            return bytes;
        }

        static bool deserialize(const char* data, size_t size, std::vector<U>& out) {
            if (size == 0) {
                out.clear();
                return true;
            }

            if (!data || (size % sizeof(U)) != 0) {
                return false;
            }

            const size_t count = size / sizeof(U);
            out.resize(count);
            std::memcpy(out.data(), data, size);
            return true;
        }

        static bool deserialize(const std::vector<char>& payload, std::vector<U>& out) {
            return deserialize(payload.data(), payload.size(), out);
        }
    };

    class PayloadCodec {
    public:
        template<typename T>
        static std::vector<char> serialize(const T& value) {
            return PayloadSerializer<std::decay_t<T>>::serialize(value);
        }

        template<typename T>
        static bool deserialize(const std::vector<char>& payload, T& out) {
            return PayloadSerializer<std::decay_t<T>>::deserialize(payload, out);
        }

        template<typename T>
        static bool deserialize(const char* data, size_t size, T& out) {
            return PayloadSerializer<std::decay_t<T>>::deserialize(data, size, out);
        }
    };
    
    template<typename T, typename StorageT = DefaultCapsuleStorage<T>>
    class MessageCapsule {
    public:
        using storage_type = StorageT;

        MessageCapsule() = default;
        MessageCapsule(uint16_t seqID, uint8_t cmd, const storage_type& rawData, int source);
        MessageCapsule(uint16_t seqID, uint8_t cmd, storage_type&& rawData, int source);
        MessageCapsule(uint16_t seqID, uint8_t cmd, uint8_t modCmd, const storage_type& rawData, int source);
        MessageCapsule(uint16_t seqID, uint8_t cmd, uint8_t modCmd, storage_type&& rawData, int source);
        MessageCapsule(uint16_t seqID, uint8_t cmd, uint8_t modCmd, val_type_t dataField, const storage_type& rawData, int source);
        MessageCapsule(uint16_t seqID, uint8_t cmd, uint8_t modCmd, val_type_t dataField, storage_type&& rawData, int source);
        MessageCapsule(const MessageCapsule&) = default;
        MessageCapsule(MessageCapsule&&) noexcept = default;
        MessageCapsule& operator=(const MessageCapsule& other) = default;
        MessageCapsule& operator=(MessageCapsule&&) noexcept = default;
        ~MessageCapsule() = default;

        // Getter for data
        storage_type& getData();
        const storage_type& getData() const;

        /**
         * @brief Get the Payload 
         * 
         * @return size_t Payload size in bytes
         */
        size_t GetPayloadSize() const {
            if constexpr (has_size_method<storage_type>::value) {
                return static_cast<size_t>(data.size());
            }
            return sizeof(storage_type);
        }

        /**
         * @brief Get the acknowledgment data that was set by the command handler using SendAck. This can be used by adapters to retrieve any response data that the command handler has set to be sent back to the adapter as part of the reply.
         * 
         * @return std::vector<T> A vector containing the acknowledgment data set by the command handler
         */
        storage_type& GetAckRaw();

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
        void setData(const storage_type& d);
        
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
         * @brief Get module data field associated with this capsule.
         */
        val_type_t getDataField() const {
            return wrtData;
        }

        /**
         * @brief Set module data field associated with this capsule.
         */
        void setDataField(val_type_t value) {
            wrtData = value;
        }

        /**
         * @brief Get module-specific command identifier.
         */
        uint8_t getModCmd() const {
            return mModCmd;
        }

        /**
         * @brief Set module-specific command identifier.
         */
        void setModCmd(uint8_t cmd) {
            mModCmd = cmd;
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
         * @brief Send a reply object using the capsule storage type.
         */
        int SendAck(uint16_t seqID, const storage_type& replyData);

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

        /**
         * @brief Set the Command object
         * 
         * @param cmd 
         */
        void SetAckRequested(bool ackRequested) {
            m_AckRequested = ackRequested;
        }

        /**
         * @brief Check if acknowledgment was requested for this message. This can be used by command handlers to determine whether they need to send an acknowledgment back to the adapter as part of the reply.
         * 
         * @return true If acknowledgment was requested for this message
         * @return false If no acknowledgment was requested for this message
         */
        bool isAckRequested() const {
            return m_AckRequested;
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
         * @brief Module specific command identifier for this message
         * 
         */
        uint8_t mModCmd = 0;

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
        storage_type rawData;

        /**
         * @brief Source identifier for the message
         * 
         */
        int source = -1;
        storage_type data;

        /**
         * @brief Sequence ID for which the reply is being sent
         * 
         */
        uint16_t m_ReplySeqID = 0;

        /**
         * @brief Optional reply field
         * 
         */
        storage_type m_ReplyData;

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

        bool m_AckRequested = false; // Flag to indicate if acknowledgment was requested for this message
    };

    template<typename T>
    class MessageAck {
    public:
        MessageAck() = default;
        MessageAck(int commandID, uint16_t seqID, const T& replyData);
        MessageAck(bool status, const T& replyData) : mStatus(status), mReplyData(replyData) {}
        MessageAck(const MessageAck&) = default;
        MessageAck(MessageAck&&) noexcept = default;
        ~MessageAck();
        MessageAck& operator=(const MessageAck& other);

        /**
         * @brief Set the reply data for this acknowledgment. This can be used by command handlers to set the response data that will be sent back to the adapter as part of the reply when SendAck is called on the associated MessageCapsule.
         * 
         * @param replyData Reply data to be sent back to the adapter
         * @return int Error code indicating success or failure of setting the reply data
         */
        int SetReplyPayload(const T& replyData) {
            mReplyData = replyData;
            return 0; // Success
        }

        /**
         * @brief Get the Command ID for which this acknowledgment is being sent
         * 
         * @return int Command ID
         */        
        int getCommandID() const;

        /**
         * @brief Get sequence ID associated with this acknowledgment.
         */
        uint16_t getSeqID() const;

        /**
         * @brief Get the reply data associated with this acknowledgment as a reference to the acknowledgment's internal storage. This can be used by command handlers to retrieve the reply data that has been set for this acknowledgment without needing to copy it into an external buffer.
         * 
         * @return T& Reference to the reply data stored in this acknowledgment
         */
        T& GetReplyData() {
            return mReplyData;
        }

        /**
         * @brief Get the status of the acknowledgment, which can be used to indicate success or failure of the command processing. This can be set by command handlers to provide feedback to the adapter about the result of processing a command.
         * 
         * @return true If the acknowledgment indicates success
         * @return false If the acknowledgment indicates failure
         */
        bool GetStatus(void) const {
            return mStatus;
        }

        int mReplyDestID;   // Module ID that expects the reply associated with this acknowledgment
        int mCommandID;     // Command ID for which this acknowledgment is being sent
        uint16_t mSeqID;    // Sequence ID for which the reply is being sent
        bool mStatus = false;
        T mReplyData;       // Optional reply data to be sent back to the adapter
    };

    class CommsPipePacket {
    public:
        struct WireHeader {
            uint16_t seqID;
            int32_t commandID;
            uint8_t status;
            uint32_t payloadSize;
        } __attribute__((__packed__));

        static std::vector<uint8_t> serialize(const MessageAck<std::vector<char>>& ack) {
            WireHeader header{};
            header.seqID = ack.mSeqID;
            header.commandID = ack.mCommandID;
            header.status = ack.GetStatus() ? 1 : 0;
            header.payloadSize = static_cast<uint32_t>(ack.mReplyData.size());

            std::vector<uint8_t> frame(sizeof(WireHeader) + ack.mReplyData.size());
            std::memcpy(frame.data(), &header, sizeof(WireHeader));
            if (!ack.mReplyData.empty()) {
                std::memcpy(frame.data() + sizeof(WireHeader), ack.mReplyData.data(), ack.mReplyData.size());
            }
            return frame;
        }

        template<typename SocketT>
        static bool send(SocketT& socket, const std::vector<uint8_t>& frame, const std::string& destIP) {
            if (frame.empty() || destIP.empty()) {
                return false;
            }

            std::string ip = destIP;
            uint8_t* payload = const_cast<uint8_t*>(frame.data());
            return socket.transmit(payload, frame.size(), ip);
        }
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
