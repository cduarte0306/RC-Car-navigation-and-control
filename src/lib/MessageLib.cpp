// Template implementation file intentionally included by MessageLib.hpp.

#ifndef MESSAGE_LIB_IMPL
#include "MessageLib.hpp"
#else

#include <chrono>

namespace Msg {

template<typename T, typename StorageT>
MessageCapsule<T, StorageT>::MessageCapsule(uint16_t seqID, uint8_t cmd, const storage_type& data, int source)
	: MessageCapsule(seqID, cmd, static_cast<uint8_t>(0), static_cast<val_type_t>(0), data, source) {
}

template<typename T, typename StorageT>
MessageCapsule<T, StorageT>::MessageCapsule(uint16_t seqID, uint8_t cmd, storage_type&& data, int source)
	: MessageCapsule(seqID, cmd, static_cast<uint8_t>(0), static_cast<val_type_t>(0), std::move(data), source) {
}

template<typename T, typename StorageT>
MessageCapsule<T, StorageT>::MessageCapsule(uint16_t seqID, uint8_t cmd, uint8_t modCmd, const storage_type& data, int source)
	: MessageCapsule(seqID, cmd, modCmd, static_cast<val_type_t>(0), data, source) {
}

template<typename T, typename StorageT>
MessageCapsule<T, StorageT>::MessageCapsule(uint16_t seqID, uint8_t cmd, uint8_t modCmd, storage_type&& data, int source)
	: MessageCapsule(seqID, cmd, modCmd, static_cast<val_type_t>(0), std::move(data), source) {
}

template<typename T, typename StorageT>
MessageCapsule<T, StorageT>::MessageCapsule(uint16_t seqID, uint8_t cmd, uint8_t modCmd, val_type_t dataField, const storage_type& data, int source)
	: seqID(seqID), command(cmd), mModCmd(modCmd), wrtData(dataField), rawData(data), data(data), source(source), m_MessageAck(seqID, {}) {
}

template<typename T, typename StorageT>
MessageCapsule<T, StorageT>::MessageCapsule(uint16_t seqID, uint8_t cmd, uint8_t modCmd, val_type_t dataField, storage_type&& data, int source)
	: seqID(seqID), command(cmd), mModCmd(modCmd), wrtData(dataField), rawData(data), data(std::move(data)), source(source), m_MessageAck(seqID, {}) {
}

template<typename T, typename StorageT>
typename MessageCapsule<T, StorageT>::storage_type& MessageCapsule<T, StorageT>::getData() {
	return data;
}

template<typename T, typename StorageT>
const typename MessageCapsule<T, StorageT>::storage_type& MessageCapsule<T, StorageT>::getData() const {
	return data;
}

template<typename T, typename StorageT>
void MessageCapsule<T, StorageT>::setData(const storage_type& d) {
	data = d;
}

template<typename T, typename StorageT>
int MessageCapsule<T, StorageT>::getSource() const {
	return source;
}

template<typename T, typename StorageT>
void MessageCapsule<T, StorageT>::setSource(int src) {
	source = src;
}

template<typename T, typename StorageT>
uint16_t MessageCapsule<T, StorageT>::getSeqID() const {
	return seqID;
}

template<typename T, typename StorageT>
int MessageCapsule<T, StorageT>::SendAck(uint16_t seqID, char* reply, int len) {
	if (!reply) {
		return -1; // Invalid reply buffer
	}
	if (len < 0) {
		return -1;
	}

	if constexpr (is_std_vector<storage_type>::value && std::is_same<typename storage_type::value_type, char>::value) {
		m_ReplyData.assign(reply, reply + len);
	} else {
		return -1;
	}

	// Store the reply sequence ID and data in the capsule for later retrieval by the adapter
	m_ReplySeqID = seqID;
	m_ReplyPresent = true; // Mark that a reply has been sent for this message
	return 0; // Success
}

template<typename T, typename StorageT>
int MessageCapsule<T, StorageT>::SendAck(uint16_t seqID, const storage_type& replyData) {
	m_ReplySeqID = seqID;
	m_ReplyData = replyData;
	m_ReplyPresent = true;
	return 0;
}

template<typename T, typename StorageT>
typename MessageCapsule<T, StorageT>::storage_type& MessageCapsule<T, StorageT>::GetAckRaw() {
	m_ReplyPresent = false; // Mark that the reply has been retrieved 
	return m_ReplyData;
}

// MessageAck implementation
template<typename T>
MessageAck<T>::MessageAck(int commandID, uint16_t seqID, const T& replyData)
	: mCommandID(commandID), mSeqID(seqID), mReplyData(replyData) {
}

template<typename T>
MessageAck<T>::~MessageAck() {
}

template<typename T>
MessageAck<T>& MessageAck<T>::operator=(const MessageAck<T>& other) {
	if (this != &other) {
		mCommandID = other.mCommandID;
		mSeqID 	= other.mSeqID;
		mReplyData = other.mReplyData;
	}
	return *this;
}

template<typename T>
int MessageAck<T>::getCommandID() const {
	return mCommandID;
}

template<typename T>
uint16_t MessageAck<T>::getSeqID() const {
	return mSeqID;
}

template<typename T>
CircularBuffer<T>::CircularBuffer(size_t capacity)
	: buffer_(capacity),
	  head_(0),
	  tail_(0),
	  size_(0),
	  capacity_(capacity) {
	if (capacity == 0) {
		throw std::invalid_argument("Capacity cannot be zero.");
	}
}

template<typename T>
void CircularBuffer<T>::killProcess() {
	std::lock_guard<std::mutex> lock(bufferMutex);
	// Notify all waiting threads to unblock (if any) before flushing the buffer
	m_BufferCv.notify_all();
	flush();
}

template<typename T>
void CircularBuffer<T>::push(const T& item) {
	std::lock_guard<std::mutex> lock(bufferMutex);
	// If empty, we need to notify any waiting threads that an item is being added
	bool isEmptyBeforePush = isEmpty();
	buffer_[head_] = item;
	head_ = (head_ + 1) % capacity_;
	if (size_ < capacity_) {
		size_++;
	} else {
		// If full, tail also moves forward
		tail_ = (tail_ + 1) % capacity_;
	}

	// Now we notify the waiting threads that an item has been added
	if (isEmptyBeforePush) {
		m_BufferCv.notify_all();
	}
}

template<typename T>
void CircularBuffer<T>::pop() {
	std::lock_guard<std::mutex> lock(bufferMutex);
	if (isEmpty()) {
		throw std::out_of_range("Buffer is empty.");
	}
	tail_ = (tail_ + 1) % capacity_;
	size_--;
}

template<typename T>
void CircularBuffer<T>::flush() {
	std::lock_guard<std::mutex> lock(bufferMutex);
	head_ = 0;
	tail_ = 0;
	size_ = 0;
}

template<typename T>
T& CircularBuffer<T>::operator[](size_t index) {
	if (index >= size_) {
		throw std::out_of_range("Index out of bounds.");
	}
	return buffer_[(tail_ + index) % capacity_];
}

template<typename T>
const T& CircularBuffer<T>::operator[](size_t index) const {
	if (index >= size_) {
		throw std::out_of_range("Index out of bounds.");
	}
	return buffer_[(tail_ + index) % capacity_];
}

template<typename T>
T& CircularBuffer<T>::peek(size_t index) {
	std::lock_guard<std::mutex> lock(bufferMutex);
	if (index >= size_) {
		throw std::out_of_range("Index out of bounds.");
	}
	return buffer_[(tail_ + index) % capacity_];
}

template<typename T>
T& CircularBuffer<T>::getHead(int timeout) {
	std::unique_lock<std::mutex> lock(bufferMutex);

	// If the buffer is empty, we wait for the signal that an item has been added
	if (isEmpty()) {
		if (timeout < 0) {
			m_BufferCv.wait(lock, [this] { return !isEmpty(); });
		} else {
			if (!m_BufferCv.wait_for(lock, std::chrono::milliseconds(timeout), [this] { return !isEmpty(); })) {
				// throw std::runtime_error("Timeout waiting for buffer item.");
			}
		}
	}

	size_t idx = (head_ + capacity_ - 1) % capacity_;
	return buffer_[idx];
}

template<typename T>
const T& CircularBuffer<T>::getHead() const {
	if (size_ == 0) {
		throw std::out_of_range("Buffer is empty.");
	}
	size_t idx = (tail_ + size_ - 1) % capacity_;
	return buffer_[idx];
}

template<typename T>
const std::vector<T>& CircularBuffer<T>::getBuffer() const {
	return buffer_;
}

template<typename T>
bool CircularBuffer<T>::isEmpty() const {
	return size_ == 0;
}

template<typename T>
bool CircularBuffer<T>::isFull() const {
	return size_ == capacity_;
}

template<typename T>
size_t CircularBuffer<T>::size() const {
	return size_;
}

template<typename T>
size_t CircularBuffer<T>::capacity() const {
	return capacity_;
}

} // namespace Msg

#endif
