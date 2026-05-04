// Template implementation file intentionally included by MessageLib.hpp.

#ifndef MESSAGE_LIB_IMPL
#include "MessageLib.hpp"
#else

#include <chrono>

namespace Msg {

template<typename T>
MessageCapsule<T>::MessageCapsule() {
}

template<typename T>
MessageCapsule<T>::~MessageCapsule() {
}

template<typename T>
MessageCapsule<T>::MessageCapsule(int command, const std::vector<T>& data, int source)
	: cmd(command), data(data), source(source) {
}

template<typename T>
MessageCapsule<T>& MessageCapsule<T>::operator=(const MessageCapsule& other) {
	if (this != &other) {
		cmd = other.cmd;
		source = other.source;
		data = other.data;
	}
	return *this;
}

template<typename T>
std::vector<T>& MessageCapsule<T>::getData() {
	return data;
}

template<typename T>
const std::vector<T>& MessageCapsule<T>::getData() const {
	return data;
}

template<typename T>
void MessageCapsule<T>::setData(const std::vector<T>& d) {
	data = d;
}

template<typename T>
int MessageCapsule<T>::getSource() const {
	return source;
}

template<typename T>
void MessageCapsule<T>::setSource(int src) {
	source = src;
}

template<typename T>
int MessageCapsule<T>::getCommand() const {
	return cmd;
}

template<typename T>
int MessageCapsule<T>::SendAck(int commandID, char* reply, int len) {
	if (!reply) {
		return -1; // Invalid reply buffer
	}
	// Store the reply command ID and data in the capsule for later retrieval by the adapter
	m_ReplyCommandID = commandID;
	m_ReplyData.assign(reply, reply + len);
	m_ReplyPresent = true; // Mark that a reply has been sent for this message
	return 0; // Success
}

template<typename T>
std::vector<T>& MessageCapsule<T>::GetAck() {
	m_ReplyPresent = false; // Mark that the reply has been retrieved 
	return m_ReplyData;
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
CircularBuffer<T>::~CircularBuffer() = default;

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
				throw std::runtime_error("Timeout waiting for buffer item.");
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
