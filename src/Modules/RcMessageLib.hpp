#pragma once

#include <mutex>
#include <vector>
#include <stdexcept> // For std::out_of_range

namespace Msg {
    template<typename T>
    class MessageCapsule {
    public:
        MessageCapsule() {}
        ~MessageCapsule() {}
        
        // Getter for data
        T& getData() { return data; }
        const T& getData() const { return data; }
        
        // Setter for data
        void setData(const T& d) { data = d; }
        
        // Getter for source
        int getSource() const { return source; }
        void setSource(int src) { source = src; }
        
    private:
        int source = -1;
        T data;
    };

    template <typename T>
    class CircularBuffer {
    public:
        // Constructor: Initializes the buffer with a given capacity
        explicit CircularBuffer(size_t capacity) :
            buffer_(capacity),
            head_(0),
            tail_(0),
            size_(0),
            capacity_(capacity) {
            if (capacity == 0) {
                throw std::invalid_argument("Capacity cannot be zero.");
            }
        }

        // Adds an element to the buffer (overwrites oldest if full)
        void push(const T& item) {
            std::lock_guard<std::mutex> lock(bufferMutex);
            buffer_[head_].setData(item);
            head_ = (head_ + 1) % capacity_;
            if (size_ < capacity_) {
                size_++;
            } else {
                // If full, tail also moves forward
                tail_ = (tail_ + 1) % capacity_;
            }
        }

        // Removes and returns the oldest element from the buffer
        void pop() {
            std::lock_guard<std::mutex> lock(bufferMutex);
            if (isEmpty()) {
                throw std::out_of_range("Buffer is empty.");
            }
            tail_ = (tail_ + 1) % capacity_;
            size_--;
        }

        // Clears the buffer completely
        void flush() {
            std::lock_guard<std::mutex> lock(bufferMutex);
            head_ = 0;
            tail_ = 0;
            size_ = 0;
        }

        // Returns a reference to the element at a specific index relative to the head
        // (0 is the oldest element, size-1 is the newest)
        T& operator[](size_t index) {
            if (index >= size_) {
                throw std::out_of_range("Index out of bounds.");
            }
            return buffer_[(tail_ + index) % capacity_].getData();
        }

        // Const version of operator[]
        const T& operator[](size_t index) const {
            if (index >= size_) {
                throw std::out_of_range("Index out of bounds.");
            }
            return buffer_[(tail_ + index) % capacity_].getData();
        }

        // Returns item at head
        T& getHead() {
            std::lock_guard<std::mutex> lock(bufferMutex);
            if (isEmpty()) {
                throw std::out_of_range("Buffer is empty.");
            }
            size_t idx = (head_ + capacity_ - 1) % capacity_;
            return buffer_[idx].getData();
        }

        // Const version of getHead
        const T& getHead() const {
            if (size_ == 0) throw std::out_of_range("empty");
            size_t idx = (tail_ + size_ - 1) % capacity_;  
            return buffer_[idx].getData();
        }

        // Checks if the buffer is empty
        bool isEmpty() const {
            return size_ == 0;
        }

        // Checks if the buffer is full
        bool isFull() const {
            return size_ == capacity_;
        }

        // Returns the current number of elements in the buffer
        size_t size() const {
            return size_;
        }

        // Returns the maximum capacity of the buffer
        size_t capacity() const {
            return capacity_;
        }

    private:
        std::vector<MessageCapsule<T>> buffer_; // Underlying storage for the buffer
        size_t head_ = 0;          // Index of the next available slot for writing
        size_t tail_ = 0;          // Index of the oldest element (next to be read)
        size_t size_ = 0;          // Current number of elements in the buffer
        size_t count = 0;          // Number of items in buffer
        size_t capacity_ = 0;      // Maximum capacity of the buffer

        std::mutex bufferMutex;    // Circular buffer mutex
    };
};
