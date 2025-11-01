#pragma once

#include <vector>
#include <stdexcept> // For std::out_of_range

namespace Msg {
    template<typename T>
    class MessageCapsule {
    public:
        MessageCapsule() {}
        ~MessageCapsule() {}
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
            buffer_[head_] = item;
            head_ = (head_ + 1) % capacity_;
            if (size_ < capacity_) {
                size_++;
            } else {
                // If full, tail also moves forward
                tail_ = (tail_ + 1) % capacity_;
            }
        }

        // Removes and returns the oldest element from the buffer
        T pop() {
            if (isEmpty()) {
                throw std::out_of_range("Buffer is empty.");
            }
            T item = buffer_[tail_];
            tail_ = (tail_ + 1) % capacity_;
            size_--;
            return item;
        }

        // Returns a reference to the element at a specific index relative to the head
        // (0 is the oldest element, size-1 is the newest)
        T& operator[](size_t index) {
            if (index >= size_) {
                throw std::out_of_range("Index out of bounds.");
            }
            return buffer_[(tail_ + index) % capacity_];
        }

        // Const version of operator[]
        const T& operator[](size_t index) const {
            if (index >= size_) {
                throw std::out_of_range("Index out of bounds.");
            }
            return buffer_[(tail_ + index) % capacity_];
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
        size_t head_;          // Index of the next available slot for writing
        size_t tail_;          // Index of the oldest element (next to be read)
        size_t size_;          // Current number of elements in the buffer
        size_t capacity_;      // Maximum capacity of the buffer
    };
};
