#pragma once

#include <atomic>
#include <cstddef>
#include <array>

namespace sensor_base {


// SPSC queue
template <typename T, size_t Capacity>
class SPSCQueue {
private:
    std::array<T, Capacity> buffer_;

    alignas(64) std::atomic<size_t> head_{0};
    alignas(64) std::atomic<size_t> tail_{0};

public:
    SPSCQueue() = default;

    bool push(const T& item) {
        size_t current_head = head_.load(std::memory_order_relaxed);
        size_t next_head = (current_head + 1) % Capacity;
        if (next_head == tail_.load(std::memory_order_relaxed)) {
            return false;
        }

        buffer_[next_head] = item;

        head_.store(next_head, std::memory_order_release);
        return true;
    }

    bool pop(T& item) {
        size_t current_tail = tail_.load(std::memory_order_relaxed);
        
        if (current_tail == head_.load(std::memory_order_relaxed)) {
            return false;
        }

        item = buffer_[current_tail];
        tail_.store((current_tail + 1) % Capacity, std::memory_order_release);
        return true;
    }

    size_t size() const {
        return (head_.load(std::memory_order_relaxed) - tail_.load(std::memory_order_relaxed)) % Capacity;
    }

    size_t max_size() const {
        return Capacity;
    }
};

} // namespace sensor_base
