#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>

namespace uwfl2::loop_closure
{

template <typename T>
class BoundedQueue
{
public:
    explicit BoundedQueue(std::size_t capacity) : capacity_(capacity) {}

    bool push_latest(T value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_ || capacity_ == 0)
        {
            return false;
        }
        if (queue_.size() == capacity_)
        {
            queue_.pop_front();
            ++dropped_;
        }
        queue_.push_back(std::move(value));
        condition_.notify_one();
        return true;
    }

    std::optional<T> wait_pop()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return closed_ || !queue_.empty(); });
        if (queue_.empty())
        {
            return std::nullopt;
        }
        T value = std::move(queue_.front());
        queue_.pop_front();
        return value;
    }

    void close()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        closed_ = true;
        condition_.notify_all();
    }

    std::size_t size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    std::size_t dropped() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return dropped_;
    }

private:
    const std::size_t capacity_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<T> queue_;
    std::size_t dropped_ = 0;
    bool closed_ = false;
};

}  // namespace uwfl2::loop_closure
