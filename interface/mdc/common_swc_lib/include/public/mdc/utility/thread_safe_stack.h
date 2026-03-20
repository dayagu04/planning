/*
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * Description:  thread safe stack
 */
#ifndef THREAD_SAFE_STACK_H
#define THREAD_SAFE_STACK_H
#include <mutex>
#include <queue>
#include <vector>
#include <algorithm>
#include <chrono>
#ifndef SCFI_SDK_X86
#include "driver/ascend_hal.h"
#endif

namespace mdc {
template <typename T, typename Tag = void>
struct IsDpRawData {
static const bool value = false;
};

template <typename T>
struct IsDpRawData<T, typename T::element_type::IsDpRawDataTag> {
static const bool value = true;
};

template <typename T>
class ThreadSafeStack {
public:
    explicit ThreadSafeStack(const int64_t timeToLive0 = 0, const size_t capLimit = 1U)
        : timeToLive(timeToLive0), capacityLimit(capLimit)
    {}
    ~ThreadSafeStack() = default;
    ThreadSafeStack(const ThreadSafeStack&) = delete;
    ThreadSafeStack& operator=(const ThreadSafeStack&) = delete;
    ThreadSafeStack(ThreadSafeStack&&) = default;
    ThreadSafeStack& operator=(ThreadSafeStack&&) & = default;

    void Push(const T& newValue)
    {
        std::lock_guard<std::mutex> lk(mut);
        ClearOldData();
        ClearOutnumberedData();

        container.push_back(newValue);
        times.push_back(std::chrono::steady_clock::now());
    }

    void Push(T&& newValue)
    {
        std::lock_guard<std::mutex> lk(mut);
        ClearOldData();
        ClearOutnumberedData();

        container.push_back(std::forward<T>(newValue));
        times.push_back(std::chrono::steady_clock::now());
    }

    std::shared_ptr<T> Pop()
    {
        std::unique_lock<std::mutex> lk(mut);
        ClearOldData();
        if (container.empty()) {
            return nullptr;
        }
        std::shared_ptr<T> res(std::make_shared<T>(container.back()));
        PopBackData();
        return res;
    }

    std::vector<std::shared_ptr<T>> GetNdata(const size_t n)
    {
        std::lock_guard<std::mutex> lk(mut);
        ClearOldData();
        auto len = std::min(capacityLimit, n);
        len = std::min(len, container.size());
        std::vector<std::shared_ptr<T>> out;
        out.reserve(len);
        auto bound = container.rbegin();
        std::advance(bound, len);
        for (auto it = container.rbegin(); (it != container.rend()) && (it != bound); ++it) {
            out.emplace_back(std::make_shared<T>(std::ref(*it)));
        }
        return out;
    }

    std::shared_ptr<T> GetOneData(const uint32_t freshDataTime = UINT32_MAX)
    {
        std::lock_guard<std::mutex> lk(mut);
        ClearOldData();
        if (container.empty()) {
            return nullptr;
        }
        const auto oldtime = times.back();
        const auto now = std::chrono::steady_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - oldtime).count();
        if (duration > static_cast<decltype(duration)>(freshDataTime)) {
            // no fresh data now
            return nullptr;
        }
        std::shared_ptr<T> data(std::make_shared<T>(container.back()));
        return data;
    }

    std::shared_ptr<T> GetLastOne(const bool erase = false)
    {
        std::lock_guard<std::mutex> lk(mut);
        ClearOldData();
        if (container.empty()) {
            return nullptr;
        }
        std::shared_ptr<T> data(std::make_shared<T>(container.front()));
        if (erase) {
            PopFrontData();
        }
        return data;
    }

    bool Empty()
    {
        std::lock_guard<std::mutex> lk(mut);
        ClearOldData();
        return container.empty();
    }

    size_t Size()
    {
        std::lock_guard<std::mutex> lk(mut);
        ClearOldData();
        return container.size();
    }

    void Clear()
    {
        std::lock_guard<std::mutex> lk(mut);
        container.clear();
        times.clear();
    }

private:
    void ClearOldData()
    {
        if (timeToLive <= 0) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        while (!times.empty()) {
            const auto oldtime = times.front();
            const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - oldtime).count();
            if (duration > timeToLive) {
                PopFrontData();
            } else {
                break;
            }
        }
    }

    inline void ClearOutnumberedData()
    {
        while (container.size() > capacityLimit) {
            PopFrontData();
        }
    }

    inline void PopBackData()
    {
        container.pop_back();
        times.pop_back();
    }

    template <typename U = T>
    inline void PopFrontData(typename std::enable_if<IsDpRawData<U, void>::value>::type* = nullptr)
    {
        auto data = container.front();
        #ifndef SCFI_SDK_X86
        if (IsDpRawData<T, void>::value) {
            halMbufFree(reinterpret_cast<Mbuf*>(data->GetMbufPtr()));
        }
        #endif
        container.pop_front();
        times.pop_front();
    }

    template <typename U = T>
    inline void PopFrontData(typename std::enable_if<!IsDpRawData<U, void>::value>::type* = nullptr)
    {
        container.pop_front();
        times.pop_front();
    }

private:
    mutable std::mutex mut;
    int64_t timeToLive{};
    size_t capacityLimit{5U};
    std::deque<T> container;
    std::deque<std::chrono::time_point<std::chrono::steady_clock>> times;
};
}  /* namespace mdc */
#endif