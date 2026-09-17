#ifndef ZF_GLOBAL_COMMON_ASYNC_UTIL_H
#define ZF_GLOBAL_COMMON_ASYNC_UTIL_H

#include <mutex>
#include <future>
#include <functional> 

#include "zf_global/zf_global.h"

BEGIN_NS_ZF

// NS_ZF::Async(&ClassA::FunA, this)
template <typename F, typename... Args>
static auto Async(F &&f, Args &&...args)
    -> std::future<typename std::result_of<F(Args...)>::type>
{
  return std::async(std::launch::async,
                    std::bind(std::forward<F>(f), std::forward<Args>(args)...));
}

static inline void Yield()
{
  std::this_thread::yield();
}

template <typename Rep, typename Period>
static void SleepFor(const std::chrono::duration<Rep, Period>& sleep_duration) {
    std::this_thread::sleep_for(sleep_duration);
}

static inline void MSleep(useconds_t msec) {
    std::this_thread::sleep_for(std::chrono::microseconds{msec * 1000});
}

static inline void USleep(useconds_t usec) {
    std::this_thread::sleep_for(std::chrono::microseconds{usec});
}

END_NS_ZF

#endif // !ZF_GLOBAL_COMMON_ASYNC_UTIL_H