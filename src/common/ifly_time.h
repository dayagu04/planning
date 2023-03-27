#pragma once
#ifdef CYBER_ENV
#include "autoplt/include/ADSTime.h"
#endif
#include <stdint.h>
#include <time.h>
#include <sys/time.h>

namespace IflyTime {

static inline double Now_ms() {
  #ifdef CYBER_ENV
    auto time_now = autoplt::ADSTime::Now();
    //double time_double = time_now.ToSecond() * 1000;  // 转毫秒
    //int64_t time_int = time_now.ToNanosecond() / 1000000; // 转毫秒
    return time_now.ToSecond() * 1000;  // 转毫秒
  #else
    struct timeval tval;
    gettimeofday(&tval, NULL);
    double time_double = tval.tv_sec * 1000;  // 转毫秒
    int64_t time_int = tval.tv_usec / 1000; // 转毫秒
    double timestamp = time_double + static_cast<double>(time_int);
    return timestamp;
  #endif
}

static inline double Now_s() {
  #ifdef CYBER_ENV
    auto time_now = autoplt::ADSTime::Now();
    return time_now.ToSecond();
  #else
    struct timeval tval;
    gettimeofday(&tval, NULL);
    return tval.tv_sec;
  #endif
}

} // namespace IflyTime