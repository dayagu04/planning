#pragma once
#if defined(CYBER_ENV)
#include "autoplt/include/ADSTime.h"
#elif defined(AP_ENV)
#include "iflyauto_time.h"
#endif
#include <stdint.h>
#include <sys/time.h>
#include <time.h>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <string>

namespace IflyTime {

static inline double Now_us() {
#if defined(CYBER_ENV)
  auto time_now = autoplt::ADSTime::Now();
  return time_now.ToSecond() * 1000000;  // 转微秒
#elif defined(AP_ENV)
  auto time_now = iflyauto::Time::Now();
  return time_now.ToSecond() * 1000000;  // 转微秒
#else
  struct timeval tval;
  gettimeofday(&tval, NULL);
  double time_double = tval.tv_sec * 1000000;  // 转毫秒
  int64_t time_int = tval.tv_usec;             // 转毫秒
  double timestamp = time_double + static_cast<double>(time_int);
  return timestamp;
#endif
}

static inline double Now_ms() {
#if defined(CYBER_ENV)
  auto time_now = autoplt::ADSTime::Now();
  // double time_double = time_now.ToSecond() * 1000;  // 转毫秒
  // int64_t time_int = time_now.ToNanosecond() / 1000000; // 转毫秒
  return time_now.ToSecond() * 1000;  // 转毫秒
#elif defined(AP_ENV)
  auto time_now = iflyauto::Time::Now();
  return time_now.ToSecond() * 1000;  // 转毫秒
#else
  struct timeval tval;
  gettimeofday(&tval, NULL);
  double time_double = tval.tv_sec * 1000;  // 转毫秒
  int64_t time_int = tval.tv_usec / 1000;   // 转毫秒
  double timestamp = time_double + static_cast<double>(time_int);
  return timestamp;
#endif
}

static inline double Now_s() {
#if defined(CYBER_ENV)
  auto time_now = autoplt::ADSTime::Now();
  return time_now.ToSecond();
#elif defined(AP_ENV)
  return iflyauto::Time::Now().ToSecond();
#else
  struct timeval tval;
  gettimeofday(&tval, NULL);
  return tval.tv_sec;
#endif
}

static inline std::string DateString() {
  auto tp = std::chrono::time_point<std::chrono::system_clock,
                                    std::chrono::microseconds>(
      std::chrono::microseconds((int64_t)Now_us()));
  auto tt = std::chrono::system_clock::to_time_t(tp);
  std::tm* now = std::gmtime(&tt);
  return std::to_string(now->tm_year + 1900) + std::to_string(now->tm_mon + 1) +
         std::to_string(now->tm_mday) + std::to_string(now->tm_hour) +
         std::to_string(now->tm_min) + std::to_string(now->tm_sec);
}

}  // namespace IflyTime