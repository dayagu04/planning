/**
 * @log
 */

#ifndef __PLANNING_COMMON_GLOG_H__
#define __PLANNING_COMMON_GLOG_H__


#include "glog/logging.h"
#include "glog/raw_logging.h"
#include <vector>
#include <string>

namespace planning {

class FilePath {
 public:
  static std::string GetName() { return GetNameRef(); }
  static void SetName(const std::string& name) { GetNameRef() = name; }
  static std::string& GetNameRef() {
    static std::string binary_name;
    return binary_name;
  }
};

#define ILOG_LEFT_BRACKET "["
#define ILOG_RIGHT_BRACKET "]"

#ifndef ILOG_MODULE_NAME
#define ILOG_MODULE_NAME planning::FilePath::GetName().c_str()
#endif

#define ILOG_DEBUG_MODULE(module) \
  VLOG(4) << ILOG_LEFT_BRACKET << module << ILOG_RIGHT_BRACKET << "[DEBUG] "

#define ILOG_DEBUG ILOG_DEBUG_MODULE(ILOG_MODULE_NAME)
#define ILOG_INFO ILOG_MODULE(ILOG_MODULE_NAME, INFO)
#define ILOG_WARN ILOG_MODULE(ILOG_MODULE_NAME, WARN)
#define ILOG_ERROR ILOG_MODULE(ILOG_MODULE_NAME, ERROR)
#define ILOG_FATAL ILOG_MODULE(ILOG_MODULE_NAME, FATAL)

#ifndef ILOG_MODULE_STREAM
#define ILOG_MODULE_STREAM(log_severity) ILOG_MODULE_STREAM_##log_severity
#endif

#ifndef ILOG_MODULE
#define ILOG_MODULE(module, log_severity) \
  ILOG_MODULE_STREAM(log_severity)(module)
#endif

#define ILOG_MODULE_STREAM_INFO(module)                         \
  google::LogMessage(__FILE__, __LINE__, google::INFO).stream() \
      << ILOG_LEFT_BRACKET << module << ILOG_RIGHT_BRACKET

#define ILOG_MODULE_STREAM_WARN(module)                            \
  google::LogMessage(__FILE__, __LINE__, google::WARNING).stream() \
      << ILOG_LEFT_BRACKET << module << ILOG_RIGHT_BRACKET

#define ILOG_MODULE_STREAM_ERROR(module)                         \
  google::LogMessage(__FILE__, __LINE__, google::ERROR).stream() \
      << ILOG_LEFT_BRACKET << module << ILOG_RIGHT_BRACKET

#define ILOG_MODULE_STREAM_FATAL(module)                         \
  google::LogMessage(__FILE__, __LINE__, google::FATAL).stream() \
      << ILOG_LEFT_BRACKET << module << ILOG_RIGHT_BRACKET

#define ILOG_INFO_IF(cond) ILOG_IF(INFO, cond, ILOG_MODULE_NAME)
#define ILOG_WARN_IF(cond) ILOG_IF(WARN, cond, ILOG_MODULE_NAME)
#define ILOG_ERROR_IF(cond) ILOG_IF(ERROR, cond, ILOG_MODULE_NAME)
#define ILOG_FATAL_IF(cond) ILOG_IF(FATAL, cond, ILOG_MODULE_NAME)
#define ILOG_IF(severity, cond, module) \
  !(cond) ? (void)0                     \
          : google::LogMessageVoidify() & ILOG_MODULE(module, severity)

#define ILOG_CHECK(cond) \
  CHECK(cond) << ILOG_LEFT_BRACKET << ILOG_MODULE_NAME << ILOG_RIGHT_BRACKET

#define ILOG_INFO_EVERY(freq)                                      \
  LOG_EVERY_N(INFO, freq) << ILOG_LEFT_BRACKET << ILOG_MODULE_NAME \
                          << ILOG_RIGHT_BRACKET
#define ILOG_WARN_EVERY(freq) \
  LOG_EVERY_N(WARNING, freq)  \
      << ILOG_LEFT_BRACKET << ILOG_MODULE_NAME << ILOG_RIGHT_BRACKET
#define ILOG_ERROR_EVERY(freq)                                      \
  LOG_EVERY_N(ERROR, freq) << ILOG_LEFT_BRACKET << ILOG_MODULE_NAME \
                           << ILOG_RIGHT_BRACKET

#if !defined(RETURN_IF_NULL)
#define RETURN_IF_NULL(ptr)              \
  if (ptr == nullptr) {                  \
    ILOG_WARN << #ptr << " is nullptr."; \
    return;                              \
  }
#endif

#if !defined(RETURN_VAL_IF_NULL)
#define RETURN_VAL_IF_NULL(ptr, val)     \
  if (ptr == nullptr) {                  \
    ILOG_WARN << #ptr << " is nullptr."; \
    return val;                          \
  }
#endif

#if !defined(RETURN_IF)
#define RETURN_IF(condition)               \
  if (condition) {                         \
    ILOG_WARN << #condition << " is met."; \
    return;                                \
  }
#endif

#if !defined(RETURN_VAL_IF)
#define RETURN_VAL_IF(condition, val)      \
  if (condition) {                         \
    ILOG_WARN << #condition << " is met."; \
    return val;                            \
  }
#endif

#if !defined(_RETURN_VAL_IF_NULL2__)
#define _RETURN_VAL_IF_NULL2__
#define RETURN_VAL_IF_NULL2(ptr, val) \
  if (ptr == nullptr) {               \
    return (val);                     \
  }
#endif

#if !defined(_RETURN_VAL_IF2__)
#define _RETURN_VAL_IF2__
#define RETURN_VAL_IF2(condition, val) \
  if (condition) {                     \
    return (val);                      \
  }
#endif

#if !defined(_RETURN_IF2__)
#define _RETURN_IF2__
#define RETURN_IF2(condition) \
  if (condition) {            \
    return;                   \
  }
#endif

struct GlogFlag {
  bool is_init = false;
};

void InitGlog(const char *file);

void StopGlog();

void CreateLogDirectory(const std::string& path);

void SignalHandler(const char* data, int size);

}  // namespace planning

#endif  // __PLANNING_COMMON_GLOG_H__
