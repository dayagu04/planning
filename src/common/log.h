#pragma once

#include <chrono>

#include "iostream"

#ifdef ENABLE_IFLYTEK_LOG
#include "iflyauto_log.h"
#endif

#define CONCAT2(x) "%s " x
#define CONCAT(x) CONCAT2(x)

namespace iflyauto {
enum LogLevel {
  LOGLEVEL_START = 0,
  FETAL,
  ERROR,
  WARNING,
  NOTICE,
  DEBUG,
  LOGLEVEL_END
};

#ifdef ENABLE_IFLYTEK_LOG
class Log {
 public:
  static Log& getInstance() {
    static Log instance;
    return instance;
  };
  void setConfig(const char* modulename, const char* filename,
                 LogLevel loglevel);
  inline const char* getModuleName() { return m_moduleName.c_str(); }
  void sync();

  ~Log();

 private:
  Log();

 private:
  std::string m_moduleName;
};

#else
class Log {
 public:
  static Log& getInstance() {
    static Log instance;
    return instance;
  };
  void setConfig(const char* modulename, const char* logpath,
                 LogLevel loglevel) {
    m_logDebug = fopen(logpath, "w");
    if (!m_logDebug) {
      printf("path is not exist,write file failed! %s\n", logpath);
    }
  };
  inline FILE* getModulePointer() { return m_logDebug; }
  void sync();

  ~Log(){};

 private:
  Log(){};

 private:
  FILE* m_logDebug;
};
#endif
// bst::Log::getInstance().getModulePointer(),
#ifdef BST
#define LOG_BST_RELEASE(severity, format, ...)                        \
  do {                                                                \
    NANO_LOG(severity, CONCAT(format),                                \
             bst::Log::getInstance().getModuleName(), ##__VA_ARGS__); \
  } while (0)

#define LOG_BST_DEBUG(severity, format, ...) \
  do {                                       \
    printf(format, ##__VA_ARGS__);           \
  } while (0)
#else
#define LOG_BST_RELEASE(severity, format, ...) \
  do {                                         \
    printf(format, ##__VA_ARGS__);             \
  } while (0)

#define LOG_BST_DEBUG(severity, format, ...) \
  do {                                       \
    printf(format, ##__VA_ARGS__);           \
  } while (0)

#endif

#ifdef NDEBUG
#define LOG_BST LOG_BST_RELEASE
#else
#define LOG_BST LOG_BST_DEBUG
#endif

#ifdef ENABLE_IFLYTEK_LOG
#define LOG_DEBUG(format, ...) FLOGD(format, ##__VA_ARGS__)
#define LOG_NOTICE(format, ...) FLOGI(format, ##__VA_ARGS__)
#define LOG_WARNING(format, ...) FLOGW(format, ##__VA_ARGS__)
#define LOG_ERROR(format, ...) FLOGE(format, ##__VA_ARGS__)
#define LOG_FATAL(format, ...) FLOGF(format, ##__VA_ARGS__)

#else
#define LOG_DEBUG(format, ...)              \
  do {                                       \
    printf(format, ##__VA_ARGS__);           \
  } while (0)
#define LOG_NOTICE(format, ...)               \
  do {                                        \
    printf(format, ##__VA_ARGS__);            \
  } while (0)
#define LOG_WARNING(format, ...)              \
  do {                                        \
    printf(format, ##__VA_ARGS__);            \
  } while (0)
#define LOG_ERROR(format, ...)                \
  do {                                        \
    printf(format, ##__VA_ARGS__);            \
  } while (0)
#define LOG_FATAL(format, ...)                \
  do {                                        \
    printf(format, ##__VA_ARGS__);            \
  } while (0)
#endif

}  // namespace iflyauto
