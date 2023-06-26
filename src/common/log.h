#pragma once

#include <chrono>
#include "iostream"

#ifndef X86
#include "Nanolog/NanoLogCpp17.h"
#endif

#define CONCAT2(x) "%s " x
#define CONCAT(x) CONCAT2(x)

namespace bst {
enum LogLevel {
  LOGLEVEL_START = 0,
  FETAL,
  ERROR,
  WARNING,
  NOTICE,
  DEBUG,
  LOGLEVEL_END
};

#ifndef X86
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
#ifndef X86
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
#define LOG_BST_RELEASE(severity, format, ...)                  \
  do {                                                          \
    fprintf(bst::Log::getInstance().getModulePointer(), format, \
            ##__VA_ARGS__);                                     \
    fflush(bst::Log::getInstance().getModulePointer());         \
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

#define LOG_DEBUG(format, ...) LOG_BST(LogLevels::DEBUG, format, ##__VA_ARGS__)
#define LOG_NOTICE(format, ...) \
  LOG_BST(LogLevels::NOTICE, format, ##__VA_ARGS__)
#define LOG_WARNING(format, ...) \
  LOG_BST(LogLevels::WARNING, format, ##__VA_ARGS__)
#define LOG_ERROR(format, ...) LOG_BST(LogLevels::ERROR, format, ##__VA_ARGS__)

}  // namespace bst
