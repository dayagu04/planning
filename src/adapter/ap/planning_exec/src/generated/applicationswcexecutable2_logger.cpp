/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
 * Description: AP-LOG代码模板
 */
#include "mdc/applicationswcexecutable2_logger.h"
#include <sys/stat.h>
#include <climits>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include "ara/core/vector.h"

namespace mdc {
namespace applicationswcexecutable2 {

LogConfig Applicationswcexecutable2Logger::logConfig_ = {
    "Applicationswcexecutable2", "ApplicationSwcExecutable2 context", "",
    ara::log::LogLevel::kInfo, ara::log::LogMode::kRemote};
std::map<ara::core::String, ara::core::String>
    Applicationswcexecutable2Logger::ctxMap_;
std::map<ara::core::String, ara::log::Logger*>
    Applicationswcexecutable2Logger::loggerCtxMap_;
std::mutex Applicationswcexecutable2Logger::ctxMapMutex_;
std::mutex Applicationswcexecutable2Logger::loggerCtxMapMutex_;

void Applicationswcexecutable2Logger::SetLogConfig() {
  logConfig_ = ParseLogConfig();
}

/* 此方法应在main函数调用 */
void Applicationswcexecutable2Logger::InitAppLogging() {
  SetLogConfig();
  if (logConfig_.logFilePath.empty()) {
    ara::log::InitLogging(logConfig_.appId, logConfig_.appDescription,
                          logConfig_.logLevel, logConfig_.logMode);
  } else {
    ara::log::InitLogging(logConfig_.appId, logConfig_.appDescription,
                          logConfig_.logLevel, logConfig_.logMode,
                          logConfig_.logFilePath);
  }
  return;
}

/* 此方法应在使用不同contexId的log之前调用，保存ctxId及对应ctxDesc，如果不调用此方法直接使用LogError等函数，
   则ctxDesc采用appDesc的值,若调用了此函数但未输入参数，则默认采用appId和appDesc
 */
void Applicationswcexecutable2Logger::InitLoggerCtx(
    const ara::core::String& ctxId = logConfig_.appId,
    const ara::core::String& ctxDesc = logConfig_.appDescription) {
  std::unique_lock<std::mutex> mutexLock(ctxMapMutex_);
  const auto ctxPtr = ctxMap_.find(ctxId);
  if (ctxPtr == ctxMap_.end()) {
    (void)ctxMap_.emplace(ctxId, ctxDesc);
  }
  return;
}

ara::log::Logger* Applicationswcexecutable2Logger::GetLogCtxInstance(
    const ara::core::String& ctxId, ara::core::String ctxDescription) {
  std::unique_lock<std::mutex> mutexLock(loggerCtxMapMutex_);
  const auto loggerPtr = loggerCtxMap_.find(ctxId);
  if (loggerPtr == loggerCtxMap_.end()) {
    if (ctxDescription.empty()) {
      ctxDescription = logConfig_.appDescription;
    }
    ara::log::Logger* logCtx =
        &ara::log::CreateLogger(ctxId, ctxDescription, logConfig_.logLevel);
    (void)loggerCtxMap_.emplace(ctxId, logCtx);
  }
  return loggerCtxMap_[ctxId];
}

ara::core::String Applicationswcexecutable2Logger::GetSpecifyLogConfig(
    const ara::core::String& logTraceName,
    const ara::core::Vector<KvsType>& logConfigValueCtx) {
  ara::core::String res;
  for (const auto& it : logConfigValueCtx) {
    if (it.GetKey() == logTraceName) {
      auto itRes = it.GetKvsString();
      if (!itRes.HasValue()) {
        return res;
      }
      return itRes.Value();
    }
  }
  return res;
}

ara::core::Vector<ara::core::String>
Applicationswcexecutable2Logger::GetSpecifyLogMode(
    ara::core::Vector<KvsType>& logConfigValueCtx) {
  ara::core::Vector<ara::core::String> res;
  for (auto& it : logConfigValueCtx) {
    if (it.GetKey() == "logTraceLogMode") {
      auto itRes = it.GetKvsArray<ara::core::String>();
      if (!itRes.HasValue()) {
        return res;
      }
      return itRes.Value();
    }
  }
  return res;
}

ara::log::LogLevel Applicationswcexecutable2Logger::GetLogLevel(
    const ara::core::String& level) {
  if (level == "info") {
    return ara::log::LogLevel::kInfo;
  }
  if (level == "fatal") {
    return ara::log::LogLevel::kFatal;
  }
  if (level == "warn") {
    return ara::log::LogLevel::kWarn;
  }
  if (level == "debug") {
    return ara::log::LogLevel::kDebug;
  }
  if (level == "verbose") {
    return ara::log::LogLevel::kVerbose;
  }
  if (level == "error") {
    return ara::log::LogLevel::kError;
  }
  return ara::log::LogLevel::kInfo;
}

ara::log::LogMode Applicationswcexecutable2Logger::GetLogMode(
    const ara::core::Vector<ara::core::String>& modeVec) {
  ara::log::LogMode logMode = ara::log::LogMode::kRemote;
  for (auto& it : modeVec) {
    if (it == "file") {
      logMode = logMode | ara::log::LogMode::kFile;
    }
    if (it == "console") {
      logMode = logMode | ara::log::LogMode::kConsole;
    }
    if (it == "network") {
      logMode = logMode | ara::log::LogMode::kRemote;
    }
  }
  return logMode;
}

LogConfig Applicationswcexecutable2Logger::ParseLogConfig() {
  ara::core::String filePath;
  ara::log::LogMode kLogMode = ara::log::LogMode::kRemote;
  LogConfig logConfig = {"SCFI", "ApplicationSwcExecutable2 context", filePath,
                         ara::log::LogLevel::kInfo, kLogMode};
  char* path = secure_getenv("CM_CONFIG_FILE_PATH");
  if (path == nullptr) {
    return logConfig;
  }
  char fileRealPath[PATH_MAX + 1] = {0};
  if (realpath(path, &fileRealPath[0]) == nullptr) {
    return logConfig;
  }

  ara::core::String configFile = path;
  configFile += "/process.json";

  /* 配置文件路径为空或非yaml文件 */
  if (configFile.empty()) {
    return logConfig;
  }

  if (!DoesFileExist(configFile)) {
    return logConfig;
  }

  auto storage =
      ara::per::OpenKeyValueStorage(ara::core::StringView(configFile));
  if (!storage.HasValue()) {
    return logConfig;
  }
  auto logStorage = std::move(storage).Value();
  auto logKeys = logStorage->GetAllKeys();
  if (!logKeys.HasValue()) {
    return logConfig;
  }
  auto logValue = std::move(logKeys).Value();
  const ara::core::String klogTrace = "logTraceProcess";
  if (std::count(logValue.begin(), logValue.end(), klogTrace) < 1) {
    return logConfig;
  }
  KvsType logConfigValue;
  (void)logStorage->GetValue(klogTrace, logConfigValue);
  auto logConfigValueRes = logConfigValue.GetKvsArray<KvsType>();
  if (!logConfigValueRes.HasValue()) {
    return logConfig;
  }
  auto logConfigValueCtx = logConfigValueRes.Value();

  auto logTraceProcessId =
      GetSpecifyLogConfig("logTraceProcessId", logConfigValueCtx);
  if (!logTraceProcessId.empty()) {
    logConfig.appId = logTraceProcessId;
  }
  auto logTraceProcessDes =
      GetSpecifyLogConfig("logTraceProcessDes", logConfigValueCtx);
  if (!logTraceProcessDes.empty()) {
    logConfig.appDescription = logTraceProcessDes;
  }
  auto logTraceDefaultLogLevel =
      GetSpecifyLogConfig("logTraceDefaultLogLevel", logConfigValueCtx);
  if (!logTraceDefaultLogLevel.empty()) {
    logConfig.logLevel = GetLogLevel(logTraceDefaultLogLevel);
  }
  auto logTraceLogMode = GetSpecifyLogMode(logConfigValueCtx);
  if (!logTraceLogMode.empty()) {
    logConfig.logMode = GetLogMode(logTraceLogMode);
  }
  auto logTraceFilePath =
      GetSpecifyLogConfig("logTraceFilePath", logConfigValueCtx);
  if (!logTraceFilePath.empty()) {
    logConfig.logFilePath = logTraceFilePath;
  }
  return logConfig;
}

bool Applicationswcexecutable2Logger::DoesFileExist(
    const ara::core::String& filePath) {
  struct stat fileInfo = {};
  if (memset_s(&fileInfo, sizeof(fileInfo), 0, sizeof(fileInfo)) != 0) {
    return false;
  }
  if (lstat(filePath.c_str(), &fileInfo) != 0) {
    return false;
  }
  if (!IsDirType(fileInfo.st_mode)) {
    return true;
  }
  return false;
}

bool Applicationswcexecutable2Logger::IsDirType(const mode_t& fileMode) {
  if (S_ISDIR(fileMode)) {
    return true;
  }
  return false;
}
} /* namespace applicationswcexecutable2 */
} /* namespace mdc */