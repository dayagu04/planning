/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
 * Description: AP-LOG代码模板
 */
#ifndef ARA_APPLICATIONSWCEXECUTABLE2_APPLICATIONSWCEXECUTABLE2_LOGGER_H
#define ARA_APPLICATIONSWCEXECUTABLE2_APPLICATIONSWCEXECUTABLE2_LOGGER_H
#include <cstdint>
#include <string>
#include <map>
#include <mutex>
#include <sys/types.h>
#include <ara/log/logstream.h>
#include "ara/log/logging.h"
#include "ara/per/file_storage.h"
#include "ara/per/key_value_storage.h"
#include "ara/per/kvs_type.h"
#include "ara/core/string.h"
namespace mdc {
namespace applicationswcexecutable2 {
namespace {
using LogStream = ara::log::LogStream;
using ara::per::kvstype::KvsType;
}

struct LogConfig {
    ara::core::String appId;
    ara::core::String appDescription;
    ara::core::String logFilePath;
    ara::log::LogLevel logLevel;
    ara::log::LogMode logMode;
};

class Applicationswcexecutable2Logger {
public:
    virtual ~Applicationswcexecutable2Logger() = default;

    /**
     * @brief 初始化日志功能，在使用日志功能前调用
     *
     */
    static void InitAppLogging();

    /**
     * @brief 初始化日志ID，应在使用不同contexId的log之前调用，保存ctxId及对应ctxDesc，
     * 如果不调用此方法直接通过GetLoggerIns调用LogError等函数，ctxDesc采用appDesc的值,
     * 若调用了此函数但未输入参数，则默认采用appId和appDesc
     */
    static void InitLoggerCtx(const ara::core::String& ctxId, const ara::core::String& ctxDesc);

    /**
     * @brief 通过调用此方法来记录日志，例如输出per相关日志，使用方式为：
     * ara::applicationswcexecutable2::Applicationswcexecutable2Logger::GetLoggerIns("PER")->LogError() << "*******";
     * 建议先调用InitLoggerCtx方法确定contexId和对应context description后再调用此方法
     */
    static ara::log::Logger* GetLoggerIns(const ara::core::String& ctxId = logConfig_.appId)
    {
        return GetLogCtxInstance(ctxId, ctxMap_[ctxId]);
    }
private:
    Applicationswcexecutable2Logger() = default;
    static ara::log::Logger* GetLogCtxInstance(const ara::core::String& ctxId, ara::core::String ctxDescription);
    /* 保存ContexId和Contex Description */
    static std::map<ara::core::String, ara::core::String> ctxMap_;
    /* 保存不同的ContexId和对应的Logger指针 */
    static std::map<ara::core::String, ara::log::Logger*> loggerCtxMap_;

    static std::mutex ctxMapMutex_;
    static std::mutex loggerCtxMapMutex_;

    static LogConfig logConfig_;
    /* 初始化logConfig_ */
    static void SetLogConfig();
    /* 解析process.json文件 */
    static LogConfig ParseLogConfig();
    static bool DoesFileExist(const ara::core::String& filePath);
    static bool IsDirType(const mode_t& fileMode);
    static ara::log::LogMode GetLogMode(const ara::core::Vector<ara::core::String>& modeVec);
    static ara::log::LogLevel GetLogLevel(const ara::core::String& level);
    static ara::core::String GetSpecifyLogConfig(const ara::core::String& logTraceName,
        const ara::core::Vector<KvsType>& logConfigValueCtx);
    static ara::core::Vector<ara::core::String> GetSpecifyLogMode(ara::core::Vector<KvsType>& logConfigValueCtx);
};
} /* namespace applicationswcexecutable2 */
} /* namespace mdc */
#endif  /* ARA_APPLICATIONSWCEXECUTABLE2_APPLICATIONSWCEXECUTABLE2_LOGGER_H */