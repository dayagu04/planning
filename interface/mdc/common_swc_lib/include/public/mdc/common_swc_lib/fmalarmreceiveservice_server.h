/* *
 * Class: FmAlarmReceiveService server declaration
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */
#ifndef MDC_FM_FMALARMRECEIVESERVICESERVER
#define MDC_FM_FMALARMRECEIVESERVICESERVER
#include <memory>





#include "mdc/fm/fmalarmreceiveservice_skeleton.h"

namespace mdc {
namespace fm {



/* method relative */
using mdc::fm::skeleton::FmAlarmReceiveServiceSkeleton;
using ReportAlarmOutput = FmAlarmReceiveServiceSkeleton::ReportAlarmOutput;
using ReportAlarmHandlerType = std::function<ara::core::Future<ReportAlarmOutput>(const mdc::fm::AlarmInfo&)>;

class FmAlarmReceiveServiceServer final {
public:
    FmAlarmReceiveServiceServer() = delete;

    explicit FmAlarmReceiveServiceServer(const uint32_t id);

    virtual ~FmAlarmReceiveServiceServer();

    bool Init();

    void Stop();

    bool IsStop() const;

    uint32_t GetInstanceId() const;

    
    /* method relative */
    ara::core::Future<ReportAlarmOutput> ReportAlarm(const mdc::fm::AlarmInfo& alarmMsg);
    void SetReportAlarmHandler(ReportAlarmHandlerType method);
    

private:
    class Impl;
    std::unique_ptr<Impl> pImpl_;
};
} /* namespace fm */
} /* namespace mdc */

#endif