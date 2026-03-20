/* *
 * Class: FmAlarmReceiveService server declaration
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */
#ifndef MDC_FM_FMALARMRECEIVESERVICESERVERIMPL
#define MDC_FM_FMALARMRECEIVESERVICESERVERIMPL


#include "mdc/common_swc_lib_logger.h"
#include "ara/core/future.h"
#include "ara/com/types.h"

#include <functional>
#include "mdc/fm/fmalarmreceiveservice_skeleton.h"

namespace mdc {
namespace fm {

using ara::com::InstanceIdentifier;
using ara::com::MethodCallProcessingMode;
using mdc::fm::skeleton::FmAlarmReceiveServiceSkeleton;


/* method relative */
using ReportAlarmOutput = FmAlarmReceiveServiceSkeleton::ReportAlarmOutput;
using ReportAlarmHandlerType = std::function<ara::core::Future<ReportAlarmOutput>(const mdc::fm::AlarmInfo&)>;
namespace {
using LOG_SPACE = mdc::common_swc_lib::CommonSwcLibLogger;
}

class FmAlarmReceiveServiceServerImpl : public FmAlarmReceiveServiceSkeleton {
public:
    explicit FmAlarmReceiveServiceServerImpl(const uint32_t id,
        const ara::com::MethodCallProcessingMode& mode = ara::com::MethodCallProcessingMode::kPoll);

    virtual ~FmAlarmReceiveServiceServerImpl();

    bool Init();

    void Stop();

    inline bool IsStop() const
    {
        return !workFlag_;
    }

    inline uint32_t GetInstanceId() const
    {
        return instanceId_;
    }
    
    /* method relative */
    ara::core::Future<ReportAlarmOutput> ReportAlarm(const mdc::fm::AlarmInfo& alarmMsg) override;
    inline void SetReportAlarmHandler(ReportAlarmHandlerType method)
    {
        reportAlarmImpl_ = method;
    }
    

private:
    

    /* 服务实例ID */
    uint32_t instanceId_;

    /* 服务标识 */
    std::atomic<bool> workFlag_;

    /* 提供服务标识 */
    std::atomic<bool> offerServFlag_;

    /* method relative */
    ReportAlarmHandlerType reportAlarmImpl_{nullptr};
};
} /* namespace fm */
} /* namespace mdc */

#endif