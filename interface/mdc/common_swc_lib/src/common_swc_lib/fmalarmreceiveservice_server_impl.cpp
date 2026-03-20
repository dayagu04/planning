/* *
 * CLASS: FmAlarmReceiveService server implementation
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */

#include "mdc/common_swc_lib/fmalarmreceiveservice_server_impl.h"

namespace mdc {
namespace fm {
FmAlarmReceiveServiceServerImpl::FmAlarmReceiveServiceServerImpl(const uint32_t id,
    const ara::com::MethodCallProcessingMode& mode)
    : FmAlarmReceiveServiceSkeleton(InstanceIdentifier(ara::core::StringView(std::to_string(id).c_str())),
          mode),
      instanceId_(id),
      workFlag_(true),
      offerServFlag_(false)
{}

FmAlarmReceiveServiceServerImpl::~FmAlarmReceiveServiceServerImpl()
{
    if (workFlag_) {
        Stop();
    }
}

bool FmAlarmReceiveServiceServerImpl::Init()
{
    if (offerServFlag_) {
        return true;
    }

    this->OfferService();
    offerServFlag_ = true;
    return true;
}

void FmAlarmReceiveServiceServerImpl::Stop()
{
    workFlag_ = false;
    if (offerServFlag_) {
        this->StopOfferService();
    }
    offerServFlag_ = false;
    LOG_SPACE::GetLoggerIns("SERV")->LogInfo() << "Data send baes stops offer service. idx: " << instanceId_;
}


ara::core::Future<ReportAlarmOutput> FmAlarmReceiveServiceServerImpl::ReportAlarm(const mdc::fm::AlarmInfo& alarmMsg)
{
    if (!reportAlarmImpl_) {
        LOG_SPACE::GetLoggerIns("SERV")->LogWarn() << "Warning: using default ReportAlarm handler!";

        ara::core::Promise<ReportAlarmOutput> promiseData;
        ReportAlarmOutput val;
        promiseData.set_value(val);
        return promiseData.get_future();
    }
    return std::move(reportAlarmImpl_(alarmMsg));
}


} /* namespace fm */
} /* namespace mdc */
