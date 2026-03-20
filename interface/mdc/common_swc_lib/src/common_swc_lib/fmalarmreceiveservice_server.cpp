/* *
 * CLASS: FmAlarmReceiveService server implementation
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */

#include "mdc/common_swc_lib/fmalarmreceiveservice_server.h"
#include "mdc/common_swc_lib/fmalarmreceiveservice_server_impl.h"

namespace mdc {
namespace fm {
class FmAlarmReceiveServiceServer::Impl {
public:
    Impl(const uint32_t id,
        const ara::com::MethodCallProcessingMode& mode = ara::com::MethodCallProcessingMode::kEvent)
    {
        FmAlarmReceiveServiceServerPtr_ = std::make_unique<FmAlarmReceiveServiceServerImpl>(id, mode);
    }
    ~Impl() {}
    const std::unique_ptr<FmAlarmReceiveServiceServerImpl>& GetFmAlarmReceiveServiceServer()
    {
        return FmAlarmReceiveServiceServerPtr_;
    }
private:
    std::unique_ptr<FmAlarmReceiveServiceServerImpl> FmAlarmReceiveServiceServerPtr_;
};

FmAlarmReceiveServiceServer::FmAlarmReceiveServiceServer(const uint32_t id)
{
    pImpl_ = std::make_unique<FmAlarmReceiveServiceServer::Impl>(id);
}

FmAlarmReceiveServiceServer::~FmAlarmReceiveServiceServer()
{}

bool FmAlarmReceiveServiceServer::Init()
{
    return pImpl_->GetFmAlarmReceiveServiceServer()->Init();
}

void FmAlarmReceiveServiceServer::Stop()
{
    pImpl_->GetFmAlarmReceiveServiceServer()->Stop();
}

bool FmAlarmReceiveServiceServer::IsStop() const
{
    return pImpl_->GetFmAlarmReceiveServiceServer()->IsStop();
}

uint32_t FmAlarmReceiveServiceServer::GetInstanceId() const
{
    return pImpl_->GetFmAlarmReceiveServiceServer()->GetInstanceId();
}

/* method relative */
ara::core::Future<ReportAlarmOutput> FmAlarmReceiveServiceServer::ReportAlarm(const mdc::fm::AlarmInfo& alarmMsg)
{
    return pImpl_->GetFmAlarmReceiveServiceServer()->ReportAlarm(alarmMsg);
}
void FmAlarmReceiveServiceServer::SetReportAlarmHandler(ReportAlarmHandlerType method)
{
    pImpl_->GetFmAlarmReceiveServiceServer()->SetReportAlarmHandler(method);
}

} /* namespace fm */
} /* namespace mdc */
