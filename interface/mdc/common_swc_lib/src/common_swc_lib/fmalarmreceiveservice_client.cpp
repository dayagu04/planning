/* *
 * CLASS: FmAlarmReceiveService client implementation
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */

#include "mdc/common_swc_lib/fmalarmreceiveservice_client.h"
#include <cstring>
#include <iostream>
#include <algorithm>
#include <unistd.h>
#include <stdio.h>
#include <cstdint>
#include "ara/core/string.h"
#include "mdc/common_swc_lib/fmalarmreceiveservice_client_impl.h"

namespace mdc {
namespace fm {
class FmAlarmReceiveServiceClient::Impl {
public:
    Impl(const uint32_t instanceId)
    {
        FmAlarmReceiveServiceClientPtr_ = std::make_unique<FmAlarmReceiveServiceClientImpl>(instanceId);
    }
    ~Impl() {}
    const std::unique_ptr<FmAlarmReceiveServiceClientImpl>& GetFmAlarmReceiveServiceClient()
    {
        return FmAlarmReceiveServiceClientPtr_;
    }
private:
    std::unique_ptr<FmAlarmReceiveServiceClientImpl> FmAlarmReceiveServiceClientPtr_;
};

FmAlarmReceiveServiceClient::FmAlarmReceiveServiceClient(const uint32_t instanceId)
{
    pImpl_ = std::make_unique<FmAlarmReceiveServiceClient::Impl>(instanceId);
}

FmAlarmReceiveServiceClient::~FmAlarmReceiveServiceClient()
{}

bool FmAlarmReceiveServiceClient::Init()
{
    return pImpl_->GetFmAlarmReceiveServiceClient()->Init();
}

void FmAlarmReceiveServiceClient::Stop()
{
    pImpl_->GetFmAlarmReceiveServiceClient()->Stop();
}



uint32_t FmAlarmReceiveServiceClient::GetRecvQSize() const
{
    return pImpl_->GetFmAlarmReceiveServiceClient()->GetRecvQSize();
}
bool FmAlarmReceiveServiceClient::IsStop() const
{
    return pImpl_->GetFmAlarmReceiveServiceClient()->IsStop();
}
uint32_t FmAlarmReceiveServiceClient::GetInstanceId() const
{
    return pImpl_->GetFmAlarmReceiveServiceClient()->GetInstanceId();
}

/* method relative */
bool FmAlarmReceiveServiceClient::ReportAlarm(const mdc::fm::AlarmInfo& alarmMsg)
{
    return pImpl_->GetFmAlarmReceiveServiceClient()->ReportAlarm(alarmMsg);
}
ReportAlarmOutput FmAlarmReceiveServiceClient::GetReportAlarmData()
{
    return pImpl_->GetFmAlarmReceiveServiceClient()->GetReportAlarmData();
}


} /* namespace fm */
} /* namespace mdc */
