/* *
 * CLASS: CameraStructContainerInterface client implementation
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */

#include "mdc/common_swc_lib/camerastructcontainerinterface_client.h"
#include <cstring>
#include <iostream>
#include <algorithm>
#include <unistd.h>
#include <stdio.h>
#include <cstdint>
#include "ara/core/string.h"
#include "mdc/common_swc_lib/camerastructcontainerinterface_client_impl.h"

namespace iflyauto {
namespace camera_struct_container {
class CameraStructContainerInterfaceClient::Impl {
public:
    Impl(const uint32_t instanceId)
    {
        CameraStructContainerInterfaceClientPtr_ = std::make_unique<CameraStructContainerInterfaceClientImpl>(instanceId);
    }
    ~Impl() {}
    const std::unique_ptr<CameraStructContainerInterfaceClientImpl>& GetCameraStructContainerInterfaceClient()
    {
        return CameraStructContainerInterfaceClientPtr_;
    }
private:
    std::unique_ptr<CameraStructContainerInterfaceClientImpl> CameraStructContainerInterfaceClientPtr_;
};

CameraStructContainerInterfaceClient::CameraStructContainerInterfaceClient(const uint32_t instanceId)
{
    pImpl_ = std::make_unique<CameraStructContainerInterfaceClient::Impl>(instanceId);
}

CameraStructContainerInterfaceClient::~CameraStructContainerInterfaceClient()
{}

bool CameraStructContainerInterfaceClient::Init()
{
    return pImpl_->GetCameraStructContainerInterfaceClient()->Init();
}

void CameraStructContainerInterfaceClient::Stop()
{
    pImpl_->GetCameraStructContainerInterfaceClient()->Stop();
}

/* event relative */
void CameraStructContainerInterfaceClient::RegisterEventNotifyHandler(const eventHandlerType handler)
{
    pImpl_->GetCameraStructContainerInterfaceClient()->RegisterEventNotifyHandler(handler);
}
void CameraStructContainerInterfaceClient::RegisterEventNotifyHandler(const eventHandlerType handler, const std::shared_ptr<ara::com::ThreadGroup>& threadGroup)
{
    pImpl_->GetCameraStructContainerInterfaceClient()->RegisterEventNotifyHandler(handler, threadGroup);
}
void CameraStructContainerInterfaceClient::EventNotify()
{
    pImpl_->GetCameraStructContainerInterfaceClient()->EventNotify();
}
void CameraStructContainerInterfaceClient::EventContainerClear()
{
    pImpl_->GetCameraStructContainerInterfaceClient()->EventContainerClear();
}
bool CameraStructContainerInterfaceClient::EventContainerEmpty()
{
    return pImpl_->GetCameraStructContainerInterfaceClient()->EventContainerEmpty();
}
RecvEventType CameraStructContainerInterfaceClient::GetEventOneData(const uint32_t freshDataTime)
{
    return pImpl_->GetCameraStructContainerInterfaceClient()->GetEventOneData(freshDataTime);
}
RecvEventType CameraStructContainerInterfaceClient::GetEventOneDataBlocking(const uint32_t blockTimeout)
{
    return pImpl_->GetCameraStructContainerInterfaceClient()->GetEventOneDataBlocking(blockTimeout);
}
std::vector<RecvEventType> CameraStructContainerInterfaceClient::GetEventNdata(const size_t n)
{
    return pImpl_->GetCameraStructContainerInterfaceClient()->GetEventNdata(n);
}

uint32_t CameraStructContainerInterfaceClient::GetRecvQSize() const
{
    return pImpl_->GetCameraStructContainerInterfaceClient()->GetRecvQSize();
}
bool CameraStructContainerInterfaceClient::IsStop() const
{
    return pImpl_->GetCameraStructContainerInterfaceClient()->IsStop();
}
uint32_t CameraStructContainerInterfaceClient::GetInstanceId() const
{
    return pImpl_->GetCameraStructContainerInterfaceClient()->GetInstanceId();
}



} /* namespace camera_struct_container */
} /* namespace iflyauto */
