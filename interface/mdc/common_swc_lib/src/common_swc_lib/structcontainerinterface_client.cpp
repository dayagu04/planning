/* *
 * CLASS: StructContainerInterface client implementation
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */

#include "mdc/common_swc_lib/structcontainerinterface_client.h"
#include <cstring>
#include <iostream>
#include <algorithm>
#include <unistd.h>
#include <stdio.h>
#include <cstdint>
#include "ara/core/string.h"
#include "mdc/common_swc_lib/structcontainerinterface_client_impl.h"

namespace iflyauto {
namespace struct_container {
class StructContainerInterfaceClient::Impl {
public:
    Impl(const uint32_t instanceId)
    {
        StructContainerInterfaceClientPtr_ = std::make_unique<StructContainerInterfaceClientImpl>(instanceId);
    }
    ~Impl() {}
    const std::unique_ptr<StructContainerInterfaceClientImpl>& GetStructContainerInterfaceClient()
    {
        return StructContainerInterfaceClientPtr_;
    }
private:
    std::unique_ptr<StructContainerInterfaceClientImpl> StructContainerInterfaceClientPtr_;
};

StructContainerInterfaceClient::StructContainerInterfaceClient(const uint32_t instanceId)
{
    pImpl_ = std::make_unique<StructContainerInterfaceClient::Impl>(instanceId);
}

StructContainerInterfaceClient::~StructContainerInterfaceClient()
{}

bool StructContainerInterfaceClient::Init()
{
    return pImpl_->GetStructContainerInterfaceClient()->Init();
}

void StructContainerInterfaceClient::Stop()
{
    pImpl_->GetStructContainerInterfaceClient()->Stop();
}

/* event relative */
void StructContainerInterfaceClient::RegisterEventNotifyHandler(const eventHandlerType handler)
{
    pImpl_->GetStructContainerInterfaceClient()->RegisterEventNotifyHandler(handler);
}

void StructContainerInterfaceClient::RegisterEventNotifyHandler(const eventHandlerType handler, const std::shared_ptr<ara::com::ThreadGroup>& threadGroup)
{
    pImpl_->GetStructContainerInterfaceClient()->RegisterEventNotifyHandler(handler, threadGroup);
}
void StructContainerInterfaceClient::EventNotify()
{
    pImpl_->GetStructContainerInterfaceClient()->EventNotify();
}
void StructContainerInterfaceClient::EventContainerClear()
{
    pImpl_->GetStructContainerInterfaceClient()->EventContainerClear();
}
bool StructContainerInterfaceClient::EventContainerEmpty()
{
    return pImpl_->GetStructContainerInterfaceClient()->EventContainerEmpty();
}
RecvEventType StructContainerInterfaceClient::GetEventOneData(const uint32_t freshDataTime)
{
    return pImpl_->GetStructContainerInterfaceClient()->GetEventOneData(freshDataTime);
}
RecvEventType StructContainerInterfaceClient::GetEventOneDataBlocking(const uint32_t blockTimeout)
{
    return pImpl_->GetStructContainerInterfaceClient()->GetEventOneDataBlocking(blockTimeout);
}
std::vector<RecvEventType> StructContainerInterfaceClient::GetEventNdata(const size_t n)
{
    return pImpl_->GetStructContainerInterfaceClient()->GetEventNdata(n);
}

uint32_t StructContainerInterfaceClient::GetRecvQSize() const
{
    return pImpl_->GetStructContainerInterfaceClient()->GetRecvQSize();
}
bool StructContainerInterfaceClient::IsStop() const
{
    return pImpl_->GetStructContainerInterfaceClient()->IsStop();
}
uint32_t StructContainerInterfaceClient::GetInstanceId() const
{
    return pImpl_->GetStructContainerInterfaceClient()->GetInstanceId();
}



} /* namespace struct_container */
} /* namespace iflyauto */
