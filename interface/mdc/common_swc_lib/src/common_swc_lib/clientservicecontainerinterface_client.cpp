/* *
 * CLASS: ClientServiceContainerInterface client implementation
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */

#include "mdc/common_swc_lib/clientservicecontainerinterface_client.h"
#include <cstring>
#include <iostream>
#include <algorithm>
#include <unistd.h>
#include <stdio.h>
#include <cstdint>
#include "ara/core/string.h"
#include "mdc/common_swc_lib/clientservicecontainerinterface_client_impl.h"

namespace iflyauto {
namespace struct_container {
class ClientServiceContainerInterfaceClient::Impl {
public:
    Impl(const uint32_t instanceId)
    {
        ClientServiceContainerInterfaceClientPtr_ = std::make_unique<ClientServiceContainerInterfaceClientImpl>(instanceId);
    }
    ~Impl() {}
    const std::unique_ptr<ClientServiceContainerInterfaceClientImpl>& GetClientServiceContainerInterfaceClient()
    {
        return ClientServiceContainerInterfaceClientPtr_;
    }
private:
    std::unique_ptr<ClientServiceContainerInterfaceClientImpl> ClientServiceContainerInterfaceClientPtr_;
};

ClientServiceContainerInterfaceClient::ClientServiceContainerInterfaceClient(const uint32_t instanceId)
{
    pImpl_ = std::make_unique<ClientServiceContainerInterfaceClient::Impl>(instanceId);
}

ClientServiceContainerInterfaceClient::~ClientServiceContainerInterfaceClient()
{}

bool ClientServiceContainerInterfaceClient::Init()
{
    return pImpl_->GetClientServiceContainerInterfaceClient()->Init();
}

void ClientServiceContainerInterfaceClient::Stop()
{
    pImpl_->GetClientServiceContainerInterfaceClient()->Stop();
}



uint32_t ClientServiceContainerInterfaceClient::GetRecvQSize() const
{
    return pImpl_->GetClientServiceContainerInterfaceClient()->GetRecvQSize();
}
bool ClientServiceContainerInterfaceClient::IsStop() const
{
    return pImpl_->GetClientServiceContainerInterfaceClient()->IsStop();
}
uint32_t ClientServiceContainerInterfaceClient::GetInstanceId() const
{
    return pImpl_->GetClientServiceContainerInterfaceClient()->GetInstanceId();
}

/* method relative */
bool ClientServiceContainerInterfaceClient::method(const iflyauto::struct_container::StructContainer& msg)
{
    return pImpl_->GetClientServiceContainerInterfaceClient()->method(msg);
}
methodOutput ClientServiceContainerInterfaceClient::GetmethodData()
{
    return pImpl_->GetClientServiceContainerInterfaceClient()->GetmethodData();
}


} /* namespace struct_container */
} /* namespace iflyauto */
