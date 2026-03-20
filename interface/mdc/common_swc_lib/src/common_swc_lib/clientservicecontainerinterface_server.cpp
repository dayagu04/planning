/* *
 * CLASS: ClientServiceContainerInterface server implementation
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */

#include "mdc/common_swc_lib/clientservicecontainerinterface_server.h"
#include "mdc/common_swc_lib/clientservicecontainerinterface_server_impl.h"

namespace iflyauto {
namespace struct_container {
class ClientServiceContainerInterfaceServer::Impl {
public:
    Impl(const uint32_t id,
        const ara::com::MethodCallProcessingMode& mode = ara::com::MethodCallProcessingMode::kEvent)
    {
        ClientServiceContainerInterfaceServerPtr_ = std::make_unique<ClientServiceContainerInterfaceServerImpl>(id, mode);
    }
    ~Impl() {}
    const std::unique_ptr<ClientServiceContainerInterfaceServerImpl>& GetClientServiceContainerInterfaceServer()
    {
        return ClientServiceContainerInterfaceServerPtr_;
    }
private:
    std::unique_ptr<ClientServiceContainerInterfaceServerImpl> ClientServiceContainerInterfaceServerPtr_;
};

ClientServiceContainerInterfaceServer::ClientServiceContainerInterfaceServer(const uint32_t id)
{
    pImpl_ = std::make_unique<ClientServiceContainerInterfaceServer::Impl>(id);
}

ClientServiceContainerInterfaceServer::~ClientServiceContainerInterfaceServer()
{}

bool ClientServiceContainerInterfaceServer::Init()
{
    return pImpl_->GetClientServiceContainerInterfaceServer()->Init();
}

void ClientServiceContainerInterfaceServer::Stop()
{
    pImpl_->GetClientServiceContainerInterfaceServer()->Stop();
}

bool ClientServiceContainerInterfaceServer::IsStop() const
{
    return pImpl_->GetClientServiceContainerInterfaceServer()->IsStop();
}

uint32_t ClientServiceContainerInterfaceServer::GetInstanceId() const
{
    return pImpl_->GetClientServiceContainerInterfaceServer()->GetInstanceId();
}

/* method relative */
ara::core::Future<methodOutput> ClientServiceContainerInterfaceServer::method(const iflyauto::struct_container::StructContainer& msg)
{
    return pImpl_->GetClientServiceContainerInterfaceServer()->method(msg);
}
void ClientServiceContainerInterfaceServer::SetmethodHandler(methodHandlerType method)
{
    pImpl_->GetClientServiceContainerInterfaceServer()->SetmethodHandler(method);
}

} /* namespace struct_container */
} /* namespace iflyauto */
