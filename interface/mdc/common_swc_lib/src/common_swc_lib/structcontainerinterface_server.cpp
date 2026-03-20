/* *
 * CLASS: StructContainerInterface server implementation
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */

#include "mdc/common_swc_lib/structcontainerinterface_server.h"
#include "mdc/common_swc_lib/structcontainerinterface_server_impl.h"

namespace iflyauto {
namespace struct_container {
class StructContainerInterfaceServer::Impl {
public:
    Impl(const uint32_t id,
        const ara::com::MethodCallProcessingMode& mode = ara::com::MethodCallProcessingMode::kPoll)
    {
        StructContainerInterfaceServerPtr_ = std::make_unique<StructContainerInterfaceServerImpl>(id, mode);
    }
    ~Impl() {}
    const std::unique_ptr<StructContainerInterfaceServerImpl>& GetStructContainerInterfaceServer()
    {
        return StructContainerInterfaceServerPtr_;
    }
private:
    std::unique_ptr<StructContainerInterfaceServerImpl> StructContainerInterfaceServerPtr_;
};

StructContainerInterfaceServer::StructContainerInterfaceServer(const uint32_t id)
{
    pImpl_ = std::make_unique<StructContainerInterfaceServer::Impl>(id);
}

StructContainerInterfaceServer::~StructContainerInterfaceServer()
{}

bool StructContainerInterfaceServer::Init()
{
    return pImpl_->GetStructContainerInterfaceServer()->Init();
}

void StructContainerInterfaceServer::Stop()
{
    pImpl_->GetStructContainerInterfaceServer()->Stop();
}

bool StructContainerInterfaceServer::IsStop() const
{
    return pImpl_->GetStructContainerInterfaceServer()->IsStop();
}

uint32_t StructContainerInterfaceServer::GetInstanceId() const
{
    return pImpl_->GetStructContainerInterfaceServer()->GetInstanceId();
}

/* event relative */
bool StructContainerInterfaceServer::SendEventData(const SendEventType& data)
{
    return pImpl_->GetStructContainerInterfaceServer()->SendEventData(data);
}

bool StructContainerInterfaceServer::SendEventData(const char* data, int32_t size)
{
    return pImpl_->GetStructContainerInterfaceServer()->SendEventData(data, size);
}
void StructContainerInterfaceServer::ClearEventContainer()
{
    pImpl_->GetStructContainerInterfaceServer()->ClearEventContainer();
}

} /* namespace struct_container */
} /* namespace iflyauto */
