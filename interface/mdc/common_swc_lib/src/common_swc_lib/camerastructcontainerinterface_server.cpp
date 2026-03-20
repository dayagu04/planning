/* *
 * CLASS: CameraStructContainerInterface server implementation
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */

#include "mdc/common_swc_lib/camerastructcontainerinterface_server.h"
#include "mdc/common_swc_lib/camerastructcontainerinterface_server_impl.h"

namespace iflyauto {
namespace camera_struct_container {
class CameraStructContainerInterfaceServer::Impl {
public:
    Impl(const uint32_t id,
        const ara::com::MethodCallProcessingMode& mode = ara::com::MethodCallProcessingMode::kPoll)
    {
        CameraStructContainerInterfaceServerPtr_ = std::make_unique<CameraStructContainerInterfaceServerImpl>(id, mode);
    }
    ~Impl() {}
    const std::unique_ptr<CameraStructContainerInterfaceServerImpl>& GetCameraStructContainerInterfaceServer()
    {
        return CameraStructContainerInterfaceServerPtr_;
    }
private:
    std::unique_ptr<CameraStructContainerInterfaceServerImpl> CameraStructContainerInterfaceServerPtr_;
};

CameraStructContainerInterfaceServer::CameraStructContainerInterfaceServer(const uint32_t id)
{
    pImpl_ = std::make_unique<CameraStructContainerInterfaceServer::Impl>(id);
}

CameraStructContainerInterfaceServer::~CameraStructContainerInterfaceServer()
{}

bool CameraStructContainerInterfaceServer::Init()
{
    return pImpl_->GetCameraStructContainerInterfaceServer()->Init();
}

void CameraStructContainerInterfaceServer::Stop()
{
    pImpl_->GetCameraStructContainerInterfaceServer()->Stop();
}

bool CameraStructContainerInterfaceServer::IsStop() const
{
    return pImpl_->GetCameraStructContainerInterfaceServer()->IsStop();
}

uint32_t CameraStructContainerInterfaceServer::GetInstanceId() const
{
    return pImpl_->GetCameraStructContainerInterfaceServer()->GetInstanceId();
}

/* event relative */
bool CameraStructContainerInterfaceServer::SendEventData(const SendEventType& data)
{
    return pImpl_->GetCameraStructContainerInterfaceServer()->SendEventData(data);
}
void CameraStructContainerInterfaceServer::ClearEventContainer()
{
    pImpl_->GetCameraStructContainerInterfaceServer()->ClearEventContainer();
}

} /* namespace camera_struct_container */
} /* namespace iflyauto */
