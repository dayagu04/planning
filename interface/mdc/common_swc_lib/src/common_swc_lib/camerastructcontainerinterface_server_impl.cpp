/* *
 * CLASS: CameraStructContainerInterface server implementation
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */

#include "mdc/common_swc_lib/camerastructcontainerinterface_server_impl.h"

namespace iflyauto {
namespace camera_struct_container {
CameraStructContainerInterfaceServerImpl::CameraStructContainerInterfaceServerImpl(const uint32_t id,
    const ara::com::MethodCallProcessingMode& mode)
    : CameraStructContainerInterfaceSkeleton(InstanceIdentifier(ara::core::StringView(std::to_string(id).c_str())),
          mode),
      instanceId_(id),
      workFlag_(true),
      offerServFlag_(false)
{}

CameraStructContainerInterfaceServerImpl::~CameraStructContainerInterfaceServerImpl()
{
    if (workFlag_) {
        Stop();
    }
}

bool CameraStructContainerInterfaceServerImpl::Init()
{
    if (offerServFlag_) {
        return true;
    }

    this->OfferService();
    offerServFlag_ = true;
    return true;
}

void CameraStructContainerInterfaceServerImpl::Stop()
{
    workFlag_ = false;
    if (offerServFlag_) {
        this->StopOfferService();
    }
    offerServFlag_ = false;
    LOG_SPACE::GetLoggerIns("SERV")->LogInfo() << "Data send baes stops offer service. idx: " << instanceId_;
}


void CameraStructContainerInterfaceServerImpl::EventNotify()
{
    sendeventCv_.notify_one();
    return;
}

bool CameraStructContainerInterfaceServerImpl::SendEventData(const SendEventType& data)
{
    if (!data) {
        LOG_SPACE::GetLoggerIns("SERV")->LogInfo() << "The input data is empty";
        return false;
    }
    // auto val = this->event.Allocate();
    // *val = *data;
    this->event.Send(*data);
    LOG_SPACE::GetLoggerIns("SERV")->LogInfo() << "Skeleton send object success! Instance id: "
                                               << GetInstanceId();
    return true;
}
void CameraStructContainerInterfaceServerImpl::TransferEventData()
{
    while (workFlag_) {
        std::unique_lock<std::mutex> lck(sendeventMtx_);
        std::cv_status status = sendeventCv_.wait_for(lck, std::chrono::seconds(1U));
        if (!workFlag_) {
            LOG_SPACE::GetLoggerIns("SERV")->LogWarn() << "Got StopFlag! Sending function return";
            return;
        }
        if (status == std::cv_status::timeout) {
            LOG_SPACE::GetLoggerIns("SERV")->LogWarn() << "Timeout while waiting for condition variable";
            continue;
        }

        auto data = *eventContainer_.Pop();
        if (!data) {
            LOG_SPACE::GetLoggerIns("SERV")->LogError() << "The data to be sent is empty";
            continue;
        }
        lck.unlock();

        auto val = this->event.Allocate();
        *val = *data;
        this->event.Send(std::move(val));
        LOG_SPACE::GetLoggerIns("SERV")->LogInfo() << "Skeleton send object success! Instance id: "
                  << GetInstanceId();
    }
}

void CameraStructContainerInterfaceServerImpl::ClearEventContainer()
{
    std::lock_guard<std::mutex> lck(sendeventMtx_);
    eventContainer_.Clear();
    return;
}

void CameraStructContainerInterfaceServerImpl::CreateEventThread()
{
    eventThreadPtr_ = std::make_unique<std::thread>(
        std::bind(&CameraStructContainerInterfaceServerImpl::TransferEventData, this));
}

} /* namespace camera_struct_container */
} /* namespace iflyauto */
