/* *
 * CLASS: StructContainerInterface server implementation
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */

#include "mdc/common_swc_lib/structcontainerinterface_server_impl.h"

namespace iflyauto {
namespace struct_container {
StructContainerInterfaceServerImpl::StructContainerInterfaceServerImpl(const uint32_t id,
    const ara::com::MethodCallProcessingMode& mode)
    : StructContainerInterfaceSkeleton(InstanceIdentifier(ara::core::StringView(std::to_string(id).c_str())),
          mode),
      instanceId_(id),
      workFlag_(true),
      offerServFlag_(false)
{}

StructContainerInterfaceServerImpl::~StructContainerInterfaceServerImpl()
{
    if (workFlag_) {
        Stop();
    }
}

bool StructContainerInterfaceServerImpl::Init()
{
    if (offerServFlag_) {
        return true;
    }

    this->OfferService();
    offerServFlag_ = true;
    return true;
}

void StructContainerInterfaceServerImpl::Stop()
{
    workFlag_ = false;
    if (offerServFlag_) {
        this->StopOfferService();
    }
    offerServFlag_ = false;
    LOG_SPACE::GetLoggerIns("SERV")->LogInfo() << "Data send baes stops offer service. idx: " << instanceId_;
}


void StructContainerInterfaceServerImpl::EventNotify()
{
    sendeventCv_.notify_one();
    return;
}

bool StructContainerInterfaceServerImpl::SendEventData(const SendEventType& data)
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

bool StructContainerInterfaceServerImpl::SendEventData(const char* data, int32_t size)
{
    if (!data) {
        LOG_SPACE::GetLoggerIns("SERV")->LogInfo() << "The input data is empty";
        return false;
    }
    std::vector<uint8_t> val(data, data+size);
    this->event.Send(val);
    return true;
}
void StructContainerInterfaceServerImpl::TransferEventData()
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

void StructContainerInterfaceServerImpl::ClearEventContainer()
{
    std::lock_guard<std::mutex> lck(sendeventMtx_);
    eventContainer_.Clear();
    return;
}

void StructContainerInterfaceServerImpl::CreateEventThread()
{
    eventThreadPtr_ = std::make_unique<std::thread>(
        std::bind(&StructContainerInterfaceServerImpl::TransferEventData, this));
}

} /* namespace struct_container */
} /* namespace iflyauto */
