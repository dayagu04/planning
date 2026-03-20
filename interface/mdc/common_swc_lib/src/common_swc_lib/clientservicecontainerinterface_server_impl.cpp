/* *
 * CLASS: ClientServiceContainerInterface server implementation
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */

#include "mdc/common_swc_lib/clientservicecontainerinterface_server_impl.h"

namespace iflyauto {
namespace struct_container {
ClientServiceContainerInterfaceServerImpl::ClientServiceContainerInterfaceServerImpl(const uint32_t id,
    const ara::com::MethodCallProcessingMode& mode)
    : ClientServiceContainerInterfaceSkeleton(InstanceIdentifier(ara::core::StringView(std::to_string(id).c_str())),
          mode),
      instanceId_(id),
      workFlag_(true),
      offerServFlag_(false)
{}

ClientServiceContainerInterfaceServerImpl::~ClientServiceContainerInterfaceServerImpl()
{
    if (workFlag_) {
        Stop();
    }
}

bool ClientServiceContainerInterfaceServerImpl::Init()
{
    if (offerServFlag_) {
        return true;
    }

    this->OfferService();
    offerServFlag_ = true;
    return true;
}

void ClientServiceContainerInterfaceServerImpl::Stop()
{
    workFlag_ = false;
    if (offerServFlag_) {
        this->StopOfferService();
    }
    offerServFlag_ = false;
    LOG_SPACE::GetLoggerIns("SERV")->LogInfo() << "Data send baes stops offer service. idx: " << instanceId_;
}


ara::core::Future<methodOutput> ClientServiceContainerInterfaceServerImpl::method(const iflyauto::struct_container::StructContainer& msg)
{
    if (!methodImpl_) {
        LOG_SPACE::GetLoggerIns("SERV")->LogWarn() << "Warning: using default method handler!";

        ara::core::Promise<methodOutput> promiseData;
        methodOutput val;
        promiseData.set_value(val);
        return promiseData.get_future();
    }
    return std::move(methodImpl_(msg));
}


} /* namespace struct_container */
} /* namespace iflyauto */
