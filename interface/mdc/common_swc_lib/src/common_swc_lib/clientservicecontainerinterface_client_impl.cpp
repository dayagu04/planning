/* *
 * CLASS: ClientServiceContainerInterface client implementation
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */

#include "mdc/common_swc_lib/clientservicecontainerinterface_client_impl.h"
#include <cstring>
#include <iostream>
#include <algorithm>
#include <unistd.h>
#include <stdio.h>
#include <cstdint>
#include "ara/core/string.h"

namespace iflyauto {
namespace struct_container {
ClientServiceContainerInterfaceClientImpl::ClientServiceContainerInterfaceClientImpl(const uint32_t instanceId)
    : instanceIdx_(instanceId),
      workFlag_(true),
      findServFlag_(false),
      registerFlag_(false),
      emReport_(false)
{}

ClientServiceContainerInterfaceClientImpl::~ClientServiceContainerInterfaceClientImpl()
{
    if (workFlag_) {
        Stop();
    }
}

bool ClientServiceContainerInterfaceClientImpl::Init()
{
    if (findServFlag_) {
        return true;
    }
    ara::core::String insIdStr = std::to_string(instanceIdx_);
    serviceHandle_ = ClientServiceContainerInterfaceProxy::StartFindService([this](
        ara::com::ServiceHandleContainer<ClientServiceContainerInterfaceProxy::HandleType> handles,
        const ara::com::FindServiceHandle findServiceHandle) {
            ClientServiceContainerInterfaceCallback(std::move(handles), findServiceHandle);
            },
        ara::com::InstanceIdentifier(ara::core::StringView(insIdStr)));
    (void)serviceHandle_;
    const uint32_t maxTimeoutCount = 10U;
    for (uint32_t timeoutCount = 0U; timeoutCount < maxTimeoutCount; timeoutCount++) {
        if (proxyPtr_ != nullptr) {
            findServFlag_ = true;
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100U));
    }
    LOG_SPACE::GetLoggerIns("CLNT")->LogError() << "Failed to find the service. instance id:" << insIdStr;
    ClientServiceContainerInterfaceProxy::StopFindService(serviceHandle_);
    return false;
}

void ClientServiceContainerInterfaceClientImpl::ClientServiceContainerInterfaceCallback(
    ara::com::ServiceHandleContainer<ClientServiceContainerInterfaceProxy::HandleType> handles,
    const ara::com::FindServiceHandle findServiceHandle)
{
    (void)findServiceHandle;
    if (proxyPtr_ != nullptr) {
        return;
    }

    for (auto singleHandle : handles) {
        ara::core::String insIdStr = std::to_string(instanceIdx_);
        if (singleHandle.GetInstanceId().ToString() == insIdStr) {
            /* 查找到指定服务实例后, 创建服务的Proxy实例 */
            std::call_once(callFlag_, [&singleHandle, this]() {
                this->proxyPtr_ = std::make_unique<ClientServiceContainerInterfaceProxy>(singleHandle);
            });
            break;
        }
    }
    return;
}

void ClientServiceContainerInterfaceClientImpl::Stop()
{
    workFlag_ = false;
    if (findServFlag_) {
        ClientServiceContainerInterfaceProxy::StopFindService(serviceHandle_);
    }
    if (proxyPtr_ != nullptr) {
        
        proxyPtr_ = nullptr;
    }
    LOG_SPACE::GetLoggerIns("CLNT")->LogInfo() << "Data receive baes stoped. idx: " << instanceIdx_;
}

void ClientServiceContainerInterfaceClientImpl::EmReportExec()
{
    const auto sleepInterval = std::chrono::milliseconds(5U);
    const uint8_t EM_REPORT_TIME_MAX = 3U;
    int count = 0;
    ara::core::Result<void> res;
    while (count < EM_REPORT_TIME_MAX) {
        res = execClient_.ReportExecutionState(ara::exec::ExecutionState::kRunning);
        if (res.HasValue()) {
            break;
        } else {
            std::this_thread::sleep_for(sleepInterval);
            count++;
        }
    }
    if (count == EM_REPORT_TIME_MAX) {
        LOG_SPACE::GetLoggerIns("CLNT")->LogError() << "EM report kRunning failed.";
    }
    return;
}

bool ClientServiceContainerInterfaceClientImpl::method(const iflyauto::struct_container::StructContainer& msg)
{
    auto res = proxyPtr_->method(msg).GetResult();
    if (!res.HasValue()) {
        LOG_SPACE::GetLoggerIns("CLNT")->LogError() << "Failed to get method data from server";
        return false;
    }
    methodOutputRes_ = res.Value();
    LOG_SPACE::GetLoggerIns("CLNT")->LogInfo() << "Get method data successfully";
    return true;
}

methodOutput ClientServiceContainerInterfaceClientImpl::GetmethodData()
{
    return methodOutputRes_;
}

} /* namespace struct_container */
} /* namespace iflyauto */
