/* *
 * Class: ClientServiceContainerInterface client declaration
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */

#ifndef IFLYAUTO_STRUCT_CONTAINER_CLIENTSERVICECONTAINERINTERFACECLIENT_IMPL
#define IFLYAUTO_STRUCT_CONTAINER_CLIENTSERVICECONTAINERINTERFACECLIENT_IMPL

#include <memory>
#include <vector>
#include <atomic>
#include "iflyauto/struct_container/clientservicecontainerinterface_proxy.h"
#include "ara/exec/execution_client.h"
#include "mdc/common_swc_lib_logger.h"
namespace iflyauto {
namespace struct_container {
using ClientServiceContainerInterfaceProxy = iflyauto::struct_container::proxy::ClientServiceContainerInterfaceProxy;

using methodOutput = iflyauto::struct_container::proxy::methods::method::Output;
namespace {
using LOG_SPACE = mdc::common_swc_lib::CommonSwcLibLogger;
}

class ClientServiceContainerInterfaceClientImpl {
public:
    explicit ClientServiceContainerInterfaceClientImpl(const uint32_t instanceId);
    virtual ~ClientServiceContainerInterfaceClientImpl();
    bool Init();
    void Stop();

    
    uint32_t GetRecvQSize() const
    {
        return recvQSize_;
    }

    bool IsStop() const
    {
        return !workFlag_;
    }

    uint32_t GetInstanceId() const
    {
        return instanceIdx_;
    }
    /* method relative */
    bool method(const iflyauto::struct_container::StructContainer& msg);
    methodOutput GetmethodData();
    
private:
    /* 服务实例ID */
    uint32_t instanceIdx_;

    /* 服务标识 */
    std::atomic<bool> workFlag_;

    /* 寻找服务标识 */
    std::atomic<bool> findServFlag_;

    /* 注册标识 */
    std::atomic<bool> registerFlag_;

    /* EM模块 */
    ara::exec::ExecutionClient execClient_ {};

    /* EM 上报标识 */
    std::atomic<bool> emReport_;

    /* 服务发现回调 避免多线程同时执行标识 */
    std::once_flag callFlag_{};

    void ClientServiceContainerInterfaceCallback(ara::com::ServiceHandleContainer<ClientServiceContainerInterfaceProxy::HandleType> handles,
        const ara::com::FindServiceHandle findServiceHandle);
    std::unique_ptr<ClientServiceContainerInterfaceProxy> proxyPtr_{nullptr};
    uint32_t recvQSize_{15U};
    ara::com::FindServiceHandle serviceHandle_{};
    void EmReportExec();

    methodOutput methodOutputRes_{};
    
};
} /* namespace struct_container */
} /* namespace iflyauto */
#endif