/* *
 * Class: ClientServiceContainerInterface server declaration
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */
#ifndef IFLYAUTO_STRUCT_CONTAINER_CLIENTSERVICECONTAINERINTERFACESERVERIMPL
#define IFLYAUTO_STRUCT_CONTAINER_CLIENTSERVICECONTAINERINTERFACESERVERIMPL


#include "mdc/common_swc_lib_logger.h"
#include "ara/core/future.h"
#include "ara/com/types.h"

#include <functional>
#include "iflyauto/struct_container/clientservicecontainerinterface_skeleton.h"

namespace iflyauto {
namespace struct_container {

using ara::com::InstanceIdentifier;
using ara::com::MethodCallProcessingMode;
using iflyauto::struct_container::skeleton::ClientServiceContainerInterfaceSkeleton;


/* method relative */
using methodOutput = ClientServiceContainerInterfaceSkeleton::methodOutput;
using methodHandlerType = std::function<ara::core::Future<methodOutput>(const iflyauto::struct_container::StructContainer&)>;
namespace {
using LOG_SPACE = mdc::common_swc_lib::CommonSwcLibLogger;
}

class ClientServiceContainerInterfaceServerImpl : public ClientServiceContainerInterfaceSkeleton {
public:
    explicit ClientServiceContainerInterfaceServerImpl(const uint32_t id,
        const ara::com::MethodCallProcessingMode& mode = ara::com::MethodCallProcessingMode::kPoll);

    virtual ~ClientServiceContainerInterfaceServerImpl();

    bool Init();

    void Stop();

    inline bool IsStop() const
    {
        return !workFlag_;
    }

    inline uint32_t GetInstanceId() const
    {
        return instanceId_;
    }
    
    /* method relative */
    ara::core::Future<methodOutput> method(const iflyauto::struct_container::StructContainer& msg) override;
    inline void SetmethodHandler(methodHandlerType method)
    {
        methodImpl_ = method;
    }
    

private:
    

    /* 服务实例ID */
    uint32_t instanceId_;

    /* 服务标识 */
    std::atomic<bool> workFlag_;

    /* 提供服务标识 */
    std::atomic<bool> offerServFlag_;

    /* method relative */
    methodHandlerType methodImpl_{nullptr};
};
} /* namespace struct_container */
} /* namespace iflyauto */

#endif