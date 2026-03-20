/* *
 * Class: ClientServiceContainerInterface server declaration
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */
#ifndef IFLYAUTO_STRUCT_CONTAINER_CLIENTSERVICECONTAINERINTERFACESERVER
#define IFLYAUTO_STRUCT_CONTAINER_CLIENTSERVICECONTAINERINTERFACESERVER
#include <memory>





#include "iflyauto/struct_container/clientservicecontainerinterface_skeleton.h"

namespace iflyauto {
namespace struct_container {



/* method relative */
using iflyauto::struct_container::skeleton::ClientServiceContainerInterfaceSkeleton;
using methodOutput = ClientServiceContainerInterfaceSkeleton::methodOutput;
using methodHandlerType = std::function<ara::core::Future<methodOutput>(const iflyauto::struct_container::StructContainer&)>;

class ClientServiceContainerInterfaceServer final {
public:
    ClientServiceContainerInterfaceServer() = delete;

    explicit ClientServiceContainerInterfaceServer(const uint32_t id);

    virtual ~ClientServiceContainerInterfaceServer();

    bool Init();

    void Stop();

    bool IsStop() const;

    uint32_t GetInstanceId() const;

    
    /* method relative */
    ara::core::Future<methodOutput> method(const iflyauto::struct_container::StructContainer& msg);
    void SetmethodHandler(methodHandlerType method);
    

private:
    class Impl;
    std::unique_ptr<Impl> pImpl_;
};
} /* namespace struct_container */
} /* namespace iflyauto */

#endif