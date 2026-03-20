/* *
 * Class: ClientServiceContainerInterface client declaration
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */

#ifndef IFLYAUTO_STRUCT_CONTAINER_CLIENTSERVICECONTAINERINTERFACECLIENT
#define IFLYAUTO_STRUCT_CONTAINER_CLIENTSERVICECONTAINERINTERFACECLIENT

#include <memory>
#include <vector>
#include <atomic>





#include "iflyauto/struct_container/clientservicecontainerinterface_proxy.h"

namespace iflyauto {
namespace struct_container {

using methodOutput = iflyauto::struct_container::proxy::methods::method::Output;
class ClientServiceContainerInterfaceClient final {
public:
    ClientServiceContainerInterfaceClient() = delete;
    explicit ClientServiceContainerInterfaceClient(const uint32_t instanceId);
    virtual ~ClientServiceContainerInterfaceClient();
    bool Init();
    void Stop();

    

    uint32_t GetRecvQSize() const;
    bool IsStop() const;
    uint32_t GetInstanceId() const;

    /* method relative */
    bool method(const iflyauto::struct_container::StructContainer& msg);
    methodOutput GetmethodData();
    
private:
    class Impl;
    std::unique_ptr<Impl> pImpl_;
};
} /* namespace struct_container */
} /* namespace iflyauto */

#endif