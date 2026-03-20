/* *
 * Class: StructContainerInterface server declaration
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */
#ifndef IFLYAUTO_STRUCT_CONTAINER_STRUCTCONTAINERINTERFACESERVER
#define IFLYAUTO_STRUCT_CONTAINER_STRUCTCONTAINERINTERFACESERVER
#include <memory>

#include "ara/diag/impl_type_bytevector.h"





namespace iflyauto {
namespace struct_container {

using eventDataType = ara::diag::ByteVector;
using SendEventType = std::shared_ptr<eventDataType>;


class StructContainerInterfaceServer final {
public:
    StructContainerInterfaceServer() = delete;

    explicit StructContainerInterfaceServer(const uint32_t id);

    virtual ~StructContainerInterfaceServer();

    bool Init();

    void Stop();

    bool IsStop() const;

    uint32_t GetInstanceId() const;

    /* event relative */
    bool SendEventData(const SendEventType& data);
    bool SendEventData(const char* data, int32_t size);
    void ClearEventContainer();
    
    

private:
    class Impl;
    std::unique_ptr<Impl> pImpl_;
};
} /* namespace struct_container */
} /* namespace iflyauto */

#endif