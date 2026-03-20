/* *
 * Class: StructContainerInterface client declaration
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */

#ifndef IFLYAUTO_STRUCT_CONTAINER_STRUCTCONTAINERINTERFACECLIENT
#define IFLYAUTO_STRUCT_CONTAINER_STRUCTCONTAINERINTERFACECLIENT

#include <memory>
#include <vector>
#include <atomic>

#include "ara/diag/impl_type_bytevector.h"
#include "ara/com/thread_group_factory.h"

namespace iflyauto {
namespace struct_container {

using eventHandlerType = std::function<void (const ara::diag::ByteVector&)>;
using eventDataType = ara::diag::ByteVector;
using RecvEventType = std::shared_ptr<eventDataType>;
class StructContainerInterfaceClient final {
public:
    StructContainerInterfaceClient() = delete;
    explicit StructContainerInterfaceClient(const uint32_t instanceId);
    virtual ~StructContainerInterfaceClient();
    bool Init();
    void Stop();

    /* event relative */
    void RegisterEventNotifyHandler(const eventHandlerType handler);
    void RegisterEventNotifyHandler(const eventHandlerType handler, const std::shared_ptr<ara::com::ThreadGroup>& threadGroup);
    void EventNotify();
    void EventContainerClear();
    bool EventContainerEmpty();
    RecvEventType GetEventOneData(const uint32_t freshDataTime = UINT32_MAX);
    RecvEventType GetEventOneDataBlocking(const uint32_t blockTimeout = UINT32_MAX);
    std::vector<RecvEventType> GetEventNdata(const size_t n);

    uint32_t GetRecvQSize() const;
    bool IsStop() const;
    uint32_t GetInstanceId() const;

    
private:
    class Impl;
    std::unique_ptr<Impl> pImpl_;
};
} /* namespace struct_container */
} /* namespace iflyauto */

#endif