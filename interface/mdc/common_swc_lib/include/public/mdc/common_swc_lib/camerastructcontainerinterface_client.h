/* *
 * Class: CameraStructContainerInterface client declaration
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */

#ifndef IFLYAUTO_CAMERA_STRUCT_CONTAINER_CAMERASTRUCTCONTAINERINTERFACECLIENT
#define IFLYAUTO_CAMERA_STRUCT_CONTAINER_CAMERASTRUCTCONTAINERINTERFACECLIENT

#include <memory>
#include <vector>
#include <atomic>

#include "iflyauto/camera_struct_container/impl_type_camerastructcontainer.h"
#include "ara/com/thread_group_factory.h"

namespace iflyauto {
namespace camera_struct_container {

using eventHandlerType = std::function<void (const iflyauto::camera_struct_container::CameraStructContainer&)>;
using eventDataType = iflyauto::camera_struct_container::CameraStructContainer;
using RecvEventType = std::shared_ptr<eventDataType>;
class CameraStructContainerInterfaceClient final {
public:
    CameraStructContainerInterfaceClient() = delete;
    explicit CameraStructContainerInterfaceClient(const uint32_t instanceId);
    virtual ~CameraStructContainerInterfaceClient();
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
} /* namespace camera_struct_container */
} /* namespace iflyauto */

#endif