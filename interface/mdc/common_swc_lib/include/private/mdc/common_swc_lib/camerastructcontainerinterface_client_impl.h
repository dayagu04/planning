/* *
 * Class: CameraStructContainerInterface client declaration
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */

#ifndef IFLYAUTO_CAMERA_STRUCT_CONTAINER_CAMERASTRUCTCONTAINERINTERFACECLIENT_IMPL
#define IFLYAUTO_CAMERA_STRUCT_CONTAINER_CAMERASTRUCTCONTAINERINTERFACECLIENT_IMPL

#include <memory>
#include <vector>
#include <atomic>
#include "iflyauto/camera_struct_container/camerastructcontainerinterface_proxy.h"
#include "ara/exec/execution_client.h"
#include <mutex>
#include <condition_variable>
#include "mdc/utility/thread_safe_stack.h"
#include "mdc/common_swc_lib_logger.h"
#include "ara/com/thread_group_factory.h"

namespace iflyauto {
namespace camera_struct_container {
using CameraStructContainerInterfaceProxy = iflyauto::camera_struct_container::proxy::CameraStructContainerInterfaceProxy;

using eventHandlerType = std::function<void (const iflyauto::camera_struct_container::CameraStructContainer&)>;
using eventDataType = iflyauto::camera_struct_container::CameraStructContainer;
using RecvEventType = std::shared_ptr<eventDataType>;
namespace {
using LOG_SPACE = mdc::common_swc_lib::CommonSwcLibLogger;
}

class CameraStructContainerInterfaceClientImpl {
public:
    explicit CameraStructContainerInterfaceClientImpl(const uint32_t instanceId);
    virtual ~CameraStructContainerInterfaceClientImpl();
    bool Init();
    void Stop();

    /* event relative */
    void RegisterEventNotifyHandler(const eventHandlerType handler);
    void RegisterEventNotifyHandler(const eventHandlerType handler, const std::shared_ptr<ara::com::ThreadGroup>& threadGroup);
    void EventNotify()
    {
        std::lock_guard<std::mutex> recvLk(eventMtx_);
        eventCv_.notify_all();
    }

    void EventContainerClear()
    {
        eventContainer_.Clear();
    }

    bool EventContainerEmpty()
    {
        return eventContainer_.Empty();
    }

    RecvEventType GetEventOneData(const uint32_t freshDataTime = UINT32_MAX);

    RecvEventType GetEventOneDataBlocking(const uint32_t blockTimeout = UINT32_MAX);

    std::vector<RecvEventType> GetEventNdata(const size_t n);
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

    void CameraStructContainerInterfaceCallback(ara::com::ServiceHandleContainer<CameraStructContainerInterfaceProxy::HandleType> handles,
        const ara::com::FindServiceHandle findServiceHandle);
    std::unique_ptr<CameraStructContainerInterfaceProxy> proxyPtr_{nullptr};
    uint32_t recvQSize_{15U};
    ara::com::FindServiceHandle serviceHandle_{};
    void EmReportExec();

    /* event relative */
    mdc::ThreadSafeStack<RecvEventType> eventContainer_;
    std::mutex eventMtx_;
    std::condition_variable eventCv_;
    void PushEventDataToContainer(const eventDataType&& data);
};
} /* namespace camera_struct_container */
} /* namespace iflyauto */
#endif