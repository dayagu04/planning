/* *
 * Class: CameraStructContainerInterface server declaration
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */
#ifndef IFLYAUTO_CAMERA_STRUCT_CONTAINER_CAMERASTRUCTCONTAINERINTERFACESERVERIMPL
#define IFLYAUTO_CAMERA_STRUCT_CONTAINER_CAMERASTRUCTCONTAINERINTERFACESERVERIMPL

#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include "mdc/common_swc_lib_logger.h"
#include "ara/core/future.h"
#include "ara/com/types.h"
#include "mdc/utility/thread_safe_stack.h"

#include "iflyauto/camera_struct_container/camerastructcontainerinterface_skeleton.h"

namespace iflyauto {
namespace camera_struct_container {

using ara::com::InstanceIdentifier;
using ara::com::MethodCallProcessingMode;
using iflyauto::camera_struct_container::skeleton::CameraStructContainerInterfaceSkeleton;
using eventDataType = iflyauto::camera_struct_container::CameraStructContainer;
using SendEventType = std::shared_ptr<eventDataType>;

namespace {
using LOG_SPACE = mdc::common_swc_lib::CommonSwcLibLogger;
}

class CameraStructContainerInterfaceServerImpl : public CameraStructContainerInterfaceSkeleton {
public:
    explicit CameraStructContainerInterfaceServerImpl(const uint32_t id,
        const ara::com::MethodCallProcessingMode& mode = ara::com::MethodCallProcessingMode::kPoll);

    virtual ~CameraStructContainerInterfaceServerImpl();

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
    /* event relative */
    bool SendEventData(const SendEventType& data);
    void ClearEventContainer();
    
    

private:
    /* event relative */
    std::unique_ptr<std::thread> eventThreadPtr_{nullptr};
    void CreateEventThread();
    void EventNotify();
    void TransferEventData();

    /* 服务实例ID */
    uint32_t instanceId_;

    /* 服务标识 */
    std::atomic<bool> workFlag_;

    /* 提供服务标识 */
    std::atomic<bool> offerServFlag_;

    /* event relative */
    mdc::ThreadSafeStack<SendEventType> eventContainer_;
    std::mutex sendeventMtx_;
    std::condition_variable sendeventCv_;
};
} /* namespace camera_struct_container */
} /* namespace iflyauto */

#endif