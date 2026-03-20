/* *
 * Class: StructContainerInterface server declaration
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */
#ifndef IFLYAUTO_STRUCT_CONTAINER_STRUCTCONTAINERINTERFACESERVERIMPL
#define IFLYAUTO_STRUCT_CONTAINER_STRUCTCONTAINERINTERFACESERVERIMPL

#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include "mdc/common_swc_lib_logger.h"
#include "ara/core/future.h"
#include "ara/com/types.h"
#include "mdc/utility/thread_safe_stack.h"

#include "iflyauto/struct_container/structcontainerinterface_skeleton.h"

namespace iflyauto {
namespace struct_container {

using ara::com::InstanceIdentifier;
using ara::com::MethodCallProcessingMode;
using iflyauto::struct_container::skeleton::StructContainerInterfaceSkeleton;
using eventDataType = ara::diag::ByteVector;
using SendEventType = std::shared_ptr<eventDataType>;

namespace {
using LOG_SPACE = mdc::common_swc_lib::CommonSwcLibLogger;
}

class StructContainerInterfaceServerImpl : public StructContainerInterfaceSkeleton {
public:
    explicit StructContainerInterfaceServerImpl(const uint32_t id,
        const ara::com::MethodCallProcessingMode& mode = ara::com::MethodCallProcessingMode::kPoll);

    virtual ~StructContainerInterfaceServerImpl();

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
    bool SendEventData(const char* data, int32_t size);
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
} /* namespace struct_container */
} /* namespace iflyauto */

#endif