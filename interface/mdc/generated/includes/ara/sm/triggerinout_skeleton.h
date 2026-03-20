/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef ARA_SM_TRIGGERINOUT_SKELETON_H
#define ARA_SM_TRIGGERINOUT_SKELETON_H

#include "ara/com/internal/skeleton/skeleton_adapter.h"
#include "ara/com/internal/skeleton/event_adapter.h"
#include "ara/com/internal/skeleton/field_adapter.h"
#include "ara/com/internal/skeleton/method_adapter.h"
#include "ara/com/crc_verification.h"
#include "ara/sm/triggerinout_common.h"
#include <cstdint>

namespace ara {
namespace sm {
namespace skeleton {
namespace events
{
}

namespace methods
{
    using AcquireFunctionGroupInfoHandle = ara::com::internal::skeleton::method::MethodAdapter;
    using ProcessSyncRequestHandle = ara::com::internal::skeleton::method::MethodAdapter;
    using ResetSystemHandle = ara::com::internal::skeleton::method::MethodAdapter;
    using ProcessAsyncRequestHandle = ara::com::internal::skeleton::method::MethodAdapter;
    static constexpr ara::com::internal::EntityId TriggerInOutAcquireFunctionGroupInfoId = 36548U; //AcquireFunctionGroupInfo_method_hash
    static constexpr ara::com::internal::EntityId TriggerInOutProcessSyncRequestId = 8148U; //ProcessSyncRequest_method_hash
    static constexpr ara::com::internal::EntityId TriggerInOutResetSystemId = 43154U; //ResetSystem_method_hash
    static constexpr ara::com::internal::EntityId TriggerInOutProcessAsyncRequestId = 477U; //ProcessAsyncRequest_method_hash
}

namespace fields
{
    using Notifier = ara::com::internal::skeleton::field::FieldAdapter<::ara::sm::TriggerDataType>;
    using Trigger = ara::com::internal::skeleton::field::FieldAdapter<::ara::sm::TriggerDataType>;
    static constexpr ara::com::internal::EntityId TriggerInOutNotifierId = 54564U; //Notifier_field_hash
    static constexpr ara::com::internal::EntityId TriggerInOutNotifierGetterId = 21625U; //Notifier_getter_hash
    static constexpr ara::com::internal::EntityId TriggerInOutTriggerId = 24507U; //Trigger_field_hash
    static constexpr ara::com::internal::EntityId TriggerInOutTriggerSetterId = 57439U; //Trigger_setter_hash
}

class TriggerInOutSkeleton {
private:
    std::unique_ptr<ara::com::internal::skeleton::SkeletonAdapter> skeletonAdapter;
    void ConstructSkeleton(const ara::com::MethodCallProcessingMode mode)
    {
        if (mode == ara::com::MethodCallProcessingMode::kEvent) {
            if (!(skeletonAdapter->SetMethodThreadNumber(skeletonAdapter->GetMethodThreadNumber(6U), 1024U))) {
#ifndef NOT_SUPPORT_EXCEPTIONS
                ara::core::ErrorCode errorcode(ara::com::ComErrc::kNetworkBindingFailure);
                throw ara::com::ComException(std::move(errorcode));
#else
                std::cerr << "Error: Not support exception, create skeleton failed!" << std::endl;
#endif
            }
        }
        const ara::core::Result<void> resultAcquireFunctionGroupInfo = AcquireFunctionGroupInfoHandle.Initialize<ara::core::Future<AcquireFunctionGroupInfoOutput>>();
        ThrowError(resultAcquireFunctionGroupInfo);
        const ara::core::Result<void> resultProcessSyncRequest = ProcessSyncRequestHandle.Initialize<ara::core::Future<ProcessSyncRequestOutput>>();
        ThrowError(resultProcessSyncRequest);
        const ara::core::Result<void> resultResetSystem = ResetSystemHandle.Initialize<ara::core::Future<ResetSystemOutput>>();
        ThrowError(resultResetSystem);
        const ara::core::Result<void> resultProcessAsyncRequest = ProcessAsyncRequestHandle.Initialize<ProcessAsyncRequestOutput>();
        ThrowError(resultProcessAsyncRequest);
        Notifier.SetGetterEntityId(fields::TriggerInOutNotifierGetterId);
        const ara::core::Result<void> resultNotifier = Notifier.Initialize();
        ThrowError(resultNotifier);
        Trigger.SetSetterEntityId(fields::TriggerInOutTriggerSetterId);
        const ara::core::Result<void> resultTrigger = Trigger.Initialize();
        ThrowError(resultTrigger);
    }

    TriggerInOutSkeleton& operator=(const TriggerInOutSkeleton&) = delete;

    static void ThrowError(const ara::core::Result<void>& result)
    {
        if (!(result.HasValue())) {
#ifndef NOT_SUPPORT_EXCEPTIONS
            ara::core::ErrorCode errorcode(result.Error());
            throw ara::com::ComException(std::move(errorcode));
#else
            std::cerr << "Error: Not support exception, create skeleton failed!" << std::endl;
#endif
        }
    }
public:
    using AcquireFunctionGroupInfoOutput = ara::sm::methods::AcquireFunctionGroupInfo::Output;
    
    using ProcessSyncRequestOutput = void;
    
    using ResetSystemOutput = void;
    
    using ProcessAsyncRequestOutput = void;
    
    class ConstructionToken {
    public:
        explicit ConstructionToken(const ara::com::InstanceIdentifier& instanceId,
            const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
            : processMode(mode),
              ptr(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::ara::sm::TriggerInOut::ServiceIdentifier, instanceId, mode)),
              AcquireFunctionGroupInfoHandle(ptr->GetSkeleton(), methods::TriggerInOutAcquireFunctionGroupInfoId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceId, mode),
              ProcessSyncRequestHandle(ptr->GetSkeleton(), methods::TriggerInOutProcessSyncRequestId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceId, mode),
              ResetSystemHandle(ptr->GetSkeleton(), methods::TriggerInOutResetSystemId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceId, mode),
              ProcessAsyncRequestHandle(ptr->GetSkeleton(), methods::TriggerInOutProcessAsyncRequestId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceId, mode),
              Notifier(ptr->GetSkeleton(), fields::TriggerInOutNotifierId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceId, mode),
              Trigger(ptr->GetSkeleton(), fields::TriggerInOutTriggerId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceId, mode) {
        }

        explicit ConstructionToken(const ara::core::InstanceSpecifier& instanceSpec,
            const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
            : processMode(mode),
              ptr(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::ara::sm::TriggerInOut::ServiceIdentifier, instanceSpec, mode)),
              AcquireFunctionGroupInfoHandle(ptr->GetSkeleton(), methods::TriggerInOutAcquireFunctionGroupInfoId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceSpec, mode),
              ProcessSyncRequestHandle(ptr->GetSkeleton(), methods::TriggerInOutProcessSyncRequestId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceSpec, mode),
              ResetSystemHandle(ptr->GetSkeleton(), methods::TriggerInOutResetSystemId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceSpec, mode),
              ProcessAsyncRequestHandle(ptr->GetSkeleton(), methods::TriggerInOutProcessAsyncRequestId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceSpec, mode),
              Notifier(ptr->GetSkeleton(), fields::TriggerInOutNotifierId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceSpec, mode),
              Trigger(ptr->GetSkeleton(), fields::TriggerInOutTriggerId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceSpec, mode) {
        }

        explicit ConstructionToken(const ara::com::InstanceIdentifierContainer instanceContainer,
            const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
            : processMode(mode),
              ptr(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::ara::sm::TriggerInOut::ServiceIdentifier, instanceContainer, mode)),
              AcquireFunctionGroupInfoHandle(ptr->GetSkeleton(), methods::TriggerInOutAcquireFunctionGroupInfoId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceContainer, mode),
              ProcessSyncRequestHandle(ptr->GetSkeleton(), methods::TriggerInOutProcessSyncRequestId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceContainer, mode),
              ResetSystemHandle(ptr->GetSkeleton(), methods::TriggerInOutResetSystemId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceContainer, mode),
              ProcessAsyncRequestHandle(ptr->GetSkeleton(), methods::TriggerInOutProcessAsyncRequestId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceContainer, mode),
              Notifier(ptr->GetSkeleton(), fields::TriggerInOutNotifierId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceContainer, mode),
              Trigger(ptr->GetSkeleton(), fields::TriggerInOutTriggerId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceContainer, mode) {
        }

        ConstructionToken(ConstructionToken&& other)
            : processMode(std::move(other.processMode)),
              ptr(std::move(other.ptr)),
              AcquireFunctionGroupInfoHandle(std::move(other.AcquireFunctionGroupInfoHandle)),
              ProcessSyncRequestHandle(std::move(other.ProcessSyncRequestHandle)),
              ResetSystemHandle(std::move(other.ResetSystemHandle)),
              ProcessAsyncRequestHandle(std::move(other.ProcessAsyncRequestHandle)),
              Notifier(std::move(other.Notifier)),
              Trigger(std::move(other.Trigger)) {
        }
        ConstructionToken& operator=(ConstructionToken && other)
        {
            if (&other != this) {
                processMode = std::move(other.processMode);
                ptr = std::move(other.ptr);
                AcquireFunctionGroupInfoHandle = std::move(other.AcquireFunctionGroupInfoHandle);
                ProcessSyncRequestHandle = std::move(other.ProcessSyncRequestHandle);
                ResetSystemHandle = std::move(other.ResetSystemHandle);
                ProcessAsyncRequestHandle = std::move(other.ProcessAsyncRequestHandle);
                Notifier = std::move(other.Notifier);
                Trigger = std::move(other.Trigger);
            }
            return *this;
        }
        ConstructionToken(const ConstructionToken&) = delete;
        ConstructionToken& operator = (const ConstructionToken&) = delete;
        std::unique_ptr<ara::com::internal::skeleton::SkeletonAdapter> GetSkeletonAdapter()
        {
            return std::move(ptr);
        }
        methods::AcquireFunctionGroupInfoHandle GetAcquireFunctionGroupInfoHandle()
        {
            return std::move(AcquireFunctionGroupInfoHandle);
        }
        methods::ProcessSyncRequestHandle GetProcessSyncRequestHandle()
        {
            return std::move(ProcessSyncRequestHandle);
        }
        methods::ResetSystemHandle GetResetSystemHandle()
        {
            return std::move(ResetSystemHandle);
        }
        methods::ProcessAsyncRequestHandle GetProcessAsyncRequestHandle()
        {
            return std::move(ProcessAsyncRequestHandle);
        }
        fields::Notifier GetNotifier()
        {
            return std::move(Notifier);
        }
        fields::Trigger GetTrigger()
        {
            return std::move(Trigger);
        }
        ara::core::Result<void> Initialize()
        {
            ara::core::Result<void> initResult;
            do {
                if (processMode == ara::com::MethodCallProcessingMode::kEvent) {
                    if(!ptr->SetMethodThreadNumber(ptr->GetMethodThreadNumber(6U), 1024U)) {
                        ara::core::ErrorCode errorCode(ara::com::ComErrc::kNetworkBindingFailure);
                        initResult.EmplaceError(errorCode);
                        break;
                    }
                }
                initResult = AcquireFunctionGroupInfoHandle.Initialize<ara::core::Future<AcquireFunctionGroupInfoOutput>>();
                if (!initResult.HasValue()) {
                    break;
                }
                initResult = ProcessSyncRequestHandle.Initialize<ara::core::Future<ProcessSyncRequestOutput>>();
                if (!initResult.HasValue()) {
                    break;
                }
                initResult = ResetSystemHandle.Initialize<ara::core::Future<ResetSystemOutput>>();
                if (!initResult.HasValue()) {
                    break;
                }
                initResult = ProcessAsyncRequestHandle.Initialize<ProcessAsyncRequestOutput>();
                if (!initResult.HasValue()) {
                    break;
                }
                Notifier.SetGetterEntityId(fields::TriggerInOutNotifierGetterId);
                initResult = Notifier.Initialize();
                if (!initResult.HasValue()) {
                    break;
                }
                Trigger.SetSetterEntityId(fields::TriggerInOutTriggerSetterId);
                initResult = Trigger.Initialize();
                if (!initResult.HasValue()) {
                    break;
                }
            } while(false);

            return initResult;
        }
    private:
        ara::com::MethodCallProcessingMode processMode;
        std::unique_ptr<ara::com::internal::skeleton::SkeletonAdapter> ptr;
        methods::AcquireFunctionGroupInfoHandle AcquireFunctionGroupInfoHandle;
        methods::ProcessSyncRequestHandle ProcessSyncRequestHandle;
        methods::ResetSystemHandle ResetSystemHandle;
        methods::ProcessAsyncRequestHandle ProcessAsyncRequestHandle;
        fields::Notifier Notifier;
        fields::Trigger Trigger;
    };
    explicit TriggerInOutSkeleton(const ara::com::InstanceIdentifier& instanceId,
                           const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
        : skeletonAdapter(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::ara::sm::TriggerInOut::ServiceIdentifier, instanceId, mode)),
          AcquireFunctionGroupInfoHandle(skeletonAdapter->GetSkeleton(), methods::TriggerInOutAcquireFunctionGroupInfoId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceId, mode),
          ProcessSyncRequestHandle(skeletonAdapter->GetSkeleton(), methods::TriggerInOutProcessSyncRequestId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceId, mode),
          ResetSystemHandle(skeletonAdapter->GetSkeleton(), methods::TriggerInOutResetSystemId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceId, mode),
          ProcessAsyncRequestHandle(skeletonAdapter->GetSkeleton(), methods::TriggerInOutProcessAsyncRequestId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceId, mode),
          Notifier(skeletonAdapter->GetSkeleton(), fields::TriggerInOutNotifierId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceId, mode),
          Trigger(skeletonAdapter->GetSkeleton(), fields::TriggerInOutTriggerId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceId, mode) {
        ConstructSkeleton(mode);
    }

    explicit TriggerInOutSkeleton(const ara::core::InstanceSpecifier& instanceSpec,
                           const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
        :skeletonAdapter(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::ara::sm::TriggerInOut::ServiceIdentifier, instanceSpec, mode)),
          AcquireFunctionGroupInfoHandle(skeletonAdapter->GetSkeleton(), methods::TriggerInOutAcquireFunctionGroupInfoId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceSpec, mode),
          ProcessSyncRequestHandle(skeletonAdapter->GetSkeleton(), methods::TriggerInOutProcessSyncRequestId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceSpec, mode),
          ResetSystemHandle(skeletonAdapter->GetSkeleton(), methods::TriggerInOutResetSystemId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceSpec, mode),
          ProcessAsyncRequestHandle(skeletonAdapter->GetSkeleton(), methods::TriggerInOutProcessAsyncRequestId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceSpec, mode),
          Notifier(skeletonAdapter->GetSkeleton(), fields::TriggerInOutNotifierId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceSpec, mode),
          Trigger(skeletonAdapter->GetSkeleton(), fields::TriggerInOutTriggerId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceSpec, mode) {
        ConstructSkeleton(mode);
    }

    explicit TriggerInOutSkeleton(const ara::com::InstanceIdentifierContainer instanceContainer,
                           const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
        :skeletonAdapter(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::ara::sm::TriggerInOut::ServiceIdentifier, instanceContainer, mode)),
          AcquireFunctionGroupInfoHandle(skeletonAdapter->GetSkeleton(), methods::TriggerInOutAcquireFunctionGroupInfoId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceContainer, mode),
          ProcessSyncRequestHandle(skeletonAdapter->GetSkeleton(), methods::TriggerInOutProcessSyncRequestId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceContainer, mode),
          ResetSystemHandle(skeletonAdapter->GetSkeleton(), methods::TriggerInOutResetSystemId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceContainer, mode),
          ProcessAsyncRequestHandle(skeletonAdapter->GetSkeleton(), methods::TriggerInOutProcessAsyncRequestId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceContainer, mode),
          Notifier(skeletonAdapter->GetSkeleton(), fields::TriggerInOutNotifierId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceContainer, mode),
          Trigger(skeletonAdapter->GetSkeleton(), fields::TriggerInOutTriggerId, ::ara::sm::TriggerInOut::ServiceIdentifier, instanceContainer, mode) {
        ConstructSkeleton(mode);
    }

    TriggerInOutSkeleton(const TriggerInOutSkeleton&) = delete;

    TriggerInOutSkeleton(TriggerInOutSkeleton&&) = default;
    TriggerInOutSkeleton& operator=(TriggerInOutSkeleton&&) = default;
    TriggerInOutSkeleton(ConstructionToken&& token) noexcept
        : skeletonAdapter(token.GetSkeletonAdapter()),
          AcquireFunctionGroupInfoHandle(token.GetAcquireFunctionGroupInfoHandle()),
          ProcessSyncRequestHandle(token.GetProcessSyncRequestHandle()),
          ResetSystemHandle(token.GetResetSystemHandle()),
          ProcessAsyncRequestHandle(token.GetProcessAsyncRequestHandle()),
          Notifier(token.GetNotifier()),
          Trigger(token.GetTrigger()) {
    }

    static ara::core::Result<ConstructionToken> Preconstruct(
        ara::com::InstanceIdentifier instanceId,
        const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
    {
        ConstructionToken token(instanceId, mode);
        const auto initResult = token.Initialize();
        ara::core::Result<ConstructionToken> result(std::move(token));
        if (!initResult.HasValue()) {
            const ara::core::ErrorCode errorCode(initResult.Error());
            result.EmplaceError(errorCode);
        }
        return result;
    }

    static ara::core::Result<ConstructionToken> Preconstruct(
        ara::core::InstanceSpecifier instanceSpec,
        const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
    {
        ConstructionToken token(instanceSpec, mode);
        const auto initResult = token.Initialize();
        ara::core::Result<ConstructionToken> result(std::move(token));
        if (!initResult.HasValue()) {
            const ara::core::ErrorCode errorCode(initResult.Error());
            result.EmplaceError(errorCode);
        }
        return result;
    }

    static ara::core::Result<ConstructionToken> Preconstruct(
        ara::com::InstanceIdentifierContainer instanceIdContainer,
        const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
    {
        ConstructionToken token(instanceIdContainer, mode);
        const auto initResult = token.Initialize();
        ara::core::Result<ConstructionToken> result(std::move(token));
        if (!initResult.HasValue()) {
            const ara::core::ErrorCode errorCode(initResult.Error());
            result.EmplaceError(errorCode);
        }
        return result;
    }

    virtual ~TriggerInOutSkeleton()
    {
        StopOfferService();
    }

    void OfferService()
    {
        skeletonAdapter->RegisterE2EErrorHandler(&TriggerInOutSkeleton::E2EErrorHandler, *this);
        skeletonAdapter->RegisterMethod(&TriggerInOutSkeleton::AcquireFunctionGroupInfo, *this, methods::TriggerInOutAcquireFunctionGroupInfoId);
        skeletonAdapter->RegisterMethod(&TriggerInOutSkeleton::ProcessSyncRequest, *this, methods::TriggerInOutProcessSyncRequestId);
        skeletonAdapter->RegisterMethod(&TriggerInOutSkeleton::ResetSystem, *this, methods::TriggerInOutResetSystemId);
        skeletonAdapter->RegisterMethod(&TriggerInOutSkeleton::ProcessAsyncRequest, *this, methods::TriggerInOutProcessAsyncRequestId);
        Notifier.VerifyValidity();
        Trigger.VerifyValidity();
        skeletonAdapter->OfferService();
    }
    void StopOfferService()
    {
        skeletonAdapter->StopOfferService();
        Notifier.ResetInitState();
        Trigger.ResetInitState();
    }
    ara::core::Future<bool> ProcessNextMethodCall()
    {
        return skeletonAdapter->ProcessNextMethodCall();
    }
    bool SetMethodThreadNumber(const std::uint16_t& number, const std::uint16_t& queueSize)
    {
        return skeletonAdapter->SetMethodThreadNumber(number, queueSize);
    }

    virtual ara::core::Future<AcquireFunctionGroupInfoOutput> AcquireFunctionGroupInfo() = 0;
    virtual ara::core::Future<ProcessSyncRequestOutput> ProcessSyncRequest(const ::ara::sm::StateTransitionVec& stateTrans) = 0;
    virtual ara::core::Future<ResetSystemOutput> ResetSystem(const ::ara::sm::ResetCode& resetParams, const ::String& user, const ::ara::sm::ResetCause& resetReason) = 0;
    virtual ProcessAsyncRequestOutput ProcessAsyncRequest(const ::ara::sm::StateTransitionVec& stateTrans) = 0;

    virtual void E2EErrorHandler(ara::com::e2e::E2EErrorCode, ara::com::e2e::DataID, ara::com::e2e::MessageCounter){}

    methods::AcquireFunctionGroupInfoHandle AcquireFunctionGroupInfoHandle;
    methods::ProcessSyncRequestHandle ProcessSyncRequestHandle;
    methods::ResetSystemHandle ResetSystemHandle;
    methods::ProcessAsyncRequestHandle ProcessAsyncRequestHandle;
    fields::Notifier Notifier;
    fields::Trigger Trigger;
};
} // namespace skeleton
} // namespace sm
} // namespace ara

#endif // ARA_SM_TRIGGERINOUT_SKELETON_H
