/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef IFLYAUTO_STRUCT_CONTAINER_STRUCTCONTAINERINTERFACE_SKELETON_H
#define IFLYAUTO_STRUCT_CONTAINER_STRUCTCONTAINERINTERFACE_SKELETON_H

#include "ara/com/internal/skeleton/skeleton_adapter.h"
#include "ara/com/internal/skeleton/event_adapter.h"
#include "ara/com/internal/skeleton/field_adapter.h"
#include "ara/com/internal/skeleton/method_adapter.h"
#include "ara/com/crc_verification.h"
#include "iflyauto/struct_container/structcontainerinterface_common.h"
#include <cstdint>

namespace iflyauto {
namespace struct_container {
namespace skeleton {
namespace events
{
    using event = ara::com::internal::skeleton::event::EventAdapter<::ara::diag::ByteVector>;
    static constexpr ara::com::internal::EntityId StructContainerInterfaceeventId = 31614U; //event_event_hash
}

namespace methods
{
}

namespace fields
{
}

class StructContainerInterfaceSkeleton {
private:
    std::unique_ptr<ara::com::internal::skeleton::SkeletonAdapter> skeletonAdapter;
    void ConstructSkeleton(const ara::com::MethodCallProcessingMode mode)
    {
        if (mode == ara::com::MethodCallProcessingMode::kEvent) {
            if (!(skeletonAdapter->SetMethodThreadNumber(skeletonAdapter->GetMethodThreadNumber(0U), 1024U))) {
#ifndef NOT_SUPPORT_EXCEPTIONS
                ara::core::ErrorCode errorcode(ara::com::ComErrc::kNetworkBindingFailure);
                throw ara::com::ComException(std::move(errorcode));
#else
                std::cerr << "Error: Not support exception, create skeleton failed!" << std::endl;
#endif
            }
        }
        const ara::core::Result<void> resultevent = event.Initialize();
        ThrowError(resultevent);
    }

    StructContainerInterfaceSkeleton& operator=(const StructContainerInterfaceSkeleton&) = delete;

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
    class ConstructionToken {
    public:
        explicit ConstructionToken(const ara::com::InstanceIdentifier& instanceId,
            const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
            : processMode(mode),
              ptr(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::iflyauto::struct_container::StructContainerInterface::ServiceIdentifier, instanceId, mode)),
              event(ptr->GetSkeleton(), events::StructContainerInterfaceeventId, ::iflyauto::struct_container::StructContainerInterface::ServiceIdentifier, instanceId){
        }

        explicit ConstructionToken(const ara::core::InstanceSpecifier& instanceSpec,
            const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
            : processMode(mode),
              ptr(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::iflyauto::struct_container::StructContainerInterface::ServiceIdentifier, instanceSpec, mode)),
              event(ptr->GetSkeleton(), events::StructContainerInterfaceeventId, ::iflyauto::struct_container::StructContainerInterface::ServiceIdentifier, instanceSpec){
        }

        explicit ConstructionToken(const ara::com::InstanceIdentifierContainer instanceContainer,
            const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
            : processMode(mode),
              ptr(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::iflyauto::struct_container::StructContainerInterface::ServiceIdentifier, instanceContainer, mode)),
              event(ptr->GetSkeleton(), events::StructContainerInterfaceeventId, ::iflyauto::struct_container::StructContainerInterface::ServiceIdentifier, instanceContainer){
        }

        ConstructionToken(ConstructionToken&& other)
            : processMode(std::move(other.processMode)),
              ptr(std::move(other.ptr)),
              event(std::move(other.event)){
        }
        ConstructionToken& operator=(ConstructionToken && other)
        {
            if (&other != this) {
                processMode = std::move(other.processMode);
                ptr = std::move(other.ptr);
                event = std::move(other.event);
            }
            return *this;
        }
        ConstructionToken(const ConstructionToken&) = delete;
        ConstructionToken& operator = (const ConstructionToken&) = delete;
        std::unique_ptr<ara::com::internal::skeleton::SkeletonAdapter> GetSkeletonAdapter()
        {
            return std::move(ptr);
        }
        events::event Getevent()
        {
            return std::move(event);
        }
        ara::core::Result<void> Initialize()
        {
            ara::core::Result<void> initResult;
            do {
                if (processMode == ara::com::MethodCallProcessingMode::kEvent) {
                    if(!ptr->SetMethodThreadNumber(ptr->GetMethodThreadNumber(0U), 1024U)) {
                        ara::core::ErrorCode errorCode(ara::com::ComErrc::kNetworkBindingFailure);
                        initResult.EmplaceError(errorCode);
                        break;
                    }
                }
                initResult = event.Initialize();
                if (!initResult.HasValue()) {
                    break;
                }
            } while(false);

            return initResult;
        }
    private:
        ara::com::MethodCallProcessingMode processMode;
        std::unique_ptr<ara::com::internal::skeleton::SkeletonAdapter> ptr;
        events::event event;
    };
    explicit StructContainerInterfaceSkeleton(const ara::com::InstanceIdentifier& instanceId,
                           const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
        : skeletonAdapter(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::iflyauto::struct_container::StructContainerInterface::ServiceIdentifier, instanceId, mode)),
          event(skeletonAdapter->GetSkeleton(), events::StructContainerInterfaceeventId, ::iflyauto::struct_container::StructContainerInterface::ServiceIdentifier, instanceId){
        ConstructSkeleton(mode);
    }

    explicit StructContainerInterfaceSkeleton(const ara::core::InstanceSpecifier& instanceSpec,
                           const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
        :skeletonAdapter(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::iflyauto::struct_container::StructContainerInterface::ServiceIdentifier, instanceSpec, mode)),
          event(skeletonAdapter->GetSkeleton(), events::StructContainerInterfaceeventId, ::iflyauto::struct_container::StructContainerInterface::ServiceIdentifier, instanceSpec){
        ConstructSkeleton(mode);
    }

    explicit StructContainerInterfaceSkeleton(const ara::com::InstanceIdentifierContainer instanceContainer,
                           const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
        :skeletonAdapter(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::iflyauto::struct_container::StructContainerInterface::ServiceIdentifier, instanceContainer, mode)),
          event(skeletonAdapter->GetSkeleton(), events::StructContainerInterfaceeventId, ::iflyauto::struct_container::StructContainerInterface::ServiceIdentifier, instanceContainer){
        ConstructSkeleton(mode);
    }

    StructContainerInterfaceSkeleton(const StructContainerInterfaceSkeleton&) = delete;

    StructContainerInterfaceSkeleton(StructContainerInterfaceSkeleton&&) = default;
    StructContainerInterfaceSkeleton& operator=(StructContainerInterfaceSkeleton&&) = default;
    StructContainerInterfaceSkeleton(ConstructionToken&& token) noexcept
        : skeletonAdapter(token.GetSkeletonAdapter()),
          event(token.Getevent()){
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

    virtual ~StructContainerInterfaceSkeleton()
    {
        StopOfferService();
    }

    void OfferService()
    {
        skeletonAdapter->RegisterE2EErrorHandler(&StructContainerInterfaceSkeleton::E2EErrorHandler, *this);
        skeletonAdapter->OfferService();
    }
    void StopOfferService()
    {
        skeletonAdapter->StopOfferService();
    }
    ara::core::Future<bool> ProcessNextMethodCall()
    {
        return skeletonAdapter->ProcessNextMethodCall();
    }
    bool SetMethodThreadNumber(const std::uint16_t& number, const std::uint16_t& queueSize)
    {
        return skeletonAdapter->SetMethodThreadNumber(number, queueSize);
    }

    virtual void E2EErrorHandler(ara::com::e2e::E2EErrorCode, ara::com::e2e::DataID, ara::com::e2e::MessageCounter){}

    events::event event;
};
} // namespace skeleton
} // namespace struct_container
} // namespace iflyauto

#endif // IFLYAUTO_STRUCT_CONTAINER_STRUCTCONTAINERINTERFACE_SKELETON_H
