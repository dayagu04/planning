/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef IFLYAUTO_STRUCT_CONTAINER_CLIENTSERVICECONTAINERINTERFACE_SKELETON_H
#define IFLYAUTO_STRUCT_CONTAINER_CLIENTSERVICECONTAINERINTERFACE_SKELETON_H

#include "ara/com/internal/skeleton/skeleton_adapter.h"
#include "ara/com/internal/skeleton/event_adapter.h"
#include "ara/com/internal/skeleton/field_adapter.h"
#include "ara/com/internal/skeleton/method_adapter.h"
#include "ara/com/crc_verification.h"
#include "iflyauto/struct_container/clientservicecontainerinterface_common.h"
#include <cstdint>

namespace iflyauto {
namespace struct_container {
namespace skeleton {
namespace events
{
}

namespace methods
{
    using methodHandle = ara::com::internal::skeleton::method::MethodAdapter;
    static constexpr ara::com::internal::EntityId ClientServiceContainerInterfacemethodId = 17484U; //method_method_hash
}

namespace fields
{
}

class ClientServiceContainerInterfaceSkeleton {
private:
    std::unique_ptr<ara::com::internal::skeleton::SkeletonAdapter> skeletonAdapter;
    void ConstructSkeleton(const ara::com::MethodCallProcessingMode mode)
    {
        if (mode == ara::com::MethodCallProcessingMode::kEvent) {
            if (!(skeletonAdapter->SetMethodThreadNumber(skeletonAdapter->GetMethodThreadNumber(1U), 1024U))) {
#ifndef NOT_SUPPORT_EXCEPTIONS
                ara::core::ErrorCode errorcode(ara::com::ComErrc::kNetworkBindingFailure);
                throw ara::com::ComException(std::move(errorcode));
#else
                std::cerr << "Error: Not support exception, create skeleton failed!" << std::endl;
#endif
            }
        }
        const ara::core::Result<void> resultmethod = methodHandle.Initialize<ara::core::Future<methodOutput>>();
        ThrowError(resultmethod);
    }

    ClientServiceContainerInterfaceSkeleton& operator=(const ClientServiceContainerInterfaceSkeleton&) = delete;

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
    using methodOutput = iflyauto::struct_container::methods::method::Output;
    
    class ConstructionToken {
    public:
        explicit ConstructionToken(const ara::com::InstanceIdentifier& instanceId,
            const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
            : processMode(mode),
              ptr(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::iflyauto::struct_container::ClientServiceContainerInterface::ServiceIdentifier, instanceId, mode)),
              methodHandle(ptr->GetSkeleton(), methods::ClientServiceContainerInterfacemethodId, ::iflyauto::struct_container::ClientServiceContainerInterface::ServiceIdentifier, instanceId, mode){
        }

        explicit ConstructionToken(const ara::core::InstanceSpecifier& instanceSpec,
            const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
            : processMode(mode),
              ptr(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::iflyauto::struct_container::ClientServiceContainerInterface::ServiceIdentifier, instanceSpec, mode)),
              methodHandle(ptr->GetSkeleton(), methods::ClientServiceContainerInterfacemethodId, ::iflyauto::struct_container::ClientServiceContainerInterface::ServiceIdentifier, instanceSpec, mode){
        }

        explicit ConstructionToken(const ara::com::InstanceIdentifierContainer instanceContainer,
            const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
            : processMode(mode),
              ptr(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::iflyauto::struct_container::ClientServiceContainerInterface::ServiceIdentifier, instanceContainer, mode)),
              methodHandle(ptr->GetSkeleton(), methods::ClientServiceContainerInterfacemethodId, ::iflyauto::struct_container::ClientServiceContainerInterface::ServiceIdentifier, instanceContainer, mode){
        }

        ConstructionToken(ConstructionToken&& other)
            : processMode(std::move(other.processMode)),
              ptr(std::move(other.ptr)),
              methodHandle(std::move(other.methodHandle)){
        }
        ConstructionToken& operator=(ConstructionToken && other)
        {
            if (&other != this) {
                processMode = std::move(other.processMode);
                ptr = std::move(other.ptr);
                methodHandle = std::move(other.methodHandle);
            }
            return *this;
        }
        ConstructionToken(const ConstructionToken&) = delete;
        ConstructionToken& operator = (const ConstructionToken&) = delete;
        std::unique_ptr<ara::com::internal::skeleton::SkeletonAdapter> GetSkeletonAdapter()
        {
            return std::move(ptr);
        }
        methods::methodHandle GetmethodHandle()
        {
            return std::move(methodHandle);
        }
        ara::core::Result<void> Initialize()
        {
            ara::core::Result<void> initResult;
            do {
                if (processMode == ara::com::MethodCallProcessingMode::kEvent) {
                    if(!ptr->SetMethodThreadNumber(ptr->GetMethodThreadNumber(1U), 1024U)) {
                        ara::core::ErrorCode errorCode(ara::com::ComErrc::kNetworkBindingFailure);
                        initResult.EmplaceError(errorCode);
                        break;
                    }
                }
                initResult = methodHandle.Initialize<ara::core::Future<methodOutput>>();
                if (!initResult.HasValue()) {
                    break;
                }
            } while(false);

            return initResult;
        }
    private:
        ara::com::MethodCallProcessingMode processMode;
        std::unique_ptr<ara::com::internal::skeleton::SkeletonAdapter> ptr;
        methods::methodHandle methodHandle;
    };
    explicit ClientServiceContainerInterfaceSkeleton(const ara::com::InstanceIdentifier& instanceId,
                           const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
        : skeletonAdapter(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::iflyauto::struct_container::ClientServiceContainerInterface::ServiceIdentifier, instanceId, mode)),
          methodHandle(skeletonAdapter->GetSkeleton(), methods::ClientServiceContainerInterfacemethodId, ::iflyauto::struct_container::ClientServiceContainerInterface::ServiceIdentifier, instanceId, mode){
        ConstructSkeleton(mode);
    }

    explicit ClientServiceContainerInterfaceSkeleton(const ara::core::InstanceSpecifier& instanceSpec,
                           const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
        :skeletonAdapter(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::iflyauto::struct_container::ClientServiceContainerInterface::ServiceIdentifier, instanceSpec, mode)),
          methodHandle(skeletonAdapter->GetSkeleton(), methods::ClientServiceContainerInterfacemethodId, ::iflyauto::struct_container::ClientServiceContainerInterface::ServiceIdentifier, instanceSpec, mode){
        ConstructSkeleton(mode);
    }

    explicit ClientServiceContainerInterfaceSkeleton(const ara::com::InstanceIdentifierContainer instanceContainer,
                           const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
        :skeletonAdapter(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::iflyauto::struct_container::ClientServiceContainerInterface::ServiceIdentifier, instanceContainer, mode)),
          methodHandle(skeletonAdapter->GetSkeleton(), methods::ClientServiceContainerInterfacemethodId, ::iflyauto::struct_container::ClientServiceContainerInterface::ServiceIdentifier, instanceContainer, mode){
        ConstructSkeleton(mode);
    }

    ClientServiceContainerInterfaceSkeleton(const ClientServiceContainerInterfaceSkeleton&) = delete;

    ClientServiceContainerInterfaceSkeleton(ClientServiceContainerInterfaceSkeleton&&) = default;
    ClientServiceContainerInterfaceSkeleton& operator=(ClientServiceContainerInterfaceSkeleton&&) = default;
    ClientServiceContainerInterfaceSkeleton(ConstructionToken&& token) noexcept
        : skeletonAdapter(token.GetSkeletonAdapter()),
          methodHandle(token.GetmethodHandle()){
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

    virtual ~ClientServiceContainerInterfaceSkeleton()
    {
        StopOfferService();
    }

    void OfferService()
    {
        skeletonAdapter->RegisterE2EErrorHandler(&ClientServiceContainerInterfaceSkeleton::E2EErrorHandler, *this);
        skeletonAdapter->RegisterMethod(&ClientServiceContainerInterfaceSkeleton::method, *this, methods::ClientServiceContainerInterfacemethodId);
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

    virtual ara::core::Future<methodOutput> method(const ::iflyauto::struct_container::StructContainer& msg) = 0;
    virtual void E2EErrorHandler(ara::com::e2e::E2EErrorCode, ara::com::e2e::DataID, ara::com::e2e::MessageCounter){}

    methods::methodHandle methodHandle;
};
} // namespace skeleton
} // namespace struct_container
} // namespace iflyauto

#endif // IFLYAUTO_STRUCT_CONTAINER_CLIENTSERVICECONTAINERINTERFACE_SKELETON_H
