/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_FM_FMALARMRECEIVESERVICE_SKELETON_H
#define MDC_FM_FMALARMRECEIVESERVICE_SKELETON_H

#include "ara/com/internal/skeleton/skeleton_adapter.h"
#include "ara/com/internal/skeleton/event_adapter.h"
#include "ara/com/internal/skeleton/field_adapter.h"
#include "ara/com/internal/skeleton/method_adapter.h"
#include "ara/com/crc_verification.h"
#include "mdc/fm/fmalarmreceiveservice_common.h"
#include <cstdint>

namespace mdc {
namespace fm {
namespace skeleton {
namespace events
{
}

namespace methods
{
    using ReportAlarmHandle = ara::com::internal::skeleton::method::MethodAdapter;
    static constexpr ara::com::internal::EntityId FmAlarmReceiveServiceReportAlarmId = 30845U; //ReportAlarm_method_hash
}

namespace fields
{
}

class FmAlarmReceiveServiceSkeleton {
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
        const ara::core::Result<void> resultReportAlarm = ReportAlarmHandle.Initialize<ara::core::Future<ReportAlarmOutput>>();
        ThrowError(resultReportAlarm);
    }

    FmAlarmReceiveServiceSkeleton& operator=(const FmAlarmReceiveServiceSkeleton&) = delete;

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
    using ReportAlarmOutput = mdc::fm::methods::ReportAlarm::Output;
    
    class ConstructionToken {
    public:
        explicit ConstructionToken(const ara::com::InstanceIdentifier& instanceId,
            const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
            : processMode(mode),
              ptr(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::mdc::fm::FmAlarmReceiveService::ServiceIdentifier, instanceId, mode)),
              ReportAlarmHandle(ptr->GetSkeleton(), methods::FmAlarmReceiveServiceReportAlarmId, ::mdc::fm::FmAlarmReceiveService::ServiceIdentifier, instanceId, mode){
        }

        explicit ConstructionToken(const ara::core::InstanceSpecifier& instanceSpec,
            const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
            : processMode(mode),
              ptr(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::mdc::fm::FmAlarmReceiveService::ServiceIdentifier, instanceSpec, mode)),
              ReportAlarmHandle(ptr->GetSkeleton(), methods::FmAlarmReceiveServiceReportAlarmId, ::mdc::fm::FmAlarmReceiveService::ServiceIdentifier, instanceSpec, mode){
        }

        explicit ConstructionToken(const ara::com::InstanceIdentifierContainer instanceContainer,
            const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
            : processMode(mode),
              ptr(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::mdc::fm::FmAlarmReceiveService::ServiceIdentifier, instanceContainer, mode)),
              ReportAlarmHandle(ptr->GetSkeleton(), methods::FmAlarmReceiveServiceReportAlarmId, ::mdc::fm::FmAlarmReceiveService::ServiceIdentifier, instanceContainer, mode){
        }

        ConstructionToken(ConstructionToken&& other)
            : processMode(std::move(other.processMode)),
              ptr(std::move(other.ptr)),
              ReportAlarmHandle(std::move(other.ReportAlarmHandle)){
        }
        ConstructionToken& operator=(ConstructionToken && other)
        {
            if (&other != this) {
                processMode = std::move(other.processMode);
                ptr = std::move(other.ptr);
                ReportAlarmHandle = std::move(other.ReportAlarmHandle);
            }
            return *this;
        }
        ConstructionToken(const ConstructionToken&) = delete;
        ConstructionToken& operator = (const ConstructionToken&) = delete;
        std::unique_ptr<ara::com::internal::skeleton::SkeletonAdapter> GetSkeletonAdapter()
        {
            return std::move(ptr);
        }
        methods::ReportAlarmHandle GetReportAlarmHandle()
        {
            return std::move(ReportAlarmHandle);
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
                initResult = ReportAlarmHandle.Initialize<ara::core::Future<ReportAlarmOutput>>();
                if (!initResult.HasValue()) {
                    break;
                }
            } while(false);

            return initResult;
        }
    private:
        ara::com::MethodCallProcessingMode processMode;
        std::unique_ptr<ara::com::internal::skeleton::SkeletonAdapter> ptr;
        methods::ReportAlarmHandle ReportAlarmHandle;
    };
    explicit FmAlarmReceiveServiceSkeleton(const ara::com::InstanceIdentifier& instanceId,
                           const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
        : skeletonAdapter(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::mdc::fm::FmAlarmReceiveService::ServiceIdentifier, instanceId, mode)),
          ReportAlarmHandle(skeletonAdapter->GetSkeleton(), methods::FmAlarmReceiveServiceReportAlarmId, ::mdc::fm::FmAlarmReceiveService::ServiceIdentifier, instanceId, mode){
        ConstructSkeleton(mode);
    }

    explicit FmAlarmReceiveServiceSkeleton(const ara::core::InstanceSpecifier& instanceSpec,
                           const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
        :skeletonAdapter(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::mdc::fm::FmAlarmReceiveService::ServiceIdentifier, instanceSpec, mode)),
          ReportAlarmHandle(skeletonAdapter->GetSkeleton(), methods::FmAlarmReceiveServiceReportAlarmId, ::mdc::fm::FmAlarmReceiveService::ServiceIdentifier, instanceSpec, mode){
        ConstructSkeleton(mode);
    }

    explicit FmAlarmReceiveServiceSkeleton(const ara::com::InstanceIdentifierContainer instanceContainer,
                           const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
        :skeletonAdapter(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::mdc::fm::FmAlarmReceiveService::ServiceIdentifier, instanceContainer, mode)),
          ReportAlarmHandle(skeletonAdapter->GetSkeleton(), methods::FmAlarmReceiveServiceReportAlarmId, ::mdc::fm::FmAlarmReceiveService::ServiceIdentifier, instanceContainer, mode){
        ConstructSkeleton(mode);
    }

    FmAlarmReceiveServiceSkeleton(const FmAlarmReceiveServiceSkeleton&) = delete;

    FmAlarmReceiveServiceSkeleton(FmAlarmReceiveServiceSkeleton&&) = default;
    FmAlarmReceiveServiceSkeleton& operator=(FmAlarmReceiveServiceSkeleton&&) = default;
    FmAlarmReceiveServiceSkeleton(ConstructionToken&& token) noexcept
        : skeletonAdapter(token.GetSkeletonAdapter()),
          ReportAlarmHandle(token.GetReportAlarmHandle()){
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

    virtual ~FmAlarmReceiveServiceSkeleton()
    {
        StopOfferService();
    }

    void OfferService()
    {
        skeletonAdapter->RegisterE2EErrorHandler(&FmAlarmReceiveServiceSkeleton::E2EErrorHandler, *this);
        skeletonAdapter->RegisterMethod(&FmAlarmReceiveServiceSkeleton::ReportAlarm, *this, methods::FmAlarmReceiveServiceReportAlarmId);
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

    virtual ara::core::Future<ReportAlarmOutput> ReportAlarm(const ::mdc::fm::AlarmInfo& alarmMsg) = 0;
    virtual void E2EErrorHandler(ara::com::e2e::E2EErrorCode, ara::com::e2e::DataID, ara::com::e2e::MessageCounter){}

    methods::ReportAlarmHandle ReportAlarmHandle;
};
} // namespace skeleton
} // namespace fm
} // namespace mdc

#endif // MDC_FM_FMALARMRECEIVESERVICE_SKELETON_H
