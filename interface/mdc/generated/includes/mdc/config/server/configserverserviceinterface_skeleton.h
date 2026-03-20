/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_CONFIG_SERVER_CONFIGSERVERSERVICEINTERFACE_SKELETON_H
#define MDC_CONFIG_SERVER_CONFIGSERVERSERVICEINTERFACE_SKELETON_H

#include "ara/com/internal/skeleton/skeleton_adapter.h"
#include "ara/com/internal/skeleton/event_adapter.h"
#include "ara/com/internal/skeleton/field_adapter.h"
#include "ara/com/internal/skeleton/method_adapter.h"
#include "ara/com/crc_verification.h"
#include "mdc/config/server/configserverserviceinterface_common.h"
#include <cstdint>

namespace mdc {
namespace config {
namespace server {
namespace skeleton {
namespace events
{
    using ParamUpdateEvent = ara::com::internal::skeleton::event::EventAdapter<::mdc::config::server::ParamUpdateData>;
    using ServerNotifyEvent = ara::com::internal::skeleton::event::EventAdapter<::mdc::config::server::ServerNotifyData>;
    static constexpr ara::com::internal::EntityId ConfigServerServiceInterfaceParamUpdateEventId = 35207U; //ParamUpdateEvent_event_hash
    static constexpr ara::com::internal::EntityId ConfigServerServiceInterfaceServerNotifyEventId = 27155U; //ServerNotifyEvent_event_hash
}

namespace methods
{
    using AnswerAliveHandle = ara::com::internal::skeleton::method::MethodAdapter;
    using DelParamHandle = ara::com::internal::skeleton::method::MethodAdapter;
    using GetMonitorClientsHandle = ara::com::internal::skeleton::method::MethodAdapter;
    using GetParamHandle = ara::com::internal::skeleton::method::MethodAdapter;
    using MonitorParamHandle = ara::com::internal::skeleton::method::MethodAdapter;
    using SetParamHandle = ara::com::internal::skeleton::method::MethodAdapter;
    using UnMonitorParamHandle = ara::com::internal::skeleton::method::MethodAdapter;
    using InitClientHandle = ara::com::internal::skeleton::method::MethodAdapter;
    static constexpr ara::com::internal::EntityId ConfigServerServiceInterfaceAnswerAliveId = 30329U; //AnswerAlive_method_hash
    static constexpr ara::com::internal::EntityId ConfigServerServiceInterfaceDelParamId = 56227U; //DelParam_method_hash
    static constexpr ara::com::internal::EntityId ConfigServerServiceInterfaceGetMonitorClientsId = 42802U; //GetMonitorClients_method_hash
    static constexpr ara::com::internal::EntityId ConfigServerServiceInterfaceGetParamId = 5505U; //GetParam_method_hash
    static constexpr ara::com::internal::EntityId ConfigServerServiceInterfaceMonitorParamId = 3875U; //MonitorParam_method_hash
    static constexpr ara::com::internal::EntityId ConfigServerServiceInterfaceSetParamId = 31959U; //SetParam_method_hash
    static constexpr ara::com::internal::EntityId ConfigServerServiceInterfaceUnMonitorParamId = 36659U; //UnMonitorParam_method_hash
    static constexpr ara::com::internal::EntityId ConfigServerServiceInterfaceInitClientId = 55067U; //InitClient_method_hash
}

namespace fields
{
}

class ConfigServerServiceInterfaceSkeleton {
private:
    std::unique_ptr<ara::com::internal::skeleton::SkeletonAdapter> skeletonAdapter;
    void ConstructSkeleton(const ara::com::MethodCallProcessingMode mode)
    {
        if (mode == ara::com::MethodCallProcessingMode::kEvent) {
            if (!(skeletonAdapter->SetMethodThreadNumber(skeletonAdapter->GetMethodThreadNumber(8U), 1024U))) {
#ifndef NOT_SUPPORT_EXCEPTIONS
                ara::core::ErrorCode errorcode(ara::com::ComErrc::kNetworkBindingFailure);
                throw ara::com::ComException(std::move(errorcode));
#else
                std::cerr << "Error: Not support exception, create skeleton failed!" << std::endl;
#endif
            }
        }
        const ara::core::Result<void> resultParamUpdateEvent = ParamUpdateEvent.Initialize();
        ThrowError(resultParamUpdateEvent);
        const ara::core::Result<void> resultServerNotifyEvent = ServerNotifyEvent.Initialize();
        ThrowError(resultServerNotifyEvent);
        const ara::core::Result<void> resultAnswerAlive = AnswerAliveHandle.Initialize<ara::core::Future<AnswerAliveOutput>>();
        ThrowError(resultAnswerAlive);
        const ara::core::Result<void> resultDelParam = DelParamHandle.Initialize<ara::core::Future<DelParamOutput>>();
        ThrowError(resultDelParam);
        const ara::core::Result<void> resultGetMonitorClients = GetMonitorClientsHandle.Initialize<ara::core::Future<GetMonitorClientsOutput>>();
        ThrowError(resultGetMonitorClients);
        const ara::core::Result<void> resultGetParam = GetParamHandle.Initialize<ara::core::Future<GetParamOutput>>();
        ThrowError(resultGetParam);
        const ara::core::Result<void> resultMonitorParam = MonitorParamHandle.Initialize<ara::core::Future<MonitorParamOutput>>();
        ThrowError(resultMonitorParam);
        const ara::core::Result<void> resultSetParam = SetParamHandle.Initialize<ara::core::Future<SetParamOutput>>();
        ThrowError(resultSetParam);
        const ara::core::Result<void> resultUnMonitorParam = UnMonitorParamHandle.Initialize<ara::core::Future<UnMonitorParamOutput>>();
        ThrowError(resultUnMonitorParam);
        const ara::core::Result<void> resultInitClient = InitClientHandle.Initialize<ara::core::Future<InitClientOutput>>();
        ThrowError(resultInitClient);
    }

    ConfigServerServiceInterfaceSkeleton& operator=(const ConfigServerServiceInterfaceSkeleton&) = delete;

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
    using AnswerAliveOutput = void;
    
    using DelParamOutput = mdc::config::server::methods::DelParam::Output;
    
    using GetMonitorClientsOutput = mdc::config::server::methods::GetMonitorClients::Output;
    
    using GetParamOutput = mdc::config::server::methods::GetParam::Output;
    
    using MonitorParamOutput = mdc::config::server::methods::MonitorParam::Output;
    
    using SetParamOutput = mdc::config::server::methods::SetParam::Output;
    
    using UnMonitorParamOutput = mdc::config::server::methods::UnMonitorParam::Output;
    
    using InitClientOutput = mdc::config::server::methods::InitClient::Output;
    
    class ConstructionToken {
    public:
        explicit ConstructionToken(const ara::com::InstanceIdentifier& instanceId,
            const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
            : processMode(mode),
              ptr(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceId, mode)),
              ParamUpdateEvent(ptr->GetSkeleton(), events::ConfigServerServiceInterfaceParamUpdateEventId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceId),
              ServerNotifyEvent(ptr->GetSkeleton(), events::ConfigServerServiceInterfaceServerNotifyEventId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceId),
              AnswerAliveHandle(ptr->GetSkeleton(), methods::ConfigServerServiceInterfaceAnswerAliveId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceId, mode),
              DelParamHandle(ptr->GetSkeleton(), methods::ConfigServerServiceInterfaceDelParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceId, mode),
              GetMonitorClientsHandle(ptr->GetSkeleton(), methods::ConfigServerServiceInterfaceGetMonitorClientsId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceId, mode),
              GetParamHandle(ptr->GetSkeleton(), methods::ConfigServerServiceInterfaceGetParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceId, mode),
              MonitorParamHandle(ptr->GetSkeleton(), methods::ConfigServerServiceInterfaceMonitorParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceId, mode),
              SetParamHandle(ptr->GetSkeleton(), methods::ConfigServerServiceInterfaceSetParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceId, mode),
              UnMonitorParamHandle(ptr->GetSkeleton(), methods::ConfigServerServiceInterfaceUnMonitorParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceId, mode),
              InitClientHandle(ptr->GetSkeleton(), methods::ConfigServerServiceInterfaceInitClientId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceId, mode){
        }

        explicit ConstructionToken(const ara::core::InstanceSpecifier& instanceSpec,
            const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
            : processMode(mode),
              ptr(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceSpec, mode)),
              ParamUpdateEvent(ptr->GetSkeleton(), events::ConfigServerServiceInterfaceParamUpdateEventId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceSpec),
              ServerNotifyEvent(ptr->GetSkeleton(), events::ConfigServerServiceInterfaceServerNotifyEventId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceSpec),
              AnswerAliveHandle(ptr->GetSkeleton(), methods::ConfigServerServiceInterfaceAnswerAliveId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceSpec, mode),
              DelParamHandle(ptr->GetSkeleton(), methods::ConfigServerServiceInterfaceDelParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceSpec, mode),
              GetMonitorClientsHandle(ptr->GetSkeleton(), methods::ConfigServerServiceInterfaceGetMonitorClientsId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceSpec, mode),
              GetParamHandle(ptr->GetSkeleton(), methods::ConfigServerServiceInterfaceGetParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceSpec, mode),
              MonitorParamHandle(ptr->GetSkeleton(), methods::ConfigServerServiceInterfaceMonitorParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceSpec, mode),
              SetParamHandle(ptr->GetSkeleton(), methods::ConfigServerServiceInterfaceSetParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceSpec, mode),
              UnMonitorParamHandle(ptr->GetSkeleton(), methods::ConfigServerServiceInterfaceUnMonitorParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceSpec, mode),
              InitClientHandle(ptr->GetSkeleton(), methods::ConfigServerServiceInterfaceInitClientId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceSpec, mode){
        }

        explicit ConstructionToken(const ara::com::InstanceIdentifierContainer instanceContainer,
            const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
            : processMode(mode),
              ptr(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceContainer, mode)),
              ParamUpdateEvent(ptr->GetSkeleton(), events::ConfigServerServiceInterfaceParamUpdateEventId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceContainer),
              ServerNotifyEvent(ptr->GetSkeleton(), events::ConfigServerServiceInterfaceServerNotifyEventId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceContainer),
              AnswerAliveHandle(ptr->GetSkeleton(), methods::ConfigServerServiceInterfaceAnswerAliveId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceContainer, mode),
              DelParamHandle(ptr->GetSkeleton(), methods::ConfigServerServiceInterfaceDelParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceContainer, mode),
              GetMonitorClientsHandle(ptr->GetSkeleton(), methods::ConfigServerServiceInterfaceGetMonitorClientsId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceContainer, mode),
              GetParamHandle(ptr->GetSkeleton(), methods::ConfigServerServiceInterfaceGetParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceContainer, mode),
              MonitorParamHandle(ptr->GetSkeleton(), methods::ConfigServerServiceInterfaceMonitorParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceContainer, mode),
              SetParamHandle(ptr->GetSkeleton(), methods::ConfigServerServiceInterfaceSetParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceContainer, mode),
              UnMonitorParamHandle(ptr->GetSkeleton(), methods::ConfigServerServiceInterfaceUnMonitorParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceContainer, mode),
              InitClientHandle(ptr->GetSkeleton(), methods::ConfigServerServiceInterfaceInitClientId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceContainer, mode){
        }

        ConstructionToken(ConstructionToken&& other)
            : processMode(std::move(other.processMode)),
              ptr(std::move(other.ptr)),
              ParamUpdateEvent(std::move(other.ParamUpdateEvent)),
              ServerNotifyEvent(std::move(other.ServerNotifyEvent)),
              AnswerAliveHandle(std::move(other.AnswerAliveHandle)),
              DelParamHandle(std::move(other.DelParamHandle)),
              GetMonitorClientsHandle(std::move(other.GetMonitorClientsHandle)),
              GetParamHandle(std::move(other.GetParamHandle)),
              MonitorParamHandle(std::move(other.MonitorParamHandle)),
              SetParamHandle(std::move(other.SetParamHandle)),
              UnMonitorParamHandle(std::move(other.UnMonitorParamHandle)),
              InitClientHandle(std::move(other.InitClientHandle)){
        }
        ConstructionToken& operator=(ConstructionToken && other)
        {
            if (&other != this) {
                processMode = std::move(other.processMode);
                ptr = std::move(other.ptr);
                ParamUpdateEvent = std::move(other.ParamUpdateEvent);
                ServerNotifyEvent = std::move(other.ServerNotifyEvent);
                AnswerAliveHandle = std::move(other.AnswerAliveHandle);
                DelParamHandle = std::move(other.DelParamHandle);
                GetMonitorClientsHandle = std::move(other.GetMonitorClientsHandle);
                GetParamHandle = std::move(other.GetParamHandle);
                MonitorParamHandle = std::move(other.MonitorParamHandle);
                SetParamHandle = std::move(other.SetParamHandle);
                UnMonitorParamHandle = std::move(other.UnMonitorParamHandle);
                InitClientHandle = std::move(other.InitClientHandle);
            }
            return *this;
        }
        ConstructionToken(const ConstructionToken&) = delete;
        ConstructionToken& operator = (const ConstructionToken&) = delete;
        std::unique_ptr<ara::com::internal::skeleton::SkeletonAdapter> GetSkeletonAdapter()
        {
            return std::move(ptr);
        }
        events::ParamUpdateEvent GetParamUpdateEvent()
        {
            return std::move(ParamUpdateEvent);
        }
        events::ServerNotifyEvent GetServerNotifyEvent()
        {
            return std::move(ServerNotifyEvent);
        }
        methods::AnswerAliveHandle GetAnswerAliveHandle()
        {
            return std::move(AnswerAliveHandle);
        }
        methods::DelParamHandle GetDelParamHandle()
        {
            return std::move(DelParamHandle);
        }
        methods::GetMonitorClientsHandle GetGetMonitorClientsHandle()
        {
            return std::move(GetMonitorClientsHandle);
        }
        methods::GetParamHandle GetGetParamHandle()
        {
            return std::move(GetParamHandle);
        }
        methods::MonitorParamHandle GetMonitorParamHandle()
        {
            return std::move(MonitorParamHandle);
        }
        methods::SetParamHandle GetSetParamHandle()
        {
            return std::move(SetParamHandle);
        }
        methods::UnMonitorParamHandle GetUnMonitorParamHandle()
        {
            return std::move(UnMonitorParamHandle);
        }
        methods::InitClientHandle GetInitClientHandle()
        {
            return std::move(InitClientHandle);
        }
        ara::core::Result<void> Initialize()
        {
            ara::core::Result<void> initResult;
            do {
                if (processMode == ara::com::MethodCallProcessingMode::kEvent) {
                    if(!ptr->SetMethodThreadNumber(ptr->GetMethodThreadNumber(8U), 1024U)) {
                        ara::core::ErrorCode errorCode(ara::com::ComErrc::kNetworkBindingFailure);
                        initResult.EmplaceError(errorCode);
                        break;
                    }
                }
                initResult = ParamUpdateEvent.Initialize();
                if (!initResult.HasValue()) {
                    break;
                }
                initResult = ServerNotifyEvent.Initialize();
                if (!initResult.HasValue()) {
                    break;
                }
                initResult = AnswerAliveHandle.Initialize<ara::core::Future<AnswerAliveOutput>>();
                if (!initResult.HasValue()) {
                    break;
                }
                initResult = DelParamHandle.Initialize<ara::core::Future<DelParamOutput>>();
                if (!initResult.HasValue()) {
                    break;
                }
                initResult = GetMonitorClientsHandle.Initialize<ara::core::Future<GetMonitorClientsOutput>>();
                if (!initResult.HasValue()) {
                    break;
                }
                initResult = GetParamHandle.Initialize<ara::core::Future<GetParamOutput>>();
                if (!initResult.HasValue()) {
                    break;
                }
                initResult = MonitorParamHandle.Initialize<ara::core::Future<MonitorParamOutput>>();
                if (!initResult.HasValue()) {
                    break;
                }
                initResult = SetParamHandle.Initialize<ara::core::Future<SetParamOutput>>();
                if (!initResult.HasValue()) {
                    break;
                }
                initResult = UnMonitorParamHandle.Initialize<ara::core::Future<UnMonitorParamOutput>>();
                if (!initResult.HasValue()) {
                    break;
                }
                initResult = InitClientHandle.Initialize<ara::core::Future<InitClientOutput>>();
                if (!initResult.HasValue()) {
                    break;
                }
            } while(false);

            return initResult;
        }
    private:
        ara::com::MethodCallProcessingMode processMode;
        std::unique_ptr<ara::com::internal::skeleton::SkeletonAdapter> ptr;
        events::ParamUpdateEvent ParamUpdateEvent;
        events::ServerNotifyEvent ServerNotifyEvent;
        methods::AnswerAliveHandle AnswerAliveHandle;
        methods::DelParamHandle DelParamHandle;
        methods::GetMonitorClientsHandle GetMonitorClientsHandle;
        methods::GetParamHandle GetParamHandle;
        methods::MonitorParamHandle MonitorParamHandle;
        methods::SetParamHandle SetParamHandle;
        methods::UnMonitorParamHandle UnMonitorParamHandle;
        methods::InitClientHandle InitClientHandle;
    };
    explicit ConfigServerServiceInterfaceSkeleton(const ara::com::InstanceIdentifier& instanceId,
                           const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
        : skeletonAdapter(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceId, mode)),
          ParamUpdateEvent(skeletonAdapter->GetSkeleton(), events::ConfigServerServiceInterfaceParamUpdateEventId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceId),
          ServerNotifyEvent(skeletonAdapter->GetSkeleton(), events::ConfigServerServiceInterfaceServerNotifyEventId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceId),
          AnswerAliveHandle(skeletonAdapter->GetSkeleton(), methods::ConfigServerServiceInterfaceAnswerAliveId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceId, mode),
          DelParamHandle(skeletonAdapter->GetSkeleton(), methods::ConfigServerServiceInterfaceDelParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceId, mode),
          GetMonitorClientsHandle(skeletonAdapter->GetSkeleton(), methods::ConfigServerServiceInterfaceGetMonitorClientsId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceId, mode),
          GetParamHandle(skeletonAdapter->GetSkeleton(), methods::ConfigServerServiceInterfaceGetParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceId, mode),
          MonitorParamHandle(skeletonAdapter->GetSkeleton(), methods::ConfigServerServiceInterfaceMonitorParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceId, mode),
          SetParamHandle(skeletonAdapter->GetSkeleton(), methods::ConfigServerServiceInterfaceSetParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceId, mode),
          UnMonitorParamHandle(skeletonAdapter->GetSkeleton(), methods::ConfigServerServiceInterfaceUnMonitorParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceId, mode),
          InitClientHandle(skeletonAdapter->GetSkeleton(), methods::ConfigServerServiceInterfaceInitClientId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceId, mode){
        ConstructSkeleton(mode);
    }

    explicit ConfigServerServiceInterfaceSkeleton(const ara::core::InstanceSpecifier& instanceSpec,
                           const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
        :skeletonAdapter(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceSpec, mode)),
          ParamUpdateEvent(skeletonAdapter->GetSkeleton(), events::ConfigServerServiceInterfaceParamUpdateEventId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceSpec),
          ServerNotifyEvent(skeletonAdapter->GetSkeleton(), events::ConfigServerServiceInterfaceServerNotifyEventId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceSpec),
          AnswerAliveHandle(skeletonAdapter->GetSkeleton(), methods::ConfigServerServiceInterfaceAnswerAliveId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceSpec, mode),
          DelParamHandle(skeletonAdapter->GetSkeleton(), methods::ConfigServerServiceInterfaceDelParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceSpec, mode),
          GetMonitorClientsHandle(skeletonAdapter->GetSkeleton(), methods::ConfigServerServiceInterfaceGetMonitorClientsId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceSpec, mode),
          GetParamHandle(skeletonAdapter->GetSkeleton(), methods::ConfigServerServiceInterfaceGetParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceSpec, mode),
          MonitorParamHandle(skeletonAdapter->GetSkeleton(), methods::ConfigServerServiceInterfaceMonitorParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceSpec, mode),
          SetParamHandle(skeletonAdapter->GetSkeleton(), methods::ConfigServerServiceInterfaceSetParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceSpec, mode),
          UnMonitorParamHandle(skeletonAdapter->GetSkeleton(), methods::ConfigServerServiceInterfaceUnMonitorParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceSpec, mode),
          InitClientHandle(skeletonAdapter->GetSkeleton(), methods::ConfigServerServiceInterfaceInitClientId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceSpec, mode){
        ConstructSkeleton(mode);
    }

    explicit ConfigServerServiceInterfaceSkeleton(const ara::com::InstanceIdentifierContainer instanceContainer,
                           const ara::com::MethodCallProcessingMode mode = ara::com::MethodCallProcessingMode::kEvent)
        :skeletonAdapter(std::make_unique<ara::com::internal::skeleton::SkeletonAdapter>(::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceContainer, mode)),
          ParamUpdateEvent(skeletonAdapter->GetSkeleton(), events::ConfigServerServiceInterfaceParamUpdateEventId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceContainer),
          ServerNotifyEvent(skeletonAdapter->GetSkeleton(), events::ConfigServerServiceInterfaceServerNotifyEventId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceContainer),
          AnswerAliveHandle(skeletonAdapter->GetSkeleton(), methods::ConfigServerServiceInterfaceAnswerAliveId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceContainer, mode),
          DelParamHandle(skeletonAdapter->GetSkeleton(), methods::ConfigServerServiceInterfaceDelParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceContainer, mode),
          GetMonitorClientsHandle(skeletonAdapter->GetSkeleton(), methods::ConfigServerServiceInterfaceGetMonitorClientsId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceContainer, mode),
          GetParamHandle(skeletonAdapter->GetSkeleton(), methods::ConfigServerServiceInterfaceGetParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceContainer, mode),
          MonitorParamHandle(skeletonAdapter->GetSkeleton(), methods::ConfigServerServiceInterfaceMonitorParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceContainer, mode),
          SetParamHandle(skeletonAdapter->GetSkeleton(), methods::ConfigServerServiceInterfaceSetParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceContainer, mode),
          UnMonitorParamHandle(skeletonAdapter->GetSkeleton(), methods::ConfigServerServiceInterfaceUnMonitorParamId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceContainer, mode),
          InitClientHandle(skeletonAdapter->GetSkeleton(), methods::ConfigServerServiceInterfaceInitClientId, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instanceContainer, mode){
        ConstructSkeleton(mode);
    }

    ConfigServerServiceInterfaceSkeleton(const ConfigServerServiceInterfaceSkeleton&) = delete;

    ConfigServerServiceInterfaceSkeleton(ConfigServerServiceInterfaceSkeleton&&) = default;
    ConfigServerServiceInterfaceSkeleton& operator=(ConfigServerServiceInterfaceSkeleton&&) = default;
    ConfigServerServiceInterfaceSkeleton(ConstructionToken&& token) noexcept
        : skeletonAdapter(token.GetSkeletonAdapter()),
          ParamUpdateEvent(token.GetParamUpdateEvent()),
          ServerNotifyEvent(token.GetServerNotifyEvent()),
          AnswerAliveHandle(token.GetAnswerAliveHandle()),
          DelParamHandle(token.GetDelParamHandle()),
          GetMonitorClientsHandle(token.GetGetMonitorClientsHandle()),
          GetParamHandle(token.GetGetParamHandle()),
          MonitorParamHandle(token.GetMonitorParamHandle()),
          SetParamHandle(token.GetSetParamHandle()),
          UnMonitorParamHandle(token.GetUnMonitorParamHandle()),
          InitClientHandle(token.GetInitClientHandle()){
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

    virtual ~ConfigServerServiceInterfaceSkeleton()
    {
        StopOfferService();
    }

    void OfferService()
    {
        skeletonAdapter->RegisterE2EErrorHandler(&ConfigServerServiceInterfaceSkeleton::E2EErrorHandler, *this);
        skeletonAdapter->RegisterMethod(&ConfigServerServiceInterfaceSkeleton::AnswerAlive, *this, methods::ConfigServerServiceInterfaceAnswerAliveId);
        skeletonAdapter->RegisterMethod(&ConfigServerServiceInterfaceSkeleton::DelParam, *this, methods::ConfigServerServiceInterfaceDelParamId);
        skeletonAdapter->RegisterMethod(&ConfigServerServiceInterfaceSkeleton::GetMonitorClients, *this, methods::ConfigServerServiceInterfaceGetMonitorClientsId);
        skeletonAdapter->RegisterMethod(&ConfigServerServiceInterfaceSkeleton::GetParam, *this, methods::ConfigServerServiceInterfaceGetParamId);
        skeletonAdapter->RegisterMethod(&ConfigServerServiceInterfaceSkeleton::MonitorParam, *this, methods::ConfigServerServiceInterfaceMonitorParamId);
        skeletonAdapter->RegisterMethod(&ConfigServerServiceInterfaceSkeleton::SetParam, *this, methods::ConfigServerServiceInterfaceSetParamId);
        skeletonAdapter->RegisterMethod(&ConfigServerServiceInterfaceSkeleton::UnMonitorParam, *this, methods::ConfigServerServiceInterfaceUnMonitorParamId);
        skeletonAdapter->RegisterMethod(&ConfigServerServiceInterfaceSkeleton::InitClient, *this, methods::ConfigServerServiceInterfaceInitClientId);
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

    virtual ara::core::Future<AnswerAliveOutput> AnswerAlive(const ::String& clientName) = 0;
    virtual ara::core::Future<DelParamOutput> DelParam(const ::String& clientName, const ::String& paramName) = 0;
    virtual ara::core::Future<GetMonitorClientsOutput> GetMonitorClients(const ::String& clientName, const ::String& paramName) = 0;
    virtual ara::core::Future<GetParamOutput> GetParam(const ::String& clientName, const ::String& paramName) = 0;
    virtual ara::core::Future<MonitorParamOutput> MonitorParam(const ::String& clientName, const ::String& paramName) = 0;
    virtual ara::core::Future<SetParamOutput> SetParam(const ::String& clientName, const ::String& paramName, const ::String& paramValue, const ::UInt8& paramType, const ::UInt8& persistType, const ::String& uuid) = 0;
    virtual ara::core::Future<UnMonitorParamOutput> UnMonitorParam(const ::String& clientName, const ::String& paramName) = 0;
    virtual ara::core::Future<InitClientOutput> InitClient(const ::String& clientName) = 0;
    virtual void E2EErrorHandler(ara::com::e2e::E2EErrorCode, ara::com::e2e::DataID, ara::com::e2e::MessageCounter){}

    events::ParamUpdateEvent ParamUpdateEvent;
    events::ServerNotifyEvent ServerNotifyEvent;
    methods::AnswerAliveHandle AnswerAliveHandle;
    methods::DelParamHandle DelParamHandle;
    methods::GetMonitorClientsHandle GetMonitorClientsHandle;
    methods::GetParamHandle GetParamHandle;
    methods::MonitorParamHandle MonitorParamHandle;
    methods::SetParamHandle SetParamHandle;
    methods::UnMonitorParamHandle UnMonitorParamHandle;
    methods::InitClientHandle InitClientHandle;
};
} // namespace skeleton
} // namespace server
} // namespace config
} // namespace mdc

#endif // MDC_CONFIG_SERVER_CONFIGSERVERSERVICEINTERFACE_SKELETON_H
