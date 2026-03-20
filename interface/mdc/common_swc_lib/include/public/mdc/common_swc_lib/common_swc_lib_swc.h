/*
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * Description: class swc
 */

#ifndef MDC_COMMONSWCLIBSWC_H
#define MDC_COMMONSWCLIBSWC_H

#include <iostream>
#include <unordered_map>
#include <algorithm>
#include <memory>
#include <atomic>
#include <mutex>
#include <sys/types.h>

#include "ara/core/string.h"
#include "ara/core/vector.h"
#include "ara/core/map.h"

namespace mdc {
class MdcYamlNode;
}

namespace iflyauto {
namespace struct_container {
class StructContainerInterfaceServer;
}
}
namespace iflyauto {
namespace struct_container {
class StructContainerInterfaceClient;
}
}
namespace iflyauto {
namespace camera_struct_container {
class CameraStructContainerInterfaceServer;
}
}
namespace iflyauto {
namespace camera_struct_container {
class CameraStructContainerInterfaceClient;
}
}
namespace mdc {
namespace fm {
class FmAlarmReceiveServiceServer;
}
}
namespace mdc {
namespace fm {
class FmAlarmReceiveServiceClient;
}
}
namespace iflyauto {
namespace struct_container {
class ClientServiceContainerInterfaceServer;
}
}
namespace iflyauto {
namespace struct_container {
class ClientServiceContainerInterfaceClient;
}
}
namespace mdc {
namespace common_swc_lib {
namespace {
using iflyauto::struct_container::StructContainerInterfaceServer;
using iflyauto::struct_container::StructContainerInterfaceClient;
using iflyauto::camera_struct_container::CameraStructContainerInterfaceServer;
using iflyauto::camera_struct_container::CameraStructContainerInterfaceClient;
using mdc::fm::FmAlarmReceiveServiceServer;
using mdc::fm::FmAlarmReceiveServiceClient;
using iflyauto::struct_container::ClientServiceContainerInterfaceServer;
using iflyauto::struct_container::ClientServiceContainerInterfaceClient;
using InitHandleType = std::function<bool()>;
using StopHandleType = std::function<void()>;
}

class CommonSwcLibSwC {
public:
    explicit CommonSwcLibSwC();

    virtual ~CommonSwcLibSwC();

    bool Init();

    void Stop();

    inline void SetInitCallback(InitHandleType handler)
    {
        initHandler_ = handler;
    }

    inline void SetStopCallback(StopHandleType handler)
    {
        stopHandler_ = handler;
    }

    inline bool IsStop() const
    {
        return !workFlag_;
    }

    ara::core::Vector<std::string> GetStructContainerInterfaceServerVec()
    {
        std::lock_guard<std::mutex> lck(structContainerInterfaceServerPortVecMtx_);
        return structContainerInterfaceServerPortVec_;
    }
    ara::core::Vector<std::string> GetStructContainerInterfaceClientVec()
    {
        std::lock_guard<std::mutex> lck(structContainerInterfaceClientPortVecMtx_);
        return structContainerInterfaceClientPortVec_;
    }
    ara::core::Vector<std::string> GetCameraStructContainerInterfaceServerVec()
    {
        std::lock_guard<std::mutex> lck(cameraStructContainerInterfaceServerPortVecMtx_);
        return cameraStructContainerInterfaceServerPortVec_;
    }
    ara::core::Vector<std::string> GetCameraStructContainerInterfaceClientVec()
    {
        std::lock_guard<std::mutex> lck(cameraStructContainerInterfaceClientPortVecMtx_);
        return cameraStructContainerInterfaceClientPortVec_;
    }
    ara::core::Vector<std::string> GetFmAlarmReceiveServiceServerVec()
    {
        std::lock_guard<std::mutex> lck(fmAlarmReceiveServiceServerPortVecMtx_);
        return fmAlarmReceiveServiceServerPortVec_;
    }
    ara::core::Vector<std::string> GetFmAlarmReceiveServiceClientVec()
    {
        std::lock_guard<std::mutex> lck(fmAlarmReceiveServiceClientPortVecMtx_);
        return fmAlarmReceiveServiceClientPortVec_;
    }
    ara::core::Vector<std::string> GetClientServiceContainerInterfaceServerVec()
    {
        std::lock_guard<std::mutex> lck(clientServiceContainerInterfaceServerPortVecMtx_);
        return clientServiceContainerInterfaceServerPortVec_;
    }
    ara::core::Vector<std::string> GetClientServiceContainerInterfaceClientVec()
    {
        std::lock_guard<std::mutex> lck(clientServiceContainerInterfaceClientPortVecMtx_);
        return clientServiceContainerInterfaceClientPortVec_;
    }
    

    std::shared_ptr<StructContainerInterfaceServer> GetStructContainerInterfaceServer(const ara::core::String& portName);
    std::shared_ptr<StructContainerInterfaceClient> GetStructContainerInterfaceClient(const ara::core::String& portName);
    std::shared_ptr<CameraStructContainerInterfaceServer> GetCameraStructContainerInterfaceServer(const ara::core::String& portName);
    std::shared_ptr<CameraStructContainerInterfaceClient> GetCameraStructContainerInterfaceClient(const ara::core::String& portName);
    std::shared_ptr<FmAlarmReceiveServiceServer> GetFmAlarmReceiveServiceServer(const ara::core::String& portName);
    std::shared_ptr<FmAlarmReceiveServiceClient> GetFmAlarmReceiveServiceClient(const ara::core::String& portName);
    std::shared_ptr<ClientServiceContainerInterfaceServer> GetClientServiceContainerInterfaceServer(const ara::core::String& portName);
    std::shared_ptr<ClientServiceContainerInterfaceClient> GetClientServiceContainerInterfaceClient(const ara::core::String& portName);
    

private:
    /**
     * @brief 加载instanceID等初始化配置
     *
     */
    bool LoadConfig();

    /**
     * @brief 解析swc管理的interface的instanceID信息
     *
     */
    bool ParsingInstanceId(const std::unique_ptr<MdcYamlNode>& config);

    /**
     * @brief 判断文件是否存在
     *
     */
    bool DoesFileExist(const ara::core::String& filePath);
    bool IsDirType(const mode_t& fileMode);

private:
    /* 初始化标识, 防止多次初始化 */
    std::atomic<bool> initFlag_;

    /* 工作标识 */
    std::atomic<bool> workFlag_;

    /* init回调函数 */
    InitHandleType initHandler_{nullptr};

    /* stop回调函数 */
    StopHandleType stopHandler_{nullptr};

    /* 参数文件路径 */
    ara::core::String configFile_{};

    /* swc标识符 */
    ara::core::String identifier_{};

    /* logger描述符 */
    ara::core::String loggerId_{};
    /* server agent */
    ara::core::Vector<uint32_t> structContainerInterfaceServerIdVec_;
    ara::core::Vector<std::string> structContainerInterfaceServerPortVec_;
    std::mutex structContainerInterfaceServerPortVecMtx_;
    ara::core::Map<std::string, uint32_t> structContainerInterfaceServerIdMap_;
    ara::core::Map<std::string, std::shared_ptr<StructContainerInterfaceServer>> structContainerInterfaceServerInsMap_;
    std::mutex structContainerInterfaceServerInsMapMtx_;
    /* server agent */
    ara::core::Vector<uint32_t> cameraStructContainerInterfaceServerIdVec_;
    ara::core::Vector<std::string> cameraStructContainerInterfaceServerPortVec_;
    std::mutex cameraStructContainerInterfaceServerPortVecMtx_;
    ara::core::Map<std::string, uint32_t> cameraStructContainerInterfaceServerIdMap_;
    ara::core::Map<std::string, std::shared_ptr<CameraStructContainerInterfaceServer>> cameraStructContainerInterfaceServerInsMap_;
    std::mutex cameraStructContainerInterfaceServerInsMapMtx_;
    /* server agent */
    ara::core::Vector<uint32_t> fmAlarmReceiveServiceServerIdVec_;
    ara::core::Vector<std::string> fmAlarmReceiveServiceServerPortVec_;
    std::mutex fmAlarmReceiveServiceServerPortVecMtx_;
    ara::core::Map<std::string, uint32_t> fmAlarmReceiveServiceServerIdMap_;
    ara::core::Map<std::string, std::shared_ptr<FmAlarmReceiveServiceServer>> fmAlarmReceiveServiceServerInsMap_;
    std::mutex fmAlarmReceiveServiceServerInsMapMtx_;
    /* server agent */
    ara::core::Vector<uint32_t> clientServiceContainerInterfaceServerIdVec_;
    ara::core::Vector<std::string> clientServiceContainerInterfaceServerPortVec_;
    std::mutex clientServiceContainerInterfaceServerPortVecMtx_;
    ara::core::Map<std::string, uint32_t> clientServiceContainerInterfaceServerIdMap_;
    ara::core::Map<std::string, std::shared_ptr<ClientServiceContainerInterfaceServer>> clientServiceContainerInterfaceServerInsMap_;
    std::mutex clientServiceContainerInterfaceServerInsMapMtx_;
    /* client agent */
    ara::core::Vector<uint32_t> structContainerInterfaceClientIdVec_;
    ara::core::Vector<std::string> structContainerInterfaceClientPortVec_;
    std::mutex structContainerInterfaceClientPortVecMtx_;
    ara::core::Map<std::string, uint32_t> structContainerInterfaceClientIdMap_;
    ara::core::Map<std::string, std::shared_ptr<StructContainerInterfaceClient>> structContainerInterfaceClientInsMap_;
    std::mutex structContainerInterfaceClientInsMapMtx_;
    /* client agent */
    ara::core::Vector<uint32_t> cameraStructContainerInterfaceClientIdVec_;
    ara::core::Vector<std::string> cameraStructContainerInterfaceClientPortVec_;
    std::mutex cameraStructContainerInterfaceClientPortVecMtx_;
    ara::core::Map<std::string, uint32_t> cameraStructContainerInterfaceClientIdMap_;
    ara::core::Map<std::string, std::shared_ptr<CameraStructContainerInterfaceClient>> cameraStructContainerInterfaceClientInsMap_;
    std::mutex cameraStructContainerInterfaceClientInsMapMtx_;
    /* client agent */
    ara::core::Vector<uint32_t> fmAlarmReceiveServiceClientIdVec_;
    ara::core::Vector<std::string> fmAlarmReceiveServiceClientPortVec_;
    std::mutex fmAlarmReceiveServiceClientPortVecMtx_;
    ara::core::Map<std::string, uint32_t> fmAlarmReceiveServiceClientIdMap_;
    ara::core::Map<std::string, std::shared_ptr<FmAlarmReceiveServiceClient>> fmAlarmReceiveServiceClientInsMap_;
    std::mutex fmAlarmReceiveServiceClientInsMapMtx_;
    /* client agent */
    ara::core::Vector<uint32_t> clientServiceContainerInterfaceClientIdVec_;
    ara::core::Vector<std::string> clientServiceContainerInterfaceClientPortVec_;
    std::mutex clientServiceContainerInterfaceClientPortVecMtx_;
    ara::core::Map<std::string, uint32_t> clientServiceContainerInterfaceClientIdMap_;
    ara::core::Map<std::string, std::shared_ptr<ClientServiceContainerInterfaceClient>> clientServiceContainerInterfaceClientInsMap_;
    std::mutex clientServiceContainerInterfaceClientInsMapMtx_;

};
}  /* namespace common_swc_lib */
}  /* namespace mdc */

#endif  /* MDC_COMMONSWCLIBSWC_H */