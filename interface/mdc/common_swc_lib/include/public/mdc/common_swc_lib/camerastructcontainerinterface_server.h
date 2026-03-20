/* *
 * Class: CameraStructContainerInterface server declaration
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */
#ifndef IFLYAUTO_CAMERA_STRUCT_CONTAINER_CAMERASTRUCTCONTAINERINTERFACESERVER
#define IFLYAUTO_CAMERA_STRUCT_CONTAINER_CAMERASTRUCTCONTAINERINTERFACESERVER
#include <memory>

#include "iflyauto/camera_struct_container/impl_type_camerastructcontainer.h"





namespace iflyauto {
namespace camera_struct_container {

using eventDataType = iflyauto::camera_struct_container::CameraStructContainer;
using SendEventType = std::shared_ptr<eventDataType>;


class CameraStructContainerInterfaceServer final {
public:
    CameraStructContainerInterfaceServer() = delete;

    explicit CameraStructContainerInterfaceServer(const uint32_t id);

    virtual ~CameraStructContainerInterfaceServer();

    bool Init();

    void Stop();

    bool IsStop() const;

    uint32_t GetInstanceId() const;

    /* event relative */
    bool SendEventData(const SendEventType& data);
    void ClearEventContainer();
    
    

private:
    class Impl;
    std::unique_ptr<Impl> pImpl_;
};
} /* namespace camera_struct_container */
} /* namespace iflyauto */

#endif