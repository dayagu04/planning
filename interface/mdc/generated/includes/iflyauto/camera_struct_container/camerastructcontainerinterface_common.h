/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef IFLYAUTO_CAMERA_STRUCT_CONTAINER_CAMERASTRUCTCONTAINERINTERFACE_COMMON_H
#define IFLYAUTO_CAMERA_STRUCT_CONTAINER_CAMERASTRUCTCONTAINERINTERFACE_COMMON_H

#include "ara/com/types.h"
#include "ara/com/init_config.h"
#include "iflyauto/camera_struct_container/impl_type_camerastructcontainer.h"
#include <cfloat>
#include <cmath>

namespace iflyauto {
namespace camera_struct_container {

class CameraStructContainerInterface {
public:
    constexpr CameraStructContainerInterface() = default;
    constexpr static ara::com::ServiceIdentifierType ServiceIdentifier = ara::com::ServiceIdentifierType("/HuaweiMDC/interface_manifests/interfaces/CameraStructContainerInterface");
    constexpr static ara::com::ServiceVersionType ServiceVersion = ara::com::ServiceVersionType("1.1");
};
} // namespace camera_struct_container
} // namespace iflyauto

#endif // IFLYAUTO_CAMERA_STRUCT_CONTAINER_CAMERASTRUCTCONTAINERINTERFACE_COMMON_H
