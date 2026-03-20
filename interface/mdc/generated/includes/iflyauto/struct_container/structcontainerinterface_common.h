/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef IFLYAUTO_STRUCT_CONTAINER_STRUCTCONTAINERINTERFACE_COMMON_H
#define IFLYAUTO_STRUCT_CONTAINER_STRUCTCONTAINERINTERFACE_COMMON_H

#include "ara/com/types.h"
#include "ara/com/init_config.h"
#include "ara/diag/impl_type_bytevector.h"
#include <cfloat>
#include <cmath>

namespace iflyauto {
namespace struct_container {

class StructContainerInterface {
public:
    constexpr StructContainerInterface() = default;
    constexpr static ara::com::ServiceIdentifierType ServiceIdentifier = ara::com::ServiceIdentifierType("/HuaweiMDC/interface_manifests/interfaces/StructContainerInterface");
    constexpr static ara::com::ServiceVersionType ServiceVersion = ara::com::ServiceVersionType("1.1");
};
} // namespace struct_container
} // namespace iflyauto

#endif // IFLYAUTO_STRUCT_CONTAINER_STRUCTCONTAINERINTERFACE_COMMON_H
