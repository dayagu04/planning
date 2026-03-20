/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef ARA_SM_TRIGGERINOUT_COMMON_H
#define ARA_SM_TRIGGERINOUT_COMMON_H

#include "ara/com/types.h"
#include "ara/com/init_config.h"
#include "ara/sm/impl_type_triggerdatatype.h"
#include "ara/sm/impl_type_functiongroupinfovec.h"
#include "ara/sm/impl_type_statetransitionvec.h"
#include "ara/sm/impl_type_resetcode.h"
#include "impl_type_string.h"
#include "ara/sm/impl_type_resetcause.h"
#include <cfloat>
#include <cmath>

namespace ara {
namespace sm {
namespace methods {
namespace AcquireFunctionGroupInfo {
struct Output {
    ::ara::sm::FunctionGroupInfoVec functionGroupInfoVec;

    static bool IsPlane()
    {
        return false;
    }

    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(functionGroupInfoVec);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(functionGroupInfoVec);
    }

    bool operator==(const Output& t) const
    {
       return (functionGroupInfoVec == t.functionGroupInfoVec);
    }
};
} // namespace AcquireFunctionGroupInfo
} // namespace methods

class TriggerInOut {
public:
    constexpr TriggerInOut() = default;
    constexpr static ara::com::ServiceIdentifierType ServiceIdentifier = ara::com::ServiceIdentifierType("/HuaweiMDC/PlatformServiceInterface/SmcServiceInterface/TriggerInOut");
    constexpr static ara::com::ServiceVersionType ServiceVersion = ara::com::ServiceVersionType("1.1");
};
} // namespace sm
} // namespace ara

#endif // ARA_SM_TRIGGERINOUT_COMMON_H
