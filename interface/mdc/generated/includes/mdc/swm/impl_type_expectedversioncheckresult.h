/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_SWM_IMPL_TYPE_EXPECTEDVERSIONCHECKRESULT_H
#define MDC_SWM_IMPL_TYPE_EXPECTEDVERSIONCHECKRESULT_H

#include "impl_type_uint8.h"
#include "mdc/swm/impl_type_sysversion.h"

namespace mdc {
namespace swm {
struct ExpectedVersionCheckResult {
    ::UInt8 state;
    ::UInt8 checkStatus;
    ::mdc::swm::SysVersion expectedVersion;
    ::mdc::swm::SysVersion currVersion;

    static bool IsPlane()
    {
        return false;
    }


    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(state);
        fun(checkStatus);
        fun(expectedVersion);
        fun(currVersion);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(state);
        fun(checkStatus);
        fun(expectedVersion);
        fun(currVersion);
    }

    template<typename F>
    void enumerate_internal(F& fun)
    {
        fun("state", state);
        fun("checkStatus", checkStatus);
        fun("expectedVersion", expectedVersion);
        fun("currVersion", currVersion);
    }

    template<typename F>
    void enumerate_internal(F& fun) const
    {
        fun("state", state);
        fun("checkStatus", checkStatus);
        fun("expectedVersion", expectedVersion);
        fun("currVersion", currVersion);
    }

    friend bool operator==(const ::mdc::swm::ExpectedVersionCheckResult& lhs, const ::mdc::swm::ExpectedVersionCheckResult& rhs) noexcept
    {
        return (lhs.state == rhs.state) && (lhs.checkStatus == rhs.checkStatus) && (lhs.expectedVersion == rhs.expectedVersion) && (lhs.currVersion == rhs.currVersion);
    }
};
} // namespace swm
} // namespace mdc


#endif // MDC_SWM_IMPL_TYPE_EXPECTEDVERSIONCHECKRESULT_H
