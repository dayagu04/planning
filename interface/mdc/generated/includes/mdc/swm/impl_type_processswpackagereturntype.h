/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_SWM_IMPL_TYPE_PROCESSSWPACKAGERETURNTYPE_H
#define MDC_SWM_IMPL_TYPE_PROCESSSWPACKAGERETURNTYPE_H

#include "impl_type_string.h"
#include "mdc/swm/impl_type_transferidtype.h"

namespace mdc {
namespace swm {
struct ProcessSwPackageReturnType {
    ::String result;
    ::mdc::swm::TransferIdType transferId{};

    static bool IsPlane()
    {
        return false;
    }


    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(result);
        fun(transferId);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(result);
        fun(transferId);
    }

    template<typename F>
    void enumerate_internal(F& fun)
    {
        fun("result", result);
        fun("transferId", transferId);
    }

    template<typename F>
    void enumerate_internal(F& fun) const
    {
        fun("result", result);
        fun("transferId", transferId);
    }

    friend bool operator==(const ::mdc::swm::ProcessSwPackageReturnType& lhs, const ::mdc::swm::ProcessSwPackageReturnType& rhs) noexcept
    {
        return (lhs.result == rhs.result) && (lhs.transferId == rhs.transferId);
    }
};
} // namespace swm
} // namespace mdc


#endif // MDC_SWM_IMPL_TYPE_PROCESSSWPACKAGERETURNTYPE_H
