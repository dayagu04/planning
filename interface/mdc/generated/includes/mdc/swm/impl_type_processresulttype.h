/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_SWM_IMPL_TYPE_PROCESSRESULTTYPE_H
#define MDC_SWM_IMPL_TYPE_PROCESSRESULTTYPE_H

#include "mdc/swm/impl_type_processswpackagereturntype.h"
#include "mdc/swm/impl_type_transferidtype.h"

namespace mdc {
namespace swm {
struct ProcessResultType {
    ::mdc::swm::ProcessSwPackageReturnType result;
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

    friend bool operator==(const ::mdc::swm::ProcessResultType& lhs, const ::mdc::swm::ProcessResultType& rhs) noexcept
    {
        return (lhs.result == rhs.result) && (lhs.transferId == rhs.transferId);
    }
};
} // namespace swm
} // namespace mdc


#endif // MDC_SWM_IMPL_TYPE_PROCESSRESULTTYPE_H
