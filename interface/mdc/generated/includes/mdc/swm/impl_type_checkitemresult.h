/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_SWM_IMPL_TYPE_CHECKITEMRESULT_H
#define MDC_SWM_IMPL_TYPE_CHECKITEMRESULT_H

#include "impl_type_string.h"
#include "impl_type_int32.h"

namespace mdc {
namespace swm {
struct CheckItemResult {
    ::String name;
    ::Int32 result;
    ::String message;

    static bool IsPlane()
    {
        return false;
    }


    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(name);
        fun(result);
        fun(message);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(name);
        fun(result);
        fun(message);
    }

    template<typename F>
    void enumerate_internal(F& fun)
    {
        fun("name", name);
        fun("result", result);
        fun("message", message);
    }

    template<typename F>
    void enumerate_internal(F& fun) const
    {
        fun("name", name);
        fun("result", result);
        fun("message", message);
    }

    friend bool operator==(const ::mdc::swm::CheckItemResult& lhs, const ::mdc::swm::CheckItemResult& rhs) noexcept
    {
        return (lhs.name == rhs.name) && (lhs.result == rhs.result) && (lhs.message == rhs.message);
    }
};
} // namespace swm
} // namespace mdc


#endif // MDC_SWM_IMPL_TYPE_CHECKITEMRESULT_H
