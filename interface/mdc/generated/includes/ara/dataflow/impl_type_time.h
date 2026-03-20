/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef ARA_DATAFLOW_IMPL_TYPE_TIME_H
#define ARA_DATAFLOW_IMPL_TYPE_TIME_H

#include "impl_type_uint32.h"

namespace ara {
namespace dataflow {
struct Time {
    ::UInt32 sec;
    ::UInt32 nsec;

    static bool IsPlane()
    {
        return true;
    }


    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(sec);
        fun(nsec);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(sec);
        fun(nsec);
    }

    template<typename F>
    void enumerate_internal(F& fun)
    {
        fun("sec", sec);
        fun("nsec", nsec);
    }

    template<typename F>
    void enumerate_internal(F& fun) const
    {
        fun("sec", sec);
        fun("nsec", nsec);
    }

    friend bool operator==(const ::ara::dataflow::Time& lhs, const ::ara::dataflow::Time& rhs) noexcept
    {
        return (lhs.sec == rhs.sec) && (lhs.nsec == rhs.nsec);
    }
};
} // namespace dataflow
} // namespace ara


#endif // ARA_DATAFLOW_IMPL_TYPE_TIME_H
