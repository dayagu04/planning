/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef ARA_SM_IMPL_TYPE_SINGLESTATEFIELD_H
#define ARA_SM_IMPL_TYPE_SINGLESTATEFIELD_H

#include "impl_type_string.h"

namespace ara {
namespace sm {
struct SingleStateField {
    ::String functionGroupName;
    ::String stateName;
    invalid processResult;

    static bool IsPlane()
    {
        return false;
    }


    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(functionGroupName);
        fun(stateName);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(functionGroupName);
        fun(stateName);
    }

    template<typename F>
    void enumerate_internal(F& fun)
    {
        fun("functionGroupName", functionGroupName);
        fun("stateName", stateName);
    }

    template<typename F>
    void enumerate_internal(F& fun) const
    {
        fun("functionGroupName", functionGroupName);
        fun("stateName", stateName);
    }

    friend bool operator==(const ::ara::sm::SingleStateField& lhs, const ::ara::sm::SingleStateField& rhs) noexcept
    {
        return (lhs.functionGroupName == rhs.functionGroupName) && (lhs.stateName == rhs.stateName) && (lhs.processResult == rhs.processResult);
    }
};
} // namespace sm
} // namespace ara


#endif // ARA_SM_IMPL_TYPE_SINGLESTATEFIELD_H
