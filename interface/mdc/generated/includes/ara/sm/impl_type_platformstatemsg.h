/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef ARA_SM_IMPL_TYPE_PLATFORMSTATEMSG_H
#define ARA_SM_IMPL_TYPE_PLATFORMSTATEMSG_H


namespace ara {
namespace sm {
struct PlatformStateMsg {
    invalid platformState;

    static bool IsPlane()
    {
        return true;
    }


    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
    }

    template<typename F>
    void enumerate(F& fun) const
    {
    }

    template<typename F>
    void enumerate_internal(F& fun)
    {
    }

    template<typename F>
    void enumerate_internal(F& fun) const
    {
    }

    friend bool operator==(const ::ara::sm::PlatformStateMsg& lhs, const ::ara::sm::PlatformStateMsg& rhs) noexcept
    {
        return (lhs.platformState == rhs.platformState);
    }
};
} // namespace sm
} // namespace ara


#endif // ARA_SM_IMPL_TYPE_PLATFORMSTATEMSG_H
