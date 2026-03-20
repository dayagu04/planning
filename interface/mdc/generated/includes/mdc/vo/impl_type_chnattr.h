/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_VO_IMPL_TYPE_CHNATTR_H
#define MDC_VO_IMPL_TYPE_CHNATTR_H


namespace mdc {
namespace vo {
struct ChnAttr {
    invalid isEnabled;

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

    friend bool operator==(const ::mdc::vo::ChnAttr& lhs, const ::mdc::vo::ChnAttr& rhs) noexcept
    {
        return (lhs.isEnabled == rhs.isEnabled);
    }
};
} // namespace vo
} // namespace mdc


#endif // MDC_VO_IMPL_TYPE_CHNATTR_H
