/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_LPM_IMPL_TYPE_MDCWAKEUPEVENTDATA_H
#define MDC_LPM_IMPL_TYPE_MDCWAKEUPEVENTDATA_H


namespace mdc {
namespace lpm {
struct MdcWakeupEventData {
    invalid isWakeup;

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

    friend bool operator==(const ::mdc::lpm::MdcWakeupEventData& lhs, const ::mdc::lpm::MdcWakeupEventData& rhs) noexcept
    {
        return (lhs.isWakeup == rhs.isWakeup);
    }
};
} // namespace lpm
} // namespace mdc


#endif // MDC_LPM_IMPL_TYPE_MDCWAKEUPEVENTDATA_H
