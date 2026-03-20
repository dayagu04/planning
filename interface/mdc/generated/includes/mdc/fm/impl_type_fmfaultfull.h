/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_FM_IMPL_TYPE_FMFAULTFULL_H
#define MDC_FM_IMPL_TYPE_FMFAULTFULL_H

#include "impl_type_uint16.h"
#include "impl_type_uint8.h"
#include "impl_type_uint32.h"
#include "impl_type_string.h"
#include "impl_type_int32.h"
#include "mdc/fm/impl_type_fmstringvec.h"

namespace mdc {
namespace fm {
struct FmFaultFull {
    ::UInt16 faultId;
    ::UInt16 faultObj;
    ::UInt8 phyCpuNo;
    ::UInt8 cpuCoreNo;
    ::UInt8 type;
    ::UInt8 clss;
    ::UInt8 level;
    ::UInt32 flag;
    ::UInt32 componentId;
    ::UInt32 count;
    ::String timeFirst;
    ::String timeLast;
    ::String para;
    ::String desc;
    ::String actionName;
    ::Int32 actionResult;
    ::mdc::fm::FmStringVec subFaultVec;

    static bool IsPlane()
    {
        return false;
    }


    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(faultId);
        fun(faultObj);
        fun(phyCpuNo);
        fun(cpuCoreNo);
        fun(type);
        fun(clss);
        fun(level);
        fun(flag);
        fun(componentId);
        fun(count);
        fun(timeFirst);
        fun(timeLast);
        fun(para);
        fun(desc);
        fun(actionName);
        fun(actionResult);
        fun(subFaultVec);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(faultId);
        fun(faultObj);
        fun(phyCpuNo);
        fun(cpuCoreNo);
        fun(type);
        fun(clss);
        fun(level);
        fun(flag);
        fun(componentId);
        fun(count);
        fun(timeFirst);
        fun(timeLast);
        fun(para);
        fun(desc);
        fun(actionName);
        fun(actionResult);
        fun(subFaultVec);
    }

    template<typename F>
    void enumerate_internal(F& fun)
    {
        fun("faultId", faultId);
        fun("faultObj", faultObj);
        fun("phyCpuNo", phyCpuNo);
        fun("cpuCoreNo", cpuCoreNo);
        fun("type", type);
        fun("clss", clss);
        fun("level", level);
        fun("flag", flag);
        fun("componentId", componentId);
        fun("count", count);
        fun("timeFirst", timeFirst);
        fun("timeLast", timeLast);
        fun("para", para);
        fun("desc", desc);
        fun("actionName", actionName);
        fun("actionResult", actionResult);
        fun("subFaultVec", subFaultVec);
    }

    template<typename F>
    void enumerate_internal(F& fun) const
    {
        fun("faultId", faultId);
        fun("faultObj", faultObj);
        fun("phyCpuNo", phyCpuNo);
        fun("cpuCoreNo", cpuCoreNo);
        fun("type", type);
        fun("clss", clss);
        fun("level", level);
        fun("flag", flag);
        fun("componentId", componentId);
        fun("count", count);
        fun("timeFirst", timeFirst);
        fun("timeLast", timeLast);
        fun("para", para);
        fun("desc", desc);
        fun("actionName", actionName);
        fun("actionResult", actionResult);
        fun("subFaultVec", subFaultVec);
    }

    friend bool operator==(const ::mdc::fm::FmFaultFull& lhs, const ::mdc::fm::FmFaultFull& rhs) noexcept
    {
        return (lhs.faultId == rhs.faultId) && (lhs.faultObj == rhs.faultObj) && (lhs.phyCpuNo == rhs.phyCpuNo) && (lhs.cpuCoreNo == rhs.cpuCoreNo) && (lhs.type == rhs.type) && (lhs.clss == rhs.clss) && (lhs.level == rhs.level) && (lhs.flag == rhs.flag) && (lhs.componentId == rhs.componentId) && (lhs.count == rhs.count) && (lhs.timeFirst == rhs.timeFirst) && (lhs.timeLast == rhs.timeLast) && (lhs.para == rhs.para) && (lhs.desc == rhs.desc) && (lhs.actionName == rhs.actionName) && (lhs.actionResult == rhs.actionResult) && (lhs.subFaultVec == rhs.subFaultVec);
    }
};
} // namespace fm
} // namespace mdc


#endif // MDC_FM_IMPL_TYPE_FMFAULTFULL_H
