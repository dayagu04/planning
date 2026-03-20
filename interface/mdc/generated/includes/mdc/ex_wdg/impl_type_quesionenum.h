/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_EX_WDG_IMPL_TYPE_QUESIONENUM_H
#define MDC_EX_WDG_IMPL_TYPE_QUESIONENUM_H

#include "impl_type_uint8.h"
namespace mdc {
namespace ex_wdg {
enum class QuesionEnum : UInt8
{
    QUESION_OPENING = 0,
    QUESION_PANDA = 1,
    QUESION_GLODEN_MONKEY = 2,
    QUESION_SOUTH_CHINA_TIGER = 3,
    QUESION_CRESTED_IBIS = 4,
    QUESION_CHINESE_STURGEON = 5,
    QUESION_CHINESE_RURAL_DOG = 6
};
} // namespace ex_wdg
} // namespace mdc


#endif // MDC_EX_WDG_IMPL_TYPE_QUESIONENUM_H
