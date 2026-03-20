/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_EX_WDG_IMPL_TYPE_ANSWERENUM_H
#define MDC_EX_WDG_IMPL_TYPE_ANSWERENUM_H

#include "impl_type_uint8.h"
namespace mdc {
namespace ex_wdg {
enum class AnswerEnum : UInt8
{
    ANSWER_OPENING = 0,
    ANSWER_PANDA = 1,
    ANSWER_GLODEN_MONKEY = 2,
    ANSWER_SOUTH_CHINA_TIGER = 3,
    ANSWER_CRESTED_IBIS = 4,
    ANSWER_CHINESE_STURGEON = 5,
    ANSWER_CHINESE_RURAL_DOG = 6
};
} // namespace ex_wdg
} // namespace mdc


#endif // MDC_EX_WDG_IMPL_TYPE_ANSWERENUM_H
