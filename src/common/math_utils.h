/**
 * @file
 * @brief Math-related util functions.
 */

#pragma once

#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>

#include <Eigen/Core>

namespace planning {

#define ifly_abs(x) std::abs(x)

#define ifly_pow(x, y) std::pow(x, y)
#define ifly_pow2(x) ((x) * (x))
#define ifly_sqrt(x) std::sqrt(x)

#define ifly_max(x, y) ((x) > (y) ? (x) : (y))
#define ifly_min(x, y) ((x) < (y) ? (x) : (y))

#define ifly_sin(x) std::sin(x)
#define ifly_cos(x) std::cos(x)
#define ifly_tan(x) std::tan(x)
#define ifly_asin(x) std::asin(x)
#define ifly_atan(x) std::atan(x)
#define ifly_atan2(y, x) std::atan2(y, x)

#define ifly_floor(x) std::floor(x)
#define ifly_ceil(x) std::ceil(x)
#define ifly_round(x) std::round(x)
#define ifly_fabs(x) std::fabs(x)
#define ifly_fmod(x, y) std::fmod(x, y)

#define ifly_float64_epsilon (1e-9)

#define ifly_deg2rad(x) ((x)*M_PI / 180.0)
#define ifly_rad2deg(x) ((x)*180.0 / M_PI)

#define ifly_cvt_kmh_to_ms(vel) (((vel)*1000) / 3600)
#define ifly_cvt_ms_to_kmh(vel) (((vel)*3600) / 1000)

#define ifly_fequal(x, y) (ifly_fabs((x) - (y)) < ifly_float64_epsilon)
#define ifly_fgreater(x, y) (((x) > (y)) && !ifly_fequal(x, y))
#define ifly_fless(x, y) (((x) < (y)) && !ifly_fequal(x, y))

#define ifly_sign(x) \
  (ifly_fgreater(x, 0.0) ? 1 : (ifly_fless(x, 0.0) ? -1 : 0))

#define IFLY_SQRT_PI         (1.77245385090551602729)
#define IFLY_SQRT_PI_INV     (0.56418958354775628695)

}  // namespace planning
