#pragma once

namespace planning {
namespace apa_planner {
// 所有异常相关，都需要定义在这里
enum class DpSpeedErrorCode {
  NONE = 0,
  FAIL_BY_VEH_HIGH_SPEED = 1,
};
}  // namespace apa_planner
}  // namespace planning