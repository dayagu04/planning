#pragma  once

#include <vector>

#include "ifly_parking_map_c.h"
#include "session.h"
#include "vec2d.h"

namespace planning {
using ParkingSlotPoints = std::vector<planning_math::Vec2d>;

// decider 和 manager有什么区别?
// decider通过决策算法（基于规则的、基于搜索的、基于机器学习的）生成一个选择结果.
// 比如动态规划搜索生成速度决策.
// manager是对客观信息的处理, 比如记录历史车位信息,
// 记录历史障碍物信息,记录当前车位信息,处理车位角点信息,等等.
class ParkingSlotManager {
 public:
  ParkingSlotManager(planning::framework::Session *session);
  ~ParkingSlotManager() = default;

 public:
  bool update(const iflyauto::ParkingInfo &parking_info);
  std::vector<ParkingSlotPoints> get_points() { return points_; };

 private:
  planning::framework::Session *session_;
  std::vector<ParkingSlotPoints> points_;
};
}  // namespace planning