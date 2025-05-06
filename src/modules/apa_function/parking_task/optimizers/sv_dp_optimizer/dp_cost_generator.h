#pragma once

#include "dp_speed_config.h"
#include "sv_graph_node.h"

namespace planning {
namespace apa_planner {
class DPCostGenerator {
 public:
  DPCostGenerator() = default;

  void Init(const double total_s, const DpSpeedConfig* config);

  void CalcStopoverCost(SVGraphNode* point);

  // 速度超过目标速度时，加速度越小越好；
  // 速度低于目标速度时: 加速度不合适，增加惩罚. 加速度合适，没有惩罚.
  // 加速度兼顾舒适性+行驶效率，一般0.5 m/s^2左右.
  double CalcSpeedUpCost(const double acc, const double node_v,
                         const double speed_limit);

  double CalcSpeedDownCost(const double acc);

  void CalcAccCost(SVGraphNode* point);

  void UpdateTotalCost(const SVGraphNode* parent, SVGraphNode* child);

  // TODO: 障碍物cost需要考虑到:静态障碍物、动态障碍物及其预测轨迹.
  // ego需要对各种类型障碍物使用合适的安全buffer，并且采取针对的驾驶模式.
  // 这是比较复杂的模块：涉及到障碍物意图、轨迹预测，自车和agent交互.
  // 等解决完现有的静态环境问题，以后升级版本时再考虑这些问题.
  void CalcObstacleCost() { return; }

  void CalcSpeedCost(const double speed_limit, SVGraphNode* point);

  // weight * jerk *jerk
  void CalcJerkCost(SVGraphNode* point);

 private:
  double total_s_ = 0.0;
  const DpSpeedConfig* config_;
};
}  // namespace apa_planner
}  // namespace planning