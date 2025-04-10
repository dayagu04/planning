#pragma once

#include "astar_decider.h"
#include "hybrid_astar_request.h"
#include "node3d.h"
#include "pose2d.h"

namespace planning {

enum PathWinReason {
  GEAR_SWITCH,
  CLOSE_TO_SLOT,
  SAFE,
  KAPPA,
};

struct PolynomialPathCost {
  float offset_to_center;
  float accumulated_s;
  float tail_heading;

  size_t point_size;

  void Clear() {
    offset_to_center = 100.0;
    tail_heading = 100.0;
    point_size = 0;
    return;
  }
};

// 路径比较器.
// 同样长度的path: 换档次数少的path更好;
// 同样档位数量的path: 车头接近中心线heading更好;
// todo: S弯更少的path，更好.
// todo: 障碍物距离更大的path，更好;
class PathComparator : public AstarDecider {
 public:
  PathComparator() = default;

  // 比较node_challenger是不是更好
  bool Compare(const AstarRequest *request, const Node3d *best_node,
               const Node3d *node_challenger);

  void Process(const Pose2D &start, const Pose2D &end) override;

  const bool PolynomialPathBetter(const PolynomialPathCost& path,
                                const PolynomialPathCost& base);

  // 比较准则：5厘米以外，距离越近越好；5厘米以内，heading越接近越好;
  const bool NodeCompare(const Pose2D &goal, const Node3d *best_node,
                         const Node3d *node_challenger);

 private:
  bool CheckVerticalSlotTailIn(const AstarRequest *request,
                               const Node3d *best_node,
                               const Node3d *node_challenger);

  bool CheckVerticalSlotHeadIn(const AstarRequest *request,
                               const Node3d *best_node,
                               const Node3d *node_challenger);

  PathWinReason win_reason_;
};

}  // namespace planning