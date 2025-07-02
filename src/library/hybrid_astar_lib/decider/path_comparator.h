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

// 路径比较器.
// 同样长度的path: 换档次数少的path更好;
// 同样档位数量的path: pose接近中心线heading更好，且启发点投影越近越好.
// todo: S弯更少的path，更好.
// todo: 障碍物距离更大的path，更好;
class PathComparator : public AstarDecider {
 public:
  PathComparator() = default;

  void SetHeuristicPose(const AstarRequest &request);

  // 比较node_challenger是不是更好
  bool Compare(const AstarRequest *request, const Node3d *best_node,
               const Node3d *node_challenger);

  void Process(const Pose2f &start, const Pose2f &end) override;

  const bool PolynomialPathBetter(const PolynomialPathCost &path,
                                  const PolynomialPathCost &base);

  // 比较准则：5厘米以外，距离越近越好；5厘米以内，heading越接近越好;
  const bool NodeCompare(const Pose2f &goal, const Node3d *best_node,
                         const Node3d *node_challenger);

 private:
  bool CheckVerticalSlotTailIn(const Node3d *best_node,
                               const Node3d *node_challenger);

  bool CheckVerticalSlotHeadIn(const Node3d *best_node,
                               const Node3d *node_challenger);

  const float GetHeuristicPointDistance(const Pose2f &node);

  const bool CheckHeuristicPointIsNice(const Pose2f &best_node,
                                       const Pose2f &node_challenger);

  const bool CheckDistanceRequest(const float &best_node_s,
                                  const float &node_challenger_s);
  bool CheckVerticalSlotHeadOut(const AstarRequest *request,
                                const Node3d *best_node,
                                const Node3d *node_challenger);

  PathWinReason win_reason_;

  /**
   *             .    o    .
   *             .         .
   *             .         .
   *             .         .
   *             .         .
   *             .         .
   *             ...........
   *
   * this pose is an heuristic pose in slot such as o point.
   */
  Pose2f heuristic_pose_;

  const AstarRequest *request_;
};

}  // namespace planning