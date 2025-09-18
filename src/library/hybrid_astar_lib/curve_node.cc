#include "curve_node.h"

#include "common_math.h"
#include "log_glog.h"

namespace planning {

int CurveNode::Set(const CurvePath& curve_path, const MapBound& XYbounds,
                   const PlannerOpenSpaceConfig& open_space_conf) {
  curve_path_ = curve_path;

  return Set(XYbounds, open_space_conf);
}

int CurveNode::Set(const MapBound& XYbounds,
                   const PlannerOpenSpaceConfig& open_space_conf) {
  if (curve_path_.segment_size < 1 || curve_path_.ptss.empty() ||
      curve_path_.ptss[0].empty()) {
    return 0;
  }

  const auto& pose = curve_path_.ptss[0][0];

  // XYbounds in xmin, xmax, ymin, ymax
  grid_index_.x = std::round((pose.pos.x() - XYbounds.x_min) *
                             open_space_conf.xy_grid_resolution_inv);

  grid_index_.y = std::round((pose.pos.y() - XYbounds.y_min) *
                             open_space_conf.xy_grid_resolution_inv);

  grid_index_.phi =
      std::round((common_math::UnifyAngle(pose.GetTheta()) - XYbounds.phi_min) *
                 open_space_conf.phi_grid_resolution_inv);

  visited_type_ = AstarNodeVisitedType::NOT_VISITED;

  ResetCost();

  // is_start_node_ = false;
  // dist_to_obs_ = 26.8f;

  if (NodeIndexValid(grid_index_)) {
    global_id_ = IDTransform(grid_index_);
  } else {
    ILOG_INFO << "invalid node id";
    pose.PrintInfo();
    grid_index_.PrintInfo();
    // invalid node
    path_.Clear();
  }

  collision_type_ = NodeCollisionType::NONE;
  gear_switch_num_ = 0;
  gear_switch_node_ = nullptr;
#if DEBUG_NODE3D
  ILOG_INFO << "new index " << index_;
#endif

  return 0;
}

}  // namespace planning