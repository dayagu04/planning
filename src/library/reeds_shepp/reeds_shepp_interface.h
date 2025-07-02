#pragma once

#include "./../hybrid_astar_lib/hybrid_astar_common.h"
#include "pose2d.h"
#include "reeds_shepp.h"
#include "rs_path_interpolate.h"
#include "rs_path_request.h"

namespace planning {

class RSPathInterface {
 public:
  RSPathInterface() = default;

  /**
   * need_interpolate: if you just want to get gear info or length info or kappa
   * info, shut down interpolation action. interpolation action will occupy too
   * much time.
   */
  void GeneShortestRSPath(RSPath *rs_path, bool *is_connected_to_goal,
                          const Pose2D *start, const Pose2D *end,
                          const double min_radius, const bool need_interpolate,
                          const bool need_anchor_point,
                          const RSPathRequestType request_type,
                          const double rs_path_sample_dist = 0.1);

  void RSPathInterpolate(RSPath *rs_path, const Pose2D *start,
                         const double min_radius);

  // just test scs path
  void GeneSCSPath(RSPath *rs_path, bool *is_connected_to_goal,
                   const Pose2D *start, const Pose2D *end,
                   const double min_radius,
                   const RSPathRequestType request_type);

  const RSAnchorPoints &GetAnchorPoint() { return rs_anchor_points_; }

 private:
  void SetAnchorPoint(RSAnchorPoints &rs_path_anchor_pts) {
    rs_anchor_points_ = rs_path_anchor_pts;
  }

  void UpdateRSPathByAnchorPoint(RSPath *rs_path, const double min_radius);

 private:
  // RSPath rs_path_;

  // rs path interpolator
  RSPathInterpolator rs_interpolate_;
  RSAnchorPoints rs_anchor_points_;
};

}  // namespace planning
