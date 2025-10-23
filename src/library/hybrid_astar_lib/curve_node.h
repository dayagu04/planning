#pragma once
#include <cstddef>
#include <vector>

#include "common_math.h"
#include "geometry_math.h"
#include "hybrid_astar_common.h"
#include "link_pose_line.h"
#include "link_pt_line.h"
#include "node3d.h"
#include "pose2d.h"
namespace planning {

#define USE_LINK_PT_LINE (1)

#define MAX_CURVE_PATH_SEG_NUM (7)

using namespace pnc;

struct CurvePath {
  int segment_size = 0;
  float path_dist = 0.0f;
  float kappa_change = 0.0f;
  float dists[MAX_CURVE_PATH_SEG_NUM];
  float kappas[MAX_CURVE_PATH_SEG_NUM];
  AstarPathGear gears[MAX_CURVE_PATH_SEG_NUM];
  AstarPathSteer steers[MAX_CURVE_PATH_SEG_NUM];
  size_t point_sizes[MAX_CURVE_PATH_SEG_NUM];
#if USE_LINK_PT_LINE
  std::vector<std::vector<common_math::PathPt<float>>> ptss;
#else
  std::vector<std::vector<pnc::geometry_lib::PathPoint>> ptss;
#endif

  int gear_change_number = 0;

  void Clear() {
    segment_size = 0;
    ptss.clear();
  }
};

class CurveNode : public Node3d {
 public:
  CurveNode() = default;
  ~CurveNode() = default;

  int Set(const CurvePath& curve_path, const MapBound& XYbounds,
          const PlannerOpenSpaceConfig& open_space_conf);

  int Set(const MapBound& XYbounds,
          const PlannerOpenSpaceConfig& open_space_conf);

  const CurvePath& GetCurvePath() const { return curve_path_; }

  CurvePath& GetMutableCurvePath() { return curve_path_; }

  void SetCurGear(const AstarPathGear& gear) { cur_gear_ = gear; }
  void SetCurKappa(const float kappa) { cur_kappa_ = kappa; }
  const float GetCurKappa() const { return cur_kappa_; }

  void SetCurGearLength(const float length) { cur_gear_length_ = length; }
  const AstarPathGear& GetCurGear() const { return cur_gear_; }
  const float GetCurGearLength() const { return cur_gear_length_; }

#if USE_LINK_PT_LINE
  void SetGearSwitchPose(const common_math::PathPt<float>& pose) {
    gear_switch_pose_ = pose;
  }

  const common_math::PathPt<float>& GetGearSwitchPose() const {
    return gear_switch_pose_;
  }

  void SetNextGearSwitchPose(const common_math::PathPt<float>& pose) {
    next_gear_switch_pose_ = pose;
  }

  const common_math::PathPt<float>& GetNextGearSwitchPose() const {
    return next_gear_switch_pose_;
  }

  void SetLPLPath(const link_pt_line::LinkPtLinePath<float>& lpl_path) {
    lpl_path_ = lpl_path;
  }

  const link_pt_line::LinkPtLinePath<float>& GetLPLPath() const {
    return lpl_path_;
  }

#else
  void SetGearSwitchPose(const geometry_lib::PathPoint& pose) {
    gear_switch_pose_ = pose;
  }

  const geometry_lib::PathPoint& GetGearSwitchPose() const {
    return gear_switch_pose_;
  }

  void SetNextGearSwitchPose(const geometry_lib::PathPoint& pose) {
    next_gear_switch_pose_ = pose;
  }

  const geometry_lib::PathPoint& GetNextGearSwitchPose() const {
    return next_gear_switch_pose_;
  }

  void SetLPLPath(const LinkPoseLinePath& lpl_path) { lpl_path_ = lpl_path; }

  const LinkPoseLinePath& GetLPLPath() const { return lpl_path_; }
#endif

  void Clear() {
    curve_path_.Clear();
    Node3d::Clear();
  }

 private:
  CurvePath curve_path_;
  AstarPathGear cur_gear_;
  float cur_gear_length_;
  float cur_kappa_;
#if USE_LINK_PT_LINE
  link_pt_line::LinkPtLinePath<float> lpl_path_;
  common_math::PathPt<float> gear_switch_pose_;
  common_math::PathPt<float> next_gear_switch_pose_;
#else
  LinkPoseLinePath lpl_path_;
  geometry_lib::PathPoint gear_switch_pose_;
  geometry_lib::PathPoint next_gear_switch_pose_;
#endif
};

}  // namespace planning