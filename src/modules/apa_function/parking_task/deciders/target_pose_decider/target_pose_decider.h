#pragma once
#include <vector>

#include "apa_slot.h"
#include "parking_task.h"

namespace planning {
namespace apa_planner {
using namespace pnc;

struct TargetPoseDeciderRequest {
  std::vector<double> lat_body_buffer_vec;
  std::vector<double> lat_mirror_buffer_vec;
  double lon_buffer = 0.0;
  ParkingScenarioType scenario_type;
  bool consider_obs = true;
  bool base_on_slot = false;
  ApaSlotLatPosPreference slot_lat_pos_preference =
      ApaSlotLatPosPreference::MID;
  bool is_searching_stage = false;
  bool ego_in_believe_slot_area = false;

  TargetPoseDeciderRequest() = default;
  TargetPoseDeciderRequest(
      const std::vector<double>& _lat_body_buffer_vec,
      const std::vector<double>& _lat_mirror_buffer_vec,
      const double _lon_buffer, const ParkingScenarioType _scenario_type,
      const bool _consider_obs = true, const bool _base_on_slot = false,
      const ApaSlotLatPosPreference _slot_lat_pos_preference =
          ApaSlotLatPosPreference::MID,
      const bool _is_searching_stage = false,
      const bool _ego_in_believe_slot_area = false)
      : lat_body_buffer_vec(_lat_body_buffer_vec),
        lat_mirror_buffer_vec(_lat_mirror_buffer_vec),
        lon_buffer(_lon_buffer),
        scenario_type(_scenario_type),
        consider_obs(_consider_obs),
        base_on_slot(_base_on_slot),
        slot_lat_pos_preference(_slot_lat_pos_preference),
        is_searching_stage(_is_searching_stage),
        ego_in_believe_slot_area(_ego_in_believe_slot_area) {}

  TargetPoseDeciderRequest(
      const ParkingScenarioType _scenario_type,
      const ApaSlotLatPosPreference _slot_lat_pos_preference =
          ApaSlotLatPosPreference::MID) {
    lat_body_buffer_vec = {0.0};
    lat_mirror_buffer_vec = {0.0};
    lon_buffer = 0.0;
    scenario_type = _scenario_type;
    consider_obs = false;
    base_on_slot = true;
    slot_lat_pos_preference = _slot_lat_pos_preference;
    is_searching_stage = false;
    ego_in_believe_slot_area = false;
  }

  ~TargetPoseDeciderRequest() = default;
};
enum class TargetPoseType {
  FAIL,
  NORMAL,
  FOLD_MIRROR,
};

struct TargetPoseDeciderResult {
  TargetPoseType target_pose_type = TargetPoseType::FAIL;
  geometry_lib::PathPoint target_pose_local;
  geometry_lib::PathPoint target_pose_global;
  double safe_lat_move_dist = 0.0;
  double safe_lon_move_dist = 0.0;
  double safe_lat_body_buffer = 0.0;
  double safe_lat_mirror_buffer = 0.0;
  double exceed_allow_max_dx = 0.0;

  void Reset() {
    target_pose_type = TargetPoseType::FAIL;
    target_pose_local.Reset();
    target_pose_global.Reset();
    safe_lat_move_dist = 0.0;
    safe_lon_move_dist = 0.0;
    safe_lat_body_buffer = 0.0;
    safe_lat_mirror_buffer = 0.0;
    exceed_allow_max_dx = 0.0;
  }
};

class TargetPoseDecider final : public ParkingTask {
 public:
  TargetPoseDecider(
      const std::shared_ptr<apa_planner::CollisionDetectorInterface>&
          col_det_interface_ptr)
      : col_det_interface_ptr_(col_det_interface_ptr) {}
  ~TargetPoseDecider() = default;

  const TargetPoseDeciderResult CalcTargetPose(
      const ApaSlot& slot, const TargetPoseDeciderRequest& request);

 private:
  const TargetPoseDeciderResult CalcTargetPoseForPerpendicularParkingIn();

  const TargetPoseDeciderResult CalcTargetPoseForPerpendicularTailOut();

  const TargetPoseDeciderResult CalcTargetPoseForPerpendicularHeadOut();

  const TargetPoseDeciderResult CalcTargetPoseForParallelTailIn();

  const TargetPoseDeciderResult CalcTargetPoseForParallelHeadingOut();

 private:
  std::shared_ptr<apa_planner::CollisionDetectorInterface>
      col_det_interface_ptr_;
  ApaSlot slot_;
  std::vector<double> lat_body_buffer_vec_;
  std::vector<double> lat_mirror_buffer_vec_;
  ApaSlotLatPosPreference slot_lat_pos_preference_ =
      ApaSlotLatPosPreference::MID;
  double lon_buffer_ = 0.0;
  bool consider_obs_ = false;
  bool base_on_slot_ = false;
  bool is_searching_stage_ = false;
  ParkingScenarioType scenario_type_;
  bool ego_in_believe_slot_area_ = false;
  TargetPoseDeciderResult result_;
};
}  // namespace apa_planner
}  // namespace planning
