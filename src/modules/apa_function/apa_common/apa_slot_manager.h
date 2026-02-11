#pragma once
#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"
#include "apa_measure_data_manager.h"
#include "apa_obstacle_manager.h"
#include "apa_slot.h"
#include "apa_state_machine_manager.h"
#include "collision_detection/collision_detector_interface.h"
#include "geometry_math.h"
#include "hmi_inner_c.h"
#include "local_view.h"
#include "target_pose_decider/target_pose_decider.h"

namespace planning {
namespace apa_planner {

struct TLane {
  Eigen::Vector2d A = Eigen::Vector2d::Zero();
  Eigen::Vector2d B = Eigen::Vector2d::Zero();
  Eigen::Vector2d C = Eigen::Vector2d::Zero();
  Eigen::Vector2d D = Eigen::Vector2d::Zero();
  Eigen::Vector2d E = Eigen::Vector2d::Zero();
  Eigen::Vector2d F = Eigen::Vector2d::Zero();
  Eigen::Vector2d G = Eigen::Vector2d::Zero();
  Eigen::Vector2d H = Eigen::Vector2d::Zero();

  double channel_width = 0.0;
  double channel_length = 0.0;

  double min_x = 0.0;
  double min_y = 0.0;
  double max_x = 0.0;
  double max_y = 0.0;

  void Reset() {
    A.setZero();
    B.setZero();
    C.setZero();
    D.setZero();
    E.setZero();
    F.setZero();
    G.setZero();
    H.setZero();

    channel_width = 0.0;
    channel_length = 0.0;

    min_x = 0.0;
    min_y = 0.0;
    max_x = 0.0;
    max_y = 0.0;
  }

  void CalcBound() {
    min_x = std::min({A.x(), B.x(), C.x(), D.x(), E.x(), F.x(), G.x(), H.x()});
    min_y = std::min({A.y(), B.y(), C.y(), D.y(), E.y(), F.y(), G.y(), H.y()});
    max_x = std::max({A.x(), B.x(), C.x(), D.x(), E.x(), F.x(), G.x(), H.x()});
    max_y = std::max({A.y(), B.y(), C.y(), D.y(), E.y(), F.y(), G.y(), H.y()});
  }

  void PrintInfo(const bool enable_print = true) const {
    ILOG_INFO_IF(enable_print)
        << "A = " << A.x() << " " << A.y() << "  B = " << B.x() << " " << B.y()
        << "  C = " << C.x() << " " << C.y() << "  D = " << D.x() << " "
        << D.y() << "  E = " << E.x() << " " << E.y() << "  F = " << F.x()
        << " " << F.y() << "  G = " << G.x() << " " << G.y()
        << "  H = " << H.x() << " " << H.y() << "  min_x = " << min_x
        << "  min_x = " << min_y << "  max_x = " << max_x
        << "  max_x = " << max_y;
  }
};

struct EgoInfoUnderSlot {
  size_t id = 0;
  SlotType slot_type = SlotType::INVALID;
  geometry_lib::SlotSide slot_side = geometry_lib::SLOT_SIDE_INVALID;
  uint32 confidence = 0;

  // slot coordinates
  geometry_lib::PathPoint cur_pose;
  geometry_lib::PathPoint origin_target_pose;
  geometry_lib::PathPoint target_pose;
  geometry_lib::PathPoint terminal_err;

  // base on global
  geometry_lib::PathPoint replan_success_origin_target_pose;

  geometry_lib::LineSegment tar_line;

  double slot_occupied_ratio = 0.0;

  double slot_occupied_ratio_postprocess = 0.0;

  double channel_width = 0.0;

  geometry_lib::PathPoint origin_pose_global;
  geometry_lib::PathPoint origin_pose_local;

  geometry_lib::GlobalToLocalTf g2l_tf;
  geometry_lib::LocalToGlobalTf l2g_tf;

  double move_slot_dist = 0.0;

  double lat_move_dist_replan_success = 0.0;
  double lon_move_dist_replan_success = 0.0;

  double lat_move_dist_every_replan = 0.0;
  double lon_move_dist_every_replan = 0.0;

  double safe_lat_body_buffer = 0.15;
  double safe_lat_mirror_buffer = 0.15;

  TargetPoseDeciderResult tar_pose_result;

  Eigen::Vector2d pt_inside = Eigen::Vector2d::Zero();

  ApaSlot slot;

  bool fix_slot = false;

  bool fix_limiter = false;

  std::pair<Eigen::Vector2d, Eigen::Vector2d> virtual_limiter;

  TLane obs_tlane;

  size_t history_id;
  SlotType history_slot_type = SlotType::INVALID;

  bool slot_disappear_flag = false;

  double relative_direction_between_ego_and_slot;

  double neigbor_front_heading = -100.0;
  double neigbor_rear_heading = -100.0;

  void Reset() {
    id = 0;
    slot_type = SlotType::INVALID;
    slot_side = geometry_lib::SLOT_SIDE_INVALID;
    cur_pose.Reset();
    target_pose.Reset();
    terminal_err.Reset();

    origin_target_pose.Reset();

    replan_success_origin_target_pose.Reset();

    confidence = 0;

    slot_occupied_ratio_postprocess = 0.0;
    slot_occupied_ratio = 0.0;
    channel_width = 0.0;

    origin_pose_global.Reset();
    origin_pose_local.Reset();

    g2l_tf.Reset();
    l2g_tf.Reset();

    move_slot_dist = 0.0;

    lat_move_dist_replan_success = 0.0;
    lon_move_dist_replan_success = 0.0;

    lat_move_dist_every_replan = 0.0;
    lon_move_dist_every_replan = 0.0;

    safe_lat_body_buffer = 0.15;
    safe_lat_mirror_buffer = 0.15;

    pt_inside.setZero();

    slot.Reset();

    fix_slot = false;

    fix_limiter = false;

    obs_tlane.Reset();

    history_id = 0;
    history_slot_type = SlotType::INVALID;

    slot_disappear_flag = false;

    relative_direction_between_ego_and_slot = 1.0;
  }
};

enum class SlotReleaseVoterType : uint8_t {
  ACCUMULATE,
  SUBTRACT,
  CLEAR,
  MAXIMUM,
  HOLD,
};

const std::string GetSlotReleaseVoterTypeString(
    const SlotReleaseVoterType release_voter_type);

class ApaSlotManager final {
 public:
  ApaSlotManager() {}
  ~ApaSlotManager() {}

  void Update(
      const LocalView* local_view,
      const iflyauto::PlanningOutput* planning_output,
      const std::shared_ptr<ApaStateMachineManager>& state_machine_ptr,
      const std::shared_ptr<ApaMeasureDataManager>& measure_data_ptr,
      const std::shared_ptr<ApaObstacleManager>& obstacle_manager_ptr,
      const std::shared_ptr<CollisionDetectorInterface>& col_det_interface_ptr);

  void Reset() {
    ego_info_under_slot_.Reset();
    dist_id_map_.clear();
    slots_map_.clear();
    slot_release_voter_.clear();
    release_slot_id_vec_.clear();
    ego_slot_min_dist_map_.clear();
    parallel_slot_release_count_map_.clear();
    parallel_slot_not_release_count_map_.clear();
    pre_plan_fail_slot_id_uset_.clear();
  }

  void GenerateReleaseSlotIdVec();

  const std::vector<size_t>& GetReleaseSlotIdVec() const {
    return release_slot_id_vec_;
  }

  const std::vector<bool>& GetReleaseSlotNarrowFlagVec() const {
    return release_slot_narrow_flag_vec_;
  }

  const bool IsTargetSlotReleaseByRule() const;

  const EgoInfoUnderSlot& GetEgoInfoUnderSlot() const {
    return ego_info_under_slot_;
  }

  EgoInfoUnderSlot& GetMutableEgoInfoUnderSlot() {
    return ego_info_under_slot_;
  }

  const SlotReleaseState GetSlotReleaseState() const;
  const SlotReleaseState GetSlotReleaseStateFreeSlot() const;

  const size_t GetEgoSlotInfoID() const;

  const std::unordered_map<size_t, ApaSlot>& GetSlotsMap() const {
    return slots_map_;
  }
  const bool GetRecommendParkOut() const { return recommend_park_out_; }

 private:
  void ParkingLotCruiseProcess();

  const bool IsEgoCloseToObs(const double body_lat_buffer = 0.268,
                             const double mirror_lat_buffer = 0.268,
                             const double lon_buffer = 0.1);

  const bool IsSlotCoarseRelease(ApaSlot& slot);

  const SlotReleaseVoterType IsPerpendicularSlotAndPassageAreaOccupied(
      ApaSlot& slot);

  const SlotReleaseVoterType IsParallelSlotAndPassageAreaOccupied(
      ApaSlot& slot);

  const bool RecommendParkOut() const;
  const bool LongitudinalConditions(const double dot_product, const ApaSlot& slot) const;
  const bool LateralConditions(double& dot_product, const ApaSlot& slot) const;
  bool IsSideParkingPerpendicularSlot(ApaSlot slot);

 private:
  std::map<double, size_t> dist_id_map_;
  std::unordered_map<size_t, ApaSlot> slots_map_;
  std::unordered_map<size_t, uint8_t> slot_release_voter_;

  std::unordered_map<size_t, double> ego_slot_min_dist_map_;

  std::unordered_set<size_t> pre_plan_fail_slot_id_uset_;

  std::vector<size_t> release_slot_id_vec_;
  std::vector<bool> release_slot_narrow_flag_vec_;

  std::shared_ptr<ApaStateMachineManager> state_machine_ptr_;
  std::shared_ptr<ApaMeasureDataManager> measure_data_ptr_;
  std::shared_ptr<ApaObstacleManager> obstacle_manager_ptr_;
  std::shared_ptr<CollisionDetectorInterface> col_det_interface_ptr_;

  EgoInfoUnderSlot ego_info_under_slot_;

  // temp use
  bool is_ego_col_vertical_ = false;
  double ego_col_safe_lat_buffer_ = 0.268;
  double ego_col_safe_lon_buffer_ = 0.15;
  bool is_ego_col_parallel_ = false;
  std::unordered_map<size_t, int> parallel_slot_release_count_map_;
  std::unordered_map<size_t, int> parallel_slot_not_release_count_map_;
  std::unordered_map<size_t, std::pair<bool,int>> perpendicular_redefine_info_map_;
  bool recommend_park_out_ = false;
};
}  // namespace apa_planner
}  // namespace planning