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
#include "local_view.h"
#include "target_pose_decider/target_pose_decider.h"
#include "hmi_inner_c.h"

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
        << "A = " << A.transpose() << "  B = " << B.transpose()
        << "  C = " << C.transpose() << "  D = " << D.transpose()
        << "  E = " << E.transpose() << "  F = " << F.transpose()
        << "  G = " << G.transpose() << "  H = " << H.transpose()
        << "  min_x = " << min_x << "  min_x = " << min_y
        << "  max_x = " << max_x << "  max_x = " << max_y;
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

  double slot_occupied_ratio = 0.0;

  double slot_occupied_ratio_postprocess = 0.0;

  double channel_width = 0.0;

  geometry_lib::PathPoint origin_pose_global;
  geometry_lib::PathPoint origin_pose_local;

  geometry_lib::GlobalToLocalTf g2l_tf;
  geometry_lib::LocalToGlobalTf l2g_tf;

  // 根据障碍物移动车位 对于垂直车位 向左为正
  double move_slot_dist = 0.0;

  // 重规划成功时的移动距离 重规划成功时才更新
  double lat_move_dist_replan_success = 0.0;
  double lon_move_dist_replan_success = 0.0;

  // 每次重规划的移动距离 只要重规划就更新
  double lat_move_dist_every_replan = 0.0;
  double lon_move_dist_every_replan = 0.0;

  // 存在目标终点的安全buffer
  double safe_lat_buffer = 0.15;

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

  void Reset() {
    id = 0;
    slot_type = SlotType::INVALID;
    slot_side = geometry_lib::SLOT_SIDE_INVALID;
    cur_pose.Reset();
    target_pose.Reset();
    terminal_err.Reset();

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

    safe_lat_buffer = 0.15;

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
  }

  void GenerateReleaseSlotIdVec();

  const std::vector<size_t>& GetReleaseSlotIdVec() const {
    return release_slot_id_vec_;
  }

  const bool IsTargetSlotReleaseByRule() const;

  const EgoInfoUnderSlot& GetEgoInfoUnderSlot() const {
    return ego_info_under_slot_;
  }

  EgoInfoUnderSlot& GetMutableEgoInfoUnderSlot() {
    return ego_info_under_slot_;
  }

  const SlotReleaseState GetSlotReleaseState() const;

  const size_t GetEgoSlotInfoID() const;

 private:
  void ParkingLotCruiseProcess();

  const bool IsEgoCloseToObs(const double body_lat_buffer = 0.268,
                             const double mirror_lat_buffer = 0.268,
                             const double lon_buffer = 0.1);

  const bool IsSlotCoarseRelease(const ApaSlot& slot);

  const SlotReleaseVoterType IsPerpendicularSlotAndPassageAreaOccupied(
      const ApaSlot& slot);

  const SlotReleaseVoterType IsParallelSlotAndPassageAreaOccupied(
      const ApaSlot& slot);

 private:
  std::map<double, size_t> dist_id_map_;
  std::unordered_map<size_t, ApaSlot> slots_map_;
  std::unordered_map<size_t, uint8_t> slot_release_voter_;

  std::vector<size_t> release_slot_id_vec_;

  std::shared_ptr<ApaStateMachineManager> state_machine_ptr_;
  std::shared_ptr<ApaMeasureDataManager> measure_data_ptr_;
  std::shared_ptr<ApaObstacleManager> obstacle_manager_ptr_;
  std::shared_ptr<CollisionDetectorInterface> col_det_interface_ptr_;

  EgoInfoUnderSlot ego_info_under_slot_;

  // temp use
  bool is_ego_col_vertical_ = false;
  bool is_ego_col_parallel_ = false;
  bool free_slot_activate = false;
  iflyauto::FreeSlotSelectedStatus is_free_slot_selected;
};
}  // namespace apa_planner
}  // namespace planning