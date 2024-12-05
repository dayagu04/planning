#pragma once

#include <limits>
#include <vector>

#include "config/vehicle_param.h"
#include "dynamic_world/dynamic_world.h"
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "reference_path.h"
#include "session.h"
#include "tasks/task.h"
#include "trajectory1d/variable_coordinate_time_optimal_trajectory.h"

namespace planning {
struct LaneChangeVehInfo {
  int32_t id = -1;
  double center_s;
  double half_length;
  double v;
};

struct LaneChangeSlot {
  LaneChangeVehInfo front_info;
  LaneChangeVehInfo back_info;
  bool is_align_front;
  bool is_acc_to_dec;
  double aligned_s;
  double acc_t;
  double dec_t;
  double acc;
  double dec;
  double aligned_space;
  double aligned_v;
  double cost;
  bool is_valid = false;
  bool is_left_lane_change;
  double front_safety_threshold = 0.0;
  double back_safe_threshold = 0.0;
};

struct SpeedAdjustStatusBuffer {
  bool current_frame_status = false;
  bool last_frame_status = false;
  bool last_two_frame_status = false;
};

class SlotInfo {
 public:
  SlotInfo() = default;
  SlotInfo(const LaneChangeVehInfo& front_veh_info,
           const LaneChangeVehInfo& back_veh_info)
      : front_veh_info_(front_veh_info), back_veh_info_(back_veh_info) {}
  SlotInfo(const LaneChangeVehInfo& front_veh_info,
           const LaneChangeVehInfo& back_veh_info, const double slot_v)
      : front_veh_info_(front_veh_info),
        back_veh_info_(back_veh_info),
        slot_v_(slot_v) {}

  ~SlotInfo() = default;

  void SetFrontVehInfo(const LaneChangeVehInfo& front_veh_info) {
    front_veh_info_ = front_veh_info;
  };
  void SetBackVehInfo(const LaneChangeVehInfo& back_veh_info) {
    back_veh_info_ = back_veh_info;
  };
  void SetAlignedS(const double aligned_s) { aligned_s_ = aligned_s; }
  void SetAlignedV(const double aligned_v) { aligned_v_ = aligned_v; }
  void SetAlignedFront(bool is_aligned_front) {
    is_align_front_ = is_aligned_front;
  }
  void SetSlotV(const double slot_v) { slot_v_ = slot_v; };
  void SetSlotCost(const double cost) { cost_ = cost; };

  const LaneChangeVehInfo& front_veh_info() const { return front_veh_info_; }
  const LaneChangeVehInfo& back_veh_info() const { return back_veh_info_; }
  const double& aligned_s() const { return aligned_s_; }
  const double& aligned_v() const { return aligned_v_; }
  const bool& is_align_front() const { return is_align_front_; }
  const double& slot_v() const { return slot_v_; }
  const double& cost() const { return cost_; };

 private:
  LaneChangeVehInfo front_veh_info_;
  LaneChangeVehInfo back_veh_info_;
  bool is_align_front_ = false;
  double aligned_s_;
  double aligned_v_;
  std::vector<double> aligned_s_vec_;
  double slot_v_;
  double cost_;
};
class SpeedAdjustDecider : public Task {
 public:
  SpeedAdjustDecider(const EgoPlanningConfigBuilder* config_builder,
                     framework::Session* session);
  virtual ~SpeedAdjustDecider() = default;

  bool Execute() override;

 private:
  bool ProcessLaneChangeStatus();
  void ClearStatus();
  void ProcessEnvInfos();
  std::pair<double, double> GetSafeAlignedDistance(const double& ego_v,
                                                   const SlotInfo& slot);
  bool GenerateCandidateSlotInfo();
  void GenerateTimeOptimalAdjustProfile();
  int SelectBestSlot();
  void GenerateAdjustTraj(int best_id, std::vector<double>* search_path);
  void CalcTargetObjsFlowVel();

  bool SlotValidCheck();

 private:
  SpeedAdjustDeciderConfig config_;
  int count_wait_state_{0};
  int last_request_ = 0;
  int last_best_slot_id_ = -1;
  LaneChangeVehInfo leading_veh_;
  std::vector<VariableCoordinateTimeOptimalTrajectory>
      variable_time_optimal_trajs_;
  SlotInfo last_best_slot_;
  std::pair<double, double> init_sl_;
  std::pair<double, double> init_va_;
  double v_cruise_{25.0};
  double target_lane_objs_flow_vel_{25.0};
  bool slot_changed_{false};

  SpeedAdjustStatusBuffer speed_adjust_status_buffer_{false, false, false};
  double retriggered_ego_speed_{25.0};
  double max_ego_speed_in_speed_adjust_{25.0};
  double min_ego_speed_in_speed_adjust_{25.0};
  double origin_ego_speed_{25.0};
  std::vector<SlotInfo> slot_point_info_;
  std::vector<LaneChangeVehInfo> lane_change_veh_info_;
  std::unordered_map<int32_t, LaneChangeVehInfo> lane_change_veh_info_id_map_;
  VehicleParam vehicle_param_;
  const std::vector<double> t_gap_ego_v_bp_{5.0, 15.0, 30.0};
  const std::vector<double> t_gap_ego_v_{1.35, 1.55, 2.0};
  std::vector<double> max_acc_ego_v_bp_{};
  std::vector<double> max_jerk_ego_v_bp_{};
  std::vector<double> max_v_max_ego_v_bp_{};
  std::vector<double> min_acc_ego_v_bp_{};
  std::vector<double> min_jerk_ego_v_bp_{};
  const std::vector<double> max_acc_ego_v_{15.0, 36.0};
  const std::vector<double> min_acc_ego_v_{15.0, 36.0};
  const std::vector<double> max_jerk_ego_v_{15.0, 36.0};
  const std::vector<double> min_jerk_ego_v_{15.0, 36.0};
  const std::vector<double> max_v_max_ego_v_{15.0, 36.0};
  std::unordered_set<int32_t> front_target_lane_id_set_;
  std::unordered_set<int32_t> rear_target_lane_id_set_;
  bool boundary_merge_point_valid_ = false;
  bool deceleration_priority_scene_ = false;
  double merge_emegency_distance_ = 1000.0;
};

}  // namespace planning