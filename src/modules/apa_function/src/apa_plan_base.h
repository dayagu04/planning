#ifndef __APA_PLAN_BASE_H__
#define __APA_PLAN_BASE_H__

#include <bits/stdint-uintn.h>
#include <sys/types.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "apa_param_setting.h"
#include "apa_world.h"
#include "collision_detection.h"
#include "geometry_math.h"
#include "lateral_path_optimizer.h"
#include "local_view.h"
#include "planning_plan_c.h"

namespace planning {
namespace apa_planner {
class ApaPlannerBase {
 public:
  enum ReplanReason {
    NOT_REPLAN,
    FIRST_PLAN,
    SEG_COMPLETED_PATH,
    SEG_COMPLETED_USS,
    STUCKED,
    DYNAMIC,
  };
  struct SimulationParam {
    bool is_complete_path = false;
    bool force_plan = false;
    bool sim_to_target = false;
    bool use_slot_in_bag = true;
    bool is_path_optimization = false;
    bool is_cilqr_optimization = false;
    bool is_reset = false;
    double sample_ds = 0.02;
    std::vector<double> target_managed_slot_x_vec;
    std::vector<double> target_managed_slot_y_vec;
    std::vector<double> target_managed_limiter_x_vec;
    std::vector<double> target_managed_limiter_y_vec;

    double q_ref_xy = 100.0;
    double q_ref_theta = 100.0;
    double q_terminal_xy = 1000.0;
    double q_terminal_theta = 9000.0;
    double q_k = 10.0;
    double q_u = 10.0;
    double q_k_bound = 100.0;
    double q_u_bound = 50.0;
  };

  struct EgoSlotInfo {
    common::SlotInfo target_managed_slot;
    std::pair<Eigen::Vector2d, Eigen::Vector2d> limiter;
    Eigen::Vector2d target_ego_pos_slot = Eigen::Vector2d::Zero();
    double target_ego_heading_slot = 0.0;

    std::vector<Eigen::Vector2d> slot_corner;
    std::vector<Eigen::Vector2d> limiter_corner;

    Eigen::Vector2d slot_center;

    size_t selected_slot_id = 0;
    size_t slot_type = 0;

    Eigen::Vector2d slot_origin_pos = Eigen::Vector2d::Zero();
    double slot_origin_heading = 0.0;
    Eigen::Vector2d slot_origin_heading_vec = Eigen::Vector2d::Zero();

    Eigen::Vector2d ego_pos_slot = Eigen::Vector2d::Zero();
    Eigen::Vector2d ego_heading_slot_vec = Eigen::Vector2d::Zero();
    double ego_heading_slot = 0.0;

    double slot_width = 3.0;
    double slot_length = 5.2;

    pnc::geometry_lib::PathPoint terminal_err;
    double slot_occupied_ratio = 0.0;

    pnc::geometry_lib::GlobalToLocalTf g2l_tf;
    pnc::geometry_lib::LocalToGlobalTf l2g_tf;

    bool first_fix_limiter = true;

    std::vector<Eigen::Vector2d> obs_pt_vec_slot;

    double origin_pt_0_heading = 0.0;
    double sin_angle = 1.0;
    Eigen::Vector2d pt_0;
    Eigen::Vector2d pt_1;

    double channel_width;

    void Reset() {
      target_managed_slot.Clear();
      selected_slot_id = 0;
      slot_type = 0;
      target_ego_pos_slot = Eigen::Vector2d::Zero();
      target_ego_heading_slot = 0.0;

      slot_corner.clear();
      limiter_corner.clear();

      slot_center.setZero();

      limiter.first.setZero();
      limiter.second.setZero();

      slot_origin_pos.setZero();

      slot_origin_heading = 0.0;
      slot_origin_heading_vec.setZero();

      ego_heading_slot_vec.setZero();
      ego_heading_slot = 0.0;

      ego_pos_slot.setZero();

      slot_width = 3.0;
      slot_length = 4.8;

      terminal_err.Set(Eigen::Vector2d(1.0, 1.0), 0.5);
      slot_occupied_ratio = 0.0;

      first_fix_limiter = true;

      obs_pt_vec_slot.clear();

      origin_pt_0_heading = 0.0;
      sin_angle = 1.0;
      pt_0.setZero();
      pt_1.setZero();

      channel_width = apa_param.GetParam().channel_width;
    }
  };

  struct PlannerStateMachine {  // planner states
    bool path_plan_success = false;
    uint8_t planning_status = ParkingStatus::PARKING_IDLE;

    void Reset() {
      path_plan_success = false;
      planning_status = ParkingStatus::PARKING_IDLE;
    }
  };

  struct Frame {
    void Reset() {
      gear_change_count = 0;
      is_replan = false;
      is_replan_first = true;
      is_replan_second = false;
      is_replan_dynamic = false;
      is_dynamic_replan_first = true;
      dynamic_replan_count = 0;
      total_plan_count = 0;
      in_slot_plan_count = 0;
      is_finished = false;
      is_fix_slot = false;
      stuck_time = 0.0;
      stuck_uss_time = 0.0;
      pause_time = 0.0;
      remain_dist = 5.01;
      remain_dist_uss = 5.01;
      spline_success = false;
      current_path_length = 0.0;
      path_extended_dist = 1.0;
      is_replan_by_uss = false;
      ego_slot_info.Reset();
      plan_stm.Reset();
      pathplan_result = 0;
      current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
      current_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;
      replan_reason = NOT_REPLAN;
      plan_fail_reason = NOT_FAILED;
      need_update_slot = true;
      correct_path_for_limiter = false;
      replan_flag = false;
    }

    bool is_replan = false;
    bool is_replan_first = true;
    bool is_replan_second = false;
    bool is_replan_dynamic = false;
    bool is_dynamic_replan_first = true;
    uint8_t dynamic_replan_count = 0;
    uint8_t replan_reason = NOT_REPLAN;
    uint8_t plan_fail_reason = NOT_FAILED;
    uint8_t total_plan_count = 0;
    uint8_t in_slot_plan_count = 0;
    bool is_finished = false;
    bool is_fix_slot = false;
    bool spline_success = false;
    bool is_replan_by_uss = false;
    uint8_t pathplan_result = 0;
    size_t gear_change_count = 0;
    double current_path_length = 0.0;
    double path_extended_dist = 1.0;
    double stuck_time = 0.0;
    double stuck_uss_time = 0.0;
    double pause_time = 0.0;
    double remain_dist = 5.01;
    double remain_dist_uss = 5.01;
    pnc::mathlib::spline x_s_spline;
    pnc::mathlib::spline y_s_spline;

    PlannerStateMachine plan_stm;
    EgoSlotInfo ego_slot_info;

    uint8_t current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
    uint8_t current_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;

    bool need_update_slot = true;

    bool correct_path_for_limiter = false;
    bool replan_flag = false;
  };

  enum PathPlannerResult {
    PLAN_FAILED,  // path plan failed
    PLAN_HOLD,    // follow last
    PLAN_UPDATE,
  };

  enum ParkingStatus {
    PARKING_IDLE,        // not in control
    PARKING_RUNNING,     // in control and no replan
    PARKING_GEARCHANGE,  // replan by non-updated path
    PARKING_PLANNING,    // replan by updated path
    PARKING_FINISHED,    // parking successful
    PARKING_FAILED,      // parking failed
    PARKING_PAUSED,      // parking paused
  };

  enum ParkingFailReason {
    NOT_FAILED,
    PAUSE_FAILED_TIME,
    STUCK_FAILED_TIME,
    UPDATE_EGO_SLOT_INFO,
    POST_PROCESS_PATH_POINT_SIZE,
    POST_PROCESS_PATH_POINT_SAME,
    SET_SEG_INDEX,
    CHECK_GEAR_LENGTH,
    PATH_PLAN_FAILED,
  };

 public:
  virtual void Init(const bool c_ilqr_enable);
  virtual void Reset() = 0;
  virtual void Update() = 0;
  virtual std::string GetName() = 0;

  virtual void SetSimuParam(const SimulationParam &param) {
    simu_param_ = param;
    is_simulation_ = true;
  }

  const PlannerStateMachine &GetPlannerStates() const {
    return frame_.plan_stm;
  }

  const Frame &GetFrame() const { return frame_; }

  const iflyauto::PlanningOutput &GetOutput() const { return planning_output_; }

  void SetApaWorldPtr(const std::shared_ptr<ApaWorld> &apa_world_ptr) {
    apa_world_ptr_ = apa_world_ptr;
  }

  const std::shared_ptr<LateralPathOptimizer> GetLateralPathOptimizerPtr()
      const {
    return lateral_path_optimizer_ptr_;
  }

 protected:
  virtual void GenPlanningOutput() = 0;
  virtual void GenPlanningPath() = 0;
  virtual const bool CheckFinished() = 0;
  virtual const bool CheckStuckFailed() = 0;
  virtual const bool CheckReplan() = 0;
  virtual void UpdateRemainDist() = 0;
  virtual const double CalRemainDistFromPath() = 0;
  virtual const double CalRemainDistFromUss() = 0;
  virtual const bool PostProcessPath() = 0;
  virtual void Log() const = 0;

  std::shared_ptr<ApaWorld> apa_world_ptr_;
  std::shared_ptr<LateralPathOptimizer> lateral_path_optimizer_ptr_;

  iflyauto::PlanningOutput planning_output_;

  Frame frame_;

  SimulationParam simu_param_;
  bool is_simulation_ = false;
  bool update_optimizer_ptr_enable = false;
};

}  // namespace apa_planner
}  // namespace planning

#endif
