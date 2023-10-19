#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Platform_Types.h"
#include "collision_detection/collision_detection.h"
#include "common/geometry_planning_io.h"
#include "diagonal/diagonal_in_geometry_plan.h"
#include "dubins_lib/dubins_lib.h"
#include "dubins_lib/geometry_math.h"
#include "frame.h"
#include "local_view.h"
#include "math/box2d.h"
#include "parking_fusion.pb.h"
#include "path_point.h"
#include "planning_plan.pb.h"
#include "slot_management/slot_management.h"
#include "slot_management_info.pb.h"
#include "speed_smoother/apa_speed_smoother.h"
#include "spline.h"
#include "spline_projection.h"
#include "uss_obstacle_avoidance/uss_obstacle_avoidance.h"
namespace planning {
namespace apa_planner {

class DiagonalInTrajectoryGenerator {
  enum PlanningStateMachine {
    INIT,
    PREPARE,
    PAUSE,
    FINAL,
    ADJUST,
    FINISH,
  };

  enum PlanAlgorithm {
    DUBINS,
    LINE_ARC,
  };

  enum DubinsPlanLevel {
    DUBINS_LEVEL_ZERO_GEAR_CHANGE,
    DUBINS_LEVEL_ONCE_GEAR_CHANGE,
    DUBINS_LEVEL_TWICE_GEAR_CHANGE,
    DUBINS_LEVEL_COUNT,
  };

  struct Measurement {
    double v_ego = 0.0;
    double heading = 0.0;
    Eigen::Vector2d ego_pos = Eigen::Vector2d::Zero();
    double standstill_timer = 0.0;
    double standstill_timer_by_pos = 0.0;
    bool static_flag = false;
  };
  struct PlanInput {
    Eigen::Vector2d target_pos;
    double target_heading = 0.0;
    double path_radius = 0.0;
  };

  struct EgoSlotInfo {
    enum SlotSegIndex {
      LEFT,
      RIGHT,
    };

    void Reset() {
      slot_origin_pos.setZero();
      slot_origin_heading = 0.0;
      ego_pos_slot.setZero();
      ego_heading_slot = 0.0;

      slot_width = 3.0;

      slot_obs.first = false;
      left_slot_obs.first = false;
      right_slot_obs.first = false;

      obstacles_vec.clear();
      obstacles_vec.reserve(4);
    }

    Eigen::Vector2d slot_origin_pos = Eigen::Vector2d::Zero();
    double slot_origin_heading = 0.0;

    Eigen::Vector2d ego_pos_slot = Eigen::Vector2d::Zero();
    double ego_heading_slot = 0.0;

    double slot_width = 3.0;

    // note that first bool means obstacle enable
    // left and right seg (0, 1) are considered for current slot
    std::pair<bool, std::array<pnc::geometry_lib::LineSegment, 2>> slot_obs;

    // just up seg is considered for left and right slot
    std::pair<bool, pnc::geometry_lib::LineSegment> left_slot_obs;
    std::pair<bool, pnc::geometry_lib::LineSegment> right_slot_obs;

    std::vector<pnc::geometry_lib::LineSegment> obstacles_vec;
  };

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  struct SimulationParam {
    uint8_t force_plan_stm = 0;
    int selected_id_ = 0;
    bool force_planning_ = false;
    uint8_t current_state_ = 0;
    bool is_complete_path = false;
    double sample_ds = 0.5;

    double sublane_width = 0.0;
    double sublane_right_length = 0.0;
    double sublane_left_length = 0.0;

    uint8_t plan_level = DUBINS_LEVEL_ONCE_GEAR_CHANGE;
  };

  DiagonalInTrajectoryGenerator() {
    managed_parking_fusion_info_ptr_ =
        std::make_shared<ParkingFusion::ParkingFusionInfo>();
    Reset();
    uss_oa_.Init();
    collision_detector_.Init();
  };

  ~DiagonalInTrajectoryGenerator() = default;

  const bool Plan(framework::Frame *const frame);

  // for pybind & simulation
  void SetLocalView(LocalView *local_view_ptr) { local_view_ = local_view_ptr; }

  PlanningOutput::PlanningOutput GetOutput() const { return planning_output_; }

  const common::SlotManagementInfo GetSlotManagementOutput() const {
    return slot_manager_.GetOutput();
  }

  // for pybind simulation
  void SetSimulationParam(SimulationParam &param) { simu_param_ = param; }
  void SetSlotWidthOffset(double slot_width_offset) {
    left_slot_width_offset_ = slot_width_offset;
    right_slot_width_offset_ = slot_width_offset;
  }

  // for apa planner pybind simulation
  const bool PathPlanOnceSimulation(
      common::SlotManagementInfo &slot_mangement_info);

  pnc::dubins_lib::DubinsLibrary GetDubinsPlanner() const {
    return dubins_planner_;
  }

  const std::vector<pnc::geometry_lib::LineSegment> GetObstacles() const {
    return collision_detector_.GetObstacles();
  }

 private:
  void UpdateMeasurement();
  void UpdateEgoSlotInfo(const int slot_index);
  void PostProcessPath();

  void GeneratePlanningOutput(
      PlanningOutput::PlanningOutput *const planning_output);

  void GeneratePlanningOutputByUssOA(
      PlanningOutput::PlanningOutput *const planning_output);

  // sub function for dubins plan
  const bool DubinsPlanOneStep(const PlanInput &plan_input,
                               const uint8_t plan_algorithm);

  const bool DubinsPlanOnceGearChange(uint8_t plan_statemachine);
  const bool DubinsPlanTwiceGearChange(uint8_t plan_statemachine);

  const bool PathEvaluateOnce() const;
  const bool CollisionCheck();
  const bool CheckIfCrossSublane() const;

  const bool CheckIfCrossSublane(
      pnc::dubins_lib::DubinsLibrary::PathPoint check_point) const;

  const bool CheckPathPointsInSlot() const;

  void PrintDubinsOutput();

  const bool CheckIfNearTerminalPoint();
  const bool CheckIfReplanByStuck() const;
  const bool CheckFinish();

  const bool CheckReplan(PlanningOutput::PlanningOutput *const planning_output);

  void PathPlanOnce(const int slot_index,
                    PlanningOutput::PlanningOutput *const planning_output);

  const bool PathPlanCoreIteration();
  void Log() const;
  const bool UpdateManagedParkingFusion(const int select_slot_index);
  void UpdateObstacles();

 private:
  // reset
  void Reset();

  // local view and frame
  const LocalView *local_view_ = nullptr;
  framework::Frame *frame_ = nullptr;

  // uss obstacle avoidance
  UssObstacleAvoidance uss_oa_;

  // collision detection
  CollisionDetector collision_detector_;

  // slot management
  SlotManagement slot_manager_;
  ParkingFusion::ParkingFusionInfo managed_parking_fusion_info_;
  std::shared_ptr<ParkingFusion::ParkingFusionInfo>
      managed_parking_fusion_info_ptr_;

  // for path planner
  pnc::dubins_lib::DubinsLibrary dubins_planner_;
  PlanInput plan_input_;
  uint8_t plan_algorithm_ = DUBINS;
  size_t replan_in_slot_count_ = 0;
  pnc::dubins_lib::DubinsLibrary::Input dubins_input_;
  pnc::dubins_lib::DubinsLibrary::Output plan_result_;
  std::vector<pnc::dubins_lib::DubinsLibrary::Output> multi_step_plan_result_;

  std::vector<pnc::geometry_lib::LineSegment>
      uss_obstacles_vec_;  // slot obstacles which considered in EgoSlotInfo are
                           // not included here

  size_t dubins_iter_count_ = 0;
  double sublane_left_length_ = 0.0;
  double sublane_right_length_ = 0.0;
  double sublane_width_ = 0.0;
  double left_slot_width_offset_ = 0.0;
  double right_slot_width_offset_ = 0.0;
  // TODO
  uint8_t current_state_ = ::FuncStateMachine::FunctionalState::INIT;

  // for pybind simulation
  PlanningOutput::PlanningOutput planning_output_;
  bool simulation_enable_flag_ = false;
  SimulationParam simu_param_;

  // system states
  pnc::geometry_lib::GlobalToLocalTf g2l_tf_;
  pnc::geometry_lib::LocalToGlobalTf l2g_tf_;
  pnc::geometry_lib::LocalToGlobalTf ego2slot_tf_;
  pnc::mathlib::spline x_s_spline_g_;
  pnc::mathlib::spline y_s_spline_g_;
  double current_path_length_ = 0.0;
  double remain_dist_ = 0.0;
  double remain_dist_uss_ = 0.0;
  pnc::dubins_lib::DubinsLibrary::PathPoint terminal_err_;
  pnc::dubins_lib::DubinsLibrary::PathPoint target_err_;
  bool spline_success_ = false;
  bool is_replan_ = false;
  bool is_replan_by_uss_ = false;
  bool is_finished_ = false;
  bool is_plan_success_ = false;
  uint8_t gear_change_count_ = 6;
  uint8_t path_level_ = DUBINS_LEVEL_ZERO_GEAR_CHANGE;
  double stuck_time_ = 0.0;
  double slot_occupied_ratio_ = 0.0;
  double parking_continue_time_ = 0.0;
  bool twice_gear_change_enable_ = true;

  Measurement measure_;
  EgoSlotInfo ego_slot_info_;
  uint8_t plan_state_machine_ = INIT;
};

}  // namespace apa_planner
}  // namespace planning