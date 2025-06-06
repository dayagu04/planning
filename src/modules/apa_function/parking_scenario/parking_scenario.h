#ifndef __PARKING_SCENARIO_H__
#define __PARKING_SCENARIO_H__

#include <bits/stdint-uintn.h>
#include <sys/types.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "apa_context.h"
#include "apa_param_config.h"
#include "apa_world.h"
#include "collision_detection/collision_detection.h"
#include "geometry_math.h"
#include "lateral_path_optimizer.h"
#include "local_view.h"
#include "parking_task/parking_task.h"
#include "planning_hmi_c.h"
#include "planning_plan_c.h"
#include "speed/speed_data.h"
#include "trajectory/trajectory.h"

namespace planning {
namespace apa_planner {
// 1.
// 对于泊车而言，不同的行为对应不同的场景，包括垂直泊入场景、垂直泊出场景,等等.
// 2. 每一个场景, 都有不同的计算任务.
// 例如计算任务包括：路径计算,速度计算,超声波处理,
// 障碍物决策,目标点位置决策,路径优化,速度优化,等等.
// 3. 对于路径计算这个任务而言，一般可以分为curve_based_path_generator,
// hybrid_astar_path_generator.
// TODO: move all scenario related codes in here. and move all task related code
// to Task. For long term development, you can create new Task and update old
// Task.
class ParkingScenario {
 public:
  enum ReplanReason {
    NOT_REPLAN,
    FIRST_PLAN,
    SEG_COMPLETED_PATH,
    SEG_COMPLETED_OBS,
    SEG_COMPLETED_SLOT_JUMP,
    STUCKED,
    DYNAMIC,
    SEG_COMPLETED_COL_DET,
    FORCE_PLAN,
  };

  struct CheckReplanParams {
    double replan_dist_path = apa_param.GetParam().max_replan_remain_dist;
    double wait_time_path = 0.068;
    double replan_dist_obs = apa_param.GetParam().max_replan_remain_dist;
    double wait_time_obs = apa_param.GetParam().uss_stuck_replan_wait_time;
    double replan_dist_slot_jump = apa_param.GetParam().max_replan_remain_dist;
    double wait_time_slot_jump = 0.168;
    double stuck_replan_time = apa_param.GetParam().stuck_replan_time;

    CheckReplanParams() = default;
    CheckReplanParams(double replan_dist_path, double wait_time_path,
                      double replan_dist_obs, double wait_time_obs,
                      double replan_dist_slot_jump, double wait_time_slot_jump,
                      double stuck_replan_time)
        : replan_dist_path(replan_dist_path),
          wait_time_path(wait_time_path),
          replan_dist_obs(replan_dist_obs),
          wait_time_obs(wait_time_obs),
          replan_dist_slot_jump(replan_dist_slot_jump),
          wait_time_slot_jump(wait_time_slot_jump),
          stuck_replan_time(stuck_replan_time) {}
    ~CheckReplanParams() = default;
  };

  // will be retired
  struct EgoSlotInfo {
    common::SlotInfo target_managed_slot;
    std::pair<Eigen::Vector2d, Eigen::Vector2d> limiter;
    Eigen::Vector2d target_ego_pos_slot = Eigen::Vector2d::Zero();
    double target_ego_heading_slot = 0.0;

    std::vector<Eigen::Vector2d> slot_corner;
    double last_move_slot_dist = 0.0;
    double move_slot_dist = 0.0;
    std::vector<Eigen::Vector2d> limiter_corner;

    Eigen::Vector2d slot_center = Eigen::Vector2d::Zero();

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
    pnc::geometry_lib::PathPoint initial_err;
    double slot_occupied_ratio = 0.0;

    pnc::geometry_lib::GlobalToLocalTf g2l_tf;
    pnc::geometry_lib::LocalToGlobalTf l2g_tf;

    bool fix_limiter = false;

    std::vector<Eigen::Vector2d> obs_pt_vec_slot;

    double origin_pt_0_heading = 0.0;
    double sin_angle = 1.0;
    Eigen::Vector2d pt_0;
    Eigen::Vector2d pt_1;

    double channel_width;

    bool fus_obj_valid_flag = false;

    void Reset() {
      target_managed_slot.Clear();
      selected_slot_id = 0;
      slot_type = 0;
      target_ego_pos_slot = Eigen::Vector2d::Zero();
      target_ego_heading_slot = 0.0;

      slot_corner.clear();
      limiter_corner.clear();
      move_slot_dist = 0.0;
      last_move_slot_dist = 0.0;

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

      fix_limiter = false;

      obs_pt_vec_slot.clear();

      origin_pt_0_heading = 0.0;
      sin_angle = 1.0;
      pt_0.setZero();
      pt_1.setZero();

      channel_width = apa_param.GetParam().channel_width;

      fus_obj_valid_flag = false;
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
      replan_flag = false;
      is_replan_first = true;
      is_replan_second = false;
      is_replan_dynamic = false;
      is_replan_by_obs = false;
      is_last_path = false;
      dynamic_replan_count = 0;
      dynamic_replan_fail_count = 0;
      ego_stop_when_slot_jumps_much = false;
      total_plan_count = 0;
      in_slot_plan_count = 0;
      is_fix_slot = false;
      stuck_time = 0.0;
      stuck_obs_time = 0.0;
      pause_time = 0.0;
      dynamic_plan_time = 0.0;
      replan_fail_time = 0.0;
      remain_dist_path = 5.01;
      remain_dist_path_last = 5.01;
      remain_dist_obs = 5.01;
      remain_dist_col_det = 5.01;
      remain_dist_slot_jump = 5.01;
      vel_target = 1.168;
      car_already_move_dist = 0.0;
      current_path_last_point_heading = 0.0;
      spline_success = false;
      current_path_length = 0.0;
      headin_current_path_length = 0.0;
      path_extended_dist = 1.0;
      ego_slot_info.Reset();
      plan_stm.Reset();
      pathplan_result = 0;
      current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
      current_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;
      replan_reason = NOT_REPLAN;
      plan_fail_reason = NOT_FAILED;
      correct_path_for_limiter = false;
      can_correct_path_for_limiter = true;
      dynamic_plan_fail_flag = false;
      gear_command = pnc::geometry_lib::SEG_GEAR_INVALID;

      is_left_empty = false;
      is_right_empty = false;

      can_first_plan_again = true;
      is_park_out_left = true;

      stuck_by_dynamic_obs = false;

      process_obs_method = ProcessObsMethod::DO_NOTHING;

      dynamic_plan_path_superior = false;
    }

    ProcessObsMethod process_obs_method = ProcessObsMethod::DO_NOTHING;

    bool can_first_plan_again = true;

    bool is_left_empty = false;
    bool is_right_empty = false;
    bool is_park_out_left = true;

    bool replan_flag = false;
    bool is_replan_first = true;
    bool is_replan_second = false;
    bool is_replan_dynamic = false;
    bool is_replan_by_obs = false;
    bool is_last_path = false;
    uint8_t dynamic_replan_count = 0;
    uint8_t dynamic_replan_fail_count = 0;
    bool ego_stop_when_slot_jumps_much = false;
    uint8_t replan_reason = NOT_REPLAN;
    uint8_t plan_fail_reason = NOT_FAILED;
    uint8_t total_plan_count = 0;
    uint8_t in_slot_plan_count = 0;
    bool is_fix_slot = false;
    bool spline_success = false;
    uint8_t pathplan_result = 0;
    double current_path_length = 0.0;
    double headin_current_path_length = 0.0;
    double path_extended_dist = 1.0;
    double vel_target = 1.168;
    double stuck_time = 0.0;
    // stuck by static obs
    double stuck_obs_time = 0.0;
    double pause_time = 0.0;
    double dynamic_plan_time = 0.0;
    // If replan fail time is long, set PARKING_FAILED.
    double replan_fail_time = 0.0;
    // remain dist for path
    double remain_dist_path = 5.01;
    double remain_dist_path_last = 5.01;
    // remain dist for obs
    double remain_dist_obs = 5.01;
    // path remain dist by fusion occ check
    double remain_dist_col_det = 5.01;
    double remain_dist_slot_jump = 5.01;
    double car_already_move_dist = 0.0;
    double current_path_last_point_heading = 0.0;
    pnc::mathlib::spline x_s_spline;
    pnc::mathlib::spline y_s_spline;
    pnc::mathlib::spline headin_x_s_spline;
    pnc::mathlib::spline headin_y_s_spline;

    PlannerStateMachine plan_stm;
    // will be retired
    EgoSlotInfo ego_slot_info;

    uint8_t current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
    uint8_t current_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;

    bool correct_path_for_limiter = false;
    bool can_correct_path_for_limiter = true;
    bool dynamic_plan_fail_flag = false;
    bool dynamic_plan_path_superior = false;

    uint8_t gear_command = pnc::geometry_lib::SEG_GEAR_INVALID;

    bool stuck_by_dynamic_obs = false;
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
    PLAN_COUNT_EXCEED_LIMIT,
    DYNAMIC_PATH_NOT_SUPERIOR,
    NO_TARGET_POSE,
  };

 public:
  virtual ~ParkingScenario() = default;

  ParkingScenario();

  ParkingScenario(const std::shared_ptr<ApaWorld> &apa_world_ptr);

  virtual void Init();
  virtual void Reset();
  virtual std::string GetName();

  void SetApaWorldPtr(const std::shared_ptr<ApaWorld> &apa_world_ptr) {
    apa_world_ptr_ = apa_world_ptr;
    return;
  }

  std::shared_ptr<ApaWorld> GetApaWorldPtr() { return apa_world_ptr_; }

  // 点击开始泊车之后，更新speed and path
  virtual void ScenarioRunning();

  // 点击了车位，但还没点击开始泊车，那么调用这个函数，尝试一下看看路径生成是否成功.
  virtual void ScenarioTry();

  const PlannerStateMachine &GetPlannerStates() const {
    return frame_.plan_stm;
  }

  const Frame &GetFrame() const { return frame_; }
  Frame *GetMutableFrame() { return &frame_; }

  const iflyauto::PlanningOutput &GetOutput() const { return planning_output_; }

  const iflyauto::APAHMIData &GetAPAHmi() const { return apa_hmi_; }

  // clear thread related
  virtual void ThreadClear();

  const std::vector<pnc::geometry_lib::PathPoint> &GetCompletePlanPathPt() {
    return complete_path_point_global_vec_;
  }

  const std::vector<pnc::geometry_lib::PathPoint> &GetCurrentGearPlanPathPt() {
    return current_path_point_global_vec_;
  }

  std::vector<pnc::geometry_lib::GeometryPath> &GetPerferredGeometryPathVec() {
    return perferred_geometry_path_vec_;
  }

 protected:
  virtual const bool CheckFinished() = 0;
  virtual void Log() const = 0;
  virtual void ExcutePathPlanningTask() = 0;

  virtual void ExcuteSpeedPlanningTask();

  virtual const bool GenTlane() = 0;
  virtual const bool GenObstacles() = 0;
  virtual const bool UpdateEgoSlotInfo() = 0;
  virtual const uint8_t PathPlanOnce() = 0;

  virtual void InitSimulation();
  virtual void UpdateStuckTime();
  virtual const bool CheckPaused() const;
  virtual const bool CheckPlanSkip() const;
  virtual void SetParkingStatus(uint8_t status);
  virtual void PublishPlanningTraj();
  virtual void GenPlanningHmiOutput();
  // No speed planning method.
  virtual void SetPlanningPath();
  void SetPlanningTraj();

  virtual const double CalRemainDistFromPath();
  virtual const double CalRemainDistFromObs(
      const double static_lon_buffer = 0.3,
      const double static_lat_buffer = apa_param.GetParam().stop_lat_inflation,
      const double dynamic_lon_buffer = 1.168,
      const double dynamic_lat_buffer = 0.368);
  virtual const bool PostProcessPath();

  // check if need replan
  virtual const bool CheckReplan(const CheckReplanParams &check_params);

  virtual const bool CheckSegCompleted(const double replan_dist,
                                       const double wait_time);

  virtual const bool CheckObsStucked(const double replan_dist,
                                     const double wait_time);

  virtual const bool CheckSlotJumpStucked(const double replan_dist,
                                          const double wait_time);

  virtual const bool CheckStuckTimeEnough(const double stuck_replan_time);

  virtual const bool CheckDynamicUpdate();

  virtual const bool CheckStuckFailed(
      const double stuck_failed_time = apa_param.GetParam().stuck_failed_time);

  virtual const bool CheckEgoPoseInBelieveObsArea(
      const double lat_expand, const double lon_expand,
      const double heading_err = 60.0);

  const geometry_lib::PathPoint GetCarFrontPoseFromCarPose(
      const geometry_lib::PathPoint &pose);

  void CreateTasks();

 protected:
  // TODO:
  // 1.
  // 场景内部，尽量只包含指针，不要创建大的数据，因为一旦场景多了之后，这些数据都是重复继承;
  // 比如：planning_output_，apa_hmi_，frame_，current_path_point_global_vec_,
  // 这种大的数据（10多Mb）就可以放到apa_world中.
  // 2. 泊车框架有2个数据流：
  // A：apa_data，存储处理的上游数据，以及发布给下游的数据；
  // B: apa_world，存储scenario, task各种内部数据;
  std::shared_ptr<ApaWorld> apa_world_ptr_;

  iflyauto::PlanningOutput planning_output_;
  iflyauto::APAHMIData apa_hmi_;

  Frame frame_;

  std::vector<pnc::geometry_lib::PathPoint> current_path_point_global_vec_;

  std::vector<pnc::geometry_lib::PathPoint> complete_path_point_global_vec_;

  // only debug for choosing the best path
  std::vector<pnc::geometry_lib::GeometryPath> perferred_geometry_path_vec_;

  trajectory::Trajectory trajectory_;
};

}  // namespace apa_planner
}  // namespace planning

#endif
