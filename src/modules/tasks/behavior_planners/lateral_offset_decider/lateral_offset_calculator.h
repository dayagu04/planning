#pragma once

#include "config/basic_type.h"
#include "define/planning_status.h"
#include "lateral_obstacle.h"
#include "scenario_state_machine.h"
#include "task.h"
#include "utils/pose2d_utils.h"
#include "virtual_lane_manager.h"

namespace planning {

class LateralOffsetCalculator {
 public:
  LateralOffsetCalculator() = default;
  LateralOffsetCalculator(const EgoPlanningConfigBuilder *config_builder);
  ~LateralOffsetCalculator() = default;

  bool Process(planning::framework::Frame *frame,
               std::shared_ptr<TaskPipelineContext> &pipeline_context,
               const std::array<std::vector<double>, 2> &avd_car_past,
               const std::array<std::vector<double>, 2> &avd_sp_car_past,
               double dist_rblane, bool flag_avd);

  double lat_offset() const { return lat_offset_; }

  const std::vector<double> &left_lane_boundary_poly() {
    return left_lane_boundary_poly_;
  };
  const std::vector<double> &right_lane_boundary_poly() {
    return right_lane_boundary_poly_;
  };
  void Reset();

 private:
  void set_left_lane_boundary_poly() {
    for (auto i = 0;
         i < flane_->get_left_lane_boundary().poly_coefficient_size(); ++i) {
      if (i < 4) {
        left_lane_boundary_poly_.push_back(
            flane_->get_left_lane_boundary().poly_coefficient(i));
      }
    }
  }

  void set_right_lane_boundary_poly() {
    for (auto i = 0;
         i < flane_->get_right_lane_boundary().poly_coefficient_size(); ++i) {
      if (i < 4) {
        right_lane_boundary_poly_.push_back(
            flane_->get_right_lane_boundary().poly_coefficient(i));
      }
    }
  }

  bool update_basic_path(const int &status);

  void update_premove_path(
      int status, bool should_premove, bool should_suspend, bool accident_ahead,
      const std::array<std::vector<double>, 2> &avd_car_past);

  bool update_avoidance_path(
      int status, bool flag_avd, bool accident_ahead, bool should_premove,
      double dist_rblane,
      const std::array<std::vector<double>, 2> &avd_car_past,
      const std::array<std::vector<double>, 2> &avd_sp_car_past);

  void calc_desired_path(const std::array<double, 4> &l_poly,
                         const std::array<double, 4> &r_poly, double l_prob,
                         double r_prob, double intercept_width,
                         std::array<double, 4> &d_poly);

  double calc_lane_width_by_dist(const std::vector<double> &left_poly,
                                 const std::vector<double> &right_poly,
                                 const double &dist_x);

  bool update(int lane_status, bool flag_avoid, bool exist_accident_ahead,
              bool execute_premove, bool should_suspend, double dist_rblane,
              const std::array<std::vector<double>, 2> &avoid_car_info,
              const std::array<std::vector<double>, 2> &avoid_sp_car_info);

  bool update_planner_output();

  bool update_lateral_info();
  void save_to_debug_info();
  bool update_planner_status();

  bool log_planner_debug_info();

  bool create_lateral_behavior_planner_msg(std::string &plan_msg);

 private:
  VisionLateralMotionPlannerConfig config_;
  planning::framework::Frame *frame_;
  bool sb_lane_ = false;
  bool sb_blane_ = false;
  bool premoving_ = false;
  bool large_lat_ = false;
  bool cross_lsolid_line_ = false;
  bool cross_rsolid_line_ = false;
  bool force_pause_ = false;
  bool l_reject_ = false;
  bool r_reject_ = false;
  double lane_width_ = 3.8;
  double lat_offset_ = 0;
  double curr_time_ = 0;
  double two_ncar_ = -1000;
  double one_ncarl_ = -1000;
  double one_ncarr_ = -1000;
  int reject_reason_ = NO_REJECTION;
  double intercept_width_ = 3.8;
  int lb_suspend_cnt_ = 0;
  double suspend_lat_offset_ = 0;

  bool cross_left_solid_line_ = false;
  bool cross_right_solid_line_ = false;
  std::array<double, 4> c_poly_;
  std::array<double, 4> d_poly_;
  std::array<double, 4> l_poly_;
  std::array<double, 4> r_poly_;
  std::vector<double> left_lane_boundary_poly_;
  std::vector<double> right_lane_boundary_poly_;
  std::array<std::vector<double>, 2> avd_car_past_;
  std::array<std::vector<double>, 2> avd_sp_car_past_;
  std::shared_ptr<ReferencePath> fix_reference_path_;
  std::shared_ptr<VirtualLane> flane_;
  std::shared_ptr<EgoStateManager> ego_cart_state_manager_;
  FrenetEgoState ego_frenet_state_;
  std::shared_ptr<VirtualLaneManager> virtual_lane_manager_;

  int two_nudge_car_;
  int one_nudge_left_car_;
  int one_nudge_right_car_;
  int lane_borrow_suspend_cnt_ = 0;
};

}  // namespace planning
