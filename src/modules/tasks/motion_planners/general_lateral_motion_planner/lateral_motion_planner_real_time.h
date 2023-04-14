#pragma once

#include <dlfcn.h>

#include "common/utils/pose2d_utils.h"
#include "modules/context/virtual_lane_manager.h"
#include "src/modules/common/config/basic_type.h"
#include "src/modules/tasks/task.h"
namespace planning {
typedef enum {
  NO_REJECTION,
  BIAS_L,
  BIAS_R,
  WIDE_REJECTION_L,
  WIDE_REJECTION_R,
  SHORT_REJECTION,
  NARROW_REJECTION
} RejectReason;

struct LateralSolverOption {
  bool enable_log{false};
};

class LateralMotionPlannerV1 : public Task {
 public:
  explicit LateralMotionPlannerV1(
      const EgoPlanningConfigBuilder *config_builder,
      const std::shared_ptr<TaskPipelineContext> &pipeline_context);

  virtual ~LateralMotionPlannerV1() = default;

  bool Execute(planning::framework::Frame *frame) override;

  bool update_basic_path(const int &lane_status);

  bool update_premove_path(
      int lane_status, bool execute_premove, bool should_suspend,
      bool exist_accident_ahead,
      const std::array<std::vector<double>, 2> &avoid_car_info);

  bool update_avoidance_path(
      int lane_status, bool flag_avoid, bool exist_accident_ahead,
      bool execute_premove, double dist_rblane,
      const std::array<std::vector<double>, 2> &avoid_car_info,
      const std::array<std::vector<double>, 2> &avoid_sp_car_info);

  bool check_premove(const int &lane_status);
  bool check_avoidance_path(const int &lane_status);

  // void calc_desired_path(const CubicPoly &l_poly, const CubicPoly &r_poly,
  //                        const double &left_prob, const double &right_prob,
  //                        const double &intercept_width,
  //                        CubicPoly &desired_poly);

  double calc_lane_width_by_dist(const std::vector<double> &left_poly,
                                 const std::vector<double> &right_poly,
                                 const double &dist_x);

  bool update(int lane_status, bool flag_avoid, bool exist_accident_ahead,
              bool execute_premove, bool should_suspend, double dist_rblane,
              const std::array<std::vector<double>, 2> &avoid_car_info,
              const std::array<std::vector<double>, 2> &avoid_sp_car_info);

  bool update_planner_output();

  bool update_lateral_info();

  bool update_planner_status();

  bool log_planner_debug_info();

  bool create_lateral_behavior_planner_msg(std::string &plan_msg);

 private:
  LateralMotionPlannerV1Config config_;

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
  std::array<double, 4> c_poly_;
  std::array<double, 4> d_poly_;
  std::array<double, 4> l_poly_;
  std::array<double, 4> r_poly_;
  std::array<std::vector<double>, 2> avd_car_past_;

  // VirtualLaneManager &virtual_lane_mgr_;
};

}  // namespace planning
