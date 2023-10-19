#include <memory>
#include "behavior_planners/vision_only_lane_change_decider/vision_only_lane_change_decider.h"
#include "lateral_obstacle.h"
#include "task.h"
#include "virtual_lane_manager.h"
#include "filters.h"

namespace planning {

class VisionLongitudinalBehaviorPlanner : public Task {
 public:
  explicit VisionLongitudinalBehaviorPlanner(
      const EgoPlanningConfigBuilder *config_builder,
      const std::shared_ptr<TaskPipelineContext> &pipeline_context);
  virtual ~VisionLongitudinalBehaviorPlanner() = default;

  bool Execute(framework::Frame *frame);

 private:
  virtual bool calculate();
  bool update();

  bool calc_cruise_accel_limits(const double v_ego);
  bool limit_accel_velocity_in_turns(const double v_ego,
                                     const double angle_steers,
                                     const std::vector<double> &d_poly,
                                     const std::vector<ReferencePathPoint> &ref_points);

  bool limit_accel_velocity_for_cutin(
      const std::vector<TrackedObject> &front_tracks,
      const std::vector<TrackedObject> &side_tracks, const string &lc_status,
      const double v_ego);
  bool calc_speed_with_leads(const TrackedObject *lead_one,
                             const TrackedObject *lead_two,
                             const string &lc_request, const double v_ego);
  bool calc_speed_with_temp_leads(const TrackedObject *temp_lead_one,
                                  const TrackedObject *temp_lead_two,
                                  const double v_ego,
                                  const bool close_to_accident,
                                  const string &lc_request,
                                  const string &lc_status);
  bool calc_speed_for_ramp(double v_ego);
  bool calc_speed_with_potential_cutin_car(
      const std::vector<TrackedObject> &front_tracks, const string &lc_request,
      const double v_cruise, const double v_ego);

  bool calc_speed_for_lane_change(const TrackedObject *lead_one,
                                  const double v_cruise, const double v_ego,
                                  const string &lc_request,
                                  const string &lc_status);

  double interp(const double x, const std::vector<double> &xp,
                const std::vector<double> &fp);
  double process_a_lead(const double a_lead);
  double calc_desired_distance(const double v_lead, const double v_ego,
                               const std::string &lc_request,
                               const bool is_accident_car = false,
                               const bool is_temp_lead = false);
  double calc_desired_speed(const double d_rel, const double d_des,
                            const double v_lead);
  bool calc_acc_accel_limits(const double d_lead, const double d_des,
                             const double v_ego, const double v_lead,
                             const double v_rel_const, const double a_lead,
                             const double v_target,
                             std::pair<double, double> &a_target,
                             const double y_min);
  double calc_positive_accel_limit(const double d_lead, const double d_des,
                                   const double v_ego, const double v_rel,
                                   const double v_target,
                                   const double a_lead_contr,
                                   const double a_max_const);
  double calc_critical_decel(const double d_lead, const double v_rel,
                             const double d_offset, const double v_offset);
  double clip(const double x, const double lo, const double hi);

  // 计算启停状态，避免二次起步
  StartStopInfo::StateType UpdateStartStopState(const TrackedObject *lead_one,
                                                const double v_ego);

  int GetCIPV(const std::shared_ptr<LateralObstacle> &lateral_obstacle,
              const std::string &lc_status);

 private:
  void update_planner_output();
  void log_planner_debug_info();
  void create_vision_longitudinal_behavior_planner_msg(std::string &plan_msg);

 private:
  // lookup tables VS speed to determine min and max accels in cruise
  const std::vector<double> _A_CRUISE_MIN_BP{0.0, 5.0, 10.0, 20.0, 40.0};
  const std::vector<double> _A_CRUISE_MIN_V{-0.5, -0.5, -0.5, -0.5, -0.3};
  // need fast accel at very low speed for stop and go
  const std::vector<double> _A_CRUISE_MAX_BP{0.0, 5.0, 10.0, 20.0, 40.0};
  const std::vector<double> _A_CRUISE_MAX_V{1.0, 0.85, 0.6, 0.5, 0.3};
  // change the ay_limit values according to the angle steers
  const std::vector<double> _A_TOTAL_MAX_BP{0., 20., 40.};
  const std::vector<double> _A_TOTAL_MAX_V{1.5, 1.9, 3.2};
  const std::vector<double> _AY_MAX_ABS_BP{5.0, 10.0, 15.0, 30.0};
  const std::vector<double> _AY_MAX_STEERS{2.0, 1.8, 1.6, 1.6};
  const std::vector<double> _AY_MAX_CURV_BP{100, 200, 400, 600};
  const std::vector<double> _AY_MAX_CURV_V{1.2, 0.6, 0.4, 0.3};
  const std::vector<double> _T_GAP_VEGO_BP{5.0, 15.0, 30.0};
  const std::vector<double> _T_GAP_VEGO_V{1.35, 1.55, 2.0};
  // linear slope
  const std::vector<double> _L_SLOPE_BP{0.0, 40.0};
  const std::vector<double> _L_SLOPE_V{0.35, 0.08};
  // parabola slope
  const std::vector<double> _P_SLOPE_BP{0., 40.0};
  const std::vector<double> _P_SLOPE_V{0.8, 0.2};
  // do not consider a_lead at 0m/s, fully consider it at 5m/s
  const std::vector<double> _A_LEAD_LOW_SPEED_BP{0.0, 5.0};
  const std::vector<double> _A_LEAD_LOW_SPEED_V{0.0, 1.0};
  // lower a_lead when distance is far away
  const std::vector<double> _A_LEAD_DISTANCE_BP{50.0, 100.0};
  const std::vector<double> _A_LEAD_DISTANCE_V{1.0, 0.5};
  // different offset based on the likelyhood that lead decels abruptly
  const std::vector<double> _DECEL_OFFSET_BP{0.0, 4.0, 15.0, 30.0, 40.0};
  const std::vector<double> _DECEL_OFFSET_V{-0.3, -0.5, -0.5, -0.4, -0.3};
  // maximum acceleration adjustment
  const std::vector<double> _A_CORR_BY_SPEED_BP{0.0, 2.0, 10.0};
  const std::vector<double> _A_CORR_BY_SPEED_V{0.4, 0.4, 0.0};
  // lower y_thres when cutin lon speed is faster
  const std::vector<double> _Y_THRES_SPEED_BP{0.0, 1.0};
  const std::vector<double> _Y_THRES_SPEED_V{0.8, 0.0};
  const std::vector<double> _CUT_IN_COEFF_BP{1.6, 1.8, 2.0, 2.5};
  const std::vector<double> _CUT_IN_COEFF_V{1.0, 1.0, 1.33, 0.9 / 0.65};
  const std::vector<double> _D_THRES_SPEED_BP{0.5, 1.0};
  const std::vector<double> _D_THRES_SPEED_V{0.5, 1.0};
  // lower prebrk by ttc
  const std::vector<double> _A_PREBRK_TTC_BP{5.0, 10.0};
  const std::vector<double> _A_PREBRK_TTC_V{1.0, 0.0};

  const double _A_MAX = 2.0;
  const double _A_MIN = -4.0;

 private:
  double v_target_ = 40.0;
  double v_limit_in_turns_ = 40.0;
  double v_limit_ramp_ = 40.0;
  double v_target_ramp_ = 40.0;
  std::pair<double, double> a_target_;
  std::pair<double, double> a_target_objective_;
  std::map<int, double> a_limit_cutin_now_;
  std::map<int, double> a_limit_cutin_history_;

  CutinMsg cutin_msg_{"none", 0.0, 1};
  std::array<std::string, 3> cutin_info_;

  VisionLongitudinalBehaviorPlannerConfig config_;

  std::shared_ptr<VisionOnlyLaneChangeDecider> lane_changing_decider_ = nullptr;
  //限制猛加速的滤波器
  pnc::filters::SlopeFilter accel_vel_filter_;
  bool is_on_ramp_ = false;
};
}  // namespace planning
