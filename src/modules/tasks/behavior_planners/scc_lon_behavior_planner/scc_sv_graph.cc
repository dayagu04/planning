#include "scc_sv_graph.h"

#include <utility>
#include "debug_info_log.h"
#include "environmental_model.h"
#include "planning_context.h"
#include "tasks/behavior_planners/scc_lon_behavior_planner/scc_lon_behavior_types.h"
namespace planning {
namespace scc {

SvGraphGenerator::SvGraphGenerator(const SccLonBehaviorPlannerConfig &plan_cfg)
    : config_(plan_cfg) {}

void SvGraphGenerator::Update(
    std::shared_ptr<common::RealTimeLonBehaviorInput> lon_behav_input) {
  lon_behav_input_ = std::move(lon_behav_input);
  LOG_DEBUG("=======Entering SvGraphGenerator::Update======= \n");
  double v_ego = lon_behav_input_->ego_info().ego_v();
  double v_cruise = lon_behav_input_->ego_info().ego_cruise();
  double steer_angle_ego = lon_behav_input_->ego_info().ego_steer_angle();
  SVBoundaries sv_boundaries;
  std::vector<double> d_polys;

  sv_boundaries_.clear();
  // 1.calc road curv sv speed limit
  const auto &d_poly_infos = lon_behav_input_->lat_output().d_poly_vec();
  for (auto d_poly_info : d_poly_infos) {
    d_polys.push_back(d_poly_info);
  }
  SVBoundary curve_sv_boundary;
  CalculateCurvSvs(v_ego, v_cruise, steer_angle_ego, d_polys,
                   curve_sv_boundary);
  sv_boundaries.emplace_back(curve_sv_boundary);
  // 2. 道路限速

  // 3. 路口等预减速

  // 4. 停车点

  // 5. 汇总sv信息
  sv_boundaries_.emplace_back(curve_sv_boundary);
  // UpdateSVGraphs(sv_boundaries);
}

void SvGraphGenerator::CalculateCurvSvs(const double v_ego,
                                        const double v_cruise,
                                        const double angle_steers,
                                        const std::vector<double> &d_poly,
                                        SVBoundary &sv_boundary) {
  LOG_DEBUG("----entering CalculateCurvSvs--- \n");
  double angle_steers_deg = angle_steers * DEG_PER_RAD;
  double base_dis = config_.dis_curv;  // 从自车位置开始，计算5s
  sv_boundary.boundary_type = SVBoundaryType::CURVATURE;
  sv_boundary.sv_bounds.resize(config_.lon_num_step + 1);

  // 获取参考线的点，其中有曲率、道路限速
  auto ref_path_points = lon_behav_input_->ref_path_points();

  SVBound sv_bound;
  for (int i = 0; i <= config_.lon_num_step; i++) {
    double s = base_dis + i * config_.delta_time * v_cruise;
    double curv = std::fabs(2 * d_poly[0] * s + d_poly[1]) /
                  std::pow(std::pow(2 * d_poly[0] * s + d_poly[1], 2) + 1, 1.5);
    double road_radius = 1 / std::max(curv, 0.0001);
    double a_y_max =
        interp(std::abs(angle_steers_deg), _AY_MAX_ABS_BP, _AY_MAX_STEERS);
    if (road_radius < 680) {
      a_y_max = interp(road_radius, _AY_MAX_CURV_BP, _AY_MAX_CURV_V);
    }
    double v_limit_road =
        std::min(std::sqrt(a_y_max * road_radius) * 0.9, v_cruise);

    sv_bound.s = s;
    sv_bound.v_bound.lower = 0.0;
    sv_bound.v_bound.upper = v_limit_road;

    sv_boundary.sv_bounds[i] = sv_bound;
    JSON_DEBUG_VALUE("RealTime_v_limit_road", v_limit_road);
  }
}

void SvGraphGenerator::UpdateSVGraphs(const SVBoundaries &sv_boundaries) {
  sv_boundaries_ = sv_boundaries;
}

void SvGraphGenerator::SetConfig(
    planning::common::RealTimeLonBehaviorTunedParams &tuned_params) {
  config_.safe_distance_base = tuned_params.safe_distance_base();
  config_.safe_distance_ttc = tuned_params.safe_distance_ttc();
  config_.t_actuator_delay = tuned_params.t_actuator_delay();
  config_.lane_keep_cutinp_threshold =
      tuned_params.lane_keep_cutinp_threshold();
  config_.lane_change_cutinp_threshold =
      tuned_params.lane_change_cutinp_threshold();
  config_.corridor_width = tuned_params.corridor_width();
  config_.preview_x = tuned_params.preview_x();
  config_.dis_zero_speed = tuned_params.dis_zero_speed();
  config_.dis_zero_speed_accident = tuned_params.dis_zero_speed_accident();
  config_.ttc_brake_hysteresis = tuned_params.ttc_brake_hysteresis();
  config_.t_curv = tuned_params.t_curv();
  config_.dis_curv = tuned_params.dis_curv();
  config_.velocity_upper_bound = tuned_params.velocity_upper_bound();
  config_.v_start = tuned_params.v_start();
  config_.distance_stop = tuned_params.distance_stop();
  config_.distance_start = tuned_params.distance_start();
}

}  // namespace scc
}  // namespace planning