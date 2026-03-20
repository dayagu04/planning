#pragma once

#include <memory>

#include "behavior_planners/long_ref_path_decider/target_marker/comfort_target.h"
#include "config/basic_type.h"
#include "dynamic_world/dynamic_agent_node.h"
#include "library/lc_idm_lib/include/basic_intelligent_driver_model.h"
#include "library/lc_idm_lib/include/longitudinal_motion_simulator_intelligent_driver_model.h"
#include "reference_path.h"
#include "src/library/lc_pure_pursuit_lib/include/basic_pure_pursuit_model.h"
#include "tasks/behavior_planners/lane_change_decider/lane_change_joint_decision_generator/src/joint_decision_speed_limit.h"
#include "utils/spline.h"

using BasicPurePursuitModel = planning::BasicPurePursuitModel;

namespace planning {
class LaneChangePathGenerateManager {
 public:
  struct State_Sim {
    double x = 0;
    double y = 0;
    double theta = 0;
    double v = 0;
    double delta = 0;
    double omega = 0;
  };

  struct LCPathResult {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> theta;
    std::vector<double> v;
    std::vector<double> t;
    spline x_t_spline;
    spline y_t_spline;
    spline theta_t_spline;

    void reset() {
      x.clear();
      y.clear();
      theta.clear();
      v.clear();
      t.clear();
    }
  };

  struct AgentPredictionTrajectoryPoints {
    std::vector<double> x_vec;
    std::vector<double> y_vec;
    std::vector<double> theta_vec;
    std::vector<double> v_vec;
    std::vector<double> t_vec;
    std::vector<double> s_vec;
    spline v_t_spline;
    spline s_t_spline;

    void Reset() {
      x_vec.clear();
      y_vec.clear();
      theta_vec.clear();
      v_vec.clear();
      s_vec.clear();
      t_vec.clear();
    }
  };
  struct ComfortIdmParameters {
    double v0 = 33.5;
    double s0 = 3.5;
    double T = 0.5;
    double a = 2.0;
    double b_max = 2.0;
    double b = 1.0;
    double b_hard = 4.0;
    double delta = 4.0;
    double max_accel_jerk = 4.0;
    double max_decel_jerk = 1.0;
    double virtual_front_s = 200.0;
    double cool_factor = 0.99;
    double eps = 1e-3;
    double dt_ = 0.2;
  };

  LaneChangePathGenerateManager(std::shared_ptr<ReferencePath> ref_path,
                                framework::Session* session);

  LaneChangePathGenerateManager(std::shared_ptr<ReferencePath> ref_path,
                                framework::Session* session,
                                const EgoPlanningConfigBuilder* config_builder);

  bool GenerateLCPath(const double lat_offset);
  bool GenerateEgoFutureTrajectory(const double lat_offset);
  bool GenerateEgoFutureTrajectory(
      const double lat_offset,
      const planning_data::DynamicAgentNode* front_agent_node);
  double ComputeLd(double v, bool is_close);

  LCPathResult& get_lc_path_result() { return lc_path_result_; }
  const TrajectoryPoints& get_ego_future_trajectory() const {
    return ego_future_trajectory_;
  }

 private:
  LaneChangeDeciderConfig lc_decider_config_;
  LCPathResult lc_path_result_;
  TrajectoryPoints ego_future_trajectory_;
  TrajectoryPoints front_node_future_trajectory_;
  BasicPurePursuitModel pp_model_;
  BasicIntelligentDriverModel idm_model_;
  // LonMotionSimulatorIntelligentDriverModel lon_motion_sim_model_;
  std::shared_ptr<ReferencePath> ref_path_;
  framework::Session* session_;
  ComfortIdmParameters comfort_idm_params_;
  State_Sim UpdateDynamicsOneStep(State_Sim state, double dt);

  bool CalculateFrontAgentPredictionInfo(
      AgentPredictionTrajectoryPoints* agent_prediction_traj_points,
      const planning_data::DynamicAgentNode* front_agent);
  void reset() {
    ego_future_trajectory_.clear();
    lc_path_result_.reset();
  }
  double CalculateComfortAcceleration(const double current_acc,
                                      const double current_vel,
                                      const double current_s,
                                      const double front_vel,
                                      const double front_s, const double tau,
                                      const double v0);
};
}  // namespace planning