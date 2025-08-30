#pragma once

#include <memory>
#include "config/basic_type.h"
#include "dynamic_world/dynamic_agent_node.h"
#include "dynamic_world/dynamic_agent_node.h"
#include "library/lc_idm_lib/include/basic_intelligent_driver_model.h"
#include "library/lc_idm_lib/include/longitudinal_motion_simulator_intelligent_driver_model.h"
#include "src/library/lc_pure_pursuit_lib/include/basic_pure_pursuit_model.h"
#include "reference_path.h"
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

    LaneChangePathGenerateManager(
        std::shared_ptr<ReferencePath> ref_path,
        framework::Session* session);

    bool GenerateLCPath(const double lat_offset);
    bool GenerateEgoFutureTrajectory(const double lat_offset);
    bool GenerateEgoFutureTrajectory(const double lat_offset, const planning_data::DynamicAgentNode* front_agent_node);
    double ComputeLd(double v, bool is_close);

     LCPathResult& get_lc_path_result()  {return lc_path_result_;}
    const TrajectoryPoints& get_ego_future_trajectory() const {return ego_future_trajectory_;}
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
    State_Sim UpdateDynamicsOneStep(State_Sim state, double dt);

    bool CalculateFrontAgentPredictionInfo(
        AgentPredictionTrajectoryPoints* agent_prediction_traj_points,
        const planning_data::DynamicAgentNode* front_agent);
    void reset(){
      ego_future_trajectory_.clear();
      lc_path_result_.reset();
    }
  };
}