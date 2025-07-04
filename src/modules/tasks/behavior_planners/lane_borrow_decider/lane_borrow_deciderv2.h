#pragma once

#include <limits>
#include <memory>
#include <vector>

#include <unordered_map>
#include "agent/agent.h"
#include "config/vehicle_param.h"
#include "define/geometry.h"
#include "dynamic_world/dynamic_world.h"
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "frenet_obstacle.h"
#include "pose2d.h"
#include "reference_path.h"
#include "session.h"
#include "task_interface/lane_borrow_decider_output.h"
#include "tasks/behavior_planners/dp_path_decider/dp_road_graph.h"
#include "tasks/task.h"
#include "virtual_lane.h"
namespace planning {
namespace lane_borrow_deciderV2 {

class LaneBorrowDecider : public Task {
 public:
  LaneBorrowDecider(const EgoPlanningConfigBuilder* config_builder,
                    framework::Session* session)
      : Task(config_builder, session) {
    config_ = config_builder->cast<LaneBorrowDeciderConfig>();
    dp_path_decider_ = std::make_unique<DPRoadGraph>(config_builder, session);
  };
  virtual ~LaneBorrowDecider() = default;

  bool Execute() override;

 private:
  bool ProcessEnvInfos();
  bool ProcessAllEnvInfos();
  void LaneTypeDistanceInfo();
  void UpdateToDP();
  void LogDebugInfo();
  bool DPDecision();
  bool LaneBorrowPreCheck();
  bool RunDP();
  bool CheckIfNoBorrowToDPLaneBorrowDriving();
  bool CheckIfDPLaneBorrowToNoBorrow();
  bool CheckIfDPLaneBorrowToDPLaneBorrowCrossing();
  bool CheckIfDPLaneBorrowCrossingToNoBorrow();
  bool IsDPSafeForBackOriginLane();
  bool CheckIfBackOriginLaneToNoBorrow();
  bool CheckIfBackOriginLaneToLaneBorrowDriving();
  bool CheckIfBackOriginLaneToLaneBorrowCrossing();

  bool CheckLaneBorrowCondition();
  void UpdateJunctionInfo();
  bool IsLaneTypeDashedOrMixed(const iflyauto::LaneBoundaryType& type);
  bool UpdateLaneBorrowDirection();
  bool SelectStaticBlockingObstcales();
  bool ObstacleDecision();
  bool IfChangeTargetLane();
  void CheckKeyObstaclesIntention(const agent::Agent* agent, bool& is_cut_in,
                                  bool& is_cut_out);
  bool CheckLeadObs();
  bool UpdateDynamicBlockingObstacles();
  BorrowDirection GetBypassDirection(
      const FrenetObstacleBoundary& frenet_obstacle_sl, const int obs_id);

  bool CheckLaneBorrowDircetion();
  bool CrossingPositionJudgment();
  Point2D CartesianRotation(const Point2D& Cartesian_point,
                            double heading_angle, double ego_x, double ego_y);

  void ClearLaneBorrowStatus();
  bool CheckBackWardObs();
  bool IsNeedResetObserve(LaneBorrowFailedReason reason);

 private:
  LaneBorrowStatus lane_borrow_status_{kNoLaneBorrow};
  double forward_solid_start_dis_{1000.0};
  double forward_solid_end_s_{1500.0};

  double distance_to_stop_line_{1000.0};
  double distance_to_cross_walk_{1000.0};
  double dis_to_traffic_lights_{10000.0};
  double obs_left_l_{-10.0};
  double obs_right_l_{10.0};
  double obs_start_s_{10.0};
  double obs_end_s_{-10.0};

  bool left_borrow_{false};
  bool right_borrow_{false};
  bool is_first_frame_to_lane_borrow_{false};
  bool is_facility_{false};
  BorrowDirection bypass_direction_{NO_BORROW};
  BorrowDirection dp_path_direction_{NO_BORROW};
  planning::common::IntersectionState intersection_state_ =
      planning::common::NO_INTERSECTION;

  std::pair<double, double> ego_sl_;  // s, l
  FrenetEgoState ego_sl_state_;
  double ego_speed_;
  int64_t front_id_{0};

  FrenetBoundary ego_frenet_boundary_;
  LaneBorrowDeciderOutput lane_borrow_decider_output_;
  double heading_angle_{0.0};
  Pose2D ego_xy_;

  int observe_frame_num_{0};
  int dp_observe_frame_num_{0};
  std::unordered_map<int, std::pair<BorrowDirection, int>> obs_direction_map_;
  int lane_change_state_{0};

  std::vector<int> static_blocked_obj_id_vec_;  // after decision
  std::vector<int> last_static_blocked_obj_id_vec_;
  std::vector<std::shared_ptr<FrenetObstacle>> static_blocked_obstacles_;
  std::shared_ptr<ReferencePath> current_reference_path_ptr_ = nullptr;
  std::shared_ptr<VirtualLane> current_lane_ptr_ = nullptr;
  std::shared_ptr<VirtualLane> left_lane_ptr_ = nullptr;
  std::shared_ptr<VirtualLane> right_lane_ptr_ = nullptr;
  LaneBorrowDeciderConfig config_;
  std::unique_ptr<DPRoadGraph> dp_path_decider_;
};
}  // namespace lane_borrow_deciderV2
}  // namespace planning