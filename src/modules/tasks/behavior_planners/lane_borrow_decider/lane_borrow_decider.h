#pragma once

#include <limits>
#include <vector>

#include "config/vehicle_param.h"
#include "dynamic_world/dynamic_world.h"
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "reference_path.h"
#include "session.h"
#include "task_interface/lane_borrow_decider_output.h"
#include "tasks/task.h"

namespace planning {

class LaneBorrowDecider : public Task {
 public:
  LaneBorrowDecider(const EgoPlanningConfigBuilder* config_builder,
                    framework::Session* session)
      : Task(config_builder, session) {
    vehicle_param_ =
        VehicleConfigurationContext::Instance()->get_vehicle_param();
  };
  virtual ~LaneBorrowDecider() = default;

  bool Execute() override;

 private:
  bool RunLaneBorrowStateMachine();

  bool CheckIfLaneBorrowDriving();
  bool CheckIfDrivingToPassSide();
  bool CheckIfLaneBorrowDrivingToBackDriving();
  bool CheckIfBackDrivingToPassSide();
  bool CheckIfBackDrivingToNoBorrow();

  void UpdateForwardMapInfo();
  void CalcDistanceToSolidLane();
  bool HasBlockingObstacle();
  void UpdateAdcInfo();

  bool IsSafeForLaneBorrow();
  bool IsSafeForBack();

  bool ClearForLaneBorrow(const double ego_speed, const double& left_bounds_l,
                          const double& right_bounds_l);
  void ClearLaneBorrowStatus();
  bool CheckIfBorrowAgain();
  void LogDebugInfo();

 private:
  LaneBorrowStatus lane_borrow_status_{kNoLaneBorrow};
  double junction_start_s_{1000.0};
  double junction_end_s_{1500.0};

  double forward_solid_start_s_{1000.0};
  double forward_solid_end_s_{1500.0};

  double distance_to_stop_line_{1000.0};
  double distance_to_cross_walk_{1000.0};

  double obs_left_l_{-10.0};
  double obs_right_l_{10.0};
  double obs_start_s_{10.0};
  double obs_end_s_{-10.0};

  bool left_borrow_{false};
  bool right_borrow_{false};

  std::pair<double, double> ego_sl_;  // s, l
  double ego_speed_;

  FrenetBoundary ego_frenet_boundary_;
  VehicleParam vehicle_param_;
  LaneBorrowDeciderOutput lane_borrow_decider_output_;

  int observe_frame_num_{0};
  std::vector<int> static_blocked_obj_vec_;

  std::pair<double, double> front_pass_point_;  // static obs area,
  std::pair<double, double> front_pass_sl_point_;
  std::pair<double, double> last_ego_center_position_;
};

}  // namespace planning