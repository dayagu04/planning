#pragma once

#include <assert.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "tasks/task.h"
#include "tasks/task_basic_types.h"
#include "common/config/basic_type.h"
#include "context/reference_path.h"
#include "context/lateral_obstacle.h" // TODO(Rui):include object_selector.h

namespace planning {

class VisionLateralBehaviorPlanner : public Task {
  public:
  explicit VisionLateralBehaviorPlanner(
      const EgoPlanningConfigBuilder *config_builder,
      const std::shared_ptr<TaskPipelineContext> &pipeline_context);

  virtual ~VisionLateralBehaviorPlanner() = default;

  bool Execute(planning::framework::Frame *frame) override;

 private:
  bool Process(const CoarsePlanningInfo &coarse_planning_info,
               LateralAvdCarsInfo &lateral_avd_cars_info);

  double update_antsides_strict();
  bool update_lfrontavds_info(bool no_near_car);
  bool update_rfrontavds_info(bool no_near_car);
  bool update_lsideavds_info(bool no_near_car);
  bool update_rsideavds_info(bool no_near_car);
  void update_avoid_cars(const CoarsePlanningInfo &coarse_planning_info,
                         LateralAvdCarsInfo &lateral_avd_cars_info);

  void update_lside_svsp_info();
  void update_rside_svsp_info();

  bool update_lateral_behavior_planner_output() {}

 private:
  VisionLateralBehaviorPlannerConfig config_;
  planning::framework::Frame *frame_;
  bool no_sp_car_ = true;

  bool is_ncar_ = false;
  double t_avd_car_ = 3.0;
  double t_avd_sp_car_ = 3.0;
  double final_y_rel_ = 10;

  int ncar_change_ = 0;
  int lbcar_change_ = 0;
  int flag_avd_ = 0;
  int avd_back_cnt_ = 0;
  int avd_leadone_ = 0;
  int pre_leadone_id_ = 0;

  int intersection_cnt_ = 0;
  double dist_rblane_ = 0;
  double lane_width_ = 0.;

  std::array<std::vector<double>, 2> avd_car_past_;

};

}  // namespace planning