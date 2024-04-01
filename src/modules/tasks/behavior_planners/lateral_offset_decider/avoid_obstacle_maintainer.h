#pragma once
#include <array>
#include <vector>
#include "config/basic_type.h"
#include "frame.h"
#include "task_basic_types.h"
#include "task_pipeline_context.h"

namespace planning {
class AvoidObstacleMaintainer {
 public:
  AvoidObstacleMaintainer() = default;
  ~AvoidObstacleMaintainer() = default;

  bool Process(planning::framework::Frame *frame,
               std::shared_ptr<TaskPipelineContext> &pipeline_context);
  const std::array<std::vector<double>, 2> &avd_car_past() const {
    return avd_car_past_;
  };
  const std::array<std::vector<double>, 2> &avd_sp_car_past() const {
    return avd_sp_car_past_;
  }
  double dist_rblane() const { return dist_rblane_; }
  bool flag_avd() const { return flag_avd_; }

 private:
  double UpdateAntsidesStrict();
  bool UpdateLFrontAvdsInfo(bool no_near_car);
  bool UpdateRFrontAvdsInfo(bool no_near_car);
  bool UpdateLSideAvdsInfo(bool no_near_car);
  bool UpdateRSideAvdsInfo(bool no_near_car);

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
  std::array<std::vector<double>, 2> avd_sp_car_past_;
};
}  // namespace planning