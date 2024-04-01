#pragma once
#include "avoid_obstacle_maintainer.h"
#include "frame.h"
#include "lateral_offset_calculator.h"

namespace planning {

class LateralOffsetDecider : public Task {
 public:
  explicit LateralOffsetDecider(
      const EgoPlanningConfigBuilder *config_builder,
      const std::shared_ptr<TaskPipelineContext> &pipeline_context);

  virtual ~LateralOffsetDecider() = default;

  bool Execute(planning::framework::Frame *frame) override;

  bool ExecuteTest(planning::framework::Frame *frame, bool pipeline_test);

  // double lat_offset() const {
  //   return lat_offset_;
  // }
 private:
  LateralOffsetDeciderConfig config_;
  AvoidObstacleMaintainer avoid_obstacle_maintainer_;
  LateralOffsetCalculator lateral_offset_calculator_;

  // double lat_offset_;
};

}  // namespace planning
