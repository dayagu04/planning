#pragma once
#include "avoid_obstacle_maintainer.h"
#include "lateral_offset_calculator.h"
#include "session.h"
#include "tasks/task.h"

namespace planning {

class LateralOffsetDecider : public Task {
 public:
  explicit LateralOffsetDecider(const EgoPlanningConfigBuilder *config_builder,
                                framework::Session *session);

  virtual ~LateralOffsetDecider() = default;

  bool Execute() override;

  bool ExecuteTest(bool pipeline_test);

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
