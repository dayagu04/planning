
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <vector>
#include "basic_types.pb.h"
#include "config/basic_type.h"
#include "config/message_type.h"
#include "ego_planning_config.h"
#include "ego_state_manager.h"
#include "reference_path.h"
#include "speed/st_point.h"
#include "src/modules/common/math/curve1d/quintic_polynomial_curve1d.h"
#include "task.h"
#include "virtual_lane.h"
#include "speed/st_boundary.h"
namespace planning {
class DPSpeedGraph : public Task {
 public:
  virtual ~DPSpeedGraph() = default;
  DPSpeedGraph(const EgoPlanningConfigBuilder *config_builder,
              framework::Session *session)
      : Task(config_builder, session) {
    config_ = config_builder->cast<DPSpeedGraphConfig>();
  };

 private:
  DPSpeedGraphConfig config_;
  std::shared_ptr<VirtualLane> current_lane_ptr_ = nullptr;
  std::shared_ptr<ReferencePath> current_reference_path_ptr_ = nullptr;
  std::shared_ptr<VirtualLane> right_lane_ptr_ = nullptr;
  std::shared_ptr<VirtualLane> left_lane_ptr_ = nullptr;
  FrenetEgoState ego_frenet_state_;
  FrenetBoundary ego_frenet_boundary_;
  std::vector<planning::common::PathPoint> dp_path_points;



 public:
  bool Execute() override;
  bool ProcessEnvInfos();
};
};  // namespace planning