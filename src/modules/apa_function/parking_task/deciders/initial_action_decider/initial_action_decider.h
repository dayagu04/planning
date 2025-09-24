#pragma once

#include <memory>
#include <vector>

#include "collision_detector_interface.h"
#include "geometry_math.h"
#include "hybrid_astar_context.h"
#include "parking_task.h"
namespace planning {
namespace apa_planner {

class InitalActionDecider final : public ParkingTask {
 public:
  InitalActionDecider() = default;
  InitalActionDecider(const std::shared_ptr<CollisionDetectorInterface>&
                          col_det_interface_ptr) {
    col_det_interface_ptr_ = col_det_interface_ptr;
  }
  ~InitalActionDecider() = default;

  void Process(InitalActionRequest& inital_action_request,
               const geometry_lib::PathPoint& ego_pose,
               const geometry_lib::LineSegment& tar_line,
               const double node_step, const double sample_dist,
               const double max_safe_dist);

 private:
  void GeneratePath(const geometry_lib::PathPoint& ego_pose,
                    const double sample_dist, const double traj_length,
                    const bool is_forward, const double turn_radius,
                    const bool is_straight,
                    std::vector<geometry_lib::PathPoint>& path);

  const double GenerateRefPathLength(const geometry_lib::PathPoint& ego_pose,
                                     const double sample_dist,
                                     const double traj_length,
                                     const bool is_forward);
};

}  // namespace apa_planner
}  // namespace planning