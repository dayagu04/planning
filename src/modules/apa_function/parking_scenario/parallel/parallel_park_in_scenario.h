#ifndef __PARALLEL_PARK_IN_PLANNER_H__
#define __PARALLEL_PARK_IN_PLANNER_H__

#include <cstdint>
#include <memory>
#include <vector>

#include "apa_world.h"
#include "dubins_lib.h"
#include "parallel_path_generator.h"
#include "src/modules/apa_function/parking_scenario/parking_scenario.h"

namespace planning {
namespace apa_planner {

class ParallelParkInScenario : public ParkingScenario {
 public:
  ParallelParkInScenario() = default;
  ParallelParkInScenario(const std::shared_ptr<ApaWorld>& apa_world_ptr) {
    SetApaWorldPtr(apa_world_ptr);
  }

  virtual void Reset() override;
  virtual std::string GetName() override { return typeid(this).name(); };
  virtual const bool UpdateEgoSlotInfo() override;
  virtual const bool GenTlane() override;
  virtual const uint8_t PathPlanOnce() override;
  virtual const bool CheckFinished() override;
  virtual const bool GenObstacles() override;
  virtual void ExcutePathPlanningTask() override;
  virtual void Log() const override;

  const double CalcSlotOccupiedRatio(const Eigen::Vector2d& terminal_err,
                                     const double slot_width,
                                     const bool is_right_side) const;
  const double CalcSlotOccupiedRatio(
      const pnc::geometry_lib::PathPoint start_pose) const;

  void GenTBoundaryObstacles();

  const Tlane& GetTlane() { return t_lane_; }

 private:
  // virtual func
  void CalStaticBufferInDiffSteps(double& lat_buffer,
                                  double& safe_uss_remain_dist) const;

  void CalDynamicBufferInDiffSteps(double& dynaminc_lat_buffer,
                                   double& dynamic_lon_buffer) const;

  Tlane t_lane_;
  std::vector<Eigen::Vector2d> obs_pt_local_vec_;
  ParallelPathGenerator parallel_path_planner_;
};

}  // namespace apa_planner
}  // namespace planning

#endif
