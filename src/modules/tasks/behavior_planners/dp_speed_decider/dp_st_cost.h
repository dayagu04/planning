#include <vector>
#include "basic_types.pb.h"
#include "ego_planning_config.h"
#include "frenet_obstacle.h"
#include "speed/st_point.h"
namespace planning {
class DPSTCost {
 public:
  explicit DPSTCost(const DPSpeedGraphConfig& dp_speed_config,
                    const std::vector<const FrenetObstacle*>& obstacles,
                    const planning::common::TrajectoryPoint& init_point);
  double GetReferenceCost(const STPoint& point,
                          const STPoint& reference_point) const;
  double GetObstacleCost(const STPoint& point) const;

  double GetSpeedCost(const STPoint& first, const STPoint& second,
                      const double speed_limit) const;
  double GetAccelCostByTwo(const double pre_speed, const STPoint& first,
                           const STPoint& second) const;
  double GetAccelCostByThree(const STPoint& first, const STPoint& second,
                             const STPoint& third) const;
  double GetJerkCostByTwo(const STPoint& pred_point, const double pre_speed,
                          const double pre_acc, const STPoint& cur_point) const;
  double GetJerkCostByThree(const STPoint& first_point,
                            const double first_speed,
                            const STPoint& second_point,
                            const STPoint& third_point) const;
  double GetJerkCostByFour(const STPoint& first, const STPoint& second,
                           const STPoint& third, const STPoint& forth) const;
  double GetAccelCost(const double accel) const;
  double GeJerkCost(const double jerk) const;

 private:
  const DPSpeedGraphConfig& dp_speed_config_;
  const std::vector<const FrenetObstacle*>& obstacles_;
  const planning::common::TrajectoryPoint init_point_;
  double unit_s_ = 0.0;
  double unit_t_ = 0.0;
  double unit_v_ = 0.0;
};

}  // namespace planning