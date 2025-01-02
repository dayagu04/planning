#ifndef GROUND_LINE_MANAGER_H
#define GROUND_LINE_MANAGER_H

#include <algorithm>
#include <cmath>
#include <vector>

#include "ehr.pb.h"
#include "fusion_groundline_c.h"
#include "vec2d.h"

namespace planning {

using GroundLinePoints = std::vector<planning_math::Vec2d>;

struct GroundLinePoint {
  enum Status {
    UNCLASSIFIED = 0,
    CLASSIFIED = 1,
    NOISE = 2,
  };

  planning_math::Vec2d point;
  Status status;

  bool operator==(const GroundLinePoint &input) {
    return (this->point == input.point && this->status == input.status);
  }

  bool operator!=(const GroundLinePoint &input) {
    return (!(this->point == input.point) || this->status != input.status);
  }
};

class GroundLineManager {
 public:
  GroundLineManager();
  ~GroundLineManager() = default;

  bool Update(const Map::StaticMap &static_map_info);

  bool Update(const iflyauto::FusionGroundLineInfo &fusion_ground_line_info);

  void UpdateParams(const int min_pts, const double eps);

  void SetIsCluster(bool is_cluster) { is_cluster_ = is_cluster; }

  const std::vector<GroundLinePoints> &GetPoints() const { return points_; };

 private:
  void Init();

  std::vector<std::vector<planning_math::Vec2d>> Execute(
      const std::vector<GroundLinePoint> &ground_line_points);

  void UpdatePoints(const std::vector<GroundLinePoint> &ground_line_points,
                    std::vector<GroundLinePoint> &points);

  std::vector<int> CalcCluster(GroundLinePoint &point,
                               std::vector<GroundLinePoint> &points);

  std::vector<planning_math::Vec2d> ExpandCluster(
      GroundLinePoint &point, std::vector<GroundLinePoint> &points);

 private:
  std::vector<GroundLinePoints> points_;
  int min_pts_;
  double eps_;
  bool is_cluster_;
};

}  // namespace planning

#endif  // GROUND_LINE_DECIDER_H
