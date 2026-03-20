#pragma once
#include <iostream>
#include <vector>

#include "common/math/polygon2d.h"
#include "src/common/vec2d.h"
#include "src/modules/common/utils/path_point.h"

namespace planning {
namespace planning_math {
enum class EgoModelType : int {
  ORIGIN = 0,
  DOUBLE_BOX = 1,    // wider in front, thinner in rear
  POLYGON = 2,       // same shape as double box using polygon
  DECAGON = 3,       // use trapezoid to replace rounded corner in front
  TETRADECAGON = 4,  // based on decagon, extract mirror
  RECTANGLE_HEXAGON =
      5,         // same shape as tetradecagon, use two overlapped shape
  HEXAGON = 6,   // rounded front corner
  PENTAGON = 7,  // bullet-shaped
};

class EgoModelManager {
 public:
  EgoModelManager();
  ~EgoModelManager() = default;
  void set_origin_model(double lat_expansion = 0.0, double lon_expansion = 0.0);
  bool set_ego_model();
  void set_params(const double deviation_length);
  void set_expansion(double lat_expansion = 0.0, double lon_expansion = 0.0);
  void set_model_type(EgoModelType ego_model_type);
  void set_model_center(const planning_math::PathPoint &point);
  EgoModelType get_ego_model_type();
  const std::vector<planning_math::Vec2d> &get_ego_model_points() const;
  const planning_math::Polygon2d &get_ego_model_polygon() const;

 private:
  EgoModelType ego_model_type_;
  std::vector<planning_math::Vec2d> origin_model_points_;
  planning_math::Polygon2d ego_model_polygon_;

  double deviation_length_;
  double lat_expansion_;
  double lon_expansion_;
  planning_math::PathPoint point_;
};
}  // namespace planning_math

}  // namespace planning