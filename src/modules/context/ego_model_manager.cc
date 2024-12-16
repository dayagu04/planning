#include "ego_model_manager.h"

#include "vehicle_config_context.h"

namespace planning {
namespace planning_math {
EgoModelManager::EgoModelManager() {
  ego_model_type_ = EgoModelType::ORIGIN;
  origin_model_points_.resize(4);
  ego_model_polygon_ = planning_math::Polygon2d();
  set_params(0);
  set_expansion(0, 0);
  set_origin_model();
}

void EgoModelManager::set_params(const double deviation_length) {
  deviation_length_ = deviation_length;
}

void EgoModelManager::set_model_type(EgoModelType ego_model_type) {
  ego_model_type_ = ego_model_type;
}

void EgoModelManager::set_expansion(double lat_expansion,
                                    double lon_expansion) {
  lat_expansion_ = lat_expansion;
  lon_expansion_ = lon_expansion;
}

EgoModelType EgoModelManager::get_ego_model_type() { return ego_model_type_; }

const std::vector<planning_math::Vec2d> &EgoModelManager::get_ego_model_points()
    const {
  return origin_model_points_;
}

const planning_math::Polygon2d &EgoModelManager::get_ego_model_polygon() const {
  return ego_model_polygon_;
}

void EgoModelManager::set_model_center(const planning_math::PathPoint &point) {
  point_ = point;
}

// NOTE: origin model is constructed base on the geometry center of the vehicle.
// Now deviation_length_ is the distance from the geometry center to the rear
// axle if point_ is central point of vehicle rear axle
void EgoModelManager::set_origin_model(double lat_expansion,
                                       double lon_expansion) {
  ego_model_type_ = EgoModelType::ORIGIN;
  double center_x = point_.x() + cos(point_.theta()) * deviation_length_;
  double center_y = point_.y() + sin(point_.theta()) * deviation_length_;
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  //   double dx1 =
  //       cos(point_.theta()) *
  //       (planner::ConfigurationContext::Instance()->get_vehicle_param().length
  //       /
  //        2.0);
  //   double dy1 =
  //       sin(point_.theta()) *
  //       (planner::ConfigurationContext::Instance()->get_vehicle_param().length
  //       /
  //        2.0);
  //   double dx2 =
  //       sin(point_.theta()) *
  //       (planner::ConfigurationContext::Instance()->get_vehicle_param().width
  //       /
  //            2.0 +
  //        lat_expansion_);
  //   double dy2 =
  //       -cos(point_.theta()) *
  //       (planner::ConfigurationContext::Instance()->get_vehicle_param().width
  //       /
  //            2.0 +
  //        lat_expansion_);
  double dx1 = cos(point_.theta()) * (vehicle_param.length / 2.0);
  double dy1 = sin(point_.theta()) * (vehicle_param.length / 2.0);
  double dx2 =
      sin(point_.theta()) * (vehicle_param.width / 2.0 + lat_expansion_);
  double dy2 =
      -cos(point_.theta()) * (vehicle_param.width / 2.0 + lat_expansion_);
  origin_model_points_[0].set_x(center_x + dx1 + dx2 +
                                cos(point_.theta()) * lon_expansion_);
  origin_model_points_[0].set_y(center_y + dy1 + dy2 +
                                sin(point_.theta()) * lon_expansion_);
  origin_model_points_[1].set_x(center_x + dx1 - dx2 +
                                cos(point_.theta()) * lon_expansion_);
  origin_model_points_[1].set_y(center_y + dy1 - dy2 +
                                sin(point_.theta()) * lon_expansion_);
  origin_model_points_[2].set_x(center_x - dx1 - dx2);
  origin_model_points_[2].set_y(center_y - dy1 - dy2);
  origin_model_points_[3].set_x(center_x - dx1 + dx2);
  origin_model_points_[3].set_y(center_y - dy1 + dy2);
  ego_model_polygon_.set_points(origin_model_points_);
}

}  // namespace planning_math

}  // namespace planning