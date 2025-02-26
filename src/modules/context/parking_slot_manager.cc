
#include "parking_slot_manager.h"

#include <climits>
#include <cstddef>
#include <limits>

#include "ego_state_manager.h"
#include "environmental_model.h"
#include "log.h"

namespace planning {

const double kMaxDistanceY = 5;
const double kMaxDistanceFrontX = 40;  // 后续根据实际需求更改
const double kMaxDistanceBackX = 30;

ParkingSlotManager::ParkingSlotManager(planning::framework::Session* session)
    : session_(session) {
  Init();
}

void ParkingSlotManager::Init() {
  limiters_.clear();
  points_.clear();
  target_slot_.clear();
  target_slot_id_ = 0;
  distance_to_target_slot_ = NL_NMAX;
  is_reached_target_slot_ = false;
}

bool ParkingSlotManager::Update(const Map::StaticMap& static_map) {
  points_.clear();
  const auto ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto park_spaces = static_map.parking_assist_info().parking_spaces();
  const size_t park_spaces_size = park_spaces.size();
  const auto& enu2car_matrix = ego_state_manager->get_enu2car();
  Eigen::Vector3d v;
  double min_x, min_y, max_x, max_y;
  for (uint8 i = 0; i < park_spaces_size; i++) {
    ParkingSlotPoints slot_point;
    auto park_space = park_spaces[i];
    // TBD: 这里没有判断其size的成员量
    // if (park_space.shape.size != 4) {
    //   LOG_NOTICE("park_space point size is not 4");
    //   continue;
    // }

    min_x = std::numeric_limits<double>::max();
    min_y = std::numeric_limits<double>::max();
    max_x = std::numeric_limits<double>::lowest();
    max_y = std::numeric_limits<double>::lowest();
    for (uint j = 0; j < park_space.shape().size(); j++) {
      // WB: c结构体没有设置size，默认4
      auto lot_point = park_space.shape(j);
      // for (const auto &lot_point : park_space.shape()) {
      v.x() = lot_point.x();
      v.y() = lot_point.y();
      v.z() = lot_point.z();
      auto park_space_point_car = enu2car_matrix(v);
      min_x = std::fmin(min_x, park_space_point_car.x());
      min_y = std::fmin(min_y, park_space_point_car.y());
      max_x = std::fmax(max_x, park_space_point_car.x());
      max_y = std::fmax(max_y, park_space_point_car.y());
      slot_point.emplace_back(
          planning_math::Vec2d(lot_point.x(), lot_point.y()));
    }
    if (((min_y > 0 && min_y < kMaxDistanceY) ||
         (max_y < 0 && max_y > -kMaxDistanceY) || (min_y <= 0 && max_y >= 0)) &&
        min_x < kMaxDistanceFrontX && max_x > -kMaxDistanceBackX) {
      points_.emplace_back(std::move(slot_point));
    }
  }
  return true;
}

bool ParkingSlotManager::Update(
    const iflyauto::ParkingFusionInfo& parking_fusion_info) {
  target_slot_.clear();
  points_.clear();
  limiters_.clear();
  is_exist_target_slot_ = false;
  is_reached_target_slot_ = false;
  const double distance_to_target_slot =
      session_->environmental_model()
              .get_route_info()
              ->get_route_info_output()
              .distance_to_target_slot;
  if (distance_to_target_slot > 20.0) {
    return false;
  }
  const size_t parking_slot_lists_size = parking_fusion_info.parking_fusion_slot_lists_size;
  const auto& parking_slot_lists = parking_fusion_info.parking_fusion_slot_lists;
  target_slot_id_ = parking_fusion_info.select_slot_id;
  for (uint8 i = 0; i < parking_slot_lists_size; i++) {
    ParkingSlotPoints slot_point;
    const auto& parking_slot = parking_slot_lists[i];
    size_t slot_id = parking_slot.id;
    auto resource_type = parking_slot.resource_type;
    if (slot_id == target_slot_id_) {
      for (const auto& corner_point : parking_slot.corner_points) {
        target_slot_.emplace_back(
            planning_math::Vec2d(corner_point.x, corner_point.y));
      }
      planning_math::Polygon2d::ComputeConvexHull(target_slot_,
                                                  &target_slot_polygon_);
      is_exist_target_slot_ = true;
    }

    // limiter
    // const size_t limiters_size = parking_slot.limiters_size;
    // const auto& limiters = parking_slot.limiters;  // line segment
    // for (const auto& limiter : limiters) {
    //   planning_math::LineSegment2d limiter_axis(
    //       planning_math::Vec2d(limiter.end_points[0].x,
    //                            limiter.end_points[0].y),
    //       planning_math::Vec2d(limiter.end_points[1].x,
    //                            limiter.end_points[1].y));
    //   limiters_.emplace_back(std::move(limiter_axis));
    // }
  }
  return true;
}

bool ParkingSlotManager::CalculateDistanceToTargetSlot(
    const std::shared_ptr<ReferencePath> &reference_path) {
  const double distance_to_target_slot =
      session_->environmental_model()
              .get_route_info()
              ->get_route_info_output()
              .distance_to_target_slot;
  distance_to_target_slot_ = distance_to_target_slot;
  if (reference_path != nullptr) {
    const double ego_s = reference_path->get_frenet_ego_state().s();
    const auto& frenet_coord = reference_path->get_frenet_coord();
    const auto& points = reference_path->get_points();
    if (distance_to_target_slot_ < 10.0) {
      distance_to_target_slot_ =
          std::min(std::fabs(points.back().path_point.s() - ego_s),
                  distance_to_target_slot_);
    }
    if ((target_slot_.empty()) ||
        (frenet_coord == nullptr)) {
      return false;
    }
    double slot_dist_to_ego = -NL_NMAX;
    if (target_slot_polygon_.is_convex()) {
      planning_math::Box2d target_slot_box = target_slot_polygon_.MinAreaBoundingBox();
      Point2D cart_pt(target_slot_box.center_x(), target_slot_box.center_y());
      Point2D frenet_pt{0.0, 0.0};
      if(frenet_coord->XYToSL(cart_pt, frenet_pt)) {
        slot_dist_to_ego = std::max(frenet_pt.x - ego_s,
                                    slot_dist_to_ego);
        if (slot_dist_to_ego < 2.0) {
          is_reached_target_slot_ = true;
        }
      } else {
        slot_dist_to_ego = distance_to_target_slot_;
      }
    } else {
      for (const auto& slot_point : target_slot_) {
        Point2D cart_pt(slot_point.x(), slot_point.y());
        Point2D frenet_pt{0.0, 0.0};
        if(frenet_coord->XYToSL(cart_pt, frenet_pt)) {
          slot_dist_to_ego = std::max(frenet_pt.x - ego_s,
                                      slot_dist_to_ego);
          if (slot_dist_to_ego < 2.0) {
            is_reached_target_slot_ = true;
          }
        } else {
          slot_dist_to_ego = distance_to_target_slot_;
          break;
        }
      }
    }
    distance_to_target_slot_ = std::fabs(slot_dist_to_ego);
    // distance_to_target_slot_ = std::min(std::fabs(slot_dist_to_ego), distance_to_target_slot_);
  }
  return true;
}
}  // namespace planning
