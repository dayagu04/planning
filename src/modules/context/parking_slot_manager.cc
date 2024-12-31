
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

ParkingSlotManager::ParkingSlotManager(planning::framework::Session *session)
    : session_(session) {
  Init();
}

void ParkingSlotManager::Init() {
  limiters_.clear();
  points_.clear();
  target_slot_.clear();
  target_slot_id_ = 0;
  distance_to_target_slot_ = NL_NMAX;
}

bool ParkingSlotManager::Update(const Map::StaticMap &static_map) {
  points_.clear();
  const auto ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto park_spaces = static_map.parking_assist_info().parking_spaces();
  const size_t park_spaces_size = park_spaces.size();
  const auto &enu2car_matrix = ego_state_manager->get_enu2car();
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

bool ParkingSlotManager::Update(const iflyauto::ParkingFusionInfo &parking_fusion_info) {
  target_slot_.clear();
  points_.clear();
  limiters_.clear();
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const size_t parking_slot_lists_size = parking_fusion_info.parking_fusion_slot_lists_size;
  const auto& parking_slot_lists = parking_fusion_info.parking_fusion_slot_lists;
  target_slot_id_ = parking_fusion_info.select_slot_id;
  const auto &enu2car_matrix = ego_state_manager->get_enu2car();
  Eigen::Vector3d v;
  double min_x, min_y, max_x, max_y;
  for (uint8 i = 0; i < parking_slot_lists_size; i++) {
    ParkingSlotPoints slot_point;
    const auto& parking_slot = parking_slot_lists[i];
    bool is_exist_target_slot = false;
    size_t slot_id = parking_slot.id;
    auto resource_type = parking_slot.resource_type;
    if ((slot_id == target_slot_id_) && (resource_type == 2)) {
      is_exist_target_slot = true;
    }

    // TBD: 这里没有判断size, c结构体没有设置size，默认4
    min_x = std::numeric_limits<double>::max();
    min_y = std::numeric_limits<double>::max();
    max_x = std::numeric_limits<double>::lowest();
    max_y = std::numeric_limits<double>::lowest();
    for (const auto& corner_point : parking_slot.corner_points) {
      v.x() = corner_point.x;
      v.y() = corner_point.y;
      v.z() = 0;
      auto park_space_point_car = enu2car_matrix(v);
      min_x = std::fmin(min_x, park_space_point_car.x());
      min_y = std::fmin(min_y, park_space_point_car.y());
      max_x = std::fmax(max_x, park_space_point_car.x());
      max_y = std::fmax(max_y, park_space_point_car.y());
      slot_point.emplace_back(
          planning_math::Vec2d(corner_point.x, corner_point.y));
      if (is_exist_target_slot) {
        target_slot_.emplace_back(
          planning_math::Vec2d(corner_point.x, corner_point.y));
      }
    }
    if (((min_y > 0 && min_y < kMaxDistanceY) ||
         (max_y < 0 && max_y > -kMaxDistanceY) || (min_y <= 0 && max_y >= 0)) &&
        min_x < kMaxDistanceFrontX && max_x > -kMaxDistanceBackX) {
      points_.emplace_back(std::move(slot_point));
    }
    // limiter
    const size_t limiters_size = parking_slot.limiters_size;
    const auto& limiters = parking_slot.limiters;  // line segment
    for (const auto& limiter : limiters) {
      planning_math::LineSegment2d limiter_axis(
        planning_math::Vec2d(limiter.end_points[0].x,
                             limiter.end_points[0].y),
        planning_math::Vec2d(limiter.end_points[1].x,
                             limiter.end_points[1].y));
      limiters_.emplace_back(std::move(limiter_axis));
    }
  }
  return true;
}

bool ParkingSlotManager::CalculateDistanceToTargetSlot(
    const std::shared_ptr<ReferencePath> &reference_path) {
  distance_to_target_slot_ = -1;
  const double distance_to_target_slot =
      session_->environmental_model()
              .get_route_info()
              ->get_route_info_output()
              .distance_to_target_slot;
  const double ego_s = reference_path->get_frenet_ego_state().s();
  const auto& frenet_coord =
      reference_path->get_frenet_coord();
  if ((target_slot_.empty()) || (frenet_coord == nullptr)) {
    distance_to_target_slot_ = distance_to_target_slot;
    return false;
  }
  for (const auto& slot_point : target_slot_) {
    Point2D cart_pt(slot_point.x(), slot_point.y());
    Point2D frenet_pt{0.0, 0.0};
    if(frenet_coord->XYToSL(cart_pt, frenet_pt)) {
      distance_to_target_slot_ = std::max(std::fabs(frenet_pt.x - ego_s),
                                          distance_to_target_slot_);
    } else {
      distance_to_target_slot_ = distance_to_target_slot;
      break;
    }
  }

  const auto& points = reference_path->get_points();
  if (distance_to_target_slot_ < 10.0) {
    distance_to_target_slot_ = std::min(
     std::fabs(points.back().path_point.s() - ego_s),
     distance_to_target_slot_);
  }
  return true;
}
}  // namespace planning
