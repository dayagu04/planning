
#include "parking_slot_manager.h"

#include <climits>
#include <cstddef>
#include <limits>

#include "ego_state_manager.h"
#include "environmental_model.h"
#include "obstacle_manager.h"
#include "planning_context.h"

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
  select_slot_id_ = 0;
  memory_slot_id_ = 0;
  target_slot_id_ = 0;
  nearest_slot_id_ = 0;
  is_reached_target_slot_ = false;
  is_reached_target_dest_ = false;
  is_exist_select_slot_ = false;
  is_exist_memory_slot_ = false;
  is_exist_target_slot_ = false;
  is_exist_nearest_slot_ = false;
  distance_to_nearest_slot_ = NL_NMAX;
}

bool ParkingSlotManager::Update(const Map::StaticMap& static_map) {
  points_.clear();
  const auto ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto& enu2car_matrix = ego_state_manager->get_enu2car();
  for(const auto& parking_floor_info: static_map.parking_floor_infos()) {
    const auto park_spaces =
        parking_floor_info.parking_assist_info().parking_spaces();
    const size_t park_spaces_size = park_spaces.size();
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
           (max_y < 0 && max_y > -kMaxDistanceY) ||
           (min_y <= 0 && max_y >= 0)) &&
          min_x < kMaxDistanceFrontX && max_x > -kMaxDistanceBackX) {
        points_.emplace_back(std::move(slot_point));
      }
    }
  }
  return true;
}

bool ParkingSlotManager::Update(
    const iflyauto::ParkingFusionInfo& parking_fusion_info) {
  target_slot_.clear();
  points_.clear();
  limiters_.clear();
  is_exist_select_slot_ = false;
  is_exist_memory_slot_ = false;
  is_exist_target_slot_ = false;
  is_reached_target_slot_ = false;//TODO
  const double distance_to_target_dest = session_->environmental_model()
                                             .get_route_info()
                                             ->get_route_info_output()
                                              .hpp_route_info_output.distance_to_target_dest;
  if (std::fabs(distance_to_target_dest) > 20.0) {
    return false;
  }
  const size_t parking_slot_lists_size =
      parking_fusion_info.parking_fusion_slot_lists_size;
  const auto& parking_slot_lists =
      parking_fusion_info.parking_fusion_slot_lists;
  for (uint8 i = 0; i < parking_slot_lists_size; i++) {
    ParkingSlotPoints slot_point;
    const auto& parking_slot = parking_slot_lists[i];
    size_t slot_id = parking_slot.id;
    auto resource_type = parking_slot.resource_type;
    for (const auto& corner_point : parking_slot.corner_points) {
      slot_point.emplace_back(
          planning_math::Vec2d(corner_point.x, corner_point.y));
    }
    points_.push_back(slot_point);

    bool curr_slot_is_selected_slot = false;
    bool curr_slot_is_memorized_slot = false;
    if (slot_id == parking_fusion_info.select_slot_id) {
      select_slot_id_ = parking_fusion_info.select_slot_id;
      is_exist_select_slot_ = true;
      curr_slot_is_selected_slot = true;
    } else if(slot_id == parking_fusion_info.memorized_slot_id) {
      memory_slot_id_ = parking_fusion_info.memorized_slot_id;
      is_exist_memory_slot_ = true;
      curr_slot_is_memorized_slot = true;
    }
    if((curr_slot_is_memorized_slot && !is_exist_target_slot_) || curr_slot_is_selected_slot) {
      is_exist_target_slot_ = true;
      target_slot_id_ = slot_id;
      // 0 和 1 为车位入口点
      double target_x = (parking_slot.corner_points[0].x + parking_slot.corner_points[1].x) / 2.0;
      double target_y = (parking_slot.corner_points[0].y + parking_slot.corner_points[1].y) / 2.0;
      target_slot_center_ = planning_math::Vec2d(target_x, target_y);
      target_slot_ = slot_point;
    }
  }
  return true;
}

bool ParkingSlotManager::CalculateDistanceToNearestSlot(
    const std::shared_ptr<ReferencePath>& reference_path) {
  const size_t successful_slot_info_list_size =
      session_->planning_context()
          .planning_output()
          .successful_slot_info_list_size;
  const auto& successful_slot_info_list =
      session_->planning_context().planning_output().successful_slot_info_list;
  const auto& parking_slots = session_->environmental_model()
                                  .get_obstacle_manager()
                                  ->get_parking_space();
  const auto& planning_init_point =
      reference_path->get_frenet_ego_state().planning_init_point();
  const auto& frenet_coord = reference_path->get_frenet_coord();
  if (successful_slot_info_list_size > 0) {
    double dist_to_nearest_slot = NL_NMAX;
    double nearest_slot_width = 3.0;
    double dist_buffer = std::max(1.5 * planning_init_point.v, 0.0);
    if (planning_init_point.v < 0.1) {
      dist_buffer = 0;
    }
    size_t last_nearest_slot_id = is_exist_nearest_slot_ ? nearest_slot_id_ : 0;
    for (size_t i = 0; i < successful_slot_info_list_size; ++i) {
      auto iter =
          parking_slots.Find((successful_slot_info_list[i].id + 6000000));
      if (iter != nullptr) {
        Point2D cart_point, frenet_point;
        cart_point.x = iter->x_center();
        cart_point.y = iter->y_center();
        if (frenet_coord != nullptr) {
          if (frenet_coord->XYToSL(cart_point, frenet_point)) {
            double s_diff = frenet_point.x - planning_init_point.frenet_state.s;
            if (iter->id() == last_nearest_slot_id) {
              is_exist_nearest_slot_ = true;
              nearest_slot_id_ = iter->id();
              distance_to_nearest_slot_ = std::max(s_diff, dist_buffer);
              return true;
            }
            if (s_diff - dist_buffer <= -0.5 * iter->width()) {
              continue;
            }
            double s_dist = std::fabs(s_diff - dist_buffer);
            if (s_dist < dist_to_nearest_slot) {
              dist_to_nearest_slot = s_dist;
              is_exist_nearest_slot_ = true;
              nearest_slot_id_ = iter->id();
              distance_to_nearest_slot_ = std::max(s_diff, dist_buffer);
            }
          }
        }
      }
    }
    if (is_exist_nearest_slot_) {
      return true;
    }
  }
  is_exist_nearest_slot_ = false;
  nearest_slot_id_ = 0;
  distance_to_nearest_slot_ = NL_NMAX;
  return false;
}

}  // namespace planning
