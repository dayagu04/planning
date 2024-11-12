#pragma once

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ad_common/math/box2d.h"
#include "config/message_type.h"
#include "pose2d.h"
#include "src/library/collision_detection/aabb2d.h"
#include "src/library/collision_detection/polygon_base.h"
#include "utils/index_list.h"
#include "common_c.h"
#include "pose2d.h"

namespace planning {

struct Limiter {
  Eigen::Vector2d start_pt = Eigen::Vector2d::Zero();
  Eigen::Vector2d end_pt = Eigen::Vector2d::Zero();
  bool valid = false;

  void Reset() {
    start_pt.setZero();
    end_pt.setZero();
    valid = false;
  }
};

struct SlotCoord {
  Eigen::Vector2d pt_1 = Eigen::Vector2d::Zero();       // left up
  Eigen::Vector2d pt_0 = Eigen::Vector2d::Zero();       // right up
  Eigen::Vector2d pt_2 = Eigen::Vector2d::Zero();       // right down
  Eigen::Vector2d pt_3 = Eigen::Vector2d::Zero();       // left_down
  Eigen::Vector2d pt_center = Eigen::Vector2d::Zero();  // center

  void Reset() {
    pt_1.setZero();
    pt_0.setZero();
    pt_2.setZero();
    pt_3.setZero();
    pt_center.setZero();
  }
};

enum class SlotType : uint8_t {
  PERPENDICULAR,
  PARALLEL,
  SLANT,
  COUNT,
  INVALID,
};

enum class SlotSide : uint8_t {
  LEFT,
  RIGHT,
  COUNT,
  INVALID,
};

struct ApaSlot {
  SlotCoord slot_coord;
  SlotType slot_type = SlotType::INVALID;
  SlotSide slot_side = SlotSide::INVALID;
  int slot_id = 0;
  Limiter limiter;
  double heading = 0.0;
  Eigen::Vector2d heading_vec = Eigen::Vector2d::Zero();

  void Reset() {
    slot_coord.Reset();
    slot_type = SlotType::INVALID;
    slot_side = SlotSide::INVALID;
    slot_id = 0;
    limiter.Reset();
    heading = 0.0;
    heading_vec.setZero();
  }
};

struct ApaSlots {
  std::vector<ApaSlot> slots_vec;
  int slot_size = 0;

  void Reset() {
    slots_vec.clear();
    slot_size = 0;
  }
};

enum class SlotReleaseMethod {
  NONE = 0,
  COARSE_RULE_BASED_RELEASE = 1,
  FINE_RULE_BASED_RELEASE = 2,
  GEOMETRY_PLANNING_RELEASE = 3,
  ASTAR_PLANNING_RELEASE = 4,
};

class ParkSlot {
 public:
  ParkSlot() = default;

  void Init();

  int PerceptionId() const { return perception_id_; }

  void SetId(const int id) {
    id_ = id;
    return;
  }

  const int Id() const { return id_; }

 private:
  //  perception
  std::vector<Eigen::Vector2d> perception_corners_;
  SlotType slot_type = SlotType::INVALID;
  SlotSide slot_side = SlotSide::INVALID;

  std::vector<Eigen::Vector2d> perception_limiters_;
  int perception_id_ = 0;

  // 记录车位高度信息
  double height_top_;
  double height_bottom_;
  iflyauto::SlotSourceType perception_source_type_;

  bool is_perception_release_;

  // planning
  // construct lot from 4 corners
  //  corner should be ordered counter-clockwise
  //  and corners[0] and corners[3] is the open edge
  std::vector<Eigen::Vector2d> ccw_corners_;

  std::vector<Eigen::Vector2d> limiters_;
  double heading = 0.0;
  Eigen::Vector2d heading_vec = Eigen::Vector2d::Zero();
  int id_ = 0;
  bool is_planning_release_;
  SlotReleaseMethod release_method_;

  // 后轴中心的停车点，该值由决策器生成
  Pose2D target_pose_;

  double left_obs_dist_;
  double right_obs_dist_;
  double top_obs_dist_;
  double bottom_obs_dist_;

  double ego_occupied_ratio_;

  ad_common::math::LineSegment2d center_line_;
};

typedef IndexedList<int, ParkSlot> IndexedParkLots;

}  // namespace planning
