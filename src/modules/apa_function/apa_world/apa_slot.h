#pragma once

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ad_common/math/box2d.h"
#include "config/message_type.h"
#include "pose2d.h"
#include "src/library/convex_collision_detection/aabb2d.h"
#include "src/library/convex_collision_detection/polygon_base.h"
#include "utils/index_list.h"
#include "common_c.h"
#include "pose2d.h"
#include "slot_management_info.pb.h"

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

enum SlotReleaseMethod : uint8_t {
  RULE_BASED_RELEASE = 0,
  GEOMETRY_PLANNING_RELEASE = 1,
  ASTAR_PLANNING_RELEASE = 2,
  SLOT_RELEASE_METHOD_MAX_NUM = 3,
};

enum class SlotReleaseState : uint8_t {
  // 不经过计算，不确定是否释放
  UNKOWN = 0,
  RELEASE = 1,
  NOT_RELEASE = 2,
};

// todo:每一个车位都应该有SlotReleaseInfo，但是暂时只有ego_slot_info选中的车位有.
struct SlotReleaseInfo {
  SlotReleaseState release_state[4];

  // 重新巡库时，重置;
  // 重新选择车位后，也要重置；
  // 同一个车位巡库中、泊车中不重置.
  void Clear() {
    for (int i = 0; i < 4; i++) {
      release_state[i] = SlotReleaseState::UNKOWN;
    }
  }
};

struct SlotObstacleInfo {
  bool is_occupied_by_obs;

  double left_side_obs_dist;
  double right_side_obs_dist;
  double top_side_obs_dist;
  double bottom_side_obs_dist;

  void Clear() {
    is_occupied_by_obs = false;

    left_side_obs_dist = 0;
    right_side_obs_dist = 0;
    top_side_obs_dist = 0;
    bottom_side_obs_dist = 0;
    return;
  }
};

enum class SlotMaterialType : uint8_t {
  UNKOWN = 0,
  SLOT_LINE = 1,
  // 机械车位
  MECHANICAL = 2,
  // 空间车位
  FREEDOM_SPACE = 3,
};

// todo: 统一所有和车位相关的信息,现在有:
// iflyauto::ParkingFusionSlot,common::SlotInfo,SlotManagementInfo,
// SlotInfoWindow. 可以统一成一种类型.
class ApaSlot {
 public:
  ApaSlot();

  void Init();

  int PerceptionId() const { return perception_id_; }

  void SetId(const int id) {
    id_ = id;
    return;
  }

  const int Id() const { return id_; }

 private:
  //  perception related：客观属性
  std::vector<Eigen::Vector2d> perception_corners_;
  SlotType slot_type = SlotType::INVALID;
  SlotSide slot_side = SlotSide::INVALID;
  std::vector<Eigen::Vector2d> perception_limiters_;
  int perception_id_ = 0;
  // 记录车位高度信息, 对于机械车位，判定底部高度和顶部高度
  double height_top_;
  double height_bottom_;
  iflyauto::SlotSourceType perception_source_type_;
  bool is_perception_release_;
  SlotMaterialType slot_material_type_;
  bool is_selected_slot_;

  // planning related: 客观属性
  // construct lot from 4 corners
  std::vector<Eigen::Vector2d> corners_;
  Limiter limiters_;
  double heading_;
  Eigen::Vector2d heading_vec_;
  int id_ = 0;
  double ego_occupied_ratio_;
  ad_common::math::LineSegment2d center_line_;
  SlotObstacleInfo obs_info_;

  // planning related: 决策属性
  bool is_planning_release_;
  SlotReleaseInfo release_info_;
  bool is_suggested_slot_;
  // todo: add advised parking directory: left/middle/right out, head/tail in

  // 后轴中心的停车点，该值由决策器生成
  Pose2D stop_pose_;
};

struct ApaSlotList {
  std::vector<ApaSlot> slots_vec;
  int slot_size = 0;

  void Reset() {
    slots_vec.clear();
    slot_size = 0;
  }
};

typedef IndexedList<int, ApaSlot> IndexedParkSlots;

}  // namespace planning
