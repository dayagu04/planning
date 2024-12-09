#pragma once

#include <cstddef>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ad_common/math/box2d.h"
#include "common_c.h"
#include "config/message_type.h"
#include "fusion_parking_slot_c.h"
#include "geometry_math.h"
#include "pose2d.h"
#include "src/library/convex_collision_detection/aabb2d.h"
#include "src/library/convex_collision_detection/polygon_base.h"
#include "utils/index_list.h"

namespace planning {
namespace apa_planner {
using namespace pnc;
struct Limiter {
  Eigen::Vector2d start_pt = Eigen::Vector2d::Zero();
  Eigen::Vector2d end_pt = Eigen::Vector2d::Zero();
  // 该值如果为false 表示没有限位器 可以根据车位角点位置来确定终点位置
  bool valid = false;

  void Reset() {
    start_pt.setZero();
    end_pt.setZero();
    valid = false;
  }
};

// 01 肯定为库口角点 23为库尾角点
// 23中点 与 01中点连线 旋转到 23中点与0角点连线 必须为顺时针
// 0和2在一侧  1和3在一侧
struct SlotCoord {
  Eigen::Vector2d pt_1 = Eigen::Vector2d::Zero();       // left up
  Eigen::Vector2d pt_0 = Eigen::Vector2d::Zero();       // right up
  Eigen::Vector2d pt_2 = Eigen::Vector2d::Zero();       // right down
  Eigen::Vector2d pt_3 = Eigen::Vector2d::Zero();       // left_down
  Eigen::Vector2d pt_center = Eigen::Vector2d::Zero();  // center

  Eigen::Vector2d pt_01_mid = Eigen::Vector2d::Zero();
  Eigen::Vector2d pt_23_mid = Eigen::Vector2d::Zero();
  Eigen::Vector2d pt_23mid_01_mid = Eigen::Vector2d::Zero();
  Eigen::Vector2d pt_01_vec = Eigen::Vector2d::Zero();
  Eigen::Vector2d pt_23_vec = Eigen::Vector2d::Zero();

  void CalExtraCoord() {
    pt_center = 0.25 * (pt_1 + pt_0 + pt_2 + pt_3);
    pt_01_mid = (pt_1 + pt_0) * 0.5;
    pt_23_mid = (pt_3 + pt_2) * 0.5;
    pt_23mid_01_mid = pt_01_mid - pt_23_mid;
    pt_01_vec = pt_1 - pt_0;
    pt_23_vec = pt_3 - pt_2;
  }

  void Reset() {
    pt_1.setZero();
    pt_0.setZero();
    pt_2.setZero();
    pt_3.setZero();
    pt_center.setZero();

    pt_01_mid.setZero();
    pt_23_mid.setZero();
    pt_23mid_01_mid.setZero();
    pt_01_vec.setZero();
    pt_23_vec.setZero();
  }

  const SlotCoord GlobalToLocal(const geometry_lib::GlobalToLocalTf& g2l_tf) {
    SlotCoord slot_coord;
    slot_coord.pt_1 = g2l_tf.GetPos(pt_1);
    slot_coord.pt_0 = g2l_tf.GetPos(pt_0);
    slot_coord.pt_2 = g2l_tf.GetPos(pt_2);
    slot_coord.pt_3 = g2l_tf.GetPos(pt_3);
    slot_coord.pt_center = g2l_tf.GetPos(pt_center);
    slot_coord.pt_01_mid = (slot_coord.pt_1 + slot_coord.pt_0) * 0.5;
    slot_coord.pt_23_mid = (slot_coord.pt_2 + slot_coord.pt_3) * 0.5;
    slot_coord.pt_23mid_01_mid = slot_coord.pt_01_mid - slot_coord.pt_23_mid;
    slot_coord.pt_01_vec = slot_coord.pt_1 - slot_coord.pt_0;
    return slot_coord;
  }
};

enum class SlotType : uint8_t {
  PERPENDICULAR,
  PARALLEL,
  SLANT,
  COUNT,
  INVALID,
};

enum SlotReleaseMethod : uint8_t {
  FUSION_RELEASE = 0,
  RULE_BASED_RELEASE = 1,
  GEOMETRY_PLANNING_RELEASE = 2,
  ASTAR_PLANNING_RELEASE = 3,
  SLOT_RELEASE_METHOD_MAX_NUM = 4,
};

enum class SlotReleaseState : uint8_t {
  // 不经过计算，不确定是否释放
  UNKOWN = 0,
  RELEASE = 1,
  NOT_RELEASE = 2,
};

enum class SlotSourceType : uint8_t {
  CAMERA,
  USS,
  CAMERA_USS,
  INVALID,
};

struct SlotReleaseInfo {
  SlotReleaseState release_state[SLOT_RELEASE_METHOD_MAX_NUM];

  void Clear() {
    for (int i = 0; i < SLOT_RELEASE_METHOD_MAX_NUM; ++i) {
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

class ApaSlot final {
 public:
  ApaSlot() = default;
  ~ApaSlot() = default;

  void Update(const iflyauto::ParkingFusionSlot& fusion_slot);

  void Reset() {
    id_ = 0;
    origin_corner_coord_global_.Reset();
    origin_corner_coord_local_.Reset();
    processed_corner_coord_global_.Reset();
    processed_corner_coord_local_.Reset();
    angle_ = 90.0;
    sin_angle_ = 1.0;
    slot_length_ = 0.0;
    slot_width_ = 0.0;

    slot_type_ = SlotType::INVALID;

    slot_source_type_ = SlotSourceType::INVALID;

    limiter_.Reset();

    release_info_.Clear();
  }

  void TransformCoordFromGlobalToLocal(
      const geometry_lib::GlobalToLocalTf& g2l_tf);

  const size_t GetId() const { return id_; }

  const SlotCoord& GetOriginCornerCoordGlobal() const {
    return origin_corner_coord_global_;
  }

  const SlotCoord& GetOriginCornerCoordLocal() const {
    return origin_corner_coord_local_;
  }

  const SlotCoord& GetProcessedCornerCoordGlobal() const {
    return processed_corner_coord_global_;
  }

  const SlotCoord& GetProcessedCornerCoordLocal() const {
    return processed_corner_coord_local_;
  }

  const double GetAngle() const { return angle_; }

  const double GetSinAngle() const { return sin_angle_; }

  const double GetLength() const { return slot_length_; }

  const double GetWidth() const { return slot_width_; }

  const SlotType GetType() const { return slot_type_; }

  const SlotSourceType GetSourceType() const { return slot_source_type_; }

  const Limiter& GetLimiter() const { return limiter_; }

  const SlotReleaseInfo& GetReleaseInfo() const { return release_info_; }

 private:
  void CorrectSlotPointOrder();

  void PostProcessSlotPoint();

 public:
  size_t id_ = 0;

  SlotCoord origin_corner_coord_global_;
  SlotCoord origin_corner_coord_local_;

  // 指将斜车位处理成垂直车位
  SlotCoord processed_corner_coord_global_;
  SlotCoord processed_corner_coord_local_;

  // 23角点中点与01角点中点连线和01角点连线的锐角或直角 一般为90 60 45
  // 垂直车位即90度
  double angle_ = 90.0;
  double sin_angle_ = 1.0;

  double slot_length_ = 0.0;
  double slot_width_ = 0.0;

  SlotType slot_type_ = SlotType::INVALID;

  SlotSourceType slot_source_type_ = SlotSourceType::INVALID;

  Limiter limiter_;

  SlotReleaseInfo release_info_;
};

const std::string GetSlotTypeString(const SlotType type);

void PrintSlotType(const SlotType type);

const std::string GetSlotReleaseStateString(const SlotReleaseState state);

void PrintSlotReleaseState(const SlotReleaseState state);

}  // namespace apa_planner
}  // namespace planning
