#include "apa_slot.h"

#include <cmath>
#include <cstddef>
#include <vector>

#include "geometry_math.h"
#include "log_glog.h"
#include "math_lib.h"

namespace planning {
namespace apa_planner {
void ApaSlot::Update(const iflyauto::ParkingFusionSlot& fusion_slot) {
  id_ = fusion_slot.id;
  confidence_ = fusion_slot.confidence;

  release_info_.Clear();
  release_info_.release_state[FUSION_RELEASE] =
      (fusion_slot.allow_parking == iflyauto::ALLOW_PARKING)
          ? SlotReleaseState::RELEASE
          : SlotReleaseState::NOT_RELEASE;

  switch (fusion_slot.type) {
    case iflyauto::PARKING_SLOT_TYPE_VERTICAL:
      slot_type_ = SlotType::PERPENDICULAR;
      break;
    case iflyauto::PARKING_SLOT_TYPE_HORIZONTAL:
      slot_type_ = SlotType::PARALLEL;
      break;
    case iflyauto::PARKING_SLOT_TYPE_SLANTING:
      slot_type_ = SlotType::SLANT;
      break;
    default:
      slot_type_ = SlotType::INVALID;
      break;
  }

  switch (fusion_slot.fusion_source) {
    case iflyauto::SLOT_SOURCE_TYPE_ONLY_CAMERA:
      slot_source_type_ = SlotSourceType::CAMERA;
      break;
    case iflyauto::SLOT_SOURCE_TYPE_ONLY_USS:
      slot_source_type_ = SlotSourceType::USS;
      break;
    case iflyauto::SLOT_SOURCE_TYPE_CAMERA_USS:
      slot_source_type_ = SlotSourceType::CAMERA_USS;
      break;
    default:
      slot_source_type_ = SlotSourceType::INVALID;
      break;
  }

  // 融合给的 01必须是库口两个角点 且02必须在一边 13必须在一边
  origin_corner_coord_global_.pt_0 << fusion_slot.corner_points[0].x,
      fusion_slot.corner_points[0].y;
  origin_corner_coord_global_.pt_1 << fusion_slot.corner_points[1].x,
      fusion_slot.corner_points[1].y;
  origin_corner_coord_global_.pt_2 << fusion_slot.corner_points[2].x,
      fusion_slot.corner_points[2].y;
  origin_corner_coord_global_.pt_3 << fusion_slot.corner_points[3].x,
      fusion_slot.corner_points[3].y;
  origin_corner_coord_global_.CalExtraCoord();

  CorrectSlotPointOrder();

  PostProcessSlotPoint();

  if (slot_type_ == SlotType::PERPENDICULAR || slot_type_ == SlotType::SLANT) {
    slot_width_ = (processed_corner_coord_global_.pt_1 -
                   processed_corner_coord_global_.pt_0)
                      .norm();
    slot_length_ = (processed_corner_coord_global_.pt_01_mid -
                    processed_corner_coord_global_.pt_23_mid)
                       .norm();
  } else if (slot_type_ == SlotType::PARALLEL) {
    slot_length_ = (processed_corner_coord_global_.pt_1 -
                    processed_corner_coord_global_.pt_0)
                       .norm();
    const geometry_lib::LineSegment line_01(
        processed_corner_coord_global_.pt_1,
        processed_corner_coord_global_.pt_0);
    slot_width_ = std::min(geometry_lib::CalPoint2LineDist(
                               processed_corner_coord_global_.pt_2, line_01),
                           geometry_lib::CalPoint2LineDist(
                               processed_corner_coord_global_.pt_3, line_01));
  }

  if (fusion_slot.limiters_size > 0) {
    limiter_.valid = true;
    if (fusion_slot.limiters_size == 1) {
      limiter_.start_pt << fusion_slot.limiters[0].end_points[0].x,
          fusion_slot.limiters[0].end_points[0].y;
      limiter_.end_pt << fusion_slot.limiters[0].end_points[1].x,
          fusion_slot.limiters[0].end_points[1].y;
    } else if (fusion_slot.limiters_size == 2) {
      limiter_.start_pt << 0.5 * (fusion_slot.limiters[0].end_points[0].x +
                                  fusion_slot.limiters[0].end_points[1].x),
          0.5 * (fusion_slot.limiters[0].end_points[0].y +
                 fusion_slot.limiters[0].end_points[1].y);
      limiter_.end_pt << 0.5 * (fusion_slot.limiters[1].end_points[0].x +
                                fusion_slot.limiters[1].end_points[1].x),
          0.5 * (fusion_slot.limiters[1].end_points[0].y +
                 fusion_slot.limiters[1].end_points[1].y);
    }
  } else {
    limiter_.valid = false;
  }
}

void ApaSlot::TransformCoordFromGlobalToLocal(
    const geometry_lib::GlobalToLocalTf& g2l_tf) {
  origin_corner_coord_local_ =
      origin_corner_coord_global_.GlobalToLocal(g2l_tf);
  processed_corner_coord_local_ =
      processed_corner_coord_global_.GlobalToLocal(g2l_tf);
}

void ApaSlot::CorrectSlotPointOrder() {
  const Eigen::Vector2d mid_pt_01 =
      (origin_corner_coord_global_.pt_0 + origin_corner_coord_global_.pt_1) *
      0.5;
  const Eigen::Vector2d mid_pt_23 =
      (origin_corner_coord_global_.pt_2 + origin_corner_coord_global_.pt_3) *
      0.5;

  const Eigen::Vector2d slot_mid_vec = mid_pt_01 - mid_pt_23;

  const Eigen::Vector2d mid_pt23_to_pt0_vec =
      origin_corner_coord_global_.pt_0 - mid_pt_23;

  const double cross = slot_mid_vec.x() * mid_pt23_to_pt0_vec.y() -
                       slot_mid_vec.y() * mid_pt23_to_pt0_vec.x();

  if (cross > 0.0) {
    std::swap(origin_corner_coord_global_.pt_0,
              origin_corner_coord_global_.pt_1);
    std::swap(origin_corner_coord_global_.pt_2,
              origin_corner_coord_global_.pt_3);
  }
}

void ApaSlot::PostProcessSlotPoint() {
  processed_corner_coord_global_ = origin_corner_coord_global_;
  if (slot_type_ != SlotType::SLANT) {
    angle_ = 90.0;
    sin_angle_ = 1.0;
    processed_corner_coord_global_.CalExtraCoord();
    return;
  }

  angle_ = std::fabs(geometry_lib::GetAngleFromTwoVec(
               origin_corner_coord_global_.pt_23mid_01mid_vec,
               origin_corner_coord_global_.pt_01_vec)) *
           kRad2Deg;

  if (angle_ > 90.0) {
    angle_ = 180.0 - angle_;
  }

  angle_ = mathlib::DoubleConstrain(angle_, 10.0, 80.0);

  sin_angle_ = std::sin(angle_ * kDeg2Rad);

  ILOG_INFO << "slant slot, should postprocess corner to perpendicular";

  const Eigen::Vector2d pt_01_vec =
      origin_corner_coord_global_.pt_1 - origin_corner_coord_global_.pt_0;
  const Eigen::Vector2d pt_01_unit_vec = pt_01_vec.normalized();

  const Eigen::Vector2d pt_02_vec =
      origin_corner_coord_global_.pt_2 - origin_corner_coord_global_.pt_0;
  const Eigen::Vector2d pt_02_unit_vec = pt_02_vec.normalized();

  const Eigen::Vector2d pt_13_vec =
      origin_corner_coord_global_.pt_3 - origin_corner_coord_global_.pt_1;
  const Eigen::Vector2d pt_13_unit_vec = pt_13_vec.normalized();

  const double cos_theta = pt_01_unit_vec.dot(pt_02_unit_vec);

  if (cos_theta > 0.0) {
    // toward right
    const double dis_0_0dot = pt_01_vec.dot(pt_02_unit_vec);
    const Eigen::Vector2d pt_0dot =
        origin_corner_coord_global_.pt_0 + dis_0_0dot * pt_02_unit_vec;
    const double dist_0dot_2 = pt_02_vec.norm() - dis_0_0dot;
    const Eigen::Vector2d pt_3dot =
        origin_corner_coord_global_.pt_1 + dist_0dot_2 * pt_02_unit_vec;
    processed_corner_coord_global_.pt_0 = pt_0dot;
    processed_corner_coord_global_.pt_3 = pt_3dot;
  } else {
    // toward left
    const Eigen::Vector2d pt_10_vec = -pt_01_vec;
    const double dist_1_1dot = pt_10_vec.dot(pt_13_unit_vec);
    const Eigen::Vector2d pt_1dot =
        origin_corner_coord_global_.pt_1 + dist_1_1dot * pt_13_unit_vec;
    const double dist_1dot_3 = pt_13_vec.norm() - dist_1_1dot;
    const Eigen::Vector2d pt_2dot =
        origin_corner_coord_global_.pt_0 + dist_1dot_3 * pt_13_unit_vec;
    processed_corner_coord_global_.pt_1 = pt_1dot;
    processed_corner_coord_global_.pt_2 = pt_2dot;
  }

  processed_corner_coord_global_.CalExtraCoord();
}

const std::vector<Eigen::Vector2d> ApaSlot::GetSlotPolygon(
    const double slot_entrance_width, const bool base_on_slot,
    const double lat_move_dist, const double lon_move_dist) const {
  Eigen::Vector2d pt0, pt1, pt2, pt3;
  Eigen::Vector2d pt_01_unit_vec, pt_23mid_01mid_vec_unit_vec;
  if (base_on_slot) {
    pt0 = origin_corner_coord_local_.pt_0;
    pt1 = origin_corner_coord_local_.pt_1;
    pt2 = origin_corner_coord_local_.pt_2;
    pt3 = origin_corner_coord_local_.pt_3;
    pt_01_unit_vec = origin_corner_coord_local_.pt_01_unit_vec;
    pt_23mid_01mid_vec_unit_vec =
        origin_corner_coord_local_.pt_23mid_01mid_unit_vec;
  } else {
    pt0 = origin_corner_coord_global_.pt_0;
    pt1 = origin_corner_coord_global_.pt_1;
    pt2 = origin_corner_coord_global_.pt_2;
    pt3 = origin_corner_coord_global_.pt_3;
    pt_01_unit_vec = origin_corner_coord_global_.pt_01_unit_vec;
    pt_23mid_01mid_vec_unit_vec =
        origin_corner_coord_global_.pt_23mid_01mid_unit_vec;
  }

  std::vector<Eigen::Vector2d> pt_vec;
  if (slot_type_ == SlotType::PERPENDICULAR || slot_type_ == SlotType::SLANT) {
    pt0 +=
        (lat_move_dist * pt_01_unit_vec +
         (lon_move_dist + slot_entrance_width) * pt_23mid_01mid_vec_unit_vec);
    pt1 +=
        (lat_move_dist * pt_01_unit_vec +
         (lon_move_dist + slot_entrance_width) * pt_23mid_01mid_vec_unit_vec);
    pt2 += (lat_move_dist * pt_01_unit_vec +
            lon_move_dist * pt_23mid_01mid_vec_unit_vec);
    pt3 += (lat_move_dist * pt_01_unit_vec +
            lon_move_dist * pt_23mid_01mid_vec_unit_vec);

    pt_vec = std::vector<Eigen::Vector2d>{pt0, pt2, pt3, pt1};
  }
  return pt_vec;
}

const bool ApaSlot::IsPointInSlot(const Eigen::Vector2d& pt,
                                  const double slot_entrance_width,
                                  const bool base_on_slot,
                                  const double lat_move_dist,
                                  const double lon_move_dist) const {
  return geometry_lib::IsPointInPolygon(
      GetSlotPolygon(slot_entrance_width, base_on_slot, lat_move_dist,
                     lon_move_dist),
      pt);
}

const bool ApaSlot::IsPointInExpandSlot(const Eigen::Vector2d& pt,
                                        const bool base_on_slot,
                                        const double lat_expand,
                                        const double lon_expand) const {
  Eigen::Vector2d pt0, pt1, pt2, pt3;
  Eigen::Vector2d pt_01_unit_vec, pt_23mid_01mid_vec_unit_vec;
  if (base_on_slot) {
    pt0 = origin_corner_coord_local_.pt_0;
    pt1 = origin_corner_coord_local_.pt_1;
    pt2 = origin_corner_coord_local_.pt_2;
    pt3 = origin_corner_coord_local_.pt_3;
    pt_01_unit_vec = origin_corner_coord_local_.pt_01_unit_vec;
    pt_23mid_01mid_vec_unit_vec =
        origin_corner_coord_local_.pt_23mid_01mid_unit_vec;
  } else {
    pt0 = origin_corner_coord_global_.pt_0;
    pt1 = origin_corner_coord_global_.pt_1;
    pt2 = origin_corner_coord_global_.pt_2;
    pt3 = origin_corner_coord_global_.pt_3;
    pt_01_unit_vec = origin_corner_coord_global_.pt_01_unit_vec;
    pt_23mid_01mid_vec_unit_vec =
        origin_corner_coord_global_.pt_23mid_01mid_unit_vec;
  }

  if (slot_type_ == SlotType::PERPENDICULAR || slot_type_ == SlotType::SLANT) {
    pt0 = pt0 - lat_expand * pt_01_unit_vec +
          lon_expand * pt_23mid_01mid_vec_unit_vec;

    pt1 = pt1 + lat_expand * pt_01_unit_vec +
          lon_expand * pt_23mid_01mid_vec_unit_vec;

    pt2 = pt2 - lat_expand * pt_01_unit_vec -
          lon_expand * pt_23mid_01mid_vec_unit_vec;

    pt3 = pt3 + lat_expand * pt_01_unit_vec -
          lon_expand * pt_23mid_01mid_vec_unit_vec;

    std::vector<Eigen::Vector2d> pt_vec{pt0, pt2, pt3, pt1};

    if (geometry_lib::IsPointInPolygon(pt_vec, pt)) {
      return true;
    }
    return false;
  } else {
    return false;
  }
}

const std::string GetSlotTypeString(const SlotType type) {
  std::string str = "invalid";
  switch (type) {
    case SlotType::PERPENDICULAR:
      str = "PERPENDICULAR";
      break;
    case SlotType::SLANT:
      str = "SLANT";
      break;
    case SlotType::PARALLEL:
      str = "PARALLEL";
      break;
    default:
      break;
  }
  return str;
}

void PrintSlotType(const SlotType type) {
  ILOG_INFO << "slot type = " << GetSlotTypeString(type);
}

const std::string GetSlotReleaseStateString(const SlotReleaseState state) {
  std::string str = "unknown";
  switch (state) {
    case SlotReleaseState::RELEASE:
      str = "release";
      break;
    case SlotReleaseState::NOT_RELEASE:
      str = "not_release";
      break;
    default:
      break;
  }
  return str;
}

void PrintSlotReleaseState(const SlotReleaseState state) {
  ILOG_INFO << "slot release state = " << GetSlotReleaseStateString(state);
}

}  // namespace apa_planner
}  // namespace planning