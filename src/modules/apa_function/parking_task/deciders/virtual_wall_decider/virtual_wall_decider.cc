#include "virtual_wall_decider.h"

#include <algorithm>
#include <cmath>

#include "aabb2d.h"
#include "apa_param_config.h"
#include "collision_box2d.h"
#include "collision_detect_types.h"
#include "log_glog.h"
#include "polygon_base.h"
#include "pose2d.h"
#include "utils_math.h"

namespace planning {
namespace apa_planner {

void VirtualWallDecider::GenerateVehPolygonInSlot(const Pose2D& ego) {
  const apa_planner::ApaParameters& config = apa_param.GetParam();
  Polygon2D ego_local_polygon;
  double veh_x_buffer = 0.3;
  double veh_y_buffer = 0.4;

  GenerateUpLeftFrameBox(
      &ego_local_polygon, -config.rear_overhanging - veh_x_buffer,
      -config.max_car_width / 2 - veh_y_buffer,
      config.car_length - config.rear_overhanging + veh_x_buffer,
      config.max_car_width / 2 + veh_y_buffer);
  ULFLocalPolygonToGlobal(&ego_polygon_in_slot_, &ego_local_polygon, ego);

  return;
}

const bool VirtualWallDecider::IsVirtualWallPointCollision(
    const Position2D& point) {
  bool is_collision;

  gjk_interface_.PolygonPointCollisionDetect(&is_collision,
                                             &ego_polygon_in_slot_, point);
  if (is_collision) {
    return true;
  }

  return false;
}

void VirtualWallDecider::Process(
    std::vector<Position2D>& points, const double slot_width,
    const double slot_length, const Pose2D& ego_pose, const Pose2D& end,
    const ParkSpaceType slot_type, const pnc::geometry_lib::SlotSide slot_side,
    const ParkingVehDirection parking_type,
    const double head_out_passage_height, const int path_fail_num) {
  start_ = ego_pose;
  end_ = end;
  slot_type_ = slot_type;
  slot_side_ = slot_side;
  GenerateVehPolygonInSlot(ego_pose);
  // vehicle boundary
  GetVehicleBound();
  points.clear();

  if (slot_type == ParkSpaceType::VERTICAL ||
      slot_type == ParkSpaceType::SLANTING) {
    double virtual_wall_x_offset =
        apa_param.GetParam().astar_config.tail_in_slot_virtual_wall_x_offset;
    if (path_fail_num > 0) {
      virtual_wall_x_offset = 2.5;
    }

    double virtual_wall_y_offset =
        apa_param.GetParam().astar_config.tail_in_slot_virtual_wall_y_offset;

    double passage_half_length =
        apa_param.GetParam().astar_config.vertical_slot_passage_length_bound /
        2;

    double passage_height =
        apa_param.GetParam().astar_config.vertical_slot_passage_height_bound;

    if (parking_type == ParkingVehDirection::HEAD_IN) {
      virtual_wall_x_offset =
          apa_param.GetParam().astar_config.head_in_slot_virtual_wall_x_offset;

      virtual_wall_y_offset =
          apa_param.GetParam().astar_config.head_in_slot_virtual_wall_y_offset;
    } else if (parking_type == ParkingVehDirection::HEAD_OUT_TO_LEFT ||
               parking_type == ParkingVehDirection::HEAD_OUT_TO_RIGHT ||
               parking_type == ParkingVehDirection::HEAD_OUT_TO_MIDDLE ||
               parking_type == ParkingVehDirection::NONE) {
      passage_half_length = 18.0;
      virtual_wall_x_offset = 1.5;
      passage_height = head_out_passage_height;
      virtual_wall_y_offset = 0.7;
    } else if (parking_type == ParkingVehDirection::TAIL_OUT_TO_LEFT ||
               parking_type == ParkingVehDirection::TAIL_OUT_TO_RIGHT ||
               parking_type == ParkingVehDirection::TAIL_OUT_TO_MIDDLE ||
               parking_type == ParkingVehDirection::NONE) {
      passage_half_length = 18.0;
      virtual_wall_x_offset = 1.5;
      passage_height =
          apa_param.GetParam().astar_config.vertical_slot_passage_height_bound;
    }

    CalcVerticalVirtualWall(points, slot_width, slot_length, ego_pose, end,
                            virtual_wall_x_offset, virtual_wall_y_offset,
                            passage_half_length, passage_height);

  } else {
    if (slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
      RightSideParallelVirtualWall(points, slot_width, slot_length, ego_pose,
                                   end);
    } else if (slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
      LeftSideParallelVirtualWall(points, slot_width, slot_length, ego_pose,
                                  end);
    }

    ILOG_INFO << "parallel slot virtual wall";
  }

  return;
}

void VirtualWallDecider::SampleInLineSegment(const Eigen::Vector2d& start,
                                             const Eigen::Vector2d& end,
                                             std::vector<Position2D>* points) {
  const Eigen::Vector2d line = end - start;
  const Eigen::Vector2d unit_line_vec = line.normalized();
  double len = line.norm();

  double s = 0.0;
  double ds = 0.4;

  int size = std::ceil(len / ds) + 1;

  Eigen::Vector2d point;
  for (size_t i = 0; i < size; i++) {
    point = start + s * unit_line_vec;
    s += ds;
    s = std::min(s, len);

    if (IsVirtualWallPointCollision(Position2D(point.x(), point.y()))) {
      continue;
    }

    points->emplace_back(Position2D(point.x(), point.y()));
  }

  return;
}

void VirtualWallDecider::SamplingInVerticalBoundary(
    const VirtualWallBoundary& slot_boundary,
    const VirtualWallBoundary& passage_boundary,
    std::vector<Position2D>& points) {
  // sampling point
  // slot right bound
  SampleInLineSegment(
      Eigen::Vector2d(slot_boundary.x_lower, slot_boundary.y_lower),
      Eigen::Vector2d(slot_boundary.x_upper, slot_boundary.y_lower), &points);

  // slot left bound
  SampleInLineSegment(
      Eigen::Vector2d(slot_boundary.x_upper, slot_boundary.y_upper),
      Eigen::Vector2d(slot_boundary.x_lower, slot_boundary.y_upper), &points);

  // passage lower right boundary
  SampleInLineSegment(
      Eigen::Vector2d(slot_boundary.x_upper, slot_boundary.y_lower),
      Eigen::Vector2d(passage_boundary.x_lower, passage_boundary.y_lower),
      &points);

  // passage lower left boundary
  SampleInLineSegment(
      Eigen::Vector2d(slot_boundary.x_upper, slot_boundary.y_upper),
      Eigen::Vector2d(passage_boundary.x_lower, passage_boundary.y_upper),
      &points);

  // upper boundary
  SampleInLineSegment(
      Eigen::Vector2d(passage_boundary.x_upper, passage_boundary.y_upper),
      Eigen::Vector2d(passage_boundary.x_upper, passage_boundary.y_lower),
      &points);

  // right boundary
  SampleInLineSegment(
      Eigen::Vector2d(passage_boundary.x_upper, passage_boundary.y_lower),
      Eigen::Vector2d(passage_boundary.x_lower, passage_boundary.y_lower),
      &points);

  // left boundary
  SampleInLineSegment(
      Eigen::Vector2d(passage_boundary.x_upper, passage_boundary.y_upper),
      Eigen::Vector2d(passage_boundary.x_lower, passage_boundary.y_upper),
      &points);

  // slot lower bound
  SampleInLineSegment(
      Eigen::Vector2d(slot_boundary.x_lower, slot_boundary.y_upper),
      Eigen::Vector2d(slot_boundary.x_lower, slot_boundary.y_lower), &points);

  return;
}

void VirtualWallDecider::SamplingInParallelBoundary(
    const VirtualWallBoundary& slot_boundary,
    const VirtualWallBoundary& passage_boundary,
    std::vector<Position2D>& points) {
  // sampling point
  // slot right bound
  if (slot_side_ == pnc::geometry_lib::SlotSide::SLOT_SIDE_RIGHT) {
    SampleInLineSegment(
        Eigen::Vector2d(slot_boundary.x_lower, slot_boundary.y_lower),
        Eigen::Vector2d(slot_boundary.x_upper, slot_boundary.y_lower), &points);
  }

  // slot left bound
  if (slot_side_ == pnc::geometry_lib::SlotSide::SLOT_SIDE_LEFT) {
    SampleInLineSegment(
        Eigen::Vector2d(slot_boundary.x_upper, slot_boundary.y_upper),
        Eigen::Vector2d(slot_boundary.x_lower, slot_boundary.y_upper), &points);
  }

  // slot lower bound
  SampleInLineSegment(
      Eigen::Vector2d(slot_boundary.x_lower, slot_boundary.y_upper),
      Eigen::Vector2d(slot_boundary.x_lower, slot_boundary.y_lower), &points);

  // slot upper bound
  SampleInLineSegment(
      Eigen::Vector2d(slot_boundary.x_upper, slot_boundary.y_upper),
      Eigen::Vector2d(slot_boundary.x_upper, slot_boundary.y_lower), &points);

  // lower boundary
  SampleInLineSegment(
      Eigen::Vector2d(passage_boundary.x_lower, passage_boundary.y_upper),
      Eigen::Vector2d(passage_boundary.x_lower, passage_boundary.y_lower),
      &points);

  // upper boundary
  SampleInLineSegment(
      Eigen::Vector2d(passage_boundary.x_upper, passage_boundary.y_lower),
      Eigen::Vector2d(passage_boundary.x_upper, passage_boundary.y_upper),
      &points);

  if (slot_side_ == pnc::geometry_lib::SlotSide::SLOT_SIDE_RIGHT) {
    // left boundary
    SampleInLineSegment(
        Eigen::Vector2d(passage_boundary.x_upper, passage_boundary.y_upper),
        Eigen::Vector2d(passage_boundary.x_lower, passage_boundary.y_upper),
        &points);

    // passage right lower boundary
    SampleInLineSegment(
        Eigen::Vector2d(slot_boundary.x_lower, slot_boundary.y_upper),
        Eigen::Vector2d(passage_boundary.x_lower, passage_boundary.y_lower),
        &points);

    // passage right upper boundary
    SampleInLineSegment(
        Eigen::Vector2d(slot_boundary.x_upper, slot_boundary.y_upper),
        Eigen::Vector2d(passage_boundary.x_upper, passage_boundary.y_lower),
        &points);
  }

  if (slot_side_ == pnc::geometry_lib::SlotSide::SLOT_SIDE_LEFT) {
    // right boundary
    SampleInLineSegment(
        Eigen::Vector2d(passage_boundary.x_upper, passage_boundary.y_lower),
        Eigen::Vector2d(passage_boundary.x_lower, passage_boundary.y_lower),
        &points);

    // passage left lower boundary
    SampleInLineSegment(
        Eigen::Vector2d(slot_boundary.x_lower, slot_boundary.y_lower),
        Eigen::Vector2d(passage_boundary.x_lower, passage_boundary.y_lower),
        &points);

    // passage left upper boundary
    SampleInLineSegment(
        Eigen::Vector2d(slot_boundary.x_upper, slot_boundary.y_lower),
        Eigen::Vector2d(passage_boundary.x_upper, passage_boundary.y_upper),
        &points);
  }

  return;
}

void VirtualWallDecider::CalcVerticalVirtualWall(
    std::vector<Position2D>& points, const double slot_width,
    const double slot_length, const Pose2D& ego_pose, const Pose2D& end,
    const double virtual_wall_x_offset, const double virtual_wall_y_offset,
    const double passage_half_length, const double passage_height) {
  // slot virtual wall
  slot_boundary_.y_lower = -slot_width / 2.0 - virtual_wall_y_offset;
  slot_boundary_.x_upper = slot_length - virtual_wall_x_offset;
  slot_boundary_.y_upper = slot_width / 2.0 + virtual_wall_y_offset;
  double lower_bound_x = -1.0;
  slot_boundary_.x_lower = lower_bound_x;

  // passage virtual wall
  // lower
  VirtualWallBoundary tmp_passage_boundary;
  tmp_passage_boundary.x_lower = slot_length - virtual_wall_x_offset;
  // passage up bound

  double passage_up_bound_x = slot_length + passage_height;
  tmp_passage_boundary.x_upper = passage_up_bound_x;
  // passage left/right bound

  tmp_passage_boundary.y_lower = -passage_half_length;
  tmp_passage_boundary.y_upper = passage_half_length;
  tmp_passage_boundary.Combine(veh_boundary_);

  // boundary只能增长，不能缩减.
  // 否则车辆移动后，缩减后的boundary无法再生成一致的path.
  passage_bound_.Combine(tmp_passage_boundary);

  // sampling point
  SamplingInVerticalBoundary(slot_boundary_, passage_bound_, points);

  return;
}

#define PARALLEL_SLOT_EXTRA_LEN (6.0)
void VirtualWallDecider::RightSideParallelVirtualWall(
    std::vector<Position2D>& points, const double slot_width,
    const double slot_length, const Pose2D& ego_pose, const Pose2D& end) {
  // slot virtual wall
  slot_boundary_.y_lower = -slot_width / 2.0 - 1.0;
  slot_boundary_.x_upper = slot_length + PARALLEL_SLOT_EXTRA_LEN;
  slot_boundary_.y_upper = slot_width / 2.0;
  slot_boundary_.x_lower = -PARALLEL_SLOT_EXTRA_LEN;

  // passage virtual wall
  // lower
  auto& config = apa_param.GetParam().astar_config;
  VirtualWallBoundary tmp_passage_boundary;
  tmp_passage_boundary.x_lower =
      -(config.parallel_passage_length / 2 - slot_length / 2);
  // passage up bound
  tmp_passage_boundary.x_upper =
      (config.parallel_passage_length / 2 + slot_length / 2);

  // passage left/right bound
  tmp_passage_boundary.y_lower = slot_boundary_.y_upper;
  tmp_passage_boundary.y_upper =
      slot_boundary_.y_upper + config.parallel_passage_width;
  tmp_passage_boundary.Combine(veh_boundary_);

  // boundary只能增长，不能缩减.
  // 否则车辆移动后，缩减后的boundary无法再生成一致的path.
  passage_bound_.Combine(tmp_passage_boundary);

  // sampling point
  SamplingInParallelBoundary(slot_boundary_, passage_bound_, points);

  return;
}

void VirtualWallDecider::LeftSideParallelVirtualWall(
    std::vector<Position2D>& points, const double slot_width,
    const double slot_length, const Pose2D& ego_pose, const Pose2D& end) {
  // slot virtual wall
  slot_boundary_.y_lower = -slot_width / 2.0;
  slot_boundary_.x_upper = slot_length + PARALLEL_SLOT_EXTRA_LEN;
  slot_boundary_.y_upper = slot_width / 2.0 + 1.0;
  slot_boundary_.x_lower = -PARALLEL_SLOT_EXTRA_LEN;

  // passage virtual wall
  // lower
  auto& config = apa_param.GetParam().astar_config;
  VirtualWallBoundary tmp_passage_boundary;
  tmp_passage_boundary.x_lower =
      -(config.parallel_passage_length / 2 - slot_length / 2);
  // passage up bound
  tmp_passage_boundary.x_upper =
      config.parallel_passage_length / 2 + slot_length / 2;

  // passage left/right bound
  tmp_passage_boundary.y_lower =
      slot_boundary_.y_lower - config.parallel_passage_width;
  tmp_passage_boundary.y_upper = slot_boundary_.y_lower;
  tmp_passage_boundary.Combine(veh_boundary_);

  // boundary只能增长，不能缩减.
  // 否则车辆移动后，缩减后的boundary无法再生成一致的path.
  passage_bound_.Combine(tmp_passage_boundary);

  // sampling point
  SamplingInParallelBoundary(slot_boundary_, passage_bound_, points);

  return;
}

void VirtualWallDecider::Init(const Pose2D& ego_pose) {
  passage_bound_ = VirtualWallBoundary(Position2D(ego_pose.x, ego_pose.y));

  return;
}

void VirtualWallDecider::GetVehicleBound() {
  double veh_upper_x = start_.x;
  double veh_lower_x = start_.x;
  double veh_left_y = start_.y;
  double veh_right_y = start_.y;
  for (int i = 0; i < ego_polygon_in_slot_.vertex_num; i++) {
    veh_upper_x = std::max(veh_upper_x, ego_polygon_in_slot_.vertexes[i].x);
    veh_lower_x = std::min(veh_lower_x, ego_polygon_in_slot_.vertexes[i].x);

    veh_left_y = std::max(veh_left_y, ego_polygon_in_slot_.vertexes[i].y);
    veh_right_y = std::min(veh_right_y, ego_polygon_in_slot_.vertexes[i].y);
  }

  double veh_buffer = 0.5;
  veh_boundary_.x_upper = veh_upper_x + veh_buffer;
  veh_boundary_.x_lower = veh_lower_x - veh_buffer;
  veh_boundary_.y_upper = veh_left_y + veh_buffer;
  veh_boundary_.y_lower = veh_right_y - veh_buffer;

  return;
}

const MapBound VirtualWallDecider::GetMaxMapBound() const {
  VirtualWallBoundary tmp = passage_bound_;
  tmp.Combine(slot_boundary_);
  tmp.Combine(veh_boundary_);

  MapBound bound;
  bound.x_max = tmp.x_upper;
  bound.x_min = tmp.x_lower;
  bound.y_max = tmp.y_upper;
  bound.y_min = tmp.y_lower;

  return bound;
}

const MapBound VirtualWallDecider::GetVehBound() const {
  MapBound bound;
  bound.x_max = veh_boundary_.x_upper;
  bound.x_min = veh_boundary_.x_lower;
  bound.y_max = veh_boundary_.y_upper;
  bound.y_min = veh_boundary_.y_lower;

  return bound;
}
}  // namespace apa_planner
}  // namespace planning