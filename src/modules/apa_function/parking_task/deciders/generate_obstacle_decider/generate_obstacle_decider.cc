#include "generate_obstacle_decider.h"

#include <algorithm>
#include <cstddef>

#include "apa_param_config.h"
#include "geometry_math.h"
#include "log_glog.h"
#include "parking_scenario.h"

namespace planning {
namespace apa_planner {

const bool GenerateObstacleDecider::GenObs(
    const EgoInfoUnderSlot& ego_info_under_slot,
    GenerateObstacleRequest request) {
  ego_info_under_slot_ = ego_info_under_slot;
  request_ = request;
  if (request_.scenario_type ==
      ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN) {
    return GenObsForPerpendicularTailIn();
  } else if (request_.scenario_type ==
             ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
    return GenObsForPerpendicularHeadingIn();
  }

  return false;
}

const bool GenerateObstacleDecider::GenObsForPerpendicularTailIn() {
  const ApaParameters& param = apa_param.GetParam();
  const ApaSlot& slot = ego_info_under_slot_.slot;
  const geometry_lib::PathPoint& ego_pose = ego_info_under_slot_.cur_pose;

  CalcVirtualTLane();

  geometry_lib::LineSegment tlane_line;
  std::vector<geometry_lib::LineSegment> tlane_line_vec;
  const std::vector<Eigen::Vector2d> tlane_vec{
      virtual_tlane_.A, virtual_tlane_.B, virtual_tlane_.C, virtual_tlane_.D,
      virtual_tlane_.E, virtual_tlane_.F, virtual_tlane_.G, virtual_tlane_.H};

  for (size_t i = 0; i < tlane_vec.size(); ++i) {
    tlane_line.SetPoints(tlane_vec[i], tlane_vec[(i + 1) % tlane_vec.size()]);
    tlane_line_vec.emplace_back(tlane_line);
  }

  std::vector<Eigen::Vector2d> tlane_obstacle_vec;
  tlane_obstacle_vec.clear();
  tlane_obstacle_vec.reserve(188);
  std::vector<Eigen::Vector2d> point_set;
  for (const auto& line : tlane_line_vec) {
    geometry_lib::SamplePointSetInLineSeg(point_set, line, param.obstacle_ds);
    tlane_obstacle_vec.insert(tlane_obstacle_vec.end(), point_set.begin(),
                              point_set.end());
  }

  std::unordered_map<size_t, ApaObstacle>& obstacles =
      obs_manager_ptr_->GetMutableObstacles();

  geometry_lib::RectangleBound del_obs_bound;
  if (request_.process_obs_method ==
      ProcessObsMethod::MOVE_OBS_OUT_CAR_SAFE_POS) {
    del_obs_bound =
        col_det_interface_ptr_->GetGJKCollisionDetectorPtr()
            ->CalCarRectangleBound(ego_info_under_slot_.origin_target_pose);

    del_obs_bound.min_y -= 0.12;
    del_obs_bound.max_y += 0.12;
    del_obs_bound.max_x = std::max(
        del_obs_bound.max_x,
        ego_info_under_slot_.slot.origin_corner_coord_local_.pt_01_mid.x() +
            4.0);

    del_obs_bound.min_x = std::min(
        del_obs_bound.min_x,
        ego_info_under_slot_.slot.origin_corner_coord_local_.pt_23_mid.x());

    del_obs_bound.PrintInfo();
  } else if (request_.process_obs_method ==
             ProcessObsMethod::MOVE_OBS_OUT_SLOT) {
    del_obs_bound.CalcBoundByPtVec(slot.GetSlotPolygon(4.0, true, 0.0, 0.0));
    del_obs_bound.PrintInfo();
  }

  // 筛选真实的障碍物 并进行替换添加
  for (auto& pair : obstacles) {
    std::vector<Eigen::Vector2d>& pt_clout_2d =
        pair.second.GetMutablePtClout2dLocal();
    if (pair.second.GetObsMovementType() == ApaObsMovementType::MOTION) {
      pt_clout_2d.clear();
      continue;
    }

    std::vector<Eigen::Vector2d> obs_vec;
    obs_vec.reserve(pt_clout_2d.size());

    for (Eigen::Vector2d& obs : pt_clout_2d) {
      // if obs is not in tlane area, lose it
      if (!geometry_lib::IsPointInPolygon(tlane_vec, obs)) {
        continue;
      }

      // if obs is in expand car, lose it
      if (!param.believe_in_fus_obs &&
          col_det_interface_ptr_->GetGJKCollisionDetectorPtr()->IsObsInCar(
              ego_pose, param.car_lat_inflation_normal + 0.0168, obs)) {
        continue;
      }

      // if obs is in del_obs_bound, move it
      if ((request_.process_obs_method == ProcessObsMethod::MOVE_OBS_OUT_SLOT ||
           request_.process_obs_method ==
               ProcessObsMethod::MOVE_OBS_OUT_CAR_SAFE_POS) &&
          del_obs_bound.IsPtInRectangleBound(obs)) {
        // move obs to inside area
        double threshold = threshold =
            slot.processed_corner_coord_local_.pt_01_mid.y() + 0.86;
        if (ego_pose.pos.y() > 0.0) {
          threshold = slot.processed_corner_coord_local_.pt_01_mid.y() - 0.86;
        }
        if (obs.y() > threshold) {
          obs.y() = del_obs_bound.max_y;
        } else {
          obs.y() = del_obs_bound.min_y;
        }
      }

      obs_vec.emplace_back(obs);
    }

    pt_clout_2d = obs_vec;
  }

  // 2. generate virtual obs base on tlane
  std::vector<Eigen::Vector2d> tlane_obs_vec;
  tlane_obs_vec.reserve(tlane_obstacle_vec.size());
  for (const Eigen::Vector2d& obs_pos : tlane_obstacle_vec) {
    if (col_det_interface_ptr_->GetGJKCollisionDetectorPtr()->IsObsInCar(
            ego_pose, 0.3, obs_pos)) {
      continue;
    }
    tlane_obs_vec.emplace_back(obs_pos);
  }
  Polygon2D polygon;
  cdl::AABB box = cdl::AABB();
  for (const Eigen::Vector2d& pt : tlane_obs_vec) {
    box.MergePoint(pt);
  }
  GeneratePolygonByAABB(&polygon, box);
  ApaObstacle virtual_obs;
  virtual_obs.SetPtClout2dLocal(tlane_obs_vec);
  virtual_obs.SetObsAttributeType(ApaObsAttributeType::VIRTUAL_POINT_CLOUD);
  virtual_obs.SetBoxLocal(box);
  virtual_obs.SetPolygonLocal(polygon);
  virtual_obs.SetId(obs_manager_ptr_->GetObsIdGenerate());
  obs_manager_ptr_->AddObstacle(virtual_obs);

  const double bound_threshold = 0.68;

  geometry_lib::RectangleBound bound(virtual_tlane_.min_x - bound_threshold,
                                     virtual_tlane_.min_y - bound_threshold,
                                     virtual_tlane_.max_x + bound_threshold,
                                     virtual_tlane_.max_y + bound_threshold);

  bound.PrintInfo();

  col_det_interface_ptr_->GetEDTCollisionDetectorPtr()->PreProcess(bound);

  return true;
}

const bool GenerateObstacleDecider::CalcVirtualTLane() {
  const ApaParameters& param = apa_param.GetParam();
  const ApaSlot& slot = ego_info_under_slot_.slot;
  // const geometry_lib::PathPoint& ego_pose = ego_info_under_slot_.cur_pose;
  // generate virtual tlane  base on slot coord
  const Eigen::Vector2d pt_01_unit_vec =
      slot.origin_corner_coord_local_.pt_01_unit_vec;

  const Eigen::Vector2d pt_23mid_01_mid_unit_vec =
      slot.origin_corner_coord_local_.pt_23mid_01mid_unit_vec;

  const Eigen::Vector2d pt_01_mid = slot.origin_corner_coord_local_.pt_01_mid;

  double half_origin_slot_width =
      slot.origin_corner_coord_local_.pt_01_vec.norm() * 0.5;

  half_origin_slot_width =
      std::max(half_origin_slot_width, param.max_car_width * 0.5 + 0.168);

  const Eigen::Vector2d virtual_left_obs =
      pt_01_mid - param.virtual_obs_x_pos * pt_23mid_01_mid_unit_vec +
      (half_origin_slot_width + param.virtual_obs_y_pos) * pt_01_unit_vec;

  const Eigen::Vector2d virtual_right_obs =
      pt_01_mid - param.virtual_obs_x_pos * pt_23mid_01_mid_unit_vec -
      (half_origin_slot_width + param.virtual_obs_y_pos) * pt_01_unit_vec;

  virtual_tlane_.Reset();
  virtual_tlane_.B = virtual_left_obs;
  virtual_tlane_.E = virtual_right_obs;

  // area_length and area length can be dynamic by ego pose
  geometry_lib::RectangleBound bound =
      col_det_interface_ptr_->GetGJKCollisionDetectorPtr()
          ->CalCarRectangleBound(ego_info_under_slot_.cur_pose);

  virtual_tlane_.channel_width =
      std::max({virtual_tlane_.channel_width, param.channel_width,
                bound.max_x -
                    ego_info_under_slot_.slot.processed_corner_coord_local_
                        .pt_01_mid.x() +
                    0.68});

  virtual_tlane_.channel_length = std::max(
      {virtual_tlane_.channel_length, 12.0 / slot.sin_angle_,
       std::max(std::fabs(bound.min_y), std::fabs(bound.max_y) + 1.68)});

  const double area_length = virtual_tlane_.channel_length;
  const double area_width =
      virtual_tlane_.channel_width + param.virtual_obs_x_pos;

  virtual_tlane_.A = virtual_tlane_.B + pt_01_unit_vec * area_length;
  virtual_tlane_.H = virtual_tlane_.A + pt_23mid_01_mid_unit_vec * area_width;

  const double origin_slot_length = (slot.origin_corner_coord_local_.pt_0 -
                                     slot.origin_corner_coord_local_.pt_2)
                                        .norm();

  virtual_tlane_.C = virtual_tlane_.B -
                     pt_23mid_01_mid_unit_vec *
                         (origin_slot_length - param.virtual_obs_x_pos + 0.68);

  virtual_tlane_.D = virtual_tlane_.E -
                     pt_23mid_01_mid_unit_vec *
                         (origin_slot_length - param.virtual_obs_x_pos + 0.68);

  virtual_tlane_.F = virtual_tlane_.E - pt_01_unit_vec * area_length;
  virtual_tlane_.G = virtual_tlane_.F + pt_23mid_01_mid_unit_vec * area_width;

  virtual_tlane_.CalcBound();
  return true;
}

const bool GenerateObstacleDecider::GenObsForPerpendicularHeadingIn() {
  return false;
}
}  // namespace apa_planner
}  // namespace planning