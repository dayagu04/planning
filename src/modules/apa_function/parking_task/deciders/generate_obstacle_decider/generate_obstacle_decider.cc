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
  bool success = false;
  const double gen_obs_start_time = IflyTime::Now_ms();
  if (request_.scenario_type ==
          ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN ||
      request_.scenario_type ==
          ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
    return GenObsForPerpendicularParkingIn();
  }

  TimeBenchmark::Instance().SetTime(TimeBenchmarkType::TB_APA_GEN_OBS_TIME,
                                    IflyTime::Now_ms() - gen_obs_start_time);
  return success;
}

const bool GenerateObstacleDecider::GenObsForPerpendicularParkingIn() {
  const ApaParameters& param = apa_param.GetParam();
  const ApaSlot& slot = ego_info_under_slot_.slot;
  const geometry_lib::PathPoint& ego_pose = ego_info_under_slot_.cur_pose;

  CalcVirtualTLane();

  geometry_lib::LineSegment tlane_line;
  std::vector<geometry_lib::LineSegment> tlane_line_vec;
  std::vector<Eigen::Vector2d> tlane_vec{
      virtual_tlane_.A, virtual_tlane_.B, virtual_tlane_.C, virtual_tlane_.D,
      virtual_tlane_.E, virtual_tlane_.F, virtual_tlane_.G, virtual_tlane_.H};

  const std::vector<Eigen::Vector2d> car_box =
      BaseCollisionDetector::GetCarBigBoxWithBuffer(0.5, 0.4, ego_pose);

  if (geometry_lib::CheckTwoPolygonRelationship(tlane_vec, car_box) !=
      geometry_lib::PolyRelation::AContainsB) {
    tlane_vec.clear();
  }

  tlane_line_vec.reserve(tlane_vec.size());
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
        col_det_interface_ptr_->GetGJKColDetPtr()->CalCarRectangleBound(
            ego_info_under_slot_.origin_target_pose);

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
    del_obs_bound.CalcBoundByPtVec(
        slot.GetCustomSlotPolygon(4.0, 0.0, 0.0, 0.0, true));
    del_obs_bound.PrintInfo();
  }

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
      if (tlane_vec.size() > 0 &&
          !geometry_lib::IsPointInPolygon(tlane_vec, obs)) {
        continue;
      }

      if (param.park_path_plan_type == ParkPathPlanType::GEOMETRY) {
        // if obs is in expand car, lose it
        if (!param.believe_in_fus_obs &&
            col_det_interface_ptr_->GetGJKColDetPtr()->IsObsInCar(
                ego_pose, param.car_lat_inflation_normal + 0.0168, obs)) {
          continue;
        }
      }

      // if obs is in del_obs_bound, move it
      if (pair.second.GetObsScemanticType() !=
              ApaObsScemanticType::SPECIFICATIONER &&
          (request_.process_obs_method == ProcessObsMethod::MOVE_OBS_OUT_SLOT ||
           request_.process_obs_method ==
               ProcessObsMethod::MOVE_OBS_OUT_CAR_SAFE_POS) &&
          del_obs_bound.IsPtInRectangleBound(obs)) {
        // move obs to inside area
        double threshold =
            slot.processed_corner_coord_local_.pt_01_mid.y() + 0.86;
        if (ego_pose.pos.y() > 0.0) {
          threshold = slot.processed_corner_coord_local_.pt_01_mid.y() - 0.86;
        }
        if (param.park_path_plan_type != ParkPathPlanType::GEOMETRY) {
          threshold = 0.0;
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
    if (col_det_interface_ptr_->GetGJKColDetPtr()->IsObsInCar(ego_pose, 0.3,
                                                              obs_pos)) {
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

  const double min_x =
      std::min(virtual_tlane_.C.x(), virtual_tlane_.D.x()) - bound_threshold;
  const double max_x =
      std::max(virtual_tlane_.G.x(), virtual_tlane_.H.x()) + bound_threshold;
  const double min_y =
      std::min(virtual_tlane_.F.y(), virtual_tlane_.G.y()) - bound_threshold;
  const double max_y =
      std::max(virtual_tlane_.A.y(), virtual_tlane_.H.y()) + bound_threshold;

  geometry_lib::RectangleBound bound(min_x, min_y, max_x, max_y);

  bound.PrintInfo();

  col_det_interface_ptr_->GetEDTColDetPtr()->PreProcess(
      bound, param.use_obs_height_method);

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
      pt_01_mid - param.virtual_obs_left_x_pos * pt_23mid_01_mid_unit_vec +
      (half_origin_slot_width + param.virtual_obs_left_y_pos) * pt_01_unit_vec;

  const Eigen::Vector2d virtual_right_obs =
      pt_01_mid - param.virtual_obs_right_x_pos * pt_23mid_01_mid_unit_vec -
      (half_origin_slot_width + param.virtual_obs_right_y_pos) * pt_01_unit_vec;

  virtual_tlane_.B = virtual_left_obs;
  virtual_tlane_.E = virtual_right_obs;

  const double rearaxle_frontoverhang_length =
      param.car_length - param.rear_overhanging;

  const double ego_max_x =
      ego_info_under_slot_.cur_pose.GetX() + rearaxle_frontoverhang_length;

  const double ego_max_y =
      ego_info_under_slot_.cur_pose.GetY() + rearaxle_frontoverhang_length;

  const double ego_min_y =
      ego_info_under_slot_.cur_pose.GetY() - rearaxle_frontoverhang_length;

  // area_length and area length can be dynamic by ego pose
  // geometry_lib::RectangleBound bound =
  //     col_det_interface_ptr_->GetGJKColDetPtr()->CalCarRectangleBound(
  //         ego_info_under_slot_.cur_pose);

  virtual_tlane_.channel_width =
      std::max({virtual_tlane_.channel_width, param.channel_width,
                ego_max_x -
                    ego_info_under_slot_.slot.processed_corner_coord_local_
                        .pt_01_mid.x() +
                    0.68});

  const double ego_y = std::max(std::fabs(ego_min_y), std::fabs(ego_max_y)) +
                       1.08 - param.virtual_obs_left_y_pos -
                       slot.slot_width_ / slot.sin_angle_ * 0.5;

  virtual_tlane_.channel_length =
      std::max({virtual_tlane_.channel_length,
                param.channel_length / slot.sin_angle_, ego_y});

  const double area_length = virtual_tlane_.channel_length;
  const double area_width =
      virtual_tlane_.channel_width +
      std::max(param.virtual_obs_left_x_pos, param.virtual_obs_right_x_pos);

  virtual_tlane_.A = virtual_tlane_.B + pt_01_unit_vec * area_length;
  virtual_tlane_.H = virtual_tlane_.A + pt_23mid_01_mid_unit_vec * area_width;

  const double origin_slot_length = (slot.origin_corner_coord_local_.pt_01_mid -
                                     slot.origin_corner_coord_local_.pt_23_mid)
                                        .norm();

  virtual_tlane_.C =
      virtual_tlane_.B -
      pt_23mid_01_mid_unit_vec *
          (origin_slot_length - param.virtual_obs_left_x_pos + 0.68);

  virtual_tlane_.D =
      virtual_tlane_.E -
      pt_23mid_01_mid_unit_vec *
          (origin_slot_length - param.virtual_obs_right_x_pos + 0.68);

  virtual_tlane_.F = virtual_tlane_.E - pt_01_unit_vec * area_length;
  virtual_tlane_.G = virtual_tlane_.F + pt_23mid_01_mid_unit_vec * area_width;

  virtual_tlane_.CalcBound();
  return true;
}

}  // namespace apa_planner
}  // namespace planning