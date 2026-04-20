#include "generate_obstacle_decider.h"

#include <algorithm>
#include <cstddef>
#include <unordered_map>
#include <vector>

#include "apa_param_config.h"
#include "geometry_math.h"
#include "log_glog.h"
#include "parking_scenario.h"

namespace planning {
namespace apa_planner {
namespace {

constexpr double kCarBigBoxLonBuffer = 0.5;
constexpr double kCarBigBoxLatBuffer = 0.4;
constexpr double kMoveObsBoundLatPadding = 0.12;
constexpr double kMoveObsBoundForwardPadding = 4.0;
constexpr double kMoveObsThresholdOffset = 0.86;
constexpr double kObsInCarLatInflation = 0.0168;
constexpr double kVirtualObsInCarLatBuffer = 0.3;
constexpr double kBoundThreshold = 0.68;
constexpr double kVirtualTLaneHalfWidthPadding = 0.168;
constexpr double kVirtualTLaneEgoYExtraMargin = 1.08;
constexpr size_t kTLaneObstacleReserveSize = 188;

bool IsPerpendicularParkingInScenario(const ParkingScenarioType scenario_type) {
  return scenario_type == ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN ||
         scenario_type == ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN;
}

bool NeedMoveObstacle(const ProcessObsMethod process_obs_method) {
  return process_obs_method == ProcessObsMethod::MOVE_OBS_OUT_SLOT ||
         process_obs_method == ProcessObsMethod::MOVE_OBS_OUT_CAR_SAFE_POS;
}

std::vector<Eigen::Vector2d> BuildTLanePolygon(const TLane& virtual_tlane) {
  return {virtual_tlane.A, virtual_tlane.B, virtual_tlane.C, virtual_tlane.D,
          virtual_tlane.E, virtual_tlane.F, virtual_tlane.G, virtual_tlane.H};
}

std::vector<geometry_lib::LineSegment> BuildClosedPolygonLineSegments(
    const std::vector<Eigen::Vector2d>& polygon_pt_vec) {
  std::vector<geometry_lib::LineSegment> line_seg_vec;
  line_seg_vec.reserve(polygon_pt_vec.size());
  for (size_t i = 0; i < polygon_pt_vec.size(); ++i) {
    geometry_lib::LineSegment line_seg;
    line_seg.SetPoints(polygon_pt_vec[i],
                       polygon_pt_vec[(i + 1) % polygon_pt_vec.size()]);
    line_seg_vec.emplace_back(line_seg);
  }
  return line_seg_vec;
}

std::vector<Eigen::Vector2d> SampleObstaclePointsOnLineSegments(
    const std::vector<geometry_lib::LineSegment>& line_seg_vec,
    const double obstacle_ds) {
  std::vector<Eigen::Vector2d> obstacle_pt_vec;
  obstacle_pt_vec.reserve(kTLaneObstacleReserveSize);

  std::vector<Eigen::Vector2d> sampled_pt_vec;
  for (const auto& line_seg : line_seg_vec) {
    sampled_pt_vec.clear();
    geometry_lib::SamplePointSetInLineSeg(sampled_pt_vec, line_seg,
                                          obstacle_ds);
    obstacle_pt_vec.insert(obstacle_pt_vec.end(), sampled_pt_vec.begin(),
                           sampled_pt_vec.end());
  }
  return obstacle_pt_vec;
}

geometry_lib::RectangleBound BuildDeleteObstacleBound(
    const GenerateObstacleRequest& request,
    const EgoInfoUnderSlot& ego_info_under_slot,
    const std::shared_ptr<GJKCollisionDetector>& gjk_det_ptr) {
  geometry_lib::RectangleBound del_obs_bound;
  const ApaSlot& slot = ego_info_under_slot.slot;

  if (request.process_obs_method ==
      ProcessObsMethod::MOVE_OBS_OUT_CAR_SAFE_POS) {
    del_obs_bound = gjk_det_ptr->CalCarRectangleBound(
        ego_info_under_slot.origin_target_pose);
    del_obs_bound.min_y -= kMoveObsBoundLatPadding;
    del_obs_bound.max_y += kMoveObsBoundLatPadding;
    del_obs_bound.max_x = std::max(
        del_obs_bound.max_x, slot.origin_corner_coord_local_.pt_01_mid.x() +
                                 kMoveObsBoundForwardPadding);
    del_obs_bound.min_x = std::min(
        del_obs_bound.min_x, slot.origin_corner_coord_local_.pt_23_mid.x());
  } else if (request.process_obs_method ==
             ProcessObsMethod::MOVE_OBS_OUT_SLOT) {
    del_obs_bound.CalcBoundByPtVec(slot.GetCustomSlotPolygon(
        kMoveObsBoundForwardPadding, 0.0, 0.0, 0.0, true));
  }

  return del_obs_bound;
}

double ComputeMoveObstacleThreshold(const ApaParameters& param,
                                    const ApaSlot& slot,
                                    const geometry_lib::PathPoint& ego_pose) {
  if (param.park_path_plan_type != ParkPathPlanType::GEOMETRY) {
    return 0.0;
  }

  if (ego_pose.pos.y() > 0.0) {
    return slot.processed_corner_coord_local_.pt_01_mid.y() -
           kMoveObsThresholdOffset;
  }
  return slot.processed_corner_coord_local_.pt_01_mid.y() +
         kMoveObsThresholdOffset;
}

void ProcessObstaclePointClouds(
    std::unordered_map<size_t, ApaObstacle>* obstacles,
    const std::vector<Eigen::Vector2d>& tlane_polygon,
    geometry_lib::RectangleBound& del_obs_bound, const bool need_move_obs,
    const double move_obs_threshold, const ApaParameters& param,
    const geometry_lib::PathPoint& ego_pose,
    const std::shared_ptr<GJKCollisionDetector>& gjk_det_ptr) {
  const bool need_filter_obs_in_car =
      param.park_path_plan_type == ParkPathPlanType::GEOMETRY &&
      !param.believe_in_fus_obs;

  for (auto& pair : *obstacles) {
    std::vector<Eigen::Vector2d>& pt_clout_2d =
        pair.second.GetMutablePtClout2dLocal();
    if (pair.second.GetObsMovementType() == ApaObsMovementType::MOTION) {
      pt_clout_2d.clear();
      continue;
    }

    std::vector<Eigen::Vector2d> filtered_obs_vec;
    filtered_obs_vec.reserve(pt_clout_2d.size());
    for (Eigen::Vector2d& obs : pt_clout_2d) {
      if (!tlane_polygon.empty() &&
          !geometry_lib::IsPointInPolygon(tlane_polygon, obs)) {
        continue;
      }

      if (need_filter_obs_in_car &&
          gjk_det_ptr->IsObsInCar(
              ego_pose, param.car_lat_inflation_normal + kObsInCarLatInflation,
              obs)) {
        continue;
      }

      if (need_move_obs &&
          pair.second.GetObsScemanticType() !=
              ApaObsScemanticType::SPECIFICATIONER &&
          del_obs_bound.IsPtInRectangleBound(obs)) {
        obs.y() = (obs.y() > move_obs_threshold) ? del_obs_bound.max_y
                                                 : del_obs_bound.min_y;
      }

      filtered_obs_vec.emplace_back(obs);
    }

    pt_clout_2d = std::move(filtered_obs_vec);
  }
}

std::vector<Eigen::Vector2d> FilterVirtualObstaclePointsOutsideCar(
    const std::vector<Eigen::Vector2d>& obstacle_pt_vec,
    const geometry_lib::PathPoint& ego_pose,
    const std::shared_ptr<GJKCollisionDetector>& gjk_det_ptr) {
  std::vector<Eigen::Vector2d> filtered_obs_vec;
  filtered_obs_vec.reserve(obstacle_pt_vec.size());
  for (const Eigen::Vector2d& obs_pos : obstacle_pt_vec) {
    if (gjk_det_ptr->IsObsInCar(ego_pose, kVirtualObsInCarLatBuffer, obs_pos)) {
      continue;
    }
    filtered_obs_vec.emplace_back(obs_pos);
  }
  return filtered_obs_vec;
}

void AddVirtualPointCloudObstacle(
    const std::vector<Eigen::Vector2d>& obstacle_pt_vec,
    const std::shared_ptr<ApaObstacleManager>& obs_manager_ptr) {
  Polygon2D polygon;
  cdl::AABB box = cdl::AABB();
  for (const Eigen::Vector2d& pt : obstacle_pt_vec) {
    box.MergePoint(pt);
  }
  GeneratePolygonByAABB(&polygon, box);

  ApaObstacle virtual_obs;
  virtual_obs.SetPtClout2dLocal(obstacle_pt_vec);
  virtual_obs.SetObsAttributeType(ApaObsAttributeType::VIRTUAL_POINT_CLOUD);
  virtual_obs.SetBoxLocal(box);
  virtual_obs.SetPolygonLocal(polygon);
  virtual_obs.SetId(obs_manager_ptr->GetObsIdGenerate());
  obs_manager_ptr->AddObstacle(virtual_obs);
}

geometry_lib::RectangleBound BuildEdtPreprocessBound(
    const TLane& virtual_tlane) {
  const double min_x =
      std::min(virtual_tlane.C.x(), virtual_tlane.D.x()) - kBoundThreshold;
  const double max_x =
      std::max(virtual_tlane.G.x(), virtual_tlane.H.x()) + kBoundThreshold;
  const double min_y =
      std::min(virtual_tlane.F.y(), virtual_tlane.G.y()) - kBoundThreshold;
  const double max_y =
      std::max(virtual_tlane.A.y(), virtual_tlane.H.y()) + kBoundThreshold;
  return geometry_lib::RectangleBound(min_x, min_y, max_x, max_y);
}

}  // namespace

const bool GenerateObstacleDecider::GenObs(
    const EgoInfoUnderSlot& ego_info_under_slot,
    GenerateObstacleRequest request) {
  ego_info_under_slot_ = ego_info_under_slot;
  request_ = request;

  bool success = false;
  const double gen_obs_start_time = IflyTime::Now_ms();
  if (IsPerpendicularParkingInScenario(request_.scenario_type)) {
    success = GenObsForPerpendicularParkingIn();
  }

  TimeBenchmark::Instance().SetTime(TimeBenchmarkType::TB_APA_GEN_OBS_TIME,
                                    IflyTime::Now_ms() - gen_obs_start_time);
  return success;
}

const bool GenerateObstacleDecider::GenObsForPerpendicularParkingIn() {
  const ApaParameters& param = apa_param.GetParam();
  const ApaSlot& slot = ego_info_under_slot_.slot;
  const geometry_lib::PathPoint& ego_pose = ego_info_under_slot_.cur_pose;
  const std::shared_ptr<GJKCollisionDetector>& gjk_det_ptr =
      col_det_interface_ptr_->GetGJKColDetPtr();
  const std::shared_ptr<EDTCollisionDetector>& edt_det_ptr =
      col_det_interface_ptr_->GetEDTColDetPtr();

  CalcVirtualTLane();

  std::vector<Eigen::Vector2d> tlane_polygon =
      BuildTLanePolygon(virtual_tlane_);

  const std::vector<Eigen::Vector2d> car_box =
      BaseCollisionDetector::GetCarBigBoxWithBuffer(
          kCarBigBoxLonBuffer, kCarBigBoxLatBuffer, ego_pose);

  if (geometry_lib::CheckTwoPolygonRelationship(tlane_polygon, car_box) !=
      geometry_lib::PolyRelation::AContainsB) {
    tlane_polygon.clear();
  }

  const std::vector<geometry_lib::LineSegment> tlane_line_vec =
      BuildClosedPolygonLineSegments(tlane_polygon);

  const std::vector<Eigen::Vector2d> tlane_obstacle_vec =
      SampleObstaclePointsOnLineSegments(tlane_line_vec, param.obstacle_ds);

  std::unordered_map<size_t, ApaObstacle>& obstacles =
      obs_manager_ptr_->GetMutableObstacles();

  const bool need_move_obs = NeedMoveObstacle(request_.process_obs_method);

  geometry_lib::RectangleBound del_obs_bound;
  double move_obs_threshold = 0.0;
  if (need_move_obs) {
    del_obs_bound =
        BuildDeleteObstacleBound(request_, ego_info_under_slot_, gjk_det_ptr);
    del_obs_bound.PrintInfo();
    move_obs_threshold = ComputeMoveObstacleThreshold(param, slot, ego_pose);
  }

  ProcessObstaclePointClouds(&obstacles, tlane_polygon, del_obs_bound,
                             need_move_obs, move_obs_threshold, param, ego_pose,
                             gjk_det_ptr);

  const std::vector<Eigen::Vector2d> virtual_obs_pt_vec =
      FilterVirtualObstaclePointsOutsideCar(tlane_obstacle_vec, ego_pose,
                                            gjk_det_ptr);

  AddVirtualPointCloudObstacle(virtual_obs_pt_vec, obs_manager_ptr_);

  const geometry_lib::RectangleBound bound =
      BuildEdtPreprocessBound(virtual_tlane_);

  bound.PrintInfo();
  edt_det_ptr->PreProcess(bound, param.use_obs_height_method);

  return true;
}

const bool GenerateObstacleDecider::CalcVirtualTLane() {
  const ApaParameters& param = apa_param.GetParam();
  const ApaSlot& slot = ego_info_under_slot_.slot;
  const geometry_lib::PathPoint& ego_pose = ego_info_under_slot_.cur_pose;
  const SlotCoord& origin_pt_local = slot.origin_corner_coord_local_;
  const SlotCoord& processed_pt_local = slot.processed_corner_coord_local_;

  const Eigen::Vector2d& pt_01_unit_vec = origin_pt_local.pt_01_unit_vec;
  const Eigen::Vector2d& pt_23mid_01_mid_unit_vec =
      origin_pt_local.pt_23mid_01mid_unit_vec;
  const Eigen::Vector2d& pt_01_mid = origin_pt_local.pt_01_mid;

  double half_origin_slot_width = origin_pt_local.pt_01_vec.norm() * 0.5;
  half_origin_slot_width =
      std::max(half_origin_slot_width,
               param.max_car_width * 0.5 + kVirtualTLaneHalfWidthPadding);

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
  const double ego_max_x = ego_pose.GetX() + rearaxle_frontoverhang_length;
  const double ego_max_y = ego_pose.GetY() + rearaxle_frontoverhang_length;
  const double ego_min_y = ego_pose.GetY() - rearaxle_frontoverhang_length;

  virtual_tlane_.channel_width = std::max(
      {virtual_tlane_.channel_width, param.channel_width,
       ego_max_x - processed_pt_local.pt_01_mid.x() + kBoundThreshold});

  const double ego_y = std::max(std::fabs(ego_min_y), std::fabs(ego_max_y)) +
                       kVirtualTLaneEgoYExtraMargin -
                       param.virtual_obs_left_y_pos -
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

  const double origin_slot_length =
      (origin_pt_local.pt_01_mid - origin_pt_local.pt_23_mid).norm();
  virtual_tlane_.C =
      virtual_tlane_.B -
      pt_23mid_01_mid_unit_vec *
          (origin_slot_length - param.virtual_obs_left_x_pos + kBoundThreshold);
  virtual_tlane_.D =
      virtual_tlane_.E - pt_23mid_01_mid_unit_vec *
                             (origin_slot_length -
                              param.virtual_obs_right_x_pos + kBoundThreshold);

  virtual_tlane_.F = virtual_tlane_.E - pt_01_unit_vec * area_length;
  virtual_tlane_.G = virtual_tlane_.F + pt_23mid_01_mid_unit_vec * area_width;

  virtual_tlane_.CalcBound();
  return true;
}

}  // namespace apa_planner
}  // namespace planning
