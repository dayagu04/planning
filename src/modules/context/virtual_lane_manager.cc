#include "virtual_lane_manager.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "ad_common/hdmap/hdmap.h"
#include "common_c.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "define/geometry.h"
#include "ego_lane_track_manager.h"
#include "ehr.pb.h"
#include "ehr_sdmap.pb.h"
#include "environmental_model.h"
#include "fusion_road_c.h"
#include "ifly_localization_c.h"
#include "ifly_parking_map.pb.h"
#include "ifly_time.h"
#include "local_view.h"
#include "log.h"
// #include "log_glog.h"
#include "math/box2d.h"
#include "math/vec2d.h"
#include "planning_context.h"
#include "reference_path_manager.h"
#include "stop_line.h"
#include "task_basic_types.h"
#include "utils/kd_path.h"
#include "utils/path_point.h"
#include "vehicle_config_context.h"
#include "virtual_lane.h"
namespace planning {

using Map::CurrentRouting;
using Map::FormOfWayType::MAIN_ROAD;
using Map::FormOfWayType::RAMP;
using ad_common::hdmap::LaneGroupConstPtr;
using ad_common::hdmap::LaneInfoConstPtr;
const double PI = 3.1415926;

namespace {
constexpr double kEpsilon = 1.0e-4;
}  // namespace

VirtualLaneManager::VirtualLaneManager(
    const EgoPlanningConfigBuilder* config_builder,
    planning::framework::Session* session): session_(session), ego_lane_track_manager_(config_builder, session){
  SetConfig(config_builder);
}

void VirtualLaneManager::SetConfig(
    const EgoPlanningConfigBuilder* config_builder) {
  config_ = config_builder->cast<EgoPlanningVirtualLaneManagerConfig>();
  is_select_split_nearing_ramp_ = config_.is_select_split_nearing_ramp;
}

std::vector<double> VirtualLaneManager::construct_reference_line_scc(void) {
  // 暂且发直线
  std::vector<double> virtual_poly(4, 0.0);
  return virtual_poly;  // 依次为常数项、一次项、二次项、三次项
}

std::vector<double> VirtualLaneManager::construct_reference_line_acc(void) {
  const double ego_v =
      session_->environmental_model().get_ego_state_manager()->ego_v();

  const double ego_yaw_rate =
      session_->environmental_model().get_ego_state_manager()->ego_yaw_rate();

  const double ego_steer_angle = session_->environmental_model()
                                     .get_ego_state_manager()
                                     ->ego_steer_angle();

  LOG_DEBUG("ego_v =  %f, ego_yaw_rate = %f, ego_steer_angle = %f\n", ego_v,
            ego_yaw_rate, ego_steer_angle);

  LOG_DEBUG("ego_v =  %f, ego_yaw_rate = %f, ego_steer_angle = %f\n", ego_v,
            ego_yaw_rate, ego_steer_angle);

  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double steer_ratio = vehicle_param.steer_ratio;

  const double wheel_base = vehicle_param.wheel_base;

  const double curv_low_spd = ego_steer_angle / steer_ratio / wheel_base;
  const double curv_high_spd = ego_yaw_rate / std::fmax(ego_v, 0.1);

  double curv_low_spd_factor;
  double curv_knee_pt1_mps = 3.0;
  double curv_knee_pt2_mps = 6.0;
  if (ego_v < curv_knee_pt1_mps) {
    curv_low_spd_factor = 1.0;
  } else if (ego_v < curv_knee_pt2_mps) {
    curv_low_spd_factor =
        (curv_knee_pt2_mps - ego_v) / (curv_knee_pt2_mps - curv_knee_pt1_mps);
  } else {
    curv_low_spd_factor = 0.0;
  }
  const double irregular_yawrate_thr = 0.75;
  if (std::fabs(ego_yaw_rate) > irregular_yawrate_thr) {
    curv_low_spd_factor = 1.0;
  }

  double curv = curv_low_spd_factor * curv_low_spd +
                (1.0 - curv_low_spd_factor) * curv_high_spd;
  LOG_DEBUG("curv =  %f\n", curv);
  curv = std::min(std::max(curv, -0.1), 0.1);

  std::vector<double> virtual_poly(4, 0.0);
  virtual_poly[2] = curv / 2.0;

  return virtual_poly;  // 依次为常数项、一次项、二次项、三次项
}

void VirtualLaneManager::construct_reference_line_msg(
    const std::vector<double>& current_lane_virtual_poly,
    iflyauto::ReferenceLineMsg& current_lane_virtual) {
  LOG_DEBUG("construct_reference_line_msg\n");
  // set default value
  current_lane_virtual.order_id = 0;
  current_lane_virtual.relative_id = 0;

  // TBD WB: 可以这么写么
  auto current_lane_virtual_type = current_lane_virtual.lane_types[0];
  current_lane_virtual_type.begin = 0.0;
  current_lane_virtual_type.end = 0.0;
  current_lane_virtual_type.type = iflyauto::LANETYPE_VIRTUAL;

  auto current_lane_virtual_mark = current_lane_virtual.lane_marks[0];
  current_lane_virtual_mark.begin = 0.0;
  current_lane_virtual_mark.end = 0.0;
  current_lane_virtual_mark.lane_mark =
      iflyauto::LaneDrivableDirection_DIRECTION_UNKNOWN;

  auto current_lane_virtual_source = current_lane_virtual.lane_sources[0];
  current_lane_virtual_source.begin = -50.0;
  current_lane_virtual_source.end = 150.0;
  current_lane_virtual_source.source = iflyauto::LaneSource_SOURCE_UNKNOWN;

  // set lane_reference_line poly
  iflyauto::LaneReferenceLine* current_lane_virtual_ref =
      &(current_lane_virtual.lane_reference_line);

  current_lane_virtual_ref->poly_coefficient_car[0] =
      current_lane_virtual_poly[0];
  current_lane_virtual_ref->poly_coefficient_car[1] =
      current_lane_virtual_poly[1];
  current_lane_virtual_ref->poly_coefficient_car[2] =
      current_lane_virtual_poly[2];
  current_lane_virtual_ref->poly_coefficient_car[3] =
      current_lane_virtual_poly[3];

  // center line poly
  const double c0 = current_lane_virtual_ref->poly_coefficient_car[0];
  const double c1 = current_lane_virtual_ref->poly_coefficient_car[1];
  const double c2 = current_lane_virtual_ref->poly_coefficient_car[2];
  const double c3 = current_lane_virtual_ref->poly_coefficient_car[3];
  LOG_DEBUG("c2 = %f\n", c2);

  // set lane_reference_line ref points
  double x = -50.0;
  double s = 0.0;
  double y_delay;
  const double heading_angle =
      session_->environmental_model().get_ego_state_manager()->heading_angle();
  auto& car2enu =
      session_->environmental_model().get_ego_state_manager()->get_car2enu();
  for (size_t i = 0; i < FUSION_ROAD_REFLINE_POINT_MAX_NUM; ++i) {
    iflyauto::ReferencePoint* current_lane_virtual_ref_point =
        &(current_lane_virtual_ref->virtual_lane_refline_points[i]);

    const double y = c0 + (c1 + (c2 + c3 * x) * x) * x;
    const double heading = c1 + (2.0 * c2 + 3.0 * c3 * x) * x;
    const double curvature =
        std::fabs(2.0 * c2 + 6.0 * c3 * x) /
        std::pow(std::pow(c1 + (2.0 * c2 + 3.0 * c3 * x) * x, 2) + 1, 1.5);

    current_lane_virtual_ref_point->track_id = 0;

    Eigen::Vector3d car_point, enu_point;
    car_point.x() = x;
    car_point.y() = y;
    car_point.z() = 0.0;
    enu_point = car2enu * car_point;

    current_lane_virtual_ref_point->car_point.x = car_point.x();
    current_lane_virtual_ref_point->car_point.y = car_point.y();
    current_lane_virtual_ref_point->enu_point.x = enu_point.x();
    current_lane_virtual_ref_point->enu_point.y = enu_point.y();
    current_lane_virtual_ref_point->enu_point.z = enu_point.z();

    current_lane_virtual_ref_point->local_point.x = enu_point.x();
    current_lane_virtual_ref_point->local_point.y = enu_point.y();
    current_lane_virtual_ref_point->local_point.z = enu_point.z();

    current_lane_virtual_ref_point->curvature = curvature;
    current_lane_virtual_ref_point->car_heading = heading;
    current_lane_virtual_ref_point->enu_heading =
        fmod(heading + heading_angle + M_PI, 2 * M_PI) - M_PI;
    current_lane_virtual_ref_point->local_heading =
        fmod(heading + heading_angle + M_PI, 2 * M_PI) - M_PI;

    current_lane_virtual_ref_point->lane_width = 3.75;

    current_lane_virtual_ref_point->distance_to_left_lane_border =
        current_lane_virtual_ref_point->lane_width * 0.5;
    current_lane_virtual_ref_point->distance_to_right_lane_border =
        current_lane_virtual_ref_point->lane_width * 0.5;
    // current_lane_virtual_ref_point->distance_to_left_road_border =
    //     current_lane_virtual_ref_point->lane_width * 0.5;
    // current_lane_virtual_ref_point->distance_to_right_road_border =
    //     current_lane_virtual_ref_point->lane_width * 0.5;
    current_lane_virtual_ref_point->distance_to_left_road_border = 20.0;
    current_lane_virtual_ref_point->distance_to_right_road_border = 20.0;

    current_lane_virtual_ref_point->speed_limit_max = 0.0;
    current_lane_virtual_ref_point->speed_limit_min = 0.0;

    current_lane_virtual_ref_point->left_road_border_type =
        iflyauto::LaneBoundaryType_MARKING_UNKNOWN;
    current_lane_virtual_ref_point->right_road_border_type =
        iflyauto::LaneBoundaryType_MARKING_UNKNOWN;
    current_lane_virtual_ref_point->left_lane_border_type =
        iflyauto::LaneBoundaryType_MARKING_UNKNOWN;
    current_lane_virtual_ref_point->right_lane_border_type =
        iflyauto::LaneBoundaryType_MARKING_UNKNOWN;

    current_lane_virtual_ref_point->is_in_intersection = false;

    current_lane_virtual_ref_point->lane_type = iflyauto::LANETYPE_VIRTUAL;

    const double delta_x = 2.0;
    if (i == 0) {
      s = 0.0;
    } else {
      s = s + std::hypot(delta_x, y - y_delay);
    }
    current_lane_virtual_ref_point->s = s;
    x += delta_x;
    y_delay = y;
  }
  current_lane_virtual_ref->virtual_lane_refline_points_size =
      FUSION_ROAD_REFLINE_POINT_MAX_NUM;

  LOG_DEBUG(
      "current_lane_virtual_ref->virtual_lane_refline_points_size =  %d\n",
      current_lane_virtual_ref->virtual_lane_refline_points_size);
  LOG_DEBUG("s_end =  %f\n", s);

  // set left_lane_boundary
  double left_lane_boundary_c0 = c0 + 3.75 * 0.5;
  iflyauto::LaneBoundary* current_lane_virtual_left =
      &(current_lane_virtual.left_lane_boundary);

  current_lane_virtual_left->existence = true;
  current_lane_virtual_left->life_time = 1;
  current_lane_virtual_left->track_id = 0;
  current_lane_virtual_left->type = iflyauto::LINE_TYPE_UNKNOWN;
  current_lane_virtual_left->poly_coefficient[0] = left_lane_boundary_c0;
  current_lane_virtual_left->poly_coefficient[1] = c1;
  current_lane_virtual_left->poly_coefficient[2] = c2;
  current_lane_virtual_left->poly_coefficient[3] = c3;
  current_lane_virtual_left->begin = -50.0;
  current_lane_virtual_left->end = 150.0;
  current_lane_virtual_left->type_segments_size = 1;
  current_lane_virtual_left->type_segments[0].length = 150.0;
  current_lane_virtual_left->type_segments[0].type =
      iflyauto::LaneBoundaryType_MARKING_SOLID;

  x = -50.0;
  double lane_boundary_delta_x = 200.0 / LANE_BOUNDARY_POINT_SET_NUM;
  // TBD: LANE_BOUNDARY_POINT_SET_NUM被指定的大小为20，不够用吧
  for (size_t i = 0; i < LANE_BOUNDARY_POINT_SET_NUM; ++i) {
    iflyauto::Point2f* current_lane_virtual_left_car_point =
        current_lane_virtual_left->car_points;
    iflyauto::Point3f* current_lane_virtual_left_enu_point =
        current_lane_virtual_left->enu_points;
    iflyauto::Point3f* current_lane_virtual_left_local_point =
        current_lane_virtual_left->local_point;

    double y = left_lane_boundary_c0 + (c1 + (c2 + c3 * x) * x) * x;
    Eigen::Vector3d car_point, enu_point;
    car_point.x() = x;
    car_point.y() = y;
    car_point.z() = 0.0;
    enu_point = car2enu * car_point;
    current_lane_virtual_left_car_point[i].x = car_point.x();
    current_lane_virtual_left_car_point[i].y = car_point.y();
    current_lane_virtual_left_enu_point[i].x = enu_point.x();
    current_lane_virtual_left_enu_point[i].y = enu_point.y();
    current_lane_virtual_left_enu_point[i].z = enu_point.z();
    current_lane_virtual_left_local_point[i].x = enu_point.x();
    current_lane_virtual_left_local_point[i].y = enu_point.y();
    current_lane_virtual_left_local_point[i].z = enu_point.z();

    x += lane_boundary_delta_x;
  }

  // set right_lane_boundary
  double right_lane_boundary_c0 = c0 - 3.75 * 0.5;
  iflyauto::LaneBoundary* current_lane_virtual_right =
      &(current_lane_virtual.right_lane_boundary);

  current_lane_virtual_right->existence = true;
  current_lane_virtual_right->life_time = 1;
  current_lane_virtual_right->track_id = 0;
  current_lane_virtual_right->type = iflyauto::LINE_TYPE_UNKNOWN;
  current_lane_virtual_right->poly_coefficient[0] = right_lane_boundary_c0;
  current_lane_virtual_right->poly_coefficient[1] = c1;
  current_lane_virtual_right->poly_coefficient[2] = c2;
  current_lane_virtual_right->poly_coefficient[3] = c3;
  current_lane_virtual_right->begin = -50.0;
  current_lane_virtual_right->end = 150.0;
  current_lane_virtual_right->type_segments_size = 1;
  current_lane_virtual_right->type_segments[0].length = 150.0;
  current_lane_virtual_right->type_segments[0].type =
      iflyauto::LaneBoundaryType_MARKING_SOLID;

  x = -50.0;
  for (size_t i = 0; i < LANE_BOUNDARY_POINT_SET_NUM; ++i) {
    iflyauto::Point2f* current_lane_virtual_right_car_point =
        current_lane_virtual_right->car_points;
    iflyauto::Point3f* current_lane_virtual_right_enu_point =
        current_lane_virtual_right->enu_points;
    iflyauto::Point3f* current_lane_virtual_right_local_point =
        current_lane_virtual_right->local_point;

    const double y = right_lane_boundary_c0 + (c1 + (c2 + c3 * x) * x) * x;
    Eigen::Vector3d car_point, enu_point;
    car_point.x() = x;
    car_point.y() = y;
    car_point.z() = 0.0;
    enu_point = car2enu * car_point;
    current_lane_virtual_right_car_point[i].x = car_point.x();
    current_lane_virtual_right_car_point[i].y = car_point.y();
    current_lane_virtual_right_enu_point[i].x = enu_point.x();
    current_lane_virtual_right_enu_point[i].y = enu_point.y();
    current_lane_virtual_right_enu_point[i].z = enu_point.z();
    current_lane_virtual_right_local_point[i].x = enu_point.x();
    current_lane_virtual_right_local_point[i].y = enu_point.y();
    current_lane_virtual_right_local_point[i].z = enu_point.z();

    x += lane_boundary_delta_x;
  }
}
void VirtualLaneManager::SetGeneratedReflineToDebugInfo(
    const iflyauto::LaneReferenceLine& refline) {
  planning::common::ReferenceLine refline_to_debug;
  for (int i = 0; i < FUSION_ROAD_LINE_POLYNOMIAL_NUM; i++) {
    refline_to_debug.add_poly_coefficient_car(refline.poly_coefficient_car[i]);
  }

  for (int i = 0; i < refline.virtual_lane_refline_points_size; i++) {
    auto virtual_ref_point = refline_to_debug.add_virtual_lane_refline_points();
    virtual_ref_point->set_track_id(
        refline.virtual_lane_refline_points[i].track_id);
    virtual_ref_point->mutable_car_point()->set_x(
        refline.virtual_lane_refline_points[i].car_point.x);
    virtual_ref_point->mutable_car_point()->set_y(
        refline.virtual_lane_refline_points[i].car_point.y);
    virtual_ref_point->mutable_enu_point()->set_x(
        refline.virtual_lane_refline_points[i].enu_point.x);
    virtual_ref_point->mutable_enu_point()->set_y(
        refline.virtual_lane_refline_points[i].enu_point.y);
    virtual_ref_point->mutable_enu_point()->set_z(
        refline.virtual_lane_refline_points[i].enu_point.z);
    virtual_ref_point->mutable_local_point()->set_x(
        refline.virtual_lane_refline_points[i].local_point.x);
    virtual_ref_point->mutable_local_point()->set_y(
        refline.virtual_lane_refline_points[i].local_point.y);
    virtual_ref_point->mutable_local_point()->set_z(
        refline.virtual_lane_refline_points[i].local_point.z);
    virtual_ref_point->set_curvature(
        refline.virtual_lane_refline_points[i].curvature);
    virtual_ref_point->set_car_heading(
        refline.virtual_lane_refline_points[i].car_heading);
    virtual_ref_point->set_enu_heading(
        refline.virtual_lane_refline_points[i].enu_heading);
    virtual_ref_point->set_local_heading(
        refline.virtual_lane_refline_points[i].local_heading);
    virtual_ref_point->set_distance_to_left_road_border(
        refline.virtual_lane_refline_points[i].distance_to_left_road_border);
    virtual_ref_point->set_distance_to_right_road_border(
        refline.virtual_lane_refline_points[i].distance_to_right_road_border);
    virtual_ref_point->set_distance_to_left_lane_border(
        refline.virtual_lane_refline_points[i].distance_to_left_lane_border);
    virtual_ref_point->set_distance_to_right_lane_border(
        refline.virtual_lane_refline_points[i].distance_to_right_lane_border);

    virtual_ref_point->set_lane_width(
        refline.virtual_lane_refline_points[i].lane_width);
    virtual_ref_point->set_speed_limit_max(
        refline.virtual_lane_refline_points[i].speed_limit_max);
    virtual_ref_point->set_speed_limit_min(
        refline.virtual_lane_refline_points[i].speed_limit_min);
    virtual_ref_point->set_left_road_border_type(
        planning::common::LineType::OTHER_LINE);
    virtual_ref_point->set_right_road_border_type(
        planning::common::LineType::OTHER_LINE);
    virtual_ref_point->set_left_lane_border_type(
        planning::common::LineType::OTHER_LINE);
    virtual_ref_point->set_right_lane_border_type(
        planning::common::LineType::OTHER_LINE);
    virtual_ref_point->set_is_in_intersection(
        refline.virtual_lane_refline_points[i].is_in_intersection);
    virtual_ref_point->set_lane_type(
        planning::common::LaneType::LANETYPE_VIRTUAL);
    virtual_ref_point->set_s(refline.virtual_lane_refline_points[i].s);
  }
  auto& debug_info_pb = DebugInfoManager::GetInstance().GetDebugInfoPb();
  debug_info_pb->add_generated_refline_info()->CopyFrom(refline_to_debug);
}

bool VirtualLaneManager::update(const iflyauto::RoadInfo& roads) {
  LOG_DEBUG("update VirtualLaneManager\n");
  current_lane_ = nullptr;
  left_lane_ = nullptr;
  right_lane_ = nullptr;
  relative_id_lanes_.clear();
  Intersection_state_ = planning::common::NO_INTERSECTION;
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_generated_refline_info()
      ->Clear();

  const iflyauto::RoadInfo* roads_ptr = &roads;
  iflyauto::RoadInfo roads_virtual;

  // 1.检查lane的有效性
  if (!CheckLaneValid(roads)) {
    LOG_DEBUG("reference line invalid!\n");
    // return false;
  }
  // if (!CheckLaneValid(roads)) {
  //   // 依次为常数项、一次项、二次项、三次项
  //   std::vector<double> current_lane_virtual_poly;
  //   iflyauto::ReferenceLineMsg current_lane_virtual;
  //   if (session_->environmental_model().function_info().function_mode() ==
  //       common::DrivingFunctionInfo::ACC) {
  //     current_lane_virtual_poly = construct_reference_line_acc();
  //     construct_reference_line_msg(current_lane_virtual_poly,
  //                                  current_lane_virtual);
  //     LOG_WARNING("[VirtualLaneManager::update] ACC construct reference
  //     line");
  //   } else if (session_->environmental_model()
  //                  .function_info()
  //                  .function_mode() == common::DrivingFunctionInfo::SCC) {
  //     if (in_intersection_ == false) {
  //       in_intersection_ = true;
  //       current_lane_virtual_poly = construct_reference_line_scc();
  //       construct_reference_line_msg(current_lane_virtual_poly,
  //                                    current_lane_virtual);
  //       intersection_lane_generated_ = current_lane_virtual;
  //     } else {
  //       current_lane_virtual = intersection_lane_generated_;
  //     }
  //     LOG_WARNING("[VirtualLaneManager::update] SCC construct reference
  //     line");
  //   } else if (session_->environmental_model()
  //                  .function_info()
  //                  .function_mode() == common::DrivingFunctionInfo::HPP) {
  //     LOG_DEBUG("[hpp mode]: lane invalid!");
  //     // return false;
  //   }else {
  //     return false;
  //   }
  //   SetGeneratedReflineToDebugInfo(current_lane_virtual.lane_reference_line);

  //   // set roads_virtual
  //   roads_virtual.msg_header = roads.msg_header;
  //   roads_virtual.isp_timestamp = roads.isp_timestamp;

  //   roads_virtual.reference_line_msg[0] = current_lane_virtual;
  //   roads_virtual.reference_line_msg_size = 1;
  //   roads_virtual.local_point_valid = roads.local_point_valid;
  //   roads_ptr = &roads_virtual;
  // } else {
  //   in_intersection_ = false;
  // }

  // 2.根据地图信息，计算需要的超视距信息
  const auto& route_info = session_->environmental_model().get_route_info();
  route_info_output_ = route_info->get_route_info_output();
  // CalculateDistanceToRampSplitMergeWithSdMap(session_);

  // 3.根据计算的超视距信息，更新需要的lane信息
  relative_id_lanes_ = UpdateLanes(roads_ptr);

#ifdef X86
  int zero_order_count = 0;
  for (const auto& lane : relative_id_lanes_) {
    if (lane->get_order_id() == 0) {
      zero_order_count += 1;
    }
  }
  if (zero_order_count > 1) {
    auto compare_relative_id = [&](std::shared_ptr<VirtualLane> lane1,
                                   std::shared_ptr<VirtualLane> lane2) {
      return lane1->get_relative_id() < lane2->get_relative_id();
    };
    std::sort(relative_id_lanes_.begin(), relative_id_lanes_.end(),
              compare_relative_id);
    int count = 0;
    for (auto& lane : relative_id_lanes_) {
      lane->set_order_id(count);
      count++;
    }
  }
#endif

  // 获取track_ego_lane的依赖
  ego_lane_track_manager_.Update(route_info_output_);

  // 4.构建车道kd_path/计算自车相对于各车道的横向距离
  ego_lane_track_manager_.CalculateVirtualLaneAttributes(relative_id_lanes_);

  // 5.track自车道
  order_ids_of_same_zero_relative_id_.clear();
  // 判断自车是否处于车道数一分二场景
  for (const auto& relative_id_lane : relative_id_lanes_) {
    if (relative_id_lane->get_relative_id() == 0) {
      order_ids_of_same_zero_relative_id_.emplace_back(
          relative_id_lane->get_order_id());
    }
  }
  const auto& location_valid = session_->environmental_model().location_valid();
  auto time_start = IflyTime::Now_ms();
  if (location_valid) {
    ego_lane_track_manager_.TrackEgoLane(relative_id_lanes_,
                                         order_ids_of_same_zero_relative_id_,
                                         virtual_id_mapped_lane_);
    const bool select_ego_lane_without_plan =
        ego_lane_track_manager_.is_select_ego_lane_without_plan();
    LOG_DEBUG("select_ego_lane_without_plan: %d \n",
              select_ego_lane_without_plan);
    JSON_DEBUG_VALUE("select_ego_lane_without_plan",
                     select_ego_lane_without_plan);

    const bool select_ego_lane_with_plan =
        ego_lane_track_manager_.is_select_ego_lane_with_plan();
    LOG_DEBUG("select_ego_lane_with_plan: %d \n", select_ego_lane_with_plan);
    JSON_DEBUG_VALUE("select_ego_lane_with_plan", select_ego_lane_with_plan);
  }
  auto time_end = IflyTime::Now_ms();
  LOG_DEBUG("track_ego_lane cost:%f\n", time_end - time_start);

  ego_lane_track_manager_.SetLastZeroRelativeIdNums(
      origin_relative_id_zero_nums_);
  JSON_DEBUG_VALUE("origin_relative_id_zero_nums",
                    origin_relative_id_zero_nums_);

  // 7.根据relative_id，判断current_lane_、left_lane_、right_lane_
  UpdateAllVirtualLaneInfo();
  if (current_lane_ == nullptr) {
    LOG_ERROR("!!!current_lane is empty!!!");
    ego_lane_track_manager_.Reset();
    return false;
  }

  // 8.更新每条lane的virtual_lane_id,便于对每条lane的持续跟踪
  ego_lane_track_manager_.UpdateLaneVirtualId(
      relative_id_lanes_, virtual_id_mapped_lane_, &last_fix_lane_virtual_id_);

  // 对下游输出是否处于主路下匝道、匝道选分叉场景
  set_is_exist_ramp_on_road(ego_lane_track_manager_.is_exist_ramp_on_road());
  LOG_DEBUG("is_exist_ramp_on_road: %d \n", is_exist_ramp_on_road_);
  JSON_DEBUG_VALUE("is_exist_ramp_on_road", is_exist_ramp_on_road_);

  set_is_exist_split_on_ramp(ego_lane_track_manager_.is_exist_split_on_ramp());
  LOG_DEBUG("is_exist_split_on_ramp: %d \n", is_exist_split_on_ramp_);
  JSON_DEBUG_VALUE("is_exist_split_on_ramp", is_exist_split_on_ramp_);

  set_is_exist_split_on_expressway(
      ego_lane_track_manager_.is_exist_split_on_expressway());
  LOG_DEBUG("is_exist_split_on_expressway_: %d \n",
            is_exist_split_on_expressway_);
  JSON_DEBUG_VALUE("is_exist_split_on_expressway",
                   is_exist_split_on_expressway_);

  set_is_exist_intersection_split(
      ego_lane_track_manager_.is_exist_intersection_split());
  LOG_DEBUG("is_exist_intersection_split: %d \n", is_exist_intersection_split_);
  JSON_DEBUG_VALUE("is_exist_intersection_split", is_exist_intersection_split_);

  const bool is_in_ramp_select_split_situation =
      ego_lane_track_manager_.is_in_ramp_select_split_situation();
  LOG_DEBUG("is_in_ramp_select_split_situation: %d \n",
            is_in_ramp_select_split_situation);
  JSON_DEBUG_VALUE("is_in_ramp_select_split_situation",
                   is_in_ramp_select_split_situation);

  const bool is_on_road_select_ramp_situation =
      ego_lane_track_manager_.is_on_road_select_ramp_situation();
  LOG_DEBUG("is_on_road_select_ramp_situation: %d \n",
            is_on_road_select_ramp_situation);
  JSON_DEBUG_VALUE("is_on_road_select_ramp_situation",
                   is_on_road_select_ramp_situation);

  // 9.生成导航变道的任务
  const auto& function_info = session_->environmental_model().function_info();
  const double cancel_mlc_dis_threshold_to_route_end = 400;
  if (route_info_output_.is_ego_on_expressway &&
      function_info.function_mode() == common::DrivingFunctionInfo::NOA) {
    const bool is_inhibitory_noa_task =
        (route_info_output_.is_exist_toll_station &&
         route_info_output_.distance_to_toll_station <
             cancel_mlc_dis_threshold_to_route_end) ||
        route_info_output_.distance_to_route_end <
            cancel_mlc_dis_threshold_to_route_end;
    if (!is_inhibitory_noa_task) {
      route_info->UpdateMLCInfoDecider(relative_id_lanes_);
      for (const auto& relative_id_lane : relative_id_lanes_) {
        relative_id_lane->update_lane_tasks(
            route_info->get_route_info_output());
      }
    }
  }
  //更新route的可视化信息
  route_info->UpdateVisionInfo();

  // 9.计算自车到停止线的距离
  UpdateEgoDistanceToStopline();

  // 10.计算自车到斑马线距离
  UpdateEgoDistanceToCrosswalk(roads_ptr);

  // 11.更新路口状态
  UpdateIntersectionState();

  LOG_DEBUG("input lane:");
  auto& debug_info_manager = DebugInfoManager::GetInstance();
  auto& planning_debug_data = debug_info_manager.GetDebugInfoPb();
  auto environment_model_debug_info =
      planning_debug_data->mutable_environment_model_info();
  environment_model_debug_info->set_currrent_lane_vitual_id(
      current_lane_virtual_id_);
  LOG_DEBUG("current lane virtual id:%d\n", current_lane_virtual_id_);
  for (const auto& lane : relative_id_lanes_) {
    LOG_DEBUG(" relative id:%d, virtual id: %d,", lane->get_relative_id(),
              lane->get_virtual_id());
  }
  LOG_DEBUG("\n");
  JSON_DEBUG_VALUE("current_lane_order_id", current_lane_->get_order_id());
  JSON_DEBUG_VALUE("current_lane_virtual_id", current_lane_->get_virtual_id());
  JSON_DEBUG_VALUE("current_lane_relative_id",
                   current_lane_->get_relative_id());

  return true;
}

const std::shared_ptr<VirtualLane> VirtualLaneManager::get_lane_with_virtual_id(
    int virtual_id) const {
  if (virtual_id_mapped_lane_.find(virtual_id) !=
      virtual_id_mapped_lane_.end()) {
    LOG_DEBUG("get lane virtual %d id\n", virtual_id);
    return virtual_id_mapped_lane_.at(virtual_id);
  } else {
    LOG_DEBUG("lane virtual %d id is null\n", virtual_id);
    return nullptr;
  }
}

const std::shared_ptr<VirtualLane> VirtualLaneManager::get_lane_with_order_id(
    uint order_id) const {
  if (order_id > lane_num_ - 1) {
    return nullptr;
  }
  return relative_id_lanes_.at(order_id);
}

void VirtualLaneManager::reset() {
  last_fix_lane_virtual_id_ = 0;
  current_lane_virtual_id_ = 0;
  relative_id_lanes_.clear();
  // order_id_mapped_lanes_.clear();
  current_lane_ = nullptr;
  left_lane_ = nullptr;
  right_lane_ = nullptr;
  last_left_diff_ = 0;
  last_right_diff_ = 0;
}

std::vector<std::shared_ptr<Obstacle>>
VirtualLaneManager::get_current_lane_obstacle() {
  std::vector<std::shared_ptr<Obstacle>> tr;
  if (current_lane_ == nullptr) {
    return tr;
  }
  int virtual_id = current_lane_->get_virtual_id();
  auto reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  std::shared_ptr<ReferencePath> reference_path =
      reference_path_manager->get_reference_path_by_lane(virtual_id);
  // tr = reference_path->cal_obstacles_on_lane() //todo
  return tr;
}

std::vector<std::shared_ptr<Obstacle>>
VirtualLaneManager::get_left_lane_obstacle() {
  std::vector<std::shared_ptr<Obstacle>> tr;
  if (left_lane_ == nullptr) {
    return tr;
  }
  int virtual_id = left_lane_->get_virtual_id();
  auto reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  std::shared_ptr<ReferencePath> reference_path =
      reference_path_manager->get_reference_path_by_lane(virtual_id);
  // tr = reference_path->cal_obstacles_on_lane() //todo
  return tr;
}

std::vector<std::shared_ptr<Obstacle>>
VirtualLaneManager::get_right_lane_obstacle() {
  std::vector<std::shared_ptr<Obstacle>> tr;
  if (right_lane_ == nullptr) {
    return tr;
  }
  int virtual_id = right_lane_->get_virtual_id();
  auto reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  std::shared_ptr<ReferencePath> reference_path =
      reference_path_manager->get_reference_path_by_lane(virtual_id);
  // tr = reference_path->cal_obstacles_on_lane() //todo
  return tr;
}

bool VirtualLaneManager::has_lane(int virtual_lane_id) {
  if (virtual_id_mapped_lane_.find(virtual_lane_id) !=
      virtual_id_mapped_lane_.end()) {
    return true;
  } else {
    return false;
  }
}

double VirtualLaneManager::get_distance_to_dash_line(
    const RequestType direction, uint virtual_id) const {
  assert(direction == RIGHT_CHANGE || direction == LEFT_CHANGE);

  double distance_to_dash_line = 0.0;
  auto lane = get_lane_with_virtual_id(virtual_id);
  if (lane == nullptr) {
    return distance_to_dash_line;
  }

  if (direction == LEFT_CHANGE) {
    const int num_of_type_segments =
        lane->get_left_lane_boundary().type_segments_size;
    for (int i = 0; i < num_of_type_segments; i++) {
      const auto& type_segment =
          lane->get_left_lane_boundary().type_segments[i];
      if (type_segment.type == iflyauto::LaneBoundaryType_MARKING_DASHED ||
          type_segment.type ==
              iflyauto::LaneBoundaryType_MARKING_SHORT_DASHED ||
          type_segment.type ==
              iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED ||
          type_segment.type ==
              iflyauto::LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED) {
        distance_to_dash_line += type_segment.length;
      } else {
        break;
      }
    }
  } else if (direction == RIGHT_CHANGE) {
    const int num_of_type_segments =
        lane->get_right_lane_boundary().type_segments_size;
    for (int i = 0; i < num_of_type_segments; i++) {
      const auto& type_segment =
          lane->get_right_lane_boundary().type_segments[i];
      if (type_segment.type == iflyauto::LaneBoundaryType_MARKING_DASHED ||
          type_segment.type ==
              iflyauto::LaneBoundaryType_MARKING_SHORT_DASHED ||
          type_segment.type ==
              iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED ||
          type_segment.type ==
              iflyauto::LaneBoundaryType_MARKING_LEFT_DASHED_RIGHT_SOLID) {
        distance_to_dash_line += type_segment.length;
      } else {
        break;
      }
    }
  }
  LOG_DEBUG("get_distance_to_dash_line :%f\n", distance_to_dash_line);
  return distance_to_dash_line;
}

double VirtualLaneManager::get_distance_to_final_dash_line(
    const RequestType direction, uint order_id) const {
  auto virtual_lane = get_lane_with_order_id(order_id);
  if (virtual_lane == nullptr) {
    return std::numeric_limits<double>::max();
  }
  return lc_map_decision_offset(virtual_lane);
}

int VirtualLaneManager::get_lane_index(
    const std::shared_ptr<VirtualLane> virtual_lane) const {
  if (virtual_lane != nullptr) {
    return virtual_lane->get_relative_id() -
           relative_id_lanes_.at(0)->get_relative_id();
  }
  return 0;
}

int VirtualLaneManager::get_tasks(
    const std::shared_ptr<VirtualLane> virtual_lane) const {
  int current_tasks = 0;
  if (virtual_lane != nullptr) {
    auto current_tasks_vector = virtual_lane->get_current_tasks();
    if (current_tasks_vector.empty()) {
      return 0;
    }
    for (int i = 0; i < current_tasks_vector.size(); i++) {
      if (current_tasks_vector[i] != current_tasks_vector[0]) {
        break;
      }
      current_tasks += current_tasks_vector[i];
    }
    // clip tasks according to lane nums
    int lane_index = get_lane_index(virtual_lane);
    int right_lane_nums = std::max((int)lane_num_ - lane_index - 1, 0);
    int left_lane_nums = lane_index;
    current_tasks =
        std::max(std::min(current_tasks, right_lane_nums), -left_lane_nums);
    return current_tasks;
  } else {
    return 0;
  }
}

bool VirtualLaneManager::must_change_lane(
    const std::shared_ptr<VirtualLane> virtual_lane,
    double on_route_distance_threshold) const {
  if (virtual_lane == nullptr) {
    return 0;
  }
  return lc_map_decision(virtual_lane) != 0 &&
         lc_map_decision_offset(virtual_lane) < on_route_distance_threshold;
}

int VirtualLaneManager::lc_map_decision(
    const std::shared_ptr<VirtualLane> virtual_lane) const {
  if (virtual_lane == nullptr) {
    return 0;
  }
  int tasks_id = get_tasks(virtual_lane);
  int lane_index = get_lane_index(virtual_lane);

  // hack valid and on rightest way
  if (virtual_lane->hack() && lane_index == (lane_num_ - 1)) {
    if (tasks_id <= 0) {
      tasks_id = std::min(tasks_id, -1);
    }
  }

  return tasks_id;
}

double VirtualLaneManager::get_distance_to_first_road_merge() const {
  double distance_to_first_road_merge = 5000.;

  return distance_to_first_road_merge;
}

double VirtualLaneManager::get_distance_to_first_road_split() const {
  double distance_to_first_road_split = 5000.;

  return distance_to_first_road_split;
}

bool VirtualLaneManager::UpdateEgoDistanceToStopline() {
  const auto& stop_line = current_lane_->get_stop_line();
  double ego_to_stopline_dis = NL_NMAX;
  const auto& location_valid = session_->environmental_model().location_valid();
  if (stop_line.type == iflyauto::LANE_LINE_TYPE_STOPLINE) {
    if (location_valid) {
      const auto& ego_state =
        session_->environmental_model().get_ego_state_manager();
      Point2D ego_cart(ego_state->ego_pose().x, ego_state->ego_pose().y);

      const auto frenet_coord = current_lane_->get_lane_frenet_coord();
      if (frenet_coord != nullptr) {
        std::vector<Point2D> stop_line_points_vec;
        stop_line_points_vec.resize(stop_line.enu_points_size);
        for (int i = 0; i < stop_line.enu_points_size; i++) {
          Point2D stop_line_pt;
          stop_line_pt.x = stop_line.enu_points[i].x;
          stop_line_pt.y = stop_line.enu_points[i].y;
          if (!frenet_coord->XYToSL(stop_line_pt, stop_line_points_vec[i])) {
            stop_line_points_vec[i].x = NL_NMAX;
            stop_line_points_vec[i].y = NL_NMAX;
          }
        }
        auto compare_y = [&](Point2D p1, Point2D p2) { return p1.y < p2.y; };
        std::sort(stop_line_points_vec.begin(), stop_line_points_vec.end(), compare_y);
        int idx = -1;
        for (int i = 0; i < stop_line_points_vec.size() - 1; i++) {
          if (stop_line_points_vec[i].y < 0.0 &&
              stop_line_points_vec[i + 1].y > 0.0 && stop_line_points_vec[i + 1].y < 100.0) {
            idx = i;
            break;
          }
        }
        if (idx != -1) {
          double ego_s = 0.0;
          double ego_l = 0.0;
          if (frenet_coord->XYToSL(ego_cart.x, ego_cart.y, &ego_s, &ego_l)) {
            ego_to_stopline_dis = (stop_line_points_vec[idx].x + stop_line_points_vec[idx + 1].x) / 2.0 - ego_s;
          }
        }

      }
    } else {
      std::vector<iflyauto::Point2f> stop_line_points_vec;
      stop_line_points_vec.resize(stop_line.car_points_size);
      for (int i = 0; i < stop_line.car_points_size; i++) {
        stop_line_points_vec[i] = stop_line.car_points[i];
      }
      auto compare_y = [&](iflyauto::Point2f p1, iflyauto::Point2f p2) {
        return p1.y < p2.y;
      };
      std::sort(stop_line_points_vec.begin(), stop_line_points_vec.end(),
                compare_y);
      int idx = -1;
      for (int i = 0; i < stop_line_points_vec.size() - 1; i++) {
        if (stop_line_points_vec[i].y < 0.0 &&
            stop_line_points_vec[i + 1].y > 0.0) {
          idx = i;
          break;
        }
      }

      if (idx != -1) {
        planning::planning_math::Vec2d p_left(stop_line_points_vec[idx].x,
                                              stop_line_points_vec[idx].y);
        planning::planning_math::Vec2d p_right(stop_line_points_vec[idx + 1].x,
                                              stop_line_points_vec[idx + 1].y);
        planning::planning_math::LineSegment2d line_seg(p_left, p_right);
        int id = 0;
        planning::StopLine lane_stop_line = planning::StopLine(id, line_seg);
        double raw_dis =
            lane_stop_line.RawDistanceTo(planning::planning_math::Vec2d(0, 0));
        ego_to_stopline_dis = -1.0 * raw_dis;
      }

    }
  }
  stopline_window_.pop_front();
  stopline_window_.push_back(ego_to_stopline_dis);

  if (stopline_window_[0] < 200.0 && stopline_window_[1] < 200.0 &&
      stopline_window_[2] < 200.0) {
    distance_to_stopline_ = ego_to_stopline_dis;
  } else if (stopline_window_[0] > 200.0 && stopline_window_[1] > 200.0 &&
             stopline_window_[2] > 200.0) {
    distance_to_stopline_ = NL_NMAX;
  }
  JSON_DEBUG_VALUE("distance_to_stopline", distance_to_stopline_);
  return true;
}

bool VirtualLaneManager::UpdateEgoDistanceToCrosswalk(
    const iflyauto::RoadInfo* roads_ptr) {
  double ego_to_crosswalk_dis = NL_NMAX;
  const auto& location_valid = session_->environmental_model().location_valid();
  std::vector<std::pair<int, double>> cw_idx_dis_vec;
  std::vector<std::vector<iflyauto::Point2f>> cross_walk_pts_vec;
  for (int i = 0; i < roads_ptr->lane_ground_markings_size; i++) {
    const auto& cross_walk = roads_ptr->lane_ground_markings[i];
    if (cross_walk.turn_type ==
            iflyauto::LaneDrivableDirection_DIRECTION_CROSS_LINE &&
        std::abs(cross_walk.orientation_angle) < 0.5) {
      std::vector<iflyauto::Point2f> cw_pts;
      if (location_valid) {
        cw_pts.resize(cross_walk.local_ground_marking_points_set_size);
        for (int idx = 0; idx < cross_walk.local_ground_marking_points_set_size;
            idx++) {
          cw_pts[idx] = cross_walk.local_ground_marking_points_set[idx];
        }
      } else {
        cw_pts.resize(cross_walk.ground_marking_points_set_size);
        for (int idx = 0; idx < cross_walk.ground_marking_points_set_size;
            idx++) {
          cw_pts[idx] = cross_walk.ground_marking_points_set[idx];
        }
      }
      cross_walk_pts_vec.push_back(cw_pts);
    }
  }

  if (location_valid) {
    const auto& ego_state =
        session_->environmental_model().get_ego_state_manager();
    Point2D ego_cart(ego_state->ego_pose().x, ego_state->ego_pose().y);
    const auto frenet_coord = current_lane_->get_lane_frenet_coord();
    if (frenet_coord != nullptr) {
      for (int j = 0; j < cross_walk_pts_vec.size(); j++) {
        auto& cw_pt_vec = cross_walk_pts_vec[j];
        std::vector<Point2D> cw_frenet_pt_vec;
        cw_frenet_pt_vec.resize(cw_pt_vec.size());
        for (int i = 0; i < cw_pt_vec.size(); i++) {
          Point2D cw_pt;
          cw_pt.x = cw_pt_vec[i].x;
          cw_pt.y = cw_pt_vec[i].y;
          if (!frenet_coord->XYToSL(cw_pt, cw_frenet_pt_vec[i])) {
            cw_frenet_pt_vec[i].x = NL_NMAX;
            cw_frenet_pt_vec[i].y = NL_NMAX;
          }
        }
        auto compare_y = [&](Point2D p1, Point2D p2) { return p1.y < p2.y; };
        std::sort(cw_frenet_pt_vec.begin(), cw_frenet_pt_vec.end(), compare_y);
        int idx = -1;
        for (int i = 0; i < cw_frenet_pt_vec.size() - 1; i++) {
          if (cw_frenet_pt_vec[i].y < 0.0 && cw_frenet_pt_vec[i + 1].y > 0.0
              && cw_frenet_pt_vec[i + 1].y < 100.0) {
            idx = i;
            break;
          }
        }
        if (idx != -1) {
          double neg_l_min_s = NL_NMAX;
          for (int i = 0; i <= idx; i++) {
            if (cw_frenet_pt_vec[i].x < neg_l_min_s) {
              neg_l_min_s = cw_frenet_pt_vec[i].x;
            }
          }
          double pos_l_min_s = NL_NMAX;
          for (int i = idx + 1; i < cw_frenet_pt_vec.size(); i++) {
            if (cw_frenet_pt_vec[i].x < pos_l_min_s) {
              pos_l_min_s = cw_frenet_pt_vec[i].x;
            }
          }
          double ego_s = 0.0;
          double ego_l = 0.0;
          if (frenet_coord->XYToSL(ego_cart.x, ego_cart.y, &ego_s, &ego_l)) {
            double ego_cw_dis = (neg_l_min_s + pos_l_min_s) * 0.5 - ego_s;
            cw_idx_dis_vec.push_back(std::make_pair(j, ego_cw_dis));
          }
        } else {
          continue;
        }

      }
    }
  } else {
    auto compare_y = [&](iflyauto::Point2f p1, iflyauto::Point2f p2) {
      return p1.y < p2.y;
    };
    for (int j = 0; j < cross_walk_pts_vec.size(); j++) {
      auto& cw_pt_vec = cross_walk_pts_vec[j];
      std::sort(cw_pt_vec.begin(), cw_pt_vec.end(), compare_y);
      int idx = -1;
      for (int i = 0; i < cw_pt_vec.size() - 1; i++) {
        if (cw_pt_vec[i].y < 0.0 && cw_pt_vec[i + 1].y > 0.0) {
          idx = i;
          break;
        }
      }
      if (idx != -1) {
        double neg_l_min_s = NL_NMAX;
        for (int i = 0; i <= idx; i++) {
          if (cw_pt_vec[i].x < neg_l_min_s) {
            neg_l_min_s = cw_pt_vec[i].x;
          }
        }
        double pos_l_min_s = NL_NMAX;
        for (int i = idx + 1; i < cw_pt_vec.size(); i++) {
          if (cw_pt_vec[i].x < pos_l_min_s) {
            pos_l_min_s = cw_pt_vec[i].x;
          }
        }
        cw_idx_dis_vec.push_back(std::make_pair(j, 0.5 * (neg_l_min_s + pos_l_min_s)));
      } else {
        continue;
      }
    }
  }

  // more than one, select the min dis crosswalk
  if (cw_idx_dis_vec.size() > 0) {
    int min_dis = NL_NMAX;
    for (int i = 0; i < cw_idx_dis_vec.size(); i++) {
      if (cw_idx_dis_vec[i].second < min_dis) {
        min_dis = cw_idx_dis_vec[i].second;
      }
    }
    ego_to_crosswalk_dis = min_dis;
  }
  crosswalk_window_.pop_front();
  crosswalk_window_.push_back(ego_to_crosswalk_dis);

  if (crosswalk_window_[0] < 200.0 && crosswalk_window_[1] < 200.0 &&
      crosswalk_window_[2] < 200.0) {
    distance_to_crosswalk_ = ego_to_crosswalk_dis;
  } else if (crosswalk_window_[0] > 200.0 && crosswalk_window_[1] > 200.0 &&
             crosswalk_window_[2] > 200.0) {
    distance_to_crosswalk_ = NL_NMAX;
  }
  JSON_DEBUG_VALUE("distance_to_crosswalk", distance_to_crosswalk_);
  return true;
}

bool VirtualLaneManager::UpdateIntersectionState() {
  if (session_->is_hpp_scene()) {
    Intersection_state_ = planning::common::NO_INTERSECTION;
    return true;
  }
  double ego_pos_x = 0.0;
  if (current_lane_->get_left_lane_boundary().car_points_size > 0 &&
      current_lane_->get_right_lane_boundary().car_points_size > 0) {
    double first_left_x =
        current_lane_->get_left_lane_boundary().car_points[0].x;
    double first_right_x =
        current_lane_->get_right_lane_boundary().car_points[0].x;
    ego_pos_x = std::max(0 - first_left_x, 0 - first_right_x);
  } else {
    Intersection_state_ = planning::common::IN_INTERSECTION;
    return true;
  }
  if ((distance_to_stopline_ < 35.0 && distance_to_stopline_ > 3.0) ||
      ((distance_to_crosswalk_ < 38.0 && distance_to_crosswalk_ > 5.0) &&
       !IsPosXOnVirtualLaneType(ego_pos_x))) {
    Intersection_state_ = planning::common::APPROACH_INTERSECTION;
  } else if (-1.0 < distance_to_stopline_ && distance_to_stopline_ <= 3.0) {
    Intersection_state_ = planning::common::IN_INTERSECTION;
  } else {
    if ((-1.5 < distance_to_crosswalk_ && distance_to_crosswalk_ <= 5.0) &&
        !IsPosXOnVirtualLaneType(ego_pos_x)) {
      Intersection_state_ = planning::common::IN_INTERSECTION;
    } else if (distance_to_crosswalk_ <= -1.5 &&
               !IsPosXOnVirtualLaneType(ego_pos_x)) {
      Intersection_state_ = planning::common::NO_INTERSECTION;
    } else if (IsPosXOnVirtualLaneType(ego_pos_x) &&
               !IsPosXOnVirtualLaneType(ego_pos_x + 8.0)) {
      Intersection_state_ = planning::common::OFF_INTERSECTION;
    } else if (IsPosXOnVirtualLaneType(ego_pos_x)) {
      if (IsEgoBothSidesHaveRoadBorder()) {
        Intersection_state_ = planning::common::NO_INTERSECTION;
      } else {
        Intersection_state_ = planning::common::IN_INTERSECTION;
      }
    } else {
      Intersection_state_ = planning::common::NO_INTERSECTION;
    }
  }
  return true;
}

bool VirtualLaneManager::IsEgoBothSidesHaveRoadBorder() {
  bool rslt = false;
  int ego_pos_idx = -1;
  double ego_pos_s = 0.0;
  const auto& vlane_ref_pts = current_lane_->lane_points();
  for (int i = 0; i < vlane_ref_pts.size(); i++) {
    if (vlane_ref_pts[i].car_point.x > 0.0) {
      ego_pos_idx = i;
      ego_pos_s = vlane_ref_pts[i].s;
      break;
    }
  }
  if (ego_pos_idx == -1) {
    return rslt;
  }
  double ego_pos_front_end = ego_pos_s + 8.0;
  int i = ego_pos_idx;
  for (; i < vlane_ref_pts.size(); i++) {
    if (vlane_ref_pts[i].s < ego_pos_front_end &&
        vlane_ref_pts[i].distance_to_left_road_border < 20.0 &&
        vlane_ref_pts[i].distance_to_right_road_border < 20.0) {
      continue;
    } else {
      break;
    }
  }
  if (i < vlane_ref_pts.size() && vlane_ref_pts[i].s >= ego_pos_front_end) {
    rslt = true;
  }
  return rslt;
}

bool VirtualLaneManager::IsPosXOnVirtualLaneType(double x_pos) {
  bool rslt = false;
  bool lane_is_virtual = false;
  const auto& lane_types_vec = current_lane_->get_lane_types();
  for (int ind = 0; ind < lane_types_vec.size(); ind++) {
    if (lane_types_vec[ind].begin <= x_pos &&
        x_pos <= lane_types_vec[ind].end) {
      if (lane_types_vec[ind].type == iflyauto::LANETYPE_VIRTUAL) {
        lane_is_virtual = true;
      } else {
        lane_is_virtual = false;
      }
      break;
    }
  }

  const auto& l_boundry = current_lane_->get_left_lane_boundary();
  bool l_boundry_is_virtual = false;
  for (int j = 0; j < l_boundry.type_segments_size; j++) {
    double seg_begin = -1;
    double seg_end = -1;
    if (j == 0) {
      seg_begin = 0;
      seg_end = l_boundry.type_segments[j].length;
      if (seg_begin <= x_pos && x_pos <= seg_end) {
        if (l_boundry.type_segments[j].type ==
            iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
          l_boundry_is_virtual = true;
        } else {
          l_boundry_is_virtual = false;
        }
        break;
      }
    } else {
      seg_begin = 0;
      for (int i = 0; i <= j - 1; i++) {
        seg_begin += l_boundry.type_segments[i].length;
      }
      seg_end = 0;
      for (int i = 0; i <= j; i++) {
        seg_end += l_boundry.type_segments[i].length;
      }
      if (seg_begin <= x_pos && x_pos <= seg_end) {
        if (l_boundry.type_segments[j].type ==
            iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
          l_boundry_is_virtual = true;
        } else {
          l_boundry_is_virtual = false;
        }
        break;
      }
    }
  }

  const auto& r_boundry = current_lane_->get_right_lane_boundary();
  bool r_boundry_is_virtual = false;
  for (int j = 0; j < r_boundry.type_segments_size; j++) {
    double seg_begin = -1;
    double seg_end = -1;
    if (j == 0) {
      seg_begin = 0;
      seg_end = r_boundry.type_segments[j].length;
      if (seg_begin <= x_pos && x_pos <= seg_end) {
        if (r_boundry.type_segments[j].type ==
            iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
          r_boundry_is_virtual = true;
        } else {
          r_boundry_is_virtual = false;
        }
        break;
      }
    } else {
      seg_begin = 0;
      for (int i = 0; i <= j - 1; i++) {
        seg_begin += r_boundry.type_segments[i].length;
      }
      seg_end = 0;
      for (int i = 0; i <= j; i++) {
        seg_end += r_boundry.type_segments[i].length;
      }
      if (seg_begin <= x_pos && x_pos <= seg_end) {
        if (r_boundry.type_segments[j].type ==
            iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
          r_boundry_is_virtual = true;
        } else {
          r_boundry_is_virtual = false;
        }
        break;
      }
    }
  }
  rslt = lane_is_virtual && (l_boundry_is_virtual && r_boundry_is_virtual);
  return rslt;
}

const double max_y_interval = 5.0;
bool VirtualLaneManager::CheckLaneValid(const iflyauto::RoadInfo& roads) {
  const double current_lane_y_thrs = 5.0;

  // 1.车道线为空
  // 2.车道中心线出现间隔>5m的点，则拒绝
  // 3.所有车道线离自车横向距离都>5m
  bool lane_valid = true;
  bool current_lane_exist = false;
  bool y_interval_valid = true;
  std::vector<int> y_interval_invalid_idx_vec;
  std::cout << "roads.reference_line_msg_size:" << roads.reference_line_msg_size
            << std::endl;
  if (roads.reference_line_msg_size == 0) {
    lane_valid = false;
    return lane_valid;
  } else {
    for (int i = 0; i < roads.reference_line_msg_size; i++) {
      const auto& ref_line = roads.reference_line_msg[i];
      const auto& points =
          ref_line.lane_reference_line.virtual_lane_refline_points;
      const int num_of_reflane_point =
          ref_line.lane_reference_line.virtual_lane_refline_points_size;
      // TBD: 需要关注,C结构体应该是一上来就分配好大小了,这里可能常为1
      if (num_of_reflane_point < 2) {
        LOG_ERROR("reflane point num less than 2 \n");
        continue;  // 如果参考点个数小于2个，则跳过此参考线
      }
      for (int j = 1; j < num_of_reflane_point; j++) {
        const auto& prev_point = points[j - 1].car_point;
        const auto& current_point = points[j].car_point;
        current_lane_exist |= (current_point.y < current_lane_y_thrs);
        double y_interval = std::fabs(current_point.y - prev_point.y);
        if (y_interval > max_y_interval) {
          LOG_ERROR("lane_valid is error \n");
          // y_interval_valid = false;
          y_interval_invalid_idx_vec.emplace_back(i);
          break;
        }
      }
    }
    y_interval_valid =
        y_interval_invalid_idx_vec.size() < roads.reference_line_msg_size;
  }

  // TBD: 校验车道中心线的连续性
  lane_valid = y_interval_valid && current_lane_exist;
  return lane_valid;
}
std::vector<std::shared_ptr<VirtualLane>> VirtualLaneManager::UpdateLanes(
    const iflyauto::RoadInfo* roads_ptr) {
  std::vector<std::shared_ptr<VirtualLane>> relative_id_lanes;
  //(1)按照order_id的顺序排序输入lanes
  std::vector<iflyauto::ReferenceLineMsg> lane_msg;
  lane_msg.reserve(roads_ptr->reference_line_msg_size);
  origin_relative_id_zero_nums_ = 0;
  for (int i = 0; i < roads_ptr->reference_line_msg_size; ++i) {
    if (roads_ptr->reference_line_msg[i]
            .lane_reference_line.virtual_lane_refline_points_size < 3) {
      continue;
    }
    if (roads_ptr->reference_line_msg[i].relative_id == 0) {
      origin_relative_id_zero_nums_ = origin_relative_id_zero_nums_ + 1;
    }
    lane_msg.emplace_back(roads_ptr->reference_line_msg[i]);
  }
  auto compare_order_id = [&](iflyauto::ReferenceLineMsg lane1,
                              iflyauto::ReferenceLineMsg lane2) {
    return lane1.order_id < lane2.order_id;
  };
  std::sort(lane_msg.begin(), lane_msg.end(), compare_order_id);

  //(2)更新来自道路融合模块的lanes，
  for (auto& lane : lane_msg) {
    std::shared_ptr<VirtualLane> virtual_lane_tmp =
        std::make_shared<VirtualLane>();
    virtual_lane_tmp->update_data(lane);
    LOG_DEBUG("lane relative_id:%d, order_id:%d\n", lane.relative_id,
              lane.order_id);
    // if (virtual_lane_tmp->get_lane_type() == iflyauto::LANETYPE_EMERGENCY)
    //   break;
    relative_id_lanes.emplace_back(virtual_lane_tmp);
  }
  lane_num_ = relative_id_lanes.size();
  return relative_id_lanes;
}

void VirtualLaneManager::UpdateAllVirtualLaneInfo() {
  auto compare_relative_id = [&](std::shared_ptr<VirtualLane> lane1,
                                 std::shared_ptr<VirtualLane> lane2) {
    return lane1->get_relative_id() < lane2->get_relative_id();
  };
  std::sort(relative_id_lanes_.begin(), relative_id_lanes_.end(),
            compare_relative_id);

  for (const auto& relative_id_lane : relative_id_lanes_) {
    if (relative_id_lane->get_relative_id() == 0) {
      current_lane_ = relative_id_lane;
      LOG_DEBUG("create current_lane_\n");
    } else if (relative_id_lane->get_relative_id() == -1) {
      left_lane_ = relative_id_lane;
    } else if (relative_id_lane->get_relative_id() == 1) {
      right_lane_ = relative_id_lane;
    }
  }
}

std::shared_ptr<planning_math::KDPath> VirtualLaneManager::MakeBoundaryPath(
    const iflyauto::LaneBoundary& boundary) {
  std::shared_ptr<planning_math::KDPath> boundary_path;
  std::vector<planning_math::Vec2d> center_line_points;
  center_line_points.clear();

  for (const auto& point : boundary.enu_points) {
    if (center_line_points.size() < boundary.enu_points_size) {
      center_line_points.emplace_back(point.x, point.y);
    } else {
      break;
    }
  }

  if (center_line_points.size() <= 2) {
    return nullptr;
  }
  std::vector<planning_math::PathPoint> boundary_points;
  boundary_points.reserve(center_line_points.size());
  for (const auto& point : center_line_points) {
    if (std::isnan(point.x()) || std::isnan(point.y())) {
      LOG_ERROR("update_center_line_points: skip NaN point");
      continue;
    }
    auto pt = planning_math::PathPoint(point.x(), point.y());
    // std ::cout << "path_point: " << pt.x() << "," << pt.y() <<std::endl;
    if (not boundary_points.empty()) {
      auto& last_pt = boundary_points.back();
      if (planning_math::Vec2d(last_pt.x() - pt.x(), last_pt.y() - pt.y())
              .Length() < 1e-2) {
        continue;
      }
    }
    boundary_points.emplace_back(pt);
  }

  if (boundary_points.size() <= 2) {
    return nullptr;
  }
  boundary_path =
      std::make_shared<planning_math::KDPath>(std::move(boundary_points));

  return boundary_path;
}

std::shared_ptr<VirtualLane> VirtualLaneManager::GetNearestLane(
    Point2D point, double* nearest_s, double* nearest_l) {
  double default_lateral_offset = 10.0;
  int current_order_id = -1;
  for (const auto& relative_id_lane : relative_id_lanes_) {
    if (relative_id_lane == nullptr) {
      continue;
    }
    std::shared_ptr<planning_math::KDPath> frenet_coord;
    if (relative_id_lane->get_lane_frenet_coord() == nullptr) {
      continue;
    }
    frenet_coord = relative_id_lane->get_lane_frenet_coord();
    Point2D frenet_point;
    double point_s = 0.0;
    double point_l = 0.0;
    if (!frenet_coord->XYToSL(point, frenet_point)) {
      continue;
    } else {
      point_l = fabs(frenet_point.y);
    }
    if (point_l < default_lateral_offset) {
      default_lateral_offset = point_l;
      current_order_id = relative_id_lane->get_order_id();
      *nearest_s = point_s;
      *nearest_l = point_l;
    }
  }

  for (const auto& relative_id_lane : relative_id_lanes_) {
    if (relative_id_lane->get_order_id() == current_order_id) {
      return relative_id_lane;
    } else {
      continue;
    }
  }
  return nullptr;
}
}  // namespace planning
