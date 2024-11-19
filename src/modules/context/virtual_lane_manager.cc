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
#include "ifly_parking_map_c.h"
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
    planning::framework::Session* session) {
  session_ = session;
  config_ = config_builder->cast<EgoPlanningVirtualLaneManagerConfig>();
  is_select_split_nearing_ramp_ = config_.is_select_split_nearing_ramp;
  ego_lane_track_manager_ = std::make_shared<EgoLaneTrackManger>(session);
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
  for (size_t i = 0; i < NUM_OF_REFLINE_POINT; ++i) {
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
      NUM_OF_REFLINE_POINT;

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
  for (int i = 0; i < NUM_OF_POLYNOMIAL; i++) {
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
  is_ego_on_expressway_ = false;
  is_ego_on_city_expressway_hmi_ = false;
  is_ego_on_expressway_hmi_ = false;
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_generated_refline_info()
      ->Clear();

  const iflyauto::RoadInfo* roads_ptr = &roads;
  iflyauto::RoadInfo roads_virtual;

  // 1.检查lane的有效性
  if (!CheckLaneValid(roads)) {
    // 依次为常数项、一次项、二次项、三次项
    std::vector<double> current_lane_virtual_poly;
    iflyauto::ReferenceLineMsg current_lane_virtual;
    if (session_->environmental_model().function_info().function_mode() ==
        common::DrivingFunctionInfo::ACC) {
      current_lane_virtual_poly = construct_reference_line_acc();
      construct_reference_line_msg(current_lane_virtual_poly,
                                   current_lane_virtual);
      LOG_WARNING("[VirtualLaneManager::update] ACC construct reference line");
    } else if (session_->environmental_model()
                   .function_info()
                   .function_mode() == common::DrivingFunctionInfo::SCC) {
      if (in_intersection_ == false) {
        in_intersection_ = true;
        current_lane_virtual_poly = construct_reference_line_scc();
        construct_reference_line_msg(current_lane_virtual_poly,
                                     current_lane_virtual);
        intersection_lane_generated_ = current_lane_virtual;
      } else {
        current_lane_virtual = intersection_lane_generated_;
      }
      LOG_WARNING("[VirtualLaneManager::update] SCC construct reference line");
    } else {
      return false;
    }
    SetGeneratedReflineToDebugInfo(current_lane_virtual.lane_reference_line);

    // set roads_virtual
    roads_virtual.msg_header = roads.msg_header;
    roads_virtual.isp_timestamp = roads.isp_timestamp;

    roads_virtual.reference_line_msg[0] = current_lane_virtual;
    roads_virtual.reference_line_msg_size = 1;
    roads_virtual.local_point_valid = roads.local_point_valid;
    roads_ptr = &roads_virtual;
  } else {
    in_intersection_ = false;
  }

  // 2.根据地图信息，计算需要的超视距信息
  CalculateDistanceToRampSplitMergeWithSdMap(session_);

  // if (session_->is_hpp_scene() && GetCurrentNearestLane(*session_)) {
  // CalculateHPPInfo(session_);
  // CalculateDistanceToTargetSlot(session_);
  // CalculateDistanceToNextSpeedBump(session_);
  // }
  LOG_DEBUG(
      "dis_to_ramp: %f, dis_to_first_road_split: %f, "
      "distance_to_first_road_merge_: %f \n",
      dis_to_ramp_, distance_to_first_road_split_,
      distance_to_first_road_merge_);
  LOG_DEBUG("is_nearing_ramp: %d, ramp_direction_: %d \n", is_nearing_ramp_,
            ramp_direction_);
  LOG_DEBUG("dis to tar slot: %f, distance_to_frist_speed_bump: %f \n",
            distance_to_target_slot_, distance_to_next_speed_bump_);
  JSON_DEBUG_VALUE("is_ego_on_expressway", is_ego_on_expressway_);
  JSON_DEBUG_VALUE("ramp_direction", static_cast<int>(ramp_direction_));
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
  ego_lane_track_manager_->Update(
      is_ego_on_expressway_, is_on_ramp_, dis_to_ramp_, is_leaving_ramp_,
      first_split_dir_dis_info_, distance_to_first_road_merge_,
      distance_to_first_road_split_, current_segment_passed_distance_,
      split_dir_dis_info_list_, sum_dis_to_last_split_point_);

  // 4.构建车道kd_path/计算自车相对于各车道的横向距离
  ego_lane_track_manager_->CalculateVirtualLaneAttributes(relative_id_lanes_);

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
    ego_lane_track_manager_->TrackEgoLane(relative_id_lanes_,
                                          order_ids_of_same_zero_relative_id_,
                                          virtual_id_mapped_lane_);
    const bool select_ego_lane_without_plan =
        ego_lane_track_manager_->is_select_ego_lane_without_plan();
    LOG_DEBUG("select_ego_lane_without_plan: %d \n",
              select_ego_lane_without_plan);
    JSON_DEBUG_VALUE("select_ego_lane_without_plan",
                     select_ego_lane_without_plan);

    const bool select_ego_lane_with_plan =
        ego_lane_track_manager_->is_select_ego_lane_with_plan();
    LOG_DEBUG("select_ego_lane_with_plan: %d \n", select_ego_lane_with_plan);
    JSON_DEBUG_VALUE("select_ego_lane_with_plan", select_ego_lane_with_plan);
  }
  auto time_end = IflyTime::Now_ms();
  LOG_DEBUG("track_ego_lane cost:%f\n", time_end - time_start);

  ego_lane_track_manager_->SetLastZeroRelativeIdNums(
      origin_relative_id_zero_nums_);

  // 6.生成导航变道的任务
  const double cancel_mlc_dis_threshold_to_route_end = 400;
  if (is_ego_on_expressway_) {
    const bool is_inhibitory_noa_task =
        (is_exist_toll_station_ &&
         distance_to_toll_station_ < cancel_mlc_dis_threshold_to_route_end) ||
        distance_to_route_end_ < cancel_mlc_dis_threshold_to_route_end;
    if (!is_inhibitory_noa_task) {
      GenerateLaneChangeTasksForNOA();
    }
  }

  // 7.根据relative_id，判断current_lane_、left_lane_、right_lane_
  UpdateAllVirtualLaneInfo();
  if (current_lane_ == nullptr) {
    LOG_ERROR("!!!current_lane is empty!!!");
    ego_lane_track_manager_.reset();
    return false;
  }

  // 8.更新每条lane的virtual_lane_id,便于对每条lane的持续跟踪
  ego_lane_track_manager_->UpdateLaneVirtualId(
      relative_id_lanes_, virtual_id_mapped_lane_, &last_fix_lane_virtual_id_);

  // 对下游输出是否处于主路下匝道、匝道选分叉场景
  set_is_exist_ramp_on_road(ego_lane_track_manager_->is_exist_ramp_on_road());
  LOG_DEBUG("is_exist_ramp_on_road: %d \n", is_exist_ramp_on_road_);
  JSON_DEBUG_VALUE("is_exist_ramp_on_road", is_exist_ramp_on_road_);

  set_is_exist_split_on_ramp(ego_lane_track_manager_->is_exist_split_on_ramp());
  LOG_DEBUG("is_exist_split_on_ramp: %d \n", is_exist_split_on_ramp_);
  JSON_DEBUG_VALUE("is_exist_split_on_ramp", is_exist_split_on_ramp_);

  set_is_exist_split_on_expressway(ego_lane_track_manager_->is_exist_split_on_expressway());
  LOG_DEBUG("is_exist_split_on_expressway_: %d \n", is_exist_split_on_expressway_);
  JSON_DEBUG_VALUE("is_exist_split_on_expressway", is_exist_split_on_expressway_);

  set_is_exist_intersection_split(
      ego_lane_track_manager_->is_exist_intersection_split());
  LOG_DEBUG("is_exist_intersection_split: %d \n", is_exist_intersection_split_);
  JSON_DEBUG_VALUE("is_exist_intersection_split", is_exist_intersection_split_);

  const bool is_in_ramp_select_split_situation =
      ego_lane_track_manager_->is_in_ramp_select_split_situation();
  LOG_DEBUG("is_in_ramp_select_split_situation: %d \n",
            is_in_ramp_select_split_situation);
  JSON_DEBUG_VALUE("is_in_ramp_select_split_situation",
                   is_in_ramp_select_split_situation);

  const bool is_on_road_select_ramp_situation =
      ego_lane_track_manager_->is_on_road_select_ramp_situation();
  LOG_DEBUG("is_on_road_select_ramp_situation: %d \n",
            is_on_road_select_ramp_situation);
  JSON_DEBUG_VALUE("is_on_road_select_ramp_situation",
                   is_on_road_select_ramp_situation);

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

void VirtualLaneManager::CalculateDistanceToRamp(
    planning::framework::Session* session) {
  const auto& local_view = session->environmental_model().get_local_view();
  const auto& hd_map = session->environmental_model().get_hd_map();
  const CurrentRouting& current_routing =
      local_view.static_map_info.current_routing();

  dis_to_ramp_ = NL_NMAX;
  double remaining_dis = 0;
  int current_index = -1;

  if (GetCurrentIndexAndDis(*session, &current_index, &remaining_dis)) {
    dis_to_ramp_ =
        remaining_dis + JudgeIfTheRamp(current_index, current_routing, hd_map);
  }
}

void VirtualLaneManager::CalculateDistanceToFirstRoadSplit(
    planning::framework::Session* session) {
  const auto& local_view = session->environmental_model().get_local_view();
  const auto& hd_map = session->environmental_model().get_hd_map();
  const CurrentRouting& current_routing =
      local_view.static_map_info.current_routing();

  distance_to_first_road_split_ = NL_NMAX;
  double remaining_dis = 0;
  int current_index = -1;

  if (GetCurrentIndexAndDis(*session, &current_index, &remaining_dis)) {
    distance_to_first_road_split_ =
        remaining_dis +
        JudgeIfTheFirstSplit(current_index, current_routing, hd_map);
  }
}

void VirtualLaneManager::CalculateDistanceToFirstRoadMerge(
    planning::framework::Session* session) {
  const auto& local_view = session->environmental_model().get_local_view();
  const auto& hd_map = session->environmental_model().get_hd_map();
  const CurrentRouting& current_routing =
      local_view.static_map_info.current_routing();

  distance_to_first_road_merge_ = NL_NMAX;
  double remaining_dis = 0;
  int current_index = -1;

  if (GetCurrentIndexAndDis(*session, &current_index, &remaining_dis)) {
    distance_to_first_road_merge_ =
        remaining_dis +
        JudgeIfTheFirstMerge(current_index, current_routing, hd_map);
  }
}

double VirtualLaneManager::JudgeIfTheRamp(
    const int current_index, const CurrentRouting& current_routing,
    const ad_common::hdmap::HDMap& hd_map) {
  const int sorted_lane_groups_num = sorted_lane_groups_in_route_.size();
  double accumulate_distance_for_lane_group = 0;

  for (int i = current_index; i < sorted_lane_groups_num; i++) {
    const uint64_t lane_group_id = sorted_lane_groups_in_route_[i];
    LaneGroupConstPtr lane_group_ptr = hd_map.GetLaneGroupById(lane_group_id);
    if (lane_group_ptr == nullptr) {
      LOG_DEBUG("fail get lane group by id for ramp!!!\n");
      return NL_NMAX;
    }
    if (i > current_index) {
      accumulate_distance_for_lane_group += lane_group_ptr->length();
    }
    // judge the lane group successor lane groups if more than 1
    const int successor_lane_group_size =
        lane_group_ptr->successor_lane_group_ids().size();
    // std::cout << "successor_lane_group_size:" << successor_lane_group_size
    //           << std::endl;
    if (successor_lane_group_size > 1 && (i + 1) < sorted_lane_groups_num) {
      uint64_t lane_group_id_next = sorted_lane_groups_in_route_[i + 1];
      LaneGroupConstPtr lane_group_ptr_next =
          hd_map.GetLaneGroupById(lane_group_id_next);
      if (lane_group_ptr_next == nullptr) {
        LOG_DEBUG("fail get lane group by id for ramp!!!\n");
        return NL_NMAX;
      }
      for (int j = 0; j < lane_group_ptr_next->way_forms().size(); j++) {
        if (lane_group_ptr_next->way_forms()[j] == RAMP) {
          LOG_DEBUG("accumulate_distance_for_lane_group for ramp :%f\n",
                    accumulate_distance_for_lane_group);
          // std::cout << "current judge ramp lane group id:" <<
          // lane_group_id_next
          //           << std::endl;
          CalculateRampDirection(hd_map, lane_group_ptr);
          return accumulate_distance_for_lane_group;
        }
      }
    }
  }
  LOG_DEBUG("no ramp in current routing\n");
  return NL_NMAX;
}

void VirtualLaneManager::CalculateRampDirection(
    const ad_common::hdmap::HDMap& hd_map, LaneGroupConstPtr lane_group_ptr) {
  // TODO(fengwang31): only consider groups with 2 successor
  if (lane_group_ptr == nullptr) {
    return;
  }
  if (lane_group_ptr->successor_lane_group_ids().size() < 2) {
    return;
  }
  ad_common::math::Vec2d group_in_route_dir_vec;
  ad_common::math::Vec2d group_not_in_route_dir_vec;
  for (int i = 0; i < 2; ++i) {
    const uint64_t group_id = lane_group_ptr->successor_lane_group_ids()[i];
    LaneGroupConstPtr lane_group_ptr = hd_map.GetLaneGroupById(group_id);
    LaneInfoConstPtr lane_info_ptr = nullptr;
    for (const auto& lane_id : lane_group_ptr->lane_ids()) {
      lane_info_ptr = hd_map.GetLaneById(lane_id);
      if (lane_info_ptr != nullptr) {
        break;
      }
    }
    if (lane_info_ptr == nullptr) {
      return;
    }
    if (lane_info_ptr->lane().points_on_central_line_size() < 2) {
      return;
    }
    const auto& points_on_central_line =
        lane_info_ptr->lane().points_on_central_line();
    ad_common::math::Vec2d group_dir_vec;
    group_dir_vec.set_x(points_on_central_line[1].x() -
                        points_on_central_line[0].x());
    group_dir_vec.set_y(points_on_central_line[1].y() -
                        points_on_central_line[0].y());
    if (lane_group_set_.count(group_id) != 0) {
      group_in_route_dir_vec = std::move(group_dir_vec);
    } else {
      group_not_in_route_dir_vec = std::move(group_dir_vec);
    }
  }
  if (group_in_route_dir_vec.CrossProd(group_not_in_route_dir_vec) > 0.0) {
    ramp_direction_ = RampDirection::RAMP_ON_RIGHT;
  } else {
    ramp_direction_ = RampDirection::RAMP_ON_LEFT;
  }
}

double VirtualLaneManager::JudgeIfTheFirstSplit(
    const int current_index, const CurrentRouting& current_routing,
    const ad_common::hdmap::HDMap& hd_map) const {
  const int sorted_lane_groups_num = sorted_lane_groups_in_route_.size();
  double accumulate_distance_for_lane_group = 0;
  for (int i = current_index; i < sorted_lane_groups_num; i++) {
    const uint64_t lane_group_id = sorted_lane_groups_in_route_[i];
    LaneGroupConstPtr lane_group_ptr = hd_map.GetLaneGroupById(lane_group_id);
    if (lane_group_ptr == nullptr) {
      LOG_DEBUG("fail get lane group by id for split!!!\n");
      return NL_NMAX;
    }
    if (i > current_index) {
      accumulate_distance_for_lane_group += lane_group_ptr->length();
    }

    // judge the lane group successor lane groups if more than 1
    const int successor_lane_group_size =
        lane_group_ptr->successor_lane_group_ids().size();
    if (successor_lane_group_size > 1) {
      LOG_DEBUG("accumulate_distance_for_lane_group for split :%f\n",
                accumulate_distance_for_lane_group);
      // std::cout << "current judge split lane group id:" << lane_group_id
      //           << std::endl;
      return accumulate_distance_for_lane_group;
    }
  }
  LOG_DEBUG("no split in current routing\n");
  return NL_NMAX;
}

double VirtualLaneManager::JudgeIfTheFirstMerge(
    const int current_index, const CurrentRouting& current_routing,
    const ad_common::hdmap::HDMap& hd_map) const {
  const int sorted_lane_groups_num = sorted_lane_groups_in_route_.size();
  double accumulate_distance_for_lane_group = 0;
  bool is_current_is_ramp = false;
  for (int i = current_index; i < sorted_lane_groups_num; i++) {
    const uint64_t lane_group_id = sorted_lane_groups_in_route_[i];
    LaneGroupConstPtr lane_group_ptr = hd_map.GetLaneGroupById(lane_group_id);
    if (lane_group_ptr == nullptr) {
      LOG_DEBUG("fail get lane group by id for merge!!!\n");
      return NL_NMAX;
    }
    if (i == current_index) {
      accumulate_distance_for_lane_group += 0;
      for (int index = 0; index < lane_group_ptr->way_forms_size(); index++) {
        if (lane_group_ptr->way_forms()[index] == RAMP) {
          // std::cout << "currrent in ramp, lane group id:" << lane_group_id
          //           << std::endl;
          is_current_is_ramp = true;
          break;
        }
      }
      if (!is_current_is_ramp) {
        // std::cout << "current lane group is not in ramp!!!" << std::endl;
        return NL_NMAX;
      }
    } else {
      accumulate_distance_for_lane_group += lane_group_ptr->length();
    }

    // judge if the road merge according to the predecessor_lane_group_ids_size
    if ((i + 1 < sorted_lane_groups_num) & is_current_is_ramp) {
      uint64_t lane_group_id_next = sorted_lane_groups_in_route_[i + 1];
      LaneGroupConstPtr lane_group_ptr_next =
          hd_map.GetLaneGroupById(lane_group_id_next);
      if (lane_group_ptr_next == nullptr) {
        LOG_DEBUG("fail get lane group by id for merge!!!\n");
        return NL_NMAX;
      }
      LOG_DEBUG("lane_group_ptr_next->way_forms size: %d\n",
                lane_group_ptr_next->way_forms().size());
      bool is_no_ramp_on_next_group = true;
      for (int j = 0; j < lane_group_ptr_next->way_forms().size(); j++) {
        // std::cout << "lane_group_ptr_next way_forms:"
        //           << lane_group_ptr_next->way_forms()[j] << ",No:" << j
        //           << std::endl;
        if (lane_group_ptr_next->way_forms()[j] == RAMP) {
          is_no_ramp_on_next_group = false;
          break;
        }
      }
      bool is_predecessor_more_than_one = false;

      // can comment out the code if want to cancel the first change lane
      // ************
      LOG_DEBUG("predecessor_lane_group_ids size: %d\n",
                lane_group_ptr_next->predecessor_lane_group_ids().size());
      if (lane_group_ptr_next->predecessor_lane_group_ids().size() > 1) {
        is_predecessor_more_than_one = true;
        LOG_DEBUG("is_predecessor_more_than_one: %d\n",
                  int(is_predecessor_more_than_one));
      }
      //******************************************************************************

      if (is_no_ramp_on_next_group || is_predecessor_more_than_one) {
        LOG_DEBUG("accumulate_distance_in_lane_group for merge :%f\n",
                  accumulate_distance_for_lane_group);
        LOG_DEBUG("judge merge lane group id: %lu\n", lane_group_id_next);
        return accumulate_distance_for_lane_group;
      }
    }
  }
  LOG_DEBUG("no road merge in current routing for merge\n");
  return NL_NMAX;
}

bool VirtualLaneManager::GetCurrentIndexAndDis(
    const planning::framework::Session& session, int* current_index,
    double* remaining_dis) {
  const auto& local_view = session.environmental_model().get_local_view();
  const CurrentRouting& current_routing =
      local_view.static_map_info.current_routing();

  const int lane_groups_num = current_routing.lane_groups_in_route_size();

  // get the remaining distance in current lane
  const double nearest_lane_total_length = nearest_lane_->total_length();
  *remaining_dis = nearest_lane_total_length - nearest_s_;
  LOG_DEBUG("nearest_lane_total_length: %f, nearest_s: %f, remaining_dis: %f\n",
            nearest_lane_total_length, nearest_s_, *remaining_dis);

  // judge the ramp lane group
  uint64_t nearest_lane_group_id = nearest_lane_->lane_group_id();
  LOG_DEBUG("nearest_lane_id::%lu\n", nearest_lane_->id());
  LOG_DEBUG("nearest_lane_group_id:%lu\n", nearest_lane_group_id);
  LOG_DEBUG("lane_groups_num nums:%d\n", lane_groups_num);

  // get the current lane group
  int current_lane_group_index = -1;
  for (int i = 0; i < sorted_lane_groups_in_route_.size(); i++) {
    if (nearest_lane_group_id == sorted_lane_groups_in_route_[i]) {
      current_lane_group_index = i;
    }
  }
  if (current_lane_group_index < 0) {
    LOG_DEBUG("fail find the current lane group in lane groups!!!\n");
    return false;
  }
  *current_index = current_lane_group_index;
  return true;
}

bool VirtualLaneManager::CalculateSortedLaneGroupIdsInRouting(
    const planning::framework::Session& session) {
  const auto& ego_state = session.environmental_model().get_ego_state_manager();
  const auto& hd_map = session.environmental_model().get_hd_map();

  const double ego_pose_x = ego_state->ego_pose_raw().x;
  const double ego_pose_y = ego_state->ego_pose_raw().y;
  ad_common::math::Vec2d point(ego_pose_x, ego_pose_y);

  // get nearest lane
  ad_common::hdmap::LaneInfoConstPtr nearest_lane;
  double nearest_s = 0.0;
  double nearest_l = 0.0;
  const int res =
      hd_map.GetNearestLane(point, &nearest_lane, &nearest_s, &nearest_l);
  if (res != 0) {
    LOG_DEBUG("no get nearest lane!!!\n");
    return false;
  }

  const uint64_t current_lane_group = nearest_lane->lane_group_id();
  sorted_lane_groups_in_route_.clear();
  // sorted_lane_groups_in_route_.reserve(lane_groups_num);
  sorted_lane_groups_in_route_.emplace_back(current_lane_group);
  LOG_DEBUG("id:%lu\n", current_lane_group);

  LaneGroupConstPtr lane_group_ptr =
      hd_map.GetLaneGroupById(current_lane_group);
  if (lane_group_ptr == nullptr) {
    LOG_DEBUG("lane_group_ptr is nullprt!!!\n");
  }
  while (lane_group_ptr != nullptr) {
    bool is_found = false;
    for (const auto& id : lane_group_ptr->successor_lane_group_ids()) {
      if (lane_group_set_.count(id) != 0) {
        sorted_lane_groups_in_route_.emplace_back(id);
        lane_group_ptr = hd_map.GetLaneGroupById(id);
        is_found = true;
        break;
      }
    }
    if (!is_found) {
      LOG_DEBUG("not found\n");
      break;
    }
  }
  LOG_DEBUG("sorted_lane_groups_in_route_ size: %zu\n",
            sorted_lane_groups_in_route_.size());
  return true;
}

bool VirtualLaneManager::JudgeEgoIfOnRamp(
    const planning::framework::Session& session) {
  const auto& hd_map = session.environmental_model().get_hd_map();
  // get the nearest lane group id
  uint64_t nearest_lane_group_id = nearest_lane_->lane_group_id();
  std::cout << "nearest_lane_group_id:" << nearest_lane_group_id << std::endl;
  // get the nearest lan group
  LaneGroupConstPtr nearest_lane_group_ptr =
      hd_map.GetLaneGroupById(nearest_lane_group_id);
  if (nearest_lane_group_ptr == nullptr) {
    LOG_DEBUG("fail get lane group by id for judge if on ramp!!!\n");
    return false;
  }
  for (int j = 0; j < nearest_lane_group_ptr->way_forms().size(); j++) {
    std::cout << "nearest_lane_group_ptr way_forms:"
              << nearest_lane_group_ptr->way_forms()[j] << ",No:" << j
              << std::endl;
    if (nearest_lane_group_ptr->way_forms()[j] == RAMP) {
      std::cout << "current ego on ramp!!!" << std::endl;
      return true;
    }
  }
  std::cout << "current ego do not on ramp!!!" << std::endl;
  return false;
}

bool VirtualLaneManager::GetCurrentNearestLane(
    const planning::framework::Session& session) {
  if (session_->environmental_model().get_hdmap_valid()) {
    const auto& local_view = session_->environmental_model().get_local_view();
    if (local_view.localization.status.status_info.mode !=
        iflyauto::IFLYStatusInfoMode::IFLY_STATUS_INFO_MODE_ERROR) {
      std::cout << "hdmap_valid is true,current timestamp:"
                << session_->environmental_model()
                       .get_local_view()
                       .static_map_info.header()
                       .timestamp()
                << std::endl;
      const auto& hd_map = session.environmental_model().get_hd_map();
      ad_common::math::Vec2d point;
      // TODO(fengwang31):把noa和hpp的定位需要合在一起
      const auto& ego_state =
          session.environmental_model().get_ego_state_manager();
      ego_pose_x_ = ego_state->ego_pose_raw().x;
      ego_pose_y_ = ego_state->ego_pose_raw().y;
      yaw_ = ego_state->ego_pose_raw().theta;
      point.set_x(ego_pose_x_);
      point.set_y(ego_pose_y_);
      // get nearest lane
      ad_common::hdmap::LaneInfoConstPtr nearest_lane;
      double nearest_s = 0.0;
      double nearest_l = 0.0;
      // const double distance = 10.0;
      // const double central_heading = pose.heading();
      // const double max_heading_difference = PI / 4;
      if (hd_map.GetNearestLane(point, &nearest_lane, &nearest_s, &nearest_l) !=
          0) {
        std::cout << "no get nearest lane!!!" << std::endl;
        return false;
      }
      // const int res = hd_map.GetNearestLaneWithHeading(
      //     point, distance, central_heading, max_heading_difference,
      //     &nearest_lane, &nearest_s, &nearest_l);
      // std::cout << "find current lane to current ego point dis:"
      //           << nearest_lane->DistanceTo(point) << std::endl;
      std::cout << "find the nearest lane!!!"
                << "nearest_s_:" << nearest_s_
                << ",nearest lane group id:" << nearest_lane->lane_group_id()
                << std::endl;
      nearest_lane_ = nearest_lane;
      nearest_s_ = nearest_s;
      return true;
    } else {
      std::cout << "localization invalid" << std::endl;
    }
  } else {
    std::cout << "hdmap  is invalid" << std::endl;
  }
  return false;
}

void VirtualLaneManager::CalculateDistanceToRampSplitMerge(
    planning::framework::Session* session) {
  if (session_->environmental_model().get_hdmap_valid()) {
    const auto& local_view = session_->environmental_model().get_local_view();
    if (local_view.localization.status.status_info.mode !=
        iflyauto::IFLYStatusInfoMode::IFLY_STATUS_INFO_MODE_ERROR) {
      std::cout << "hdmap_valid is true,current timestamp:"
                << session_->environmental_model()
                       .get_local_view()
                       .static_map_info.header()
                       .timestamp()
                << std::endl;
      if (GetCurrentNearestLane(*session_)) {
        lane_group_set_.clear();
        const CurrentRouting& current_routing =
            local_view.static_map_info.current_routing();
        const int lane_groups_num = current_routing.lane_groups_in_route_size();
        for (int i = 0; i < lane_groups_num; i++) {
          lane_group_set_.insert(
              current_routing.lane_groups_in_route()[i].lane_group_id());
        }
        ramp_direction_ = RampDirection::RAMP_NONE;
        CalculateSortedLaneGroupIdsInRouting(*session_);
        CalculateDistanceToRamp(session_);
        CalculateDistanceToFirstRoadSplit(session_);
        CalculateDistanceToFirstRoadMerge(session_);
        is_on_ramp_ = JudgeEgoIfOnRamp(*session_);
        std::cout << "ramp_direction:" << ramp_direction_ << std::endl;
        std::cout << "is_on_ramp_:" << is_on_ramp_ << std::endl;
      } else {
        std::cout << "current do not find nearest lane!!!" << std::endl;
      }
    } else {
      std::cout << "localization invalid" << std::endl;
    }
  } else {
    ResetForRampInfo();
  }
}

void VirtualLaneManager::CalculateDistanceToRampSplitMergeWithSdMap(
    planning::framework::Session* session) {
  ResetForRampInfo();
  const auto& local_view = session_->environmental_model().get_local_view();
  if (!session_->environmental_model().get_sdmap_valid()) {
    ResetForRampInfo();
    std::cout << "sd_map is invalid!!!" << std::endl;
    return;
  }
  std::cout << "sd_map valid!!!" << std::endl;

  if (local_view.localization.status.status_info.mode ==
      iflyauto::IFLYStatusInfoMode::IFLY_STATUS_INFO_MODE_ERROR) {
    std::cout << "localization invalid" << std::endl;
    ResetForRampInfo();
    return;
  }

  ad_common::math::Vec2d current_point;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& pose = ego_state->location_enu();
  ego_pose_x_ = pose.position.x;
  ego_pose_y_ = pose.position.y;
  std::cout << "ego_pose_x_:" << ego_pose_x_ << "ego_pose_y_:" << ego_pose_y_
            << std::endl;
  current_point.set_x(ego_pose_x_);
  current_point.set_y(ego_pose_y_);
  const auto& sd_map = session_->environmental_model().get_sd_map();
  double nearest_s = 0;
  double nearest_l = 0;
  const double max_search_length = 7000.0;  // 搜索7km范围内得地图信息
  const double search_distance = 50.0;
  const double max_heading_diff = PI / 4;
  const double ego_heading_angle = ego_state->heading_angle();
  const auto current_segment = sd_map.GetNearestRoadWithHeading(
      current_point, search_distance, ego_heading_angle, max_heading_diff,
      nearest_s, nearest_l);
  // const auto current_segment =
  // sd_map.GetNearestRoad(current_point,nearest_s,nearest_l);

  current_segment_passed_distance_ = nearest_s;
  LOG_DEBUG("current_segment_passed_distance:%f\n",
            current_segment_passed_distance_);
  JSON_DEBUG_VALUE("current_segment_passed_distance",
                   current_segment_passed_distance_);

  if (!current_segment) {
    ResetForRampInfo();
    return;
  } else {
    is_in_sdmaproad_ = true;
  }

  if (current_segment->shape_points_size() > 0) {
      const auto& temp_point_LLH = current_segment->shape_points(0);
      std::cout << "lat:" << temp_point_LLH.lat() << 
      ",lon:" << temp_point_LLH.lon() <<
      ",height:" << temp_point_LLH.height() << std::endl;
    JSON_DEBUG_VALUE("point_LLH_lat", temp_point_LLH.lat());
    JSON_DEBUG_VALUE("point_LLH_lat", temp_point_LLH.lon());
    JSON_DEBUG_VALUE("point_LLH_lat", temp_point_LLH.height());
  }
  JSON_DEBUG_VALUE("current_segment_id", current_segment->id());
  std::cout << "forward_lane_num:" << current_segment->forward_lane_num() << std::endl;
  JSON_DEBUG_VALUE("forward_lane_num", current_segment->forward_lane_num());
  if (current_segment->priority() == SdMapSwtx::RoadPriority::EXPRESSWAY ||
      current_segment->priority() == SdMapSwtx::RoadPriority::CITY_EXPRESSWAY) {
    is_ego_on_expressway_ = true;
    if (current_segment->priority() == SdMapSwtx::RoadPriority::EXPRESSWAY) {
      is_ego_on_expressway_hmi_ = true;
    } else {
      is_ego_on_city_expressway_hmi_ = true;
    }
  } else {
    std::cout << "current position not in EXPRESSWAY!!!" << std::endl;
    ResetForRampInfo();
    return;
  }
  is_on_highway_ =
      current_segment->priority() == SdMapSwtx::RoadPriority::EXPRESSWAY;
  is_on_ramp_ = current_segment->usage() == SdMapSwtx::RoadUsage::RAMP;
  // 计算ramp信息
  const auto& ramp_info =
      sd_map.GetRampInfo(current_segment->id(), nearest_s, max_search_length);
  if (ramp_info.second > 0) {
    dis_to_ramp_ = ramp_info.second;
    const auto previous_seg =
        sd_map.GetPreviousRoadSegment(ramp_info.first->id());
    if (previous_seg) {
      SplitSegInfo split_seg_info;
      split_seg_info = MakesureSplitDirection(*previous_seg, sd_map);
      ramp_direction_ = split_seg_info.split_direction;
    } else {
      std::cout << "previous_seg is nullprt!!!!!" << std::endl;
      ramp_direction_ = RAMP_NONE;
    }

  } else {
    dis_to_ramp_ = NL_NMAX;
    ramp_direction_ = RAMP_NONE;
  }
  // 计算merge信息
  //  TODO(fengwang31):是否需要考虑merge的方向
  const auto& merge_info = sd_map.GetMergeInfoList(
      current_segment->id(), nearest_s, max_search_length);
  if (!merge_info.empty()) {
    const auto seg_of_first_road_merge = merge_info.begin()->first;
    const auto next_seg_of_first_road_merge =
        sd_map.GetNextRoadSegment(merge_info.begin()->first->id());
    int traverse_num = 0;
    bool is_find_first_merge_onfo = false;
    for (int i = 0; i < merge_info.size(); i++) {
      const auto& merge_info_temp = merge_info[i];
      if (merge_info_temp.second > kEpsilon) {
        const auto& merge_seg = merge_info_temp.first;
        if (!merge_seg) {
          break;
        }
        const auto& merge_seg_last_seg =
            sd_map.GetPreviousRoadSegment(merge_seg->id());
        if (!merge_seg_last_seg) {
          break;
        }

        if (!is_find_first_merge_onfo) {
          if (merge_seg_last_seg->usage() == SdMapSwtx::RoadUsage::RAMP &&
              merge_seg->usage() != SdMapSwtx::RoadUsage::RAMP && is_on_ramp_) {
            is_ramp_merge_to_road_on_expressway_ = true;
          }
          if (merge_seg_last_seg->usage() != SdMapSwtx::RoadUsage::RAMP &&
              merge_seg->usage() != SdMapSwtx::RoadUsage::RAMP &&
              !is_on_ramp_ && is_ego_on_expressway_) {
            is_road_merged_by_other_lane_ = true;
          }
          if (merge_seg_last_seg->usage() == SdMapSwtx::RoadUsage::RAMP &&
              merge_seg->usage() == SdMapSwtx::RoadUsage::RAMP && is_on_ramp_) {
            is_ramp_merge_to_ramp_on_expressway_ = true;
          }
          first_merge_direction_ = MakesureMergeDirection(*merge_seg, sd_map);
          distance_to_first_road_merge_ = merge_info_temp.second;
          is_find_first_merge_onfo = true;
          traverse_num++;
        } else if (is_find_first_merge_onfo) {
          second_merge_direction_ = MakesureMergeDirection(*merge_seg, sd_map);
          distance_to_second_road_merge_ = merge_info_temp.second;
          traverse_num++;
        }

        if (traverse_num >= 2) {
          break;
        }
      } else {
        continue;
      }
    }

    if (next_seg_of_first_road_merge != nullptr) {
      if (seg_of_first_road_merge->usage() == SdMapSwtx::RoadUsage::RAMP &&
          next_seg_of_first_road_merge->usage() == SdMapSwtx::RoadUsage::RAMP) {
        is_continuous_ramp_ = true;
      }
    }
  } else {
    distance_to_first_road_merge_ = NL_NMAX;
    std::cout << "merge_info.empty()!!!!!!!" << std::endl;
  }

  // 计算split信息
  first_split_dir_dis_info_ = std::make_pair(None, NL_NMAX);
  const auto& split_info = sd_map.GetSplitInfoList(
      current_segment->id(), nearest_s, max_search_length);
  if (!split_info.empty()) {
    bool is_find_first_split_info = false;
    int traverse_num = 0;
    for (int i = 0; i < split_info.size(); i++) {
      const auto split_segment = split_info[i].first;
      if (split_segment && split_info[i].second > 0) {
        if (!is_find_first_split_info) {
          distance_to_first_road_split_ = split_info[i].second;
          SplitSegInfo split_seg_info;
          split_seg_info = MakesureSplitDirection(*split_segment, sd_map);
          first_split_direction_ = split_seg_info.split_direction;
          split_seg_forward_lane_nums_ = split_seg_info.split_seg_forward_lane_nums;
          split_next_seg_forward_lane_nums_ = split_seg_info.split_next_seg_forward_lane_nums;
          is_find_first_split_info = true;
          traverse_num++;
          first_split_dir_dis_info_ = std::make_pair(
              static_cast<SplitRelativeDirection>(first_split_direction_),
              distance_to_first_road_split_);
        } else if (is_find_first_split_info) {
          distance_to_second_road_split_ = split_info[i].second;
          SplitSegInfo split_seg_info;
          split_seg_info = MakesureSplitDirection(*split_segment, sd_map);
          second_split_direction_ = split_seg_info.split_direction;
          traverse_num++;
        }
        if (traverse_num >= 2) {
          break;
        }
      }
    }
  } else {
    distance_to_first_road_split_ = NL_NMAX;
    first_split_direction_ = RAMP_NONE;
    std::cout << "split_info.empty()!!!!!!!" << std::endl;
  }

  split_dir_dis_info_list_.clear();
  if (!split_info.empty()) {
    for (int i = 0; i < 2; i++) {
      if (i < split_info.size()) {
        const auto split_segment = split_info[i].first;
        double distance_to_road_split = NL_NMAX;
        RampDirection split_direction = RAMP_NONE;
        std::pair<SplitRelativeDirection, double> split_dir_dis_info;
        if (split_info[i].second > 0 && split_segment) {
          distance_to_road_split = split_info[i].second;
          SplitSegInfo split_seg_info;
          split_seg_info = MakesureSplitDirection(*split_segment, sd_map);
          split_direction = split_seg_info.split_direction;
        } else {
          distance_to_road_split = NL_NMAX;
          split_direction = RAMP_NONE;
        }
        split_dir_dis_info =
            std::make_pair(static_cast<SplitRelativeDirection>(split_direction),
                           distance_to_road_split);
        split_dir_dis_info_list_.emplace_back(split_dir_dis_info);
      }
    }
  } else {
    distance_to_first_road_split_ = NL_NMAX;
    first_split_direction_ = RAMP_NONE;
    std::cout << "split_info.empty()!!!!!!!" << std::endl;
  }
  // 计算距离上一个merge点的信息
  const SdMapSwtx::Segment* last_merge_seg = current_segment;
  is_accumulate_dis_to_last_merge_point_more_than_threshold_ = false;
  double sum_dis_to_last_merge_point = nearest_s;
  sum_dis_to_last_merge_point_ = NL_NMAX;
  if (!is_on_ramp_) {
    while (last_merge_seg->in_link().size() == 1) {
      last_merge_seg = sd_map.GetPreviousRoadSegment(last_merge_seg->id());
      // 判断是否为nullptr
      if (!last_merge_seg) {
        break;
      } else {
        sum_dis_to_last_merge_point =
            sum_dis_to_last_merge_point + last_merge_seg->dis();
      }
    }
    if (last_merge_seg && last_merge_seg->in_link().size() == 2) {
      // fengwang31:目前仅针对inlink是2的情况做处理
      const auto& merge_last_seg =
          sd_map.GetPreviousRoadSegment(last_merge_seg->id());
      if (merge_last_seg && merge_last_seg->usage() == SdMapSwtx::RAMP &&
          last_merge_seg->usage() != SdMapSwtx::RAMP) {
        sum_dis_to_last_merge_point_ = sum_dis_to_last_merge_point;
      }
    }
    if (sum_dis_to_last_merge_point_ > dis_threshold_to_last_merge_point_) {
      is_accumulate_dis_to_last_merge_point_more_than_threshold_ = true;
    }
  }
  JSON_DEBUG_VALUE("sum_dis_to_last_merge_point", sum_dis_to_last_merge_point_);

  // 计算在高速主路上距离上一个split点的信息
  const SdMapSwtx::Segment* last_split_seg =
      sd_map.GetPreviousRoadSegment(current_segment->id());
  double sum_dis_to_last_split_point = nearest_s;
  sum_dis_to_last_split_point_ = NL_NMAX;
  if (current_segment->usage() != SdMapSwtx::RAMP) {
    if (last_split_seg != nullptr) {
      while (last_split_seg->out_link().size() == 1) {
        sum_dis_to_last_split_point =
            sum_dis_to_last_split_point + last_split_seg->dis();
        last_split_seg = sd_map.GetPreviousRoadSegment(last_split_seg->id());
        if (!last_split_seg) {
          break;
        }
      }
      if (last_split_seg && last_split_seg->out_link().size() == 2) {
        if (last_split_seg->usage() != SdMapSwtx::RAMP) {
          sum_dis_to_last_split_point_ = sum_dis_to_last_split_point;
        }
      }
    }
  }
  JSON_DEBUG_VALUE("sum_dis_to_last_split_point", sum_dis_to_last_split_point_);

  // 计算在匝道上距离上一个split点的信息
  const SdMapSwtx::Segment* temp_last_split_seg =
      sd_map.GetPreviousRoadSegment(current_segment->id());
  double accumulate_dis_ego_to_last_split_point = nearest_s;
  accumulate_dis_ego_to_last_split_point_ = NL_NMAX;
  if (temp_last_split_seg) {
    while (temp_last_split_seg->out_link().size() == 1) {
      accumulate_dis_ego_to_last_split_point =
          accumulate_dis_ego_to_last_split_point + temp_last_split_seg->dis();
      temp_last_split_seg =
          sd_map.GetPreviousRoadSegment(temp_last_split_seg->id());
      if (!temp_last_split_seg) {
        break;
      }
    }
    if (temp_last_split_seg &&
        temp_last_split_seg->out_link().size() == 2) {
      accumulate_dis_ego_to_last_split_point_ =
          accumulate_dis_ego_to_last_split_point;
      SplitSegInfo split_seg_info;
      split_seg_info = MakesureSplitDirection(*temp_last_split_seg, sd_map);
      last_split_seg_dir_ = split_seg_info.split_direction;
    }
  }

  // 计算到路线终点的距离
  double dis_to_end = NL_NMAX;
  int result = sd_map.GetDistanceToRouteEnd(current_segment->id(), nearest_s,
                                            dis_to_end);
  if (result == 0) {
    distance_to_route_end_ = dis_to_end;
  } else {
    distance_to_route_end_ = NL_NMAX;
  }
  JSON_DEBUG_VALUE("distance_to_route_end", distance_to_route_end_);

  // 计算到最近收费站的距离
  const auto& toll_station_info = sd_map.GetTollStationInfo(
      current_segment->id(), nearest_s, max_search_length);
  if (toll_station_info.first != nullptr) {
    distance_to_toll_station_ = toll_station_info.second;
    is_exist_toll_station_ = true;
  } else {
    std::cout << "not find toll station" << std::endl;
    distance_to_toll_station_ = NL_NMAX;
    is_exist_toll_station_ = false;
  }

  //判断是否在匝道汇入主路场景，2个条件满足一个即可：
  // 1、在匝道上接近汇入点100米以内；
  // 2、在离开汇入点后超过500米，则视为已经完成匝道汇入主路；
  lane_num_except_emergency_ = lane_num_;
  if (lane_num_ > 0) {
    if (relative_id_lanes_[lane_num_ - 1]->get_lane_type() ==
        iflyauto::LANETYPE_EMERGENCY) {
      lane_num_except_emergency_ -= 1;
    }
  }
  is_leaving_ramp_ = false;
  if (lane_num_except_emergency_ > 0) {
    if (is_ramp_merge_to_road_on_expressway_ &&
        distance_to_first_road_merge_ < 100) {
      is_leaving_ramp_ = true;
    }
  }

  //判断是否是正在接近匝道
  const double dis_between_first_road_split_and_ramp =
      distance_to_first_road_split_ - dis_to_ramp_;
  const double allow_error = 5.0;
  is_nearing_ramp_ =
      fabs(dis_between_first_road_split_and_ramp) < allow_error &&
      dis_to_ramp_ < 3000.;

  //判断哪个场景在前
  if (is_leaving_ramp_ && is_nearing_ramp_) {
    if (distance_to_first_road_merge_ < dis_to_ramp_) {
      // merge在ramp的前面
      is_nearing_ramp_ = false;
    } else {
      // ramp在merge的前面
      is_leaving_ramp_ = false;
    }
  }
}

bool VirtualLaneManager::UpdateEgoDistanceToStopline() {
  const auto& stop_line = current_lane_->get_stop_line();
  double ego_to_stopline_dis = NL_NMAX;
  if (stop_line.type == iflyauto::LANE_LINE_TYPE_STOPLINE) {
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
  std::vector<std::vector<iflyauto::Point2f>> cross_walk_pts_vec;
  for (int i = 0; i < roads_ptr->lane_ground_markings_size; i++) {
    const auto& cross_walk = roads_ptr->lane_ground_markings[i];
    if (cross_walk.turn_type ==
            iflyauto::LaneDrivableDirection_DIRECTION_CROSS_LINE &&
        std::abs(cross_walk.orientation_angle) < 0.5) {
      std::vector<iflyauto::Point2f> cw_pts;
      cw_pts.resize(cross_walk.ground_marking_points_set_size);
      for (int idx = 0; idx < cross_walk.ground_marking_points_set_size;
           idx++) {
        cw_pts[idx] = cross_walk.ground_marking_points_set[idx];
      }
      cross_walk_pts_vec.push_back(cw_pts);
    }
  }
  auto compare_y = [&](iflyauto::Point2f p1, iflyauto::Point2f p2) {
    return p1.y < p2.y;
  };
  std::vector<std::pair<int, double>> cw_idx_dis_vec;
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
      // double pt_x = std::min(cw_pt_vec[idx].x, cw_pt_vec[idx + 1].x);
      double pt_x = (cw_pt_vec[idx].x + cw_pt_vec[idx + 1].x) / 2.0;
      planning::planning_math::Vec2d p_left(pt_x, cw_pt_vec[idx].y);
      planning::planning_math::Vec2d p_right(pt_x, cw_pt_vec[idx + 1].y);
      planning::planning_math::LineSegment2d line_seg(p_left, p_right);
      double raw_dis =
          line_seg.RawDistanceTo(planning::planning_math::Vec2d(0, 0));
      cw_idx_dis_vec.push_back(std::make_pair(j, -1.0 * raw_dis));
    } else {
      continue;
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
  if ((distance_to_stopline_ < 25.0 && distance_to_stopline_ > 3.0) ||
      ((distance_to_crosswalk_ < 28.0 && distance_to_crosswalk_ > 5.0) &&
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
// void VirtualLaneManager::CalculateHPPInfo(
//     planning::framework::Session* session) {
//   const auto& local_view = session_->environmental_model().get_local_view();
//   // ego box
//   const auto& vehicle_param =
//       VehicleConfigurationContext::Instance()->get_vehicle_param();
//   const auto center_x =
//       ego_pose_x_ + std::cos(yaw_) * vehicle_param.rear_axis_to_center;
//   const auto center_y =
//       ego_pose_y_ + std::sin(yaw_) * vehicle_param.rear_axis_to_center;
//   const ad_common::math::Box2d ego_box(
//       {center_x, center_y}, yaw_, vehicle_param.length, vehicle_param.width);
//   // if on hpp lane
//   if (nearest_lane_->IsOnLane(ego_box)) {
//     std::cout << "is on hpp lane!" << std::endl;
//     is_on_hpp_lane_ = true;
//     const auto trace_start = local_view.parking_map_info.trace_start;
//     const ad_common::math::Vec2d trace_start_point_2d = {trace_start.enu.x,
//                                                          trace_start.enu.x};
//     // get trace_start point projection s
//     double trace_start_point_accumulate_s;
//     double trace_start_point_lateral;
//     if (nearest_lane_->GetProjection(trace_start_point_2d,
//                                      &trace_start_point_accumulate_s,
//                                      &trace_start_point_lateral)) {
//       // std::cout << "trace_start point s:" <<
//       trace_start_point_accumulate_s
//       //           << ",lateral:" << trace_start_point_lateral << std::endl;
//     } else {
//       std::cout << " trace_start point get projection fail!! " << std::endl;
//       return;
//     }
//     // calculate sum distance
//     bool is_reached_trace_start_point =
//         nearest_s_ >= trace_start_point_accumulate_s;
//     const ad_common::math::Vec2d point(ego_pose_x_, ego_pose_y_);
//     if (is_reached_trace_start_point) {
//       std::cout << "reached trace start point!!" << std::endl;
//       is_reached_hpp_start_point_ = true;
//       if (last_point_hpp_.x() != NL_NMAX && last_point_hpp_.y() != NL_NMAX) {
//         sum_distance_driving_ += point.DistanceTo(last_point_hpp_);
//       } else {
//         sum_distance_driving_ = 0;
//       }
//       last_point_hpp_ = point;
//       // std::cout << "sum_distance_driving_:" << sum_distance_driving_
//       //           << std::endl;
//     } else {
//       std::cout << "cur point s less than trace start s" << std::endl;
//     }
//   } else {
//     std::cout << "not in hpp lane!!!" << std::endl;
//     ResetHpp();
//   }
// }

void VirtualLaneManager::ResetHpp() {
  is_on_hpp_lane_ = false;
  is_reached_hpp_start_point_ = false;
  sum_distance_driving_ = -1;
  last_point_hpp_.set_x(NL_NMAX);
  last_point_hpp_.set_y(NL_NMAX);
}

// void VirtualLaneManager::CalculateDistanceToTargetSlot(
//     planning::framework::Session* session) {
//   distance_to_target_slot_ = NL_NMAX;
//   const auto& local_view = session->environmental_model().get_local_view();
//   const auto& hd_map = session->environmental_model().get_hd_map();

//   // get target slot projection point on line
//   ad_common::hdmap::LaneInfoConstPtr tar_slot_nearest_lane;
//   double tar_slot_nearest_s = 0.0;
//   double tar_slot_nearest_l = 0.0;

//   const auto& lines = local_view.static_map_info.road_map().lanes();
//   if (!lines.empty()) {
//     const auto& last_point = lines[0].points_on_central_line().rbegin();
//     const double tar_slot_pose_x = last_point->x();
//     const double tar_slot_pose_y = last_point->y();
//     const int tar_slot_res = hd_map.GetNearestLane(
//         {tar_slot_pose_x, tar_slot_pose_y}, &tar_slot_nearest_lane,
//         &tar_slot_nearest_s, &tar_slot_nearest_l);
//     if (tar_slot_res != 0) {
//       std::cout << "not get target slot projection point on line!!!"
//                 << std::endl;
//       return;
//     }
//     distance_to_target_slot_ = tar_slot_nearest_s - nearest_s_;
//   } else {
//     std::cout << "lines is empty from road_map!!!" << std::endl;
//   }
// }

// void VirtualLaneManager::CalculateDistanceToNextSpeedBump(
//     planning::framework::Session* session) {
//   distance_to_next_speed_bump_ = NL_NMAX;
//   auto& local_view = session_->environmental_model().get_local_view();
//   const auto& road_marks =
//   local_view.parking_map_info.road_tile_info.road_mark; const auto& hd_map =
//   session->environmental_model().get_hd_map();

//   double distance_to_speed_bump_tmp = 0;
//   for (auto& road_mark : road_marks) {
//     if (road_mark.type == iflyauto::ROAD_MARK_TYPE_SPEED_BUMP &&
//         road_mark.shape_size == 4) {
//       ad_common::hdmap::LaneInfoConstPtr speed_bump_nearest_lane;
//       double speed_bump_nearest_s = 0.0;
//       double speed_bump_nearest_l = 0.0;

//       ad_common::math::Vec2d speed_bump_center_point(
//           (road_mark.shape[0].boot.x + road_mark.shape[3].boot.x) * 0.5,
//           (road_mark.shape[0].boot.y + road_mark.shape[3].boot.y) * 0.5);
//       const int speed_bump_res = hd_map.GetNearestLane(
//           speed_bump_center_point, &speed_bump_nearest_lane,
//           &speed_bump_nearest_s, &speed_bump_nearest_l);
//       if (speed_bump_res != 0) {
//         std::cout << "not get speed_bump projection point on line!!!"
//                   << std::endl;
//         continue;
//       } else {
//         std::cout << "get s for speed_bump projection point on line:"
//                   << speed_bump_nearest_s << std::endl;
//       }
//       distance_to_speed_bump_tmp = speed_bump_nearest_s - nearest_s_;
//       if (distance_to_speed_bump_tmp > 0) {  // TODO: 假设挡位为前进档
//         distance_to_next_speed_bump_ = distance_to_speed_bump_tmp;
//         break;
//       }
//     }
//   }
// }

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

void VirtualLaneManager::ResetForRampInfo() {
  is_on_ramp_ = false;
  dis_to_ramp_ = NL_NMAX;
  ramp_direction_ = RampDirection::RAMP_NONE;
  distance_to_first_road_merge_ = NL_NMAX;
  distance_to_first_road_split_ = NL_NMAX;
  distance_to_second_road_merge_ = NL_NMAX;
  distance_to_second_road_split_ = NL_NMAX;
  distance_to_route_end_ = NL_NMAX;
  is_in_sdmaproad_ = false;
  is_ego_on_expressway_ = false;
  first_split_direction_ = RampDirection::RAMP_NONE;
  first_merge_direction_ = RampDirection::RAMP_NONE;
  second_split_direction_ = RampDirection::RAMP_NONE;
  second_merge_direction_ = RampDirection::RAMP_NONE;
  is_leaving_ramp_ = false;
  sum_dis_to_last_merge_point_ = NL_NMAX;
  sum_dis_to_last_split_point_ = NL_NMAX;
  accumulate_dis_ego_to_last_split_point_ = NL_NMAX;
  distance_to_toll_station_ = NL_NMAX;
  is_ego_on_city_expressway_hmi_ = false;
  is_ego_on_expressway_hmi_ = false;
  split_dir_dis_info_list_.clear();
  is_exist_toll_station_ = false;
  is_ramp_merge_to_road_on_expressway_ = false;
  is_road_merged_by_other_lane_ = false;
  is_ramp_merge_to_ramp_on_expressway_ = false;
  other_lane_merge_dir = RampDirection::RAMP_NONE;
  is_nearing_other_lane_merge_to_road_point_ = false;
  is_on_highway_ = false;
  split_seg_forward_lane_nums_ = 0;
  split_next_seg_forward_lane_nums_ = 0;
  lc_nums_for_split_ = 0;
  last_split_seg_dir_ = RAMP_NONE;
}

SplitSegInfo VirtualLaneManager::MakesureSplitDirection(
    const ::SdMapSwtx::Segment& split_segment,
    const ad_common::sdmap::SDMap& sd_map) {
  const auto out_link_size = split_segment.out_link_size();
  SplitSegInfo split_seg_info;
  split_seg_info.split_direction = RAMP_NONE;
  split_seg_info.split_next_seg_forward_lane_nums = 0;
  split_seg_info.split_seg_forward_lane_nums = 0;
  const auto& out_link = split_segment.out_link();
  // fengwang31(TODO):暂时假设在匝道上的分叉口只有两个方向
  if (out_link_size == 2) {
    const auto split_next_segment =
        sd_map.GetNextRoadSegment(split_segment.id());
    if (!split_next_segment) {
      std::cout << "out segment is nullptr!!!!!!!!" << std::endl;
      return split_seg_info;
    }
    ad_common::math::Vec2d segment_in_route_dir_vec;
    ad_common::math::Vec2d segment_not_in_route_dir_vec;
    Point2D anchor_point_of_cur_seg_to_next_seg = {
        split_segment.enu_points().rbegin()->x(),
        split_segment.enu_points().rbegin()->y()};

    auto other_segment = out_link[0].id() == split_next_segment->id()
                             ? out_link[1]
                             : out_link[0];
    split_seg_info.split_seg_forward_lane_nums = split_segment.forward_lane_num();
    split_seg_info.split_next_seg_forward_lane_nums = other_segment.forward_lane_num();
    // const auto other_segment = sd_map.GetRoadSegmentById(other_segment_id);
    const auto& split_next_segment_enu_point = split_next_segment->enu_points();
    const auto& other_segment_enu_point = other_segment.enu_points();
    if (split_next_segment_enu_point.size() > 1 &&
        other_segment_enu_point.size() > 1) {
      segment_in_route_dir_vec.set_x(split_next_segment_enu_point[1].x() -
                                     anchor_point_of_cur_seg_to_next_seg.x);
      segment_in_route_dir_vec.set_y(split_next_segment_enu_point[1].y() -
                                     anchor_point_of_cur_seg_to_next_seg.y);
      segment_not_in_route_dir_vec.set_x(other_segment_enu_point[1].x() -
                                         anchor_point_of_cur_seg_to_next_seg.x);
      segment_not_in_route_dir_vec.set_y(other_segment_enu_point[1].y() -
                                         anchor_point_of_cur_seg_to_next_seg.y);
      if (segment_in_route_dir_vec.CrossProd(segment_not_in_route_dir_vec) >
          0.0) {
        split_seg_info.split_direction = RampDirection::RAMP_ON_RIGHT;
      } else {
        split_seg_info.split_direction = RampDirection::RAMP_ON_LEFT;
      }
    } else {
      std::cout << "enu points error!!!!!!!!!!" << std::endl;
    }
  } else {
    std::cout << "out_link_size != 2!!!!!!!1" << std::endl;
  }
  return split_seg_info;
}

RampDirection VirtualLaneManager::MakesureMergeDirection(
    const ::SdMapSwtx::Segment& merge_segment,
    const ad_common::sdmap::SDMap& sd_map) {
  const auto in_link_size = merge_segment.in_link_size();
  RampDirection merge_direction = RAMP_NONE;
  const auto& in_link = merge_segment.in_link();
  // fengwang31(TODO):暂时假设在merge处只有两个方向
  if (in_link_size == 2) {
    const auto merge_last_segment =
        sd_map.GetPreviousRoadSegment(merge_segment.id());
    if (!merge_last_segment) {
      std::cout << "in segment is nullptr!!!!!!!!" << std::endl;
      return merge_direction;
    }
    ad_common::math::Vec2d segment_in_route_dir_vec;
    ad_common::math::Vec2d segment_not_in_route_dir_vec;
    Point2D anchor_point_of_cur_seg_to_last_seg = {
        merge_segment.enu_points().begin()->x(),
        merge_segment.enu_points().begin()->y()};

    auto other_segment =
        in_link[0].id() == merge_last_segment->id() ? in_link[1] : in_link[0];
    const auto& merge_last_segment_enu_point = merge_last_segment->enu_points();
    const auto& other_segment_enu_point = other_segment.enu_points();
    const int point_num = merge_last_segment_enu_point.size();
    const int other_point_num = other_segment_enu_point.size();
    if (point_num > 1 && other_point_num > 1) {
      segment_in_route_dir_vec.set_x(
          merge_last_segment_enu_point[point_num - 2].x() -
          anchor_point_of_cur_seg_to_last_seg.x);
      segment_in_route_dir_vec.set_y(
          merge_last_segment_enu_point[point_num - 2].y() -
          anchor_point_of_cur_seg_to_last_seg.y);
      segment_not_in_route_dir_vec.set_x(
          other_segment_enu_point[other_point_num - 2].x() -
          anchor_point_of_cur_seg_to_last_seg.x);
      segment_not_in_route_dir_vec.set_y(
          other_segment_enu_point[other_point_num - 2].y() -
          anchor_point_of_cur_seg_to_last_seg.y);
      if (segment_in_route_dir_vec.CrossProd(segment_not_in_route_dir_vec) >
          0.0) {
        merge_direction = RampDirection::RAMP_ON_LEFT;
      } else {
        merge_direction = RampDirection::RAMP_ON_RIGHT;
      }
    } else {
      std::cout << "enu points error!!!!!!!!!!" << std::endl;
    }
  } else {
    std::cout << "out_link_size != 2!!!!!!!1" << std::endl;
  }
  return merge_direction;
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

void VirtualLaneManager::GenerateLaneChangeTasksForNOA() {
  // 判断ego是否在最右边车道上
  bool is_ego_on_rightest_lane = true;
  bool is_ego_on_leftest_lane = true;
  for (const auto& relative_id_lane : relative_id_lanes_) {
    if (relative_id_lane->get_relative_id() > 0) {
      is_ego_on_rightest_lane = false;
    }
    if (relative_id_lane->get_relative_id() < 0) {
      is_ego_on_leftest_lane = false;
    }
  }
  // 为了临时hack处理在匝道延长车道上的case，使得自车能在匝道延长车道上也能变道至lane上。
  // 同时满足以下4个条件则认为，还在匝道延长线上：
  //  1、自车当前不在匝道上；
  //  2、且距离下一个匝道距离在1300m以上,距离上一个merge点在700m以内；
  //  3、当前在最右边车道上；
  //  4、当前是在expressway上。
  if (!is_on_ramp_ && dis_to_ramp_ > dis_threshold_to_last_merge_point_ &&
      !is_accumulate_dis_to_last_merge_point_more_than_threshold_ &&
      is_ego_on_rightest_lane && is_on_highway_) {
    is_leaving_ramp_ = true;
  }
  //这里是hack匝道延长线800m范围内不在最右侧车道，如果也接近匝道了
  //根据到匝道的距离判断是匝道延长线汇入在前还是匝道在前
  if (is_nearing_ramp_ &&
      !is_accumulate_dis_to_last_merge_point_more_than_threshold_ &&
      !is_on_ramp_ && dis_to_ramp_ > dis_threshold_to_last_merge_point_ &&
      is_on_highway_) {
    is_nearing_ramp_ = false;
  }
  // fengwang31:临时处理合肥测试路线上右边长匝道反复变道问题，后续需要删除
  if (is_nearing_ramp_ && dis_to_ramp_ > 1180 &&
      sum_dis_to_last_merge_point_ > dis_threshold_to_last_merge_point_ &&
      sum_dis_to_last_merge_point_ < 810 && is_on_highway_ && !is_on_ramp_) {
    is_nearing_ramp_ = false;
  }

  //(2)、判断当前在高速主路上是否在接近汇入点，如果接近汇入点800米以内，不让自车呆最右侧车道上，以避开汇流区域
  // fengwang31:增加优先级判断，如果前方汇入距离前方匝道的距离较近，那么避开汇入区域的逻辑被抑制
  bool is_ramp_and_merge_dis_error_more_than_threshold = 
      dis_to_ramp_ - distance_to_first_road_merge_ > 500;
  bool is_driving_dir_conflict = 
      (first_merge_direction_ == RAMP_ON_LEFT && ramp_direction_ == RAMP_ON_RIGHT) ||
      (first_merge_direction_ == RAMP_ON_RIGHT && ramp_direction_ == RAMP_ON_LEFT);
  bool is_car_flow_conflict = 
      (is_ego_on_rightest_lane && first_merge_direction_ == RAMP_ON_LEFT) ||
      (is_ego_on_leftest_lane && first_merge_direction_ == RAMP_ON_RIGHT);
  if (is_road_merged_by_other_lane_ &&
      distance_to_first_road_merge_ < dis_threshold_to_is_merged_point_ &&
      is_car_flow_conflict &&
      is_ramp_and_merge_dis_error_more_than_threshold &&
      is_on_highway_) {
    is_nearing_other_lane_merge_to_road_point_ = true;
  }
  JSON_DEBUG_VALUE("is_nearing_other_lane_merge_to_road_point",
                   is_nearing_other_lane_merge_to_road_point_);

  //(3)、判断高速前方汇入点在前，还是匝道在前
  if (is_nearing_ramp_ && is_road_merged_by_other_lane_ &&
      is_ramp_and_merge_dis_error_more_than_threshold && 
      !is_on_ramp_ &&
      is_driving_dir_conflict &&
      is_ego_on_expressway_) {
    is_nearing_ramp_ = false;
  }

  //(4)、判断是否接近split区域
  bool is_nearing_split = false;
  double err_buffer = 10;
  double nearing_split_dis_threshold = 2000;
  if (is_nearing_ramp_ &&
      distance_to_first_road_split_ < dis_to_ramp_ - err_buffer) {
    is_nearing_split = true;
  } else if (is_nearing_other_lane_merge_to_road_point_ &&
       distance_to_first_road_split_ < distance_to_first_road_merge_) {
     is_nearing_split = true;
  } else if (distance_to_first_road_split_ < nearing_split_dis_threshold) {
    is_nearing_split = true;
  }
  //(5)、判断是否需要生成split的变道任务
  if (is_nearing_split) {
    //后面的车道数比当前车道数少一条的sceneray，意味着可能有一条车道从当前分流出去
    if ((split_seg_forward_lane_nums_ - split_next_seg_forward_lane_nums_) >= 0) {
      if (first_split_direction_ == RAMP_ON_LEFT &&
          is_ego_on_rightest_lane) {
        lc_nums_for_split_ = -1;
      } else if (first_split_direction_ == RAMP_ON_RIGHT &&
          is_ego_on_leftest_lane) {
        lc_nums_for_split_ = 1;
      }
    }
  }
  //(6)、判断当前是否在split的起点后100m范围内；
  bool is_ego_on_split_region = false;
  double length_for_split_region = std::numeric_limits<double>::max();
  const double dis_to_next_split_point = 
      std::min(distance_to_first_road_split_, dis_to_ramp_);
  std::array<double, 2> xp{50, 150};
  std::array<double, 2> fp{0, 100};
  const double split_region_dis_threshold = interp(dis_to_next_split_point, xp, fp);
  if (accumulate_dis_ego_to_last_split_point_ < split_region_dis_threshold) {
    is_ego_on_split_region = true;
  }
  //(7)、判断当前是否需要在下匝道的分流区域继续生成下匝道的变道请求；
  bool is_exit_lane_on_last_ramp_dir = 
      last_split_seg_dir_ == RAMP_NONE ? false : 
      (last_split_seg_dir_ == RAMP_ON_LEFT ? !is_ego_on_leftest_lane : !is_ego_on_rightest_lane);
  bool is_need_continue_lc_on_off_ramp_region = 
      is_exit_lane_on_last_ramp_dir &&
      is_ego_on_split_region;
  int need_continue_lc_num_on_off_ramp_region = 0;
  if (is_need_continue_lc_on_off_ramp_region) {
    need_continue_lc_num_on_off_ramp_region = 
    last_split_seg_dir_ == RAMP_ON_LEFT ? -1 : 1;
  }
   
  JSON_DEBUG_VALUE("is_leaving_ramp", is_leaving_ramp_);
  JSON_DEBUG_VALUE("is_nearing_ramp", is_nearing_ramp_);
  JSON_DEBUG_VALUE("distance_to_ramp", dis_to_ramp_);
  JSON_DEBUG_VALUE("distance_to_first_road_merge",
                   distance_to_first_road_merge_);
  JSON_DEBUG_VALUE("distance_to_first_road_split",
                   distance_to_first_road_split_);
  GeneralTaskMapInfo general_task_map_info;
  general_task_map_info.distance_to_ramp = dis_to_ramp_;
  general_task_map_info.distance_to_first_road_merge =
      distance_to_first_road_merge_;
  general_task_map_info.distance_to_first_road_split =
      distance_to_first_road_split_;
  general_task_map_info.lane_num_except_emergency = lane_num_except_emergency_;
  general_task_map_info.is_nearing_ramp = is_nearing_ramp_;
  general_task_map_info.is_leaving_ramp = is_leaving_ramp_;
  general_task_map_info.is_on_ramp = is_on_ramp_;
  general_task_map_info.is_nearing_other_lane_merge_to_road_point =
      is_nearing_other_lane_merge_to_road_point_;
  general_task_map_info.is_ramp_merge_to_road_on_expressway =
      is_ramp_merge_to_road_on_expressway_;
  general_task_map_info.is_ramp_merge_to_ramp_on_expressway =
      is_ramp_merge_to_ramp_on_expressway_;
  general_task_map_info.ramp_direction = ramp_direction_;
  general_task_map_info.first_split_direction = first_split_direction_;
  general_task_map_info.first_merge_direction = first_merge_direction_;
  general_task_map_info.second_merge_direction = second_merge_direction_;
  general_task_map_info.second_split_direction = second_split_direction_;
  general_task_map_info.distance_to_second_road_merge =
      distance_to_second_road_merge_;
  general_task_map_info.distance_to_second_road_split =
      distance_to_second_road_split_;
  general_task_map_info.sum_dis_to_last_split_point_on_ramp =
      accumulate_dis_ego_to_last_split_point_;
  general_task_map_info.split_seg_forward_lane_nums = split_seg_forward_lane_nums_;
  general_task_map_info.split_next_seg_forward_lane_nums = split_next_seg_forward_lane_nums_;
  general_task_map_info.lc_nums_for_split = lc_nums_for_split_;
  general_task_map_info.is_ego_on_split_region = is_ego_on_split_region;
  general_task_map_info.need_continue_lc_num_on_off_ramp_region = need_continue_lc_num_on_off_ramp_region;

  //(3)、对每一条lane，根据超视距信息，更新每一条lane的变道次数。
  for (const auto& relative_id_lane : relative_id_lanes_) {
    relative_id_lane->update_lane_tasks(general_task_map_info);
  }
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
  std::vector<Vec2d> center_line_points;
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
    std::shared_ptr<KDPath> frenet_coord;
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

  for(const auto& relative_id_lane : relative_id_lanes_) {
    if (relative_id_lane->get_order_id() == current_order_id) {
      return relative_id_lane;
    } else {
      continue;
    }
  }
  return nullptr;
}
}  // namespace planning
