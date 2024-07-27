#include "virtual_lane_manager.h"

// #include <cyber/common/log.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "ad_common/hdmap/hdmap.h"
#include "common_c.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "define/geometry.h"
#include "ehr.pb.h"
#include "ehr_sdmap.pb.h"
#include "environmental_model.h"
#include "fusion_road_c.h"
#include "ifly_localization_c.h"
#include "ifly_parking_map_c.h"
#include "ifly_time.h"
#include "local_view.h"
#include "localization_c.h"
#include "log.h"
// #include "log_glog.h"
#include "math/box2d.h"
#include "math/vec2d.h"
#include "planning_context.h"
#include "reference_path_manager.h"
#include "task_basic_types.h"
#include "utils/kd_path.h"
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
constexpr double kLatDistanceMaxStandardThr = 3.6;
constexpr double kLastPlanLengthThr = 2.0;
constexpr double kNeighborLaneCenterThr = 3.0;
constexpr double kFarLaneCenterThr = 5.0;
constexpr double kBoundaryCrossEgoBehindThr = 5.0;
constexpr double kBoundaryCrossEgoFrontThr = 10.0;
constexpr double kEnableFarLaneCenterThr = 1.2;
constexpr double kCrossLaneCostDefault = 0.2;
constexpr double kInitPosCostStandardThr = 3.6;
constexpr double kInitPosCostWeight = 1.0;
constexpr double kCumuLateralDistanceCostWeight = 1.5;
constexpr double kCrossLaneCostWeight = 1.0;
constexpr double kLaneChangeExecutionWeightRatio = 4.0;
constexpr int32_t kLaneCenterMinPointsThr = 3;
constexpr double kMinCostLength = 30.0;
}  // namespace

VirtualLaneManager::VirtualLaneManager(
    const EgoPlanningConfigBuilder* config_builder,
    planning::framework::Session* session) {
  session_ = session;
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
  for (size_t i = 0; i < NUM_OF_REFLINE_POINT; ++i) {
    iflyauto::ReferencePoint* current_lane_virtual_ref_point =
        &(current_lane_virtual_ref->virtual_lane_refline_points[i]);

    const double y = c0 + (c1 + (c2 + c3 * x) * x) * x;
    const double heading = c1 + (2.0 * c2 + 3.0 * c3 * x) * x;
    const double curvature =
        std::fabs(2.0 * c2 + 6.0 * c3 * x) /
        std::pow(std::pow(c1 + (2.0 * c2 + 3.0 * c3 * x) * x, 2) + 1, 1.5);

    const double half_car_width =
        VehicleConfigurationContext::Instance()->get_vehicle_param().width;

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
    current_lane_virtual_ref_point->distance_to_left_road_border =
        current_lane_virtual_ref_point->lane_width * 0.5;
    current_lane_virtual_ref_point->distance_to_right_road_border =
        current_lane_virtual_ref_point->lane_width * 0.5;

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
  const double allow_error = 5.0;
  current_lane_ = nullptr;
  left_lane_ = nullptr;
  right_lane_ = nullptr;
  relative_id_lanes_.clear();
  is_ego_on_expressway_ = false;
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_generated_refline_info()
      ->Clear();

  const iflyauto::RoadInfo* roads_ptr = &roads;
  iflyauto::RoadInfo roads_virtual;
  
  //(1)、检查lane的有效性
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
    roads_virtual.header = roads.header;
    roads_virtual.isp_timestamp = roads.isp_timestamp;

    roads_virtual.reference_line_msg[0] = current_lane_virtual;
    roads_virtual.reference_line_msg_size = 1;
    roads_virtual.local_point_valid = roads.local_point_valid;
    roads_ptr = &roads_virtual;
  } else {
    in_intersection_ = false;
  }

  //(2)、根据地图信息，计算需要的超视距信息
  CalculateDistanceToRampSplitMergeWithSdMap(session_);

  // if (session_->is_hpp_scene() && GetCurrentNearestLane(*session_)) {
    // CalculateHPPInfo(session_);
    // CalculateDistanceToTargetSlot(session_);
    // CalculateDistanceToNextSpeedBump(session_);
  // }

  double dis_between_first_road_split_and_ramp =
      distance_to_first_road_split_ - dis_to_ramp_;
  is_nearing_ramp_ =
      fabs(dis_between_first_road_split_and_ramp) < allow_error &&
      dis_to_ramp_ < 3000.;
  LOG_DEBUG(
      "dis_to_ramp: %f, dis_to_first_road_split: %f, "
      "distance_to_first_road_merge_: %f \n",
      dis_to_ramp_, distance_to_first_road_split_, distance_to_first_road_merge_);
  LOG_DEBUG("is_nearing_ramp: %d, ramp_direction_: %d \n", is_nearing_ramp_, ramp_direction_);
  LOG_DEBUG("dis to tar slot: %f, distance_to_frist_speed_bump: %f \n",
            distance_to_target_slot_, distance_to_next_speed_bump_);
  JSON_DEBUG_VALUE("is_ego_on_expressway",is_ego_on_expressway_);
  //(3)、根据计算的超视距信息，更新需要的lane信息
  relative_id_lanes_ = UpdateLanes(roads_ptr);
  
  //(4)生成导航变道的任务
  if (is_ego_on_expressway_) {
    GenerateLaneChangeTasksForNOA();
  } else {
    ResetForRampInfo();
  }

  // 构建车道kd_path/计算自车相对于各车道的横向距离
  CalculateVirtualLaneAttributes();

  // track自车道
  const auto& location_valid = session_->environmental_model().location_valid();
  auto time_start = IflyTime::Now_ms();
  if (location_valid) {
    TrackEgoLane();
  }
  auto time_end = IflyTime::Now_ms();
  LOG_DEBUG("track_ego_lane cost:%f\n", time_end - time_start);

  //(4)生成导航变道的任务
  GenerateLaneChangeTasksForNOA();

  //(5)、根据relative_id，判断current_lane_、left_lane_、right_lane_
  UpdateAllVirtualLaneInfo();
  if (current_lane_ == nullptr) {
    LOG_ERROR("!!!current_lane is empty!!!");
    return false;
  }

  //(6)、更新每条lane的virtual_lane_id,便于对每条lane的持续跟踪
  UpdateLaneVirtualId();

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

void VirtualLaneManager::UpdateLaneVirtualId() {
  // LaneChangeStatus change_status = is_lane_change();
  int lane_virtual_id;
  const auto& lane_change_decider_output = session_->mutable_planning_context()
                                         ->mutable_lane_change_decider_output();
  const auto& coarse_planning_info = lane_change_decider_output.coarse_planning_info;
  int last_ego_lane_order_id = 0;
  int current_ego_lane_order_id = last_ego_lane_order_id;
  int order_id_diff = 0;
  auto lc_last_state = coarse_planning_info.source_state;
  auto lc_state = coarse_planning_info.target_state;
  auto lc_request = lane_change_decider_output.lc_request;
  int target_lane_vitrual_id = lane_change_decider_output.target_lane_virtual_id;
  int target_lane_order_id = 0;
  int order_and_virtual_diff = 0;
  double lane_maping_diff_total = std::numeric_limits<double>::max();

  if ((lc_request == LEFT_CHANGE || lc_request == RIGHT_CHANGE) && (lc_state != ROAD_NONE)) {
    auto iter = virtual_id_mapped_lane_.find(target_lane_vitrual_id);
    if (iter != virtual_id_mapped_lane_.end()) {
      auto target_lane_frenet_coord= iter->second->get_lane_frenet_coord();
      if (target_lane_frenet_coord != nullptr) {
        for (const auto& relative_id_lane : relative_id_lanes_) {
          auto& lane_points = relative_id_lane->lane_points();
          if (lane_points.size() <= 2) {
            continue;
          }
          int default_point_nums = 20;
          int point_nums = 0;
          double lane_mapping_cost = 0.0;
          double total_lateral_offset = 0.0; 
          for (const auto &point : lane_points) {
            if (std::isnan(point.local_point.x) || std::isnan(point.local_point.y)) {
              LOG_ERROR("update_lane_points: skip NaN point");
              continue;
            }
            double lateral_offset = 0.0;
            double ego_s, ego_l;
            if (!target_lane_frenet_coord->XYToSL(point.local_point.x, point.local_point.y, &ego_s,
                                      &ego_l)) {
              lateral_offset = 10.0;
            } else {
              lateral_offset = ego_l;
            }
            total_lateral_offset += lateral_offset;
            point_nums += 1;
            if (point_nums >= default_point_nums) {
              break;
            }
          }
          lane_mapping_cost = std::fabs(total_lateral_offset / point_nums);

          if (lane_mapping_cost < lane_maping_diff_total) {
            lane_maping_diff_total = lane_mapping_cost;
            target_lane_order_id = relative_id_lane->get_order_id();
          }
        }
        order_and_virtual_diff = target_lane_vitrual_id - target_lane_order_id;

        virtual_id_mapped_lane_.clear();
        for (auto& lane : relative_id_lanes_) {
          auto lane_virtual_id = lane->get_order_id() + order_and_virtual_diff;
          virtual_id_mapped_lane_[lane_virtual_id] = lane;
          lane->set_virtual_id(lane_virtual_id);
          if (lane->get_relative_id() == 0) {
            current_lane_virtual_id_ = lane_virtual_id;
          }
        }
        return;
      } else {
        virtual_id_mapped_lane_.clear();
        for (auto& lane : relative_id_lanes_) {
          auto lane_virtual_id = lane->get_relative_id() + current_lane_virtual_id_;
          virtual_id_mapped_lane_[lane_virtual_id] = lane;
          lane->set_virtual_id(lane_virtual_id);
        }
        return;
      }
    }
  } 

  lane_virtual_id = current_lane_virtual_id_;
  // update fixlane id
  const auto& lateral_outputs =
      session_->planning_context().lateral_behavior_planner_output();
  if (lateral_outputs.lc_status == "none") {
    // sync fix lane id with current lane id while no lane change
    update_last_fix_lane_id(lane_virtual_id);
  }

  virtual_id_mapped_lane_.clear();
  for (auto& lane : relative_id_lanes_) {
    auto lane_virtual_id = lane->get_relative_id() + current_lane_virtual_id_;
    virtual_id_mapped_lane_[lane_virtual_id] = lane;
    lane->set_virtual_id(lane_virtual_id);
  }
  virtual_lane_relative_id_switch_flag_ = false;
}

// LaneChangeStatus VirtualLaneManager::is_lane_change() {
//   LaneChangeStatus change_status = LaneChangeStatus::NO_LANE_CHANGE;
//   const float lane_change_thre = 1;

//   if (virtual_id_mapped_lane_.find(current_lane_virtual_id_) !=
//       virtual_id_mapped_lane_.end()) {
//     auto& last_virtual_lane = virtual_id_mapped_lane_[current_lane_virtual_id_];
//     double left_C0, right_C0, last_left_C0, last_right_C0;
//     if (last_virtual_lane->get_left_lane_boundary().existence) {
//       last_left_C0 =
//           last_virtual_lane->get_left_lane_boundary().poly_coefficient[0];
//     } else {
//       last_left_C0 = 8.0;
//     }

//     if (last_virtual_lane->get_right_lane_boundary().existence) {
//       last_right_C0 =
//           last_virtual_lane->get_right_lane_boundary().poly_coefficient[0];
//     } else {
//       last_right_C0 = -8.0;
//     }

//     if (current_lane_->get_left_lane_boundary().existence) {
//       left_C0 = current_lane_->get_left_lane_boundary().poly_coefficient[0];
//     } else {
//       left_C0 = 8.0;
//     }

//     if (current_lane_->get_right_lane_boundary().existence) {
//       right_C0 = current_lane_->get_right_lane_boundary().poly_coefficient[0];
//     } else {
//       right_C0 = -8.0;
//     }

//     double left_diff = left_C0 - last_left_C0;
//     double right_diff = right_C0 - last_right_C0;

//     if (left_diff > lane_change_thre && right_diff > lane_change_thre &&
//         last_left_diff_ < lane_change_thre) {
//       change_status = ON_LEFT_LANE;
//     } else if (left_diff < -lane_change_thre &&
//                right_diff < -lane_change_thre &&
//                last_right_diff_ > -lane_change_thre) {
//       change_status = ON_RIGHT_LANE;
//     }
//     last_left_diff_ = left_diff;
//     last_right_diff_ = right_diff;
//     std::cout << "last_left_C0: " << last_left_C0 << " left_C0: " << left_C0
//               << " left_diff: " << left_diff
//               << " last_right_C0: " << last_right_C0
//               << " right_C0: " << right_C0 << " right_diff: " << right_diff
//               << " last_left_diff_: " << last_left_diff_
//               << " change_status: " << change_status << std::endl;

//   } else {
//     last_left_diff_ = 0;
//     last_right_diff_ = 0;
//   }

//   return change_status;
// }

void VirtualLaneManager::reset() {
  last_fix_lane_virtual_id_ = 0;
  current_lane_virtual_id_ = 0;
  virtual_id_mapped_lane_.clear();
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
  const auto& local_view = session.environmental_model().get_local_view();
  const auto& hd_map = session.environmental_model().get_hd_map();
  const auto& pose = local_view.localization_estimate.pose;

  const double ego_pose_x = pose.local_position.x;
  const double ego_pose_y = pose.local_position.y;
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
    if (local_view.localization_estimate.msf_status.msf_status !=
        iflyauto::MsfStatusType::MsfStatusType_ERROR) {
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
    if (local_view.localization_estimate.msf_status.msf_status !=
        iflyauto::MsfStatusType_ERROR) {
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
  const auto& local_view = session_->environmental_model().get_local_view();
  if (!session_->environmental_model().get_sdmap_valid()) {
    ResetForRampInfo();
    std::cout << "sd_map is invalid!!!" << std::endl;
    return;
  }
  std::cout << "sd_map valid!!!" << std::endl;

  if (local_view.localization_estimate.msf_status.msf_status ==
      iflyauto::MsfStatusType_ERROR) {
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
  std::cout << "ego_pose_x_:" << ego_pose_x_ 
            << "ego_pose_y_:" << ego_pose_y_ << std::endl;
  current_point.set_x(ego_pose_x_);
  current_point.set_y(ego_pose_y_);
  const auto& sd_map = session_->environmental_model().get_sd_map();
  double nearest_s = 0;
  double nearest_l = 0;
  const double search_distance = 50.0;
  const double max_heading_diff = PI / 4;
  const double ego_heading_angle = ego_state->heading_angle();
  const auto current_segment = sd_map.GetNearestRoadWithHeading(current_point,search_distance,ego_heading_angle,max_heading_diff,nearest_s,nearest_l);
  // const auto current_segment = sd_map.GetNearestRoad(current_point,nearest_s,nearest_l);

  current_segment_passed_distance_ = nearest_s;
  LOG_DEBUG("current_segment_passed_distance:%f\n", current_segment_passed_distance_);
  JSON_DEBUG_VALUE("current_segment_passed_distance", current_segment_passed_distance_);

  if (!current_segment) {
    ResetForRampInfo();
    return;
  } 
  if (current_segment->priority() != SdMapSwtx::RoadPriority::EXPRESSWAY) {
    std::cout << "current position not in EXPRESSWAY!!!" << std::endl;
    ResetForRampInfo();
    return;
  } else {
    is_ego_on_expressway_ = true;
  }
  //计算ramp信息
  const auto& ramp_info = sd_map.GetRampInfo(current_point);
  if (ramp_info.second > 0) {
    dis_to_ramp_ = ramp_info.second;
    const auto previous_seg = sd_map.GetPreviousRoadSegment(ramp_info.first->id());
    if (previous_seg) {
      ramp_direction_ = MakesureSplitDirection(*previous_seg, sd_map);
    } else {
      std::cout << "previous_seg is nullprt!!!!!" << std::endl;
      ramp_direction_ = RAMP_NONE;
    }

  } else {
    dis_to_ramp_ = NL_NMAX;
    ramp_direction_ = RAMP_NONE;
  }
  //计算merge信息
  //TODO(fengwang31):是否需要考虑merge的方向
  const auto& merge_info = sd_map.GetMergeInfoList(current_point);
  if (!merge_info.empty()) {
    if (merge_info.begin()->second > 0) {
      distance_to_first_road_merge_ = merge_info.begin()->second;
    } else {
      distance_to_first_road_merge_ = NL_NMAX;
    }
  } else {
    distance_to_first_road_merge_ = NL_NMAX;
    std::cout << "merge_info.empty()!!!!!!!" << std::endl;
  }

  is_on_highway_ = current_segment->priority() == SdMapSwtx::RoadPriority::EXPRESSWAY;
  is_on_ramp_ = current_segment->usage() == SdMapSwtx::RoadUsage::RAMP;
  //计算split信息
  const auto& split_info = sd_map.GetSplitInfoList(current_point);
  if (!split_info.empty()) {
    const auto split_segment = split_info.begin()->first;
    if (split_info.begin()->second > 0 && split_segment) {
        distance_to_first_road_split_ = split_info.begin()->second;
        first_split_direction_ = MakesureSplitDirection(*split_segment, sd_map);
    } else {
      distance_to_first_road_split_ = NL_NMAX;
      first_split_direction_ = RAMP_NONE;
    }
  } else {
    distance_to_first_road_split_ = NL_NMAX;
    first_split_direction_ = RAMP_NONE;
    std::cout << "split_info.empty()!!!!!!!" << std::endl;
  }
  //计算距离上一个merge点的信息
  const SdMapSwtx::Segment * last_merge_seg = current_segment;
  is_accumulate_dis_to_last_merge_point_more_than_threshold_ = false;
  double sum_dis_to_last_merge_point = nearest_s;
  if (!is_on_ramp_) {
    while(last_merge_seg->in_link().size() == 1 ) {
      if (sum_dis_to_last_merge_point > 700) {
        break;
      } else {
        last_merge_seg = sd_map.GetPreviousRoadSegment(last_merge_seg->id());
        //判断是否为nullptr
        if (!last_merge_seg) {
          break;
        } else {
          sum_dis_to_last_merge_point = sum_dis_to_last_merge_point + last_merge_seg->dis();
        }
      }
    }
    if (sum_dis_to_last_merge_point > 700) {
      is_accumulate_dis_to_last_merge_point_more_than_threshold_ = true;
    }
  }
  sum_dis_to_last_merge_point_ = sum_dis_to_last_merge_point;
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
  is_ego_on_expressway_ = false;
  first_split_direction_ = RampDirection::RAMP_NONE;
  is_leaving_ramp_ = false;
}

RampDirection VirtualLaneManager::MakesureSplitDirection (const ::SdMapSwtx::Segment& split_segment,const ad_common::sdmap::SDMap& sd_map) {
  const auto out_link_size = split_segment.out_link_size();
  RampDirection ramp_direction = RAMP_NONE;
    const auto& out_link = split_segment.out_link();
    //fengwang31(TODO):暂时假设在匝道上的分叉口只有两个方向
    if (out_link_size == 2) {              
      const auto split_next_segment = sd_map.GetNextRoadSegment(split_segment.id());
      ad_common::math::Vec2d segment_in_route_dir_vec;
      ad_common::math::Vec2d segment_not_in_route_dir_vec;
      std::cout << "out_link[0].id():" << out_link[0].id() << std::endl;
      std::cout << "out_link[1].id():" << out_link[1].id() << std::endl;
      std::cout << "split_next_segment->id()" << split_next_segment->id() << std::endl;

      auto other_segment = out_link[0].id() == split_next_segment->id() ? out_link[1] : out_link[0];
      // const auto other_segment = sd_map.GetRoadSegmentById(other_segment_id);
      if (!split_next_segment) {
        std::cout << "out segment is nullptr!!!!!!!!" << std::endl;
        return ramp_direction;
      }
      const auto& split_next_segment_enu_point = split_next_segment->enu_points();
      const auto& other_segment_enu_point = other_segment.enu_points();
      if (split_next_segment_enu_point.size() > 1 && other_segment_enu_point.size() > 1) {
        segment_in_route_dir_vec.set_x(split_next_segment_enu_point.rbegin()->x() - split_next_segment_enu_point.begin()->x());
        segment_in_route_dir_vec.set_y(split_next_segment_enu_point.rbegin()->y() - split_next_segment_enu_point.begin()->y());
        segment_not_in_route_dir_vec.set_x(other_segment_enu_point.rbegin()->x() - other_segment_enu_point.begin()->x());
        segment_not_in_route_dir_vec.set_y(other_segment_enu_point.rbegin()->y() - other_segment_enu_point.begin()->y());
        if (segment_in_route_dir_vec.CrossProd(segment_not_in_route_dir_vec) > 0.0) {
          ramp_direction = RampDirection::RAMP_ON_RIGHT;
        } else {
          ramp_direction = RampDirection::RAMP_ON_LEFT;
        }
      } else {
        std::cout << "enu points error!!!!!!!!!!" << std::endl;
      }
    } else {
      std::cout << "out_link_size != 2!!!!!!!1" << std::endl;
    }
  return ramp_direction;
}

std::vector<std::shared_ptr<VirtualLane>> 
VirtualLaneManager::UpdateLanes(const iflyauto::RoadInfo* roads_ptr) {
  std::vector<std::shared_ptr<VirtualLane>> relative_id_lanes;
  //(1)按照order_id的顺序排序输入lanes
  std::vector<iflyauto::ReferenceLineMsg> lane_msg;
  lane_msg.reserve(roads_ptr->reference_line_msg_size);
  int relative_id_zero_nums = 0;
  for (int i = 0; i < roads_ptr->reference_line_msg_size; ++i) {
    if (roads_ptr->reference_line_msg[i].relative_id == 0) {
      relative_id_zero_nums = relative_id_zero_nums + 1;
    }
    lane_msg.emplace_back(roads_ptr->reference_line_msg[i]);
  }
  auto compare_order_id = [&](iflyauto::ReferenceLineMsg lane1,
                              iflyauto::ReferenceLineMsg lane2) {
    return lane1.order_id < lane2.order_id;
  };
  std::sort(lane_msg.begin(),lane_msg.end(),compare_order_id);

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
  return relative_id_lanes;
}

void VirtualLaneManager::GenerateLaneChangeTasksForNOA() {
  lane_num_ = relative_id_lanes_.size();
  int lane_num_except_emergency = lane_num_;
  if (lane_num_ > 0) {
    if (relative_id_lanes_[lane_num_ - 1]->get_lane_type() ==
        iflyauto::LANETYPE_EMERGENCY) {
      lane_num_except_emergency -= 1;
    }
  }
  //(1)判断是否在匝道汇入主路场景，2个条件满足一个即可：
  //1、在匝道上接近汇入点100米以内；
  //2、在离开汇入点后超过500米，则视为已经完成匝道汇入主路；
  is_leaving_ramp_ = false;
  if (lane_num_except_emergency > 0) {
    if (distance_to_first_road_merge_ < 100 || 
        (sum_dis_to_last_merge_point_ < 500 && !is_on_ramp_ && is_ego_on_expressway_)) {
      is_leaving_ramp_ = true;
    }
  }

  //判断ego是否在最右边车道上
  bool is_ego_on_rightest_lane = true;
  for (const auto& relative_id_lane : relative_id_lanes_) {
    if (relative_id_lane->get_relative_id() > 0) {
      is_ego_on_rightest_lane = false;
      break;
    } 
  }
  //为了临时hack处理在匝道延长车道上的case，使得自车能在匝道延长车道上也能变道至lane上。
  //同时满足以下4个条件则认为，还在匝道延长线上：
  //1、自车当前不在匝道上；
  //2、且距离下一个匝道距离在1300m以上,距离上一个merge点在700m以内；
  //3、当前在最右边车道上；
  //4、当前是在expressway上。
  if (!is_on_ramp_ &&
      dis_to_ramp_ > 1300 && !is_accumulate_dis_to_last_merge_point_more_than_threshold_ &&
      is_ego_on_rightest_lane &&
      is_ego_on_expressway_) {
    is_leaving_ramp_ = true;
  }

  //(2)、对每一条lane，根据超视距信息，更新每一条lane的变道次数。
  for (const auto& relative_id_lane : relative_id_lanes_) {
    if (dis_to_ramp_ < 3000.0 || is_leaving_ramp_ ||(is_on_ramp_ && distance_to_first_road_merge_ > distance_to_first_road_split_)) {
      relative_id_lane->update_lane_tasks(dis_to_ramp_,
      distance_to_first_road_merge_,distance_to_first_road_split_, is_nearing_ramp_,
                                          ramp_direction_, first_split_direction_, is_leaving_ramp_,
                                          lane_num_except_emergency,is_on_ramp_);
    }
    if (relative_id_lane->get_relative_id() == 0) {
      auto left_boundary_type =
          relative_id_lane->get_left_lane_boundary().type_segments_size > 0
              ? relative_id_lane->get_left_lane_boundary().type_segments[0].type
              : iflyauto::LaneBoundaryType_MARKING_UNKNOWN;
      bool is_solid_left_boundary =
          left_boundary_type == iflyauto::LaneBoundaryType_MARKING_SOLID;

      auto right_boundary_type =
          relative_id_lane->get_right_lane_boundary().type_segments_size > 0
              ? relative_id_lane->get_right_lane_boundary()
                    .type_segments[0]
                    .type
              : iflyauto::LaneBoundaryType_MARKING_UNKNOWN;
      bool is_solid_right_boundary =
          right_boundary_type == iflyauto::LaneBoundaryType_MARKING_SOLID;
      JSON_DEBUG_VALUE("is_solid_left_boundary", is_solid_left_boundary);
      JSON_DEBUG_VALUE("is_solid_right_boundary", is_solid_right_boundary);
    }
  }
}

void VirtualLaneManager::TrackEgoLane() {
  const auto& planning_context = session_->planning_context();
  const auto& planning_result = planning_context.last_planning_result();
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto& lane_change_status = lane_change_decider_output.curr_state;
  const bool lane_keep_status = lane_change_status == ROAD_NONE;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& function_info = session_->environmental_model().function_info();
  const bool active = session_->environmental_model().GetVehicleDbwStatus();
  auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  int zero_relative_id_nums = 0;
  double select_split_min_distance_threshold = 10.0;
  is_exist_split_on_ramp_ = false;
  is_exist_ramp_on_road_ = false;

  // 判断自车是否处于车道数一分二场景
  std::vector<int> order_ids_of_same_zero_relative_id;
  order_ids_of_same_zero_relative_id.clear();

  for (const auto& relative_id_lane : relative_id_lanes_) {
    if (relative_id_lane->get_relative_id() == 0) {
      order_ids_of_same_zero_relative_id.emplace_back(relative_id_lane->get_order_id());
      zero_relative_id_nums += 1;
    }
  }
  if (!active) {
    SelectEgoLaneWithoutPlan();
  } else {
    if (!planning_result.traj_points.empty()) {
      double last_plan_length = planning_result.traj_points.back().s -
                                planning_result.traj_points.front().s;   
      if (last_plan_length < kLastPlanLengthThr) {
        SelectEgoLaneWithoutPlan();
        return;
      }
      if (is_on_highway_ && zero_relative_id_nums == 2) {
        if (!is_on_ramp_ && dis_to_ramp_ < 3000 && !is_leaving_ramp_ && lane_keep_status) {
          //hack::针对分流 感知未提供分汇流点信息 作如下后处理
          PreprocessRoadSplit(order_ids_of_same_zero_relative_id);
          LOG_DEBUG("is_exist_ramp_on_road: %d \n", is_exist_ramp_on_road_);
          JSON_DEBUG_VALUE("is_exist_ramp_on_road_",is_exist_ramp_on_road_);

          if (is_exist_ramp_on_road_) {
            return;
          }
        }
        if (is_on_ramp_ && lane_keep_status && current_segment_passed_distance_ > select_split_min_distance_threshold) {
          //选择匝道上的分叉
          PreprocessRampSplit(order_ids_of_same_zero_relative_id);
          LOG_DEBUG("is_exist_split_on_ramp: %d \n", is_exist_split_on_ramp_);
          JSON_DEBUG_VALUE("is_exist_split_on_ramp",is_exist_split_on_ramp_);

          if (is_exist_split_on_ramp_) {
            return;
          }
        }
      }
      SelectEgoLaneWithPlan(zero_relative_id_nums);
    } else {
      SelectEgoLaneWithoutPlan();
    }
  }

  LOG_DEBUG("virtual_lane_relative_id_switch_flag: %d \n", virtual_lane_relative_id_switch_flag_);
  JSON_DEBUG_VALUE("virtual_lane_relative_id_switch_flag",virtual_lane_relative_id_switch_flag_);
  return;
}

void VirtualLaneManager::SelectEgoLaneWithoutPlan() {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  double ego2lane_min = std::numeric_limits<double>::max();
  int origin_order_id = 0;
  int relative_id_diff = 0;

  for (auto& relative_id_lane : relative_id_lanes_) {
    if (relative_id_lane != nullptr) {
      if (relative_id_lane->get_lane_frenet_coord() != nullptr) {
        std::shared_ptr<KDPath> frenet_coord = relative_id_lane->get_lane_frenet_coord();
        double ego_s, ego_l;
        if (!frenet_coord->XYToSL(ego_state->ego_pose().x,
                                  ego_state->ego_pose().y, &ego_s, &ego_l)) {
          continue;
        }

        if (ego_s < frenet_coord->Length()) {
          if (std::fabs(ego_l) < ego2lane_min) {
            ego2lane_min = std::fabs(ego_l);
            origin_order_id = relative_id_lane->get_order_id();
            relative_id_lane->set_relative_id(0);
          }
        }
      }
    }
  }

  for (auto& lane : relative_id_lanes_) {
    int lane_order_id = lane->get_order_id();
    int lane_relative_id = lane_order_id - origin_order_id;
    lane->set_relative_id(lane_relative_id);
  }

  return;
}

void VirtualLaneManager::SelectEgoLaneWithPlan(int zero_relative_id_nums) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  int origin_order_id = 0;
  int current_order_id = 0;
  int relative_id_diff = 0;
  const auto& plannig_init_point = ego_state->planning_init_point();
  Point2D ego_cart_point{plannig_init_point.lat_init_state.x(),
                         plannig_init_point.lat_init_state.y()};
  const auto& planned_traj_points =
      session_->planning_context().last_planning_result().traj_points;
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  std::shared_ptr<KDPath> planned_path;
  std::vector<Vec2d> planner_path_points;
  std::unordered_map<int32_t, std::vector<double>> lane_cost_list;

  for (auto& traj_point : planned_traj_points) {
    planner_path_points.emplace_back(traj_point.x, traj_point.y);
  }

  if (planner_path_points.size() <= 2) {
    return;;
  }
  std::vector<planning_math::PathPoint> path_points;
  path_points.reserve(planner_path_points.size());
  for (const auto &point : planner_path_points) {
    if (std::isnan(point.x()) || std::isnan(point.y())) {
      LOG_ERROR("update_planner_path_points: skip NaN point");
      continue;
    }
    auto pt = planning_math::PathPoint(point.x(), point.y());
    // std ::cout << "path_point: " << pt.x() << "," << pt.y() <<std::endl;
    if (not path_points.empty()) {
      auto &last_pt = path_points.back();
      if (planning_math::Vec2d(last_pt.x() - pt.x(), last_pt.y() - pt.y())
              .Length() < 1e-2) {
        continue;
      }
    }
    path_points.emplace_back(pt);
  }

  if (path_points.size() <= 2) {
    return;
  }
  planned_path =
      std::make_shared<planning_math::KDPath>(std::move(path_points));

  double ego_s, ego_l;
  if (!planned_path->XYToSL(ego_cart_point.x, ego_cart_point.y, &ego_s,
                            &ego_l)) {
    return;
  }

  double min_s = std::numeric_limits<double>::max();
  for (const auto& relative_id_lane : relative_id_lanes_) {
    if (relative_id_lane == nullptr) {
      continue;
    }
    if (relative_id_lane->get_relative_id() == 0) {
      origin_order_id = relative_id_lane->get_order_id();
    }
    auto& lane_points = relative_id_lane->lane_points();
    if (lane_points.size() < kLaneCenterMinPointsThr) {
      continue;
    }
    const auto& last_point = lane_points.back();
    double back_s, back_l;
    if (!planned_path->XYToSL(last_point.local_point.x,
                              last_point.local_point.y, &back_s, &back_l)) {
      continue;
    }
    const auto& front_point = lane_points.front();
    double front_s, front_l;
    if (!planned_path->XYToSL(front_point.local_point.x,
                              front_point.local_point.y, &front_s, &front_l)) {
      continue;
    }
    const double larger_s = std::fmax(front_s, back_s);
    if (min_s > larger_s) {
      min_s = larger_s;
    }
  }
  const double preview_s = ego_state->ego_v() * 3.0 + ego_s;
  min_s = std::fmin(preview_s, min_s);
  min_s = std::fmax(kMinCostLength, min_s);

  const auto& lane_change_status = lane_change_decider_output.curr_state;
  bool is_lane_change = (lane_change_status == ROAD_LC_LCHANGE ||
                         lane_change_status == ROAD_LC_RCHANGE ||
                         lane_change_status == ROAD_LC_LBACK ||
                         lane_change_status == ROAD_LC_RBACK);
  bool is_lane_change_execution = (lane_change_status == ROAD_LC_LCHANGE ||
                                   lane_change_status == ROAD_LC_RCHANGE);
  const double k_init_pos_cost_weight =
      is_lane_change_execution
          ? kLaneChangeExecutionWeightRatio * kInitPosCostWeight
          : kInitPosCostWeight;

  double clane_min_diff_total = std::numeric_limits<double>::max();
  for (auto& relative_id_lane : relative_id_lanes_) {
    if (relative_id_lane == nullptr) {
      continue;
    }
    auto& lane_points = relative_id_lane->lane_points();
    if (lane_points.size() < kLaneCenterMinPointsThr) {
      continue;
    }
    double total_cost = 0.0;
    double cumu_lat_dis_cost = 0.0;
    double init_pose_cost = 0.0;
    double crosslane_cost = 0.0;
    double cumu_lat_dis_front = 0.0;
    double cumu_lat_dis_back = 0.0;
    int cumu_lat_dis_front_count = 0;
    int cumu_lat_dis_back_count = 0;

    if ((!is_lane_change) && CalcCrosslaneStatus(relative_id_lane, lane_points)) {
      crosslane_cost = kCrossLaneCostDefault;
    }

    std::shared_ptr<KDPath> frenet_coord;
    for (auto& refline_pt : lane_points) {
      double s, l;
      if (!planned_path->XYToSL(refline_pt.local_point.x,
                                refline_pt.local_point.y, &s, &l)) {
        continue;
      }
      if (s > ego_s && s < planned_path->Length() && l > 0.0) {
        cumu_lat_dis_front += std::fabs(l);
        cumu_lat_dis_front_count++;
      }
      if (s > ego_s && s < planned_path->Length() && l < 0.0) {
        cumu_lat_dis_back += std::fabs(l);
        cumu_lat_dis_back_count++;
      }

      if (s > min_s) {
        break;
      }

      if (s > planned_path->Length()) {
        break;
      }
    }
    if (cumu_lat_dis_front_count == 0 && cumu_lat_dis_back_count == 0) {
      continue;
    }

    if (cumu_lat_dis_front_count > 0) {
      cumu_lat_dis_front /=
          (cumu_lat_dis_front_count * kLatDistanceMaxStandardThr);
    }
    if (cumu_lat_dis_back_count > 0) {
      cumu_lat_dis_back /=
          (cumu_lat_dis_back_count * kLatDistanceMaxStandardThr);
    }
    cumu_lat_dis_cost = std::fmax(cumu_lat_dis_front, cumu_lat_dis_back);

    if (relative_id_lane->get_lane_frenet_coord() == nullptr) {
      frenet_coord = nullptr;
      continue;
    }
    frenet_coord = relative_id_lane->get_lane_frenet_coord();

    double ego_s_lane, ego_l_lane;
    if (!frenet_coord->XYToSL(ego_cart_point.x, ego_cart_point.y, &ego_s_lane,
                              &ego_l_lane)) {
      init_pose_cost = 0.0;
    } else {
      init_pose_cost = std::fabs(ego_l_lane) / kInitPosCostStandardThr;
    }
    total_cost = kCumuLateralDistanceCostWeight * cumu_lat_dis_cost +
                 kCrossLaneCostWeight * crosslane_cost +
                 k_init_pos_cost_weight * init_pose_cost;
    std::vector<double> cost_list{cumu_lat_dis_cost, crosslane_cost,
                                  init_pose_cost, total_cost};
    lane_cost_list[relative_id_lane->get_order_id()] = cost_list;

    if (total_cost < clane_min_diff_total) {
      clane_min_diff_total = total_cost;
      current_order_id = relative_id_lane->get_order_id();
      relative_id_lane->set_relative_id(0);
    }
  }

  for (auto& lane : relative_id_lanes_) {
    int lane_order_id = lane->get_order_id();
    int lane_relative_id = lane_order_id - current_order_id;
    lane->set_relative_id(lane_relative_id);
  }

  if (zero_relative_id_nums == 1 && current_order_id != origin_order_id) {
    virtual_lane_relative_id_switch_flag_ = true;
  }
  return;
}

bool VirtualLaneManager::CalcCrosslaneStatus(
    const std::shared_ptr<VirtualLane> lane,
    const std::vector<iflyauto::ReferencePoint>& center_line_pathpoints) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& plannig_init_point = ego_state->planning_init_point();
  bool cross_lane = false;
  Vec2d ego_pose(plannig_init_point.lat_init_state.x(),
                 plannig_init_point.lat_init_state.y());

  double left_width = 0.5 * lane->width();
  double right_width = 0.5 * lane->width();
  double ego_lateral_offset = lane->get_ego_lateral_offset();
  bool is_outside_lane = false;
  is_outside_lane = (ego_lateral_offset > left_width) ||
                    (ego_lateral_offset < -right_width);

  if (!is_outside_lane) {
    const auto& left_boundary = lane->get_left_lane_boundary();
    const auto& right_boundary = lane->get_right_lane_boundary();
    std::shared_ptr<planning_math::KDPath> left_boundary_path;
    std::shared_ptr<planning_math::KDPath> right_boundary_path;
    left_boundary_path = MakeBoundaryPath(left_boundary);
    right_boundary_path = MakeBoundaryPath(right_boundary);

    if (left_boundary_path != nullptr) {
      CalcBoundaryCross(*left_boundary_path, center_line_pathpoints,
                        &cross_lane);
    }
    if (cross_lane) {
      return true;
    }

    if (right_boundary_path != nullptr) {
      CalcBoundaryCross(*right_boundary_path, center_line_pathpoints,
                        &cross_lane);
    }
    if (cross_lane) {
      return true;
    }
  }

  return false;
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
  for (const auto &point : center_line_points) {
    if (std::isnan(point.x()) || std::isnan(point.y())) {
      LOG_ERROR("update_center_line_points: skip NaN point");
      continue;
    }
    auto pt = planning_math::PathPoint(point.x(), point.y());
    // std ::cout << "path_point: " << pt.x() << "," << pt.y() <<std::endl;
    if (not boundary_points.empty()) {
      auto &last_pt = boundary_points.back();
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

void VirtualLaneManager::CalcBoundaryCross(
    const planning_math::KDPath& lane_boundary_path,
    const std::vector<iflyauto::ReferencePoint>& center_line_pathpoints,
    bool* cross_lane) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& plannig_init_point = ego_state->planning_init_point();

  double ego_x = plannig_init_point.lat_init_state.x();
  double ego_y = plannig_init_point.lat_init_state.y();
  double ego_s = 0.0, ego_l = 0.0;
  if (!lane_boundary_path.KdtreeValid()) {
    *cross_lane = false;
    return;
  }
  if (lane_boundary_path.KdtreeValid() &&
      !lane_boundary_path.XYToSL(ego_x, ego_y, &ego_s, &ego_l)) {
    *cross_lane = false;
    return;
  }

  bool llane_pos = false;
  bool llane_neg = false;
  for (const auto& ref_pt : center_line_pathpoints) {
    double s, l;
    if (!lane_boundary_path.XYToSL(ref_pt.local_point.x, ref_pt.local_point.y,
                                   &s, &l)) {
      continue;
    }
    if (s > lane_boundary_path.Length()) {
      continue;
    }
    if (l > 0.0) {
      llane_pos = true;
    }
    if (l < 0.0) {
      llane_neg = true;
    }
    if (l * ego_l < 0.0 && (s > ego_s - kBoundaryCrossEgoBehindThr) &&
        (s < ego_s + kBoundaryCrossEgoFrontThr)) {
      *cross_lane = true;
      break;
    }
    if (llane_pos && llane_neg) {
      *cross_lane = true;
      break;
    }
  }
  return;
}

void VirtualLaneManager::PreprocessRoadSplit(const std::vector<int>& order_ids) {
  const double kSameLaneThresholdRange = 0.2;
  const double kDefaultWidth = 3.75;
  const int lane_nums = relative_id_lanes_.size();
  int origin_order_id = 0;

  if (order_ids.size() < 2) {
    is_exist_ramp_on_road_ = false;
    return;
  }

  if (ramp_direction_ == RAMP_ON_RIGHT) {
    is_exist_ramp_on_road_ = true;
    relative_id_lanes_[order_ids[1]]->set_relative_id(0);
    origin_order_id = relative_id_lanes_[order_ids[1]]->get_order_id();
  } else if (ramp_direction_ == RAMP_ON_LEFT) {
    is_exist_ramp_on_road_ = true;
    relative_id_lanes_[order_ids[0]]->set_relative_id(0);
    origin_order_id = relative_id_lanes_[order_ids[0]]->get_order_id();
  } else {
    is_exist_ramp_on_road_ = false;
    return;
  }

  for (auto& lane : relative_id_lanes_) {
    int lane_order_id = lane->get_order_id();
    int lane_relative_id = lane_order_id - origin_order_id;
    lane->set_relative_id(lane_relative_id);
  }
  return;
}

void VirtualLaneManager::PreprocessRampSplit(const std::vector<int>& order_ids) {
  int origin_order_id = 0;
  if (order_ids.size() < 2) {
    return;
  }

  if (distance_to_first_road_merge_ > distance_to_first_road_split_) {
    is_exist_split_on_ramp_ = true;
    if (first_split_direction_ == RAMP_ON_RIGHT) {
      relative_id_lanes_[order_ids[1]]->set_relative_id(0);
      origin_order_id = relative_id_lanes_[order_ids[1]]->get_order_id();        
    } else if (first_split_direction_ == RAMP_ON_LEFT) {
      relative_id_lanes_[order_ids[0]]->set_relative_id(0);
      origin_order_id = relative_id_lanes_[order_ids[0]]->get_order_id(); 
    } else {
      is_exist_split_on_ramp_ = false;
      return;
    }
  } else {
    is_exist_split_on_ramp_ = false;
    return;
  }

  for (auto& lane : relative_id_lanes_) {
    int lane_order_id = lane->get_order_id();
    int lane_relative_id = lane_order_id - origin_order_id;
    lane->set_relative_id(lane_relative_id);
  }
  return;
}

void VirtualLaneManager::CalculateVirtualLaneAttributes() {
  const auto& location_valid = session_->environmental_model().location_valid();
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& plannig_init_point = ego_state->planning_init_point();
  Point2D ego_cart_point{plannig_init_point.lat_init_state.x(),
                         plannig_init_point.lat_init_state.y()};
  for (auto& relative_id_lane : relative_id_lanes_) {
    double ego_lateral_offset = 0.0;
    if (relative_id_lane != nullptr) {
      if (location_valid) {
        auto& lane_points = relative_id_lane->lane_points();
        if (lane_points.size() <= 2) {
          continue;
        }
        std::shared_ptr<KDPath> frenet_coord;
        std::vector<planning_math::PathPoint> path_points;
        path_points.reserve(lane_points.size());
        for (const auto &point : lane_points) {
          if (std::isnan(point.local_point.x) || std::isnan(point.local_point.y)) {
            LOG_ERROR("update_lane_points: skip NaN point");
            continue;
          }
          auto pt = planning_math::PathPoint(point.local_point.x, point.local_point.y);
          // std ::cout << "path_point: " << pt.x() << "," << pt.y() <<std::endl;
          if (not path_points.empty()) {
            auto &last_pt = path_points.back();
            if (planning_math::Vec2d(last_pt.x() - pt.x(), last_pt.y() - pt.y())
                    .Length() < 1e-2) {
              continue;
            }
          }
          path_points.emplace_back(pt);
        }

        if (path_points.size() <= 2) {
          frenet_coord = nullptr;
          continue;
        }
        frenet_coord =
            std::make_shared<planning_math::KDPath>(std::move(path_points));
        relative_id_lane->set_lane_frenet_coord(frenet_coord);
        
        double ego_s, ego_l;
        if (!frenet_coord->XYToSL(ego_cart_point.x, ego_cart_point.y, &ego_s,
                                  &ego_l)) {
          ego_lateral_offset = 0.0;
        }
        ego_lateral_offset = ego_l;
      } else {
        auto& lane_points = relative_id_lane->lane_points();
        if (lane_points.size() <= 2) {
          continue;
        }
        std::shared_ptr<KDPath> frenet_coord;
        std::vector<planning_math::PathPoint> path_points;
        path_points.reserve(lane_points.size());
        for (const auto &point : lane_points) {
          if (std::isnan(point.car_point.x) || std::isnan(point.car_point.y)) {
            LOG_ERROR("update_lane_points: skip NaN point");
            continue;
          }
          auto pt = planning_math::PathPoint(point.car_point.x, point.car_point.y);
          // std ::cout << "path_point: " << pt.x() << "," << pt.y() <<std::endl;
          if (not path_points.empty()) {
            auto &last_pt = path_points.back();
            if (planning_math::Vec2d(last_pt.x() - pt.x(), last_pt.y() - pt.y())
                    .Length() < 1e-2) {
              continue;
            }
          }
          path_points.emplace_back(pt);
        }
        if (path_points.size() <= 2) {
          frenet_coord = nullptr;
          continue;
        }
        frenet_coord =
            std::make_shared<planning_math::KDPath>(std::move(path_points));
        relative_id_lane->set_lane_frenet_coord(frenet_coord);

        double ego_s, ego_l;
        if (!frenet_coord->XYToSL(ego_cart_point.x, ego_cart_point.y, &ego_s,
                                  &ego_l)) {
          ego_lateral_offset = 0.0;
        }
        ego_lateral_offset = ego_l;
      }
      relative_id_lane->set_ego_lateral_offset(ego_lateral_offset);
    } else {
      continue;
    }
  }
}

void VirtualLaneManager::UpdateAllVirtualLaneInfo() {

  auto compare_relative_id = [&](std::shared_ptr<VirtualLane> lane1,
                                 std::shared_ptr<VirtualLane> lane2) {
    return lane1->get_relative_id() < lane2->get_relative_id();
  };
  std::sort(relative_id_lanes_.begin(), relative_id_lanes_.end(),
            compare_relative_id);

  const auto& virtual_lane_mgr =
      session_->environmental_model().get_virtual_lane_manager();
  for (const auto& relative_id_lane : relative_id_lanes_) {
    if (relative_id_lane->get_relative_id() == 0) {
      current_lane_ = relative_id_lane;
      LOG_DEBUG("create current_lane_\n");
      double right_dash_line_len = virtual_lane_mgr->get_distance_to_dash_line(
          RIGHT_CHANGE, current_lane_->get_virtual_id());
      double left_dash_line_len = virtual_lane_mgr->get_distance_to_dash_line(
          LEFT_CHANGE, current_lane_->get_virtual_id());
      JSON_DEBUG_VALUE("right_dash_line_len", right_dash_line_len);
      JSON_DEBUG_VALUE("left_dash_line_len", left_dash_line_len);
    } else if (relative_id_lane->get_relative_id() == -1) {
      left_lane_ = relative_id_lane;
    } else if (relative_id_lane->get_relative_id() == 1) {
      right_lane_ = relative_id_lane;
    }
  }
}

}  // namespace planning
