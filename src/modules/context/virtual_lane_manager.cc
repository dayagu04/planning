#include "virtual_lane_manager.h"

#include <cyber/common/log.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "ad_common/hdmap/hdmap.h"
#include "common_c.h"
#include "debug_info_log.h"
#include "ehr.pb.h"
#include "environmental_model.h"
#include "fusion_road_c.h"
#include "ifly_localization_c.h"
#include "ifly_parking_map_c.h"
#include "ifly_time.h"
#include "localization_c.h"
#include "log.h"
#include "log_glog.h"
#include "math/box2d.h"
#include "planning_context.h"
#include "reference_path_manager.h"
#include "task_basic_types.h"
#include "vehicle_config_context.h"

namespace planning {

using Map::CurrentRouting;
using Map::FormOfWayType::MAIN_ROAD;
using Map::FormOfWayType::RAMP;
using ad_common::hdmap::LaneGroupConstPtr;
using ad_common::hdmap::LaneInfoConstPtr;
const double PI = 3.1415926;

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

    current_lane_virtual_ref_point->distance_to_left_road_border =
        half_car_width;
    current_lane_virtual_ref_point->distance_to_right_road_border =
        half_car_width;
    current_lane_virtual_ref_point->distance_to_left_lane_border =
        half_car_width;
    current_lane_virtual_ref_point->distance_to_right_lane_border =
        half_car_width;

    current_lane_virtual_ref_point->lane_width = 3.75;
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

    current_lane_virtual_ref_point->lane_type =
        iflyauto::LaneType::LANETYPE_VIRTUAL;

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

bool VirtualLaneManager::update(const iflyauto::RoadInfo& roads) {
  LOG_DEBUG("update VirtualLaneManager\n");
  const double allow_error = 5.0;
  current_lane_ = nullptr;
  left_lane_ = nullptr;
  right_lane_ = nullptr;
  relative_id_lanes_.clear();
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_generated_refline_info()
      ->Clear();

  const iflyauto::RoadInfo* roads_ptr = &roads;
  iflyauto::RoadInfo roads_virtual;

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

    auto& debug_info_pb = DebugInfoManager::GetInstance().GetDebugInfoPb();
    // @gpxu: 需要重新适配一下
    // debug_info_pb->add_generated_refline_info()->CopyFrom(
    //     *((planning::common::ReferenceLine*)
    //           current_lane_virtual.mutable_lane_reference_line()));

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

  is_local_valid_ = roads_ptr->local_point_valid;
  if (session_->environmental_model().function_info().function_mode() ==
      common::DrivingFunctionInfo::NOA) {
    CalculateDistanceToRampSplitMerge(session_);
  }

  if (session_->is_hpp_scene() && GetCurrentNearestLane(*session_)) {
    // CalculateHPPInfo(session_);
    // CalculateDistanceToTargetSlot(session_);
    // CalculateDistanceToNextSpeedBump(session_);
  }
  double dis_to_first_road_split = distance_to_first_road_split();
  double dis_between_first_road_split_and_ramp =
      dis_to_first_road_split - dis_to_ramp_;
  bool is_nearing_ramp =
      fabs(dis_between_first_road_split_and_ramp) < allow_error &&
      dis_to_ramp_ < 3000.;
  bool is_lane_merging = false;
  LOG_DEBUG(
      "dis_to_ramp: %f, dis_to_first_road_split: %f, "
      "distance_to_first_road_merge_: %f \n",
      dis_to_ramp_, dis_to_first_road_split, distance_to_first_road_merge_);
  LOG_DEBUG("is_nearing_ramp:%d \n", is_nearing_ramp);
  LOG_DEBUG("dis to tar slot: %f, distance_to_frist_speed_bump: %f \n",
            distance_to_target_slot_, distance_to_next_speed_bump_);

  for (int i = 0; i < roads_ptr->reference_line_msg_size; i++) {
    const auto& lane = roads_ptr->reference_line_msg[i];
    std::shared_ptr<VirtualLane> virtual_lane_tmp =
        std::make_shared<VirtualLane>();
    if (lane.lane_merge_split_point.merge_split_point_data_size == 0) {
      virtual_lane_tmp->update_data(lane);
      LOG_DEBUG("this lane has no merge_split info\n");
    } else {
      const auto& lane_merge_split_point_data =
          lane.lane_merge_split_point.merge_split_point_data[0];
      if (is_nearing_ramp) {
        LOG_DEBUG("lane_merge_split_point_data.distance:%f\n",
                  lane_merge_split_point_data.distance);
        if (((is_select_split_nearing_ramp_ &&
              lane_merge_split_point_data.is_split) ||
             (!is_select_split_nearing_ramp_ &&
              lane_merge_split_point_data.is_continue)) &&
            lane.relative_id == 0) {
          virtual_lane_tmp->update_data(lane);
        } else if ((lane.relative_id == 1 &&
                    lane_merge_split_point_data.is_split &&
                    lane_merge_split_point_data.distance < -5. &&
                    ramp_direction_ == RAMP_ON_RIGHT) ||
                   (lane.relative_id == -1 &&
                    lane_merge_split_point_data.is_split &&
                    relative_id_lanes_.size() == lane.order_id &&
                    ramp_direction_ == RAMP_ON_LEFT)) {
          virtual_lane_tmp->update_data(lane);
        } else if (lane_merge_split_point_data.is_continue) {
          virtual_lane_tmp->update_data(lane);
        } else if (lane.relative_id <= 0 &&
                   relative_id_lanes_.size() == lane.order_id) {
          virtual_lane_tmp->update_data(lane);
        } else if (lane.relative_id == 1 &&
                   !lane_merge_split_point_data.is_split &&
                   lane_merge_split_point_data.orientation == 1 &&
                   !lane_merge_split_point_data.is_continue) {
          virtual_lane_tmp->update_data(lane);
        } else {
          if (lane.relative_id == -1) {
            // std::cout << "6666666666666666666666666666: "
            //              "lane_merge_split_point_data.is_continue: "
            //           << lane_merge_split_point_data.is_continue
            //           << " lane_merge_split_point_data.is_split: "
            //           << lane_merge_split_point_data.is_split << std::endl;
          }
          continue;
        }
      } else {
        if (lane_merge_split_point_data.is_continue) {
          virtual_lane_tmp->update_data(lane);
        } else if (lane.relative_id == 0 &&
                   !lane_merge_split_point_data.is_split &&
                   lane_merge_split_point_data.orientation == 1 &&
                   relative_id_lanes_.size() == lane.order_id &&
                   lane.order_id >= 3) {
          virtual_lane_tmp->update_data(lane);
          is_lane_merging = true;
        } else if (lane.relative_id == 0 &&
                   relative_id_lanes_.size() == lane.order_id) {
          virtual_lane_tmp->update_data(lane);
        } else if (lane.relative_id == 1 &&
                   !lane_merge_split_point_data.is_split &&
                   !lane_merge_split_point_data.is_continue &&
                   lane_merge_split_point_data.orientation == 1) {
          virtual_lane_tmp->update_data(lane);
          std::cout << "update lane in merge area when ramp merge to road!!!"
                    << std::endl;
        } else {
          continue;
        }
      }
    }

    LOG_DEBUG("lane relative_id:%d, order_id:%d\n", lane.relative_id,
              lane.order_id);
    if (virtual_lane_tmp->get_lane_type() == iflyauto::LANETYPE_EMERGENCY)
      break;
    relative_id_lanes_.emplace_back(virtual_lane_tmp);
  }

  lane_num_ = relative_id_lanes_.size();
  double lane_num_except_emergency = lane_num_;
  if (lane_num_ > 0 && relative_id_lanes_[lane_num_ - 1]->get_lane_type() ==
                           iflyauto::LANETYPE_EMERGENCY) {
    lane_num_except_emergency -= 1;
  }
  if (distance_to_first_road_merge_ < 100. || is_lane_merging ||
      relative_id_lanes_[lane_num_except_emergency - 1]->get_lane_type() ==
          iflyauto::LANETYPE_ACCELERATE) {
    is_leaving_ramp_ = true;
  } else if (lane_num_except_emergency >= 3 &&
             relative_id_lanes_[lane_num_except_emergency - 1]
                     ->get_relative_id() >= lane_num_except_emergency - 3) {
    is_leaving_ramp_ = false;
  }
  for (const auto& relative_id_lane : relative_id_lanes_) {
    std::cout << "VirtualLaneManager::update_lane_tasks():: order_id_: "
              << relative_id_lane->get_order_id()
              << " lane_type: " << relative_id_lane->get_lane_type()
              << " lane_num_except_emergency: " << lane_num_except_emergency
              << " is_leaving_ramp_: " << is_leaving_ramp_ << std::endl;

    if (relative_id_lane->get_lane_type() == iflyauto::LANETYPE_EMERGENCY)
      break;
    // 正处于匝道汇主路交汇区的判断条件
    //  1、当前车道有交汇点信息
    //  2、自车所在车道为占据路权的normal车道；
    //  3、右边的车道不是分叉；且是向左汇入；且不占路权的accelerate车道。
    bool is_in_merge_area = false;
    if (relative_id_lane->get_lane_merge_split_point()
                .merge_split_point_data_size > 0 &&
        relative_id_lane->get_relative_id() == 0 &&
        relative_id_lane->get_lane_merge_split_point()
                .merge_split_point_data[0]
                .distance > 0) {
      const auto& lane_merge_split_point =
          relative_id_lane->get_lane_merge_split_point()
              .merge_split_point_data[0];
      const bool current_lane_is_continue = lane_merge_split_point.is_continue;
      const bool is_current_lane_normal = relative_id_lane->get_lane_type() ==
                                          iflyauto::LaneType::LANETYPE_NORMAL;
      // get right lane
      std::shared_ptr<VirtualLane> right_lane;
      for (const auto& get_relative_id_lane : relative_id_lanes_) {
        if (get_relative_id_lane->get_relative_id() == 1) {
          const auto& lane_order_id = get_relative_id_lane->get_order_id();
          right_lane = get_lane_with_order_id(lane_order_id);
          std::cout << "get right lane!!!" << std::endl;
          break;
        }
      }
      if (right_lane != NULL) {
        if (right_lane->get_lane_merge_split_point()
                .merge_split_point_data_size > 0) {
          const auto& right_lane_merge_info =
              right_lane->get_lane_merge_split_point()
                  .merge_split_point_data[0];
          const bool is_right_lane_accelerate =
              right_lane->get_lane_type() ==
              iflyauto::LaneType::LANETYPE_ACCELERATE;
          if (current_lane_is_continue && is_current_lane_normal &&
              is_right_lane_accelerate && !right_lane_merge_info.is_split &&
              right_lane_merge_info.orientation == 1 &&
              !right_lane_merge_info.is_continue) {
            is_in_merge_area = true;
            std::cout << "is_in_merge_area!!!!" << std::endl;
          }
        } else {
          std::cout << "right lane no merge info!!!1" << std::endl;
        }
      } else {
        std::cout << "right_lane == NULL" << std::endl;
      }
    }
    relative_id_lane->set_is_in_merge_area(is_in_merge_area);
    if (dis_to_first_road_split < 3000.0 || is_leaving_ramp_) {
      relative_id_lane->update_lane_tasks(dis_to_ramp_, is_nearing_ramp,
                                          ramp_direction_, is_leaving_ramp_,
                                          lane_num_except_emergency);
    }
  }

  auto compare_relative_id = [&](std::shared_ptr<VirtualLane> lane1,
                                 std::shared_ptr<VirtualLane> lane2) {
    return lane1->get_relative_id() < lane2->get_relative_id();
  };
  std::sort(relative_id_lanes_.begin(), relative_id_lanes_.end(),
            compare_relative_id);

  // if (relative_id_lanes_.size() == 0) {
  //   std::shared_ptr<VirtualLane> virtual_lane_tmp =
  //   std::make_shared<VirtualLane>(); iflyauto::Lane lane;
  //   lane.set_order_id(0);
  //   lane.set_relative_id(0);
  //   lane.set_ego_lateral_offset(0);
  //   lane.set_lane_type(iflyauto::LANE_TYPE_NORMAL);
  //   lane.set_lane_marks(iflyauto::LaneDrivableDirection::DIRECTION_STRAIGHT);
  //   lane.mutable_lane_reference_line().set_existence(false);
  //   lane.mutable_lane_merge_split_point.set_existence(false);
  //   lane.mutable_left_lane_boundary.set_existence(false);
  //   lane.mutable_right_lane_boundary.set_existence(false);
  //   virtual_lane_tmp->update_data(lane);
  //   relative_id_lanes_.emplace_back(virtual_lane_tmp);
  // }

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

  if (current_lane_ == nullptr) {
    LOG_ERROR("!!!current_lane is empty!!!");
    return false;
  }

  update_virtual_id();
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

  return true;
}

const std::shared_ptr<VirtualLane> VirtualLaneManager::get_lane_with_virtual_id(
    int virtual_id) const {
  if (virtual_id_mapped_lane_.find(virtual_id) !=
      virtual_id_mapped_lane_.end()) {
    LOG_DEBUG("get lane virtual %d id\n", virtual_id);
    return virtual_id_mapped_lane_[virtual_id];
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

void VirtualLaneManager::update_virtual_id() {
  LaneChangeStatus change_status = is_lane_change();
  int lane_virtual_id;
  if (change_status == LaneChangeStatus::NO_LANE_CHANGE) {
    lane_virtual_id = current_lane_virtual_id_;
  } else if (change_status == LaneChangeStatus::ON_LEFT_LANE) {
    lane_virtual_id = current_lane_virtual_id_ - 1;
  } else {
    lane_virtual_id = current_lane_virtual_id_ + 1;
  }

  current_lane_virtual_id_ = lane_virtual_id;
  // update fixlane id
  const auto& lateral_outputs =
      session_->planning_context().lateral_behavior_planner_output();
  if (lateral_outputs.lc_status == "none") {
    // sync fix lane id with current lane id while no lane change
    update_last_fix_lane_id(lane_virtual_id);
  }

  virtual_id_mapped_lane_.clear();
  for (const auto& lane : relative_id_lanes_) {
    auto lane_virtual_id = lane->get_relative_id() + current_lane_virtual_id_;
    virtual_id_mapped_lane_[lane_virtual_id] = lane;
    lane->set_virtual_id(lane_virtual_id);
  }
}

LaneChangeStatus VirtualLaneManager::is_lane_change() {
  LaneChangeStatus change_status = LaneChangeStatus::NO_LANE_CHANGE;
  const float lane_change_thre = 1;

  if (virtual_id_mapped_lane_.find(current_lane_virtual_id_) !=
      virtual_id_mapped_lane_.end()) {
    auto& last_virtual_lane = virtual_id_mapped_lane_[current_lane_virtual_id_];
    double left_C0, right_C0, last_left_C0, last_right_C0;
    if (last_virtual_lane->get_left_lane_boundary().existence) {
      last_left_C0 =
          last_virtual_lane->get_left_lane_boundary().poly_coefficient[0];
    } else {
      last_left_C0 = 8.0;
    }

    if (last_virtual_lane->get_right_lane_boundary().existence) {
      last_right_C0 =
          last_virtual_lane->get_right_lane_boundary().poly_coefficient[0];
    } else {
      last_right_C0 = -8.0;
    }

    if (current_lane_->get_left_lane_boundary().existence) {
      left_C0 = current_lane_->get_left_lane_boundary().poly_coefficient[0];
    } else {
      left_C0 = 8.0;
    }

    if (current_lane_->get_right_lane_boundary().existence) {
      right_C0 = current_lane_->get_right_lane_boundary().poly_coefficient[0];
    } else {
      right_C0 = -8.0;
    }

    double left_diff = left_C0 - last_left_C0;
    double right_diff = right_C0 - last_right_C0;

    if (left_diff > lane_change_thre && right_diff > lane_change_thre &&
        last_left_diff_ < lane_change_thre) {
      change_status = ON_LEFT_LANE;
    } else if (left_diff < -lane_change_thre &&
               right_diff < -lane_change_thre &&
               last_right_diff_ > -lane_change_thre) {
      change_status = ON_RIGHT_LANE;
    }
    last_left_diff_ = left_diff;
    last_right_diff_ = right_diff;
    std::cout << "last_left_C0: " << last_left_C0 << " left_C0: " << left_C0
              << " left_diff: " << left_diff
              << " last_right_C0: " << last_right_C0
              << " right_C0: " << right_C0 << " right_diff: " << right_diff
              << " last_left_diff_: " << last_left_diff_
              << " change_status: " << change_status << std::endl;

  } else {
    last_left_diff_ = 0;
    last_right_diff_ = 0;
  }

  return change_status;
}

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
      if (session_->is_hpp_scene()) {
        // TODO(xjli32): location_valid含义模糊
        if (!session_->environmental_model().location_valid()) {
          return false;
        }
        const auto& ego_state =
            session.environmental_model().get_ego_state_manager();
        ego_pose_x_ = ego_state->ego_pose_raw().x;
        ego_pose_y_ = ego_state->ego_pose_raw().y;
        yaw_ = ego_state->ego_pose_raw().theta;
        point.set_x(ego_pose_x_);
        point.set_y(ego_pose_y_);
        std::cout << "in hpp,"
                  << "ego_pose_x_:" << ego_pose_x_
                  << ",ego_pose_y_:" << ego_pose_y_ << std::endl;
      } else {
        const auto& pose = local_view.localization_estimate.pose;
        const double ego_pose_x = pose.local_position.x;
        const double ego_pose_y = pose.local_position.y;
        point.set_x(ego_pose_x);
        point.set_y(ego_pose_y);
        std::cout << "in NOA,"
                  << "ego_pose_x:" << point.x() << ",ego_pose_y:" << point.y()
                  << std::endl;
      }

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
      for (int i = 1; i < num_of_reflane_point; i++) {
        const auto& prev_point = points[i - 1].car_point;
        const auto& current_point = points[i].car_point;
        current_lane_exist |= (current_point.y < current_lane_y_thrs);
        double y_interval = std::fabs(current_point.y - prev_point.y);
        if (y_interval > max_y_interval) {
          LOG_ERROR("lane_valid is error \n");
          y_interval_valid = false;
          break;
        }
      }
    }
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
}
}  // namespace planning