#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "basic_types.pb.h"
#include "config/basic_type.h"
#include "environmental_model.h"
#include "gap_selector.pb.h"
#include "planning_debug_info.pb.h"
#include "tasks/behavior_planners/gap_selector_decider/gap_selector_decider.h"
#include "tasks/behavior_planners/gap_selector_decider/gap_selector_interface.h"
#include "trajectory1d/trajectory1d.h"
#include "utils/frenet_coordinate_system.h"
#include "utils/kd_path.h"
#include "utils/path_point.h"

#include "serialize_utils.h"

namespace py = pybind11;
using namespace planning;
constexpr int PointNum = 25;
constexpr double delta_t = 0.2;
static GapSelectorDecider *pBase = nullptr;
static GapSelectorInterface *pInterface = nullptr;
static Json json;
static const char *file_name = "test.json";
static EgoPlanningConfigBuilder config_builder(json, file_name);
static framework::Session session;

int Init() {
  pBase = new GapSelectorDecider(&config_builder, &session);
  pInterface = new GapSelectorInterface();
  return 0;
}

void Clear() {
  // clear
  pBase->loninfo_time_optimal_.clear();
  pBase->gap_selector_path_spline_.clear();
  pBase->gap_list_.clear();
  pBase->st_time_optimal_.clear();
  pBase->front_gap_car_st_boundaries_.clear();
  pBase->rear_gap_car_st_boundaries_.clear();
  pBase->front_careful_car_st_boundary_.clear();
  pBase->front_careful_car_id_origin_lane_ = -1;

  pBase->traj_time_optimal_.valid_ = false;
  pBase->nearby_gap_.front_agent_id = -1;
  pBase->nearby_gap_.rear_agent_id = -1;

  pBase->agent_node_mgr_ = nullptr;
  pBase->base_frenet_coord_ = nullptr;
  return;
}

void FeedInput(bool closed_loop) {
  if (closed_loop) {
    pBase->gap_drive_style_ = planning::NONESTYLE;
    const double dt = 0.1;
    const auto &traj_points = *pBase->traj_points_ptr_;
    auto N = traj_points.size();

    std::vector<double> x_vec(N);
    std::vector<double> y_vec(N);
    std::vector<double> a_vec(N);
    std::vector<double> v_vec(N);
    std::vector<double> curv_vec(N);
    std::vector<double> theta_vec(N);
    std::vector<double> t_vec(N);

    double angle_offset = 0.;
    static const double pi_const = 3.141592654;

    pnc::mathlib::spline x_t_spline;
    pnc::mathlib::spline y_t_spline;
    pnc::mathlib::spline a_t_spline;
    pnc::mathlib::spline v_t_spline;
    pnc::mathlib::spline theta_t_spline;

    for (auto i = 0; i < N; i++) {
      x_vec[i] = traj_points[i].x;
      y_vec[i] = traj_points[i].y;
      a_vec[i] = traj_points[i].a;
      v_vec[i] = traj_points[i].v;
      t_vec[i] = traj_points[i].t;

      if (i == 0) {
        theta_vec[i] = traj_points[i].heading_angle;
      } else {
        const auto delta_theta = theta_vec[i] - theta_vec[i - 1];
        if (delta_theta > 1.5 * pi_const) {
          angle_offset -= 2.0 * pi_const;
        } else if (delta_theta < -1.5 * pi_const) {
          angle_offset += 2.0 * pi_const;
        }
        theta_vec[i] = traj_points[i].heading_angle + angle_offset;
      }
    }
    x_t_spline.set_points(t_vec, x_vec);
    y_t_spline.set_points(t_vec, y_vec);
    a_t_spline.set_points(t_vec, a_vec);
    v_t_spline.set_points(t_vec, v_vec);
    theta_t_spline.set_points(t_vec, theta_vec);

    // calc curv
    double curv = 0.;
    curv = (x_t_spline.deriv(1, dt) * y_t_spline.deriv(2, dt) -
            y_t_spline.deriv(1, dt) * x_t_spline.deriv(2, dt)) /
           std::pow(x_t_spline.deriv(1, dt) * x_t_spline.deriv(1, dt) +
                        y_t_spline.deriv(1, dt) * y_t_spline.deriv(1, dt),
                    1.5);

    pBase->planning_init_point_.x = x_t_spline(dt);
    pBase->planning_init_point_.y = y_t_spline(dt);
    pBase->planning_init_point_.a = a_t_spline(dt);
    Point2D ego_frenet_point{0., 0.};
    if (pBase->base_frenet_coord_->XYToSL(
            Point2D{pBase->planning_init_point_.x,
                    pBase->planning_init_point_.y},
            ego_frenet_point) == TRANSFORM_SUCCESS) {
      LOG_ERROR("Closed Loop Update Init Frenet Point Success!");
      pBase->planning_init_point_.frenet_state.s = ego_frenet_point.x;
      pBase->planning_init_point_.frenet_state.r = ego_frenet_point.y;
    }
    std::cout << "\n frenet coord updated frenet l is:" << ego_frenet_point.y
              << std::endl;
    pBase->planning_init_point_.v = v_t_spline(dt);
    pBase->planning_init_point_.heading_angle = theta_t_spline(dt);
    pBase->planning_init_point_.curvature = curv;
    pBase->ego_cart_point_ = {pBase->planning_init_point_.x,
                              pBase->planning_init_point_.y};
    pBase->ego_planning_init_s_ = {pBase->planning_init_point_.frenet_state.s,
                                   pBase->planning_init_point_.v,
                                   pBase->planning_init_point_.a};
    pBase->ego_l_ = pBase->planning_init_point_.frenet_state.r;

    // state machine update
    pBase->gap_selector_state_machine_info_.lc_pass_time += 0.1;
    // TODO: reserve for lc wait time

  } else {
    pBase->cruise_vel_ = pInterface->gap_selector_feed_info_.cruise_vel;
    pBase->gap_selector_state_machine_info_ = {
        pInterface->gap_selector_feed_info_.gap_selector_info.lane_cross,
        pInterface->gap_selector_feed_info_.gap_selector_info.lc_triggered,
        pInterface->gap_selector_feed_info_.gap_selector_info.lb_triggered,
        pInterface->gap_selector_feed_info_.gap_selector_info.lc_in,
        pInterface->gap_selector_feed_info_.gap_selector_info.lb_in,
        pInterface->gap_selector_feed_info_.gap_selector_info.lc_pass_time,
        pInterface->gap_selector_feed_info_.gap_selector_info.lc_wait_time,
        pInterface->gap_selector_feed_info_.gap_selector_info.lc_wait_time,
        static_cast<bool>(pInterface->gap_selector_feed_info_.gap_selector_info
                              .path_requintic),
        pInterface->gap_selector_feed_info_.gap_selector_info.lc_cancel,
        pInterface->gap_selector_feed_info_.gap_selector_info.gs_skip,
        pInterface->gap_selector_feed_info_.lc_request_buffer,
        pInterface->gap_selector_feed_info_.ego_l_buffer};
    pBase->traj_points_ptr_ = &pInterface->gap_selector_feed_info_.traj_points;
    pBase->ego_cart_point_ = {
        pInterface->gap_selector_feed_info_.planning_init_point_.x,
        pInterface->gap_selector_feed_info_.planning_init_point_.y};
    pBase->ego_planning_init_s_ = {
        pInterface->gap_selector_feed_info_.planning_init_point_.frenet_state.s,
        pInterface->gap_selector_feed_info_.planning_init_point_.v,
        pInterface->gap_selector_feed_info_.planning_init_point_.a};
    pBase->ego_l_ =
        pInterface->gap_selector_feed_info_.planning_init_point_.frenet_state.r;
    pBase->planning_init_point_ =
        pInterface->gap_selector_feed_info_.planning_init_point_;
  }
  pBase->transform_.SetEgoState(pBase->planning_init_point_.heading_angle,
                                pBase->planning_init_point_.x,
                                pBase->planning_init_point_.y);
}
bool Preprocessor(planning::common::GapSelectorInput &input, bool close_loop,
                  bool enable, int front_obj_id, double front_obj_add_vel,
                  double front_obj_add_s, int rear_obj_id,
                  double rear_obj_add_vel, double rear_obj_add_s) {
  if (!enable) {
    pBase->ResetAllInfo();
    // //
    // pInterface->gap_selector_feed_info_.gap_selector_info.last_gap_selector_path_spline
    // // // @cailiu2: reserve
    pInterface->gap_selector_feed_info_.traj_points.clear();
    pInterface->gap_selector_feed_info_.target_state = 0;
    pInterface->gap_selector_feed_info_.map_target_lane_obstacles.clear();
    pInterface->gap_selector_feed_info_.map_gs_care_obstacles.clear();
    pInterface->gap_selector_feed_info_.map_origin_lane_obstacles.clear();
    pInterface->gap_selector_feed_info_.origin_refline_points.clear();
    pInterface->gap_selector_feed_info_.target_refline_points.clear();
    pInterface->gap_selector_feed_info_.origin_lane_s_width_vec.clear();
    pInterface->gap_selector_feed_info_.target_lane_s_width_vec.clear();

    pInterface->gap_selector_feed_info_.gap_selector_info = {
        false, false, false, false, false, 0.0, 0.0, false, false, false, 0};
    pInterface->gap_selector_feed_info_.ego_l_buffer = {0., 0., 0.};
    pInterface->gap_selector_feed_info_.lc_request_buffer = {0, 0, 0};

    return false;
  }

  pBase->target_state_ = pInterface->gap_selector_feed_info_.target_state;
  pBase->gap_selector_state_machine_info_.origin_lane_id =
      pInterface->gap_selector_feed_info_.origin_lane_id;
  pBase->gap_selector_state_machine_info_.target_lane_id =
      pInterface->gap_selector_feed_info_.target_lane_id;
  pBase->gap_selector_state_machine_info_.current_lane_id =
      pInterface->gap_selector_feed_info_.current_lane_id;
  pBase->cruise_vel_ = pInterface->gap_selector_feed_info_.cruise_vel;
  pBase->ego_l_cur_lane_ = pInterface->gap_selector_feed_info_.ego_l_cur_lane;

  pBase->agent_node_mgr_ = std::make_shared<AgentNodeManager>(
      pInterface->gap_selector_feed_info_.origin_refline_points,
      pInterface->gap_selector_feed_info_.target_refline_points,
      pInterface->gap_selector_feed_info_.map_gs_care_obstacles,
      pInterface->gap_selector_feed_info_.map_target_lane_obstacles,
      pInterface->gap_selector_feed_info_.map_origin_lane_obstacles,
      pInterface->gap_selector_feed_info_.target_state);

  // current refline kd path
  auto current_points =
      pInterface->gap_selector_feed_info_.current_refline_points;
  std::vector<planning_math::PathPoint> current_path_points;
  for (size_t i = 0; i < current_points.size(); i++) {
    current_path_points.emplace_back(planning_math::PathPoint{
        current_points[i].first, current_points[i].second});
  }
  if (current_path_points.size() > 3) {
    pBase->current_lane_coord_ptr_ =
        std::make_shared<KDPath>(std::move(current_path_points));
  }

  // origin refline kd path
  auto origin_points =
      pInterface->gap_selector_feed_info_.origin_refline_points;
  std::vector<planning_math::PathPoint> origin_path_points;
  for (size_t i = 0; i < origin_points.size(); i++) {
    origin_path_points.emplace_back(planning_math::PathPoint{
        origin_points[i].first, origin_points[i].second});
  }
  if (origin_path_points.size() > 3) {
    pBase->origin_lane_coord_ptr_ =
        std::make_shared<KDPath>(std::move(origin_path_points));
  }

  // target refline kd path
  auto target_points =
      pInterface->gap_selector_feed_info_.target_refline_points;
  std::vector<planning_math::PathPoint> target_path_points;
  for (size_t i = 0; i < target_points.size(); i++) {
    target_path_points.emplace_back(planning_math::PathPoint{
        target_points[i].first, target_points[i].second});
  }
  if (target_points.size() > 3) {
    pBase->target_lane_coord_ptr_ =
        std::make_shared<KDPath>(std::move(target_path_points));
  }

  // // feed input
  FeedInput(close_loop);

  if (front_obj_id > 0) {
    pBase->agent_node_mgr_->RefineObjInitState(
        (int64_t)front_obj_id, front_obj_add_vel, front_obj_add_s);
  };
  if (rear_obj_id) {
    pBase->agent_node_mgr_->RefineObjInitState(
        (int64_t)rear_obj_id, rear_obj_add_vel, rear_obj_add_s);
  };
  return true;
}

int UpdateBytes(py::bytes &gap_selector_input_bytes, bool closed_loop,
                bool enable, int front_obj_id, float front_obj_add_vel,
                double front_obj_add_s, int rear_obj_id,
                double rear_obj_add_vel, double rear_obj_add_s,
                double v_cruise) {
  planning::common::GapSelectorInput gap_selector_input =
      BytesToProto<planning::common::GapSelectorInput>(
          gap_selector_input_bytes);
  pInterface->Parse(gap_selector_input);

  bool update_env_ok = Preprocessor(
      gap_selector_input, closed_loop, enable, front_obj_id, front_obj_add_vel,
      front_obj_add_s, rear_obj_id, rear_obj_add_vel, rear_obj_add_s);
  int gap_status = 0;
  if (update_env_ok) {
    if (v_cruise > 0) {
      pInterface->gap_selector_feed_info_.cruise_vel = v_cruise;
    }

    GapSelectorDeciderOutput gap_selector_decider_output;
    gap_status = pBase->Update(gap_selector_decider_output);
  }

  return gap_status;
}

int GetDriveStyle() {
  GapDriveStyle drive_style = pBase->gap_drive_style_;
  switch (drive_style) {
    case GapDriveStyle::NONESTYLE:
      return 1;
    case GapDriveStyle::OnlyRearFasterCar:
      return 2;
    case GapDriveStyle::OnlyRearSlowerCar:
      return 3;
    case GapDriveStyle::RearCarFaster:
      return 4;
    case GapDriveStyle::FrontCarFaster:
      return 5;
    case GapDriveStyle::NoDecisionForBothCar:
      return 6;
    case GapDriveStyle::OnlyFrontFasterCar:
      return 7;
    case GapDriveStyle::OnlyFrontSlowerCar:
      return 8;
    case GapDriveStyle::Free:
      return 9;
  }
}

int GetFrontGapAgentInfo() {
  const Gap &nearby_gap = pBase->nearby_gap_;
  const std::shared_ptr<AgentNodeManager> agent_node_mgr =
      pBase->agent_node_mgr_;
  const std::unordered_map<int, Obstacle> &map_obstalces =
      agent_node_mgr->map_gs_care_obstacles();
  planning::common::ObstacleCornalPoints obstacle_cornal;
  if (nearby_gap.front_agent_id > 0) {
    auto iter = map_obstalces.find(nearby_gap.front_agent_id);
    if (iter != map_obstalces.end()) {
      for (auto &point : iter->second.perception_points()) {
        obstacle_cornal.add_x_vec(point.x());
        obstacle_cornal.add_y_vec(point.y());
      }
    }
  }

  std::string serialized_message;
  obstacle_cornal.SerializeToString(&serialized_message);
  // return serialized_message;
  return nearby_gap.front_agent_id;
}

int GetRearGapAgentInfo() {
  const Gap &nearby_gap = pBase->nearby_gap_;
  const std::shared_ptr<AgentNodeManager> agent_node_mgr =
      pBase->agent_node_mgr_;
  const std::unordered_map<int, Obstacle> &map_obstalces =
      agent_node_mgr->map_gs_care_obstacles();
  planning::common::ObstacleCornalPoints obstacle_cornal;
  if (nearby_gap.rear_agent_id > 0) {
    auto iter = map_obstalces.find(nearby_gap.rear_agent_id);
    if (iter != map_obstalces.end()) {
      for (auto &point : iter->second.perception_points()) {
        obstacle_cornal.add_x_vec(point.x());
        obstacle_cornal.add_y_vec(point.y());
      }
    }
  }

  std::string serialized_message;
  obstacle_cornal.SerializeToString(&serialized_message);
  // return serialized_message;
  return nearby_gap.rear_agent_id;
}

py::bytes TimeOptimalStateLimit() {
  planning::common::StateLimit state_limit;
  state_limit.set_a_max(pBase->traj_time_optimal_.state_limit().a_max);
  state_limit.set_a_min(pBase->traj_time_optimal_.state_limit().a_min);
  state_limit.set_j_max(pBase->traj_time_optimal_.state_limit().j_max);
  state_limit.set_j_min(pBase->traj_time_optimal_.state_limit().j_min);
  state_limit.set_p_end(pBase->traj_time_optimal_.state_limit().p_end);
  state_limit.set_v_end(pBase->traj_time_optimal_.state_limit().v_end);
  state_limit.set_v_max(pBase->traj_time_optimal_.state_limit().v_max);
  state_limit.set_v_min(pBase->traj_time_optimal_.state_limit().v_min);

  std::string serialized_message;
  state_limit.SerializeToString(&serialized_message);
  return serialized_message;
}

py::bytes GetOutputBytes() {
  // construct output
  planning::common::GapSelectorOutput output;
  std::cout << "gs_traj_size: " << pBase->traj_points_ptr_->size() << std::endl;
  std::cout << "interface gs traj size: "
            << pInterface->gap_selector_feed_info_.traj_points.size()
            << std::endl;
  // print("\ngs_traj_size:", pBase->traj_points_ptr_->size());
  for (auto i = 0; i < pBase->traj_points_ptr_->size(); i++) {
    planning::common::TrajectoryPoint *traj_point =
        output.mutable_gs_traj()->Add();
    traj_point->set_a(pBase->traj_points_ptr_->at(i).a);
    traj_point->set_x(pBase->traj_points_ptr_->at(i).x);
    traj_point->set_y(pBase->traj_points_ptr_->at(i).y);
    traj_point->set_v(pBase->traj_points_ptr_->at(i).v);
    traj_point->set_s(pBase->traj_points_ptr_->at(i).s);
    traj_point->set_l(pBase->traj_points_ptr_->at(i).l);
    traj_point->set_t(pBase->traj_points_ptr_->at(i).t);
    traj_point->set_heading_angle(pBase->traj_points_ptr_->at(i).heading_angle);
  }

  // path spline points
  output.mutable_path_spline_data()->Reserve(2);
  if (!pBase->gap_selector_path_spline_.empty()) {
    planning::common::PathData *path_data =
        output.mutable_path_spline_data()->Add();
    for (auto j = 0; j < 2 * (PointNum + 1); j++) {
      planning::common::Point2d *point = path_data->add_points();
      point->set_x(pBase->gap_selector_path_spline_[0].x_s_spline(
          j * delta_t * pBase->ego_planning_init_s_[1]));
      point->set_y(pBase->gap_selector_path_spline_[0].y_s_spline(
          j * delta_t * pBase->ego_planning_init_s_[1]));
    }
  }
  if ((int)pBase->path_spline_.path_spline_status != 0) {
    planning::common::PathData *path_data =
        output.mutable_path_spline_data()->Add();
    for (auto j = 0; j < 2 * (PointNum + 1); j++) {
      planning::common::Point2d *point = path_data->add_points();
      point->set_x(pBase->path_spline_.x_s_spline(
          j * delta_t * pBase->ego_planning_init_s_[1]));
      point->set_y(pBase->path_spline_.y_s_spline(
          j * delta_t * pBase->ego_planning_init_s_[1]));
    }
  }

  if (pBase->origin_lane_coord_ptr_ != nullptr) {
    for (auto i = 0; i < pBase->origin_lane_coord_ptr_->path_points().size();
         i++) {
      planning::common::Point2d *origin_coord_point =
          output.add_frenet_coord_points();
      origin_coord_point->set_x(
          pBase->origin_lane_coord_ptr_->path_points().at(i).x());
      origin_coord_point->set_y(
          pBase->origin_lane_coord_ptr_->path_points().at(i).y());
    }
  }

  if (pBase->target_lane_coord_ptr_ != nullptr) {
    for (auto i = 0; i < pBase->target_lane_coord_ptr_->path_points().size();
         i++) {
      planning::common::Point2d *taget_coord_point =
          output.add_target_coord_points();
      taget_coord_point->set_x(
          pBase->target_lane_coord_ptr_->path_points().at(i).x());
      taget_coord_point->set_y(
          pBase->target_lane_coord_ptr_->path_points().at(i).y());
    }
  }

  if (pBase->current_lane_coord_ptr_ != nullptr) {
    for (auto i = 0; i < pBase->current_lane_coord_ptr_->path_points().size();
         i++) {
      planning::common::Point2d *current_coord_point =
          output.add_current_coord_points();
      current_coord_point->set_x(
          pBase->current_lane_coord_ptr_->path_points().at(i).x());
      current_coord_point->set_y(
          pBase->current_lane_coord_ptr_->path_points().at(i).y());
    }
  }

  // for st boundaries
  for (auto i = 0; i < pBase->front_gap_car_st_boundaries_.size(); i++) {
    planning::common::STPoint *st_point_upper =
        output.add_upper_st_boundary_front_car();
    st_point_upper->set_x(pBase->front_gap_car_st_boundaries_[i].first.x());
    st_point_upper->set_y(pBase->front_gap_car_st_boundaries_[i].first.y());
    st_point_upper->set_s(pBase->front_gap_car_st_boundaries_[i].first.s());
    st_point_upper->set_t(pBase->front_gap_car_st_boundaries_[i].first.t());
    planning::common::STPoint *st_point_lower =
        output.add_lower_st_boundary_front_car();
    st_point_lower->set_x(pBase->front_gap_car_st_boundaries_[i].second.x());
    st_point_lower->set_y(pBase->front_gap_car_st_boundaries_[i].second.y());
    st_point_lower->set_s(pBase->front_gap_car_st_boundaries_[i].second.s());
    st_point_lower->set_t(pBase->front_gap_car_st_boundaries_[i].second.t());
  }
  for (auto i = 0; i < pBase->rear_gap_car_st_boundaries_.size(); i++) {
    planning::common::STPoint *st_point_upper =
        output.add_upper_st_boundary_rear_car();
    st_point_upper->set_x(pBase->rear_gap_car_st_boundaries_[i].first.x());
    st_point_upper->set_y(pBase->rear_gap_car_st_boundaries_[i].first.y());
    st_point_upper->set_s(pBase->rear_gap_car_st_boundaries_[i].first.s());
    st_point_upper->set_t(pBase->rear_gap_car_st_boundaries_[i].first.t());
    planning::common::STPoint *st_point_lower =
        output.add_lower_st_boundary_rear_car();
    st_point_lower->set_x(pBase->rear_gap_car_st_boundaries_[i].second.x());
    st_point_lower->set_y(pBase->rear_gap_car_st_boundaries_[i].second.y());
    st_point_lower->set_s(pBase->rear_gap_car_st_boundaries_[i].second.s());
    st_point_lower->set_t(pBase->rear_gap_car_st_boundaries_[i].second.t());
  }

  for (auto i = 0; i < pBase->front_careful_car_st_boundary_.size(); i++) {
    planning::common::STPoint *st_point_upper =
        output.add_upper_st_boundary_origin_front_car();
    st_point_upper->set_x(pBase->front_careful_car_st_boundary_[i].first.x());
    st_point_upper->set_y(pBase->front_careful_car_st_boundary_[i].first.y());
    st_point_upper->set_s(pBase->front_careful_car_st_boundary_[i].first.s());
    st_point_upper->set_t(pBase->front_careful_car_st_boundary_[i].first.t());
    planning::common::STPoint *st_point_lower =
        output.add_lower_st_boundary_origin_front_car();
    st_point_lower->set_x(pBase->front_careful_car_st_boundary_[i].second.x());
    st_point_lower->set_y(pBase->front_careful_car_st_boundary_[i].second.y());
    st_point_lower->set_s(pBase->front_careful_car_st_boundary_[i].second.s());
    st_point_lower->set_t(pBase->front_careful_car_st_boundary_[i].second.t());
  }

  // if (!pBase->gap_selector_path_spline_.empty()) {
  //   auto path_spline = pBase->gap_selector_path_spline_[0];
  //   std::vector<double> x_spline_points;
  //   std::vector<double> y_spline_points;
  //   for (auto i = 0; i < PointNum; i++) {
  //     planning::common::Point2d *origin_spline_point =
  //         output.add_origin_spline_points();
  //     origin_spline_point->set_x(path_spline.x_s_spline.get_x().at(i));
  //     origin_spline_point->set_y(path_spline.x_s_spline.get_y().at(i));
  //   }
  // }

  if (pBase->nearby_gap_.front_agent_id > -1) {
    auto iter = pBase->agent_node_mgr_->agent_node_target_lane_map().find(
        pBase->nearby_gap_.front_agent_id);

    if (iter != pBase->agent_node_mgr_->agent_node_target_lane_map().end()) {
      ObstaclePredicatedPoints front_obstacle_pred_info =
          iter->second.obstacle_pred_info;
      for (auto i = 0; i < front_obstacle_pred_info.size(); i++) {
        planning::common::ObstaclePredicatedPoint *obstacle_predicate_point =
            output.add_obstacle_predicate_points_front_gap_car();
        obstacle_predicate_point->set_x(front_obstacle_pred_info[i].x);
        obstacle_predicate_point->set_y(front_obstacle_pred_info[i].y);
      }
    } else {
      std::cout
          << "\n front_agent_id > -1, but agent node lane map doesnt exists "
          << std::endl;
    }
  }
  if (pBase->nearby_gap_.rear_agent_id > -1) {
    auto iter = pBase->agent_node_mgr_->agent_node_target_lane_map().find(
        pBase->nearby_gap_.rear_agent_id);

    if (iter != pBase->agent_node_mgr_->agent_node_target_lane_map().end()) {
      ObstaclePredicatedPoints rear_obstacle_pred_info =
          iter->second.obstacle_pred_info;
      for (auto i = 0; i < rear_obstacle_pred_info.size(); i++) {
        planning::common::ObstaclePredicatedPoint *obstacle_predicate_point =
            output.add_obstacle_predicate_points_rear_gap_car();
        obstacle_predicate_point->set_x(rear_obstacle_pred_info[i].x);
        obstacle_predicate_point->set_y(rear_obstacle_pred_info[i].y);
      }
    } else {
      std::cout
          << "\n rear_agent_id > -1, but agent node lane map doesnt exists "
          << std::endl;
    }
  }
  // for corss line point

  output.mutable_cross_line_point()->set_x(
      pBase->gap_selector_path_spline_[0]
          .crossed_line_point_info.crossed_line_point.x);
  output.mutable_cross_line_point()->set_y(
      pBase->gap_selector_path_spline_[0]
          .crossed_line_point_info.crossed_line_point.y);
  std::string serialized_message;
  output.SerializeToString(&serialized_message);
  return serialized_message;
}

py::bytes GetVisionInfoBytes(py::bytes &gap_selector_input_bytes) {
  planning::common::GapSelectorInput gap_selector_input =
      BytesToProto<planning::common::GapSelectorInput>(
          gap_selector_input_bytes);

  planning::common::VisionInfo vision_debug_info;

  vision_debug_info.set_last_path_spline_status(
      pBase->path_spline_.path_spline_status);
  vision_debug_info.set_cur_path_spline_status(
      pBase->gap_selector_path_spline_[0].path_spline_status);
  vision_debug_info.set_ego_init_s(pBase->ego_planning_init_s_[0]);
  vision_debug_info.set_ego_init_v(pBase->ego_planning_init_s_[1]);
  vision_debug_info.set_ego_cur_l(pBase->planning_init_point_.frenet_state.r);
  vision_debug_info.set_front_gap_obj_id(pBase->nearby_gap_.front_agent_id);
  vision_debug_info.set_rear_gap_obj_id(pBase->nearby_gap_.rear_agent_id);
  if (pBase->nearby_gap_.front_agent_id > -1) {
    auto iter = pBase->agent_node_mgr_->agent_node_target_lane_map().find(
        pBase->nearby_gap_.front_agent_id);
    vision_debug_info.set_front_gap_obj_cur_s(iter->second.cur_s);
    vision_debug_info.set_front_gap_obj_raw_vel(iter->second.raw_vel);
  }
  if (pBase->nearby_gap_.rear_agent_id > -1) {
    auto iter = pBase->agent_node_mgr_->agent_node_target_lane_map().find(
        pBase->nearby_gap_.rear_agent_id);
    vision_debug_info.set_rear_gap_obj_cur_s(iter->second.cur_s);
    vision_debug_info.set_rear_gap_obj_raw_vel(iter->second.raw_vel);
  }

  vision_debug_info.set_drive_style((int)pBase->gap_drive_style_);
  vision_debug_info.set_gap_selector_status((int)0);

  // reserve
  vision_debug_info.set_refine_lc_time(0.);
  vision_debug_info.set_expected_lc_l(0.);
  vision_debug_info.set_expected_lc_s(0.);
  vision_debug_info.set_lc_v_end(0.);
  vision_debug_info.set_lh_v_end(0.);
  vision_debug_info.set_cur_lc_path_collision_idx(-1);
  vision_debug_info.set_last_lc_path_collision_idx(-1);

  vision_debug_info.mutable_quintic_parames()->set_lc_time(0.);
  vision_debug_info.mutable_quintic_parames()->mutable_x0()->set_x(0.);
  vision_debug_info.mutable_quintic_parames()->mutable_x0()->set_y(0.);
  vision_debug_info.mutable_quintic_parames()->mutable_dx0()->set_x(0.);
  vision_debug_info.mutable_quintic_parames()->mutable_dx0()->set_y(0.);
  vision_debug_info.mutable_quintic_parames()->mutable_ddx0()->set_x(0.);
  vision_debug_info.mutable_quintic_parames()->mutable_ddx0()->set_y(0.);
  vision_debug_info.mutable_quintic_parames()->mutable_xt()->set_x(0.);
  vision_debug_info.mutable_quintic_parames()->mutable_xt()->set_y(0.);
  vision_debug_info.mutable_quintic_parames()->mutable_dxt()->set_x(0.);
  vision_debug_info.mutable_quintic_parames()->mutable_dxt()->set_y(0.);
  vision_debug_info.mutable_quintic_parames()->mutable_ddxt()->set_x(0.);
  vision_debug_info.mutable_quintic_parames()->mutable_ddxt()->set_y(0.);
  vision_debug_info.mutable_quintic_parames()->set_expected_dis(0.);
  vision_debug_info.mutable_quintic_parames()->set_expected_l(0.);

  vision_debug_info.mutable_lc_request_buffer()->Resize(3, -1);
  vision_debug_info.mutable_ego_l_buffer()->Resize(3, -100);

  vision_debug_info.set_request(gap_selector_input.request());
  vision_debug_info.set_origin_lane_id(gap_selector_input.origin_lane_id());
  vision_debug_info.set_target_lane_id(gap_selector_input.target_lane_id());
  vision_debug_info.set_current_lane_id(gap_selector_input.current_lane_id());
  for (size_t i = 0;
       i < pBase->gap_selector_state_machine_info_.lc_request_buffer.size();
       i++) {
    vision_debug_info.mutable_lc_request_buffer()->Set(
        i, gap_selector_input.gap_selector_info().lc_request_buffer(i));
    vision_debug_info.mutable_ego_l_buffer()->Set(
        i, gap_selector_input.gap_selector_info().ego_l_buffer(i));
  }

  std::string serialized_message;
  vision_debug_info.SerializeToString(&serialized_message);
  return serialized_message;
}

PYBIND11_MODULE(gap_selector_py, m) {
  m.doc() = "m";
  m.def("Init", &Init)
      .def("UpdateBytes", &UpdateBytes)
      .def("GetOutputBytes", &GetOutputBytes)
      .def("GetDriveStyle", GetDriveStyle)
      .def("TimeOptimalStateLimit", TimeOptimalStateLimit)
      .def("GetFrontGapAgentInfo", GetFrontGapAgentInfo)
      .def("GetRearGapAgentInfo", GetRearGapAgentInfo)
      .def("GetVisionInfoBytes", GetVisionInfoBytes);
}