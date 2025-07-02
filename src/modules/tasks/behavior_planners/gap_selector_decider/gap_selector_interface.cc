#include "gap_selector_interface.h"

#include <memory>
#include <vector>

#include "config/basic_type.h"
#include "debug_info_log.h"
#include "gap_selector.pb.h"
#include "ilqr_define.h"
#include "planning_context.h"
namespace planning {

void GapSelectorInterface::Store(
    const framework::Session *session,
    const GapSelectorStateMachineInfo &gap_selector_state_machine_info) {
  auto gap_selector_input = DebugInfoManager::GetInstance()
                                .GetDebugInfoPb()
                                ->mutable_gap_selector_input();
  gap_selector_input->Clear();
  auto &ref_path_mgr =
      session->environmental_model().get_reference_path_manager();
  auto &ego_state_mgr = session->environmental_model().get_ego_state_manager();

  Point2D ego_frenet_point;
  if (!ref_path_mgr->get_reference_path_by_current_lane()
           ->get_frenet_coord()
           ->XYToSL(Point2D{ego_state_mgr->ego_pose().x,
                            ego_state_mgr->ego_pose().y},
                    ego_frenet_point)) {
    ILOG_ERROR << "Enmergency error! Ego Pose Cart2SL failed!";
  }

  // -------------------------- lane id
  gap_selector_input->set_origin_lane_id(session->planning_context()
                                             .lane_change_decider_output()
                                             .origin_lane_virtual_id);
  gap_selector_input->set_target_lane_id(session->planning_context()
                                             .lane_change_decider_output()
                                             .target_lane_virtual_id);
  gap_selector_input->set_current_lane_id(session->environmental_model()
                                              .get_virtual_lane_manager()
                                              ->current_lane_virtual_id());
  gap_selector_input->set_ego_l_cur_lane(
      session->environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_current_lane()
          ->get_frenet_ego_state()
          .l());

  //---------------------------- planning init state
  //-----------------------------------------
  gap_selector_input->set_cruise_vel(
      session->environmental_model().get_ego_state_manager()->ego_v_cruise());

  gap_selector_input->mutable_planning_init_state()->set_x(
      ego_state_mgr->ego_pose().x);
  gap_selector_input->mutable_planning_init_state()->set_y(
      ego_state_mgr->ego_pose().y);
  gap_selector_input->mutable_planning_init_state()->set_heading_angle(
      ego_state_mgr->planning_init_point().heading_angle);
  gap_selector_input->mutable_planning_init_state()->set_curvature(
      ego_state_mgr->planning_init_point().curvature);
  gap_selector_input->mutable_planning_init_state()->set_v(
      ego_state_mgr->planning_init_point().v);
  gap_selector_input->mutable_planning_init_state()->set_a(
      ego_state_mgr->planning_init_point().a);
  gap_selector_input->mutable_planning_init_state()
      ->mutable_frenet_state()
      ->set_s(ego_frenet_point.x);
  gap_selector_input->mutable_planning_init_state()
      ->mutable_frenet_state()
      ->set_r(ego_frenet_point.y);

  //---------------------------- reline points
  //-----------------------------------------
  auto virtual_lane_mgr =
      session->environmental_model().get_virtual_lane_manager();

  auto current_refline_points =
      gap_selector_input->mutable_current_refline_points();
  auto origin_lane_s_width = gap_selector_input->mutable_origin_lane_s_width();
  current_refline_points->Clear();
  origin_lane_s_width->Clear();
  const std::shared_ptr<VirtualLane> current_lane =
      virtual_lane_mgr->get_lane_with_virtual_id(
          session->environmental_model()
              .get_virtual_lane_manager()
              ->current_lane_virtual_id());
  const auto &lane_points = current_lane->lane_points();
  const auto &cur_reference_path = current_lane->get_reference_path();
  for (const auto &point : lane_points) {
    planning::common::Point2d *Point = current_refline_points->Add();
    Point->set_x(point.local_point.x);
    Point->set_y(point.local_point.y);
  }

  for (const auto &refpath_point : cur_reference_path->get_points()) {
    planning::common::Point2d *x_width = origin_lane_s_width->Add();
    x_width->set_x(refpath_point.path_point.s());
    x_width->set_y(current_lane->width_by_s(refpath_point.path_point.s()));
  }

  auto origin_refline_points =
      gap_selector_input->mutable_origin_refline_points();
  origin_refline_points->Clear();
  const std::shared_ptr<VirtualLane> origin_lane =
      virtual_lane_mgr->get_lane_with_virtual_id(
          session->planning_context()
              .lane_change_decider_output()
              .origin_lane_virtual_id);
  const auto &origin_lane_points = origin_lane->lane_points();
  for (const auto &point : origin_lane_points) {
    planning::common::Point2d *Point = origin_refline_points->Add();
    Point->set_x(point.local_point.x);
    Point->set_y(point.local_point.y);
  }

  const auto &lane_change_decider_output =
      session->planning_context().lane_change_decider_output();
  const auto state = lane_change_decider_output.curr_state;
  const auto lc_request_direction = lane_change_decider_output.lc_request;
  bool is_LC_LCHANGE =
      ((state == kLaneChangeExecution) || (state == kLaneChangeComplete)) &&
      (lc_request_direction == LEFT_CHANGE);
  bool is_LC_RCHANGE =
      ((state == kLaneChangeExecution) || (state == kLaneChangeComplete)) &&
      (lc_request_direction == RIGHT_CHANGE);
  auto target_state = is_LC_LCHANGE ? 1 : (is_LC_RCHANGE ? 2 : 0);
  gap_selector_input->set_request(target_state);

  const std::shared_ptr<VirtualLane> target_lane =
      target_state == 1
          ? virtual_lane_mgr->get_left_lane()
          : (target_state == 2 ? virtual_lane_mgr->get_right_lane() : nullptr);

  if (target_lane != nullptr) {
    const auto &tar_reference_path = target_lane->get_reference_path();
    auto target_refline_points = DebugInfoManager::GetInstance()
                                     .GetDebugInfoPb()
                                     ->mutable_gap_selector_input()
                                     ->mutable_target_refline_points();
    auto target_lane_s_width = DebugInfoManager::GetInstance()
                                   .GetDebugInfoPb()
                                   ->mutable_gap_selector_input()
                                   ->mutable_target_lane_s_width();
    target_refline_points->Clear();
    target_lane_s_width->Clear();
    for (const auto &point : target_lane->lane_points()) {
      planning::common::Point2d *Point = target_refline_points->Add();
      Point->set_x(point.local_point.x);
      Point->set_y(point.local_point.y);
    }

    for (const auto &refpath_point : tar_reference_path->get_points()) {
      planning::common::Point2d *x_width = target_lane_s_width->Add();
      x_width->set_x(refpath_point.path_point.s());
      x_width->set_y(target_lane->width_by_s(refpath_point.path_point.s()));
    }
  }

  // --------------------- inner state machine -----------------------
  auto state_machine_info = gap_selector_input->mutable_gap_selector_info();
  state_machine_info->set_lane_cross(
      gap_selector_state_machine_info.lane_cross);
  state_machine_info->set_lc_triggered(
      gap_selector_state_machine_info.lc_triggered);
  state_machine_info->set_lb_triggered(
      gap_selector_state_machine_info.lb_triggered);
  state_machine_info->set_lc_in(gap_selector_state_machine_info.lc_in);
  state_machine_info->set_lb_in(gap_selector_state_machine_info.lb_in);
  state_machine_info->set_lc_pass_time(
      gap_selector_state_machine_info.lc_pass_time);
  state_machine_info->set_lc_wait_time(
      gap_selector_state_machine_info.lc_wait_time);
  state_machine_info->set_path_requintic(
      gap_selector_state_machine_info.path_requintic);
  state_machine_info->set_lc_cancel(gap_selector_state_machine_info.lc_cancel);
  state_machine_info->set_gs_skip(gap_selector_state_machine_info.gs_skip);
  int lc_request = is_LC_LCHANGE ? 1 : (is_LC_RCHANGE ? 2 : 0);
  state_machine_info->set_lc_request(lc_request);

  state_machine_info->mutable_lc_request_buffer()->Resize(3, 0);
  for (auto i = 0; i < gap_selector_state_machine_info.lc_request_buffer.size();
       i++) {
    state_machine_info->mutable_lc_request_buffer()->Set(
        i, gap_selector_state_machine_info.lc_request_buffer[i]);
  }

  state_machine_info->mutable_ego_l_buffer()->Resize(3, 0);
  for (auto i = 0; i < gap_selector_state_machine_info.ego_l_buffer.size();
       i++) {
    state_machine_info->mutable_ego_l_buffer()->Set(
        i, gap_selector_state_machine_info.ego_l_buffer[i]);
  }

  // -----------------obstacle map info -----------------------------------
  auto mutable_gs_care_objs = gap_selector_input->mutable_gs_care_objs();
  mutable_gs_care_objs->Clear();

  const auto map_gs_care_obstacles = session->environmental_model()
                                         .get_obstacle_manager()
                                         ->get_gs_care_obstacles()
                                         .Dict();

  for (const auto &obj : map_gs_care_obstacles) {
    planning::common::PredictionObject *pred_object =
        mutable_gs_care_objs->Add();
    pred_object->set_id(obj.first);
    pred_object->set_type(
        planning::common::PredictionObject::
            OBJECT_TYPE_COUPE);  // note all obj is under same consideration!
    pred_object->set_fusion_source(obj.second.fusion_source());
    pred_object->set_timestamp_us(
        0.);  // note all obj is not consider the time delay
    pred_object->set_delay_time(
        0.);  // note all obj is not consider the time delay
    pred_object->set_intention(planning::common::PredictionObject::COMMON);
    pred_object->set_b_backup_freemove(false);
    pred_object->set_cutin_score(0.);
    pred_object->set_position_x(obj.second.x_center());  // cart pos x
    pred_object->set_position_y(obj.second.y_center());  // cart pos y
    pred_object->set_length(obj.second.length());
    pred_object->set_width(obj.second.width());
    pred_object->set_speed(obj.second.velocity());
    pred_object->set_yaw(obj.second.heading_angle());
    pred_object->set_theta(obj.second.velocity_angle());

    pred_object->set_acc(obj.second.acceleration());
    pred_object->set_relative_position_x(obj.second.x_relative_center());
    pred_object->set_relative_position_y(
        obj.second.y_relative_center());  // cart pos x
    pred_object->set_relative_speed_x(
        obj.second.x_relative_velocity());  // cart pos y
    pred_object->set_relative_speed_y(obj.second.y_relative_velocity());
    pred_object->set_relative_acceleration_x(0.);            // reserve
    pred_object->set_relative_acceleration_y(0.);            // reserve
    pred_object->set_acceleration_relative_to_ground_x(0.);  // reserve
    pred_object->set_acceleration_relative_to_ground_y(0.);  // reserve
    pred_object->set_relative_theta(obj.second.relative_heading_angle());
  }

  auto origin_lane_objs =
      gap_selector_input->mutable_origin_lane_obstacles_ids();
//   const auto map_origin_lane_obstacles =
//       session->environmental_model()
//           .get_reference_path_manager()
//           ->get_reference_path_by_lane(session->environmental_model()
//                                            .get_virtual_lane_manager()
//                                            ->get_current_lane()
//                                            ->get_virtual_id(),
//                                        false)
//           ->mutable_obstacles_in_lane_map();
//   origin_lane_objs->Resize(map_origin_lane_obstacles.size(), -1);
//   for (auto i = 0; i < map_origin_lane_obstacles.size(); i++) {
//     origin_lane_objs->Set(i, map_origin_lane_obstacles[i]);
//   }

  if (target_lane != nullptr) {
    auto target_lane_objs =
        gap_selector_input->mutable_target_lane_obstacle_ids();

    // const auto map_target_lane_obstacles =
    //     session->environmental_model()
    //         .get_reference_path_manager()
    //         ->get_reference_path_by_lane(target_lane->get_virtual_id(), false)
    //         ->mutable_obstacles_in_lane_map();

    // target_lane_objs->Resize(map_target_lane_obstacles.size(), -1);
    // for (auto i = 0; i < map_target_lane_obstacles.size(); i++) {
    //   target_lane_objs->Set(i, map_target_lane_obstacles[i]);
    // }
  }
  return;
}

void GapSelectorInterface::Store(
    const GapSelectorPathSpline &gap_selector_path_spline,
    const TrajectoryPoints &traj_points) {
  // path spline

  auto gap_selector_input = DebugInfoManager::GetInstance()
                                .GetDebugInfoPb()
                                ->mutable_gap_selector_input();

  auto path_spline_info = gap_selector_input->mutable_gap_selector_info()
                              ->mutable_last_gap_selector_path_spline();
  path_spline_info->Clear();
  path_spline_info->mutable_start_state()->set_p(
      gap_selector_path_spline.start_state.p);
  path_spline_info->mutable_start_state()->set_v(
      gap_selector_path_spline.start_state.v);
  path_spline_info->mutable_start_state()->set_a(
      gap_selector_path_spline.start_state.a);
  path_spline_info->mutable_start_state()->set_j(
      gap_selector_path_spline.start_state.j);
  path_spline_info->mutable_start_state()->set_t(
      gap_selector_path_spline.start_state.t);

  path_spline_info->mutable_start_cart_point()->set_x(
      gap_selector_path_spline.start_cart_point.x);
  path_spline_info->mutable_start_cart_point()->set_y(
      gap_selector_path_spline.start_cart_point.y);

  path_spline_info->set_path_spline_status(
      (int)(gap_selector_path_spline.path_spline_status));
  const auto &x_vec = gap_selector_path_spline.x_s_spline.get_y();
  const auto &y_vec = gap_selector_path_spline.y_s_spline.get_y();
  const auto &s_vec = gap_selector_path_spline.x_s_spline.get_x();
  assert(x_vec.size() == y_vec.size());

  path_spline_info->mutable_s_vector_spline()->Resize(y_vec.size(), 0.);
  path_spline_info->mutable_x_vector_spline()->Resize(y_vec.size(), 0.);
  path_spline_info->mutable_y_vector_spline()->Resize(y_vec.size(), 0.);
  for (auto i = 0; i < x_vec.size(); i++) {
    path_spline_info->mutable_s_vector_spline()->Set(i, s_vec[i]);
    path_spline_info->mutable_y_vector_spline()->Set(i, y_vec[i]);
    path_spline_info->mutable_x_vector_spline()->Set(i, x_vec[i]);
  }

  // traj points
  auto mutable_traj_points = gap_selector_input->mutable_trajectory_points();
  mutable_traj_points->Clear();
  for (auto i = 0; i < traj_points.size(); i++) {
    planning::common::TrajectoryPoint *traj_point = mutable_traj_points->Add();
    traj_point->set_x(traj_points[i].x);
    traj_point->set_y(traj_points[i].y);
    traj_point->set_s(traj_points[i].s);
    traj_point->set_l(traj_points[i].l);
    traj_point->set_a(traj_points[i].a);
    traj_point->set_t(traj_points[i].t);
    traj_point->set_v(traj_points[i].v);
    traj_point->set_curvature(traj_points[i].curvature);
    traj_point->set_heading_angle(traj_points[i].heading_angle);
    traj_point->set_frenet_valid(traj_points[i].frenet_valid);
  }

  auto n = mutable_traj_points->size();
  std::cout << "\n n is: " << n << std::endl;
  return;
}

void GapSelectorInterface::Parse(planning::common::GapSelectorInput &input) {
  // planning init point
  gap_selector_feed_info_.ego_cart_point[0] = input.planning_init_state().x();
  gap_selector_feed_info_.ego_cart_point[1] = input.planning_init_state().y();

  gap_selector_feed_info_.ego_init_s[0] =
      input.planning_init_state().frenet_state().s();
  gap_selector_feed_info_.ego_init_s[1] = input.planning_init_state().v();
  gap_selector_feed_info_.ego_init_s[2] = input.planning_init_state().a();

  gap_selector_feed_info_.planning_init_point_.x =
      input.planning_init_state().x();
  gap_selector_feed_info_.planning_init_point_.y =
      input.planning_init_state().y();
  gap_selector_feed_info_.planning_init_point_.a =
      input.planning_init_state().a();
  gap_selector_feed_info_.planning_init_point_.frenet_state.s =
      input.planning_init_state().frenet_state().s();
  gap_selector_feed_info_.planning_init_point_.frenet_state.r =
      input.planning_init_state().frenet_state().r();
  gap_selector_feed_info_.planning_init_point_.curvature =
      input.planning_init_state().curvature();
  gap_selector_feed_info_.planning_init_point_.v =
      input.planning_init_state().v();
  gap_selector_feed_info_.planning_init_point_.heading_angle =
      input.planning_init_state().heading_angle();

  gap_selector_feed_info_.cruise_vel = input.cruise_vel();

  gap_selector_feed_info_.origin_lane_id = input.origin_lane_id();
  gap_selector_feed_info_.target_lane_id = input.target_lane_id();
  gap_selector_feed_info_.current_lane_id = input.current_lane_id();

  // parse refline
  auto &origin_refline_points = gap_selector_feed_info_.origin_refline_points;
  origin_refline_points.clear();
  origin_refline_points.resize(input.origin_refline_points_size());

  for (auto i = 0; i < input.origin_refline_points_size(); i++) {
    origin_refline_points[i] = std::make_pair<double, double>(
        input.origin_refline_points(i).x(), input.origin_refline_points(i).y());
  }

  auto &target_refline_points = gap_selector_feed_info_.target_refline_points;
  target_refline_points.clear();
  target_refline_points.resize(input.target_refline_points_size());

  for (auto i = 0; i < input.target_refline_points_size(); i++) {
    target_refline_points[i] = std::make_pair<double, double>(
        input.target_refline_points(i).x(), input.target_refline_points(i).y());
  }

  auto &current_refline_points = gap_selector_feed_info_.current_refline_points;
  current_refline_points.clear();
  current_refline_points.resize(input.current_refline_points_size());
  for (auto i = 0; i < input.current_refline_points_size(); i++) {
    current_refline_points[i] =
        std::make_pair<double, double>(input.current_refline_points(i).x(),
                                       input.current_refline_points(i).y());
  }

  auto &origin_lane_s_width_vec =
      gap_selector_feed_info_.origin_lane_s_width_vec;
  auto &target_lane_s_width_vec =
      gap_selector_feed_info_.target_lane_s_width_vec;
  origin_lane_s_width_vec.clear();
  target_lane_s_width_vec.clear();
  origin_lane_s_width_vec.resize(input.origin_lane_s_width_size());
  target_lane_s_width_vec.resize(input.target_lane_s_width_size());

  for (auto i = 0; i < input.origin_lane_s_width_size(); i++) {
    origin_lane_s_width_vec[i] = std::make_pair<double, double>(
        input.origin_lane_s_width(i).x(), input.origin_lane_s_width(i).y());
  }
  for (auto i = 0; i < input.target_lane_s_width_size(); i++) {
    target_lane_s_width_vec[i] = std::make_pair<double, double>(
        input.target_lane_s_width(i).x(), input.target_lane_s_width(i).y());
  }

  // gap selector info

  auto &gap_selector_info = gap_selector_feed_info_.gap_selector_info;
  gap_selector_info.lane_cross = input.gap_selector_info().lane_cross();
  gap_selector_info.lc_triggered = input.gap_selector_info().lc_triggered();
  gap_selector_info.lb_triggered = input.gap_selector_info().lb_triggered();
  gap_selector_info.lc_in = input.gap_selector_info().lc_in();
  gap_selector_info.lb_in = input.gap_selector_info().lb_in();
  gap_selector_info.lc_pass_time = input.gap_selector_info().lc_pass_time();
  gap_selector_info.lc_wait_time = input.gap_selector_info().lc_wait_time();
  gap_selector_info.path_requintic = input.gap_selector_info().path_requintic();
  gap_selector_info.gs_skip = input.gap_selector_info().gs_skip();
  gap_selector_info.lc_request = input.gap_selector_info().lc_request();

  for (auto i = 0; i < input.gap_selector_info().lc_request_buffer_size();
       i++) {
    gap_selector_feed_info_.lc_request_buffer[i] =
        input.gap_selector_info().lc_request_buffer(i);
  }
  for (auto i = 0; i < input.gap_selector_info().ego_l_buffer_size(); i++) {
    gap_selector_feed_info_.ego_l_buffer[i] =
        input.gap_selector_info().ego_l_buffer(i);
  }
  // last gap spline info
  auto &spline_info = gap_selector_info.last_gap_selector_path_spline;
  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> s_vec;
  x_vec.resize(input.gap_selector_info()
                   .last_gap_selector_path_spline()
                   .x_vector_spline_size());
  y_vec.resize(input.gap_selector_info()
                   .last_gap_selector_path_spline()
                   .y_vector_spline_size());
  s_vec.resize(input.gap_selector_info()
                   .last_gap_selector_path_spline()
                   .s_vector_spline_size());
  for (auto i = 0; i < input.gap_selector_info()
                           .last_gap_selector_path_spline()
                           .x_vector_spline_size();
       i++) {
    x_vec[i] = input.gap_selector_info()
                   .last_gap_selector_path_spline()
                   .x_vector_spline(i);
    y_vec[i] = input.gap_selector_info()
                   .last_gap_selector_path_spline()
                   .y_vector_spline(i);
    s_vec[i] = input.gap_selector_info()
                   .last_gap_selector_path_spline()
                   .s_vector_spline(i);
  }

  spline_info.start_state.a = input.gap_selector_info()
                                  .last_gap_selector_path_spline()
                                  .start_state()
                                  .a();
  spline_info.start_state.p = input.gap_selector_info()
                                  .last_gap_selector_path_spline()
                                  .start_state()
                                  .p();
  spline_info.start_state.v = input.gap_selector_info()
                                  .last_gap_selector_path_spline()
                                  .start_state()
                                  .v();
  spline_info.start_state.j = input.gap_selector_info()
                                  .last_gap_selector_path_spline()
                                  .start_state()
                                  .j();
  spline_info.start_state.t = input.gap_selector_info()
                                  .last_gap_selector_path_spline()
                                  .start_state()
                                  .t();

  spline_info.start_cart_point.x = input.gap_selector_info()
                                       .last_gap_selector_path_spline()
                                       .start_cart_point()
                                       .x();
  spline_info.start_cart_point.y = input.gap_selector_info()
                                       .last_gap_selector_path_spline()
                                       .start_cart_point()
                                       .y();
  spline_info.path_spline_status = input.gap_selector_info()
                                       .last_gap_selector_path_spline()
                                       .path_spline_status();
  if (((input.gap_selector_info()
            .last_gap_selector_path_spline()
            .path_spline_status() == 2) ||
       (input.gap_selector_info()
            .last_gap_selector_path_spline()
            .path_spline_status() == 3)) &&
      s_vec.size() > 3) {  // this hack for spline s_vec, x_vec size problems
    spline_info.x_s_spline.set_points(s_vec, x_vec);
    spline_info.y_s_spline.set_points(s_vec, y_vec);
  }

  // parse tarj points

  auto &traj_points = gap_selector_feed_info_.traj_points;
  traj_points.clear();
  traj_points.resize(input.mutable_trajectory_points()->size());
  std::cout << "input traj points size"
            << input.mutable_trajectory_points()->size() << std::endl;
  for (auto i = 0; i < input.mutable_trajectory_points()->size(); i++) {
    // std::cout << "input x:" << input.trajectory_points().Get(i).x()
    //           << std::endl;
    traj_points[i].x = input.trajectory_points().Get(i).x();
    traj_points[i].y = input.trajectory_points().Get(i).y();
    traj_points[i].a = input.trajectory_points().Get(i).a();
    traj_points[i].s = input.trajectory_points().Get(i).s();
    traj_points[i].l = input.trajectory_points().Get(i).l();
    traj_points[i].heading_angle =
        input.trajectory_points().Get(i).heading_angle();
    traj_points[i].curvature = input.trajectory_points().Get(i).curvature();
    traj_points[i].t = input.trajectory_points().Get(i).t();
    traj_points[i].frenet_valid =
        input.trajectory_points().Get(i).frenet_valid();
    // std::cout << "size i:\n " << i << std::endl;
    //  print("size i: \n", )
  }

  // parse obstacle map info
  gap_selector_feed_info_.map_gs_care_obstacles.clear();
  gap_selector_feed_info_.map_origin_lane_obstacles.clear();
  gap_selector_feed_info_.map_target_lane_obstacles.clear();
  for (auto i = 0; i < input.gs_care_objs().size(); i++) {
    PredictionObject obstacle_info;
    obstacle_info.position_x = input.gs_care_objs(i).position_x();
    obstacle_info.position_y = input.gs_care_objs(i).position_y();
    obstacle_info.relative_position_x =
        input.gs_care_objs(i).relative_position_x();
    obstacle_info.relative_position_y =
        input.gs_care_objs(i).relative_position_y();
    obstacle_info.relative_theta = input.gs_care_objs(i).relative_theta();
    obstacle_info.theta = input.gs_care_objs(i).theta();
    obstacle_info.length = input.gs_care_objs(i).length();
    obstacle_info.width = input.gs_care_objs(i).width();
    obstacle_info.id = input.gs_care_objs(i).id();
    obstacle_info.type = iflyauto::ObjectType::OBJECT_TYPE_COUPE;
    obstacle_info.speed = input.gs_care_objs(i).speed();
    obstacle_info.yaw = input.gs_care_objs(i).yaw();
    obstacle_info.acc = input.gs_care_objs(i).acc();
    obstacle_info.relative_theta = input.gs_care_objs(i).relative_theta();
    obstacle_info.relative_acceleration_x = 0.;
    obstacle_info.relative_acceleration_y = 0.;
    obstacle_info.relative_speed_x = input.gs_care_objs(i).relative_speed_x();
    obstacle_info.relative_speed_y = input.gs_care_objs(i).relative_speed_y();
    obstacle_info.relative_acceleration_x = 0.;
    obstacle_info.relative_acceleration_y = 0.;
    obstacle_info.acceleration_relative_to_ground_x = 0.;
    obstacle_info.acceleration_relative_to_ground_y = 0.;
    obstacle_info.b_backup_freemove = false;

    Obstacle obj(obstacle_info.id, obstacle_info, false, 0.);
    gap_selector_feed_info_.map_gs_care_obstacles.insert(
        std::make_pair(obstacle_info.id, obj));
  }

  for (auto i = 0; i < input.target_lane_obstacle_ids_size(); i++) {
    gap_selector_feed_info_.map_target_lane_obstacles.emplace_back(
        input.target_lane_obstacle_ids(i));
  }

  for (auto i = 0; i < input.origin_lane_obstacles_ids_size(); i++) {
    gap_selector_feed_info_.map_origin_lane_obstacles.emplace_back(
        input.origin_lane_obstacles_ids(i));
  }
  return;
}

void GapSelectorInterface::ReplayCollect(
    const TrajectoryPoints &traj_points,
    const GapSelectorStateMachineInfo &gap_selector_state_machine_info,
    const GapSelectorPathSpline &gap_selector_path_spline,
    const std::vector<Gap> &gap_list, const Gap &nearby_gap,
    const std::vector<std::pair<STPoint, STPoint>> &front_gap_car_st_boundaries,
    const std::vector<std::pair<STPoint, STPoint>> &rear_gap_car_st_boundaries,
    const std::vector<std::pair<STPoint, STPoint>>
        &current_lane_front_car_st_boundaries,
    const int ego_current_lane_id, const std::vector<STPoint> &st_time_optimal,
    const GapSelectorStatus gap_status, const double front_car_dynamic_dis,
    const double rear_car_dynamic_dis, const double ego_lane_car_dynamic_dis) {
  auto gap_selector_replay_info = DebugInfoManager::GetInstance()
                                      .GetDebugInfoPb()
                                      ->mutable_gap_selector_replay_info();
  gap_selector_replay_info->Clear();
  gap_selector_replay_info->set_origin_lane_id(
      gap_selector_state_machine_info.origin_lane_id);
  gap_selector_replay_info->set_current_lane_id(
      gap_selector_state_machine_info.current_lane_id);
  gap_selector_replay_info->set_target_lane_id(
      gap_selector_state_machine_info.target_lane_id);
  gap_selector_replay_info->set_origin_lane_id(
      gap_selector_state_machine_info.origin_lane_id);

  auto gap_selector_path_spline_info =
      gap_selector_replay_info->mutable_gap_selector_path_spline();

  if (!gap_selector_path_spline.x_s_spline.get_x().empty()) {
    const auto &x_vec = gap_selector_path_spline.x_s_spline.get_y();
    const auto &y_vec = gap_selector_path_spline.y_s_spline.get_y();
    const auto &s_vec = gap_selector_path_spline.x_s_spline.get_x();
    assert(x_vec.size() == y_vec.size());

    gap_selector_path_spline_info->mutable_s_vector_spline()->Resize(
        y_vec.size(), 0.);
    gap_selector_path_spline_info->mutable_x_vector_spline()->Resize(
        y_vec.size(), 0.);
    gap_selector_path_spline_info->mutable_y_vector_spline()->Resize(
        y_vec.size(), 0.);
    for (auto i = 0; i < x_vec.size(); i++) {
      gap_selector_path_spline_info->mutable_s_vector_spline()->Set(i,
                                                                    s_vec[i]);
      gap_selector_path_spline_info->mutable_y_vector_spline()->Set(i,
                                                                    y_vec[i]);
      gap_selector_path_spline_info->mutable_x_vector_spline()->Set(i,
                                                                    x_vec[i]);
    }

    gap_selector_path_spline_info->mutable_start_cart_point()->set_x(
        gap_selector_path_spline.start_cart_point.x);
    gap_selector_path_spline_info->mutable_start_cart_point()->set_y(
        gap_selector_path_spline.start_cart_point.y);

    gap_selector_path_spline_info->set_path_spline_status(
        (int)(gap_selector_path_spline.path_spline_status));

    gap_selector_replay_info->mutable_quintic_p0()->set_x(
        gap_selector_path_spline.quintic_p0.x);
    gap_selector_replay_info->mutable_quintic_p0()->set_y(
        gap_selector_path_spline.quintic_p0.y);
    gap_selector_replay_info->mutable_quintic_pe()->set_x(
        gap_selector_path_spline.quintic_pe.x);
    gap_selector_replay_info->mutable_quintic_pe()->set_y(
        gap_selector_path_spline.quintic_pe.y);
    gap_selector_replay_info->mutable_quintic_stitched_p()->set_x(
        gap_selector_path_spline.stitched_p.x);
    gap_selector_replay_info->mutable_quintic_stitched_p()->set_y(
        gap_selector_path_spline.stitched_p.y);
  }

  // traj points
  auto gs_traj_points = gap_selector_replay_info->mutable_gs_traj_points();
  for (auto i = 0; i < traj_points.size(); i++) {
    planning::common::TrajectoryPoint *traj_point = gs_traj_points->Add();
    traj_point->set_x(traj_points[i].x);
    traj_point->set_y(traj_points[i].y);
    traj_point->set_s(traj_points[i].s);
    traj_point->set_l(traj_points[i].l);
    traj_point->set_a(traj_points[i].a);
    traj_point->set_t(traj_points[i].t);
    traj_point->set_v(traj_points[i].v);
    traj_point->set_curvature(traj_points[i].curvature);
    traj_point->set_heading_angle(traj_points[i].heading_angle);
    traj_point->set_frenet_valid(traj_points[i].frenet_valid);
  }

  gap_selector_replay_info->set_lane_cross(
      gap_selector_state_machine_info.lane_cross);
  gap_selector_replay_info->set_lc_triggered(
      gap_selector_state_machine_info.lc_triggered);
  gap_selector_replay_info->set_lb_triggered(
      gap_selector_state_machine_info.lb_triggered);
  gap_selector_replay_info->set_lc_in(gap_selector_state_machine_info.lc_in);
  gap_selector_replay_info->set_lb_in(gap_selector_state_machine_info.lb_in);
  gap_selector_replay_info->set_gs_skip(
      gap_selector_state_machine_info.gs_skip);
  gap_selector_replay_info->set_lc_cancel(
      gap_selector_state_machine_info.lc_cancel);
  gap_selector_replay_info->set_lc_request(
      gap_selector_state_machine_info.lc_request_buffer[2]);
  gap_selector_replay_info->set_lc_pass_time(
      gap_selector_state_machine_info.lc_pass_time);
  gap_selector_replay_info->set_lc_wait_time(
      gap_selector_state_machine_info.lc_wait_time);

  gap_selector_replay_info->mutable_lc_request_buffer()->Resize(3, 0);
  for (auto i = 0; i < gap_selector_state_machine_info.lc_request_buffer.size();
       i++) {
    gap_selector_replay_info->mutable_lc_request_buffer()->Set(
        i, gap_selector_state_machine_info.lc_request_buffer[i]);
  }

  gap_selector_replay_info->mutable_ego_l_buffer()->Resize(3, 0);
  for (auto i = 0; i < gap_selector_state_machine_info.ego_l_buffer.size();
       i++) {
    gap_selector_replay_info->mutable_ego_l_buffer()->Set(
        i, gap_selector_state_machine_info.ego_l_buffer[i]);
  }

  gap_selector_replay_info->set_path_requintic(
      gap_selector_state_machine_info.path_requintic);

  gap_selector_replay_info->mutable_cross_line_point_global()->set_x(
      gap_selector_path_spline.crossed_line_point_info.crossed_line_point.x);
  gap_selector_replay_info->mutable_cross_line_point_global()->set_y(
      gap_selector_path_spline.crossed_line_point_info.crossed_line_point.y);

  // gap info
  gap_selector_replay_info->mutable_nearby_gap()->set_front_agent_id(
      nearby_gap.front_agent_id);
  gap_selector_replay_info->mutable_nearby_gap()->set_rear_agent_id(
      nearby_gap.rear_agent_id);
  //   gap_selector_replay_info->set_current_front_agent_id(ego_current_lane_id);
  //   gap_selector_replay_info->set_front_car_dynamic_dis(front_car_dynamic_dis);
  //   gap_selector_replay_info->set_rear_car_dynamic_dis(rear_car_dynamic_dis);
  //   gap_selector_replay_info->set_ego_lane_car_dynamic_dis(
  //       ego_lane_car_dynamic_dis);
  // front gap pred objs
  for (auto i = 0; i < front_gap_car_st_boundaries.size(); i++) {
    planning::common::ObstaclePredicatedPoint *obj_pred_point =
        gap_selector_replay_info
            ->mutable_obstacle_predicate_points_front_gap_car()
            ->Add();
    // obj_pred_point->set_x(front_nearby_car_pred_points.obstacle_pred_info[i].x);
    // obj_pred_point->set_y(front_nearby_car_pred_points.obstacle_pred_info[i].y);
    obj_pred_point->set_s(front_gap_car_st_boundaries[i].first.s());
    obj_pred_point->set_t(front_gap_car_st_boundaries[i].first.t());
  }
  // rear gap pred objs
  for (auto i = 0; i < rear_gap_car_st_boundaries.size(); i++) {
    planning::common::ObstaclePredicatedPoint *obj_pred_point =
        gap_selector_replay_info
            ->mutable_obstacle_predicate_points_rear_gap_car()
            ->Add();
    obj_pred_point->set_s(rear_gap_car_st_boundaries[i].second.s());
    obj_pred_point->set_t(rear_gap_car_st_boundaries[i].second.t());
  }

  // front car pred objs
  //   for (auto i = 0; i < current_lane_front_car_st_boundaries.size(); i++) {
  //     planning::common::ObstaclePredicatedPoint *obj_pred_point =
  //         gap_selector_replay_info
  //             ->mutable_obstacle_predicate_points_current_lane_car()
  //             ->Add();
  //     obj_pred_point->set_s(current_lane_front_car_st_boundaries[i].second.s());
  //     obj_pred_point->set_t(current_lane_front_car_st_boundaries[i].second.t());
  //   }

  // gap list

  // ego time optimal
  for (auto i = 0; i < st_time_optimal.size(); i++) {
    planning::common::ObstaclePredicatedPoint *ego_st_optimal_point =
        gap_selector_replay_info->mutable_ego_time_optimal()->Add();
    ego_st_optimal_point->set_s(st_time_optimal[i].s());
    ego_st_optimal_point->set_t(st_time_optimal[i].t());
  }

  // set gap selector status
  gap_selector_replay_info->set_gap_selector_status((int)gap_status);
  return;
}
}  // namespace planning
