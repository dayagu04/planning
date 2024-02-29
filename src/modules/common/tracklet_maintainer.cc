#include <cstddef>
#include <memory>
#include <vector>

#include "path_point.h"
#include "refline.h"
#include "utils/path_point.h"
#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "../../common/planning_gflags.h"
#include "debug_info_log.h"
#include "environment_model_debug_info.pb.h"
#include "environmental_model.h"
#include "ifly_time.h"
#include "path_point.h"
#include "planning_context.h"
#include "planning_output_context.h"
#include "tracklet_maintainer.h"
#include "virtual_lane_manager.h"
namespace planning {
TrackletSequentialState *LifecycleDict::get(int uid) {
  auto iter = data_dict_.find(uid);
  if (iter != data_dict_.end()) {
    return &(iter->second);
  } else {
    // if not exist, create
    TrackletSequentialState state;
    data_dict_.insert(std::make_pair(uid, state));
  }

  return &(iter->second);
}

bool LifecycleDict::set(int uid, const TrackletSequentialState &state) {
  auto iter = data_dict_.find(uid);
  if (iter == data_dict_.end()) {
    data_dict_.insert(std::make_pair(uid, state));
  } else {
    iter->second = state;
  }

  return false;
}

void LifecycleDict::mark_dirty(int uid) { dirty_set_.insert(uid); }

void LifecycleDict::remove_clean() {
  if (dirty_set_.empty()) {
    data_dict_.clear();
    return;
  }

  std::map<int, TrackletSequentialState> new_dict;
  for (auto id : dirty_set_) {
    auto iter = data_dict_.find(id);
    if (iter != data_dict_.end()) {
      new_dict.insert(std::make_pair(id, iter->second));
    }
  }

  data_dict_ = new_dict;
  dirty_set_.clear();
}

TrackletMaintainer::TrackletMaintainer(planning::framework::Session *session) {
  session_ = session;
  s_ego_ = 0;
  l_ego_ = 0;
  theta_ego_ = 0;
  vl_ego_ = 0;
  vs_ego_ = 0;
}

TrackletMaintainer::~TrackletMaintainer() {
  for (auto iter = object_map_.begin(); iter != object_map_.end(); ++iter) {
    delete iter->second;
  }

  object_map_.clear();
}

void TrackletMaintainer::apply_update(
    const std::shared_ptr<EgoStateManager> ego_state,
    const std::vector<PredictionObject> &predictions,
    std::vector<TrackedObject> &tracked_objects, LeadCars &lead_cars,
    bool isRedLightStop, bool hdmap_valid) {
  hdmap_valid_ = hdmap_valid;
  ego_state_ = ego_state;
  LeadCars leadcars;
  std::vector<TrackedObject *> objects;
  std::vector<PathPoint> path_points;
  frenet_coord_ = nullptr;
  bool is_location_valid = session_->environmental_model().location_valid();
  auto &lateral_output =
      session_->planning_context().lateral_behavior_planner_output();

  auto flane = session_->environmental_model()
                   .get_virtual_lane_manager()
                   ->get_last_fix_lane();
  if (is_location_valid) {
    recv_prediction_objects(predictions, objects);
    if (flane != nullptr && flane->get_reference_path() != nullptr) {
      // FrenetCoordinateSystemParameters frenet_parameters;
      // init frenet parameters
      // frenet_parameters.zero_speed_threshold = 0.1;
      // frenet_parameters.coord_transform_precision = 0.01;
      // frenet_parameters.step_s = 0.3;
      // frenet_parameters.coarse_step_s = 1.0;
      // frenet_parameters.optimization_gamma = 0.5;
      // frenet_parameters.max_iter = 15;
      auto &ref_path = flane->get_reference_path();
      // std::vector<Point2D> coord_points;
      // for (auto ref_point : ref_path->get_points()) {
      //   double ego_fx = std::cos(ego_state_.ego_pose_raw().theta);
      //   double ego_fy = std::sin(ego_state_.ego_pose_raw().theta);
      //   double ego_lx = -ego_fy;
      //   double ego_ly = ego_fx;
      //   double dx = ref_point.path_point.x - ego_state_.ego_pose_raw().x;
      //   double dy = ref_point.path_point.y - ego_state_.ego_pose_raw().y;

      //   ref_point.path_point.x = dx * ego_fx + dy * ego_fy;
      //   ref_point.path_point.y = dx * ego_lx + dy * ego_ly;
      //   path_points.emplace_back(ref_point.path_point);
      //   coord_points.emplace_back(
      //       Point2D(ref_point.path_point.x, ref_point.path_point.y));
      // }
      // frenet_coord_ = std::make_shared<FrenetCoordinateSystem>(
      //     coord_points, frenet_parameters);

      std::vector<planning_math::PathPoint> coord_points;
      for (auto ref_point : ref_path->get_points()) {
        double ego_fx = std::cos(ego_state_->ego_pose_raw().theta);
        double ego_fy = std::sin(ego_state_->ego_pose_raw().theta);
        double ego_lx = -ego_fy;
        double ego_ly = ego_fx;
        double dx = ref_point.path_point.x - ego_state_->ego_pose_raw().x;
        double dy = ref_point.path_point.y - ego_state_->ego_pose_raw().y;

        ref_point.path_point.x = dx * ego_fx + dy * ego_fy;
        ref_point.path_point.y = dx * ego_lx + dy * ego_ly;
        planning_math::PathPoint path_point{ref_point.path_point.x,
                                            ref_point.path_point.y};
        coord_points.emplace_back(path_point);
      }
      frenet_coord_ = std::make_shared<KDPath>(std::move(coord_points));
    }
  } else {
    recv_relative_prediction_objects(predictions, objects);
    if (flane != nullptr && flane->get_reference_path() != nullptr) {
      frenet_coord_ = flane->get_reference_path()->get_frenet_coord();
    }
  }

  // 长时暂时无lateral_output
  calc(objects, lateral_output.scenario, lateral_output.flane_width,
       lateral_output.lat_offset, lateral_output.borrow_bicycle_lane,
       lateral_output.enable_intersection_planner, lateral_output.dist_rblane,
       lateral_output.tleft_lane, lateral_output.rightest_lane,
       lateral_output.dist_intersect, lateral_output.intersect_length,
       lateral_output.left_faster, lateral_output.right_faster, leadcars,
       isRedLightStop, lateral_output.isFasterStaticAvd,
       lateral_output.isOnHighway, lateral_output.d_poly,
       lateral_output.c_poly);

  set_default_value(objects);

  tracked_objects.resize(objects.size());
  for (size_t i = 0; i < objects.size(); i++) {
    tracked_objects[i] = *objects[i];
    objects[i]->last_recv_time = objects[i]->timestamp;
  }

  lead_cars = leadcars;
}

void TrackletMaintainer::recv_prediction_objects(
    const std::vector<PredictionObject> &predictions,
    std::vector<TrackedObject *> &objects) {
  double pi = std::atan(1.0) * 4;
  std::map<int, TrackedObject *> new_map;

  double ego_fx = std::cos(ego_state_->ego_pose_raw().theta);
  double ego_fy = std::sin(ego_state_->ego_pose_raw().theta);
  double ego_lx = -ego_fy;
  double ego_ly = ego_fx;

  for (auto &p : predictions) {
    if (p.type == 0) {
      continue;
    }

    // WB HACK： Only use front obstacles in long time planning
    if (p.type == 0 ||
        (!(p.fusion_source & OBSTACLE_SOURCE_CAMERA)) &&
            (p.relative_position_x > 0 &&
             tan(25) > fabs(p.relative_position_y / p.relative_position_x)) ||
        fabs(p.relative_position_y) > 10 || p.length == 0 || p.width == 0) {
      LOG_DEBUG("[obstacle_prediction_update] ignore obstacle! : [%d] \n",
                p.id);
      continue;
    }
    double dx = p.position_x - ego_state_->ego_pose_raw().x;
    double dy = p.position_y - ego_state_->ego_pose_raw().y;

    double rel_x = dx * ego_fx + dy * ego_fy;
    double rel_y = dx * ego_lx + dy * ego_ly;

    if (rel_x < -50 || rel_x > 150 || p.trajectory_array.size() == 0) {
      continue;
    }

    TrackedObject *origin = nullptr;

    auto iter = object_map_.find(p.id);

    if (iter != object_map_.end()) {
      origin = iter->second;
      object_map_.erase(iter);
      new_map.insert(std::make_pair(origin->track_id, origin));
    } else {
      origin = new TrackedObject();
      origin->track_id = p.id;
      new_map.insert(std::make_pair(origin->track_id, origin));
    }

    origin->timestamp = p.timestamp_us / 1000000.0;
    origin->type = p.type;
    origin->fusion_source = p.fusion_source;
    origin->fusion_type = 2;

    double speed_yaw =
        p.trajectory_array[0].trajectory[0].yaw;  // 现在yaw这个字段为姿态角

    double abs_vx = p.speed * std::cos(speed_yaw);
    double abs_vy = p.speed * std::sin(speed_yaw);
    double rot_vx = abs_vx * ego_fx + abs_vy * ego_fy;

    double abs_ax = p.acc * std::cos(speed_yaw);
    double abs_ay = p.acc * std::sin(speed_yaw);
    double rot_ax = abs_ax * ego_fx + abs_ay * ego_fy;

    double theta = p.yaw - ego_state_->ego_pose_raw().theta;
    if (theta > pi) {
      theta -= 2 * pi;
    } else if (theta < -pi) {
      theta += 2 * pi;
    }

    origin->position_type = (rel_x > 2.0) ? 0 : 1;

    origin->length = p.length;
    origin->width = p.width;

    origin->center_x = rel_x;
    origin->center_y = rel_y;
    origin->theta = theta;
    origin->speed_yaw = speed_yaw;  // p.trajectory_array[0].trajectory[0].theta
    origin->y_rel_ori = rel_y;

    origin->a = p.acc;
    origin->v = p.speed;
    origin->v_lead = rot_vx;
    origin->v_lead_k = rot_vx;
    origin->v_lead_raw = rot_vx;

    origin->a_lead = rot_ax;
    origin->a_lead_k = rot_ax;

    origin->oncoming = (origin->v_lead < -3.9);

    // calculate fisheye related for cutin
    fisheye_helper(p, *origin);

    int idx = 0;
    for (auto &tr : p.trajectory_array) {
      if (tr.trajectory.empty()) {
        idx++;
        continue;
      }
      TrackedObject *object = nullptr;
      int track_id = hash_prediction_id(origin->track_id, idx);

      iter = object_map_.find(track_id);
      if (iter != object_map_.end()) {
        object = iter->second;
        object_map_.erase(iter);

        if (object != origin) {
          *object = *origin;
          object->track_id = track_id;
        }
        new_map.insert(std::make_pair(track_id, object));
      } else {
        iter = new_map.find(track_id);
        if (iter != new_map.end()) {
          object = iter->second;

          if (object != origin) {
            *object = *origin;
            object->track_id = track_id;
          }
        } else {
          object = new TrackedObject(*origin);
          object->track_id = track_id;
          new_map.insert(std::make_pair(track_id, object));
        }
      }

      dx = tr.trajectory[0].x - ego_state_->ego_pose_raw().x;
      dy = tr.trajectory[0].y - ego_state_->ego_pose_raw().y;
      rel_x = dx * ego_fx + dy * ego_fy;
      rel_y = dx * ego_lx + dy * ego_ly;
      object->center_x = rel_x;
      object->center_y = rel_y;

      object->prediction.prob = tr.prob;
      object->prediction.interval = tr.prediction_interval;
      object->prediction.num_of_points = tr.num_of_points;
      object->prediction.const_vel_prob = tr.const_vel_prob;
      object->prediction.const_acc_prob = tr.const_acc_prob;
      object->prediction.still_prob = tr.still_prob;
      object->prediction.coord_turn_prob = tr.coord_turn_prob;

      size_t size = tr.trajectory.size();

      object->trajectory.x.resize(size);
      object->trajectory.y.resize(size);
      object->trajectory.yaw.resize(size);
      object->trajectory.speed.resize(size);

      object->trajectory.std_dev_x.resize(size);
      object->trajectory.std_dev_y.resize(size);
      object->trajectory.std_dev_yaw.resize(size);
      object->trajectory.std_dev_speed.resize(size);

      object->trajectory.relative_ego_x.resize(size);
      object->trajectory.relative_ego_y.resize(size);
      object->trajectory.relative_ego_yaw.resize(size);
      object->trajectory.relative_ego_speed.resize(size);

      object->trajectory.relative_ego_std_dev_x.resize(size);
      object->trajectory.relative_ego_std_dev_y.resize(size);
      object->trajectory.relative_ego_std_dev_yaw.resize(size);
      object->trajectory.relative_ego_std_dev_speed.resize(size);

      for (size_t i = 0; i < size; i++) {
        object->trajectory.x[i] = tr.trajectory[i].x;
        object->trajectory.y[i] = tr.trajectory[i].y;
        object->trajectory.yaw[i] = tr.trajectory[i].yaw;
        object->trajectory.speed[i] = tr.trajectory[i].speed;

        object->trajectory.std_dev_x[i] = tr.trajectory[i].std_dev_x;
        object->trajectory.std_dev_y[i] = tr.trajectory[i].std_dev_y;
        object->trajectory.std_dev_yaw[i] = tr.trajectory[i].std_dev_yaw;
        object->trajectory.std_dev_speed[i] = tr.trajectory[i].std_dev_speed;

        object->trajectory.relative_ego_x[i] = tr.trajectory[i].relative_ego_x;
        object->trajectory.relative_ego_y[i] = tr.trajectory[i].relative_ego_y;
        // Todo:clren
        // 后续根据情况修改成与recv_relative_prediction_objects类似的写法
        double theta = tr.trajectory[i].yaw - ego_state_->ego_pose_raw().theta;
        if (theta > pi) {
          theta -= 2 * pi;
        } else if (theta < -pi) {
          theta += 2 * pi;
        }
        // object->trajectory.relative_ego_yaw[i] =
        // tr.trajectory[i].relative_ego_yaw;
        object->trajectory.relative_ego_yaw[i] = theta;
        object->trajectory.relative_ego_speed[i] =
            tr.trajectory[i].relative_ego_speed;

        object->trajectory.relative_ego_std_dev_x[i] =
            tr.trajectory[i].relative_ego_std_dev_x;
        object->trajectory.relative_ego_std_dev_y[i] =
            tr.trajectory[i].relative_ego_std_dev_y;
        object->trajectory.relative_ego_std_dev_yaw[i] =
            tr.trajectory[i].relative_ego_std_dev_yaw;
        object->trajectory.relative_ego_std_dev_speed[i] =
            tr.trajectory[i].relative_ego_std_dev_speed;
      }

      objects.push_back(object);
      idx++;
    }
  }

  for (auto iter = object_map_.begin(); iter != object_map_.end(); ++iter) {
    delete iter->second;
  }

  object_map_.clear();
  object_map_ = new_map;
}

// use relative interface when hdmap valid is false
void TrackletMaintainer::recv_relative_prediction_objects(
    const std::vector<PredictionObject> &predictions,
    std::vector<TrackedObject *> &objects) {
  // double pi = std::atan(1.0) * 4;
  std::map<int, TrackedObject *> new_map;
  auto ego_state = session_->environmental_model().get_ego_state_manager();

  // double ego_fx = std::cos(ego_state_->ego_pose_raw().theta);
  // double ego_fy = std::sin(ego_state_->ego_pose_raw().theta);
  // double ego_lx = -ego_fy;
  // double ego_ly = ego_fx;

  for (auto &p : predictions) {
    // 过滤未与相机融合， 且在相机FOV之内的
    if ((p.type == 0 && p.relative_speed_x + ego_state_->ego_v() < 1.) ||
        (!(p.fusion_source & OBSTACLE_SOURCE_CAMERA)) &&
            (p.relative_position_x > 0 &&
             tan(25) > fabs(p.relative_position_y / p.relative_position_x)) ||
        fabs(p.relative_position_y) > 10 || p.length == 0 || p.width == 0) {
      LOG_DEBUG("[obstacle_prediction_update] ignore obstacle! : [%d] \n",
                p.id);
      continue;
    }

    // double dx = p.position_x - ego_state_->ego_pose_raw().x;
    // double dy = p.position_y - ego_state_->ego_pose_raw().y;

    // double rel_x = dx * ego_fx + dy * ego_fy;
    // double rel_y = dx * ego_lx + dy * ego_ly;

    double rel_x = p.relative_position_x;
    double rel_y = p.relative_position_y;

    if (rel_x < -45 || rel_x > 120 || p.trajectory_array.size() == 0) {
      continue;
    }

    TrackedObject *origin = nullptr;

    auto iter = object_map_.find(p.id);

    if (iter != object_map_.end()) {
      origin = iter->second;
      object_map_.erase(iter);
      new_map.insert(std::make_pair(origin->track_id, origin));

      // set history results
      origin->has_history = true;
      origin->c0_history = origin->c0;
      origin->d_center_cpath_hostory = origin->d_center_cpath;
    } else {
      origin = new TrackedObject();
      origin->track_id = p.id;
      new_map.insert(std::make_pair(origin->track_id, origin));

      origin->has_history = false;
    }

    origin->timestamp = p.timestamp_us / 1000000.0;
    origin->type = p.type;
    origin->fusion_source = p.fusion_source;
    origin->fusion_type = 2;

    // double speed_yaw = p.trajectory_array[0].trajectory[0].yaw;
    // double abs_vx = p.speed * std::cos(speed_yaw);
    // double abs_vy = p.speed * std::sin(speed_yaw);
    // double rot_vx = abs_vx * ego_fx + abs_vy * ego_fy;

    // double abs_ax = p.acc * std::cos(speed_yaw);
    // double abs_ay = p.acc * std::sin(speed_yaw);
    // double rot_ax = abs_ax * ego_fx + abs_ay * ego_fy;

    // double theta = p.yaw - ego_state_->ego_pose_raw().theta;
    // if (theta > pi) {
    //   theta -= 2 * pi;
    // } else if (theta < -pi) {
    //   theta += 2 * pi;
    // }

    origin->position_type = (rel_x > 2.0) ? 0 : 1;

    origin->length = p.length;
    origin->width = p.width;

    origin->center_x = rel_x;
    origin->center_y = rel_y;
    origin->theta = p.relative_theta;
    origin->y_rel_ori = rel_y;
    origin->v_x = p.relative_speed_x + ego_state_->ego_v();
    origin->v_y = p.relative_speed_y;
    origin->speed_yaw = std::atan2(p.relative_speed_y, origin->v_x);
    origin->a = p.acc;
    origin->v = std::hypot(origin->v_x, p.relative_speed_y);

    origin->v_lead = origin->v_x;
    origin->v_lead_k = origin->v_lead;
    origin->v_lead_raw = origin->v_lead;
    origin->vy_abs = p.relative_speed_y;
    origin->vy_rel = p.relative_speed_y;

    // origin->a_lead = p.acceleration_relative_to_ground_x;
    origin->a_lead = p.relative_acceleration_x + ego_state->ego_acc();
    origin->a_lead_k = p.relative_acceleration_x + ego_state->ego_acc();

    origin->oncoming = (origin->v_lead < -3.9);

    // calculate fisheye related for cutin
    fisheye_helper(p, *origin);

    int idx = 0;
    for (auto &tr : p.trajectory_array) {
      TrackedObject *object = nullptr;
      int track_id = hash_prediction_id(origin->track_id, idx);

      iter = object_map_.find(track_id);
      if (iter != object_map_.end()) {
        object = iter->second;
        object_map_.erase(iter);

        if (object != origin) {
          *object = *origin;
          object->track_id = track_id;
        }
        new_map.insert(std::make_pair(track_id, object));
      } else {
        iter = new_map.find(track_id);
        if (iter != new_map.end()) {
          object = iter->second;

          if (object != origin) {
            *object = *origin;
            object->track_id = track_id;
          }
        } else {
          object = new TrackedObject(*origin);
          object->track_id = track_id;
          new_map.insert(std::make_pair(track_id, object));
        }
      }

      object->prediction.prob = tr.prob;
      object->prediction.interval = tr.prediction_interval;
      object->prediction.num_of_points = tr.num_of_points;
      object->prediction.const_vel_prob = tr.const_vel_prob;
      object->prediction.const_acc_prob = tr.const_acc_prob;
      object->prediction.still_prob = tr.still_prob;
      object->prediction.coord_turn_prob = tr.coord_turn_prob;

      size_t size = tr.trajectory.size();

      object->trajectory.x.resize(size);
      object->trajectory.y.resize(size);
      object->trajectory.yaw.resize(size);
      object->trajectory.speed.resize(size);

      object->trajectory.std_dev_x.resize(size);
      object->trajectory.std_dev_y.resize(size);
      object->trajectory.std_dev_yaw.resize(size);
      object->trajectory.std_dev_speed.resize(size);

      object->trajectory.relative_ego_x.resize(size);
      object->trajectory.relative_ego_y.resize(size);
      object->trajectory.relative_ego_yaw.resize(size);
      object->trajectory.relative_ego_speed.resize(size);

      object->trajectory.relative_ego_std_dev_x.resize(size);
      object->trajectory.relative_ego_std_dev_y.resize(size);
      object->trajectory.relative_ego_std_dev_yaw.resize(size);
      object->trajectory.relative_ego_std_dev_speed.resize(size);

      for (size_t i = 0; i < size; i++) {
        object->trajectory.x[i] = tr.trajectory[i].x;
        object->trajectory.y[i] = tr.trajectory[i].y;
        object->trajectory.yaw[i] = tr.trajectory[i].yaw;
        object->trajectory.speed[i] = tr.trajectory[i].speed;

        object->trajectory.std_dev_x[i] = tr.trajectory[i].std_dev_x;
        object->trajectory.std_dev_y[i] = tr.trajectory[i].std_dev_y;
        object->trajectory.std_dev_yaw[i] = tr.trajectory[i].std_dev_yaw;
        object->trajectory.std_dev_speed[i] = tr.trajectory[i].std_dev_speed;

        object->trajectory.relative_ego_x[i] = tr.trajectory[i].relative_ego_x;
        object->trajectory.relative_ego_y[i] = tr.trajectory[i].relative_ego_y;
        object->trajectory.relative_ego_yaw[i] =
            tr.trajectory[i].relative_ego_yaw;
        if (origin->fusion_source & OBSTACLE_SOURCE_CAMERA) {
          object->trajectory.relative_ego_yaw[i] = origin->speed_yaw;
        }

        object->trajectory.relative_ego_speed[i] =
            tr.trajectory[i].relative_ego_speed;
        object->trajectory.relative_ego_std_dev_x[i] =
            tr.trajectory[i].relative_ego_std_dev_x;
        object->trajectory.relative_ego_std_dev_y[i] =
            tr.trajectory[i].relative_ego_std_dev_y;
        object->trajectory.relative_ego_std_dev_yaw[i] =
            tr.trajectory[i].relative_ego_std_dev_yaw;
        object->trajectory.relative_ego_std_dev_speed[i] =
            tr.trajectory[i].relative_ego_std_dev_speed;
      }

      objects.push_back(object);
      idx++;
    }
  }

  for (auto iter = object_map_.begin(); iter != object_map_.end(); ++iter) {
    delete iter->second;
  }

  object_map_.clear();
  object_map_ = new_map;
}

void TrackletMaintainer::fisheye_helper(const PredictionObject &prediction,
                                        TrackedObject &object) {
  double pi = std::atan(1.0) * 4;
  double half_length = prediction.length / 2;
  double half_width = prediction.width / 2;
  double v_cos = std::cos(prediction.relative_theta);
  double v_sin = std::sin(prediction.relative_theta);

  // f_l, f_r, l_r, l_l
  std::vector<Point2D> edge_points{{half_length, half_width},
                                   {half_length, -half_width},
                                   {-half_length, -half_width},
                                   {-half_length, half_width}};

  for (auto &point : edge_points) {
    Point2D edge_point = point;
    point.x = edge_point.x * v_cos - edge_point.y * v_sin +
              prediction.relative_position_x;
    point.y = edge_point.x * v_sin + edge_point.y * v_cos +
              prediction.relative_position_y;
  }

  std::vector<Point2D> edge_points_l{edge_points[0], edge_points[3]};
  std::vector<Point2D> edge_points_r{edge_points[1], edge_points[2]};

  double dis_0 = std::hypot(edge_points_l[0].x, edge_points_l[0].y);
  double dis_1 = std::hypot(edge_points_r[0].x, edge_points_r[0].y);

  // find nearest edge points
  if (dis_0 > dis_1) {
    object.points_3d_f = edge_points_r[0];
    object.points_3d_r = edge_points_r[1];
  } else {
    object.points_3d_f = edge_points_l[0];
    object.points_3d_r = edge_points_l[1];
  }

  // calcuate cutin related
  // 转换为障碍物相对自车车头距离
  auto vehicle_param = session_->vehicle_config_context().get_vehicle_param();
  object.location_head = std::max(object.points_3d_f.x, object.points_3d_r.x) -
                         vehicle_param.rear_axis_to_front_edge;
  object.location_tail = std::min(object.points_3d_f.x, object.points_3d_r.x) -
                         vehicle_param.rear_axis_to_front_edge;

  if (std::abs(object.points_3d_f.y) < std::abs(object.points_3d_r.y)) {
    object.y_min = object.points_3d_f.y;
  } else {
    object.y_min = object.points_3d_r.y;
  }

  if (object.location_head * object.location_tail < 0.0) {
    object.y_x0 = (object.points_3d_f.x * object.points_3d_r.y -
                   object.points_3d_r.x * object.points_3d_f.y) /
                  (object.points_3d_f.x - object.points_3d_r.x);
  } else {
    object.y_x0 = 0.0;
  }
}

void TrackletMaintainer::calc(
    std::vector<TrackedObject *> &tracked_objects, int scenario,
    double lane_width, double lat_offset, bool borrow_bicycle_lane,
    bool enable_intersection_planner, double dist_rblane, bool tleft_lane,
    bool rightest_lane, double dist_intersect, double intersect_length,
    bool left_faster, bool right_faster, LeadCars &lead_cars,
    bool isRedLightStop, bool isFasterStaticAvd, bool isOnHighway,
    std::vector<double> d_poly, std::vector<double> c_poly) {
  seq_state_.remove_clean();
  double v_ego = ego_state_->ego_v();
  double ego_rear_axis_to_front_edge = session_->vehicle_config_context()
                                           .get_vehicle_param()
                                           .rear_axis_to_front_edge;
  if (frenet_coord_ != nullptr) {
    Point2D frenet_point;
    if (frenet_coord_->XYToSL(Point2D(ego_rear_axis_to_front_edge, 0),
                              frenet_point)) {
      s_ego_ = frenet_point.x;
      l_ego_ = frenet_point.y;
      double theta = frenet_coord_->GetPathCurveHeading(frenet_point.x);
      vl_ego_ = v_ego * std::sin(-theta);
      vs_ego_ = v_ego * std::cos(-theta);
    }

    for (auto item : tracked_objects) {
      item->v_ego = v_ego;
      item->v_rel = item->v_lead - v_ego;
      bool frenet_transform_valid = false;

      double d_poly_offset = lat_offset;
      if ((d_poly.size() == 4) && (c_poly.size() == 4)) {
        d_poly_offset = d_poly[3] - c_poly[3];
      }

      frenet_transform_valid = fill_info_with_refline(*item, d_poly_offset);
      if (!hdmap_valid_) {
        fill_deriv_info(*item);
      }
      // only use obstacle with camera source
      if ((item->fusion_source & OBSTACLE_SOURCE_CAMERA) &&
          frenet_transform_valid) {
        is_potential_lead_one(*item, v_ego);
      }
      calc_intersection_with_refline(*item, enable_intersection_planner);
    }
  }

  select_lead_cars(tracked_objects, lead_cars);

  for (auto tr : tracked_objects) {
    check_accident_car(*tr, v_ego, scenario, dist_intersect, intersect_length,
                       left_faster, right_faster, isRedLightStop,
                       isFasterStaticAvd, isOnHighway);
  }

  for (auto tr : tracked_objects) {
    check_prebrk_object(*tr, v_ego, lane_width);
  }

  for (auto tr : tracked_objects) {
    // ignore obj without camera source
    if (!(tr->fusion_source & OBSTACLE_SOURCE_CAMERA)) {
      tr->is_avd_car = false;
      continue;
    }
    tr->is_avd_car = is_potential_avoiding_car(
        *tr, lead_cars.lead_one, v_ego, lane_width, scenario,
        borrow_bicycle_lane, dist_rblane, tleft_lane, rightest_lane,
        dist_intersect, intersect_length, isRedLightStop);
  }

  is_leadone_potential_avoiding_car(lead_cars.lead_one, scenario, lane_width,
                                    borrow_bicycle_lane, rightest_lane,
                                    dist_intersect, isRedLightStop);
  auto &debug_info_manager = DebugInfoManager::GetInstance();
  auto &planning_debug_data = debug_info_manager.GetDebugInfoPb();
  auto environment_model_debug_info =
      planning_debug_data->mutable_environment_model_info();
  environment_model_debug_info->set_ego_s(s_ego_);
  environment_model_debug_info->set_ego_l(l_ego_);
  environment_model_debug_info->mutable_obstacle()->Clear();
  for (auto tr : tracked_objects) {
    planning::common::Obstacle *obstacle =
        environment_model_debug_info->add_obstacle();
    obstacle->set_id(tr->track_id);
    obstacle->set_type(tr->type);
    obstacle->set_s(tr->s);
    obstacle->set_l(tr->l);
    obstacle->set_s_to_ego(tr->d_rel);
    obstacle->set_max_l_to_ref(tr->d_max_cpath);
    obstacle->set_min_l_to_ref(tr->d_min_cpath);
    obstacle->set_s_with_min_l(tr->s_min);
    obstacle->set_s_with_max_l(tr->s_max);
    obstacle->set_nearest_l_to_desire_path(tr->d_path);
    obstacle->set_nearest_l_to_ego(tr->d_path_self);
    obstacle->set_vs_lat_relative(tr->v_lat);
    obstacle->set_vs_lon_relative(tr->v_rel);
    obstacle->set_vs_lon(tr->v_lead);
    obstacle->set_nearest_y_to_desired_path(tr->y_rel);
    obstacle->set_is_accident_car(tr->is_accident_car);
    obstacle->set_is_accident_cnt(tr->is_accident_cnt);
    obstacle->set_is_avoid_car(tr->is_avd_car);
    obstacle->set_is_lane_lead_obstacle(tr->is_lead);
    obstacle->set_current_lead_obstacle_to_ego(tr->is_temp_lead);
    // obstacle->set_cutin_p(tr->cutinp);
  }

  // auto ad_info = session_->mutable_planning_output_context()
  //                    ->mutable_planning_hmi_info()
  //                    ->mutable_ad_info();
  // feed_hmi
  // for (auto tr : tracked_objects) {
  //   if (!(tr->fusion_source & OBSTACLE_SOURCE_CAMERA)) {
  //     continue;
  //   }
  //   auto obstacle = ad_info->add_obstacle_info();
  //   obstacle->set_id(tr->track_id);
  //   obstacle->set_center_x(tr->center_x);
  //   obstacle->set_center_y(tr->center_y);
  //   // obstacle->set_type(tr->type)
  //   obstacle->set_speed_x(tr->v_x);
  //   obstacle->set_speed_y(tr->v_y);
  //   obstacle->set_heading(tr->speed_yaw);  // 这里应该是yaw角
  //   obstacle->mutable_size()->set_length(tr->length);
  //   obstacle->mutable_size()->set_width(tr->width);
  //   obstacle->mutable_size()->set_height(tr->height);
  //   if (lead_cars.lead_one != nullptr && lead_cars.lead_one->track_id ==
  //   tr->track_id ||
  //       lead_cars.temp_lead_one != nullptr &&
  //       lead_cars.temp_lead_one->track_id == tr->track_id) {
  //     obstacle->set_lon_status(::PlanningHMI::ObstacleLonStatus::FOLLOWING);
  //   } else {
  //     obstacle->set_lon_status(::PlanningHMI::ObstacleLonStatus::NORMAL);
  //   }
  // }
  // Point2D cart_point;
  // if (frenet_coord_->FrenetCoord2CartCoord(Point2D(s_ego_ + 5, 0),
  // cart_point) ==
  //       TRANSFORM_SUCCESS) {
  //   ad_info->mutable_landing_point()->mutable_relative_pos()->set_x(cart_point.x);
  //   ad_info->mutable_landing_point()->mutable_relative_pos()->set_y(cart_point.y);
  //   ad_info->mutable_landing_point()->mutable_relative_pos()->set_z(0);
  //   ad_info->mutable_landing_point()->set_heading(0);
  // }
}

bool TrackletMaintainer::fill_info_with_refline(TrackedObject &item,
                                                double lat_offset) {
  double half_length = item.length * 0.5;

  double half_width;
  bool frenet_transform_valid = true;
  if (item.type > 10000) {
    half_width = std::max(0.5, std::min(2.0, item.width * 0.5));
  } else if (item.type > 0) {
    half_width = std::max(1.0, std::min(2.0, item.width * 0.5));
  } else {
    half_width = item.width * 0.5;
  }

  double speed_yaw;
  if (item.trajectory.relative_ego_yaw.size() > 0) {
    speed_yaw = item.trajectory.relative_ego_yaw[0];
  } else {
    if (hdmap_valid_) {
      speed_yaw = item.theta;
    } else {
      speed_yaw = item.speed_yaw;
    }
  }

  double s, l, v_s, v_l, theta;
  Point2D frenet_point;
  if (frenet_coord_->XYToSL(Point2D(item.center_x, item.center_y),
                            frenet_point)) {
    s = frenet_point.x;
    l = frenet_point.y;
    theta = frenet_coord_->GetPathCurveHeading(frenet_point.x);
    v_l = item.v * std::sin(speed_yaw - theta);
    v_s = item.v * std::cos(speed_yaw - theta);
  } else {
    frenet_transform_valid = false;
  }
  item.vs_rel = v_s - vs_ego_;
  item.v_lead = v_s;
  item.v_lead_k = v_s;
  item.v_lead_raw = v_s;
  item.v_rel = v_s - vs_ego_;
  item.s = s;
  item.l = l;
  item.l0 = l_ego_;
  item.c0 = l_ego_;
  item.v_lat = (l > 0) ? v_l : -v_l;
  // std::cout << "object track_id: " << item.track_id
  //           << " ego l_ego_ : " << l_ego_ << " ego vl_: " << vl_ego_
  //           << " item.s: " << item.s << " item.l: " << item.l
  //           << " item.v_lat: " << item.v_lat << " item.vy_rel: " <<
  //           item.vy_rel
  //           << " v_l: " << v_l << std::endl;
  if (hdmap_valid_) {
    item.a_lead = item.a * std::cos(item.theta - theta);
    item.a_lead_k = item.a_lead;
    item.v_lat = (l > 0) ? v_l : -v_l;
    item.v_lat_self = item.v_lat;
    item.vy_rel = v_l;
  }

  std::array<int, 2> sgn_list{1, -1};
  std::vector<double> bbox_s;
  std::vector<double> bbox_l;
  std::vector<double> bbox_l_pos;

  if (item.l >= lat_offset) {
    item.d_path_raw =
        item.l -
        (0.5 * item.width * std::fabs(std::cos(item.theta - theta)) +
         0.5 * item.length * std::sin(std::fabs(item.theta - theta))) -
        lat_offset;
    if (item.d_path_raw < 0) {
      item.d_path_raw = 0;
    }
  } else {
    item.d_path_raw =
        item.l +
        (0.5 * item.width * std::fabs(std::cos(item.theta - theta)) +
         0.5 * item.length * std::sin(std::fabs(item.theta - theta))) -
        lat_offset;
    if (item.d_path_raw > 0) {
      item.d_path_raw = 0;
    }
  }

  for (auto sgn_length : sgn_list) {
    for (auto sgn_width : sgn_list) {
      double _s = item.s +
                  sgn_length * std::cos(item.theta - theta) * half_length -
                  sgn_width * std::sin(item.theta - theta) * item.width * 0.5;

      if ((item.s - s_ego_) * (_s - s_ego_) <= 0) {
        half_width = item.width * 0.5;
        break;
      }
    }
  }

  for (auto sgn_length : sgn_list) {
    for (auto sgn_width : sgn_list) {
      double _s = item.s +
                  sgn_length * std::cos(item.theta - theta) * half_length -
                  sgn_width * std::sin(item.theta - theta) * half_width;

      double _l = item.l +
                  sgn_length * std::sin(item.theta - theta) * half_length +
                  sgn_width * std::cos(item.theta - theta) * half_width;

      bbox_s.push_back(_s);
      bbox_l.push_back(_l);

      if (_s >= s_ego_) {
        bbox_l_pos.push_back(_l);
      }
    }
  }

  if (bbox_s.empty()) {
    item.d_rel = 0;
  } else {
    if (item.s > s_ego_) {
      double min_s = *std::min_element(bbox_s.begin(), bbox_s.end());
      item.d_rel = (min_s > s_ego_) ? (min_s - s_ego_) : 0;
    } else {
      double max_s = *std::max_element(bbox_s.begin(), bbox_s.end());
      item.d_rel = (max_s < s_ego_) ? (max_s - s_ego_) : 0;
    }
  }

  std::vector<double> bbox_l_self;
  std::vector<double> bbox_l_self_pos;

  for (auto value : bbox_l) {
    bbox_l_self.push_back(value - l_ego_);
  }

  for (auto value : bbox_l_pos) {
    bbox_l_self_pos.push_back(value - l_ego_);
  }

  double min_l =
      (!bbox_l.empty()) ? *std::min_element(bbox_l.begin(), bbox_l.end()) : 0;
  double max_l =
      (!bbox_l.empty()) ? *std::max_element(bbox_l.begin(), bbox_l.end()) : 0;
  double min_l_pos =
      (!bbox_l_pos.empty())
          ? *std::min_element(bbox_l_pos.begin(), bbox_l_pos.end())
          : 0;
  double max_l_pos =
      (!bbox_l_pos.empty())
          ? *std::max_element(bbox_l_pos.begin(), bbox_l_pos.end())
          : 0;
  double min_l_self =
      (!bbox_l_self.empty())
          ? *std::min_element(bbox_l_self.begin(), bbox_l_self.end())
          : 0;
  double max_l_self =
      (!bbox_l_self.empty())
          ? *std::max_element(bbox_l_self.begin(), bbox_l_self.end())
          : 0;
  double min_l_self_pos =
      (!bbox_l_self_pos.empty())
          ? *std::min_element(bbox_l_self_pos.begin(), bbox_l_self_pos.end())
          : 0;
  double max_l_self_pos =
      (!bbox_l_self_pos.empty())
          ? *std::max_element(bbox_l_self_pos.begin(), bbox_l_self_pos.end())
          : 0;

  if (item.l >= lat_offset) {
    item.d_path = ((item.l - lat_offset) * (min_l - lat_offset) > 0)
                      ? (min_l - lat_offset)
                      : 0;

    if (!bbox_l_pos.empty()) {
      item.d_path_pos = ((item.l - lat_offset) * (min_l_pos - lat_offset) > 0)
                            ? (min_l_pos - lat_offset)
                            : 0;
    } else {
      item.d_path_pos = 100;
    }
  } else {
    item.d_path = ((item.l - lat_offset) * (max_l - lat_offset) > 0)
                      ? (max_l - lat_offset)
                      : 0;

    if (!bbox_l_pos.empty()) {
      item.d_path_pos = ((item.l - lat_offset) * (max_l_pos - lat_offset) > 0)
                            ? (max_l_pos - lat_offset)
                            : 0;
    } else {
      item.d_path_pos = 100;
    }
  }

  if (item.l >= l_ego_) {
    item.d_path_self = ((item.l - l_ego_) * min_l_self > 0) ? min_l_self : 0;

    if (!bbox_l_pos.empty()) {
      item.d_path_self_pos =
          ((item.l - l_ego_) * min_l_self_pos > 0) ? min_l_self_pos : 0;
    } else {
      item.d_path_self_pos = 100;
    }
  } else {
    item.d_path_self = ((item.l - l_ego_) * max_l_self > 0) ? max_l_self : 0;

    if (!bbox_l_pos.empty()) {
      item.d_path_self_pos =
          ((item.l - l_ego_) * max_l_self_pos > 0) ? max_l_self_pos : 0;
    } else {
      item.d_path_self_pos = 100;
    }
  }

  item.d_center_cpath = item.l;
  item.d_max_cpath = max_l;
  item.d_min_cpath = min_l;

  item.s_center = item.s;

  auto it = std::find(bbox_l.begin(), bbox_l.end(), max_l);
  size_t index = (size_t)(it - bbox_l.begin());

  if (index < bbox_s.size()) {
    item.s_max = std::max(bbox_s[index], 0.0);
  }

  it = std::find(bbox_l.begin(), bbox_l.end(), min_l);
  index = (size_t)(it - bbox_l.begin());

  if (index < bbox_s.size()) {
    item.s_min = std::max(bbox_s[index], 0.0);
  }

  item.y_rel = item.y_rel_ori - item.l + item.d_path;
  item.d_path = std::fabs(item.d_path);
  item.d_path_self = std::fabs(item.d_path_self);
  item.d_path_pos = std::fabs(item.d_path_pos);
  item.d_path_self_pos = std::fabs(item.d_path_self_pos);
  return frenet_transform_valid;
}

void TrackletMaintainer::fill_deriv_info(TrackedObject &item) {
  LOG_DEBUG("----fill_deriv_info-----\n");
  double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;
  if (!item.has_history) {
    return;
  }

  double interval = (item.last_recv_time == 0.0)
                        ? planning_cycle_time
                        : (item.timestamp - item.last_recv_time);

  double pi = std::atan(1.0) * 4;

  double v_lat_unfiltered;
  if (item.c0 != DBL_MAX && item.d_center_cpath_hostory != DBL_MAX &&
      item.d_center_cpath_hostory != 100) {
    if (std::fabs(item.c0 - item.c0_history) > 0.2) {
      v_lat_unfiltered = item.v_lat;
    } else {
      v_lat_unfiltered = equal_zero(interval)
                             ? 0.0
                             : ((std::fabs(item.d_center_cpath) -
                                 std::fabs(item.d_center_cpath_hostory)) /
                                interval);
    }
  }

  TrackletSequentialState *sequential_state = seq_state_.get(item.track_id);
  seq_state_.mark_dirty(item.track_id);
  if (sequential_state == nullptr) {
    return;
  }

  double v_lat_error = v_lat_unfiltered - item.v_lat;
  sequential_state->v_lat_deriv += 5.0 * v_lat_error * interval;
  // item.v_lat += (sequential_state->v_lat_deriv + 3.2 * v_lat_error) *
  // interval;

  // double k_v_lat = 2 * pi * 0.2 * interval / (1 + 2 * pi * 0.2 * interval);
  double k_v_lat = 0.4;
  item.v_lat = k_v_lat * v_lat_unfiltered + (1 - k_v_lat) * item.v_lat;
  // obstacle without camera source, set to zero
  if ((item.fusion_source != OBSTACLE_SOURCE_CAMERA) &&
      (item.fusion_source != OBSTACLE_SOURCE_F_RADAR_CAMERA)) {
    item.v_lat = 0.0;
  }
  item.v_lat_self = item.v_lat;

  // double v_lat_self_unfiltered = equal_zero(interval) ? 0.0 :
  //     (item.d_path_self - item.history->d_path_self) / interval;
  // double k_v_lat = 2 * pi * 0.2 * interval / (1 + 2 * pi * 0.2 * interval);
  // item.v_lat_self = k_v_lat * v_lat_self_unfiltered + (1 - k_v_lat) *
  //     item.v_lat_self;

  // double a_rel_unfiltered = equal_zero(interval) ? 0.0:
  //     (item.v_rel - item.history->v_rel);
  // double a_rel_unfiltered = std::max(-10.0, std::min(10.0,
  // a_rel_unfiltered)); double k_a_lead = 2 * pi * 0.5 * interval / (1 + 2 * pi
  // * 0.5 * interval); item.a_rel = (k_a_lead == 1.0) ? 0.0 :
  //     (k_a_lead * a_rel_unfiltered + (1 - k_a_lead) *item.a_rel);
}

void TrackletMaintainer::calc_intersection_with_refline(
    TrackedObject &item, bool enable_intersection_planner) {
  if (0 == item.type) {
    item.trajectory.intersection = 0;
    return;
  }

  if ((tan(M_PI / 3) * (-item.center_x - 5.0) > std::fabs(item.center_y)) &&
      item.center_x < -5.0) {
    item.trajectory.intersection = 0;
    return;
  }

  auto &x = item.trajectory.x;
  auto &y = item.trajectory.y;
  std::vector<double> ego_x;
  std::vector<double> ego_y;
  std::vector<double> ego_speed;
  std::vector<double> ego_yaw;
  double ego_fx = std::cos(ego_state_->ego_pose_raw().theta);
  double ego_fy = std::sin(ego_state_->ego_pose_raw().theta);
  double ego_lx = -ego_fy;
  double ego_ly = ego_fx;
  for (int i = 0; i < x.size(); i++) {
    double dx = x[i] - ego_state_->ego_pose_raw().x;
    double dy = y[i] - ego_state_->ego_pose_raw().y;

    double rel_x = dx * ego_fx + dy * ego_fy;
    double rel_y = dx * ego_lx + dy * ego_ly;
    ego_x.push_back(rel_x);
    ego_y.push_back(rel_y);
    ego_speed.push_back(item.trajectory.speed[i]);
    ego_yaw.push_back(item.trajectory.yaw[i] -
                      ego_state_->ego_pose_raw().theta);
  }
  if (ego_x.size() > 0) {
    double half_car_width =
        max(1.1, 0.5 * item.width * std::fabs(cos(item.theta)) +
                     0.5 * item.length * sin(std::fabs(item.theta)));
    double half_lane_width = 0.;
    if (enable_intersection_planner) {
      half_lane_width = -0.45;
    }

    double ignorance_threshold = half_car_width + half_lane_width;

    double s0 = 0.0;
    double l0 = 0.0;
    double v_s = 0.0, v_l = 0.0, theta = 0.0;
    double smin = 0.0;
    double lmin = 0.0;
    double smax = 0.0;
    double lmax = 0.0;
    double send = 0.0;
    double lend = 0.0;
    int valid_range = min((int)ego_x.size() - 1, 25);
    int end_range = min((int)ego_x.size() - 1, 25);

    int max_idx = 0;
    int end_idx = 0;
    int min_idx = valid_range;
    if (ego_x[0] == ego_x[ego_x.size() - 1]) {
      item.trajectory.intersection = 0;
      return;
    }
    Point2D frenet_point;
    if (frenet_coord_->XYToSL(Point2D(ego_x[0], ego_y[0]), frenet_point)) {
      s0 = frenet_point.x;
      l0 = frenet_point.y;
    }

    if (std::fabs(l0) <= ignorance_threshold) {
      if ((item.prediction.still_prob > 0.9 ||
           item.trajectory.speed[0] < 0.1) &&
          !item.is_lead) {
        item.trajectory.intersection = 0;
        return;
      }
      if (s0 - s_ego_ < 0) {
        item.trajectory.intersection = 0;
        return;
      } else {
        for (int i = valid_range; i >= 0; i--) {
          if (ego_x[i] < 0 || ego_x[i] > 80) {
            continue;
          }
          max_idx = i;
          break;
        }
        // double *v;
        // double *yaw;
        // *v = ego_speed[max_idx];
        // *yaw = ego_yaw[max_idx];
        if (frenet_coord_->XYToSL(Point2D(ego_x[max_idx], ego_y[max_idx]),
                                  frenet_point)) {
          smax = frenet_point.x;
          lmax = frenet_point.y;
        }

        if (std::fabs(lmax) > ignorance_threshold) {
          item.trajectory.intersection = 10000;
        } else {
          item.trajectory.intersection = 1;
        }
        return;
      }
    }

    for (int i = 0; i < valid_range; i++) {
      if (ego_x[i] < 0) {
        continue;
      }

      min_idx = i;
      break;
    }

    for (int i = valid_range; i >= 0; i--) {
      if (ego_x[i] < 0 || ego_x[i] > 80) {
        continue;
      }

      max_idx = i;
      break;
    }

    for (int i = end_range; i >= 0; i--) {
      if (ego_x[i] < 0 || ego_x[i] > 80) {
        continue;
      }

      end_idx = i;
      break;
    }

    if (min_idx >= max_idx) {
      item.trajectory.intersection = 0;
      return;
    }
    double end_speed_yaw;
    if (item.trajectory.yaw.size() > 0) {
      end_speed_yaw =
          item.trajectory.yaw[end_idx] - ego_state_->ego_pose_raw().theta;
    }
    if (frenet_coord_->XYToSL(Point2D(ego_x[min_idx], ego_y[min_idx]),
                              frenet_point)) {
      smin = frenet_point.x;
      lmin = frenet_point.y;
    }
    if (frenet_coord_->XYToSL(Point2D(ego_x[max_idx], ego_y[max_idx]),
                              frenet_point)) {
      smax = frenet_point.x;
      lmax = frenet_point.y;
    }
    if (frenet_coord_->XYToSL(Point2D(ego_x[end_idx], ego_y[end_idx]),
                              frenet_point)) {
      send = frenet_point.x;
      lend = frenet_point.y;
      theta = frenet_coord_->GetPathCurveHeading(frenet_point.x);
      v_l = item.v * std::sin(end_speed_yaw - theta);
      v_s = item.v * std::cos(end_speed_yaw - theta);
    }

    double v_l_end = (lend > 0) ? v_l : -v_l;
    double v_s_end = v_s;
    bool is_same_side = ((lmin > 0 && lend > 0) || (lmin <= 0 && lend <= 0));

    int sgn = lmin > ignorance_threshold ? -1 : 1;

    double min_ignore_thre = calc_ignorance_threshold(
        item, min_idx, sgn, enable_intersection_planner);
    double max_ignore_thre = calc_ignorance_threshold(
        item, max_idx, sgn, enable_intersection_planner);
    double end_ignore_thre = calc_ignorance_threshold(
        item, end_idx, sgn, enable_intersection_planner);

    if (std::fabs(lmin) <= min_ignore_thre && min_idx <= max_idx) {
      if (40 == min_idx) {
        min_idx = 39;
      }
      item.trajectory.intersection = min_idx;
      if ((lmin + min_ignore_thre) * (lend + end_ignore_thre) < 0 &&
          ((!is_same_side &&
            (std::fabs(lend) > 2 ||
             (v_s_end <= -3.0 && std::fabs(lend) > item.width / 2))) ||
           (v_l_end > 0.6) || (std::fabs(lend) < 0.5 && v_l_end < -0.5))) {
        item.trajectory.intersection = 10000;
      }
      return;
    }

    if ((lmin + min_ignore_thre) * (lmax + max_ignore_thre) >= 0) {
      double smid = 0.0;
      double lmid = 0.0;
      int mid_idx = (min_idx + max_idx) / 2;
      if (frenet_coord_->XYToSL(Point2D(ego_x[mid_idx], ego_y[mid_idx]),
                                frenet_point)) {
        smid = frenet_point.x;
        lmid = frenet_point.y;
      }
      min_ignore_thre = calc_ignorance_threshold(item, min_idx, sgn,
                                                 enable_intersection_planner);
      double mid_ignore_thre = calc_ignorance_threshold(
          item, mid_idx, sgn, enable_intersection_planner);

      if ((lmin + min_ignore_thre) * (lmid + mid_ignore_thre) <= 0) {
        max_idx = mid_idx;
        lmax = lmid;
      } else if ((lmax + max_ignore_thre) * (lmid + mid_ignore_thre) <= 0) {
        min_idx = mid_idx;
        lmin = lmid;
      } else {
        item.trajectory.intersection = 0;
        return;
      }
    }

    while (min_idx <= max_idx) {
      double smid = 0.0;
      double lmid = 0.0;
      int mid_idx = (min_idx + max_idx) / 2;
      if (frenet_coord_->XYToSL(Point2D(ego_x[mid_idx], ego_y[mid_idx]),
                                frenet_point)) {
        smid = frenet_point.x;
        lmid = frenet_point.y;
      }

      double min_ignore_thre = calc_ignorance_threshold(
          item, min_idx, sgn, enable_intersection_planner);
      double mid_ignore_thre = calc_ignorance_threshold(
          item, mid_idx, sgn, enable_intersection_planner);

      if ((lmin + min_ignore_thre) * (lmid + mid_ignore_thre) > 0) {
        if (max_idx <= mid_idx + 1) {
          item.trajectory.intersection = mid_idx;
          if ((lmin + min_ignore_thre) * (lend + end_ignore_thre) < 0 &&
              ((!is_same_side &&
                (std::fabs(lend) > 2 ||
                 (v_s_end <= -3.0 && std::fabs(lend) > item.width / 2))) ||
               (v_l_end > 0.6) || (std::fabs(lend) < 0.5 && v_l_end < -0.5))) {
            item.trajectory.intersection = 10000;
          }

          if (mid_idx == 40) {
            item.trajectory.intersection = 39;
          }
          return;
        }

        min_idx = mid_idx;
        smin = smid;
        lmin = lmid;
      } else {
        if (mid_idx <= min_idx + 1) {
          item.trajectory.intersection = mid_idx;
          if ((lmin + min_ignore_thre) * (lend + end_ignore_thre) < 0 &&
              ((!is_same_side &&
                (std::fabs(lend) > 2 ||
                 (v_s_end <= -3.0 && std::fabs(lend) > item.width / 2))) ||
               (v_l_end > 0.6) || (std::fabs(lend) < 0.5 && v_l_end < -0.5))) {
            item.trajectory.intersection = 10000;
          }

          if (mid_idx == 40) {
            item.trajectory.intersection = 39;
          }
          return;
        }

        max_idx = mid_idx;
        smax = smid;
        lmax = lmid;
      }
    }

    item.trajectory.intersection = 0;
  }
}

double TrackletMaintainer::calc_ignorance_threshold(
    TrackedObject &item, int idx, int sgn, bool enable_intersection_planner) {
  double half_car_width = std::max(
      1.1,
      0.5 * item.width * std::fabs(cos(item.trajectory.relative_ego_yaw[idx])) +
          0.5 * item.length *
              sin(std::fabs(item.trajectory.relative_ego_yaw[idx])));

  double half_lane_width = 0.;
  if (enable_intersection_planner) {
    if (sgn == -1) half_lane_width = -0.45;
  }

  double ignorance_threshold = half_lane_width + half_car_width;
  return sgn * ignorance_threshold;
}

void TrackletMaintainer::check_accident_car(
    TrackedObject &item, double v_ego, int scenario, double dist_intersect,
    double intersect_length, bool left_faster, bool right_faster,
    bool isRedLightStop, bool isFasterStaticAvd, bool isOnHighway) {
  LOG_DEBUG("----check_accident_car-----\n");
  double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;
  std::array<double, 5> xp{0, 10, 15, 20, 30};
  std::array<double, 5> fp{250, 200, 170, 140, 100};
  std::array<double, 5> xp_lat{-1.0, -0.7, -0.5, 0.0, 0.5};
  std::array<double, 5> fp_lat{1.0, 0.8, 0.4, 0.2, 0.1};
  double accident_discern_threshold = 0.0;
  // double l_ego = ego_state_->ego_frenet().y;
  double l_ego = l_ego_;

  if (scenario == LOCATION_INTER &&
      item.d_rel - dist_intersect <= intersect_length) {
    accident_discern_threshold = 50.0;
  } else {
    accident_discern_threshold = interp(v_ego, xp, fp);
  }

  if (item.type != 10001 && !isOnHighway) {
    if (isFasterStaticAvd && item.d_rel < 40.0) {
      accident_discern_threshold = 50.0;
    }
    if (item.d_rel < 15.0) {
      accident_discern_threshold *=
          interp(std::max(l_ego - item.d_max_cpath, item.d_min_cpath - l_ego),
                 xp_lat, fp_lat);
    }
    accident_discern_threshold = std::max(accident_discern_threshold, 25.0);
  }

  double gap = (item.last_recv_time == 0.0)
                   ? planning_cycle_time
                   : (item.timestamp - item.last_recv_time);

  if (item.type == 20001 && (item.is_lead || item.is_temp_lead) &&
      item.d_rel < 40.0 && item.v_lead > -3 && item.v_lead < 0.5 &&
      std::fabs(item.v_lat) < 0.2) {
    item.is_accident_cnt =
        std::min(5 * (item.is_accident_cnt + gap),
                 accident_discern_threshold * planning_cycle_time);
  } else if (((!isRedLightStop && (item.is_lead || item.is_temp_lead)) ||
              item.is_accident_car) &&
             item.v_lead > -3 && item.v_lead < 0.5 &&
             std::fabs(item.v_lat) < 0.2 &&
             ((left_faster || right_faster) || scenario == LOCATION_INTER)) {
    item.is_accident_cnt =
        std::min(item.is_accident_cnt + gap,
                 accident_discern_threshold * planning_cycle_time);
  } else {
    int count = (int)((gap + 0.01) / planning_cycle_time);
    item.is_accident_cnt =
        std::max(item.is_accident_cnt - accident_discern_threshold / 6 * count *
                                            planning_cycle_time,
                 0.0);
  }
  if (item.is_accident_cnt >=
      accident_discern_threshold * planning_cycle_time) {
    item.is_accident_car = true;
  } else if (equal_zero(item.is_accident_cnt)) {
    item.is_accident_car = false;
  }
}

void TrackletMaintainer::check_prebrk_object(TrackedObject &item, double v_ego,
                                             double lane_width) {
  double coe = 1.6;
  double threshold_dis = std::max((v_ego * v_ego) / (2 * 4) * coe, 12.5);
  // bool lat_ignore_condition = ((item.d_path > 1.6 && item.type < 10000) ||
  //    (item.d_path > 1.6 && item.type >= 10000));
  bool lat_ignore_condition = (item.d_path > 1.6);

  bool ignore_car = ((item.v_lead > 2.5 || item.oncoming) && item.type < 10000);
  bool long_ignore_condition = (item.d_rel > threshold_dis * 2);

  item.need_pre_brk = 0;
  if (item.is_lead || lat_ignore_condition || long_ignore_condition ||
      ignore_car) {
    item.need_pre_brk = 0;
  } else if (item.d_rel > threshold_dis) {
    item.need_pre_brk = 1;
  } else if (item.d_rel > 0) {
    item.need_pre_brk = 2;
  } else {
    item.need_pre_brk = 0;
  }

  if (item.d_rel < 10 && item.d_rel > -4) {
    if ((item.d_path < 2.7 && item.type < 10000) ||
        (item.d_path < 3.5 && item.type >= 10000)) {
      item.need_limit_acc = true;
    } else {
      item.need_limit_acc = false;
    }
  } else {
    item.need_limit_acc = false;
  }
}

bool TrackletMaintainer::is_potential_lead_one(TrackedObject &item,
                                               double v_ego) {
  LOG_DEBUG("----is_potential_lead_one-----\n");
  double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;
  double gap = (item.last_recv_time == 0.0)
                   ? planning_cycle_time
                   : std::max((item.timestamp - item.last_recv_time),
                              planning_cycle_time);
  LOG_DEBUG("the gap is : [%f]ms \n", gap);
  std::array<double, 5> xp1{1.5, 5.0, 10.0, 40.0, 60.0 + v_ego / 1.2};
  std::array<double, 5> fp1{0.3, 0.4, 0.3, 0.25, 0.0};
  double t_lookahead = interp(item.d_rel, xp1, fp1);

  std::array<double, 3> xp2{1.5, 5.0, 10.0};
  std::array<double, 3> fp2{-0.85, -0.9, -0.85};
  double max_d_offset = interp(item.d_rel, xp2, fp2);

  if (std::fabs(item.d_path_pos) < 3.0 && item.v_lat < -0.2) {
    item.lat_coeff = std::min(item.lat_coeff * 1.035, 1.5);
  } else {
    item.lat_coeff = std::max(item.lat_coeff * 0.9, 1.0);
  }

  double lat_corr =
      std::max(max_d_offset, std::min(0.0, t_lookahead * item.v_lat)) *
      item.lat_coeff;
  LOG_DEBUG("max_d_offset is: [%f], t_lookahead: [%f], lat_corr is: [%f]\n",
            max_d_offset, t_lookahead, lat_corr);
  if (item.oncoming && item.d_rel >= 0) {
    lat_corr = 0.0;
  }

  double lead_d_path_thr;
  std::array<double, 5> xp3{0, 5, 40, 80, 120};
  std::array<double, 5> fp3{1.28, 1.28, 1.21, 1.19, 1.1};
  double result = interp(item.d_rel, xp3, fp3);

  double d_path = std::max(item.d_path_pos + lat_corr, 0.0);
  LOG_DEBUG("item's id is: [%d], item.d_rel is: [%f], item.v_lat is: [%f]\n",
            item.track_id, item.d_rel, item.v_lat);
  LOG_DEBUG("item.d_path_pos is: [%f], lat_corr is: [%f], d_path is : [%f]\n",
            item.d_path_pos, lat_corr, d_path);
  if (item.type < 10000 && item.v_lead > 0.2) {
    lead_d_path_thr = result;
  } else if (item.type < 10000) {
    lead_d_path_thr = result - 0.25;
  } else {
    lead_d_path_thr = result - 0.2;
  }

  if (item.oncoming && item.d_rel > 50) {
    lead_d_path_thr = 0.3;
  }
  LOG_DEBUG("lead_d_path_thr is: [%f]\n", lead_d_path_thr);
  if (hdmap_valid_) {
    item.leadone_confidence_cnt = 0.0;
    item.is_lead = d_path < lead_d_path_thr && item.d_rel >= 0.0;
  } else {
    LOG_DEBUG("the gap is : [%f]ms \n", gap);
    if (d_path < lead_d_path_thr && item.d_rel > 0.0) {
      item.leadone_confidence_cnt =
          std::min(item.leadone_confidence_cnt + gap, 50 * planning_cycle_time);
    } else {
      LOG_DEBUG("!!!!!!!gap is : [%f] \n", gap);
      int count = (int)((gap + 0.01) / planning_cycle_time);
      item.leadone_confidence_cnt = std::max(
          item.leadone_confidence_cnt - 10 * count * planning_cycle_time, 0.0);
      LOG_DEBUG("!!!!!!!leadone_confidence_cnt is : [%f] \n",
                item.leadone_confidence_cnt);
    }

    std::array<double, 5> xp4{0, 30, 60, 90, 120};
    std::array<double, 5> fp4{1, 1, 1, 5, 10};
    std::array<double, 5> fp4_2{1, 1, 1, 1, 1};
    double lead_confidence_thrshld = 1.0;
    if (item.type == 1) {
      lead_confidence_thrshld = interp(item.d_rel, xp4, fp4_2);
    } else {
      lead_confidence_thrshld = interp(item.d_rel, xp4, fp4);
    }
    LOG_DEBUG("lead_confidence_thrshld is : [%f]\n", lead_confidence_thrshld);
    item.is_lead = item.leadone_confidence_cnt >=
                   lead_confidence_thrshld * planning_cycle_time;
  }
  LOG_DEBUG("item.is_lead: [%d]\n", item.is_lead);

  // calculate cutin
  if (hdmap_valid_) {
  } else {
    LOG_DEBUG("----fill_possibility_of_cutin-----\n");
    double ttc = std::max(item.d_path - 1.5, 0.0) / std::max(-item.v_lat, 0.01);
    ttc = std::min(15.0, ttc);

    double new_ttc;
    if (item.last_ttc == std::numeric_limits<double>::min()) {
      new_ttc = ttc;
      item.last_ttc = ttc;
    } else {
      double ttc_decay_rate = 0.5;
      new_ttc = ttc * (1.0 - ttc_decay_rate) + item.last_ttc * ttc_decay_rate;
    }
    double lower_dist_threshold = (item.v_lead > -1.0) ? -10.0 : -5.0;
    double upper_dist_threshold =
        20 + 2 * std::pow(std::min(item.v_rel, 0.0), 2);

    double is_need_consider = (item.v_lat < -0.2) && (new_ttc < 10.0);
    double temp = item.d_rel + new_ttc * item.v_rel;
    double is_in_range =
        (temp > lower_dist_threshold && temp < upper_dist_threshold);

    std::array<double, 5> xp5{1.5, 5.0, 10.0, 40.0, 60.0 + v_ego / 1.2};
    std::array<double, 5> fp5{0.8, 1.0, 0.8, 0.7, 0.0};
    t_lookahead = interp(item.d_rel, xp5, fp5);
    lat_corr = std::max(max_d_offset, std::min(0.0, t_lookahead * item.v_lat)) *
               item.lat_coeff;
    if (item.oncoming && item.d_rel >= 0) {
      lat_corr = 0.0;
    }

    d_path = std::max(item.d_path_pos + lat_corr, 0.0);
    if (d_path < lead_d_path_thr && item.d_rel > 0.0) {
      item.cutin_confidence_cnt =
          std::min(item.cutin_confidence_cnt + gap, 50 * planning_cycle_time);
    } else {
      int count = (int)((gap + 0.01) / planning_cycle_time);
      item.cutin_confidence_cnt = std::max(
          item.cutin_confidence_cnt - 10 * count * planning_cycle_time, 0.0);
      LOG_DEBUG("!!!!!!!cutin_confidence_cnt is : [%f] \n",
                item.cutin_confidence_cnt);
    }

    std::array<double, 5> xp4{0, 30, 60, 90, 120};
    std::array<double, 5> fp4{1, 1, 1, 5, 10};
    std::array<double, 5> fp4_2{1, 1, 1, 1, 1};
    double cutin_confidence_cnt = 1.0;
    if (item.type == 1) {
      cutin_confidence_cnt = interp(item.d_rel, xp4, fp4_2);
    } else {
      cutin_confidence_cnt = interp(item.d_rel, xp4, fp4);
    }
    LOG_DEBUG("cutin_confidence_cnt is : [%f]\n", cutin_confidence_cnt);
    if (is_need_consider && is_in_range) {
      if (item.cutin_confidence_cnt >=
          cutin_confidence_cnt * planning_cycle_time) {
        item.cutinp =
            std::max(0.2, item.cutinp - gap * std::max(item.v_lat, -1.0) /
                                            std::max(item.d_path, 0.01));
      } else {
        item.cutinp = item.cutinp - std::max(0.1, item.v_lat / 3);
      }
    } else {
      item.cutinp = item.cutinp - std::max(0.1, item.v_lat / 3);
    }
    item.last_ttc = new_ttc;
    item.cutinp = std::max(0.0, std::min(1.0, item.cutinp));
    if (item.oncoming) {
      item.cutinp = 0.0;
    }
    LOG_DEBUG("cutin_p is : [%f]\n", item.cutinp);
  }
  return item.is_lead;
}

bool TrackletMaintainer::is_potential_lead_two(TrackedObject &item,
                                               TrackedObject *lead_one) {
  LOG_DEBUG("----is_potential_lead_two-----\n");
  // Only use obstacle fusion with camera
  if (!(item.fusion_source & OBSTACLE_SOURCE_CAMERA)) {
    return false;
  }

  double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;
  if (lead_one == nullptr) {
    return false;
  }

  bool is_diff_d_rel = (std::fabs(item.d_rel - lead_one->d_rel) > 5.0 ||
                        lead_one->type == 20001 || lead_one->type == 10001);
  bool is_diff_v_rel = (std::fabs(item.v_rel - lead_one->v_rel) > 1.0);
  bool is_diff_y_rel = (std::fabs(item.y_rel - lead_one->y_rel) > 1.0);

  if (hdmap_valid_) {
    item.leadtwo_confidence_cnt = 0.0;
    return (is_diff_d_rel || is_diff_v_rel || is_diff_y_rel) &&
           item.d_rel > 0.0;
  } else {
    double gap = (item.last_recv_time == 0.0)
                     ? planning_cycle_time
                     : (item.timestamp - item.last_recv_time);

    if ((is_diff_d_rel || is_diff_v_rel || is_diff_y_rel) && item.d_rel > 0.0) {
      item.leadtwo_confidence_cnt =
          std::min(item.leadtwo_confidence_cnt + gap, 50 * planning_cycle_time);
    } else {
      int count = (int)((gap + 0.01) / planning_cycle_time);
      item.leadtwo_confidence_cnt = std::max(
          item.leadtwo_confidence_cnt - 10 * count * planning_cycle_time, 0.0);
    }

    std::array<double, 5> xp{0, 30, 60, 90, 120};
    std::array<double, 5> fp{2, 4, 6, 10, 50};
    double lead_confidence_time = interp(item.d_rel, xp, fp);

    return item.leadtwo_confidence_cnt >=
           lead_confidence_time * planning_cycle_time;
  }
}

bool TrackletMaintainer::is_potential_temp_lead_one(TrackedObject &item,
                                                    double v_ego,
                                                    bool refline_update) {
  // Only use obstacle fusion with camera
  if (!(item.fusion_source & OBSTACLE_SOURCE_CAMERA)) {
    item.is_temp_lead = false;
    return item.is_temp_lead;
  }
  LOG_DEBUG("----is_potential_temp_lead_one-----\n");
  double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;
  std::array<double, 5> xp1{1.5, 5.0, 10.0, 40.0, 60.0 + v_ego / 1.2};
  std::array<double, 5> fp1{0.8, 1.0, 0.8, 0.7, 0.0};
  double t_lookahead = interp(item.d_rel, xp1, fp1);

  std::array<double, 3> xp2{1.5, 5.0, 10.0};
  std::array<double, 3> fp2{-0.85, -0.9, -0.85};
  double max_d_offset = interp(item.d_rel, xp2, fp2);

  double lat_corr =
      std::max(max_d_offset, std::min(0.0, t_lookahead * item.v_lat_self));
  if (item.oncoming && item.d_rel >= 0) {
    lat_corr = 0.0;
  }
  LOG_DEBUG("v_lat_self is: [%f], v_lat: [%f]\n", item.v_lat_self, item.v_lat);
  LOG_DEBUG("max_d_offset is: [%f], t_lookahead: [%f], lat_corr is: [%f]\n",
            max_d_offset, t_lookahead, lat_corr);
  double d_path_self = std::max(item.d_path_self_pos + lat_corr, 0.0);
  item.d_path_self_ori =
      std::max(std::fabs(item.y_rel_ori - item.dy_self) + lat_corr, 0.0);

  double lead_d_path_thr;
  std::array<double, 5> xp3{0, 5, 40, 80, 120};
  std::array<double, 5> fp3{1.28, 1.28, 1.21, 1.19, 1.1};
  double result = interp(item.d_rel, xp3, fp3);

  if (item.type < 10000 && item.v_lead > 0.2) {
    lead_d_path_thr = result - 0.15;
  } else if (item.type < 10000) {
    lead_d_path_thr = result - 0.25;
  } else {
    lead_d_path_thr = result - 0.2;
  }

  if (item.oncoming && item.d_rel > 50) {
    lead_d_path_thr = 0.3;
  }

  bool is_get_threshold = (d_path_self < lead_d_path_thr);
  bool is_in_range = false;
  if (refline_update) {
    is_in_range =
        ((item.l > item.l0 && item.l < 0) || (item.l < item.l0 && item.l > 0));
  } else {
    is_in_range =
        ((item.y_rel_ori > item.dy_self && item.y_rel_ori < item.dy) ||
         (item.y_rel_ori < item.dy_self && item.y_rel_ori > item.dy));
  }
  LOG_DEBUG("item's id is: [%d], item.d_rel is: [%f], item.v_lat is: [%f]\n",
            item.track_id, item.d_rel, item.v_lat);
  LOG_DEBUG(
      "item.d_path_self_pos is: [%f], lat_corr is: [%f], d_path_self is : "
      "[%f]\n",
      item.d_path_self_pos, lat_corr, d_path_self);
  LOG_DEBUG(
      "item.l is: [%f], l0 is: [%f], y_rel_ori is : [%f], dy_self is : [%f]\n",
      item.l, item.l0, item.y_rel_ori, item.dy_self);

  if (hdmap_valid_) {
    item.tleadone_confidence_cnt = 0.0;
    item.is_temp_lead = (is_get_threshold || is_in_range) && item.d_rel >= 0.0;
  } else {
    double gap = (item.last_recv_time == 0.0)
                     ? planning_cycle_time
                     : (item.timestamp - item.last_recv_time);

    if ((is_get_threshold || is_in_range) && item.d_rel >= 0.0) {
      item.tleadone_confidence_cnt = std::min(
          item.tleadone_confidence_cnt + gap, 50 * planning_cycle_time);
    } else {
      int count = (int)((gap + 0.01) / planning_cycle_time);
      item.tleadone_confidence_cnt = std::max(
          item.tleadone_confidence_cnt - 10 * count * planning_cycle_time, 0.0);
    }

    std::array<double, 5> xp4{0, 30, 60, 90, 120};
    std::array<double, 5> fp4{1, 1, 1, 10, 50};
    double lead_confidence_time = interp(item.d_rel, xp4, fp4);

    item.is_temp_lead = item.tleadone_confidence_cnt >=
                        lead_confidence_time * planning_cycle_time;
  }
  LOG_DEBUG("item.is_temp_lead: [%d]\n", item.is_temp_lead);
  return item.is_temp_lead;
}

bool TrackletMaintainer::is_potential_temp_lead_two(
    TrackedObject &item, TrackedObject *temp_lead_one) {
  LOG_DEBUG("----is_potential_temp_lead_two-----\n");
  double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;
  if (temp_lead_one == nullptr) {
    return false;
  }
  // Only use obstacle fusion with camera
  if (!(item.fusion_source & OBSTACLE_SOURCE_CAMERA)) {
    return false;
  }

  bool is_diff_d_rel =
      (std::fabs(item.d_rel - temp_lead_one->d_rel) > 5.0 ||
       temp_lead_one->type == 20001 || temp_lead_one->type == 10001);
  bool is_diff_v_rel = (std::fabs(item.v_rel - temp_lead_one->v_rel) > 1.0);
  bool is_diff_y_rel = (std::fabs(item.y_rel - temp_lead_one->y_rel) > 1.0);

  if (hdmap_valid_) {
    item.tleadtwo_confidence_cnt = 0.0;
    return (is_diff_d_rel || is_diff_v_rel || is_diff_y_rel) &&
           item.d_rel > 0.0;
  } else {
    double gap = (item.last_recv_time == 0.0)
                     ? planning_cycle_time
                     : (item.timestamp - item.last_recv_time);

    if ((is_diff_d_rel || is_diff_v_rel || is_diff_y_rel) && item.d_rel > 0.0) {
      item.tleadtwo_confidence_cnt = std::min(
          item.tleadtwo_confidence_cnt + gap, 50 * planning_cycle_time);
    } else {
      int count = (int)((gap + 0.01) / planning_cycle_time);
      item.tleadtwo_confidence_cnt = std::max(
          item.tleadtwo_confidence_cnt - 10 * count * planning_cycle_time, 0.0);
    }

    std::array<double, 5> xp{0, 30, 60, 90, 120};
    std::array<double, 5> fp{2, 2, 2, 20, 80};
    double lead_confidence_time = interp(item.d_rel, xp, fp);

    return item.tleadtwo_confidence_cnt >
           lead_confidence_time * planning_cycle_time;
  }
}

bool TrackletMaintainer::is_potential_avoiding_car(
    TrackedObject &item, TrackedObject *lead_one, double v_ego,
    double lane_width, int scenario, bool borrow_bicycle_lane,
    double dist_rblane, bool tleft_lane, bool rightest_lane,
    double dist_intersect, double intersect_length, bool isRedLightStop) {
  LOG_DEBUG("----is_potential_avoiding_car-----\n");
  double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;
  item.is_ncar = false;
  double ego_car_width = 2.2;
  double lat_safety_buffer = 0.2;
  // double l_ego = ego_state_->ego_frenet.y;
  // TODO: ego_state relative
  double l_ego = 0.;
  double dist_limit;

  std::array<double, 3> xp{20, 40, 60};
  std::array<double, 3> fp{0.3, 0.12, 0.09};
  double near_car_d_lane_thr = interp(item.d_rel, xp, fp);

  bool is_not_full_in_road = (std::fabs(item.y_rel) > 0.0);
  bool is_in_range = (item.d_rel < 20.0 && item.v_rel < 1.0);
  bool is_about_to_enter_range =
      (item.d_rel < std::min(std::fabs(15.0 * item.v_rel), 60.0) &&
       item.v_rel < -2.5);
  bool cross_solid_line = false;

  if (scenario != LocationEnum::LOCATION_INTER) {
    if (is_not_full_in_road && (is_in_range || is_about_to_enter_range)) {
      if (item.d_min_cpath != DBL_MAX && item.d_max_cpath != DBL_MAX) {
        if (item.type != 20001) {
          dist_limit = lane_width * 0.5 + near_car_d_lane_thr;
        } else {
          dist_limit = lane_width * 0.5 - 0.2;
        }
        bool is_same_side = ((item.d_min_cpath > 0 && item.d_max_cpath > 0) ||
                             (item.d_min_cpath <= 0 && item.d_max_cpath <= 0));

        bool is_need_avoid =
            (item.d_max_cpath < 0 &&
             std::fabs(item.d_max_cpath) < dist_limit) ||
            (item.d_min_cpath > 0 && item.d_min_cpath < dist_limit) ||
            (borrow_bicycle_lane && item.d_max_cpath > 0 &&
             item.d_min_cpath < 0 && item.v_lead < 0.5);

        bool can_avoid =
            (item.d_min_cpath >
             std::min(((ego_car_width + lat_safety_buffer) - lane_width / 2),
                      0.9)) ||
            (item.d_max_cpath <
             std::max((lane_width / 2 - (ego_car_width + lat_safety_buffer)),
                      -0.9)) ||
            (dist_rblane > 0 &&
             ((lane_width / 2 + item.d_min_cpath + dist_rblane >= 2.2 &&
               borrow_bicycle_lane && item.v_lead < 0.2) ||
              lane_width / 2 - item.d_max_cpath >= 2.4));

        if (dist_intersect == 1000 && lane_width > 5. && lead_one != nullptr &&
            !lead_one->is_accident_car) {
          can_avoid = item.d_min_cpath > 0.9 || item.d_max_cpath < -0.5;
        }

        cross_solid_line =
            (((item.d_min_cpath <=
               std::min(((ego_car_width + lat_safety_buffer) - lane_width / 2),
                        0.9)) &&
              ((item.d_min_cpath >= 0 && !rightest_lane && item.type < 10000) ||
               (item.d_max_cpath > 0 && item.type == 20001))) ||
             ((item.d_max_cpath >=
               std::max((lane_width / 2 - (ego_car_width + lat_safety_buffer)),
                        -0.9)) &&
              ((item.d_max_cpath <= 0 && !tleft_lane && item.type < 10000) ||
               (item.d_min_cpath < 0 && item.type == 20001)))) &&
            lead_one != nullptr && item.track_id == lead_one->track_id &&
            item.v_lead < 0.3 && abs(item.v_lat) < 0.2 &&
            dist_intersect - item.d_rel >= -5 &&
            dist_intersect - item.d_rel < 50.0 &&
            (!isRedLightStop || item.type == 20001);

        item.is_ncar =
            (is_same_side && is_need_avoid && can_avoid) ||
            (can_avoid && borrow_bicycle_lane && item.d_max_cpath > 0 &&
             item.d_max_cpath < dist_limit + 2.2 && item.v_lead < 0.5) ||
            (rightest_lane && item.d_max_cpath < 0 &&
             std::fabs(item.d_max_cpath) < dist_limit && item.v_lead < 0.5) ||
            cross_solid_line;
      }
    }
  } else {
    if (is_not_full_in_road && (is_in_range || is_about_to_enter_range)) {
      if (item.d_min_cpath != DBL_MAX && item.d_max_cpath != DBL_MAX) {
        if (item.type != 20001) {
          dist_limit = 1.8 + near_car_d_lane_thr;
        } else {
          dist_limit = 1.6;
        }
        bool is_same_side = ((item.d_min_cpath > 0 && item.d_max_cpath > 0) ||
                             (item.d_min_cpath <= 0 && item.d_max_cpath <= 0));
        bool is_not_intersect = (item.trajectory.intersection == 0);
        bool is_need_avoid =
            (item.d_max_cpath < 0.65 &&
             std::fabs(item.d_max_cpath) < dist_limit) ||
            (item.d_min_cpath > -0.65 && item.d_min_cpath < dist_limit) ||
            (item.d_path < dist_limit);

        bool static_can_avoid = (item.d_min_cpath - l_ego > -0.6 ||
                                 l_ego - item.d_max_cpath > -0.6);
        bool dynamic_can_avoid =
            (item.d_min_cpath - l_ego > 0.0 || l_ego - item.d_max_cpath > 0.0);
        bool leftlane_can_avoid = (item.d_min_cpath - l_ego > -0.6 ||
                                   l_ego - item.d_max_cpath > -0.5);
        bool accid_avoid = (item.v_lead < 0.5);

        item.is_ncar = (is_same_side && is_need_avoid && dynamic_can_avoid &&
                        is_not_intersect) ||
                       (((is_need_avoid && accid_avoid && static_can_avoid &&
                          !tleft_lane) ||
                         (is_need_avoid && accid_avoid && leftlane_can_avoid &&
                          tleft_lane)) &&
                        item.d_rel - dist_intersect <= intersect_length + 5.);
      }
    }
  }

  double ncar_count;
  if (item.type < 10000) {
    if (item.d_rel >= 20) {
      std::array<double, 2> xp2{-7.5, -2.5};
      std::array<double, 2> fp2{5, 20};
      ncar_count = interp(item.v_rel, xp2, fp2);
    } else {
      std::array<double, 4> xp2{-5, -2.499, 0, 1};
      std::array<double, 4> fp2{2, 3, 4, 20};
      std::array<double, 4> xp3{0, 2, 5, 10};
      std::array<double, 4> fp3{4, 3, 2, 0};

      ncar_count = interp(item.v_rel, xp2, fp2) + interp(item.v_ego, xp3, fp3);
    }
  } else if (item.type != 20001) {
    std::array<double, 2> xp2{20, 40};
    std::array<double, 2> fp2{5, 10};
    ncar_count = interp(item.d_rel, xp2, fp2);
  } else {
    ncar_count = 1.0;
  }
  if (cross_solid_line) {
    std::array<double, 4> xp4{0, 0.4, 0.8};
    std::array<double, 4> fp4{30, 20, 5};
    ncar_count += interp(
        std::min(std::fabs(item.d_min_cpath), std::fabs(item.d_max_cpath)), xp4,
        fp4);
  }

  double gap = (item.last_recv_time == 0.0)
                   ? planning_cycle_time
                   : (item.timestamp - item.last_recv_time);
  int count = (int)((gap + 0.01) / planning_cycle_time);

  if (item.is_ncar) {
    if (item.trajectory.intersection == 0 ||
        (item.type < 10000 && std::fabs(item.v_lead) < 0.5 &&
         item.v_lat > -0.2 && item.v_lat < 0.3) ||
        (item.type > 10000 && (std::fabs(item.v_lat) < 0.3)) ||
        borrow_bicycle_lane || rightest_lane) {
      if ((item.v_lat > -0.5 && item.v_lat < 0.3 && item.type < 10000) ||
          (std::fabs(item.v_lat) < 0.3 && item.type > 10000)) {
        item.ncar_count =
            std::min(item.ncar_count + gap, 100 * planning_cycle_time);
      }
    } else {
      item.ncar_count = std::max(
          item.ncar_count - 5 * (int)(std::fabs(item.v_lat) / 0.3 + 0.5) *
                                count * planning_cycle_time,
          0.0);
    }

    if (item.ncar_count < ncar_count * planning_cycle_time &&
        item.timestamp > item.close_time + 2.5) {
      item.is_ncar = false;
      return false;
    } else if (item.ncar_count >= ncar_count * planning_cycle_time) {
      if (item.ncar_count_in == false) {
        item.ncar_count = 100 * planning_cycle_time;
      }

      item.close_time = item.timestamp;
      return true;
    }
  } else {
    item.ncar_count =
        std::max(item.ncar_count - 2 * count * planning_cycle_time, 0.0);
    if (item.v_rel > 1.5) {
      item.ncar_count =
          std::max(item.ncar_count - 5 * count * planning_cycle_time, 0.0);
    }

    if (item.trajectory.intersection > 0) {
      item.ncar_count =
          std::max(item.ncar_count - 10 * count * planning_cycle_time, 0.0);
    }

    if (item.d_max_cpath > 0 && item.d_min_cpath < 0) {
      item.ncar_count = 0;
    }

    if (item.ncar_count > 60 * planning_cycle_time) {
      item.ncar_count_in = true;
      return true;
    } else {
      item.ncar_count = 0;
      item.ncar_count_in = false;
      return false;
    }
  }

  return false;
}

bool TrackletMaintainer::is_leadone_potential_avoiding_car(
    TrackedObject *lead_one, int scenario, double lane_width,
    bool borrow_bicycle_lane, bool rightest_lane, double dist_intersect,
    bool isRedLightStop) {
  if (lead_one == nullptr) {
    return false;
  }

  bool is_in_range = (lead_one->d_rel < 20.0 && lead_one->v_rel < 1.0);
  bool is_about_to_enter_range =
      (lead_one->d_rel < std::min(std::fabs(15.0 * lead_one->v_rel), 60.0)) &&
      lead_one->v_rel < -2.5;

  if (is_in_range || is_about_to_enter_range) {
    double ego_car_width = 2.2;
    double near_end_pos = 0.5 * lane_width - 0.7 * (lane_width - ego_car_width);
    double far_end_pos = 0.5 * lane_width + 0.2;

    bool is_in_avoid_range_by_nearest_point =
        lead_one->d_path >= near_end_pos && lead_one->d_path < far_end_pos;

    bool is_in_avoid_range_by_nearest_line_in_left =
        lead_one->d_min_cpath >= near_end_pos &&
        lead_one->d_min_cpath < far_end_pos &&
        lead_one->d_max_cpath >= near_end_pos;

    bool is_in_avoid_range_by_nearest_line_in_right =
        lead_one->d_max_cpath > -far_end_pos &&
        lead_one->d_max_cpath <= -near_end_pos &&
        lead_one->d_min_cpath <= -near_end_pos;

    lead_one->is_avd_car =
        (lead_one->is_avd_car) &&
        (is_in_avoid_range_by_nearest_point ||
         is_in_avoid_range_by_nearest_line_in_left ||
         is_in_avoid_range_by_nearest_line_in_right || borrow_bicycle_lane ||
         scenario == LocationEnum::LOCATION_INTER || rightest_lane ||
         (dist_intersect - lead_one->d_rel < 50 &&
          dist_intersect - lead_one->d_rel >= -5 &&
          (!isRedLightStop || lead_one->type == 20001)));
  }

  return lead_one->is_avd_car;
}

void TrackletMaintainer::select_lead_cars(
    const std::vector<TrackedObject *> &tracked_objects, LeadCars &lead_cars) {
  TrackedObject *lead_one = nullptr;
  TrackedObject *lead_two = nullptr;
  TrackedObject *temp_lead_one = nullptr;
  TrackedObject *temp_lead_two = nullptr;

  double v_ego = ego_state_->ego_v();

  for (auto tr : tracked_objects) {
    if (tr->is_lead == false) {
      continue;
    }

    if (lead_one == nullptr || tr->d_rel < lead_one->d_rel) {
      lead_one = tr;
    }
  }

  for (auto tr : tracked_objects) {
    if (tr->is_lead == false || lead_one == tr ||
        is_potential_lead_two(*tr, lead_one) == false) {
      continue;
    }

    if (lead_two == nullptr || tr->d_rel < lead_two->d_rel) {
      lead_two = tr;
    }
  }

  for (auto tr : tracked_objects) {
    if (tr->is_lead == true ||
        is_potential_temp_lead_one(*tr, v_ego, frenet_coord_ != nullptr) ==
            false) {
      continue;
    }

    if (temp_lead_one == nullptr || tr->d_rel < temp_lead_one->d_rel) {
      temp_lead_one = tr;
    }
  }

  for (auto tr : tracked_objects) {
    if (tr->is_lead == true || temp_lead_one == tr ||
        is_potential_temp_lead_one(*tr, v_ego, frenet_coord_ != nullptr) ==
            false ||
        is_potential_temp_lead_two(*tr, temp_lead_one) == false) {
      continue;
    }

    if (temp_lead_two == nullptr || tr->d_rel < temp_lead_two->d_rel) {
      temp_lead_two = tr;
    }
  }

  lead_cars.lead_one = lead_one;
  lead_cars.lead_two = lead_two;
  lead_cars.temp_lead_one = temp_lead_one;
  lead_cars.temp_lead_two = temp_lead_two;
}

void TrackletMaintainer::set_default_value(
    const std::vector<TrackedObject *> &tracked_objects) {
  for (auto tr : tracked_objects) {
    if (tr->d_center_cpath == DBL_MAX) {
      tr->d_center_cpath = 100;
    }

    if (tr->d_center_cpathk == DBL_MAX) {
      tr->d_center_cpathk = 100;
    }

    if (tr->v_center_cpathk == DBL_MAX) {
      tr->v_center_cpathk = 100;
    }

    if (tr->d_max_cpath == DBL_MAX) {
      tr->d_max_cpath = 100;
    }

    if (tr->d_min_cpath == DBL_MAX) {
      tr->d_min_cpath = 100;
    }

    if (tr->y_rel_ori == DBL_MAX) {
      tr->y_center_rel = tr->y_rel;
    } else {
      tr->y_center_rel = tr->y_rel_ori;
    }

    if (tr->timestamp == DBL_MAX) {
      tr->timestamp = 0;
    }

    if (tr->length == DBL_MAX) {
      tr->length = 5.0;
    }

    if (tr->width == DBL_MAX) {
      tr->width = 2.2;
    }

    if (tr->theta == DBL_MAX) {
      tr->theta = 0.0;
    }
  }
}

}  // namespace planning
