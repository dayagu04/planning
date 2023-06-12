#ifndef PLANNING_COMMON_TRACKED_OBJECT_H_
#define PLANNING_COMMON_TRACKED_OBJECT_H_

#include <float.h>
#include <vector>
#include "define/geometry.h"

namespace planning {

struct PredictionInfo {
  double prob;
  double interval;
  int num_of_points;
  double const_vel_prob;
  double const_acc_prob;
  double still_prob;
  double coord_turn_prob;
};

struct PredictionTrajectoryEx {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::vector<double> speed;

  std::vector<double> std_dev_x;
  std::vector<double> std_dev_y;
  std::vector<double> std_dev_yaw;
  std::vector<double> std_dev_speed;

  std::vector<double> relative_ego_x;
  std::vector<double> relative_ego_y;
  std::vector<double> relative_ego_yaw;
  std::vector<double> relative_ego_speed;

  std::vector<double> relative_ego_std_dev_x;
  std::vector<double> relative_ego_std_dev_y;
  std::vector<double> relative_ego_std_dev_yaw;
  std::vector<double> relative_ego_std_dev_speed;

  int intersection = 0;
};

struct TrackedObject {
  double timestamp = 0;

  int track_id = 0;
  int type = 0;
  int fusion_type = 0;
  int position_type = 0;

  double length = 0;
  double width = 0;
  double height = 0;

  double center_x = 0;
  double center_y = 0;
  double s = 0;
  double l = 0;
  double theta = 0;
  double speed_yaw = 0;

  double a = 0;
  double v = 0;
  double v_lead = 0;
  double v_lead_k = 0;
  double v_lead_raw = 0;
  double v_rel = 0;
  double vy_rel = 0;
  double vy_abs = 0;
  double a_rel = 0;
  double a_lead = 0;
  double a_lead_k = 0;
  double y_rel_ori = 0;

  double l0 = 0;
  double c0 = 0;
  double c_path = 0;
  double d0 = 0;
  double d_rel = 0;
  double y_rel = 0;
  double dy = 0;
  double dy_self = 0;
  double d_path = 0;
  double d_path_raw = 0;
  double d_path_pos = 0;
  double d_path_self = 0;
  double d_path_self_pos = 0;
  double d_path_self_ori = 0;
  double d_center_cpath = DBL_MAX;
  double d_center_cpathk = DBL_MAX;
  double v_center_cpathk = DBL_MAX;
  double d_max_cpath = DBL_MAX;
  double d_min_cpath = DBL_MAX;
  double v_lat = 0;
  double v_lat_self = 0;
  double s_center = 0;
  double s_max = 0;
  double s_min = 0;
  double vs_rel = 0;

  double close_time = 0;
  double last_ttc = 0;
  double is_accident_cnt = 0;
  bool is_accident_car = false;
  bool is_avd_car = false;
  bool is_ncar = false;
  bool ncar_count_in = false;
  double ncar_count = 0;
  double lat_coeff = 1.0;
  bool oncoming = false;
  bool stationary = false;

  bool is_lead = false;
  bool is_temp_lead = false;
  double leadone_confidence_cnt = 0;
  double leadtwo_confidence_cnt = 0;
  double tleadone_confidence_cnt = 0;
  double tleadtwo_confidence_cnt = 0;

  double last_recv_time = 0;

  double v_ego = 0;
  double y_center_rel = 0;

  int need_pre_brk = 0;
  bool need_limit_acc = false;

  double cutinp = 0.0;

  // fisheye related for cutin
  Point2D points_3d_f = {DBL_MAX, DBL_MAX};
  Point2D points_3d_r = {DBL_MAX, DBL_MAX};
  double location_head = DBL_MAX;
  double location_tail = DBL_MAX;
  double y_min = DBL_MAX;
  double y_x0 = 0.0;

  // history results
  bool has_history = false;
  double c0_history = DBL_MAX;
  double d_center_cpath_hostory = DBL_MAX;

  PredictionInfo prediction;
  PredictionTrajectoryEx trajectory;
};

struct LeadCars {
  LeadCars() {
    lead_one = nullptr;
    lead_two = nullptr;
    temp_lead_one = nullptr;
    temp_lead_two = nullptr;
  }

  LeadCars(const LeadCars &source) {
    if (source.lead_one != nullptr) {
      lead_one = new TrackedObject(*source.lead_one);
    }

    if (source.lead_two != nullptr) {
      lead_two = new TrackedObject(*source.lead_two);
    }

    if (source.temp_lead_one != nullptr) {
      temp_lead_one = new TrackedObject(*source.temp_lead_one);
    }

    if (source.temp_lead_two != nullptr) {
      temp_lead_two = new TrackedObject(*source.temp_lead_two);
    }
  }

  LeadCars &operator=(const LeadCars &source) {
    clear();

    if (source.lead_one != nullptr) {
      lead_one = new TrackedObject(*source.lead_one);
    }

    if (source.lead_two != nullptr) {
      lead_two = new TrackedObject(*source.lead_two);
    }

    if (source.temp_lead_one != nullptr) {
      temp_lead_one = new TrackedObject(*source.temp_lead_one);
    }

    if (source.temp_lead_two != nullptr) {
      temp_lead_two = new TrackedObject(*source.temp_lead_two);
    }

    return *this;
  }

  void clear() {
    if (lead_one != nullptr) {
      delete lead_one;
      lead_one = nullptr;
    }

    if (lead_two != nullptr) {
      delete lead_two;
      lead_two = nullptr;
    }

    if (temp_lead_one != nullptr) {
      delete temp_lead_one;
      temp_lead_one = nullptr;
    }

    if (temp_lead_two != nullptr) {
      delete temp_lead_two;
      temp_lead_two = nullptr;
    }
  }

  TrackedObject *lead_one;
  TrackedObject *lead_two;
  TrackedObject *temp_lead_one;
  TrackedObject *temp_lead_two;
};

} // namespace planning

#endif
