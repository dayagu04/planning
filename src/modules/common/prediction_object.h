#ifndef PREDICTION_OBJECT_H
#define PREDICTION_OBJECT_H

#include <string>
#include <vector>

#include "common.h"
#include "common_c.h"
#include "config/basic_type.h"

namespace planning {

struct PredictionTrajectoryPoint {
  double relative_time{0.0};
  float x{0.0};
  float y{0.0};
  float yaw{0.0};  // for obs
  float speed{0.0};
  float acc{0.0};
  float theta{0.0};  // for speed
  float prob{1.0};

  float std_dev_x{0.0};
  float std_dev_y{0.0};
  float std_dev_yaw{0.0};
  float std_dev_speed{0.0};

  float relative_ego_x{0.0};
  float relative_ego_y{0.0};
  float relative_ego_yaw{0.0};
  float relative_ego_speed{0.0};
  float relative_ego_acc_x{0.0};
  float relative_ego_acc_y{0.0};

  float relative_ego_std_dev_x{0.0};
  float relative_ego_std_dev_y{0.0};
  float relative_ego_std_dev_yaw{0.0};
  float relative_ego_std_dev_speed{0.0};
};

struct PredictionTrajectory {
  float prob{1.0};
  // float prediction_interval;
  // unsigned int num_of_points;
  std::string intention{"none"};
  std::string source{"none"};
  // bool b_backup_freemove{false};
  bool b_valid_sigma{false};
  // std::vector<std::string> reserved;

  float prediction_interval{0.0};
  unsigned int num_of_points{41};

  // object kinematics state probability
  float const_vel_prob{1.0};   // constant velocity probability
  float const_acc_prob{1.0};   // constant acceleration probability
  float still_prob{1.0};       // still probability
  float coord_turn_prob{0.0};  // coordinated turn (CT) kinematics probability

  bool b_minor_modal{false};

  std::vector<PredictionTrajectoryPoint> trajectory;
};

struct PredictionObject {
  uint id;
  iflyauto::ObjectType type;
  uint fusion_source;
  double timestamp_us;
  double delay_time{0.0};
  ObstacleIntentType intention;
  bool b_backup_freemove{false};
  double cutin_score;
  float position_x;
  float position_y;
  float length;
  float width;
  float speed;
  float speed_fusion;
  float yaw;    // for obj
  float theta;  // for velocity
  float acc;
  float acc_fusion;
  iflyauto::ObjectMotionType motion_pattern_current;
  bool is_oversize_vehicle{false};
  bool is_VRU{false};
  bool is_traffic_facilities{false};
  bool is_car{false};
  bool is_static{false};
  // add relative info for highway
  float relative_position_x;
  float relative_position_y;
  float relative_speed_x;
  float relative_speed_y;
  float relative_acceleration_x;
  float relative_acceleration_y;
  float acceleration_relative_to_ground_x;
  float acceleration_relative_to_ground_y;
  float relative_theta;
  std::vector<Point3d> bottom_polygon_points;
  std::vector<Point3d> top_polygon_points;
  bool trajectory_valid{false};

  std::vector<PredictionTrajectory> trajectory_array;
};

static constexpr int kPredictionHashBit = 100000000;
int hash_prediction_id(int perception_id, int traj_index);
int inverse_hash_prediction_id(int prediction_id);

}  // namespace planning
#endif
