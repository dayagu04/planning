#pragma once

#include "base_convert.h"
#include "c/prediction_c.h"
#include "struct_convert/common_c.h"
#include "struct_convert/fusion_objects_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::GaussianInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.sigma_x, ros_v.sigma_x, type);
  convert(struct_v.sigma_y, ros_v.sigma_y, type);
  convert(struct_v.correlation, ros_v.correlation, type);
  convert(struct_v.area_probability, ros_v.area_probability, type);
  convert(struct_v.ellipse_a, ros_v.ellipse_a, type);
  convert(struct_v.ellipse_b, ros_v.ellipse_b, type);
  convert(struct_v.theta_a, ros_v.theta_a, type);
}

template <typename T2>
void convert(iflyauto::PredictionTrajectoryPoint &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.confidence, ros_v.confidence, type);
  convert(struct_v.position, ros_v.position, type);
  convert(struct_v.yaw, ros_v.yaw, type);
  convert(struct_v.theta_vel, ros_v.theta_vel, type);
  convert(struct_v.velocity, ros_v.velocity, type);
  convert(struct_v.relative_position, ros_v.relative_position, type);
  convert(struct_v.relative_velocity, ros_v.relative_velocity, type);
  convert(struct_v.relative_yaw, ros_v.relative_yaw, type);
  convert(struct_v.relative_theta_vel, ros_v.relative_theta_vel, type);
  convert(struct_v.gaussian_info, ros_v.gaussian_info, type);
}

template <typename T2>
void convert(iflyauto::PredictionTrajectory &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.confidence, ros_v.confidence, type);
  convert(struct_v.time_stamp, ros_v.time_stamp, type);
  convert(struct_v.relative_time, ros_v.relative_time, type);
  convert(struct_v.trajectory_point_size, ros_v.trajectory_point_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.trajectory_point_size >= 0 && struct_v.trajectory_point_size <= PREDICTION_TRAJ_POINT_NUM) {
      ros_v.trajectory_point.resize(struct_v.trajectory_point_size);
    } else {
      std::cout << "convert/prediction_c.h:" << __LINE__ 
                << " [convert][TO_ROS] trajectory_point_size=" << struct_v.trajectory_point_size 
                << " not in range PREDICTION_TRAJ_POINT_NUM=" << PREDICTION_TRAJ_POINT_NUM 
                << std::endl;
      ros_v.trajectory_point_size = PREDICTION_TRAJ_POINT_NUM;
      ros_v.trajectory_point.resize(PREDICTION_TRAJ_POINT_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.trajectory_point.size(); i0++) {
      convert(struct_v.trajectory_point[i0], ros_v.trajectory_point[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.trajectory_point_size > PREDICTION_TRAJ_POINT_NUM || ros_v.trajectory_point_size < 0 || ros_v.trajectory_point.size() > PREDICTION_TRAJ_POINT_NUM) {
      std::cout << "convert/prediction_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] trajectory_point_size=" << ros_v.trajectory_point_size 
                << " ros_v.trajectory_point.size()=" << ros_v.trajectory_point.size()
                << " not in range PREDICTION_TRAJ_POINT_NUM=" << PREDICTION_TRAJ_POINT_NUM 
                << std::endl;
    }
    if (ros_v.trajectory_point.size() > PREDICTION_TRAJ_POINT_NUM) {
      for (size_t i0 = 0; i0 < PREDICTION_TRAJ_POINT_NUM; i0++) {
        convert(struct_v.trajectory_point[i0], ros_v.trajectory_point[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.trajectory_point.size(); i0++) {
        convert(struct_v.trajectory_point[i0], ros_v.trajectory_point[i0], type);
      }
    }
  }
  //
}

template <typename T2>
void convert(iflyauto::ObstacleIntent &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.type, ros_v.type, type);
}

template <typename T2>
void convert(iflyauto::PredictionObject &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.fusion_obstacle, ros_v.fusion_obstacle, type);
  convert(struct_v.trajectory, ros_v.trajectory, type);
  convert(struct_v.obstacle_intent, ros_v.obstacle_intent, type);
}

template <typename T2>
void convert(iflyauto::PredictionResult &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.prediction_obstacle_list_size, ros_v.prediction_obstacle_list_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.prediction_obstacle_list_size >= 0 && struct_v.prediction_obstacle_list_size <= PREDICTION_OBSTACLE_MAX_NUM) {
      ros_v.prediction_obstacle_list.resize(struct_v.prediction_obstacle_list_size);
    } else {
      std::cout << "convert/prediction_c.h:" << __LINE__ 
                << " [convert][TO_ROS] prediction_obstacle_list_size=" << struct_v.prediction_obstacle_list_size 
                << " not in range PREDICTION_OBSTACLE_MAX_NUM=" << PREDICTION_OBSTACLE_MAX_NUM 
                << std::endl;
      ros_v.prediction_obstacle_list_size = PREDICTION_OBSTACLE_MAX_NUM;
      ros_v.prediction_obstacle_list.resize(PREDICTION_OBSTACLE_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.prediction_obstacle_list.size(); i0++) {
      convert(struct_v.prediction_obstacle_list[i0], ros_v.prediction_obstacle_list[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.prediction_obstacle_list_size > PREDICTION_OBSTACLE_MAX_NUM || ros_v.prediction_obstacle_list_size < 0 || ros_v.prediction_obstacle_list.size() > PREDICTION_OBSTACLE_MAX_NUM) {
      std::cout << "convert/prediction_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] prediction_obstacle_list_size=" << ros_v.prediction_obstacle_list_size 
                << " ros_v.prediction_obstacle_list.size()=" << ros_v.prediction_obstacle_list.size()
                << " not in range PREDICTION_OBSTACLE_MAX_NUM=" << PREDICTION_OBSTACLE_MAX_NUM 
                << std::endl;
    }
    if (ros_v.prediction_obstacle_list.size() > PREDICTION_OBSTACLE_MAX_NUM) {
      for (size_t i0 = 0; i0 < PREDICTION_OBSTACLE_MAX_NUM; i0++) {
        convert(struct_v.prediction_obstacle_list[i0], ros_v.prediction_obstacle_list[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.prediction_obstacle_list.size(); i0++) {
        convert(struct_v.prediction_obstacle_list[i0], ros_v.prediction_obstacle_list[i0], type);
      }
    }
  }
  //
}

