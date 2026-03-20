#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/GaussianInfo.h"
#include "struct_msgs_v2_10/GaussianInfo.h"
#include "struct_msgs/ObstacleIntent.h"
#include "struct_msgs_v2_10/ObstacleIntent.h"
#include "struct_msgs/PredictionObject.h"
#include "struct_msgs_v2_10/PredictionObject.h"
#include "struct_msgs/PredictionResult.h"
#include "struct_msgs_v2_10/PredictionResult.h"
#include "struct_msgs/PredictionTrajectory.h"
#include "struct_msgs_v2_10/PredictionTrajectory.h"
#include "struct_msgs/PredictionTrajectoryPoint.h"
#include "struct_msgs_v2_10/PredictionTrajectoryPoint.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::GaussianInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.sigma_x, ros_v.sigma_x, type);
	convert(old_ros_v.sigma_y, ros_v.sigma_y, type);
	convert(old_ros_v.correlation, ros_v.correlation, type);
	convert(old_ros_v.area_probability, ros_v.area_probability, type);
	convert(old_ros_v.ellipse_a, ros_v.ellipse_a, type);
	convert(old_ros_v.ellipse_b, ros_v.ellipse_b, type);
	convert(old_ros_v.theta_a, ros_v.theta_a, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ObstacleIntent &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.type, ros_v.type, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PredictionObject &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.fusion_obstacle, ros_v.fusion_obstacle, type);
	convert(old_ros_v.trajectory, ros_v.trajectory, type);
	convert(old_ros_v.obstacle_intent, ros_v.obstacle_intent, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PredictionResult &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.prediction_obstacle_list_size, ros_v.prediction_obstacle_list_size, type);
	ros_v.prediction_obstacle_list.resize(old_ros_v.prediction_obstacle_list.size());
	for (int i = 0; i < ros_v.prediction_obstacle_list.size(); i++) {
	    convert(old_ros_v.prediction_obstacle_list[i], ros_v.prediction_obstacle_list[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PredictionTrajectory &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.confidence, ros_v.confidence, type);
	convert(old_ros_v.time_stamp, ros_v.time_stamp, type);
	convert(old_ros_v.relative_time, ros_v.relative_time, type);
	convert(old_ros_v.trajectory_point_size, ros_v.trajectory_point_size, type);
	ros_v.trajectory_point.resize(old_ros_v.trajectory_point.size());
	for (int i = 0; i < ros_v.trajectory_point.size(); i++) {
	    convert(old_ros_v.trajectory_point[i], ros_v.trajectory_point[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PredictionTrajectoryPoint &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.confidence, ros_v.confidence, type);
	convert(old_ros_v.position, ros_v.position, type);
	convert(old_ros_v.yaw, ros_v.yaw, type);
	convert(old_ros_v.theta_vel, ros_v.theta_vel, type);
	convert(old_ros_v.velocity, ros_v.velocity, type);
	convert(old_ros_v.relative_position, ros_v.relative_position, type);
	convert(old_ros_v.relative_velocity, ros_v.relative_velocity, type);
	convert(old_ros_v.relative_yaw, ros_v.relative_yaw, type);
	convert(old_ros_v.relative_theta_vel, ros_v.relative_theta_vel, type);
	convert(old_ros_v.gaussian_info, ros_v.gaussian_info, type);
}

REG_CONVERT_SINGLE(_iflytek_prediction_prediction_result_converter, "/iflytek/prediction/prediction_result", PredictionResult);
