#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/IFLYIMU.h"
#include "struct_msgs_v2_10/IFLYIMU.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYIMU &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.sensor_meta, ros_v.sensor_meta, type);
	for (int i = 0; i < 64; i++) {
	    convert(old_ros_v.imu_name[i], ros_v.imu_name[i], type);
	}
	convert(old_ros_v.temperature_val, ros_v.temperature_val, type);
	convert(old_ros_v.gyro_selftest_result, ros_v.gyro_selftest_result, type);
	convert(old_ros_v.acc_selftest_result, ros_v.acc_selftest_result, type);
	convert(old_ros_v.acc_val, ros_v.acc_val, type);
	convert(old_ros_v.angular_rate_val, ros_v.angular_rate_val, type);
}

REG_CONVERT_SINGLE(_iflytek_sensor_imu_converter, "/iflytek/sensor/imu", IFLYIMU);
