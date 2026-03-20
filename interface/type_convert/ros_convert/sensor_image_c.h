#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/Camera_Image_Info.h"
#include "struct_msgs_v2_10/Camera_Image_Info.h"
#include "struct_msgs/Front_Camera_Image_Info.h"
#include "struct_msgs_v2_10/Front_Camera_Image_Info.h"
#include "struct_msgs/Panorama_Camera_Image_Info.h"
#include "struct_msgs_v2_10/Panorama_Camera_Image_Info.h"
#include "struct_msgs/Surround_Camera_Image_Info.h"
#include "struct_msgs_v2_10/Surround_Camera_Image_Info.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::Camera_Image_Info &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.sensor_meta, ros_v.sensor_meta, type);
	for (int i = 0; i < 8; i++) {
	    convert(old_ros_v.frame_id[i], ros_v.frame_id[i], type);
	}
	for (int i = 0; i < 8; i++) {
	    convert(old_ros_v.format[i], ros_v.format[i], type);
	}
	for (int i = 0; i < 8; i++) {
	    convert(old_ros_v.camera_view_angle[i], ros_v.camera_view_angle[i], type);
	}
	for (int i = 0; i < 16; i++) {
	    convert(old_ros_v.camera_resolution[i], ros_v.camera_resolution[i], type);
	}
	convert(old_ros_v.height, ros_v.height, type);
	convert(old_ros_v.width, ros_v.width, type);
	convert(old_ros_v.dmabuf_fd_index, ros_v.dmabuf_fd_index, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::Front_Camera_Image_Info &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.front_camera_data, ros_v.front_camera_data, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::Panorama_Camera_Image_Info &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.front120_camera_data, ros_v.front120_camera_data, type);
	convert(old_ros_v.rear_camera_data, ros_v.rear_camera_data, type);
	convert(old_ros_v.side_fl_camera_data, ros_v.side_fl_camera_data, type);
	convert(old_ros_v.side_fr_camera_data, ros_v.side_fr_camera_data, type);
	convert(old_ros_v.side_rl_camera_data, ros_v.side_rl_camera_data, type);
	convert(old_ros_v.side_rr_camera_data, ros_v.side_rr_camera_data, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::Surround_Camera_Image_Info &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.right_camera_data, ros_v.right_camera_data, type);
	convert(old_ros_v.front_camera_data, ros_v.front_camera_data, type);
	convert(old_ros_v.left_camera_data, ros_v.left_camera_data, type);
	convert(old_ros_v.rear_camera_data, ros_v.rear_camera_data, type);
}

REG_CONVERT_SINGLE(_iflytek_sensor_camera_front_120_full_resolution_h265_converter, "/iflytek/sensor/camera/front_120_full_resolution_h265", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_front_30_full_resolution_h265_converter, "/iflytek/sensor/camera/front_30_full_resolution_h265", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_rear_full_resolution_h265_converter, "/iflytek/sensor/camera/rear_full_resolution_h265", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_side_fl_full_resolution_h265_converter, "/iflytek/sensor/camera/side_fl_full_resolution_h265", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_side_fr_full_resolution_h265_converter, "/iflytek/sensor/camera/side_fr_full_resolution_h265", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_side_rl_full_resolution_h265_converter, "/iflytek/sensor/camera/side_rl_full_resolution_h265", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_side_rr_full_resolution_h265_converter, "/iflytek/sensor/camera/side_rr_full_resolution_h265", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_surround_right_full_resolution_h265_converter, "/iflytek/sensor/camera/surround/right_full_resolution_h265", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_surround_front_full_resolution_h265_converter, "/iflytek/sensor/camera/surround/front_full_resolution_h265", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_surround_left_full_resolution_h265_converter, "/iflytek/sensor/camera/surround/left_full_resolution_h265", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_surround_rear_full_resolution_h265_converter, "/iflytek/sensor/camera/surround/rear_full_resolution_h265", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_surround_image_right_converter, "/iflytek/sensor/camera/surround/image/right", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_surround_image_front_converter, "/iflytek/sensor/camera/surround/image/front", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_surround_image_left_converter, "/iflytek/sensor/camera/surround/image/left", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_surround_image_rear_converter, "/iflytek/sensor/camera/surround/image/rear", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_surround_resize_image_right_converter, "/iflytek/sensor/camera/surround/resize_image/right", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_surround_resize_image_front_converter, "/iflytek/sensor/camera/surround/resize_image/front", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_surround_resize_image_left_converter, "/iflytek/sensor/camera/surround/resize_image/left", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_surround_resize_image_rear_converter, "/iflytek/sensor/camera/surround/resize_image/rear", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_front_30_converter, "/iflytek/sensor/camera/front_30", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_front_120_converter, "/iflytek/sensor/camera/front_120", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_front_120_eagle_converter, "/iflytek/sensor/camera/front_120_eagle", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_rear_converter, "/iflytek/sensor/camera/rear", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_side_fl_converter, "/iflytek/sensor/camera/side_fl", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_side_fr_converter, "/iflytek/sensor/camera/side_fr", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_side_rl_converter, "/iflytek/sensor/camera/side_rl", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_side_rr_converter, "/iflytek/sensor/camera/side_rr", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_dc_camera_front_30_converter, "/iflytek/dc/camera/front_30", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_dc_camera_front_120_converter, "/iflytek/dc/camera/front_120", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_dc_camera_rear_converter, "/iflytek/dc/camera/rear", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_dc_camera_side_fl_converter, "/iflytek/dc/camera/side_fl", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_dc_camera_side_fr_converter, "/iflytek/dc/camera/side_fr", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_dc_camera_side_rl_converter, "/iflytek/dc/camera/side_rl", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_dc_camera_side_rr_converter, "/iflytek/dc/camera/side_rr", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_dc_camera_surround_right_converter, "/iflytek/dc/camera/surround_right", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_dc_camera_surround_front_converter, "/iflytek/dc/camera/surround_front", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_dc_camera_surround_left_converter, "/iflytek/dc/camera/surround_left", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_dc_camera_surround_rear_converter, "/iflytek/dc/camera/surround_rear", Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_front_image_converter, "/iflytek/sensor/camera/front/image", Front_Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_panorama_image_converter, "/iflytek/sensor/camera/panorama/image", Panorama_Camera_Image_Info);
REG_CONVERT_SINGLE(_iflytek_sensor_camera_surround_image_converter, "/iflytek/sensor/camera/surround/image", Surround_Camera_Image_Info);
