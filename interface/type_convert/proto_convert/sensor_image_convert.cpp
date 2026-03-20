#include "proto_convert/proto_convert.h"

/*to ros*/
void CameraImageInfoToRos(SensorImage::Camera_Image_Info obj, proto_msgs::Camera_Image_Info &msg) {
  msg.frame_id = obj.frame_id();
  msg.format = obj.format();
  msg.camera_view_angle = obj.camera_view_angle();
  msg.camera_resolution = obj.camera_resolution();
  msg.height = obj.height();
  msg.width = obj.width();
  msg.exposure_starting_utc_time = obj.exposure_starting_utc_time();
  msg.isp_arrival_utc_timestamp = obj.isp_arrival_utc_timestamp();
  msg.isp_arrival_monotonic_timestamp = obj.isp_arrival_monotonic_timestamp();
  msg.exposure_duration = obj.exposure_duration();
  msg.isp_arrival_to_app_capture_duration_time = obj.isp_arrival_to_app_capture_duration_time();
  msg.dmabuf_fd_index = obj.dmabuf_fd_index();
  //
  char *reverse_img_data = const_cast<char *>(obj.data().c_str());
  unsigned char *img = reinterpret_cast<unsigned char *>(reverse_img_data);
  msg.data.resize(obj.data().size());
  memcpy(&msg.data[0], img, obj.data().size());
}

/*to proto*/
void CameraImageInfoToProto(SensorImage::Camera_Image_Info &obj, proto_msgs::Camera_Image_Info msg) {
  obj.set_frame_id(msg.frame_id);
  obj.set_format(msg.format);
  obj.set_camera_view_angle(msg.camera_view_angle);
  obj.set_camera_resolution(msg.camera_resolution);
  obj.set_height(msg.height);
  obj.set_width(msg.width);
  obj.set_exposure_starting_utc_time(msg.exposure_starting_utc_time);
  obj.set_isp_arrival_utc_timestamp(msg.isp_arrival_utc_timestamp);
  obj.set_isp_arrival_monotonic_timestamp(msg.isp_arrival_monotonic_timestamp);
  obj.set_exposure_duration(msg.exposure_duration);
  obj.set_isp_arrival_to_app_capture_duration_time(msg.isp_arrival_to_app_capture_duration_time);
  obj.set_dmabuf_fd_index(msg.dmabuf_fd_index);
  //
  const void *img = &(msg.data)[0];
  obj.set_data(img, msg.data.size());
}
