#pragma once
#include "proto/around_view_camera_perception_debug_info.pb.h"
#include "proto/common.pb.h"
#include "proto/ehr.pb.h"
#include "proto/sensor_image.pb.h"

/*to ros*/
void OriginDataToRos(Map::OriginData obj, proto_msgs::OriginData &msg);
void StaticMapToRos(Map::StaticMap obj, proto_msgs::StaticMap &msg);
void CameraImageInfoToRos(SensorImage::Camera_Image_Info obj, proto_msgs::Camera_Image_Info &msg);
void AroundViewCameraPerceptionInfoToRos(AroundViewCameraPerceptionResult::AroundViewCameraPerceptionInfo obj,
                                         proto_msgs::AroundViewCameraPerceptionInfo &msg);
void HeaderToRos(Common::Header obj, proto_msgs::common_Header &msg);
/*to proto*/
void OriginDataToProto(Map::OriginData &obj, proto_msgs::OriginData msg);
void StaticMapToProto(Map::StaticMap &obj, proto_msgs::StaticMap msg);
void CameraImageInfoToProto(SensorImage::Camera_Image_Info &obj, proto_msgs::Camera_Image_Info msg);
void AroundViewCameraPerceptionInfoToProto(AroundViewCameraPerceptionResult::AroundViewCameraPerceptionInfo &obj,
                                           proto_msgs::AroundViewCameraPerceptionInfo msg);
void HeaderToProto(Common::Header &obj, proto_msgs::common_Header msg);
