#include "proto_convert/proto_convert.h"
#include <iostream>

int main() {
  AroundViewCameraPerceptionResult::AroundViewCameraPerceptionInfo obj1;
  AroundViewCameraPerceptionResult::AroundViewCameraPerceptionInfo obj11;
  obj1.set_id(11);
  proto_msgs::AroundViewCameraPerceptionInfo msg1;
  AroundViewCameraPerceptionInfoToRos(obj1, msg1);
  AroundViewCameraPerceptionInfoToProto(obj11, msg1);
  std::cout << msg1.id << std::endl;
  std::cout << obj11.id() << std::endl;

  Common::Header obj2;
  Common::Header obj22;
  obj2.set_timestamp(22);
  proto_msgs::common_Header msg2;
  HeaderToRos(obj2, msg2);
  HeaderToProto(obj22, msg2);
  std::cout << msg2.timestamp << std::endl;
  std::cout << obj22.timestamp() << std::endl;

  Map::OriginData obj3;
  Map::OriginData obj33;
  obj3.set_seq(33);
  proto_msgs::OriginData msg3;
  OriginDataToRos(obj3, msg3);
  OriginDataToProto(obj33, msg3);
  std::cout << msg3.seq << std::endl;
  std::cout << obj33.seq() << std::endl;


  Map::StaticMap obj4;
  Map::StaticMap obj44;
  Map::RoadMap *road_map = obj4.mutable_road_map();
  road_map->set_timestamp(44);
  proto_msgs::StaticMap msg4;
  StaticMapToRos(obj4, msg4);
  StaticMapToProto(obj44, msg4);
  std::cout << msg4.road_map.timestamp << std::endl;
  std::cout << obj44.road_map().timestamp() << std::endl;

  SensorImage::Camera_Image_Info obj5;
  SensorImage::Camera_Image_Info obj55;
  obj5.set_frame_id("55");
  proto_msgs::Camera_Image_Info msg5;
  CameraImageInfoToRos(obj5, msg5);
  CameraImageInfoToProto(obj55, msg5);
  std::cout << msg5.frame_id << std::endl;
  std::cout << obj55.frame_id() << std::endl;
  return 0;
}