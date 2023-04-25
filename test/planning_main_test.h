#include <mutex> 
#include <ros/ros.h>
#include "proto_msgs/FusionOut.h"
#include "proto_msgs/LaneMarkers.h"
#include "proto_msgs/TrajectoryPoint.h"
#include "proto_msgs/PlanningOutput.h"
#include "proto_msgs/RadarSurroundData.h"
#include "proto_msgs/RadarSurroundInfo.h"
#include "proto_msgs/VehicleEgoMotion.h"
#include "proto_msgs/ControlCommand.h"
#include "proto_msgs/MPCTrajectoryPoint.h"
#include "old_include/proto/EgoMotion.pb.h"
#include "old_include/proto/PerceptionOut.pb.h"
#include "old_include/proto/FusionOut.pb.h"
#include "old_include/proto/PlanningOutput.pb.h"
#include "old_include/proto/Radar.pb.h"
#include "old_include/proto/ControlCommand.pb.h"

struct OldLocalView {
  FeiMa::FusionOut::FusionOut fusion_out;
  FeiMa::EgoMotion::VehicleEgoMotion ego_motion;
  J2::Radar::Radar_Surround_Info srnd_radar_info;

  // 待补充输入
  // FeiMa::SystemFunc::ModuleStatus module_status;
  // FeiMa::VehicleStatus vehicle_status;
};