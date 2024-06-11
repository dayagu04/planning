#pragma once
#include <memory>
#include "ehr.pb.h"
#include "planning_debug_info.pb.h"
#include "proto_msgs/PlanningDebugInfo.h"
#include "proto_msgs/StaticMap.h"

namespace planning {
namespace planning_player {

void ConvertPlanningDebugInfoMsg(
    const std::shared_ptr<planning::common::PlanningDebugInfo> &obj,
    proto_msgs::PlanningDebugInfo &msg);

void StaticMapToProto(Map::StaticMap &obj, proto_msgs::StaticMap &msg);

}  // namespace planning_player
}  // namespace planning