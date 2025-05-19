#include "head_out_end_decider.h"

namespace planning {

const bool HeadOutEndDecider::Process(
    const MapBound &XYbounds, const PlannerOpenSpaceConfig &config,
    const std::shared_ptr<NodeCollisionDetect> &collision_detect,
    const AstarRequest request, Pose2D &end, Node3d *astar_end_node,
    HybridAStarResult *result) {
  for (int i = 0; i < 10; i++) {
    ILOG_INFO << " end_node dedcider [" << i << "]";
    astar_end_node->Set(NodePath(end), XYbounds, config, 0.0);
    astar_end_node->SetGearType(AstarPathGear::NONE);
    astar_end_node->SetPathType(AstarPathType::END_NODE);
    astar_end_node->DebugString();

    if (!astar_end_node->IsNodeValid()) {
      ILOG_ERROR << "end_node invalid";

      result->fail_type = AstarFailType::OUT_OF_BOUND;
      return false;
    }

    // check end

    if (!collision_detect->ValidityCheckByEDT(astar_end_node)) {
      ILOG_INFO << "end_node in collision with obstacles "
                << static_cast<int>(astar_end_node->GetConstCollisionType())
                << "end pose : " << end.GetX() << ", " << end.GetY();

    } else {
      break;
    }

    switch (request.direction_request) {
      case ParkingVehDirection::HEAD_OUT_TO_LEFT:
        end.y -= 1.0;
        end.x = 8.0;
        break;
      case ParkingVehDirection::HEAD_OUT_TO_RIGHT:
        end.y += 1.0;
        end.x = 8.0;
        break;
      case ParkingVehDirection::HEAD_OUT_TO_MIDDLE:
        end.x += 0.5;
        break;
      default:
        ILOG_INFO << "Direction error";
        break;
    }

    astar_end_node->Clear();
  }

  return true;
}

}  // namespace planning