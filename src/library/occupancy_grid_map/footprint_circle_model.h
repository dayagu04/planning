#pragma once

#include <cstdint>
#include "./../hybrid_astar_lib/hybrid_astar_common.h"
#include "pose2d.h"
#include "transform2d.h"

namespace planning {

#define footprint_circle_num (10)

struct FootPrintCircle {
  Position2D pos;
  float radius;
  float safe_buffer;
};

struct FootPrintCircleList {
  int size;
  FootPrintCircle circles[footprint_circle_num];

  // if this circle no collision, then no need to check other circle
  FootPrintCircle max_circle;
};

// todo: use different height osbtacle to check different vehicle components.
class FootPrintCircleModel {
 public:
  FootPrintCircleModel() = default;

  void Init(const float lat_safe_buffer, const float lon_safe_buffer,
            const float mirror_buffer);

  void UpdateSafeBuffer(const float lat_safe_buffer,
                        const float lon_safe_buffer, const float mirror_buffer);

  void LocalToGlobalFast(FootPrintCircleList *global_circle,
                         const Pose2D &veh_pose);

  void LocalModelToGlobalModel(FootPrintCircleList *global_circle,
                               const Pose2D &veh_pose);

  const FootPrintCircleList GetLocalFootPrintCircleByGear(const bool is_drive);

  const FootPrintCircleList GetLocalFootPrintCircle();

  void LocalToGlobalByTF(FootPrintCircleList *global_circle, Transform2d *tf);

  void LocalToGlobalByGear(FootPrintCircleList *global_circle, Transform2d *tf,
                           const AstarPathGear gear);

 private:
  void DebugCircle(FootPrintCircle *circle);

  void DebugCircles(FootPrintCircleList *circles);

 private:
  // vehicle coordinate system
  FootPrintCircleList local_circles_;

  // for different gear, use different safe buffer
  FootPrintCircleList drive_gear_circles_;
  FootPrintCircleList reverse_gear_circles_;
};

}  // namespace planning