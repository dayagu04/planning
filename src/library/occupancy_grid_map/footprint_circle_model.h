#pragma once

#include <cstdint>

#include "./../hybrid_astar_lib/hybrid_astar_common.h"
#include "pose2d.h"
#include "transform2d.h"

namespace planning {

#define FOOTPRINT_CIRCLE_NUM (12)

struct FootPrintCircle {
  Position2D pos;
  float radius;
  float safe_buffer;
};

struct FootPrintCircleList {
  int size;
  FootPrintCircle circles[FOOTPRINT_CIRCLE_NUM];

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
                        const float lon_safe_buffer, const float mirror_buffer,
                        const double big_circle_safe_buffer = 0.35);

  void LocalToGlobalFast(FootPrintCircleList *global_circle,
                         const Pose2D &veh_pose);

  void LocalModelToGlobalModel(FootPrintCircleList *global_circle,
                               const Pose2D &veh_pose);

  const FootPrintCircleList GetLocalFootPrintCircleByGear(const bool is_drive);

  const FootPrintCircleList GetLocalFootPrintCircleByGear(
      const AstarPathGear gear) const;

  const FootPrintCircleList GetLocalFootPrintCircle();

  void LocalToGlobalByTF(FootPrintCircleList *global_circle, Transform2d *tf);

  void LocalToGlobalByGear(FootPrintCircleList *global_circle, Transform2d *tf,
                           const AstarPathGear gear) const;

  FootPrintCircleList *GetMutableGlobalFPCircleByGear(const AstarPathGear gear);

 private:
  void DebugCircle(const FootPrintCircle *circle) const;

  void DebugCircles(const FootPrintCircleList *circles) const;

 private:
  // vehicle coordinate system
  FootPrintCircleList local_circles_;

  // for different gear, use different safe buffer
  FootPrintCircleList drive_gear_circles_;
  FootPrintCircleList reverse_gear_circles_;

  FootPrintCircleList global_circles_;
  FootPrintCircleList global_drive_gear_circles_;
  FootPrintCircleList global_reverse_gear_circles_;
};

}  // namespace planning