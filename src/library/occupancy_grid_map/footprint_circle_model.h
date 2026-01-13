#pragma once

#include <cstdint>

#include "./../hybrid_astar_lib/hybrid_astar_common.h"
#include "pose2d.h"
#include "transform2d.h"

namespace planning {

#define FOOTPRINT_CIRCLE_NUM (12)

enum FootPrintCircleID : int8_t {
  LEFT_UPPER_CORNER = 0,
  RIGHT_UPPER_CORNER = 1,
  RIGHT_MORROR = 2,
  RIGHT_LOWER_CORNER = 3,
  LEFT_LOWER_CORNER = 4,
  LEFT_MORROR = 5,
  BIG_CIRCLE_UPPER = 6,
  BIG_CIRCLE_MIDDLE_UPPER = 7,
  BIG_CIRCLE_MIDDLE_LOWER = 8,
  BIG_CIRCLE_LOWER = 9,

  // lon buffer
  LON_BUFFER_LEFT_CIRCLE = 10,
  LON_BUFFER_RIGHT_CIRCLE = 11,
};

struct FootPrintCircle {
  Position2f pos;
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
            const float mirror_buffer, const bool is_chassis_model = false);

  // lat_safe_buffer: lateral buffer
  // lon_safe_buffer: lon buffer
  // mirror_buffer: mirror buffer
  // big_circle_safe_buffer: for accelerate computation, this buffer can filt
  // collision;
  void UpdateSafeBuffer(const bool fold_mirror, const float lat_safe_buffer,
                        const float lon_safe_buffer, const float mirror_buffer,
                        const float big_circle_safe_buffer = 0.35,
                        const bool is_chassis_model = false);

  void LocalToGlobalFast(FootPrintCircleList *global_circle,
                         const Pose2f &veh_pose);

  void LocalModelToGlobalModel(FootPrintCircleList *global_circle,
                               const Pose2f &veh_pose);

  const FootPrintCircleList GetLocalFootPrintCircleByGear(const bool is_drive);

  const FootPrintCircleList GetLocalFootPrintCircleByGear(
      const AstarPathGear gear) const;

  const FootPrintCircleList GetLocalFootPrintCircle();

  void LocalToGlobalByTF(FootPrintCircleList *global_circle, Transform2f *tf);

  void LocalToGlobalByGear(FootPrintCircleList *global_circle, Transform2f *tf,
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

// If path is circle, need more safe buffer than straight path to decrease human
// fear.
enum HierarchySafeBuffer : int8_t {
  // straight path
  INSIDE_SLOT_BUFFER = 0,
  OUTSIDE_SLOT_BUFFER = 1,
  // circle path
  CIRCLE_PATH_INSIDE_SLOT_BUFFER = 2,
  CIRCLE_PATH_OUTSIDE_SLOT_BUFFER = 3,
  MAX_NUMBER = 4,
};

enum ObstacleHeightType : uint8_t {
  MID_HIGH = 0,
  LOW = 1,
  HEIGHT_LAYER_NUM = 2,
  // The number of layers in MultiHeightFootPrintView must be strictly equal to
  // the number of layers in EDT
};
struct HierarchyBufferCircleFootPrint {
  FootPrintCircleModel footprint_model[ObstacleHeightType::HEIGHT_LAYER_NUM]
                                      [HierarchySafeBuffer::MAX_NUMBER];
};
struct MultiHeightFootPrintView {
  FootPrintCircleModel height_model[ObstacleHeightType::HEIGHT_LAYER_NUM];
};

}  // namespace planning