#include "footprint_circle_model.h"

#include "log_glog.h"
#include "pose2d.h"
#include "src/modules/apa_function/apa_param_config.h"
#include "transform2d.h"

namespace planning {

void FootPrintCircleModel::Init(const float lat_safe_buffer,
                                const float lon_safe_buffer,
                                const float mirror_buffer) {
  UpdateSafeBuffer(lat_safe_buffer, lon_safe_buffer, mirror_buffer);
  ILOG_INFO << "footprint init success";

  return;
}

void FootPrintCircleModel::UpdateSafeBuffer(
    const float lat_safe_buffer, const float lon_safe_buffer,
    const float mirror_buffer, const float big_circle_safe_buffer) {
  const std::vector<float> &circle_x = apa_param.GetParam().footprint_circle_x;
  const std::vector<float> &circle_y = apa_param.GetParam().footprint_circle_y;
  const std::vector<float> &circle_r = apa_param.GetParam().footprint_circle_r;

  // min_lon_buffer:
  // 1. gear is reverse, car head buffer;
  // 2. gear is drive, car tail buffer
  // Perception object is invading, so this value is small, or else planning
  // is often failed.
  float min_lon_buffer = 0.01;
  local_circles_.max_circle.pos = Position2D(circle_x[0], circle_y[0]);
  local_circles_.max_circle.radius =
      (float)(circle_r[0] + big_circle_safe_buffer);
  local_circles_.max_circle.safe_buffer = big_circle_safe_buffer;
  local_circles_.size = 0;

  // small circle
  // todo: add safe buffer for mirrors
  for (size_t i = 1; i < circle_x.size(); i++) {
    if (local_circles_.size >= FOOTPRINT_CIRCLE_NUM) {
      continue;
    }

    local_circles_.circles[local_circles_.size].pos.x = circle_x[i];
    local_circles_.circles[local_circles_.size].pos.y = circle_y[i];

    // mirror
    if (i == 3 || i == 6) {
      local_circles_.circles[local_circles_.size].radius =
          (float)circle_r[i] + mirror_buffer;
      local_circles_.circles[local_circles_.size].safe_buffer = mirror_buffer;
    } else if (i == 1 || i == 5) {
      local_circles_.circles[local_circles_.size].radius = (float)circle_r[i];
      local_circles_.circles[local_circles_.size].pos.y += lat_safe_buffer;
      local_circles_.circles[local_circles_.size].safe_buffer = lat_safe_buffer;
    } else if (i == 7) {
      local_circles_.circles[local_circles_.size].radius =
          (float)circle_r[i] + lat_safe_buffer;
      local_circles_.circles[local_circles_.size].pos.x -= lat_safe_buffer;
      local_circles_.circles[local_circles_.size].safe_buffer = lat_safe_buffer;
    } else if (i == 4 || i == 2) {
      local_circles_.circles[local_circles_.size].radius = (float)circle_r[i];
      local_circles_.circles[local_circles_.size].pos.y -= lat_safe_buffer;
      local_circles_.circles[local_circles_.size].safe_buffer = lat_safe_buffer;
    } else if (i == 10) {
      local_circles_.circles[local_circles_.size].radius =
          (float)circle_r[i] + lat_safe_buffer;
      local_circles_.circles[local_circles_.size].pos.x += lat_safe_buffer;
      local_circles_.circles[local_circles_.size].safe_buffer = lat_safe_buffer;
    } else {
      local_circles_.circles[local_circles_.size].radius =
          (float)circle_r[i] + lat_safe_buffer;
      local_circles_.circles[local_circles_.size].safe_buffer = lat_safe_buffer;
    }

    local_circles_.size++;
  }

  // gear drive
  // front buffer: lon_safe_buffer
  drive_gear_circles_ = local_circles_;
  drive_gear_circles_.size = 12;
  drive_gear_circles_.circles[3].pos.x =
      local_circles_.circles[3].pos.x - min_lon_buffer;
  drive_gear_circles_.circles[4].pos.x =
      local_circles_.circles[4].pos.x - min_lon_buffer;
  drive_gear_circles_.circles[9].pos.x =
      local_circles_.circles[9].pos.x - min_lon_buffer;

  drive_gear_circles_.circles[10] = local_circles_.circles[0];
  drive_gear_circles_.circles[10].pos.y -= lat_safe_buffer;
  drive_gear_circles_.circles[10].pos.x += lon_safe_buffer;

  drive_gear_circles_.circles[11] = local_circles_.circles[1];
  drive_gear_circles_.circles[11].pos.y += lat_safe_buffer;
  drive_gear_circles_.circles[11].pos.x += lon_safe_buffer;
  drive_gear_circles_.circles[6].pos.x += lon_safe_buffer;

  global_drive_gear_circles_ = drive_gear_circles_;

  // DebugCircles(&drive_gear_circles_);

  // gear reverse
  reverse_gear_circles_ = local_circles_;
  reverse_gear_circles_.size = 12;
  reverse_gear_circles_.circles[0].pos.x =
      local_circles_.circles[0].pos.x + min_lon_buffer;
  reverse_gear_circles_.circles[1].pos.x =
      local_circles_.circles[1].pos.x + min_lon_buffer;
  reverse_gear_circles_.circles[6].pos.x =
      local_circles_.circles[6].pos.x + min_lon_buffer;

  reverse_gear_circles_.circles[10] = local_circles_.circles[4];
  reverse_gear_circles_.circles[10].pos.y -= lat_safe_buffer;
  reverse_gear_circles_.circles[10].pos.x -= lon_safe_buffer;

  reverse_gear_circles_.circles[11] = local_circles_.circles[3];
  reverse_gear_circles_.circles[11].pos.y += lat_safe_buffer;
  reverse_gear_circles_.circles[11].pos.x -= lon_safe_buffer;
  reverse_gear_circles_.circles[9].pos.x -= lon_safe_buffer;

  global_reverse_gear_circles_ = reverse_gear_circles_;

  // DebugCircles(&reverse_gear_circles_);

  // normal circles, move circle for safe

  local_circles_.circles[0].pos.x =
      local_circles_.circles[0].pos.x + min_lon_buffer;
  local_circles_.circles[1].pos.x =
      local_circles_.circles[1].pos.x + min_lon_buffer;
  local_circles_.circles[6].pos.x =
      local_circles_.circles[6].pos.x + min_lon_buffer;

  local_circles_.circles[3].pos.x =
      local_circles_.circles[3].pos.x - min_lon_buffer;
  local_circles_.circles[4].pos.x =
      local_circles_.circles[4].pos.x - min_lon_buffer;
  local_circles_.circles[9].pos.x =
      local_circles_.circles[9].pos.x - min_lon_buffer;

  global_circles_ = local_circles_;

  // DebugCircles(&local_circles_);
  // ILOG_INFO << "footprint init success";

  return;
}

void FootPrintCircleModel::LocalToGlobalFast(FootPrintCircleList *global_circle,
                                             const Pose2D &veh_pose) {
  Transform2d tf;
  tf.SetBasePose(veh_pose);

  // max circle
  tf.ULFLocalPointToGlobal(&(global_circle->max_circle.pos),
                           local_circles_.max_circle.pos);

  // fill circle
  for (int i = 0; i < local_circles_.size; i++) {
    tf.ULFLocalPointToGlobal(&(global_circle->circles[i].pos),
                             local_circles_.circles[i].pos);
  }

  global_circle->size = local_circles_.size;

  return;
}

void FootPrintCircleModel::LocalToGlobalByTF(FootPrintCircleList *global_circle,
                                             Transform2d *tf) {
  // max circle
  tf->ULFLocalPointToGlobal(&(global_circle->max_circle.pos),
                            local_circles_.max_circle.pos);

  // DebugCircle(&local_circles.max_circle);

  // fill circle
  for (int i = 0; i < local_circles_.size; i++) {
    tf->ULFLocalPointToGlobal(&(global_circle->circles[i].pos),
                              local_circles_.circles[i].pos);

    // DebugCircle(&local_circles.circles[i]);
  }

  global_circle->size = local_circles_.size;

  return;
}

void FootPrintCircleModel::LocalToGlobalByGear(
    FootPrintCircleList *global_circle, Transform2d *tf,
    const AstarPathGear gear) const {
  const FootPrintCircleList *local;

  if (gear == AstarPathGear::DRIVE) {
    local = &drive_gear_circles_;
  } else if (gear == AstarPathGear::REVERSE) {
    local = &reverse_gear_circles_;
  } else {
    local = &local_circles_;
  }

  // max circle
  tf->ULFLocalPointToGlobal(&(global_circle->max_circle.pos),
                            local->max_circle.pos);
  // DebugCircles(local);

  // fill circle
  for (int i = 0; i < local->size; i++) {
    tf->ULFLocalPointToGlobal(&(global_circle->circles[i].pos),
                              local->circles[i].pos);
  }

  global_circle->size = local->size;

  return;
}

void FootPrintCircleModel::LocalModelToGlobalModel(
    FootPrintCircleList *global_circle, const Pose2D &veh_pose) {
  Transform2d tf;
  tf.SetBasePose(veh_pose);

  // max circle
  tf.ULFLocalPointToGlobal(&(global_circle->max_circle.pos),
                           local_circles_.max_circle.pos);

  global_circle->max_circle.radius = local_circles_.max_circle.radius;

  // fill circle
  for (int i = 0; i < local_circles_.size; i++) {
    tf.ULFLocalPointToGlobal(&(global_circle->circles[i].pos),
                             local_circles_.circles[i].pos);

    global_circle->circles[i].radius = local_circles_.circles[i].radius;
  }

  global_circle->size = local_circles_.size;

  return;
}

const FootPrintCircleList FootPrintCircleModel::GetLocalFootPrintCircleByGear(
    const bool is_drive) {
  if (is_drive) {
    return drive_gear_circles_;
  }
  return reverse_gear_circles_;
}

const FootPrintCircleList FootPrintCircleModel::GetLocalFootPrintCircleByGear(
    const AstarPathGear gear) const {
  if (gear == AstarPathGear::DRIVE) {
    return drive_gear_circles_;
  } else if (gear == AstarPathGear::REVERSE) {
    return reverse_gear_circles_;
  }
  return local_circles_;
}

const FootPrintCircleList FootPrintCircleModel::GetLocalFootPrintCircle() {
  return local_circles_;
}

void FootPrintCircleModel::DebugCircle(const FootPrintCircle *circle) const {
  ILOG_INFO << "x " << circle->pos.x << " y " << circle->pos.y << " r "
            << circle->radius << ",safe buffer = " << circle->safe_buffer;
}

void FootPrintCircleModel::DebugCircles(
    const FootPrintCircleList *circles) const {
  DebugCircle(&circles->max_circle);

  for (int i = 0; i < circles->size; i++) {
    DebugCircle(&circles->circles[i]);
  }
}

FootPrintCircleList *FootPrintCircleModel::GetMutableGlobalFPCircleByGear(
    const AstarPathGear gear) {
  if (gear == AstarPathGear::DRIVE) {
    return &global_drive_gear_circles_;
  } else if (gear == AstarPathGear::REVERSE) {
    return &global_reverse_gear_circles_;
  }
  return &global_circles_;
}

}  // namespace planning