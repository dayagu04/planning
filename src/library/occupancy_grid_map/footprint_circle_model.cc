#include "footprint_circle_model.h"

#include "log_glog.h"
#include "pose2d.h"
#include "transform2d.h"

#include "./../../modules/apa_function/src/apa_param_setting.h"

namespace planning {

void FootPrintCircleModel::Init(const float lat_safe_buffer,
                                const float lon_safe_buffer,
                                const float mirror_buffer) {
  UpdateSafeBuffer(lat_safe_buffer, lon_safe_buffer, mirror_buffer);
  ILOG_INFO << "footprint init success";

  return;
}

void FootPrintCircleModel::UpdateSafeBuffer(const float lat_safe_buffer,
                                            const float lon_safe_buffer,
                                            const float mirror_buffer) {
  const std::vector<double> &circle_x = apa_param.GetParam().footprint_circle_x;
  const std::vector<double> &circle_y = apa_param.GetParam().footprint_circle_y;
  const std::vector<double> &circle_r = apa_param.GetParam().footprint_circle_r;

  // big circle, buffer is 0.35
  double big_circle_safe_buffer = 0.35;
  double min_lon_buffer = 0.05;
  local_circles_.max_circle.pos = Position2D(circle_x[0], circle_y[0]);
  local_circles_.max_circle.radius =
      (float)(circle_r[0] + big_circle_safe_buffer);
  local_circles_.max_circle.safe_buffer = big_circle_safe_buffer;
  local_circles_.size = 0;

  // small circle
  // todo: add safe buffer for mirrors
  for (size_t i = 1; i < circle_x.size(); i++) {
    if (local_circles_.size >= footprint_circle_num) {
      continue;
    }

    local_circles_.circles[local_circles_.size].pos.x = circle_x[i];
    local_circles_.circles[local_circles_.size].pos.y = circle_y[i];

    // mirror
    if (i == 3 || i == 6) {
      local_circles_.circles[local_circles_.size].radius =
          (float)circle_r[i] + mirror_buffer;
      local_circles_.circles[local_circles_.size].safe_buffer = mirror_buffer;
    } else if (i == 1 || i == 2 || i == 7) {
      local_circles_.circles[local_circles_.size].radius =
          (float)circle_r[i] + lat_safe_buffer;
      local_circles_.circles[local_circles_.size].pos.x -= lat_safe_buffer;
      local_circles_.circles[local_circles_.size].safe_buffer = lat_safe_buffer;
    } else if (i == 4 || i == 5 || i == 10) {
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
  drive_gear_circles_.circles[0].pos.x =
      local_circles_.circles[0].pos.x + lon_safe_buffer;
  drive_gear_circles_.circles[1].pos.x =
      local_circles_.circles[1].pos.x + lon_safe_buffer;
  drive_gear_circles_.circles[6].pos.x =
      local_circles_.circles[6].pos.x + lon_safe_buffer;

  drive_gear_circles_.circles[3].pos.x =
      local_circles_.circles[3].pos.x - min_lon_buffer;
  drive_gear_circles_.circles[4].pos.x =
      local_circles_.circles[4].pos.x - min_lon_buffer;
  drive_gear_circles_.circles[9].pos.x =
      local_circles_.circles[9].pos.x - min_lon_buffer;

  // DebugCircles(&drive_gear_circles_);

  // gear reverse
  reverse_gear_circles_ = local_circles_;
  reverse_gear_circles_.circles[0].pos.x =
      local_circles_.circles[0].pos.x + min_lon_buffer;
  reverse_gear_circles_.circles[1].pos.x =
      local_circles_.circles[1].pos.x + min_lon_buffer;
  reverse_gear_circles_.circles[6].pos.x =
      local_circles_.circles[6].pos.x + min_lon_buffer;

  reverse_gear_circles_.circles[3].pos.x =
      local_circles_.circles[3].pos.x - lon_safe_buffer;
  reverse_gear_circles_.circles[4].pos.x =
      local_circles_.circles[4].pos.x - lon_safe_buffer;
  reverse_gear_circles_.circles[9].pos.x =
      local_circles_.circles[9].pos.x - lon_safe_buffer;

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

  if (gear == AstarPathGear::drive) {
    local = &drive_gear_circles_;
  } else if (gear == AstarPathGear::reverse) {
    local = &reverse_gear_circles_;
  } else {
    local = &local_circles_;
  }

  // max circle
  tf->ULFLocalPointToGlobal(&(global_circle->max_circle.pos),
                            local->max_circle.pos);

  // DebugCircle(&local_circles.max_circle);

  // fill circle
  for (int i = 0; i < local->size; i++) {
    tf->ULFLocalPointToGlobal(&(global_circle->circles[i].pos),
                              local->circles[i].pos);

    // DebugCircle(&local_circles.circles[i]);
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
  if (gear == AstarPathGear::drive) {
    return drive_gear_circles_;
  } else if (gear == AstarPathGear::reverse) {
    return reverse_gear_circles_;
  }
  return local_circles_;
}

const FootPrintCircleList FootPrintCircleModel::GetLocalFootPrintCircle() {
  return local_circles_;
}

void FootPrintCircleModel::DebugCircle(FootPrintCircle *circle) {
  ILOG_INFO << "x " << circle->pos.x << " y " << circle->pos.y << " r "
            << circle->radius;
}

void FootPrintCircleModel::DebugCircles(FootPrintCircleList *circles) {
  DebugCircle(&circles->max_circle);

  for (int i = 0; i < circles->size; i++) {
    DebugCircle(&circles->circles[i]);
  }
}

}  // namespace planning