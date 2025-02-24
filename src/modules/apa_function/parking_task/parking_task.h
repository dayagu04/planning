#pragma once

#include <memory>
#include <string>

#include "collision_detection/collision_detector_interface.h"

namespace planning {

enum class TaskType {
  CENTER_LINE_DECIDER,
  SLOT_RELEASE_DECIDER,
  TARGET_POSE_DECIDER,
  VIRTUAL_WALL_DECIDER,
  SLOT_MANAGER,
  OBSTACLE_MANAGER,
  PATH_OPTIMIZER,
  SPEED_LIMIT_DECIDER,
  STOP_DECIDER,
  SPEED_OPTIMIZER,
  ASTAR_PATH_GENERATOR,
  CURVE_BASED_PATH_GENERATOR,
};

// todo：move all park task to the task pipline in next refact. But for now,
// the first refact keep old logic.
// 为了让各个模块容易维护,所以增加Task.
// 开发原则：
// 1.
// 为了保持开发的原子性,所有的计算任务都是一个Task,大的任务包括换道决策,小的任务包括目标点
// 选择.
// 2. 相同功能的task,
// 尽量放到一个名字的task内部，然后不同用户调用的时候通过参数或者场景来选择
// 自己需要的API.
class ParkingTask {
 public:
  ParkingTask();

  virtual ~ParkingTask() = default;

  const std::string& Name() const;

  virtual bool Init();

  virtual void Execute();

  virtual void Clear();

  // push debug info in here.
  virtual void TaskDebug();

  void SetCollisionDetectorIntefacePtr(
      const std::shared_ptr<apa_planner::CollisionDetectorInterface>&
          collision_detector_interface_ptr) {
    collision_detector_interface_ptr_ = collision_detector_interface_ptr;
  }

 protected:
  std::string name_;

  TaskType type_;

  std::shared_ptr<apa_planner::CollisionDetectorInterface>
      collision_detector_interface_ptr_ = nullptr;
};

}  // namespace planning
