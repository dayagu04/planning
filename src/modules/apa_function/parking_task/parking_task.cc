#include "parking_task.h"

namespace planning {

ParkingTask::ParkingTask() : name_("") {}

bool ParkingTask::Init() {

  return true;
}

const std::string& ParkingTask::Name() const { return name_; }

void ParkingTask::Execute() {
  return;
}

void ParkingTask::Clear() { return; }

void ParkingTask::TaskDebug() { return; }

}  // namespace planning
