#include "parking_task.h"

namespace planning {
namespace apa_planner {

ParkingTask::ParkingTask() : name_("") {}

const bool ParkingTask::Init() { return true; }

const std::string& ParkingTask::Name() const { return name_; }

void ParkingTask::Execute() { return; }

const bool ParkingTask::Update() { return false; }

void ParkingTask::Reset() { return; }

void ParkingTask::TaskDebug() { return; }
}  // namespace apa_planner
}  // namespace planning
