#include "stop_line.h"

namespace planning {

StopLine::StopLine(int id) : id_(id) {
}

StopLine::StopLine(int id, const planning::planning_math::LineSegment2d& stop_line)
    : id_(id), stop_line_(stop_line) {
  //id_ = id;
  //stop_line_ = planning::planning_math::LineSegment2d(stop_line.start(), stop_line.end());
}

StopLine::StopLine(int id, planning::planning_math::LineSegment2d&& stop_line)
    : id_(id), stop_line_(std::move(stop_line)) {
}

int StopLine::track_id() const {
  return id_;
}

const planning::planning_math::LineSegment2d& StopLine::stop_line() const {
  return stop_line_;
}

} // namespace planning
