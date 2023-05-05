#include "apa_planner/common/planning_log_helper.h"

#include <string>

#include "ifly_time.h"

namespace planning {

std::ofstream PlanningLogHelper::ofs_;
bool PlanningLogHelper::is_init_ = false;

void PlanningLogHelper::Init() {
  auto time_now = autoplt::ADSTime::Now();
  std::string date = time_now.ToDateString();
  std::string log_file = "/asw/Planning/log/planning.log." + date;
  ofs_.open(log_file);
  is_init_ = true;
}

std::ofstream &PlanningLogHelper::GetOfs() {
  if (!is_init_) {
    Init();
  }
  return ofs_;
}

}  // namespace planning
