#include "apa_sim_log_helper.h"

#include <string>

namespace planning {

std::ofstream ApaSimLogHelper::ofs_;
bool ApaSimLogHelper::is_init_ = false;

void ApaSimLogHelper::Init() {
  std::string log_file = "/asw/Planning/log/sim.log";
  ofs_.open(log_file);
  is_init_ = true;
}

std::ofstream &ApaSimLogHelper::GetOfs() {
  if (!is_init_) {
    Init();
  }
  return ofs_;
}

}  // namespace planning
