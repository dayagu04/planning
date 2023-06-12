#pragma once

#include <fstream>

#include "macro.h"

namespace planning {

class PlanningLogHelper {
 private:
  DECLARE_SINGLETON(PlanningLogHelper);

 public:
  static std::ofstream& GetOfs();

 private:
  static void Init();

 private:
  static std::ofstream ofs_;
  static bool is_init_;
};

#define PLANNING_LOG PlanningLogHelper::GetOfs()

}  // namespace planning