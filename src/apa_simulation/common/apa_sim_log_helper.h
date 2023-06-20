#pragma once

#include <fstream>

#include "macro.h"

namespace planning {

class ApaSimLogHelper {
 private:
  DECLARE_SINGLETON(ApaSimLogHelper);

 public:
  static std::ofstream& GetOfs();

 private:
  static void Init();

 private:
  static std::ofstream ofs_;
  static bool is_init_;
};

#define APA_SIM_LOG ApaSimLogHelper::GetOfs()

}  // namespace planning