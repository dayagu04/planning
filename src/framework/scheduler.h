#ifndef ZNQC_FRAMEWORK_SCHEDULER_H
#define ZNQC_FRAMEWORK_SCHEDULER_H

#include <map>

#include "module.h"
#include "registry.h"
#include "session.h"

namespace planning {
namespace framework {

class Scheduler {
 public:
  Scheduler();
  ~Scheduler();

  void Init(Session *session);

  void Reset();
  void RunOnce();

 private:
  bool InitModuleList(Session *session);

  Session *session_;
  std::map<const char *, BaseModule *, common::StringLesser> module_map_{};
  int64_t run_count_;

  DISALLOW_COPY_AND_ASSIGN(Scheduler);
};

}  // namespace framework
}  // namespace planning

#endif  // ZNQC_FRAMEWORK_SCHEDULER_H
