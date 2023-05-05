#ifndef ZNQC_FRAMEWORK_FRAME_H
#define ZNQC_FRAMEWORK_FRAME_H

#include "macro.h"
#include "arena.h"
#include "session.h"
#include "context/planning_context.h"
#include "context/planning_output_context.h"
#include "context/environmental_model.h"

namespace planning {
namespace framework {

class Frame : public planning::common::Arena {
 public:
  Frame(){};
  Frame(Session* session);
  ~Frame();

  const Session* session() const { return session_; }
  Session* mutable_session() { return session_; }
 private:
  Session* session_;

  DISALLOW_COPY_AND_ASSIGN(Frame);
};

}  // namespace framework
}  // namespace planning

#endif  // ZNQC_FRAMEWORK_FRAME_H
