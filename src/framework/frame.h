#ifndef ZNQC_FRAMEWORK_FRAME_H
#define ZNQC_FRAMEWORK_FRAME_H

#include "arena.h"
#include "macro.h"
#include "session.h"

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
