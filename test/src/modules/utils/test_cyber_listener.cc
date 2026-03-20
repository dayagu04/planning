#include <cyber/binary.h>
#include "cyber/cyber.h"
#include "cyber/proto/parameter.pb.h"
#include "log_glog.h"

void MessageCallback(const std::shared_ptr<apollo::cyber::proto::Param>& msg) {
  ILOG_INFO << "i am listener";
  ILOG_INFO << "msg content: " << msg->DebugString();
}

using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::cyber::proto::Param;

int main(int argc, char* argv[]) {
  FilePath::SetName("test_listener");

  // init cyber framework
  apollo::cyber::Init("test_listener");
  // create listener node
  auto listener_node = apollo::cyber::CreateNode("listener");

  // create listener
  auto listener = listener_node->CreateReader<apollo::cyber::proto::Param>(
      "channel/chatter", MessageCallback);

  apollo::cyber::WaitForShutdown();
  return 0;
}
