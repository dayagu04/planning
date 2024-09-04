#include <cyber/binary.h>
#include "cyber/cyber.h"
#include "cyber/proto/parameter.pb.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "log_glog.h"

using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::cyber::proto::Param;

int main(int argc, char* argv[]) {
  FilePath::SetName("test_talker");

  // init cyber framework
  apollo::cyber::Init("test_talker");

  // create talker node
  auto test_talker_node = apollo::cyber::CreateNode("talker");

  // create talker
  auto talker = test_talker_node->CreateWriter<Param>("channel/chatter");

  // 秒
  Rate rate(10.0);
  while (apollo::cyber::OK()) {
    static uint64_t seq = 0;
    auto msg = std::make_shared<Param>();
    msg->set_name("hello, i am talker");

    talker->Write(msg);

    ILOG_INFO << msg->DebugString();

    rate.Sleep();
  }

  apollo::cyber::WaitForShutdown();
  return 0;
}