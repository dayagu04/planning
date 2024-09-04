#include <cyber/binary.h>
#include "cyber/cyber.h"
#include "cyber/proto/parameter.pb.h"
#include "opencv2/opencv.hpp"
#include "log_glog.h"

// 导入window ip地址,ipconfig可以查询
// export DISPLAY=10.5.166.104:0.0
// echo $DISPLAY
// apt-get install xarclock
// xarclock 可以查询是否生效

struct opencv_test_data {
  std::string msg_;
};

static opencv_test_data test_data_;

void MessageCallback(const std::shared_ptr<apollo::cyber::proto::Param>& msg) {
  ILOG_INFO << "i am listener";
  ILOG_INFO << "msg content: " << msg->DebugString();

  test_data_.msg_ = msg->name();

  std::cout << msg->DebugString() << std::endl;
}

using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::cyber::proto::Param;

int main() {
  FilePath::SetName("test_listener");

  // init cyber framework
  apollo::cyber::Init("test_listener");
  // create listener node
  auto listener_node = apollo::cyber::CreateNode("listener");

  // create listener
  auto listener = listener_node->CreateReader<apollo::cyber::proto::Param>(
      "channel/chatter", MessageCallback);

  cv::Mat viz(1000, 800, CV_8UC3);
  cv::namedWindow("viz", 0);

  cv::imshow("viz", viz);

  cv::waitKey();

  apollo::cyber::WaitForShutdown();

  return 0;
}