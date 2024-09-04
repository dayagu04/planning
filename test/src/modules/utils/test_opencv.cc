#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/opencv.hpp"
#include "log_glog.h"

// 导入window ip地址,ipconfig可以查询
// export DISPLAY=10.5.166.104:0.0
// echo $DISPLAY
// apt-get install xarclock
// xarclock 可以查询是否生效

// distanceTransform:
// InputArray 	src, 8bit单通道图像
// OutputArray 	dst, 32bit float型单通道图像，到其他0像素的距离
// OutputArray 	labels,
// int 	distanceType,
// int 	maskSize,
// int 	labelType = DIST_LABEL_CCOMP

int main() {
  cv::Mat viz(1000, 800, CV_8UC3);
  cv::namedWindow("viz", 0);

  cv::imshow("viz", viz);

  cv::Mat occupancy_grid_map(200, 200, CV_8UC1, cv::Scalar(200));

  cv::Mat edt;

  // 计算每一个像素到其他零像素的最近距离
  cv::distanceTransform(occupancy_grid_map, edt, CV_DIST_L2,
                        CV_DIST_MASK_PRECISE);

  cv::imshow("occupancy_grid_map", occupancy_grid_map);
  cv::imshow("edt", edt);

  cv::waitKey(0);

  cv::imwrite("./ogm.png", occupancy_grid_map);

  // apollo::cyber::WaitForShutdown();

  return 0;
}