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

/**
 *              -------------------------->  x (colume)
 *              |
 *              |
 *              |
 *              |
 *              |
 *              |
 *              |
 *              V  y (row)
 * @note
 * @retval None
 */

int main() {
  // 0-255, 0 黑色
  cv::Mat ogm(200, 200, CV_8UC1, cv::Scalar(200));

  // for (size_t i = 80; i < 120; i++) {
  //   for (size_t j = 80; j < 120; j++) {
  //     ogm.at<uchar>(i, j) = 0;
  //   }
  // }

  // for (size_t i = 0; i < 20; i++) {
  //   for (size_t j = 0; j < 20; j++) {
  //     ogm.at<uchar>(i, j) = 0;
  //   }
  // }

  // for (size_t i = 180; i < 200; i++) {
  //   for (size_t j = 180; j < 200; j++) {
  //     ogm.at<uchar>(i, j) = 0;
  //   }
  // }

  ogm.at<uchar>(100, 100) = 0;

  cv::Mat edt;

  // 计算每一个像素到其他零像素的最近距离
  cv::distanceTransform(ogm, edt, CV_DIST_L2, CV_DIST_MASK_PRECISE);

  cv::imwrite("./ogm.png", ogm);
  cv::imwrite("./edt.png", edt);

  FilePath::SetName("edt");
  FLAGS_log_dir = "/asw/planning/glog";
  InitGlog(FilePath::GetName().c_str());
  FLAGS_max_log_size = 10;

  ILOG_INFO << "channel " << ogm.channels();
  ILOG_INFO << "channel " << edt.channels();

  ILOG_INFO << "c " << ogm.cols << " r " << ogm.rows;
  ILOG_INFO << "c " << edt.cols << " r " << edt.rows;

  for (size_t i = 0; i < ogm.cols; i++)
  {
    for (size_t j = 0; j < ogm.rows; j++)
    {
      ILOG_INFO << i << " j " << j << "x " << (int)ogm.at<uchar>(i, j);
    }
  }

  for (size_t i = 0; i < edt.cols; i++)
  {
    for (size_t j = 0; j < edt.rows; j++)
    {
      ILOG_INFO << i << " j " << j << "x " << (float)edt.at<float>(i, j);
    }
  }

  StopGlog();

  return 0;
}