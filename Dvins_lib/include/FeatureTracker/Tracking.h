#ifndef TRACKING_H_
#define TRACKING_H_

#include "Utility/Common.h"
#include "Utility/ConfigParam.h"

#include "Camera_models/camera_models/CameraFactory.h"
#include "Camera_models/camera_models/CataCamera.h"
#include "Camera_models/camera_models/PinholeCamera.h"

using namespace camodocal;

namespace Dvins {

class Tracking {
public:
  Tracking(const vector<string> &Calib_filenames);
  ~Tracking();

  void readIntrinsicParameter(const vector<string> &Calib_filenames);
  // 对图像进行跟踪并返回跟踪的特征点的属性（空间位置3d，像素位置2d，像素位移2d）
  map<int, vector<pair<int, Vector7d>>>
  trackImg(double _cur_time, const cv::Mat &_img0,
           const cv::Mat &_img1 = cv::Mat());
  // 对特征点周围进行掩模
  void setMask();
  // 利用F矩阵消除异常点
  void rejectWithF();

  // 行列
  int row, col;
  // 掩模图像
  Mat mask;
  // 当前帧图像和先前帧图像
  Mat prev_img, cur_img;
  // 先前特征点，当前特征点
  vector<cv::Point2f> prev_pts, cur_pts;
  // NOTE: vector<int> ids含义
  vector<int> ids;
  // 记录特征点的跟踪次数
  vector<int> track_cnt;

  // 分别表示当前帧的时间和前一帧的时间
  double cur_time, prev_time;
  vector<camodocal::CameraPtr> mCamera; // 相机模型

  bool stereo_cam; // 双目
};
}

#endif
