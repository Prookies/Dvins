#include "FeatureTracker/Tracking.h"
#include "Utility/ConfigParam.h"

namespace Dvins {
Tracking::Tracking(const vector<string> &Calib_filename) {
  // NOTE:FeatureTracker构造函数
  // 读取相机内参
  readIntrinsicParameter(Calib_filename);
  //  cout << endl << "Camera Parameters: " << endl;
  //  for (size_t i = 0; i < mCamera.size(); i++) {
  //    cout << "Camera 1:" << endl << mCamera[i] << endl;
  //  }
}

Tracking::~Tracking() {}

/**
 * @brief FeatureTracker::setMask
 * 根据特征点的位置绘制一张掩模图像
 */
void Tracking::setMask() {
  // 初始化掩模图像
  mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));

  // 保留长期跟踪的特征点
  vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

  for (unsigned int i = 0; i < cur_pts.size(); i++)
    cnt_pts_id.push_back(
        make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));

  sort(cnt_pts_id.begin(), cnt_pts_id.end(),
       [](const pair<int, pair<cv::Point2f, int>> &a,
          const pair<int, pair<cv::Point2f, int>> &b) {
         return a.first > b.first;
       });

  cur_pts.clear();
  ids.clear();
  track_cnt.clear();

  for (auto &it : cnt_pts_id) {
    if (mask.at<uchar>(it.second.first) == 255) {
      cur_pts.push_back(it.second.first);
      ids.push_back(it.second.second);
      track_cnt.push_back(it.first);
      cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
    }
  }
}

map<int, vector<pair<int, Vector7d>>> Tracking::trackImg(double _cur_time,
                                                         const cv::Mat &_img0,
                                                         const cv::Mat &_img1) {
  // 对输入图像进行跟踪
  cur_time = _cur_time;
  cur_img = _img0;

  cv::Mat rightImg = _img1;
  // 对图像进行直方图均衡化
  if (EQUALIZE) {
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(cur_img, cur_img);
    clahe->apply(rightImg, rightImg);
  }
}

void Tracking::readIntrinsicParameter(const vector<string> &Calib_filenames) {
  for (size_t i = 0; i < Calib_filenames.size(); i++) {
    LOG(INFO) << "reading paramerter of camera: " + Calib_filenames[i] << endl;
    camodocal::CameraPtr camera =
        CameraFactory::instance()->generateCameraFromYamlFile(
            Calib_filenames[i]);
    mCamera.push_back(camera);
  }
  // 并不打算该变量，可以用SENSOR代替
  if (Calib_filenames.size() == 2)
    stereo_cam = 1;
}
}
