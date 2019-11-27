#include "FeatureTracker.h"
#include "ConfigParam.h"

namespace Dvins {
FeatureTracker::FeatureTracker() {
  // NOTE:FeatureTracker构造函数
}

FeatureTracker::~FeatureTracker() {}

/**
 * @brief FeatureTracker::setMask
 * 根据特征点的位置绘制一张掩模图像
 */
void FeatureTracker::setMask() {
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

map<int, vector<pair<int, Vector7d>>>
FeatureTracker::trackImg(double _cur_time, const cv::Mat &_img0,
                         const cv::Mat &_img1) {
  // 对输入图像进行跟踪
  cur_time = _cur_time;
  cur_img = _img0;

  cv::Mat rightImg = _img1;
}
}
