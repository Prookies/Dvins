#ifndef FeatureTracker_H_
#define FeatureTracker_H_

#include "Common.h"

namespace Dvins {

class FeatureTracker{
public:

    FeatureTracker();
    ~FeatureTracker();

    // 对图像进行跟踪并返回跟踪的特征点的属性（空间位置，像素位置，像素位移）
    map<int, vector<pair<int, Vector7d>>> trackImg(double _cur_time, const cv::Mat &_img0, const cv::Mat &_img1 = cv::Mat());
    // 对特征点周围进行掩模
    void setMask();
    // 利用F矩阵消除异常点
    void rejectWithF();

    // 行列
    int row, col;
    // 掩模图像
    Mat mask;
    // 先前特征点，当前特征点
    vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts;
    // NOTE: vector<int> ids含义
    vector<int> ids;
    // 记录特征点的跟踪次数
    vector<int> track_cnt;
};
}



#endif
