#include "System.h"

namespace Dvins {

System::System(ConfigParam *pParams)
    : mbActiveLocalizationMode(false), mbDeactivateLocalizationMode(false),
      mpParams(pParams) {
  LOG(INFO) << "This is Dvins." << endl;
}

System::~System() { LOG(INFO) << "~System()" << endl; }

cv::Mat System::TrackStereo(const double t, const cv::Mat &img0,
                            const cv::Mat &img1) {
  if (mpParams->msensor != mpParams->STEREO) {
    LOG(INFO) << "ERROR: you called TrackStereo but input sensor was not set "
                 "to Stereo."
              << endl;
    exit(-1);
  }

  {
    unique_lock<mutex> lock(mMutexMode);
    if (mbActiveLocalizationMode) {
      // TODO: 激活定位模式
      mbActiveLocalizationMode = false;
    }
    if (mbDeactivateLocalizationMode) {
      // TODO: 取消定位模式
      mbDeactivateLocalizationMode = false;
    }
  }

  {
    unique_lock<mutex> lock(mMutexReset);
    if (mbReset) {
      // TODO: 重置系统
      mbReset = false;
    }
  }

  cv::Mat Tcw; // 世界坐标系到相机坐标系的变换矩阵

  return Tcw;
}

void System::ProcessIMU(const double t, const Vector3d vGyr,
                        const Vector3d vAcc) {
  // TODO:处理IMU数据
}

void System::ProcessStereoImg(const double t, const Mat &_img0,
                              const Mat &_img1) {
  // TODO:处理图像数据
  inputImgCnt++;
  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
  if (_img1.empty())
    featureFrame = FeatureTracker.trackImg(t, _img0);
  else if (_img0.empty())
    featureFrame = FeatureTracker.trackImg(t, _img0, _img1);
}
}
