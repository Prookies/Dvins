#include "Estimator/System.h"

namespace Dvins {

System::System(string sConfigFile)
    : mbActiveLocalizationMode(false), mbDeactivateLocalizationMode(false) {
  LOG(INFO) << "This is Dvins." << endl;
  mpParams = new ConfigParam(sConfigFile);
  mpTracker = new Tracking(CAM_NAMES);
}

System::~System() { LOG(INFO) << "~System()" << endl; }

cv::Mat System::TrackStereo(const double t, const cv::Mat &img0,
                            const cv::Mat &img1) {
  if (SENSOR != Dvins::STEREO) {
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

void System::InputIMU(const double t, const Vector3d vGyr,
                      const Vector3d vAcc) {
  // TODO:处理IMU数据
}

void System::InputIMG(const double t, const Mat &_img0, const Mat &_img1) {
  // TODO:处理图像数据
  // 输入的图像的数量，可以考虑放在Frame结构中
  inputImgCnt++;
  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;

  // NOTE: 新增,提取的3D点，后面应该和上面集成到一起，用3D点去优化
  vector<pair<int, Vector3d>> feature3d;

  if (SENSOR == STEREO) {
    // TODO: 创建当前帧
    featureFrame = mpTracker->trackImg(t, _img0, _img1);
  }

  // 创建帧对象
  // mCurrentFrame = Frame(t, _img0, _img1);
  // 跟踪
  // Track();
}
}
