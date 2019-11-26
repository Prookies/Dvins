#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "Common.h"
#include "ConfigParam.h"
#include "ImuData.h"

namespace Dvins {

class System {
public:
  // Input sensor

  System(ConfigParam *pParams = nullptr);
  ~System();

  void ProcessIMU(const double t, const Vector3d vGyr, const Vector3d vAcc);
  void ProcessStereoImg(const double t, const Mat &_img0, const Mat &_img1);
  cv::Mat TrackStereo(const double t, const cv::Mat &img0, const cv::Mat &img1);

private:
  // 计数
  int inputImgCnt;
  // 图像队列
  queue<pair<double, cv::Mat>> mqLimgs;
  queue<pair<double, cv::Mat>> mqRimgs;
  // IMU队列
  queue<ImuData> mqImus;
  // 数据读写互斥锁
  mutex mMutexImg;
  mutex mMutexImu;

  // Reset flag
  mutex mMutexReset;
  bool mbReset = false;

  // Change mode flags
  mutex mMutexMode;
  bool mbActiveLocalizationMode;
  bool mbDeactivateLocalizationMode;

private:
  ConfigParam *mpParams = nullptr;
};
}

#endif
