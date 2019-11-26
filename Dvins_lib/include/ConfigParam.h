#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "Common.h"

namespace Dvins {

const int WINDOW_SIZE = 10; // 窗口大小
const int NUM_OF_F = 1000;  // NOTE: 特征点数目

extern double INIT_DEPTH;   // 初始深度
extern double MIN_PARALLAX; // NOTE:

class ConfigParam {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum eSensor { MONOCULAR = 0, STEREO = 1, RGBD = 2 };

  enum SIZE_PARAMETERIZATION {
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
  };

  enum StateOrder { O_P = 0, O_R = 3, O_V = 6, O_BA = 9, O_BG = 12 };

  enum NoiseOrder { O_AN = 0, O_GN = 3, O_AW = 6, O_GW = 9 };

public:
  ConfigParam(const string &sConfigFile);
  ~ConfigParam();

public:
  const double focal_length;
  const int window_size;
  const int num_of_f;

  double init_depth;
  double min_parallax;
  int estimate_extrinsic;

  // IMU参数
  double acc_n, acc_w;
  double gyr_n, gyr_w;

  // 相机与IMU外参
  vector<Matrix3d> Ric;
  vector<Vector3d> Tic;
  double td;
  int estimate_td;
  Vector3d g;

  double bias_acc_threshold;
  double bias_gyr_threshold;

  // 求解参数
  double solver_time;
  int num_iterations;

  // 输出结果
  string ex_calib_result_path;
  string dvins_result_path;
  string output_folder;

  // 话题
  string imu_topic;
  string img0_topic, img1_topic;

  // 相机参数
  int fisheye;
  string fisheye_mask;
  vector<string> cam_names;
  int num_of_cam;
  eSensor msensor;

  // 图像参数
  int max_cnt;
  int min_dist;
  int row, col;
  int rolling_shutter;
  int equalize;

  // 系统参数
  double f_threshold;
  int show_track;
  int flow_back;
  int use_imu;
};
}

#endif // PARAMETERS_H
