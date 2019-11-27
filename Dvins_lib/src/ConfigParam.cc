#include "ConfigParam.h"

namespace Dvins {
ConfigParam::ConfigParam(const string &sConfigFile) {
  cv::FileStorage fsSetting(sConfigFile, cv::FileStorage::READ);
  if (!fsSetting.isOpened()) {
    LOG(ERROR) << "ERROR: Wrong path to settings" << endl;
    return;
  }

  fsSetting["image0_topic"] >> IMAGE0_TOPIC;
  fsSetting["image1_topic"] >> IMAGE1_TOPIC;
  MAX_CNT = fsSetting["max_cnt"];
  MIN_DIST = fsSetting["min_dist"];
  F_THRESHOLD = fsSetting["F_threshold"];
  SHOW_TRACK = fsSetting["show_track"];
  FLOW_TRACK = fsSetting["flow_back"];

  USE_IMU = fsSetting["imu"];
  // LOG(INFO) << "USE_IMU: " << USE_IMU << endl;
  if (USE_IMU) {
    fsSetting["IMU_TOPIC"] >> IMU_TOPIC;
    // LOG(INFO) << "IMU_TOPIC: " << IMU_TOPIC << endl;
    ACC_N = fsSetting["ACC_N"];
    ACC_W = fsSetting["ACC_W"];
    GYR_N = fsSetting["GYR_N"];
    GYR_W = fsSetting["GYR_W"];
    G.z() = fsSetting["g_norm"];
  }

  SOLVER_TIME = fsSetting["max_SOLVER_TIME"];
  NUM_ITERATIONS = fsSetting["max_NUM_ITERATIONS"];
  MIN_PARALLAX = fsSetting["keyframe_parallax"];
  MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH; // NOTE: 为什么要除以焦距

  fsSetting["output_path"] >> OUTPUT_FOLDER;
  DVINS_RESULT_PATH = OUTPUT_FOLDER + "/vio.csv";
  LOG(INFO) << "result path: " << DVINS_RESULT_PATH << endl;
  std::ofstream fout(DVINS_RESULT_PATH, ios::out);
  fout.close();

  // 相机和IMU的外参
  ESTIMATE_EXTRINSIC = fsSetting["estimate_extrinsic"];
  if (ESTIMATE_EXTRINSIC == 2) {
    LOG(INFO)
        << " have no prior about extrinsic param, calibrate extrinsic param"
        << endl;
    RIC.push_back(Matrix3d::Identity());
    TIC.push_back(Vector3d::Zero());
    EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
  } else {
    if (ESTIMATE_EXTRINSIC == 1) {
      LOG(INFO) << " Optimize extrinsic param around initial guess!" << endl;
      EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    } else if (ESTIMATE_EXTRINSIC == 0) {
      LOG(INFO) << "fix extrinsic param" << endl;
    }

    cv::Mat cv_T;
    fsSetting["body_T_cam0"] >> cv_T;
    Eigen::Matrix4d T;
    cv::cv2eigen(cv_T, T);
    RIC.push_back(T.block<3, 3>(0, 0));
    TIC.push_back(T.block<3, 1>(0, 3));
  }

  size_t pn = sConfigFile.find_last_of('/');
  string sConfigPath = sConfigFile.substr(0, pn);
  LOG(INFO) << "Config Path: " << sConfigPath << endl;

  // 一定会读取第一个相机的配置文件
  string scam0Calib;
  fsSetting["cam0_calib"] >> scam0Calib;
  string scam0file = sConfigPath + "/" + scam0Calib;
  CAM_NAMES.push_back(scam0file);
  LOG(INFO) << "cam0 config file: " << scam0file << endl;
  // 根据所给的传感器类型来判断使用单目、双目还是RGBD
  // 从配置文件中读取传感器的类型，0:单目；1：双目；2：RGBD
  // NOTE: 需要在配置文件中进行设置
  SENSOR = fsSetting["sensor"];
  if (SENSOR == MONOCULAR) {
    // TODO: 单目有待开发
    NUM_OF_CAM = 1;
  } else if (SENSOR == STEREO) {
    // 双目模式
    NUM_OF_CAM = 2;
    string scam1Calib;
    fsSetting["cam1_calib"] >> scam1Calib;
    string scam1file = sConfigPath + "/" + scam1Calib;
    CAM_NAMES.push_back(scam1file);
    LOG(INFO) << "cam1 config file: " << scam1file << endl;

    cv::Mat cv_T;
    fsSetting["body_T_cam1"] >> cv_T;
    Matrix4d T;
    cv::cv2eigen(cv_T, T);
    RIC.push_back(T.block<3, 3>(0, 0));
    TIC.push_back(T.block<3, 1>(0, 3));

  } else if (SENSOR == RGBD) {
    // TODO: RGBD有待开发
    NUM_OF_CAM = 1;
  } else {
    // 不期待的模式，报错
    LOG(ERROR) << "sensor不正确" << endl;
  }

  INIT_DEPTH = 5.0;
  BIAS_ACC_THRESHOLD = 0.1;
  BIAS_GYR_THRESHOLD = 0.1;

  TD = fsSetting["td"];
  ESTIMATE_TD = fsSetting["estimate_td"];
  if (ESTIMATE_TD)
    LOG(INFO)
        << "Unsynchronized sensors, online estimate time offset, initial td: "
        << TD << endl;
  else
    LOG(INFO) << "Synchronized sensors, fix time offset: " << TD << endl;

  EQUALIZE = fsSetting["equalize"];
  FISHEYE = fsSetting["fisheye"];
  fsSetting["fisheye_mask"] >> FISHEYE_MASK;
  ROW = fsSetting["image_height"];
  COL = fsSetting["image_width"];
  LOG(INFO) << "ROW: " << ROW << "; "
            << "COL: " << COL << endl;

  //    rolling_shutter = fsSetting["rolling_shutter"];
  //    if(rolling_shutter)
  //    {
  //        tr =
  //    }

  fsSetting.release();

  cout << "readParameters: " << endl
       << "INIT_DEPTH: " << INIT_DEPTH << endl
       << "MIN_PARALLAX: " << MIN_PARALLAX << endl
       << "ACC_N: " << ACC_N << endl
       << "ACC_W: " << ACC_W << endl
       << "GYR_N: " << GYR_N << endl
       << "GYR_W: " << GYR_W << endl
       << "Ric0: " << endl
       << RIC[0] << endl
       << "Tic0: " << TIC[0].transpose() << endl
       << "Ric1: " << endl
       << RIC[1] << endl
       << "Tic1: " << TIC[1].transpose() << endl
       << "G: " << G.transpose() << endl
       << "BIAS_ACC_THRESHOLD: " << BIAS_ACC_THRESHOLD << endl
       << "BIAS_GYR_THRESHOLD: " << BIAS_GYR_THRESHOLD << endl
       << "SOLVER_TIME: " << SOLVER_TIME << endl
       << "NUM_ITERATIONS: " << NUM_ITERATIONS << endl
       << "ESTIMATE_EXTRINSIC: " << ESTIMATE_EXTRINSIC << endl
       << "ESTIMATE_TD: " << ESTIMATE_TD << endl
       << "ROLLING_SHUTTER: " << ROLLING_SHUTTER << endl
       << "ROW: " << ROW << endl
       << "COL: " << COL << endl
       << "TD: " << TD << endl
       << "FOCAL_LENGTH: " << FOCAL_LENGTH << endl
       << "IMAGE0_TOPIC: " << IMAGE0_TOPIC << endl
       << "IMAGE1_TOPIC: " << IMAGE1_TOPIC << endl
       << "IMU_TOPIC: " << IMU_TOPIC << endl
       << "FISHEYE_MASK: " << FISHEYE_MASK << endl
       << "CAM_NAME[0]: " << CAM_NAMES[0] << endl
       << "CAM_NAME[1]: " << CAM_NAMES[1] << endl
       << "MAX_CNT: " << MAX_CNT << endl
       << "MIN_DIST: " << MIN_DIST << endl
       << "F_THRESHOLD: " << F_THRESHOLD << endl
       << "SHOW_TRACK: " << SHOW_TRACK << endl
       << "EQUALIZE: " << EQUALIZE << endl
       << "FISHEYE: " << FISHEYE << endl;
}

ConfigParam::~ConfigParam() { LOG(INFO) << "~ConfigParam()" << endl; }
}
