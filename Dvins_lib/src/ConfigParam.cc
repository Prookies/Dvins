#include "ConfigParam.h"

namespace Dvins {
ConfigParam::ConfigParam(const string &sConfigFile)
    : focal_length(460.), window_size(10), num_of_f(1000) {
  cv::FileStorage fsSetting(sConfigFile, cv::FileStorage::READ);
  if (!fsSetting.isOpened()) {
    LOG(ERROR) << "ERROR: Wrong path to settings" << endl;
    return;
  }

  fsSetting["image0_topic"] >> img0_topic;
  fsSetting["image1_topic"] >> img1_topic;
  max_cnt = fsSetting["max_cnt"];
  min_dist = fsSetting["min_dist"];
  f_threshold = fsSetting["F_threshold"];
  show_track = fsSetting["show_track"];
  flow_back = fsSetting["flow_back"];

  use_imu = fsSetting["imu"];
  LOG(INFO) << "USE_IMU: " << use_imu << endl;
  if (use_imu) {
    fsSetting["imu_topic"] >> imu_topic;
    LOG(INFO) << "IMU_TOPIC: " << imu_topic << endl;
    acc_n = fsSetting["acc_n"];
    acc_w = fsSetting["acc_w"];
    gyr_n = fsSetting["gyr_n"];
    gyr_w = fsSetting["gyr_w"];
    g.z() = fsSetting["g_norm"];
  }

  solver_time = fsSetting["max_solver_time"];
  num_iterations = fsSetting["max_num_iterations"];
  min_parallax = fsSetting["keyframe_parallax"];
  min_parallax = min_parallax / focal_length;

  fsSetting["output_path"] >> output_folder;
  dvins_result_path = output_folder + "/vio.csv";
  LOG(INFO) << "result path: " << dvins_result_path << endl;
  std::ofstream fout(dvins_result_path, ios::out);
  fout.close();

  estimate_extrinsic = fsSetting["estimate_extrinsic"];
  if (estimate_extrinsic == 2) {
    LOG(INFO)
        << " have no prior about extrinsic param, calibrate extrinsic param"
        << endl;
    Ric.push_back(Matrix3d::Identity());
    Tic.push_back(Vector3d::Zero());
    ex_calib_result_path = output_folder + "/extrinsic_parameter.csv";
  } else {
    if (estimate_extrinsic == 1) {
      LOG(INFO) << " Optimize extrinsic param around initial guess!" << endl;
      ex_calib_result_path = output_folder + "/extrinsic_parameter.csv";
    } else if (estimate_extrinsic == 0) {
      LOG(INFO) << "fix extrinsic param" << endl;
    }

    cv::Mat cv_T;
    fsSetting["body_T_cam0"] >> cv_T;
    Eigen::Matrix4d T;
    cv::cv2eigen(cv_T, T);
    Ric.push_back(T.block<3, 3>(0, 0));
    Tic.push_back(T.block<3, 1>(0, 3));
  }

  num_of_cam = fsSetting["num_of_cam"];
  LOG(INFO) << "camera number " << num_of_cam << endl;

  size_t pn = sConfigFile.find_last_of('/');
  string sConfigPath = sConfigFile.substr(0, pn);
  LOG(INFO) << "Config Path: " << sConfigPath << endl;

  string scam0Calib;
  fsSetting["cam0_calib"] >> scam0Calib;
  string scam0file = sConfigPath + "/" + scam0Calib;
  cam_names.push_back(scam0file);
  LOG(INFO) << "cam0 config file: " << scam0file << endl;

  if (num_of_cam == 2) {
    msensor = STEREO;
    string scam1Calib;
    fsSetting["cam1_calib"] >> scam1Calib;
    string scam1file = sConfigPath + "/" + scam1Calib;
    cam_names.push_back(scam1file);
    LOG(INFO) << "cam1 config file: " << scam1file << endl;

    cv::Mat cv_T;
    fsSetting["body_T_cam1"] >> cv_T;
    Matrix4d T;
    cv::cv2eigen(cv_T, T);
    Ric.push_back(T.block<3, 3>(0, 0));
    Tic.push_back(T.block<3, 1>(0, 3));
  }

  init_depth = 5.0;
  bias_acc_threshold = 0.1;
  bias_gyr_threshold = 0.1;

  td = fsSetting["td"];
  estimate_td = fsSetting["estimate_td"];
  if (estimate_td)
    LOG(INFO)
        << "Unsynchronized sensors, online estimate time offset, initial td: "
        << td << endl;
  else
    LOG(INFO) << "Synchronized sensors, fix time offset: " << td << endl;

  equalize = fsSetting["equalize"];
  fisheye = fsSetting["fisheye"];
  fsSetting["fisheye_mask"] >> fisheye_mask;
  row = fsSetting["image_height"];
  col = fsSetting["image_width"];
  LOG(INFO) << "ROW: " << row << "; "
            << "COL: " << col << endl;

  //    rolling_shutter = fsSetting["rolling_shutter"];
  //    if(rolling_shutter)
  //    {
  //        tr =
  //    }

  fsSetting.release();

  cout << "readParameters: " << endl
       << "INIT_DEPTH: " << init_depth << endl
       << "MIN_PARALLAX: " << min_parallax << endl
       << "ACC_N: " << acc_n << endl
       << "ACC_W: " << acc_w << endl
       << "GYR_N: " << gyr_n << endl
       << "GYR_W: " << gyr_w << endl
       << "Ric0: " << endl
       << Ric[0] << endl
       << "Tic0: " << Tic[0].transpose() << endl
       << "Ric1: " << endl
       << Ric[1] << endl
       << "Tic1: " << Tic[1].transpose() << endl
       << "G: " << g.transpose() << endl
       << "BIAS_ACC_THRESHOLD: " << bias_acc_threshold << endl
       << "BIAS_GYR_THRESHOLD: " << bias_gyr_threshold << endl
       << "SOLVER_TIME: " << solver_time << endl
       << "NUM_ITERATIONS: " << num_iterations << endl
       << "ESTIMATE_EXTRINSIC: " << estimate_extrinsic << endl
       << "ESTIMATE_TD: " << estimate_td << endl
       << "ROLLING_SHUTTER: " << rolling_shutter << endl
       << "ROW: " << row << endl
       << "COL: " << col << endl
       << "TD: " << td << endl
       << "FOCAL_LENGTH: " << focal_length << endl
       << "IMAGE0_TOPIC: " << img0_topic << endl
       << "IMAGE1_TOPIC: " << img1_topic << endl
       << "IMU_TOPIC: " << imu_topic << endl
       << "FISHEYE_MASK: " << fisheye_mask << endl
       << "CAM_NAME[0]: " << cam_names[0] << endl
       << "CAM_NAME[1]: " << cam_names[1] << endl
       << "MAX_CNT: " << max_cnt << endl
       << "MIN_DIST: " << min_dist << endl
       << "F_THRESHOLD: " << f_threshold << endl
       << "SHOW_TRACK: " << show_track << endl
       << "EQUALIZE: " << equalize << endl
       << "FISHEYE: " << fisheye << endl;
}

ConfigParam::~ConfigParam() { LOG(INFO) << "~ConfigParam()" << endl; }
}
