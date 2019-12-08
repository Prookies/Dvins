#include "Utility/ConfigParam.h"

namespace Dvins {

double FOCAL_LENGTH;
double WINDOW_SIZE; // 窗口大小
int NUM_OF_F;       // NOTE: 特征点数目

double INIT_DEPTH;      // 初始深度
double MIN_PARALLAX;    // 关键帧选择的阈值（像素单位）
int ESTIMATE_EXTRINSIC; // IMU和相机的外参Rt:0准确;1不准确；2：没有

/// IMU参数
// 加速度计和陀螺仪噪声和随机偏置标准差
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC; // 从相机到IMU的旋转矩阵
std::vector<Eigen::Vector3d> TIC; // 从相机到IMU的平移向量
Eigen::Vector3d G;                // 重力[0,0,g]

double BIAS_ACC_THRESHOLD; // Ba阈值
double BIAS_GYR_THRESHOLD; // Ba阈值

/// 求解器参数
double SOLVER_TIME; // 最大解算时间（以保证实时性）
int NUM_ITERATIONS; // 最大解算器迭代次数（以保证实时性）

/// 系统参数
bool PUB_THIS_FRAME; // 是否需要发布特征点
int FREQ;            // 控制图像光流跟踪的频率
double NUM_OF_CAM;   // 相机的个数
int USE_IMU; // 由于一定是使用IMU的，所以不会对USE_IMU进行设置
int SENSOR;
bool MULTPLE_THREAD;
double F_THRESHOLD; // ransac算法的门限
int SHOW_TRACK;     // 是否发布跟踪点的图像
int FLOW_BACK;
int STEREO_TRACK;                 // 双目跟踪则为1
std::string EX_CALIB_RESULT_PATH; // 相机与IMU外参的输出路径OUTPUT_PATH +
                                  // "/extrinsic_parameter.csv"
std::string DVINS_RESULT_PATH;    // OUTPUT_PATH + "/vins_result_no_loop.csv"
std::string OUTPUT_FOLDER;        // 输出文件夹

/// 同步参数
double TD; // IMU和cam的时间差. unit: s. readed image clock + td = real
           // image clock (IMU clock)
int ESTIMATE_TD; // 在线校准IMU和camera时间

/// 特征点参数
int MAX_CNT;  // 特征点最大个数
int MIN_DIST; // 特征点之间的最小间隔

/// 相机参数
int FISHEYE;         // 如果是鱼眼相机则为1
double TR;           // 卷帘快门每帧时间
int ROLLING_SHUTTER; // 1：卷帘快门相机；0：全局快门相机
std::vector<std::string> CAM_NAMES; // 相机参数配置文件名
vector<Mat> CAM_K;                  // 相机内参
vector<Mat> CAM_D;                  // 畸变系数
vector<Mat> CAM_R;
vector<Mat> CAM_P;
Mat M1l, M2l, M1r, M2r;

/// 图像参数
int EQUALIZE;    // 如果光太亮或太暗则为1，进行直方图均衡化
double ROW, COL; // 图片的宽和高

/// ROS TOPIC
std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
std::string IMU_TOPIC;    // IMU的ROS TOPIC
std::string FISHEYE_MASK; // 鱼眼相机mask图的位置

ConfigParam::ConfigParam(const string &sConfigFile) {
  cv::FileStorage fsSetting(sConfigFile, cv::FileStorage::READ);
  if (!fsSetting.isOpened()) {
    LOG(ERROR) << "ERROR: Wrong path to settings" << endl;
    return;
  }

  FOCAL_LENGTH = 460.0;
  WINDOW_SIZE = 10;

  INIT_DEPTH = 5.0;
  BIAS_ACC_THRESHOLD = 0.1;
  BIAS_GYR_THRESHOLD = 0.1;

  ROW = fsSetting["image_height"];
  COL = fsSetting["image_width"];

  fsSetting["image0_topic"] >> IMAGE0_TOPIC;
  fsSetting["image1_topic"] >> IMAGE1_TOPIC;
  MAX_CNT = fsSetting["max_cnt"];
  MIN_DIST = fsSetting["min_dist"];
  F_THRESHOLD = fsSetting["F_threshold"];
  SHOW_TRACK = fsSetting["show_track"];
  FLOW_BACK = fsSetting["flow_back"];

  USE_IMU = fsSetting["imu"];
  // LOG(INFO) << "USE_IMU: " << USE_IMU << endl;
  if (USE_IMU) {
    fsSetting["imu_topic"] >> IMU_TOPIC;
    // LOG(INFO) << "IMU_TOPIC: " << IMU_TOPIC << endl;
    ACC_N = fsSetting["acc_n"];
    ACC_W = fsSetting["acc_w"];
    GYR_N = fsSetting["gyr_n"];
    GYR_W = fsSetting["gyr_w"];
    G.z() = fsSetting["g_norm"];
  }

  SOLVER_TIME = fsSetting["max_solver_time"];
  NUM_ITERATIONS = fsSetting["max_num_iterations"];
  MIN_PARALLAX = fsSetting["keyframe_parallax"];
  MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH; // NOTE: 为什么要除以焦距

  fsSetting["output_path"] >> OUTPUT_FOLDER;
  DVINS_RESULT_PATH = OUTPUT_FOLDER + "/vio.csv";
  LOG(INFO) << "result path: " << DVINS_RESULT_PATH << endl;
  std::ofstream fout(DVINS_RESULT_PATH, ios::out);
  fout.close();

  size_t pn = sConfigFile.find_last_of('/');
  string sConfigPath = sConfigFile.substr(0, pn);
  // LOG(INFO) << "Config Path: " << sConfigPath << endl;

  // 一定会读取第一个相机的配置文件
  string scam0Calib;
  fsSetting["cam0_calib"] >> scam0Calib;
  string scam0file = sConfigPath + "/" + scam0Calib;
  CAM_NAMES.push_back(scam0file);
  cv::Mat cv_LeftK;
  fsSetting["LEFT.K"] >> cv_LeftK;
  CAM_K.push_back(cv_LeftK);
  cv::Mat cv_LeftD;
  fsSetting["LEFT.D"] >> cv_LeftD;
  CAM_D.push_back(cv_LeftD);
  // LOG(INFO) << "cam0 config file: " << scam0file << endl;
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
    cv::Mat cv_RightK;
    fsSetting["RIGHT.K"] >> cv_RightK;
    CAM_K.push_back(cv_RightK);
    cv::Mat cv_RightD;
    fsSetting["RIGHT.D"] >> cv_RightD;
    CAM_D.push_back(cv_RightD);

    cv::Mat cv_LeftR;
    fsSetting["LEFT.R"] >> cv_LeftR;
    CAM_R.push_back(cv_LeftR);
    cv::Mat cv_LeftP;
    fsSetting["LEFT.P"] >> cv_LeftP;
    CAM_P.push_back(cv_LeftP);

    cv::Mat cv_RightR;
    fsSetting["RIGHT.R"] >> cv_RightR;
    CAM_R.push_back(cv_RightR);
    cv::Mat cv_RightP;
    fsSetting["RIGHT.P"] >> cv_RightP;
    CAM_P.push_back(cv_RightP);

    cv::initUndistortRectifyMap(CAM_K[0], CAM_D[0], CAM_R[0],
                                CAM_P[0].rowRange(0, 3).colRange(0, 3),
                                cv::Size(COL, ROW), CV_32F, M1l, M2l);
    cv::initUndistortRectifyMap(CAM_K[1], CAM_D[1], CAM_R[1],
                                CAM_P[1].rowRange(0, 3).colRange(0, 3),
                                cv::Size(COL, ROW), CV_32F, M1r, M2r);

    cout << "M1l = " << endl << M1l << endl;
    cout << "M1r = " << endl << M1r << endl;

  } else if (SENSOR == RGBD) {
    // TODO: RGBD有待开发
    NUM_OF_CAM = 1;
  } else {
    // 不期待的模式，报错
    LOG(ERROR) << "sensor不正确" << endl;
  }

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

    // 相机1与IMU的外参
    cv::Mat cv_T;
    fsSetting["body_T_cam0"] >> cv_T;
    Eigen::Matrix4d Til;
    cv::cv2eigen(cv_T, Til);
    RIC.push_back(Til.block<3, 3>(0, 0));
    TIC.push_back(Til.block<3, 1>(0, 3));
    if (NUM_OF_CAM == 2) {
      // 相机2与IMU的外参
      fsSetting["body_T_cam1"] >> cv_T;
      Eigen::Matrix4d Tir;
      cv::cv2eigen(cv_T, Tir);
      RIC.push_back(Tir.block<3, 3>(0, 0));
      TIC.push_back(Tir.block<3, 1>(0, 3));
      //      cout << "相机2到相机1的变换矩阵：" << endl << T.inverse() * Tir <<
      //      endl;
    }
  }

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

  // LOG(INFO) << "ROW: " << ROW << "; " << "COL: " << COL << endl;

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
       << "FISHEYE: " << FISHEYE << endl
       << "WINDOW_SIZE: " << WINDOW_SIZE << endl
       << "LEFT.K: " << endl
       << CAM_K[0] << endl
       << "LEFT.D: " << endl
       << CAM_D[0] << endl
       << "LEFT.R: " << endl
       << CAM_R[0] << endl
       << "LEFT.P: " << endl
       << CAM_P[0] << endl
       << "RIGHT.K: " << endl
       << CAM_K[1] << endl
       << "RIGHT.D: " << endl
       << CAM_D[1] << endl
       << "RIGHT.R: " << endl
       << CAM_R[1] << endl
       << "RIGHT.P: " << endl
       << CAM_P[1] << endl;
}

ConfigParam::~ConfigParam() { LOG(INFO) << "~ConfigParam()" << endl; }
}
