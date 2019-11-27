#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "Common.h"

namespace Dvins {

const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10; // 窗口大小
const int NUM_OF_F = 1000;  // NOTE: 特征点数目

extern double INIT_DEPTH;   // 初始深度
extern double MIN_PARALLAX; // 关键帧选择的阈值（像素单位）
extern int ESTIMATE_EXTRINSIC; // IMU和相机的外参Rt:0准确;1不准确；2：没有

// 加速度计和陀螺仪噪声和随机偏置标准差
extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC; // 从相机到IMU的旋转矩阵
extern std::vector<Eigen::Vector3d> TIC; // 从相机到IMU的平移向量
extern Eigen::Vector3d G;                // 重力[0,0,g]

extern double BIAS_ACC_THRESHOLD; // Ba阈值
extern double BIAS_GYR_THRESHOLD; // Ba阈值
extern double SOLVER_TIME;        // 最大解算时间（以保证实时性）
extern int NUM_ITERATIONS; // 最大解算器迭代次数（以保证实时性）
extern std::string EX_CALIB_RESULT_PATH; // 相机与IMU外参的输出路径OUTPUT_PATH +
                                         // "/extrinsic_parameter.csv"
extern std::string
    DVINS_RESULT_PATH;            // OUTPUT_PATH + "/vins_result_no_loop.csv"
extern std::string OUTPUT_FOLDER; // 输出文件夹
extern std::string IMU_TOPIC;     // IMU topic名"/imu0"
extern double TD; // IMU和cam的时间差. unit: s. readed image clock + td = real
                  // image clock (IMU clock)
extern int ESTIMATE_TD;     // 在线校准IMU和camera时间
extern double TR;           // 卷帘快门每帧时间
extern int ROLLING_SHUTTER; // 1：卷帘快门相机；0：全局快门相机
extern double ROW, COL;     // 图片的宽和高
extern double NUM_OF_CAM;   // 相机的个数
// 由于一定是使用IMU的，所以不会对USE_IMU进行设置
extern int USE_IMU;
extern int SENSOR;
// 由于一定是用多线程进行编程的，所以也不会对MULTIPLE_THREAD进行设置
extern bool MULTPLE_THREAD;

// 图像的ROS TOPIC
extern std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
extern std::string IMU_TOPIC;              // IMU的ROS TOPIC
extern std::string FISHEYE_MASK;           // 鱼眼相机mask图的位置
extern std::vector<std::string> CAM_NAMES; // 相机参数配置文件名
extern int MAX_CNT;                        // 特征点最大个数
extern int MIN_DIST;                       // 特征点之间的最小间隔
extern int FREQ;                           // 控制图像光流跟踪的频率
extern double F_THRESHOLD;                 // ransac算法的门限
extern int SHOW_TRACK;                     // 是否发布跟踪点的图像
extern int FLOW_TRACK;
extern int STEREO_TRACK; // 双目跟踪则为1
extern int EQUALIZE; // 如果光太亮或太暗则为1，进行直方图均衡化
extern int FISHEYE;         // 如果是鱼眼相机则为1
extern bool PUB_THIS_FRAME; // 是否需要发布特征点

enum eSensor { MONOCULAR = 0, STEREO = 1, RGBD = 2 };

enum eSIZE_PARAMETERIZATION {
  SIZE_POSE = 7,
  SIZE_SPEEDBIAS = 9,
  SIZE_FEATURE = 1
};

enum eStateOrder { O_P = 0, O_R = 3, O_V = 6, O_BA = 9, O_BG = 12 };

enum eNoiseOrder { O_AN = 0, O_GN = 3, O_AW = 6, O_GW = 9 };

class ConfigParam {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ConfigParam(const string &sConfigFile);
  ~ConfigParam();
};
}

#endif // PARAMETERS_H
