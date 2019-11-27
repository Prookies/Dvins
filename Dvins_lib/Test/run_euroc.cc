#include "Common.h"
#include "System.h"

// -1: 退出程序；0: 程序停止执行；1：运行程序
static int sigflag = 0;
static mutex MutexSig;
static condition_variable condSig;

// 数据
static vector<pair<double, pair<Vector3d, Vector3d>>>
    vImus; // 前面的3维向量表示角速度，后面的3维向量表示加速度
static vector<pair<double, cv::Mat>> vImgs0;
static vector<pair<double, cv::Mat>> vImgs1;

// 数据缓存区
static queue<pair<double, pair<Vector3d, Vector3d>>> qImus;
static queue<pair<double, cv::Mat>> qImgs0;
static queue<pair<double, cv::Mat>> qImgs1;

// 文件目录
static string sDataPath = "/home/lab202/Dataset/EuRoc/MH-05/mav0";
static string sConfigFile = "../config/euroc/stereo_imu_config.yaml";
static string sVocFile = "../Vocabulary/brief_k10L6.bin";

// 参数类以及SLAM系统
static Dvins::ConfigParam *pParams;
static Dvins::System *pSLAM;

// 互斥锁
static mutex MutexBuf; // 防止多个线程同时对缓存区进行操作

// 中断服务标志位
static volatile sig_atomic_t InterruptSig;

// 中断服务程序
void sigint_function(int sig) {
  LOG(INFO) << "SIGINT catch: " << sig << endl;
  InterruptSig = 1;
}

// 测试
void testSig();
void waitSig();
bool checkSig();

//  如果使用ros的话，可以不用这两个函数
void loadImus();
void loadImg();
// 将数据存储到缓存区中，这三个函数作为线程运行
void PubImuData();
void PubImg0();
void PubImg1();
// 接收数据
void ReceiveImg();
void ReceiveImu();

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;

  if (argc == 4) {
    LOG(INFO) << "从输入的参数读取数据以及配置文件" << endl;
    sDataPath = argv[1];
    sConfigFile = argv[2];
    sVocFile = argv[3];
  }

  signal(SIGINT, sigint_function);

  LOG(INFO) << "ConfigFile = " << sConfigFile << endl;

  // sConfigFile是相对路径，其需要在要求的目录下打开才有效
  pParams = new Dvins::ConfigParam(sConfigFile);
  pSLAM = new Dvins::System(pParams);

  // 加载数据
  loadImus();
  loadImg();
  LOG(INFO) << "成功加载imu数据和img数据" << endl;

  thread Thd_testSig(testSig);

  // 发布数据
  //  thread Thd_PubImuData(PubImuData);
  //  thread Thd_PubImg0(PubImg0);
  //  thread Thd_PubImg1(PubImg1);

  // 接收数据
  //  thread Thd_ReceiveImg(ReceiveImg);

  //  Thd_PubImuData.join();
  //  Thd_PubImg0.join();
  //  Thd_PubImg1.join();
  //  Thd_ReceiveImg.join();
  Thd_testSig.join();

  if (pSLAM) {
    delete pSLAM;
  }
  pSLAM = nullptr;

  if (pParams) {
    delete pParams;
  }
  pParams = nullptr;

  LOG(INFO) << "main end" << endl;

  return 0;
}

/**
 * @brief loadImus 加载IMU数据
 */
void loadImus() {
  string sImu_data_file = sDataPath + "/imu0/data.csv";
  LOG(INFO) << "loadImus sImu_data_file: " << sImu_data_file << endl;
  std::ifstream fsImu;
  fsImu.open(sImu_data_file.c_str());
  if (!fsImu.is_open()) {
    LOG(ERROR) << "Failed to open imu file: " << sImu_data_file << endl;
    return;
  }

  std::string sImuline;
  while (std::getline(fsImu, sImuline) && !sImuline.empty()) {
    char c = sImuline.at(0);
    if (c < '0' || c > '9')
      continue;

    //        cout << sImuline << endl;
    stringstream ssData;
    ssData << sImuline;

    double tmpData;
    Vector7d vImu;
    int cnt = 0;
    while (ssData >> tmpData) {
      vImu[cnt] = tmpData;
      cnt++;
      if (cnt == 7)
        break;
      if (ssData.peek() == ',' || ssData.peek() == ' ')
        ssData.ignore();
    }
    //    cout << fixed << vImu.transpose() << endl;

    vImus.push_back(make_pair(vImu[0] * 1e-9,
                              make_pair(Vector3d(vImu[1], vImu[2], vImu[3]),
                                        Vector3d(vImu[4], vImu[5], vImu[6]))));

    //        if(checkSig())
    //            waitSig();
  }
  fsImu.close();

  cout << "IMU数据数量为：" << vImus.size() << endl;

  LOG(INFO) << "load imu data success!" << endl;
}

/**
 * @brief loadImg 加载图像数据
 */
void loadImg() {
  string sImg0_data_file = sDataPath + "/cam0/data.csv";
  string sImg1_data_file = sDataPath + "/cam1/data.csv";
  LOG(INFO) << "loadImg sImg0_data_file: " << sImg0_data_file << ";"
            << "loadImg sImg1_data_file: " << sImg1_data_file << endl;

  ifstream fsImg;
  std::string sImgline;

  fsImg.open(sImg0_data_file.c_str());
  if (!fsImg.is_open()) {
    LOG(ERROR) << "Failed to open img0 file: " << sImg0_data_file << endl;
    return;
  }

  while (getline(fsImg, sImgline) && !sImgline.empty()) {
    char c = sImgline.at(0);
    if (c < '0' || c > '9')
      continue;

    stringstream ssData;
    ssData << sImgline;
    double timeStamp;
    string sImgName;
    ssData >> timeStamp;
    ssData.ignore();
    ssData >> sImgName;

    string sImgFile = sDataPath + "/cam0/data/" + sImgName;
    cv::Mat img = cv::imread(sImgFile.c_str(), 0);
    vImgs0.push_back(make_pair(timeStamp * 1e-9, img));
  }
  fsImg.close();

  fsImg.open(sImg1_data_file.c_str());
  if (!fsImg.is_open()) {
    LOG(ERROR) << "Failed to open img0 file: " << sImg1_data_file << endl;
    return;
  }
  while (getline(fsImg, sImgline) && !sImgline.empty()) {
    char c = sImgline.at(0);
    if (c < '0' || c > '9')
      continue;

    stringstream ssData;
    ssData << sImgline;
    double timeStamp;
    string sImgName;
    ssData >> timeStamp;
    ssData.ignore();
    ssData >> sImgName;

    string sImgFile = sDataPath + "/cam1/data/" + sImgName;
    cv::Mat img = cv::imread(sImgFile.c_str(), 0);
    vImgs1.push_back(make_pair(timeStamp * 1e-9, img));
  }

  if (vImgs0.size() != vImgs1.size()) {
    LOG(ERROR) << "双目图像的数量不一致" << endl;
    return;
  }
  cout << "图像数量为：" << vImgs0.size() << endl;
  //  for (size_t i = 0; i < vImgs0.size(); i++) {
  //    cout << fixed << "t0 = " << vImgs0[i].first << ", t0 = " <<
  //    vImgs1[i].first
  //         << endl;
  //    imshow("img0", vImgs0[i].second);
  //    imshow("img1", vImgs1[i].second);
  //    cvWaitKey(0);
  //  }

  fsImg.close();

  LOG(INFO) << "load img Data success!" << endl;
}

/* 程序暂停 */

void testSig() {
  char signal;
  while (1) {
    cin >> signal;
    if (signal == 'p') {
      unique_lock<mutex> lock(MutexSig);
      sigflag = 1;
      LOG(INFO) << "通过该次运行" << endl;
    } else if (signal == 'q') {
      unique_lock<mutex> lock(MutexSig);
      sigflag = -1;
      LOG(INFO) << "结束测试" << endl;
      break;
    }

    if (InterruptSig)
      break;
  }
  LOG(INFO) << "退出测试" << endl;
}

bool checkSig() {
  unique_lock<mutex> lock(MutexSig);
  if (sigflag == -1)
    return false;
  return true;
}

void waitSig() {
  unique_lock<mutex> lock(MutexSig);
  while (sigflag == 0) {
    usleep(1000);
  }
  if (sigflag == 1)
    sigflag = 0;
  else {
    sigflag = -1;
  }
}

/* 发布数据函数 */

/**
 * @brief PubImuData 发布IMU数据，并存储在缓存区中
 */
void PubImuData() {
  if (!vImus.empty()) {
    for (size_t i = 0; i < vImus.size(); i++) {
      MutexBuf.lock();
      qImus.push(vImus[i]);
      MutexBuf.unlock();
      //      cout << "IMU: " << vImus[i].first << " " <<
      //      vImus[i].second.first.transpose()
      //           << " " << vImus[i].second.second.transpose() << endl;

      if (InterruptSig)
        break;
    }
  }
}
/**
 * @brief PubImg0 发布左目图像数据
 */
void PubImg0() {
  if (!vImgs0.empty()) {
    for (size_t i = 0; i < vImgs0.size(); i++) {
      MutexBuf.lock();
      qImgs0.push(vImgs0[i]);
      MutexBuf.unlock();

      if (InterruptSig)
        break;
    }
  }
}
/**
 * @brief PubImg1 发布右目图像数据
 */
void PubImg1() {
  if (!vImgs1.empty()) {
    for (size_t i = 0; i < vImgs1.size(); i++) {
      MutexBuf.lock();
      qImgs1.push(vImgs1[i]);
      MutexBuf.unlock();

      if (InterruptSig)
        break;
    }
  }
}

/* 接收数据函数 */

void ReceiveImu() {
  LOG(INFO) << "开始接收IMU数据" << endl;
  while (1) {
    // TODO: 将接受到的IMU数据进行实时标定
    // TODO: 当机器人处于静止的时候，可以利用这段时间的IMU数据进行标定
    // 将imu数据输入到系统中
    double t = qImus.front().first;
    Vector3d gry(qImus.front().second.first);
    Vector3d acc(qImus.front().second.second);
    pSLAM->ProcessIMU(t, gry, acc);
    if (InterruptSig)
      break;
  }
}

void ReceiveImg() {
  LOG(INFO) << "开始接收图像数据" << endl;
  while (1) {
    if (Dvins::SENSOR == Dvins::STEREO) {
      //            LOG(INFO) << "双目模式" << endl;
      cv::Mat img0, img1;
      double timeStamp = 0;
      MutexBuf.lock();
      if (!qImgs0.empty() && !qImgs1.empty()) {
        double time0 = qImgs0.front().first;
        double time1 = qImgs1.front().first;
        //                cout<< fixed << "time0 = " << time0 << "; " << "time1
        //                = " << time1 << endl;
        if (time0 < time1 - 0.03) {
          qImgs0.pop();
          LOG(INFO) << "throw img0" << endl;
        } else if (time0 > time1 + 0.03) {
          qImgs1.pop();
          LOG(INFO) << "throw img1" << endl;
        } else {
          timeStamp = qImgs0.front().first;
          img0 = qImgs0.front().second;
          qImgs0.pop();
          img1 = qImgs1.front().second;
          qImgs1.pop();
          //                cv::imshow("img0", img0);
          //                cv::imshow("img1", img1);
          //                cv::waitKey(0);
        }

      } else {
        // 当队列中数据为空时，此时应该休眠
        usleep(1000);
      }
      MutexBuf.unlock();

      if (!img0.empty() && !img1.empty())
        pSLAM->ProcessStereoImg(timeStamp, img0, img1);
    } else {
      // TODO: 其他模式有待开发
      usleep(1000);
    }

    if (InterruptSig)
      break;
  }
}
