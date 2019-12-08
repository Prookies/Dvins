#ifndef COMMON_H
#define COMMON_H

// 常用的一些头文件

// std
#include <algorithm>
#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <condition_variable>
#include <signal.h>
#include <stdlib.h>

using namespace std;

// for Eigen
#include <Eigen/Core>
#include <Eigen/Dense> // linear algebra
#include <Eigen/Geometry>
#include <Eigen/StdVector> // for vector of Eigen objects

using namespace Eigen;

// other things I need in optimiztion
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;

// for cv
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using cv::Mat;

// glog
#include <glog/logging.h>

// SLAM中常用的结构

#endif // COMMON_H
