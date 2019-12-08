#ifndef IMUDATA_H_
#define IMUDATA_H_

#include "Utility/Common.h"

namespace Dvins {
class ImuData {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuData(const Vector3d g, const Vector3d a, const double &t)
      : _g(g), _a(a), _t(t) {}

  // convariance of measurement
  static Matrix3d _gyrMeasCov;
  static Matrix3d _accMeasCov;

  Vector3d _g;
  Vector3d _a;
  double _t;
};
}

#endif
