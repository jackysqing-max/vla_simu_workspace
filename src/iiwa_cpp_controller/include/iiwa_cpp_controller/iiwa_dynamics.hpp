#pragma once
#include <Eigen/Dense>

#include "franka_model.h"

// 统一用 Eigen::Matrix<double,7,1> / Eigen::Matrix<double,7,7>
using Vec7 = Eigen::Matrix<double, 7, 1>;
using Mat7 = Eigen::Matrix<double, 7, 7>;

// `franka_model.h` already declares `GravityVector(q)` and
// `CoriolisMatrix(q, dq)`. Keep a small compatibility shim here so the
// controller can continue using the `FrictionTorque` name.
inline Vec7 FrictionTorque(const Vec7 &dq) {
  return Friction(dq);
}
