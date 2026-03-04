#pragma once
#include <Eigen/Dense>

// 统一用 Eigen::Matrix<double,7,1> / Eigen::Matrix<double,7,7>
using Vec7 = Eigen::Matrix<double, 7, 1>;
using Mat7 = Eigen::Matrix<double, 7, 7>;

/**
 * TODO: 用你自己的 iiwa 动力学/参数替换这些函数实现。
 * 这里先给“可编译的占位实现”，默认返回 0。
 */

// 重力项 g(q)
inline Vec7 GravityVector(const Vec7 &q) {
  (void)q;
  return Vec7::Zero();
}

// 科里奥利/离心矩阵 C(q,dq)，使得 tau_c = C(q,dq) * dq
inline Mat7 CoriolisMatrix(const Vec7 &q, const Vec7 &dq) {
  (void)q; (void)dq;
  return Mat7::Zero();
}

// 摩擦项（可选）
inline Vec7 FrictionTorque(const Vec7 &dq) {
  (void)dq;
  return Vec7::Zero();
}
