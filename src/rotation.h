// Copyright 2019 joydeepb@cs.umass.edu
//
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
// Templated helper functions for converting between SO(3) and Lie Algebra.
//
//========================================================================
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
//========================================================================

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#ifndef ROTATION_H
#define ROTATION_H

namespace {
const double kEpsilon = 1e-7;
}  // namespace

template<typename T>
Eigen::Matrix<T, 3, 3> SkewSymmetric(const Eigen::Matrix<T, 3, 1> &w) {
  Eigen::Matrix<T, 3, 3> m;
  m << T(0)  , -w.z(), w.y(),
       w.z() , T(0)  , -w.x(),
       -w.y(), w.x() , T(0);
  return m;
}

template<typename T>
Eigen::Matrix<T, 3, 1> FromSkewSymmetric(const Eigen::Matrix<T, 3, 3> &s) {
  Eigen::Matrix<T, 3, 1> w(
      s(2, 1),
      s(0, 2),
      s(1, 0)
  );
  return w;
}

template<typename T>
Eigen::Matrix<T, 3, 3> Identity() {
  return Eigen::Matrix<T, 3, 3>::Identity();
}

template<typename T>
Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3>& R) {
  const T cos_theta = T(0.5) * (R.trace() - T(1.0));
  if (cos_theta > T(1.0 - kEpsilon)) {
    // Small-angle approximation.
    const Eigen::Matrix<T, 3, 3> s = T(0.5) * (R - R.transpose());
    return FromSkewSymmetric(s);
  }
  // Inverse Rodrigues' formula.
  const T theta = acos(cos_theta);
  const Eigen::Matrix<T, 3, 3> s =
      theta / (T(2.0) * sin(theta)) * (R - R.transpose());
  return FromSkewSymmetric(s);
}

template<typename T>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1>& w) {
  const T theta = w.norm();
  const Eigen::Matrix<T, 3, 3> s = SkewSymmetric(w);
  if (theta < T(kEpsilon)) {
    // Small-angle approximation.
    return (Identity<T>() + s);
  }
  // Rodrigues' formula.
  return (Identity<T>() +
          sin(theta) / theta * s +
          (T(1.0) - cos(theta)) / (theta * theta) * s * s);
}

template <typename T>
Eigen::Matrix<T, 3, 3> GetRodriguesJacobianInverse(
    const Eigen::Matrix<T, 3, 1> &w) {
  Eigen::Matrix<T, 3, 3> e_S = SkewSymmetric(w);
  const T th = w.norm();
  if (th < T(kEpsilon)) {
    return (Identity<T>() + T(0.5) * e_S);
  } else {
    return (Identity<T>()
            + T(0.5) * e_S + (T(1.0) / (th * th)
            - (T(1.0) + cos(th)) / (T(2.0) * th * sin(th))) * e_S * e_S);
  }
}

template <typename T>
Eigen::Matrix<T, 3, 3> GetRodriguesJacobian(
    const Eigen::Matrix<T, 3, 1> &w) {
  Eigen::Matrix<T, 3, 3> e_S = SkewSymmetric(w);
  const T th = w.norm();
  if (th < T(kEpsilon)) {
    return (Identity<T>() - T(0.5) * e_S);
  } else {
    return (Identity<T>()
            - (T(1.0) - cos(th)) / (th * th) * e_S
            + (th - sin(th)) / (th * th * th) * e_S * e_S);
  }
}

template <typename T>
Eigen::AngleAxis<T> LieAlgebraToAngleAxis(
    const Eigen::Matrix<T, 3, 1>& lie) {
  typedef Eigen::Matrix<T, 3, 1> Vector3T;
  const T theta = lie.norm();
  if (theta > T(kEpsilon)) {
    return Eigen::AngleAxis<T>(theta, lie / theta);
  } else {
    return Eigen::AngleAxis<T>(T(0), Vector3T(T(1), T(0), T(0)));
  }
}

template <typename T>
Eigen::AngleAxis<T> RotationMatrixToAngleAxis(
    const Eigen::Matrix<T, 3, 3>& R) {
  typedef Eigen::Matrix<T, 3, 1> Vector3T;
  const T cos_theta = T(0.5) * (R.trace() - T(1.0));
  if (cos_theta > T(1.0 - kEpsilon)) {
    // Small-angle approximation.
    const Eigen::Matrix<T, 3, 3> s = T(0.5) * (R - R.transpose());
    const Vector3T lie = FromSkewSymmetric(s);
    return LieAlgebraToAngleAxis(lie);
  }
  // Inverse Rodrigues' formula.
  const T theta = acos(cos_theta);
  const Eigen::Matrix<T, 3, 3> s =
  theta / (T(2.0) * sin(theta)) * (R - R.transpose());
  const Vector3T lie = FromSkewSymmetric(s);
  return LieAlgebraToAngleAxis(lie);
}

#endif  // ROTATION_H
