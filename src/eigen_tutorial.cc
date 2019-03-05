#include <stdio.h>
#include <iostream>
#include <cmath>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

using std::cout;
using std::endl;
using std::sqrt;
using std::sin;
using std::cos;

using Eigen::Matrix3f;
using Eigen::Vector3f;
using Eigen::Translation3f;
using Eigen::Quaternionf;
using Eigen::AngleAxisf;

void DemoBasics() {
  cout << "Initialize a vector v1.\n";
  Vector3f v1(1.0, 2.0, 3.0);

  cout << "Read elements of the vector:\n"
       << "v1.x = " << v1.x() << "\n"
       << "v1.y = " << v1.y() << "\n"
       << "v1.z = " << v1.z() << "\n";

  cout << "Write 10 to the x coordinate of v1:\n";
  v1.x() = 10.0;
  cout << "v1.x = " << v1.x() << "\n";

  cout << "Print the vector to stdout:\n" << v1 << "\n";

  cout << "Initialize a matrix m1.\n";
  Matrix3f m1;
  m1  << 2, 0, 0,
         0, 0, 4,
         0, 1, 0;

  cout << "Multiply matrix times vector.\n";
  Vector3f v2 = m1 * v1;
  cout << "Resulting vector:\n" << v2 << "\n";
}

void DemoRotations() {
  float angle1 = M_PI / 4.0;
  Vector3f axis1(1, 0, 0);
  AngleAxisf r1(angle1, axis1);

  Matrix3f m1 = r1.toRotationMatrix();
  cout << m1 << "\n";

  Quaternionf q1 = Quaternionf(r1);

  Matrix3f m2 = q1.toRotationMatrix();
  cout << "m2: \n" << m2 << "\n";

  Matrix3f m3;
  m3 << 1,  0,               0,
        0, cos(M_PI / 4.0), -sin(M_PI / 4.0),
        0, sin(M_PI / 4.0), cos(M_PI / 4.0);
  AngleAxisf r2 = AngleAxisf(m3);
  float angle2 = r2.angle();
  Vector3f axis2 = r2.axis();
  cout << "Angle:\n" << angle2 << "\nAxis:\n" << axis2 << "\n";
}

void DemoTransforms() {

}

void DemoHomogeneous() {
}

template <typename T>
void DemoTemplated() {
  // This is a templated 3D vector.
  Eigen::Matrix<T, 3, 1> v1(T(1), T(2), T(3));

  // '.cast<T>()' converts types to template T.
  Vector3f v2(1, 2, 3);
  Eigen::Matrix<T, 3, 1> v3 = v2.cast<T>();
  cout << "Cast vector:\n" << v3 << "\n";

  // Templated rotations.
  T angle1 = T(M_PI / 6.0);
  Eigen::Matrix<T, 3, 1> axis1(T(1), T(0), T(0));
  Eigen::AngleAxis<T> r1(angle1, axis1);

  // Templated rotation conversions.
  Eigen::Quaternion<T> r2 = Eigen::Quaternion<T>(r1);
  Eigen::AngleAxis<T> r3 = Eigen::AngleAxis<T>(r2);
  T angle3 = r3.angle();
  Eigen::Matrix<T, 3, 1> axis3 = r3.axis();
  cout << "Angle:\n" << angle3 << "\nAxis:\n" << axis3 << "\n";

  // Templated affine Transform.
  Eigen::Transform<T, 3, Eigen::Affine> a1;
}

int main() {
  cout << "Basics: Vectors and Matrices.\n";
  DemoBasics();

  cout << "\n\n\nDifferent representations of rotation.\n";
  DemoRotations();

  cout << "\n\n\nCoordinate Transforms.\n";
  DemoTransforms();

  cout << "\n\n\nHomogeneous coordinates.\n";
  DemoHomogeneous();

  cout << "\n\n\nTemplated Eigen.\n";
  DemoTemplated<double>();
  return 0;
}
