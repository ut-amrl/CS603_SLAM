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

using Eigen::Affine3f;
using Eigen::Homogeneous;
using Eigen::Matrix4f;
using Eigen::Matrix3f;
using Eigen::Vector3f;
using Eigen::Translation3f;
using Eigen::Transform;
using Eigen::Quaternionf;
using Eigen::AngleAxisf;

void DemoBasics() {
  cout << "Basic initialization" << endl;
  cout << "Initialize a vector v1." << endl;
  Vector3f v1(1.0, 2.0, 3.0);

  cout << "Read elements of the vector:" << endl
       << "v1.x = " << v1.x() << endl
       << "v1.y = " << v1.y() << endl
       << "v1.z = " << v1.z() << endl;

  cout << "Write 10 to the x coordinate of v1:" << endl;
  v1.x() = 10.0;
  cout << "v1.x = " << v1.x() << endl;

  cout << "Print the vector to stdout:\n" << v1 << endl;

  cout << "Initialize a matrix m1." << endl;
  Matrix3f m1;
  m1  << 2, 0, 0,
         0, 0, 4,
         0, 1, 0;

  cout << "Multiply matrix times vector." << endl;
  Vector3f v2 = m1 * v1;
  cout << "Resulting vector:\n" << v2 << endl;
}

void DemoRotations() {
  cout << "Rotations demonstration" << endl;
  float angle1 = M_PI / 4.0;
  Vector3f axis1(1, 0, 0);

  cout << "Create a rotation around an axis" << endl;
  AngleAxisf r1(angle1, axis1);

  cout << "Convert that rotation to a rotation matrix" << endl;
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
  cout << "Angle:\n" << angle2 << "\nAxis:\n" << axis2 << endl;
}

void DemoTransforms() {
  cout << "Transformations demonstration" << endl;

  cout << "Create a point (A vector representing displacement from the origin)"
       << endl;

  Vector3f p(0, 1, 0);

  cout << "Create a translation" << endl;
  Translation3f translation(0, 1, 0);

  cout << "Create a translation from a vector" << endl;
  Vector3f t(0, 1, 0);
  Translation3f translation2(t);

  cout << "Apply the translation" << endl;
  Vector3f p2 = translation * p;

  cout << p2 << endl;

  cout << "Apply an Angle Axis Rotation" << endl;

  float angle1 = M_PI / 4.0;
  Vector3f axis1(1, 0, 0);

  AngleAxisf aa(angle1, axis1);
  Vector3f p3 = aa * p;

  cout << p3 << endl;

  cout << "Create a generic affine transform" << endl;
  Affine3f transform = translation * aa;

  cout << "Apply a transform to a vector" << endl;
  Vector3f p4 = transform * p;
  cout << p4 << endl;

  cout << "Convert to a matrix" << endl;
  Matrix4f m1 = transform.matrix();
  cout << m1 << endl;

  cout << "Extract just the translation" << endl;
  Vector3f v = transform.translation();
  cout << v << endl;

  cout << "Extract just the rotation" << endl;
  Matrix3f m2 = transform.rotation();
  cout << m2 << endl;
}

void DemoHomogeneous() {
  cout << "Homogeneous demo" << endl;
  Vector3f v(1, 1, 0);

  cout << "Get a homogeneous version of a vector" << endl;
  Homogeneous<Vector3f, 0> h = v.homogeneous();

  cout << "Original vector: " << endl << v << endl;
  cout << "Homogeneous version: " << endl << h << endl;

  cout << "Convert a homogeneous vector into a normalized regular vector"
       << endl;
  Vector3f norm = h.hnormalized();
  cout << norm << endl;
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
