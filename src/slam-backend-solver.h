#ifndef SLAM_BACKEND_SOLVER_H
#define SLAM_BACKEND_SOLVER_H

#include <vector>

#include "ceres/ceres.h"
#include "slam_types.h"

namespace slam {
// Shutdown flag, updated by signal handler.
extern bool shutdown_;

// Pinhole camera intrinsics parameters.
// The convention used here matches that of OpenCV:
// https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/
//     camera_calibration.html
struct CameraIntrinsics {
  // Focal length x.
  float fx;
  // Focal length y.
  float fy;
  // Principal point x.
  float cx;
  // Principal point y.
  float cy;
};

// The camera extrinsics consists of the coordinate transform from the
// camera to the robot pose. That is, it consists of the translation and
// rotation that takes a point from the camera frame to the robot frame.
struct CameraExtrinsics {
  // 3D vector of translation.
  float translation[3];
  // Rotation in scaled angle-axis (Lie algebra) form.
  float rotation[3];
};

void GetMap(const CameraIntrinsics& intrinsics,
            const CameraExtrinsics& extrinsics,
            const slam_types::SLAMProblem& problem,
            const std::vector<slam_types::SLAMNodeSolution>& solution,
            std::vector<Eigen::Vector3f>* map);

bool SolveSLAM(const CameraIntrinsics& intrinsics,
               const CameraExtrinsics& extrinsics,
               const slam_types::SLAMProblem& slam_problem,
               std::vector<slam_types::SLAMNode>* solved_nodes);

}  // namespace slam

#endif  // SLAM_BACKEND_SOLVER_H
