#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

#include "gui_helpers.h"
#include "gflags/gflags.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "slam_types.h"

#include "slam-backend-solver.h"

using Eigen::Affine3f;
using Eigen::Translation3f;
using Eigen::AngleAxisf;
using Eigen::Vector3f;
using Eigen::Vector2f;
using Eigen::Quaternionf;
using gui_helpers::InitializeMarker;
using gui_helpers::Color4f;
using gui_helpers::AddLine;
using gui_helpers::AddPoint;
using std::cout;
using std::vector;
using slam_types::FeatureMatch;
using slam_types::VisionFeature;
using slam_types::VisionFactor;
using slam_types::OdometryFactor;
using slam_types::SLAMProblem;
using slam_types::SLAMNode;
using slam_types::SLAMNodeSolution;

DECLARE_int32(v);
DEFINE_int32(max_steps, 10000, "Maximum number of solver steps");
DEFINE_int32(max_vfactors, 0, "Maximum number of vision factors");

namespace slam {

bool shutdown_ = false;

template<typename T> Eigen::Transform<T, 3, Eigen::Affine>
PoseArrayToAffine(const T* rotation, const T* translation) {
  typedef Eigen::Transform<T, 3, Eigen::Affine> Affine3T;
  typedef Eigen::Matrix<T, 3, 1> Vector3T;
  typedef Eigen::AngleAxis<T> AngleAxisT;
  typedef Eigen::Translation<T, 3> Translation3T;

  const Vector3T rotation_axis(rotation[0], rotation[1], rotation[2]);
  const T rotation_angle = rotation_axis.norm();
  AngleAxisT rotation_aa(rotation_angle, rotation_axis / rotation_angle);
  if (rotation_axis.norm() < T(1e-8)) {
    rotation_aa = AngleAxisT(T(0), Vector3T(T(1), T(0), T(0)));
  }
  const Translation3T translation_tf(
    translation[0], translation[1], translation[2]);
  const Affine3T transform = translation_tf * rotation_aa;
  return transform;
}

class VisualizationCallback : public ceres::IterationCallback {
public:
  explicit VisualizationCallback(const CameraIntrinsics& intrinsics,
                                 const CameraExtrinsics& extrinsics,
                                 const SLAMProblem& problem,
                                 vector<SLAMNodeSolution>* solution) :
      intrinsics(intrinsics),
      extrinsics(extrinsics),
      solution_ptr(solution),
      problem(problem) {
    poses_publisher =
        n.advertise<visualization_msgs::Marker>("slam_poses", 1);
    map_publisher =
        n.advertise<visualization_msgs::Marker>("slam_map", 1);
    frontend_map_publisher =
        n.advertise<visualization_msgs::Marker>("reference_map", 1);
    InitializeMarker(visualization_msgs::Marker::POINTS,
                     Color4f::kWhite,
                     0.03,
                     0.03,
                     0.03,
                     &map);
    InitializeMarker(visualization_msgs::Marker::LINE_LIST,
                     Color4f::kGreen,
                     0.01,
                     0.1,
                     0.1,
                     &poses);
    InitializeMarker(visualization_msgs::Marker::POINTS,
                     Color4f::kWhite,
                     0.01,
                     0.01,
                     0.01,
                     &reference_map);
  }

  ceres::CallbackReturnType operator()(
      const ceres::IterationSummary& summary) {
    PublishVisualization();
    if (shutdown_) {
      printf("Aborting optimization\n");
      return ceres::SOLVER_ABORT;
    }
    return ceres::SOLVER_CONTINUE;
  }

  void PublishVisualization() {
    const vector<SLAMNodeSolution>& solution = *solution_ptr;
    map.points.clear();
    map.colors.clear();
    poses.points.clear();
    poses.colors.clear();
    const Affine3f cam_to_robot = PoseArrayToAffine(
      extrinsics.rotation,
      extrinsics.translation);
    for (const SLAMNodeSolution& n : solution) {
      const Affine3f robot_to_map =
          PoseArrayToAffine(&(n.pose[3]), &(n.pose[0])).cast<float>();
      const Vector3f p0 = robot_to_map * cam_to_robot * Vector3f(0, 0, 0);
      const Vector3f px = robot_to_map * cam_to_robot * Vector3f(0, 0, 0.5);
      const Vector3f py = robot_to_map * cam_to_robot * Vector3f(-0.25, 0, 0);
      AddLine(p0, px, Color4f::kGreen, &poses);
      AddLine(p0, py, Color4f::kRed, &poses);
    }
    vector<Vector3f> map_points;
    GetMap(intrinsics, extrinsics, problem, solution, &map_points);
    for (const Vector3f& p : map_points) {
      AddPoint(p, Color4f(1, 1, 1, 0.2), &map);
    }
    for (int i = 0; i < 1; ++i) {
      poses_publisher.publish(poses);
      map_publisher.publish(map);
      ros::spinOnce();
      // ros::Duration(0.005).sleep();
    }
  }

private:
  ros::NodeHandle n;
  ros::Publisher poses_publisher;
  ros::Publisher map_publisher;
  ros::Publisher frontend_map_publisher;
  visualization_msgs::Marker reference_map;
  visualization_msgs::Marker map;
  visualization_msgs::Marker poses;
  const CameraIntrinsics& intrinsics;
  const CameraExtrinsics& extrinsics;
  vector<SLAMNodeSolution>* solution_ptr;
  const SLAMProblem& problem;
};

void GetMap(const CameraIntrinsics& intrinsics,
            const CameraExtrinsics& extrinsics,
            const SLAMProblem& problem,
            const vector<SLAMNodeSolution>& solution,
            vector<Vector3f>* map_ptr) {
}

bool SolveSLAM(const CameraIntrinsics& intrinsics,
               const CameraExtrinsics& extrinsics,
               const SLAMProblem& slam_problem,
               vector<SLAMNode>* solved_nodes) {
  ceres::Problem ceres_problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  options.num_threads = 12;
  options.minimizer_progress_to_stdout = (FLAGS_v > 2);
  options.max_num_iterations = FLAGS_max_steps;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.function_tolerance = 0;
  options.parameter_tolerance = 0;
  options.gradient_tolerance = 0;
  options.update_state_every_iteration = true;

  vector<SLAMNodeSolution> solution;
  for (size_t i = 0; i < slam_problem.nodes.size(); ++i) {
    CHECK_EQ(i, slam_problem.nodes[i].id);
    solution.push_back(SLAMNodeSolution(slam_problem.nodes[i]));
  }

  int num_feature_matches = 0;
  for (const VisionFactor& f : slam_problem.vision_factors) {
    num_feature_matches += f.feature_matches.size();
  }

  // TODO: Add odometry factors to the problem here.
  // TODO: Add vision factors to the problem here.

  VisualizationCallback visualization_callback(
      intrinsics,
      extrinsics,
      slam_problem,
      &solution);
  options.callbacks.push_back(&visualization_callback);

  for (int i = 0; i < 100; ++i) {
    visualization_callback(ceres::IterationSummary());
    ros::Duration(0.01).sleep();
  }

  ceres::Solve(options, &ceres_problem, &summary);
  cout << summary.FullReport() << "\n";
  printf("Final reprojection root mean squared error: %f\n",
         sqrt(summary.final_cost / static_cast<double>(num_feature_matches)));
  visualization_callback(ceres::IterationSummary());
  return (summary.termination_type == ceres::CONVERGENCE ||
      summary.termination_type == ceres::USER_SUCCESS);
}

}  // namespace slam
