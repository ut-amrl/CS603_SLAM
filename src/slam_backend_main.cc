#include <signal.h>
#include <stdio.h>
#include <string>
#include <algorithm>
#include <vector>


#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "CS603_SLAM/SLAMProblem.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "slam_types.h"
#include "ros_to_slam.h"
#include "slam_backend_solver.h"

using Eigen::AngleAxisf;
using Eigen::Matrix3f;
using Eigen::Vector3f;
using slam::CameraExtrinsics;
using slam::CameraIntrinsics;
using std::string;
using std::endl;
using std::vector;

DECLARE_int32(v);
DEFINE_string(input, "", "Input dataset file");
DEFINE_bool(visualize, false, "Whether to visualize the data or not");
DEFINE_string(problem_topic, "slam_problem", "Name of the SLAMProblem topic");

void LoadData(const char* file,
              CameraExtrinsics* extrinsics,
              CameraIntrinsics* intrinsics,
              slam_types::SLAMProblem* problem) {
  // Load bag file into proper structures
  rosbag::Bag input_bag;
  try {
    input_bag.open(file, rosbag::BagMode::Read);
  } catch(rosbag::BagException exception) {
    LOG(ERROR) << "Problem loading bag file " << file << " reason:\n"
        << exception.what() << std::endl;
  }
  std::vector<std::string> topics;
  topics.push_back(FLAGS_problem_topic.c_str());
  topics.push_back("intrinsics");
  topics.push_back("extrinsics");
  rosbag::View input_view(input_bag, rosbag::TopicQuery(topics));
  bool loaded_slam = false;
  bool intrinsics_loaded = false;
  bool extrinsics_loaded = false;
  for (rosbag::View::iterator it = input_view.begin();
       ros::ok() && it != input_view.end();
       ++it) {
    const rosbag::MessageInstance& message = *it;
    {
      const CS603_SLAM::SLAMProblemPtr slam_problem_msg =
          message.instantiate<CS603_SLAM::SLAMProblem>();
      if (slam_problem_msg != NULL) {
        RosToSLAMProblem(*slam_problem_msg, problem);
        loaded_slam = true;
      }
    }
    {
      const CS603_SLAM::CameraExtrinsicsPtr ext_msg =
          message.instantiate<CS603_SLAM::CameraExtrinsics>();
      if (ext_msg != NULL) {
        *extrinsics = RosToExtrinsics(*ext_msg);
        extrinsics_loaded = true;
      }
    }
    {
      const CS603_SLAM::CameraIntrinsicsPtr int_msg =
          message.instantiate<CS603_SLAM::CameraIntrinsics>();
      if (int_msg != NULL) {
        *intrinsics = RosToIntrinsics(*int_msg);
        intrinsics_loaded = true;
      }
    }
  }
  if (loaded_slam) {
    printf("Loaded SLAM problem with %d nodes, %d odometry factors, "
           "%d vision factors (%.2f/pose avg)\n",
           static_cast<int>(problem->nodes.size()),
           static_cast<int>(problem->odometry_factors.size()),
           static_cast<int>(problem->vision_factors.size()),
           static_cast<float>(problem->vision_factors.size()) /
              static_cast<float>((problem->nodes.size() - 1)));
  } else {
    fprintf(stderr, "ERROR: no SLAM problem loaded!\n");
    exit(1);
  }
  if (!intrinsics_loaded || !extrinsics_loaded) {
    fprintf(stderr, "ERROR: camera params not loaded!\n");
    exit(1);
  }
}

void SignalHandler(int signum) {
  printf("Exiting with signal %d\n", signum);
  slam::shutdown_ = true;
  exit(0);
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_input == "") {
    fprintf(stderr, "ERROR: Must specify input file!\n");
    return 1;
  }

  // Initialize ROS.
  ros::init(argc, argv, "slam_backend");
  ros::NodeHandle n;

  signal(SIGINT, SignalHandler);

  // Load dataset.
  slam_types::SLAMProblem problem;
  CameraExtrinsics extrinsics;
  CameraIntrinsics intrinsics;
  LoadData(FLAGS_input.c_str(), &extrinsics, &intrinsics, &problem);

  if (true) {
    printf("WARNING: DISABLING Z-axis motion from odometry!\n");
    for (slam_types::SLAMNode& n : problem.nodes) {
      n.pose.loc.z() = 0;
    }
  }

  // Play the data through the SLAM backend solver.
  std::vector<slam_types::SLAMNode> solution;
  slam::SolveSLAM(intrinsics, extrinsics, problem, &solution);

  return 0;
}
