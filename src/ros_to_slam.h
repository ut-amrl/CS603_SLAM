#ifndef __ROS_TO_SLAM_H__
#define __ROS_TO_SLAM_H__

#include "CS603_SLAM/CameraExtrinsics.h"
#include "CS603_SLAM/CameraIntrinsics.h"
#include "CS603_SLAM/RobotPose.h"
#include "CS603_SLAM/SLAMNode.h"
#include "CS603_SLAM/FeatureMatch.h"
#include "CS603_SLAM/OdometryFactor.h"
#include "CS603_SLAM/SLAMProblem.h"
#include "CS603_SLAM/VisionFactor.h"
#include "CS603_SLAM/VisionFeature.h"
#include "slam_types.h"
#include "slam-backend-solver.h"

static slam_types::FeatureMatch
RosToFeatureMatch(const CS603_SLAM::FeatureMatch& ros_feature_match) {
  slam_types::FeatureMatch feature_m;
  feature_m.id_current = ros_feature_match.id_current;
  feature_m.id_initial = ros_feature_match.id_initial;
  return feature_m;
}

static slam_types::VisionFeature
RosToVisionFeature(const CS603_SLAM::VisionFeature& ros_feature) {
  slam_types::VisionFeature feature;
  feature.id = ros_feature.id;
  feature.pixel[0] = ros_feature.pixel.x;
  feature.pixel[1] = ros_feature.pixel.y;
  feature.pixel[2] = ros_feature.pixel.z;
  feature.point3d.x() = ros_feature.point3d.x;
  feature.point3d.y() = ros_feature.point3d.y;
  feature.point3d.z() = ros_feature.point3d.z;
//   CHECK(std::isfinite(feature.point3d.x()));
//   CHECK(std::isfinite(feature.point3d.y()));
//   CHECK(std::isfinite(feature.point3d.z()));
  return feature;
}

static slam_types::RobotPose
RosToRobotPose(const CS603_SLAM::RobotPose& ros_pose) {
  auto loc = ros_pose.loc;
  auto orient = ros_pose.angle;
  slam_types::RobotPose pose = slam_types::RobotPose(Eigen::Vector3f(loc.x,
                                                                     loc.y,
                                                                     loc.z),
                                                 Eigen::Quaternionf(orient.w,
                                                                    orient.x,
                                                                    orient.y,
                                                                    orient.z));
  return pose;
}

static slam_types::SLAMNode
RosToSlamNode(const CS603_SLAM::SLAMNode& ros_node) {
  slam_types::SLAMNode node;
  node.id = ros_node.id;
  node.timestamp = ros_node.timestamp;
  node.pose = RosToRobotPose(ros_node.pose);
  for (auto rfeature : ros_node.features) {
    node.features.push_back(RosToVisionFeature(rfeature));
  }
  return node;
}

static slam_types::OdometryFactor
RosToOdometryFactor(const CS603_SLAM::OdometryFactor& ros_odom) {
  slam_types::OdometryFactor odom;
  odom.pose_i = ros_odom.pose_i;
  odom.pose_j = ros_odom.pose_j;
  auto orient = ros_odom.rotation;
  odom.rotation = Eigen::AngleAxisf(Eigen::Quaternionf(orient.w,
                                                       orient.x,
                                                       orient.y,
                                                       orient.z));
  auto loc = ros_odom.translation;
  odom.translation = Eigen::Vector3f(loc.x, loc.y, loc.z);
  return odom;
}

static slam_types::VisionFactor
RosToVisionFactor(const CS603_SLAM::VisionFactor& ros_vfactor) {
  slam_types::VisionFactor vfactor;
  vfactor.pose_current = ros_vfactor.pose_current;
  vfactor.pose_initial = ros_vfactor.pose_initial;
  for (auto ros_feature_match : ros_vfactor.feature_matches) {
    vfactor.feature_matches.push_back(RosToFeatureMatch(ros_feature_match));
  }
  return vfactor;
}

void  RosToSLAMProblem(const CS603_SLAM::SLAMProblem& ros_problem,
                       slam_types::SLAMProblem* problem) {
  problem->nodes.clear();
  problem->vision_factors.clear();
  problem->odometry_factors.clear();
  for (auto ros_node : ros_problem.nodes) {
    problem->nodes.push_back(RosToSlamNode(ros_node));
  }
  for (auto ros_vision : ros_problem.vision_factors) {
    problem->vision_factors.push_back(RosToVisionFactor(ros_vision));
  }
  for (auto ros_odom : ros_problem.odometry_factors) {
    problem->odometry_factors.push_back(RosToOdometryFactor(ros_odom));
  }
}


static slam::CameraIntrinsics
RosToIntrinsics(const CS603_SLAM::CameraIntrinsics& ros_k) {
  slam::CameraIntrinsics k;
  k.fx = ros_k.fx;
  k.cx = ros_k.cx;
  k.fy = ros_k.fy;
  k.cy = ros_k.cy;
  return k;
}

static slam::CameraExtrinsics
RosToExtrinsics(const CS603_SLAM::CameraExtrinsics& ros_a) {
  slam::CameraExtrinsics a;
  for (int i = 0; i < 3; ++i) {
    a.translation[i] = ros_a.translation[i];
    a.rotation[i] = ros_a.rotation[i];
  }
  return a;
}


#endif  // __SLAM_TO_ROS_H__
