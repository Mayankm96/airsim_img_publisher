#ifndef TfCallback_H
#define TfCallback_H

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include "airsim_img_publisher/QuatRotEuler.h"

// Publish ground truth tf for the position/orientation of the quadcopter and camera
void groundTruthPosePublisher(geometry_msgs::Pose CamPose_gt, const ros::Time& timestamp, const std::string child_frame = "ground_truth");

// Publish gps tf for the position/orientation of the quadcopter and camera
void gpsPosePublisher(geometry_msgs::Pose CamPose, const ros::Time& timestamp, const std::string child_frame = "gps");

// Publish odom messages
void odomPublisher(geometry_msgs::Pose CamPose_gt, geometry_msgs::Twist twist, const ros::Time& timestamp, const ros::Publisher &odom_pub, const std::string child_frame = "ground_truth");

#endif
