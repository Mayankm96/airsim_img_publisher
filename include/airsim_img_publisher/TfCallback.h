#ifndef TfCallback_H
#define TfCallback_H

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include "airsim_img_publisher/QuatRotEuler.h"

// Publish static tf between base_frame_id and camera_frame_id of quadcopter and camera respectively
void fakeStaticCamPosePublisher(const std::string base_frame_id, const std::string camera_frame_id, const ros::Time& timestamp);

// Publish ground truth tf for the position/orientation of the quadcopter and camera (parent frame: 'world')
void groundTruthPosePublisher(const geometry_msgs::Pose& CamPose_gt, const ros::Time& timestamp, const std::string child_frame);

// Publish gps tf for the position/orientation of the quadcopter and camera (parent frame: 'world')
void gpsPosePublisher(const geometry_msgs::Pose& CamPose, const ros::Time& timestamp, const std::string child_frame);

// Publish odom messages (parent frame: 'world')
void odomPublisher(const geometry_msgs::Pose& CamPose_gt, const geometry_msgs::Twist& twist, const ros::Time& timestamp, const ros::Publisher &odom_pub, const std::string child_frame);

#endif
