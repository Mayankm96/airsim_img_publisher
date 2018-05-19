#ifndef TfCallback_H
#define TfCallback_H

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include "airsim_img_publisher/QuatRotEuler.h"

//Publish tf for the position/orientation of the quadcopter and camera
void tfCallback(const nav_msgs::Odometry::ConstPtr &msg);

#endif
