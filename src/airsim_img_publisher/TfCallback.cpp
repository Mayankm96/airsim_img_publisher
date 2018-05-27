#include "airsim_img_publisher/TfCallback.h"

// Publish ground truth tf for the position/orientation of the quadcopter and camera
void groundTruthPosePublisher(geometry_msgs::Pose CamPose_gt, const ros::Time& timestamp, const std::string child_frame)
{
    //ground truth values
    static tf::TransformBroadcaster br_gt;
    tf::Transform transformQuad_gt, transformCamera_gt;
    tf::Transform transformOrig;
    transformCamera_gt.setOrigin(tf::Vector3(CamPose_gt.position.x,
                                        -CamPose_gt.position.y,
                                        -CamPose_gt.position.z));

    transformOrig.setOrigin(tf::Vector3(CamPose_gt.position.x,
                                        -CamPose_gt.position.y,
                                        -CamPose_gt.position.z));

    tf::Quaternion OrigOrientation(CamPose_gt.orientation.x, CamPose_gt.orientation.y, CamPose_gt.orientation.z, CamPose_gt.orientation.w);
    tf::Quaternion orientation(CamPose_gt.orientation.x, CamPose_gt.orientation.y, CamPose_gt.orientation.z, CamPose_gt.orientation.w);

    double oRoll, oPitch, oYaw;
    tf::Matrix3x3(OrigOrientation).getRPY(oRoll, oPitch, oYaw);
    tf::Quaternion rpy_quat = tf::createQuaternionFromRPY(oRoll, -oPitch, -oYaw);
    tf::Quaternion rotated = tf::Quaternion(0, 0, 0.707, -0.707) * rpy_quat;
    tf::Quaternion rotated_2 = tf::Quaternion(0.707, 0, 0, -0.707) * rotated;

    transformOrig.setRotation(rpy_quat);

    tf::Quaternion conv2flu(sin(M_PI/4), 0.0, sin(M_PI/4), 0.0);

    conv2flu *= orientation;

    transformCamera_gt.setRotation(orientation);
    br_gt.sendTransform(tf::StampedTransform(transformOrig, timestamp, "world", child_frame));
}

// Publish gps tf for the position/orientation of the quadcopter and camera
void gpsPosePublisher(geometry_msgs::Pose CamPose, const ros::Time& timestamp, const std::string child_frame)
{
    static tf::TransformBroadcaster br;
    tf::Transform transformQuad;

    transformQuad.setOrigin(tf::Vector3(CamPose.position.y,
                                        CamPose.position.x,
                                        -CamPose.position.z));
    geometry_msgs::Vector3 rpy =  quat2rpy(CamPose.orientation);
    rpy.y = -rpy.y;
    rpy.z = -rpy.z + M_PI/2.0;

    geometry_msgs::Quaternion q_body2cam = setQuat(0.5, -0.5, 0.5, -0.5);

    geometry_msgs::Quaternion q_cam = rpy2quat(rpy);
    q_cam = quatProd(q_body2cam, q_cam);
    transformQuad.setRotation(tf::Quaternion(q_cam.x, q_cam.y, q_cam.z, q_cam.w));

    br.sendTransform(tf::StampedTransform(transformQuad, timestamp, "world", child_frame));
}

// Publish odom messages
void odomPublisher(geometry_msgs::Pose CamPose_gt, geometry_msgs::Twist twist, const ros::Time& timestamp, const ros::Publisher &odom_pub, const std::string child_frame)
{
    // conversion to euler angles
    tf::Quaternion orientation(CamPose_gt.orientation.x, CamPose_gt.orientation.y, CamPose_gt.orientation.z, CamPose_gt.orientation.w);
    double oRoll, oPitch, oYaw;
    tf::Matrix3x3(orientation).getRPY(oRoll, oPitch, oYaw);

    //publish odometry
    nav_msgs::Odometry odom;
    odom.header.stamp = timestamp;
    odom.header.frame_id = "world";
    odom.child_frame_id = child_frame;

    odom.pose.pose.position.x = CamPose_gt.position.x;
    odom.pose.pose.position.y = -CamPose_gt.position.y;
    odom.pose.pose.position.z = -CamPose_gt.position.z;

    odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(oRoll, -oPitch, -oYaw);

    odom.twist.twist = twist;

    odom_pub.publish(odom);
}
