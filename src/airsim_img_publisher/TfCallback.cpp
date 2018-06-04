#include "airsim_img_publisher/TfCallback.h"

// Publish static tf between base_frame_id and camera_frame_id of quadcopter and camera respectively
void fakeStaticCamPosePublisher(const std::string base_frame_id, const std::string camera_frame_id, const int cameraID, const ros::Time& timestamp)
{
  static tf::TransformBroadcaster br_cam;
  tf::Transform transformCam;

  transformCam.setOrigin(tf::Vector3(0, 0, 0));

  // align the frame between base_link and cam
  tf::Quaternion orientation;
  if(cameraID != 3)
  {
    // look-in-front camera orientation
    orientation = tf::Quaternion(0.707, 0, 0, -0.707) * tf::Quaternion(0, 0, 0.707, -0.707);
  }
  else
  {
    // look-towards-ground camera orientation
    orientation = tf::Quaternion(0.707, 0.707, 0, 0) * tf::Quaternion(0, 0, 0, 1);
  }
  transformCam.setRotation(orientation);

  br_cam.sendTransform(tf::StampedTransform(transformCam, timestamp, base_frame_id, camera_frame_id));
}

// Publish static tf between base_frame_id, camera_center_frame_id and stereo frames of quadcopter and camera respectively
void fakeStaticStereoCamPosePublisher(const std::string base_frame_id, const std::string camera_center_frame_id, const ros::Time& timestamp, const double Tx)
{
  static tf::TransformBroadcaster br_cam, br_left_cam, br_right_cam;
  tf::Transform transformCam, transformLeftCam, transformRightCam;

  // align the frame between base_link and cam with look-towards-ground camera orientation
  transformCam.setOrigin(tf::Vector3(0, 0, 0));
  tf::Quaternion orientation = tf::Quaternion(0.707, 0.707, 0, 0) * tf::Quaternion(0, 0, 0, 1);
  transformCam.setRotation(orientation);
  // left optical camera frame
  transformLeftCam.setOrigin(tf::Vector3(0, 0, 0));
  transformLeftCam.setRotation(tf::Quaternion(1, 0, 0, 0));
  // right optical camera frame
  transformRightCam.setOrigin(tf::Vector3(Tx, 0, 0));
  transformRightCam.setRotation(tf::Quaternion(1, 0, 0, 0));

  br_cam.sendTransform(tf::StampedTransform(transformCam, timestamp, base_frame_id, camera_center_frame_id));
  br_left_cam.sendTransform(tf::StampedTransform(transformLeftCam, timestamp, camera_center_frame_id, "left_optical_frame"));
  br_right_cam.sendTransform(tf::StampedTransform(transformRightCam, timestamp, camera_center_frame_id, "right_optical_frame"));
}

// Publish ground truth tf for the position/orientation of the quadcopter and camera
void groundTruthPosePublisher(const geometry_msgs::Pose& CamPose_gt, const ros::Time& timestamp, const std::string frame)
{
  //ground truth values
  static tf::TransformBroadcaster br_gt;
  tf::Transform transformQuad_gt;

  transformQuad_gt.setOrigin(tf::Vector3(CamPose_gt.position.x,
                                      -CamPose_gt.position.y,
                                      -CamPose_gt.position.z));

  tf::Quaternion quadOrientation(CamPose_gt.orientation.x, CamPose_gt.orientation.y, CamPose_gt.orientation.z, CamPose_gt.orientation.w);

  double oRoll, oPitch, oYaw;
  tf::Matrix3x3(quadOrientation).getRPY(oRoll, oPitch, oYaw);
  tf::Quaternion rpy_quat = tf::createQuaternionFromRPY(oRoll, -oPitch, -oYaw);
  transformQuad_gt.setRotation(rpy_quat);

  br_gt.sendTransform(tf::StampedTransform(transformQuad_gt, timestamp, "world", frame));
}

// Publish gps tf for the position/orientation of the quadcopter and camera
void gpsPosePublisher(const geometry_msgs::Pose& CamPose, const ros::Time& timestamp, const std::string frame)
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

  br.sendTransform(tf::StampedTransform(transformQuad, timestamp, "world", frame));
}

// Publish odom messages
void odomPublisher(const geometry_msgs::Pose& CamPose_gt, const geometry_msgs::Twist& twist, const ros::Time& timestamp, const ros::Publisher &odom_pub, const std::string frame)
{
  // conversion to euler angles
  tf::Quaternion orientation(CamPose_gt.orientation.x, CamPose_gt.orientation.y, CamPose_gt.orientation.z, CamPose_gt.orientation.w);
  double oRoll, oPitch, oYaw;
  tf::Matrix3x3(orientation).getRPY(oRoll, oPitch, oYaw);

  //publish odometry
  nav_msgs::Odometry odom;
  odom.header.stamp = timestamp;
  odom.header.frame_id = "world";
  odom.child_frame_id = frame;

  odom.pose.pose.position.x = CamPose_gt.position.x;
  odom.pose.pose.position.y = -CamPose_gt.position.y;
  odom.pose.pose.position.z = -CamPose_gt.position.z;

  odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(oRoll, -oPitch, -oYaw);

  odom.twist.twist = twist;

  odom_pub.publish(odom);
}
