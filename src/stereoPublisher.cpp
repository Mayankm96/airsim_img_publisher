#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <iterator>
// pthread
#include <thread>
#include <mutex>
#include <signal.h>
// openCV
#include <opencv2/highgui/highgui.hpp>
// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
// airsim
#include "common/Common.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
// include
#include "airsim_img_publisher/AirSimClientStereoWrapper.h"
#include "airsim_img_publisher/TfCallback.h"
#include "airsim_img_publisher/ImageProcessing.h"

using namespace std;

extern std::mutex client_mutex;
extern volatile bool exit_out;

void sigIntHandler(int sig)
{
    //my_thread.join();
    // client_mutex.lock();
    ros::shutdown();
    //abort();
    exit_out = true;
    std::cout << "killing the main thread" << std::endl;

    std::ofstream myfile;
    std::string filename = ros::package::getPath("airsim_img_publisher") + "/src/debug.txt";
    myfile.open(filename.c_str(), std::ofstream::app);
    myfile << "killing the main thread" << std::endl;
    myfile.close();
    // client_mutex.unlock();
}

sensor_msgs::CameraInfo getCameraParams(int cameraID, double Tx = 0)
{
    double Fx, Fy, cx, cy, width, height;
    sensor_msgs::CameraInfo CameraParam;
    std::string camera_frame_id;

    switch(cameraID)
    {
      // front center camera (cameraID = 0)
      case 0: camera_frame_id = "front_center_optical";
              break;
      // front left camera (cameraID = 1)
      case 1: camera_frame_id = "front_left_optical";
              break;
      // front right camera (cameraID = 2)
      case 2: camera_frame_id = "front_right_optical";
              break;
      // bottom center camera (cameraID = 3)
      case 3: camera_frame_id = "bottom_center_optical";
              break;
      // bottom back camera (cameraID = 3)
      case 4: camera_frame_id = "bottom_back_optical";
              break;
      default: ROS_FATAL("CameraID must be between 0-4");
               exit(1);
    }

    // Read camera parameters from launch file
    ros::param::get("~Fx", Fx);
    ros::param::get("~Fy", Fy);
    ros::param::get("~cx", cx);
    ros::param::get("~cy", cy);
    ros::param::get("~width", width);
    ros::param::get("~height", height);

    CameraParam.header.frame_id = camera_frame_id;
    CameraParam.height = height;
    CameraParam.width = width;
    CameraParam.distortion_model = "plumb_bob";
    CameraParam.D = {0.0, 0.0, 0.0, 0.0, 0.0};
    CameraParam.K = {Fx,  0.0, cx,
                     0.0, Fy,  cy,
                     0.0, 0.0, 1};
    CameraParam.R = {1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0};
    CameraParam.P = {Fx,  0.0, cx,  0.0,
                     0.0, Fy,  cy,  0.0,
                     0.0, 0.0, 1.0, 0.0};

    CameraParam.binning_x = 0;
    CameraParam.binning_y = 0;

    return CameraParam;
}

int main(int argc, char **argv)
{
  //Start ROS ----------------------------------------------------------------
  ros::init(argc, argv, "airsim_stereoPublisher");
  ros::NodeHandle n("~");

  //Parameters ---------------------------------------------------------------

  // loop rate
  double loop_rate_hz;  // loop rate (in Hz)
  ros::param::param<double>("~loop_rate", loop_rate_hz, 50);
  ros::Rate loop_rate(loop_rate_hz);

  // communicating with Airsim
  string ip_addr; // server IP address
  int portParam;  // server port
  ros::param::param<std::string>("~Airsim_ip",ip_addr, "127.0.0.1");
  ros::param::param<int>("~Airsim_port", portParam, 41451);
  // type cast port to uint16 format
  uint16_t port = portParam;

  // localizing camera
  std::string localization_method;
  if(! ros::param::get("~localization_method", localization_method))
  {
    ROS_WARN("Using default locatization method: ground_truth");
    localization_method = "ground_truth";
  }

  // tf tree frame names
  std::string base_frame_id;
  bool tf_cam_flag;
  double Tx;
  ros::param::param<double>("~Tx", Tx, 0.25);
  ros::param::param<bool>("~tf_cam_flag", tf_cam_flag, true);
  ros::param::param<std::string>("~base_frame_id", base_frame_id, "base_link");

  // camera paramters
  sensor_msgs::CameraInfo msgLeftCameraInfo = getCameraParams(1, 0);
  sensor_msgs::CameraInfo msgRightCameraInfo = getCameraParams(2, Tx);

  // Verbose
  ROS_INFO("Image publisher started! Connecting to:");
  ROS_INFO("IP: %s", ip_addr.c_str());
  ROS_INFO("Port: %d", port);
  ROS_INFO("Localization Method: %s", localization_method.c_str());
  ROS_INFO("Depth Baseline: %f", Tx);
  ROS_INFO("Publishing fake drone transformation: %d", tf_cam_flag);

  // Publishers ---------------------------------------------------------------
  image_transport::ImageTransport it(n);

  image_transport::Publisher rgb_pub = it.advertise("/airsim/rgb/image_raw", 1);
  image_transport::Publisher rgb_left_pub = it.advertise("/airsim/left/image_raw", 1);
  image_transport::Publisher rgb_right_pub = it.advertise("/airsim/right/image_raw", 1);
  image_transport::Publisher depth_pub = it.advertise("/airsim/depth_registered/depth", 1);

  ros::Publisher imgParamCam_pub = n.advertise<sensor_msgs::CameraInfo> ("/airsim/camera_info", 1);
  ros::Publisher imgParamRight_pub = n.advertise<sensor_msgs::CameraInfo> ("/airsim/left/camera_info", 1);
  ros::Publisher imgParamLeft_pub = n.advertise<sensor_msgs::CameraInfo> ("/airsim/right/camera_info", 1);
  ros::Publisher imgParamRgb_pub = n.advertise<sensor_msgs::CameraInfo> ("/airsim/rgb/camera_info", 1);
  ros::Publisher imgParamDepth_pub = n.advertise<sensor_msgs::CameraInfo> ("/airsim/depth/camera_info", 1);

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);

  //ROS Messages
  sensor_msgs::ImagePtr msgImgLeft, msgImgRight, msgDepth;

  //Main ---------------------------------------------------------------

  //Local variables
  AirSimClientStereoWrapper airsim_sampler(ip_addr.c_str(), port, localization_method);

  std::thread poll_frame_thread(&AirSimClientStereoWrapper::poll_frame, &airsim_sampler);
  signal(SIGINT, sigIntHandler);

  // *** F:DN end of communication with simulator (Airsim)
  ros::Time last_timestamp(0);
  while (ros::ok())
  {
    ros::Time start_hook_t = ros::Time::now();

    auto imgs = airsim_sampler.image_decode();
    if (!imgs.valid_data)
    {
        continue;
    }

    // image timestamp
    uint32_t timestamp_s = uint32_t(imgs.timestamp / 1000000000);
    uint32_t timestamp_ns = uint32_t(imgs.timestamp % 1000000000);
    ros::Time timestamp(timestamp_s, timestamp_ns);
    if(imgs.timestamp != uint64_t(timestamp_s)*1000000000 + timestamp_ns)
    {
        std::cout << "---------------------failed" << std::setprecision(30) << imgs.timestamp << "!="
                  << std::setprecision(30) << timestamp_s*1000000000 + timestamp_ns << std::endl;
        ROS_ERROR_STREAM("coversion in img publisher failed");
    }

    // *** F:DN conversion of opencv images to ros images
    msgImgLeft = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgs.left_rgb).toImageMsg();
    msgImgRight = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgs.right_rgb).toImageMsg();
    msgDepth = cv_bridge::CvImage(std_msgs::Header(), "32FC1", imgs.depth).toImageMsg();

    //Stamp messages
    msgLeftCameraInfo.header.stamp = timestamp;
    msgRightCameraInfo.header.stamp = timestamp;
    msgImgLeft->header = msgLeftCameraInfo.header;
    msgImgRight->header = msgRightCameraInfo.header;
    msgDepth->header =  msgLeftCameraInfo.header;

    if (timestamp > last_timestamp)
    {
      //Publish images
      rgb_pub.publish(msgImgLeft);
      rgb_left_pub.publish(msgImgLeft);
      rgb_right_pub.publish(msgImgRight);
      depth_pub.publish(msgDepth);

      imgParamCam_pub.publish(msgLeftCameraInfo);
      imgParamLeft_pub.publish(msgLeftCameraInfo);
      imgParamRight_pub.publish(msgRightCameraInfo);
      imgParamRgb_pub.publish(msgLeftCameraInfo);
      imgParamDepth_pub.publish(msgLeftCameraInfo);

      // Publish transforms into tf tree
      // tf from base_frame_id to camera_frame_id
      if(tf_cam_flag)
        fakeStaticStereoCamPosePublisher(base_frame_id, Tx, timestamp);
      // tf from "world" to base_frame_id
      if(localization_method == "gps")
        gpsPosePublisher(imgs.pose, timestamp, base_frame_id);
      else
        groundTruthPosePublisher(imgs.pose_gt, timestamp, base_frame_id);

      // Publish odometry messages
      odomPublisher(imgs.pose_gt, imgs.twist, timestamp, odom_pub, base_frame_id);

      // Update time sequence
      last_timestamp = timestamp;
    }

    ros::spinOnce();

    ros::Time end_hook_t = ros::Time::now();
    ROS_INFO_STREAM("Decoding and publishing frame time: " << end_hook_t - start_hook_t);

    loop_rate.sleep();
  }

  exit_out = true;
  poll_frame_thread.join();

  return 0;
}
