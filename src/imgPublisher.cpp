#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <iostream>
#include <math.h>
#include <iterator>
#include "common/Common.hpp"
#include <fstream>
#include "input_sampler.h"
#include "Callbacks/callbacks.h"
#include <signal.h>
#include "stereo_msgs/DisparityImage.h"
#include <thread>
#include <mutex>
#include <profile_manager/profiling_data_srv.h>
#include <profile_manager/start_profiling_srv.h>

using namespace std;

// Profiling
long long g_poll_decode_acc = 0;
int g_poll_decode_ctr = 0;
bool CLCT_DATA;

void log_data_before_shutting_down(){

    std::string ns = ros::this_node::getName();
    profile_manager::profiling_data_srv profiling_data_srv_inst;
    
    profiling_data_srv_inst.request.key = "poll_decode";
    profiling_data_srv_inst.request.value = (((double)g_poll_decode_acc)/1e9)/g_poll_decode_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
        }
    }
} 

string localization_method;
//msr::airlib::MultirotorRpcLibClient * client;
extern std::mutex client_mutex;
extern volatile bool exit_out;
void sigIntHandlerPrivate(int sig)
{
    
    log_data_before_shutting_down();
    //my_thread.join(); 
    // client_mutex.lock(); 
    ros::shutdown();
    //abort();
    exit_out = true; 
    std::cout << "killing the main thread" << std::endl;
    std::ofstream myfile;
    myfile.open("/home/wcui/catkin_ws/blah.txt", std::ofstream::app);
    myfile << "killing the main thread" << std::endl;
    myfile.close(); 
    // client_mutex.unlock();
}



sensor_msgs::CameraInfo getCameraParams(){
    double Tx, Fx, Fy, cx, cy, width, height;
    sensor_msgs::CameraInfo CameraParam;

    // Read camera parameters from launch file
    ros::param::get("/airsim_imgPublisher/Tx",Tx);
    ros::param::get("/airsim_imgPublisher/Fx",Fx);
    ros::param::get("/airsim_imgPublisher/Fy",Fy);
    ros::param::get("/airsim_imgPublisher/cx",cx);
    ros::param::get("/airsim_imgPublisher/cy",cy);
    ros::param::get("/airsim_imgPublisher/scale_x",width);
    ros::param::get("/airsim_imgPublisher/scale_y",height);
    ros::param::get("/CLCT_DATA",CLCT_DATA);


    //CameraParam.header.frame_id = "camera";
    CameraParam.header.frame_id = localization_method;

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
    CameraParam.P = {Fx,  0.0, cx,  Tx, 
                     0.0, Fy,  cy,  0.0, 
                     0.0, 0.0, 1.0, 0.0};

    CameraParam.binning_x = 0;
    CameraParam.binning_y = 0;

    return CameraParam;
}

void CameraPosePublisher(geometry_msgs::Pose CamPose, geometry_msgs::Pose CamPose_gt, const ros::Time& timestamp)
{
    static tf::TransformBroadcaster br;
    tf::Transform transformQuad, transformCamera;
    const double sqrt_2 = 1.41421356237;
    transformCamera.setOrigin(tf::Vector3(CamPose.position.y,
                                        CamPose.position.x,
                                        -CamPose.position.z));

    geometry_msgs::Vector3 rpy =  quat2rpy(CamPose.orientation);
    rpy.y = -rpy.y;
    rpy.z = -rpy.z + M_PI/2.0;

    geometry_msgs::Quaternion q_body2cam = setQuat(0.5, -0.5, 0.5, -0.5);

    geometry_msgs::Quaternion q_cam = rpy2quat(rpy);
    q_cam = quatProd(q_body2cam, q_cam);
    transformCamera.setRotation(tf::Quaternion(q_cam.x,
                                             q_cam.y,
                                             q_cam.z, 
                                             q_cam.w));

    if (localization_method == "gps"){ //note that slam itself posts this transform
        br.sendTransform(tf::StampedTransform(transformCamera, timestamp, "world", localization_method));
    }  
    
    
    //ground truth values
    static tf::TransformBroadcaster br_gt;
    tf::Transform transformQuad_gt, transformCamera_gt;
    transformCamera_gt.setOrigin(tf::Vector3(CamPose_gt.position.y,
                                        CamPose_gt.position.x,
                                        -CamPose_gt.position.z));

    geometry_msgs::Vector3 rpy_gt =  quat2rpy(CamPose_gt.orientation);
    rpy_gt.y = -rpy_gt.y;
    rpy_gt.z = -rpy_gt.z + M_PI/2.0;

    geometry_msgs::Quaternion q_body2cam_gt = setQuat(0.5, -0.5, 0.5, -0.5);

    geometry_msgs::Quaternion q_cam_gt = rpy2quat(rpy_gt);
    q_cam_gt = quatProd(q_body2cam_gt, q_cam_gt);
    transformCamera_gt.setRotation(tf::Quaternion(q_cam_gt.x,
                                             q_cam_gt.y,
                                             q_cam_gt.z, 
                                             q_cam_gt.w));
    br_gt.sendTransform(tf::StampedTransform(transformCamera_gt, timestamp, "world", "ground_truth"));
}

void do_nothing(){
    return;
}
//std::thread poll_frame_thread(do_nothing);


int main(int argc, char **argv)
{
  
    
  //Start ROS ----------------------------------------------------------------
  ros::init(argc, argv, "airsim_imgPublisher");
  ros::NodeHandle n;
  ros::Rate loop_rate(20);

    
  //Publishers ---------------------------------------------------------------
  image_transport::ImageTransport it(n);

  // image_transport::Publisher imgL_pub = it.advertise("/Airsim/left/image_raw", 1);
  image_transport::Publisher imgR_pub = it.advertise("/Airsim/right/image_raw", 1);
  image_transport::Publisher depth_pub_front = it.advertise("/Airsim/depth_front", 1);
  image_transport::Publisher depth_pub_back = it.advertise("/Airsim/depth_back", 1);


   ros::Publisher imgParamL_pub = n.advertise<sensor_msgs::CameraInfo> ("/Airsim/left/camera_info", 1);
  ros::Publisher imgParamR_pub = n.advertise<sensor_msgs::CameraInfo> ("/Airsim/right/camera_info", 1);
  ros::Publisher imgParamDepth_pub = n.advertise<sensor_msgs::CameraInfo> ("/Airsim/camera_info", 1);
  ros::Publisher disparity_pub = n.advertise<stereo_msgs::DisparityImage> ("/Airsim/disparity", 1);
  //ROS Messages
  sensor_msgs::ImagePtr msgImgL, msgImgR, msgDepth_front, msgDepth_back;
  sensor_msgs::CameraInfo msgCameraInfo;

  //Parameters for communicating with Airsim
  string ip_addr;
  int portParam;
  ros::param::get("/airsim_imgPublisher/Airsim_ip",ip_addr);
  ros::param::get("/airsim_imgPublisher/Airsim_port", portParam);
  uint16_t port = portParam;

  // Parameter for localizing camera
  if(!ros::param::get("/airsim_imgPublisher/localization_method", localization_method)){
    ROS_FATAL_STREAM("you have not set the localization method");
    return -1;
  }

   //this connects us to the drone 
  //client = new msr::airlib::MultirotorRpcLibClient(ip_addr, port);
  //client->enableApiControl(false);


  //Verbose
  ROS_INFO("Image publisher started! Connecting to:");
  ROS_INFO("IP: %s", ip_addr.c_str());
  ROS_INFO("Port: %d", port);
  
  //Local variables
  input_sampler input_sample__obj(ip_addr.c_str(), port, localization_method);
   msgCameraInfo = getCameraParams();

  bool all_front = false;
  if (!ros::param::get("/airsim_imgPublisher/all_front",all_front)){
      ROS_ERROR_STREAM("all front is not defined for airsim_imgPublisher");
      exit(0);
  }
   
  std::thread poll_frame_thread(&input_sampler::poll_frame, 
          &input_sample__obj, all_front);
  signal(SIGINT, sigIntHandlerPrivate);

  while (ros::ok())
  {
      ros::Time start_hook_t = ros::Time::now();
      auto imgs = input_sample__obj.image_decode(all_front);
      if (!imgs.valid_data) {
          continue;
      }

      uint32_t timestamp_s = uint32_t(imgs.timestamp / 1000000000);
      uint32_t timestamp_ns = uint32_t(imgs.timestamp % 1000000000);
      ros::Time timestamp(timestamp_s, timestamp_ns);
      if(imgs.timestamp != uint64_t(timestamp_s)*1000000000 + timestamp_ns){
          std::cout<<"---------------------failed"<<std::setprecision(30)<<imgs.timestamp<< "!=" 
              <<std::setprecision(30)<<timestamp_s*1000000000 + timestamp_ns<<std::endl;
          ROS_ERROR_STREAM("coversion in img publisher failed");
      }

      cv::Mat disparityImageMat;
      imgs.depth_front.convertTo(disparityImageMat, CV_8UC1);
      imgs.depth_back.convertTo(disparityImageMat, CV_8UC1);
      stereo_msgs::DisparityImage disparityImg;
      disparityImg.header.stamp = timestamp;

      disparityImg.header.frame_id= localization_method;
      //disparityImg.header.frame_id= "camera";

      disparityImg.f = 128; //focal length, half of the image width
      disparityImg.T = .14; //baseline, half of the distance between the two cameras
      disparityImg.min_disparity = .44; // f.t/z(depth max)
      disparityImg.max_disparity = 179; // f.t/z(depth min)
      disparityImg.delta_d = .018; //possibly change
      disparityImg.image = *(cv_bridge::CvImage(std_msgs::Header(), "8UC1", disparityImageMat).toImageMsg());
      disparityImg.valid_window.x_offset = 0;
      disparityImg.valid_window.y_offset = 0;
      disparityImg.valid_window.height =  144;
      disparityImg.valid_window.width =  256;
      disparityImg.valid_window.do_rectify =  false; //possibly change

      // *** F:DN conversion of opencv images to ros images
      // msgImgL = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgs.left).toImageMsg();
      msgImgR = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgs.right).toImageMsg();
      msgDepth_front = cv_bridge::CvImage(std_msgs::Header(), "32FC1", imgs.depth_front).toImageMsg();
      msgDepth_back = cv_bridge::CvImage(std_msgs::Header(), "32FC1", imgs.depth_back).toImageMsg();

      //Stamp messages
      msgCameraInfo.header.stamp = timestamp;
      // msgImgL->header.stamp = msgCameraInfo.header.stamp;
      msgImgR->header.stamp = msgCameraInfo.header.stamp;
      msgDepth_front->header.stamp =  msgCameraInfo.header.stamp;
      msgDepth_back->header.stamp =  msgCameraInfo.header.stamp;

      // Set the frame ids
      msgDepth_front->header.frame_id = localization_method;
      msgDepth_back->header.frame_id = localization_method;

      //Publish transforms into tf tree
      CameraPosePublisher(imgs.pose, imgs.pose_gt, timestamp);

      //Publish images
      imgR_pub.publish(msgImgR);
      depth_pub_front.publish(msgDepth_front);
      depth_pub_back.publish(msgDepth_back);
      imgParamL_pub.publish(msgCameraInfo);
      imgParamR_pub.publish(msgCameraInfo);
      imgParamDepth_pub.publish(msgCameraInfo);
      disparity_pub.publish(disparityImg);

      ros::spinOnce();

      ros::Time end_hook_t = ros::Time::now();

      if (CLCT_DATA) { 
          g_poll_decode_acc += (imgs.poll_time + ((end_hook_t - start_hook_t).toSec()*1e9));
          //ROS_INFO_STREAM("decode "<< (((end_hook_t - start_hook_t).toSec()*1e9))); 
          //ROS_INFO_STREAM("decode "<< imgs.poll_time);
          
          g_poll_decode_ctr++; 
      }
  }
  exit_out = true; 
  poll_frame_thread.join();
  //ros::shutdown(); 
  return 0;
}

