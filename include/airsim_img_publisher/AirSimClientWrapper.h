#ifndef AirSimClientWrapper_H
#define AirSimClientWrapper_H

#include <stdint.h>
#include <string>
#include <fstream>
// openCV
#include <opencv2/opencv.hpp>
// ros
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
// airsim
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/Common.hpp"

using ImageReq = msr::airlib::ImageCaptureBase::ImageRequest;
using ImageRes = msr::airlib::ImageCaptureBase::ImageResponse;
using ImageTyp = msr::airlib::ImageCaptureBase::ImageType;

struct image_response_decoded
{
	cv::Mat left;
	cv::Mat right;

	cv::Mat depth_front;
	cv::Mat depth_back;
	cv::Mat planar_depth;
	cv::Mat disparity;

	geometry_msgs::Pose pose;
	geometry_msgs::Pose pose_gt; //ground truth
	geometry_msgs::Twist twist;
	bool valid_data = true;
	uint64_t timestamp;
};

struct image_response
{
 std::vector<ImageRes> image;

 msr::airlib::Vector3r p;
 msr::airlib::Quaternionr q;
 msr::airlib::MultirotorState state;

 uint64_t timestamp;
};

class AirSimClientWrapper
{
	public:
		// constructor
		AirSimClientWrapper();
		AirSimClientWrapper(const std::string& ip_addr, uint16_t port);
		AirSimClientWrapper(const std::string& ip_addr, uint16_t port, std::string localization_method);
		// destructor
		~AirSimClientWrapper();

		// *** F:DN Control functions
		void connect();
		void connect(const std::string& ip_addr, uint16_t port);

		// *** F:DN Odometry functions
	  geometry_msgs::Twist twist();

		// *** F:DN Camera functions
		// cv::Mat poll_frame();
		// cv::Mat poll_frame_depth();

	  void do_nothing();
	  void poll_frame(bool);
	  struct image_response_decoded poll_frame_and_decode();
	  struct image_response_decoded image_decode(bool);

	private:
		// airsim api client
		msr::airlib::MultirotorRpcLibClient * client_;
		std::string ip_addr_;
		uint16_t port_;
	  std::string localization_method_;

	  uint64_t timestamp_offset_;
	  bool first_image_ = true;
};

#endif
