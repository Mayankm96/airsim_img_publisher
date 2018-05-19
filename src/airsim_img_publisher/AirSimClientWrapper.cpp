#include "airsim_img_publisher/AirSimClientWrapper.h"

#include <iostream>
#include <vector>
#include <cstring>
#include <mutex>
#include <chrono>
// ros
#include <ros/package.h>
// airsim
#include "common/Common.hpp"
#include "common/VectorMath.hpp"

using namespace std::chrono;

std::mutex image_response_queue_mutex;
std::queue<struct image_response> image_response_queue;
std::mutex client_mutex;
volatile bool exit_out = false;

static void convertToPlanDepth(const cv::Mat& input, cv::Mat& output, float f = 128)
{
	int width = input.cols;
	int height = input.rows;

	float center_i = width / 2.0f - 1;
	float center_j = height / 2.0f - 1;

  // output = cv::Mat(height, width, CV_32FC1);

	for (int i = 0; i < width; ++i)
	{
		for (int j = 0; j < height; ++j)
		{
			float dist = std::sqrt((i - center_i)*(i - center_i) + (j - center_j)*(j - center_j));
			float denom = (dist / f);
			denom *= denom;
			denom = std::sqrt(1 + denom);
			output.at<float>(j, i) = input.at<float>(j, i) / denom;
		}
	}
}

static void convertToDisparity(const cv::Mat& input, cv::Mat& output, float f = 128, float baseline_meters = 0.14)
{
	int width = input.cols;
	int height = input.rows;
	int size = width * height;

	output = cv::Mat(height, width, CV_32FC1);

	for (int i = 0; i < width; ++i)
	{
		for (int j = 0; j < height; ++j)
		{
			output.at<float>(j, i) = f * baseline_meters / input.at<float>(j, i);
		}
	}
}

AirSimClientWrapper::AirSimClientWrapper() : client_(0), localization_method_("ground_truth"), ip_addr_("not provided"), port_(000)
{
	connect();
}

AirSimClientWrapper::AirSimClientWrapper(const std::string& ip_addr, uint16_t port) : client_(0), localization_method_("ground_truth")
{
  this->ip_addr_ = ip_addr;
  this->port_ = port;
  connect(ip_addr, port);
}

AirSimClientWrapper::AirSimClientWrapper(const std::string& ip_addr, uint16_t port, std::string localization_method) : client_(0)
{
  this->ip_addr_ = ip_addr;
  this->port_ = port;
  this->localization_method_ = localization_method;
	connect(ip_addr, port);
}

AirSimClientWrapper::~AirSimClientWrapper()
{
	if (client_ != 0)
		delete client_;
}

void AirSimClientWrapper::connect()
{
	if (client_ != 0)
		delete client_;
	client_ = new msr::airlib::MultirotorRpcLibClient();
}

void AirSimClientWrapper::connect(const std::string& ip_addr, uint16_t port)
{
	if (client_ != 0)
		delete client_;
	client_ = new msr::airlib::MultirotorRpcLibClient(ip_addr, port);
}

void AirSimClientWrapper::poll_frame(bool all_front)
{
    static uint64 last_time_stamp = 0;
    const int max_tries = 1000000;

    ImageTyp image_type;
    int cameraId;
    if (all_front) {
         cameraId = 0;
         image_type = ImageTyp::Scene;
    }else{
         cameraId = 4;
         image_type = ImageTyp::DepthPlanner;
    }
    std::vector<ImageReq> request = {ImageReq(cameraId, image_type),
	    ImageReq(0, ImageTyp::DepthPlanner)};

    try{
        struct image_response response;
        while(true){
            ros::Time start_hook_t = ros::Time::now();
            client_mutex.lock();
            if (exit_out) {
                std::cout << "killing the poll thread" << std::endl;
                client_mutex.unlock();
                return;
            }

            response.image = client_->simGetImages(request);
            response.p = client_->getPosition();
            response.q = client_->getOrientation();

            for (int i = 0; response.image.size() != request.size() && i < max_tries; i++) {
                response.image = client_->simGetImages(request);
            }

						response.timestamp = response.image.at(0).time_stamp;

            if (last_time_stamp >= response.timestamp) {
                ROS_ERROR_STREAM("imag time stamps shouldn't be out of order"<< last_time_stamp<< " " <<response.timestamp<< " ");
            }
            last_time_stamp = response.timestamp;

            image_response_queue_mutex.lock();
            if (response.image.size() == request.size()) {
                image_response_queue.push(response);
            }
            image_response_queue_mutex.unlock();

            client_mutex.unlock();
        }
    }
    catch(...){
        printf("got here");
        return;
        exit(0);
    }
}

void AirSimClientWrapper::do_nothing(void) {
    while(true){
      ;
    }
}

struct image_response_decoded AirSimClientWrapper::image_decode(bool all_front){
    try
		{
        image_response_queue_mutex.lock();
        if (image_response_queue.empty())
				{
            image_response_queue_mutex.unlock();
            struct image_response_decoded result;
            result.valid_data = false;
            return  result;
        }

        struct image_response response = image_response_queue.back();
        std::queue<struct image_response>().swap(image_response_queue);
        image_response_queue_mutex.unlock();

        struct image_response_decoded result;

#if CV_MAJOR_VERSION==3
        // result.left = cv::imdecode(response.at(0).image_data_uint8, cv::IMREAD_COLOR);
        result.depth_front = cv::imdecode(response.image.at(1).image_data_uint8, cv::IMREAD_GRAYSCALE);

        if(all_front)
				{
            result.right = cv::imdecode(response.image.at(0).image_data_uint8, cv::IMREAD_COLOR);
        }
				else
				{
            result.depth_back = cv::imdecode(response.image.at(0).image_data_uint8, cv::IMREAD_GRAYSCALE);
        }

#else
        // result.left = cv::imdecode(response.at(0).image_data_uint8, CV_LOAD_IMAGE_COLOR);
        result.depth_front = cv::imdecode(response.image.at(1).image.image_data_uint8, CV_LOAD_IMAGE_GRAYSCALE);

        if (all_front)
				{
            result.right = cv::imdecode(response.image.at(0).image_data_uint8, CV_LOAD_IMAGE_COLOR);
        }
				else
				{
            result.depth_back = cv::imdecode(response.image.at(0).image.image_data_uint8, CV_LOAD_IMAGE_GRAYSCALE);
        }
#endif

        result.depth_front.convertTo(result.depth_front, CV_32FC1, 25.6/256);

        if (!all_front)
				{
            result.depth_back.convertTo(result.depth_back, CV_32FC1, 25.6/256);
        }

        //ground truth values
        static auto initial_pos_gt = response.image.back().camera_position;
        result.pose_gt.position.x = response.image.back().camera_position.x() - initial_pos_gt.x();
        result.pose_gt.position.y = response.image.back().camera_position.y() - initial_pos_gt.y();
        result.pose_gt.position.z = response.image.back().camera_position.z() - initial_pos_gt.z();

        result.pose_gt.orientation.x = response.image.back().camera_orientation.x();
        result.pose_gt.orientation.y = response.image.back().camera_orientation.y();
        result.pose_gt.orientation.z = response.image.back().camera_orientation.z();
        result.pose_gt.orientation.w = response.image.back().camera_orientation.w();

        static msr::airlib::Vector3r initial_pos_gps = client_->getPosition();

        if(this->localization_method_ == "gps") {
            result.pose.position.x = response.p.x() - initial_pos_gps.x();
            result.pose.position.y = response.p.y() - initial_pos_gps.y();
            result.pose.position.z = response.p.z() - initial_pos_gps.z();

            result.pose.orientation.x = response.q.x();
            result.pose.orientation.y = response.q.y();
            result.pose.orientation.z = response.q.z();
            result.pose.orientation.w = response.q.w();
        }

        result.timestamp = response.timestamp;
        return result;
    }
    catch(...)
		{
        exit(0);
    }
}


struct image_response_decoded AirSimClientWrapper::poll_frame_and_decode()
{
		std::string file_to_output_name = ros::package::getPath("airsim_img_publisher") + "/src/timing.txt";
    file_to_output_.open(file_to_output_name.c_str(), std::ios_base::app);

    steady_clock::time_point allf_s; //total function time s
    steady_clock::time_point allf_e; //total function time e
    steady_clock::time_point partf_s; //one invocation of tracker start
    steady_clock::time_point partf_e; //one invocation of tracker end
    steady_clock::time_point partf2_s; //one invocation of tracker start
    steady_clock::time_point partf2_e; //one invocation of tracker end

		allf_s  = steady_clock::now();

    struct image_response_decoded result;
    const int max_tries = 1000000;

    std::vector<ImageReq> request = {
        // ImageRequest(0, ImageType::Scene),
        ImageReq(1, ImageTyp::Scene),
        ImageReq(1, ImageTyp::DepthPlanner)
    };

    //result.twist = twist();

    partf_s = steady_clock::now();
    std::vector<ImageRes> response = client_->simGetImages(request);

    partf_e = steady_clock::now();
    auto partf_t = duration_cast<milliseconds>(partf_e - partf_s).count();




    for (int i = 0; response.size() != request.size() && i < max_tries; i++) {
        response = client_->simGetImages(request);
    }


    partf2_s  = steady_clock::now();

    if (response.size() == request.size()) {
#if CV_MAJOR_VERSION==3
        // result.left = cv::imdecode(response.at(0).image_data_uint8, cv::IMREAD_COLOR);
        result.right = cv::imdecode(response.at(0).image_data_uint8, cv::IMREAD_COLOR);
        result.depth_front = cv::imdecode(response.at(1).image_data_uint8, cv::IMREAD_GRAYSCALE);
#else
        // result.left = cv::imdecode(response.at(0).image_data_uint8, CV_LOAD_IMAGE_COLOR);
        result.right = cv::imdecode(response.at(0).image_data_uint8, CV_LOAD_IMAGE_COLOR);
        result.depth_front = cv::imdecode(response.at(1).image_data_uint8, CV_LOAD_IMAGE_GRAYSCALE);
#endif

        /*
           int width = response.at(1).width;
           int height = response.at(1).height;
           std::vector<float>& floats = response.at(1).image_data_float;

           result.depth = cv::Mat(height, width, CV_32FC1);
           for (int i = 0; i < height; i++) {
           for (int j = 0; j < width; j++) {
           float dist = floats[i*width + j];
           result.depth.at<float>(i,j) = dist;
           }
           }
           */

        result.depth_front.convertTo(result.depth_front, CV_32FC1, 25.6/256);

        // result.planar_depth = cv::Mat(height, width, CV_32FC1);
        // convertToPlanDepth(result.depth, result.depth);

        // result.disparity = cv::Mat(height, width, CV_32FC1);
        // convertToDisparity(result.depth, result.disparity);
    } else {
        std::cerr << "Images not returned successfully" << std::endl;
    }
    partf2_e  = steady_clock::now();
    auto partf2_t = duration_cast<milliseconds>(partf2_e - partf2_s).count();

    //ground truth values
    static auto initial_pos_gt= response.back().camera_position;
    result.pose_gt.position.x = response.back().camera_position.x() - initial_pos_gt.x();
    result.pose_gt.position.y = response.back().camera_position.y() - initial_pos_gt.y();
    result.pose_gt.position.z = response.back().camera_position.z() - initial_pos_gt.z();

    result.pose_gt.orientation.x = response.back().camera_orientation.x();
    result.pose_gt.orientation.y = response.back().camera_orientation.y();
    result.pose_gt.orientation.z = response.back().camera_orientation.z();
    result.pose_gt.orientation.w = response.back().camera_orientation.w();

    if(this->localization_method_ == "gps") {
        static auto initial_pos_gps = client_->getPosition();
        auto p = client_->getPosition();
        auto q = client_->getOrientation();
        result.pose.position.x = p.x() - initial_pos_gps.x();
        result.pose.position.y = p.y() - initial_pos_gps.y();
        result.pose.position.z = p.z() - initial_pos_gps.z();

        result.pose.orientation.x = q.x();
        result.pose.orientation.y = q.y();
        result.pose.orientation.z = q.z();
        result.pose.orientation.w = q.w();
    }


    allf_e = steady_clock::now();
    auto allf_t = duration_cast<milliseconds>(allf_e - allf_s).count();
    file_to_output_<<"part_f"<<partf_t<< std::endl;
    file_to_output_<<"part2_f"<<partf2_t<< std::endl;
    file_to_output_<<"all_f"<<allf_t<< std::endl;
    file_to_output_.close();

    return result;
}

geometry_msgs::Twist AirSimClientWrapper::twist()
{
    geometry_msgs::Twist result;
    auto lv = client_->getVelocity();

    // Set linear velocities
    result.linear.x = lv.y();
    result.linear.y = lv.x();
    result.linear.z = -(lv.z());

    // Set angular velocities (we can't currently get them, so just assume they're all 0)
    result.angular.x = 0;
    result.angular.y = 0;
    result.angular.z = 0;

    return result;
}
