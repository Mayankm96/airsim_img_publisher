#include "airsim_img_publisher/ImageProcessing.h"

void convertToPlanDepth(const cv::Mat& input, cv::Mat& output, float f)
{
	int width = input.cols;
	int height = input.rows;

	float center_i = width / 2.0f - 1;
	float center_j = height / 2.0f - 1;

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

void convertToDisparity(const cv::Mat& input, cv::Mat& output, float f, float baseline_meters)
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
