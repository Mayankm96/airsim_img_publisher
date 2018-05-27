#ifndef ImageProcessing_H
#define ImageProcessing_H

#include <stdint.h>
#include <string>
#include <fstream>
// openCV
#include <opencv2/opencv.hpp>

// function to convert to plannar depth data
void convertToPlanDepth(const cv::Mat& input, cv::Mat& output, float f = 128);

// function to convert to disparity images
void convertToDisparity(const cv::Mat& input, cv::Mat& output, float f = 128, float baseline_meters = 0.14);

#endif
