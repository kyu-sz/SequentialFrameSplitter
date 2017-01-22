#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

void MakeColorWheel(std::vector<cv::Scalar> &color_wheel);
void MotionToColor(cv::Mat flow, cv::Mat &color);