#ifndef __EXTRINSICCALIB_H__
#define __EXTRINSICCALIB_H__

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <experimental/filesystem>
#include <atomic>
#include <sstream>
#include <stdexcept>

bool student_extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder);

#endif
