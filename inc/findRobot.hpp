#ifndef __FINDROBOT_H__
#define __FINDROBOT_H__

#include <opencv2/opencv.hpp>
#include "student_image_elab_interface.hpp"
#include <string>
#include "extrinsicCalib.hpp"

bool student_findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder);


#endif
