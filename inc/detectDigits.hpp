#ifndef __DETECTDIGITS_HPP__
#define __DETECTDIGITS_HPP__

#include <opencv2/opencv.hpp>
#include "student_image_elab_interface.hpp"
#include <vector>
#include <iostream>

void detectDigits(cv::Mat image, cv::Mat greenObjs);
int detectSingleDigit(cv::Rect Rect, cv::Mat img, cv::Mat greenObjs, std::vector<std::pair<cv::Mat, int> > templates);
std::vector<std::pair<cv::Mat, int> > augmentTemplates(std::string templatesFolder);

#endif
