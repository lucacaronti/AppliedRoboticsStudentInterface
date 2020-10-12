#ifndef __PROCESSMAP_HPP__
#define __PROCESSMAP_HPP__

#include <opencv2/opencv.hpp>
#include "student_image_elab_interface.hpp"
#include <vector>
#include <iostream>

namespace student_processMap{
    bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder);
}



#endif
