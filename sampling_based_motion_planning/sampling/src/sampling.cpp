#include <iostream>
#include <ctime>
#include <map>
#include <math.h>
#include "sampling.hpp"
#include "halton.hpp"


cv::Point2d Sampling::generate_rnd_point(const int max_X, const int max_Y){
    double x = double(rand() % max_X);
    double y = double(rand() % max_Y);
    return cv::Point2d(x,y);
}

std::vector<cv::Point2d> Sampling::generate_N_rnd_points(const int N, const int max_X, const int max_Y){
    std::vector<cv::Point2d> rnd_points;
    for(int i = 0; i < N; i++){
        rnd_points.emplace_back(this->generate_rnd_point(max_X, max_Y));
    }
    return rnd_points;
}

std::vector<cv::Point2d> Sampling::generate_N_Halton_points(const int N){
    
    double *num_halton[N];
    std::vector<cv::Point2d> points;
    for (int i = 0; i < N; i++)
    {
        num_halton[i] = halton(i + 1, 2);
        points.push_back(cv::Point2d(num_halton[i][0], num_halton[i][1]));
    }
    for (int i = 0; i < N; i++)
        delete num_halton[i];

    return points;
}

Sampling::Sampling(){
    srand((unsigned) time(0));
}
Sampling::~Sampling(){

}