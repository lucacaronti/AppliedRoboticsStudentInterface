#include <iostream>
#include <ctime>
#include "sampling.hpp"

// library for plots
#define CVPLOT_HEADER_ONLY
#include <CvPlot/cvplot.h>

void plot(const std::vector<cv::Point2d> points){
    auto axes = CvPlot::makePlotAxes();
    for(auto itvp = points.begin(); itvp != points.end(); itvp++ ){
        axes.create<CvPlot::Series>(*itvp);
    }
    cv::Mat mat = axes.render(300, 400);
    cv::imshow("mywindow", mat);
    cv::waitKey();
}

cv::Point2d Sampling::generate_rnd_point(const int max_X, const int max_Y){
    double x = int(rand() % max_X);
    double y = int(rand() % max_Y);
    std::cout<<"x: "<<x<<" y: "<<y<<std::endl;
    return cv::Point2d(x,y);
}

std::vector<cv::Point2d> Sampling::generate_N_rnd_points(const int N, const int max_X, const int max_Y){
    std::vector<cv::Point2d> rnd_points;
    for(int i = 0; i < N; i++){
        rnd_points.emplace_back(this->generate_rnd_point(max_X, max_Y));
    }
    return rnd_points;
}
Sampling::Sampling(){
    srand((unsigned) time(0));
}
Sampling::~Sampling(){

}

int main(){
        
    Sampling s;
    std::vector<cv::Point2d> points;
    points = s.generate_N_rnd_points(100,100,100);
    plot(points);
}