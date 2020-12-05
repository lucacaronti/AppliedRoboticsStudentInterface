#include <iostream>
#include <ctime>
#include "sampling.hpp"

// #include "hammersley.hpp"
#include "halton.hpp"
#include <fstream>
// library for plots
#define CVPLOT_HEADER_ONLY
// #include <CvPlot/cvplot.h>

// void plot(const std::vector<cv::Point2d> points){
//     auto axes = CvPlot::makePlotAxes();
//     for(auto itvp = points.begin(); itvp != points.end(); itvp++ ){
//         std::vector<cv::Point2d> tmp_point;
//         tmp_point.push_back(*itvp);
//         axes.create<CvPlot::Series>(tmp_point, "k-o");
//     }
//     cv::Mat mat = axes.render(800, 800);
//     cv::imshow("mywindow", mat);
//     cv::waitKey();
//     cv::destroyWindow("mywindow");
// }

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
    // points = s.generate_N_rnd_points(1000,100,100);

    ofstream halton_file;
    halton_file.open("halton.txt");


    int size_halton = 100000;
    int base[2];
    base[0] = 2;
    base[0] = 2;
    double *num_halton[size_halton];
    for(int i = 0; i < size_halton; i++){
        // num_halton[i] = halton_base(i+1, 2, base);
        num_halton[i] = halton(i+1, 2);
        halton_file << num_halton[i][0]<<" "<<num_halton[i][1]<<"\n";
    }

    halton_file.close();
    for(int i = 0 ; i < size_halton; i++)
        delete num_halton[i];


    // ofstream hammersley_file;
    // hammersley_file.open("hammersley.txt");

    // // int size_hammersley = 1000;
    // // double *num_hammersley[size_hammersley];
    // // for(int i = 0; i < size_hammersley; i++){
    // //     num_hammersley[i] = hammersley(int(rand() % 1000), 2, 2);
    // //     hammersley_file << num_hammersley[i][0]<<" "<<num_hammersley[i][1]<<"\n";
    // // }
    // int size_hammersley = 500;
    // double *num_hammersley;
    // num_hammersley = hammersley_sequence(0,size_hammersley,2,2);
    // for(int i = 0; i < (size_hammersley-1)*2; i+=2){
    //     hammersley_file << num_hammersley[i]<<" "<<num_hammersley[i+1]<<"\n";
    // }
    // hammersley_file.close();
    // // for(int i = 0 ; i < size_hammersley; i++)
    // delete num_hammersley;
    
}