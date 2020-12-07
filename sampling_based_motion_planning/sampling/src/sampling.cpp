#include <iostream>
#include <ctime>
#include <map>
#include <math.h>
#include "sampling.hpp"
#include "dijkstra.hpp"
#include <opencv2/flann.hpp>

// #include "hammersley.hpp"
#include "halton.hpp"
#include <fstream>
// library for plots
#define CVPLOT_HEADER_ONLY
#include <CvPlot/cvplot.h>

void plot(const std::vector<cv::Point2d> points){
    auto axes = CvPlot::makePlotAxes();
    for(auto itvp = points.begin(); itvp != points.end(); itvp++ ){
        std::vector<cv::Point2d> tmp_point;
        tmp_point.push_back(*itvp);
        axes.create<CvPlot::Series>(tmp_point, "k-o");
    }
    cv::Mat mat = axes.render(800, 800);
    cv::imshow("mywindow", mat);
    cv::waitKey();
    cv::destroyWindow("mywindow");
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

double dist2d(cv::Point2d p1, cv::Point2d p2){
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y,2));
}


int main(int argc, char* argv[]){
    
    if(argc != 3){
        std::cout << "Error, usage: ./sampling.out n_sample n_neighbours"<<std::endl;
        return -1;
    }
    int size_halton = atoi(argv[1]);
    unsigned int max_neighbours = atoi(argv[2]);

    Sampling s;
    std::vector<cv::Point2d> points;
    // points = s.generate_N_rnd_points(1000,100,100);

    ofstream halton_file;
    halton_file.open("halton.txt");


    int base[2];
    base[0] = 2;
    base[0] = 2;
    double *num_halton[size_halton];
    for(int i = 0; i < size_halton; i++){
        // num_halton[i] = halton_base(i+1, 2, base);
        num_halton[i] = halton(i+1, 2);
        halton_file << num_halton[i][0]<<" "<<num_halton[i][1]<<"\n";
        points.push_back(cv::Point2d(num_halton[i][0], num_halton[i][1]));
    }

    halton_file.close();
    for(int i = 0 ; i < size_halton; i++)
        delete num_halton[i];

    plot(points);


    cv::Mat_<float> features(0,2);

    for(auto && point : points) {
        //Fill matrix
        cv::Mat row = (cv::Mat_<float>(1, 2) << static_cast<float>(point.x), static_cast<float>(point.y));
        features.push_back(row);
    }
    // std::cout << features << std::endl;

    cv::flann::Index flann_index(features, cv::flann::KDTreeIndexParams(4), cvflann::EUCLIDEAN);


    
    cv::Mat indices, dists; //neither assume type nor size here !

    flann_index.knnSearch(features, indices, dists, max_neighbours);

    cerr << indices.type() << endl << indices << endl;
    cerr << dists.type() << endl << dists << endl;
    Dijkstra d;
    

    auto axes = CvPlot::makePlotAxes();
    for(int i = 0 ; i < indices.rows; i++){
        std::vector<std::vector<cv::Point2d> > tmp_vect_points;
        // tmp_points.push_back(points[i]);
        for(int j = 1 ; j < max_neighbours; j++){
            int index = indices.at<int>(i,j);
            // double dist = dists.at<float>(i,j);
            double dist = dist2d(points[i], points[index]);
            bool ret = d.addEdge(points[i], points[index], dist);
            // if(ret)
            std::vector<cv::Point2d> tmp_points;
            tmp_points.emplace_back(points[i]);
            tmp_points.emplace_back(points[index]);
            tmp_vect_points.emplace_back(tmp_points);
        }
        for(auto it = tmp_vect_points.begin(); it != tmp_vect_points.end(); it++)
            axes.create<CvPlot::Series>(*it, "k-o");
    }
    cv::Mat mat = axes.render(1000, 1000);
    cv::imshow("mywindow", mat);
    cv::waitKey();
    cv::destroyWindow("mywindow");

    d.print();
    std::vector<cv::Point2d> best_path;
    std::cout << "Start:(" << points[0].x << "," << points[0].y << ")" << std::endl;
    std::cout << "End:(" << points[size_halton - 1].x << "," << points[size_halton - 1].y << ")" << std::endl;
    d.shortesPath(points[0], points[size_halton - 1], best_path);
    axes.create<CvPlot::Series>(best_path, "b-o");
    for(auto it = best_path.begin(); it != best_path.end(); it++){
        std::cout << "node(" << it->x << "," << it->y << ")" << std::endl;
    }
    mat = axes.render(1000, 1000);
    cv::imshow("mywindow", mat);
    cv::waitKey();
    cv::destroyWindow("mywindow");
    // for(auto itp = best_path.begin(); itp != best_path.end(); itp++){

    //     axes.create<CvPlot::Series>(*itp, "b-o");
    // }




    // struct KDTreeIndexParams : public cv::flann::IndexParams{
    //     KDTreeIndexParams( int trees = 4 );
    // };
    // cv::flann::Index kdTree(cv::Mat(points).reshape(1), indexParams);
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
    return 0;
}