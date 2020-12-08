#include <iostream>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/flann.hpp>
#include "sampling.hpp"
#include "dijkstra.hpp"
#include <chrono>
// library for plots
#define CVPLOT_HEADER_ONLY
#include <CvPlot/cvplot.h>

// Plot vector of points //
void plot_points(const std::vector<cv::Point2d> points)
{
    auto axes = CvPlot::makePlotAxes();
    for (auto itvp = points.begin(); itvp != points.end(); itvp++)
    {
        std::vector<cv::Point2d> tmp_point;
        tmp_point.push_back(*itvp);
        axes.create<CvPlot::Series>(tmp_point, "k-o");
    }
    cv::Mat mat = axes.render(800, 800);
    cv::imshow("mywindow", mat);
    cv::waitKey();
    cv::destroyWindow("mywindow");
}

// Plot a graph end the best path //
void plot_paths(const Dijkstra& d, const std::vector<cv::Point2d>& best_path){
    auto axes = CvPlot::makePlotAxes();

    std::vector<std::vector<cv::Point2d> > connections;

    for(auto itd = d.nodes.begin(); itd != d.nodes.end(); itd++){
        for(auto itp = itd->second.begin(); itp != itd->second.end(); itp++){
            std::vector<cv::Point2d> tmp_connection;
            tmp_connection.emplace_back(itd->first);
            tmp_connection.emplace_back(itp->second);
            connections.emplace_back(tmp_connection);
        }
    }
    for (auto it = connections.begin(); it != connections.end(); it++)
        axes.create<CvPlot::Series>(*it, "g-");

    axes.create<CvPlot::Series>(best_path, "k-");

    cv::Mat mat = axes.render(1000, 1000);
    cv::imshow("mywindow", mat);
    cv::waitKey();
    cv::destroyWindow("mywindow");
}

// Calculate the euclidian distance from 2 2d points //
double dist2d(cv::Point2d p1, cv::Point2d p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

int main(int argc, char *argv[]){

    if (argc != 6)
    {
        std::cout << "Error, usage: ./sbmp.out n_sample n_neighbours sampling_type start end" << std::endl;
        std::cout << "Example ./sbmp 1000 20 halton 0 999"<<std::endl;
        return -1;
    }
    unsigned int N_points = atoi(argv[1]);
    unsigned int max_neighbours = atoi(argv[2]);
    std::string sampling_type = std::string(argv[3]);
    unsigned int start = atoi(argv[4]);
    unsigned int end = atoi(argv[5]);

    auto time_start = std::chrono::high_resolution_clock::now();

    /********** Sampling **********/
    Sampling s;
    std::vector<cv::Point2d> sample_points;

    if(sampling_type == "halton"){
        sample_points = s.generate_N_Halton_points(N_points);
        sample_points[start] = cv::Point2d(0,0);
        sample_points[end] = cv::Point2d(1,1);
    }
    else if(sampling_type == "random"){
        sample_points = s.generate_N_rnd_points(N_points, 1000, 1000);
        sample_points[start] = cv::Point2d(0, 0);
        sample_points[end] = cv::Point2d(1000, 1000);
    }
    else{
        std::cout<<"Error, "<<sampling_type<<" arg not found"<<std::endl;
        return -1;
    }
    /********** End sampling **********/

    auto time_sampling = std::chrono::high_resolution_clock::now();

    /********** Start kd-tree construction and search **********/
    cv::Mat_<float> features(0, 2);

    for (auto &&point : sample_points)
    {
        //Fill matrix
        cv::Mat row = (cv::Mat_<float>(1, 2) << static_cast<float>(point.x), static_cast<float>(point.y));
        features.push_back(row);
    }

    cv::flann::Index flann_index(features, cv::flann::KDTreeIndexParams(1), cvflann::EUCLIDEAN);

    cv::Mat indices, dists; //neither assume type nor size here !

    flann_index.knnSearch(features, indices, dists, max_neighbours);

    auto time_knn = std::chrono::high_resolution_clock::now();

    /********** End kd-tree construction and search **********/

    /********** Create a graph for Dijkstra **********/
    Dijkstra d;

    auto time_construct_graph_start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < indices.rows; i++)
    {
        bool found_connection = false;
        for (int j = 1; j < max_neighbours; j++)
        {
            int index = indices.at<int>(i, j);
            double dist = dist2d(sample_points[i], sample_points[index]);
            bool ret = d.addEdge(sample_points[i], sample_points[index], dist);
            if(ret) found_connection = true;
        }
    }
    /********** End graph creation **********/

    auto time_construct_graph_ens = std::chrono::high_resolution_clock::now();

    auto time_dijkstra_start = std::chrono::high_resolution_clock::now();
    std::vector<cv::Point2d> best_path;

    /********** Find best path with Dijkstra **********/
    if(!d.shortesPath(sample_points[start], sample_points[end], best_path)){
        std::cout<<"Path not found, incraese the number of neighbours"<<std::endl;
    }
    /********** End Dijkstra **********/

    auto time_dijkstra_end = std::chrono::high_resolution_clock::now();

    /********** Print time stats **********/
    auto duration_sampling = std::chrono::duration_cast<std::chrono::microseconds>(time_sampling - time_start);
    std::cout<<"Duration sampling: "<<duration_sampling.count()<<" us"<<std::endl;

    auto duration_knn = std::chrono::duration_cast<std::chrono::microseconds>(time_knn - time_sampling);
    std::cout << "Duration knn: " << duration_knn.count() << " us" << std::endl;

    auto duration_construction_graph = std::chrono::duration_cast<std::chrono::microseconds>(time_construct_graph_ens - time_construct_graph_start);
    std::cout << "Duration construction graph: " << duration_construction_graph.count() << " us" << std::endl;

    auto duration_dijkstra = std::chrono::duration_cast<std::chrono::microseconds>(time_dijkstra_end - time_dijkstra_start);
    std::cout << "Duration Dijkstra: " << duration_dijkstra.count() << " us" << std::endl;


    // Plot path
    plot_paths(d, best_path);

    return 0;
}