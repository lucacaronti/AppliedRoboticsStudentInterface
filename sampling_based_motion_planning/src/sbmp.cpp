#include <iostream>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/flann.hpp>
#include <chrono>
#include "sbmp.hpp"
// library for plots
#define CVPLOT_HEADER_ONLY
#include <CvPlot/cvplot.h>

#define MAIN_DEFINED

/*!
 * @brief Polt class sampled points
!*/
void Sbmp::plot_points() const
{
    auto axes = CvPlot::makePlotAxes();
    for (auto itvp = sample_points.begin(); itvp != sample_points.end(); itvp++)
    {
        std::vector<cv::Point2d> tmp_point;
        tmp_point.push_back(*itvp);
        axes.create<CvPlot::Series>(tmp_point, "k-o");
    }
    cv::Mat mat = axes.render(800, 800);
    cv::imshow("mywindow", mat);
    cv::waitKey();
    // cv::destroyWindow("mywindow");
}

/*!
 * @brief Polt the graph, the best path and the obstacles
!*/
void Sbmp::plot_paths(const std::vector<cv::Point2d>& best_path, const std::vector<std::vector<cv::Point2d> >& obstacles) const{
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
    
    for (auto it = obstacles.begin(); it != obstacles.end(); it++){
        axes.create<CvPlot::Series>(*it, "b-o");

        std::vector<cv::Point2f> tmp_line;
        tmp_line.emplace_back(it->at(0));
        tmp_line.emplace_back(it->at(it->size()-1));
        axes.create<CvPlot::Series>(tmp_line, "b-o");

    }

    axes.create<CvPlot::Series>(best_path, "k-");

    cv::Mat mat = axes.render(1000, 1000);
    cv::imshow("mywindow", mat);
    cv::waitKey();
    // cv::destroyWindow("mywindow");
}
/*!
 * @brief Check if a segment intersect with obstacles
 * @param[in] const std::vector<cv::Point2d> segment    vector of 2 points
 * @param[in] const std::vector<std::vector<cv::Point2d> >& obstacles   obstacles
!*/
bool doIntersectWithObstacles(const std::vector<cv::Point2d> segment, const std::vector<std::vector<cv::Point2d> >& obstacles){
    Intersections intersections;
    for(auto itvvp = obstacles.begin(); itvvp != obstacles.end(); itvvp++){ //Iterate for each obstacle
        for(auto itvp = itvvp->begin(); itvp != itvvp->end(); itvp++){ //Iterate for each face of polygon
            cv::Point2d point_a(itvp->x, itvp->y);
            cv::Point2d point_b;
            if(itvp+1 == itvvp->end()){
                point_b.x = itvvp->at(0).x;
                point_b.y = itvvp->at(0).y;
            }else{
                point_b.x = (itvp+1)->x;
                point_b.y = (itvp+1)->y;
            }
            std::vector<cv::Point2d> obstacle_face = {point_a, point_b}; // Create polygon face line 
            // if(intersections.intersLineLine(segment, obstacle_face)){ // Check if intersect
            if(doIntersect(segment[0], segment[1], obstacle_face[0], obstacle_face[1])){
                return true;
            }
        }
    }
    return false;
}

// Calculate the euclidian distance from 2 2d points //
double dist2d(cv::Point2d p1, cv::Point2d p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

/*** Sbpm class functions ***/
/****************************/
Sbmp::Sbmp(/* args */)
{
    N_jobs = 1;
}

Sbmp::~Sbmp()
{
}

/*!
 * @brief Start sampling
 * @param[in] const unsigned int N_points     Number of points to sample
 * @param[in] const double size_x=1           X width of sampled map
 * @param[in] const double size_y=1           Y height of sampled map
 * @param[in] const sample_type st            sample type
!*/
void Sbmp::sample(const unsigned int N_points,const double size_x=1,const double size_y=1,const sample_type st){
    if(st == halton_sampling){
        sample_points = s.generate_N_Halton_points_multithread(N_points,this->N_jobs); //Generate N_points in 2D space normalized between (0,0) and (1,1)
        if(size_x != 1 || size_y != 1){ //Check is a resize is needed
            for(auto it = sample_points.begin(); it != sample_points.end(); it++){
                it->x *= size_x;
                it->y *= size_y;
            }
        }
    }
    else if(st == random_sampling){
        sample_points = s.generate_N_rnd_points(N_points, size_x, size_y); //Generate N_points in 2D space normalized between (0,0) and (size_x,size_y)
    }
}

/*!
 * @brief Erase sampled points inside obstacles
 * @param[in] const std::vector<std::vector<cv::Point2d> >& obstacles Obstacles
!*/
void Sbmp::erase_sample_inside_obstacles(const std::vector<std::vector<cv::Point2d> >& obstacles){
    std::vector<cv::Point2d> sample_points_ok;
    for(auto itvp=sample_points.begin(); itvp != sample_points.end(); itvp++){ // Iterate all sample points
        bool is_ok = true;
        for(auto itvvp = obstacles.begin(); itvvp != obstacles.end(); itvvp++){ // Iterate all obstacles
            std::vector<cv::Point2f> tmp_obstacle_f;
            for(auto it_ob = itvvp->begin(); it_ob != itvvp->end(); it_ob++){
                tmp_obstacle_f.emplace_back(float(it_ob->x), float(it_ob->y));
            }
            if(cv::pointPolygonTest(tmp_obstacle_f, cv::Point2f(float(itvp->x), float(itvp->y)), false) != -1){ // Check is point is inside the polygon
                is_ok = false;
                break; // Stop che for loop
            }
        }
        if(is_ok)
            sample_points_ok.push_back(*itvp);
    }
    this->sample_points.clear();
    this->sample_points = sample_points_ok;
}

/*!
 * @brief Create a graph of points
 * @param[in] const unsigned int N_neighbours     Number of points neighbours 
 * @param[in] const std::vector<std::vector<cv::Point2d> >& obstacles Obstacles
!*/
void Sbmp::create_graph(const unsigned int N_neighbours, const std::vector<std::vector<cv::Point2d> >& obstacles){

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

    flann_index.knnSearch(features, indices, dists, N_neighbours + 1);
    /********** End kd-tree construction and search **********/

    /********** Create a graph **********/
    for (int i = 0; i < indices.rows; i++) // Iterate each point
    {
        for (unsigned int j = 1; j < N_neighbours + 1; j++) // Find N_neighbours
        {
            int index = indices.at<int>(i, j); // take index of neighbour point
            double dist = dist2d(sample_points[i], sample_points[index]); // Calculate the distance between points
            bool collision = false;
            std::vector<cv::Point2d> segment = {sample_points[i], sample_points[index]}; // Create the graph line
            collision = doIntersectWithObstacles(segment, obstacles);
            if(!collision)
                d.addEdge(sample_points[i], sample_points[index], dist); // Add connection to the graph
        }
    }
    /********** End graph creation **********/
}

/*!
 * @brief Find shortest path with Dijkstra
 * @param[in]  cv::Point2d start   start point 
 * @param[in]  cv::Point2d end     end point
 * @param[in/out]  std::vector<cv::Point2d>& best_path resulting best path
 * @return[bool] false if no path is found, true otherwise
!*/
bool Sbmp::find_shortest_path(const cv::Point2d start_point,const cv::Point2d end_point, std::vector<cv::Point2d>& best_path)const{
    return d.shortesPath(start_point, end_point, best_path);
}
bool Sbmp::find_shortest_path_and_optimized(const std::vector<cv::Point2d> points,const std::vector<std::vector<cv::Point2d> >& obstacle_list, std::vector<cv::Point2d>& best_path)const{
    for(auto it_p = points.begin(); it_p != points.end() - 1; it_p++){
        std::vector<cv::Point2d> tmp_path;
        if(!this->find_shortest_path(*it_p, *(it_p+1), tmp_path)){
            return false;
        }
        // this->plot_paths(tmp_path, obstacle_list);
        this->best_path_optimizer(tmp_path, obstacle_list);
        // this->plot_paths(tmp_path, obstacle_list);
        if(best_path.size() != 0 && (best_path[best_path.size()-1] == tmp_path[0])){
          best_path.insert(best_path.end(), tmp_path.begin()+1, tmp_path.end());
        }else{
          best_path.insert(best_path.end(), tmp_path.begin(), tmp_path.end());
        }
    }
    return true;
}


/*!
 * @brief Add a custom point to sampled points
 * @param[in]  cv::Point2d pt   cusotm point 
!*/
void Sbmp::add_custom_point(cv::Point2d pt){
    sample_points.emplace_back(pt);
}

/*!
 * @brief Find best path discardable points
 * @param[in]  const std::vector<cv::Point2d>& best_path    best path
 * @param[in]  const unsigned int start_index               start index of best_path
 * @param[in]  const unsigned int end_index                 end index of best_path
 * @param[in]  const std::vector<std::vector<cv::Point2d> >& obstacles  obstacles
 * @param[out] std::vector<cv::Point2d>& discarded_points   discardable points
!*/
void Sbmp::find_discardable_points(const std::vector<cv::Point2d>& best_path,const unsigned int start_index, const unsigned int end_index, const std::vector<std::vector<cv::Point2d> >& obstacles, std::vector<cv::Point2d>& discarded_points)const{
    if(end_index - start_index < 2){
        return;
    }
    std::vector<cv::Point2d> tmp_segment;
    tmp_segment.push_back(cv::Point2d(best_path[start_index]));
    tmp_segment.push_back(cv::Point2d(best_path[end_index]));
    if(doIntersectWithObstacles(tmp_segment, obstacles)){
        find_discardable_points(best_path, start_index + int((end_index-start_index)/2), end_index, obstacles, discarded_points);
        find_discardable_points(best_path, start_index, end_index - (int((end_index-start_index)/2) + int((end_index-start_index)%2)), obstacles, discarded_points);
    }else{
        for(unsigned int i = start_index+1; i < end_index; i++){
            discarded_points.emplace_back(cv::Point2d(best_path[i]));
        }
        return;
    }
}

/*!
 * @brief Optimize best path
 * @param[in]  std::vector<cv::Point2d>& best_path    best path
 * @param[in]  const std::vector<std::vector<cv::Point2d> >& obstacles  obstacles
!*/
void Sbmp::best_path_optimizer(std::vector<cv::Point2d>& best_path,const std::vector<std::vector<cv::Point2d> >& obstacles)const{
    std::vector<cv::Point2d> discarded_points;
    do{
        discarded_points.clear();
        this->find_discardable_points(best_path,0, best_path.size()-1, obstacles, discarded_points);
        for(auto it_wp = discarded_points.begin(); it_wp != discarded_points.end(); it_wp++){
            for(auto it_bp = best_path.begin(); it_bp != best_path.end(); it_bp++){
                if(*it_wp == *it_bp){
                    best_path.erase(it_bp);
                    break;
                }
            }
        }
    } while (!discarded_points.empty());
    
}

#ifdef MAIN_DEFINED

/************************************/
/*************** MAIN ***************/
/************************************/
int main(int argc, char *argv[]){

    std::vector<cv::Point2d> borders;
    borders.emplace_back(cv::Point2d(0,0));
    borders.emplace_back(cv::Point2d(1.56,0));
    borders.emplace_back(cv::Point2d(1.56,1.06));
    borders.emplace_back(cv::Point2d(0,1.06));

    std::vector<std::vector<cv::Point2d> > obstacles;
    std::vector<cv::Point2d> tmp_obstacle;
    // tmp_obstacle.emplace_back(cv::Point2d());

    tmp_obstacle.emplace_back(cv::Point2d(0.90675,0.6435));
    tmp_obstacle.emplace_back(cv::Point2d(0.88725,0.69615));
    tmp_obstacle.emplace_back(cv::Point2d(0.9399,0.72735));
    tmp_obstacle.emplace_back(cv::Point2d(0.9789,0.69615));
    tmp_obstacle.emplace_back(cv::Point2d(0.96135,0.64545));
    obstacles.emplace_back(tmp_obstacle);
    tmp_obstacle.clear();
	
	tmp_obstacle.emplace_back(cv::Point2d(0.69615,0.64155));
    tmp_obstacle.emplace_back(cv::Point2d(0.76245,0.77805));
    tmp_obstacle.emplace_back(cv::Point2d(0.7761,0.77805));
    tmp_obstacle.emplace_back(cv::Point2d(0.85215,0.64545));
    obstacles.emplace_back(tmp_obstacle);
    tmp_obstacle.clear();

    tmp_obstacle.emplace_back(cv::Point2d(0.4953,0.59085));
    tmp_obstacle.emplace_back(cv::Point2d(0.5577,0.69615));
    tmp_obstacle.emplace_back(cv::Point2d(0.61425,0.5967));
    obstacles.emplace_back(tmp_obstacle);
    tmp_obstacle.clear();

    tmp_obstacle.emplace_back(cv::Point2d(0.31785,0.59085));
    tmp_obstacle.emplace_back(cv::Point2d(0.2886,0.6747));
    tmp_obstacle.emplace_back(cv::Point2d(0.351,0.7176));
    tmp_obstacle.emplace_back(cv::Point2d(0.4251,0.67665));
    tmp_obstacle.emplace_back(cv::Point2d(0.4017,0.59085));
    obstacles.emplace_back(tmp_obstacle);
    tmp_obstacle.clear();

    tmp_obstacle.emplace_back(cv::Point2d(0.16185,0.59085));
    tmp_obstacle.emplace_back(cv::Point2d(0.2028,0.66105));
    tmp_obstacle.emplace_back(cv::Point2d(0.23985,0.59085));
    obstacles.emplace_back(tmp_obstacle);
    tmp_obstacle.clear();

    tmp_obstacle.emplace_back(cv::Point2d(0.039,0.58695));
    tmp_obstacle.emplace_back(cv::Point2d(0.02535,0.64545));
    tmp_obstacle.emplace_back(cv::Point2d(0.06045,0.67275));
    tmp_obstacle.emplace_back(cv::Point2d(0.08385,0.67275));
    tmp_obstacle.emplace_back(cv::Point2d(0.1209,0.6435));
    tmp_obstacle.emplace_back(cv::Point2d(0.1014,0.5889));
    obstacles.emplace_back(tmp_obstacle);
    tmp_obstacle.clear();

    tmp_obstacle.emplace_back(cv::Point2d(0.93405,0.4446));
    tmp_obstacle.emplace_back(cv::Point2d(0.8892,0.52455));
    tmp_obstacle.emplace_back(cv::Point2d(0.93795,0.61425));
    tmp_obstacle.emplace_back(cv::Point2d(0.83155,0.6123));
    tmp_obstacle.emplace_back(cv::Point2d(0.88225,0.5304));
    tmp_obstacle.emplace_back(cv::Point2d(0.83545,0.44265));
    obstacles.emplace_back(tmp_obstacle);
    tmp_obstacle.clear();

    tmp_obstacle.emplace_back(cv::Point2d(0.6942,0.4446));
    tmp_obstacle.emplace_back(cv::Point2d(0.6942,0.5772));
    tmp_obstacle.emplace_back(cv::Point2d(0.82485,0.5811));
    tmp_obstacle.emplace_back(cv::Point2d(0.82095,0.44265));
    obstacles.emplace_back(tmp_obstacle);
    tmp_obstacle.clear();

    tmp_obstacle.emplace_back(cv::Point2d(0.49335,0.44265));
    tmp_obstacle.emplace_back(cv::Point2d(0.48945,0.54405));
    tmp_obstacle.emplace_back(cv::Point2d(0.59085,0.54405));
    tmp_obstacle.emplace_back(cv::Point2d(0.5928,0.4446));
    obstacles.emplace_back(tmp_obstacle);
    tmp_obstacle.clear();

    tmp_obstacle.emplace_back(cv::Point2d(0.3198,0.44265));
    tmp_obstacle.emplace_back(cv::Point2d(0.28665,0.4992));
    tmp_obstacle.emplace_back(cv::Point2d(0.32955,0.56745));
    tmp_obstacle.emplace_back(cv::Point2d(0.39975,0.56745));
    tmp_obstacle.emplace_back(cv::Point2d(0.43485,0.5109));
    tmp_obstacle.emplace_back(cv::Point2d(0.4056,0.45045));
    obstacles.emplace_back(tmp_obstacle);
    tmp_obstacle.clear();

    tmp_obstacle.emplace_back(cv::Point2d(0.05265,0.4368));
    tmp_obstacle.emplace_back(cv::Point2d(0.02535,0.47775));
    tmp_obstacle.emplace_back(cv::Point2d(0.04875,0.5226));
    tmp_obstacle.emplace_back(cv::Point2d(0.10335,0.5187));
    tmp_obstacle.emplace_back(cv::Point2d(0.12285,0.4836));
    tmp_obstacle.emplace_back(cv::Point2d(0.10335,0.44265));
    obstacles.emplace_back(tmp_obstacle);
    tmp_obstacle.clear();

    tmp_obstacle.emplace_back(cv::Point2d(0.15795,0.43485));
    tmp_obstacle.emplace_back(cv::Point2d(0.1599,0.5031));
    tmp_obstacle.emplace_back(cv::Point2d(0.22035,0.50895));
    tmp_obstacle.emplace_back(cv::Point2d(0.2301,0.43875));
    obstacles.emplace_back(tmp_obstacle);
    tmp_obstacle.clear();



    if (argc != 5)
    {
        std::cout << "Error, usage: ./sbmp.out n_sample n_neighbours sampling_type N_jobs" << std::endl;
        std::cout << "Example ./sbmp 1000 20 halton 1"<<std::endl;
        return -1;
    }
    unsigned int N_points = atoi(argv[1]);
    unsigned int max_neighbours = atoi(argv[2]);
    std::string sampling_type = std::string(argv[3]);

    Sbmp sbmp;
    sbmp.N_jobs = atoi(argv[4]);

    /********** Sampling **********/
    auto time_sampling_start = std::chrono::high_resolution_clock::now();
    // sbmp.sample(N_points, borders[1].x, borders[2].y);
    // sbmp.sample(N_points, 1, 1);
    sbmp.sample(N_points, 1.0, 1.0, random_sampling);
    // sbmp.plot_points();
    sbmp.add_custom_point(cv::Point2d(0,0));
    sbmp.add_custom_point(cv::Point2d(1,1));
    // obstacles.clear();
    sbmp.erase_sample_inside_obstacles(obstacles);
    auto time_sampling_end = std::chrono::high_resolution_clock::now();
    /********** End sampling **********/

    /********** Start graph construction **********/
    auto time_construct_graph_start = std::chrono::high_resolution_clock::now();
    sbmp.create_graph(max_neighbours, obstacles);
    auto time_construct_graph_end = std::chrono::high_resolution_clock::now();
    /********** End graph construction **********/


    std::vector<cv::Point2d> best_path;
    /********** Find best path with Dijkstra **********/
    auto time_dijkstra_start = std::chrono::high_resolution_clock::now();
    if(!sbmp.find_shortest_path(cv::Point2d(0,0), cv::Point2d(1,1), best_path))
        std::cout<<"Path not found, incraese the number of neighbours"<<std::endl;
    auto time_dijkstra_end = std::chrono::high_resolution_clock::now();
    /********** End Dijkstra **********/

    /********** Print time stats **********/
    auto duration_sampling = std::chrono::duration_cast<std::chrono::microseconds>(time_sampling_end - time_sampling_start);
    std::cout<<"Duration sampling: "<<duration_sampling.count()<<" us"<<std::endl;

    auto duration_construction_graph = std::chrono::duration_cast<std::chrono::microseconds>(time_construct_graph_end - time_construct_graph_start);
    std::cout << "Duration construction graph: " << duration_construction_graph.count() << " us" << std::endl;

    auto duration_dijkstra = std::chrono::duration_cast<std::chrono::microseconds>(time_dijkstra_end - time_dijkstra_start);
    std::cout << "Duration Dijkstra: " << duration_dijkstra.count() << " us" << std::endl;

    // Plot path
    sbmp.plot_paths(best_path, obstacles);
    // sbmp.plot_points();
    return 0;

    
    sbmp.best_path_optimizer(best_path, obstacles);
    sbmp.plot_paths(best_path, obstacles);

    /********** Dubins **********/
    DubinsCurve dc;
    dc.set_k(50);
    dc.add_start_data(best_path[0].x, best_path[0].y, 0);
    dc.add_final_data(best_path[best_path.size()-1].x, best_path[best_path.size()-1].y, 0);
    for(auto it = best_path.begin() + 1; it != best_path.end() - 1; it++){
        dc.add_middle_points(it->x, it->y);
    }
    dc.solver(3,16);
    dc.plot();

    best_path.clear();
    /********** Find best path with Dijkstra **********/
    time_dijkstra_start = std::chrono::high_resolution_clock::now();
    if(!sbmp.find_shortest_path(sbmp.sample_points[20], sbmp.sample_points[1], best_path))
        std::cout<<"Path not found, incraese the number of neighbours"<<std::endl;
    time_dijkstra_end = std::chrono::high_resolution_clock::now();
    /********** End Dijkstra **********/
    duration_dijkstra = std::chrono::duration_cast<std::chrono::microseconds>(time_dijkstra_end - time_dijkstra_start);
    std::cout << "Duration Dijkstra: " << duration_dijkstra.count() << " us" << std::endl;

    // Plot path
    sbmp.plot_paths(best_path, obstacles);

    return 0;
}

#endif