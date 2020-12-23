#ifndef __SBMP_HPP__
#define __SBMP_HPP__

#include "sampling.hpp"
#include "dijkstra.hpp"
#include "dubins.hpp"
#include "dubins_primitives.hpp"
#include "intersections.hpp"

enum sample_type {halton_sampling, random_sampling};

class Sbmp{
private:
    /* data */
    void find_discardable_points(std::vector<cv::Point2d>& best_path, unsigned int start_index, unsigned int end_index, std::vector<std::vector<cv::Point2d> >& obstacles, std::vector<cv::Point2d>& discarded_points);
public:
    /* Attributes */
    Sampling s;
    Dijkstra d;
    std::vector<cv::Point2d> sample_points;
    unsigned int N_jobs;
    /* Methods */
    Sbmp(/* args */);
    ~Sbmp();
    void sample(unsigned int N_points, double size_x, double size_y, sample_type st=halton_sampling);
    void add_custom_point(cv::Point2d pt);
    void erase_sample_inside_obstacles(std::vector<std::vector<cv::Point2d> >& obstacles);
    void create_graph(unsigned int N_neighbours, std::vector<std::vector<cv::Point2d> >& obstacles);
    bool find_shortest_path(cv::Point2d start_point, cv::Point2d end_point, std::vector<cv::Point2d>& best_path);
    void best_path_optimizer(std::vector<cv::Point2d>& best_path, std::vector<std::vector<cv::Point2d> >& obstacles);
    void plot_points() const;
    void plot_paths(const std::vector<cv::Point2d>& best_path, const std::vector<std::vector<cv::Point2d> >& obstacles) const;

};


#endif
