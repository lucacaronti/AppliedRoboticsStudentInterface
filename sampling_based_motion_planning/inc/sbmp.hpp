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
    /* Private methods */
    void find_discardable_points(const std::vector<cv::Point2d>& best_path, const unsigned int start_index, const unsigned int end_index, const std::vector<std::vector<cv::Point2d> >& obstacles, std::vector<cv::Point2d>& discarded_points)const;
public:
    /* Attributes */
    Sampling s;
    Dijkstra d;
    std::vector<cv::Point2d> sample_points;
    unsigned int N_jobs;
    /* Methods */
    Sbmp();
    ~Sbmp();
    void sample(const unsigned int N_points, const double size_x, const double size_y, const sample_type st=halton_sampling);
    void add_custom_point(cv::Point2d pt);
    void erase_sample_inside_obstacles(const std::vector<std::vector<cv::Point2d> >& obstacles);
    void create_graph(const unsigned int N_neighbours, const std::vector<std::vector<cv::Point2d> >& obstacles);
    bool find_shortest_path(const cv::Point2d start_point, const cv::Point2d end_point, std::vector<cv::Point2d>& best_path)const;
    void best_path_optimizer(std::vector<cv::Point2d>& best_path, const std::vector<std::vector<cv::Point2d> >& obstacles)const;
    void plot_points() const;
    void plot_paths(const std::vector<cv::Point2d>& best_path, const std::vector<std::vector<cv::Point2d> >& obstacles) const;

};


#endif
