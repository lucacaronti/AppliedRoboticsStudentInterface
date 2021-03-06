#ifndef __DIJKSTRA_HPP__
#define __DIJKSTRA_HPP__

#include <map>
#include <opencv2/opencv.hpp>

typedef std::pair<double, cv::Point2d> distNode_t;

class Dijkstra
{
public:
    std::map<cv::Point2d, std::list<distNode_t> > nodes;

    Dijkstra(); //Costructor
    ~Dijkstra();

    bool addEdge(const cv::Point2d start, const cv::Point2d end, const double weight);

    bool shortesPath(const cv::Point2d start, const cv::Point2d end, std::vector<cv::Point2d>& best_path) const;

    void print() const;
};

#endif
