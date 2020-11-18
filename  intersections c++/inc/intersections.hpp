#ifndef COLLISIONS
#define COLLISIONS

#include <iostream>
#include <cmath>
#include <tuple>
#include <vector>
#include <assert.h>
#include <cstdlib>
#include <ctime>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

// library for plots
#define CVPLOT_HEADER_ONLY
#include <CvPlot/cvplot.h>


using namespace std;
using namespace cv;

double cross2D(Point2d v1, Point2d v2);
double dot2D(Point2d v1, Point2d v2);


class Intersections{
private:
    double minX, maxX, minY, maxY;
    double xDiff, yDiff;
    vector<Point2d> pts;
    vector<vector<Point2d>> all_lines;

public:
  Intersections();
  ~Intersections();
  void set_X(double _minX, double _maxX);
  void set_Y(double _minY, double _maxY);
  Point2d random_point();
  void add_line(vector<Point2d> line);
  bool intersLineLine(vector<Point2d> line_a, vector<Point2d> line_b);
  void plot();
};

#endif