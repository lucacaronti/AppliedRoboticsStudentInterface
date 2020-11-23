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
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

double cross2D(Point2d v1, Point2d v2);
double dot2D(Point2d v1, Point2d v2);


class Intersections{
private:
  double minX, maxX, minY, maxY;
  double xDiff, yDiff;
  vector<Point2d> pts;
  double img_size;
  
public:
  Intersections();
  ~Intersections();
  void solve(int mode);

  void set_X(double _minX, double _maxX);
  void set_Y(double _minY, double _maxY);
  Point2d random_Point2d();
  int random_radius();

  bool intersLineLine(vector<Point2d> line_a, vector<Point2d> line_b);

  bool intersCircleLine(Point2d center, int radius, vector<Point2d> line);

  Mat img;
  void set_plot(double size);
  Point2d resize(Point2d p);
  int resize_radius(int r);
  void plot();
};

#endif