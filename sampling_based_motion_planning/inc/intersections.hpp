#ifndef COLLISIONS
#define COLLISIONS

#include <iostream>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <ctime>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

bool onSegment(Point2d p, Point2d q, Point2d r) ;
int orientation(Point2d p, Point2d q, Point2d r) ;
bool doIntersect(Point2d p1, Point2d q1, Point2d p2, Point2d q2) ;

#endif