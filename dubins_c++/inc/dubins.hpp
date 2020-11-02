#ifndef DUBINS
#define DUBINS

#include <iostream>
#include <cmath>
#include <tuple>
#include <vector>
#include <assert.h>
#include "primitives.hpp"



#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

// library for plots
#define CVPLOT_HEADER_ONLY
#include <CvPlot/cvplot.h>


using namespace std;
using namespace cv;

struct arc {
    double x0;
    double y0;
    double th0;
    double xf;
    double yf;
    double thf;
    double k;
    double L;
};

struct curve {
  arc a1;
  arc a2;
  arc a3;
  double L;
};

curve dubinscurve (double x0, double y0, double th0, double s1,double s2, double s3, double k0, double k1, double k2);

arc dubinsarc(double x0, double y0, double th0, double k, double L);

double sinc(double t);

double mod2pi(double ang);

double rangeSymm(double ang);

bool check(double s1, double k0, double s2, double k1, double s3, double k2, double th0, double thf);

tuple<double, double, double, double> scaleToStandard(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax);

tuple<double, double, double> scaleFromStandard(double lambda, double sc_s1, double sc_s2, double sc_s3);

pair<int, curve> shortest_path(double x0, double y0, double th0, double xf, double yf, double thf, double kmax);

tuple<double, double, double> circline(double s, double x0, double y0, double th0, double k);

tuple<vector<float>, vector<float>> plotarc(arc arc);

void plotdubins(curve curve);

#endif