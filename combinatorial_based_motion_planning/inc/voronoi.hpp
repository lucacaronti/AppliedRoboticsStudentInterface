#ifndef VORONOI
#define VORONOI

#include <iostream>
#include <cmath>
#include <tuple>
#include <vector>
#include <assert.h>
#include <cstdlib>
#include <ctime>
#include <string>

#include <boost/polygon/polygon.hpp>
#include <boost/polygon/voronoi.hpp>

#include <voronoi_visual_utils.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::x;
using boost::polygon::y;
using boost::polygon::low;
using boost::polygon::high;

typedef boost::polygon::point_data<double> point_type;
typedef boost::polygon::segment_data<double> segment_type;

typedef voronoi_diagram<double>::cell_type cell_type;
typedef voronoi_diagram<double>::cell_type::source_index_type source_index_type;
typedef voronoi_diagram<double>::cell_type::source_category_type source_category_type;

typedef voronoi_diagram<double>::edge_type edge_type;
typedef voronoi_diagram<double>::vertex_type vertex_type;
typedef voronoi_diagram<double>::const_edge_iterator const_edge_iterator;


std::vector<point_type> points;
std::vector<segment_type> segments;

// plot
Mat image(800, 800, CV_8UC3, Scalar(255, 255, 255));
Point resize(Point p);
void clip_infinite_edge(const edge_type& edge, vector<point_type>* clipped_edge);
void sample_curved_edge(const edge_type& edge, vector<point_type>* sampled_edge);
point_type retrieve_point(const cell_type& cell);
segment_type retrieve_segment(const cell_type& cell);


#endif