#include <vector>
#include <string>

#include "intersections.hpp"

double cross2D(Point2d v1, Point2d v2){
	double cprod = v1.x * v2.y - v1.y * v2.x;
	return cprod;
}

double dot2D(Point2d v1, Point2d v2){
	double dprod = v1.x * v2.x + v1.y * v2.y;
	return dprod;
}

bool Intersections::intersLineLine(vector<Point2d> line_a, vector<Point2d> line_b){

	double t;
	vector<double> ts;

	double x1 = line_a[0].x;
	double y1 = line_a[0].y;

	double x2 = line_a[1].x;
	double y2 = line_a[1].y;

	double x3 = line_b[0].x;
	double y3 = line_b[0].y;

	double x4 = line_b[1].x;
	double y4 = line_b[1].y;

	double minX1 = min(x1,x2);
	double minY1 = min(y1,y2);
	double maxX1 = max(x1,x2);
	double maxY1 = max(y1,y2);
	   
	double minX2 = min(x3,x4);
	double minY2 = min(y3,y4);
	double maxX2 = max(x3,x4);
	double maxY2 = max(y3,y4);

	if (maxX2<minX1 || minX2>maxX1 || maxY2<minY1 || minY2>maxY1) {
		// cout << "[Error] Wrong input values" << endl;
		return false;
	}

	Point2d q = Point2d(x1, y1);
	Point2d s = Point2d(x2-q.x, y2-q.y);

	Point2d p = Point2d(x3, y3);
	Point2d r = Point2d(x4-p.x, y4-p.y);

	Point2d diffPQ = Point2d(q.x-p.x, q.y-p.y);

	double crossRS = cross2D(r, s);
	double crossDiffR = cross2D(diffPQ,r);
   	double crossDiffS = cross2D(diffPQ,s);

   	if(crossRS == 0 && crossDiffR == 0){
   		double dotRR = dot2D(r,r);
       	double dotSR = dot2D(s,r);
       	double t0 = dot2D(diffPQ,r)/dotRR;
       	double t1 = t0+dotSR/dotRR;
       	if(dotSR<0){
       		if(t0>=0 && t0<=1){
       			ts = {max(t1,double(0)), min(t0,double(1))};
       		}
       	}else{
       		if(t1>=0 && t0<=1){
       			ts = {max(t0,double(0)), min(t1,double(1))};
       		}
       	}
   	}else{
   		if(crossRS == 0 && crossDiffR != 0){
   			return false;
   		}else{
   			t = crossDiffS/crossRS;
   			double u = crossDiffR/crossRS;
   			if(t>=0 && t<=1 && u>=0 && u<=1){
   				ts = {t};
   			}
   		}
   	}

   	for(int t=0; t<ts.size(); t++){
   		Point2d pt = Point2d(p.x+ts[t]*r.x, p.y+ts[t]*r.y);
   		pts.emplace_back(pt);
   	}
   	
   	return true;

}

void Intersections::plot(){
	auto axes = CvPlot::makePlotAxes();

	vector<double> data_x;
	vector<double> data_y;

	vector<Point2d> line;
	vector<Point2d> pt;

	for(int i=0; i<all_lines.size(); i++){
		line = all_lines[i];
		data_x = {};
		data_y = {};
		for(int p=0; p<line.size(); p++){
			data_x.emplace_back(line[p].x);
			data_y.emplace_back(line[p].y);
		}
		if(i==0){
			axes.create<CvPlot::Series>(data_x, data_y, "-b");	
		}
		else{
			axes.create<CvPlot::Series>(data_x, data_y, "-k");
		}
		
	}

	for(int i=0; i<pts.size(); i++){
		axes.create<CvPlot::Series>(pts[i].x, pts[i].y, "r-o");
	}

	Mat mat = axes.render(800, 800);
    imshow("Intersections", mat);

    waitKey();
}

/* -------------------------------------------- */
/*         Intersection Class methods           */
/* -------------------------------------------- */
Intersections::Intersections(){
    minX = 0;
	maxX = 0;
	minY = 0;
	maxY = 0;
}

Intersections::~Intersections(){
}

void Intersections::set_X(double _minX, double _maxX){
	minX = _minX;
	maxX = _maxX;
	xDiff = _maxX - _minX;
}

void Intersections::set_Y(double _minY, double _maxY){
	minY = _minY;
	maxY = _maxY;
	xDiff = _maxY - _minY;
}

Point2d Intersections::random_point(){
	Point2d p;
	p.x = double(rand() / (RAND_MAX + 1.) * (maxX-minX)+minX);
	p.y = double(rand() / (RAND_MAX + 1.) * (maxY-minY)+minY);
	return p;
}

void Intersections::add_line(vector<Point2d> line){
	all_lines.emplace_back(line);
}

/* -------------------------------------------- */
/*        		       main       			    */
/* -------------------------------------------- */
int main()
{
	srand((unsigned) time(0));
    Intersections inters;

    inters.set_X(-10, 10);
    inters.set_Y(-10, 10);

    vector<Point2d> line_a;
    vector<Point2d> line_b;

    int iters = 0;
    bool res;

    for(int i=0; i<1; i++){
    	line_a = {inters.random_point(), inters.random_point()};

    	inters.add_line(line_a);

    	while(iters<10){
    		line_b = {inters.random_point(), inters.random_point()};

    		res = inters.intersLineLine(line_a, line_b);

    		if(res){ 
    			inters.add_line(line_b);
    			iters+=1;
    		}
    	}
    	  
    }

    inters.plot();
}