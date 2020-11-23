#include <vector>
#include <string>

#include "intersections.hpp"

/* ============================================ */
/*         Intersection Class methods           */
/* ============================================ */
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
	yDiff = _maxY - _minY;
}

Point2d Intersections::random_Point2d(){
	Point2d p;
	p.x = double(rand() / (RAND_MAX + 1.) * (maxX-minX)+minX);
	p.y = double(rand() / (RAND_MAX + 1.) * (maxY-minY)+minY);
	return p;
}

int Intersections::random_radius(){
	int radius = int(rand() / (RAND_MAX + 1.)*hypot(xDiff,yDiff)/4);
	if(radius>0){return(radius);} 
	return(1);
}

/* -------------------------------------------- */
/*                 Line-Line                    */
/* -------------------------------------------- */
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
   		drawMarker(img, resize(pt), Scalar(0,0,255),MARKER_TILTED_CROSS ,20,2,LINE_AA);
   	}
   	
   	return true;

}

/* -------------------------------------------- */
/*                 Circle-Line                  */
/* -------------------------------------------- */
bool Intersections::intersCircleLine(Point2d center, int r, vector<Point2d> line){

	vector<double> ts;

	double a = center.x;
	double b = center.y;

	double x1 = line[0].x;
	double y1 = line[0].y;

	double x2 = line[1].x;
	double y2 = line[1].y;

	double p1 = 2*x1*x2;
    double p2 = 2*y1*y2;
    double p3 = 2*a*x1;
    double p4 = 2*a*x2;
    double p5 = 2*b*y1;
    double p6 = 2*b*y2;

    double c1 = pow(x1,2)+pow(x2,2)-p1+pow(y1,2)+pow(y2,2)-p2;
    double c2 = -2*pow(x2,2)+p1-p3+p4-2*pow(y2,2)+p2-p5+p6;
    double c3 = pow(x2,2)-p4+pow(a,2)+pow(y2,2)-p6+pow(b,2)-pow(r,2);

    double delta = pow(c2,2)-4*c1*c3;
    double t1, t2, x, y;

    if(delta<0){
    	return(false);
    }
    else{
    	if(delta>0){
    		double deltaSq = sqrt(delta);
    		t1 = (-c2+deltaSq)/(2*c1);
        	t2 = (-c2-deltaSq)/(2*c1);
    	}
    	else{
    		t1 = -c2/(2*c1);
        	t2 = t1;
    	}
    }

    if(t1>=0 && t1<=1){
    	x = x1*t1+x2*(1-t1);
    	y = y1*t1+y2*(1-t1);
    	pts.emplace_back(Point2d(x,y));
    	ts.emplace_back(t1);
    }

    if(t2 >=0 && t2<=1 && t2!=t1){
	    x = x1*t2+x2*(1-t2);
	    y = y1*t2+y2*(1-t2);
	    pts.emplace_back(Point2d(x,y));
    	ts.emplace_back(t2);
	}

	Point2d pt;
	for(int k=0; k<pts.size(); k++){
		pt = pts[k];
		drawMarker(img, resize(pt), Scalar(0,0,255),MARKER_TILTED_CROSS,20 ,2 ,LINE_AA);
	}
    return(true);

}

/* -------------------------------------------- */
/*               For opencv plot                */
/* -------------------------------------------- */

void Intersections::set_plot(double size){
	img_size = size;
	img = Mat::zeros(Size(img_size, img_size), CV_8UC3);
	img = Scalar( 255,  255,  255);
}

Point2d Intersections::resize(Point2d p){
	double ratioX = img_size/xDiff;
	double ratioY = img_size/yDiff;

	Point2d p_opencv;
	p_opencv.x = (p.x-minX)*ratioX;
	p_opencv.y = (maxY-p.y)*ratioY;

	return(p_opencv);
}

int Intersections::resize_radius(int r){
	double ratio = img_size/xDiff;
	return(r*ratio);
}

/* -------------------------------------------- */
/*        		       main       			    */
/* -------------------------------------------- */
void Intersections::solve(int mode){
	
	// mode:	0 Line-Line
	// 			1 Circle-Line
	

	srand((unsigned) time(0));

    set_plot(800);

    set_X(-10, 10);
    set_Y(-10, 10);

    vector<Point2d> line_a;
    vector<Point2d> line_b;

    Point2d center;
    int radius;

    int nSegm = 10;
    int s = 0;
    int nIters = 1;

    bool res;

    for(int i=0; i<nIters; i++){

    	if(mode==0){
    		line_a = {random_Point2d(), random_Point2d()};
    		line(img, resize(line_a[0]), resize(line_a[1]),Scalar(255,0,0), 2, LINE_AA );
    	}

    	if(mode==1){
    		center = random_Point2d();
	    	radius = random_radius();
	    	circle(img, resize(center), resize_radius(radius), Scalar(255,0,0), 2, LINE_AA);
    	}    	

    	while(s<nSegm){
    		line_b = {random_Point2d(), random_Point2d()};

    		if(mode==0){
    			res = intersLineLine(line_a, line_b);
    		}

    		if(mode==1){
    			res = intersCircleLine(center, radius, line_b);
    		}

    		if(true){ 
    			line(img, resize(line_b[0]), resize(line_b[1]), Scalar(0,0,0), 1, LINE_AA );
    			s+=1;
    		}
    		
    	}
    	  
    }

    char window[] = "Intersections";
    imshow(window, img);
    moveWindow(window, 0, 0);
    waitKey();
}

int main()
{
	Intersections inters;
	inters.solve(1);
	return(0);
}