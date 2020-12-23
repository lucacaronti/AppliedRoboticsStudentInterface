#include <vector>
#include <string>
#include <cmath>

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

// Given three colinear points p, q, r, the function checks if 
// point q lies on line segment 'pr' 
bool onSegment(Point2d p, Point2d q, Point2d r) 
{ 
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && 
        q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y)) 
       return true; 
  
    return false; 
} 
  
bool intersection(Point2d o1, Point2d p1, Point2d o2, Point2d p2)
{
    Point2d x = o2 - o1;
    Point2d d1 = p1 - o1;
    Point2d d2 = p2 - o2;

    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < /*EPS*/1e-8)
        return false;
    return true;
}

// To find orientation of ordered triplet (p, q, r). 
// The function returns following values 
// 0 --> p, q and r are colinear 
// 1 --> Clockwise 
// 2 --> Counterclockwise 
int orientation(Point2d p, Point2d q, Point2d r) 
{ 
    // See https://www.geeksforgeeks.org/orientation-3-ordered-points/ 
    // for details of below formula. 
    int val = (q.y - p.y) * (r.x - q.x) - 
              (q.x - p.x) * (r.y - q.y); 
  
    if (val == 0) return 0;  // colinear 
  
    return (val > 0)? 1: 2; // clock or counterclock wise 
} 
  
// The main function that returns true if line segment 'p1q1' 
// and 'p2q2' intersect. 
bool doIntersect(Point2d p1, Point2d q1, Point2d p2, Point2d q2) 
{ 
    // Find the four orientations needed for general and 
    // special cases 
    int o1 = orientation(p1, q1, p2); 
    int o2 = orientation(p1, q1, q2); 
    int o3 = orientation(p2, q2, p1); 
    int o4 = orientation(p2, q2, q1); 
  
    // General case 
    if (o1 != o2 && o3 != o4){
        return true; 
	}
    // Special Cases 
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1 
    if (o1 == 0 && onSegment(p1, p2, q1)){return true; }
  
    // p1, q1 and q2 are colinear and q2 lies on segment p1q1 
    if (o2 == 0 && onSegment(p1, q2, q1)){return true; }
  
    // p2, q2 and p1 are colinear and p1 lies on segment p2q2 
    if (o3 == 0 && onSegment(p2, p1, q2)){return true; } 
  
     // p2, q2 and q1 are colinear and q1 lies on segment p2q2 
    if (o4 == 0 && onSegment(p2, q1, q2)){return true; }
  
    return false; // Doesn't fall in any of the above cases 
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
	// cout<<"dati: "<<x1<<" "<<y1<<" "<<x2<<" "<<y2<<" "<<endl;
	// cout<<"dati: "<<x3<<" "<<y3<<" "<<x4<<" "<<y4<<" "<<endl<<endl;
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
	// cout<<"dati: "<<crossRS<<" "<<crossDiffR<<" "<<DBL_EPSILON<<endl;
   	if(fabs(crossRS) <= DBL_EPSILON && fabs(crossDiffR) <= DBL_EPSILON){
		// cout<<"true "<<crossRS<<" "<<crossDiffR<<endl;
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
		// cout<<crossRS<<" "<<crossDiffR<<" "<<DBL_EPSILON<<endl;
   		if(fabs(crossRS) <= DBL_EPSILON && fabs(crossDiffR) > DBL_EPSILON){
			   cout<<"false"<<endl;
   			return false;
   		}else{
   			t = crossDiffS/crossRS;
   			double u = crossDiffR/crossRS;
   			if(t>=0 && t<=1 && u>=0 && u<=1){
   				ts = {t};
   			}
   		}
   	}

   	// for(int t=0; t<ts.size(); t++){
   	// 	Point2d pt = Point2d(p.x+ts[t]*r.x, p.y+ts[t]*r.y);
   	// 	pts.emplace_back(pt);
   	// 	drawMarker(img, resize(pt), Scalar(0,0,255),MARKER_TILTED_CROSS ,20,2,LINE_AA);
   	// }
   	
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