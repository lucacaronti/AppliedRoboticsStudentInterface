#include "primitives.hpp"
#include "plot.h"

#include <vector>
#include <string>

/* ------------------------------------------- */
/*              data structures                */
/* ------------------------------------------- */
// Create a structure representing an arc of a Dubins curve (straight or circular)
arc dubinsarc(double x0, double y0, double th0, double k, double L){
    arc c;
    c.x0 = x0;
    c.y0 = y0;
    c.th0 = th0;
    c.k = k;
    c.L = L;
    
    tie(c.xf, c.yf, c.thf) = circline(L, x0, y0, th0, k);

    return c;
}
// Create a structure representing a Dubins curve (composed by three arcs)
curve dubinscurve (double x0, double y0, double th0, 
                    double s1, double s2, double s3, double k0, double k1, double k2){
    curve d;
    d.a1 = dubinsarc(x0, y0, th0, k0, s1);
    d.a2 = dubinsarc(d.a1.xf, d.a1.yf, d.a1.thf, k1, s2);
    d.a3 = dubinsarc(d.a2.xf, d.a2.yf, d.a2.thf, k2, s3);
    d.L = d.a1.L + d.a2.L + d.a3.L;

    return d;
}

/* ------------------------------------------- */
/*             auxiliary functions             */
/* ------------------------------------------- */
// Implementation of function sinc(t), returning 1 for t==0, and sin(t)/t otherwise
double sinc(double t){
    double s;

    if(abs(t) < 0.002){
        s = 1 - pow(t, 2) * (1/6 - pow(t, 2)/20);
    }else{
        s = sin(t)/t;
    }
    return s;
}

// Normalize an angle (in range [0,2*pi))
double mod2pi(double ang){
	double out = ang;

    while(out < 0){
        out = out + 2 * M_PI;
    }

    while(out >= 2 * M_PI){
        out = out - 2 * M_PI;
    }

    return out;
}

// Normalize an angular difference (range (-pi, pi])
double rangeSymm(double ang){
	double out = ang;
	
	while(out <= -M_PI){
        out = out + 2 * M_PI;
    }

	while(out > M_PI){
        out = out - 2 * M_PI;
    }

	return out;
}

// Check validity of a solution by evaluating explicitly the 3 equations  
// defining a Dubins problem (in standard form)
bool check(double s1, double k0, double s2, double k1, double s3, double k2, double th0, double thf){
	double x0 = -1;
	double y0 = 0;
  	double xf = 1;
  	double yf = 0;

	double eq1 = x0 + s1 * sinc((1/2.) * k0 * s1) * cos(th0 + (1/2.) * k0 * s1) 
			        + s2 * sinc((1/2.) * k1 * s2) * cos(th0 + k0 * s1 + (1/2.) * k1 * s2)
			        + s3 * sinc((1/2.) * k2 * s3) * cos(th0 + k0 * s1 + k1 * s2 + (1/2.) * k2 * s3) - xf;

	double eq2 = y0 + s1 * sinc((1/2.) * k0 * s1) * sin(th0 + (1/2.) * k0 * s1)
			        + s2 * sinc((1/2.) * k1 * s2) * sin(th0 + k0 * s1 + (1/2.) * k1 * s2)
                    + s3 * sinc((1/2.) * k2 * s3) * sin(th0 + k0 * s1 + k1 * s2 + (1/2.) * k2 * s3) - yf;

	double eq3 = rangeSymm(k0 * s1 + k1 * s2 + k2 * s3 + th0 - thf);

	bool Lpos = (s1 > 0) || (s2 > 0) || (s3 > 0);
	bool res = (sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < 1.e-10) && Lpos;

	return res;	
}

/* -------------------------------------------- */
/* Functions to scale and solve Dubins problems */
/* -------------------------------------------- */
// Scale the input problem to standard form (x0: -1, y0: 0, xf: 1, yf: 0)
tuple<double, double, double, double> scaleToStandard(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax){
    double dx = xf - x0;
    double dy = yf - y0;
    double phi = atan2(dy, dx);
    double lambda = hypot(dx, dy)/2;

    double sc_th0 = mod2pi(th0 - phi); 
    double sc_thf = mod2pi(thf - phi); 
    double sc_Kmax = Kmax * lambda;

    return make_tuple(sc_th0, sc_thf, sc_Kmax, lambda);
}

// Scale the solution to the standard problem back to the original problem
tuple<double, double, double> scaleFromStandard(double lambda, double sc_s1, double sc_s2, double sc_s3){
    double s1 = sc_s1 * lambda;
    double s2 = sc_s2 * lambda;
    double s3 = sc_s3 * lambda;

    return make_tuple(s1,s2,s3);
}

/* ------------------------------------------------------- */
/* Solve the Dubins problem for the given input parameters */
/* ------------------------------------------------------- */
// Solve the Dubins problem for the given input parameters.
// Return the type and the parameters of the optimal curve
pair<int, curve> dubins_shortest_path(double x0, double y0, double th0, double xf, double yf, double thf, double kmax){
    double sc_s1, sc_s2, sc_s3;

    double sc_th0, sc_thf, sc_Kmax, lambda;
    tie(sc_th0, sc_thf, sc_Kmax, lambda) = scaleToStandard(x0, y0, th0, xf, yf, thf, kmax);

    // Define the functions corresponding to the different primitives, and the 
    // corresponding curvatue signs
    vector<tuple<bool, double, double, double> (*)(double, double, double)> primitives = 
        {&LSL, &RSR, &LSR, &RSL, &RLR, &LRL}; 

    double ksigns[6][6] = {
        { 1, 0, 1 }, //LSL
        {-1, 0,-1 }, //RSR
        { 1, 0,-1 }, //LSR
        {-1, 0, 1 }, //RSL
        {-1, 1,-1 }, //RLR
        { 1,-1, 1 }  //LRL
    };

    // Try all the possible primitives, to find the optimal solution
    double L = INFINITY;
    int pidx = -1;
    for(uint i = 0; i < primitives.size(); i++){
        bool ok;
        double sc_s1_c, sc_s2_c, sc_s3_c;

        tie(ok, sc_s1_c, sc_s2_c, sc_s3_c) = primitives[i](sc_th0, sc_thf, sc_Kmax);
        double Lcur = sc_s1_c + sc_s2_c + sc_s3_c;

        if(ok && Lcur < L){
            L = Lcur;
            sc_s1 = sc_s1_c;
            sc_s2 = sc_s2_c;
            sc_s3 = sc_s3_c;

            pidx = i;
        }
    }

    curve cur;

    if(pidx >= 0){
        // Transform the solution to the problem in standard form to the 
        // solution of the original problem (scale the lengths)  
        double s1, s2, s3;
        tie(s1, s2, s3) = scaleFromStandard(lambda, sc_s1, sc_s2, sc_s3);

        // Construct the Dubins curve object with the computed optimal parameters
        cur = dubinscurve(x0, y0, th0, s1, s2, s3, ksigns[pidx][0]*kmax, ksigns[pidx][1]*kmax, ksigns[pidx][2]*kmax);

        // Check the correctness of the algorithm
        // assert(check(sc_s1, ksigns[pidx][1]*sc_Kmax, sc_s2, ksigns[pidx][2]*sc_Kmax, sc_s3, ksigns[pidx][3]*sc_Kmax, sc_th0, sc_thf));
    }

    return make_pair(pidx, cur);
}


/* -------------------------------------------- */
/*        Functions to plot Dubins curves       */
/* -------------------------------------------- */
// Evaluate an arc (circular or straight) composing a Dubins curve, at a given arc-length s
tuple<double, double, double> circline(double s, double x0, double y0, double th0, double k){
    double x = x0 + s * sinc(k * s / 2.0) * cos(th0 + k * s /2);
    double y = y0 + s * sinc(k * s / 2.0) * sin(th0 + k * s /2);
    double th = mod2pi(th0 + k * s);

    return make_tuple(x, y, th);
}

// Plot an arc (circular or straight) composing a Dubins curve
tuple<vector<float>, vector<float>> plotarc(arc arc){
    int npts = 1000;
    vector<float> data_x;
    vector<float> data_y;

    double s;
    double x;
    double y;
    double th;

    for(int j=0; j<npts; j++){
        s = arc.L/npts*j;
        tie(x, y, th) = circline(s, arc.x0, arc.y0, arc.th0, arc.k);
        if(x!=0){
            data_x.push_back(x);
            data_y.push_back(y); 
        }  
    }
    
    return make_tuple(data_x, data_y);

}

// Plot a Dubins curve
void plotdubins(curve curve){
    vector<float> data_x;
    vector<float> data_y;

    auto axes = CvPlot::makePlotAxes();

    // Plot arcs
    tie(data_x, data_y) = plotarc(curve.a1);
    axes.create<CvPlot::Series>(data_x, data_y, "-g"); 

    tie(data_x, data_y) = plotarc(curve.a2);
    axes.create<CvPlot::Series>(data_x, data_y, "-b"); 

    tie(data_x, data_y) = plotarc(curve.a3);
    axes.create<CvPlot::Series>(data_x, data_y, "-r"); 

    Mat mat = axes.render(300, 400);
    imshow("Dubins Best Path", mat);

    waitKey();

    return;
}

// Plot a Multi Dubins curve
void multiplotdubins(const vector<pair<int,curve> > & multi_curve){
    vector<float> data_x;
    vector<float> data_y;

    auto axes = CvPlot::makePlotAxes();

    for(long unsigned int i = 0; i < multi_curve.size(); i++){
        // Plot arcs
        tie(data_x, data_y) = plotarc(multi_curve[i].second.a1);
        axes.create<CvPlot::Series>(data_x, data_y, "-g"); 

        tie(data_x, data_y) = plotarc(multi_curve[i].second.a2);
        axes.create<CvPlot::Series>(data_x, data_y, "-b"); 

        tie(data_x, data_y) = plotarc(multi_curve[i].second.a3);
        axes.create<CvPlot::Series>(data_x, data_y, "-r"); 

        axes.create<CvPlot::Series>(float(multi_curve[i].second.a1.x0), float(multi_curve[i].second.a1.y0), "k-o");
        if(i == 0){
            axes.create<CvPlot::Series>(float(multi_curve[i].second.a3.xf), float(multi_curve[i].second.a3.yf), "k-o");
        }
    }

    Mat mat = axes.render(800, 800);
    imshow("Dubins Best Path", mat);

    waitKey();

    return;
}

DubinsCurve::DubinsCurve(){
    x0 = 0;
    xf = 0;
    y0 = 0;
    yf = 0;
    th0 = 0;
    thf = 0;
    kmax = 0;
}

DubinsCurve::~DubinsCurve(){
}

void DubinsCurve::set_k(double _k){
    kmax = _k;
}

void DubinsCurve::add_start_data(double _x0, double _y0, double _th0){
    x0 = _x0;
    y0 = _y0;
    th0 = _th0;
}
void DubinsCurve::add_final_data(double _xf, double _yf, double _thf){
    xf = _xf;
    yf = _yf;
    thf = _thf;
}

void normalize_0_2pi(vector<double> & angles){
    for(auto it_double = angles.begin(); it_double != angles.end(); it_double++){
        if(*it_double > 2*M_PI){
            *it_double = *it_double - 2*M_PI;
        }
    }
}
// Fill a vector with all theta to test. From 0 to 2*PI with a given step
void generate_angles(vector<double> & angles, double step){
    for(double theta = 0; theta < 2*M_PI ; theta += step){
        angles.push_back(theta);
    }
}

void generate_angles_with_range(vector<double>& angles, double theta0, double h, int k){
    for(int i = -k/2; i <= k/2 ; i++){
        angles.push_back(theta0 + 3*h/k * i);
    }
    normalize_0_2pi(angles);
}

void DubinsCurve::add_middle_points(double px, double py){
    middle_points.emplace_back(Point2d(px,py));
}
bool DubinsCurve::solver(int m, int k){
    for(int i = 0 ; i < m ; i++){
        double l_value;
        vector<pair<int, curve> > tmp_multi_curve; 
        l_value = L(0, middle_points.size() + 1, middle_points, x0, y0, th0, xf, yf, thf, kmax, multi_curve,tmp_multi_curve,i, k);
        multi_curve.clear();
        multi_curve = tmp_multi_curve;
        cout<<"With m = "<<i<<" -> L = "<<l_value<<endl;
    }
    for(auto it_mc = multi_curve.begin(); it_mc != multi_curve.end(); it_mc++){
        if(it_mc->first < 0)
            return false;
    }
    return true;
}

void DubinsCurve::plot() const{
    vector<float> data_x;
    vector<float> data_y;

    auto axes = CvPlot::makePlotAxes();

    for(long unsigned int i = 0; i < multi_curve.size(); i++){
        // Plot arcs
        tie(data_x, data_y) = plotarc(multi_curve[i].second.a1);
        axes.create<CvPlot::Series>(data_x, data_y, "-g"); 

        tie(data_x, data_y) = plotarc(multi_curve[i].second.a2);
        axes.create<CvPlot::Series>(data_x, data_y, "-b"); 

        tie(data_x, data_y) = plotarc(multi_curve[i].second.a3);
        axes.create<CvPlot::Series>(data_x, data_y, "-r"); 

        axes.create<CvPlot::Series>(float(multi_curve[i].second.a1.x0), float(multi_curve[i].second.a1.y0), "k-o");
        if(i == 0){
            axes.create<CvPlot::Series>(float(multi_curve[i].second.a3.xf), float(multi_curve[i].second.a3.yf), "k-o");
        }
    }

    Mat mat = axes.render(800, 800);
    imshow("Dubins Best Path", mat);

    waitKey();

    return;
}

// Compute the distance of subset path. Save also the found curves
double minD_j(double x0,double y0, double xf, double yf, double k, const vector<double>& th0s, double thf, \
        vector<pair<int ,curve> >& multi_curve ){
    
    vector<double>::const_iterator itd; // Define an iterator

    curve best_curve;
    best_curve.L = INFINITY;
    int best_curve_pidx = -1;

    // Find the best curve trying with all angles
    for(itd = th0s.begin(); itd != th0s.end(); itd++){
        int pidx;
        curve curve;
        tie(pidx, curve) = dubins_shortest_path(x0, y0, *itd, xf, yf, thf, k); // Compute a single dubins problem
        if(pidx >= 0){
            if(curve.L < best_curve.L){
                best_curve = curve;
                best_curve_pidx = pidx;
            }
        }else{
            cout<<"ERRORE"<<endl;
        }
    }
    // Save the best curve
    multi_curve.emplace_back(pair<int, struct curve>(best_curve_pidx, best_curve));
    return best_curve.L;
}

double L(int j, int n, const vector<Point2d>& middle_points, 
        double x0, double y0, double th0, double xf, double yf, double thf, double kmax,
        vector<pair<int ,curve> >& multi_curve ,vector<pair<int ,curve> >& tmp_multi_curve , int m, int k){
    
    if(n < 2){ // Check if there is at least one intermediate point
        vector<double> firsts_th0;
        firsts_th0.emplace_back(th0); // Set that first thet0 is known
        return minD_j(x0, y0, xf, yf, kmax, firsts_th0, thf, tmp_multi_curve);
    }else{
        if(j == n-1){
            vector<double> angles;
            if(m == 0){
                generate_angles(angles, 2*M_PI/k);
            }else{
                double h = 2*M_PI*pow(3,m)/(pow(k*2,m));
                generate_angles_with_range(angles, multi_curve[n-j-1].second.a1.th0, h, k);
            }
            // Last case (j == n-1) -> L = 0 
            return minD_j(middle_points[j-1].x, middle_points[j-1].y, xf, yf, kmax, angles, thf, tmp_multi_curve);
        }
        // Define temporaney l (length)
        double tmp_L = L(j+1, n, middle_points, x0, y0, th0, xf, yf, thf, kmax, multi_curve, tmp_multi_curve, m, k);
        // Define the last th0 found
        double last_th0 = tmp_multi_curve[n-j-2].second.a1.th0;
        if(j == 0){
            vector<double> firsts_th0;
            firsts_th0.emplace_back(th0); // Set that first thet0 is known
            return minD_j(x0, y0, middle_points[j].x, middle_points[j].y, kmax, firsts_th0, last_th0, tmp_multi_curve) + tmp_L;
        }else{
            vector<double> angles;
            if(m == 0){
                generate_angles(angles, 2*M_PI/k);
            }else{
                double h = 2*M_PI*pow(3,m)/(pow(k*2,m));
                generate_angles_with_range(angles, multi_curve[n-j-1].second.a1.th0, h, k);
            }
            return minD_j(middle_points[j-1].x, middle_points[j-1].y, middle_points[j].x, middle_points[j].y, kmax, angles, last_th0, tmp_multi_curve) + tmp_L;
        }
    }
}

int main()
{

    DubinsCurve dubinsCurve;

    dubinsCurve.set_k(1);
    dubinsCurve.add_start_data(1,1,-M_PI/2);
    dubinsCurve.add_final_data(4,4,-M_PI/2);
    dubinsCurve.add_middle_points(2,3);
    dubinsCurve.add_middle_points(3,5);
    dubinsCurve.add_middle_points(1,15);
    dubinsCurve.add_middle_points(10,11);
    dubinsCurve.add_middle_points(3,13);
    dubinsCurve.add_middle_points(2,10);
    dubinsCurve.add_middle_points(10,4);
    dubinsCurve.add_middle_points(4,1);

    bool ret = dubinsCurve.solver(4, 16);
    if(!ret) {cout<<"[ERROR] unable to solve dubins curve"<<endl; exit(EXIT_FAILURE);}
    
    dubinsCurve.plot();
}
