#include "dubins.hpp"
#include "plot.h"


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
    for(int i = 0; i < primitives.size(); i++){
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
    int npts = 100;
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

int main()
{

    // Define problem data
    double x0 = 0;
    double y0 = 0;
    double th0 = -M_PI/2;
    double xf = 4; 
    double yf = 0;
    double thf = -M_PI/2;
    double kmax = 1.0;

    // Find optimal Dubins solution
    int pidx;
    curve curve;
    tie(pidx, curve) = dubins_shortest_path(x0, y0, th0, xf, yf, thf, kmax);

    // Plot and display solution if valid
    if (pidx > 0){
        plotdubins(curve);
    }
    else{
        cout << "Failed!" << endl;
    }
}
