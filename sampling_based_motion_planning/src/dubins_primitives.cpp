#include "primitives.hpp"
#include <iostream>
#include <cmath>
#include <vector>
using namespace std;
/* ------------------------------------------- */
/*                   LSL                       */
/* ------------------------------------------- */
tuple<bool, double, double, double> LSL(double sc_th0, double sc_thf, double sc_Kmax){
    double invK = 1 / sc_Kmax;

    double C = cos(sc_thf) - cos(sc_th0);

    double S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);

    double temp1 = atan2(C, S);

    double sc_s1 = invK * mod2pi(temp1 - sc_th0);

    double temp2 = 2 + 4 * pow(sc_Kmax, 2) - 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));
    if(temp2 < 0){
        return make_tuple(false, 0, 0, 0);
    }

    double sc_s2 = invK * sqrt(temp2);
    double sc_s3 = invK * mod2pi(sc_thf - temp1);

    return make_tuple(true, sc_s1, sc_s2, sc_s3);
}

/* ------------------------------------------- */
/*                   RSR                       */
/* ------------------------------------------- */
tuple<bool, double, double, double> RSR(double sc_th0, double sc_thf, double sc_Kmax){
    double invK = 1 / sc_Kmax;

    double C = cos(sc_th0) - cos(sc_thf);

    double S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);

    double temp1 = atan2(C, S);
    double sc_s1 = invK * mod2pi(sc_th0 - temp1);

    double temp2 = 2 + 4 * pow(sc_Kmax, 2) - 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));
    if(temp2 < 0){
        return make_tuple(false, 0, 0, 0);
    }

    double sc_s2 = invK * sqrt(temp2);
    double sc_s3 = invK * mod2pi(temp1 - sc_thf);

    return make_tuple(true, sc_s1, sc_s2, sc_s3);
}

/* ------------------------------------------- */
/*                   LSR                       */
/* ------------------------------------------- */
tuple<bool, double, double, double> LSR(double sc_th0, double sc_thf, double sc_Kmax){
    double invK = 1 / sc_Kmax;

    double C = cos(sc_th0) + cos(sc_thf);

    double S = 2 * sc_Kmax + sin(sc_th0) + sin(sc_thf);

    double temp1 = atan2(-C, S);
    double temp3 = 4 * pow(sc_Kmax, 2) - 2 + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));
    
    if(temp3 < 0){
        return make_tuple(false, 0, 0, 0);
    }

    double sc_s2 = invK * sqrt(temp3);
    double temp2 = -atan2(-2, sc_s2 * sc_Kmax);
    double sc_s1 = invK * mod2pi(temp1 + temp2 - sc_th0);
    double sc_s3 = invK * mod2pi(temp1 + temp2 - sc_thf);

    return make_tuple(true, sc_s1, sc_s2, sc_s3);
}

/* ------------------------------------------- */
/*                   RSL                       */
/* ------------------------------------------- */
tuple<bool, double, double, double> RSL(double sc_th0, double sc_thf, double sc_Kmax){
    double invK = 1 / sc_Kmax;

    double C = cos(sc_th0) + cos(sc_thf);

    double S = 2 * sc_Kmax - sin(sc_th0) - sin(sc_thf);

    double temp1 = atan2(C, S);
    double temp3 = 4 * pow(sc_Kmax, 2) - 2 + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));
    
    if(temp3 < 0){
        return make_tuple(false, 0, 0, 0);
    }

    double sc_s2 = invK * sqrt(temp3);
    double temp2 = atan2(2, sc_s2 * sc_Kmax);
    double sc_s1 = invK * mod2pi(sc_th0 - temp1 + temp2);
    double sc_s3 = invK * mod2pi(sc_thf - temp1 + temp2);

    return make_tuple(true, sc_s1, sc_s2, sc_s3);
}

/* ------------------------------------------- */
/*                   RLR                       */
/* ------------------------------------------- */
tuple<bool, double, double, double> RLR(double sc_th0, double sc_thf, double sc_Kmax){
    double invK = 1 / sc_Kmax;

    double C = cos(sc_th0) - cos(sc_thf);

    double S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);

    double temp1 = atan2(C, S);
    double temp2 = 0.125 * (6 - 4 * pow(sc_Kmax, 2) + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));

    if(abs(temp2) > 1){
        return make_tuple(false, 0, 0, 0);
    }

    double sc_s2 = invK * mod2pi(2 * M_PI - acos(temp2));
    double sc_s1 = invK * mod2pi(sc_th0 - temp1 + 0.5 * sc_s2 * sc_Kmax);
    double sc_s3 = invK * mod2pi(sc_th0 - sc_thf + sc_Kmax * (sc_s2 - sc_s1));

    return make_tuple(true, sc_s1, sc_s2, sc_s3);
}

/* ------------------------------------------- */
/*                   LRL                       */
/* ------------------------------------------- */
tuple<bool, double, double, double> LRL(double sc_th0, double sc_thf, double sc_Kmax){
    double invK = 1 / sc_Kmax;

    double C = cos(sc_thf) - cos(sc_th0);

    double S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);

    double temp1 = atan2(C, S);
    double temp2 = 0.125 * (6 - 4 * pow(sc_Kmax, 2) + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));

    if(abs(temp2) > 1){
        return make_tuple(false, 0, 0, 0);
    }

    double sc_s2 = invK * mod2pi(2 * M_PI - acos(temp2));
    double sc_s1 = invK * mod2pi(temp1 - sc_th0 + 0.5 * sc_s2 * sc_Kmax);
    double sc_s3 = invK * mod2pi(sc_thf - sc_th0 + sc_Kmax * (sc_s2 - sc_s1));

    return make_tuple(true, sc_s1, sc_s2, sc_s3);
}
