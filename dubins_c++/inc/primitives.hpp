#ifndef PRIMITIVES
#define PRIMITIVES

#include <iostream>
#include <cmath>
#include <tuple>
#include <vector>
#include "dubins.hpp"
using namespace std;

tuple<bool, double, double, double> LSL(double sc_th0, double sc_thf, double sc_Kmax);

tuple<bool, double, double, double> RSR(double sc_th0, double sc_thf, double sc_Kmax);

tuple<bool, double, double, double> LSR(double sc_th0, double sc_thf, double sc_Kmax);

tuple<bool, double, double, double> RSL(double sc_th0, double sc_thf, double sc_Kmax);

tuple<bool, double, double, double> RLR(double sc_th0, double sc_thf, double sc_Kmax);

tuple<bool, double, double, double> LRL(double sc_th0, double sc_thf, double sc_Kmax);



#endif