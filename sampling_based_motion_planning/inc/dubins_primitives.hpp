#ifndef PRIMITIVES
#define PRIMITIVES

#include <tuple>
#include "dubins.hpp"


std::tuple<bool, double, double, double> LSL(double sc_th0, double sc_thf, double sc_Kmax);

std::tuple<bool, double, double, double> RSR(double sc_th0, double sc_thf, double sc_Kmax);

std::tuple<bool, double, double, double> LSR(double sc_th0, double sc_thf, double sc_Kmax);

std::tuple<bool, double, double, double> RSL(double sc_th0, double sc_thf, double sc_Kmax);

std::tuple<bool, double, double, double> RLR(double sc_th0, double sc_thf, double sc_Kmax);

std::tuple<bool, double, double, double> LRL(double sc_th0, double sc_thf, double sc_Kmax);



#endif