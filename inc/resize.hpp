#include <iostream>
#include <string>
#include <vector>
#include "utils.hpp"
#include "clipper.hpp"

std::vector<Polygon> resizeObstacles(const std::vector<Polygon> &obstacles, double robotSize);
Polygon resizeBorders(const Polygon &borders, double robotSize);