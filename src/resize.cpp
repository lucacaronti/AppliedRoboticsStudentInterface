#include "resize.hpp"

#define INT_ROUND 1000.

std::vector<Polygon> resizeObstacles(const std::vector<Polygon> &obstacles, double robotSize){
    std::vector<Polygon> newObstacles;

    ClipperLib::Clipper cl;
    ClipperLib::Paths mergedObstacles;

    for (const Polygon &obstacle : obstacles) {
        ClipperLib::Path clipperSrcObstacle;
        ClipperLib::Paths clipperNewObstacle;
        

        // transform Polygon to Clipper
        for (const auto &point : obstacle) {
            clipperSrcObstacle << ClipperLib::IntPoint(point.x * INT_ROUND, point.y * INT_ROUND);
        }

        ClipperLib::ClipperOffset co;
        co.AddPath(clipperSrcObstacle, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
        co.Execute(clipperNewObstacle, robotSize);

        cl.AddPaths(clipperNewObstacle, ClipperLib::ptSubject, true);

    }

    cl.Execute(ClipperLib::ctUnion, mergedObstacles, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

    for(const ClipperLib::Path &path : mergedObstacles){
        Polygon newPolygon;
        for(const ClipperLib::IntPoint &pt: path){
            double x = pt.X / INT_ROUND;
            double y = pt.Y / INT_ROUND;
            newPolygon.emplace_back(x, y);
        }
        newObstacles.emplace_back(newPolygon);
    }

    return newObstacles;
}

Polygon resizeBorders(const Polygon &borders, double robotSize){
    Polygon newBorders;

    ClipperLib::Path clipperSrcBorder;
    ClipperLib::Paths clipperNewBorder;

    // transform Polygon to Clipper
    for (const auto &point : borders) {
        clipperSrcBorder << ClipperLib::IntPoint(point.x * INT_ROUND, point.y * INT_ROUND);
    }

    ClipperLib::ClipperOffset co;
    co.AddPath(clipperSrcBorder, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
    co.Execute(clipperNewBorder, -robotSize);

    for(const ClipperLib::Path &path : clipperNewBorder){
        for(const ClipperLib::IntPoint &pt: path){
            double x = pt.X / INT_ROUND;
            double y = pt.Y / INT_ROUND;
            newBorders.emplace_back(x, y);
        }
    }

    Polygon orderedBorders;
    orderedBorders.emplace_back(newBorders[2]);
    orderedBorders.emplace_back(newBorders[3]);
    orderedBorders.emplace_back(newBorders[0]);
    orderedBorders.emplace_back(newBorders[1]);

    return orderedBorders;
}

