#include "resize.hpp"

#define INT_ROUND 1000.

/*!
 * @brief Enlarge borders of the obstacles to not intersect the robot path
 * @param[in]  const Polygon &borders    initial obstacles
 * @param[in]  double robotSize  size of the robot
 * @return[Polygon] resized obstacles
!*/
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

        // apply offset to Clipper polygon 
        ClipperLib::ClipperOffset co;
        co.AddPath(clipperSrcObstacle, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
        co.Execute(clipperNewObstacle, robotSize);

        // add the inflated obstacle to the path
        cl.AddPaths(clipperNewObstacle, ClipperLib::ptSubject, true);

    }

    // merge all resized clipper obstacles to a new merged path
    cl.Execute(ClipperLib::ctUnion, mergedObstacles, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

    // transform Clipper to Polygon
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

/*!
 * @brief Reduce borders of the arena to not intersect the robot path
 * @param[in]  const Polygon &borders    initial borders
 * @param[in]  double robotSize  size of the robot
 * @return[Polygon] resized borders
!*/
Polygon resizeBorders(const Polygon &borders, double robotSize){
    Polygon newBorders;

    ClipperLib::Path clipperSrcBorder;
    ClipperLib::Paths clipperNewBorder;

    // transform Polygon to Clipper
    for (const auto &point : borders) {
        clipperSrcBorder << ClipperLib::IntPoint(point.x * INT_ROUND, point.y * INT_ROUND);
    }

    // apply offset to Clipper polygon 
    ClipperLib::ClipperOffset co;
    co.AddPath(clipperSrcBorder, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
    co.Execute(clipperNewBorder, -robotSize);

    // transform Clipper to Polygon
    for(const ClipperLib::Path &path : clipperNewBorder){
        for(const ClipperLib::IntPoint &pt: path){
            double x = pt.X / INT_ROUND;
            double y = pt.Y / INT_ROUND;
            newBorders.emplace_back(x, y);
        }
    }

    // reorder borders 
    Polygon orderedBorders;
    orderedBorders.emplace_back(newBorders[2]);
    orderedBorders.emplace_back(newBorders[3]);
    orderedBorders.emplace_back(newBorders[0]);
    orderedBorders.emplace_back(newBorders[1]);

    return orderedBorders;
}

