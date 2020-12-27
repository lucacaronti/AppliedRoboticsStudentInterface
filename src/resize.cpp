#include "resize.hpp"

std::vector<Polygon> resizeObstacles(const std::vector<Polygon> &obstacles, double robotSize){
    std::vector<Polygon> newObstacles;

    for (const Polygon &obstacle : obstacles) {
        ClipperLib::Path clipperSrcObstacle;
        ClipperLib::Paths clipperNewObstacle;
        

        // transform Polygon to Clipper
        for (const auto &point : obstacle) {
            clipperSrcObstacle << ClipperLib::IntPoint(point.x * 1000.0, point.y * 1000.0);
        }

        ClipperLib::ClipperOffset co;
        co.AddPath(clipperSrcObstacle, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
        co.Execute(clipperNewObstacle, robotSize);

        Polygon newPolygon;
        // transform Clipper to Polygon
        for(const ClipperLib::Path &path: clipperNewObstacle){
            for(const ClipperLib::IntPoint &pt: path){
                double x = pt.X / 1000.0;
                double y = pt.Y / 1000.0;
                newPolygon.emplace_back(x, y);
            }
        }

        newObstacles.emplace_back(newPolygon);
    }
    

    return newObstacles;
}