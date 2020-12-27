#include "findRobot.hpp"
#include <math.h>
#include "utils.hpp"

// #define DEBUG_ACTIVE
// #define MAIN_ACTIVE // remember to change also the CmakeList.txt

/*!
  * Find the distance between two points
  * @param[in]  PI      firt point
  * @param[in]  P2      second point
  * @return[double]  the distance between points
  */
double findDistance(const Point& P1, const Point& P2){
    return sqrt( pow(P1.x - P2.x, 2) + pow(P1.y - P2.y, 2));
}

/*!
  * Find the vertex of a tringle (if it's equilater return P3)
  * @param[in]  PI      first point
  * @param[in]  P2      second point
  * @param[in]  P3      third point
  * @return[Point]  the vertex point
  */
Point findVertex(const Point& P1, const Point& P2, const Point& P3){
    double side_length[3];
    /* find distance between points */
    side_length[0] = findDistance(P1,P2);
    side_length[1] = findDistance(P1,P3);
    side_length[2] = findDistance(P2,P3);

    /* choose as vertex the corner connected to the two longhest sides */
    if((side_length[0] > side_length[2]) && (side_length[1] > side_length[2])) return P1;
    else if((side_length[0] > side_length[1]) && (side_length[2] > side_length[1]))return P2;
    else return P3;
}

bool student_findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){
    #ifdef DEBUG_ACTIVE
    cv::imshow("Original image", img_in);
    cv::waitKey();
    #endif

    /* convert image to HSV color space */
    cv::Mat img_hsv;
    cv::cvtColor(img_in, img_hsv, cv::COLOR_BGR2HSV);

    #ifdef DEBUG_ACTIVE
    cv::imshow("HSV image", img_hsv);
    cv::waitKey();
    #endif

    /* define low and high blue thresold values */
    cv::Scalar blueHSV_L(105,40,40);
    cv::Scalar blueHSV_H(125,255,255);
    cv::Mat blue_mask;

    /* apply the blue mask */
    cv::inRange(img_hsv, blueHSV_L, blueHSV_H, blue_mask);

    #ifdef DEBUG_ACTIVE
    cv::imshow("Blue mask", blue_mask);
    cv::waitKey();
    #endif

    /* find countorus */
    std::vector<std::vector<cv::Point> > blue_contours;
    cv::findContours(blue_mask, blue_contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
    std::vector<std::vector<cv::Point> >::iterator itvp; // create iterator
    int polygons_counter = 0;

    /* approximate contours */
    for(itvp = blue_contours.begin(); itvp != blue_contours.end(); itvp ++){
        cv::approxPolyDP(*itvp, *itvp, 9, true);
        polygons_counter++;
    }

    /* if there are more than one polygon so there is an arror.
     * FUTURE_IMPLEMENTATION: choose the biggest one
    */
    if(polygons_counter != 1){
        std::cout<<"[ERROR] Found "<<polygons_counter<<" polygons"<<std::endl;
        return false;
    }

    int corner_counter = 0;
    std::vector<cv::Point>::iterator itp;

    /* count the number of polygon's corners */
    for(itp = blue_contours[0].begin(); itp != blue_contours[0].end(); itp ++){
        corner_counter++;
    }

    /* if corner are more then 3 so there is an error */
    if(corner_counter != 3){
        std::cout<<"[ERROR] Found "<<corner_counter<<" corners"<<std::endl;
        return false;
    }

    /* save the polygon data into triangle variable (scaling them) */
    for(int i = 0 ; i < 3 ; i++){
        triangle.emplace_back(blue_contours[0][i].x/scale, blue_contours[0][i].y/scale);
    }
    Point P1,P2,P3;
    P1 = triangle[0];
    P2 = triangle[1];
    P3 = triangle[2];

    double cx,cy;

    /* find center of triangle */
    cx = (P1.x + P2.x + P3.x)/3;
    cy = (P1.y + P2.y + P3.y)/3;

    #ifdef DEBUG_ACTIVE
    std::cout<<"[INFO] Found triangle with cx = "<<cx<<", cy = "<<cy<<std::endl;
    std::cout<<"Distance P1-P2: "<<findDistance(P1,P2)<<std::endl;
    std::cout<<"Distance P1-P3: "<<findDistance(P1,P3)<<std::endl;
    std::cout<<"Distance P2-P3: "<<findDistance(P2,P3)<<std::endl;
    #endif

    /* find vertex of triangle */
    Point vertex = findVertex(P1,P2,P3);

    #ifdef DEBUG_ACTIVE
    std::cout<<"Vertex: "<<vertex.x<<" "<<vertex.y<<std::endl;
    std::cout<<"P1: "<<P1.x<<" "<<P1.y<<std::endl;
    std::cout<<"P2: "<<P2.x<<" "<<P2.y<<std::endl;
    std::cout<<"P3: "<<P3.x<<" "<<P3.y<<std::endl;
    #endif

    /* colculate theta */
    theta = atan2(cx-vertex.x, cy-vertex.y);
    x = cx;
    y = cy;

    #ifdef DEBUG_ACTIVE
    std::cout<<"Theta: "<<theta<<std::endl;
    #endif

    #ifdef DEBUG_ACTIVE
    cv::drawContours(img_in, blue_contours, -1, cv::Scalar(0,0,0), 3, cv::LINE_AA);
    // display che image
    cv::imshow("Image Contours", img_in);
    cv::waitKey();
    #endif

    return true;
}

#ifdef MAIN_ACTIVE
int main(int argc, char* argv[]){
    if(argc != 2){
        std::cout<<"[ERROR] Argument error, usage: ./findRobot image_name.jpg"<<std::endl;
        return -1;
    }
    std::string imageName(argv[1]);
    cv::Mat image = cv::imread(imageName);
    if(!image.empty()){
        std::cout<<"[INFO] Loaded "<<imageName<< " image"<<std::endl;
    }else{
        std::cout<<"[ERROR] Unable to load "<<imageName<< " image"<<std::endl;
        return -1;
    }

    // std::vector<Point3f> object_points;
    // cv::Mat camera_matrix, rvec, tvec;

    // student_extrinsicCalib(image, object_points, camera_matrix, rvec, tvec, "/tmp");

    Polygon triangle;
    double x,y, theta;

    student_findRobot(image, double(4), triangle, x, y, theta, "/tmp");

    return 0;
}
#endif
