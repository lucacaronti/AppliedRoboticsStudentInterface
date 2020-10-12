#include "../inc/processMap.hpp"

/*!
    * \brief Print the statistics of the polygons shapes
    * \param[in]    _Polygons   The vector with Polygons inside
    * \param[in]    _name       The name of Polygons to print
*/
void printPolygonsShapeStat(const std::vector<Polygon>& _Polygons, const std::string& _name){
    std::vector<Polygon>::const_iterator itvp; // create an iterator
    int counter_polygons = 0; // polygon counter
    std::cout<<"-- "<< _name<<std::endl;
    for(itvp = _Polygons.begin(); itvp != _Polygons.end(); itvp++){
        int counter_polygons_shape = 0; // polygon shape counter
        Polygon::const_iterator itp; // create an iterator
        for(itp = itvp->begin(); itp != itvp->end(); itp++) counter_polygons_shape++; // count the number of shapes
        std::cout<<"  |--Polygon number "<<counter_polygons<<" has "<< counter_polygons_shape<< " shapes"<<std::endl; // print the stats
        counter_polygons++; // increase the counter
    }
    std::cout<<std::endl;
}
bool student_processMap::processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon> >& victim_list, Polygon& gate, const std::string& config_folder){
    cv::Mat imgHSV; // create image HSV
    cv::cvtColor(img_in, imgHSV, cv::COLOR_BGR2HSV); // convert BGR into HSV colorspace
    bool debug = false; // choose if enable debug or not

    // low and high green thresold values
    cv::Scalar greenHSV_L(52,40,40);
    cv::Scalar greenHSV_H(72,255,255);

    // low and high blue thresold values
    cv::Scalar blueHSV_L(105,40,40);
    cv::Scalar blueHSV_H(125,255,255);

    // low and high red thresold values
    cv::Scalar redHSV_L_1(170,40,40);
    cv::Scalar redHSV_H_1(180,255,255);
    cv::Scalar redHSV_L_2(0,40,40);
    cv::Scalar redHSV_H_2(10,255,255);

    cv::Mat greenObjs;
    cv::Mat blueObjs;
    cv::Mat redObjs;

    cv::inRange(imgHSV, greenHSV_L, greenHSV_H, greenObjs); // extract green objects
    cv::inRange(imgHSV, blueHSV_L, blueHSV_H, blueObjs); // extract blue objects

    // create 2 temporary Matrices 
    cv::Mat* redObjs_1 = new cv::Mat;
    cv::Mat* redObjs_2 = new cv::Mat;

    cv::inRange(imgHSV, redHSV_L_1, redHSV_H_1, *redObjs_1); // extract first half of red objects
    cv::inRange(imgHSV, redHSV_L_2, redHSV_H_2, *redObjs_2); // extract second half of red objects
    cv::bitwise_or(*redObjs_1, *redObjs_2, redObjs); // merge the two extracted objects
    delete redObjs_1, redObjs_2; // delete the 2 temporary Matrices 

    // display iamges if degub is enable
    if(debug){
        cv::imshow("Green Ojbs", greenObjs);
        cv::imshow("Blue Ojbs", blueObjs);
        cv::imshow("Red Ojbs", redObjs);
        cv::waitKey();
        cv::destroyAllWindows();
    }

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7,7)); // create kernel
    // make erosion
    cv::erode(greenObjs, greenObjs, element);
    cv::erode(blueObjs, blueObjs, element);
    cv::erode(redObjs, redObjs, element);
    
    // display iamges if degub is enable
    if(debug){
        cv::imshow("Green Ojbs after erosion", greenObjs);
        cv::imshow("Blue Ojbs after erosion", blueObjs);
        cv::imshow("Red Ojbs after erosion", redObjs);
        cv::waitKey();
        cv::destroyAllWindows();
    }

    // make dilatation
    cv::dilate(greenObjs, greenObjs, element);
    cv::dilate(blueObjs, blueObjs, element);
    cv::dilate(redObjs, redObjs, element);

    // display iamges if degub is enable
    if(debug){
        cv::imshow("Green Ojbs after dilate", greenObjs);
        cv::imshow("Blue Ojbs after dilate", blueObjs);
        cv::imshow("Red Ojbs after dilate", redObjs);
        cv::waitKey();
        cv::destroyAllWindows();
    }

    // create vectors of polygons
    // **NOTE** : change typedef into utils.hpp with "typedef std::vector<cv::Point> Polygon;"
    std::vector<Polygon> green_polygons; 
    std::vector<Polygon> blue_polygons;
    std::vector<Polygon> red_polygons;

    // find contours for each color
    cv::findContours(greenObjs, green_polygons, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(blueObjs, blue_polygons, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(redObjs, red_polygons, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // print polygons shape statistics if debug is enable
    if(debug){
        printPolygonsShapeStat(green_polygons, std::string("Green"));
        printPolygonsShapeStat(blue_polygons, std::string("Blue"));
        printPolygonsShapeStat(red_polygons, std::string("Red"));
    }

    std::vector<Polygon>::iterator itvp; // create iterator
    // make the approximation on the contours
    for(itvp = green_polygons.begin(); itvp != green_polygons.end(); itvp ++)cv::approxPolyDP(*itvp, *itvp, 5, true);
    for(itvp = blue_polygons.begin(); itvp != blue_polygons.end(); itvp ++)cv::approxPolyDP(*itvp, *itvp, 5, true);
    for(itvp = red_polygons.begin(); itvp != red_polygons.end(); itvp ++)cv::approxPolyDP(*itvp, *itvp, 5, true);

    // print polygons shape statistics if debug is enable
    if(debug){
        printPolygonsShapeStat(green_polygons, std::string("Green after approximation"));
        printPolygonsShapeStat(blue_polygons, std::string("Blue after approximation"));
        printPolygonsShapeStat(red_polygons, std::string("Red after approximation"));
    }

    // draw contours
    cv::drawContours(img_in, green_polygons, -1, cv::Scalar(0,0,0), 1, cv::LINE_AA);
    cv::drawContours(img_in, blue_polygons, -1, cv::Scalar(0,0,0), 1, cv::LINE_AA);
    cv::drawContours(img_in, red_polygons, -1, cv::Scalar(0,0,0), 1, cv::LINE_AA);

    // display che image
    cv::imshow("Image Contours", img_in);

    cv::waitKey();

    return true;
}


int main(int argc, char* argv[]){
    if(argc != 2){
        std::cout<<"[ERROR] Argument error, usage: ./processMap image_name.jpg"<<std::endl;
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
    std::vector<Polygon> obstacle_list;
    std::vector<std::pair<int,Polygon> > victim_list;
    Polygon gate;

    student_processMap::processMap(image, double(1/3), obstacle_list, victim_list, gate, "/tmp");

    return 0;
}