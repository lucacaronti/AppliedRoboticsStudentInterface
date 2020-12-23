#include "processMap.hpp"
#include "utils.hpp"
#include "detectDigits.hpp"

// #define BLUE_GATE
// #define DEBUG_ACTIVE
// #define MAIN_ACTIVE // remember to change also the CmakeList.txt

/*!
    * \brief Print the statistics of the polygons shapes
    * \param[in]    _Polygons   The vector with Polygons inside
    * \param[in]    _name       The name of Polygons to print
*/
void printPolygonsShapeStat(const std::vector<std::vector<cv::Point> >& _Polygons, const std::string& _name){
    std::vector<std::vector<cv::Point> >::const_iterator itvp; // create an iterator
    int counter_polygons = 0; // polygon counter
    std::cout<<"-- "<< _name<<std::endl;
    for(itvp = _Polygons.begin(); itvp != _Polygons.end(); itvp++){
        int counter_polygons_shape = 0; // polygon shape counter
        std::vector<cv::Point>::const_iterator itp; // create an iterator
        for(itp = itvp->begin(); itp != itvp->end(); itp++) counter_polygons_shape++; // count the number of shapes
        std::cout<<"  |--Polygon number "<<counter_polygons<<" has "<< counter_polygons_shape<< " shapes"<<std::endl; // print the stats
        counter_polygons++; // increase the counter
    }
    std::cout<<std::endl;
}
bool student_processMap::processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon> >& victim_list, Polygon& gate, const std::string& config_folder){
    cv::Mat imgHSV; // create image HSV
    cv::cvtColor(img_in, imgHSV, cv::COLOR_BGR2HSV); // convert BGR into HSV colorspace

    // low and high green thresold values
    cv::Scalar greenHSV_L(52,40,40);
    cv::Scalar greenHSV_H(72,255,255);

    #ifdef BLUE_GATE
        // low and high blue thresold values
        cv::Scalar blueHSV_L(105,40,40);
        cv::Scalar blueHSV_H(125,255,255);
    #endif

    // low and high red thresold values
    cv::Scalar redHSV_L_1(170,40,40);
    cv::Scalar redHSV_H_1(180,255,255);
    cv::Scalar redHSV_L_2(0,40,40);
    cv::Scalar redHSV_H_2(10,255,255);

    cv::Mat greenObjs;
    #ifdef BLUE_GATE
        cv::Mat blueObjs;
    #endif
    cv::Mat redObjs;

    cv::inRange(imgHSV, greenHSV_L, greenHSV_H, greenObjs); // extract green objects

    #ifdef BLUE_GATE
        cv::inRange(imgHSV, blueHSV_L, blueHSV_H, blueObjs); // extract blue objects
    #endif

    // create 2 temporary Matrices 
    cv::Mat* redObjs_1 = new cv::Mat;
    cv::Mat* redObjs_2 = new cv::Mat;

    cv::inRange(imgHSV, redHSV_L_1, redHSV_H_1, *redObjs_1); // extract first half of red objects
    cv::inRange(imgHSV, redHSV_L_2, redHSV_H_2, *redObjs_2); // extract second half of red objects
    cv::bitwise_or(*redObjs_1, *redObjs_2, redObjs); // merge the two extracted objects
    
    // delete the 2 temporary Matrices 
    delete redObjs_1;
    delete redObjs_2; 

    // display iamges if degub is enable
    #ifdef DEBUG_ACTIVE 
        cv::imshow("Green Ojbs", greenObjs);
        #ifdef BLUE_GATE
            cv::imshow("Blue Ojbs", blueObjs);
        #endif
        cv::imshow("Red Ojbs", redObjs);
        cv::waitKey();
    #endif

    std::vector<std::pair<cv::Mat, int> > templates = augmentTemplates("template");
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Point> approx_curve;

    cv::findContours(greenObjs, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (unsigned int i = 0; i < contours.size(); ++i) {
        cv::approxPolyDP(contours[i], approx_curve, 3, true);

        if (approx_curve.size() > 7) {
            int ret = detectSingleDigit(boundingRect(cv::Mat(approx_curve)),img_in ,greenObjs,templates);
            if(ret != -1){
                cv::fillConvexPoly(greenObjs, contours[i], cv::Scalar(255));
                Polygon tmp_victim;
                std::vector<cv::Point>::iterator itvp;
                for(itvp = approx_curve.begin(); itvp != approx_curve.end(); itvp++){
                    tmp_victim.emplace_back(itvp->x/scale, itvp->y/scale);
                }
                victim_list.emplace_back(ret, tmp_victim);
                #ifdef DEBUG_ACTIVE
                    std::cout<<"Found victim number: "<<ret<<std::endl;
                #endif
            }

        }
    }

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)); // create kernel
    // make erosion
    #ifdef BLUE_GATE
        cv::erode(blueObjs, blueObjs, element);
    #else
        cv::erode(greenObjs, greenObjs, element);
    #endif
    cv::erode(redObjs, redObjs, element);
    
    // display images if degub is enable
    #ifdef DEBUG_ACTIVE
        #ifdef BLUE_GATE
            cv::imshow("Blue Ojbs after erosion", blueObjs);
        #else
            cv::imshow("Green Ojbs after erosion", greenObjs);
        #endif
        cv::imshow("Red Ojbs after erosion", redObjs);
        cv::waitKey();
    #endif

    // make dilatation
    #ifdef BLUE_GATE
        cv::dilate(blueObjs, blueObjs, element);
    #else
        cv::dilate(greenObjs, greenObjs, element);
    #endif
    cv::dilate(redObjs, redObjs, element);

    // display images if degub is enable
    #ifdef DEBUG_ACTIVE
        #ifdef BLUE_GATE
            cv::imshow("Blue Ojbs after dilate", blueObjs);
        #else
            cv::imshow("Green Ojbs after dilate", greenObjs);
        #endif
        cv::imshow("Red Ojbs after dilate", redObjs);
        cv::waitKey();
    #endif

    // create vectors of polygons
    #ifdef BLUE_GATE
        std::vector<std::vector<cv::Point> > blue_polygons;
    #else
        std::vector<std::vector<cv::Point> > green_polygons; 
    #endif
    std::vector<std::vector<cv::Point> > red_polygons;

    // find contours for each color
    #ifdef BLUE_GATE
        cv::findContours(blueObjs, blue_polygons, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    #else
        cv::findContours(greenObjs, green_polygons, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    #endif
    cv::findContours(redObjs, red_polygons, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // print polygons shape statistics if debug is enable
    #ifdef DEBUG_ACTIVE
        #ifdef BLUE_GATE
        printPolygonsShapeStat(blue_polygons, std::string("Blue"));
        #else
        printPolygonsShapeStat(green_polygons, std::string("Green"));
        #endif
        printPolygonsShapeStat(red_polygons, std::string("Red"));
    #endif

    std::vector<std::vector<cv::Point> >::iterator itvvp; // create iterator
    // make the approximation on the contours
    #ifdef BLUE_GATE
        for(itvvp = blue_polygons.begin(); itvvp != blue_polygons.end(); itvvp ++)
            cv::approxPolyDP(*itvvp, *itvvp, 5, true);
    #else
        for(itvvp = green_polygons.begin(); itvvp != green_polygons.end(); itvvp ++)
            cv::approxPolyDP(*itvvp, *itvvp, 3, true);
    #endif
    for(itvvp = red_polygons.begin(); itvvp != red_polygons.end(); itvvp ++)
        cv::approxPolyDP(*itvvp, *itvvp, 5, true);

    #ifdef BLUE_GATE
        std::vector<std::vector<cv::Point>> blue_squares;
    #else
        std::vector<std::vector<cv::Point>> green_squares;
    #endif
    
    #ifdef BLUE_GATE
        for(itvvp = blue_polygons.begin(); itvvp != blue_polygons.end(); itvvp ++){
            if(itvvp->size() == 4){
                blue_squares.emplace_back(*itvvp);
            }
        }
        if(blue_squares.size() != 1){
            std::cerr<<"[ERROR] Found more than one gate"<<std::endl;
            return false;
        }
        #ifdef DEBUG_ACTIVE
            std::cout<<"Blue GATE found"<<std::endl;
        #endif
    #else
        for(itvvp = green_polygons.begin(); itvvp != green_polygons.end(); itvvp ++){
            if(itvvp->size() < 7){
                cv::approxPolyDP(*itvvp, *itvvp, 10, true);
                if(itvvp->size() == 4){
                    green_squares.emplace_back(*itvvp);
                }
            }
        }
        if(green_squares.size() != 1){
            std::cerr<<"[ERROR] Found more than one gate"<<std::endl;
            // return false;
        }
        #ifdef DEBUG_ACTIVE
            std::cout<<"Green GATE found"<<std::endl;
        #endif
    #endif

    // print polygons shape statistics if debug is enable
    #ifdef DEBUG_ACTIVE
        #ifdef BLUE_GATE
            printPolygonsShapeStat(blue_squares, std::string("Blue after approximation"));
        #else
            printPolygonsShapeStat(green_polygons, std::string("Green after approximation"));
        #endif
        printPolygonsShapeStat(red_polygons, std::string("Red after approximation"));
    #endif

    // draw contours
    #ifdef BLUE_GATE
        cv::drawContours(img_in, blue_squares, -1, cv::Scalar(0,0,0), 1, cv::LINE_AA);
    #else
        cv::drawContours(img_in, green_squares, -1, cv::Scalar(0,0,0), 1, cv::LINE_AA);
    #endif
    cv::drawContours(img_in, red_polygons, -1, cv::Scalar(0,0,0), 1, cv::LINE_AA);

    #ifdef DEBUG_ACTIVE
        // display che image
        cv::imshow("Image Contours", img_in);
        cv::waitKey();
    #endif

    for(itvvp = red_polygons.begin(); itvvp != red_polygons.end(); itvvp ++){
        std::vector<cv::Point>::iterator itvp;
        Polygon tmp_polygon;
        for(itvp = itvvp->begin(); itvp != itvvp->end(); itvp++){
            tmp_polygon.emplace_back(itvp->x/scale, itvp->y/scale);
        }
        obstacle_list.emplace_back(tmp_polygon);
    }

    #ifdef BLUE_GATE
        std::vector<cv::Point>::iterator itvp;
        for(itvp = blue_squares[0].begin(); itvp != blue_squares[0].end(); itvp++){
            gate.emplace_back(itvp->x/scale, itvp->y/scale);
        }
    #else
        std::vector<cv::Point>::iterator itvp;
        for(itvp = green_squares[0].begin(); itvp != green_squares[0].end(); itvp++){
            gate.emplace_back(itvp->x/scale, itvp->y/scale);
        }
    #endif
        
    return true;
}

#ifdef MAIN_ACTIVE
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
#endif
