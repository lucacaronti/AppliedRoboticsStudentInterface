#include "../inc/processMap.hpp"

#define ROTATION_DEGREES  30

cv::Mat rotate(const cv::Mat src, int angle){
    // get rotation matrix for rotating the image around its center in pixel coordinates
    cv::Point2f center((src.cols-1)/2.0, (src.rows-1)/2.0);
    cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
    // determine bounding rectangle, center not relevant
    cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), src.size(), angle).boundingRect2f();
    // adjust transformation matrix
    rot.at<double>(0,2) += bbox.width/2.0 - src.cols/2.0;
    rot.at<double>(1,2) += bbox.height/2.0 - src.rows/2.0;

    cv::Mat dst;
    cv::warpAffine(src, dst, rot, bbox.size());
    return dst;
}

std::vector<std::pair<cv::Mat, int> > augmentTemplates(std::string templatesFolder){
    std::vector<std::pair<cv::Mat, int> > templates;

    bool debug = false; // choose if enable debug or not

    for (int i = 0; i < 6; i++)
    {
        std::string imageName = "../src/" + templatesFolder + "/" + std::to_string(i) + ".png";
        cv::Mat numTemplate = cv::imread(imageName);
        
        // cv::flip(numTemplate, numTemplate, 1);

        for (int j = 0; j < 360/ROTATION_DEGREES; j++)
        {
            cv::Mat numTemplate_rotated = rotate(numTemplate, ROTATION_DEGREES*j);

            if (debug){
                cv::imshow("Rotated templates",numTemplate_rotated);
                cv::waitKey(0);
            }

            templates.emplace_back(numTemplate_rotated, i);
            
        }
    }

    return templates;
}

void detectDigits(cv::Rect Rect, cv::Mat img, cv::Mat greenObjs, std::vector<std::pair<cv::Mat, int> > templates){

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2 * 2) + 1, (2 * 2) + 1)); // create kernel
    cv::dilate(greenObjs, greenObjs, element);
    cv::erode(greenObjs, greenObjs, element);

    std::vector<Polygon> green_polygons;
    cv::findContours(greenObjs, green_polygons, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat greenObjsInv, filtered(img.rows, img.cols, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::bitwise_not(greenObjs, greenObjsInv);

    //cv::imshow("inv",greenObjsInv);
    //cv::waitKey(0);

    img.copyTo(filtered, greenObjsInv);

    cv::Mat processROI(filtered, Rect); // extract ROI

    if (processROI.empty()) {
        return;
    }
    
    cv::resize(processROI, processROI, cv::Size(200, 200)); // resize the ROI
    cv::threshold( processROI, processROI, 100, 255, 0 ); // threshold and binarize the image, to suppress some noise

    cv::imshow("Process ROI",processROI);
    cv::waitKey(0);



    double maxScore = 0;
    int num = -1;
    cv::Mat temp;
    for (int i = 0; i < templates.size(); ++i) {


        // Match the ROI with the templates
        cv::Mat result;

        cv::matchTemplate(processROI, templates[i].first, result, cv::TM_CCOEFF);
        double score;
        cv::minMaxLoc(result, nullptr, &score); 

        if (score > maxScore) {
            maxScore = score;
            num = templates[i].second;
            temp = templates[i].first;
        }
    }

    std::string title = "Best fitting template: "+std::to_string(num);

    cv::imshow(title,temp);
    cv::waitKey(0);
    

}

int main(int argc, char* argv[]){
    if(argc != 2){
        std::cout<<"[ERROR] Argument error, usage: ./detectDigits image_name.jpg"<<std::endl;
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

    std::vector<std::pair<cv::Mat, int> > templates = augmentTemplates("template");

    //convert hsv
    cv::Mat imgHSV;
    cv::cvtColor(image, imgHSV, cv::COLOR_BGR2HSV);

    // low and high green thresold values
    cv::Scalar greenHSV_L(52,40,40);
    cv::Scalar greenHSV_H(72,255,255);

    //get green objects
    cv::Mat greenObjs;
    cv::inRange(imgHSV, greenHSV_L, greenHSV_H, greenObjs); // extract green objects

    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;

    cv::findContours(greenObjs, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours.size(); ++i) {
        approxPolyDP(contours[i], approx_curve, 3, true);

        if (approx_curve.size() > 10) {
            detectDigits(boundingRect(cv::Mat(approx_curve)),image,greenObjs,templates);
        }
    }

    
}