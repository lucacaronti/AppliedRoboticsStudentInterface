// color_space_hsv.cpp
// Adapted from OpenCV sample: samples/cpp/tutorial_code/ImgProc/Threshold_inRange.cpp

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <string>

using namespace std;
using namespace cv;

/** Function Headers */
void on_low_h_thresh_trackbar(int, void *);
void on_high_h_thresh_trackbar(int, void *);
void on_low_s_thresh_trackbar(int, void *);
void on_high_s_thresh_trackbar(int, void *);
void on_low_v_thresh_trackbar(int, void *);
void on_high_v_thresh_trackbar(int, void *);

/** Global Variables */
int low_h=30, low_s=30, low_v=30;
int high_h=100, high_s=100, high_v=100;

/** @function main */
int main(int argc, char* argv[])
{
    if (argc != 2) {
        cout << "Usage: " << argv[0] << " <image>" << endl;
        return 0;
    }
    
    //! [mat]
    Mat frame, frame_threshold;
    //! [mat]
    //! [cap]
    //! [cap]
    //! [window]
    namedWindow("Original Image", WINDOW_AUTOSIZE);
    namedWindow("Filtered Image", WINDOW_AUTOSIZE);
    //! [window]
    //! [trackbar]
    //-- Trackbars to set thresholds for RGB values
    createTrackbar("Low H","Filtered Image", &low_h, 180, on_low_h_thresh_trackbar);
    createTrackbar("High H","Filtered Image", &high_h, 180, on_high_h_thresh_trackbar);
    createTrackbar("Low S","Filtered Image", &low_s, 255, on_low_s_thresh_trackbar);
    createTrackbar("High S","Filtered Image", &high_s, 255, on_high_s_thresh_trackbar);
    createTrackbar("Low V","Filtered Image", &low_v, 255, on_low_v_thresh_trackbar);
    createTrackbar("High V","Filtered Image", &high_v, 255, on_high_v_thresh_trackbar);
    
    string filename = argv[1];
    frame = imread(filename.c_str());
    if(frame.empty())
        throw runtime_error("Failed to open file " + filename);
    
    cvtColor(frame, frame, cv::COLOR_BGR2HSV);
    imshow("Original Image", frame);
    
    //! [trackbar]
    while((char)waitKey(1)!='q'){
        //! [while]
        //-- Detect the object based on HSV Range Values
        inRange(frame, Scalar(low_h,low_s,low_v), Scalar(high_h,high_s,high_v), frame_threshold);
        //! [while]
        //! [show]
        //-- Show the frames
        imshow("Filtered Image",frame_threshold);
        //! [show]
    }
    return 0;
}
//! [low]
/** @function on_low_h_thresh_trackbar */
void on_low_h_thresh_trackbar(int, void *)
{
    low_h = min(high_h-1, low_h);
    setTrackbarPos("Low H","Filtered Image", low_h);
}
//! [low]
//! [high]
/** @function on_high_h_thresh_trackbar */
void on_high_h_thresh_trackbar(int, void *)
{
    high_h = max(high_h, low_h+1);
    setTrackbarPos("High H", "Filtered Image", high_h);
}
//![high]
/** @function on_low_s_thresh_trackbar */
void on_low_s_thresh_trackbar(int, void *)
{
    low_s = min(high_s-1, low_s);
    setTrackbarPos("Low S","Filtered Image", low_s);
}

/** @function on_high_s_thresh_trackbar */
void on_high_s_thresh_trackbar(int, void *)
{
    high_s = max(high_s, low_s+1);
    setTrackbarPos("High S", "Filtered Image", high_s);
}

/** @function on_low_v_thresh_trackbar */
void on_low_v_thresh_trackbar(int, void *)
{
    low_v= min(high_v-1, low_v);
    setTrackbarPos("Low V","Filtered Image", low_v);
}

/** @function on_high_v_thresh_trackbar */
void on_high_v_thresh_trackbar(int, void *)
{
    high_v = max(high_v, low_v+1);
    setTrackbarPos("High V", "Filtered Image", high_v);
}
