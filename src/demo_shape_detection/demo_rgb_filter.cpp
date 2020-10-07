// color_space_rgb.cpp
// Adapted from OpenCV sample: samples/cpp/tutorial_code/ImgProc/Threshold_inRange.cpp

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <string>

using namespace std;
using namespace cv;

/** Function Headers */
void on_low_r_thresh_trackbar(int, void *);
void on_high_r_thresh_trackbar(int, void *);
void on_low_g_thresh_trackbar(int, void *);
void on_high_g_thresh_trackbar(int, void *);
void on_low_b_thresh_trackbar(int, void *);
void on_high_b_thresh_trackbar(int, void *);

/** Global Variables */
int low_r=30, low_g=30, low_b=30;
int high_r=100, high_g=100, high_b=100;

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
    createTrackbar("Low R","Filtered Image", &low_r, 255, on_low_r_thresh_trackbar);
    createTrackbar("High R","Filtered Image", &high_r, 255, on_high_r_thresh_trackbar);
    createTrackbar("Low G","Filtered Image", &low_g, 255, on_low_g_thresh_trackbar);
    createTrackbar("High G","Filtered Image", &high_g, 255, on_high_g_thresh_trackbar);
    createTrackbar("Low B","Filtered Image", &low_b, 255, on_low_b_thresh_trackbar);
    createTrackbar("High B","Filtered Image", &high_b, 255, on_high_b_thresh_trackbar);
    
    string filename = argv[1];
    frame = imread(filename.c_str());
    if(frame.empty())
        throw runtime_error("Failed to open file " + filename);
    imshow("Original Image", frame);
    
    //! [trackbar]
    while((char)waitKey(1)!='q'){
        //! [while]
        //-- Detect the object based on RGB Range Values
        inRange(frame, Scalar(low_b,low_g,low_r), Scalar(high_b,high_g,high_r), frame_threshold);
        //! [while]
        //! [show]
        //-- Show the frames
        imshow("Filtered Image",frame_threshold);
        //! [show]
    }
    return 0;
}
//! [low]
/** @function on_low_r_thresh_trackbar */
void on_low_r_thresh_trackbar(int, void *)
{
    low_r = min(high_r-1, low_r);
    setTrackbarPos("Low R","Filtered Image", low_r);
}
//! [low]
//! [high]
/** @function on_high_r_thresh_trackbar */
void on_high_r_thresh_trackbar(int, void *)
{
    high_r = max(high_r, low_r+1);
    setTrackbarPos("High R", "Filtered Image", high_r);
}
//![high]
/** @function on_low_g_thresh_trackbar */
void on_low_g_thresh_trackbar(int, void *)
{
    low_g = min(high_g-1, low_g);
    setTrackbarPos("Low G","Filtered Image", low_g);
}

/** @function on_high_g_thresh_trackbar */
void on_high_g_thresh_trackbar(int, void *)
{
    high_g = max(high_g, low_g+1);
    setTrackbarPos("High G", "Filtered Image", high_g);
}

/** @function on_low_b_thresh_trackbar */
void on_low_b_thresh_trackbar(int, void *)
{
    low_b= min(high_b-1, low_b);
    setTrackbarPos("Low B","Filtered Image", low_b);
}

/** @function on_high_b_thresh_trackbar */
void on_high_b_thresh_trackbar(int, void *)
{
    high_b = max(high_b, low_b+1);
    setTrackbarPos("High B", "Filtered Image", high_b);
}
