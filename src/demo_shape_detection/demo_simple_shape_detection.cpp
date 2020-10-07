// shape_detection.cpp:
// Detect shapes on a computer-generated image

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

static const int W_0      = 300;
static const int H_0      = 0;
static const int OFFSET_W = 10;
static const int OFFSET_H = 100;


void processImage(cv::Mat& img)
{
  // Display original image
  cv::imshow("Original", img);
  cv::moveWindow("Original", W_0, H_0);

  // Convert color space from BGR to HSV
  cv::Mat hsv_img;
  cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);
  
  // Display HSV image
  cv::imshow("HSV", hsv_img);
  cv::moveWindow("HSV", W_0+img.cols+OFFSET_W, H_0);

  // Find red regions: h values around 0 (positive and negative angle: [0,15] U [160,179])
  cv::Mat red_mask_low, red_mask_high, red_mask;
  cv::inRange(hsv_img, cv::Scalar(0, 10, 10), cv::Scalar(15, 255, 255), red_mask_low);
  cv::inRange(hsv_img, cv::Scalar(160, 10, 10), cv::Scalar(179, 255, 255), red_mask_high);
  cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask); // combine together the two binary masks
  cv::imshow("RED_filter", red_mask);
  cv::moveWindow("RED_filter", W_0, H_0+img.rows+OFFSET_H);

  // Find blue regions
  cv::Mat blue_mask;
  cv::inRange(hsv_img, cv::Scalar(90, 10, 10), cv::Scalar(130, 255, 255), blue_mask);
  cv::imshow("BLUE_filter", blue_mask);
  cv::moveWindow("BLUE_filter", W_0+img.cols+OFFSET_W, H_0+img.rows+OFFSET_H);
  
  // Find green regions
  cv::Mat green_mask;
  cv::inRange(hsv_img, cv::Scalar(45, 10, 10), cv::Scalar(75, 255, 255), green_mask);
  cv::imshow("GREEN_filter", green_mask);
  cv::moveWindow("GREEN_filter", W_0+2*(img.cols+OFFSET_W), H_0);

  // Find black regions (filter on saturation and value)
  cv::Mat black_mask;
  cv::inRange(hsv_img, cv::Scalar(0, 0, 0), cv::Scalar(180, 10, 10), black_mask);  
  cv::imshow("BLACK_filter", black_mask);
  cv::moveWindow("BLACK_filter", W_0+2*(img.cols+OFFSET_W), H_0+img.rows+OFFSET_H);

  // Wait keypress
  cv::waitKey(0);

  // Find contours
  std::vector<std::vector<cv::Point>> contours, contours_approx;
  std::vector<cv::Point> approx_curve;
  cv::Mat contours_img;

  // Process black mask
  cv::findContours(black_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // find external contours of each blob
  contours_img = img.clone();
  drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
  std::cout << "N. contours: " << contours.size() << std::endl;
  for (int i=0; i<contours.size(); ++i)
  {
    std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
    approxPolyDP(contours[i], approx_curve, 3, true); // fit a closed polygon (with less vertices) to the given contour,
                                                      // with an approximation accuracy (i.e. maximum distance between 
                                                      // the original and the approximated curve) of 3
    contours_approx = {approx_curve};
    drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 5, cv::LINE_AA);
    std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
  }
  std::cout << std::endl;
  cv::imshow("Original", contours_img);
  cv::waitKey(0);

  // Process red mask
  contours_img = img.clone();
  cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
  std::cout << "N. contours: " << contours.size() << std::endl;
  for (int i=0; i<contours.size(); ++i)
  {
    std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
    approxPolyDP(contours[i], approx_curve, 3, true);
    contours_approx = {approx_curve};
    drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
    std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
  }
  std::cout << std::endl;
  cv::imshow("Original", contours_img);
  cv::waitKey(0);

  // Process blue mask
  contours_img = img.clone();
  cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
  std::cout << "N. contours: " << contours.size() << std::endl;
  for (int i=0; i<contours.size(); ++i)
  {
    std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
    approxPolyDP(contours[i], approx_curve, 3, true);
    contours_approx = {approx_curve};
    drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
    std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
  }
  std::cout << std::endl;
  cv::imshow("Original", contours_img);
  cv::waitKey(0);

}

int main( int, char** argv )
{
  /// Load an image
  cv::Mat img = cv::imread( argv[1] );

  if( !img.data ){
    printf(" Error opening image\n");
    printf(" Usage: ./demo_simple_shape_detection [image_name] \n"); 
    return -1; 
  }
    
  processImage(img);
  return 0;
}
