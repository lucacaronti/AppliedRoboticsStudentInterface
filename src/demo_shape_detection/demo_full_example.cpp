// full_example.cpp:
// Detect shapes on a camera-captured image

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <iostream>

static const int W_0      = 300;
static const int H_0      = 0;
static const int OFFSET_W = 10;
static const int OFFSET_H = 100;

const double MIN_AREA_SIZE = 30;

void processImage()
{
  // Load image from file
  std::string filename = "imgs/img01.jpg";
  cv::Mat img = cv::imread(filename.c_str());

  if(img.empty()) {
    throw std::runtime_error("Failed to open the file " + filename);
  }

  // Display original image
  cv::imshow("Original", img);
  cv::moveWindow("Original", W_0, H_0);

  // Convert color space from BGR to HSV
  cv::Mat hsv_img;
  cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);

  // Display HSV image
  cv::imshow("HSV", hsv_img);
  cv::moveWindow("HSV", W_0+img.cols+OFFSET_W, H_0);

  // Preparing the kernel matrix
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));

  // Definining contour containers
  cv::Mat contours_img;
  std::vector<std::vector<cv::Point>> contours, contours_approx;
  std::vector<cv::Point> approx_curve;

  // Find red regions: h values around 0 (positive and negative angle: [0,15] U [160,179])
  cv::Mat red_mask_low, red_mask_high, red_mask;
  cv::inRange(hsv_img, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), red_mask_low);
  cv::inRange(hsv_img, cv::Scalar(160, 50, 50), cv::Scalar(179, 255, 255), red_mask_high);
  cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask); // combine together the two binary masks

  // Filter (applying an erosion and dilation) the image
  cv::erode(red_mask, red_mask, kernel);
  cv::dilate(red_mask, red_mask, kernel);
 
  cv::imshow("RED_filter", red_mask);
  cv::moveWindow("RED_filter", W_0, H_0+img.rows+OFFSET_H);

  // Process red mask
  contours_img = img.clone();
  cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
  std::cout << "N. contours: " << contours.size() << std::endl;
  for (int i=0; i<contours.size(); ++i)
  {
    std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
    approxPolyDP(contours[i], approx_curve, 7, true);
    contours_approx = {approx_curve};
    drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
    std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
  }
  cv::imshow("Original", contours_img);
  cv::waitKey(0);

  // Find blue regions
  cv::Mat blue_mask;
  cv::inRange(hsv_img, cv::Scalar(90, 50, 40), cv::Scalar(135, 255, 255), blue_mask);

  // Filter (applying erosion and dilation) the image
  kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2*2) + 1, (2*2)+1));
  cv::erode(blue_mask, blue_mask, kernel);
  cv::dilate(blue_mask, blue_mask, kernel);

  cv::imshow("BLUE_filter", blue_mask);
  cv::moveWindow("BLUE_filter", W_0+img.cols+OFFSET_W, H_0+img.rows+OFFSET_H);

  // Process blue mask
  contours_img = img.clone();
  cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
  std::cout << "N. contours: " << contours.size() << std::endl;
  for (int i=0; i<contours.size(); ++i)
  {
    std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
    double area = cv::contourArea(contours[i]);
    if (area < MIN_AREA_SIZE) continue; // filter too small contours to remove false positives
    approxPolyDP(contours[i], approx_curve, 7, true);
    contours_approx = {approx_curve};
    drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
    std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
  }

  cv::imshow("Original", contours_img);
  cv::waitKey(0);


  // Find black regions
  cv::Mat black_mask;
  cv::inRange(hsv_img, cv::Scalar(0, 0, 0), cv::Scalar(180, 40, 40), black_mask);
  
  // Filter (applying dilation, blurring, dilation and erosion) the image
  kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((4*2) + 1, (4*2)+1));
  cv::dilate(black_mask, black_mask, kernel);
  cv::GaussianBlur(black_mask, black_mask, cv::Size(5, 5), 2, 2);
  cv::dilate(black_mask, black_mask, kernel);
  cv::erode(black_mask, black_mask, kernel);
  
  cv::imshow("BLACK_filter", black_mask);
  cv::moveWindow("BLACK_filter", W_0+2*(img.cols+OFFSET_W), H_0);

  // Process black mask
  contours_img = img.clone();
  cv::findContours(black_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
  std::cout << "N. contours: " << contours.size() << std::endl;
  for (int i=0; i<contours.size(); ++i)
  {
    std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
    approxPolyDP(contours[i], approx_curve, 7, true);
    contours_approx = {approx_curve};
    drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
    std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
  }
  cv::imshow("Original", contours_img);
  cv::waitKey(0);

}

int main()
{
  processImage();
  return 0;
}
