#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "processMap.hpp"
#include "findRobot.hpp"
#include "extrinsicCalib.hpp"

#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <sstream>
namespace student {

  /**
   * @author Luca Caronti and Riccardo Scilla
   * @version 1.0
   * @date 06-10-2020
   * @mainpage Implementation of functions for Robot Planning course a.y. 2020-2021
  */

  /*!
  * @brief This function can be used to replace the simulator camera and test the 
  * developed pipeline on a set of custom image
  * @param[out] image_out      The loaded raw image 
  * @param[in]  config_folder  A custom string from config file.
  */
  void loadImage(cv::Mat& img_out, const std::string& config_folder){  

    std::cout<<"****************************************************"<<std::endl;
    std::cout<<"Enter the name of image to load, use relative( "<< config_folder << " ) path or absolute path: "<<std::endl;
    std::string image_path;
    std::cin >> image_path;

    if(image_path[0] != '/'){
      image_path += config_folder;
    }

    img_out = cv::imread(image_path); // load the image
    
    //if fail to read the image
    if ( img_out.empty() ) 
    { 
      throw std::logic_error( "STUDENT FUNCTION - LOAD IMAGE - UNABLE TO LOAD THE IMAGE" + image_path );
    }
  }

 int imgCounter = 0;

  /*!
  * @brief Generic listener used from the image listener node. 
  * @param[in] image_in       Input image to store
  * @param[in] topic          Topic from where the image is taken
  * @param[in] config_folder  A custom string from config file.
  */
  void genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){

  

    cv::imshow(topic, img_in); // show the image

    // wait for a key & save if 's'
    char c;
    c = cv::waitKey(30);

    if (c == 's') {
      std::string fileName = config_folder + "/img_" + topic + "_" + std::to_string(imgCounter) + ".jpg";
      cv::imwrite(fileName, img_in);
      std::cout << "Saved image " << fileName << std::endl;
      imgCounter++;
    }

    //throw std::logic_error( "STUDENT FUNCTION - IMAGE LISTENER - NOT CORRECTLY IMPLEMENTED" );
  }

  /*!
  * @brief Finds arena pose from 3D(object_points)-2D(image_in) point correspondences.
  * @param[in]  image_in       Input image to store
  * @param[in]  object_points  3D position of the 4 corners of the arena, following a counterclockwise order starting from the one near the red line.
  * @param[in]  camera_matrix  3x3 floating-point camera matrix 
  * @param[out] rvec           Rotation vectors estimated linking the camera and the arena
  * @param[out] tvec           Translation vectors estimated for the arena
  * @param[in]  config_folder  A custom string from config file.
  * @return[bool] false if there are some errors, true otherwise
  */
  bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder){
    return student_extrinsicCalib(img_in, object_points, camera_matrix, rvec, tvec, config_folder);
  }

  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
          const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){
    static bool optimize = true;

    if (!optimize) {
      // Slow version
      cv::undistort(img_in, img_out, cam_matrix, dist_coeffs);
    }
    else 
    {
      // Optimized version
      static bool init_undistort_map = false;
      static cv::Mat full_map1, full_map2;

      if (!init_undistort_map) {
        cv::Mat R;
        cv::initUndistortRectifyMap(cam_matrix, dist_coeffs, R, cam_matrix, 
                                img_in.size(), CV_16SC2, full_map1, full_map2);

        init_undistort_map = true;
      }
      // Initialize output image
      cv::remap(img_in, img_out, full_map1, full_map2, cv::INTER_LINEAR);
    }

    //throw std::logic_error( "STUDENT FUNCTION - IMAGE UNDISTORT - NOT IMPLEMENTED" );  

  }

  void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, 
                        const cv::Mat& tvec, const std::vector<cv::Point3f>& object_points_plane, 
                        const std::vector<cv::Point2f>& dest_image_points_plane, 
                        cv::Mat& plane_transf, const std::string& config_folder){
    cv::Mat image_points;

    // project points
    cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), image_points);

    plane_transf = cv::getPerspectiveTransform(image_points, dest_image_points_plane);
    //throw std::logic_error( "STUDENT FUNCTION - FIND PLANE TRANSFORM - NOT IMPLEMENTED" );  
  }

  /*!
  * Applies a perspective transformation to an image.
  * @param[in]  image_in       input image
  * @param[out] image_out      unwarped image
  * @param[in]  transf         plane perspective trasform (3x3 matrix)
  * @param[in]  config_folder  A custom string from config file.
  */
  void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, 
            const std::string& config_folder){
    cv::warpPerspective(img_in, img_out, transf, img_in.size());  
  }

  /*!
  * Process the image to detect victims, obtacles and the gate
  * @param[in]  image_in       input image
  * @param[in]  scale          1px/scale = X meters
  * @param[out] obstacle_list  list of obstacle polygon (vertex in meters)
  * @param[out] victim_list    list of pair victim_id and polygon (vertex in meters)
  * @param[out] gate           polygon representing the gate (vertex in meters)
  * @param[in]  config_folder  A custom string from config file.
  */
  bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder){
    return student_processMap::processMap(img_in, scale, obstacle_list, victim_list, gate, config_folder); // see implementation in processMap.cpp
  }

  /*!
  * Process the image to detect the robot pose
  * @param[in]  image_in       input image
  * @param[in]  scale          1px/scale = X meters
  * @param[out] x              x position of the robot in the arena reference system
  * @param[out] y              y position of the robot in the arena reference system
  * @param[out] theta          yaw of the robot in the arena reference system
  * @param[in]  config_folder  A custom string from config file.
  */
  bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION - FIND ROBOT - NOT IMPLEMENTED" );    
  }

  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, Path& path){
    throw std::logic_error( "STUDENT FUNCTION - PLAN PATH - NOT IMPLEMENTED" );     
  }


}

