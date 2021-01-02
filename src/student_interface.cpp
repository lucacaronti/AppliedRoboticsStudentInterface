#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "processMap.hpp"
#include "findRobot.hpp"
#include "extrinsicCalib.hpp"
#include "sbmp.hpp"

#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <sstream>
#include <map>
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
    std::cout<<"[STUDENT] : loadImage"<<std::endl;
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
    std::cout<<"[STUDENT] : genericImageListener"<<std::endl;
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
    std::cout<<"[STUDENT] : extrinsicCalib"<<std::endl;
    return student_extrinsicCalib(img_in, object_points, camera_matrix, rvec, tvec, config_folder);
  }

  /*!
  * Transforms an image to compensate for lens distortion.
  * @param[in]  image_in       distorted image
  * @param[out] image_out      undistorted image
  * @param[in]  camera_matrix  3x3 floating-point camera matrix 
  * @param[out] dist_coeffs    distortion coefficients [k1,k2,p1,p2,k3]
  * @param[in]  config_folder  A custom string from config file.
  */
  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
          const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){
    std::cout<<"[STUDENT] : imageUndistort"<<std::endl;
    static bool optimize = true;

    if (!optimize) {
      // Slow version
      cv::undistort(img_in, img_out, cam_matrix, dist_coeffs);
    }
    else{
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

  }


  /*!
  * Calculates a perspective transform from four pairs of the corresponding points.
  * @param[in]  camera_matrix  3x3 floating-point camera matrix 
  * @param[in]  rvec           Rotation vectors estimated linking the camera and the arena
  * @param[in]  tvec           Translation vectors estimated for the arena
  * @param[in]  object_points_plane  3D position of the 4 corners of the arena, following a counterclockwise order starting from the one near the red line.  
  * @param[in ] dest_image_points_plane   destinatino point in px of the object_points_plane
  * @param[out] plane_transf   plane perspective trasform (3x3 matrix)
  * @param[in]  config_folder  A custom string from config file.
  */
  void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, 
                        const cv::Mat& tvec, const std::vector<cv::Point3f>& object_points_plane, 
                        const std::vector<cv::Point2f>& dest_image_points_plane, 
                        cv::Mat& plane_transf, const std::string& config_folder){
    std::cout<<"[STUDENT] : findPlaneTranform"<<std::endl;
    cv::Mat image_points;

    // project points
    cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), image_points);

    plane_transf = cv::getPerspectiveTransform(image_points, dest_image_points_plane);
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
    std::cout<<"[STUDENT] : unwarp"<<std::endl;
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
    std::cout<<"[STUDENT] : processMap"<<std::endl;
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
    std::cout<<"[STUDENT] : findRobot"<<std::endl;
    return student_findRobot(img_in, scale, triangle, x, y, theta, config_folder);
  }

  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list,  const std::vector<std::pair<int,Polygon>>& victim_list,  const Polygon& gate, const float x, const float y, const float theta,  Path& path, const std::string& config_folder){
    std::cout<<"[STUDENT] : planPath"<<std::endl;
    
    /* Convert obstacle type into std::vector<cv::Point2d> */
    std::vector<std::vector<cv::Point2d> > obstacle_list_d;
    for(auto it_ol = obstacle_list.begin(); it_ol != obstacle_list.end(); it_ol++){
      std::vector<cv::Point2d> tmp_obstacle;
      for(auto it_pol = it_ol->begin(); it_pol != it_ol->end(); it_pol++){
        tmp_obstacle.emplace_back(cv::Point2d(double(it_pol->x), double(it_pol->y)));
      }
      obstacle_list_d.push_back(tmp_obstacle);
    }

    /* Convert gate type into std::vector<cv::Point2f> */
    std::vector<cv::Point2f> gate_f;
    for(auto it_pol = gate.begin(); it_pol != gate.end(); it_pol++){
      gate_f.emplace_back(cv::Point2f(it_pol->x, it_pol->y));
    }

    /* Convert victim type into std::vector<std::pair<int,std::vector<cv::Point2d> > >, and calculate their center*/
    std::vector<std::pair<int,std::vector<cv::Point2d> > > victim_list_d;
    std::map<int, cv::Point2d> victims_center;
    for(auto it_vl = victim_list.begin(); it_vl != victim_list.end(); it_vl++){
      std::vector<cv::Point2d> tmp_polygon_d;
      std::vector<cv::Point2f> tmp_polygon_f;
      for(auto it_pol = it_vl->second.begin(); it_pol != it_vl->second.end(); it_pol++){
        tmp_polygon_d.emplace_back(cv::Point2d(double(it_pol->x), double(it_pol->y)));
        tmp_polygon_f.emplace_back(cv::Point2f(float(it_pol->x), float(it_pol->y)));
      }
      victim_list_d.emplace_back(std::pair<int,std::vector<cv::Point2d> >(it_vl->first, tmp_polygon_d));
      cv::Moments m_victim = cv::moments(tmp_polygon_f, false);
      cv::Point2d victim_center = Point2d(m_victim.m10/m_victim.m00 , m_victim.m01/m_victim.m00);
      victims_center.insert(std::pair<int, cv::Point2d>(it_vl->first, victim_center));

    }

    /* Calculate gate center */
    cv::Moments m_gate = cv::moments(gate_f, false);
    cv::Point2d gate_center = Point2d(m_gate.m10/m_gate.m00 , m_gate.m01/m_gate.m00);
  
    cv::Point2d start_point_d = Point2d(double(x),double(y));

    Sbmp sbmp;
    unsigned int N_sample = 5000;
    sbmp.sample(N_sample, double(borders[1].x), double(borders[2].y));
    sbmp.add_custom_point(start_point_d);
    sbmp.add_custom_point(victims_center[1]);
    sbmp.add_custom_point(victims_center[3]);
    sbmp.add_custom_point(victims_center[4]);
    sbmp.add_custom_point(victims_center[5]);
    sbmp.add_custom_point(gate_center);
    // sbmp.plot_points();
    sbmp.erase_sample_inside_obstacles(obstacle_list_d);

    unsigned int num_neighbours = 4;
    sbmp.create_graph(num_neighbours, obstacle_list_d);

    /* Calculate path directly from robot to gate */
    std::vector<cv::Point2d> best_path;
    if(!sbmp.find_shortest_path(start_point_d, gate_center, best_path))
        std::cout<<"Path not found, incraese the number of neighbours"<<std::endl;

    sbmp.plot_paths(best_path, obstacle_list_d);
    sbmp.best_path_optimizer(best_path, obstacle_list_d);
    sbmp.plot_paths(best_path, obstacle_list_d);

    // DubinsCurve dc;
    // dc.set_k(50);
    // std::cout<<theta<<std::endl;
    // dc.add_start_data(best_path[0].x, best_path[0].y, theta);
    // dc.add_final_data(best_path[best_path.size()-1].x, best_path[best_path.size()-1].y, M_PI_2);
    // for(auto it = best_path.begin() + 1; it != best_path.end() - 1; it++){
    //     dc.add_middle_points(it->x, it->y);
    // }
    // dc.solver(3,16);
    // dc.plot();

    std::vector<cv::Point2d> mission_1_points = {start_point_d, victims_center[1], victims_center[3], victims_center[4], victims_center[5], gate_center};
    std::vector<cv::Point2d> mission_1_path;
    for(auto it_m1p = mission_1_points.begin(); it_m1p != mission_1_points.end() - 1; it_m1p++){
      std::vector<cv::Point2d> tmp_mission_1_path;
      if(!sbmp.find_shortest_path(*it_m1p, *(it_m1p+1), tmp_mission_1_path)){
        std::cout<<"Path not found, incraese the number of neighbours"<<std::endl;
      }
      sbmp.best_path_optimizer(tmp_mission_1_path, obstacle_list_d);
      if(mission_1_path.size() != 0 && (mission_1_path[mission_1_path.size()-1] == tmp_mission_1_path[0])){
        mission_1_path.insert(mission_1_path.end(), tmp_mission_1_path.begin()+1, tmp_mission_1_path.end());
      }else{
        mission_1_path.insert(mission_1_path.end(), tmp_mission_1_path.begin(), tmp_mission_1_path.end());
      }
    }
    sbmp.plot_paths(mission_1_path, obstacle_list_d);

    for(auto it = mission_1_path.begin(); it != mission_1_path.end(); it++){
      cout<<it->x<<","<<it->y<<endl;
    }
    std::cout<<mission_1_path.size()<<std::endl;

    DubinsCurve dc_mission_1;
    dc_mission_1.set_k(50);
    std::cout<<theta<<std::endl;
    dc_mission_1.add_start_data(mission_1_path[0].x, mission_1_path[0].y, theta);
    dc_mission_1.add_final_data(mission_1_path[mission_1_path.size()-1].x, mission_1_path[mission_1_path.size()-1].y, M_PI_2);
    for(auto it = mission_1_path.begin() + 1; it != mission_1_path.end() - 1; it++){
        dc_mission_1.add_middle_points(it->x, it->y);
    }
    dc_mission_1.solver(3,16);
    dc_mission_1.plot();

    // /* Calculate path considering the victims */
    // // Victim 1
    // best_path.clear();
    // std::cout<<start_point_d.x<<","<<start_point_d.y<<" "<<victims_center[1].x<<","<<victims_center[1].y<<std::endl;
    // if(!sbmp.find_shortest_path(start_point_d, victims_center[1], best_path))
    //     std::cout<<"Path not found, incraese the number of neighbours"<<std::endl;
    // // sbmp.best_path_optimizer(best_path, obstacle_list_d);
    // sbmp.plot_paths(best_path, obstacle_list_d);

    // //Victim 3
    // std::cout<<victims_center[1].x<<","<<victims_center[1].y<<" "<<victims_center[3].x<<","<<victims_center[3].y<<std::endl;
    // best_path.clear();
    // if(!sbmp.find_shortest_path(victims_center[1], victims_center[3], best_path))
    //     std::cout<<"Path not found, incraese the number of neighbours"<<std::endl;
    // // sbmp.best_path_optimizer(best_path, obstacle_list_d);
    // sbmp.plot_paths(best_path, obstacle_list_d);

    // //Victim 4
    // std::cout<<victims_center[3].x<<","<<victims_center[3].y<<" "<<victims_center[4].x<<","<<victims_center[4].y<<std::endl;
    // best_path.clear();
    // if(!sbmp.find_shortest_path(victims_center[3], victims_center[4], best_path))
    //     std::cout<<"Path not found, incraese the number of neighbours"<<std::endl;
    // // sbmp.best_path_optimizer(best_path, obstacle_list_d);
    // sbmp.plot_paths(best_path, obstacle_list_d);

    // //Victim 5
    // std::cout<<victims_center[4].x<<","<<victims_center[4].y<<" "<<victims_center[5].x<<","<<victims_center[5].y<<std::endl;
    // best_path.clear();
    // if(!sbmp.find_shortest_path(victims_center[4], victims_center[5], best_path))
    //     std::cout<<"Path not found, incraese the number of neighbours"<<std::endl;
    // // sbmp.best_path_optimizer(best_path, obstacle_list_d);
    // sbmp.plot_paths(best_path, obstacle_list_d);

    // //Gate
    // std::cout<<victims_center[5].x<<","<<victims_center[5].y<<" "<<gate_center.x<<","<<gate_center.y<<std::endl;
    // best_path.clear();
    // if(!sbmp.find_shortest_path(victims_center[5], gate_center, best_path))
    //     std::cout<<"Path not found, incraese the number of neighbours"<<std::endl;
    // // sbmp.best_path_optimizer(best_path, obstacle_list_d);
    // sbmp.plot_paths(best_path, obstacle_list_d);


   

    throw std::logic_error( "STUDENT FUNCTION - PLAN PATH - NOT IMPLEMENTED" );     
  }
}

