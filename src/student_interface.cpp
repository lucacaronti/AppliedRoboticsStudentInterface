#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "processMap.hpp"
#include "findRobot.hpp"
#include "extrinsicCalib.hpp"
#include "sbmp.hpp"
#include "resize.hpp"

#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <sstream>
#include <map>
#include <pthread.h>
#include <mutex>


#define MISSION_1
// #define MISSION_2
// #define MISSION_2_fast

// #define THETA_GATE 0 // Rigth side
#define THETA_GATE M_PI_2 // Top side
// #define THETA_GATE M_PI // Left side
// #define THETA_GATE -M_PI_2 // Bottom side

# define Njobs 4

double computeLength(cv::Point2d currentPoint, cv::Point2d finalPoint, float theta, const std::vector<std::vector<cv::Point2d> > obstacles, Sbmp sbmp,  int val){
  std::vector<cv::Point2d> tmp_path;
  std::vector<cv::Point2d> tmp_points = {currentPoint, finalPoint};

  if(!sbmp.find_shortest_path_and_optimized(tmp_points, obstacles, tmp_path)){
    std::cout<<"Path not found, incraese the number of neighbours"<<std::endl;
    return 0;
  }

  DubinsCurve dc;
  dc.set_k(50);
  dc.add_start_data(currentPoint.x, currentPoint.y, theta);
  dc.add_final_data(finalPoint.x, finalPoint.y, THETA_GATE);
  for(auto it_m2p = tmp_path.begin() + 1; it_m2p != tmp_path.end() - 1; it_m2p++){
    dc.add_middle_points(it_m2p->x, it_m2p->y);
  }  
  double length;
  bool ret;
  tie(ret, length) = dc.solver(1,16);
  
  if(!ret){
    cout<<"Error in dubins solver"<<endl;
    return 0;
  }

  /* Print information */
  cout<<" distance to "<< val <<": "<< length <<endl;
  return length;
}

class VictimPath{
  public:
    std::vector<int> victim_order;
    double length;
    VictimPath(){};
    VictimPath(const double& _length,const std::vector<int>& _victim_order){
      length = _length;
      victim_order = _victim_order;
    }
    VictimPath(const VictimPath& vp){
      victim_order = vp.victim_order;
      length = vp.length;
    }
    friend bool operator<(const VictimPath& vp_a, const VictimPath& vp_b){
      return vp_a.length < vp_b.length;
    }
    friend bool operator>(const VictimPath& vp_a, const VictimPath& vp_b){
      return vp_a.length > vp_b.length;
    }
};

int factorial(int n){
    if(n > 1){
      return n * factorial(n-1);
    }else{
      return 1;
    }
  }

/*!
  * @brief This function finds all possible combination of a given vector of numbers
  * @param[in]  std::vector<int>& input                 Input vector fo numbers
  * @param[out] std::vector<std::vector<int> >& output  Output vector of all possible combinations of numbers
  * @return[void*] Returns the number of combinations found
  */
unsigned int findAllCombinations(std::vector<int>& input, std::vector<std::vector<int> >& output){
    unsigned int count = 0;
    do{
      for(unsigned int i = 0; i < input.size() ; i++){
        if(count % factorial(i) == 0){
          std::vector<int> tmp_vector;
          for(auto it_vn = input.begin(); it_vn != input.end() - i; it_vn++ ){
            tmp_vector.emplace_back(*it_vn);
          }
          output.emplace_back(tmp_vector);
        }
      }
      count++;
    }while (std::next_permutation(input.begin(),input.end()));
    return count;
  }

std::mutex mtx;
struct thread_args_t
{
    const Sbmp* sbmp;
    const cv::Point2d start_point;
    const cv::Point2d end_point;
    const double theta;
    const std::vector<int>* victimsCombination;
    const std::map<int, cv::Point2d>* victims_center;
    const std::vector<std::vector<cv::Point2d> >* obstacle_list_d;
    std::priority_queue<VictimPath ,std::vector<VictimPath>, std::greater<VictimPath > >* decision_list;
    thread_args_t(const Sbmp* _sbmp, const cv::Point2d _start_point, const cv::Point2d _end_point, const double _theta, const std::vector<int>* _victimsCombination, const std::map<int, cv::Point2d>* _victims_center, const std::vector<std::vector<cv::Point2d> >* _obstacle_list_d, std::priority_queue<VictimPath ,std::vector<VictimPath>, std::greater<VictimPath > >* _decision_list):
      sbmp(_sbmp), start_point(_start_point), end_point(_end_point), theta(_theta), victimsCombination(_victimsCombination), victims_center(_victims_center), obstacle_list_d(_obstacle_list_d), decision_list(_decision_list) 
    {}
};

/*!
  * @brief This thread's body finds the length of a given sequences of victims. Appends the results in decision_list parameter
  * @param[in]  void* _args   Thread argument, type thread_args_t
  * @return[void*] Returns false if there is some error, true otherwise
  */

void* fillDecisionlist_thread(void* _args){
  thread_args_t *args;
  args = (thread_args_t*)_args;

  std::vector<cv::Point2d> tmp_path;
  std::vector<cv::Point2d> tmp_points = {args->start_point};
  /* generate temporaney mission points */
  for(auto it_v = args->victimsCombination->begin(); it_v != args->victimsCombination->end(); it_v++){
    tmp_points.emplace_back(args->victims_center->at(*it_v).x ,args->victims_center->at(*it_v).y);
  }
  tmp_points.push_back(args->end_point);

  /* Find best path for mission points */
  if(!args->sbmp->find_shortest_path_and_optimized(tmp_points, *(args->obstacle_list_d), tmp_path)){
    std::cout<<"Path not found, incraese the number of neighbours"<<std::endl;
    return (void*)false;
  }
  
  /* Find path lenght with dubins curve */
  DubinsCurve dc;
  dc.set_k(50);
  dc.add_start_data(args->start_point.x, args->start_point.y, args->theta);
  dc.add_final_data(args->end_point.x, args->end_point.y, THETA_GATE);
  for(auto it_m2p = tmp_path.begin() + 1; it_m2p != tmp_path.end() - 1; it_m2p++){
    dc.add_middle_points(it_m2p->x, it_m2p->y);
  }
  double length;
  bool ret;
  tie(ret, length) = dc.solver(1,16);
  if(!ret){
    cout<<"Error in dubins solver"<<endl;
    return (void*)false;
  }

  /* Print information */
  cout<<" Analyzing victim order: ";
  for(auto it_vn = args->victimsCombination->begin(); it_vn != args->victimsCombination->end(); it_vn++){
    cout<<*it_vn<<" ";
  }
  cout<<" length = "<<length<<endl;

  /* Save found length */
  mtx.lock();
  args->decision_list->emplace(length, *(args->victimsCombination));
  mtx.unlock();

  return (void*)true;
}


 /*!
  * @brief This function finds the best combination overall possible combinations of victims. To change the victim policy change parameter delta
  * @param[in]  const Sbmp& sbmp                                              Sbmp class with the map graph
  * @param[in]  const std::vector<std::vector<int> >& allVictimsCombination   Vector of all possible victims combinations
  * @param[in]  const cv::Point2d start_point                                 Start point
  * @param[in]  const cv::Point2d end_point                                   End point
  * @param[in]  const double theta                                            Start theta
  * @param[in]  const std::map<int, cv::Point2d>& victims_center              Map with victims centers data
  * @param[in]  const std::vector<int> victim_numbers                         Vector with victims numbers
  * @param[in]  const std::vector<std::vector<cv::Point2d> >& obstacle_list_d Vector of obscales
  * @param[out] std::vector<cv::Point2d>& mission_points                      Best path with victim points, (plus start and end points)
  * @return[bool] Returns false if there is some error, true otherwise
  */

bool findBestCombinations(const Sbmp& sbmp,const std::vector<std::vector<int> >& allVictimsCombination,const cv::Point2d start_point, const cv::Point2d end_point, const double theta,const std::map<int, cv::Point2d>& victims_center, const std::vector<int> victim_numbers, const std::vector<std::vector<cv::Point2d> >& obstacle_list_d, std::vector<cv::Point2d>& mission_points){
  std::priority_queue<VictimPath ,std::vector<VictimPath>, std::greater<VictimPath > > decision_list;

  /* Find length for all possible victim combination */
  for(unsigned int it = 0; it < allVictimsCombination.size() - Njobs; it+=Njobs){
    pthread_t* threads = new pthread_t[Njobs];
    thread_args_t* thread_args[Njobs];

    /* Allocate thread arguments data, and create new threads*/
    for(unsigned int i = 0 ; i < Njobs; i++){
      thread_args[i] = new thread_args_t(&sbmp, start_point, end_point, theta, &allVictimsCombination[it+i], &victims_center, &obstacle_list_d, &decision_list);
      int ret = pthread_create(&threads[i], NULL, fillDecisionlist_thread, thread_args[i]);
      if(ret){
          std::cout<<"[ERROR] findBestCombinations unable to create thread, error :"<<ret<<std::endl;
          return false;
      }
    }
    /* Join thread */
    for(unsigned int i = 0; i < Njobs; i++){
      bool thread_ret;
      int ret = pthread_join(threads[i], (void**)&thread_ret);
      if(ret || !thread_ret){
          std::cout<<"[ERROR] findBestCombinations unable to join thread error :"<<ret<<std::endl;
          return false;
      }else{
      }
    }
    delete[] threads;
    for(unsigned int i = 0; i < Njobs; i++){
      delete thread_args[i];
    }
  }
  /* Find best combination for each number of victims (1, 2, ... up to the number of victims) */
  vector<VictimPath> best_decision_list;
  while(!decision_list.empty()){
    VictimPath vp(decision_list.top());
    if(vp.victim_order.size()-1 == best_decision_list.size()){
      best_decision_list.push_back(vp);

      /* Print information */
      cout<<"Best victims order (for N victims = "<<best_decision_list.size()<<" is:";
      for(auto it_v = vp.victim_order.begin(); it_v != vp.victim_order.end(); it_v++){
        cout<<*it_v<<" ";
      }
      cout<<" with length = "<<vp.length<<endl;
      /***/

      if(best_decision_list.size() == victim_numbers.size()){
        cout<<"Found all best decisions, one for each number of victim"<<endl;
        break;
      }
    }
    decision_list.pop();
  }

  // Decision parameter
  double delta = 0.1;

  /* Find the best path overall */
  vector<VictimPath>::iterator it_best_combination;
  for(auto it_bdl = best_decision_list.begin(); it_bdl != best_decision_list.end(); it_bdl++){
    if(it_bdl == best_decision_list.begin()){
      mission_points.clear();
      for(auto it_v = it_bdl->victim_order.begin(); it_v != it_bdl->victim_order.end(); it_v++){
        mission_points.push_back(victims_center.at(*it_v));
      }
      it_best_combination = it_bdl;
    }

    if(it_bdl+1 == best_decision_list.end()){ // Check if it is the last element
      mission_points.clear();
      for(auto it_v = it_bdl->victim_order.begin(); it_v != it_bdl->victim_order.end(); it_v++){
        mission_points.push_back(victims_center.at(*it_v));
      }
      it_best_combination = it_bdl;
    }else if(it_bdl->length * (1+delta) >= (it_bdl+1)->length){
      mission_points.clear();
      for(auto it_v = (it_bdl+1)->victim_order.begin(); it_v != (it_bdl+1)->victim_order.end(); it_v++){
        mission_points.push_back(victims_center.at(*it_v));
      }
      delta /= 2;
      it_best_combination = (it_bdl+1);
    }else{
      break;
    }
  }
  mission_points.insert(mission_points.begin(), start_point);
  mission_points.push_back(end_point);

  /* print decision */
  std::cout<<"Final decision: ";
  for(auto it_vp = it_best_combination->victim_order.begin(); it_vp != it_best_combination->victim_order.end(); it_vp++){
    std::cout<<*it_vp<<" ";
  }
  cout<<" with length = "<<it_best_combination->length<<std::endl;

  return true;
}

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
  
  /*!
  * Plan a safe and fast path in the arena
  * @param[in]  borders        border of the arena [m]
  * @param[in] obstacle_list  list of obstacle polygon [m]
  * @param[in] victim_list    list of pair victim_id and polygon [m]
  * @param[in] gate           polygon representing the gate [m]
  * @param[in] x              x position of the robot in the arena reference system
  * @param[in] y              y position of the robot in the arena reference system
  * @param[in] theta          yaw of the robot in the arena reference system
  * @param[out] path          Output path of planned path
  * @param[in]  config_folder  A custom string from config file.
  */
  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list,  const std::vector<std::pair<int,Polygon>>& victim_list,  const Polygon& gate, const float x, const float y, const float theta,  Path& path, const std::string& config_folder){
    std::cout<<"[STUDENT] : planPath"<<std::endl;

    auto resize_obstacle_list = resizeObstacles(obstacle_list,90); //robot_size=100 is better
    auto resize_borders = resizeBorders(borders,90); //robot_size=100 is better

    /* Convert obstacle type into std::vector<cv::Point2d> */
    std::vector<std::vector<cv::Point2d> > obstacle_list_d;
    for(auto it_ol = resize_obstacle_list.begin(); it_ol != resize_obstacle_list.end(); it_ol++){
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
    sbmp.sample(N_sample, double(resize_borders[0].x), double(resize_borders[1].x), double(resize_borders[2].y));
    sbmp.add_custom_point(start_point_d);
    for(auto it_vc = victims_center.begin(); it_vc != victims_center.end(); it_vc++){
      sbmp.add_custom_point(it_vc->second);
    }
    sbmp.add_custom_point(gate_center);
    sbmp.erase_sample_inside_obstacles(obstacle_list_d);
    // sbmp.plot_points();

    unsigned int num_neighbours = 4;
    sbmp.create_graph(num_neighbours, obstacle_list_d);

    /* Calculate path directly from robot to gate */
    #ifdef MISSION_0
    // std::vector<cv::Point2d> best_path;
    // if(!sbmp.find_shortest_path(start_point_d, gate_center, best_path))
    //     std::cout<<"Path not found, incraese the number of neighbours"<<std::endl;

    // sbmp.plot_paths(best_path, obstacle_list_d);
    // sbmp.best_path_optimizer(best_path, obstacle_list_d);
    // sbmp.plot_paths(best_path, obstacle_list_d);
    #endif

    /*** MISSION 1 ***/
    #ifdef MISSION_1
    std::vector<cv::Point2d> mission_1_points = {start_point_d};
    for(auto it_vc = victims_center.begin(); it_vc != victims_center.end(); it_vc++){
      mission_1_points.push_back(it_vc->second);
    }
    mission_1_points.push_back(gate_center);
    std::vector<cv::Point2d> mission_1_path;
    if(!sbmp.find_shortest_path_and_optimized(mission_1_points, obstacle_list_d, mission_1_path)){
      std::cout<<"Path not found, incraese the number of neighbours"<<std::endl;
      return false;
    }
    // sbmp.plot_paths(mission_1_path, obstacle_list_d);

    DubinsCurve dc_mission_1;
    dc_mission_1.set_k(50);
    dc_mission_1.add_start_data(mission_1_path[0].x, mission_1_path[0].y, theta);
    dc_mission_1.add_final_data(mission_1_path[mission_1_path.size()-1].x, mission_1_path[mission_1_path.size()-1].y, THETA_GATE);
    for(auto it = mission_1_path.begin() + 1; it != mission_1_path.end() - 1; it++){
        dc_mission_1.add_middle_points(it->x, it->y);
    }
    dc_mission_1.solver(2,16);
    // dc_mission_1.plot();

    path = dc_mission_1.computePath();
    cout<<"Path size: " << path.size()<<endl;
    #endif
    /*** END MISSION 1 PATH PLANNING ***/

    /*** START MISSION 2 ***/
    #ifdef MISSION_2
    std::vector<cv::Point2d> mission_2_points;
    std::vector<cv::Point2d> mission_2_path;
    std::vector<int> victim_numbers;
    for(auto it_vc = victims_center.begin(); it_vc != victims_center.end(); it_vc++){
      victim_numbers.push_back(it_vc->first);
    }
    std::vector<std::vector<int> > allVictimsCombination;

    findAllCombinations(victim_numbers, allVictimsCombination);
    
    findBestCombinations(sbmp, allVictimsCombination, start_point_d, gate_center, theta, victims_center, victim_numbers, obstacle_list_d, mission_2_points);

    
    if(!sbmp.find_shortest_path_and_optimized(mission_2_points, obstacle_list_d, mission_2_path)){
      std::cout<<"Path not found, incraese the number of neighbours"<<std::endl;
      return false;
    }
    // sbmp.plot_paths(mission_2_path, obstacle_list_d);

    DubinsCurve dc_mission_2;
    dc_mission_2.set_k(50);
    dc_mission_2.add_start_data(start_point_d.x, start_point_d.y, theta);
    dc_mission_2.add_final_data(gate_center.x, gate_center.y, M_PI_2);
    for(auto it = mission_2_path.begin() + 1; it != mission_2_path.end() - 1; it++){
        dc_mission_2.add_middle_points(it->x, it->y);
    }
    dc_mission_2.solver(2,16);
    // dc_mission_2.plot();
    path = dc_mission_2.computePath();
    #endif
    /*** END MISSION 2 ***/

    #ifdef MISSION_2_fast
    
    std::vector<cv::Point2d> mission_2_points;
    std::vector<cv::Point2d> mission_2_path;

    double best_len, len, vict_gate, cur_gate;
    cv::Point2d currentPoint;
    cv::Point2d best_point;

    mission_2_points.emplace_back(start_point_d);

    while(mission_2_points.back()!=gate_center){

      best_len = 1000;
      for(auto it_m2p = victims_center.begin(); it_m2p != victims_center.end(); it_m2p++){
        currentPoint = mission_2_points.back();

        if(find(mission_2_points.begin(), mission_2_points.end(), it_m2p->second) == mission_2_points.end()){      

          len = computeLength(currentPoint, it_m2p->second, theta, obstacle_list_d, sbmp, it_m2p->first);
          cur_gate = computeLength(currentPoint, gate_center, theta, obstacle_list_d, sbmp, 99);
          vict_gate = computeLength(it_m2p->second, gate_center, theta, obstacle_list_d, sbmp, 99);

          if(len==0 || cur_gate==0 || vict_gate==0){
            std::cout<<"Mission 2: Error in compute length"<<std::endl;
            return false;
          }

          if(len < best_len && vict_gate < cur_gate*1.1){
            best_len = len;
            best_point = it_m2p->second;
            cout << "next best: " << it_m2p->first << endl;
          }
          cout << endl;

        }
      }

      if(best_point == currentPoint){
        mission_2_points.emplace_back(gate_center);
      }
      else{
        mission_2_points.emplace_back(best_point);
      }
      cout << "-----------------------" << endl;
    }

    cout << "BEST SEQUENCE FOUND" << endl;

    if(!sbmp.find_shortest_path_and_optimized(mission_2_points, obstacle_list_d, mission_2_path)){
      std::cout<<"Path not found, incraese the number of neighbours"<<std::endl;
      return false;
    }
    // sbmp.plot_paths(mission_2_path, obstacle_list_d);

    DubinsCurve dc_mission_2;
    dc_mission_2.set_k(50);
    dc_mission_2.add_start_data(start_point_d.x, start_point_d.y, theta);
    dc_mission_2.add_final_data(gate_center.x, gate_center.y, M_PI_2);
    for(auto it = mission_2_path.begin() + 1; it != mission_2_path.end() - 1; it++){
        dc_mission_2.add_middle_points(it->x, it->y);
    }
    dc_mission_2.solver(2,16);
    // dc_mission_2.plot();
    path = dc_mission_2.computePath();
    
    #endif


    
    return true;
 
  }
}

