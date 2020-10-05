#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <opencv2/opencv.hpp>
#include <experimental/filesystem>
#include <atomic>
#include <stdexcept>
#include <sstream>
namespace student {

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

  /****************************************************/
  /**************** EXTRINSIC CALIB *******************/
  /****************************************************/

  bool readCSV(std::vector<std::string>& string_vector, std::fstream& file, int& elements_for_line){
    std::string line;
    elements_for_line = 0;
    // Read data, line by line
    while(std::getline(file, line)){
      
      int elements_for_line_tmp = 0;
      // Create a stringstream of the current line
      std::stringstream ss(line);

      // Extract each column name
      while(std::getline(ss, line, ',')){
          elements_for_line_tmp ++;
          string_vector.push_back(line);
      }
      if(elements_for_line == 0){
        elements_for_line = elements_for_line_tmp;
      }else if(elements_for_line_tmp != elements_for_line){
        std::cout<<"Error in CSV format"<<std::endl;
        return false;
      }
    }
    return true;
  }



struct mouseCallbackUserData_t{
  std::vector<cv::Point2f>* points;
  const cv::Mat* image;
  int points_counter;
  int num_points_to_take;
  std::atomic<bool> done;
};

void CallBackFunc(int event, int x, int y, int flags, void* userdata){
  if  ( event == cv::EVENT_LBUTTONDOWN ){
    mouseCallbackUserData_t* mouseCallbackUserData = (struct mouseCallbackUserData_t*)userdata;
    mouseCallbackUserData->points->emplace_back(x,y);
    mouseCallbackUserData->points_counter++;
    if(mouseCallbackUserData->points_counter == mouseCallbackUserData->num_points_to_take){
        mouseCallbackUserData->done.store(true);
    }

    cv::circle(*(mouseCallbackUserData->image), cv::Point(x,y), 20, cv::Scalar(0,0,255), -1);
    //show the image
    cv::imshow("Image", *(mouseCallbackUserData->image));
  }
}

void selectNpoints(const cv::Mat& image, std::vector<cv::Point2f>& allPoints, int num_points_to_take){

  mouseCallbackUserData_t mouseCallbackUserData;
  mouseCallbackUserData.points = &allPoints;
  mouseCallbackUserData.num_points_to_take = num_points_to_take;
  // mouseCallbackUserData.done.store(false);
  mouseCallbackUserData.image = &image;

  //Create a window
  cv::namedWindow("Image", 1);

  //set the callback function for any mouse event
  cv::setMouseCallback("Image", CallBackFunc, &mouseCallbackUserData);

  //show the image
  cv::imshow("Image", image);

  while (!mouseCallbackUserData.done.load()) {
    cv::waitKey(50);
  }

  cv::destroyAllWindows();
}

  bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder){
    std::cout<<"[DEBUG] entered in extrinsicCalib"<<std::endl;
    // Define the configuration file name
    std::string extrinsicCalibFileName = "extrinsicCalibData.csv";
    std::string extrinsicCalibFilePath = config_folder + "/" + extrinsicCalibFileName;

    std::vector<cv::Point2f> image_points; // Define image points type
    std::cout<<"[DEBUG] try to find conf file: "<< extrinsicCalibFilePath <<std::endl;
    // Check if configuration file already exists
    if(std::experimental::filesystem::exists(extrinsicCalibFilePath)){
      std::cout<<"[DEBUG] found conf file: "<< extrinsicCalibFilePath <<std::endl;
      std::fstream confFileIn; // Define configuration file to read

      confFileIn.open(extrinsicCalibFilePath, std::ios::in);

      if(!confFileIn.is_open()) throw std::logic_error( "STUDENT FUNCTION - EXTRINSIC CALIB - CAN'T OPEN '" + extrinsicCalibFilePath + "' FILE");
      
      std::vector<std::string> allLines;
      int num_elements_for_line;

      bool ret = readCSV(allLines, confFileIn, num_elements_for_line);
      if(ret == false || num_elements_for_line != 2){
        confFileIn.close();
        throw std::logic_error( "STUDENT FUNCTION - EXTRINSIC CALIB - ERROR IN READING CSV FILE");
      }else{

        if(allLines[0].compare("x") == 0 && allLines[1].compare("y") == 0){
          std::vector<std::string>::iterator its = allLines.begin(); // Define an iterator for allLines
          its += 2; // waste the first line
          for(its; its != allLines.end(); its++){
            double x, y;
            x = std::stod(*its);
            its++;
            y = std::stod(*its);
            image_points.emplace_back(x,y); // Insert coordinates into points vector
          }
        } 
      }
      confFileIn.close();

    }else{
      std::cout<<"[DEBUG] NOT found conf file: "<< extrinsicCalibFilePath <<std::endl;
      selectNpoints(img_in, image_points, 4);
      std::experimental::filesystem::create_directories(config_folder);

      std::fstream confFileOut; // Define configuration file to read
      confFileOut.open(extrinsicCalibFilePath, std::ios::out);
      if(!confFileOut.is_open()) throw std::logic_error( "STUDENT FUNCTION - EXTRINSIC CALIB - CAN'T CREATE '" + extrinsicCalibFilePath + "' FILE");

      confFileOut << "x,y\n";

      std::vector<cv::Point2f>::iterator itp;
      for(itp = image_points.begin(); itp != image_points.end(); itp++){
        confFileOut << itp->x << "," << itp->y << "\n";
      }
      confFileOut.close();
    // }
    cv::Mat dist_coeffs = cv::Mat1d::zeros(1,4);
    bool ok = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);
    return ok;
    }
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
    throw std::logic_error( "STUDENT FUNCTION - FIND PLANE TRANSFORM - NOT IMPLEMENTED" );  
  }


void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, 
            const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION - UNWRAP - NOT IMPLEMENTED" );   
  }

  bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION - PROCESS MAP - NOT IMPLEMENTED" );   
  }

  bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION - FIND ROBOT - NOT IMPLEMENTED" );    
  }

  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, Path& path){
    throw std::logic_error( "STUDENT FUNCTION - PLAN PATH - NOT IMPLEMENTED" );     
  }


}

