#include "extrinsicCalib.hpp"

#define DEBUG_ACTIVE



struct mouseCallbackUserData_t{
    std::vector<cv::Point2f> *points;
    const cv::Mat *image;
    int points_counter;
    int num_points_to_take;
    std::atomic<bool> done;
};

/*!
  * \brief This function raed a CSV file and returns a string vector with all elements and a variable 
  *     that indicates how many elements there are for each line.
  * \param[in]  file                festream file to read
  * \param[out] string_vector       vector with all elements read
  * \param[out] elements_for_line   number of elements for line
  * \return (bool) -> True if there aren't errors, false otherwise
  */
bool readCSV(std::fstream &file, std::vector<std::string> &string_vector, int &elements_for_line){
    std::string line;
    elements_for_line = 0;

    // Read data, line by line
    while (std::getline(file, line)){

        int elements_for_line_tmp = 0;
        // Create a stringstream of the current line
        std::stringstream ss(line);

        // Extract each column
        while (std::getline(ss, line, ',')){
            elements_for_line_tmp++;
            string_vector.push_back(line);
        }
        if (elements_for_line == 0){
            elements_for_line = elements_for_line_tmp;
        }
        // Check if there are error in CSV format
        else if (elements_for_line_tmp != elements_for_line){
            std::cout << "Error in CSV format" << std::endl;
            return false;
        }
    }
    return true;
}

/*!
  * \brief Write points into CSV file
  * \param[in]     file        file in which save the points
  * \param[in]     points      vector of points
  */
void writePointsCSV(std::fstream &file, std::vector<cv::Point2f> points){
    file << "x,y\n"; // save x and y as column arguments
    std::vector<cv::Point2f>::iterator itp; // create an iterator
    for (itp = points.begin(); itp != points.end(); itp++)
    {
        file << itp->x << "," << itp->y << "\n"; // save points
    }
}

/*!
  * \brief Call back funtion when mouse key is pressed on the image
  * \param[in]     event        type of event
  * \param[in]     x            x coordinate of pressed point
  * \param[in]     y            y coordinate of pressed point
  * \param[in/out] userdata     user data (mouseCallbackUserData_t)
  */
void CallBackFunc(int event, int x, int y, int flags, void *userdata){
    // Check if event is corresponding of left button pressed
    if (event == cv::EVENT_LBUTTONDOWN){
        mouseCallbackUserData_t *mouseCallbackUserData = (struct mouseCallbackUserData_t *)userdata;
        mouseCallbackUserData->points->emplace_back(x, y); // save coordinates
        mouseCallbackUserData->points_counter++; // increase counter
        // Draw a circle on pressed image
        cv::circle(*(mouseCallbackUserData->image), cv::Point(x, y), 20, cv::Scalar(0, 255, 255), -1);
        //show the image
        cv::imshow("Image", *(mouseCallbackUserData->image));

        // Check is all points have been selected
        if (mouseCallbackUserData->points_counter >= mouseCallbackUserData->num_points_to_take){
            mouseCallbackUserData->done.store(true); // Store exit condition
        }
    }
}

/*!
  * \brief This function allow to select with mouse pointer N pints inside the image
  * \param[in]  image               image in which select points
  * \param[out] allPoints           vector of points 2D selected
  * \param[in]  num_points_to_take  number of points to slect
  */
void selectNpoints(const cv::Mat &image, std::vector<cv::Point2f> &allPoints, int num_points_to_take){

    // Initialize mouseCallbackUserData
    mouseCallbackUserData_t mouseCallbackUserData;
    mouseCallbackUserData.points = &allPoints;
    mouseCallbackUserData.num_points_to_take = num_points_to_take;
    mouseCallbackUserData.points_counter = 0;
    mouseCallbackUserData.done.store(false);
    mouseCallbackUserData.image = &image;

    //Create a window
    cv::namedWindow("Image", 1);

    //show the image
    cv::imshow("Image", image);

    //set the callback function for any mouse event
    cv::setMouseCallback("Image", CallBackFunc, &mouseCallbackUserData);

    //Wait until all points are selected
    while (!mouseCallbackUserData.done.load()){
        cv::waitKey(50);
    }
}

bool student_extrinsicCalib(const cv::Mat &img_in, std::vector<cv::Point3f> object_points, const cv::Mat &camera_matrix, cv::Mat &rvec, cv::Mat &tvec, const std::string &config_folder){
    
    #ifdef DEBUG_ACTIVE 
    std::cout << "[DEBUG] entered in extrinsicCalib" << std::endl;
    #endif

    // Define the configuration file name
    std::string extrinsicCalibFileName = "extrinsicCalibData.csv";
    std::string extrinsicCalibFilePath = config_folder + "/" + extrinsicCalibFileName;

    // Define image points type
    std::vector<cv::Point2f> image_points;

    #ifdef DEBUG_ACTIVE
    std::cout << "[DEBUG] try to find conf file: " << extrinsicCalibFilePath << std::endl;
    #endif

    // Check if configuration file already exists
    if (std::experimental::filesystem::exists(extrinsicCalibFilePath)){
        #ifdef DEBUG_ACTIVE
        std::cout << "[DEBUG] found conf file: " << extrinsicCalibFilePath << std::endl;
        #endif

        // Define configuration file to read
        std::fstream confFileIn; 

        // Open file
        confFileIn.open(extrinsicCalibFilePath, std::ios::in);

        if (!confFileIn.is_open())
            throw std::logic_error("STUDENT FUNCTION - EXTRINSIC CALIB - CAN'T OPEN '" + extrinsicCalibFilePath + "' FILE");

        // Define vector os strings that will contain all CSV lines 
        std::vector<std::string> allLines;
        int num_elements_for_line;

        // Read CSV
        bool ret = readCSV(confFileIn, allLines, num_elements_for_line);

        // Check if there are errors
        if (ret == false || num_elements_for_line != 2){
            confFileIn.close();
            throw std::logic_error("STUDENT FUNCTION - EXTRINSIC CALIB - ERROR IN READING CSV FILE");
        }
        else{
            // Check if first column is 'x' and second column is 'y'
            if (allLines[0].compare("x") == 0 && allLines[1].compare("y") == 0){
                std::vector<std::string>::iterator its = allLines.begin(); // Define an iterator for allLines
                its += 2; // waste the first line                               
                for (its; its != allLines.end(); its++){
                    double x, y;
                    x = std::stod(*its);
                    its++;
                    y = std::stod(*its);
                    image_points.emplace_back(x, y); // Insert coordinates into points vector
                }
            }
        }
        confFileIn.close();
    }
    else{
        #ifdef DEBUG_ACTIVE
        std::cout << "[DEBUG] NOT found conf file: " << extrinsicCalibFilePath << std::endl;
        #endif

        //Select N points from image
        selectNpoints(img_in, image_points, 4);

        std::fstream confFileOut; // Define configuration file to read
        //Open CSV file
        confFileOut.open(extrinsicCalibFilePath, std::ios::out);
        if (!confFileOut.is_open())
            throw std::logic_error("STUDENT FUNCTION - EXTRINSIC CALIB - CAN'T CREATE '" + extrinsicCalibFilePath + "' FILE");

        //Write CSV
        writePointsCSV(confFileOut, image_points);

        //Close CSV file
        confFileOut.close();

    }

    cv::Mat dist_coeffs = cv::Mat1d::zeros(1, 4);
    bool ok = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);
    return ok;
}