# Extrinsic Calibration

Functions | Descriptions                                
--------------------------------|---------------------------------------------
`bool `[`readCSV`](#extrinsicCalib_readCSV)`(std::fstream &file, std::vector<std::string> &string_vector, int &elements_for_line)`            |  This function raed a CSV file and returns a string vector with all elements and a variable 
`void `[`writePointsCSV`](#extrinsicCalib_writePointsCSV)`(std::fstream &file, std::vector<cv::Point2f> points)`            |  Writes points into CSV file 
`void `[`CallBackFunc`](#extrinsicCalib_CallBackFunc)`(int event, int x, int y, int flags, void *userdata)`            |  Call back funtion when mouse key is pressed on the image 
`void `[`selectNpoints`](#extrinsicCalib_selectNpoints)`(const cv::Mat &image, std::vector<cv::Point2f> &allPoints, int num_points_to_take)`            |  This function allow to select with mouse pointer N points inside the image

---

#### `bool `[`readCSV`](#extrinsicCalib_readCSV)`(std::fstream &file, std::vector<std::string> &string_vector, int &elements_for_line)`

This function raed a CSV file and returns a string vector with all elements and a variable that indicates how many elements there are for each line.
##### Parameters

* `file [in]`                festream file to read
* `string_vector [out]`       vector with all elements read
* `elements_for_line [out]`   number of elements for line
##### Returns
* `(bool)` True if there aren't errors, false otherwise

---

#### `void `[`writePointsCSV`](#extrinsicCalib_writePointsCSV)`(std::fstream &file, std::vector<cv::Point2f> points)`

Writes points into CSV file
##### Prameters

* `file [out]` file in which save the points
* `points [in]` vector of points

---

#### `void `[`CallBackFunc`](#extrinsicCalib_CallBackFunc)`(int event, int x, int y, int flags, void *userdata)`

Call back funtion when mouse key is pressed on the image
##### Parameters

* `event [in]` type of event
* `x [in]` x coordinate of pressed point
* `y [in]` y coordinate of pressed point
* `userdata [in/out]` user data (mouseCallbackUserData_t)

---

#### `void `[`selectNpoints`](#extrinsicCalib_selectNpoints)`(const cv::Mat &image, std::vector<cv::Point2f> &allPoints, int num_points_to_take)`
This function allows to select with mouse pointer N pints inside the image

##### Parameters
* `image [in]` image in which select points
* `allPoints [out]` vector of points 2D selected
* `num_points_to_take [in]` number of points to slect

---