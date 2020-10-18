# Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`namespace `[`student`](#namespacestudent) | 
`struct `[`mouseCallbackUserData_t`](#structmouseCallbackUserData__t) | 

# namespace `student` 

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public void `[`loadImage`](#namespacestudent_1a3117c968a47bf95f86bdb813a3b64e56)`(cv::Mat & img_out,const std::string & config_folder)`            | This function can be used to replace the simulator camera and test the developed pipeline on a set of custom image.
`public void `[`genericImageListener`](#namespacestudent_1a3b726e7af03a643c06dcde23057a82ea)`(const cv::Mat & img_in,std::string topic,const std::string & config_folder)`            | Generic listener used from the image listener node.
`public bool `[`extrinsicCalib`](#namespacestudent_1a6103f938ce28f8820c48c089d5f95098)`(const cv::Mat & img_in,std::vector< cv::Point3f > object_points,const cv::Mat & camera_matrix,cv::Mat & rvec,cv::Mat & tvec,const std::string & config_folder)`            | Finds arena pose from 3D(object_points)-2D(image_in) point correspondences.
`public void `[`imageUndistort`](#namespacestudent_1aceb2a29362b8223a9d3601d9496e1c98)`(const cv::Mat & img_in,cv::Mat & img_out,const cv::Mat & cam_matrix,const cv::Mat & dist_coeffs,const std::string & config_folder)`            | Transforms an image to compensate for lens distortion. 
`public void `[`findPlaneTransform`](#namespacestudent_1a528d33658d0d4d982a46f18b7abb4a70)`(const cv::Mat & cam_matrix,const cv::Mat & rvec,const cv::Mat & tvec,const std::vector< cv::Point3f > & object_points_plane,const std::vector< cv::Point2f > & dest_image_points_plane,cv::Mat & plane_transf,const std::string & config_folder)`            | Calculates a perspective transform from four pairs of the corresponding points. 
`public void `[`unwarp`](#namespacestudent_1a6b8caf348979f55e58a75193233c219d)`(const cv::Mat & img_in,cv::Mat & img_out,const cv::Mat & transf,const std::string & config_folder)`            | Applies a perspective transformation to an image. 
`public bool `[`processMap`](#namespacestudent_1a153a17ef667d7c10b8f33d815b9bc1bc)`(const cv::Mat & img_in,const double scale,std::vector< Polygon > & obstacle_list,std::vector< std::pair< int, Polygon >> & victim_list,Polygon & gate,const std::string & config_folder)`            | Process the image to detect victims, obtacles and the gate 
`public bool `[`findRobot`](#namespacestudent_1afd56b779672a672e15ac45dc927b8a6b)`(const cv::Mat & img_in,const double scale,Polygon & triangle,double & x,double & y,double & theta,const std::string & config_folder)`            | Process the image to detect the robot pose 
`public bool `[`planPath`](#namespacestudent_1af1e10cfae640e646bc14763f1bc19d0b)`(const Polygon & borders,const std::vector< Polygon > & obstacle_list,const std::vector< std::pair< int, Polygon >> & victim_list,const Polygon & gate,const float x,const float y,const float theta,Path & path)`            | 

## Members

#### `public void `[`loadImage`](#namespacestudent_1a3117c968a47bf95f86bdb813a3b64e56)`(cv::Mat & img_out,const std::string & config_folder)` 

This function can be used to replace the simulator camera and test the developed pipeline on a set of custom image.

#### Parameters
* `image_out` The loaded raw image 

* `config_folder` A custom string from config file.

#### `public void `[`genericImageListener`](#namespacestudent_1a3b726e7af03a643c06dcde23057a82ea)`(const cv::Mat & img_in,std::string topic,const std::string & config_folder)` 

Generic listener used from the image listener node.

#### Parameters
* `image_in` Input image to store 

* `topic` Topic from where the image is taken 

* `config_folder` A custom string from config file.

#### `public bool `[`extrinsicCalib`](#namespacestudent_1a6103f938ce28f8820c48c089d5f95098)`(const cv::Mat & img_in,std::vector< cv::Point3f > object_points,const cv::Mat & camera_matrix,cv::Mat & rvec,cv::Mat & tvec,const std::string & config_folder)` 

Finds arena pose from 3D(object_points)-2D(image_in) point correspondences.

#### Parameters
* `image_in` Input image to store 

* `object_points` 3D position of the 4 corners of the arena, following a counterclockwise order starting from the one near the red line. 

* `camera_matrix` 3x3 floating-point camera matrix 

* `rvec` Rotation vectors estimated linking the camera and the arena 

* `tvec` Translation vectors estimated for the arena 

* `config_folder` A custom string from config file. 

#### Returns
[bool] false if there are some errors, true otherwise

#### `public void `[`imageUndistort`](#namespacestudent_1aceb2a29362b8223a9d3601d9496e1c98)`(const cv::Mat & img_in,cv::Mat & img_out,const cv::Mat & cam_matrix,const cv::Mat & dist_coeffs,const std::string & config_folder)` 

Transforms an image to compensate for lens distortion. 
#### Parameters
* `image_in` distorted image 

* `image_out` undistorted image 

* `camera_matrix` 3x3 floating-point camera matrix 

* `dist_coeffs` distortion coefficients [k1,k2,p1,p2,k3] 

* `config_folder` A custom string from config file.

#### `public void `[`findPlaneTransform`](#namespacestudent_1a528d33658d0d4d982a46f18b7abb4a70)`(const cv::Mat & cam_matrix,const cv::Mat & rvec,const cv::Mat & tvec,const std::vector< cv::Point3f > & object_points_plane,const std::vector< cv::Point2f > & dest_image_points_plane,cv::Mat & plane_transf,const std::string & config_folder)` 

Calculates a perspective transform from four pairs of the corresponding points. 
#### Parameters
* `camera_matrix` 3x3 floating-point camera matrix 

* `rvec` Rotation vectors estimated linking the camera and the arena 

* `tvec` Translation vectors estimated for the arena 

* `object_points_plane` 3D position of the 4 corners of the arena, following a counterclockwise order starting from the one near the red line. 

* `dest_image_points_plane` destinatino point in px of the object_points_plane 

* `plane_transf` plane perspective trasform (3x3 matrix) 

* `config_folder` A custom string from config file.

#### `public void `[`unwarp`](#namespacestudent_1a6b8caf348979f55e58a75193233c219d)`(const cv::Mat & img_in,cv::Mat & img_out,const cv::Mat & transf,const std::string & config_folder)` 

Applies a perspective transformation to an image. 
#### Parameters
* `image_in` input image 

* `image_out` unwarped image 

* `transf` plane perspective trasform (3x3 matrix) 

* `config_folder` A custom string from config file.

#### `public bool `[`processMap`](#namespacestudent_1a153a17ef667d7c10b8f33d815b9bc1bc)`(const cv::Mat & img_in,const double scale,std::vector< Polygon > & obstacle_list,std::vector< std::pair< int, Polygon >> & victim_list,Polygon & gate,const std::string & config_folder)` 

Process the image to detect victims, obtacles and the gate 
#### Parameters
* `image_in` input image 

* `scale` 1px/scale = X meters 

* `obstacle_list` list of obstacle polygon (vertex in meters) 

* `victim_list` list of pair victim_id and polygon (vertex in meters) 

* `gate` polygon representing the gate (vertex in meters) 

* `config_folder` A custom string from config file.

#### `public bool `[`findRobot`](#namespacestudent_1afd56b779672a672e15ac45dc927b8a6b)`(const cv::Mat & img_in,const double scale,Polygon & triangle,double & x,double & y,double & theta,const std::string & config_folder)` 

Process the image to detect the robot pose 
#### Parameters
* `image_in` input image 

* `scale` 1px/scale = X meters 

* `x` x position of the robot in the arena reference system 

* `y` y position of the robot in the arena reference system 

* `theta` yaw of the robot in the arena reference system 

* `config_folder` A custom string from config file.

#### `public bool `[`planPath`](#namespacestudent_1af1e10cfae640e646bc14763f1bc19d0b)`(const Polygon & borders,const std::vector< Polygon > & obstacle_list,const std::vector< std::pair< int, Polygon >> & victim_list,const Polygon & gate,const float x,const float y,const float theta,Path & path)` 

# struct `mouseCallbackUserData_t` 

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public std::vector< cv::Point2f > * `[`points`](#structmouseCallbackUserData__t_1a94d11bf0072afbf657a6e3e223e147e7) | 
`public const cv::Mat * `[`image`](#structmouseCallbackUserData__t_1a16b82d611db5ab9fe8bc880cde512718) | 
`public int `[`points_counter`](#structmouseCallbackUserData__t_1af5d7eed78e62edb4ad517af533d409f5) | 
`public int `[`num_points_to_take`](#structmouseCallbackUserData__t_1aa3458529c62d57165df6037d327d8a04) | 
`public std::atomic< bool > `[`done`](#structmouseCallbackUserData__t_1a8202a5674a77159537a850f2665bcd75) | 

## Members

#### `public std::vector< cv::Point2f > * `[`points`](#structmouseCallbackUserData__t_1a94d11bf0072afbf657a6e3e223e147e7) 

#### `public const cv::Mat * `[`image`](#structmouseCallbackUserData__t_1a16b82d611db5ab9fe8bc880cde512718) 

#### `public int `[`points_counter`](#structmouseCallbackUserData__t_1af5d7eed78e62edb4ad517af533d409f5) 

#### `public int `[`num_points_to_take`](#structmouseCallbackUserData__t_1aa3458529c62d57165df6037d327d8a04) 

#### `public std::atomic< bool > `[`done`](#structmouseCallbackUserData__t_1a8202a5674a77159537a850f2665bcd75) 

Generated by [Moxygen](https://sourcey.com/moxygen)