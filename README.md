
# qrb_ros_calibrator
<update with your project name and a short description>
<Table of Contents?>

## Overview
qrb_ros_calibrator provide the calibration tools as follow:
- 2D-LiDAR and Odometry extrinsic parameter calibration tool
- 2D-LiDAR and Camera extrinsic parameter calibration tool
## Build
### Install the dependency
```bash
sudo apt-get update
sudo apt-get install libpcl-dev libeigen3-dev qt5-default ros-humble-nav2-* libceres-dev libopencv-dev qtbase5-dev qt5-qmake
```
### Build the tool
To build the 2D-LiDAR and Odometry extrinsic parameter calibration tool:
```bash
cd qrb_ros_calibrator/
cd qrb_laser_odom_calibr_lib
mkdir build
cd build
cmake ..
make
sudo make install
cd ../../
colcon build --package-select qrb_ros_laser_odom_calibrator
```
To build the 2D-LiDAR and Camera extrinsic parameter calibration tool:
```bash
cd qrb_ros_calibrator/
cd qrb_laser_cam_calibrator_lib
mkdir build
cd build
cmake ..
make
sudo make install
cd ../../
colcon build --package-select qrb_ros_laser_camera_calibrator
```
## Run

<details>
<summary>2D-LiDAR and Odometry Calibration</summary>

### Preparation
Calibration target construction: User use two boards with different length to construct two edges of a triangle.

Put the Calibration target in front of the 2D LiDAR

Generate the input file:

```bash
cd qrb_ros_calibrator/
source install/setup.bash
ros2 run qrb_ros_laser_odom_calibrator qrb_ros_inputfile_template_generator
```
Then the parameters_input.yaml file will generated in current folder.

Edit the parameters_input.yaml, and give the laser_topic name and odom topic name for data capturing.

Edit the parameters_input.yaml, and give the long_edge_length and short_edge_length that the board you use.

### Running the calibrator
```bash
ros2 run qrb_ros_laser_odom_calibrator qrb_ros_laser_odom_calibrator
```
### Data Capture
The calibrator will capture data by subscribe ROS topic.

User can capture a frame of data by click the button of "Capture Data"

Then user control the AMR/Vehicle to move and rotate then stop and capture a frame of data(Keep the calibration target in the FOV of 2D LiDAR).

The user click the button of "Capture Data" to capture one frame of data each time the AMR stops, repeating multiple times.

We recommend that users collect data more than 10 times

### Detect Features
After data capturing, user can click the button of "Detect Features" to detect the line features in the point cloud.

We draw the detected lines of the calibration target in "Line detection results" window using red corlor.

User can check the detection result, if the result is wrong, user can change the parameters by slide the sliders to get the new detected result.

### Parameters Interpretation
> **Note:**
> We use RANSAC to detect lines

max_dist_seen_as_continuous: Max distance seen as continuous in point cloud.

line_length_tolerance: Max length tolerance between detected line and target line(user input).

ransac_fitline_dist_th: The max distance threshold that taken as inner point when fitting 2d line.

### Calibration
User can check every line detection results by click "Next Frame" or "Last Frame" button.

Then click the button of Calibrate to solve the extrinsic paramters.

The rotation matrix and translation vector between 2D LiDAR frame to Odometry frame will be saved in "extrinsic.yaml" file in current folder.

</details>






<details>
<summary>2D-LiDAR and Camera Calibration</summary>

### Preparation
Calibration target: Checkerboard.

Put the Calibration target in front of the 2D LiDAR and Camera

Generate the input file:
```bash
cd qrb_ros_calibrator/
source install/setup.bash
ros2 run qrb_ros_laser_camera_calibrator qrb_ros_inputfile_template_generator
```

Then the parameters_input.yaml file will generated in current folder.

### Parameters Interpretation
> **Note:**
> We use RANSAC to detect lines

> **Note:**
> When capture first frame of data, user should put the calibration board in front of 2d-lidar meanwhile keep the axis of calibration board coordinate system parallel to the axis of laser coordinate system as much as possible for initial extrinsic guess.

image_topic_name: The ros topic name of the camera image.

laser_topic_name: The ros topic name of the 2D LiDAR scan.

relative_dist_from_laser2chessboard_origin: The distance(m) from laser plane to the origin point of calibration board coordinate system when capture first frame of data.

chessboard_length_in_laser_frame: The length of the checkerboard is scanned into line.

laser_x_wrt_chessboard: The axis of the checkerboard corresponding to the x-axis of the 2D lidar. If the direction is reversed, add "-" after the character. 

laser_y_wrt_chessboard: The axis of the checkerboard corresponding to the x-axis of the 2D lidar. If the direction is reversed, add "-" after the character.

laser_z_wrt_chessboard: The axis of the checkerboard corresponding to the x-axis of the 2D lidar. If the direction is reversed, add "-" after the character.

intrinsic: The camera intrinsic matrix

distortion: The camera distortion vector

chessborad_rows: The rows of the cornor points in the checkerboard.

chessborad_cols: The columns of the cornor points in the checkerboard.

chessboard_square_height: The heigth(mm) of the square in the checkerboard.

chessboard_square_width: The heigth(mm) of the square in the checkerboard.

left_margin_length: The length(mm) of margin to the left of the checker pattern of the checkerboard.

right_margin_length: The length(mm) of margin to the left of the checker pattern of the checkerboard.

up_margin_length: The length(mm) of margin above the checker pattern of the checkerboard.

down_margin_length: The length(mm) of margin below the checker pattern of the checkerboard.

max_dist_seen_as_continuous: Max distance seen as continuous in point cloud.

line_length_tolerance: Max length tolerance between detected line and target line(user input).

ransac_fitline_dist_th: The max distance threshold that taken as inner point when fitting 2d line.

User need to change the above parameters in parameters_input.yaml file according to the actual scenario.

### Running the calibrator
```bash
ros2 run qrb_ros_laser_camera_calibrator qrb_ros_laser_camera_calibrator
```
### Data Capture

The calibrator will capture data by subscribe ROS topic.

User can capture a frame of data by click the button of "Capture Data"

User put the calibration board in front of 2d-lidar meanwhile keep the axis of calibration board coordinate system parallel to the axis of laser coordinate system as much as possible.

Then user click the button of "Capture Data" to capture first frame of data.

Rotate and move the calibration board and capture data by pressing "Capture Data" button many times.

We recommend that users collect data more than 10 times
### Detect Features
After data capturing, user can click the button of "Detect Features" to detect the line features in the point cloud.

We draw the detected lines of the calibration target in "Line detection results" window using red corlor.

User can check the detection result, if the result is wrong, user can change the parameters by slide the sliders to get the new detected result.

### Calibration
User can check every line detection results by click "Next Frame" or "Last Frame" button.

Then click the button of Calibrate to solve the extrinsic paramters.

The rotation matrix and translation vector between 2D LiDAR frame to Camera frame will be saved in "extrinsic.yaml" file in current folder.

</details>


## Contributing

We would love to have you as a part of the QRB ROS community. Whether you are helping us fix bugs, proposing new features, improving our documentation, or spreading the word, please refer to our [contribution guidelines](./CONTRIBUTING.md) and [code of conduct](./CODE_OF_CONDUCT.md).

- Bug report: If you see an error message or encounter failures, please create a [bug report](../../issues)
- Feature Request: If you have an idea or if there is a capability that is missing and would make development easier and more robust, please submit a [feature request](../../issues)

<Update link with template>


## Authors

* **Mengwei Tao** - *Initial work* - [quic-mengtao](https://github.com/quic-mengtao)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.


## License

Project is licensed under the [BSD-3-clause License](https://spdx.org/licenses/BSD-3-Clause.html). See [LICENSE](./LICENSE) for the full license text.

