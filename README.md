<div align="center">
  <h1>QRB ROS CALIBRATOR</h1>
  <p>ROS Packages for Sensor Extrinsic Parameters Calibration</p>
</div>

## 👋 Overview
qrb_ros_calibrator provide the calibration tools as follow:
- 2D-LiDAR and Odometry extrinsic parameter calibration tool

<div align="center">
  <img src="./docs/assets/2DLiDAR and Odometry arch.jpg" alt="architecture">
</div>

- The [`qrb_laser_odom_calibr_lib`](https://github.com/qualcomm-qrb-ros/qrb_ros_calibrator/tree/main/qrb_laser_odom_calibr_lib) is our algorithm package. It provide the data processing and extrinsic solving function to the Odometry LiDAR Calibrator APP
- The [`qrb_ros_laser_odom_calibr`](https://github.com/qualcomm-qrb-ros/qrb_ros_calibrator/tree/main/qrb_ros_laser_odom_calibr) is a ROS2 package, which provide data collection function by subscribing ros topic of sensor node and provide the UI which is created by QT for user.

- 2D-LiDAR and Camera extrinsic parameter calibration tool

<div align="center">
  <img src="./docs/assets/2DLiDAR and Camera arch.jpg" alt="architecture">
</div>

- The [`qrb_laser_cam_calibrator_lib`](https://github.com/qualcomm-qrb-ros/qrb_ros_calibrator/tree/main/qrb_laser_cam_calibrator_lib) is our algorithm package. It provide the data processing and extrinsic solving function to the Camera LiDAR Calibrator APP
- The [`qrb_ros_laser_camera_calibr`](https://github.com/qualcomm-qrb-ros/qrb_ros_calibrator/tree/main/qrb_ros_laser_camera_calibr) is a ROS2 package, which provide data collection function by subscribing ros topic of sensor node and provide the UI which is created by QT for user.

## ⚓ APIs

### 🔹 `qrb_laser_odom_calibr_lib` APIs

<table>
  <tr>
    <th>Function</th>
    <th>Parameters</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>Calibrator(std::vector<Laser_Data>& laser_data_set_in, std::vector<Odom_Data>& odom_data_set_in)</td>
    <td>
      <ul>
        <li><b>laser_data_set_in</b>: Laser data set used for calibration</li>
        <li><b>odom_data_set_in</b>: Odometry data set used for calibration</li>
      </ul>
    </td>
    <td>Get the Laser data and odometry data</td>
  </tr>
  <tr>
    <td>void get_parameters_adjust(double& max_dist_seen_as_continuous, double& line_length_tolerance, double& ransac_fitline_dist_th)</td>
    <td>
      <ul>
        <li><b>max_dist_seen_as_continuous</b>: Max distance seen as continuous</li>
        <li><b>line_length_tolerance</b>: Max length tolerance between detected line and target line</li>
        <li><b>ransac_fitline_dist_th</b>: The max distance threshold that taken as inner point when fitting 2d line</li>
      </ul>
    </td>
    <td>Get the value of the parameters that need to be adjusted</td>
  </tr>
  <tr>
    <td>void update_parameters_detect(const int& id,const double& max_dist_seen_as_continuous, const double& line_length_tolerance, const double& ransac_fitline_dist_th)</td>
    <td>
      <ul>
        <li><b>id</b>: Data frame index</li>
        <li><b>max_dist_seen_as_continuous</b>: Max distance seen as continuous</li>
        <li><b>line_length_tolerance</b>: Max length tolerance between detected line and target line</li>
        <li><b>ransac_fitline_dist_th</b>: The max distance threshold that taken as inner point when fitting 2d line</li>
      </ul>
    </td>
    <td>Update the parameters and detect the line features</td>
  </tr>
  <tr>
    <td>bool calibrate()</td>
    <td>Empty</td>
    <td>Execute calibration and get the calibration result</td>
  </tr>
</table>

### 🔹 `qrb_laser_cam_calibrator_lib` APIs


<table>
  <tr>
    <th>Function</th>
    <th>Parameters</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>initialize()</td>
    <td>Empty</td>
    <td>Initialize the calibrator</td>
  </tr>
  <tr>
    <td>set_laser_plane(const LaserPlane &amp; laser_plane)</td>
    <td>
      <ul>
        <li><b>laser_plane</b>: Captured laser plane</li>
      </ul>
    </td>
    <td>Set the laser plane for initial rotation guess</td>
  </tr>
  <tr>
    <td>set_laser_axis(const std::vector&lt;std::string&gt; &amp; laser_axis)</td>
    <td>
      <ul>
        <li><b>laser_axis</b>: The axises correspondence between laser axis and chessboard axis</li>
      </ul>
    </td>
    <td>Set the laser axis w.r.t chessboard coordinate frame for initial rotation guess</td>
  </tr>
  <tr>
    <td>find_chessboard(const cv::Mat &amp; image)</td>
    <td>
      <ul>
        <li><b>image</b>: Input image</li>
      </ul>
    </td>
    <td>Check if there is a chessboard in the image</td>
  </tr>
  <tr>
    <td>input_data(const std::vector&lt;CameraData&gt; &amp; cam_data_set_in, const std::vector&lt;LaserData&gt; &amp; laser_data_set_in)</td>
    <td>
      <ul>
        <li><b>cam_data_set_in</b>: Input camera data set</li>
        <li><b>laser_data_set_in</b>: Input laser data set</li>
      </ul>
    </td>
    <td>Give the captured data to calibrator</td>
  </tr>
  <tr>
    <td>update_parameters_detect(const int &amp; index, const double &amp; max_dist_seen_as_continuous, const double &amp; ransac_fitline_dist_th)</td>
    <td>
      <ul>
        <li><b>index</b>: The index of data set</li>
        <li><b>max_dist_seen_as_continuous</b>: Distance threshold for continuity</li>
        <li><b>ransac_fitline_dist_th</b>: RANSAC inner point distance threshold</li>
      </ul>
    </td>
    <td>Detect line in i-th laser data using specific parameters</td>
  </tr>
  <tr>
    <td>get_parameters_adjust(double &amp; max_dist_seen_as_continuous, double &amp; ransac_fitline_dist_th)</td>
    <td>
      <ul>
        <li><b>max_dist_seen_as_continuous</b>: Distance threshold for continuity</li>
        <li><b>ransac_fitline_dist_th</b>: RANSAC inner point distance threshold</li>
      </ul>
    </td>
    <td>Get the user input parameters</td>
  </tr>
  <tr>
    <td>get_topic_name(std::string &amp; laser_topic, std::string &amp; image_topic)</td>
    <td>
      <ul>
        <li><b>laser_topic</b>: Laser topic name</li>
        <li><b>image_topic</b>: Image topic name</li>
      </ul>
    </td>
    <td>Get the ROS topic name for data subscription</td>
  </tr>
  <tr>
    <td>draw_projection_result(std::vector&lt;cv::Mat&gt; &amp; project_result_img)</td>
    <td>
      <ul>
        <li><b>project_result_img</b>: Input image set</li>
      </ul>
    </td>
    <td>Project laser points into image based on calibrated result</td>
  </tr>
  <tr>
    <td>calibrate()</td>
    <td>Empty</td>
    <td>Execute calibration</td>
  </tr>
  <tr>
    <td>save_result()</td>
    <td>Empty</td>
    <td>Save the calibration result in extrinsic.xml file</td>
  </tr>
</table>


## 👨‍💻 Build
### Install the dependency for Ubuntu22.04 with ROS2 Humble
```bash
sudo apt-get update
sudo apt-get install libunwind-dev libpcl-dev libeigen3-dev ros-humble-nav2-* libceres-dev libopencv-dev qtbase5-dev libqt5svg5-dev
```
### Install the dependency for Ubuntu24.04 with ROS2 Jazzy
```bash
sudo apt-get update
sudo apt-get install libunwind-dev libpcl-dev libeigen3-dev ros-jazzy-nav2-* libceres-dev libopencv-dev qtbase5-dev libqt5svg5-dev
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
colcon build --packages-select qrb_ros_laser_odom_calibrator
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
colcon build --packages-select qrb_ros_laser_camera_calibrator
```
## 🚀 Usage

<details>
<summary>2D-LiDAR and Odometry Calibration</summary>

### Preparation
Calibration target construction: User use two boards with different length to construct two edges of a triangle. See the picture below:

<div align="center">
  <img src="./docs/assets/2DLiDAR and calibration target.jpg" alt="architecture">
</div>

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
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
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

- max_dist_seen_as_continuous: Max distance seen as continuous in point cloud.

- line_length_tolerance: Max length tolerance between detected line and target line(user input).

- ransac_fitline_dist_th: The max distance threshold that taken as inner point when fitting 2d line.

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

- image_topic_name: The ros topic name of the camera image.

- laser_topic_name: The ros topic name of the 2D LiDAR scan.

- relative_dist_from_laser2chessboard_origin: The distance(m) from laser plane to the origin point of calibration board coordinate system when capture first frame of data.

- chessboard_length_in_laser_frame: The length of the checkerboard is scanned into line.

- laser_x_wrt_chessboard: The axis of the checkerboard corresponding to the x-axis of the 2D lidar. If the direction is reversed, add "-" after the character. 

- laser_y_wrt_chessboard: The axis of the checkerboard corresponding to the x-axis of the 2D lidar. If the direction is reversed, add "-" after the character.

- laser_z_wrt_chessboard: The axis of the checkerboard corresponding to the x-axis of the 2D lidar. If the direction is reversed, add "-" after the character.

- intrinsic: The camera intrinsic matrix

- distortion: The camera distortion vector

- chessborad_rows: The rows of the cornor points in the checkerboard.

- chessborad_cols: The columns of the cornor points in the checkerboard.

- chessboard_square_height: The heigth(mm) of the square in the checkerboard.

- chessboard_square_width: The heigth(mm) of the square in the checkerboard.

- left_margin_length: The length(mm) of margin to the left of the checker pattern of the checkerboard.

- right_margin_length: The length(mm) of margin to the left of the checker pattern of the checkerboard.

- up_margin_length: The length(mm) of margin above the checker pattern of the checkerboard.

- down_margin_length: The length(mm) of margin below the checker pattern of the checkerboard.

- max_dist_seen_as_continuous: Max distance seen as continuous in point cloud.

- line_length_tolerance: Max length tolerance between detected line and target line(user input).

- ransac_fitline_dist_th: The max distance threshold that taken as inner point when fitting 2d line.

User need to change the above parameters in parameters_input.yaml file according to the actual scenario.

### Running the calibrator
```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
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


## 🤝 Contributing

We would love to have you as a part of the QRB ROS community. Whether you are helping us fix bugs, proposing new features, improving our documentation, or spreading the word, please refer to our [contribution guidelines](./CONTRIBUTING.md) and [code of conduct](./CODE_OF_CONDUCT.md).

- Bug report: If you see an error message or encounter failures, please create a [bug report](../../issues)
- Feature Request: If you have an idea or if there is a capability that is missing and would make development easier and more robust, please submit a [feature request](../../issues)

<Update link with template>


## ❤️ Contributors

* **Mengwei Tao** - *Initial work* - [quic-mengtao](https://github.com/quic-mengtao)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.


## 📜 License

Project is licensed under the [BSD-3-clause License](https://spdx.org/licenses/BSD-3-Clause.html). See [LICENSE](./LICENSE) for the full license text.

