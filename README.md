# airsim_img_publisher

## Overview

This repository is a fork of the [publishAirsimImgs](https://github.com/marcelinomalmeidan/publishAirsimImgs) repository by [Marcelino Almeida](https://github.com/marcelinomalmeidan). It is a preliminary solution to integrate ROS and AirSim plugin till Microsoft provides an alternative one.

Features within this repo:
* Publishes rgb data into the topic `/airsim/rgb/image_raw`
* Publishes depth data into the topic `/airsim/depth`
* Publishes surface normals data into the topic `/airsim/normals/image_raw`
* Publishes segmentation labels data into the topic `/airsim/segmentation/image_raw`
* Publishes camera calibration parameters into `/airsim/depth/camera_info`
* Publishes a tf tree with the origin (`world`), the position/orientation of the quadcoper (`base_link`), and the position/orientation of the camera (`camera_frame`)

The `airsim_img_publisher` package has been tested under ROS Kinetic and Ubuntu 16.04LTS. The source code is released under [MIT Licence](LICENSE).

## Changelog

* The header files have been renamed and shifted to the `include` directory
* Fixed bugs in the code
* README modified for better understanding
* Another transform has been added from the drone's `base_frame_id` frame to `camera_frame_id`
* Changed the names of topics that are published
* Added publishing of normals and segmentation images as well
* Script added to fly the drone in lawn-mower surveillance pattern

## Installation

### Building from Source

#### Dependencies

* [__Eigen__](http://eigen.tuxfamily.org/index.php?title=Main_Page)
```
sudo apt-get install libeigen3-dev
```
* [__AirSim__](https://github.com/Microsoft/AirSim): In order to link the client-side of the plugin with this project, build the fork of the plugin available [here](https://github.com/Mayankm96/AirSim_ROS)
```
cd ~/
git clone https://github.com/Mayankm96/AirSim_ROS.git
./setup.sh
./build.sh
```  
* [__mavros__](http://wiki.ros.org/mavros): In order ti publish the `tf` tree, mavros is required to communicate with PX4 and get the pose
```
sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
```

#### Building
* To build from source, clone the latest version from this repository into your catkin workspace
```
cd ~/catkin_ws/src
https://github.com/Mayankm96/airsim_img_publisher.git
```
* In order to run mavros, you can follow the example in [`mavrosAirsim.launch`](launch/mavrosAirsim.launch). Note that you will have to change the `fcu_url` parameter to match the IP/Ports in which Airsim is running. All these informations can be found in the `settings.json` file for your Airsim configuration. The ports you are looking for are the "LogViewerPort" and the "UdpPort". Note that the settings.json file have to be configured such that "LogViewerHostIp" and "UdpIp" both have the IP of the computer that will run mavros.

* Set the correct path to `AIRSIM_ROOT` in the [`CMakeLists.txt`](CMakeLists.txt) file.

* To compile the package:
```
cd ~/catkin_ws
catkin_make
```

## Usage

Before running the nodes in the package, you need to run Airsim plugin in the Unreal Engine. In case you are unfamiliar on how to do so, refer to the tutorials available [here](https://github.com/Microsoft/AirSim#tutorials).

### Running image publisher
- Run mavros:
```
roslaunch airsim_img_publisher mavrosAirsim.launch
```
- Change the IP configuration in ```/launch/pubImages```  to match the IP in which Airsim is running. Then:
```
roslaunch airsim_img_publisher pubPointCloud.launch
```

### Create octomap
```
roslaunch airsim_img_publisher octomap.launch
```
### Visualization on rviz

An rviz configuration file can be found at [`/rviz/rvizConfig.rviz`](rviz/rvizConfig.rviz). This configuration allows a user to see the published images, as well as the tf tree.
```
rosrun rviz rviz -d ~/catkin_ws/src/airsim_img_publisher/rviz/rvizConfig.rviz
```

## Nodes

### airsim_imgPublisher ([`imgPublisher.cpp`](src/imgPublisher.cpp))

This is a client node which interfaces with the AirSim plugin to retrieve the drone's pose and camera images.

#### Published Topics

* **`/airsim/rgb/image_raw`** ([sensor_msgs/Image])

	The rgb camera images.

* **`/airsim/depth`** ([sensor_msgs/Image])

	The depth camera images in 32FC1 encoding.

* **`/airsim/normals/image_raw`** ([sensor_msgs/Image])

	The surface normals image in `bgr8` encoding.

* **`/airsim/segmentation/image_raw`** ([sensor_msgs/Image])

	The semantic segmentation labels in `bgr8` encoding.

* **`/airsim/camera_info`** ([sensor_msgs/CameraInfo])

  The rgb camera paramters.

* **`/airsim/depth/camera_info`** ([sensor_msgs/CameraInfo])

  The depth camera paramters.

* **`/tf`**
  tf tree with the origin (`world`), the position/orientation of the quadcoper (`base_link`), and the position/orientation of the camera (`camera_frame`)

### Parameters
* **AirSim Communication:** `Airsim_ip` (server's IP address), `Airsim_port` (server's port)
* **tf frame names:** `camera_frame_id`, `base_frame_id`
* **Camera parameters:** `Fx`, `Fy`, `cx`, `cz`, `width`, `height`
* **Localization Method:** `localization_method` (can be either `ground_truth` or `gps`)
* **Camera ID:** `cameraID` (camera on the drone to use (possible values: 0-4))
* **Publishing frequency:** `loop_rate`

[sensor_msgs/Image]: http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
[sensor_msgs/CameraInfo]: http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
