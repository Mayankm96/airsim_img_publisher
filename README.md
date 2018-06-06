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
* Added separate node to publish stereo camera images only

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

#### Building
* To build from source, clone the latest version from this repository into your catkin workspace
```
cd ~/catkin_ws/src
https://github.com/Mayankm96/airsim_img_publisher.git
```
* In order to run AirSim, you will have to change the `Airsim_ip` and `Airsim_port` parameters to match the IP/Ports in which Airsim is running. All these informations can be found in the `settings.json` file (located at `~/Documents/AirSim`) for your Airsim configuration. The ports you are looking for are the "LogViewerPort" and the "UdpPort". Note that the settings.json file have to be configured such that "LogViewerHostIp" and "UdpIp" both have the IP of the computer that will run AirSim.

* Set the correct path to `AIRSIM_ROOT` in the [`CMakeLists.txt`](CMakeLists.txt) file.

* To compile the package:
```
cd ~/catkin_ws
catkin_make
```

## Usage

Before running the nodes in the package, you need to run Airsim plugin in the Unreal Engine. In case you are unfamiliar on how to do so, refer to the tutorials available [here](https://github.com/Microsoft/AirSim#tutorials).

### Running image publisher

- Change the IP and Port configurations in [`launch/pubImages.launch`](launch/pubImages.launch)  to match the settings in which Airsim is running. Then:
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
rosrun rviz rviz -d ~/catkin_ws/src/airsim_img_publisher/rviz/octomapConfig.rviz
```

## Nodes

### airsim_imgPublisher

This is a client node at ([`imgPublisher.cpp`](src/imgPublisher.cpp)) interfaces with the AirSim plugin to retrieve the drone's pose and camera images (rgb, depth, normals and segmentation).

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

  tf tree with the origin (`world`), the position/orientation of the quadcoper (`base_frame_id`), and the position/orientation of the camera (frame selected on basis of `cameraID`)

### Parameters
* **AirSim Communication:** `Airsim_ip` (server's IP address), `Airsim_port` (server's port)
* **tf frame names:** `base_frame_id`
* **Camera parameters:** `Fx`, `Fy`, `cx`, `cz`, `width`, `height`
* **Localization Method:** `localization_method` (can be either `ground_truth` or `gps`)
* **Camera ID:** `cameraID` (camera on the drone to use (possible values: 0-4))
* **Publishing frequency:** `loop_rate`
* **Publishing tf between `camera_frame_id` and base_frame_id:** `tf_cam_flag`

__NOTE:__ In the modified blueprint of the drone for UE4, all cameras are downward-facing.

### airsim_stereoPublisher

This is a client node at ([`stereoPublisher.cpp`](src/stereoPublisher.cpp)) interfaces with the AirSim plugin to retrieve the drone's pose and stereo camera images.

#### Published Topics

* **`/airsim/rgb/image_raw`** ([sensor_msgs/Image])

	The rgb camera images on left camera frame.

* **`/airsim/rgb/image_raw`** ([sensor_msgs/Image])

	The rgb camera images on left camera frame.

* **`/airsim/rgb/image_raw`** ([sensor_msgs/Image])

	The rgb camera images on right camera frame.

* **`/airsim/depth_registered/depth`** ([sensor_msgs/Image])

	The depth camera images in 32FC1 encoding on left camera frame.

* **`/airsim/camera_info`** ([sensor_msgs/CameraInfo])

  The rgb camera paramters.

* **`/airsim/depth/camera_info`** ([sensor_msgs/CameraInfo])

  The depth camera paramters.

* **`/tf`**

  tf tree with the origin (`world`), the position/orientation of the quadcoper (`base_frame_id`), and the position/orientation of the stereo camera

### Parameters
* **AirSim Communication:** `Airsim_ip` (server's IP address), `Airsim_port` (server's port)
* **tf frame names:** `base_frame_id`
* **Camera parameters:** `Fx`, `Fy`, `cx`, `cz`, `width`, `height`
* **Depth baseline:** `Tx`
* **Localization Method:** `localization_method` (can be either `ground_truth` or `gps`)
* **Publishing frequency:** `loop_rate`
* **Publishing tf between `base_frame_id` and camera frames:** `tf_cam_flag`

[sensor_msgs/Image]: http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
[sensor_msgs/CameraInfo]: http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
