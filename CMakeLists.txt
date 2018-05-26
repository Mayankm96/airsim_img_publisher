cmake_minimum_required(VERSION 2.8.3)

project(airsim_img_publisher)

# Path to catkin workspace
# set(AIRSIM_WS "/home/mayankm/ais_valada/fount_landing/airsim_ws")
set(AIRSIM_WS "/export/airsim/fount_landing/airsim_ws")
message(STATUS " AIRSIM_WS: ${AIRSIM_WS}")

# Path to AirSim repository compiled with g++
# set(AIRSIM_ROOT "/home/mayankm/AirSim_ROS")
set(AIRSIM_ROOT "/export/airsim/AirSim_ROS")
message(STATUS " AIRSIM_ROOT: ${AIRSIM_ROOT}")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# F:DN to be added when airsim communication is desired
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF) #behzad added, need this always

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
	sensor_msgs
	nav_msgs
  image_transport
  cv_bridge
  tf
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL    REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS
		roscpp
		rospy
		std_msgs
		sensor_msgs
		nav_msgs
		image_transport
		cv_bridge
		tf
)

###########
## Build ##
###########
include_directories(
  ${AIRSIM_ROOT}/AirLib/include
  ${AIRSIM_ROOT}/AirLib/include/controllers
  # ${AIRSIM_ROOT}/AirLib/deps/MavLinkCom/include
  # ${AIRSIM_ROOT}/AirLib/deps/rpclib/include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
  #/usr/local/cuda/include/
  include
)

add_definitions(
  ${PCL_DEFINITIONS}
)

link_directories(
  ${AIRSIM_ROOT}/AirLib/lib
  ${AIRSIM_ROOT}/AirLib/deps/MavLinkCom/lib
  ${AIRSIM_ROOT}/AirLib/deps/rpclib/lib
  ${PCL_LIBRARY_DIRS}
)

# add C++ executable node airsim_imgPublisher
add_executable(airsim_imgPublisher
  src/imgPublisher.cpp
  src/airsim_img_publisher/AirSimClientWrapper.cpp
  src/airsim_img_publisher/QuatRotEuler.cpp
  src/airsim_img_publisher/TfCallback.cpp
)

target_link_libraries(airsim_imgPublisher
  ${catkin_LIBRARIES}
  AirLib
  MavLinkCom
  rpc
  pthread
  ${OpenCV_LIBS}
)
