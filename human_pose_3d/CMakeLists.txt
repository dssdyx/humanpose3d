cmake_minimum_required(VERSION 3.0.2)
project(human_pose_3d)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
  geometry_msgs
  monitors
)


find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package(InferenceEngine 2.0 REQUIRED)
find_package(ngraph REQUIRED)
find_package(Eigen3 REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES human_pose_3d
  CATKIN_DEPENDS monitors
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${InferenceEngine_INCLUDE_DIRS}
)

# add_library(monitors STATIC
#   include/monitors/presenter.cpp
#   include/monitors/cpu_monitor.cpp
#   include/monitors/memory_monitor.cpp
#   include/monitors/presenter.h
#   include/monitors/cpu_monitor.h
#   include/monitors/memory_monitor.h
# )
#target_link_libraries(monitors PRIVATE opencv_core opencv_imgproc)

add_executable(${PROJECT_NAME}_node 
   src/human_pose.cpp
   src/human_pose_estimator.cpp
   src/peak.cpp
   src/render_human_pose.cpp
   src/human_pose_node.cpp
)
#target_include_directories(${PROJECT_NAME}_node PRIVATE include )
target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBRARIES} ${InferenceEngine_LIBRARIES} ${catkin_LIBRARIES} pthread)


