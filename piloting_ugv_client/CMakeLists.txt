cmake_minimum_required(VERSION 3.0.2)
project(piloting_ugv_client)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  piloting-mavsdk-ros
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS piloting-mavsdk-ros roscpp std_msgs
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
