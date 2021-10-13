cmake_minimum_required(VERSION 2.8.3)
project(mavsdk_catkin)

add_definitions(-std=c++17)

find_package(catkin REQUIRED)

include(ExternalProject)

ExternalProject_Add(mavsdk_src  
  GIT_REPOSITORY  https://github.com/fada-catec/piloting-mavsdk.git
  GIT_TAG v4.0.0
  UPDATE_COMMAND ""
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX}
             -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
)

add_library(${PROJECT_NAME} src/dependency_tracker.cc)
add_dependencies(${PROJECT_NAME} mavsdk_src)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS 
  DEPENDS 
  CFG_EXTRAS extras.cmake
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/mavsdk
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")

