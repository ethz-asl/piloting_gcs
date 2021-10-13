project(piloting_ugv_client)
cmake_minimum_required(VERSION 3.5.1)

#####################
## Project Options ##
#####################

## Force some variables that could be defined in the command line to be written to cache
option(WARNINGS_ARE_ERRORS "Treat warnings as errors"                                  OFF)
option(WARNINGS_ANSI_ISO   "Issue all the mandatory diagnostics listed in C standard"  ON)
option(WARNINGS_EFFCPP     "Issue all the warnings listed in the book of Scot Meyers"  OFF)

add_compile_options(-std=c++17)
add_compile_options(-Wall)

if(${WARNINGS_ANSI_ISO})
   add_compile_options(-Wcast-align)
   add_compile_options(-Wpedantic)
   add_compile_options(-Wextra)
else()
   add_compile_options(-Wno-narrowing)
   add_compile_options(-Wno-delete-non-virtual-dtor)
   add_compile_options(-Wno-unnamed-type-template-args)
endif()

if(${WARNINGS_ARE_ERRORS})
   add_compile_options(-Werror)
endif()

if(${WARNINGS_EFFCPP})
   add_compile_options(-Weffc++)
endif()

find_package(catkin REQUIRED 
   COMPONENTS
      roscpp
      std_msgs
      geometry_msgs
      message_filters
      message_generation
      mavsdk_catkin
)

add_message_files(DIRECTORY msg)
add_service_files(DIRECTORY srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
   INCLUDE_DIRS   include
   LIBRARIES      ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_filters message_generation mavsdk_catkin)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${MAVSDK_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
  src/client_node.cpp
  src/client_services.cpp
  src/client_subscribers.cpp
  src/parameters.cpp)
  add_dependencies(${PROJECT_NAME}  ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${MAVSDK_LIBRARIES})

add_executable(client_node src/main.cpp)
target_link_libraries(client_node ${PROJECT_NAME})


# Example GCS to run the alarm client
add_executable(gcs_alarm src/gcs/gcs_alarm.cpp)
target_link_libraries(gcs_alarm ${catkin_LIBRARIES})
