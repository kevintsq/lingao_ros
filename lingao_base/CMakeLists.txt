cmake_minimum_required(VERSION 2.8.3)
project(lingao_base)

##################
## Dependencies ##
##################

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#依赖包
find_package(catkin REQUIRED COMPONENTS
    roscpp 
    tf2
		tf2_ros
    lingao_msgs 
    geometry_msgs 
    sensor_msgs 
    nav_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lingao_base
  CATKIN_DEPENDS roscpp tf2 tf2_ros nav_msgs lingao_msgs geometry_msgs sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(lingao_base_driver
  src/base_driver.cpp
  src/Serial_Async.cpp
  src/data_stream.cpp
  src/TCP_Async.cpp
  src/UDP_Async.cpp
  )


# lingao base driver node
add_executable(lingao_base_node src/lingao_base_node.cpp)
add_dependencies(lingao_base_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lingao_base_node lingao_base_driver ${catkin_LIBRARIES})

