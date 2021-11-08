cmake_minimum_required(VERSION 2.8.3)
project(lingao_bringup)

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
    rospy
    sensor_msgs 
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES lingao_bringup
  # CATKIN_DEPENDS roscpp tf2 tf2_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

# flat_world_imu_node node
add_executable(flat_world_imu_node src/flat_world_imu_node.cpp)
add_dependencies(flat_world_imu_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(flat_world_imu_node  ${catkin_LIBRARIES})

# flat_world_odom_node node
add_executable(flat_world_odom_node src/flat_world_odom_node.cpp)
add_dependencies(flat_world_odom_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(flat_world_odom_node  ${catkin_LIBRARIES})


