cmake_minimum_required(VERSION 3.0.2)
project(spds_pallet_recognision)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
find_package(PCL REQUIRED) # This includes all modules
find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3 REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	pcl_conversions
	pcl_ros
  roscpp
  rospy
  std_msgs
)


catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs pcl_conversions pcl_ros)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pallet_recognision src/spds_pallet_recognision_node.cpp)
target_link_libraries(pallet_recognision ${catkin_LIBRARIES} ${PCL_LIBRARIES})
