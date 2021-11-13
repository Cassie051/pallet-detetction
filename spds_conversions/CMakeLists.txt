cmake_minimum_required(VERSION 2.8.3)
project(spds_conversions)

find_package(catkin REQUIRED spds_msgs tf)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS spds_msgs tf
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
