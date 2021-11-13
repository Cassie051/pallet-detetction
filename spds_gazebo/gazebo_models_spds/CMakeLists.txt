cmake_minimum_required(VERSION 2.8.3)
project(gazebo_models_spds)

find_package(catkin REQUIRED)

catkin_package(
)

##################################
## Update the GAZEBO_MODEL_PATH ##
##################################

if(CMAKE_HOST_UNIX)
  catkin_add_env_hooks(90.gazebo_models_spds SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
else()
  message(FATAL_ERROR "Only UNIX builds supported.")
#  catkin_add_env_hooks(90.gazebo_models_spds SHELLS bat DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
endif()

#############
## Install ##
#############

install(DIRECTORY
  models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/delete_humans_from_gazebo.py
  scripts/download_all_gazebo_org_models.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
