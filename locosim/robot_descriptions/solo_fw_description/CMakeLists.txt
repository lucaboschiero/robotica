######################
# set up the project #
######################
cmake_minimum_required(VERSION 2.8.3)
project(solo_fw_description)


# required to use std::shared_ptr in code AND to link the DG plugins
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")


############################
# Search for dependencies. #
############################

# find dependencies using catkin

find_package(catkin REQUIRED solo_description
  COMPONENTS
)
catkin_package(
  CATKIN_DEPENDS
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)



install(DIRECTORY robots/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/robots
  PATTERN ".svn" EXCLUDE)



install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY urdfs/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdfs
  PATTERN ".svn" EXCLUDE)



catkin_install_python(PROGRAMS scripts/go0
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
