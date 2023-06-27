cmake_minimum_required(VERSION 2.8.3)
project(robotica)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs detection_msgs message_generation)
find_package(Eigen3 3.3 REQUIRED)

add_message_files(
  FILES
  Coord.msg
)

generate_messages(
  DEPENDENCIES
  detection_msgs
  std_msgs
)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})


add_library(ur5kinematics.lib src/cpp/ur5kinematics.cpp)
add_library(circlecheck.lib src/cpp/circlecheck.cpp)
add_executable(motion src/cpp/custom_joint_publisher.cpp)
target_link_libraries(motion ${catkin_LIBRARIES})
target_link_libraries(motion ur5kinematics.lib)
target_link_libraries(motion circlecheck.lib)
install(TARGETS motion ur5kinematics.lib circlecheck.lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
