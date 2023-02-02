
cmake_minimum_required(VERSION 2.8.3)
project(lab_palopoli)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs detection_msgs custom_msgs)
find_package(Eigen3 3.3 REQUIRED)


## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})


add_library(ur5kinematics.lib ur5kinematics.cpp)
add_executable(custom_joint_publisher custom_joint_publisher.cpp)
target_link_libraries(custom_joint_publisher ${catkin_LIBRARIES})
target_link_libraries(custom_joint_publisher ur5kinematics.lib)
install(TARGETS custom_joint_publisher ur5kinematics.lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(point point.cpp)
target_link_libraries(point ${catkin_LIBRARIES})
install(TARGETS point
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_dependencies(point ${catkin_EXPORTED_TARGETS})
