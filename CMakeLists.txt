cmake_minimum_required(VERSION 3.0.2)
project(wall_following)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ackermann_msgs roscpp sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(FILES
  launch/wall_following.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(wall_following src/wall_following_cpp.cpp)

target_link_libraries(wall_following ${catkin_LIBRARIES})