cmake_minimum_required(VERSION 2.8.3)
project(xjrobot_inspection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
)

include_directories(
  ../platform/include
  include/inspection
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  inspection src/main.cpp src/inspection.cpp ../platform/lib/CsvProcess.cpp ../platform/lib/NodeManage.cpp
)

target_link_libraries(
  inspection ${catkin_LIBRARIES}
)
