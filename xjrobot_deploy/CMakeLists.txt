cmake_minimum_required(VERSION 2.8.3)
project(xjrobot_deploy)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  xjrobot_interface_msg
  xjrobot_interior_msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES deploy
 CATKIN_DEPENDS roscpp rospy std_msgs
 DEPENDS system_lib
)

include_directories(
  ../platform/include
  include/deploy
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  deploy src/main.cpp src/deploy.cpp ../platform/lib/CsvProcess.cpp
)

target_link_libraries(
  deploy
  ${catkin_LIBRARIES}
)
