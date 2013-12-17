cmake_minimum_required(VERSION 2.8.3)

project(nxt_ros)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package()

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
