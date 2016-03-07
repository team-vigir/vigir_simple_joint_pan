cmake_minimum_required(VERSION 2.8.3)
project(vigir_simple_joint_pan)

find_package(catkin REQUIRED COMPONENTS
  rospy
  trajectory_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vigir_simple_joint_pan
#  CATKIN_DEPENDS rospy trajectory_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

FILE(GLOB BIN_FILES "bin/*")

# install executables
install(PROGRAMS ${BIN_FILES} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})