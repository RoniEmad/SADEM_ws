cmake_minimum_required(VERSION 3.0.2)
project(autonomous_sadem)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES
  autonomous_sadem
  CATKIN_DEPENDS
  geometry_msgs
  rospy
  std_msgs
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

catkin_python_setup()

catkin_install_python(PROGRAMS
  src/autonomous_sadem/control.py
  #src/autonomous_sadem/helpers.py
  src/autonomous_sadem/joystick_ubuntu.py
  src/autonomous_sadem/joystick.py
  #src/autonomous_sadem/pid.py
  #src/autonomous_sadem/ppm.py
  src/autonomous_sadem/slam.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)