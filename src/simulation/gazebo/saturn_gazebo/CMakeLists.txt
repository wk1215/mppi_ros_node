cmake_minimum_required(VERSION 3.0.2)
project(saturn_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install scripts
catkin_install_python(PROGRAMS
  scripts/process_urdf.py
  scripts/hexapod_vel_driver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
    config
    meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
