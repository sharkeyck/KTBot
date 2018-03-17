cmake_minimum_required(VERSION 2.8.3)
project(ktbot_core)

find_package(catkin REQUIRED COMPONENTS
  ktbot_lidar
  ktbot_control
  roslaunch
)

roslaunch_add_file_check(launch)

# catkin_python_setup()
