cmake_minimum_required(VERSION 2.8.3)
project(kt_core)

find_package(catkin REQUIRED COMPONENTS
  kt_lidar
  kt_wheel_controller
  roslaunch
)

roslaunch_add_file_check(launch)

# catkin_python_setup()
