cmake_minimum_required(VERSION 2.8.3)
project(kt_wheel_controller)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs message_generation)
catkin_package(CATKIN_DEPENDS message_runtime)
catkin_python_setup()

add_message_files(
  FILES
  kt_wheel_controller/src/MotorStatus.msg
)
