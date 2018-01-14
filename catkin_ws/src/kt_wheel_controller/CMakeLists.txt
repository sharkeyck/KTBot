cmake_minimum_required(VERSION 2.8.3)
project(kt_wheel_controller)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs message_generation)

add_message_files(
  FILES
  MotorStatus.msg
)

catkin_python_setup()
generate_messages()
catkin_package(
  CATKIN_DEPENDS message_runtime
)



