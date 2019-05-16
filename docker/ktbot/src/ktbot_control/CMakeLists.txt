cmake_minimum_required(VERSION 2.8.3)
project(ktbot_control)

# C++ 11
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  ros_control_boilerplate
  lx16a_driver
  roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    ros_control_boilerplate
    lx16a_driver
    roscpp
  LIBRARIES
    ktbot_hw_interface
)

include_directories(
  include
  ${lx16a_driver_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# KTBot Hardware Interface
add_library(
  ktbot_hw_interface src/ktbot_hw_interface.cpp
)

target_link_libraries(
  ktbot_hw_interface
  generic_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(ktbot_hw_main src/ktbot_hw_main.cpp)
target_link_libraries(
  ktbot_hw_main
  ktbot_hw_interface
  generic_hw_control_loop
  ${catkin_LIBRARIES}
)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    ktbot_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    ktbot_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(
  DIRECTORY include/ktbot_control/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
