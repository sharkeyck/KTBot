cmake_minimum_required(VERSION 2.8.3)
project(lx16a_driver)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS 
    roscpp
  LIBRARIES
    lx16a_driver_cpp
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

add_library(lx16a_driver_cpp src/driver.cpp)
target_link_libraries(lx16a_driver_cpp ${catkin_LIBRARIES})

# Main control executable
add_executable(${PROJECT_NAME}_example_cpp src/example.cpp)
set_target_properties(${PROJECT_NAME}_example_cpp PROPERTIES OUTPUT_NAME example_cpp PREFIX "")
target_link_libraries(${PROJECT_NAME}_example_cpp
  lx16a_driver_cpp
  ${catkin_LIBRARIES}
)

## Install -----------------------------

# Install libraries
install(TARGETS
    lx16a_driver_cpp
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install executables
install(TARGETS
    ${PROJECT_NAME}_example_cpp
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)