cmake_minimum_required(VERSION 3.0.2)
project(wbots_main)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  cv_bridge
)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Declare a catkin package
catkin_package()

# Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS} # Include directories for catkin packages
  ${OpenCV_INCLUDE_DIRS} # Include directories for OpenCV
)

# Add executable
add_executable(WarehouseBots_RS2 src/Archive/wbots_main.cpp)

# Link libraries to the executable
target_link_libraries(WarehouseBots_RS2
  ${catkin_LIBRARIES} # ROS libraries
  ${OpenCV_LIBRARIES} # openCV libraries
)

catkin_install_python(PROGRAMS scripts/nav_goals.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


