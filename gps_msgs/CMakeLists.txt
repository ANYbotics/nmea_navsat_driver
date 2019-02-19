# Minimum cmake version
cmake_minimum_required(VERSION 2.8.3)

# Create project
project(gps_msgs)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
)

# Declare the service and message files to be built
add_message_files(
  FILES
    RtkNavSatStatus.msg
    RtkNavSatFix.msg
)
generate_messages(
  DEPENDENCIES
    std_msgs
)

# Generate a package
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)