cmake_minimum_required(VERSION 3.5)
project(az_body_tracker)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# You must change the following line to point to the location of the Azure Kinect ROS2 Driver
set(azure_kinect_ros_driver_INCLUDE_DIRS "/home/do/ros2_ws/src/Azure_Kinect_ROS_Driver/include")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(azbt_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(k4a 1.4.1 REQUIRED)
find_package(k4abt 1.1.2 REQUIRED)

add_executable(az_body_tracker src/TrackerNode.cpp src/az_body_tracker.cpp)


ament_target_dependencies(az_body_tracker rclcpp 
  std_msgs sensor_msgs visualization_msgs azbt_msgs)

target_include_directories(az_body_tracker PUBLIC
  ${azbt_msgs_INCLUDE_DIRS}
  ${azure_kinect_ros_driver_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(az_body_tracker 
  k4abt::k4abt 
  k4a::k4a 
)

install(TARGETS az_body_tracker
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
