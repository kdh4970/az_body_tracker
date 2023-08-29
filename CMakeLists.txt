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
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/cmake)
set(glfw3_DIR "/usr/local/lib/cmake/glfw3")
set(Eigen3_DIR "/usr/lib/cmake/eigen3")
# set(azure_kinect_ros_driver_INCLUDE_DIRS "/home/jskimlab1804/catkin_ws/src/Azure_Kinect_ROS_Driver/include")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_subdirectory(window_controller_3d)
# find dependencies
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  roslib
  std_msgs
  azbt_msgs
  sensor_msgs
  visualization_msgs
  genmsg
  Eigen3
  message_generation
  azure_kinect_ros_driver
  dynamic_reconfigure
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs azbt_msgs visualization_msgs)
generate_dynamic_reconfigure_options(cfg/az_body_tracker.cfg)

catkin_package(
  CATKIN_DEPENDS rospy roscpp dynamic_reconfigure
)
find_package(k4a 1.4.1 QUIET MODULE REQUIRED)
find_package(k4abt 1.1.2 QUIET MODULE REQUIRED)
message("include dir : '${azure_kinect_ros_driver_INCLUDE_DIRS}'")
include_directories(${catkin_INCLUDE_DIRS} include src 
  ${azure_kinect_ros_driver_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

add_executable(az_body_tracker src/az_body_tracker.cpp src/TrackerNode.cpp include/TrackerNode.h)

## if you wanna see default visualizer(window_controller_3d), uncomment below line.
# target_compile_definitions(az_body_tracker PUBLIC ENABLE_DEFAULT_VISUALIZER)
add_dependencies(az_body_tracker azure_kinect_ros_driver_node)
target_link_libraries(az_body_tracker 
k4abt::k4abt 
k4a::k4a 
window_controller_3d::window_controller_3d
${azure_kinect_ros_driver_LIBRARIES}
# azure_kinect_ros_driver
${Eigen_LIBRARIES}
${catkin_LIBRARIES}
)

target_include_directories(az_body_tracker 
PUBLIC
  ${azure_kinect_ros_driver_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
add_dependencies(az_body_tracker az_body_tracker_generate_messages_cpp)
add_dependencies(az_body_tracker az_body_tracker_gencfg)

install(TARGETS az_body_tracker
  DESTINATION lib/${PROJECT_NAME})
catkin_package(CATKIN_DEPENDS roscpp roslib message_runtime std_msgs sensor_msgs azbt_msgs visualization_msgs)

catkin_install_python(PROGRAMS
  scripts/transform_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
