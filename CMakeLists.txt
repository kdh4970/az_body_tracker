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
add_subdirectory(window_controller_3d)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(glfw3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
#================================================================
# Force running the Findk4a.cmake module
find_package(k4a 1.4.0 REQUIRED)

# Try to find and enable the body tracking SDK
find_package(k4abt 1.1.2 REQUIRED)
# This reads the K4A_LIBS and K4A_INSTALL_REQUIRED variables and decides how to install
# the various shared objects / DLLs
#=================================================================

add_executable(main src/TrackerNode.cpp src/az_body_tracker.cpp)
target_link_libraries(main 
k4abt::k4abt 
k4a::k4a 
window_controller_3d::window_controller_3d
glfw)

ament_target_dependencies(main rclcpp 
  std_msgs)
target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)



add_executable(alpha src/TrackerNode.cpp src/az_body_tracker_test.cpp)
target_link_libraries(alpha 
k4abt::k4abt 
k4a::k4a 
window_controller_3d::window_controller_3d
glfw)

ament_target_dependencies(alpha rclcpp 
  std_msgs)
target_include_directories(alpha PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


install(TARGETS main
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS alpha
  DESTINATION lib/${PROJECT_NAME})

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
