cmake_minimum_required(VERSION 3.8)
project(motion_capture_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

# add dependencies
add_subdirectory(deps/libobjecttracker EXCLUDE_FROM_ALL)
add_subdirectory(deps/libmotioncapture EXCLUDE_FROM_ALL)

include_directories(
  deps/libobjecttracker/include
  deps/libmotioncapture/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(motion_capture_tracking_node
  src/motion_capture_tracking_node.cpp
)
target_link_libraries(motion_capture_tracking_node
  libobjecttracker
  libmotioncapture
)
ament_target_dependencies(motion_capture_tracking_node
  rclcpp
  sensor_msgs
  tf2_ros
)
target_include_directories(motion_capture_tracking_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(motion_capture_tracking_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS motion_capture_tracking_node
  DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
