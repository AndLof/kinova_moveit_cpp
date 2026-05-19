cmake_minimum_required(VERSION 3.8)
project(kinova_moveit_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(move_ee src/move_ee.cpp)

ament_target_dependencies(move_ee
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
)

add_executable(cartesian_path src/cartesian_path.cpp)

ament_target_dependencies(cartesian_path
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
)

add_executable(rotate_360 src/rotate_360.cpp)

ament_target_dependencies(rotate_360
  rclcpp
  moveit_ros_planning_interface
)

add_executable(rotate_base src/rotate_base.cpp)

ament_target_dependencies(rotate_base
  rclcpp
  sensor_msgs
  trajectory_msgs
)

install(TARGETS 
  move_ee
  cartesian_path
  rotate_360
  rotate_base
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
