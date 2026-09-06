cmake_minimum_required(VERSION 3.8)
project(kinova_moveit_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# MoveIt e PCL richiedono C++17 (uso std::optional, std::clamp)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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

# --- dipendenze aggiuntive per kinova_explorer_nbv ---
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
# Per debug
find_package(visualization_msgs REQUIRED)

# PCL: include, link dir e definizioni globali (PCL non e' un pacchetto ament)
include_directories(${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

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

# --- nuovo nodo next-best-view ---
add_executable(kinova_explorer_nbv_v2 src/kinova_explorer_nbv_v2.cpp)
ament_target_dependencies(kinova_explorer_nbv_v2
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  geometry_msgs
  shape_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  octomap_msgs
  visualization_msgs # for debug
)
# PCL, OctoMap ed Eigen non sono pacchetti ament: si linkano qui
target_link_libraries(kinova_explorer_nbv_v2
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  Eigen3::Eigen
)

# --- nodo next-best-view v3 (spostamento laterale di Spot lungo y del corpo) ---
add_executable(kinova_explorer_nbv_v3 src/kinova_explorer_nbv_v3.cpp)
ament_target_dependencies(kinova_explorer_nbv_v3
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  geometry_msgs
  shape_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  octomap_msgs
  visualization_msgs # for debug
)
# PCL, OctoMap ed Eigen non sono pacchetti ament: si linkano qui
target_link_libraries(kinova_explorer_nbv_v3
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  Eigen3::Eigen
)

add_executable(kinova_explorer_nbv_validation src/kinova_explorer_nbv_validation.cpp)
ament_target_dependencies(kinova_explorer_nbv_validation
  rclcpp
  moveit_ros_planning_interface 
  moveit_core 
  moveit_msgs 
  geometry_msgs
  shape_msgs 
  sensor_msgs 
  tf2 
  tf2_ros 
  tf2_sensor_msgs 
  pcl_conversions
  octomap_msgs 
  visualization_msgs)
  
target_link_libraries(kinova_explorer_nbv_validation
  ${PCL_LIBRARIES} 
  ${OCTOMAP_LIBRARIES} 
  Eigen3::Eigen)

install(TARGETS
  move_ee
  cartesian_path
  rotate_360
  rotate_base
  kinova_explorer_nbv_v2
  kinova_explorer_nbv_v3
  kinova_explorer_nbv_validation
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()