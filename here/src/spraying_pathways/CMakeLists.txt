cmake_minimum_required(VERSION 3.8)
project(spraying_pathways)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  # For Python support
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(srdfdom REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_py REQUIRED)
find_package(moveit_core REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_action REQUIRED)    ###########
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)

# Make headers available to everything built in this package
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Install & export headers (so downstream packages can include them)
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)


# === Build C++ Executables ===
add_executable(go_home_node src/go_home.cpp)
ament_target_dependencies(go_home_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
  yaml-cpp
)

target_link_libraries(go_home_node yaml-cpp)

add_executable(follow_joints_positions_node src/follow_joints_positions.cpp)
ament_target_dependencies(follow_joints_positions_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
)

add_executable(cartesian_path_planner_cubes_v2_node src/cartesian_path_planner_cubes_v2.cpp)
ament_target_dependencies(cartesian_path_planner_cubes_v2_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
)

add_executable(cartesian_path_planner_trajectory_v1_node src/cartesian_path_planner_trajectory_v1.cpp)
ament_target_dependencies(cartesian_path_planner_trajectory_v1_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
)

add_executable(cartesian_path_planner_trajectory_v1_mycobot320_node src/cartesian_path_planner_trajectory_v1_mycobot320.cpp)
ament_target_dependencies(cartesian_path_planner_trajectory_v1_mycobot320_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
)

add_executable(cartesian_path_planner_cubes_v3_node src/cartesian_path_planner_cubes_v3.cpp)
ament_target_dependencies(cartesian_path_planner_cubes_v3_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
)

add_executable(cartesian_path_planner_cubes_test_v1_node src/cartesian_path_planner_cubes_test_v1.cpp)
ament_target_dependencies(cartesian_path_planner_cubes_test_v1_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
)

add_executable(cartesian_path_planner_cubes_test_v2_node src/cartesian_path_planner_cubes_test_v2.cpp)
ament_target_dependencies(cartesian_path_planner_cubes_test_v2_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
)

add_executable(cartesian_path_planner_cubes_test_go_v1_node src/cartesian_path_planner_cubes_test_go_v1.cpp)
ament_target_dependencies(cartesian_path_planner_cubes_test_go_v1_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
)

add_executable(cartesian_path_planner_cubes_test_go_v1_mycobot320_node src/cartesian_path_planner_cubes_test_go_v1_mycobot320.cpp)
ament_target_dependencies(cartesian_path_planner_cubes_test_go_v1_mycobot320_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
)

add_executable(cartesian_path_planner_cubes_test_go_v2_node src/cartesian_path_planner_cubes_test_go_v2.cpp)
ament_target_dependencies(cartesian_path_planner_cubes_test_go_v2_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
)

add_executable(cartesian_path_planner_cubes_v4_node src/cartesian_path_planner_cubes_v4.cpp)
ament_target_dependencies(cartesian_path_planner_cubes_v4_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
)

add_executable(lidar_surface_scanner_node src/lidar_surface_scanner.cpp)
ament_target_dependencies(lidar_surface_scanner_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
  pcl_conversions
)

add_executable(flat_fan_spraying_node src/flat_fan_spraying.cpp)
ament_target_dependencies(flat_fan_spraying_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
  pcl_conversions
  visualization_msgs
)

add_executable(test_node src/test.cpp)
ament_target_dependencies(test_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
  pcl_conversions
  visualization_msgs
  moveit_core
  moveit_ros_planning
  tf2_geometry_msgs
  yaml-cpp
)

target_link_libraries(test_node yaml-cpp)

add_executable(flat_fan_spraying_v2_node src/flat_fan_spraying_v2.cpp)
ament_target_dependencies(flat_fan_spraying_v2_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
  pcl_conversions
  visualization_msgs
  moveit_core
  moveit_ros_planning
  tf2_geometry_msgs
  yaml-cpp
)

target_link_libraries(flat_fan_spraying_v2_node yaml-cpp)

add_executable(moving_obstacle src/moving_obstacle.cpp)
ament_target_dependencies(moving_obstacle
  rclcpp
  geometry_msgs
  gazebo_msgs
  ament_index_cpp
)

add_executable(flat_fan_spraying_v3_node src/flat_fan_spraying_v3.cpp)
ament_target_dependencies(flat_fan_spraying_v3_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
  pcl_conversions
  visualization_msgs
  moveit_core
  moveit_ros_planning
  tf2_geometry_msgs
  yaml-cpp
)

target_link_libraries(flat_fan_spraying_v3_node yaml-cpp)

add_executable(safety_stop_on_unknown_node src/safety_stop_on_unknown.cpp)
ament_target_dependencies(safety_stop_on_unknown_node
  rclcpp
  rclcpp_action
  sensor_msgs
  control_msgs
  moveit_ros_planning_interface
  trajectory_msgs
  )

add_executable(flat_fan_spraying_v4_node src/flat_fan_spraying_v4.cpp)
ament_target_dependencies(flat_fan_spraying_v4_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
  pcl_conversions
  visualization_msgs
  moveit_core
  moveit_ros_planning
  tf2_geometry_msgs
  yaml-cpp
  control_msgs
)
target_link_libraries(flat_fan_spraying_v4_node yaml-cpp)

add_executable(inspect_flatness_node src/inspect_flatness.cpp)
ament_target_dependencies(inspect_flatness_node
  rclcpp
  sensor_msgs
  moveit_ros_planning_interface
  tf2_geometry_msgs
)

add_executable(magnesite_pusher src/magnesite_pusher.cpp)
ament_target_dependencies(magnesite_pusher
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
  tf2_ros
  tf2_geometry_msgs
)

# ONNX Runtime path (downloaded to project deps)
set(ONNXRUNTIME_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../deps/onnxruntime" CACHE PATH "ONNX Runtime root")
if(NOT EXISTS ${ONNXRUNTIME_DIR})
  message(FATAL_ERROR "ONNX Runtime not found at ${ONNXRUNTIME_DIR}. Please download/extract onnxruntime-linux-x64 to ros2_ws/deps/onnxruntime")
endif()

add_executable(magnesite_conveyor_ros_node src/magnesite_conveyor_ros.cpp)
ament_target_dependencies(magnesite_conveyor_ros_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  image_geometry
  tf2_ros
)
target_include_directories(magnesite_conveyor_ros_node PRIVATE
  ${ONNXRUNTIME_DIR}/include
)
target_link_libraries(magnesite_conveyor_ros_node
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so
  ${OpenCV_LIBS}
)

# self_filter_node temporarily disabled due to missing headers
# add_executable(self_filter_node src/self_filter.cpp)
# ament_target_dependencies(self_filter_node
#   rclcpp
#   sensor_msgs
#   pcl_ros
#   pcl_conversions
#   tf2_ros
#   tf2_sensor_msgs
#   tf2_eigen
#   srdfdom
#   moveit_ros_planning
# )

# === Install C++ Executables ===
install(TARGETS
  go_home_node
  follow_joints_positions_node
  cartesian_path_planner_cubes_v2_node
  cartesian_path_planner_trajectory_v1_node
  cartesian_path_planner_trajectory_v1_mycobot320_node
  cartesian_path_planner_cubes_v3_node
  cartesian_path_planner_cubes_test_v1_node
  cartesian_path_planner_cubes_test_v2_node
  cartesian_path_planner_cubes_test_go_v1_node
  cartesian_path_planner_cubes_test_go_v1_mycobot320_node
  cartesian_path_planner_cubes_test_go_v2_node
  cartesian_path_planner_cubes_v4_node
  lidar_surface_scanner_node
  flat_fan_spraying_node
  test_node
  flat_fan_spraying_v2_node
  moving_obstacle
  flat_fan_spraying_v3_node
  flat_fan_spraying_v4_node
  safety_stop_on_unknown_node
  inspect_flatness_node
  magnesite_pusher
  magnesite_conveyor_ros_node
  DESTINATION lib/${PROJECT_NAME}
)

# === Install Python Executables ===
install(PROGRAMS
  scripts/trajectory_logger.py
  scripts/plot_trajectory_log.py
  scripts/depth_dip_detector_v1.py
  scripts/transform_camera_pointcloud.py
  scripts/ee_velocity_monitor.py
  scripts/unknown_object_detector.py
  scripts/pointcloud_transform_and_unknown_filter.py
  scripts/pointcloud_transform_and_unknown_filter_v2.py
  scripts/pointcloud_transform_and_unknown_filter_v3.py
  scripts/obstacles_tracking.py
  scripts/test_mycobot_setup.sh
  scripts/mycobot_driver.py
  scripts/spray_valve_node.py
  scripts/pointcloud_collision_filter.py
  scripts/robot_self_filter.py
  scripts/magnesite_conveyor_ros.py
  scripts/magnesite_conveyor_ros_kf.py
  scripts/conveyor_rendezvous_solver.py
  scripts/magnesite_conveyor_rendezvous.py
  DESTINATION lib/${PROJECT_NAME}
)

# === Install Other Resources ===
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)


# === Linting / Testing (optional) ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# === Finalize ===
ament_package()