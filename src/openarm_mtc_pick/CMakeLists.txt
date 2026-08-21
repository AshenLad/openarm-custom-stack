cmake_minimum_required(VERSION 3.16)
project(openarm_mtc_pick)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

add_executable(scene_manager src/scene_manager.cpp)
ament_target_dependencies(scene_manager
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  shape_msgs
)

add_executable(mtc_environment_check src/mtc_environment_check.cpp)
ament_target_dependencies(mtc_environment_check
  moveit_task_constructor_core
  rclcpp
)

add_executable(mtc_pregrasp src/mtc_pregrasp.cpp)
ament_target_dependencies(mtc_pregrasp
  control_msgs
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  rclcpp
  sensor_msgs
  shape_msgs
)

install(TARGETS scene_manager mtc_environment_check mtc_pregrasp
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/build_overlay.bash scripts/setup_overlay.bash
  scripts/fake_blue_cube_pose_publisher.py
  scripts/randomized_fake_regression.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Eigen3 REQUIRED)
  ament_add_gtest(test_run_mode test/test_run_mode.cpp)
  target_include_directories(test_run_mode PRIVATE src)
  ament_add_gtest(test_tcp_geometry test/test_tcp_geometry.cpp)
  target_include_directories(test_tcp_geometry PRIVATE src
    ${EIGEN3_INCLUDE_DIRS})
  ament_lint_cmake()
  ament_xmllint()
endif()

ament_package()
