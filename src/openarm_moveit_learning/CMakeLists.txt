cmake_minimum_required(VERSION 3.8)
project(openarm_moveit_learning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(minimal_moveit_demo
  src/minimal_moveit_demo.cpp
  )
add_executable(scene_obstacle_demo
  src/scene_obstacle_demo.cpp
)

foreach(target minimal_moveit_demo scene_obstacle_demo)
  ament_target_dependencies(${target}
    rclcpp
    moveit_ros_planning_interface
    moveit_msgs
    shape_msgs
    geometry_msgs
  )
endforeach()

install(TARGETS minimal_moveit_demo scene_obstacle_demo
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

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
