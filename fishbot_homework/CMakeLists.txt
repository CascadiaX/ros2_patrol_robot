cmake_minimum_required(VERSION 3.8)
project(fishbot_homework)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)

add_executable(get_robot_pose src/get_robot_pose.cpp)
ament_target_dependencies(get_robot_pose rclcpp tf2 tf2_ros geometry_msgs tf2_geometry_msgs)

add_executable(nav_to_pose src/nav_to_pose.cpp)
ament_target_dependencies(nav_to_pose rclcpp rclcpp_action nav2_msgs)

add_executable(waypoint_follower src/waypoint_follower.cpp)
ament_target_dependencies(waypoint_follower rclcpp rclcpp_action nav2_msgs)

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
install(TARGETS
  get_robot_pose nav_to_pose waypoint_follower
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
