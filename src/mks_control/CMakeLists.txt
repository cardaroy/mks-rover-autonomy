cmake_minimum_required(VERSION 3.8)
project(mks_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(joy_to_cmd_vel src/joy_to_cmd_vel.cpp)
ament_target_dependencies(joy_to_cmd_vel
  rclcpp
  sensor_msgs
  geometry_msgs
)

add_executable(cmd_vel_to_drive src/cmd_vel_to_drive.cpp)
ament_target_dependencies(cmd_vel_to_drive
  rclcpp
  geometry_msgs
  std_msgs
)

install(TARGETS
  joy_to_cmd_vel
  cmd_vel_to_drive
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
