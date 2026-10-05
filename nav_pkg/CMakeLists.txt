cmake_minimum_required(VERSION 3.8)
project(nav_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
DIRECTORY
  launch maps config behavior_trees
DESTINATION
  share/${PROJECT_NAME})

install(DIRECTORY
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/test_navigate_through_poses.py
  scripts/test_clockwise_rolling_goal.py
  scripts/test_follow_waypoints.py
  scripts/test_clockwise_gate_goal.py
  scripts/speed_manager_node.py

  scripts/screenshot_daemon.py
  scripts/client.py

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
