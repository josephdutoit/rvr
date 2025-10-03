cmake_minimum_required(VERSION 3.8)
project(rvr_ui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rvr_msgs REQUIRED)
find_package(Curses REQUIRED)

# keyboard node executable
add_executable(keyboard_node src/keyboard_node.cpp)
ament_target_dependencies(keyboard_node rclcpp rvr_msgs)
target_link_libraries(keyboard_node ${CURSES_LIBRARIES})
install(TARGETS keyboard_node DESTINATION lib/${PROJECT_NAME})

add_executable(joystick_node src/joystick_node.cpp)
ament_target_dependencies(joystick_node rclcpp rvr_msgs)
target_link_libraries(joystick_node ${CURSES_LIBRARIES})
install(TARGETS joystick_node DESTINATION lib/${PROJECT_NAME})

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
